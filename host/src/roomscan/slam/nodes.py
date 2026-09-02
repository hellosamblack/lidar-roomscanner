"""Per-keyframe cloud store that sits BESIDE the TSDF (Issue #150).

Why this exists (`docs/superpowers/specs/2026-08-19-slam-posegraph-backbone-design.md`
§Decision 3): our poses are baked into the `VoxelBlockGrid` at integrate time,
so correcting a pose today means de-integrating and re-integrating. That -- not
the size of the drift -- is the structural blocker keeping loop closure behind
an evidence gate. RTAB-Map applies a graph re-optimization as *one mat4 write
per node* (`RTABMapApp.cpp:2036-2039`), and this module is that representation:

    TSDF          -- the live fused surface, unchanged, still the thing you see
    NodeCloudStore -- one decimated cloud per admitted keyframe, in the SENSOR
                      frame, plus the world pose it was integrated with

A loop closure is then: optimize the graph -> `update_poses()` -> rebuild a
fresh TSDF as a batch replay. No interactive de-integration anywhere.

**The points are in the sensor frame. That is the whole point.** Storing world
points would read as a working store right up until the first correction, at
which point it would re-pose nothing -- the pose would already be fused into
every coordinate. `world_points()` applies the pose on demand instead, so a
correction is one 4x4 write and zero point rewrites.

Cost is measurable rather than estimated: `nbytes` is the exact retained array
cost (12 B/point + 128 B of pose per node), because the spec's kill criterion
for #150 is stated in MB of host RAM for a 10-minute scan and a proxy would
decide it wrongly. `decimation` is the lever that moves that number; the
measured #149 keyframe gate (15-32% of frames admitted) is what bounds the node
COUNT.

`max_nodes` is a hard retention ceiling that REFUSES new nodes rather than
evicting old ones -- eviction would punch a hole in the graph this store
exists to serve. `Mapper` deliberately does not set it yet: the spec asks for
the per-node cost to be measured on the two reference captures before a cap is
chosen, and a cap picked ahead of that measurement would silently truncate the
very measurement it was picked from.
"""
from __future__ import annotations

import numpy as np


def _validate_pose(pose) -> np.ndarray:
    T = np.asarray(pose, dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f"pose must be a 4x4 matrix, got shape {T.shape}")
    return T.copy()


class MapNode:
    """One admitted keyframe: a decimated sensor-frame cloud plus its pose.

    Constructible on its own so an offline pose-graph pass can build nodes from
    a recorded trajectory without a live `Mapper`.
    """

    __slots__ = ("node_id", "_pose", "points", "frame_index", "t_device_us")

    def __init__(self, node_id: int, pose, points: np.ndarray,
                 frame_index: int = -1, t_device_us: float | None = None):
        pts = np.asarray(points)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got shape {pts.shape}")
        self.node_id = int(node_id)
        self._pose = _validate_pose(pose)
        # Copy, always: the caller is usually handing over a slice of a per-frame
        # buffer it is about to overwrite. Aliasing it would leave every node
        # holding the LAST frame's geometry, which looks fine until you read it.
        # `np.array(..., copy=True)`, NOT `ascontiguousarray`: the latter returns
        # the INPUT unchanged when it is already C-contiguous float32 -- i.e. it
        # silently aliases in exactly the common case (caught by
        # test_the_store_copies_its_input_...).
        self.points = np.array(pts, dtype=np.float32, copy=True, order="C")
        self.frame_index = int(frame_index)
        self.t_device_us = None if t_device_us is None else float(t_device_us)

    @property
    def pose(self) -> np.ndarray:
        """World-from-sensor 4x4 this node's cloud is currently posed at."""
        return self._pose

    def set_pose(self, pose) -> None:
        """Re-pose this node -- the entire cost of applying a graph correction
        to this node's share of the map. Validated BEFORE assignment so a
        malformed pose can never half-land."""
        self._pose = _validate_pose(pose)

    @property
    def point_count(self) -> int:
        return int(self.points.shape[0])

    @property
    def nbytes(self) -> int:
        """Exact retained bytes: the cloud plus the pose."""
        return int(self.points.nbytes + self._pose.nbytes)

    def world_points(self) -> np.ndarray:
        """The cloud in world coordinates at the CURRENT pose (float32,
        computed on demand -- never cached, or a `set_pose` would have to
        invalidate it and the stale copy would be the thing anyone read)."""
        R = self._pose[:3, :3].astype(np.float32)
        t = self._pose[:3, 3].astype(np.float32)
        return self.points @ R.T + t

    def __repr__(self) -> str:      # pragma: no cover - debugging aid
        return (f"MapNode(id={self.node_id}, points={self.point_count}, "
                f"frame={self.frame_index})")


class NodeCloudStore:
    """Ordered, append-only collection of `MapNode`s keyed by node id.

    Append-only by design: node ids are never reused, so a graph edge recorded
    against an id can never silently come to mean a different keyframe.
    """

    def __init__(self, decimation: int = 1, max_nodes: int = 0):
        # <= 1 means "keep every point"; the numpy stride must be >= 1 or the
        # slice would reverse (negative) or raise (zero).
        self.decimation = max(1, int(decimation))
        self.max_nodes = max(0, int(max_nodes))
        self._nodes: list[MapNode] = []
        self._by_id: dict[int, MapNode] = {}
        self._next_id = 0
        self.dropped = 0

    # ---- admission ---------------------------------------------------------
    def add(self, pose, points: np.ndarray, frame_index: int = -1,
            t_device_us: float | None = None) -> MapNode | None:
        """Retain one keyframe. Returns the new node, or None when `max_nodes`
        is reached (and increments `dropped`) -- a caller that ignores the
        return value keeps working, it just stops growing."""
        if self.max_nodes and len(self._nodes) >= self.max_nodes:
            self.dropped += 1
            return None
        pts = np.asarray(points)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points must be (N, 3), got shape {pts.shape}")
        if self.decimation > 1:
            pts = pts[::self.decimation]
        node = MapNode(self._next_id, pose, pts, frame_index=frame_index,
                       t_device_us=t_device_us)
        self._next_id += 1
        self._nodes.append(node)
        self._by_id[node.node_id] = node
        return node

    def clear(self) -> None:
        """Release every retained cloud. `_next_id` deliberately keeps
        counting: a node id must never be reused by a later graph."""
        self._nodes.clear()
        self._by_id.clear()

    # ---- the pose-graph write path ----------------------------------------
    def update_poses(self, poses: dict[int, np.ndarray]) -> int:
        """Apply a graph optimization result: one 4x4 write per named node.
        Returns how many nodes moved.

        All-or-nothing: every id and every pose is validated before ANY of them
        is applied, so a bad id in the middle of an optimization result cannot
        leave the map half-corrected (a half-corrected map is worse than an
        uncorrected one -- it is silently inconsistent)."""
        validated = {}
        for node_id, pose in poses.items():
            node = self._by_id.get(int(node_id))
            if node is None:
                raise KeyError(f"no such node id: {node_id}")
            validated[node] = _validate_pose(pose)
        for node, pose in validated.items():
            node.set_pose(pose)
        return len(validated)

    # ---- reads -------------------------------------------------------------
    def __len__(self) -> int:
        return len(self._nodes)

    def __iter__(self):
        return iter(self._nodes)

    @property
    def nodes(self) -> tuple[MapNode, ...]:
        return tuple(self._nodes)

    def get(self, node_id: int) -> MapNode | None:
        return self._by_id.get(int(node_id))

    @property
    def point_count(self) -> int:
        return int(sum(n.point_count for n in self._nodes))

    @property
    def nbytes(self) -> int:
        """Exact retained array bytes across every node -- the number the
        spec's kill criterion is stated in, not an estimate of it."""
        return int(sum(n.nbytes for n in self._nodes))

    @property
    def nbytes_mb(self) -> float:
        return self.nbytes / (1024.0 * 1024.0)

    def world_cloud(self) -> np.ndarray:
        """Every node's cloud at its current pose, concatenated in node-id
        order. (N, 3) float32; (0, 3) when empty."""
        if not self._nodes:
            return np.zeros((0, 3), dtype=np.float32)
        return np.concatenate([n.world_points() for n in self._nodes], axis=0)

    def stats(self) -> dict:
        """The quantities the spec asks to be measured before #150 is committed
        to: node count, retained points, exact bytes, and the decimation that
        produced them."""
        n = len(self._nodes)
        points = self.point_count
        return {
            "nodes": n,
            "points": points,
            "points_per_node": (points / n) if n else 0.0,
            "bytes": self.nbytes,
            "mb": self.nbytes_mb,
            "decimation": self.decimation,
            "max_nodes": self.max_nodes,
            "dropped": self.dropped,
        }
