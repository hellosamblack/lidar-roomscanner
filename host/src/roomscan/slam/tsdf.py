"""TSDF map: a thin wrapper over Open3D's tensor VoxelBlockGrid for frame-to-model
SLAM. All poses are 4x4 world<-camera; integrate/raycast take the world->camera
`extrinsic` = inv(pose). CPU only.

Open3D 0.19 API notes (verified against the installed build -- see
.superpowers/sdd/task-4-report.md for the full trace):
  - `ray_cast(...)`'s render_attributes keys ARE `vertex`/`normal`/`depth` as the
    task brief expected; results come back as an `o3d.t.geometry.TensorMap`
    (dict-like via `result['vertex']`, not `.keys()`).
  - `ray_cast`'s default `range_map_down_factor=8` hangs/burns CPU indefinitely
    on our small (~54x42) depth images (repeated "Could not generate full range
    map" reallocation that never converges). MUST pass `range_map_down_factor=1`
    for images this small.
  - `ray_cast`'s returned `normal` points *away* from the camera (+z into the
    surface for a fronto-parallel wall), the opposite of the usual "outward
    toward the sensor" convention -- confirmed empirically (median normal.z was
    exactly +1.0 for a wall at z=+1 viewed from the origin). We negate it so
    callers get camera-facing normals as point-to-plane ICP conventionally
    expects.
  - `extract_point_cloud()` / `extract_triangle_mesh()` raise a C++
    `SetPointColors` shape-mismatch error if `attr_names` omits `color` --
    unconditionally, even with 0 points requested. We declare an unused
    `color` attribute solely to keep those two methods usable; we never
    populate it (integrate() always uses the depth-only overload).

Task 9.5 perf note: `raycast()`'s cost is dominated by how many voxel blocks
it visits. Passing `hashmap().active_buf_indices()` (ALL blocks ever
integrated) makes cost scale with total map size, not the current view -- the
`t_reconstruction_system/ray_casting.py` example instead bounds this to
`vbg.compute_unique_block_coordinates(depth, intrinsic, extrinsic,
depth_scale, depth_max)`, the same frustum-bounded set `integrate()` already
uses. `raycast()` now accepts an optional `block_coords` (or a `depth_hint`
to derive them) so `Mapper` can pass the current frame's frustum; omitting it
keeps the original all-active-blocks behavior for callers/tests that don't
have a depth hint handy.
"""
from __future__ import annotations

import numpy as np
import open3d as o3d

_CPU = o3d.core.Device("CPU:0")


class TsdfMap:
    def __init__(self, voxel_size: float = 0.01, trunc_multiplier: float = 8.0,
                 block_resolution: int = 8, block_count: int = 40000,
                 depth_scale: float = 1000.0, depth_max: float = 5.0):
        self.voxel_size = voxel_size
        self.trunc_multiplier = trunc_multiplier
        self.depth_scale = depth_scale
        self.depth_max = depth_max
        self._empty = True
        self._vbg = o3d.t.geometry.VoxelBlockGrid(
            attr_names=("tsdf", "weight", "color"),
            attr_dtypes=(o3d.core.float32, o3d.core.float32, o3d.core.float32),
            attr_channels=((1,), (1,), (3,)),
            voxel_size=voxel_size,
            block_resolution=block_resolution,
            block_count=block_count,
            device=_CPU,
        )

    def _depth_image(self, depth_mm: np.ndarray) -> o3d.t.geometry.Image:
        d = np.ascontiguousarray(depth_mm, dtype=np.float32)
        return o3d.t.geometry.Image(o3d.core.Tensor(d, device=_CPU))

    def integrate(self, depth_mm: np.ndarray, intrinsic: o3d.core.Tensor,
                  extrinsic: np.ndarray) -> None:
        depth = self._depth_image(depth_mm)
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        coords = self._vbg.compute_unique_block_coordinates(
            depth, intr, ext, self.depth_scale, self.depth_max, self.trunc_multiplier)
        self._vbg.integrate(coords, depth, intr, ext,
                            self.depth_scale, self.depth_max, self.trunc_multiplier)
        self._empty = False

    def frustum_block_coords(self, depth_mm: np.ndarray, intrinsic: o3d.core.Tensor,
                              extrinsic: np.ndarray) -> o3d.core.Tensor:
        """Unique voxel-block coordinates visible from `extrinsic` given a depth
        hint -- the same frustum-bounded set `integrate()` uses, exposed so
        `Mapper` can bound `raycast()`'s cost to the current view instead of
        the whole map."""
        depth = self._depth_image(depth_mm)
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        return self._vbg.compute_unique_block_coordinates(
            depth, intr, ext, self.depth_scale, self.depth_max, self.trunc_multiplier)

    def raycast(self, intrinsic: o3d.core.Tensor, extrinsic: np.ndarray,
                width: int, height: int,
                block_coords: o3d.core.Tensor | None = None,
                depth_hint: np.ndarray | None = None) -> o3d.t.geometry.PointCloud | None:
        """block_coords/depth_hint are optional and bound raycast cost to a
        subset of blocks (e.g. the current view frustum) instead of every
        active block in the map. Pass `block_coords` directly (from
        `frustum_block_coords`) or a `depth_hint` to have it computed here;
        omit both to fall back to the original all-active-blocks behavior."""
        if self._empty:
            return None
        ext = o3d.core.Tensor(np.asarray(extrinsic, dtype=np.float64), device=_CPU)
        intr = intrinsic.to(_CPU)
        if block_coords is not None:
            coords = block_coords
        elif depth_hint is not None:
            coords = self.frustum_block_coords(depth_hint, intrinsic, extrinsic)
        else:
            hashmap = self._vbg.hashmap()
            active_idx = hashmap.active_buf_indices()
            if active_idx.shape[0] == 0:
                return None
            coords = hashmap.key_tensor()[active_idx]
        if coords.shape[0] == 0:
            return None
        result = self._vbg.ray_cast(
            coords, intr, ext, width, height,
            render_attributes=["vertex", "normal", "depth"],
            depth_scale=self.depth_scale, depth_min=0.1,
            depth_max=self.depth_max, weight_threshold=1.0,
            trunc_voxel_multiplier=self.trunc_multiplier,
            range_map_down_factor=1)
        vertex = result["vertex"].numpy().reshape(-1, 3)
        normal = -result["normal"].numpy().reshape(-1, 3)
        depth = result["depth"].numpy().reshape(-1)
        keep = depth > 0.0
        if not keep.any():
            return None
        pc = o3d.t.geometry.PointCloud(_CPU)
        pc.point.positions = o3d.core.Tensor(vertex[keep].astype(np.float32))
        pc.point.normals = o3d.core.Tensor(normal[keep].astype(np.float32))
        return pc

    def mesh(self) -> o3d.t.geometry.TriangleMesh:
        return self._vbg.extract_triangle_mesh()

    def point_cloud(self) -> o3d.t.geometry.PointCloud:
        return self._vbg.extract_point_cloud()
