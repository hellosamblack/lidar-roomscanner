# Live-view fps Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Keep the live SLAM viewport at ≥30 fps (target 120), flat as the map grows, on both the local and remote backends, by moving all O(map-size) mesh work off the GUI tick and decoupling pose from mesh on the remote wire — while preserving the exact final map.

**Architecture:** Component A adds an off-GUI-thread `MeshPrep` stage that takes the worker's newest mesh, adaptively decimates it (display-only), bakes shading, splits walls, and extracts the floor grid into a plain-data `MeshPacket`; the panel tick only builds Open3D geometry from that packet at a throttled cadence and feeds the measured upload time back for the adaptive controller. Component B splits the remote transport into a tiny per-frame `pose` message (sent immediately) and an occasional `mesh` message, with the client accumulating the trajectory from pose deltas instead of receiving the full trajectory every frame. Both preserve the `latest() -> (mesh, trajectory, FrameStep) | None` contract, so the panel is agnostic to which backend it holds.

**Tech Stack:** Python 3.11–3.12, NumPy, Open3D 0.19 (tensor + legacy geometry, `visualization.gui`), stdlib `socket`/`threading`, pytest.

## Global Constraints

Copied verbatim from the spec; every task's requirements implicitly include these.

- **Preserve the worker contract:** both `SlamWorker` and `RemoteSlamWorker` keep `latest() -> (mesh, trajectory, FrameStep) | None`. The panel and `MeshPrep` are backend-agnostic consumers of that tuple.
- **Decimation is display-only.** The saved/offline map always comes from the full-resolution `mapper.mesh()`; the final artifact must be byte-identical to today. Never route a decimated mesh into any save/record/post-process path.
- **Do not change** the SLAM algorithm, the map, the wire *framing* (`wire.py`'s `[len][json header][array bytes]` layout), or the device protocol (`protocol.py`).
- **Do not raise the sensor data rate** (firmware/I3C — out of scope). The 120 fps target is a viewport-render goal, not a new-scan-data goal.
- **Config defaults** (`[slam]` table of `roomscan.toml`): `mesh_upload_hz = 3.0`, `live_vertex_budget = 150000`, `fps_budget_ms = 8.0`. Defaults chosen so a small map stays full-res and only large maps decimate.
- **Tests are GPU-free and GUI-free.** Use `pytest.importorskip("open3d")` where a real tensor/legacy mesh is needed; drive `ControlPanel` methods **unbound on a lightweight stand-in** (the established `test_panel_walls.py` / `test_panel_showcase.py` pattern) rather than instantiating a real window.
- **Existing panel unit tests stay green:** `host/tests/test_panel_walls.py`, `host/tests/test_panel_ux.py`. In particular `_upload_slam_mesh` and its wall-split behavior are unchanged — the new live path is a *separate* `_upload_mesh_packet` method; `_upload_slam_mesh` remains for Showcase PROCESSING/FINAL.

**Test runner (all tasks):** from `F:\git\personal\lidar\roomscanner\host`, run
`.venv\Scripts\python.exe -m pytest <path>::<test> -v`
(`pythonpath = ["src", "."]` is set in `host/pyproject.toml`, so no editable install is needed). On a Git-Bash shell use `.venv/Scripts/python.exe -m pytest ...`.

**Commit discipline:** commit at the end of each task (DRY, YAGNI, TDD, frequent commits). Do not merge to `main`; this work is on `feature/phase6-slam`.

---

## File Structure

**Component A — off-thread adaptive mesh (new + modified):**
- Create `host/src/roomscan/slam/meshprep.py` — `MeshPacket` dataclass, the pure `prepare_packet(...)` function, the `_submesh_arrays(...)` helper, and the threaded `MeshPrep` class. One responsibility: turn a worker mesh into a ready-to-upload packet, off the GUI thread.
- Modify `host/src/roomscan/panel.py` — add `_upload_mesh_packet` / `_upload_floor_grid_from_packet`; wire `MeshPrep` into `_render_slam_frame` at a throttled cadence with adaptive feedback and lifecycle teardown; add the viewport render-fps counter.
- Modify `host/src/roomscan/metrics_hud.py` — add a "VIEW" (viewport fps) row to the HUD.
- Modify `host/src/roomscan/slam/config.py` — add `mesh_upload_hz`, `live_vertex_budget`, `fps_budget_ms` to `SlamConfig`.

**Component B — pose/mesh transport split (modified):**
- Modify `host/src/roomscan/slam/wire.py` — add `POSE`/`MESH` tag constants and `pose_message(...)` / `mesh_message(...)` builders. No framing change.
- Modify `host/src/roomscan/slam/service.py` — `serve_client` sends a `pose` message per frame immediately, interleaves a `mesh` message only when a new mesh is ready, and stops sending the full trajectory.
- Modify `host/src/roomscan/slam/remote.py` — `_recv_loop` dispatches by tag, accumulates the trajectory from pose deltas, and caches the mesh on `mesh` messages.

**Tests (new + modified):**
- Create `host/tests/test_slam_meshprep.py` (Tasks 1–2), `host/tests/test_panel_meshpacket.py` (Task 4), `host/tests/test_panel_viewfps.py` (Task 6).
- Modify `host/tests/test_slam_config.py` (Task 3), `host/tests/test_slam_wire.py` (Task 7), `host/tests/test_slam_service.py` (Task 8), `host/tests/test_slam_remote.py` (Task 9).

---

## Task 1: `MeshPacket` + pure `prepare_packet` (shading + decimation + wall split + floor grid)

**Files:**
- Create: `host/src/roomscan/slam/meshprep.py`
- Test: `host/tests/test_slam_meshprep.py`

**Interfaces:**
- Consumes (existing, unchanged): `roomscan.slam.shading.{wall_triangle_mask, shade_brightness, shade_colors, height_base_colors, height_tint_hue, wavefront_glow, mesh_colors_are_meaningful}`; `roomscan.slam.frames.world_up`; `roomscan.theme.floor_grid_lines`. These are the SAME helpers `panel._upload_slam_mesh` uses today — reuse them, do not re-derive.
- Produces (later tasks rely on these exact names/types):
  - `MeshPacket` dataclass fields: `non_wall_verts: np.ndarray (N,3) f64`, `non_wall_colors: np.ndarray (N,3) f64`, `non_wall_tris: np.ndarray (M,3) i32`, `wall_verts: np.ndarray (P,3) f64`, `wall_colors: np.ndarray (P,3) f64`, `wall_tris: np.ndarray (Q,3) i32`, `floor_pts: np.ndarray (K,3) f64`, `floor_lines: np.ndarray (L,2) i64`, `mesh_seq: int`, `source_vertex_count: int`, `decimated: bool`, `wall_mode: str`.
  - `prepare_packet(mesh, *, wall_mode: str, glow_origin, mesh_seq: int, vertex_budget: int, decimate: bool, up=None) -> MeshPacket`.
  - `_submesh_arrays(verts: np.ndarray, colors: np.ndarray, tris: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]` (dense-remapped verts, colors, tris).

Full-res mode (`wall_mode == "solid"` or no triangles) puts the whole shaded mesh in `non_wall_*` and leaves `wall_*` empty — mirroring `_upload_slam_mesh`'s solid branch exactly.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_slam_meshprep.py`:

```python
import numpy as np
import pytest

pytest.importorskip("open3d")
import open3d as o3d

from roomscan.slam.meshprep import MeshPacket, prepare_packet, _submesh_arrays


def _corner_tensor_mesh():
    """One unambiguous wall triangle (normal ~world-Z, perpendicular to
    world-up [0,-1,0]) + one unambiguous floor triangle (normal ~world-Y).
    Same fixture shape as test_panel_walls.py."""
    verts = np.array([
        [0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0],   # wall triangle
        [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0],   # floor triangle
    ], dtype=np.float32)
    tris = np.array([[0, 1, 2], [3, 4, 5]], dtype=np.int32)
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(tris)
    return m


def _grid_tensor_mesh(n=40):
    """A dense flat grid mesh with ~n*n vertices, for exercising decimation."""
    xs, ys = np.meshgrid(np.linspace(0, 1, n), np.linspace(0, 1, n))
    verts = np.stack([xs.ravel(), ys.ravel(), np.zeros(n * n)], axis=1).astype(np.float32)
    tris = []
    for r in range(n - 1):
        for c in range(n - 1):
            a = r * n + c; b = a + 1; d = a + n; e = d + 1
            tris.append([a, b, d]); tris.append([b, e, d])
    m = o3d.t.geometry.TriangleMesh()
    m.vertex.positions = o3d.core.Tensor(verts)
    m.triangle.indices = o3d.core.Tensor(np.asarray(tris, np.int32))
    return m


def test_submesh_arrays_remaps_to_dense_indices():
    verts = np.arange(18, dtype=np.float64).reshape(6, 3)
    colors = np.zeros((6, 3))
    tris = np.array([[3, 4, 5]], dtype=np.int32)   # references only verts 3,4,5
    v, c, t = _submesh_arrays(verts, colors, tris)
    assert v.shape == (3, 3)
    np.testing.assert_array_equal(v, verts[3:6])
    np.testing.assert_array_equal(t, np.array([[0, 1, 2]], dtype=np.int32))


def test_solid_mode_puts_whole_mesh_in_non_wall_no_walls():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=7,
                         vertex_budget=10_000, decimate=False)
    assert isinstance(pkt, MeshPacket)
    assert pkt.mesh_seq == 7
    assert pkt.decimated is False
    assert pkt.wall_mode == "solid"
    assert len(pkt.non_wall_tris) == 2       # whole mesh, unsplit
    assert len(pkt.wall_tris) == 0
    assert pkt.non_wall_colors.shape == pkt.non_wall_verts.shape


def test_translucent_mode_splits_wall_from_floor():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="translucent", glow_origin=None, mesh_seq=1,
                         vertex_budget=10_000, decimate=False)
    assert len(pkt.non_wall_tris) == 1       # the floor triangle
    assert len(pkt.wall_tris) == 1           # the wall triangle
    # dense-remapped: each submesh's triangle indices point inside its own verts
    assert pkt.non_wall_tris.max() < len(pkt.non_wall_verts)
    assert pkt.wall_tris.max() < len(pkt.wall_verts)


def test_floor_grid_populated_from_vertices():
    m = _corner_tensor_mesh()
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=10_000, decimate=False)
    # the corner fixture spans x,z in [0,1] -> a non-degenerate floor grid
    assert len(pkt.floor_pts) >= 2
    assert len(pkt.floor_lines) >= 1


def test_decimation_kicks_in_past_budget():
    m = _grid_tensor_mesh(40)                # ~1600 verts
    n_src = len(m.vertex.positions)
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=200, decimate=True)
    assert pkt.decimated is True
    assert pkt.source_vertex_count == n_src
    assert len(pkt.non_wall_verts) < n_src   # actually reduced


def test_no_decimation_below_budget_or_when_disabled():
    m = _grid_tensor_mesh(20)                # ~400 verts
    n_src = len(m.vertex.positions)
    # below budget: stays full-res even with decimate=True
    pkt = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                         vertex_budget=10_000, decimate=True)
    assert pkt.decimated is False
    assert len(pkt.non_wall_verts) == n_src
    # over budget but decimate=False: still full-res
    pkt2 = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                          vertex_budget=10, decimate=False)
    assert pkt2.decimated is False
    assert len(pkt2.non_wall_verts) == n_src


def test_glow_origin_changes_colors():
    m = _corner_tensor_mesh()
    base = prepare_packet(m, wall_mode="solid", glow_origin=None, mesh_seq=0,
                          vertex_budget=10_000, decimate=False)
    glowed = prepare_packet(m, wall_mode="solid", glow_origin=np.array([0.0, 0.0, 0.0]),
                            mesh_seq=0, vertex_budget=10_000, decimate=False)
    assert not np.allclose(base.non_wall_colors, glowed.non_wall_colors)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_meshprep.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.slam.meshprep'`.

- [ ] **Step 3: Write the implementation**

Create `host/src/roomscan/slam/meshprep.py`:

```python
"""Off-GUI-thread mesh preparation for the live SLAM view (Component A).

Takes the newest worker mesh, adaptively decimates it (display-only), bakes the
same shading `panel._upload_slam_mesh` uses, splits walls from floor/ceiling,
and extracts the floor grid -- all the O(map-size) work -- into a plain-data
`MeshPacket` the GUI tick can upload cheaply. The saved/offline map always comes
from the full-resolution `mapper.mesh()`; decimation here never touches it.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import numpy as np

_IDLE_SLEEP_S = 0.005


@dataclass
class MeshPacket:
    non_wall_verts: np.ndarray     # (N,3) f64
    non_wall_colors: np.ndarray    # (N,3) f64
    non_wall_tris: np.ndarray      # (M,3) i32 -- dense indices into non_wall_verts
    wall_verts: np.ndarray         # (P,3) f64
    wall_colors: np.ndarray        # (P,3) f64
    wall_tris: np.ndarray          # (Q,3) i32 -- dense indices into wall_verts
    floor_pts: np.ndarray          # (K,3) f64
    floor_lines: np.ndarray        # (L,2) i64
    mesh_seq: int
    source_vertex_count: int
    decimated: bool
    wall_mode: str


def _submesh_arrays(verts: np.ndarray, colors: np.ndarray, tris: np.ndarray):
    """Dense-remap a triangle subset to 0..K-1, carrying the referenced verts +
    colors. Numpy twin of panel._wall_submesh (which builds a legacy mesh); this
    returns arrays so the packet stays GUI-handle-free."""
    if tris.shape[0] == 0:
        return (np.zeros((0, 3), np.float64), np.zeros((0, 3), np.float64),
                np.zeros((0, 3), np.int32))
    uniq, remap = np.unique(tris.reshape(-1), return_inverse=True)
    new_tris = remap.reshape(tris.shape).astype(np.int32)
    return verts[uniq], colors[uniq], new_tris


def prepare_packet(mesh, *, wall_mode: str, glow_origin, mesh_seq: int,
                   vertex_budget: int, decimate: bool, up=None) -> MeshPacket:
    """Pure: tensor SLAM/TSDF `mesh` -> ready-to-upload `MeshPacket`.

    Shading mirrors panel._upload_slam_mesh exactly (reflectance-meaningful ->
    grey * brightness * height-hue; else height-cued base * shade_colors), plus
    the live wavefront glow when `glow_origin` is not None. `decimate` (True when
    the adaptive controller says the last upload blew the frame budget) triggers
    quadric decimation to ~`vertex_budget` verts; below budget, or when False,
    the mesh passes through full-res (`decimated=False`)."""
    from .shading import (height_base_colors, height_tint_hue,
                          mesh_colors_are_meaningful, shade_brightness,
                          shade_colors, wall_triangle_mask, wavefront_glow)
    from .frames import world_up
    from ..theme import floor_grid_lines
    if up is None:
        up = world_up()

    legacy = mesh.cpu().to_legacy()
    source_vertex_count = len(legacy.vertices)

    decimated = False
    n_tris = len(legacy.triangles)
    if decimate and source_vertex_count > vertex_budget and n_tris > 0:
        target_tris = max(4, int(n_tris * vertex_budget / source_vertex_count))
        legacy = legacy.simplify_quadric_decimation(
            target_number_of_triangles=target_tris)
        decimated = True

    legacy.compute_vertex_normals()
    normals = np.asarray(legacy.vertex_normals)
    verts = np.asarray(legacy.vertices)
    raw_colors = np.asarray(legacy.vertex_colors)
    if mesh_colors_are_meaningful(raw_colors):
        brightness = shade_brightness(normals)
        hue = height_tint_hue(verts, up)
        final_colors = np.clip(raw_colors * brightness[:, None] * hue, 0.0, 1.0)
    else:
        base = height_base_colors(verts, up)
        final_colors = shade_colors(normals, base=base)
    if glow_origin is not None:
        final_colors = wavefront_glow(verts, glow_origin, final_colors)

    floor_pts, floor_lines = (np.zeros((0, 3)), np.zeros((0, 2), np.int64))
    if len(verts) > 0:
        mn, mx = verts.min(axis=0), verts.max(axis=0)
        floor_pts, floor_lines = floor_grid_lines(mn, mx, up=up, spacing=0.5)

    tris = np.asarray(legacy.triangles)
    if wall_mode == "solid" or tris.shape[0] == 0:
        return MeshPacket(
            non_wall_verts=verts, non_wall_colors=final_colors, non_wall_tris=tris.astype(np.int32),
            wall_verts=np.zeros((0, 3)), wall_colors=np.zeros((0, 3)),
            wall_tris=np.zeros((0, 3), np.int32),
            floor_pts=floor_pts, floor_lines=floor_lines,
            mesh_seq=mesh_seq, source_vertex_count=source_vertex_count,
            decimated=decimated, wall_mode=wall_mode)

    legacy.compute_triangle_normals()
    wall_mask = wall_triangle_mask(np.asarray(legacy.triangle_normals), up=up)
    nw_v, nw_c, nw_t = _submesh_arrays(verts, final_colors, tris[~wall_mask])
    w_v, w_c, w_t = _submesh_arrays(verts, final_colors, tris[wall_mask])
    return MeshPacket(
        non_wall_verts=nw_v, non_wall_colors=nw_c, non_wall_tris=nw_t,
        wall_verts=w_v, wall_colors=w_c, wall_tris=w_t,
        floor_pts=floor_pts, floor_lines=floor_lines,
        mesh_seq=mesh_seq, source_vertex_count=source_vertex_count,
        decimated=decimated, wall_mode=wall_mode)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_meshprep.py -v`
Expected: PASS (all 8 tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/meshprep.py host/tests/test_slam_meshprep.py
git commit -m "feat(slam): MeshPacket + pure prepare_packet (off-thread mesh prep)"
```

---

## Task 2: `MeshPrep` threaded wrapper (latest-wins slots + adaptive controller)

**Files:**
- Modify: `host/src/roomscan/slam/meshprep.py` (append the `MeshPrep` class)
- Test: `host/tests/test_slam_meshprep.py` (append)

**Interfaces:**
- Consumes: `prepare_packet(...)`, `MeshPacket` (Task 1).
- Produces (Task 5 relies on these exact signatures):
  - `MeshPrep(vertex_budget: int = 150000, fps_budget_ms: float = 8.0, up=None)`
  - `submit(mesh, *, mesh_seq: int, glow_origin, wall_mode: str) -> None` (latest-wins input; never blocks)
  - `latest() -> MeshPacket | None` (consume-once: returns the pending packet and clears the slot)
  - `note_upload_ms(ms: float) -> None` (adaptive feedback from the tick's measured upload wall-time)
  - `run_once() -> bool` (synchronous: pop input, prepare, publish; True if it processed one — this is what tests call, no thread needed)
  - `start() -> None` / `stop() -> None` (daemon-thread lifecycle, mirrors `SlamWorker`)

Adaptive rule: `decimate = self._last_upload_ms > self.fps_budget_ms`. `_last_upload_ms` starts at 0.0 (full-res until proven expensive).

- [ ] **Step 1: Write the failing tests**

Append to `host/tests/test_slam_meshprep.py`:

```python
from roomscan.slam.meshprep import MeshPrep


def test_meshprep_run_once_publishes_packet_and_consumes_input():
    m = _corner_tensor_mesh()
    prep = MeshPrep(vertex_budget=10_000, fps_budget_ms=8.0)
    assert prep.latest() is None
    assert prep.run_once() is False        # empty input slot
    prep.submit(m, mesh_seq=3, glow_origin=None, wall_mode="solid")
    assert prep.run_once() is True
    pkt = prep.latest()
    assert pkt is not None and pkt.mesh_seq == 3
    assert prep.latest() is None           # consume-once


def test_meshprep_latest_wins_input():
    m = _corner_tensor_mesh()
    prep = MeshPrep(vertex_budget=10_000)
    prep.submit(m, mesh_seq=1, glow_origin=None, wall_mode="solid")
    prep.submit(m, mesh_seq=2, glow_origin=None, wall_mode="solid")  # overwrites
    prep.run_once()
    assert prep.latest().mesh_seq == 2      # only the newest survives


def test_meshprep_adaptive_decimates_after_slow_upload():
    m = _grid_tensor_mesh(40)               # ~1600 verts
    prep = MeshPrep(vertex_budget=200, fps_budget_ms=8.0)
    # last upload was fast -> full-res
    prep.submit(m, mesh_seq=1, glow_origin=None, wall_mode="solid")
    prep.run_once()
    assert prep.latest().decimated is False
    # report a slow upload -> next packet decimates
    prep.note_upload_ms(50.0)
    prep.submit(m, mesh_seq=2, glow_origin=None, wall_mode="solid")
    prep.run_once()
    assert prep.latest().decimated is True
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_meshprep.py -k meshprep -v`
Expected: FAIL with `ImportError: cannot import name 'MeshPrep'`.

- [ ] **Step 3: Write the implementation**

Append to `host/src/roomscan/slam/meshprep.py`:

```python
class MeshPrep:
    """Runs `prepare_packet` off the GUI thread with latest-wins in/out slots and
    an adaptive decimation controller. Mirrors slam.worker.SlamWorker's threading
    shape (daemon thread, lock-guarded slots, bounded-join stop)."""

    def __init__(self, vertex_budget: int = 150_000, fps_budget_ms: float = 8.0,
                 up=None):
        self._vertex_budget = int(vertex_budget)
        self._fps_budget_ms = float(fps_budget_ms)
        self._up = up
        self._last_upload_ms = 0.0

        self._in_lock = threading.Lock()
        self._in_slot = None            # (mesh, mesh_seq, glow_origin, wall_mode) | None
        self._out_lock = threading.Lock()
        self._out_slot = None           # MeshPacket | None

        self._thread: threading.Thread | None = None
        self._stop_evt = threading.Event()

    @property
    def fps_budget_ms(self) -> float:
        return self._fps_budget_ms

    def submit(self, mesh, *, mesh_seq: int, glow_origin, wall_mode: str) -> None:
        with self._in_lock:
            self._in_slot = (mesh, mesh_seq, glow_origin, wall_mode)

    def note_upload_ms(self, ms: float) -> None:
        self._last_upload_ms = float(ms)

    def run_once(self) -> bool:
        with self._in_lock:
            item, self._in_slot = self._in_slot, None
        if item is None:
            return False
        mesh, mesh_seq, glow_origin, wall_mode = item
        decimate = self._last_upload_ms > self._fps_budget_ms
        pkt = prepare_packet(mesh, wall_mode=wall_mode, glow_origin=glow_origin,
                             mesh_seq=mesh_seq, vertex_budget=self._vertex_budget,
                             decimate=decimate, up=self._up)
        with self._out_lock:
            self._out_slot = pkt
        return True

    def latest(self):
        with self._out_lock:
            pkt, self._out_slot = self._out_slot, None
        return pkt

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop_evt.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def _run_loop(self) -> None:
        while not self._stop_evt.is_set():
            if not self.run_once():
                time.sleep(_IDLE_SLEEP_S)

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_meshprep.py -v`
Expected: PASS (all Task 1 + Task 2 tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/meshprep.py host/tests/test_slam_meshprep.py
git commit -m "feat(slam): MeshPrep threaded wrapper with adaptive decimation controller"
```

---

## Task 3: `SlamConfig` view-cadence fields

**Files:**
- Modify: `host/src/roomscan/slam/config.py:44-74` (add three fields to the `SlamConfig` dataclass)
- Test: `host/tests/test_slam_config.py` (append)

**Interfaces:**
- Produces (Task 5 relies on these): `SlamConfig.mesh_upload_hz: float = 3.0`, `SlamConfig.live_vertex_budget: int = 150000`, `SlamConfig.fps_budget_ms: float = 8.0`. Loaded from the `[slam]` table by the existing `SlamConfig.load()` (which already filters to known field names — no loader change needed).

- [ ] **Step 1: Write the failing test**

Append to `host/tests/test_slam_config.py`:

```python
def test_view_cadence_defaults():
    from roomscan.slam.config import SlamConfig
    cfg = SlamConfig()
    assert cfg.mesh_upload_hz == 3.0
    assert cfg.live_vertex_budget == 150000
    assert cfg.fps_budget_ms == 8.0


def test_view_cadence_overrides_from_toml(tmp_path):
    from roomscan.slam.config import SlamConfig
    p = tmp_path / "roomscan.toml"
    p.write_text(
        "[slam]\n"
        "mesh_upload_hz = 5.0\n"
        "live_vertex_budget = 80000\n"
        "fps_budget_ms = 4.0\n",
        encoding="utf-8")
    cfg = SlamConfig.load(p)
    assert cfg.mesh_upload_hz == 5.0
    assert cfg.live_vertex_budget == 80000
    assert cfg.fps_budget_ms == 4.0
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_config.py -k view_cadence -v`
Expected: FAIL with `AttributeError: 'SlamConfig' object has no attribute 'mesh_upload_hz'`.

- [ ] **Step 3: Add the fields**

In `host/src/roomscan/slam/config.py`, add after the `remote_addr` field (currently `config.py:74`):

```python
    remote_addr: str = "127.0.0.1:5555"
    # Live-view render cadence (Component A -- off-thread adaptive mesh). The
    # heavy mesh/ribbon/floor upload runs at most `mesh_upload_hz` times/sec on
    # the GUI tick; MeshPrep decimates a packet to ~`live_vertex_budget` verts
    # only once an upload's measured wall-time exceeds `fps_budget_ms` (~120 fps
    # per-upload ceiling). Display-only: the saved map is always full-res.
    mesh_upload_hz: float = 3.0
    live_vertex_budget: int = 150000
    fps_budget_ms: float = 8.0
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_config.py -v`
Expected: PASS (new tests + all existing config tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/config.py host/tests/test_slam_config.py
git commit -m "feat(slam): add mesh_upload_hz/live_vertex_budget/fps_budget_ms to SlamConfig"
```

---

## Task 4: Panel `_upload_mesh_packet` + `_upload_floor_grid_from_packet` (headless)

**Files:**
- Modify: `host/src/roomscan/panel.py` (add two methods near `_upload_slam_mesh`, `panel.py:1540`)
- Test: `host/tests/test_panel_meshpacket.py` (new)

**Interfaces:**
- Consumes: `MeshPacket` (Task 1). Existing panel geometry names `_MESH_GEOM`, `_MESH_WALLS_GEOM`, `_FLOOR_GRID_GEOM`; materials `self.mesh_material`, `self.wall_translucent_material`, `self.wall_wire_material`, `self.floor_material`; `self._o3d`; `theme.FLOOR_GRID`.
- Produces (Task 5 relies on these): `ControlPanel._upload_mesh_packet(self, packet: MeshPacket) -> None` and `ControlPanel._upload_floor_grid_from_packet(self, pts, lines) -> None`.

`_upload_mesh_packet` reproduces `_upload_slam_mesh`'s *upload* half (geometry build + add_geometry) sourced from packet arrays; the *shading/split* half already ran in `MeshPrep`. `_upload_slam_mesh` itself is left untouched (Showcase PROCESSING/FINAL still use it).

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_panel_meshpacket.py`:

```python
import numpy as np
import pytest

pytest.importorskip("open3d")
import open3d as o3d

import roomscan.panel as panel_mod
from roomscan.slam.meshprep import MeshPacket

_MESH_GEOM = panel_mod._MESH_GEOM
_MESH_WALLS_GEOM = panel_mod._MESH_WALLS_GEOM
_FLOOR_GRID_GEOM = panel_mod._FLOOR_GRID_GEOM


class _FakeScene:
    def __init__(self):
        self.geoms = {}

    def has_geometry(self, name):
        return name in self.geoms

    def add_geometry(self, name, geom, material):
        self.geoms[name] = (geom, material)

    def remove_geometry(self, name):
        del self.geoms[name]


class _FakeSceneWidget:
    def __init__(self):
        self.scene = _FakeScene()


class _FakePanel:
    def __init__(self, wall_mode="solid"):
        self._o3d = o3d
        self.scene_widget = _FakeSceneWidget()
        self.wall_mode = wall_mode
        self.mesh_material = "MESH_MATERIAL"
        self.wall_translucent_material = "WALL_TRANSLUCENT_MATERIAL"
        self.wall_wire_material = "WALL_WIRE_MATERIAL"
        self.floor_material = "FLOOR_MATERIAL"
        self._floor_last_bounds = None


def _solid_packet(wall_mode="solid"):
    verts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    colors = np.tile([0.5, 0.5, 0.5], (3, 1))
    tris = np.array([[0, 1, 2]], np.int32)
    return MeshPacket(
        non_wall_verts=verts, non_wall_colors=colors, non_wall_tris=tris,
        wall_verts=np.zeros((0, 3)), wall_colors=np.zeros((0, 3)),
        wall_tris=np.zeros((0, 3), np.int32),
        floor_pts=np.zeros((0, 3)), floor_lines=np.zeros((0, 2), np.int64),
        mesh_seq=1, source_vertex_count=3, decimated=False, wall_mode=wall_mode)


def _split_packet(wall_mode="translucent"):
    nwv = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    wv = np.array([[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])
    tri = np.array([[0, 1, 2]], np.int32)
    grey = lambda n: np.tile([0.5, 0.5, 0.5], (n, 1))
    return MeshPacket(
        non_wall_verts=nwv, non_wall_colors=grey(3), non_wall_tris=tri,
        wall_verts=wv, wall_colors=grey(3), wall_tris=tri,
        floor_pts=np.zeros((0, 3)), floor_lines=np.zeros((0, 2), np.int64),
        mesh_seq=1, source_vertex_count=6, decimated=False, wall_mode=wall_mode)


def test_solid_packet_uploads_one_opaque_mesh():
    fake = _FakePanel()
    panel_mod.ControlPanel._upload_mesh_packet(fake, _solid_packet())
    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM}
    geom, material = geoms[_MESH_GEOM]
    assert material == "MESH_MATERIAL"
    assert len(geom.triangles) == 1


def test_translucent_packet_uploads_wall_and_floor_geoms():
    fake = _FakePanel(wall_mode="translucent")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("translucent"))
    geoms = fake.scene_widget.scene.geoms
    assert set(geoms) == {_MESH_GEOM, _MESH_WALLS_GEOM}
    assert geoms[_MESH_WALLS_GEOM][1] == "WALL_TRANSLUCENT_MATERIAL"
    assert isinstance(geoms[_MESH_WALLS_GEOM][0], o3d.geometry.TriangleMesh)


def test_wireframe_packet_uploads_walls_as_lineset():
    fake = _FakePanel(wall_mode="wireframe")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("wireframe"))
    geoms = fake.scene_widget.scene.geoms
    assert geoms[_MESH_WALLS_GEOM][1] == "WALL_WIRE_MATERIAL"
    assert isinstance(geoms[_MESH_WALLS_GEOM][0], o3d.geometry.LineSet)


def test_stale_geoms_cleared_on_reupload():
    fake = _FakePanel(wall_mode="translucent")
    panel_mod.ControlPanel._upload_mesh_packet(fake, _split_packet("translucent"))
    assert _MESH_WALLS_GEOM in fake.scene_widget.scene.geoms
    # a later solid packet must drop the stale walls geom
    panel_mod.ControlPanel._upload_mesh_packet(fake, _solid_packet("solid"))
    assert set(fake.scene_widget.scene.geoms) == {_MESH_GEOM}


def test_floor_grid_uploaded_when_present():
    fake = _FakePanel()
    pkt = _solid_packet()
    pkt.floor_pts = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]])
    pkt.floor_lines = np.array([[0, 1]], np.int64)
    panel_mod.ControlPanel._upload_mesh_packet(fake, pkt)
    assert _FLOOR_GRID_GEOM in fake.scene_widget.scene.geoms
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_meshpacket.py -v`
Expected: FAIL with `AttributeError: type object 'ControlPanel' has no attribute '_upload_mesh_packet'`.

- [ ] **Step 3: Write the implementation**

In `host/src/roomscan/panel.py`, add these two methods immediately after `_upload_slam_mesh` (ends at `panel.py:1647`, before `_on_slam_toggle`):

```python
    def _upload_mesh_packet(self, packet):
        """Build + add_geometry from a `MeshPrep.MeshPacket` (Component A). The
        O(map-size) shading/decimation/wall-split already ran off the GUI thread
        in MeshPrep; this only materializes Open3D geometry and uploads it. Twin
        of `_upload_slam_mesh`'s upload half, sourced from packet arrays; that
        method stays for Showcase PROCESSING/FINAL."""
        o3d = self._o3d
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if sc.has_geometry(_MESH_WALLS_GEOM):
            sc.remove_geometry(_MESH_WALLS_GEOM)

        if len(packet.non_wall_tris) > 0:
            m = o3d.geometry.TriangleMesh()
            m.vertices = o3d.utility.Vector3dVector(packet.non_wall_verts)
            m.triangles = o3d.utility.Vector3iVector(packet.non_wall_tris)
            m.vertex_colors = o3d.utility.Vector3dVector(packet.non_wall_colors)
            sc.add_geometry(_MESH_GEOM, m, self.mesh_material)

        if len(packet.wall_tris) > 0:
            wm = o3d.geometry.TriangleMesh()
            wm.vertices = o3d.utility.Vector3dVector(packet.wall_verts)
            wm.triangles = o3d.utility.Vector3iVector(packet.wall_tris)
            wm.vertex_colors = o3d.utility.Vector3dVector(packet.wall_colors)
            if packet.wall_mode == "translucent":
                sc.add_geometry(_MESH_WALLS_GEOM, wm, self.wall_translucent_material)
            else:   # "wireframe"
                wire = o3d.geometry.LineSet.create_from_triangle_mesh(wm)
                # BUG-009: a <2-point / 0-segment LineSet hard-crashes Filament.
                if len(wire.points) >= 2:
                    wire.colors = o3d.utility.Vector3dVector(
                        np.tile([[0.45, 0.60, 0.75]], (len(wire.lines), 1)))
                    sc.add_geometry(_MESH_WALLS_GEOM, wire, self.wall_wire_material)

        self._upload_floor_grid_from_packet(packet.floor_pts, packet.floor_lines)

    def _upload_floor_grid_from_packet(self, pts, lines):
        """Upload the pre-extracted floor grid from a packet. Replaces the
        per-tick `mesh.vertex.positions.cpu().numpy()` copy that used to live in
        `_update_floor_grid` -- MeshPrep already did that O(size) copy + bounds
        off-thread. BUG-009: never upload a <2-point / 0-segment LineSet."""
        o3d = self._o3d
        sc = self.scene_widget.scene
        if sc.has_geometry(_FLOOR_GRID_GEOM):
            sc.remove_geometry(_FLOOR_GRID_GEOM)
        if len(pts) >= 2 and len(lines) > 0:
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(pts)
            ls.lines = o3d.utility.Vector2iVector(lines)
            ls.colors = o3d.utility.Vector3dVector(
                np.tile([list(theme.FLOOR_GRID)], (len(lines), 1)))
            sc.add_geometry(_FLOOR_GRID_GEOM, ls, self.floor_material)
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_meshpacket.py tests/test_panel_walls.py -v`
Expected: PASS (new packet tests + the untouched `_upload_slam_mesh` wall tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_meshpacket.py
git commit -m "feat(panel): _upload_mesh_packet + floor-grid upload from MeshPacket"
```

---

## Task 5: Wire `MeshPrep` into `_render_slam_frame` (throttled cadence + adaptive feedback + lifecycle)

**Files:**
- Modify: `host/src/roomscan/panel.py` — `__init__` (near `panel.py:552`, the SLAM view state), `_render_slam_frame` (`panel.py:1209-1282`), `_on_slam_toggle` (`panel.py:1649`), `_on_close` (`panel.py:1004`)
- Test: exercised by Task 10 replay validation (GUI wiring — no headless unit test; the pieces it composes are all unit-tested in Tasks 1–4).

**Interfaces:**
- Consumes: `MeshPrep` (Task 2), `SlamConfig.{mesh_upload_hz, live_vertex_budget, fps_budget_ms}` (Task 3), `_upload_mesh_packet` (Task 4). Existing: `self.slam_worker.latest()`, `self.wall_mode`, `time.monotonic`.
- Produces: no new public API; `_render_slam_frame` now feeds `MeshPrep` and uploads packets at `mesh_upload_hz` instead of doing O(size) work inline.

Design notes for the implementer:
- The heavy per-tick block being replaced is `panel.py:1266-1274` (`_upload_slam_mesh` + `_update_floor_grid` + `mesh.vertex.positions.cpu().numpy()`). The cheap per-tick work (pose label, `slam_ms`, `_update_fov_geometry`, follow-camera, trajectory) stays every tick.
- `MeshPrep` is created lazily next to `self.slam_worker` and torn down wherever the worker is (`_on_slam_toggle` off-path and `_on_close`).
- `_slam_last_mesh_obj` still gates *feeding* MeshPrep (only on a genuinely new worker mesh object). The `mesh_upload_hz` throttle gates *uploading* a ready packet.

- [ ] **Step 1: Add MeshPrep state to `__init__`**

In `host/src/roomscan/panel.py`, in the SLAM view state block (after `panel.py:554`, `self._slam_last_mesh_obj = None`), add:

```python
        # Component A (off-thread adaptive mesh): MeshPrep does the O(map-size)
        # shading/decimation/wall-split/floor work off the GUI thread; the tick
        # only uploads its ready packet at `_mesh_upload_period` s, feeding the
        # measured upload wall-time back for adaptive decimation. See
        # slam/meshprep.py + docs/superpowers/plans/2026-07-13-live-view-fps.md.
        from .slam.config import SlamConfig as _SlamCfg
        _view_cfg = _SlamCfg.load()
        self.mesh_prep = None
        self._mesh_prep_seq = 0
        self._last_mesh_upload_t = 0.0
        self._mesh_upload_period = (1.0 / _view_cfg.mesh_upload_hz
                                    if _view_cfg.mesh_upload_hz > 0 else 0.0)
        self._live_vertex_budget = _view_cfg.live_vertex_budget
        self._fps_budget_ms = _view_cfg.fps_budget_ms
```

- [ ] **Step 2: Replace the heavy upload block in `_render_slam_frame`**

In `host/src/roomscan/panel.py`, replace the worker-creation block (`panel.py:1227-1235`) so `MeshPrep` is created + started alongside the worker:

```python
        if self.slam_worker is None:
            from .slam.worker import SlamWorker
            from .slam.config import preferred_device
            from .slam.backend import make_slam_worker
            from .slam.meshprep import MeshPrep
            h, w = depth.shape
            self.slam_worker = make_slam_worker(w, h, fov_h=self.args.fov_h,
                                                fov_v=self.args.fov_v,
                                                device=preferred_device())
            self.slam_worker.start()
            self.mesh_prep = MeshPrep(vertex_budget=self._live_vertex_budget,
                                      fps_budget_ms=self._fps_budget_ms)
            self.mesh_prep.start()
```

Then replace the heavy mesh block (`panel.py:1266-1274`, from `sc = self.scene_widget.scene` through `self._slam_last_mesh_obj = mesh`) with the feed + throttled-upload path:

```python
        # Feed MeshPrep only when the worker publishes a genuinely NEW mesh
        # object (identity check) -- all the O(map-size) work happens on its
        # thread, never here.
        if (mesh is not None and mesh is not self._slam_last_mesh_obj
                and len(mesh.vertex.positions) > 0):
            self._mesh_prep_seq += 1
            self.mesh_prep.submit(mesh, mesh_seq=self._mesh_prep_seq,
                                  glow_origin=step.pose[:3, 3], wall_mode=self.wall_mode)
            self._slam_last_mesh_obj = mesh

        # Upload a ready packet at most `mesh_upload_hz` times/sec; measure the
        # upload wall-time and feed it back to MeshPrep's adaptive controller.
        now = time.monotonic()
        if now - self._last_mesh_upload_t >= self._mesh_upload_period:
            packet = self.mesh_prep.latest()
            if packet is not None:
                t0 = time.monotonic()
                self._upload_mesh_packet(packet)
                self.mesh_prep.note_upload_ms((time.monotonic() - t0) * 1000.0)
                self._last_mesh_upload_t = now
```

(The existing trajectory-ribbon and `_slam_camera_frame` calls at `panel.py:1276-1282` stay unchanged below this block.)

- [ ] **Step 3: Tear down MeshPrep with the worker**

In `_on_close` (`panel.py:1004`), add the MeshPrep stop right after the `self.slam_worker.stop()` line (`panel.py:1007`):

```python
        if self.slam_worker is not None:
            self.slam_worker.stop()        # join the SLAM worker thread before teardown
        if self.mesh_prep is not None:
            self.mesh_prep.stop()          # join the off-thread mesh-prep worker
```

In `_on_slam_toggle`'s toggle-off path, the worker is stopped + nulled at `panel.py:1666-1668` and `self._slam_last_mesh_obj = None` is already reset at `panel.py:1670`. Insert the MeshPrep teardown immediately after `self.slam_worker = None` (`panel.py:1668`):

```python
            if self.slam_worker is not None:
                self.slam_worker.stop()
                self.slam_worker = None
            if self.mesh_prep is not None:
                self.mesh_prep.stop()          # join the off-thread mesh-prep worker
                self.mesh_prep = None
            self._last_mesh_upload_t = 0.0
```

(Do not add a second `self._slam_last_mesh_obj = None` — line 1670 already does it. The invariant: MeshPrep is stopped and nulled exactly when `slam_worker` is, and the `slam_worker is None` guard in Step 2 re-creates both together on the next enable.)

- [ ] **Step 4: Verify no regression in the existing panel suite**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_walls.py tests/test_panel_ux.py tests/test_panel_showcase.py tests/test_slam_meshprep.py -v`
Expected: PASS (Component A composed pieces still green; no headless test instantiates the live tick, so this confirms nothing broke in the shared methods).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py
git commit -m "feat(panel): drive live SLAM mesh through off-thread MeshPrep at throttled cadence"
```

---

## Task 6: Viewport render-fps counter + HUD "VIEW" row

**Files:**
- Modify: `host/src/roomscan/panel.py` — `__init__` (add the deque), `_on_tick` (`panel.py:1054`, append a tick + throttled log), `_update_metrics` (`panel.py:2326`, pass `view_fps` to the HUD)
- Modify: `host/src/roomscan/metrics_hud.py` — `_rows` (`metrics_hud.py:78`) and `render_hud` (`metrics_hud.py:115`) accept a `view_fps` and add a "VIEW" row
- Test: `host/tests/test_panel_viewfps.py` (new)

**Interfaces:**
- Produces: `ControlPanel._record_view_tick(self, now: float) -> None` and `ControlPanel._view_fps(self, now: float) -> float` (pure over `self._view_ticks`, testable unbound); `metrics_hud.render_hud(snap, *, view_fps: float = 0.0, ...)`.
- The viewport counter is distinct from the data-render fps (`metrics.render_fps`, ticked per DATA frame in `_render_frame`) and from `self._fps`. It counts every `_on_tick` firing = true viewport refresh rate = the ≥30/120 validation metric.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_panel_viewfps.py`:

```python
from collections import deque
import roomscan.panel as panel_mod


class _FakeViewFpsPanel:
    def __init__(self):
        self._view_ticks = deque()
        self._view_fps_window_s = 1.0


def test_view_fps_zero_below_two_ticks():
    fake = _FakeViewFpsPanel()
    assert panel_mod.ControlPanel._view_fps(fake, 0.0) == 0.0
    panel_mod.ControlPanel._record_view_tick(fake, 0.0)
    assert panel_mod.ControlPanel._view_fps(fake, 0.0) == 0.0   # still one tick


def test_view_fps_counts_ticks_per_second():
    fake = _FakeViewFpsPanel()
    for i in range(11):                       # 11 ticks spanning 1.0 s -> 10 fps
        panel_mod.ControlPanel._record_view_tick(fake, i * 0.1)
    fps = panel_mod.ControlPanel._view_fps(fake, 1.0)
    assert abs(fps - 10.0) < 1e-6


def test_view_fps_trims_old_ticks_outside_window():
    fake = _FakeViewFpsPanel()
    panel_mod.ControlPanel._record_view_tick(fake, 0.0)   # older than the window
    for i in range(1, 7):
        panel_mod.ControlPanel._record_view_tick(fake, 2.0 + i * 0.1)
    # at now=2.6, the t=0.0 tick is >1s old and must be dropped
    assert all(t >= 1.6 for t in fake._view_ticks)


def test_render_hud_accepts_view_fps():
    import numpy as np
    from roomscan.metrics import MetricsSnapshot
    from roomscan.metrics_hud import render_hud
    snap = MetricsSnapshot(0.0, [], 0.0, None)
    img = render_hud(snap, view_fps=42.0)
    assert isinstance(img, np.ndarray) and img.ndim == 3
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_viewfps.py -v`
Expected: FAIL — `AttributeError` on `_view_fps` / `render_hud() got an unexpected keyword argument 'view_fps'`.

- [ ] **Step 3: Implement the counter + HUD row**

In `host/src/roomscan/panel.py` `__init__`, next to the other fps state (`panel.py:524`, after `self._fps = 0.0`), add:

```python
        # Viewport render-fps: counts every _on_tick firing (true refresh rate,
        # distinct from data-fps self._fps and the per-DATA-frame metrics.render_fps).
        # This is the ≥30/120 live-view validation metric (Component A).
        from collections import deque as _deque
        self._view_ticks = _deque()
        self._view_fps_window_s = 1.0
        self._last_view_fps_log = 0.0
```

Add the two methods (place them just before `_on_tick`, `panel.py:1054`):

```python
    def _record_view_tick(self, now):
        self._view_ticks.append(now)
        while self._view_ticks and now - self._view_ticks[0] > self._view_fps_window_s:
            self._view_ticks.popleft()

    def _view_fps(self, now):
        ticks = self._view_ticks
        if len(ticks) < 2:
            return 0.0
        span = ticks[-1] - ticks[0]
        return (len(ticks) - 1) / span if span > 0 else 0.0
```

At the very top of `_on_tick` (`panel.py:1055`, before `redraw = False`), record the tick:

```python
    def _on_tick(self):
        self._record_view_tick(time.monotonic())
        redraw = False
```

In the throttled UI-refresh block (`panel.py:1076-1082`, inside `if now - self._last_ui >= _UI_PERIOD:`), add a periodic view-fps log after `self._drain_log()`:

```python
            if now - self._last_view_fps_log >= 2.0:
                self._last_view_fps_log = now
                self.bus.publish(f"view fps: {self._view_fps(now):.0f}")
```

In `_update_metrics` (`panel.py:2334`), pass the viewport fps to the HUD:

```python
        img = render_hud(snap, view_fps=self._view_fps(time.monotonic()))
```

In `host/src/roomscan/metrics_hud.py`, thread `view_fps` through. Change `_rows` (`metrics_hud.py:78`) to accept it and prepend a VIEW row above the FPS row:

```python
def _rows(snap: MetricsSnapshot, usb_capacity_bps: float, fps_target: float,
          view_fps: float = 0.0) -> list[_Row]:
    rows = []
    rows.append(_Row("VIEW", f"{view_fps:.0f}",
                     frac=view_fps / fps_target if fps_target > 0 else None))
    rows.append(_Row("FPS", f"{snap.render_fps:.0f}",
                     frac=snap.render_fps / fps_target if fps_target > 0 else None))
    # ... existing rows unchanged ...
```

And `render_hud` (`metrics_hud.py:115`) — add the keyword and forward it:

```python
def render_hud(snap: MetricsSnapshot, *, view_fps: float = 0.0, width: int = 320,
               row_h: int = 22, ...):
    ...
    rows = _rows(snap, usb_capacity_bps, fps_target, view_fps=view_fps)
    ...
```

(Read `metrics_hud.py:78-115` first to keep the exact existing signature/body; the only additions are the `view_fps` param, the VIEW `_Row`, and forwarding it.)

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_viewfps.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py host/src/roomscan/metrics_hud.py host/tests/test_panel_viewfps.py
git commit -m "feat(panel): viewport render-fps counter + HUD VIEW row"
```

---

## Task 7: Wire `pose`/`mesh` message tags (Component B)

**Files:**
- Modify: `host/src/roomscan/slam/wire.py` (add constants + two builders, `wire.py:79` after `mesh_to_arrays`)
- Test: `host/tests/test_slam_wire.py` (append)

**Interfaces:**
- Produces (Tasks 8–9 rely on these): `wire.POSE = "pose"`, `wire.MESH = "mesh"`; `wire.pose_message(fid, pose, fitness, rmse, tracking_lost, slam_ms, tracking_lost_count) -> dict`; `wire.mesh_message(mesh_seq, mesh) -> dict`. Both dicts carry a `"type"` scalar and round-trip through the existing `encode_message`/`decode_message` framing unchanged (no framing change).

- [ ] **Step 1: Write the failing tests**

Append to `host/tests/test_slam_wire.py`:

```python
def test_pose_message_roundtrip():
    pose = np.eye(4, dtype=np.float32)
    msg = wire.pose_message(5, pose, fitness=0.8, rmse=0.02,
                            tracking_lost=False, slam_ms=9.1, tracking_lost_count=3)
    assert msg["type"] == wire.POSE
    out = wire.decode_message(wire.encode_message(msg))
    assert out["type"] == "pose"
    assert out["fid"] == 5
    assert out["fitness"] == pytest.approx(0.8)
    assert out["tracking_lost"] is False
    assert out["tracking_lost_count"] == 3
    np.testing.assert_array_equal(out["pose"], pose)
    assert "mesh_v" not in out            # a pose message carries no mesh


def test_mesh_message_roundtrip():
    o3d = pytest.importorskip("open3d")
    v = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0]], np.float32)
    t = np.array([[0, 1, 2]], np.int32)
    src = o3d.t.geometry.TriangleMesh()
    src.vertex["positions"] = o3d.core.Tensor(v)
    src.triangle["indices"] = o3d.core.Tensor(t)
    msg = wire.mesh_message(4, src)
    assert msg["type"] == wire.MESH and msg["mesh_seq"] == 4
    out = wire.decode_message(wire.encode_message(msg))
    assert out["type"] == "mesh" and out["mesh_seq"] == 4
    rebuilt = wire.arrays_to_mesh(out)
    np.testing.assert_array_equal(rebuilt.vertex["positions"].numpy(), v)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_wire.py -k message_roundtrip -v`
Expected: FAIL with `AttributeError: module 'roomscan.slam.wire' has no attribute 'pose_message'`.

- [ ] **Step 3: Implement the builders**

In `host/src/roomscan/slam/wire.py`, add after `mesh_to_arrays` (`wire.py:79`):

```python
POSE = "pose"
MESH = "mesh"


def pose_message(fid, pose, fitness, rmse, tracking_lost, slam_ms,
                 tracking_lost_count) -> dict:
    """A per-frame `pose` message: tiny + fixed-size, sent every frame so it is
    never delayed behind a growing mesh (Component B)."""
    return {
        "type": POSE,
        "fid": int(fid),
        "pose": np.asarray(pose, np.float32),
        "fitness": float(fitness),
        "rmse": float(rmse),
        "tracking_lost": bool(tracking_lost),
        "slam_ms": float(slam_ms),
        "tracking_lost_count": int(tracking_lost_count),
    }


def mesh_message(mesh_seq, mesh) -> dict:
    """A `mesh` message: sent only when a new throttled mesh is ready. Carries
    the same array payload as `mesh_to_arrays` plus its `mesh_seq` identity."""
    out = {"type": MESH, "mesh_seq": int(mesh_seq)}
    out.update(mesh_to_arrays(mesh))
    return out
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_wire.py -v`
Expected: PASS (new + all existing wire tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/wire.py host/tests/test_slam_wire.py
git commit -m "feat(slam): wire pose/mesh tagged-message builders"
```

---

## Task 8: Service pose/mesh split (`serve_client`)

**Files:**
- Modify: `host/src/roomscan/slam/service.py` — `serve_client` (`service.py:37-77`)
- Test: `host/tests/test_slam_service.py` — rewrite `test_service_returns_stepresult_per_frame` (`test_slam_service.py:47-73`)

**Interfaces:**
- Consumes: `wire.pose_message`, `wire.mesh_message`, `wire.POSE`, `wire.MESH` (Task 7).
- Produces: per received frame, `serve_client` sends exactly one `pose` message immediately; on a frame where `worker.latest()` yields a **new** mesh object it additionally sends one `mesh` message (pose first, mesh second). It no longer sends the trajectory.

Design decision (documented deviation from the spec's literal "runs its SlamWorker threaded"): `serve_client` stays synchronous per frame so the `pose` message's `fid` always corresponds to the frame just stepped (fid↔pose correspondence). Sending the pose message *before* the mesh message is what decouples pose delivery from mesh transfer on the wire — the transport win the spec targets. Mesh *extraction* still happens inside `worker.run_once()`; that is GPU compute (~ms), not the O(map-size) transfer the spec removes.

- [ ] **Step 1: Rewrite the failing test**

Replace `test_service_returns_stepresult_per_frame` in `host/tests/test_slam_service.py` (`test_slam_service.py:47-73`) with:

```python
def test_service_sends_pose_per_frame_and_mesh_when_ready():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        srv.serve_client(conn)
        conn.close()
    th = threading.Thread(target=accept_once, daemon=True); th.start()

    cli = socket.create_connection(("127.0.0.1", port)); cli.settimeout(5)
    poses, meshes = [], []
    # 8 frames worth of messages; each frame yields 1 pose (+ maybe 1 mesh).
    # Read until we've seen 8 poses.
    while len(poses) < 8:
        m = wire.recv_message(cli)
        assert m is not None
        if m["type"] == wire.POSE:
            poses.append(m)
        else:
            assert m["type"] == wire.MESH
            meshes.append(m)
        if len(poses) < 8 and m["type"] == wire.POSE:
            wire.send_message(cli, _synthetic_frame(len(poses)))
    # kick the first frame
    # (send the first frame before the loop; see note below)
```

Because the service is request/response per frame, the driver must send a frame, then drain the resulting pose (+optional mesh) before sending the next. Use this cleaner driver instead:

```python
def test_service_sends_pose_per_frame_and_mesh_when_ready():
    srv = SlamService(device="CPU:0", fov_h=55.0, fov_v=42.0)
    lsock = socket.socket(); lsock.bind(("127.0.0.1", 0)); lsock.listen(1)
    port = lsock.getsockname()[1]

    def accept_once():
        conn, _ = lsock.accept()
        srv.serve_client(conn)
        conn.close()
    th = threading.Thread(target=accept_once, daemon=True); th.start()

    cli = socket.create_connection(("127.0.0.1", port)); cli.settimeout(5)

    def drain_until_pose():
        """Read messages until a pose arrives; collect any mesh seen first/after."""
        got_mesh = []
        while True:
            m = wire.recv_message(cli)
            assert m is not None
            if m["type"] == wire.MESH:
                got_mesh.append(m)
            elif m["type"] == wire.POSE:
                return m, got_mesh

    poses, mesh_seen = [], 0
    for fid in range(8):
        wire.send_message(cli, _synthetic_frame(fid))
        pose, meshes = drain_until_pose()
        poses.append(pose)
        mesh_seen += len(meshes)
        for mm in meshes:
            assert "mesh_v" in mm and mm["mesh_seq"] >= 1
    cli.close(); lsock.close(); th.join(timeout=2)

    assert [p["fid"] for p in poses] == list(range(8))
    for p in poses:
        assert p["pose"].shape == (4, 4)
        assert isinstance(p["tracking_lost"], bool)
        assert "traj" not in p                 # trajectory no longer resent
    assert mesh_seen >= 1                       # a mesh was sent at least once
```

Note: the pose message may arrive before or after the mesh message on a mesh frame; `drain_until_pose` handles either interleaving. In practice the service sends pose first, so `meshes` will be collected on the *next* frame's drain — the `mesh_seen >= 1` assertion tolerates that.

- [ ] **Step 2: Run the test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_service.py -k pose_per_frame -v`
Expected: FAIL — the current `serve_client` sends one combined message with `traj` and no `type` tag, so `m["type"]` raises `KeyError`.

- [ ] **Step 3: Rewrite `serve_client`**

Replace the body of `serve_client` in `host/src/roomscan/slam/service.py` (`service.py:37-77`) with:

```python
    def serve_client(self, conn) -> None:
        worker = None
        last_mesh = object()          # sentinel; never equal to a real mesh
        mesh_seq = 0
        while True:
            msg = wire.recv_message(conn)
            if msg is None:
                break
            depth = np.asarray(msg["depth"], np.float32)
            if worker is None:
                h, w = depth.shape
                eff_kwargs = _effective_kwargs(self._mapper_kwargs, msg.get("cfg"))
                self._last_effective_kwargs = eff_kwargs
                worker = SlamWorker(w, h, mesh_every=self._mesh_every,
                                    device=self._device, **eff_kwargs)
            quat = np.asarray(msg["quat"], np.float32)
            pressure = msg.get("pressure")
            refl = msg.get("reflectance")
            conf = msg.get("confidence")
            worker.submit(depth, quat, pressure,
                          reflectance=None if refl is None else np.asarray(refl, np.float32),
                          confidence=None if conf is None else np.asarray(conf, np.float32))
            worker.run_once()
            mesh, _traj, step = worker.latest()

            # Pose first: tiny, sent immediately, never delayed behind a mesh
            # transfer or the (no-longer-sent) full trajectory.
            wire.send_message(conn, wire.pose_message(
                msg["fid"], step.pose, step.fitness, step.rmse,
                step.tracking_lost, step.slam_ms, worker.tracking_lost_count))

            # Mesh only when the worker published a new one (identity check).
            if mesh is not None and mesh is not last_mesh:
                mesh_seq += 1
                wire.send_message(conn, wire.mesh_message(mesh_seq, mesh))
                last_mesh = mesh
```

- [ ] **Step 4: Run the service + wire tests to verify they pass**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_service.py tests/test_slam_wire.py -v`
Expected: PASS. (`test_serve_survives_bad_client_and_keeps_serving` and the `_effective_kwargs` tests are unaffected by the send-format change.)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/service.py host/tests/test_slam_service.py
git commit -m "feat(slam): service sends pose-per-frame + mesh-when-ready (no full-traj resend)"
```

---

## Task 9: Client dispatch + trajectory delta accumulation (`RemoteSlamWorker._recv_loop`)

**Files:**
- Modify: `host/src/roomscan/slam/remote.py` — `__init__` (`remote.py:26-42`, add the trajectory list) and `_recv_loop` (`remote.py:103-120`)
- Test: `host/tests/test_slam_remote.py` (append; existing tests remain and must stay green)

**Interfaces:**
- Consumes: `wire.POSE`, `wire.MESH`, `wire.arrays_to_mesh` (Tasks 7–8); the service's tagged message stream (Task 8).
- Produces: `latest()` still returns `(mesh, trajectory, FrameStep) | None` (Global Constraint). `trajectory` now grows on the client from appended `pose` deltas rather than being resent each frame.

- [ ] **Step 1: Write the failing test**

Append to `host/tests/test_slam_remote.py`:

```python
def test_remote_worker_accumulates_trajectory_from_pose_deltas():
    port, lsock, th, srv = _serve_on_ephemeral()
    rw = RemoteSlamWorker(W, H, addr=f"127.0.0.1:{port}", fov_h=55.0, fov_v=42.0)
    assert rw.connect() is True
    rw.start()
    depth = np.full((H, W), 500.0, np.float32)
    quat = np.array([1.0, 0.0, 0.0, 0.0], np.float32)
    last_len = 0
    for _ in range(400):                       # drive enough frames to grow the traj
        rw.submit(depth, quat, None)
        time.sleep(0.01)
        got = rw.latest()
        if got is not None:
            _mesh, traj, _step = got
            last_len = len(traj)
            if last_len >= 3:                  # trajectory grew from >=3 pose deltas
                break
    rw.stop(); lsock.close(); th.join(timeout=2)
    assert last_len >= 3
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_remote.py -k accumulates_trajectory -v`
Expected: FAIL — the current `_recv_loop` reads `res["pose"]`/`res["traj"]` unconditionally and would `KeyError` on the new tagged pose message (which has no `"traj"`), so `latest()` never publishes.

- [ ] **Step 3: Add the trajectory list + rewrite `_recv_loop`**

In `host/src/roomscan/slam/remote.py` `__init__`, add next to the mesh-cache state (`remote.py:41-42`):

```python
        self._last_mesh_seq = -1
        self._last_mesh = None
        self._trajectory = []          # accumulated from pose deltas (no full-traj resend)
```

Replace `_recv_loop` (`remote.py:103-120`) with the tag dispatch:

```python
    def _recv_loop(self) -> None:
        while not self._stop_evt.is_set():
            try:
                res = wire.recv_message(self._sock)
            except OSError:
                break
            if res is None:
                break
            if res.get("type") == wire.MESH:
                if res["mesh_seq"] != self._last_mesh_seq and "mesh_v" in res:
                    self._last_mesh = wire.arrays_to_mesh(res)
                    self._last_mesh_seq = res["mesh_seq"]
                continue
            # pose message: update step, grow the trajectory, publish
            step = FrameStep(pose=np.asarray(res["pose"], np.float64),
                             fitness=res["fitness"], rmse=res["rmse"],
                             tracking_lost=res["tracking_lost"], slam_ms=res["slam_ms"])
            self._tracking_lost_count = res["tracking_lost_count"]
            self._trajectory.append(np.asarray(res["pose"], np.float64))
            with self._out_lock:
                self._out_slot = (self._last_mesh, list(self._trajectory), step)
```

- [ ] **Step 4: Run the remote suite to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_slam_remote.py -v`
Expected: PASS — the new trajectory test plus the existing `test_remote_worker_publishes_results`, `test_remote_worker_forwards_client_mapper_cfg_to_service`, `test_start_is_idempotent_...`, and the two unreachable-service tests (which never touch `_recv_loop`).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/slam/remote.py host/tests/test_slam_remote.py
git commit -m "feat(slam): remote client dispatches pose/mesh tags + accumulates trajectory deltas"
```

---

## Task 10: Replay validation on both backends (before/after render-fps)

**Files:**
- No source changes. Runs the panel over a map-growing capture and records the viewport render-fps early-third vs late-third.
- Uses: an existing capture under `host/captures/` (e.g. `captures/phase6_motion_ref.bin`, referenced in `slam/config.py`'s comments) or any `--replay` capture with enough motion to grow the map.

**Success criteria (from the spec):** render-fps **≥30 sustained (target 120) and flat** (late-third / early-third ≈ 1.0), on **both** `backend=local` and `backend=remote`. Compare against the pre-Component-A code's render-fps on the same capture to quantify the win.

- [ ] **Step 1: Run the full host test suite (regression gate)**

Run: `.venv\Scripts\python.exe -m pytest tests/ -q`
Expected: PASS (no regressions across the whole suite; confirms Tasks 1–9 compose cleanly).

- [ ] **Step 2: Locate a map-growing capture**

Run: `ls host/captures/*.bin`
Pick one with sustained motion (the map must grow over the run so the "flat as it grows" property is actually exercised). Note its path as `<CAP>`.

- [ ] **Step 3: Local backend — capture render-fps early vs late**

Confirm `[slam]` in `roomscan.toml` has `backend = "local"` (or unset — local is the default). Run the panel with SLAM enabled over the capture:

Run: `.venv\Scripts\python.exe -m roomscan.panel --replay <CAP> --panel`
Then in the panel: enable the **SLAM** checkbox, let it play through, and watch the periodic `view fps: N` lines in the **Events** log (added in Task 6). Record the value in roughly the first third of the run and the last third.
Expected: `view fps` ≥ 30 throughout, and the late-third value ≈ the early-third value (flat — no downward drift as the map grows).

- [ ] **Step 4: Remote backend — repeat with the GPU container service**

Start the container SLAM service (per `docs/superpowers/specs/2026-07-13-slam-gpu-container-service-design.md`), set `[slam] backend = "remote"` (and `remote_addr` if non-default) in `roomscan.toml`, then repeat Step 3.
Expected: `view fps` ≥ 30 and flat, same as local. (If the service is unreachable the panel logs the local-fallback line from `backend.make_slam_worker` — in that case the run is really local; ensure the service is up before trusting the remote result.)

- [ ] **Step 5: Record the before/after and commit the note**

Capture the numbers (pre-Component-A baseline vs. post, both backends) into the plan's companion or a short note under `docs/` if the owner wants it tracked, then run the `status-sync` skill before declaring the work shipped (mandated at ship time). Final regression run:

Run: `.venv\Scripts\python.exe -m pytest tests/ -q`
Expected: PASS.

```bash
git add -A
git commit -m "test(slam): validate live-view render-fps flat >=30 on local + remote backends"
```

---

## Self-Review

**1. Spec coverage:**
- Component A / `MeshPrep` + `MeshPacket` (off-thread decimate/split/floor) → Tasks 1–2. ✔
- Adaptive decimation (full-res until it would cost fps) → Task 1 `prepare_packet(decimate=...)` + Task 2 controller. ✔
- Panel throttled-cadence upload + cheap per-tick path preserved → Task 5. ✔
- Floor grid computed in MeshPrep, replacing the per-tick `.cpu().numpy()` → Task 1 (`floor_pts`) + Task 4 (`_upload_floor_grid_from_packet`) + Task 5 (heavy block removed). ✔
- render-fps (viewport) counter → HUD + log → Task 6. ✔
- Config `mesh_upload_hz` / `live_vertex_budget` / `fps_budget_ms` with the spec's defaults → Task 3. ✔
- Component B wire `pose`/`mesh` tag → Task 7. ✔
- Service pose-immediately + mesh-when-new, no full-traj → Task 8. ✔
- Client dispatch + trajectory delta + mesh cache → Task 9. ✔
- Validation: render-fps counter, replay drive both backends, unit tests (MeshPrep decimation, wall split match, wire round-trip, remote trajectory-delta), existing panel tests green → Tasks 1–9 tests + Task 10. ✔
- Shared invariant `latest() -> (mesh, trajectory, FrameStep)` preserved, decimation display-only, final map byte-identical → Global Constraints + Task 8 design note + Task 5 (`_slam_last_mesh_obj` feeds full-res worker mesh; save paths untouched). ✔
- Non-goals respected: no GPU-OOM work, no algorithm/map/framing/protocol change, no sensor-rate change → nothing in the plan touches those. ✔

**2. Placeholder scan:** No "TBD"/"add error handling"/"similar to Task N" — every code step carries the actual content. Task 10 is inherently a manual validation task (GUI replay) and lists concrete commands + numeric success gates rather than code. ✔

**3. Type consistency:** `MeshPacket` field names are identical across Tasks 1, 2, 4 (`non_wall_verts/colors/tris`, `wall_verts/colors/tris`, `floor_pts/floor_lines`, `mesh_seq`, `source_vertex_count`, `decimated`, `wall_mode`). `prepare_packet(mesh, *, wall_mode, glow_origin, mesh_seq, vertex_budget, decimate, up)` is called with those exact keywords in Task 2. `MeshPrep.submit(mesh, *, mesh_seq, glow_origin, wall_mode)` / `latest()` / `note_upload_ms(ms)` / `run_once()` / `start()` / `stop()` match between Tasks 2 and 5. `wire.pose_message(...)` / `wire.mesh_message(...)` / `wire.POSE` / `wire.MESH` are defined in Task 7 and consumed with the same names in Tasks 8–9. `_view_fps(now)` / `_record_view_tick(now)` and `render_hud(..., view_fps=...)` match between Task 6's source and tests. ✔

---

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-13-live-view-fps.md`. Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

Which approach?
