# Surface Interpolation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let adjacent depth-camera points that are close enough get covered by a triangle mesh instead of drawn as dots, with two switchable adjacency strategies (grid / spatial), in the `roomscan-panel` GUI.

**Architecture:** A new pure-numpy/Open3D module (`surface.py`) supplies two mesh builders — `grid_triangles` (raster-adjacency, runs every frame) and `alpha_shape_mesh` (3D-proximity via Open3D's alpha shape, throttled to ~4Hz). `panel.py` wires panel state → these builders → a second Open3D `TriangleMesh` scene geometry alongside the existing point-cloud dots, splitting each frame's valid points into "covered" (hidden, drawn by the mesh) and "lone" (still dots).

**Tech Stack:** Python 3.11, numpy, Open3D 0.18+ (already a hard dependency — no new packages). Design doc: `docs/superpowers/plans/2026-07-09-surface-interpolation-design.md`.

## Global Constraints

- Python `>=3.11,<3.13`; `numpy>=1.26`; `open3d>=0.18` (`host/pyproject.toml`) — no new dependencies.
- `ViewerConfig` (`host/src/roomscan/config.py`) stays a single flat `[viewer]` TOML table — new fields are flat `key = value` lines only, no nested tables/arrays (see that file's module docstring).
- No CLI flags for panel-only settings — config-file-only, filled via `_fill_panel_fields`/`_PANEL_FIELDS` in `panel.py`, exactly like `point_size`/`ir_colormap`/`near_mode` already work.
- `panel.py`'s GUI-widget wiring itself has no automated tests in this codebase — only pure helper functions are unit-tested (see `test_panel.py`'s module docstring: "The Open3D gui shell itself is supervised-run verified"). Tasks that touch only widget construction/wiring are verified by running the full existing test suite (regression gate) plus a final supervised manual run, not new automated tests.
- All new/changed pure functions must ship with unit tests in this plan; no placeholders.

---

### Task 1: `Deprojector.grid()` — preserve raster adjacency

**Files:**
- Modify: `host/src/roomscan/deproject.py`
- Test: `host/tests/test_deproject.py`

**Interfaces:**
- Produces: `Deprojector.grid(depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]` returning `(pts, valid)` where `pts` is `(h, w, 3)` float64 metres (garbage, not NaN, at invalid cells) and `valid` is `(h, w)` bool. `Deprojector.__call__` keeps its existing signature/behavior (verified byte-identical below), now implemented in terms of `grid()`.

- [ ] **Step 1: Write the failing tests**

Append to `host/tests/test_deproject.py`:

```python
def test_grid_matches_call_at_valid_cells():
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.array([[0.0, 1000.0, np.inf], [1500.0, np.nan, 2000.0], [500.0, 500.0, 500.0]],
                     dtype=np.float32)
    pts_grid, valid = d.grid(depth)
    assert pts_grid.shape == (3, 3, 3)
    assert valid.shape == (3, 3)
    assert valid.tolist() == [[False, True, False], [True, False, True], [True, True, True]]
    assert np.allclose(pts_grid[valid], d(depth))


def test_grid_center_zone_matches_call():
    d = Deprojector(width=3, height=3, fov_h_deg=90.0, fov_v_deg=90.0)
    depth = np.full((3, 3), 2000.0, dtype=np.float32)
    pts_grid, valid = d.grid(depth)
    assert valid.all()
    assert np.allclose(pts_grid[1, 1], [0.0, 0.0, 2.0], atol=1e-9)
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_deproject.py -v -k grid`
Expected: FAIL with `AttributeError: 'Deprojector' object has no attribute 'grid'`

- [ ] **Step 3: Implement `grid()` and refactor `__call__` to use it**

In `host/src/roomscan/deproject.py`, replace the existing `__call__` method:

```python
    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        z = depth_mm.astype(np.float64, copy=False)
        valid = np.isfinite(z) & (z > 0.0) & (z < self.max_range_mm)
        x = z * self._tan_x
        y = z * self._tan_y
        y = np.broadcast_to(y, z.shape)
        x = np.broadcast_to(x, z.shape)
        return np.stack([x[valid], y[valid], z[valid]], axis=1) / 1000.0
```

with:

```python
    def grid(self, depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Like __call__, but returns the full (h, w) raster shape instead of
        filtering + flattening -- callers that need row/col adjacency (surface
        triangulation) can't recover it from __call__'s already-flattened
        output. Returns (pts, valid): pts is (h, w, 3) metres, with garbage
        (not NaN, cheap) at invalid cells -- callers must consult `valid`
        before using a cell. valid is the (h, w) bool mask __call__ already
        computed internally, just not returned there."""
        z = depth_mm.astype(np.float64, copy=False)
        valid = np.isfinite(z) & (z > 0.0) & (z < self.max_range_mm)
        x = np.broadcast_to(z * self._tan_x, z.shape)
        y = np.broadcast_to(z * self._tan_y, z.shape)
        return np.stack([x, y, z], axis=-1) / 1000.0, valid

    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        pts, valid = self.grid(depth_mm)
        return pts[valid]
```

- [ ] **Step 4: Run the full deproject test suite to verify no regression and new tests pass**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_deproject.py -v`
Expected: all PASS (including the pre-existing tests — `__call__`'s behavior is unchanged, only re-implemented)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/deproject.py host/tests/test_deproject.py
git commit -m "feat(host): Deprojector.grid() preserves raster adjacency for surface interpolation"
```

---

### Task 2: `surface.py` — `grid_triangles()`

**Files:**
- Create: `host/src/roomscan/surface.py`
- Test: `host/tests/test_surface.py` (new)

**Interfaces:**
- Consumes: nothing from other tasks (pure numpy).
- Produces: `grid_triangles(pts_grid: np.ndarray, valid: np.ndarray, threshold_pct: float) -> tuple[np.ndarray, np.ndarray]` returning `(triangles, covered)` — `triangles` is `(T, 3)` int64 flat-index (`r*w+c`) vertex triples; `covered` is `(h*w,)` bool, `True` where that grid point participates in >=1 emitted triangle.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_surface.py`:

```python
import numpy as np

from roomscan.surface import grid_triangles


def _flat_grid(z_grid):
    h, w = z_grid.shape
    pts = np.zeros((h, w, 3))
    pts[..., 2] = z_grid
    return pts


def test_flat_surface_fully_triangulated():
    pts = _flat_grid(np.ones((3, 3)))
    valid = np.ones((3, 3), dtype=bool)
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (8, 3)   # 2x2 quads, 2 triangles each
    assert covered.all()


def test_step_edge_refuses_bridging_triangle():
    # (2,3) grid: cols 0-1 near (z=1.0), col 2 far (z=2.0, a 100% jump).
    # The quad spanning cols 1-2 straddles the step and must be refused;
    # col 2's points have no other neighbor quad (they're the last column),
    # so they end up uncovered.
    z = np.array([[1.0, 1.0, 2.0], [1.0, 1.0, 2.0]])
    pts = _flat_grid(z)
    valid = np.ones((2, 3), dtype=bool)
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (2, 3)                  # only the cols 0-1 quad
    assert covered.tolist() == [True, True, False, True, True, False]


def test_invalid_row_blocks_triangles_that_touch_it():
    # (4,2) grid, row 1 invalid: row 0 only quads with row 1 (blocked, stays
    # uncovered); rows 2-3 are both valid and quad normally.
    pts = _flat_grid(np.ones((4, 2)))
    valid = np.ones((4, 2), dtype=bool)
    valid[1, :] = False
    triangles, covered = grid_triangles(pts, valid, threshold_pct=5.0)
    assert triangles.shape == (2, 3)                  # only the rows 2-3 quad
    assert covered.tolist() == [False, False, False, False, True, True, True, True]
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_surface.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.surface'`

- [ ] **Step 3: Implement `grid_triangles()`**

Create `host/src/roomscan/surface.py`:

```python
"""Mesh builders for the panel's opt-in surface-interpolation mode: cover
adjacent depth-camera points with a triangle mesh instead of drawing them as
dots, when they're close enough. Two adjacency strategies -- see
docs/superpowers/plans/2026-07-09-surface-interpolation-design.md.
"""
from __future__ import annotations

import numpy as np


def grid_triangles(pts_grid: np.ndarray, valid: np.ndarray,
                   threshold_pct: float) -> tuple[np.ndarray, np.ndarray]:
    """Raster-adjacency triangulation: pts_grid is (h, w, 3), valid is (h, w)
    bool (both from Deprojector.grid()). For each 2x2 cell, considers two
    candidate triangles (upper-left: (r,c),(r,c+1),(r+1,c); lower-right:
    (r,c+1),(r+1,c+1),(r+1,c)). A triangle is emitted iff all three corners
    are valid AND every pairwise depth gap among them satisfies
    abs(za - zb) <= threshold_pct/100 * min(za, zb) -- relative to the
    nearer point, so one threshold behaves consistently from 0.5m to 5m, and
    a foreground/background straddle (gap large relative to either depth) is
    refused. Fully vectorized (no Python-level loop over cells).

    Returns (triangles, covered): triangles is (T, 3) int64 flat-index
    (r*w+c) vertex triples; covered is (h*w,) bool, True where a grid point
    participates in >=1 emitted triangle (i.e. should be hidden from the dot
    cloud)."""
    h, w, _ = pts_grid.shape
    z = pts_grid[..., 2]
    idx = np.arange(h * w).reshape(h, w)

    def close(za, zb):
        return np.abs(za - zb) <= (threshold_pct / 100.0) * np.minimum(za, zb)

    z00, z01, z10, z11 = z[:-1, :-1], z[:-1, 1:], z[1:, :-1], z[1:, 1:]
    v00, v01, v10, v11 = valid[:-1, :-1], valid[:-1, 1:], valid[1:, :-1], valid[1:, 1:]
    i00, i01, i10, i11 = idx[:-1, :-1], idx[:-1, 1:], idx[1:, :-1], idx[1:, 1:]

    ul_ok = v00 & v01 & v10 & close(z00, z01) & close(z00, z10) & close(z01, z10)
    lr_ok = v01 & v11 & v10 & close(z01, z11) & close(z01, z10) & close(z11, z10)

    ul_tris = np.stack([i00[ul_ok], i01[ul_ok], i10[ul_ok]], axis=1)
    lr_tris = np.stack([i01[lr_ok], i11[lr_ok], i10[lr_ok]], axis=1)
    triangles = np.concatenate([ul_tris, lr_tris], axis=0).astype(np.int64)

    covered = np.zeros(h * w, dtype=bool)
    if triangles.size:
        covered[triangles.ravel()] = True
    return triangles, covered
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_surface.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/surface.py host/tests/test_surface.py
git commit -m "feat(host): grid_triangles() -- raster-adjacency surface triangulation"
```

---

### Task 3: `surface.py` — `alpha_shape_mesh()`

**Files:**
- Modify: `host/src/roomscan/surface.py`
- Test: `host/tests/test_surface.py`

**Interfaces:**
- Consumes: an `o3d.geometry.PointCloud` the caller has already populated with `.points` and `.colors`.
- Produces: `alpha_shape_mesh(pcd, threshold_m: float) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]` returning `(mesh, covered)` — `mesh` is the alpha-shape result (own vertex/vertex_color arrays, vertex-color-carrying, NOT indexed into `pcd`); `covered` is `(N,)` bool over `pcd`'s original point order.

- [ ] **Step 1: Write the failing tests**

Append to `host/tests/test_surface.py`:

```python
import open3d as o3d

from roomscan.surface import alpha_shape_mesh


def _make_pcd(points, colors=None):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    if colors is not None:
        pcd.colors = o3d.utility.Vector3dVector(colors)
    return pcd


def test_alpha_shape_too_few_points_returns_empty_uncovered():
    pcd = _make_pcd(np.random.default_rng(0).random((3, 3)))
    mesh, covered = alpha_shape_mesh(pcd, threshold_m=0.1)
    assert len(mesh.triangles) == 0
    assert covered.tolist() == [False, False, False]


def test_alpha_shape_covers_a_flat_patch_but_not_a_far_outlier():
    # A small flat patch (mimics a single depth-camera sweep, which is
    # inherently surface-like) plus one point far away. Alpha shape builds a
    # 2D boundary surface -- points strictly interior to a genuine 3D blob
    # would NOT all be covered, but a planar patch's points all sit on that
    # boundary, so they should all end up covered; the outlier can't join
    # any simplex within the threshold and must stay uncovered.
    rng = np.random.default_rng(0)
    xs, ys = np.meshgrid(np.linspace(-0.1, 0.1, 6), np.linspace(-0.1, 0.1, 6))
    patch = np.stack([xs.ravel(), ys.ravel(), np.full(36, 1.0) + rng.normal(0, 0.001, 36)], axis=1)
    outlier = np.array([[5.0, 5.0, 5.0]])
    pts = np.vstack([patch, outlier])
    colors = np.tile([0.2, 0.4, 0.6], (37, 1))
    pcd = _make_pcd(pts, colors)

    mesh, covered = alpha_shape_mesh(pcd, threshold_m=0.08)
    assert len(mesh.triangles) > 0
    assert mesh.has_vertex_colors()
    assert covered[:36].all()
    assert not covered[36]
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_surface.py -v -k alpha_shape`
Expected: FAIL with `ImportError: cannot import name 'alpha_shape_mesh'`

- [ ] **Step 3: Implement `alpha_shape_mesh()`**

Append to `host/src/roomscan/surface.py`:

```python
def alpha_shape_mesh(pcd, threshold_m: float):
    """3D-proximity ("spatial") adjacency via Open3D's alpha-shape
    reconstruction: alpha *is* the distance threshold. pcd is an
    o3d.geometry.PointCloud the caller has already populated with .points
    (and, for coloring, .colors).

    Needs >=4 non-degenerate points; with fewer (or a degenerate/coplanar
    configuration Qhull rejects) returns (empty mesh, all-False) rather than
    raising -- the caller falls back to drawing every point as a dot that
    frame.

    Returns (mesh, covered): mesh is the raw create_from_point_cloud_alpha_shape
    result (own vertex/vertex_color arrays -- NOT indexed into pcd, and not
    even the same vertex count: alpha shape drops points that don't end up on
    the reconstructed 2D boundary, and reorders + reindexes the rest).
    covered is an (N,) bool over pcd's ORIGINAL point order, recovered by
    nearest-neighbor matching each mesh vertex back to pcd within a 1e-4 m
    tolerance -- empirically, alpha shape's own float32 round-trip only ever
    displaces a vertex by ~1e-7 m at scanner scale, so this tolerance is
    generous against that noise while staying far below any real point
    spacing (no risk of matching the wrong point)."""
    import open3d as o3d

    n = len(pcd.points)
    empty = o3d.geometry.TriangleMesh()
    covered = np.zeros(n, dtype=bool)
    if n < 4:
        return empty, covered
    try:
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha=threshold_m)
    except RuntimeError:
        return empty, covered
    mesh_verts = np.asarray(mesh.vertices)
    if len(mesh_verts) == 0:
        return mesh, covered
    tree = o3d.geometry.KDTreeFlann(pcd)
    tol2 = (1e-4) ** 2
    for v in mesh_verts:
        _, idx, dist2 = tree.search_knn_vector_3d(v, 1)
        if dist2[0] <= tol2:
            covered[idx[0]] = True
    return mesh, covered
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_surface.py -v`
Expected: all PASS (5 tests total: 3 from Task 2 + 2 from this task)

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/surface.py host/tests/test_surface.py
git commit -m "feat(host): alpha_shape_mesh() -- spatial-adjacency surface triangulation"
```

---

### Task 4: Config fields + panel-field wiring

**Files:**
- Modify: `host/src/roomscan/config.py`
- Modify: `host/src/roomscan/panel.py` (`_PANEL_FIELDS` only)
- Test: `host/tests/test_config.py`
- Test: `host/tests/test_panel.py`

**Interfaces:**
- Produces: `ViewerConfig.surface_enabled: bool = False`, `.surface_mode: str = "grid"`, `.surface_threshold_pct: float = 4.0`. `panel._PANEL_FIELDS` includes these three names, so `_fill_panel_fields(args)` fills them from `roomscan.toml` (or built-in defaults) exactly like `point_size` etc.

- [ ] **Step 1: Write the failing tests**

In `host/tests/test_config.py`, extend `test_panel_fields_have_expected_defaults_on_fresh_config`:

```python
def test_panel_fields_have_expected_defaults_on_fresh_config():
    cfg = ViewerConfig()
    assert cfg.point_size == 5.0
    assert cfg.ir_colormap == "gray"
    assert cfg.ir_freeze_range is False
    assert cfg.panel_width == 340
    assert cfg.surface_enabled is False
    assert cfg.surface_mode == "grid"
    assert cfg.surface_threshold_pct == 4.0
```

Extend `test_save_then_load_roundtrip_panel_fields`:

```python
def test_save_then_load_roundtrip_panel_fields(tmp_path):
    path = tmp_path / "roomscan.toml"
    original = ViewerConfig(ir_colormap="turbo", ir_freeze_range=True,
                             point_size=8.0, panel_width=400,
                             near_mode="emphasis", near_cutoff_m=2.25, near_emphasis=0.8,
                             surface_enabled=True, surface_mode="spatial",
                             surface_threshold_pct=7.5)
    saved_path = original.save(path)
    assert saved_path == path

    loaded = ViewerConfig.load(path)
    assert loaded == original
    assert loaded.ir_colormap == "turbo"
    assert loaded.ir_freeze_range is True
    assert loaded.point_size == 8.0
    assert loaded.panel_width == 400
    assert loaded.near_mode == "emphasis"
    assert loaded.near_cutoff_m == 2.25
    assert loaded.near_emphasis == 0.8
    assert loaded.surface_enabled is True
    assert loaded.surface_mode == "spatial"
    assert loaded.surface_threshold_pct == 7.5
```

Extend `test_load_old_config_file_missing_panel_keys_falls_back_to_defaults`:

```python
def test_load_old_config_file_missing_panel_keys_falls_back_to_defaults(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text(
        '[viewer]\n'
        'color = "reflectance"\n'
        'fov_h = 54.65\n'
        'fov_v = 42.50\n'
        'replay_fps = 25.0\n'
        'port = "COM7"\n',
        encoding="utf-8",
    )
    cfg = ViewerConfig.load(path)
    assert cfg.color == "reflectance"  # old fields still honored
    assert cfg.point_size == 5.0
    assert cfg.ir_colormap == "gray"
    assert cfg.ir_freeze_range is False
    assert cfg.panel_width == 340
    assert cfg.surface_enabled is False
    assert cfg.surface_mode == "grid"
    assert cfg.surface_threshold_pct == 4.0
```

In `host/tests/test_panel.py`, extend `_bare_args`:

```python
def _bare_args(**over):
    ns = argparse.Namespace(point_size=None, ir_colormap=None, ir_freeze_range=None,
                            panel_width=None, near_mode=None, near_cutoff_m=None,
                            near_emphasis=None, surface_enabled=None, surface_mode=None,
                            surface_threshold_pct=None)
    for k, v in over.items():
        setattr(ns, k, v)
    return ns
```

Extend `test_fill_panel_fields_uses_builtin_defaults_when_no_config`:

```python
def test_fill_panel_fields_uses_builtin_defaults_when_no_config(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))   # empty dir -> no roomscan.toml
    args = _bare_args()
    _fill_panel_fields(args)
    assert args.point_size == 5.0
    assert args.ir_colormap == "gray"
    assert args.ir_freeze_range is False
    assert args.panel_width == 340
    assert args.near_mode == "window"
    assert args.near_cutoff_m == 1.5
    assert args.near_emphasis == 0.5
    assert args.surface_enabled is False
    assert args.surface_mode == "grid"
    assert args.surface_threshold_pct == 4.0
```

Extend `test_fill_panel_fields_pulls_from_config_file`:

```python
def test_fill_panel_fields_pulls_from_config_file(tmp_path, monkeypatch):
    monkeypatch.setenv("APPDATA", str(tmp_path))
    ViewerConfig(ir_colormap="turbo", ir_freeze_range=True,
                 point_size=5.0, panel_width=400,
                 surface_enabled=True, surface_mode="spatial", surface_threshold_pct=7.5).save()
    args = _bare_args()
    _fill_panel_fields(args)
    assert args.ir_colormap == "turbo"
    assert args.ir_freeze_range is True
    assert args.point_size == 5.0
    assert args.panel_width == 400
    assert args.surface_enabled is True
    assert args.surface_mode == "spatial"
    assert args.surface_threshold_pct == 7.5
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_config.py tests/test_panel.py -v -k "panel_fields or surface"`
Expected: FAIL — `TypeError: ViewerConfig.__init__() got an unexpected keyword argument 'surface_enabled'`, and `AttributeError`/`AssertionError` on the `_bare_args`/`_fill_panel_fields` tests.

- [ ] **Step 3: Add the fields**

In `host/src/roomscan/config.py`, in the `ViewerConfig` dataclass, after `near_emphasis: float = 0.5`:

```python
    near_mode: str = "window"          # near-contrast: off|window|emphasis|equalize
    near_cutoff_m: float = 1.5         # window-mode near/far boundary (metres)
    near_emphasis: float = 0.5         # emphasis-mode strength 0..1
    surface_enabled: bool = False
    surface_mode: str = "grid"          # "grid" | "spatial"
    surface_threshold_pct: float = 4.0
```

In `host/src/roomscan/panel.py`, replace the `_PANEL_FIELDS` tuple:

```python
_PANEL_FIELDS = ("point_size", "ir_colormap", "ir_freeze_range", "panel_width",
                 "near_mode", "near_cutoff_m", "near_emphasis")
```

with:

```python
_PANEL_FIELDS = ("point_size", "ir_colormap", "ir_freeze_range", "panel_width",
                 "near_mode", "near_cutoff_m", "near_emphasis",
                 "surface_enabled", "surface_mode", "surface_threshold_pct")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd host && .venv/Scripts/python.exe -m pytest tests/test_config.py tests/test_panel.py -v`
Expected: all PASS

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/config.py host/src/roomscan/panel.py host/tests/test_config.py host/tests/test_panel.py
git commit -m "feat(host): persist surface-interpolation settings in roomscan.toml"
```

---

### Task 5: Panel render pipeline — mesh geometry, camera bounds, grid + spatial wiring

**Files:**
- Modify: `host/src/roomscan/panel.py`

**Interfaces:**
- Consumes: `Deprojector.grid()` (Task 1), `grid_triangles()`/`alpha_shape_mesh()` (Tasks 2-3), `ViewerConfig.surface_*`/`_PANEL_FIELDS` (Task 4).
- Produces: `ControlPanel.surface_enabled: bool`, `.surface_mode: str`, `.surface_threshold_pct: float` instance attributes (read by Task 6's UI); `ControlPanel._remove_mesh_geometry()` (called by Task 6's checkbox handler).

No new automated tests for this task (GUI-scene wiring — see Global Constraints); verified by the full existing suite (regression) plus Task 7's supervised run.

- [ ] **Step 1: Add module-level constants and the import**

In `host/src/roomscan/panel.py`, replace:

```python
_USECASES = [(0, "AR_RANGE (~32 fps)"), (1, "AR_PRECISION (~28 fps)")]
_COLOR_MODES = ("depth", "reflectance", "confidence")
_IR_COLORMAPS = ("gray", "turbo")
_IR_UPSCALE = 6                 # 54x42 zones -> 324x252 px, nearest-neighbor
_GEOM = "cloud"
```

with:

```python
_USECASES = [(0, "AR_RANGE (~32 fps)"), (1, "AR_PRECISION (~28 fps)")]
_COLOR_MODES = ("depth", "reflectance", "confidence")
_IR_COLORMAPS = ("gray", "turbo")
_SURFACE_MODES = ("grid", "spatial")
_IR_UPSCALE = 6                 # 54x42 zones -> 324x252 px, nearest-neighbor
_GEOM = "cloud"
_MESH_GEOM = "surface"
```

Add the import, after `from .sources import FileSource, Recorder, SerialSource, pump`:

```python
from .surface import alpha_shape_mesh, grid_triangles
```

- [ ] **Step 2: Add `__init__` state**

Replace:

```python
        # render state
        self.deproj: Deprojector | None = None
        self.pcd = o3d.geometry.PointCloud()
        self._camera_set = False
```

with:

```python
        # render state
        self.deproj: Deprojector | None = None
        self.pcd = o3d.geometry.PointCloud()
        self.mesh = o3d.geometry.TriangleMesh()
        self._last_all_pts: np.ndarray | None = None      # full valid-point set, for camera framing
        self._camera_set = False
```

Replace:

```python
        # near-contrast state
        self.near_mode = args.near_mode if getattr(args, "near_mode", None) in _NEAR_MODES else "window"
        self.near_cutoff_m = float(getattr(args, "near_cutoff_m", 1.5) or 1.5)
        self.near_emphasis = float(getattr(args, "near_emphasis", 0.5) or 0.5)
        self._ir_last_auto: tuple[float, float] | None = None
        self._ir_frozen: tuple[float, float] | None = None
        self._ir_unavailable_shown = False
```

with:

```python
        # near-contrast state
        self.near_mode = args.near_mode if getattr(args, "near_mode", None) in _NEAR_MODES else "window"
        self.near_cutoff_m = float(getattr(args, "near_cutoff_m", 1.5) or 1.5)
        self.near_emphasis = float(getattr(args, "near_emphasis", 0.5) or 0.5)
        self._ir_last_auto: tuple[float, float] | None = None
        self._ir_frozen: tuple[float, float] | None = None
        self._ir_unavailable_shown = False

        # surface-interpolation state (opt-in: adjacent points close enough
        # get covered by a mesh instead of drawn as dots -- see docs/
        # superpowers/plans/2026-07-09-surface-interpolation-design.md)
        self.surface_enabled = bool(getattr(args, "surface_enabled", False))
        self.surface_mode = args.surface_mode if getattr(args, "surface_mode", None) in _SURFACE_MODES else "grid"
        self.surface_threshold_pct = float(getattr(args, "surface_threshold_pct", 4.0) or 4.0)
        self._last_surface_rebuild = 0.0       # spatial-mode throttle timer
        self._surface_covered: np.ndarray | None = None
```

Replace:

```python
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = float(getattr(args, "point_size", 5.0))
        self._dark_bg = True
```

with:

```python
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = float(getattr(args, "point_size", 5.0))
        self.mesh_material = rendering.MaterialRecord()
        self.mesh_material.shader = "defaultUnlit"
        self._dark_bg = True
```

- [ ] **Step 3: Replace `_show_cloud` and `_reset_camera` with geometry-management methods**

Replace:

```python
    def _show_cloud(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        sc.add_geometry(_GEOM, self.pcd, self.material)
        if not self._camera_set and len(self.pcd.points):
            self._reset_camera()

    def _reset_camera(self):
        bounds = self.pcd.get_axis_aligned_bounding_box()
        ext = float(bounds.get_extent().max())
        if ext <= 0:
            return
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())  # projection + near/far
        self._cam_target = np.asarray(bounds.get_center(), dtype=np.float64)
        self._cam_radius = ext * 1.8
        self._cam_az = 0.0
        self._apply_camera()
        self._camera_set = True
```

with:

```python
    def _show_geometries(self, all_pts):
        """Push the dot cloud to the scene and (re)frame the camera from the
        FULL valid point set for this frame -- `all_pts` is every valid point
        before the covered/lone split, so framing doesn't shrink once most
        points move into the mesh. The mesh geometry itself is managed
        separately by _show_mesh_geometry/_remove_mesh_geometry, called from
        _render_surface, since only surface mode touches it."""
        self._last_all_pts = all_pts
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        sc.add_geometry(_GEOM, self.pcd, self.material)
        if not self._camera_set and len(all_pts):
            self._reset_camera()

    def _show_mesh_geometry(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if len(self.mesh.triangles) > 0:
            sc.add_geometry(_MESH_GEOM, self.mesh, self.mesh_material)

    def _remove_mesh_geometry(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)

    def _reset_camera(self):
        all_pts = self._last_all_pts
        if all_pts is None or len(all_pts) == 0:
            return
        bounds = self._o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            self._o3d.utility.Vector3dVector(all_pts))
        ext = float(bounds.get_extent().max())
        if ext <= 0:
            return
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())  # projection + near/far
        self._cam_target = np.asarray(bounds.get_center(), dtype=np.float64)
        self._cam_radius = ext * 1.8
        self._cam_az = 0.0
        self._apply_camera()
        self._camera_set = True
```

(`_on_reset_view`'s existing call site, `self._reset_camera()` with no arguments, needs no change — it now reads `self._last_all_pts` instead of `self.pcd`'s bounds.)

- [ ] **Step 4: Add `_render_surface` and `_rebuild_spatial_mesh`**

Add these two new methods directly after `_reset_camera` (same place `_show_cloud`/`_reset_camera` were):

```python
    def _render_surface(self, depth, rot_pts, colors):
        """Split this frame's points into covered (hidden, drawn by the mesh)
        and lone (still dots), per the selected adjacency mode. Always leaves
        self.pcd holding only the lone points -- caller still calls
        _show_geometries(rot_pts) afterward with the FULL point set so camera
        framing isn't affected by the split."""
        h, w = depth.shape
        if self.surface_mode == "spatial":
            now = time.monotonic()
            if now - self._last_surface_rebuild >= _UI_PERIOD:
                self._rebuild_spatial_mesh(rot_pts, colors)
                self._last_surface_rebuild = now
            covered = self._surface_covered
            if covered is None or len(covered) != len(rot_pts):
                covered = np.zeros(len(rot_pts), dtype=bool)
        else:
            pts_grid, valid_grid = self.deproj.grid(depth)
            triangles, covered_grid = grid_triangles(pts_grid, valid_grid, self.surface_threshold_pct)
            covered = covered_grid[valid_grid.ravel()]
            mesh_verts = _rot_xy(pts_grid.reshape(-1, 3), self._rot)
            colors_grid = np.zeros((h * w, 3), dtype=np.float64)
            colors_grid[valid_grid.ravel()] = colors
            self.mesh.vertices = self._o3d.utility.Vector3dVector(mesh_verts)
            self.mesh.vertex_colors = self._o3d.utility.Vector3dVector(colors_grid)
            self.mesh.triangles = self._o3d.utility.Vector3iVector(triangles.astype(np.int32))
            self._show_mesh_geometry()
        self.pcd.points = self._o3d.utility.Vector3dVector(rot_pts[~covered])
        self.pcd.colors = self._o3d.utility.Vector3dVector(colors[~covered])

    def _rebuild_spatial_mesh(self, rot_pts, colors):
        """Throttled to ~_UI_PERIOD by the caller -- alpha shape's 3D
        triangulation is real per-call cost, unlike grid mode's vectorized
        numpy pass."""
        o3d = self._o3d
        if len(rot_pts) < 4:
            self._surface_covered = np.zeros(len(rot_pts), dtype=bool)
            self._remove_mesh_geometry()
            return
        pcd_src = o3d.geometry.PointCloud()
        pcd_src.points = o3d.utility.Vector3dVector(rot_pts)
        pcd_src.colors = o3d.utility.Vector3dVector(colors)
        threshold_m = max((self.surface_threshold_pct / 100.0) * float(np.mean(rot_pts[:, 2])), 1e-6)
        mesh, covered = alpha_shape_mesh(pcd_src, threshold_m)
        self.mesh.vertices = mesh.vertices
        self.mesh.vertex_colors = mesh.vertex_colors
        self.mesh.triangles = mesh.triangles
        self._surface_covered = covered
        self._show_mesh_geometry()
```

- [ ] **Step 5: Wire `_render_frame`**

Replace:

```python
    def _render_frame(self, item):
        o3d = self._o3d
        header, outputs = item
        self._last_item = item
        self._latest_outputs = outputs
        depth = outputs["depth"]
        h, w = depth.shape
        if self.deproj is None:
            self.deproj = Deprojector(w, h, self.args.fov_h, self.args.fov_v)
        pts = self.deproj(depth)
        if len(pts):
            plane = None if self.color_mode == "depth" else outputs.get(self.color_mode)
            if plane is not None:
                valid = np.isfinite(depth) & (depth > 0.0) & (depth < self.deproj.max_range_mm)
                vals = plane[valid].astype(np.float64, copy=False)
            else:
                if self.color_mode != "depth" and not self._color_fallback_warned:
                    self.bus.publish(f"no '{self.color_mode}' plane in stream — coloring by depth")
                    self._color_fallback_warned = True
                vals = pts[:, 2]
            colors = cloud_colors(vals, pts[:, 2], mode=self.near_mode,   # z-based, so pre-rotation
                                  cutoff_m=self.near_cutoff_m, emphasis=self.near_emphasis)
            self.pcd.points = o3d.utility.Vector3dVector(_rot_xy(pts, self._rot))
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            self.pcd.points = o3d.utility.Vector3dVector(pts)
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
        self._show_cloud()
        self._shown += 1
```

with:

```python
    def _render_frame(self, item):
        o3d = self._o3d
        header, outputs = item
        self._last_item = item
        self._latest_outputs = outputs
        depth = outputs["depth"]
        h, w = depth.shape
        if self.deproj is None:
            self.deproj = Deprojector(w, h, self.args.fov_h, self.args.fov_v)
        pts = self.deproj(depth)
        if len(pts):
            plane = None if self.color_mode == "depth" else outputs.get(self.color_mode)
            if plane is not None:
                valid = np.isfinite(depth) & (depth > 0.0) & (depth < self.deproj.max_range_mm)
                vals = plane[valid].astype(np.float64, copy=False)
            else:
                if self.color_mode != "depth" and not self._color_fallback_warned:
                    self.bus.publish(f"no '{self.color_mode}' plane in stream — coloring by depth")
                    self._color_fallback_warned = True
                vals = pts[:, 2]
            colors = cloud_colors(vals, pts[:, 2], mode=self.near_mode,   # z-based, so pre-rotation
                                  cutoff_m=self.near_cutoff_m, emphasis=self.near_emphasis)
            rot_pts = _rot_xy(pts, self._rot)
            if self.surface_enabled:
                self._render_surface(depth, rot_pts, colors)
            else:
                self._remove_mesh_geometry()
                self.pcd.points = o3d.utility.Vector3dVector(rot_pts)
                self.pcd.colors = o3d.utility.Vector3dVector(colors)
            self._show_geometries(rot_pts)
        else:
            self._remove_mesh_geometry()
            self.pcd.points = o3d.utility.Vector3dVector(pts)
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self._show_geometries(pts)
        self._shown += 1
```

- [ ] **Step 6: Run the full test suite to verify no regression**

Run: `cd host && .venv/Scripts/python.exe -m pytest -q`
Expected: all PASS (surface_enabled defaults to False, so `_render_frame`'s behavior for every existing test is unchanged — the new branch is never taken)

- [ ] **Step 7: Commit**

```bash
git add host/src/roomscan/panel.py
git commit -m "feat(host): panel render pipeline for surface interpolation (grid + spatial)"
```

---

### Task 6: Panel UI — Surface group

**Files:**
- Modify: `host/src/roomscan/panel.py`

**Interfaces:**
- Consumes: `self.surface_enabled`/`.surface_mode`/`.surface_threshold_pct` and `self._remove_mesh_geometry()` (Task 5).
- Produces: nothing consumed elsewhere — this is the leaf UI layer.

No new automated tests (GUI widget construction — see Global Constraints); verified by the full test suite (regression) plus Task 7's supervised run.

- [ ] **Step 1: Add the Surface group to `_build_panel`**

In `host/src/roomscan/panel.py`, insert a new group between the end of the View group and the start of the IR Monitor group. Replace:

```python
        view.add_child(vrow)

        # --- IR Monitor ---
```

with:

```python
        view.add_child(vrow)

        # --- Surface (opt-in: interpolate adjacent points into a mesh) ---
        surf = self._group("Surface", open=False)
        self.chk_surface = gui.Checkbox("Enable surface interpolation")
        self.chk_surface.checked = self.surface_enabled
        self.chk_surface.set_on_checked(self._on_surface_enabled)
        surf.add_child(self.chk_surface)
        sg = self._labeled_grid()
        sg.add_child(gui.Label("Adjacency"))
        self.cb_surface_mode = gui.Combobox()
        for m in _SURFACE_MODES:
            self.cb_surface_mode.add_item(m)
        self.cb_surface_mode.selected_index = _SURFACE_MODES.index(self.surface_mode)
        self.cb_surface_mode.set_on_selection_changed(self._on_surface_mode)
        sg.add_child(self.cb_surface_mode)
        sg.add_child(gui.Label("Threshold %"))
        self.sl_surface_threshold = gui.Slider(gui.Slider.DOUBLE)
        self.sl_surface_threshold.set_limits(0.5, 15.0)
        self.sl_surface_threshold.double_value = self.surface_threshold_pct
        self.sl_surface_threshold.set_on_value_changed(self._on_surface_threshold)
        sg.add_child(self.sl_surface_threshold)
        surf.add_child(sg)

        # --- IR Monitor ---
```

- [ ] **Step 2: Add the handlers**

Insert after `_on_near_value` (right before `_show_help`). Replace:

```python
    def _on_near_value(self, value):
        if self.near_mode == "window":
            self.near_cutoff_m = float(value)
        elif self.near_mode == "emphasis":
            self.near_emphasis = float(value)

    def _show_help(self, *_):
```

with:

```python
    def _on_near_value(self, value):
        if self.near_mode == "window":
            self.near_cutoff_m = float(value)
        elif self.near_mode == "emphasis":
            self.near_emphasis = float(value)

    def _on_surface_enabled(self, checked):
        self.surface_enabled = checked
        self.bus.publish(f"surface interpolation -> {'on' if checked else 'off'}")
        if not checked:
            self._remove_mesh_geometry()

    def _on_surface_mode(self, text, index):
        self.surface_mode = text
        self._last_surface_rebuild = 0.0   # force an immediate spatial rebuild on switch
        self.bus.publish(f"surface adjacency -> {text}")

    def _on_surface_threshold(self, value):
        self.surface_threshold_pct = float(value)

    def _show_help(self, *_):
```

- [ ] **Step 3: Run the full test suite to verify no regression**

Run: `cd host && .venv/Scripts/python.exe -m pytest -q`
Expected: all PASS

- [ ] **Step 4: Commit**

```bash
git add host/src/roomscan/panel.py
git commit -m "feat(host): Surface group UI -- enable/adjacency/threshold controls"
```

---

### Task 7: Supervised verification

**Files:** none (verification only).

This project's convention (documented in `test_panel.py`'s module docstring) is that the Open3D GUI shell itself has no automated test coverage — it's verified by a supervised run, the same way firmware changes are verified on-target rather than by a unit test. This task is that run.

- [ ] **Step 1: Run the full automated suite one more time as a final regression gate**

Run: `cd host && .venv/Scripts/python.exe -m pytest -q`
Expected: all PASS

- [ ] **Step 2: Hand off to the user for a live visual check**

Ask the user to run (from `host/`, with a real display — this dev box has no GPU/display for Open3D's offscreen path, so this step cannot be done headlessly):

```
.venv/Scripts/python.exe -m roomscan.panel --replay synthetic.bin --panel
```

or, for a more realistic scan, `--replay ../recordings/2026-07-08-room-scan.bin`. Checklist to confirm:

- With Surface off (default): behavior is pixel-identical to before this plan — dots only.
- Expand the new "Surface" group, check "Enable surface interpolation" (Grid mode, default 4% threshold): flat regions (walls/floor) render as a shaded mesh instead of dots; a foreground object's silhouette against a background wall still shows a gap (not bridged).
- Drag the Threshold % slider: lower values shrink the mesh back toward dots at depth edges/noisy regions; higher values bridge more.
- Switch Adjacency to Spatial: mesh still forms (may look slightly different from Grid at the same threshold — expected, different algorithm); the status fps counter should stay reasonable (spatial mode's mesh only rebuilds ~4x/sec, not every frame).
- Rotate 90 / Reset / camera orbit-pan-zoom all still work with surface mode on.
- `--save-config`, relaunch: Surface group's checkbox/mode/threshold restore to what was set.

- [ ] **Step 3: Record the outcome**

If the user reports an issue, treat it as a bug against the specific task above (Task 5 for render-pipeline issues, Task 6 for UI issues) rather than closing out this task — fix, re-run the affected task's steps, and re-verify here.
