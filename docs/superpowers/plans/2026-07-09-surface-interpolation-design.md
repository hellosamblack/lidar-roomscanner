# Surface interpolation for adjacent points — design

Status: approved, pending implementation plan.
Owner: panel (`host/src/roomscan/panel.py`), Phase 3.5 GUI panel follow-on.

## Goal

Today the panel's `SceneWidget` renders every deprojected depth zone as an
independent dot (`o3d.geometry.PointCloud`, `defaultUnlit`, tunable point
size). At close range or on flat surfaces (walls, floors) adjacent zones are
close enough in 3D that a triangulated surface reads far better than a sparse
dot field — while zones that straddle a real depth discontinuity (an object's
silhouette against a distant wall) must **not** be bridged, or the surface
lies about the scene's geometry.

Add an opt-in "surface interpolation" mode: adjacent points closer than a
tunable threshold are covered by a triangle mesh instead of drawn as dots;
points with no qualifying neighbor stay dots. Two adjacency strategies,
switchable at runtime:

- **Grid** (default): adjacency = neighboring pixels in the depth raster.
  Cheap, no spatial search, and is the natural relationship for a
  raster-shaped ToF frame.
- **Spatial**: adjacency = 3D proximity regardless of grid position, for
  cases grid-adjacency under-connects. Costs a per-rebuild 3D triangulation,
  so it's throttled (see below).

## 1. `Deprojector.grid()` (new method, `host/src/roomscan/deproject.py`)

The existing `Deprojector.__call__` filters to valid zones and flattens,
which discards row/col adjacency — the very thing grid-mode triangulation
needs. Add a sibling method that preserves shape:

```python
def grid(self, depth_mm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Like __call__, but keeps the (h, w) raster shape instead of filtering
    + flattening. Returns (pts_grid, valid): pts_grid is (h, w, 3) metres
    (garbage, not NaN, at invalid cells -- cheap and fine since callers must
    consult `valid` before using a cell); valid is the (h, w) bool mask
    __call__ already computes internally."""
```

Purely additive — `__call__` and all existing call sites (`viewer.py`,
`panel.py`'s current path, `tools/panel_view.py`, `tools/measure_scene.py`)
are untouched.

## 2. Mesh builders (new module `host/src/roomscan/surface.py`)

Pure functions, unit-tested without Open3D GUI state (matches this
codebase's existing style — `_rot_xy`, `_orbit_eye` in `panel.py` are tested
the same way).

```python
def grid_triangles(pts_grid, valid, threshold_pct) -> tuple[np.ndarray, np.ndarray]:
    """pts_grid: (h,w,3), valid: (h,w) bool, threshold_pct: max relative depth
    gap (percent) allowed between any two corners of a triangle for it to be
    emitted.

    For each 2x2 cell, considers two candidate triangles (upper-left:
    (r,c),(r,c+1),(r+1,c); lower-right: (r,c+1),(r+1,c+1),(r+1,c)). A
    triangle is emitted iff all three corners are valid AND every pairwise
    depth gap among them satisfies abs(za - zb) <= threshold_pct/100 *
    min(za, zb) -- relative to the nearer point, so one threshold behaves
    consistently from 0.5m to 5m, and a foreground/background straddle
    (gap large relative to either depth) is refused.

    Returns (triangles (T,3) int array of flat r*w+c vertex indices,
    covered (h*w,) bool -- True where a grid point participates in >=1
    emitted triangle, i.e. should be hidden from the dot cloud).
    """


def alpha_shape_triangles(pcd, threshold_m) -> tuple[np.ndarray, np.ndarray]:
    """Wraps o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape
    (alpha = threshold_m). Returns (triangles, covered) in the same shape as
    grid_triangles's return, computed from the mesh's referenced-vertex set
    vs. pcd's full point count. Not pure numpy (needs an o3d PointCloud) but
    still unit-testable against small synthetic clouds (a flat cluster fully
    covered; an isolated outlier point left uncovered)."""
```

Threshold semantics differ deliberately: grid mode's threshold is a
**percent of depth** (matches per-zone angular spacing, which is what
actually varies with distance); spatial mode's threshold is a **metric
distance** derived from `threshold_pct * mean(valid depth)` at call time,
since alpha shape has no notion of per-point relative depth. Both are driven
by the same panel slider (0.5%–15%, default 4%) — spatial mode just
converts it to metres each rebuild.

## 3. Config + persistence

`ViewerConfig` (`host/src/roomscan/config.py`) gains three fields, following
the existing config-backed-only pattern (`point_size`, `ir_colormap`, ... —
no CLI flags, filled from `roomscan.toml` via `_fill_panel_fields` /
`_PANEL_FIELDS`, saved back on `--save-config`):

```python
surface_enabled: bool = False
surface_mode: str = "grid"          # "grid" | "spatial"
surface_threshold_pct: float = 4.0
```

Add the same three names to `_PANEL_FIELDS` in `panel.py`.

## 4. Panel UI

New collapsible **Surface** group (closed by default, ordered after View —
matches Status/View/Device/... pattern in `_build_panel`):

- Checkbox `Enable surface interpolation` (default off, so nothing changes
  for existing users until they opt in).
- Combobox `Adjacency`: Grid / Spatial (default Grid).
- Slider `Threshold %`: 0.5–15, default 4.0.

Handlers mirror the existing `_on_color`/`_on_near_mode`/`_on_point_size`
pattern: mutate panel state, no immediate redraw needed (next `_render_frame`
picks it up).

## 5. Render pipeline changes (`panel.py`)

`_render_frame` currently: deproject → color → set `self.pcd` → `_show_cloud`.

When `surface_enabled` is false: unchanged.

When true:
- **Grid mode**: every frame — `self.deproj.grid(depth)` → color the full
  grid the same way `cloud_colors` already colors the flat array (reshape
  in/out) → `grid_triangles(...)` → rotate vertex positions with the
  existing `_rot_xy` (topology is decided in raw sensor space; rotation is a
  pure z-preserving xy transform applied after, same as today) → update the
  `TriangleMesh` geometry; update `self.pcd` to hold only the *uncovered*
  points/colors.
- **Spatial mode**: point positions/colors update every frame as today
  (cheap). The mesh rebuild (`alpha_shape_triangles`, and the resulting
  covered/lone split) is throttled to the existing `_UI_PERIOD` (~4Hz)
  cadence already used for status/IR/log — reuses that timer rather than
  adding a new one. Between rebuilds, the last mesh and covered-set stay in
  place while dot positions/colors for currently-uncovered points keep
  moving live.

New material: `self.mesh_material` (`defaultUnlit`, same as the point
material) — Open3D's unlit shader interpolates per-vertex colors across a
triangle (Gouraud-style), so the mesh reads with exactly the same
depth/reflectance/confidence coloring as the dots, just interpolated instead
of discrete. `_show_cloud` becomes `_show_geometries`, managing both the
`"cloud"` (dots) and a new `"surface"` (mesh) geometry — add/remove/update
each independently; when surface is disabled or a mode-switch/frame yields
zero triangles, the mesh geometry is simply removed (or never added).

**Camera framing fix**: `_reset_camera` currently frames from
`self.pcd.get_axis_aligned_bounding_box()`. Once most points move into the
mesh, `self.pcd` alone would under-represent the scene extent on first
frame. Fix: build the framing bounds from the *full* valid point set each
frame (covered + lone, before the dot/mesh split), independent of which
geometry is currently rendering them.

## 6. Out of scope / explicit non-goals

- No lit shading / normals (per earlier decision — stays visually
  consistent with the existing unlit point cloud).
- No persistence of the mesh across frames beyond what's needed for spatial
  mode's throttled rebuild (no accumulation/registration across frames —
  same "live, replaced every update" model as the point cloud today).
- No config/CLI flags beyond the three `ViewerConfig` fields (matches how
  `point_size`/`ir_colormap`/`near_mode` already work — config-file-only,
  no argparse flags).

## 7. Testing plan

- `test_deproject.py`: `Deprojector.grid()` on small synthetic depths —
  shape, valid mask, and that it matches `__call__`'s filtered output at the
  valid cells.
- New `test_surface.py`: `grid_triangles` on synthetic grids — a flat
  surface (fully triangulated, all covered), a hard step edge (triangles
  refused across it, edge points on both sides stay uncovered where they'd
  only connect across the step), an all-invalid row (no triangles touch
  it). `alpha_shape_triangles` on tiny synthetic clouds — a tight cluster
  (covered) plus one far outlier (uncovered).
- `test_panel.py`: extend the existing `_fill_panel_fields` tests to cover
  the three new fields; no new Open3D-GUI-dependent tests needed since the
  mesh-building logic itself is covered above and the panel only wires pure
  functions together (same pattern as `_rot_xy`/`_orbit_eye` today).
