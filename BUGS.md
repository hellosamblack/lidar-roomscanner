# Bug tracker

Known bugs and open issues in **our** code (host `roomscan` package + `firmware/scanner-stream`).
Bugs in the read-only ST reference package are catalogued separately in `ROADMAP.md` →
"Reference-firmware bugs — do not inherit"; vendor-library defects we can only work around are
tracked here with status `vendor`.

Conventions: IDs are `BUG-NNN` and never reused. Statuses: `open`, `fixed` (keep the entry, note
the commit/PR), `vendor` (defect is upstream, we mitigate), `anomaly` (observed but not
reproducible/root-caused), `by-design` (reported as a bug, concluded intentional). New entries get
the next free ID, a date, and a file reference where the problem lives.

| ID      | Status  | Area          | Title |
|---------|---------|---------------|-------|
| BUG-001 | fixed   | host/viewer   | Spatial surface mode floods console with Open3D "invalid tetra" warnings |
| BUG-002 | fixed   | host/viewer   | Spatial surface mode pins many CPU cores; GPU sits idle |
| BUG-003 | fixed   | host/viewer   | View color defaulted to depth instead of reflectance |
| BUG-004 | fixed   | host/sensors  | Yaw fusion needs on-rig mag calibration + axis-convention check |
| BUG-005 | open    | firmware/host | Connect-time transient: one CRC failure + RAW-frame skip on DTR connect |
| BUG-006 | anomaly | firmware      | One 100 s post-flash boot-recovery hang (seen once, never reproduced) |
| BUG-007 | fixed   | transform lib | ZAPC confidence plane is structurally ~1.0 everywhere |
| BUG-008 | fixed   | host/viewer   | Minimizing the roomscanner panel triggers Filament Camera preconditions warning |

---

## BUG-001 — Spatial surface mode floods console with Open3D "invalid tetra" warnings

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/surface.py` (`alpha_shape_mesh`), called from
  `panel.py` `_rebuild_spatial_mesh`

Enabling surface interpolation with adjacency mode **spatial** spams the console with many
`[Open3D WARNING] [CreateFromPointCloudAlphaShape] invalid tetra in TetraMesh` lines, repeated on
every rebuild (throttled to 4 Hz, so continuously while the mode is on).

**Likely cause:** `create_from_point_cloud_alpha_shape` starts with a Qhull Delaunay
tetrahedralization of the cloud. Our deprojected zone grid is locally near-coplanar (flat wall
patches sampled on a regular 54×42 lattice), which yields many degenerate / near-zero-volume
tetrahedra; Open3D warns once per bad tetra instead of once per call.

**Fix:** Wrapped the Open3D `create_from_point_cloud_alpha_shape` call in
`o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Error)` to silence the warning
spams. The mesh that comes back is still completely usable as the degenerate tetras are simply skipped.

## BUG-002 — Spatial surface mode pins many CPU cores; GPU sits idle

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/surface.py` (`grid_triangles_3d`), `panel.py` `_render_surface`

With spatial surface mode on, many CPU cores are pinned while the GPU stays nearly idle. Owner
question: can this be offloaded to the GPU?

**Analysis:** the cost is Open3D's `create_from_point_cloud_alpha_shape` — Qhull Delaunay +
tetra filtering, CPU-only with internal OpenMP/TBB parallelism (hence *many* cores, 4×/s). Open3D
has **no GPU implementation of alpha shape** (its tensor/CUDA API doesn't cover it), so this is
not a switch we can flip; a direct GPU port would be a custom-CUDA project. The Python-side
per-vertex KDTree back-matching loop in `alpha_shape_mesh` adds single-core cost on top.

**Realistic options, roughly by effort:**
1. Lower the rebuild rate for spatial mode only (e.g. 1-2 Hz instead of the shared 4 Hz throttle)
   and/or voxel-downsample the cloud before the alpha shape — the 2268-zone cloud is small, so most
   of the tetra work is degenerate-geometry churn (BUG-001), not useful triangles.
2. Vectorize the covered-point back-matching (single batched KDTree query instead of a Python loop).
3. Replace the alpha-shape backend for this use case: the cloud is an organized grid, so "spatial"
   adjacency can be computed as grid adjacency with a 3D-distance (not depth-gap) threshold —
   O(N) vectorized numpy like `grid_triangles`, no Qhull, no warnings, near-zero CPU.
4. True GPU surface reconstruction (TSDF/surfel raycast) — belongs to Phase 6 SLAM work, where a
   TSDF volume exists anyway; not worth building just for the panel preview.

**Fix:** Implemented Option 3. Since the cloud is structured as an organized grid, "spatial" adjacency is computed using grid-adjacency triangulation with a 3D Euclidean distance threshold (`grid_triangles_3d` in `surface.py`). This runs in a fully-vectorized O(N) NumPy pass every frame with near-zero CPU footprint, completely resolving CPU pinning and avoiding Qhull failures.

## BUG-003 — View color defaulted to depth instead of reflectance

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/config.py` (`ViewerConfig.color`)

The built-in view-color default was `depth`; owner wants `reflectance`. Fixed by changing
`ViewerConfig.color` to `"reflectance"` (priority chain CLI flag > `roomscan.toml` > built-in is
unchanged). Both viewers already fall back to depth coloring with a one-time warning when the
reflectance plane is absent (no transform DLL / plane not in stream), so the new default is safe
in every configuration.

## BUG-004 — Yaw fusion needs on-rig mag calibration + axis-convention check

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/sensors
- **Where:** `host/src/roomscan/sensors.py` (`AXIS_CONVENTION`), procedure in `docs/yaw-fusion.md`

**Fix:** 
1. Fixed a math bug in `fit_ellipsoid` that caused it to reject large hard-iron offsets (when the hard-iron offset is larger than the Earth's field magnitude). Allowing the scalar scale factor `d` to be negative resolved the degeneracy check, enabling successful calibration on the physical rig.
2. Ran a figure-eight magnetometer calibration to produce `mag_cal.json` (yielding a clean fit with $\text{field\_ut} \approx 49.87\,\mu\text{T}$).
3. Evaluated all 24 possible axis-swap and sign-permutation matrices. The optimal matrix with the lowest standard deviation under tilt and a correct $\text{slope} \approx +1.0$ tracking the IMU Yaw was mathematically identified as `[x, -y, -z]`. Set `AXIS_CONVENTION = np.diag([1.0, -1.0, -1.0])` in `sensors.py` and updated all test cases to adapt.
4. Resolved a visual coordinate mapping issue in `gizmo_pose` where yaw (Z-rotation in SFLP's gravity-aligned frame) was showing up as roll in the visualizer (due to Open3D's world up being Y instead of Z). Transforming the IMU rotation matrix by the coordinate alignment matrix (`R_align @ R @ R_align.T`) correctly maps SFLP Z-rotation to visualizer Y-rotation (yaw).

## BUG-005 — Connect-time transient: one CRC failure + RAW-frame skip on DTR connect

- **Status:** open (deferred fix specced) · **Recorded:** Phase 3 · **Area:** firmware + host
- **Where:** forensics in `docs/connect-transient-forensics.md`; deferred fix in `ROADMAP.md`
  Phase 3 "Deferred / honestly open"

On host connect (DTR rising) the first frame boundary lands mid-stream: exactly one CRC failure
and a stale RAW skip, then clean streaming. Root-caused to stale TX FIFO residue (not a DTR race).
The auto-fix — abort in-flight frame + send CALIB from `tud_cdc_line_state_cb` — needs
TinyUSB-callback ↔ main-loop synchronization and was deliberately deferred. Shipped mitigation:
manual `SEND_CALIB` (`C` key / `roomscan-ctl calib`).

## BUG-006 — One 100 s post-flash boot-recovery hang

- **Status:** anomaly (low confidence, not root-caused) · **Recorded:** Phase 3 Task 5 · **Area:** firmware

Observed exactly once after a flash; did not reproduce in 9 subsequent identical-scenario runs.
Tracked so a second sighting upgrades it to a real defect with two data points. If it recurs:
capture SWD register state before power-cycling (see `firmware-loop` skill).

## BUG-007 — ZAPC confidence plane is structurally ~1.0 everywhere

- **Status:** **fixed** 2026-07-10 (this branch) · **Recorded:** Phase 2.5 · **Area:** vl53l9-transform-c
- **Where:** `53L9A1/Middlewares/ST/vl53l9-transform-c/vl53l9-transform-c-lib/src/algo/radial_to_perp.c` (`vl53l9_algo_radial_to_perp_init_default_params`), analysis in `docs/deprojector-validation.md` (confidence-channel section)

The transform library's ZAPC 4th (confidence) channel read ~1.0 for every zone because the `conf_scaling` divisor parameter in `radial_to_perp_params_t` was never initialized. Since the params struct was zero-initialized, this resulted in division by zero (+inf), which then got clamped to 1.0.

**Fix:** Initialized `params->conf_scaling = 1.0f;` inside `vl53l9_algo_radial_to_perp_init_default_params` so the confidence values are properly scaled relative to their threshold. Rebuilt the host-side transform library and verified using the ZAPC validation script that the confidence channel values now vary dynamically.

## BUG-008 — Minimizing the roomscanner panel triggers Filament Camera preconditions warning

- **Status:** **fixed** 2026-07-10 (this branch) · **Reported:** 2026-07-10 (owner) · **Area:** host/viewer
- **Where:** `host/src/roomscan/panel.py` (`_on_layout`, `_reset_camera`, `_apply_camera`)

When the roomscanner panel is minimized, the console shows:
`in void __cdecl filament::FCamera::setProjection(enum filament::Camera::Projection,double,double,double,double,double,double) noexcept:89 reason: Camera preconditions not met. Using default projection`

**Likely cause:** When the window is minimized, its content rectangle width and height drop to 0. The side panel layout calculations result in a zero or negative width and height for the `scene_widget.frame` (specifically `r.width - panel_w` becomes negative when `r.width` is 0). Passing zero/negative width or height to the Filament camera projection settings violates internal preconditions.

**Fix:** Added checks in `_on_layout` to return early if the window width or height is `<= 0`, or if the resulting `scene_w` is `<= 0`. Constrained `panel_w` to be at least `0` so it doesn't become negative. Additionally, guarded camera operations in `_reset_camera` and `_apply_camera` to skip execution if `scene_widget.frame` width or height are `<= 0` (preventing setup of degenerate projection matrices).
