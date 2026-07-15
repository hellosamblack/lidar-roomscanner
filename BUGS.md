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
| BUG-009 | fixed   | host/panel    | SLAM/Showcase trajectory LineSet with a single point hard-crashes Filament (segfault) |
| BUG-010 | by-design | host/panel  | A Recorder capture started well into a session lacks CALIB and can't be post-processed |
| BUG-011 | fixed   | host/panel    | Floating HUD toggles unclickable — control `ImageWidget`s swallow clicks before the SceneWidget's `set_on_mouse` |
| BUG-012 | fixed   | host/panel    | Per-frame `srgbColor` Filament console spam from `defaultUnlitTransparency` material |
| BUG-013 | fixed   | host/panel    | SLAM-mode Record never stops/processes — action cluster armed the classic SLAM view, not the Showcase pipeline |
| BUG-014 | fixed   | host/panel    | First-person IR overlay renders edge-on (white/black) or not at all — first-person camera clobbered + texture not bound as albedo |
| BUG-015 | fixed   | host/panel    | Overlays → Sensors toggle showed nothing — sensor widgets lived only in the settings dialog, no floating overlay |
| BUG-016 | fixed   | host/panel    | First-person IR overlay: upside-down texture, hidden on a fresh launch, and oversized vs. the real point cloud |
| BUG-017 | fixed   | host/panel    | Panel launch always fails "port in use" — its own ST-Link log-tail thread races the CDC-missing serial fallback for the same COM port |
| BUG-018 | fixed   | host/panel    | Launch failures (missing/busy scanner port) never appeared in app.log — printed to console only |
| BUG-019 | fixed   | host/sources  | Ethernet preference was fragile: `.local` resolution always failed on Windows, and the "retry" loop only ever sent one wake packet |

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

## BUG-009 — SLAM/Showcase trajectory LineSet with a single point hard-crashes Filament (segfault)

- **Status:** **fixed** 2026-07-11 (this branch) · **Reported:** 2026-07-11 (Task 12, Showcase mode)
  · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_render_slam_frame`'s trajectory upload block (Task 10,
  the classic SLAM view -- `_show_showcase_trajectory` in this same file, added by Task 12, sidesteps
  it, see that method's docstring)

Reproduced live, deterministically, replaying `captures/phase6_motion_ref.bin` through a real
`ControlPanel` (`gui.Application.instance.run_one_tick()`), on the very first successful
`SlamWorker`/`Mapper.step()` result: the trajectory at that point has exactly 1 pose. The existing
code builds an Open3D `LineSet` with 1 point and (since `len(pts) >= 2` gates setting `.lines`) 0
line segments, then uploads it via `scene.add_geometry(...)`. This crashes with:
```
in class filament::VertexBuffer *__cdecl filament::VertexBuffer::Builder::build(class filament::Engine &):111
reason: vertexCount cannot be 0
[Open3D WARNING] Resource [VertexBuffer, 0, hash: ...] not found.
[Open3D WARNING] Resource [IndexBuffer, 0, hash: ...] not found.
```
...followed by a hard process segfault a few ticks later (not always the very next tick -- timing-
dependent). Confirmed via a minimal repro script that toggles the classic SLAM checkbox alone (no
Showcase code involved) and ticks the panel: same crash, same tick offset. Not exercised previously
because nothing had driven the panel through `run_one_tick()` fast enough, immediately after
enabling the SLAM view with no "warm-up" frames rendered first, to reach the first 1-point
trajectory publish before the *next* mesh/trajectory render call replaced it with a ≥2-point one.

**Likely cause:** Filament's `VertexBuffer`/`IndexBuffer` builders reject (well, crash on) a
0-vertex-index (or otherwise degenerate) buffer being the very first `unlitLine`-shaded geometry
added to the scene under certain engine states, rather than raising a catchable Python exception.

**Fix (applied 2026-07-11, this branch):** guarded `_render_slam_frame`'s trajectory block the same
way `_show_showcase_trajectory` already did: skip the upload while `len(trajectory) < 2` instead of
uploading a point-only `LineSet`. The classic SLAM view (`chk_slam`) no longer hits this.

## BUG-010 — A Recorder capture started well into a session lacks CALIB and can't be post-processed

- **Status:** **by-design**, mitigated for live mode 2026-07-11 (Task 12) · **Reported:** 2026-07-11
  (Task 12, Showcase mode) · **Area:** host/panel

The scanner device streams its `CALIB` control frame once, near the very start of a session.
`roomscan.slam.cli._load_frames` (and therefore `PostProcessWorker.from_capture`) needs that CALIB
frame in the capture to run `TransformStage` and produce any depth frames at all -- without it,
`_load_frames` returns `frames=[], width=None, height=None`. The panel's `Recorder` (Record/Stop
button) just dumps raw bytes from whenever `Record` was pressed onward; if the user enables
Showcase mode and presses Record well after the device/replay session already started (the normal
case), the CALIB frame has already gone by and never lands in the new `.bin`.

**Mitigation (Task 12):** `panel.py`'s `_enter_showcase_recording` now dispatches
`CommandCode.SEND_CALIB` (the same command the Device group's "CALIB" button sends) every time
Showcase's Record is pressed, so a live device re-streams CALIB into the just-opened recording.
`CommandDispatcher.dispatch()` already no-ops harmlessly ("not available in replay") when there's
no live device, so **replay-mode Showcase recordings starting after tick 0 of the replay file are
still unprocessable** -- confirmed live: `PostProcessWorker` degrades gracefully (see
`showcase.py`'s `_publish_construction_failure`: a terminal `done=True`, 0-frames/0-verts publish,
not a hang or crash) rather than blocking PROCESSING forever, but the resulting "scan" is empty.
Recording from the very start of a replay file works fine. Marked `by-design` rather than `open`
because live mode (the feature's primary use case) is fixed; a full replay-mode fix would need
`_load_frames` to tolerate a missing CALIB (e.g. reuse the panel's already-warm `TransformStage`
instead of a fresh one) -- out of scope here.

## BUG-011 — Floating HUD toggles unclickable (mouse passthrough)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` — HUD widget creation in `_build_overlay`, `_on_mouse`

The two-mode/HUD redesign (Phase 6 panel UX) draws each floating control (mode switch, view toggle,
action cluster, IR control, status chip) as a `gui.ImageWidget` added to the window and positioned
over the `SceneWidget`. Click routing was done through `scene_widget.set_on_mouse(self._on_mouse)` →
`HudLayout.hit_test`. But Open3D dispatches a mouse event to the **topmost child widget** whose frame
contains the cursor: over a control that is its `ImageWidget`, which has no handler and does not
forward, so the SceneWidget's `set_on_mouse` never fired and `hit_test` never ran. Every HUD toggle
was dead (camera orbit still worked everywhere the controls didn't cover). This was the exact failure
the Task-9 note in `_on_mouse` anticipated ("if the ImageWidget itself consumes clicks...").

**Fix:** `gui.ImageWidget` has its own `set_on_mouse`, so each HUD widget now binds
`w.set_on_mouse(self._on_hud_widget_mouse)` — the widget that's actually on top handles its own
clicks. The new handler reuses the existing `HudLayout.hit_test` / `_dispatch_hud_hit` unchanged
(event coords are window-absolute, so segments and the IR opacity slider work as-is) and consumes
every event over a control so it never leaks to camera nav. The now-dead HUD-intercept block was
removed from `_on_mouse` — that also fixed a latent bug where a click in a *hidden* control's screen
region still dispatched to it (the SceneWidget handler used the full layout regardless of visibility).
Regression tests in `host/tests/test_panel_modes.py` (`test_on_hud_widget_mouse_*`). The on-screen
click still wants an owner eyeball (Filament can't render headless), but the mechanism is API-sound.

## BUG-012 — Per-frame `srgbColor` Filament console spam

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner) · **Area:** host/panel
- **Where:** Open3D 0.19 library bug; worked around in `host/src/roomscan/logfilter.py` (wired in
  `panel.py` `run()`)

The console floods, at the sensor frame rate, with:
```
in ... filament::UniformInterfaceBlock::getUniformOffset(...):NNN
reason: uniform named "srgbColor" not found
```
Root cause (verified against the shipped resources): of Open3D 0.19's `.filamat` shaders **only**
`defaultUnlit.filamat` declares the `srgbColor` uniform; `defaultUnlitTransparency.filamat` does not —
yet Open3D's shared `FilamentScene::UpdateDefaultUnlit` binds `srgbColor` unconditionally, so Filament
warns on every material bind of a translucent geometry. The first-person IR billboard
(`_update_ir_overlay`) does `remove_geometry`+`add_geometry` with that transparency material **every
frame** in first-person mode (the default), so one warning prints per frame. It is cosmetic — rendering
is unaffected. Filament writes it at the C runtime level (fd 2), so `contextlib.redirect_stderr` and
Open3D's verbosity control can't touch it.

**Fix:** `logfilter.install_filament_stderr_filter()` interposes an OS pipe on fd 2 and a daemon reader
thread that drops exactly the two warning lines (matched on the `srgbColor` / `getUniformOffset`
substrings — specific enough that no genuine error collides) and re-emits everything else verbatim.
Verified end-to-end: a UCRT-level write (the same runtime Filament links) of the warning is dropped
while a sentinel survives (`host/tests/test_logfilter.py`). Opt out with `ROOMSCAN_KEEP_FILAMENT_LOGS=1`.

**Fix:** Added checks in `_on_layout` to return early if the window width or height is `<= 0`, or if the resulting `scene_w` is `<= 0`. Constrained `panel_w` to be at least `0` so it doesn't become negative. Additionally, guarded camera operations in `_reset_camera` and `_apply_camera` to skip execution if `scene_widget.frame` width or height are `<= 0` (preventing setup of degenerate projection matrices).

## BUG-013 — SLAM-mode Record never stops/processes (action cluster orphaned)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_set_mode` + `__init__` mode application

The panel keeps two mutually-exclusive machines: `slam_enabled` (the classic always-on live SLAM
view, `_render_slam_frame`) and `showcase_enabled` (the record→process→reveal state machine over
`ShowcasePhase`, `_render_showcase_frame`). The two-mode redesign spec is explicit that SLAM mode IS
the record→process→reveal flow: *"SLAM: map building = the former SLAM view AND Showcase flow, merged.
Record → process → reveal is the Showcase pipeline under the hood."* But `_set_mode(VIEW_SLAM)` (and
the `__init__` default-mode application) called `_on_slam_toggle(True)` — arming the classic view, not
the showcase machine. So `showcase_phase` stayed `None`, `_hud_action_labels` was pinned at the IDLE
`[REC, LOAD, CLR]` set forever, and `_on_record` (which only bridges into
`_enter_showcase_recording`/`_enter_showcase_processing` `if self.showcase_enabled`) just wrote a raw
`.bin` with no phase transition. Clicking REC therefore never became STOP and never kicked off
processing — the action cluster was orphaned.

**Fix:** `_set_mode(VIEW_SLAM)` and the `__init__` mode application now call `_on_showcase_toggle(True)`
so SLAM mode drives the showcase machine (its RECORDING phase runs the same live SLAM preview = the
"former SLAM view"). Leaving SLAM tears down showcase (and the now-unused classic view only if it was
somehow on). Regression: `test_panel_modes.py::test_set_mode_slam_arms_showcase_not_classic_slam` /
`test_set_mode_real_time_disables_showcase`. On-screen record→process→reveal flow still wants an owner
eyeball (Filament can't render headless).

## BUG-014 — First-person IR overlay renders edge-on (white/black) or not at all

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` `_apply_real_time_first_person`, `_apply_camera_mode`,
  `_update_ir_overlay`

Two independent faults on the first-person IR billboard (`ir_overlay.camera_locked_quad` +
`_update_ir_overlay`), which is a camera-locked quad built to face a +Z first-person camera:

1. **Camera clobber (the "edge-on, white one side / black the other" in Real-Time).** Entering
   Real-Time first-person, `_apply_real_time_first_person` set the fixed `look_at` camera but left
   `_camera_set = False`. The very next cloud frame's `_show_geometries` sees `not _camera_set` and
   calls `_reset_camera` → `setup_camera(bounds)`, replacing the first-person view with a bounds-framed
   orbit camera. The +Z-facing billboard is then seen from the side (edge-on); its two triangles show
   front (textured/white) vs back (unlit/black). **Fix:** `_apply_real_time_first_person` now sets
   `_camera_set = True` to pin the view; `_apply_camera_mode` resets it to `False` when Real-Time
   switches to ORBIT so the cloud reframes. (SLAM first-person was never clobbered — it rides
   `_apply_follow_camera` every frame — but it *was* dead because of BUG-013, so IR "didn't show in
   SLAM" until that fix routed SLAM through the showcase RECORDING path that updates the billboard.)
2. **Texture not bound.** The mesh carried `.textures` + `triangle_uvs`, but the `MaterialRecord` had
   no `albedo_img`, so the Filament unlit shader fell back to the plain white `base_color`. **Fix:**
   `_update_ir_overlay` now sets `self.ir_overlay_material.albedo_img` to the IR image (the reliable
   Filament albedo slot).

Regression: `test_panel_modes.py::test_real_time_first_person_pins_camera_set` /
`test_real_time_first_person_noop_without_viewport`; quad geometry stays covered by
`test_ir_overlay.py`. On-screen render (texture + orientation) still wants an owner eyeball.

**Follow-up (2026-07-14, owner on-rig round 2):** the `_camera_set` pin above then made the IR overlay
render *nothing at all* — pinning stopped `_reset_camera` from ever running, so the camera **projection
was never set** and the near cloud/billboard fell outside a stale/degenerate frustum. Together with the
owner's other first-person feedback this became a first-person overhaul (confirmed design via a two-part
question — first-person = look out through the sensor at the cloud fixed in front + IR overlay; cloud
sensor-fixed in first-person, gravity-aligned in orbit):
- **Projection:** `_apply_real_time_first_person` now sets an explicit perspective projection
  (`camera.set_projection(60, aspect, 0.05, 50, Vertical)`) before `look_at`, and is also re-applied from
  `_on_layout` (so a session opening straight into Real-Time first-person isn't left projection-less) and
  self-heals in the render path if `_camera_set` is cleared.
- **True first-person (not a camera model + orbiting image):** in Real-Time first-person the cloud is
  kept in the raw **sensor frame** (no IMU rotation, so it stays dead ahead as you aim), the IMU "camera
  model" gizmo is removed (`_remove_camera_gizmo` — it lingered from orbit), and mouse nav is swallowed so
  a stray drag can't arcball out of the fixed view. Orbit keeps the gravity-aligned cloud + gizmo.
- **Camera never decimated (#2):** the follow camera's flat `_FOLLOW_SMOOTH=0.12` EMA lagged real motion
  ~0.3 s ("feels like the system didn't notice you moved"). `_follow_alpha` makes the weight
  velocity-adaptive — sub-`_FOLLOW_SNAP_M` (3 cm/frame) jitter still smooths, genuine motion tracks 1:1.
- **Orbit auto-zoom (#4):** entering ORBIT in either mode clears `_camera_set`, so `_reset_camera`
  (Real-Time) / `_slam_camera_frame` (SLAM) refits the view to all content on the next frame.

Regression: `test_panel_modes.py` (`test_real_time_first_person_aims_view_without_pinning`,
`test_follow_alpha_*`, `test_apply_camera_mode_orbit_clears_camera_set`, `test_remove_camera_gizmo_*`,
`test_on_mouse_swallows_nav_in_real_time_first_person`). All camera/render behavior still needs an owner
on-rig eyeball (Filament can't render headless).

**Follow-up (2026-07-14, owner on-rig round 3):** "first-person doesn't work right away — I have to go to
orbit and back." Root cause: the projection-pin approach above (`_camera_set=True` in
`_apply_real_time_first_person`) *blocked* `_reset_camera`, so at startup/mode-switch the projection was
never established and first-person rendered wrong until an orbit round-trip ran `_reset_camera` to prime
it. **Fix:** stop pinning and stop setting the projection in `_apply_real_time_first_person` — it now only
aims the `look_at` view, and is re-applied **every Real-Time first-person frame** (after `_show_geometries`
lets `_reset_camera` own the projection from the live cloud bounds), plus from `_on_layout`. This mirrors
SLAM exactly (`_slam_camera_frame`'s `setup_camera` once + `_apply_follow_camera`'s per-frame `look_at`),
so first-person is correct from the first frame with no orbit round-trip. (SLAM first-person already worked
this way — it activates on the RECORDING follow.)

**Follow-up (2026-07-14, owner on-rig round 4):** with first-person now rendering the cloud correctly, the
IR billboard still didn't appear with the opacity slider at full. Cause: `_set_ir_opacity` only set the
opacity — the draw gate is `fp and ir_overlay_enabled`, and nothing flipped `ir_overlay_enabled`, so the
slider did nothing until the (non-obvious) "IR" label was also clicked. **Fix:** the slider now doubles as
the on/off control (`_set_ir_opacity` enables the overlay for opacity > 0.02, hides it at ~0; toggling on
at 0 opacity bumps it to 1.0). Verified the `defaultUnlitTransparency` shader *does* carry an `albedo`
texture sampler, so the `albedo_img` binding renders the IR image (not the round-1 white base_color).
Added a one-time `_update_ir_overlay` log when enabled but the stream has no reflectance (depth-only).

## BUG-015 — Overlays → Sensors toggle showed nothing (no floating overlay)

- **Status:** **fixed** 2026-07-14 · **Reported:** 2026-07-14 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_build_overlay`, `_on_layout`, `_update_sensors`,
  `_toggle_sensors_menu`), `host/src/roomscan/sensors_widgets.py` (`render_sensors_overlay`)

The redesign's **Overlays → Sensors** menu item toggled `sensors_panel` and logged, but nothing appeared:
the compass + pressure/temp widgets were only ever built into the **settings dialog**'s "Sensors" group
(not a menu target), so the menu had no floating overlay to show — unlike **Overlays → Metrics**, which
drives the top-left `metrics_hud` ImageWidget. Toggling Sensors therefore read as an empty overlay.

**Fix:** added a floating **Sensors overlay** mirroring the metrics HUD — a new pure
`sensors_widgets.render_sensors_overlay(heading, pressure_hist, temp_hist)` composites the compass dial +
heading readout and the pressure/temp sparklines into one panel image, drawn into a top-right
`gui.ImageWidget` (`_build_overlay`/`_on_layout`), refreshed on the ≤4 Hz UI tick (`_update_sensors`), and
shown/hidden by `_toggle_sensors_menu`. The settings-dialog Sensors group is retained for the Reset
Baseline control (its display-widget updates are now `hasattr`-guarded, closing a latent crash when Sensors
was toggled on after being built-disabled). Also (owner request) the app now **defaults to Real-Time
first-person** (`ViewerConfig.mode` "slam" → "real_time"). Tests: `render_sensors_overlay` shape/no-data/
heading-change; config default. On-screen placement still wants an owner eyeball.

## BUG-016 — First-person IR overlay: upside-down, hidden on launch, oversized

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, on-rig) · **Area:** host/panel
- **Where:** `host/src/roomscan/ir_overlay.py` (`camera_locked_quad`), `host/src/roomscan/config.py`
  (`ViewerConfig.ir_overlay`), `host/src/roomscan/panel.py` (`_update_ir_overlay`)

The on-rig eyeball that BUG-014/ROADMAP flagged as still outstanding ("IR billboard texture
render/UV orientation + opacity") surfaced three independent faults:

1. **Upside-down texture.** `camera_locked_quad`'s UVs mapped the top-left vertex to `v=0`, but
   Open3D/Filament samples textures bottom-left-origin (OpenGL convention) while the reflectance
   image (`reflectance_to_rgb`/`o3d.geometry.Image`) is row-major top-down like every other array
   in this codebase — rendering the billboard upside down. **Fix:** flip `v` (TL/TR/BR/BL now map
   to `v=1,1,0,0`).
2. **Hidden on a fresh launch despite the opacity slider sitting at 50%.** `ViewerConfig.ir_overlay`
   defaulted to `False` while `ir_opacity` defaults to `0.5` — inconsistent with the "opacity > 0.02
   implies enabled" invariant `_set_ir_opacity`/`_toggle_ir_overlay` already enforce on every runtime
   interaction (round 4 of BUG-014). A fresh install (no saved config yet) started in that
   self-contradictory state. **Fix:** default `ir_overlay` to `True`.
3. **Oversized relative to the real point cloud** (owner screenshots: "look at the size of the
   person in the foreground, compared to that same person in the overlay" — the same content
   filled far more of the billboard's real-terms footprint than its rectangle implied). Root cause:
   `_update_ir_overlay` built the quad from the *viewing* eye that every caller passes — Real-Time's
   fixed `[0,0,-_FOLLOW_BACK_OFF_M]`, SLAM/showcase's `follow_camera_target(pose)` result — which
   sits `_FOLLOW_BACK_OFF_M` (0.3 m) *behind* the true sensor origin ("a hair of context", per
   `follow_camera_target`'s own docstring). But `camera_locked_quad` sizes the quad as a true
   sensor-FOV footprint anchored at the sensor's own origin (matches `capture_square_corners`'s apex
   convention, verified by `test_quad_corners_match_capture_square_convention`) — building it from a
   point 0.3 m further back than that origin inflates the quad by `dist/(dist-back_off)` ≈ 43% at
   `dist=1.0`, dwarfing the real IR content drawn inside it. Verified numerically (no on-rig render
   needed — Filament can't render headless on this box): projecting the Deprojector's true FOV-corner
   rays and the billboard's corners through the same look-at + perspective transform showed the
   un-fixed quad ~23% oversized in NDC space at a representative 1.5 m scan depth; reconstructing the
   apex (`eye + _FOLLOW_BACK_OFF_M * forward`, the inverse of `follow_camera_target`'s
   `eye = sensor_pos - back_off*forward`) closed the gap to ~6%, fully explained by the
   Deprojector's zone-center (vs. edge) convention and the quad's fixed 1.0 m placement vs. the
   test depth — not a further bug. **Fix:** `_update_ir_overlay` now reconstructs the true apex
   before calling `camera_locked_quad`, fixing all three call sites (Real-Time, SLAM follow,
   showcase RECORDING) at once. Regression: `test_ir_overlay_sized_from_true_sensor_apex_not_the_offset_eye`.

Tests: 576 host tests green. On-screen render still wants an owner eyeball to confirm the fixes read
correctly (this pass was code+numerics only — Filament can't render headless on this box).

## BUG-017 — Panel launch always fails "port in use", every retry, no external process

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_stlink_logger_thread`, `_open_source`),
  `host/src/roomscan/sources.py` (`SerialSource.find_port`)

`roomscan-panel` failed to launch with `error: the scanner port is in use: ... PermissionError`,
offering to close the one roomscan process it found holding the port — which was **the process
asking the question**. Killing it and relaunching reproduced identically every time.

Root cause: two features raced for the same COM port, entirely within a single process, whichever
port happened to be a Nucleo's ST-Link VID (`0x0483`) device. **(1)** Today's ST-Link log-tail
addition, `_stlink_logger_thread`, starts as a daemon thread at the very top of `run()` — before the
scanner port is even opened — and opens the first ST-Link-VID port it finds to tail firmware
`printf` output. **(2)** `SerialSource.find_port()` had a "milestone 1a" fallback (from before native
USB CDC existed): if the CAFE:4001 CDC device isn't enumerated, fall back to the first ST-Link-VID
port and treat it as the scanner data port — but in the current architecture that port only ever
carries plain-text debug printfs, never protocol frames, so this fallback couldn't actually have
worked even had it won the race. `_open_source`'s scanner-open attempt (the fallback branch) only
runs after `get_best_source`'s ~5 s UDP probe, so the logger thread — with zero delay — reliably won
the race for that port every time, and `_open_source` saw its own process's PermissionError.

**Fix (two parts):**
1. **Removed the vestigial fallback.** `SerialSource.find_port()` now only matches the CDC device
   (`CAFE:4001`); it no longer treats an ST-Link VCOM port as a candidate scanner port, so it no
   longer competes with `_stlink_logger_thread` for it. (Flashing over SWD via
   `STM32_Programmer_CLI` is on a separate USB interface from the VCOM UART bridge — holding the
   VCOM open for logging never blocks a flash.)
2. **A busy serial port is no longer a launch blocker.** Ethernet (Phase 5) is the production
   transport now, so `_open_source` warns and falls back to a listening `UdpSource` instead of
   aborting the app when the serial fallback is busy (still offers the interactive close-the-holder
   prompt first, when useful). Regression: `test_open_source_busy_port_warns_and_falls_back_to_udp`
   (`host/tests/test_panel.py`).

Tests: 576 host tests green.

## BUG-018 — Launch failures never appeared in app.log

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner) · **Area:** host/panel
- **Where:** `host/src/roomscan/panel.py` (`_report`, `_open_source`)

Surfaced immediately after BUG-017: a missing-scanner launch failure (`error: scanner not found: no
scanner serial port found among [...]`) printed correctly to the console, but "None of these were
logged in .../app.log". Two independent sources of console-only output turned out to be involved —
the `[run]`/`[tip]`/`[hint]` lines are `echo`ed by `view-panel.bat` itself, entirely outside Python,
so they can never reach a Python-managed log file — but the actual diagnostic (`_open_source`'s
error/warning messages) *is* Python's own, and simply used `print(..., file=sys.stderr)` directly
instead of going through the `logging` module, bypassing the `RotatingFileHandler` that
`_setup_app_logger` (already called before `_open_source` in `run()`) attaches to the root logger.

**Fix:** added `_report(msg, level="error"|"warning")`, which prints to stderr (unchanged
console behavior) *and* logs at the matching level; `_open_source`'s four message sites now go
through it. Verified end-to-end (not just via `caplog`): a forced missing-port failure now leaves the
exact console text in `logs/app.log`. Regression: `test_open_source_messages_are_logged_not_just_printed`
(`host/tests/test_panel.py`).

Tests: 577 host tests green.

## BUG-019 — Ethernet preference was fragile (two independent bugs)

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner: "we had comms over ethernet
  working prior to this... it's supposed to prefer ethernet") · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource._resolve_target`, `get_best_source`)

`get_best_source` is supposed to prefer Ethernet (Phase 5's production transport), probing UDP for
5 s before falling back to serial. Two independent bugs made that probe unreliable:

1. **`.local` resolution always fails on Windows.** `_resolve_target` tried
   `socket.gethostbyname("roomscanner.local")` first — but Windows has no native mDNS resolver
   without Bonjour installed, so this always raises `gaierror` (confirmed on-box:
   `gethostbyname('roomscanner.local')` → `gaierror(11001, 'getaddrinfo failed')`), meaning the
   *every-time* path was the broadcast fallback, never the (more reliable, unicast) mDNS-resolved
   address — despite `zeroconf` already being an installed dependency and `tools/query_mdns.py`
   already proving the correct call (`Zeroconf().get_service_info("_roomscan._udp.local.",
   "roomscanner._roomscan._udp.local.")`, matching the lwIP mdns advertisement from ROADMAP Phase 5)
   works. That correct call was simply never wired into `UdpSource`.
2. **The "retry" loop only ever sent one wake packet.** `get_best_source` set the *socket's own*
   timeout to the full 5 s probe window *before* entering the retry loop — so the very first
   `udp.read()` call itself blocked for up to the whole window internally (returning early only on
   data), leaving the outer `while time.time() - t0 < 5.0` no real second iteration to resend on.
   The board doesn't know the host's address up front (needs the "wake" datagram to learn where to
   reply), and UDP has no delivery guarantee — one dropped packet silently killed Ethernet
   preference for the entire launch, no retry, every time.

**Fix:**
1. `_resolve_target` now queries mDNS properly via zeroconf's `get_service_info` (injectable
   `zeroconf_factory` for tests) and only falls back to subnet broadcast if that finds nothing or
   errors.
2. `get_best_source` now uses a short per-read socket timeout (0.2 s) with a real wall-clock polling
   loop, resending the wake packet every `resend_s` (default 0.5 s) — both now parameters
   (`probe_s`, `resend_s`) so tests don't need to wait out a real 5 s window.

`UdpSource`/`get_best_source` had zero prior test coverage. Added: mDNS-success /
mDNS-not-found-falls-back-to-broadcast / zeroconf-error-falls-back-to-broadcast
(`test_resolve_target_*`), and a loopback-free regression proving the retry loop actually resends
and returns promptly on data instead of blocking out the full probe window
(`test_get_best_source_resends_wake_packet_and_returns_promptly_on_data`), all in
`host/tests/test_sources.py`.

**Still open:** at the time of this fix, neither `socket.gethostbyname` nor a live
`tools/query_mdns.py` mDNS query found the device on this machine — i.e. this fix makes Ethernet
discovery *robust*, but doesn't by itself prove the device is currently reachable over Ethernet from
this PC. Worth an on-rig check: is the Ethernet cable actually connected right now (direct link,
self-assigned `172.31.253.1`/`.2` per ROADMAP Phase 5, or a real DHCP-served LAN)? The firmware
itself falls back to USB CDC if it doesn't have a leased Ethernet IP at boot, so "not found over
Ethernet" can also legitimately mean the device decided to stream over CDC instead.

Tests: 581 host tests green.
