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
| BUG-005 | fix unverified | firmware/host | Connect-time transient: one CRC failure + RAW-frame skip on DTR connect — DTR-callback fix implemented, but the CDC path cannot be exercised on this host |
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
| BUG-020 | fixed   | host/native   | Native transform loader was Windows-only (`.dll` name + `build/Release/` path) — blank viewer on the Linux headless host; reader fault never surfaced to the page |
| BUG-021 | fixed   | host/web      | Web viewer loaded Three.js from a CDN (unpkg) — headless/remote browser couldn't fetch it, `app.js` threw on the bare `three` import, page stuck at "Offline" |
| BUG-022 | fixed   | host/web      | Headless-host browser has no WebGL — `new THREE.WebGLRenderer()` threw "Error creating WebGL context", viewer stuck "Offline"; auto-open now passes Chrome `--enable-unsafe-swiftshader` |
| BUG-023 | fixed   | firmware      | System clock sourced from the ST-LINK's MCO — board is dead (silent network, PHY link LED on) whenever the ST-LINK cable is unplugged |
| BUG-024 | fixed   | host/sources  | A *missing* CDC serial port aborted the launch of an Ethernet-only deployment |
| BUG-025 | fixed   | host/sources  | `UdpSource` retargeted onto its own looped-back broadcast wake, adopting the host itself as the device |
| BUG-026 | fixed   | host/web      | Web UI never gravity-aligned the view — boot the board upside down and both IR and point cloud render upside down |
| BUG-027 | fixed   | firmware      | SFLP quaternion decimated 480 Hz → 30 Hz by keeping one sample of ~16, aliasing the whole noise band into the output |
| BUG-028 | fixed   | firmware      | Ethernet hot-plug replug wedges board (LD2 freezes) — lwIP double-add assertion on `mdns_resp_add_netif` |
| BUG-029 | fixed   | host/sources  | `UdpSource` stream recovery fails due to `255.255.255.255` fallback keepalives not routing on some Linux network configs |
| BUG-030 | fixed   | host/sensors  | Magnetometer calibration is direction-dependent: |B| ranges 47→85 µT with tilt — root-caused to a ~59 µT wrong hard-iron offset (+ tripod, BUG-034); owner re-fit 2026-07-30, validated on an independent room sweep |
| BUG-031 | fixed   | firmware      | ToF frame timestamp and IMU FIFO drain skewed ~0.9 ms — the drain sits 24.3 ms past the frame-ready edge and breathes with load; fixed by latching the LSM clock AT the edge (stream 13), 1070 → 18 µs RMS |
| BUG-032 | fixed   | host/slam     | GPU SLAM OOMs on a long scan — Open3D's CUDA cache grows ~5.1 MiB/frame from the throttled mesh extraction (NOT the per-frame path, which is byte-flat) |
| BUG-033 | fixed   | host/web      | Sensors card outgrew the dock band (~1600 px of flat, half-duplicated rows) — jitter table unreachable, whole card auto-collapsed on a narrow window |
| BUG-034 | by-design | environment   | The tripod adds 15–27 µT of magnetic field — heading is unreliable while the device is mounted on it, regardless of calibration |
| BUG-035 | fixed   | host/slam     | TSDF block capacity (40k) was *below* a real room sweep's demand (42.9k); running at ~97% of it stalled map growth and collapsed tracking for the last 18% of the scan (mechanism unproven — the grid *does* rehash) |
| BUG-036 | fixed   | host/slam     | A single fixed ICP correspondence radius (0.05) made one bad frame terminal — 423 frames (22% of a room circuit) silently dead-reckoned; fixed by retrying only on failure at 0.10 |
| BUG-037 | fixed   | host/slam     | `baro_weight = 0.05` fed the barometer's ~267 mm RMS of per-frame white noise straight into the pose — ~35% of reported path was invented vertical motion (flattering every %-of-path drift figure) and a drifting baro owned the height outright; replaced by a low-passed, bounded-authority complementary correction |
| BUG-038 | fixed   | host/web      | The live point cloud is frustum-culled against a stale, zero-radius `boundingSphere` — latent in World view, fatal in FPV (filed as a duplicate "BUG-033"; renumbered 2026-07-30) |
| BUG-039 | fixed   | host/sensors  | `imufusion._correct_yaw` measured heading error about **body Z** on a body frame whose X is Up — 3.76° from gimbal lock, giving 1.69° mean heading error that loop gain could not fix; replaced by a world-Z swing-twist term (2.218° → 0.053° p95, bit-identical at level attitudes). Filter still gated off |
| BUG-040 | fixed   | host/web      | The web UI's Drops/Gaps HUD rows were structurally pinned at 0 — `web.py` read `MetricsSnapshot.drops/gaps` (dataclass default 0) and nothing ever merged the reader's `Stats`; only the deprecated `panel.py` did. Since a lost UDP fragment makes the host discard the whole frame, a seq gap is the ONLY evidence of transport loss, so the primary UI could not see packet loss at all |
| BUG-041 | fixed   | firmware/eth  | `ETH_SendFrame_Gather` burst all 11 fragments of a depth frame back-to-back into an 8-deep TX descriptor ring, and **abandoned the frame mid-burst** on any `udp_sendto`/`pbuf_alloc` failure — the already-sent fragments then being guaranteed waste. Replaced by a slot-FIFO pacer that meters fragments from `ETH_Process()` and retries rather than abandoning |
| BUG-042 | fixed   | host/sources  | UDP reassembly required `frag_idx == expected` and appended, so a merely **reordered** datagram — which UDP explicitly permits — discarded the whole 14.8 KB frame exactly like a lost one, and counted nothing. Now reassembled into indexed slots, with counters separating reorder / loss / duplicate / invalid |

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

- **Status:** **fix implemented 2026-07-30, UNVERIFIED on hardware** (see "Why unverified") ·
  **Recorded:** Phase 3 · **Area:** firmware + host
- **Where:** `firmware/scanner-stream/Src/vl53l9_app.c` (`tud_cdc_line_state_cb`, `rs_cdc_send`);
  forensics in `docs/connect-transient-forensics.md`

On host connect (DTR rising) the first frame boundary lands mid-stream: exactly one CRC failure
and a stale RAW skip, then clean streaming. Shipped mitigation: manual `SEND_CALIB` (`C` key /
`roomscan-ctl calib`).

**Correction to this entry (2026-07-30):** it used to say "root-caused to stale TX FIFO residue
(not a DTR race)". `docs/connect-transient-forensics.md`, which is the primary evidence, says the
opposite in as many words — *"**Not** stale TX FIFO residue — the truncated bytes are a genuine
frame-1 payload prefix"*. The forensic root cause is `rs_cdc_send()`'s own 100 ms abort firing
once, because the host's reader has not started draining by the time the device begins frame 1;
the bytes on the wire are the intact **front** of a real frame, cut off mid-send. The summary here
had drifted from the document it cites.

### Implemented 2026-07-30 (the deferred `tud_cdc_line_state_cb` fix)

`tud_cdc_line_state_cb` now watches for the DTR **edge**; on a rise it clears the CDC TX FIFO and
sets one volatile flag. `rs_cdc_send()` checks that flag after each `tud_task()` and abandons the
frame in flight (the new host can only ever see its tail); the acquisition loop consumes the flag
at its existing per-frame safe point and zeroes `rs_calib_countdown`, so the next frame group leads
with CALIB instead of the host waiting out up to 63 frames.

The synchronisation that deferred this turned out to be smaller than it looked, and the reason is
checkable rather than assumed: TinyUSB's class callbacks **do not run in interrupt context** in
this build. `USB_DRD_FS_IRQHandler` calls `tud_int_handler`, which only enqueues a `dcd_event_t`
(`vendor/tinyusb/src/device/usbd.c`, `osal_queue_send`); `tud_task()` dequeues and dispatches. So
the callback runs on the acquisition loop's own thread — but **reentrantly**, since `rs_cdc_send()`
calls `tud_task()` between chunks. That makes it a reentrancy problem, not a concurrency one: one
volatile flag set in the callback and consumed in the loop is sufficient, and no state the loop
owns (`raw_mem_index`, the CALIB countdown, the in-flight byte cursor) is touched from the callback.

**It cannot affect the Ethernet path, by construction.** The callback only fires for a USB host,
and the abort only shortens the CDC copy: `rs_send_generic_cdc()` hands the whole frame to
`ETH_SendFrame_Gather()` *before* the CDC loop starts, so an Ethernet host cannot see a truncated
frame regardless of what a USB host does. On-rig after this change, over Ethernet: 30.3 fps,
0 CRC failures, 0 drops, 0 gaps.

### Why unverified

**The native CDC device does not exist on this host.** `lsusb` sees only the ST-LINK
(`0483:374e`) — the board's USB_USER port is powered from the battery bridge, not connected to
this machine, so `CAFE:4001` never enumerates and there is no DTR to raise. Even if it did,
`/dev/ttyACM*` come back `root:root` mode 0 after every replug on this box and `dialout` does not
help (see the `firmware-build-on-linux` memory). So no claim is made that the transient is gone:
the code path has never executed. What *is* verified is that shipping it did not disturb the
Ethernet path, which is what the rig actually runs on.

To verify, on a machine with the board's USB_USER cable attached: capture with
`host/tools/capture.py --seconds 15`, and check `capture_analyze` reports **0** CRC failures in
the connect region (today: exactly 1) and that the first frame after connect is CALIB.

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

---

## BUG-020 — Native transform loader was Windows-only → blank viewer on the Linux headless host

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, post-migration: "migrated this
  to a headless host... launching view-web.sh loads the page, but I see no data") · **Area:** host/native
- **Where:** `host/src/roomscan/native.py` (`_DLL_NAME`, `_candidate_paths`, `_BUILD_HINT`);
  observability side in `host/src/roomscan/web.py` + `host/src/roomscan/static/app.js`

The project was developed on Windows; this was its first run on Linux (a headless Proxmox/LXC host
streaming from the board over Ethernet). The board streams RAW `3DMD` frames and the PC runs the
`vl53l9-transform-c` pipeline through a compiled native library (`host/transform/`, Phase 2). The
loader that finds and `ctypes.CDLL()`s that library was hardcoded to Windows in two ways:

1. **Library name.** `_DLL_NAME = "roomscan_transform.dll"` — but Linux CMake (single-config
   Ninja/Make) emits `libroomscan_transform.so`, and macOS `libroomscan_transform.dylib`.
2. **Search path.** `_candidate_paths()` only looked in `build/Release/` — the MSVC multi-config
   layout. Single-config generators emit straight into `build/`.

Net effect on Linux: `_find_dll()` returned `None`, `Transform.__init__` raised
`RuntimeError(_BUILD_HINT)` on the first RAW frame, and — because `viewer._reader` deliberately
swallows exceptions into `fault` so one bad frame can't kill the process — the web viewer sat
**silently blank**. `get_best_source` had already succeeded (UDP probe reads raw bytes, needs no
native lib), so the socket was connected and the page said "Live", masking the real failure. The
build hint it *would* have shown was itself Windows-only (`-G "Visual Studio 18 2026"`).

Prerequisite (done by owner, same session): the `53L9A1` reference package — previously an untracked
sibling dir (`../53L9A1/`) that never migrated — was vendored in-repo at `firmware/vendor/53L9A1/`,
and `host/transform/CMakeLists.txt`'s `PKG_ROOT` repointed there. That unblocked the build; this bug
is the loader half.

**Fix:**
1. `_DLL_NAME` is now chosen per `sys.platform`: `.dll` (win32) / `.dylib` (darwin) / `.so` (else).
2. `_candidate_paths()` searches both `build/Release/<name>` (MSVC) and `build/<name>`
   (single-config), in addition to the `ROOMSCAN_TRANSFORM_DLL` env override and the alongside-package
   location.
3. `_BUILD_HINT` is platform-aware (Linux/macOS get `-DCMAKE_BUILD_TYPE=Release` + `cmake --build`,
   and the correct library name).
4. **Observability** (so the next silent-blank is a one-line answer, cf. BUG-018): `web.py` logs the
   selected transport at startup (`[source] Ethernet/UDP -> ip:port` / `Serial CDC` / `Replay`,
   flushed), a watchdog thread prints `[FATAL] reader thread stopped: ...` to stderr the instant
   `fault` is set, and the WebSocket handler sends a JSON `{"type":"error"}` text frame to the page on
   fault. `app.js` now distinguishes text frames (status/error → shown in the connection indicator)
   from binary point frames instead of blindly feeding every message into `Float32Array`.

Verified end-to-end against the live board on the Linux host: loader resolves
`host/transform/build/libroomscan_transform.so`, UDP source connects to `172.17.2.58`, pipeline
produces valid depth `(42×54)`, 100% valid, 564–12000 mm, at ~30.5 fps.

**Note:** the full uvicorn server can't be exercised from inside the agent's sandbox (the TCP bind is
killed, exit 144), so the end-to-end verification drives the identical source→pump→`TransformStage`
data path directly rather than through `roomscan.web`. The server code paths themselves were
import-checked only. A running instance started *before* the loader fix keeps the old module in
memory (Python caches imports) — it must be **restarted** to pick up the fix.

**Follow-on robustness (same session):** `UdpSource` now re-sends a wake
datagram every `keepalive_s` (default 1.0 s) from `read()`, not just during the
startup probe. The board unicasts frames to whichever host last woke it and
clears that target only on reboot, so a board reset / link flap / a second
client claiming the stream previously silenced the viewer permanently (it never
re-woke). Symptom seen live: a launched instance held UDP 5000 and picked
Ethernet, then processed zero frames (flat CPU) while the board streamed
elsewhere. Keepalive makes the app re-claim the target and self-heal. Verified:
streaming intact with keepalive on (110 slots/5 s); 580 host tests pass.

---

## BUG-021 — Web viewer loaded Three.js from a CDN → "Offline", blank on headless host

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, post-migration: "It says
  offline") · **Area:** host/web
- **Where:** `host/src/roomscan/static/index.html` (import map), `host/src/roomscan/static/app.js`

The Three.js viewer front-end resolved its `import * as THREE from 'three'` and
`three/addons/controls/OrbitControls.js` via an import map pointing at
`https://unpkg.com/three@0.160.0/...`, and pulled Inter/JetBrains-Mono from Google Fonts. On the
Windows dev box (with internet in the same browser) this worked. On the migrated headless host the
rendering browser couldn't fetch the CDN module — a remote browser without a route to unpkg, a proxy,
or a stale tab from a load that failed — so the bare `three` specifier failed to resolve, `app.js`
threw at module-eval, `connect()` never ran, and no WebSocket was ever opened. Because
`#conn-text`'s initial markup is literally `Offline`, the page just sat there: server serving live
frames (verified: a WebSocket client and a fresh Chrome both pulled ~2160-pt frames), browser showing
"Offline", no error the user could see. Note the host shell *could* `curl` unpkg fine — CDN
reachability from the shell says nothing about the browser actually rendering the page.

**Fix:** vendored `three.module.js` + `OrbitControls.js` into
`host/src/roomscan/static/vendor/three/` and repointed the import map at `/static/vendor/three/...`,
so the viewer is fully self-contained (same rationale as the Artifacts "no external hosts" rule and
the firmware's vendored tinyusb/lwip). The Google-Fonts `<link>` is left as-is: it degrades to system
fonts if unreachable and never blocks scripts. Verified end-to-end with headless Chrome against the
live server: status **Live**, 2247 points, point cloud rendering, zero console errors.

**Note:** this was the third distinct Windows→Linux migration gap in one session, after BUG-020
(native loader) and the BUG-020 keepalive follow-on. All three shared a shape: something implicit on
the dev box (a built DLL, a CDN, a still-awake board) that the fresh host didn't have.

---

## BUG-022 — Headless-host browser has no WebGL → viewer stuck "Offline"

- **Status:** **fixed** 2026-07-15 · **Reported:** 2026-07-15 (owner, via the BUG-021 in-browser
  diag trace: `Uncaught Error: Error creating WebGL context @ three.module.js`) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` (`_open_browser`)

After BUG-021 vendored three.js, the diag trace showed the real wall: three.js loaded (`THREE r160`),
`app.js` ran, then `new THREE.WebGLRenderer()` (module top-level) threw **"Error creating WebGL
context"** — so execution aborted *before* `connect()`, no WebSocket ever opened, page frozen at its
initial "Offline". The host is a headless Proxmox/LXC box with no GPU; the VNC X server (`:1`) has
software OpenGL (llvmpipe, GL 4.5, confirmed via `glxinfo`), but **Chrome 150 refuses software WebGL
by default** (SwiftShader-for-WebGL is gated behind a flag since ~Chrome 137). Measured on-box:

| Chrome launch | WebGL |
|---|---|
| no flags (what the auto-open used) | **NO-WEBGL** |
| `--enable-unsafe-swiftshader` | OK (SwiftShader) |
| `--use-gl=angle --use-angle=swiftshader` | OK (SwiftShader) |
| `--use-gl=angle --use-angle=gl --ignore-gpu-blocklist` | OK (Mesa llvmpipe GL 4.5) |

`webbrowser.open` (the viewer's auto-open) passes no flags — exactly the failing row.

**Fix:** `_open_browser()` launches google-chrome/chromium with `--enable-unsafe-swiftshader` on
Linux (only *permits* the software fallback; a real GPU still uses hardware, so it's unconditional-
safe), falling back to `webbrowser.open`. `ROOMSCAN_NO_BROWSER=1` skips auto-open for remote viewing.
Verified: with the flag, Chrome on `:1` reports `WEBGL-OK`; the viewer renders Live (headless-Chrome
screenshot: status Live, ~2100 pts, point cloud). This was the 4th distinct headless-migration gap
(after BUG-020 loader, BUG-020 keepalive, BUG-021 CDN) — all "implicit on the dev box, absent on the
fresh host".

**The in-browser diagnostic panel (added this session) is what cracked it** — it turned an opaque
"Offline" into the exact throwing line. Keep it.

---

## BUG-023 — System clock sourced from the ST-LINK's MCO

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner: "I am now powering the device
  over USBUSER and have removed the STLINK connection… this is preventing me from launching the
  application (which does not depend on stlink, just ethernet)") · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/main.c` (`SystemClock_Config`),
  `53L9A1_PostprocessSingle.ioc` (`RCC.PLLSourceVirtual`/`PLLM`/`PLLN`/`PLLFRACN`)

The NUCLEO-H563ZI has **no HSE crystal fitted**. `OSC_IN`/`PH0` is driven by `STLK_MCO`, the 8 MHz MCO
output of the on-board ST-LINK MCU (MB1404 schematic sheet 9: `T_MCO` → R77 → buffer U10 → SB49/SB50 →
`PH0-OSC_IN`; X3's footprint is unpopulated) — which is why the generated config asked for
`RCC_HSE_BYPASS_DIGITAL` rather than a crystal. That buffer runs off `3V3_STLK`, derived from
`VBUS_STLK`, i.e. **from the ST-LINK USB cable on CN1**.

So with the ST-LINK unplugged and the board powered from USB_USER (JP2 moved to 9-10 per the
schematic's "In SINK mode, the jumper JP2 must be set to 'USB USER'"), the system clock simply had no
source. The firmware wedged before `ETH_Init()` and the board sat there **powered, PHY link LED on,
activity LED flashing, and the network completely silent** — no DHCP, no ARP entry, no mDNS, nothing on
UDP :5000. Those PHY LEDs come from the LAN8742's own power-on autonegotiation and the switch's
broadcast traffic, so they indicate nothing about whether firmware is running. Diagnosis: board absent
from a full `172.17.2.0/24` ARP sweep and from a raw mDNS query pinned to the board's NIC.

**Attempted and rejected:** detecting the `HAL_RCC_OscConfig` failure and falling back to HSI. It
rescued a *forced* HSE failure (`RCC_HSE_ON` with no crystal, ST-LINK attached) on the bench but **did
not rescue the real unplugged case**, so HSE's absence does not present as a clean `HSERDY` timeout.
Best theory: the undriven `OSC_IN` pin floats and self-oscillates on noise in digital-bypass mode, so
`HSERDY` sets and PLL1 then locks onto garbage instead of timing out. Do not try to detect this.

**Fix:** PLL1 sources from **HSI unconditionally**; HSE is never enabled. `PLLM 4 / PLLN 31 /
FRACN 2048` puts HSI/4 = 16 MHz in the same `VCIRANGE_3` input band the 8 MHz HSE used, giving the same
500 MHz VCO and the same 250 MHz SYSCLK — every downstream clock is bit-identical. USB runs off HSI48
and Ethernet off the LAN8742's own 25 MHz crystal, so neither transport is affected. The only cost is
HSI's ~1% RC accuracy on frame timestamps, measured as immaterial: **91.5 fps decoded-frame rate vs the
91.4 fps HSE baseline**, inside run-to-run noise. Owner-verified streaming with the ST-LINK physically
unplugged.

**Also added, because this failure was invisible:** boot-progress LEDs (`BootLedsInit` /
`rs_boot_heartbeat` in `main.c`) — all dark = never reached `main()`; LD3 red = wedged in init or
`Error_Handler()`/`handle_error()`; LD1 green = clocks + peripherals up; LD2 yellow blinking = the
acquisition loop is turning over. Note the heartbeat must live inside `vl53l9_app()`'s own `while(1)`:
that function never returns, so a heartbeat in `main()`'s outer loop fires exactly once at boot and
then sits dark while the board streams normally (this cost a diagnostic round-trip).

---

## BUG-024 — A missing CDC serial port aborted an Ethernet-only launch

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner, same session as BUG-023) ·
  **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`get_best_source`)

`get_best_source` probes UDP first and falls back to `SerialSource`, whose `find_port()` raises
`RuntimeError: no scanner serial port found among [...]` when no CAFE:4001 device is enumerated. On a
headless Ethernet rig — ST-LINK unplugged, USB_USER carrying power only — there is legitimately no
scanner serial port at all, so `roomscan-web` died with a serial traceback for a deployment that only
ever wanted Ethernet. It also lost a launch whenever the board happened to be booting (DHCP) during the
5 s probe window.

**Fix:** a *missing* port is no longer a launch blocker — Ethernet has been the production transport
since Phase 5. `get_best_source` hands back the UDP source instead, whose keepalive keeps re-waking the
board so the stream starts by itself once the device appears, and prints a clear one-line reason to
stderr. A *busy* port still raises: that means a real scanner port exists and something else holds it,
which `panel._open_source` offers to resolve interactively. Classification reuses
`portguard.classify_open_error`.

---

## BUG-025 — `UdpSource` retargeted onto its own looped-back broadcast wake

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (found while verifying BUG-024: the
  launch banner read `[source] Ethernet/UDP · 172.17.2.54` — the host's *own* address) ·
  **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource.read`)

When mDNS resolves nothing, `_resolve_target` falls back to broadcasting the wake datagram. Linux loops
that 1-byte broadcast straight back into the same socket, and `read()` set `self.target_ip = addr[0]`
from **any** sender before checking the length — so the source adopted *the host itself* as the device
on its very first read, permanently. Every subsequent wake and keepalive then went to us instead of the
board, which is never told where to stream. Self-poisoning, and it would have broken Ethernet discovery
even with a perfectly healthy board (it is only masked when mDNS succeeds and supplies a unicast IP).

**Fix:** only a datagram long enough to be a real fragment (≥ 6 B, the fragment sub-header) may
retarget; short datagrams return `b""` without touching `target_ip`.

## BUG-026 — Web UI never gravity-aligned the view; "Reset Heading" cannot fix tilt

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner: "powering it up with the board
  upside down shows an upside down view in both IR and point cloud, and Reset Heading doesn't fix it")
  · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` (`_broadcaster`), `host/src/roomscan/static/ir.js`

Gravity alignment existed **only in the deprecated desktop panel** and was never ported to the web app
when it became the primary UI (Web Phases 1–5). `panel.py:3020` rolled the IR pane with
`ir_gravity_rot`, and `panel.py:1332-1337` gravity-aligned the orbit-mode cloud with
`T_WORLD_TO_CV @ R @ T_CV_TO_BODY` — but `web.py` shipped raw deprojected CV-frame points and an
unrotated IR array. `ir_gravity_rot` was imported nowhere outside `panel.py` and its tests. The
orientation matrix *was* already on the wire (`build_sensor_message`'s `rot`), but its only consumer
was the 2D gizmo canvas; `scene.js` never read it.

"Reset Heading" is a red herring by construction, not a second bug: it sends `reset_fusion` →
`YawFusion.reset()`, which clears only the accumulated **yaw** delta. The fusion is built on
`graft_yaw`, whose contract is "roll/pitch are preserved" — tilt comes from the accelerometer and is
deliberately never touched (`docs/coordinate-frames.md`). No yaw reset can un-flip a view.

**Fix:** the broadcaster pre-multiplies POINT_CLOUD/SURFACE positions by `display_rotation(quat)` and
rolls IR_IMAGE via `ir_gravity_rot` (owner chose continuous full alignment, desktop-panel parity).
`ir.js` now sets `canvas.style.aspectRatio` from the message, because a 90°/270° roll makes the pane
portrait (42×54) and the CSS had a hardcoded `aspect-ratio: 4/3` that would have squashed it.
Verified live: the broadcast cloud's ray grid is destroyed (2094×2016 distinct directions) and `rot⁻¹`
recovers exactly the 54×42 sensor grid. Protocol note in `docs/web-protocol.md`.

**Follow-up (2026-07-29, owner: "the IR preview window needs to rotate too").** The fix above gave the
cloud *continuous* alignment but the IR pane only the panel's **quarter-turn snap**, so the two disagreed
by up to 45°: at ~40° of roll the snap is **zero turns** and the pane sat still while the cloud tilted the
full 40°. Half-ported parity, not a new root cause. The rotation is now split — the server keeps the snap
(pixel-exact, and it is what makes the dimensions swap) and publishes the ≤45° remainder as
**`ir_roll_deg`** on the `sensor` message; `ir.js` finishes it with a CSS transform, so the 54×42 image is
never resampled. `ir_roll_deg` is CCW-positive (`np.rot90`'s sense, which is CCW on screen too), so CSS —
which turns clockwise — negates it; the sign was established by a marker test, not by reasoning about
conventions. It is computed from the **smoothed** display quat, the same one the snap uses, or the two
disagree at a 45° boundary and the pane jumps. The image now spins inside a fixed **square** frame
(`#ir-frame`) scaled so the rotated bounding box always fits, so the card never changes shape and no field
of view is cropped (empty corners at intermediate angles; worst at 45°, where it exactly inscribes).
Verified by DOM readback at 14.6°/31.6°/44.6° residual (all `fits:true`; at 44.6° the bbox is exactly
278×278 in a 278 px frame) and live at 169° roll, where snap 180° + residual −11.0° sums to the 169° the
cloud uses. **Caught in review:** `build_sensor_message` already had a *local* `display_quat` (the
yaw-offset orientation view), so the new parameter of that name was silently clobbered and the residual
was being taken from the raw fused quat — renamed `ir_display_quat`.

**Follow-up 2 (2026-07-29, owner: "it's also rotating the contents — if a target is upright in one
orientation it should remain upright at all orientations").** The roll was applied in the **wrong
direction**, a sign inversion inherited from `panel.py`. `atan2(gx, gy)` measures *where gravity sits*;
the correction is its **negative**. Applying `+angle` turned the content the wrong way, so rather than
holding still it counter-rotated at **2x** the board's rate — worse than doing nothing.

Why the first round missed it: both verifications were **sign-blind**. The upside-down check used 180°,
where a sign flip is a no-op (−180 ≡ +180), and the 90° check only asserted the width/height swap, which
happens either way. Fixed by one negation in `ir_gravity_angle_deg` (so `ir_gravity_rot`,
`ir_gravity_residual_deg` and the CSS transform all follow).

The trap that caused it: `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` rotates points in the **CV frame, where Y
points down**, so a positive rotation there is *clockwise* on screen — while `np.rot90` is
*counter-clockwise*. The intended angle was right all along and matched the cloud exactly; only the
application flipped.

**Proven two independent ways, neither of which is a restatement of the formula.** (1) Exact geometry: the
IR image and the cloud come off the same 54x42 grid, so wherever the *verified* aligned cloud puts the
image's +u axis on screen is where the rotated image must put it — that came out as exactly the negative of
what the code applied, at every angle tested. This is now the regression test
`test_ir_gravity_angle_matches_the_point_cloud_rotation`, plus
`test_applied_rotation_stabilises_rather_than_doubling` which asserts the 2x failure mode directly.
(2) Real data: on `captures/web_20260729_174331.bin` (a physical boresight roll over a 179° span, so the
scene genuinely co-rotates), the dominant edge orientation of the raw reflectance plus the applied rotation
has a spread of **15-18°** under the fix versus **24-48°** inverted and **46-60°** uncorrected — i.e. the old
convention was *worse than applying no correction at all*, the signature of double rotation. The ~15° floor
is the structure-tensor metric's own resolution on a 54x42 image, not residual error (it does not tighten as
edge quality rises). Owner confirmed visually on the same recording. Two tests that had *encoded* the
inversion (`test_sensors.py::test_ir_gravity_rot_roll_90_cw`/`_ccw`, asserting 1 and 3) were corrected to 3
and 1 with the reasoning written down.

## BUG-027 — SFLP quaternion aliased by unfiltered 480 Hz → 30 Hz decimation

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner, after BUG-026: "the point cloud
  is very slightly jittery now, especially noticeable at the edges") · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/rs_lsm.c` (`rs_lsm_read_latest`)

Gravity-aligning the cloud (BUG-026) multiplied the orientation signal by the scene's lever arm, which
exposed a latent firmware defect: SFLP runs at 480 Hz, the host consumes one sample per ToF frame
(~30 Hz), and `rs_lsm_read_latest` drained the FIFO keeping only the **last** quaternion of each
~16-sample batch. Point-sampling a 480 Hz signal at 30 Hz folds the entire 0–240 Hz noise band into
0–15 Hz — textbook aliasing, with no anti-alias filter anywhere in the chain. Measured on a stationary
rig: 0.14° mean / 0.25° p95 of change per frame, net 0.14° over 15 s against 22.9° summed absolute
(ratio 0.006 — essentially all noise), i.e. ~7 mm mean / 13 mm p95 of edge shimmer at 3 m.

Two adjacent defaults were wrong for this rig and fixed in the same pass: the gyro's **LPF1 was
bypassed** (POR default), leaving the chain 187 Hz wide at ODR 480 Hz (AN5763 Table 20, the ODR = 480 Hz
block — the 342 Hz row is 960 Hz); and **LIS2MDL `CFG_REG_B` sat at its 0x00 POR default**, i.e. the
4.5 mG RMS / ODR÷2 corner of AN5069 Table 9 with both the low-pass and offset cancellation off.

**Fix:** `RS_LSM_SFLP_AVERAGE` averages the batch (correct decimation, √N on white noise; sign-aligned
before accumulating since q and −q are the same rotation); `RS_LSM_GY_LPF1_*` enables gyro LPF1 at
28.4 Hz; `CFG_REG_B = OFF_CANC | LPF` moves the mag to 3.0 mG RMS / 25 Hz. **Measured, firmware alone,
host smoothing bypassed:** 0.0329 → 0.0118 deg/frame (1.72 → 0.62 mm of edge motion at 3 m), **2.8×**,
with streams 7/9/10 still at 30.3 Hz, 0 drops, 0 gaps.

**Remaining floor is the LSM's FIFO encoding, not the sensor.** The SFLP game-rotation vector is
batched as three **IEEE-754 half-precision** components with w reconstructed
(`sflp_word_to_quat`); the fp16 ulp near 0.7 is ~4.9e-4, and a perturbation δ in a quaternion's vector
part is ≈2δ of angle → ~0.056° per step. Dithered over 16 samples that predicts 0.014 deg/frame against
0.0118 measured, and the residual's directional coherence is **0.14** — *below* the ~0.32 of white noise
at window 10, i.e. anti-correlated, the signature of quantization dither rather than sensor noise or
tripod vibration. AN5763 §6.5: SFLP data is readable **from the FIFO only**, so there is no
higher-precision register path. ~~Because fp16 is floating point the floor should **vary with
orientation** (finer near identity) — an untested prediction. Beating it means leaving the SFLP FIFO
format (batch raw XL/GY and fuse host-side); **open, not attempted.**~~ Analysis + method in
`docs/iks4a1-stacking.md` → "Orientation-noise pass"; measure with `host/tools/orientation_probe.py`.

**Superseded 2026-07-29 — both open items above were closed:**

*Orientation dependence: **CONFIRMED**.* Two-point test (owner rotated the rig ~90°). Old pose fp16
step RMS 0.03956°, k_eff 4.85, model 0.01797° vs **measured 0.01782° (ratio 0.99)**; new pose step
0.04846°, k_eff 4.84, model 0.02202° vs measured 0.02670° (ratio 1.21). Step ratio new/old 1.225,
measured ratio 1.498 — right direction, ~21% under-predicted at the coarser pose.

*The floor is **dither-limited**, not step-limited — a quieter board measures WORSE.* Averaging 16
samples only buys √16 if input noise keeps the quantizer toggling. Measured k_eff ≈ **5 of 16**
(tie fractions 14–28%, holds up to 18 consecutive frames), corroborated independently by the 1.85×
excess of measured over naive model under √k scaling → k_eff ≈ 4.7. This explained an apparent
0.0118 → 0.0183 "regression" after a power cycle that was **not** a regression: last session
back-solves to k_eff ≈ 16 (well dithered). Nothing had broken.

*Escaping fp16: **shipped as stream 11** (`RS_STREAM_IMU_RAW`) — 480 Hz raw FIFO pass-through of
GY/XL/timestamp/SFLP-gravity/SFLP-gbias, all 16-bit fixed point. Verified on-target: 100.01% of
samples delivered, 0 gaps, median tick delta exactly 96 ticks = 480.0 Hz. Host complementary filter
`roomscan.imufusion` built and **gated OFF by default** (SLAM non-regression guarded by test);
synthetic gain 6.2× on tilt in the under-dithered regime. **Not yet wired into the live display —
that is the resume point.**

*Also note (2026-07-29): the fp16 floor turned out **not** to be what the owner was actually seeing.
The visible noise is the eCompass — see BUG-030.*

## BUG-028 — Ethernet hot-plug replug wedges board (lwIP double-add assertion)

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner) · **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/ethernet_transport.c`

When the Ethernet cable is unplugged and plugged back in, the firmware wedged. The PHY link LED turned on, but LD2 stopped blinking, indicating an infinite loop. The root cause was that `ETH_Process` called `mdns_resp_add_netif` a second time on the same network interface after the DHCP lease was re-acquired. The ST port's `LWIP_PLATFORM_ASSERT` trapped the "Double add" assertion into an infinite `while(1)` loop without triggering a hard fault handler (so LD3 stayed off).

**Fix:** Introduced a static flag `mdns_added` to ensure `mdns_resp_add_netif` and `mdns_resp_add_service` are only called once. Subsequent IP updates now correctly rely only on `mdns_resp_netif_settings_changed`.

## BUG-029 — UdpSource stream recovery fails due to broadcast routing

- **Status:** **fixed** 2026-07-28 · **Reported:** 2026-07-28 (owner) · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` (`UdpSource._maybe_keepalive`)

Even after the firmware crash (BUG-028) was fixed, the `UdpSource` stream would not recover after a replug until the python server was restarted. The source was correctly detecting a stream timeout (>2s) but was falling back to sending its keepalive wake datagram to `255.255.255.255`. On many Linux/Docker host setups, raw broadcasts to `255.255.255.255` fail to route out of the physical Ethernet interface and instead go to a virtual bridge, meaning the board never received the keepalives.

**Fix:** Updated `_maybe_keepalive` to actively re-query the board's IP via mDNS (`_resolve_target`) every keepalive interval when the stream is dead. This learns the new (or same) IP and resumes unicast wake packets, gracefully restoring the stream.

## BUG-030 — Magnetometer calibration is direction-dependent: |B| ranges 47→85 µT with tilt

- **Status:** **fixed 2026-07-30** (owner re-fit, validated on independent data — see "RESOLUTION"
  at the end of this entry) · **Reported:** 2026-07-29 (owner: "most of the noise I see in the UI is
  in the eCompass") · **Area:** host/sensors (calibration data, not code)
- **Where:** `mag_cal.json` at the **repo root** — the path a `roomscan-web` started from the repo
  root resolves `ViewerConfig.mag_cal_path` to; consumed via `host/src/roomscan/magcal.py` →
  `MagCalibration.apply()`. (Was `host/mag_cal.json`, fitted 2026-07-15, `field_ut = 49.87`;
  **that file is deleted** — see "The two-file trap" below.)

A correctly calibrated magnetometer reports a **constant** field magnitude at every orientation —
that is the defining property. Ours does not. Measured on a deliberate braced tilt sweep
(2026-07-29, `captures/web_20260729_061440.bin`, 8 stationary holds, current calibration applied):

| tilt from vertical | 0.3° | 0.2° | 30.5° | 60.6° | 80.0° | 90.6° | 30.6° | 2.8° |
|---|---|---|---|---|---|---|---|---|
| heading | 147.0° | 144.4° | 157.9° | 149.7° | **79.6°** | **239.6°** | 158.1° | 146.2° |
| **\|B\| µT** | 50.5 | 47.4 | 58.7 | 76.5 | 81.1 | 85.1 | 62.8 | 50.7 |

Accurate at ceiling-facing (≈ the fitted 49.87 µT) and degrading monotonically to ~1.7× toward
horizontal — the signature of an **incomplete calibration tumble**: good coverage in one attitude
family, poor everywhere else. Consequence is not noise but **systematic heading error up to ~90°**,
and it occurs precisely in the horizontal wall-scanning attitude the device is actually used in.
(An earlier near-vertical tripod pose read ~107–109 µT, i.e. a 2.15× anomaly — consistent story.)
That deviation also exceeds `YawFusion.anomaly_frac` (0.3 → ±15 µT), so yaw fusion is silently
**gated off** at those poses while the displayed `heading` — which ignores the gates — keeps showing
the biased value.

**The compass noise is magnetometer-dominated, not orientation-dominated.** Holding the quaternion
fixed and varying only the magnetometer reproduces essentially all of the heading jitter at every
attitude (mag-only 1.297° of 1.299° total at 0°; 0.780° of 0.779° at 30°; 1.185° of 1.187° at 60°).
Tilt-error propagation contributes 0.004–0.05° below 60°, rising to 0.182° at 80° and 0.944° at
90.6° (the DT0058 gimbal blow-up — real, but an order of magnitude below the calibration error).
Orientation-estimate jitter itself is excellent throughout: p95 0.006–0.079°/frame.

**Application-note review 2026-07-29** (`docs/imu-mag-appnote-review-2026-07-29.md`) narrowed this:
- **Ruled out:** magnetometer byte-tearing (missing BDU was real and is fixed in `46b81b3`, but measured
  on 27339 stationary samples the max jump is 23 LSB with ZERO above 64 LSB — no tears); **installation
  error** (DT0103 — de-rotation preserves magnitude, so it cannot change |B|); and `AXIS_CONVENTION`
  (an orthogonal sign matrix cannot change magnitude).
- **Arithmetically excluded:** a simple hard-iron residual. Reaching the observed 85 µT max from a
  ~50 µT field needs |δ| ≈ 35 µT, which would drive the minimum to ~15 µT; measured minimum is 47.4 µT.
- **Still live**, both consistent with the 85.1/47.4 = **1.80** max/min ratio: a soft-iron/diagonal-gain
  error the fit mis-estimates (follows the device anywhere), or a **world-fixed interferer — most
  likely the tripod**, since tilting on it *translates* the sensor through an arc past ferrous mass.
  The latter also explains the ~66 µT mean vs the 49.87 µT fit if the original tumble was hand-held.
- **Therefore calibrate HAND-HELD in open space, away from the tripod.** Calibrating while mounted
  would bake a position-dependent error into the fit that misbehaves in handheld use — the actual use
  case. Discriminating test (~2 min): a hand-held level/45°/vertical check; flat |B| implicates the
  tripod, a persistent 1.8:1 swing implicates the soft-iron model.
- **Also found (DT0103, separate from |B|):** a general ellipsoid fit is **ambiguous up to a rotation** —
  it can yield a perfectly constant |B| while systematically rotating the field vector, i.e. right
  magnitude, wrong heading. 3D fitting cannot detect this; DT0103's accelerometer-assisted method pins
  the magnetometer frame to the body frame. Worth adopting for heading accuracy independently of BUG-030.

## ROOT CAUSE (2026-07-29) — two faults stacked; fixing one does not fix the other

An owner hand-held tumble in open space (`captures/web_20260729_175941.bin`, 718 samples, ~24 s)
produced a candidate fit that is **validated on independent data**:

| capture (under the candidate fit) | \|B\| mean | std | ratio |
|---|---|---|---|
| tumble 17:59 — hand-held, *fit source* | 49.85 µT | 0.65% | 1.05 |
| tumble 17:43 — hand-held, **independent** | **50.06 µT** | **1.23%** | 1.07 |
| tilt sweep — on tripod, moving | 77.18 µT | 11.25% | 1.83 |
| stationary 900 s — on tripod, fixed 86° | 64.64 µT | **0.46%** | 1.04 |

**Fault 1 — the saved hard-iron offset was wrong by ~59 µT.** Saved `[44.4, −27.6, −41.7]` vs the new
`[25.4, −5.5, +9.2]`, mostly in z. Both soft-iron matrices are near-identity (candidate eigenvalues
0.966/0.991/1.045, axis-gain ratio 1.08), so **soft-iron was never the problem**. Under the saved
calibration the tumble spans 14.7 → 107.6 µT, ratio **7.3**.

**Fault 2 — the tripod is a genuine magnetic interferer, ~15–27 µT (a third to a half of Earth's
field).** See BUG-034; heading is unreliable while the device is mounted, *regardless of calibration*.
The proof is row 4 above: held stationary on the tripod at a fixed attitude, |B| is exquisitely
consistent (0.46%) but at the **wrong magnitude** (64.6 vs 49.9) — an added field is steady when
nothing moves. Row 3, where the sensor swings through an arc on the tripod, becomes 11.25% because the
added field varies with **position**, not orientation.

**Two earlier conclusions in this entry were WRONG and are corrected here** (kept, not deleted, per
repo convention):
- ~~"a simple hard-iron residual is arithmetically excluded"~~ — the arithmetic was right (|δ| ≈ 35 µT
  implies a minimum near 15 µT) but it was applied to a **one-dimensional** tilt sweep that never
  visited the cancelling orientation. The full tumble reaches **14.7 µT**, exactly as predicted.
  Hard iron was the answer all along.
- ~~the tilt sweep is valid independent validation data~~ — it is tripod-contaminated, so testing a fit
  against it measures the tripod. See the `validating-against-suspect-data` memory.

**Remedy (owner, via the UI — in progress):** re-fit and **Save & apply** in the calibration modal,
which also exercises the hot-reload path not yet tested with a real fit. Coverage on the 17:59 tumble
was 54/92 cells (59%), just under the tool's 60% marginal bar, so more tumbling raises confidence —
but the field metrics are good *and* generalise to unseen data, which is the stronger evidence.
Calibrate **hand-held in open space, away from the tripod**: calibrating while mounted would bake a
position-dependent error into a fit used hand-held. ~~`host/mag_cal.json` is still the 2026-07-15 file
until that is done.~~ Sources: DT0058, DT0059, DT0103, AN5069 §5/§8.

## RESOLUTION (2026-07-30) — owner re-fit, validated against a capture the fit never saw

The owner ran a full tumble through the calibration modal **hand-held, off the tripod, with the
metal tripod mount plate still attached** (body-fixed, so its hard iron is calibratable — and it is
now baked into the fit; **removing the plate invalidates the calibration**). Saved via **Save &
apply**, which exercised the hot-reload path for the first time with a real fit. Then recorded an
independent 118 s room sweep, `captures/roomSweepFull20260730.bin` (3574 mag samples at 30.3 Hz,
streams 7/8/9/10/11/12, walking the room).

New fit: hard iron `[24.8, −5.8, +8.6]` (|offset| 26.9 µT), `field_ut` 48.09, soft-iron axis-gain
ratio **1.091**. That offset sits **58.3 µT** from the superseded one — matching the ~59 µT the root
cause above predicted.

Scored with the new `host/tools/mag_check.py` (`capture_magcheck` MCP tool):

| metric on the sweep | new fit | superseded 2026-07-15 fit |
|---|---|---|
| **attitude-locked error** (the verdict) | **0.29 µT = 0.56%** → good | 3.44 µT = 3.81% → marginal |
| **tilt ramp**, \|B\| max/min across 10 tilt bins | **1.042×** → good | 2.721× → bad |
| \|B\| by tilt, 0° (ceiling) → 90° (horizontal) → 180° | 51.5 → 51.5 → 52.7 (flat) | 40.3 → 92.1 → 109.5 (ramp) |
| `YawFusion` `gated:anomaly` | **0%** | 58.6% |
| `YawFusion` `active` | **64.8%** | 6.2% |

The gating row is the practical consequence: fusion was silently off for most of a scan and is now
on (the remainder is `gated:motion` 30% / `gated:gimbal` 5%, both by design). Live on the desk
afterwards: `has_mag_cal=true`, `fusion="Active"`, |B| 50.24 µT over a 49.76–50.87 range.

**The raw spread is NOT the calibration.** `field_consistency` scores this good fit "bad" on the
sweep (std 4.58%, bias +6.91%) — that metric assumes every sample came from one place, which is true
of a tumble and false of a 118 s walk. Detrending |B| with a 5 s rolling median splits the 2.35 µT
total std into **2.00 µT of slow spatial drift** (the building's ferrous mass moving the ambient
field under the operator — |B| walks 49.9 → 53.6 → 48.3 → 54.4 across the room while the *within-bin*
std is often 0.4 µT) and only 0.29 µT locked to body attitude. See `magsweep.attitude_locked_error`.

**Still unproven: heading direction.** |B| flatness proves magnitude, not direction — an ellipsoid
fit is ambiguous up to a rotation (DT0103, noted in the app-note review above), which would give the
right magnitude at a systematically rotated heading. The near-spherical soft iron bounds that at
~2.5°, but the sweep cannot measure it: there is no ground truth in it, and a mag-vs-SFLP-yaw proxy
is invalid because `quat_yaw_deg` is ZYX yaw about body **Z (Forward)** while the SFLP body frame has
**X = Up** — that is not heading. The test that would settle it is a braced, fixed-compass-heading
tilt sweep through level → 45° → vertical, hand-held.

### The two-file trap (found while validating this)

`ViewerConfig.mag_cal_path` is **relative**, so it resolves against the server's cwd — the repo root
in practice. The tracked calibration lived at `host/mag_cal.json`, which the running server therefore
never loaded: it had **no calibration at all** from boot until the 07:22 save created the root file.
Anything run from `host/` (`tools/mag_calibrate.py`, the deprecated panel) would meanwhile have
silently applied the superseded fit.

Fixed by keeping **one** tracked file, at the root, and deleting `host/mag_cal.json`. A missing file
fails loudly and safely (`has_mag_cal=false`, `fusion="gated:no-cal"` in the UI); a stale one is
silently wrong, which is what happened here.

## BUG-031 — ToF frame timestamp and IMU FIFO drain are skewed ~0.9 ms

- **Status:** **fixed** 2026-07-30 (stream 13 IMU_SYNC) · **Reported:** 2026-07-29 (analysis) ·
  **Area:** firmware
- **Where:** `firmware/scanner-stream/Src/vl53l9_app.c` (frame stamp) vs `Src/rs_lsm.c` (FIFO drain)

For a **handheld** scanner, timing error between a depth frame and the IMU sample used to orient it
dominates the orientation-noise floor: at 100 °/s, 1 ms of skew is 0.1° of misalignment — ~10× the
stationary quantization floor. Measured against the LSM's own FIFO timestamps: **1.9 ms RMS /
3.4 ms p95 / 6.2 ms max**. Two causes — `rs_time_us()` was `HAL_GetTick() * 1000` (1 ms granular),
and the stamp was taken at *send* time, so variable processing/transmit latency folded in.

**Mitigated 2026-07-29** by a TIM2-based microsecond clock and moving the stamp to the sensor's
FRAME_READY edge (end of integration). **Measured after: 1072 µs RMS** — better, but far from the
predicted "tens of µs". Note the measurement itself has a ~600 µs floor (it compares against the
*last* IMU sample of each batch, whose phase varies by up to one 2.083 ms sample period), implying
~890 µs of genuine residual.

**Costed 2026-07-30 → recommendation: keep SFLP, do not take ODR-triggered mode.** Full note:
`docs/odr-triggered-sync-costing-2026-07-30.md`. (a) The gating precondition is untested — the
host-side alternative had a broken heading loop (**BUG-039**; *fixed 2026-07-30, and the
recommendation is unchanged* — the precondition went from unmeasurable to merely unmeasured, since
no capture in the repo carries orientation ground truth). (b) INT2 *is* routed and free
(CN9.5 → PE14 = TIM1_CH4), so availability does not settle it — but AN5763 Table 15's **40 ms
minimum T_ref** vs our very stable **33.00 ms** frame period forbids 1:1, and **480 Hz is absent
from the ODR-triggered ODRsel set**. (c) Losing SFLP also removes gravity *and* gbias from stream 11:
raw accel exits the 5% gate **23.1%** of room-sweep frames (SFLP gravity: 0/3000), and gbias is
0.18–0.20 °/s with no host replacement (`KI_BIAS_HZ = 0.0`). (d) Stream 13 `IMU_SYNC` (below) makes
the skew a *measured* hardware quantity with SFLP left on, which removes the last thing
ODR-triggered mode was going to buy.

⚠ **One premise of that costing note is now known wrong.** Its §2.3 assumed the frame-ready stamp sits
at the FIFO batch *end*, and concluded the quaternion was ~15.4 ms **stale**. Stream 13 measured the
actual geometry: the drain sits **+24.3 ms past** the edge, so the quat batch midpoint is **+7.76 ms
AFTER** it — the orientation **leads** the depth frame by ~0.30° at 38.5 °/s. Correcting it in the
"stale" direction would roughly *double* the error. The note's structural conclusions (a)–(c) are
unaffected; its sign is not.

**Remaining root cause (hypothesis, since confirmed and fixed):** the IMU FIFO is drained *later* in the loop
than the ToF frame-ready stamp, so the offset between them still breathes with processing load. The
principled fix is to capture the LSM timestamp **at the frame-ready moment** rather than inferring it
from whichever FIFO words happen to be present at drain time. Verification requires **real motion** —
both defects are invisible on a stationary rig.

**A hardware option, with a hard trade-off (found 2026-07-30 while reading DT0155).** ST's
**ODR-triggered mode** phase-locks the LSM6DSV16X's data generation to an external reference on the
**INT2** pin — the device aligns both frequency and phase to that signal's edges (DT0155,
*"Synchronizing multiple sensors using ODR-triggered mode in MEMS devices"*). Driving INT2 from the
ToF frame clock would make this skew a *hardware* property instead of a software race, which is
strictly stronger than any drain-ordering fix.

⚠ **But it is mutually exclusive with SFLP.** AN5763, verbatim: *"ODR-triggered mode is not compatible
with the pedometer, relative tilt, **SFLP**, DRDY mask, or activity/inactivity functionality (only
motion/stationary can be used)."* It also forbids some ODR settings (CTRL1 `ODR_XL` 0001/0010/1100,
CTRL2 `ODR_G` 0010/1100) and is incompatible with Qvar/EIS. So it is a choice, not an addition:

- **Keep SFLP** (stream 9, the on-chip game rotation vector that `Mapper` uses as its rotation prior)
  and live with a software-timed skew; or
- **Take ODR-triggered sync** and fuse orientation on the host from raw XL/GY — which is exactly what
  stream 11 (480 Hz raw IMU FIFO) and the currently gated-off `roomscan.imufusion` already exist for.

Worth costing before more software mitigation: option 2 removes the skew at the source but puts the
whole orientation path on the host, so it needs `imufusion` to first prove it beats SFLP.

**Costed and declined 2026-07-30** (`docs/odr-triggered-sync-costing-2026-07-30.md`): keep SFLP.
`imufusion` does not currently beat it, AN5763 Table 15's 40 ms minimum `T_ref` does not fit a
33 ms frame period, and 480 Hz is not in ODR-triggered mode's ODR set. The fix below removes the
last thing it would have bought.

### Fixed 2026-07-30 — measure the correspondence instead of inferring it (stream 13)

**The hypothesis was tested before anything was built, and it held.** Every 64th frame also carries
the 2332-byte CALIB blob, which is sent *before* the FIFO drain — a load experiment the firmware
was already running for free. Over 5331 static frames those frames' pairing sat **655 µs later**
than a plain frame's (Welch t = **−5.8**; reproduced at −688 µs / t = −6.1 and −783 µs / t = −4.9
on two later captures). The gap between the frame stamp and the drain therefore moves with
processing load, exactly as predicted.

**The fix:** the firmware now reads the LSM's `TIMESTAMP` register (0x40–0x43) **at the FRAME_READY
edge itself**, in the one window where the shared I3C bus is idle — immediately after the event ack,
before the ToF's DMA readout is kicked — and ships it as **stream 13 (IMU_SYNC)** with the delay
from the edge and the read's own duration, so the residual uncertainty is *reported* rather than
assumed. Costs one 4-byte register read per frame (**26.4 µs, std 0.5**) and no frame rate
(30.26 → 30.29 fps, 0 CRC failures, 0 gaps over 15692 frames).

**Before / after**, same rig, same static scene, same estimator family, windowed 2 s clock fits
(`host/tools/skew_check.py`, MCP `capture_skew`):

| | RMS | p95 | max | frame-to-frame |
|---|---|---|---|---|
| **before** — inferred from the last FIFO word (5332 frames, 176 s) | 1069.9 µs | 1260.6 µs | 5745.9 µs | 1628.7 µs |
| **after** — measured at the edge (3119 frames, 103 s) | **18.3 µs** | **35.8 µs** | **82.3 µs** | **11.0 µs** |

58× on RMS. Two honesty notes on those numbers:

- The old estimator is **unchanged** by the fix (1069.9 → 1050.1 µs on the after-capture) and always
  will be: it measures the drain's phase and lag, which are still there — we simply stopped
  depending on them. The two rows answer "how well can we place a ToF frame on the IMU clock",
  which is the question, not "did one number move".
- The 18.3 µs is **window-dependent** (18 / 38 / 150 µs at 2 / 5 / 20 s) because what survives is
  the two oscillators drifting against each other, lag-1 autocorrelation **0.992** — not per-frame
  skew. The window-free number is the residual's first difference: **11.0 µs**. The old estimator
  is window-*in*dependent (1070 / 1073 / 1085 µs) because its error is white, and it can never
  beat 601 µs anyway (one 2.083 ms sample period of FIFO phase, uniform).

**What the measurement then exposed — and it is bigger than the bug it closes.** With the edge
finally on the LSM's clock, the drain turns out to sit **+24.3 ms** past it (std 848 µs — which is
the "breathing" the host-side residual had inferred at ~900 µs, from the other side). That kills a
premise everyone had been reasoning from, including the ODR costing note above, which assumed the
FRAME_READY stamp sat at the *batch end*. It is 23.1 ms from it. Consequences, measured over 3119
frames:

- Stream 9's quaternion is a **batch mean** (`RS_LSM_SFLP_AVERAGE`, shipped for BUG-027's 2.8×
  noise cut), so it carries the batch's **midpoint** orientation. That midpoint sits **+7.76 ms
  AFTER** the frame-ready edge (std 0.68) — the batch straddles the edge.
- So the orientation attached to a depth frame **leads** it by ~7.8 ms (~0.30° at 38.5 °/s), ~9×
  the skew this bug was about. It is **not** ~15 ms stale, and a correction that propagates the
  quaternion *forward* — the natural reading of "stale" — would roughly double the error.
- Stream 13 therefore also carries `quat_mid_ticks` + `quat_n`, and
  `roomscan.protocol.ImuSync.quat_offset_us()` returns the signed offset, so nobody has to
  re-derive the sign from a capture. Firmware's value agrees with an independent host-side
  computation from stream 11's timestamps to **0.02 ms** (+7.74 vs +7.76).

**Still open, and deliberately not done here:** nothing yet *applies* that correction. The host
still pairs stream 9 with a frame by `seq` and uses it as-is, so the ~7.8 ms lead is measured,
documented and unremoved. Correcting it means propagating the averaged quaternion backward with
stream 11's gyro words — a change to the orientation the SLAM prior and the whole UI read, which
wants its own before/after on a moving capture (`docs/superpowers/plans/2026-07-29-orientation-resume.md`
§4.5). Filed as the next step rather than smuggled in behind a timing fix.

---

## BUG-032 — GPU SLAM OOMs on a long scan (Open3D CUDA cache grows per extraction)

- **Status:** **fixed** 2026-07-29 · **Reported:** 2026-07-16 (CUDA at-scale validation, finding #4) ·
  **Area:** host/slam
- **Where:** `host/src/roomscan/slam/tsdf.py` (`_extract_vbg` / `_release_cache_if_due`)
- **Sub-phase:** ROADMAP 6.G

Over a 68 m walk the GPU SLAM path crept to **~11.7 GB** of CUDA memory and died on a `ParallelFor`
allocation failure, capping how long an unattended GPU scan could run.

**The stated hypothesis was wrong in an important way.** It had been attributed to "the caching
allocator + **per-frame** temporaries never released". Measured with the new rig
(`host/tools/slam_gpu_memory.py`, which logs NVML device bytes *and* active block count per frame so
map-growth and work-growth can be separated):

- **the per-frame path does not leak at all.** 4000 frames / 80 m of integrate + raycast + ICP with no
  extraction: device memory stayed **byte-identical** at 937361408 B while the map grew 900 → 17k blocks.
- **the throttled extraction does.** Add `mesh()` at the live cadence (`SlamWorker._MESH_EVERY = 5`) and
  memory climbs 523 → **5483 MiB over 1500 frames (5.13 MiB/frame)** with the block count nearly flat.
  On this 8 GiB card that OOMs a few hundred frames later.

Mechanism: `_extract_vbg()` does a whole-grid `self._vbg.cpu()` copy (itself the fix for CUDA bug #3 —
marching cubes OOMs on-GPU). Its temporaries scale with the active-block count, so each extraction asks
Open3D's caching allocator for a slightly **larger** block than the last, and the previous one is cached
but never reused again.

**Fix:** `TsdfMap.release_cache_every` — `o3d.core.cuda.release_cache()` after every Nth *extraction*
(default 1; 0 disables; no-op on a CPU grid). Extractions, not frames, is the right cadence: a
frame-counter release would fire mostly on frames that allocated nothing. Exposed as
`[slam] release_cache_every` and plumbed through `Mapper`, the `roomscan-slam` CLI, and
`web.SlamRunner`; the remote backend forwards it in its existing mapper-kwargs JSON.

**After:** 4000 frames / 80 m — **longer than the 68 m walk that OOM'd** — peak **651 MiB**, ending at
523 MiB, tail growth **0.005 MiB/frame**. No per-frame cost: p50 6.1 ms unchanged, p90 7.0 → 7.1,
p99 8.6 → 8.8, wall 62.3 s → 62.7 s over an identical 1500-frame run. The unfixed run would have needed
~21 GB to reach the same frame count.

**Guard:** `tools/slam-container/cuda_smoke.py::run_memory_ceiling` — 1200 frames at the live extraction
cadence, asserting peak ≤ 1500 MiB, tail growth ≤ 0.5 MiB/frame, and that releases actually happened
(so the knob being disabled, or the hook being unwired, fails loudly). Unit coverage for the cadence and
wiring is in `host/tests/test_slam_tsdf.py` (monkeypatched, so it runs on a CPU box).

---

## BUG-033 — Sensors card outgrew the dock band; readouts unreachable and half-duplicated

- **Status:** **fixed** 2026-07-29 · **Reported:** 2026-07-29 (owner: "clean up our sensors panel,
  it's cluttered and hard to read") · **Area:** host/web
- **Where:** `host/src/roomscan/static/index.html` (`#sensors-card` markup + the `.sensor-*` CSS),
  `host/src/roomscan/static/sensors.js`

The card had accreted four flat blocks — Sensors, Orientation View, Raw Orientation, Jitter — plus two
button pairs, ~25 rows in a 232 px rail. Two consequences, one cosmetic and one functional:

- **Functional:** at ~1600 px the card was roughly twice the dock band (883 px at a 1000 px viewport).
  `.dock > * { max-height: 100% }` capped it and the overflow was simply *clipped* with no scroll, so
  the Jitter table — the whole point of the 2026-07-28 noise work — could not be read at any window
  size; and `layout.js`'s degradation ladder ends in `autoCollapse('sensors-card')`, so a narrow
  window hid the card entirely rather than a part of it.
- **Cosmetic but load-bearing for readability:** the numbers were *duplicated*. "Orientation View" and
  "Raw Orientation" both printed Roll/Pitch/Yaw, and in the default `zyx` mode those are the same
  quantity to the digit; heading appeared three times (compass caption, raw heading, and the
  mislabelled "Heading" row that actually showed *fusion state*); World mode printed its gravity+mag
  caveat twice, because `#orient-world-note` said what `ORIENT_MODE_DESC.world` already said. On top
  of that the shared `.hud-row` 16 px gap plus long strings ("p95 0.026° · mean 0.006°",
  "Orientation (always trustworthy)", "No mag calibration") wrapped nearly every row onto two lines,
  and the `<select>` clipped its own longest option mid-word.

**Fix — three always-visible tiers plus one disclosure.** Visible: gizmo + compass → full-width mode
`<select>` → the three (renamable, now borderless-until-hover) orientation values → conditional ⚠
warnings → `Fusion` state, colour-coded by `fusion_key` (green active / amber gated / muted off), with
`Reset Heading` + `Calibrate Mag` beside it → Environment. Inside a collapsed `#sensor-diag`
`<details>`: the mode's singularity note, yaw offset + `Zero Yaw`/`Clear`, full-precision raw ZYX +
quat, and Jitter as a `label · p95 · mean` grid under one unit heading. **Precision was not reduced
anywhere** and the only element removed is the duplicate world note (with its now-dead
`worldNote` binding).

**Two gotchas worth keeping:**

1. **Chrome does not pass a flex-shrunk `<details>`'s height to its content.** The obvious
   construction — `<details>` as a flex column, `.sensor-details__body` as the `flex: 1 1 auto;
   min-height: 0; overflow-y: auto` scroll box — measured `det h=376` with `body h=426`: the body kept
   its natural height and spilled past the card, because Chrome renders details content in an
   anonymous `::details-content` box, so the body is not a direct flex item. The scroll box has to be
   the **`<details>` itself** (`min-height: 0; overflow-y: auto`), with the summary `position: sticky`
   and an *opaque* background so scrolled rows don't read through the glass card.
2. **The scroll container needs its own `pointer-events: auto`.** `.hud-card * { pointer-events: none }`
   sets it explicitly on every descendant, so it defeats inheritance — a deliberate, scoped exception
   to the left dock's "never intercept OrbitControls" rule, justified because the drawer already held
   pointer-active buttons and the wheel would otherwise dolly the camera instead of scrolling.

No `/ws` message, field, or precision changed — `docs/web-protocol.md` is unaffected. Verified in
headless Chrome against `captures/tilt_sweep_20260729.bin` at 1600×1000 and 1280×720: card within the
band in both states (`card=865` / `603`, zero overflow past the dock), drawer scrolls to the full
jitter table, and mode switching, World-mode gating (`Zero Yaw` disabled, ⚠ invalid), zero-yaw/clear
and label rename (propagating to the jitter row labels) all still round-trip through the server.
959 tests, 0 console errors.

## BUG-034 — The tripod adds 15–27 µT: heading is unreliable while mounted

- **Status:** **by-design** (environmental, not our code — recorded so it survives BUG-030's closure) ·
  **Found:** 2026-07-29 while root-causing BUG-030 · **Area:** environment / operating procedure
- **Where:** no code. A property of the physical rig.

Separated from BUG-030 deliberately: BUG-030 is a calibration-data fault that an owner re-fit closes,
whereas this is a permanent constraint that must outlive it.

Measured with a magnetometer calibration that is *known good* (validated on two independent hand-held
tumbles at 49.85 µT / 0.65% and 50.06 µT / 1.23%):

| condition | \|B\| mean | std | interpretation |
|---|---|---|---|
| hand-held, open space | ~50 µT | 0.65–1.23% | Earth's field, correct |
| tripod, **stationary** at a fixed attitude | 64.64 µT | **0.46%** | added field — steady, wrong magnitude |
| tripod, tilting through an arc | 77.18 µT | 11.25% | added field varies with **position** |

The stationary row is the diagnostic tell: **exquisite consistency at the wrong value is the signature
of an added constant**, not of model error or noise. Tilting on a tripod *translates* the sensor through
an arc past ferrous mass (head, centre column, screws), so the field at the sensor changes with position
rather than orientation — which is why the tilt sweep looked like a calibration failure.

**Consequences:**
- **Never calibrate the magnetometer while mounted** — it bakes a position-dependent error into a fit
  that is then used hand-held (the actual use case).
- **Do not trust heading, `absolute_heading`, or `YawFusion` output from tripod-mounted captures**, and
  do not use such captures to validate a calibration. Existing tripod captures
  (`captures/web_20260729_061440.bin`, `captures/stationary_stream11_20260728_190311.bin`) are affected.
- Tripod captures remain perfectly good for ToF, orientation-noise (gravity/gyro) and SLAM work — this
  is a magnetometer-only constraint.

**The tripod is only the loudest instance (2026-07-30).** The *room* does the same thing at a smaller
amplitude: over BUG-030's validation sweep, |B| under a known-good calibration walked 49.9 → 53.6 →
48.3 → 54.4 µT as the operator crossed the room, while the spread *within* any 10 s window stayed
around 0.4 µT. That is ~±6% of position-dependent ambient field with no tripod involved, and it is
irreducible — no calibration can subtract a field that depends on where you are standing. Practical
consequences: judge a calibration by `magsweep.attitude_locked_error` (which detrends this out) rather
than by raw |B| spread, and expect `YawFusion.anomaly_frac` (0.3) to be doing real work indoors.
- Untested: whether a different mount, or more separation between the sensor and the tripod head, brings
  |B| back to ~50 µT. That would confirm the mechanism and might restore mounted heading.

---

## BUG-038 — The live point cloud is frustum-culled against a stale, zero-radius bounding sphere

*(Filed 2026-07-30 as a second "BUG-033" — an ID collision with the sensors-card entry above.
Renumbered to BUG-038 on 2026-07-30 per this tracker's "IDs are never reused" convention. Commits
and docs written before the renumber may still refer to this as BUG-033.)*

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 (surfaced by the FPV view mode) ·
  **Area:** host/web frontend
- **Where:** `host/src/roomscan/static/scene.js` (`points` / `uncovPoints` / `surfaceMesh`)

Three.js computes `BufferGeometry.boundingSphere` **lazily, once**, and never invalidates it. The live
cloud rewrites its position attribute every frame, so the sphere that gets cached is the one from the
*first* render — before any `POINT_CLOUD` arrived, when the 300k-vertex buffer was still all zeros.
That is a sphere of **centre (0,0,0), radius 0**, and it never updates again.

World mode survived this purely by luck: its default camera sits at (0.5, 0, -1.5) looking at (0,0,1),
so the origin falls inside the frustum, the degenerate point-sphere "intersects", and the object is
drawn. It was a live tripwire the whole time — orbiting until the origin left the frustum would have
blanked the cloud.

**The FPV/Mirror view modes step straight on it.** They park the camera *at* the origin, which puts the
cached centre behind the near plane (`distanceToPoint = -0.02 < -radius = 0`), so the cull test fails
and the entire cloud disappears — a black viewport with no error, while the data was arriving and
correct all along. Confirmed by probing the first vertex's NDC in the render loop: `(0.53, -0.68, 0.95)`,
comfortably inside the frustum, with `frustumCulled=true` and `bs=(0,0,0)/r0`.

**Fix:** `frustumCulled = false` on the three live-updating objects (`points`, `uncovPoints`,
`surfaceMesh`). Recomputing the sphere per frame is the alternative and is far more expensive — a
300k-vertex pass that ignores `drawRange` — for objects that are always in front of the viewer.

*Diagnostic lesson:* the symptom (black viewport) looked like a camera-placement bug, and the frame
algebra was re-derived twice before instrumenting. The decisive measurement was three numbers printed
from inside the render loop — projected NDC, `boundingSphere`, `frustumCulled` — which separated "the
geometry is in the wrong place" from "the geometry is fine and something refused to draw it".

---

## BUG-035 — TSDF block capacity sat below a real room sweep; the map silently stopped

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/tsdf.py` (`DEFAULT_BLOCK_COUNT`, `_check_saturation`)
- **Found by:** replaying the owner's first full room sweep
  (`captures/roomSweepFull20260730.bin`) through the sub-phase 6.G memory rig.

`TsdfMap` hard-coded a `block_count` of 40,000 and plumbed it nowhere — no `Mapper` kwarg, no `[slam]`
key — so it could not be raised without editing source. That 40,000 turned out to sit *below* what one
real room scan needs, and running right at it broke the scan.

**Measured on the owner's sweep** (3525 depth frames, 1 cm voxels, CUDA:0):

| `block_count` | peak blocks | tracking lost | fitness at frame 3500 |
|---|---|---|---|
| 40,000 (old default) | 38,937 — **saturated** at 97.3% | **560 / 3525** | 0.10 |
| 120,000 | **42,917** (true demand) | **11 / 3525** | 1.00 |

The scan needs 42,917 blocks — **7% more than the old default**. It overshot by a hair, and the failure
was not graceful:

- blocks stop growing at frame **2879** (38,937, i.e. 97.3% of capacity) and never move again;
- tracking-lost begins at frame **2909** — **30 frames later**. Because SLAM is frame-to-model, once
  the map stops gaining geometry, ICP has nothing fresh to register the next frame against;
- median ICP fitness **0.887 → 0.127**; **0** lost frames before saturation, **560** after;
- **646 frames — 18% of the scan** — produce a ruined trajectory and no new map;
- Open3D also emits `stdgpu::vector::size : Size out of bounds: -2 not in [0, 18275]. Clamping to 0`,
  i.e. internal state going bad rather than a clean "full" signal.

Nothing logged. The scan simply stopped mapping, and it was only visible here because the 6.G rig
records the active block count next to the memory every frame.

**⚠ The mechanism is NOT "the grid cannot grow" — that was wrong and is corrected here.** An earlier
version of this entry said the `VoxelBlockGrid` pre-allocates and never grows. It does grow: driving a
CUDA grid across the boundary shows a clean rehash **40,000 → 80,000 at 99.2% load**, continuing to
52,027 blocks with no errors, identically on CPU and CUDA. The real 40,000 run froze at **97.3%** —
just *below* that rehash trigger.

Best current hypothesis, **unproven**: insertion failures in the ~97–99% load band beneath the rehash
threshold, which the `stdgpu` underflow message is consistent with. The competing explanation — that
ICP degraded first and the frozen block count is a symptom rather than a cause — is not excluded by
frame ordering alone, though it does not explain why 3× the capacity fixes it. What *is* established
is the effect and the mitigation: 560 lost frames at 40,000 vs 11 at both 120,000 and 160,000, on the
same capture, measured twice.

**Fix:** `DEFAULT_BLOCK_COUNT = 160000` (~3.7× this scan, ~2.3 GiB of pre-allocated device memory at a
measured ~14.2 KiB/block — 1707 MiB at 120,000), plumbed through `Mapper(block_count=…)`,
`[slam] block_count`, the `roomscan-slam` CLI, `web.SlamRunner`, and `slam_gpu_memory.py --block-count`.
Plus `TsdfMap._check_saturation()`, which warns **once** at 90% of capacity naming the config key — the
defect was never really "40,000 is too small" (any fixed number can be), it was that crossing the limit
was invisible.

**Verified on the shipped defaults**, same capture: **11 lost frames of 3525**, fitness 1.00 at the end,
42,917/160,000 blocks (27% of capacity), memory flat at 0.013 MiB/frame. Step latency p50 **7.9** ms /
p90 9.8 / p99 11.9 — *better* than the 40,000 run (8.7 / 11.6 / 14.8), so the larger pre-allocation is
free per-frame. The saturation check is one hashmap size read, measured at **5.8 µs/call** (0.02 s
across the whole scan) and early-outs permanently once fired. (An earlier run of this same
configuration showed p99 22.1 ms; that was a concurrent CPU job on the box, not the change — re-measured
clean.)

**Note it was 6.G that exposed this.** Before the CUDA-cache fix (BUG-032) this scan OOM'd around frame
900, long before capacity mattered. Removing the memory ceiling moved the wall to the next constraint.

**Bigger scans:** the grid is device-homogeneous — Open3D offers no managed/unified memory, so blocks
live wherever the grid does; you cannot hold them in host RAM while integrating on the GPU. A CUDA grid
is capped by VRAM (~460k blocks in the ~6.5 GiB usable on the 8 GiB card, ~10× this scan); a CPU grid
(`[slam] device = "CPU:0"`) is bounded only by system RAM. **Measured on this same sweep:** the CPU grid
completed it with **0 lost frames of 3525** at 46,037 blocks and zero VRAM. (It needs ~7% more blocks
than CUDA — 46,037 vs 42,917 — from float-rounding differences in the transform, not a defect.) The
per-step cost of that is the ~2.1× CPU/GPU ratio from the CUDA at-scale validation (CPU 18.94 ms vs GPU
8.85 ms median), which still fits the ~28 fps sensor ceiling; this run did not time steps separately.
Extraction already crosses the device line — `_extract_vbg()` copies to CPU because CUDA marching cubes
OOMs at scale.

---

## BUG-036 — One bad frame is terminal: a single fixed ICP correspondence radius

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/odometry.py` (`register_escalating`),
  `slam/mapper.py` (`icp_retry_dist`, `icp_escalations`, `lost_flags`),
  `slam/metrics.py` (`tracking_stats`), `slam/config.py` (`icp_retry_dist`)
- **Found by:** checking loop closure on the owner's two room circuits
  (`captures/coffeeRoomCircuitMnt.bin`, `captures/coffeeRoomCircuitNoMnt.bin`).

`max_dist` was one fixed 0.05 m serving two jobs it cannot both do. A frame whose frame-to-model
residual exceeds it finds **zero** correspondences; `predict_pose` then freezes translation at
`t_prev` and nothing relocalizes, so the frozen pose drifts further from truth and every later
raycast fails too. One frame kills the rest of the scan.

**Measured on `coffeeRoomCircuitMnt.bin`** (1889 depth frames, 62.4 s, 1 cm voxels, CUDA:0):

- tracking died at frame **1466** (t+48.4 s) in **one unbroken 423-frame run to the end** — 22% of
  the capture;
- ICP fitness was **0.919 in the 4 s before it died**. There is no degradation to watch for; it
  falls off a cliff. Fitness reads 0.086 for ~130 frames (ICP running, failing the gate), then
  **exactly 0.000** to the end (the raycast finds no map at all);
- depth was healthy throughout — post-gate valid count never below 1821 of 2268 — so this is not an
  empty-frame problem;
- it still **reported a plausible 2.05 m "drift"**. That number is not a measurement; it is where
  the estimate stood when it died, held for the last 22% of the run. Nothing logged.

**The fix — escalate only on failure.** Keep 0.05 as the first attempt; retry a *failed* frame once
at `icp_retry_dist` (0.10). A wider radius is not free, so it must not apply to healthy frames:

| capture | config | lost | start-end gap | path | escalations |
|---|---|---|---|---|---|
| Mnt | 0.05 fixed (old) | **423** | 2.047 m (7.48%) | 27.37 m | — |
| Mnt | 0.05 → 0.10 retry | **0** | 1.521 m (4.80%) | 31.68 m | **1** |
| NoMnt | 0.05 fixed (old) | 0 | 0.150 m (0.46%) | 32.50 m | — |
| NoMnt | 0.05 → 0.10 retry | 0 | **0.150 m** (bit-identical) | 32.50 m | **0** |
| NoMnt | 0.10 fixed | 0 | 0.953 m (2.63%) | 36.25 m | — |

**One retry, in one frame out of 1889, recovered the whole second half of the scan**, and the clean
run is bit-identical (`0.1501406473934001` / `32.50240193119565` before and after) because it never
escalates. Cost: +0.4 ms p50, nowhere near the ~35 ms budget. A third rung at 0.20 never fired, so
only one retry is implemented. `icp_retry_dist = 0` restores the old single-attempt path exactly.

Rejected with data, on the same two captures: a **fixed** 0.10 (rescues Mnt but degrades NoMnt
0.150 → 0.953 m), **6-DoF** (518 lost, first at frame 535 — much worse), and a looser `min_fitness`
(identical to the radius change alone: the gate is not what fails, correspondences are).

**Also fixed here: the silence.** `Mapper.lost_flags` now records the per-frame flag and
`metrics.tracking_stats` reports `trailing_lost` / `longest_lost_run` / `died`. A count alone hides
this failure — 423 lost of 1889 reads like "78% tracked fine" when in fact the run ended and the
tail is fabricated. `roomscan-slam` now prints `<-- THE RUN DIED` when a run ends in a sustained
lost streak, and the `--json` report carries the same under `tracking`.

**Not fixed, and not claimed:** Mnt still closes at 4.80% versus NoMnt's 0.46%. The retry makes that
run *complete*, not *accurate*. The remaining error is partly BUG-037 (barometer). Its horizontal
residual is unexplained — the mount is **not** in the FOV (no body-fixed occluder: zero pixels with
median < 700 mm, IQR < 60 mm, seen > 90% in either capture) and the motion profile does not
discriminate (NoMnt had *more* close-and-fast exposure — 4.5% vs 3.5% of frames, longest run 37
frames at 337 mm / 75 °/s vs 28 frames at 383 mm / 64 °/s — and never lost tracking). No mechanism
is asserted for it.

**Still open architecturally:** the retry makes a single bad *frame* survivable; it does not make a
bad *second* survivable. There is still no relocalization — a lost pose can never re-find the map.

---

## BUG-037 — Height is slaved to a barometer that wanders metres per minute

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 · **Area:** host/slam
- **Where:** `host/src/roomscan/slam/mapper.py` (`_apply_baro_z`), `slam/config.py` (`baro_weight`)
- **Found by:** scoring the owner's two room circuits against a ceiling-bookend ground truth
  (see "How this was measured" below).

`_apply_baro_z` blends the pose's height toward the barometric height every frame at
`baro_weight = 0.05` — a ~20-frame (0.66 s) time constant. Height is therefore effectively *slaved*
to the barometer. Over these two ~1-minute captures the sensor wandered **2.8 m and 2.6 m of
apparent altitude** (33.7 Pa and 31.2 Pa), with **43 cm and 12 cm of net drift** start-to-end.

**Measured** (ladder fix applied in both rows, so this isolates the barometer):

| capture | `baro_weight` | height error | horizontal gap | reported path |
|---|---|---|---|---|
| Mnt | 0.05 (default) | **−581 mm** | 1.406 m | 31.68 m |
| Mnt | 0 (off) | **−21 mm** | 0.941 m | 20.53 m |
| NoMnt | 0.05 (default) | −7 mm | **0.150 m** | 32.50 m |
| NoMnt | 0 (off) | −22 mm | 0.462 m | 21.73 m |

Two separate problems:

1. **Height error.** True height error is ~0 for both runs (ground truth below). The mounted run is
   off by **581 mm** with the barometer on and **21 mm** with it off — a 28× improvement. The run
   with 43 cm of barometric drift is exactly the one with the large height error.
2. **Invented path length.** Turning the barometer off cuts reported path by ~35% in *both* runs
   (31.68 → 20.53 m, 32.50 → 21.73 m). The operator walked a level circuit; roughly **10 m of
   "path" is vertical motion the barometer invented**. This inflates every `%-of-path` drift figure
   in the SLAM reports, i.e. it has been flattering our numbers.

**It is not a simple flag flip.** NoMnt's *horizontal* closure gets worse with the barometer off
(0.150 → 0.462 m), and `baro_weight = 0.01` was worse than both ends on the mounted run (−450 mm),
so the response is not monotonic. This needs a proper sweep against ground truth, not a default
change. Do not "fix" it by setting `baro_weight = 0` on this evidence.

**How this was measured — ceiling bookends.** The owner's captures start and end with the device
parked facing the ceiling (elevation 80.1°/80.2° and 89.2°/89.2°) at an identical measured range
(1420/1420 mm and 1453/1452 mm). Same elevation + same range to a flat ceiling ⇒ **same height**, so
the true height error over the loop is ~0 with no external instrumentation. That independently
corroborates the clean run (SLAM says 7 mm) and condemns the mounted one.

⚠ **This is an opportunistic check, not a protocol.** Do not build tooling that *requires* a parked
bookend: the owner's objection (2026-07-30) is that reaching the table puts the operator in the
sensor's FOV. Score it when a capture happens to have stationary bookends; never demand one.

---

### Resolution (2026-07-30)

The table above reproduces exactly on CUDA:0 (all twelve numbers), and the bookends re-verify
independently: elevation matches to 0.17°/0.04° and range to 0.1 mm/0.6 mm with the device
stationary at both ends, so the true height change over each loop is **<1 mm**. What the table does
*not* say is that most of its columns are chaos — see "the measurement was the second defect" below.

#### What the defect actually was

Not "over-trusting a drifting barometer" — that was the visible half. Decomposing the pressure trace
(NoMnt: total σ 4.32 Pa) splits it into **σ 3.12 Pa of frame-to-frame white noise** and only ~3.0 Pa
of slow wander. In apparent altitude that is:

| band | magnitude | vs ICP |
|---|---|---|
| white noise, per frame | **267 mm RMS** (380 mm frame-to-frame, max 1.9 m) | — |
| slow wander, per minute | ~0.4–0.5 m (0.43 m net on Mnt) | ICP's own vertical drift ~20–100 mm/min |

Three distinct faults followed from feeding that raw into a fixed-gain blend:

1. **Unfiltered white noise went straight into the pose.** At a 0.66 s blend the pose absorbs ~5% of
   a 267 mm-RMS signal every frame ⇒ **~12 mm of vertical step per frame**, which is precisely what
   was measured: 11.5 / 11.8 / 15.1 mm per frame across the three captures. That is the whole of the
   "invented path" — it scales with *frame count*, not with distance walked.
2. **A blend gives the barometer DC authority 1.0.** Below its corner frequency the pose *becomes*
   the barometer, in exactly the band where the barometer is ~20× the worse instrument. There was no
   notion that the two estimates have different uncertainties.
3. **The datum was one sample.** `_ref_pa` froze on the first pressure reading — a single draw from
   that 267 mm distribution, baked in as a constant altitude offset for the whole run at gain 1.

#### The measurement was the second defect

Most of the original table cannot support the weight it was given. Re-running the *identical*
configuration under numerically innocuous perturbations (CPU vs CUDA float ordering, starting one
frame later, `max_dist` 0.05 → 0.0501) moves the answers by more than the effect being measured. A
controlled test settles it: a deliberate **3 mm** one-shot height nudge, barometer otherwise off,
moves the final height error by **146 mm** and the loop closure by **0.37 m**.

So: **single-run SLAM comparisons below ~0.3 m of closure or ~0.2 m of height are realization noise.**
The specific claim that blocked the obvious fix — "NoMnt's horizontal closure gets worse with the
barometer off, 0.150 → 0.462 m" — is not real. Across a 10-member ensemble NoMnt's closure is
0.68 ± 0.36 m with the old blend and 0.61 ± 0.25 m with it off; 0.150 was simply the luckiest member.
Everything below is an ensemble mean ± sd over 10 perturbations (effectively 9 — `weight_threshold`
+1e-4 turned out to be a no-op and duplicates the base run).

#### The fix

`baro_weight` is retired. The constraint is now a **low-passed, bounded-authority complementary
correction** whose whole lifetime contribution is exactly

```
baro_correction = baro_authority · LPF_tau(h_baro − h_icp)
```

with both parameters *measured quantities* rather than a magic gain:

- **`baro_tau_frames = 900`** (~30 s at 30 fps) — the noise filter. 267 mm RMS through a 1/900 EMA
  leaves ~6 mm. Answers fault (1).
- **`baro_authority = 0.05`** — the least-squares share of two drifting estimates,
  q_icp²/(q_icp² + q_baro²) ≈ 0.09²/(0.09² + 0.45²) ≈ 0.04, rounded up. Answers fault (2). *That this
  equals the old `baro_weight`'s numeral is a coincidence of arithmetic, not of meaning* — the old
  gain's DC authority was 1.0.
- The datum is the mean of the first 90 frames (`_BARO_REF_FRAMES`), and the filter opens at 0 rather
  than at the first innovation. Answers fault (3).
- New `Mapper.baro_correction_m` reports how much height came from the barometer; `roomscan-slam`
  prints it and puts it in `--json`, and `slam_rerender` surfaces it.

Plumbed the same way `block_count` was: `Mapper` / `[slam]` / CLI (`--baro-authority`, added so the
default can be re-measured) / `slam/service.py` / `web.SlamRunner` / `slam_gpu_memory.py`. A config
still carrying `baro_weight` loads fine — unknown keys are ignored — and is *not* reinterpreted.

#### Before / after (ensemble means ± sd, n = 10)

`roomSweepFull20260730.bin` is the independent check: nothing was tuned on it, and it has **no
bookend** (it ends mid-sweep, elevation 26.8° → 8.7°, still moving) so its height column is a
*reported height change*, not an error, and its closure is not a loop closure. It is in the table for
the path column only.

| capture | config | height error | horizontal closure | reported path | of which vertical |
|---|---|---|---|---|---|
| Mnt | old `baro_weight = 0.05` | 458 ± 232 mm | 1.286 ± 0.249 m | 31.52 m | 21.72 m (69%) |
| Mnt | **new default** | **102 ± 113 mm** | **0.912 ± 0.310 m** | **20.88 m** | **6.75 m (32%)** |
| Mnt | authority 0 (off) | 67 ± 57 mm | 0.912 ± 0.192 m | 20.77 m | 6.70 m |
| NoMnt | old `baro_weight = 0.05` | 125 ± 138 mm | 0.683 ± 0.364 m | 33.74 m | 23.27 m (69%) |
| NoMnt | **new default** | **125 ± 76 mm** | **0.738 ± 0.187 m** | **23.89 m** | **7.41 m (31%)** |
| NoMnt | authority 0 (off) | 204 ± 280 mm | 0.608 ± 0.252 m | 22.23 m | 7.29 m |
| Sweep † | old `baro_weight = 0.05` | (418 ± 202 mm) | (0.812 m) | 72.20 m | 53.25 m (74%) |
| Sweep † | **new default** | (963 ± 503 mm) | (2.550 m) | **45.15 m** | **19.31 m (43%)** |
| Sweep † | authority 0 (off) | (520 ± 170 mm) | (2.918 m) | 44.40 m | 19.38 m |

† not a closed loop and no bookend — parenthesised columns have no ground truth.

**What improved, robustly:** the invented path is gone. Reported path drops **34% / 29% / 37%** and
the vertical share drops from ~70% to ~32%, on all three captures including the one it was not tuned
on. The whole-run barometric correction is now **9 / 10 / 27 mm** instead of hundreds.

**Height error:** the old blend's 458 mm on the mounted run (the one whose barometer drifted 43 cm)
is gone — 102 mm, a 4.5× improvement, and the *spread* halves. Note the original entry's headline
"−581 mm vs −21 mm, 28×" overstated it: those were single members of ensembles whose means are
458 mm and 67 mm.

**Honest negative:** on this evidence the fixed constraint is **not distinguishable from switching
the barometer off**. It is better on NoMnt (125 vs 204 mm) and worse on Mnt (102 vs 67 mm), both
inside the chaos band, and its total contribution is ~10 mm — arithmetically far too small to explain
either difference. With *this* barometer, on a 1-minute scan, the correct authority is small enough
to be worth nothing. It is kept in this shape because the parameters are now measurable: a quieter
barometer (the firmware writes LPS22DF `CTRL_REG1 = 0x20`, which by that register map is minimum
averaging — untested hypothesis for the 3.1 Pa, which is measured) or a scan long enough for ICP's
drift to overtake the barometer's would make it earn its place by moving a number, not an argument.

**Consequence for every `%-of-path` figure.** They were all divided by an inflated denominator. The
headline "frame-to-model closes a room to **0.46 %**" was a lucky single run (0.150 m) over an
inflated path (32.50 m). Honestly: **0.74 ± 0.19 m over 23.9 m ≈ 3 %** on NoMnt and
**0.91 ± 0.31 m over 20.9 m ≈ 4.4 %** on Mnt. Still decent absolute closure for a 54×42 imager with
no loop closure, but ~5× the number 6.D's "does loop closure earn its complexity" argument was
resting on. `ROADMAP.md` and `CLAUDE.md` are corrected.

**Not fixed / still open:** the barometer's slow wander itself (environmental + sensor, not a
software defect); the LPS22DF averaging configuration in `rs_lsm.c`, which is the only lever that
would make the constraint useful — deliberately *not* touched here, the rig was live and streaming;
and ICP's own vertical jitter, which is what the remaining ~19 m of vertical path on the sweep is.

---

## BUG-039 — Host IMU fusion measures heading error about the wrong axis (near gimbal lock)

- **Status:** **fixed** 2026-07-30 · **Reported:** 2026-07-30 (found while costing ODR-triggered
  sync, BUG-031) · **Area:** host/sensors
- **Where:** `host/src/roomscan/imufusion.py` (`_correct_yaw`),
  `host/src/roomscan/sensors.py` (`graft_yaw_error_deg`, new). The filter remains **gated off** —
  see the warning at the end of this entry.
- **Found by:** `docs/odr-triggered-sync-costing-2026-07-30.md` §1

`_correct_yaw` measured its heading error with `quat_yaw_deg` — ZYX yaw, i.e. rotation about **body
Z** — but this device's SFLP body frame has **X = Up**. On the stationary capture the ZYX pitch is
86.2°, which is **3.76° from gimbal lock**, so the quantity the loop was nulling is not heading.

Measured heading error vs SFLP: **1.689° mean / 2.217° p95**. Shortening `tau_yaw` to 0.3 s barely
moved it (1.703°) — the signature of a *wrong measurement*, not a mistuned gain.

**The fix.** New `sensors.graft_yaw_error_deg(target, quat)` — the world-Z **swing-twist** of the
residual `target ⊗ quat*`, i.e. `2·atan2(rel_z, rel_w)`. That is the exact inverse of `graft_yaw`
(the loop's own actuator), so the loop now nulls precisely the quantity it can correct, and it is the
*optimal* pure-heading correction rather than merely a different one. It has no singularity at any
attitude, so unlike `YawFusion` — which defends against the same ZYX degeneracy with a
`gimbal_margin_deg = 15°` gate that would have gated this whole capture out — it needs no gate.
`_correct_yaw` is a one-line change on top.

**Measured, before → after, world-Z heading error vs SFLP, same bytes both ways** (`tau_yaw` 1.0 s;
the whole ensemble, not one capture — single-run metrics in this repo have proven chaotic):

| capture | ZYX pitch p95 | before mean/p95 | after mean/p95 |
|---|---|---|---|
| `stationary_stream11_20260728_190311` (3000 fr) | 86.2° | 1.689 / 2.218 | **0.017 / 0.053** |
| `stream11_verify` (304 fr) | 80.8° | 0.014 / 0.037 | **0.006 / 0.016** |
| `coffeeRoomCircuitNoMnt` (1988 fr) | 85.0° | 0.994 / 2.826 | **0.573 / 1.550** |
| `coffeeRoomCircuitMnt` (1928 fr) | 74.6° | 0.775 / 2.120 | **0.549 / 1.538** |
| `roomSweepFull20260730` (3000 fr) | 77.3° | 0.573 / 1.430 | **0.509 / 1.266** |
| `tilt_sweep_20260729` (3000 fr) | −0.1° | 0.011 / 0.026 | 0.011 / 0.026 |
| `web_20260730_181252` (2780 fr) | −0.2° | 0.003 / 0.015 | 0.003 / 0.015 |

The two **zero-pitch captures are bit-identical** before and after. That is the control, and it is
the point: the misreading scales as `tilt × tan(pitch)`, so it is 0.04° at level and ~7° at 86°.
This is a frame error at the attitudes this device actually flies, not a general retune.

**Tests that would have caught it** (`test_imu_fusion.py::test_yaw_loop_measures_heading_about_
world_z_not_body_z`, plus four in `test_sensors.py`). Verified by re-running the suite with the old
term monkeypatched back in globally: the axis test fails at 0.138° against a `< 0.01°` bound, and
nothing else in 1079 tests notices. The fixture holds the device still 4° from gimbal lock and lets
the reference's *static* fp16 tilt quantisation be the only error; the expected legacy failure is
**derived from the fixture** (it converges to exactly the ZYX-yaw misreading of that quantisation),
not hard-coded. A companion test records the trap that let this ship: on a residual that is *pure
heading*, both conventions agree exactly even at gimbal lock — so the obvious test, a clean 30°
heading offset, passes either way and proves nothing about the axis.

**Why it matters beyond the filter itself.** `imufusion` beating SFLP is the stated precondition for
trading SFLP away for hardware ODR-triggered sync (BUG-031). While this defect stands, that
precondition is untested, not merely unmet — the comparison that would decide it is measuring the
wrong thing.

⚠ **Do not treat "fixed this ⇒ imufusion beats SFLP" as established.** The only shipped evidence for
the filter is synthetic (`test_imu_fusion.py` is all-synthetic by its own docstring), the within-capture
win is measured at **one attitude, stationary**, and under motion the accel-referenced metric
**saturates** (SFLP 0.879°, imufusion 0.864°, SFLP-gravity 0.868° — all within 2%). **No capture in
the repo can adjudicate the two under motion**, and there is no orientation ground truth anywhere in
the repo. Fixing the axis makes the comparison *meaningful*; it does not make it *decided*.

**The gate is therefore deliberately unchanged** (2026-07-30): `SensorState(imu_fusion=None)` is
still the default and nothing in the repo constructs an `ImuFusion` outside its own module and tests
— verified by grep, and by the unchanged `test_slam_non_regression_*` guards. Note also what the
numbers above are and are not: they are *agreement with SFLP*, not accuracy. The table says the
filter's heading now follows its own anchor instead of fighting it; it says nothing about whether
either estimator points the right way. **Heading direction remains unvalidated repo-wide** (resume
doc §4.6) and nothing here touches it.

---

## BUG-040 — Web UI cannot see transport loss (Drops/Gaps always 0)

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (owner, while diagnosing a SLAM run) · **Area:** host/web
- **Where:** `host/src/roomscan/web.py` `_broadcaster` / `build_metrics_message`

`MetricsRegistry` measures rates and bandwidth; it has no notion of frame *sequencing*, so
`MetricsSnapshot.drops`/`.gaps` keep their dataclass default of `0`. The desktop panel filled them
in (`panel.py:2978`, `replace(snap, drops=..., gaps=...)` off the reader's `Stats`), but the web
broadcaster never did — it passed the raw snapshot straight to `build_metrics_message`. The reader
*was* maintaining the counters correctly the whole time (`reader.py:121` calls `stats.update`);
only the hand-off was missing.

Why it matters more than a cosmetic zero: the Ethernet transport splits a 14,878-byte depth frame
into 11 UDP datagrams, and the host's reassembly (`sources.py`) requires every fragment of a `seq`
**in order** — one lost or reordered datagram silently discards the whole frame. The discarded
frame never reaches the decoder, so the loss surfaces *only* as a header sequence gap. With that
row pinned at 0, `roomscan-web` — the primary, supported UI — had no way to show packet loss.

**Fix:** merge the reader's `Stats` into the snapshot in the broadcaster, guarded with `getattr`
so a partially-built `app.state` (tests, replay harnesses) still broadcasts.

**Test:** `test_metrics_broadcast_reports_reader_drops_and_gaps` drives a real uvicorn server over
a websocket and asserts the counters arrive; removing the `replace(...)` fails it with `0 != 7`.

---

## BUG-041 — UDP fragments burst into an 8-deep TX ring, and a failed send abandoned the frame

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (owner: "is UDP the right transport?") · **Area:** firmware/eth
- **Where:** `firmware/scanner-stream/Src/ethernet_transport.c` `ETH_SendFrame_Gather`

Two defects in one loop:

1. **Burst.** A depth frame is `RS_HEADER_SIZE + 14842 + CRC` = 14,878 B → 11 datagrams, pushed
   back-to-back with no pacing. `ETH_TX_DESC_CNT` is **8** (`stm32h5xx_hal_conf.h:186`), and at
   250 MHz the CPU enqueues a 1400 B memcpy far faster than the DMA drains a descriptor (~112 µs
   each at 100 Mbit), so the ring can be overrun by the frame's own fragments. Through a Wi-Fi
   bridge the same burst shape is what overflows a cheap AP's queue.
2. **Mid-frame abandon.** On `pbuf_alloc` returning NULL or `udp_sendto` returning non-`ERR_OK`,
   the old code `return false`d immediately — leaving fragments 0..k-1 already on the wire. Those
   are guaranteed waste: the host needs every fragment of a `seq`, so a partial frame is a lost
   frame *plus* wasted bandwidth.

Loss here is amplified ~11× (one datagram kills a 14.8 KB frame) and is not benign for SLAM: a
dropped depth frame doubles the inter-frame motion ICP has to solve, which is the same failure
mode as BUG-036 (a 52 mm step against a 0.05 m correspondence radius killed 54% of a run).

**Fix:** frames are copied into a fixed slot FIFO (8 × 15,104 B) and their fragments metered out
from `ETH_Process()`. Two invariants: frames drain **strictly in order, one at a time** (the host
resets its reassembly buffer the instant a different `seq` arrives, so interleaving would destroy
both frames), and a refused send **retries** rather than abandoning. Back-pressure on a full FIFO
drains synchronously rather than dropping, because the caller's raw double-buffer is about to be
reused. The per-call fragment budget is adaptive — sized from the outstanding count so the queue
clears inside `ETH_TX_WINDOW_MS` (25 ms) regardless of how coarsely `ETH_Process` is called
(`platform_wait_for_event` busy-waits in 5 ms slices, so a fixed inter-fragment gap would
under-drain and the backlog would grow without bound).

Cost: `bss` 54,256 → 175,200 B (184 KB static of 640 KB; 456 KB still free), `text` +424 B.

**Verified on-rig (2026-07-31):** flashed and soaked 90 s — 15,909 frames, **0 CRC failures, 0
bytes skipped, no anomalies**, device rate 30.303 Hz (identical to the pre-flash baseline),
0 drops / 0 gaps.

**Not yet demonstrated:** the loss *reduction*. The rig was on a direct cable for this test (the
RavPower FileHub was powered on but not bridged), so there was no loss to remove — the soak proves
no regression, not benefit. Re-measure with the Wi-Fi bridge in path, now that BUG-040 makes the
Gaps row readable.

---

## BUG-042 — A reordered UDP datagram destroyed a whole frame, silently

- **Status:** **fixed** 2026-07-31 · **Reported:** 2026-07-31 (found while costing transport compression) · **Area:** host/sources
- **Where:** `host/src/roomscan/sources.py` `UdpSource.read`

A depth frame is 11 UDP datagrams. Reassembly accepted a fragment only when
`frag_idx == self._expected_frag` and then *appended* it, so position was implied by arrival order.
UDP guarantees neither ordering nor delivery — a reordered pair therefore discarded the entire frame
just as surely as a lost datagram would, and nothing counted it. The frame never reached the decoder,
so the only downstream trace was a header sequence gap, which BUG-040 had left unwired in the web UI.
Net effect: the two most likely transport faults were both invisible, and one of them was
self-inflicted.

**Fix:** reassemble into indexed slots — fragment *k* always lands at index *k*, and the frame
completes when the last hole fills, in any arrival order. Joining slots `0..n-1` reconstructs the
frame exactly because the sender chunks at a fixed 1400 B with only the tail short, so the index
alone determines position. Added `frames_incomplete` / `frags_lost` (real loss),
`frags_reordered` (recovered, previously fatal), `frags_duplicate` and `frags_invalid`, which
separate causes that used to be indistinguishable. A fragment whose `total_frags` disagrees with the
rest of its seq is rejected rather than resizing the frame mid-flight.

Safe because the firmware's paced TX (BUG-041) drains frames strictly in order, one at a time, so
fragments of two seqs never interleave — a single in-flight frame buffer is sufficient.

**Tests:** `test_reassembly_*` in `host/tests/test_sources.py`. Reintroducing the strict-order check
fails `accepts_out_of_order_fragments` and `counts_a_genuinely_lost_fragment`.

**Related, same commit — CRC32 made table-driven.** `rs_crc32` (`firmware/scanner-stream/Src/rs_protocol.c`)
was table-free bit-serial: 8 shift/xor iterations per byte over header+payload for every frame
(~119,000 inner iterations at 14,874 B). Replaced with a 16-entry nibble table (64 B). Measured
**2.21× faster** on x86 -O2 (125.2 → 56.7 µs/frame) — note this is *below* the ~5× an earlier
estimate assumed, so the on-target saving is likely ~1.2–1.8 ms/frame, not 2.2–3.2. Bit-exactness
verified natively over 3,857 cases (all single bytes, all lengths 0–600, 3,000 full-frame trials
using the firmware's incremental `rs_crc32(rs_crc32(0,hdr,32), payload, …)` seeding, plus the
standard `CRC32("123456789") == 0xCBF43926` vector), then confirmed on-target: 13,434 frames
streamed with **0 CRC failures** — a single divergent bit would have failed every frame.

The loop is sensor-rate-limited (30.3 Hz) and spends ~20 ms of each 33 ms period in
`platform_wait_for_event`'s spin, so this buys **headroom, not throughput** — there is no fps change
to observe. It is worth having because it is the enabling margin for on-MCU compression, where
compressing before the CRC also shrinks what the CRC must cover.
