# Panel UI redesign — modes, floating HUD, menu-driven settings

**Date:** 2026-07-13
**Status:** approved (brainstorm), pending implementation plan
**Branch:** feature/phase6-slam
**Owner:** hellosamblack

## 1. Overview

Restructure the `roomscan-panel` GUI (`host/src/roomscan/panel.py`) from a
sidebar-driven, multi-mode window into a **two-mode, first-person, HUD-driven**
instrument:

- Collapse **SLAM** and **Showcase** into a single **SLAM** mode.
- Two top-level view modes: **Real-Time** and **SLAM**.
- Both default to **first-person** (camera rides the sensor pose); a
  **First-person / Orbit** toggle frees the camera.
- Move the "clunky" sidebar controls into a **menubar → dialog(s)**.
- Float only the primary interactions in the 3D view, custom-drawn in the
  established instrument language (cyan accent, mono, corner ticks — `theme.py`,
  `cards.py`).
- Add a first-person **2D IR overlay** with an opacity slider.
- Fix the **camera-gizmo flicker** in first-person.

Reference layout (mock over a real scene screenshot):
`scratchpad/mock_ui.png` (session artifact).

## 2. Goals / non-goals

**Goals**
- One clear mode switch; no separate Showcase concept in the UI.
- First-person by default in both modes, gizmo hidden and not flickering.
- SLAM mode exposes **Record** and **Load** (Load handles both `.bin` and `.ply`).
- First-person IR overlay with adjustable opacity, in both modes.
- Settings reachable from a menubar, out of the 3D view.
- Primary controls float in-scene, custom-drawn (approach **B**).

**Non-goals (this spec)**
- No change to the SLAM/TSDF math, protocol, or firmware.
- No change to the reveal card / wavefront / stage work already shipped — they
  are reused as-is.
- No new telemetry design (metrics HUD stays as an optional overlay).
- The classic keyboard-only `roomscan-view` window is out of scope (untouched).

## 3. Mode & view-state model

Two orthogonal axes, both owned by the panel:

- **mode ∈ {REAL_TIME, SLAM}** — the segmented switch (top-center).
  - REAL_TIME: live deprojected cloud (today's raw view). No map, no recording.
  - SLAM: map building = the former SLAM view *and* Showcase flow, merged.
    Record → process → reveal is the Showcase pipeline under the hood.
- **camera ∈ {FIRST_PERSON, ORBIT}** — the view toggle (top-right). Default
  FIRST_PERSON in both modes.
  - In **SLAM**, FIRST_PERSON rides the live SLAM pose (`step.pose`) via the
    existing `follow_camera` path.
  - In **REAL_TIME** there is no world pose, so FIRST_PERSON is the fixed
    sensor-origin camera looking along the sensor's forward axis (+Z) — the raw
    cloud is already in the sensor frame, so this is the natural "what the
    sensor sees" view. ORBIT is today's turntable orbit.

**SLAM sub-state** (drives the action cluster + reveal), reusing the existing
`ShowcasePhase` state machine (`slam/showcase.py`): IDLE → RECORDING →
PROCESSING → FINAL. On FINAL the camera auto-switches to ORBIT for the reveal
(auto-orbit + reveal card, already built); the user can return to FIRST_PERSON.

**Load** (SLAM only): a file dialog.
- `.bin` → runs the existing capture→process→reveal pipeline on saved data.
- `.ply` → displays the mesh (orbit), no reprocessing.

Switching mode REAL_TIME→SLAM or back tears down the other mode's geometries
(reuse `_remove_live_view_geometries` / `_remove_slam_geometries`).

## 4. Architecture

`panel.py` is already ~2600 lines and does too much. Extract the new surfaces
into focused modules; `panel.py` becomes the orchestrator that owns threads,
state, and scene, and delegates presentation.

- **`hud.py` (new, mostly pure)** — renders each floating control to an RGBA
  image and owns the **hit-test model**. No Open3D imports; unit-tested like
  `theme.py`/`cards.py`.
  - `render_mode_switch(active) -> RGBA`, `render_view_toggle(active) -> RGBA`,
    `render_action_cluster(phase, is_replay) -> RGBA`,
    `render_ir_control(enabled, opacity) -> RGBA`,
    `render_status_chip(tracking, fps) -> RGBA`.
  - A `HudLayout` helper: given the scene rect, returns each control's window
    rect; and `hit_test(x, y) -> ControlHit | None` mapping a click to a
    control + sub-region (e.g. which segment, slider fraction from x).
  - Controls are value-objects (rect + state); rendering and hit-testing are
    pure functions over them → fully unit-testable without a window.
- **`ir_overlay.py` (new, pure geometry)** — `camera_locked_quad(eye, forward,
  up, fov_h, fov_v, dist) -> (verts(4,3), uvs(4,2), tris(2,3))`: the billboard
  quad spanning the FoV at `dist` in front of the first-person eye. Pure,
  unit-tested. Panel uploads it with `defaultUnlitTransparency` + the live IR
  texture, `base_color[3] = opacity`.
- **`settings_dialog.py` (new)** — builds **one** settings dialog with the
  existing grouped controls (color, point size, near-contrast, surface, IR
  colormap/freeze, wall mode, device: usecase/exposure/ping/calib/reinit, replay
  fps) as collapsible sections. Reuses today's `_group`/`_labeled_grid` helpers
  (lifted out of `panel.py`). The **View** and **Device** menu entries both open
  this single dialog (optionally scrolled to their section); the old
  always-visible sidebar (`_build_panel`) is retired.
- **`panel.py` (orchestrator)** — owns reader thread, slot, SLAM/showcase
  workers, scene, mode/camera state; builds the menubar; positions HUD +
  overlay + IR quad; routes the scene mouse handler through `hud.hit_test`
  before camera nav.

## 5. Component detail

### 5.1 Floating HUD (approach B)
- Each control is a `Window`-child `gui.ImageWidget` positioned in `_on_layout`
  (same mechanism as the metrics overlay / reveal card), showing an
  `hud.render_*` image.
- **Interaction** goes through the existing `SceneWidget.set_on_mouse`
  (`_on_mouse`): at the top, call `HudLayout.hit_test(x, y)`; on a hit, perform
  the action and return `CONSUMED` (skip camera nav); otherwise fall through to
  orbit/pan/zoom.
- **Feasibility probe (plan task 0):** confirm the scene mouse handler still
  receives clicks over a floating `ImageWidget` (i.e. the image doesn't consume
  them). If it *does* consume, fallback: place a matching invisible/borderless
  `gui.Button` per control region as the click layer over each image. Decide
  before building the rest of the HUD.
- Controls & actions:
  - Mode switch → set mode REAL_TIME/SLAM.
  - View toggle → set camera FIRST_PERSON/ORBIT.
  - IR control → toggle overlay; slider fraction (from click/drag x) → opacity.
  - Action cluster (SLAM): Record (↔ Stop while RECORDING), Load, Clear.
  - Status chip: read-only (tracking + fps).

### 5.2 IR overlay
- Only in FIRST_PERSON. When enabled, each rendered frame:
  build `ir_overlay.camera_locked_quad(...)` from the current first-person
  eye/forward (SLAM: `step.pose`; Real-Time: the fixed sensor-origin camera),
  texture it with the latest IR image (`ir_image` / `reflectance_to_rgb`),
  material `defaultUnlitTransparency`, `base_color=[1,1,1,opacity]`. Re-add
  geometry per frame (2 triangles, cheap).
- Removed when overlay off, camera→ORBIT, or IR frame unavailable.

### 5.3 Camera-gizmo flicker fix
- Root cause: `_hide_first_person_clutter` removes the gizmo every render tick,
  while the ≤4 Hz `_update_camera_gizmo` re-adds it (`_gizmo_added` reset).
- Fix: gate `_update_camera_gizmo` on `camera == ORBIT and imu_gizmo`. In
  FIRST_PERSON the gizmo is never added, so nothing to hide/re-add → no flicker.

### 5.4 Menubar + settings
- `gui.Application.instance.menubar` with: **View**, **Device**, **Overlays**,
  **Help**.
  - View / Device → open the single `settings_dialog` (grouped sliders/combos —
    the old sidebar content), scrolled to the relevant section.
  - Overlays → checkable items: Metrics HUD, Sensors, Events (toggle the
    existing overlay widgets).
  - Help → the existing help dialog.
- Native menu items can't host sliders, so sliders/combos live in the dialog,
  not as menu items.

## 6. Data / control flow
- Reader thread → latest-wins slot (unchanged).
- UI tick: render frame per mode; update HUD images only when their state
  changes (avoid per-tick re-render); IR quad rebuilt per frame when active;
  status chip on the ≤4 Hz tick.
- Mouse: HUD hit-test → action; else camera nav.
- Config persistence (`--save-config`): add mode, camera, ir_overlay,
  ir_opacity; keep existing keys.

## 7. Testing
- **Pure/unit** (no window): `hud.render_*` (shape/accent-pixels/determinism),
  `HudLayout.hit_test` (region→action, slider x→fraction, misses),
  `ir_overlay.camera_locked_quad` (planarity, faces camera, FoV extent),
  mode/camera/phase transition helpers, gizmo-gating predicate.
- **Live smoke** (proven `Application.render_to_image` harness): construct panel,
  drive each mode, screenshot; assert geometries present/absent per state
  (mesh, floor grid, IR quad, gizmo) and no crash. Visual before/after captures
  for review.
- Update/retire tests tied to the old sidebar widgets.

## 8. Risks
- **Mouse fallthrough over ImageWidgets** (§5.1 probe) — highest risk; gates the
  HUD approach. Fallback defined.
- **IR texture-per-frame perf** — small image; re-add is cheap; verify fps holds
  on the live smoke.
- **panel.py churn** — mitigate by extracting `hud.py` / `ir_overlay.py` /
  `settings_dialog.py` and shrinking `panel.py` to orchestration.

## 9. Out of scope / future
- True integration-recency wavefront (still proximity-based).
- Real sun/shadow lighting.
- Restyling the metrics HUD / sensors widgets (telemetry is debug).
