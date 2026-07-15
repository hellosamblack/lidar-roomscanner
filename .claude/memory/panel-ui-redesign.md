---
name: panel-ui-redesign
description: "Panel UI redesign (two-mode/first-person/HUD) on feature/phase6-slam; HUD mouse-passthrough CONFIRMED+FIXED via ImageWidget.set_on_mouse + srgbColor spam fixed via fd-2 logfilter (561 tests); rest of GUI runtime still needs on-rig eyeball"
metadata: 
  node_type: memory
  type: project
  originSessionId: a82dc465-c6c2-474e-ba55-438e3e754c25
---

The `roomscan-panel` GUI was restructured from a sidebar-driven, multi-mode window into a
**two-mode (Real-Time / SLAM), first-person-by-default, HUD-driven** instrument. Spec:
`docs/superpowers/specs/2026-07-13-panel-ui-redesign-design.md`; plan:
`docs/superpowers/plans/2026-07-14-panel-ui-redesign.md` (subagent-driven, 13 tasks + 4 review fixes,
`feature/phase6-slam`, commits `d654f93..8e24f6b`, 554 host tests green headless).

**What shipped:** SLAM mode absorbs the former Showcase record→process→reveal (no separate Showcase
concept); First-person/Orbit toggle (first-person default both modes); sidebar retired → menubar + one
`settings_dialog.py`; floating in-scene HUD (mode switch / view toggle / action cluster / IR control /
status chip) — new pure unit-tested modules `instrument.py` (primitives shared with `cards.py`),
`hud.py` (renders + `HudLayout` hit-test), `ir_overlay.py` (first-person IR billboard quad); gizmo-
flicker fix (gizmo gated on orbit only); `mode`/`camera`/`ir_overlay`/`ir_opacity` config persistence.
The HUD mode-switch + view-toggle are the **sole** mode/camera authority (old SLAM/Showcase/Follow
checkboxes removed, owner decision).

**HUD mouse-passthrough — CONFIRMED on-rig + FIXED (2026-07-14):** owner reported HUD toggles
unclickable. Root cause: each HUD control is a `gui.ImageWidget` stacked over the `SceneWidget`; Open3D
routes a click to the topmost child under the cursor, so clicks on a control never reached
`scene_widget.set_on_mouse` → `hit_test` never ran. Fix was NOT the planned invisible-`gui.Button` layer:
`ImageWidget` has its **own** `set_on_mouse`, so each HUD widget now carries
`w.set_on_mouse(self._on_hud_widget_mouse)` which reuses the existing `HudLayout.hit_test` /
`_dispatch_hud_hit` unchanged (event coords are window-absolute, sliders/segments preserved). The dead +
latently-buggy HUD block in `_on_mouse` (dispatched to *hidden* controls) was removed. Also fixed the
srgbColor console spam: cosmetic Open3D-0.19 Filament warning (`defaultUnlitTransparency.filamat` lacks the
`srgbColor` uniform that `UpdateDefaultUnlit` binds; per-frame IR billboard re-add triggers it) — new
`host/src/roomscan/logfilter.py` interposes a pipe on C-level fd 2 and drops the two warning lines
(UCRT fd-2 capture verified end-to-end; opt out `ROOMSCAN_KEEP_FILAMENT_LOGS=1`). +7 tests, 561 green.
Still owner-supervised-run only for the on-screen click + live-window silencing confirmation.

**STILL OPEN — remaining GUI runtime UNVERIFIED-BY-RUNTIME** (Filament needs a display). Owner on-rig run:
- Smoke `host/tools/panel_ui_smoke.py` + manual eyeball; confirm toggles now respond + console quiet.
- Mode/camera switching + first-person cameras (both modes); IR billboard texture render + **UV
  orientation** + opacity slider; settings-dialog re-open widget lifetime (persistent `_settings_root`);
  config persistence across `--save-config`.

**Known Minors for the bench pass** (from the final whole-branch review): settings dialog is a plain
`gui.Vert` not `ScrollableVert` (9 sections can overflow a short window, hiding the Close row);
`_on_metrics_overlay` doesn't update the Overlays-menu checkmark (one-way sync); Real-Time ORBIT doesn't
reframe until first drag; brief first-reveal blank-flash of a HUD control.

Three final-review cross-task bugs were caught + fixed pre-bench: `camera` config field read the wrong
attribute (`self.camera` vs `self.camera_mode`) so ORBIT never persisted (fixed f08693b + regression);
`_hud_tracking` was read but never written so the status chip was stuck at "--" (fixed f08693b);
HUD-vs-dialog-checkbox mode duality (fixed 8e24f6b by removing the checkboxes).

Related: [[live-view-fps-rendering]], [[mapping-pipeline-plan]], [[status-sync-guardrails]].
