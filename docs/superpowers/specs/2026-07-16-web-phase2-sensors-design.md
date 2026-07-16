# Web Phase 2 — Sensors (IMU/env streams 9/10)

**Date:** 2026-07-16
**Status:** implementing
**Branch:** `main` (commit-to-main workflow)
**Owner:** hellosamblack
**Predecessor:** Web Phase 1 (`2026-07-15-web-phase1-core-instrument-design.md`) — ✅ done

## 1. Overview

Phase 2 brings the desktop panel's **Sensors** group to the web app: the
orientation gizmo, tilt-compensated compass, and pressure/temperature
sparklines fed by wire streams **9 (IMU_QUAT)** and **10 (ENV)**. Like Phase 1
this is **host-side only** — no wire-protocol or firmware change — and it lands
on the extension points Phase 1 deliberately built: a new `sensor` JSON message
on the multiplexed `/ws`, a new periodic broadcast tick, and one new frontend
ES module (`sensors.js`) wired through the `ws.js` hub.

The load-bearing sensor **math is reused, not reimplemented**: `web.py`
constructs the same `SensorState` + `YawFusion` + `MagCalibration` the panel
builds (`panel.py:525-541`), feeds it from the shared reader via the
`_run_reader(state=...)` slot Phase 1 left as `None`, and the message builder
calls the existing `sensors.py` helpers (`quat_to_matrix`, `T_WORLD_TO_CV`,
`T_CV_TO_BODY`, `absolute_heading`, `AXIS_CONVENTION`). Nothing in `sensors.py`,
`magcal.py`, `protocol.py`, or `panel.py` is edited.

## 2. Goals

- Feed streams 9/10 into the web reader (currently discarded: `state=None`).
- A live orientation gizmo driven by the SFLP quaternion, oriented into the
  same Open3D-CV display frame the point cloud uses.
- A tilt-compensated magnetic compass (drift-free `absolute_heading`, calibrated
  mag when `mag_cal.json` is present) matching the desktop's 0°=N, clockwise dial.
- Pressure and temperature sparklines with a 256-sample history and live values.
- Yaw-fusion status surfaced to the UI.
- Graceful no-data: a ToF-only session (no IKS4A1) shows the section idle with
  placeholders and emits **no** `sensor` traffic until a 9/10 frame arrives.

## 3. Non-goals (this phase)

- World-frame point-cloud accumulation / baseline-yaw reset (the desktop's
  `graft_yaw(-baseline)` map) — the point cloud stays raw per-frame as in
  Phase 1; revisit with SLAM (Web Phase 4).
- SHT40 humidity (unstreamed on the wire).
- On-rig mag re-calibration UI.
- Recording/playback, SLAM, showcase, settings — Phases 3–6.

## 4. Backend (`web.py`)

### 4.1 Shared state (built once in `main()`)

Mirror `panel.py:525-541` via `getattr` defaults so it works against
`viewer.resolve_args` (which doesn't define the panel's sensor args):

- `mag_cal = MagCalibration.load(getattr(args,"mag_cal_path","mag_cal.json") or "mag_cal.json")` when `getattr(args,"yaw_fusion",True)`, else `None`.
- `fusion = YawFusion(calibration=mag_cal, tau_s=…, anomaly_frac=…, motion_rate_dps=…, gimbal_margin_deg=…)` (panel defaults) when yaw-fusion on, else `None`.
- `sensor_state = SensorState(fusion=fusion)`.
- Stored on `app.state.sensor_state` / `app.state.mag_cal`.

The reader thread's `state=None` kwarg becomes `state=sensor_state`.
`_run_reader` already routes every DATA frame to `state.feed(frame)` inside a
guarded try/except (`panel.py:476-480`), and `SensorState.feed` self-filters to
streams 9/10 — no reader change needed. Streams 9/10 also already reach
`metrics` (labels "IMU"/"Env", `metrics.py:35-47`), so the Phase-1 HUD's
per-stream rows light up for free.

### 4.2 `build_sensor_message(sensor_state, mag_cal) -> dict | None`

A pure, socket-free helper (unit-testable, like Phase 1's `pack_*`/`build_metrics_message`):

- `quat = sensor_state.fused_quat()`, `env = sensor_state.latest_env()`,
  `press_hist`/`temp_hist` = the history snapshots.
- Returns **`None`** when there is no data at all (no quat, no env, empty
  history) → broadcaster skips the send (§4.3), keeping ToF-only sessions quiet.
- `rot`: `T_WORLD_TO_CV @ quat_to_matrix(*quat) @ T_CV_TO_BODY`, flattened
  row-major to 9 floats — the same display rotation `gizmo_pose` builds
  (`sensors.py:183-192`), computed server-side so the sign/permutation matrices
  live in exactly one place. `null` when no quat.
- `heading`: calibrated mag → `AXIS_CONVENTION @ mag_cal.apply(mag)` when a cal
  is loaded (`panel.py:3176-3178`), then `absolute_heading(quat, mag)`; `null`
  unless both quat and env are present.
- `pressure_pa`, `temp_c`, `mag_ut` (calibrated), `fusion` (`fusion_status()`),
  and rounded `pressure_hist`/`temp_hist` lists.

### 4.3 Broadcast tick

Add `SENSOR_INTERVAL = 1/15 s` and a gate in the single `_broadcaster` task
(alongside the IR and metrics gates): build and `_broadcast_text` the `sensor`
message when non-`None`. 15 Hz is smooth for a handheld gizmo and comfortably
above the ~4 Hz sparkline need; history is included every message so a
late-joining tab's sparklines are instantly full (Phase-1 late-joiner rule).

## 5. WebSocket protocol addition

One new **JSON** message (no new binary tag needed — the payload is tiny):

```
{"type":"sensor",
 "have_quat": bool,
 "rot": [r00,r01,r02, r10,r11,r12, r20,r21,r22] | null,   // display rotation, row-major
 "heading": deg | null,          // absolute_heading, 0=N clockwise
 "pressure_pa": float | null,
 "temp_c": float | null,
 "mag_ut": [mx,my,mz] | null,    // calibrated
 "fusion": "off"|"init"|"active"|"gated:*",
 "pressure_hist": [float,...],   // <=256
 "temp_hist": [float,...]}
```

`ws.js` already re-emits any JSON message keyed by `type`, so no `ws.js` change.

## 6. Frontend (`sensors.js` + DOM/CSS)

New module `sensors.js`, subscribed to `"sensor"`, constructed in `app.js`. All
visuals are **2D canvas** (no second WebGL context — keeps the headless
SwiftShader box cheap and robust):

- **Gizmo** — orthographic projection of the `rot` basis triad: each column
  → screen `(x=rot[c], y=rot[3+c])` (canvas +y is down, matching the scene's
  y-down CV up-vector). X/Y/Z drawn red/green/blue with tip letters; a far-Z
  cue dims axes pointing away.
- **Compass** — dial ring + N/E/S/W ticks + needle at `heading` (0=up=N,
  clockwise: tip `(cx+r·sinθ, cy−r·cosθ)`), matching `render_compass`
  (`sensors_widgets.py:30-44`); heading text under it.
- **Sparklines** — pressure + temp, min/max autoscaled polyline over the
  history array + a current-value readout, matching `render_sparkline`.
- **Fusion status** text.

Placement: a new **Sensors** section appended to the **left rail** (§8.1 of the
Phase-1 spec: "Phase 2 adds compass/gizmo/sparklines below it"). The rail is
`pointer-events:none`; the sensor visuals are read-only so that's correct. The
section is always present (stable layout) and shows `—` placeholders until data
arrives.

## 7. Error handling / edge cases

- No sensor frames ever → `build_sensor_message` returns `None`, nothing is
  broadcast, the section stays at placeholders.
- Missing `mag_cal.json` → `MagCalibration.load` returns `None`; fusion runs in
  `gated:no-cal` and heading uses raw mag (exactly the panel's behavior).
- Malformed 9/10 payload → swallowed by `_run_reader`'s per-frame try/except and
  `SensorState.feed`'s decode; never reaches the broadcaster.
- Frontend: a `sensor` message with `null` fields renders placeholders, never
  throws; one bad message can't take down the canvas (guarded draws).

## 8. Testing

Backend (pytest, extends `test_web.py`):

- `build_sensor_message` shape/units: `None` on empty state; `rot` is 9 floats
  and equals `T_WORLD_TO_CV @ R @ T_CV_TO_BODY`; heading/pressure/temp/mag
  populated; history lengths track fed samples.
- `SensorState` populates via `_run_reader` against a synthetic capture that
  interleaves DEPTH_ZF32 + IMU_QUAT + ENV frames (new `_make_sensor_capture`
  helper) — quat and env land, history grows.
- Full host suite stays green (Phase 1 was 606 passed).

Frontend: manual, headless Chrome via `host/tools/web_ui_shot.py` against the
synthetic sensor capture — confirm gizmo rotates, compass points, sparklines
fill, values update.

## 9. Migration / rollout

Confined to `host/src/roomscan/web.py`, `host/src/roomscan/static/`
(`sensors.js` new, `app.js`/`index.html` extended), and `host/tests/test_web.py`.
No wire/firmware change; `panel.py` untouched. Single commit to `main`.
