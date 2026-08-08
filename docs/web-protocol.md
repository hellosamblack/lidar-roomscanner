# The `roomscan-web` `/ws` protocol

The desktop panel talks Python-to-Python; the web app talks **browser ↔ FastAPI over WebSocket**.
Through Web Phase 6 that was a single socket at `/ws`; BUG-061 (2026-08-02) split off a second,
dedicated `/ws-mesh` socket for the binary MESH stream (see "The mesh channel — /ws-mesh" below) —
everything else (metrics/sensor/state/session/cmd/etc, and the other binary tags) is unchanged on
`/ws`. That contract has grown one message type at a time across Web Phases 1–3, and it lives
entirely inside `host/src/roomscan/web.py` builder functions — there is **no enum registry** the
way the *binary wire* protocol has `docs/protocol.md` + `protocol.py`. This doc is that missing
index: every `/ws`(-mesh) frame, its shape, and where it is built/consumed, so the next phase (SLAM
trajectory + mesh) has one place to hook into.

This is the **app protocol** (host ↔ browser). It is unrelated to the **device wire protocol**
(MCU ↔ host, `docs/protocol.md`). The two only meet at the reader thread: device frames are decoded,
transformed, and *re-encoded* into these `/ws` messages.

**Governing design specs** (the `§N` refs in `web.py` docstrings point at Phase 1's):
`docs/superpowers/specs/completed/2026-07-15-web-phase1-core-instrument-design.md` (binary tags + metrics/state/cmd/log/event),
`.../2026-07-16-web-phase2-sensors-design.md` (`sensor`),
`.../2026-07-16-web-phase3-recording-playback-design.md` (`session`/`captures` + inbound transport).

## Framing

**Two sockets**, `/ws` and `/ws-mesh`, each carrying two encodings distinguished by WS frame opcode:

- **Binary frames** — high-rate render payloads. First 4 bytes are a little-endian `u32` **tag**;
  the frontend switches on it (`ws.js`). Tags: `TAG_POINT_CLOUD = 1`, `TAG_IR_IMAGE = 2`,
  `TAG_MESH = 3`, `TAG_SURFACE = 4`, `TAG_MAGPOSE = 5` (`web.py`). **`TAG_MESH` is the one tag that
  does not ride `/ws`**: since BUG-061 it travels exclusively on the dedicated `/ws-mesh` socket
  (own connect/reconnect, credit-gated — see "The mesh channel — /ws-mesh" below); `/ws` no longer
  emits it. Every other tag is unchanged. Add new binary tags here and keep them contiguous, and in
  lockstep across `web.py` / `ws.js` / this table (the `protocol-change` skill) — and if a tag ever
  needs to move sockets again, treat it as a breaking wire change (see Invariants).
- **Text frames** — JSON objects, always with a `"type"` string field. Everything that isn't a
  per-frame render payload (metrics, sensors, UI state, control echoes, logs) is JSON on `/ws`. The
  sole exception is `mesh_ack`, the one JSON message type that lives on `/ws-mesh` instead.

All numeric binary fields are little-endian. JSON numbers go raw over the wire; the frontend formats
for display (units, precision).

### The app's one outbound third-party request

`roomscan-web` is otherwise entirely local. The single exception (2026-07-31) is
`roomscan/weather.py`: **one HTTPS GET to `api.open-meteo.com` every 30 minutes**, for the current
sea-level reference pressure the Sensors card's elevation readout is measured against. It is stdlib
`urllib.request` inside `asyncio.to_thread` (no dependency for one GET per half hour, the existing
`mcp_server/tools_rig.py` / `session.py` / `tools/web_ui_shot.py` precedent), it cannot raise into
the broadcaster, and every failure mode falls back to 101325 Pa and **says so on the wire** via
`sensor.msl_source`. A box with no uplink (this rig is often behind a battery Wi-Fi bridge) works
unchanged, just with `msl_source: "fallback"`. Location comes from `[viewer] latitude` / `longitude`,
cadence from `[viewer] msl_refresh_s`. If a second outbound request is ever added, it belongs in
this section — the property worth keeping is that the list is short enough to read.

## The mesh channel — `/ws-mesh`

**BUG-061 (2026-08-02).** `roomscan-web`'s WebSocket layer (uvicorn/ASGI) has no write-side
backpressure: `send_bytes` just hands the frame to the OS and returns, so a single broadcaster
`await`ing it for one slow client blocks every other message queued behind it on that connection —
pose, metrics, everything. A whole-map MESH resend is multiple MB; on a slow/Wi-Fi link that turned
into the reported 15 s of display lag, because the 30 Hz `slam` pose JSON was sharing a connection
(and an `await`) with an occasional multi-MB mesh. MESH now has its own socket, `/ws-mesh`, with
credit-based flow control (`MeshFlow`, `_pump_mesh`, `web.py`) so it can never again sit in front of
anything on `/ws`:

- **One mesh in flight per connection.** A connection gets a new mesh only once it has acknowledged
  (or timed out on) the last one it was sent — never a queue of stale intermediate meshes.
- **Latest-wins, by bytes identity.** `_cache_latest_mesh` holds exactly one cached mesh (the newest);
  a mesh a flow never got to send before it was superseded is simply never sent — its `superseded`
  counter increments instead. Identity is by the `bytes` object itself, **not** `mesh_seq`: the seq
  resets to 0 on `_reset_slam` (map teardown), so a seq comparison would wedge a flow that had already
  seen a higher number from the previous map.
- **Current mesh pushed on connect.** A late joiner (a fresh tab, or a reconnect) gets whatever is
  cached immediately in `websocket_mesh_endpoint`, rather than waiting for SLAM/Detailed to produce a
  "new" one.
- **Never-acked ("legacy") connections are throttled, not starved.** A connection that never sends
  `mesh_ack` (an old cached client, a diagnostic tool) still gets the latest mesh, just capped to at
  most one push per `LEGACY_MESH_INTERVAL_S` (5 s) instead of an open loop.
- **A 60 s ack timeout clears the credit, not the mesh.** `MESH_ACK_TIMEOUT_S` frees a flow that never
  acked so it isn't wedged forever, but it does **not** re-push the same bytes — `last_sent_obj` is
  untouched, so `_pump_mesh`'s identity check still sees "nothing new" until an actual new mesh is
  cached; only then does the flow send again.

Protocol on the wire: `/ws-mesh` sends binary `TAG_MESH` frames only (no other tag, no JSON out) and
accepts one JSON message in, `{"type": "mesh_ack", "seq": <mesh_seq>}` — see the inbound table below.
The client (`slam.js`) only sends that ack once the mesh is actually consumed: after `renderMeshes`/
`applyLines` swap the geometry in, inside a `requestAnimationFrame` callback, so the credit reflects
"uploaded to the GPU," not "bytes parsed." `window.__slamDiag` exposes `lastMeshSeq`/`lastMeshAt`/
`acksSent` for headless verification of that latency. `session.py`'s `RigSession` (the MCP client)
auto-acks every mesh immediately on its own best-effort `/ws-mesh` connection — it has no rendering
to wait on, so it exists only to keep `binary_counts`/`streaming` accurate, never to exercise
backpressure.

## Outbound — server → browser

### Binary

| tag | name | layout | built by |
|-----|------|--------|----------|
| 1 | `POINT_CLOUD` | `u32 tag · f32[3N] positions · f32[3N] colors` (positions then colors, concatenated) | `pack_point_cloud` |
| 2 | `IR_IMAGE` | `u32 tag · u16 width · u16 height · u8[w*h*3] RGB` | `pack_ir_image` `web.py:212` |
| 3 | `MESH` | `9×u32 header (tag, mesh_seq, flags, then 6 counts) · per-submesh f32 pos·f32 col·u32 idx · floor f32 pos·u32 line-idx` | `pack_mesh` (web Phase 4) — a SLAM `MeshPacket`; flags bit0=decimated, bit1=walls_split; emitted on the mesh-throttle cadence only. **Delivered exclusively over `/ws-mesh`, not `/ws`, since BUG-061** — see "The mesh channel — /ws-mesh" above for the credit-gated delivery contract |
| 4 | `SURFACE` | `u32 tag · u16 w · u16 h · u32 n_tris · f32[3·W·H] positions · f32[3·W·H] colors · u8[W·H] valid · u32[3·T] tri_indices · u8[W·H] covered` | `pack_surface_cloud` — grid-ordered positions+colors + triangulated mesh indices; sent instead of POINT_CLOUD when `surface_enabled` is true |
| 5 | `MAGPOSE` | `u32 tag · u32 seq · f32[4] quat(w,x,y,z) · f32[3] field_dir_body · f32[3] gravity_body · f32 field_ut · f32 dev_pct · f32 dip_deg · i16 live_cell · i16 filled_cell · u16 flags · u16 pad` — **68 bytes**, ~2.0 kB/s at 30 Hz | `pack_magpose` / `build_magpose` (magcal 3D feedback, 2026-07-29) — sent **only to `state.magcal_clients`** on the 30 Hz broadcaster tick. `live_cell`/`filled_cell` are `-1` for none; `dev_pct`/`dip_deg` may be `NaN` (no calibration / no gravity). `flags`: bit0 collecting, bit1 stationary, bit2 mag_anomaly *(reserved, Phase 2)*, bit3 have_quat, bit4 provisional_binning, bit5 sample_rejected. See "Magnetometer sweep" below |

`POINT_CLOUD` (or `SURFACE` in surface mode) goes out every broadcast tick (so late joiners see data within ~36 ms);
`IR_IMAGE` rides a slower cadence (`web.py:855`, `:869`).

**Frame of reference — positions are gravity-aligned, not sensor-frame.** `POINT_CLOUD`/`SURFACE`
positions are pre-multiplied server-side by `display_rotation(fused_quat)` — the canonical
`T_WORLD_TO_CV @ R @ T_CV_TO_BODY` sandwich from `docs/coordinate-frames.md`, the same matrix shipped
as the `sensor` message's `rot` — so the scene reads upright however the board is held (desktop-panel
orbit parity). With no orientation yet (ToF-only session, or before the first stream-9 sample) it falls
back to the raw sensor frame — identity rotation. The client applies **no** further rotation to the
cloud; doing so would double-count.

**...and which frame exactly is `state.view_mode`'s job** (camera views, owner ask 2026-07-29,
expanded 2026-08-01). For the live point cloud, the three modes are server-side rotations of the same
cloud; the client never composes one. SLAM and Detailed remain in their stable map frame and move the
shared Three.js camera from the latest scanner pose instead; Mirror flips only that map viewport. Preview
is a 2D thumbnail, so World/FPV keep its pixels unchanged and Mirror flips it left-right.

| `view_mode` | rotation baked into the payload | client camera |
|---|---|---|
| `world` | `display_rotation(quat)` — the paragraph above, unchanged | OrbitControls, world grid visible |
| `fpv` | `boresight_view_frame(R) @ R` — look down the sensor's optical axis, **still gravity-levelled** | locked just above/behind the origin, aimed down CV +Z; no orbit/pan/zoom, grid hidden |
| `mirror` | `diag(-1,1,1) @` the above — left-right flip (CV X is Right) | same locked pose |

`boresight_view_frame` builds a CV camera frame whose forward is the boresight and whose right is
forced perpendicular to world down, so composing it with `R` nets out to a **pure roll about the
boresight** — the same un-rolling `ir_gravity_rot` already applies to the IR pane, which is why the
FPV view and the IR monitor agree on which way is up. The owner requirement is that FPV/Mirror
"respect gravity the same way world does": the camera follows the sensor's *aim*, but the scene never
rolls. Shipping the raw sensor frame instead (the obvious first implementation) fails exactly that.

Two consequences worth stating, because both are load-bearing:
- **The mode is resolved server-side on purpose.** The cloud is rotated by the *smoothed* display quat
  at the broadcast rate, while the client's only orientation feed (`sensor.rot`) is raw and half as
  fast — a client-rotated camera would lag and slosh against the geometry it is meant to be locked to.
  Baking the frame in makes the client camera a static pose, matched by construction.
- **A vertical boresight has no "level".** Aimed at the ceiling or the floor the right-axis cross
  product degenerates, so `boresight_view_frame` falls back to the sensor's own up axis. A handheld
  scanner visits that attitude constantly; without the fallback the cloud goes NaN.

**Camera framing — one baseline, three offsets** (`state.view_cam`, owner ask 2026-07-30). The client
camera is never *at* the sensor's optical centre: a camera exactly there reproduces the depth image's
own projection, so every point lands where it already was in 2D and the render reads as a flat
picture. The offset that fixes that is user-tunable and persisted, and **all three modes are described
against one reference — the FPV ground truth, a camera at the sensor looking down its boresight.
All-zero offsets reproduce that camera exactly, in every mode.**

| field | meaning | range |
|---|---|---|
| `distance_m` | back off along the view axis | 0 … 15 |
| `height_m` | lift the eye (world is Y-down, so up is −y) | −5 … 10 |
| `rotation_deg` | swing the eye about the vertical axis through the aim point, positive to the right; the aim point stays on the axis so the subject stays framed | −180 … 180 |

The modes differ only in what "the axis" is: `fpv`/`mirror` ride the live boresight (the server has
already rotated the cloud into the boresight view frame), `world` uses the fixed world forward. Those
coincide when the board is held in the reference pose, which is what makes this one baseline rather
than three unrelated cameras. Defaults: world `(4.2, 2.6, 0)` — an elevated establishing shot;
fpv/mirror `(0.30, 0.20, 0)` — just over the sensor's shoulder, enough parallax to read depth.

**Auto-orbit** (`orbit_enabled` / `orbit_speed_deg_s`, owner ask 2026-07-30) slowly circles the world
view: **azimuth only**, so elevation and distance stay exactly where the sliders put them. It is
**world-only** — a locked view has nothing to circle — and the client greys the controls out elsewhere.
Implemented as OrbitControls' own `autoRotate` (whose speed unit is 6 °/s), *not* by animating
`rotation_deg`: that would churn a persisted value at frame rate and write `roomscan.toml` continuously.
So the orbit is a live camera animation and the stored `rotation_deg` is left alone — a running orbit
does not drift the saved framing. `scene.js` passes a real wall-clock delta to `controls.update(dt)`,
because the default assumes a fixed 1/60 s step and this host renders the live scene nearer 13 fps.
Default off at 6 °/s (one revolution per minute); negative reverses, 0 parks it without disabling.

**Oscillate mode** (`orbit_mode` / `orbit_amplitude_deg`, owner ask 2026-07-31) is a second way to drive
the same auto-orbit: instead of circling forever (`orbit_mode = "continuous"`, the original behaviour
above), `"oscillate"` swings the camera in a triangle wave about the azimuth where oscillation started —
run one way for N°, reverse and run 2N°, reverse and run 2N°, repeat, netting a swing of ±N (the
`orbit_amplitude_deg` slider, 5–180°, default 45) about the start. World-only, same gate as continuous
orbit. Built entirely client-side on top of `autoRotate`/`autoRotateSpeed`: `scene.js` tracks
`controls.getAzimuthalAngle()` each World frame and flips `autoRotateSpeed`'s sign when the accumulated
offset from the start azimuth passes ±N — the server only stores/validates the mode and amplitude, it
does not drive the wave. `getAzimuthalAngle()` wraps to (−π, π], so the accumulator is built from
per-frame **unwrapped** steps, not the raw wrapped angle, or an amplitude past 180° would latch going one
direction forever. The start azimuth is captured (client-side, not persisted) the moment oscillation
actually begins — entering World, enabling auto-orbit, or switching to oscillate while already
orbiting in World — never at page load.

Client side (`scene.js` `poseFor`) the pose is `target + Ryaw(rotation)·(0, −height, −(look_ahead +
distance))` with `look_ahead = 1 m`, which is what makes the zero offset land exactly on the sensor.
In world the computed pose is the *default/reset* framing and OrbitControls takes over from there;
in fpv/mirror it is the locked pose. Ranges are validated on the wire **and** on config load, so a
hand-edited `roomscan.toml` cannot park the camera somewhere the UI can't recover from.

With no orientation (ToF-only) `fpv` is `None` (the cloud is already the sensor's own view) and
`mirror` is the bare flip. The mode applies to **real-time only** — SLAM mode ships a mesh and drives
its own follow camera, and the client disables the control there.

**`IR_IMAGE` is rolled to match, but in two halves.** The server pre-rotates by whole quarter turns
(`ir_gravity_rot` → `np.rot90`), which is pixel-exact and free, so **`width`/`height` swap on a 90°/270°
roll** (54×42 ↔ 42×54) — clients must size from the message, never assume landscape. The snap alone can
be up to 45° short, which is visible: at ~40° of roll it is *zero* turns, so the pane sits still while
the cloud tilts the full 40°. The remainder therefore rides the `sensor` message as **`ir_roll_deg`**
and the client finishes the job with a CSS transform, so the 54×42 image is never resampled and stays
pixel-crisp. Unlike the cloud, the IR client *must* apply that residual — it is the other half of the
rotation, not a duplicate.

`ir_roll_deg` is the rotation to **apply**, not where gravity sits — the two differ by a sign, and getting
that backwards is the bug that shipped first (BUG-026 follow-up, 2026-07-29): the pane turned the wrong way,
so instead of holding still the content counter-rotated at **twice** the board's rate. `ir_gravity_angle_deg`
negates `atan2` for exactly this reason. **Do not "simplify" that negation away** — it is invisible in the
two checks you would naturally reach for (a 180° flip is its own inverse; a 90° turn swaps width/height
either way), and the sign is pinned instead by
`test_ir_gravity_angle_matches_the_point_cloud_rotation`, which derives the expected rotation from the
verified cloud path. The trap: `T_WORLD_TO_CV @ R @ T_CV_TO_BODY` rotates points in the CV frame where **Y
points down**, so a positive rotation there is *clockwise* on screen, while `np.rot90` is *counter-clockwise*.

`ir_roll_deg` is **CCW-positive** (`np.rot90`'s sense, which is also counter-clockwise on screen since
row 0 renders at the top), so a CSS `rotate()` — which turns clockwise — needs the **negated** value.
It is computed from the *smoothed* display quat, the same one the snap uses; deriving it from the raw
fused quat instead would let snap and residual disagree near a 45° boundary and make the pane jump.
`null` before the first smoothed sample, and the client then applies no residual rather than guessing.
**Mirror mode flips the pane, and nothing else does.** `view_mode == "mirror"` adds a signed
`scaleX(-1)` to the same CSS transform, matching the server's X negation of the cloud (the IR raster
and the cloud grid share one index space, so the two operations are the same flip). `world` and `fpv`
render the pane identically — the gravity roll is *not* a function of the view mode (owner: "IR view is
unchanged for all but mirror"). Order matters in the transform list: CSS applies it right-to-left, so
the scale is listed **before** the rotate, which flips the already-rotated pane rather than the raw
image. A uniform scale commutes with a rotation and a signed one does not, which is why the original
`rotate(...) scale(k)` order was harmless and is no longer.

Because the image rotates freely, `ir.js` renders it inside a fixed **square** frame (`#ir-frame`),
scaled so the rotated bounding box always fits: the card never changes shape as the board rolls and no
field of view is cropped, at the cost of empty corners at intermediate angles (worst at 45°, where the
image exactly inscribes the square).

The display quat is **smoothed** first (`OrientationSmoother`). Raw fused orientation carries ~0.14°
mean / 0.25° p95 of zero-mean noise per update, which a 3 m lever arm turns into visible shimmer at the
cloud edges; a magnitude deadband can't suppress it without also dragging on a slow deliberate pan
(~0.7°/frame), so the gate is directional **coherence** over a trailing window — the same discriminator
as the SLAM stationarity hold, sharing `roomscan.motion.coherence`. Measured on the live rig
(2026-07-28): 1.72 mm → 0.34 mm mean edge motion at 3 m, with coherent motion still tracked 1:1.
Smoothing is **display-only** — SLAM reads `sensor_state.fused_quat()` directly, so it can never reach
the reconstruction, and the `sensor` message's `rot` stays **raw** so the gizmo remains an honest
sensor readout.

**Raw orientation numerics + jitter (owner ask, 2026-07-28).** The `sensor` message also carries
`orientation_raw` (the quaternion + Euler angles + heading at full precision) and `jitter` (rolling
frame-to-frame noise stats). Both are **pre-`OrientationSmoother`** — the same raw
`sensor_state.fused_quat()` SLAM sees, not the shimmer-suppressed display quat above — and computed
server-side from **full-precision** internal values, never from `rot`/`heading`: those are rounded
(5dp / 1dp) for the wire, which censors changes below ~0.0006°/0.05° — the same rounding-hides-the-signal
trap `docs/iks4a1-stacking.md` → "Orientation-noise pass" already hit once, here avoided by computing
jitter off the unrounded values before they're rounded for the wire. `orientation_raw.quat` is
`[w,x,y,z]` at 6dp as received (not re-normalized — "raw" means raw);
`roll_deg`/`pitch_deg`/`yaw_deg` are `sensors.quat_roll_deg`/`quat_pitch_deg`/`quat_yaw_deg` (ZYX
Tait-Bryan) at 4dp; `heading_deg` is `absolute_heading` (same math as `heading` above, just unrounded).
All four are `None` until the first stream-9 sample, and `heading`/`heading_deg` are `None`
again whenever the boresight is aimed within 10 deg of vertical -- a compass bearing does not
exist there, and reporting one anyway was BUG-058 (the value tracked *roll*).

`jitter` is `{"window_s": 5.0, "<signal>": {"mean_deg", "p95_deg", "n"}, ...}` for
`roll`/`pitch`/`yaw`/`heading`/`orientation`, tracked by `web.OrientationJitter` over a trailing
`SENSOR_JITTER_WINDOW_S` (named constant, `web.py`) window of `sensor` messages (~75 samples at the
15 Hz `SENSOR_INTERVAL`). Each stat is the **absolute** frame-to-frame change (a magnitude, not a
signed drift): `roll`/`pitch`/`yaw`/`heading` diff with `sensors.wrap180` so a heading crossing
360°→0° doesn't register as a 360° jump; `orientation` is the quaternion dot-product angle
(`quat_angle_deg`) between consecutive **re-normalized** quats — skipping the re-normalization lets a
not-quite-unit float32 quat push `|dot|` past 1.0, get clamped to exactly 1.0, and silently report a
**zero** angle for the smallest real steps (this bit the 2026-07-28 measurement). **`p95_deg` is the
headline statistic** (measured CV 1.45%, the most stable of mean/median/p95 on this signal); `mean_deg`
is secondary; **median is deliberately not reported** — quantization ties pile up at near-zero steps
and make it the least stable (CV 4.41%). `n < 2` reports `mean_deg`/`p95_deg` as `None` (not enough
samples yet), not `0.0`.

**Orientation decomposition modes + labels (owner ask, 2026-07-28; world mode follow-up same day).**
Roll/pitch/yaw are not always valid names for the axes — they presume a fixed body "forward" a handheld
scanner doesn't have — and the ZYX Tait-Bryan decomposition above has a gimbal lock (pitch → ±90°) that
bites at exactly the attitudes a handheld device visits (measured live at ~86° pitch: roll/yaw both went
degenerate). `sensor.orientation_view` reports the **selected** decomposition of the same
`sensor_state.fused_quat()` — still presentation-only, still never touching `display_rotation`/
`fused_quat()`/the SLAM path. Modes (`web._VALID_ORIENTATION_MODES`, math in `sensors.py`'s "Alternate
orientation decompositions" section):

| mode | reports | singularity |
|------|---------|-------------|
| `zyx` | the same numbers as `orientation_raw` (`quat_roll_deg`/`quat_pitch_deg`/`quat_yaw_deg`) | `pitch` → ±90° — body **X (Up)** axis → world vertical |
| `zxy` | alternate Tait-Bryan (`quat_roll_alt_deg`/`quat_pitch_alt_deg`/`quat_yaw_alt_deg`, sequence `R = Rz·Rx·Ry`) | `pitch` → ±90° — body **Y (Right)** axis → world vertical (a disjoint attitude from `zyx`'s lock — device rolled onto its side, not aimed steeply up/down) |
| `boresight` | azimuth/elevation/roll of the ToF optical axis (`boresight_view_deg`) — meaningful under any grip because it reports where the *sensor* points, not an arbitrary body "forward". The boresight is body **+Z**: `docs/coordinate-frames.md` "The four frames" lists SFLP body Z=Forward, and `T_CV_TO_BODY` maps the ToF (CV) frame's Z=Forward onto body Z unchanged, so body Z is the optical axis under both frames' own definitions | `elevation` → ±90° — pointing straight at the ceiling/floor |
| `world` (default) | gravity+magnetometer TRIAD/eCompass reference (owner: "could we use gravity + the magnetometer..."): `pitch_deg`=tilt-of-the-boresight-from-horizontal, `yaw_deg`=heading-from-magnetic-north (reuses `absolute_heading` verbatim, not re-derived -- the boresight's compass bearing minus magnetic north's, both read in the quat's own frame, so its drifting datum cancels; `null` when the aim is within 10 deg of vertical, with `valid: false` and a reason, BUG-058), `roll_deg`=twist about the boresight referenced to true vertical — the gravity-only half of TRIAD (`triad_roll_deg`), no mag/gyro involved. Drift-free and **grip-independent**, but a poor *dynamic* tracker: linear acceleration is indistinguishable from gravity tilt, so `pitch_deg`/`roll_deg` degrade badly while the device is moving — that dynamic weakness is exactly why the gyro + complementary filter drive the primary orientation instead. Not "better", just a different, absolute-referenced view | `pitch_deg` (tilt) → ±90° — boresight → vertical, where the TRIAD roll's perpendicular reference collapses |

**Web UI pinned to World (owner ask, 2026-07-31).** The Sensors card's decomposition picker
(`<select id="orient-mode-select">`) is gone — the owner only ever used World — so `ViewerConfig.orientation_mode`
now defaults to `"world"` and `web.ui_from_config` coerces **any** stored value to `"world"` on load (a
config file written before this change can't strand a fresh boot in a mode with no picker to change it
back from). `DEFAULT_AXIS_LABELS` changed from `("Roll", "Pitch", "Yaw")` to `("Roll", "Tilt", "Heading")`
to match what World's three slots actually are (see the `world` row above); the labels stay
user-renamable. `set_orientation {mode}` and the `zyx`/`zxy`/`boresight` math above are **unchanged and
still on the wire** — only the deprecated desktop panel (`panel.py`) exercises them now.

`orientation_view` shape: `{mode, labels[3], roll_deg, pitch_deg, yaw_deg, singularity_margin_deg,
near_singularity, valid, reason}` plus, in `world` mode only, `gravity_source`("imu_raw"|"quat"),
`mag_norm_ut`, `mag_expected_ut`, `motion_stable`. `singularity_margin_deg` is `90 - |the mode's singular
component|`; `near_singularity` fires within `ORIENTATION_SINGULARITY_MARGIN_DEG` (15°, a threshold
inherited from the since-deleted `YawFusion.gimbal_margin_deg` — BUG-051; this one is a **display**
warning about the selected decomposition's own conditioning and no longer mirrors any live gate). In
`world` mode, `valid`/`reason` gate the **mag-dependent `yaw_deg` (heading) only** — `roll_deg`/`pitch_deg`
(gravity-only) are reported regardless, their own trustworthiness being `near_singularity`, not `valid`.
`valid` is false when either (a) the calibrated+axis-corrected mag magnitude deviates from the fitted
`field_ut` by more than `WORLD_MODE_MAG_ANOMALY_FRAC` (0.3, reusing `YawFusion.anomaly_frac`'s notion —
on the live rig this reads ~107-109 µT against a 49.87 µT fit, a ~2.15× anomaly, so World mode is
currently WRONG and the UI must show that, not a confident bad number) or (b) `imu_raw`'s accel batch
norm strays >`WORLD_MODE_ACCEL_TOL_G` (0.15 g) from 1 g (device accelerating). `gravity_source` is
`"imu_raw"` when the stream-11 SFLP gravity FIFO tag (0x17, fixed ±2g/0.061 mg-LSB, ~16× finer in tilt
than the fp16-encoded SFLP quaternion) was available, else `"quat"` (the `R.T @ [0,0,-1]` fallback
`ir_gravity_rot` already uses). `labels` are the user-renamable slot names (`orientation_labels` below) —
positional, same 3 slots regardless of mode.

`orientation_raw`/`jitter`'s `roll`/`pitch`/`yaw` fields are **unaffected** — always ZYX, exactly as
before (back-compat); it is `orientation_view` and `jitter`'s `roll`/`pitch`/`yaw` entries that switch
meaning with the mode. `jitter.orientation` (quat dot-product) and `jitter.heading` (magnetic) are
**convention-independent** and always mean the same thing — the trustworthy signals when comparing across
a mode switch. On a mode change, `OrientationJitter` purges only the `roll`/`pitch`/`yaw` window entries
(and resets their running previous-value state) — diffing a `zyx` roll against a `boresight` roll from the
sample before the switch would be nonsense.

**"Zero yaw here" (owner ask, 2026-07-29: "Yaw is at -43 degrees. Wish I could reset that").** The SFLP
quaternion is a **game rotation vector** — accel+gyro only, no magnetometer — so its yaw origin is
whatever attitude the sensor happened to be at power-on/init, and it free-runs with gyro drift; roll/pitch
are gravity-referenced and absolute, yaw is not. `UiState.yaw_offset_deg` lets the user pin a new zero:
`zero_yaw` (inbound, below) captures the **currently displayed mode's** `yaw_deg` and stores the
`sensors.graft_yaw` delta that would zero it (`web._YAW_GRAFT_SIGN` — `zyx`/`zxy` need the *negative* of
the captured yaw, `boresight`'s azimuth has the opposite handedness and needs the *positive*; derived
numerically against the real decompositions, not assumed). Every subsequent `sensor` message applies that
delta via `graft_yaw` to a **display-only copy** of the quat before it is decomposed — never to
`sensor_state.fused_quat()` itself — so `rot`/`heading`/`orientation_raw`/`jitter.orientation` and
everything SLAM consumes are computed from the untouched quat exactly as before (same guarantee as the
mode picker above). `graft_yaw` is used deliberately instead of adding/subtracting degrees from a Euler
field: it rotates about world +Z, which providably preserves roll/pitch/tilt, so the value fed into
`orientation_view` stays a valid rotation. **World mode is excluded**: its `yaw_deg` is the *absolute*
magnetic `heading_full` (`absolute_heading`), not a function of the quat's own yaw component, so grafting
it would be meaningless — the offset is force-zeroed in `orientation_view_out` whenever `mode == "world"`,
`zero_yaw` no-ops (logs a warning) while World is selected, and the Sensors card disables the "Zero Yaw
Here" button for the same reason (`sensors.js`'s `isWorld` guard). Because the offset is a **constant**
per session, it cancels exactly in any frame-to-frame diff — `jitter`'s roll/pitch/yaw magnitudes (p95/
mean) are unaffected by whether an offset is set, only the absolute reading shifts. The offset is echoed
in `state.yaw_offset_deg` (0.0 when none) and persisted to `roomscan.toml`'s `[viewer]` table exactly like
`orientation_mode`/`orientation_labels`.

**Magnetometer sweep + calibration quality (owner ask, 2026-07-29: "visualize ideal and current
magnetometer sweeping ... help the user understand what angles they're missing, and show calibration
quality").** The `magcal` message/action pair backs the Sensors card's **Calibrate Mag** modal
(`static/magcal.js`, math in `roomscan/magsweep.py`, fit reused verbatim from `magcal.fit_ellipsoid`).

*Why it exists.* A correct calibration reports a **constant |B| in every orientation**. The
calibration shipped 2026-07-15 did not — across a deliberate tilt sweep it read 50.5 µT at 0° tilt
(ceiling-facing) rising monotonically to 85.1 µT at 90° (horizontal), ~1.7×, producing heading errors up
to ~90° in exactly the wall-scanning attitude the scanner is used in. That is the signature of an
incomplete tumble, and nothing in the pipeline measured it. The modal therefore (a) shows missing
coverage **during** collection and (b) forces an explicit quality preview before a fit can overwrite the
saved file.

*Binning — a Fibonacci sphere lattice, not lat/lon.* Cells are the Voronoi cells of a
`SPHERE_CELLS = 92`-point Fibonacci (golden-spiral) lattice; a sample lands in the cell whose direction
it is closest to (one argmax of dot products — no wrap/branch logic). Lat/lon bins are rejected because
their areas differ by ~50× between pole and equator, so a lat/lon "coverage %" over-weights the poles
and would report *covered* for a tumble that missed most of the sphere — the exact misjudgement that
produced the bad calibration. Fibonacci cells are near-equal-area (nearest-neighbour spacing varies
< 1.2×, ~20°), so *fraction of cells occupied* **is** *fraction of the sphere covered*, and a K-cell gap
means the same thing wherever it sits. 92 cells ⇒ ~12° cell radius: fillable by a careful hand tumble,
coarse enough that a whole missing attitude family is many cells wide.

*What is binned.* The **calibrated, body-frame** field direction
(`AXIS_CONVENTION @ cal.apply(raw)`), **not** the raw direction. This is load-bearing: the rig's
hard-iron offset (~65 µT) exceeds the field (~50 µT), so raw sample vectors live in a cone around the
offset and their directions cover barely half the sphere however thoroughly you tumble — binned raw,
coverage could never reach 100%. Binned calibrated, a cell means "the device was held such that the
field entered it from this direction", which is both actionable and what the ellipsoid fit must span.
Precedence for the binning calibration is candidate → saved → `magsweep.provisional_calibration` (a
bounding-box hard-iron estimate used only for display on a first-ever tumble, never fitted, never saved);
the message reports which was used in `binning`.

*Quality metrics and their thresholds* (all computed server-side; the components are always shipped
alongside the headline, deliberately — a single opaque score is what let the bad calibration through):

| metric | definition | good | marginal | bad |
|---|---|---|---|---|
| **field spread** (headline) | `std(|B|)/mean(|B|)` over all samples after applying the calibration (`std_pct`) | < 2% | < 5% | ≥ 5% |
| **field bias** | `(mean(|B|) − field_ut)/field_ut` (`bias_pct`) | < 2% | < 5% | ≥ 5% |
| **coverage** | fraction of the 92 cells holding ≥ 1 sample | ≥ 85% | ≥ 60% | < 60% |
| **samples** | raw sample count | ≥ 300 | ≥ 100 | < 100 |

`field.verdict` is the worse of spread and bias (`spread_verdict`/`bias_verdict` are both shipped), and
the block's headline `verdict` is the worst of field / coverage / samples, with `limited_by` naming the
driving component and `reason` phrased as **what to do about it** — the three failures need opposite
responses (a spread/bias failure means the calibration is wrong; a coverage/sample failure means the
*measurement* is not yet conclusive, and "limited by coverage" alone reads as an indictment of the file
when it is really an indictment of the sweep). `field` also carries `min_ut`/`max_ut`/`ratio` (the ×1.7
the tilt sweep exposed) and `residual_rms_ut` (RMS deviation from the sphere of radius `field_ut`, which
combines both failure modes: `sqrt(std² + bias²)`). The 2% bar matches the existing convention in
`docs/yaw-fusion.md` ("< 0.02 is a clean fit").

Two traps the component split exists to close, both observed for real:
- **Spread alone is not sufficient.** Driving the modal on-rig on 2026-07-29, 255 stationary samples read
  |B| = 101.96 µT against the saved calibration's own `field_ut` of 49.87 µT — a ×2.04 bias — yet scored
  `std_pct = 0.22%`. A perfectly self-consistent calibration at completely the wrong magnitude must not
  score `good`, hence `bias_pct`.
- **Coverage must be read next to field, never folded into it.** A calibration fitted from a cap of the
  sphere is perfectly self-consistent *over that cap* and would score `field: good` on its own samples —
  which is exactly how the 2026-07-15 calibration passed unnoticed.

*Surfacing the existing defect.* `cell_dev_pct[i]` is the mean signed |B| error in cell *i* as a percent
of `field_ut`, computed under whichever calibration `view` selects. Point `view` at `current` and the
saved calibration's direction-dependent error becomes a picture: a hue that changes across the map **is**
a direction-dependent calibration error.

*Workflow and safety.* `start` → live coverage/quality → `stop` (fits a **candidate**, in memory only) →
preview → `save` or `discard`. `save` is the only action with a side effect: it writes to
`ViewerConfig.mag_cal_path` (default `mag_cal.json`, resolved once in `main()` so the web modal, the
loader and `panel.py:440` all agree on the file) in the same `{offset, matrix, field_ut}` format
`tools/mag_calibrate.py` writes, then hot-reloads via `web.install_mag_calibration` — which updates
`state.mag_cal`, the `YawFusion` filter's own captured `cal`, **and** calls `reset_fusion()` so the yaw
delta re-snaps instead of low-passing a stale one. No server restart is needed. `save` with no candidate
is refused, so the saved calibration can never be replaced by a fit nobody previewed; `discard` keeps the
samples so the user can keep tumbling into the same cloud.

*Isolation.* Everything except `save` is pure observation. Collecting/fitting/previewing/discarding —
**including the 30 Hz `MAGPOSE` stream** — leave `display_rotation`, the point-cloud bytes, `fused_quat()`
and the loaded calibration bit-identical; guarded by
`tests/test_magsweep.py::test_magcal_preview_does_not_touch_display_path`, which drives a full
open → pose-stream → close cycle. The live view keeps streaming while the modal is open (the modal is an
overlay, not a mode).

**3D feedback — the two-channel split (owner ask, 2026-07-29; design
`docs/superpowers/specs/completed/2026-07-29-magcal-3d-feedback-design.md`, Phase 1).** The modal's hero is now a
WebGL "coverage shell" (`static/magcal3d.js`), with the 2D Lambert disc pair kept as the **fallback**
renderer. Two things follow for this protocol:

*The split.* Raising the `magcal` JSON to 30 Hz would be ~60 kB/s and 30 × `JSON.parse` per second of
UI-thread work for data (cell counts, verdicts, coverage) that changes at **human** speed — the wrong axis
to scale. Instead the channel is split by rate of change, exactly as the rest of the app splits it:
the **binary `MAGPOSE` tag 5** carries pose + field direction + gravity + live cell at 30 Hz (68 B), and
the **`magcal` JSON** stays the 5 Hz truth. `filled_cell` is what lets the JSON stay slow: the fast channel
carries the *delta* ("this sample just lit cell 47") so a cell goes solid the instant it fills, while the
slow channel reconciles the counts (`MagSweepSession.sync_occupied`). Binary rather than a small 30 Hz
JSON because orientation must not be measured off a rounded decimal — `sensor.rot` is 5 dp; f32 avoids
inventing a second rounding policy.

*Frames — what the client is allowed to compute.* Per `docs/coordinate-frames.md` and the "server-side
math stays server-side" invariant, **no sign or permutation matrix is ever written in JS**. The body-fixed
hero needs **no transform at all** (`cell_dirs`, `field_dir_body`, `gravity_body` are already SFLP-body
unit vectors — which is also why it renders correctly on a session with no stream 9). The world-fixed
Steering widget maps a body vector to the renderer's world as `T_WORLD_TO_CV · R · v_body` — note **not**
the `T_WORLD_TO_CV · R · T_CV_TO_BODY` sandwich, which maps *CV* points; ours are already body points, so
the `T_CV_TO_BODY` leg is absent. `T_WORLD_TO_CV` is shipped once as `t_world_to_cv[9]` (row-major) on
`open` and applied as a static matrix; the only per-frame client math is `quat → Quaternion` and a slerp.
The steering rotation likewise arrives as an explicit body `guidance_axis.axis` + `angle_deg`
(`axis = unit(t × d)`, `angle = acos(t·d)`, `magsweep.rotation_to`), so the client never re-derives it.

*Guidance has no dip or compass assumption.* The old text ("point the Top face toward magnetic north and
downward") assumed northern-hemisphere dip *and* that the user knows where north is. It is replaced by the
exact body-axis rotation above, plus a countdown of the cells left in the gap being steered at.

## Ranging profiles & manual sensor control (Task 10, 2026-08-03)

`docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md` Task 10. Two
server-owned, device-authoritative state objects — `web.RangingState` (the ranging profile: Room
Mapping / Precision / High Frame-Rate / Manual) and `web.ImuEnvRateState` (the independent
streams-9/10/11 poll rate, cmd 11/12) — both broadcast together on **one** outbound `ranging`
message, but with **two independent `pending` flags**: a manual-params edit and an IMU/env rate
change are separate atomic commands and one must never block the other (plan item 8: "a second,
independent pending command, not a fifth field bolted onto Manual ranging").

*Applied state is committed only from a real device readback*, never assumed and never optimistic
(`web._commit_ranging_ack`/`web._refresh_ranging_state`/`web._refresh_imu_env_state`). `SET_RANGING_PROFILE`
(cmd 8) keeps the legacy 12-byte ACK, which carries only the applied `ProfileId` — no ranging_mode/
fps/exposure/power — so a successful preset switch always chains exactly one `GET_RANGING_CONFIG`
(cmd 10) round trip to learn what was actually applied. `SET_MANUAL_PARAMS` (cmd 9) needs no second
trip: its own 16-byte ACK already *is* the readback. At startup (and on device reconnect), `_init_ranging_state`
restores both halves from `GET_RANGING_CONFIG`/`GET_IMU_ENV_RATE` — never an assumed firmware default.

*Which of the four buttons is "applied" is inferred, not stored on the wire.* A device readback
carries only `ranging_mode`/`frame_period_us`/`exposure_ms`/`power_mode` — no profile id — so
`web._match_profile_id` compares the readback against `roomscan.profiles.PRESETS` for an exact
match; anything that doesn't match one of the three presets reads as `"manual"`, whether it was
reached via `SET_MANUAL_PARAMS` or happens to be a configuration no preset describes.

*Every number is `roomscan.profiles`'s, never a second copy.* `estimate` is
`profiles.ProfileEstimate` serialized by `web._estimate_to_json` — range/power/I3C-bus-airtime
figures and their warnings/errors are exactly what that pure model computes for the **applied**
config; `web.py`/`controls.js` hardcode no threshold or formula beyond the documented **70%/85%**
I3C-bar green/yellow/red cutoffs (the spec's own bar-coloring convention, matching the existing
`.hud-bar__fill`/`is-warn`/`is-crit` pattern the Resources card and TSDF-blocks gauge already use).
`transport` is one of `udp`/`cdc`/`replay`/`none` (`web._transport_kind`, mirroring
`transport_counters`'s existing `_live_underlying` type check) — the CDC-above-60-fps warning is
`estimate.transport_warning`, computed server-side from this field, never guessed client-side from
URL/browser-location/link-rate. `measured_fps` reuses the *same* `MetricsRegistry.snapshot()` the
`metrics` message's `streams[].device_hz` already reports for the ToF stream (`RAW_3DMD`/
`DEPTH_ZF32`) — no second measurement.

*Three distinct FPS numbers, never conflated.* `applied.fps` is what was requested and accepted;
`estimate.expected_delivered_fps` (`profiles.expected_delivered_fps`, from the 2026-08-03 measured
hardware ceiling investigation — see `profiles.py`'s module docstring) is the **honest** rate the
sensor is expected to actually produce, which equals `applied.fps` only at/below that exposure's
measured 1x delivery ceiling; above it, the sensor still accepts the request but delivers
period-multiples, and `validate_manual_params` now folds a WARNING about exactly this into
`estimate.warnings` (rendered separately from `estimate.transport_warning`, which keeps its own
dedicated field/row). `measured_fps` is the actual observed device cadence, independent of both.
The Manual FPS slider must never silently apply the request without this consequence being visible.
`estimate.ceiling_fps` (`profiles.ceiling_fps_int`, the measured integer 1x ceiling for the applied
exposure) is what the Manual FPS slider **greys out above** — the client also mirrors the small
`_MEASURED_CEILING_FPS` table so the grey-out tracks the exposure live while editing, one commit
ahead of the server echo. The full-curve ceiling was **remeasured on-rig 2026-08-05** (an exposure/fps
sweep; IMU-load-independent), superseding the 2026-08-03 derived line above 8 ms which was optimistic
by 1–2 fps (it predicted 16 ms → 30.8 fps 1x; the sensor actually tops out at 29).

*Honest with today's firmware.* Commands 11/12 (`SET_IMU_ENV_RATE`/`GET_IMU_ENV_RATE`) are live as
of Task 7 (`3f4b307`, hardware-gated 2026-08-04): coupled mode (rate 0, the default) is
byte-identical to pre-feature firmware, and a decoupled rate is genuinely applied on its own
TIM2-paced service tick independent of the concurrent ToF profile. `ImuEnvRateState` still commits
`applied_rate_hz` only from a real device readback, never optimistically — against any firmware that
still ACKs `UNKNOWN_CMD` for these commands (e.g. a device not yet reflashed past Task 7), it reports
that as `error`, leaves `initialized: false`, and stays "coupled" rather than adopting the request as
if confirmed (this fallback path stays regression-covered by
`test_set_imu_env_rate_unknown_cmd_reports_error_and_stays_coupled` in `test_web.py`, which now
exercises the pre-Task-7-firmware case rather than the everyday one).

*One-way flow, same rule as every other segmented control.* The four-way profile selector's active
segment and the Coupled/Explicit toggle's `coupled` half are driven **entirely** by the `ranging`
echo; clicking never sets them locally. The Manual panel's own ranging-mode/power-mode sub-segments
and the IMU/env Explicit-Hz slider are the one deliberate exception — they are input widgets
composing the *next* request, not read-outs of applied state, so they hold a local selection between
edits (seeded from the applied config whenever nothing is pending), exactly like the View card's
camera-framing sliders already do. `pending` gates its whole control group client-side (disabled,
not queued) in addition to the server-side reject in `_set_profile`/`_set_manual_params`/
`_set_imu_env_rate` (a second change while pending is rejected, never silently overwriting the
in-flight request).

### JSON (`type` → shape)

| type | key fields | built by | notes |
|------|-----------|----------|-------|
| `metrics` | `render_fps`, `transform_fps`, `browser_fps`, `streams[]{stream_id,label,device_hz,host_hz,bytes_per_s,jitter_ms}`, `link_bytes_per_s`, `resources`{proc_cpu_percent,n_cores,proc_rss,ram_total,ram_used,sys_cpu_percent,gpu_util,proc_vram,vram_total,gpu_name,gpu_source,device_vram_used,device_vram_total,device_vram_source}, `drops`, `gaps`, `ws`{clients,mesh_clients,mesh_in_flight,legacy_mesh_clients,mesh_superseded_total,mesh_ack_lag_s_max,mesh_ack_timeouts_total,latest_mesh_bytes}, `transport`{frames_incomplete,frags_lost,frags_reordered,frags_duplicate,frags_invalid,link_bytes_per_s,fw_tx_queue_high_water,fw_tx_pending_fragments,fw_tx_enqueue_drops,fw_tx_stack_stalls,fw_tx_emitted_bytes,fw_active_transport,fw_stats_age_s}\|null | `build_metrics_message` / `resources_to_dict` / `ws_flow_counters` / `transport_counters` | metrics cadence (4 Hz); `device_hz`/`jitter_ms` may be null. **`render_fps` vs `transform_fps` vs `browser_fps` (Task 9, 2026-08-03):** `render_fps` ticks when the broadcaster's ~30 Hz loop (`POINT_INTERVAL`) pulls a fresh frame off the latest-wins slot, REGARDLESS of display mode; `transform_fps` ticks on the reader thread every time `TransformStage.feed` completes, uncapped by presentation at all; `browser_fps` ticks only at the literal point-cloud send site (`point_cloud` display only), so unlike `render_fps` it does not over-count while `slam`/`detailed` is showing (no point cloud goes to the browser then). At a source at or below the presentation rate `render_fps`≈`transform_fps` in `point_cloud` display; a source running faster (e.g. a 90 Hz high-framerate profile) makes them diverge, and that gap is **deliberate decimation**, not loss — compare a stream's own `host_hz` (pre-transform arrival) against `transform_fps` to see transform-stage loss instead. SLAM's own submit/process/replace counters (`frames_submitted`/`frames_processed`/`frames_overwritten`) ride the `slam` message, not here — see below. `resources` was hardcoded null from Phase 1 until 2026-07-31; a `ResourceSampler` is now constructed in `main()` and passed to the `MetricsRegistry`. It carries **two scopes**: this process (`proc_*`, psutil + per-process NVML) and the **box** (`sys_cpu_percent`, `ram_used`/`ram_total`, `device_vram_*`) — the owner's question is headroom, and the ceiling belongs to the box. Device VRAM comes from `slam.gpumem.Nvml` (ctypes NVML, **no dependency**); `pynvml` is *not* installed here, so `gpu_util`/`proc_vram`/`gpu_name` are normally null and `gpu_source` is `"n/a"`. Every field except `proc_cpu_percent`/`n_cores`/`proc_rss`/`ram_total` is null-able — a GPU-less box reports absence, never a 0 that reads as free headroom. `resources` itself is null until the sampler's first sample (and forever on a state built without one, e.g. tests). `ws` (BUG-061 Part A4, 2026-08-02) is `/ws` + `/ws-mesh` connection/credit health, straight off the two live client collections (no I/O): `clients` is the `/ws` connection count (also a performance variable — see BUG-060 above); `mesh_clients` is `/ws-mesh` connections; `mesh_in_flight` is flows currently holding an un-acked mesh; `legacy_mesh_clients` counts flows that have never once acked (throttled to `LEGACY_MESH_INTERVAL_S`); `mesh_superseded_total` and `mesh_ack_timeouts_total` are cumulative counters — the number that should climb under load is `mesh_superseded_total`, never a growing queue; `mesh_ack_lag_s_max` is the slowest observed ack round-trip across flows (`null` with no acks yet); `latest_mesh_bytes` is the size of the currently cached mesh (`0` when none has been produced yet). **`transport` (`web.transport_counters`, Task 6 step 5, 2026-08-03) is `null` whenever the live source isn't `UdpSource`** (replay, CDC, or no device) — never fabricated zeros, which would read as a healthy UDP link. `frames_incomplete`/`frags_lost`/`frags_reordered`/`frags_duplicate`/`frags_invalid` predate this task (BUG-042) — `reordered` vs `lost` is what makes "did the pacer help?" answerable, since a merely-reordered datagram used to destroy its frame exactly like a dropped one. New this task: `link_bytes_per_s` is `UdpSource`'s own rolling ~1 s window over **every** reassembled frame's wire bytes (DATA+EVENT+ACK), distinct from the top-level `link_bytes_per_s` above (which sums only per-stream DATA meters) — the two can legitimately differ (EVENT/ACK traffic, or a stream `metrics.py` doesn't label). `fw_tx_queue_high_water`/`fw_tx_pending_fragments`/`fw_tx_enqueue_drops`/`fw_tx_stack_stalls`/`fw_tx_emitted_bytes`/`fw_active_transport` are the firmware's own Ethernet TX-pacer counters, opportunistically decoded from the periodic EVENT code 7 `TX_QUEUE_STATS` frame (`parse_tx_queue_stats_event`, `UdpSource._maybe_capture_firmware_stats`) — all six stay `null` until the first such event has actually arrived (older firmware, or a capture predating this feature, never sends one); `fw_active_transport` is the firmware's own belief about which transport DATA is going out on (`none`/`cdc`/`udp`), a second, independent read of the same fact `ranging.transport` reports from the host's `_transport_kind` — the two are expected to agree once both are populated, but come from different ends of the wire. `fw_stats_age_s` is wall-clock seconds since that last TX_QUEUE_STATS sample, so a stale/frozen reading is visible rather than silently repeating as if current |
| `sensor` | `have_quat`, `rot`[9 row-major], `heading`, `pressure_pa`, `temp_c`, `mag_ut`[3], `fusion`(human label), `fusion_key`(raw status), `has_mag_cal`, `pressure_hist[]`, `temp_hist[]`, `orientation_raw`{quat[4],roll_deg,pitch_deg,yaw_deg,heading_deg}, `jitter`{window_s,roll/pitch/yaw/heading/orientation→{mean_deg,p95_deg,n}}, `orientation_view`{mode,labels[3],roll_deg,pitch_deg,yaw_deg,singularity_margin_deg,near_singularity,valid,reason,yaw_offset_deg,...}, `ir_roll_deg`, `elevation_ft`, `elevation_datum_ft`, `elevation_hist[]`, `pressure_hpa`, `msl_pa`, `msl_source`(api\|stale\|fallback), `msl_age_s`, `elevation_tau_s` | `build_sensor_message` | **None (silent) on a ToF-only session**; `rot`/`heading` computed server-side so the frontend never re-derives sign/permutation matrices — see `docs/coordinate-frames.md`. `fusion` is a human-friendly label (`_FUSION_LABELS`); `fusion_key` is the raw `YawFusion.status` (`off`/`init`/`active`/`gated:no-cal`/`gated:motion`/`gated:anomaly`; `gated:gimbal` was removed by BUG-051 and no longer appears on the wire). `pressure_hist`/`temp_hist` are decimated (1 sample/2 s, ~10 min window) for sparklines. `orientation_raw`/`jitter` add full-precision numerics + noise stats alongside the existing (also raw, just rounded) `rot`/`heading` — see the frame-of-reference section above; `rot`/`heading` themselves are unchanged by this addition. `orientation_view` is the selected-mode decomposition + singularity/validity — see "Orientation decomposition modes" above; `yaw_offset_deg` echoes the applied "Zero yaw here" offset (0.0 in World mode, where it never applies) — see "Zero yaw here" below. `ir_roll_deg` is the residual in-plane gravity roll the IR pane's server-side quarter-turn snap leaves behind, CCW-positive, `null` until the display quat exists — see the `IR_IMAGE` paragraph above; `ir.js` applies it as a CSS transform and must negate it. **Elevation (owner ask, 2026-07-31):** `elevation_ft` is barometric height above sea level in feet, computed with `slam.frames.baro_height_m` — the *same* formula SLAM uses, never a second one — against `msl_pa`, and **low-passed** by a `elevation_tau_s`-second EMA (BUG-037 measured ~267 mm RMS of white noise per barometer sample, ~1.2 ft: a raw readout is unreadable). `pressure_pa`/`pressure_hist` are **kept** (the Diagnostics drawer still shows the raw Pascals); `elevation_hist` is those same decimated samples in feet, so the sparkline and the readout are one quantity. `msl_source` reports where the sea-level reference came from and is **not** cosmetic: `fallback` means no reference was fetched and the absolute height may be a couple of hundred feet off (the Δ reading is unaffected — a constant reference cancels in a difference). `elevation_datum_ft` echoes `UiState.elevation_datum_ft`, and is `null` when the readout is absolute |
| `state` | `source`(live\|view\|splat), `display`(point_cloud\|preview\|slam\|detailed), `selected_capture`, `selected_splat`, `slam_available`, `detailed`{exists,current,stale,paths,manifest?}, plus the existing color/IR/view/SLAM preferences | `_state_message` | `mode` remains a compatibility alias only; clients must use `source` + `display`. Preview is View-only and shows the selected capture's orientation-only thumbnail. `selected_splat` is the slug of the Gaussian-splat reconstruction the Splat source renders (`null` until one is picked). Detailed cache state is capture-keyed and never implies regeneration. The message is sent first on connect and after every source/display change. |
| `splats` | `items[]{slug,name,gaussians,bytes,mtime,ply_url,transform,preset,stats,imported}` (newest first) | `build_splats_message` | The offline Gaussian-splat reconstructions under `results/splats/`, for the Live/View/Splat picker (`splat.js`). `ply_url` is under the existing `/results` static mount and `transform` is the 4×4 levelling matrix — so a splat is fetched + oriented with **no new binary tag or route**. `preset`/`stats` are the build's recorded settings + quality metrics (metadata on the splat). `imported=true` marks a dir with a `.ply` but no manifest of ours (e.g. a Scaniverse export dropped in as `SomeName.ply`) — it lists (count read from the PLY header, `ply_url` uses the real filename) and the client loads it with an **identity** transform. Sent on connect, on entering the Splat source, on `list_splats`, and when a build completes. |
| `source_videos` | `{items[]{video,bytes,mtime,has_splat,has_current,splats[]{slug,name,gaussians,depth_lambda,state,ply_url}}, defaults{…SplatPreset}}` (newest first) | `build_source_videos_message` | The Splat-mode **capture viewer**: the `.mp4`/`.mov` source videos in `captures/`, each cross-referenced against the splats built from it (`state` = `current`/`stale` vs the file on disk; one video can back several builds, e.g. photometry + depth). `defaults` seeds the build-settings form. Sent on entering the Splat source, on `list_source_videos`, and when a build completes. |
| `splat_build` | `{video,name,slug,phase,fraction,done,elapsed_s,eta_s,stats?,error?}` | `SplatRunner.poll` / `build_splat` handler | Progress of an offline splat build (subprocess-isolated CLI). Mirrors `detailed` in shape; drives the `#splat-build-status` banner + the shared `metrics.resources` bars in `splat.js`. Emitted throttled while building (fraction change or ~2 s) and once on completion (`phase` `done`/`failed`). The build runs regardless of `display`, so this can arrive while `display=point_cloud`. |
| `session` | `mode`(live\|replay), `source_label`, `has_live`, `recording{active,path,elapsed_s,bytes,last_name}`, `playback{is_replay,capture_name,paused,speed_fps,loop,position,total_frames,elapsed_s,duration_s,timestamped}` | `build_session_message` | `elapsed_s`/`duration_s` come from valid monotonic DATA `FrameHeader.t_us` (TIM2); legacy captures fall back to frame count / 30 FPS. |
| `captures` | `items[]{name,bytes,mtime,frames,has_stream_9,duration_s,timestamped,has_thumb,slam,detailed_estimate}` (newest first) | `build_captures_message` | `detailed_estimate` is `{frames,seconds,calibrated,cpu_warning,note}` from the active server preset, used by the Build confirmation before any job starts; an uncalibrated preset says so instead of inventing a duration. `has_stream_9=false` means Point cloud remains usable but SLAM choices must be disabled. **§12 additions:** `has_thumb` says a tile is already cached on disk (the client requests `/thumb/<name>` either way — the route generates on demand); `slam` is `{exists,current,frames,path_m,gap_m,area_m2}` from the capture's `results/<stem>.slam.json`, or `null`. Distance and area only exist where a reconstruction does — they are outputs of the SLAM run, not of the capture — so the browser renders an em dash, never a 0. `area_m2` is absent from every manifest written before 2026-07-31 and reads as `null`; it is deliberately not backfilled (recomputing it means re-running the reconstruction). |
| `detailed` | `started?`, `estimate?`, `capture`, `phase`(frames\|extracting_mesh\|offline_only\|loading_cached\|cached\|failed), `processed`, `total`, `fraction`, `done`, `elapsed_s`, `eta_s`, `mesh_every`, `pose?[16]`, `stats?`, `mesh_seq`, `reason?` | `DetailedRunner` | The start acknowledgement carries `total` + `mesh_every` immediately, so the client can show that the click took effect before the first worker batch. Thereafter progress is server-owned: `elapsed_s` is wall-clock time since this build started and `eta_s` is the current progress-derived remaining time (`null` before one frame completes; `0` when done). `extracting_mesh` is emitted before Open3D synchronizes/extracts a progressive preview, so a temporarily unchanged frame count is explained rather than looking hung. `loading_cached` is emitted immediately while a saved PLY is read off the event loop; Detailed pauses replay because playback cannot affect its immutable reconstruction. `mesh_every` says when the next progressive geometry extraction is expected. `pose` is the latest world←camera pose (or the final sidecar `.tum` pose for a cached build), letting FPV/Mirror frame the fixed map around its scanner. The viewport keeps the progress overlay visible while each reconstructed MESH packet arrives on the existing binary tag. `cached` is a saved sidecar being viewed, not an active build. |
| `slam` | `ts`, `pose`[16], `follow{eye,center,up}`, `traj_tail[][3]`, `traj_len`, `fitness`, `rmse`, `tracking_lost`, `slam_ms`, `frames_integrated`, `mesh_seq`, `mesh_verts`, `blocks_used`, `blocks_capacity`, `blocks_configured`, `device` | `build_slam_message` (web Phase 4) | every processed frame in SLAM mode; follow eye/center/up computed server-side; traj downsampled to ≤256. `ts` and `device` are BUG-061 Part B additions (2026-08-02): `ts` is the server's wall-clock `time.time()` at build time, letting the client measure its own pose age (`slamAgeS`) without trusting frame-arrival timing to infer staleness — previously invisible now that MESH and `slam` can arrive on different sockets at different cadences. `device` is the resolved SLAM compute device string (e.g. `"CUDA:0"`) — `null`/omitted until `SlamRunner` has actually built its worker, since the build is async; before BUG-061 this was visible only in a log line. **`follow` is no longer consumed by the web client in World view** (owner choice, 2026-08-02, "track, keep my framing"): in World + Follow, `slam.js` now pans the orbit camera+target from `pose`'s own translation (`sceneApi.trackTarget`) instead of jumping to the server's fixed nose-camera `follow.eye/center/up` shot, which preserves the user's own zoom/angle across the scan. `follow` itself is unchanged on the wire and still consumed as before by FPV/Mirror (which stay scanner-relative via `setSlamPose`) — only the World-view branch stopped reading it. The three `blocks_*` fields (2026-07-31) are the TSDF hash-grid gauge — BUG-035's ceiling, which stalls map growth and collapses frame-to-model tracking ~30 frames later with **no error at all**. `blocks_capacity` is the grid's LIVE capacity (Open3D rehashes to grow); `blocks_configured` is the `[slam] block_count` the operator can actually raise. Sampled inside `Mapper` at ~4 Hz, **not per frame** — `TsdfMap.block_usage()` is a device sync on CUDA — so all three are `null` between samples, before the first one, and from a worker predating the gauge. Null means *unknown*, never *empty map* |
| `saved` | `items[]{name,bytes,mtime}` (newest first) | `build_saved_message` (web Phase 4) | `results/*.ply`; on connect and after a Save completes |
| `deleted` | `deleted[]{name,bytes,sidecars[]}`, `refused[]{name,reason}`, `bytes` | `_handle_delete_captures` (§12) | The result of one `delete_captures`. Reports what was **actually** deleted, not what was requested — every refusal carries a reason (`not found` / `currently recording` / `currently playing and there is no live source to switch to` / `delete failed`). Followed immediately by fresh `captures` + `saved` + `session` + `state`. |
| `magcal` | `collecting`, `sample_count`, `elapsed_s`, `cells`(92), `cell_counts`[92], `cell_dev_pct`[92] (null = empty cell), `view`(current\|candidate), `live_cell`, `live_dir`[3], `gaps[]{size,fraction,centroid[3],face}`, `guidance`, `guidance_axis`{axis[3],angle_deg,text,target[3],target_cell,region_size,from_face,to_face}, `live_fit`{samples,used,field_ut,std_pct,bias_pct,residual_rms_ut,spread_verdict,bias_verdict,verdict,error}, `motion`{stationary,spread_deg,window_s,n}, `has_current`, `has_candidate`, `binning`(candidate\|current\|provisional\|raw), `fit_error`, `current`/`candidate`→{samples,samples_verdict,field{mean_ut,std_ut,std_pct,min_ut,max_ut,ratio,residual_rms_ut,expected_ut,bias_pct,spread_verdict,bias_verdict,verdict},coverage{cells,occupied,empty,fraction,verdict},verdict,limited_by,reason}, `current_field_ut`, `candidate_field_ut`, `saved_path`; **on `open` only**: `cell_dirs`[92][3], `t_world_to_cv`[9] (row-major) | `magsweep.build_report` via `web._magcal_report` | **Per-tab, not broadcast**: sent at `MAGCAL_INTERVAL` (**5 Hz**) only to sockets in `state.magcal_clients` (i.e. tabs that sent `magcal/open`), plus immediately on `open` and after every state-changing action. A session with the modal closed everywhere costs nothing. `cell_dirs`/`t_world_to_cv` are deterministic constants and ride the `open` report ONLY (4490 B → 1982 B per tick, a 56% cut); the client caches them. The 30 Hz render payload is the binary `MAGPOSE` channel, not this one. See "Magnetometer sweep" below |
| `ranging` | `transport`(udp\|cdc\|replay\|none), `initialized`, `applied`{profile,ranging_mode,exposure_ms,fps,power_mode}\|null, `measured_fps`, `estimate`{profile,ranging_mode,fps,exposure_ms,power_mode,dss_enabled,frame_period_us,i3c_xfer_ms,i3c_bus_utilization_pct,i3c_airtime_left_pct,power_mw,max_range_m,min_distance_mm,transport_warning,imu_env_rate_hz,imu_env_coupled,expected_delivered_fps,ceiling_fps,warnings[],errors[],ok}\|null, `requested`{kind(profile\|manual\|null),profile,manual{ranging_mode,fps,exposure_ms,power_mode}}, `pending`, `error`, `imu_env`{initialized,applied_rate_hz,coupled,requested_rate_hz,pending,warning,error} | `web._ranging_message` | See "Ranging profiles & manual sensor control" above. Sent on connect (so a second/late-joining tab is current immediately, same as `state`/`session`/`saved`), after every `set_profile`/`set_manual_params`/`set_imu_env_rate`, and on the ~4 Hz metrics tick (to keep `measured_fps` live). `applied` is `null` until the first successful device readback (`initialized: false`) — never a guessed default. `estimate` is `null` until `applied` exists (nothing to estimate yet). `pending`/`imu_env.pending` are two INDEPENDENT flags — a manual-params edit and an IMU/env rate change never block each other |
| `event` | `code`, `detail`, `msg` | `classify_bus_line` `web.py:142` | from a device EVENT bus line. `msg` is **human-legible** (`protocol.describe_event`): every code is named (`TRIGGER_TIMEOUT`, `AUTO_WAKE_MOTION`, …) and **TX_QUEUE_STATS (code 7)** — whose payload past code+detail is three binary `u32` counters, not ASCII — is decoded (`transport=… high_water=… pending=… drops=… stalls=… emitted=…`) instead of ascii/`replace`-garbled as it was before 2026-08-05 |
| `cmd` | `label`, `status`(ok\|busy\|timeout\|error), `detail` | `classify_bus_line` `web.py:151` | command-result echo; `status` via `_cmd_status` `web.py:156` |
| `log` | `line` | `classify_bus_line` `web.py:145,153` | catch-all bus line |
| `comm` | `targets[]{id,label,addr,state(up\|down\|absent\|unknown),detail,informational?}` | `commstatus.probe` / `web._comm_probe_loop` | Top-bar server→rig link health (owner ask, 2026-08-05). Three hops: **filehub** (RavPower bridge, `ROOMSCAN_FILEHUB_IP`, default 172.17.2.57) and **scanner** (STM32, from the live UDP peer in the source label, else `ROOMSCAN_SCANNER_IP` default 172.17.2.58) are reachability by `ping`; the scanner detail is enriched with live fps when frames flow. **stlink** is informational (an ST-Link on THIS host) — `absent` is a normal, non-fault state on the Ethernet rig. Broadcast every `COMM_PROBE_INTERVAL_S` (3 s) and cached on `state.last_comm` so a fresh client gets it on connect |

`event`/`cmd`/`log` are **all produced by one classifier**, `classify_bus_line` (`web.py:123`), which
reads raw reader/dispatcher bus lines and tags them. A free-text line that happens to contain ` -> `
is gated against `command_labels` (labels we actually dispatched) so it can't be mis-tagged as a `cmd`.

## Inbound — browser → server

All inbound is JSON with a `"type"`; routed by `_handle_inbound` (`web.py:942`) for messages sent on
**`/ws`**. Unknown types warn and are dropped. The `record`/`list_captures`/`load_capture`/`go_live`/
`transport`/`rename_capture` handlers all require a `SessionController` (`ctrl is not None`) — absent
in a `--replay`-launched process with no live source.

One inbound type lives entirely outside `_handle_inbound`, on the separate `/ws-mesh` socket instead:

| type | fields | effect | handler |
|------|--------|--------|---------|
| `mesh_ack` | `seq` (echoes the acked `mesh_seq`) | **Sent on `/ws-mesh`, never `/ws`.** Frees that connection's one in-flight MESH credit (`flow.in_flight = False`, `flow.acked_ever = True`) and, if a newer mesh has been cached since, immediately sends it rather than waiting for the next broadcaster tick — see "The mesh channel — `/ws-mesh`" above. No echo; the effect is observable only via `metrics.ws` | `websocket_mesh_endpoint` |

The rest of the table below is `/ws` only:

| type | fields | effect | handler |
|------|--------|--------|---------|
| `cmd` | `name`, `param` | resolve → `CommandCode` (`resolve_command` `web.py:293`) and dispatch to the device; **in replay** publishes `"<label> -> not available in replay"` instead of a round-trip | `web.py:949` |
| `set_color` | `mode` | set point-cloud color plane (validated against `_VALID_COLOR_MODES`) → echo `state` | `web.py:1006` |
| `set_ir` | `colormap`, `freeze` | set IR colormap / freeze range (validated) → echo `state` | `web.py:1014` |
| `set_calibrated` | `enabled`(bool) | "Calibrated / Uncalibrated" toggle — apply the mode-aware flat-field reflectance correction or show raw sensor FPN. **Host-shared server state, not a per-client preference**: one `UiState` + one `TransformStage` for all clients, so it updates `stage.flatfield_enabled` on the live stage immediately (the host emits corrected reflectance to *every* client) and persists once on the host (`[calibration] flatfield_enabled`) → echo `state`. The *map* is chosen host-side by the device's current ranging mode (`stage.set_ranging_mode`, driven from `_commit_ranging_ack`), Precision and Ambient carrying different maps (2026-08-04 cross-room study). The `state` echo adds `calibrated`(bool) and `calibration_available`(bool = any map configured; the client disables the toggle + shows a note when false) | `_handle_inbound` |
| `record` | `on` | start/stop recording to `captures/web_<ts>.bin` → echo `session` + fresh `captures` | `web.py:963` |
| `list_captures` | — | broadcast a fresh `captures` | `web.py:971` |
| `rename_capture` | `name`, `from?` | rename the most-recently-stopped take (`SessionController._last_recorded_name`) to `name` (`.bin` appended if omitted) → echo `session` + fresh `captures`. Rejected (silently, from the client's view — a `log` line is published) if there is nothing pending, `name` is empty/traversal/a path separator, or the target already exists; the file keeps its old name and `session.recording.last_name` is unchanged, which is how the naming modal detects failure. **`from` (§12) is optional and additive**: absent ⇒ byte-identical to the above (the post-record modal and the `rig_*` MCP tools depend on it); present ⇒ rename *that* capture, from the View-page browser (`SessionController.rename_capture`). The named file's `results/<stem>.{ply,tum,slam.json}` sidecar and its cached thumbnail move with it, and `manifest["capture"]["name"]` is patched — without the patch `sidecar_status` reports `stale` forever and the UI offers a rebuild that would recompute a byte-identical result. **Refused** if the source is the actively-recording file (the `Recorder` holds an open write handle). The currently-**replaying** capture is handled rather than refused: POSIX `rename(2)` keeps `FileSource`'s open fd valid, so it is renamed under the lock and `replay_path` repointed (on Windows this raises `OSError`, which is caught and reported as a refusal) | `web.py` |
| `load_capture` | `name` | swap reader → replay (`sanitize_capture_name` → basename-only, `.bin`, must-exist; off-loop via `to_thread`) → echo `session` | `web.py:974` |
| `delete_captures` | `names[]`, `sidecars?` (default `true`) | **The only destructive action in the app** (§12). Order is load-bearing: `names` must be a list and the batch is capped at `_MAX_DELETE_BATCH` (200); each name resolves through `sanitize_capture_name`, and an unresolvable entry is logged and **dropped**, never fatal to the rest of the batch; the **currently-replaying** capture is switched away from FIRST (`switch_to_live` + SLAM reset + `source=live`/`display=point_cloud`/`selected_capture=null`) because unlinking under a running replay reader gives a broken stream, not a clean state — and if there is no live source to switch to (a `--replay`-launched process) *that one file* is refused; the **actively-recording** file is refused outright; only then is anything unlinked, off-loop. Sidecars and thumbnails go with the capture by default (`sidecars: false` keeps them): they are derived artifacts of a file that no longer exists, and an orphaned `.ply` would keep appearing in the Saved-maps list pointing at nothing. Replies `deleted`, then `captures` + `saved` + `session` + `state` | `_handle_delete_captures` |
| `set_browser` | `sort?`(recent\|name\|size\|duration), `view?`(grid\|list), `thumbs?`(bool) | View-page browser prefs; persisted to `[viewer] web_browser_sort` / `web_browser_view` / `web_browser_thumbs` → echo `state`. Validated with **no partial mutation** on a bad enum (the `set_idle`/`set_orientation` shape): a message carrying one valid and one invalid field changes nothing. Sorting is applied **client-side** off the already-broadcast `captures` array, so changing it costs no directory rescan; `thumbs: false` renders tiles with no `<img>` at all, so a slow box never pays | `web.py` |
| `go_live` | — | swap reader → live proxy → echo `session` | `web.py:982` |
| `transport` | `action`(pause\|resume\|speed\|loop\|restart\|seek), `value` | playback control; `seek`/`restart` run off-loop via `to_thread` → echo `session` | `web.py:986` |
| `set_view` | `colormap?`(turbo\|gray), `point_size?`, `point_size_auto?`, `see_through?`, `surface?`, `surface_mode?`(grid\|spatial), `surface_threshold?`, `view_mode?`(world\|fpv\|mirror), `cam_distance?`, `cam_height?`, `cam_rotation?`, `cam_reset?`, `orbit_mode?`(continuous\|oscillate), `orbit_amplitude?` | Shared viewport display: colormap, point size, surface interpolation toggle + settings, camera view, camera framing; all optional, only provided fields update → echo `state`. `view_mode` applies to Point cloud, Preview, SLAM and Detailed: it picks the frame the live cloud is shipped in (`view_rotation`, see "Frame of reference" above), moves the map camera from the scanner pose for SLAM/Detailed, and flips the preview thumbnail in Mirror. It is persisted to `[viewer] web_view_mode`; an unknown value is rejected+logged, never stored. The `cam_*` fields edit the **currently selected** mode's framing only (the three sliders show that mode, so there is exactly one thing they can mean) — handled *after* `view_mode`, so a combined message lands on the mode being switched to, not the one being left. Out-of-range or non-numeric values are dropped, keeping the current value. `cam_reset` restores that one mode's defaults. Persisted as nine flat floats, `[viewer] web_cam_<mode>_{distance_m,height_m,rotation_deg}`. `orbit?`/`orbit_speed?` (−60…60 °/s) drive the world-view auto-orbit → `[viewer] web_orbit_enabled` / `web_orbit_speed_deg_s`; an out-of-range or non-numeric speed is dropped, keeping the current one (0 is legal — it parks the orbit without disabling it). `orbit_mode?`/`orbit_amplitude?` (5…180°, default 45) pick continuous-vs-oscillate and the oscillate half-swing — see "Oscillate mode" above — → `[viewer] web_orbit_mode` / `web_orbit_amplitude_deg`; an unknown mode is rejected+logged (no mutation), an out-of-range or non-numeric amplitude is dropped, keeping the current one. `point_size_auto` scales each point by its range from the sensor (a client-side shader uniform, no wire change) so every zone subtends the same solid angle; with it on, `point_size` means the size at **1 m of range** rather than metres. `see_through?` (0…1, default 0 = off, persisted to `[viewer] web_see_through`) is the x-ray strength: the client redraws the point cloud / surface / SLAM mesh with the depth test **inverted** (`GreaterDepth`, `depthWrite` off), so only the fragments that lost the depth test — i.e. exactly the occluded ones — are blended back over their occluder at that alpha. Deliberately not a plain material opacity, which would fade every surface against the background including the ones hiding nothing; un-occluded geometry is byte-identical to the opaque render. At 0 the extra objects are `visible = false`, so the default costs no draw calls. Out-of-range or non-numeric values are dropped, keeping the current one | `web.py` |
| `set_idle` | `enabled?`, `level?`(soft\|hard) | sensor auto-idle prefs (laser-wear reduction); persisted to `[viewer]` → echo `state`. Disabling cancels any armed idle timer. The idle itself is driven server-side off per-connection *activity* (see below), not by an inbound message | this change |
| `idle_state` | `active`(bool) | Client-side activity heartbeat/park signal (`idle.js`, 2026-08-03), keyed by the `client_id` query param shared across `/ws`/`/ws-mesh`. `active: true` — sent on user activity and as a ~30 s heartbeat while engaged — refreshes this connection's activity clock and fast-paths a wake if it was the one bringing the count back above zero. `active: false` parks immediately, without waiting out `sensor_idle_activity_timeout_s`, and fast-paths the idle decision. No `state` echo — this is per-connection bookkeeping, not shared UI state. A connection that never sends this (an MCP probe, an old cached client) still ages out on its own via the periodic reconcile tick below | `web.py` |
| `set_mode` | `mode`(realtime\|slam) | switch top-bar mode; arms/disarms the `SlamRunner` off-loop (lazy worker build) → echo `state` | web Phase 4 |
| `set_source` | `source`(live\|view\|splat) | switch the top-level Live/View/Splat source; View reuses the selected capture → state + session. **`splat` leaves the reader untouched** (a splat is static geometry the client draws, not a device/replay stream): it just sets the source, resets any live SLAM, and echoes `splats` + `state` | Live/View |
| `load_splat` | `slug` (or `name`) | pick which offline Gaussian-splat reconstruction the Splat source renders. Validated against the slugs actually under `results/splats/` (basename-only). Sets `selected_splat` and switches `source` to `splat` → echo `state`. The client (`splat.js`) fetches the splat's `.ply` over HTTP from the `/results` mount | Splat |
| `list_splats` | — | request a fresh `splats` list → echo `splats` | Splat |
| `list_source_videos` | — | request a fresh `source_videos` list (the Splat capture viewer) → echo `source_videos` | Splat |
| `build_splat` | `{video,name,params?,force?}` | start an offline splat build for a `captures/` video. `video` is validated by `sanitize_video_name` (basename, `.mp4`/`.mov`, must exist); `params` is **allow-listed to `SplatPreset` field names** (they become CLI flags); `name` is the display name / slug. Runs the CLI as a **subprocess** (a gsplat/COLMAP/CUDA-OOM crash cannot take down the server), **one at a time** (8 GB card) — refused with `{started:false,reason}` while another splat build (incl. the detached CLI) holds the GPU. Echoes a `splat_build` result + `source_videos`; progress then streams as `splat_build` | Splat |
| `set_display` | `display`(point_cloud\|preview\|slam\|detailed) | select the shared display. Preview is View-only and requires a selected capture; it shows the same orientation-only thumbnail as the library card. Detailed is View-only; SLAM choices reject a capture without stream 9. **Entering Live SLAM (`display == "slam"` and `source == "live"`) also starts a recording**, and leaving it stops one (owner ask, 2026-07-31): a live scan is unrepeatable — the same reasoning that kept Live SLAM's one-shot Save (BUG-043) — and a recording makes it replayable and Detailed-reconstructable afterwards. Only the take *SLAM itself* started is stopped (`SessionController._auto_recording`); a manually started recording is never ended by a display switch. Echoes `session` + fresh `captures` when it acts, so the client's existing falling-edge latch pops the rename modal. A no-op — never an error — with no live source, while another take is running, or with `[viewer] slam_auto_record = false`. `set_mode`, `set_source`, `go_live` and `load_capture` all stop it on the way out | Live/View |
| `generate_detailed` / `regenerate_detailed` | — | start an explicit server-owned Detailed sidecar build; it never mutates the capture. The client switches to Detailed display so the progressive MESH packets and the visible progress/ETA overlay appear immediately | Live/View |
| `slam_opt` | `trajectory?`, `walls?`(solid\|split), `follow?` | SLAM display toggles → echo `state` | web Phase 4 |
| `save` | — | write full-res `mapper.mesh()` + trajectory → `results/web_<ts>.ply`/`.tum` (off-loop); toast + `saved` echo. **Live SLAM only** (`source == "live"` and `display == "slam"`; owner decision 2026-07-31): a live scan is unrepeatable, so if Record wasn't running the frames are gone the moment the map is dropped, and that one-shot export has to stay. Replay SLAM is deliberately preview-only and refuses with a reason — **Detailed** is the capture-keyed sidecar there, and it writes its own artifacts. Refusals are bus lines, never a silent no-op | web Phase 4 / Live/View |
| `reset_fusion` | — | reset the `YawFusion` filter state (clears accumulated yaw correction); the heading snaps fresh on the next valid magnetometer sample. Publishes `"heading fusion reset"` on the bus | sensors card |
| `set_orientation` | `mode?`(zyx\|zxy\|boresight\|world), `labels?`[3] | select the orientation decomposition mode and/or rename the 3 axis-label slots (validated: unknown mode is rejected+logged, labels are trimmed/length-capped/defaulted per-slot via `_sanitize_axis_labels`); persisted to `[viewer]` → echo `state` | `web.py` |
| `zero_yaw` | — | capture the CURRENTLY DISPLAYED mode's `yaw_deg` at the current attitude and store the `graft_yaw` delta that zeroes it (`UiState.yaw_offset_deg`); persisted to `[viewer]` → echo `state`. **No-op (logged) in World mode** — its heading is absolute magnetic north, not offsettable — and when there is no orientation yet | `web.py` |
| `magcal` | `action`(open\|close\|start\|stop\|reset\|save\|discard\|view), `cal?`(current\|candidate) | magnetometer sweep/calibration modal (`web._handle_magcal`): `open`/`close` subscribe this socket to `magcal` reports; `start`/`stop` bracket a collection (`stop` also runs `magcal.fit_ellipsoid` → a **candidate**, never saved); `save` writes the candidate to `ViewerConfig.mag_cal_path` and hot-reloads it (`web.install_mag_calibration`); `discard` drops the candidate but KEEPS the samples; `reset` clears the samples; `view` picks which calibration colours the map. Unknown actions are rejected+logged. Every action echoes a fresh `magcal` | `web._handle_magcal` |
| `clear_yaw_offset` | — | reset `yaw_offset_deg` to 0.0 (back to raw SFLP yaw); persisted to `[viewer]` → echo `state` | `web.py` |
| `set_elevation_datum` | `on` (**bool, strictly**) | capture the current **smoothed** elevation into `UiState.elevation_datum_ft` (`on: true`) so the Sensors card reads change-since instead of absolute height, or clear it (`on: false`); persisted to `[viewer] elevation_datum_ft` → echo `state`. A non-bool `on` is rejected+logged with **no mutation** (a truthy string must not set a datum, and a falsy one must not clear an existing one). With no barometer reading yet it refuses with a bus line rather than capturing a datum of 0 ft. Smoothed, not raw, on purpose: a datum taken from one sample would bake ~1.2 ft of noise in as a constant offset for the whole session — the same mistake BUG-037 found in the SLAM height datum | `web.py` |
| `set_profile` | `profile`(stability\|precision\|high_framerate\|manual) | **Host-composed (2026-08-05):** the server resolves the preset to its `profiles.PRESETS` entry and applies it as ONE `SET_MANUAL_PARAMS` (cmd 9, whose ACK already carries the config) plus the preset's own IMU/env rate (cmd 11) — NOT the firmware `SET_RANGING_PROFILE`, whose fixed table cannot carry an IMU rate. The device readback is re-labelled back to the preset by `_match_profile_id`, so `applied.profile` is the preset. Unknown/non-preset name: rejected+logged, no device round-trip. Rejected client-**and**-server-side (no-op, logged) while another ranging command is already `pending`; the IMU half is skipped if the ranging half errors or is rejected | `web._set_profile` |
| `set_manual_params` | `ranging_mode`(ambient\|precision), `fps`(1-100), `exposure_ms`(1-16), `power_mode`(ulp\|lp\|regular) | validated through `roomscan.profiles.validate_manual_params` FIRST (a rejected request never touches the device); on success, the WHOLE candidate goes as **one** atomic `SET_MANUAL_PARAMS` (cmd 9) — never four independent operations — and commits `applied_*` straight from that ACK (already the readback) → echo `ranging`. Same pending-rejection rule as `set_profile` | `web._set_manual_params` |
| `set_imu_env_rate` | `rate_hz`(0 = coupled, else 1-480) | independent pending command from ranging (own `ImuEnvRateState.pending`): validated through `profiles.validate_imu_env_rate` (reports, doesn't silently drop, a requested rate above the 60 Hz sensor-hub cycle — stream 10/env will sub-sample), then one `SET_IMU_ENV_RATE` (cmd 11) off the event loop → echo `ranging`. **Live as of Task 7** (`3f4b307`, hardware-gated): the ACK's own `applied` field is the readback, and `applied_rate_hz` is committed only from that ACK — a device still on pre-Task-7 firmware (`UNKNOWN_CMD`) reports it as `imu_env.error` and leaves `applied_rate_hz` untouched rather than adopting the request as if confirmed | `web._set_imu_env_rate` |

`view_colormap` is presentation state for every 3D display: Point cloud, SLAM,
and Detailed. SLAM/Detailed re-map their already-shaded mesh scalar in the
browser, so the palette changes without altering the reconstruction or its
saved sidecar.

## Invariants (hold when adding a message)

- **One-way state flow.** The server is authoritative: inbound control mutates server state, then the
  server **echoes** the resulting `state`/`session`/`captures`. The frontend drives *all* active/disabled
  UI from that echo, never optimistically — so multiple tabs stay in sync. New control types must echo.
- **Untrusted inbound.** Every inbound handler validates before acting (`_VALID_COLOR_MODES`,
  `_VALID_IR_COLORMAPS`, `sanitize_capture_name`, `resolve_command` returning None on unknown). A new
  inbound type parsing a client-supplied string/path/enum does the same — reject, log, drop; never trust
  the field. (Retro checklist: adversarial/malformed input case for any new parser of untrusted bytes.)
- **Server-side math stays server-side.** `sensor.rot`/`heading` are computed in Python so the
  sign/permutation matrices live in exactly one place (`docs/coordinate-frames.md`). A SLAM
  trajectory/pose message should follow suit — send world-frame poses the browser renders verbatim,
  don't ship raw quats + matrices for JS to re-multiply.
- **Silent-when-empty.** `build_sensor_message` returns None (broadcaster sends nothing) when there's no
  sensor data. A SLAM message with no map yet should likewise stay silent rather than send an empty hull.
- **Off-loop for blocking work.** Anything that scans a file or joins a thread (`load_capture`, `seek`,
  `restart`) is dispatched via `asyncio.to_thread` so the single broadcaster/event-loop never stalls.
  A SLAM integrate/raycast that blocks belongs off-loop the same way.
- **Server-driven sensor auto-idle is activity-based, not connection-based (2026-08-03).** To spare
  the ToF laser (VCSEL) — only the laser; IMU/env keep streaming while idled, see `docs/protocol.md`
  v1 rev 2026-08-03 (b) — the server watches per-connection *activity* (`_active_viewer_count`), not
  raw `len(state.clients)`: a connection counts as active only while it has sent something recently
  (`idle_state` heartbeat/park, or any other inbound message) within `sensor_idle_activity_timeout_s`,
  or while a live recording is running. `_viewer_left`/`_viewer_arrived` fire on real connect/disconnect
  as a fast path; a periodic reconcile tick (same cadence as the activity timeout) is what actually ages
  out a connection that goes silent without ever cleanly disconnecting — the failure the old
  connection-count model missed entirely (an agent's stale probe, or a tab left open unattended, used to
  block auto-idle forever). It acts only on a live device we're streaming from (not a replay excursion —
  the command-ACK path there is the file, not the device), and a boot-time `startup-wake` guarantees a
  device a prior server left idled resumes streaming. Parking (via `idle_state`) also excludes that
  `client_id` from the broadcaster's POINT_CLOUD/IR_IMAGE sends and from `/ws-mesh`'s pump — releasing
  the *server's* load from a parked client, not just stopping that client's own rendering. `set_idle`
  tunes enable/depth; `idle_state` is the activity signal itself.
- **Persisted display state seeds the same `state` echo.** Web Phase 5: durable UI prefs live in
  `roomscan.toml` [viewer]`, seed `UiState` at boot (`web.ui_from_config`) and are written back on change
  (`web._persist_ui`, best-effort, reload-then-save). A new *durable* toggle adds a `ViewerConfig` field +
  the map on both sides; a purely *ephemeral* one stays out of the file. Persistence never adds a message —
  it rides the existing connect-time `state` echo, so the one-way-flow invariant is untouched.
- **Moving a binary tag to a different socket is a breaking wire change.** BUG-061 moved `TAG_MESH`
  from `/ws` to `/ws-mesh`; that is not a payload-format change (`pack_mesh` didn't move) but it still
  requires the client and every socket consumer to change in the **same commit** — `ws.js` (second
  socket + demux), `slam.js` (ack timing), and `RigSession` in `host/src/roomscan/mcp_server/session.py`
  (the MCP client, which reconnects both sockets and auto-acks). A server shipped without the matching
  client update silently stops delivering that tag to old clients; `session.py`'s `_connect_mesh` is
  deliberately best-effort for exactly this reason (an older server with no `/ws-mesh` route must not
  take down the main session). Treat any future tag-to-socket move the same way the `protocol-change`
  skill treats a wire-format change: spec + both ends + this doc, in lockstep.

## Outside the socket — `GET /thumb/{name}` (capture thumbnails, §12)

A capture's thumbnail PNG, generated on demand. **Deliberately HTTP, not a binary `/ws` tag.** An
`<img loading="lazy">` gives viewport-paced fetching, browser disk caching, per-image cancellation and
parallelism for free; a `TAG_THUMB` would need request/response correlation bolted onto a channel that
is strictly one-way server push, and ~34 × ~25 KB PNGs would interleave with the 30 Hz `POINT_CLOUD`
and add head-of-line delay to the live render. `/results` is the precedent for file-shaped payloads.

| Endpoint | Method | Returns | Notes |
|---|---|---|---|
| `/thumb/{name}` | GET | `image/png`, or 404 | `sanitize_capture_name` is the **whole security surface** (basename-only, `.bin`, must exist under the captures dir) — the same one `load_capture` uses. Generated in `asyncio.to_thread` behind a `Semaphore(2)` plus an in-flight future map, so a grid of lazy `<img>` loads cannot starve `load_capture`/`seek`/`_broadcast_captures` in the shared default executor. Cached at `thumbs/<stem>__<size>_<mtime_ns>.png`. **Never 500s** (the `/api/*` precedent): unknown name, hostile name, truncated capture and render failure are all `404` + `Cache-Control: no-store`. A hit is `public, max-age=31536000, immutable` — safe because the capture's identity is in both the cache filename and the client's `?v=<mtime>`. |
| `/capture_video/{name}` | GET | the source video, or 404 | Streams a `captures/` phone video for the Splat capture viewer's in-browser `<video>` preview. `sanitize_video_name` is the **whole security surface** (basename-only, `.mp4`/`.mov`, must exist). A narrow route, **not** a `captures/` static mount — no directory listing, one surface. `FileResponse` is **Range-capable**, so the browser seeks/streams without the server buffering the (hundreds-of-MB) file into memory. Miss → `404` + `no-store`. |

Rendering (`roomscan.thumbs`, a pure module that never imports `web`): `sample_capture_frames` walks
**headers only**, then seeks to and reads **only** the ~40 chosen depth frames, the CALIB governing
each, and the IMU_QUAT preceding each — so a 1 GB capture costs the same payload bytes as a 10 MB one
(proved in `test_thumbs.py` with a counting wrapper on the file object, not by inspection). The image
is a **rotational sweep, not a map**: with no translation estimate every sampled frame is deprojected
about one origin, so it shows where the scanner was *aimed*. The tile's tooltip says exactly that.
A capture with no stream 9 falls back to one colorized depth raster from ~10% in.

## Outside the socket — `/api/*` maintenance endpoints

Two owner actions are **plain HTTP POSTs, not `/ws` messages** (2026-07-31). They act on the server
process and its host rather than on instrument state, and `/api/restart` could not use the socket
anyway — the socket is what it destroys.

| Endpoint | Method | Returns | Notes |
|---|---|---|---|
| `/api/bridge-mode` | POST | `{ok, returncode, output, error}` | Runs `filehub-bridgemode.sh` (RavPower FileHub → transparent bridge). Never raises: missing script, missing `expect`, timeout and non-zero exit all come back as a readable `error`/`output`. |
| `/api/restart` | POST | `{ok, restart_in_s, argv}` | Spawns a detached `sh -c 'sleep N; exec …'` child, then `os._exit(0)`. `ws.js`'s reconnect-with-backoff brings the UI back on its own. |

Both are **POST-only** so a prefetch, crawler or browser refresh can never fire them, and both are
**unauthenticated by owner decision** while the server binds `0.0.0.0` — anyone who can reach port
8000 can trigger them. Acceptable on the isolated rig LAN; gate on `request.client.host` if that
ever changes. Driven by `admin.js`, which takes the hub only to watch `conn` (so the Restart button
can clear its busy state when the socket returns) and otherwise stays off the protocol entirely.

Bridge Mode is behind a confirm modal on purpose: the script is **step 3 of a 4-step physical
sequence** (unplug Ethernet → power-cycle the FileHub → run script → replug), and running it out of
order makes the FileHub treat its LAN port as WAN. The modal states the ordering rather than letting
one click imply "fix it".

## Frontend consumers

12 vanilla ES modules under `host/src/roomscan/static/`, wired through a hub in `app.js` (no build step,
no framework), plus `layout.js` — a deliberately *classic* (non-module) script owning the two-dock
column-wrapping layout and the diagnostics panel, so both survive a failure of the module graph
(see `docs/web-ui-testing.md` -> "The dock layout"). Binary tags are demuxed in `ws.js`; each JSON `type` is routed to its module
(`metrics.js`, `sensors.js`, `capture.js`, `browser.js`, `controls.js`, `ir.js`, `slam.js`, `magcal.js`, …). `slam.js` (web Phase 4)
renders the SLAM mesh/trajectory into `scene.js`'s single Three.js context (via a handle `app.js` passes
it) and drives the follow camera — no second WebGL context. Saved maps download from a `/results/<name>`
static mount. Element ids for driving the UI headlessly are catalogued in `docs/web-ui-testing.md`.
