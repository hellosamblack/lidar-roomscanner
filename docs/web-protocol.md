# The `roomscan-web` `/ws` protocol

The desktop panel talks Python-to-Python; the web app talks **browser ↔ FastAPI over a single
WebSocket** at `/ws`. That contract has grown one message type at a time across Web Phases 1–3, and
it lives entirely inside `host/src/roomscan/web.py` builder functions — there is **no enum registry**
the way the *binary wire* protocol has `docs/protocol.md` + `protocol.py`. This doc is that missing
index: every `/ws` frame, its shape, and where it is built/consumed, so the next phase (SLAM
trajectory + mesh) has one place to hook into.

This is the **app protocol** (host ↔ browser). It is unrelated to the **device wire protocol**
(MCU ↔ host, `docs/protocol.md`). The two only meet at the reader thread: device frames are decoded,
transformed, and *re-encoded* into these `/ws` messages.

**Governing design specs** (the `§N` refs in `web.py` docstrings point at Phase 1's):
`docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md` (binary tags + metrics/state/cmd/log/event),
`.../2026-07-16-web-phase2-sensors-design.md` (`sensor`),
`.../2026-07-16-web-phase3-recording-playback-design.md` (`session`/`captures` + inbound transport).

## Framing

One socket, two encodings, distinguished by WS frame opcode:

- **Binary frames** — high-rate render payloads. First 4 bytes are a little-endian `u32` **tag**;
  the frontend switches on it (`app.js`). Tags: `TAG_POINT_CLOUD = 1`, `TAG_IR_IMAGE = 2`
  (`web.py:75-76`). Add new binary tags here and keep them contiguous.
- **Text frames** — JSON objects, always with a `"type"` string field. Everything that isn't a
  per-frame render payload (metrics, sensors, UI state, control echoes, logs) is JSON.

All numeric binary fields are little-endian. JSON numbers go raw over the wire; the frontend formats
for display (units, precision).

## Outbound — server → browser

### Binary

| tag | name | layout | built by |
|-----|------|--------|----------|
| 1 | `POINT_CLOUD` | `u32 tag · f32[3N] positions · f32[3N] colors` (positions then colors, concatenated) | `pack_point_cloud` `web.py:204` |
| 2 | `IR_IMAGE` | `u32 tag · u16 width · u16 height · u8[w*h*3] RGB` | `pack_ir_image` `web.py:212` |
| 3 | `MESH` | `9×u32 header (tag, mesh_seq, flags, then 6 counts) · per-submesh f32 pos·f32 col·u32 idx · floor f32 pos·u32 line-idx` | `pack_mesh` (web Phase 4) — a SLAM `MeshPacket`; flags bit0=decimated, bit1=walls_split; emitted on the mesh-throttle cadence only |

`POINT_CLOUD` goes out every broadcast tick (so late joiners see data within ~36 ms);
`IR_IMAGE` rides a slower cadence (`web.py:855`, `:869`).

### JSON (`type` → shape)

| type | key fields | built by | notes |
|------|-----------|----------|-------|
| `metrics` | `render_fps`, `streams[]{stream_id,label,device_hz,host_hz,bytes_per_s,jitter_ms}`, `link_bytes_per_s`, `resources`(null), `drops`, `gaps` | `build_metrics_message` `web.py:221` | metrics cadence; `device_hz`/`jitter_ms` may be null |
| `sensor` | `have_quat`, `rot`[9 row-major], `heading`, `pressure_pa`, `temp_c`, `mag_ut`[3], `fusion`, `pressure_hist[]`, `temp_hist[]` | `build_sensor_message` `web.py:246` | **None (silent) on a ToF-only session**; `rot`/`heading` computed server-side so the frontend never re-derives sign/permutation matrices — see `docs/coordinate-frames.md` |
| `state` | `color_mode`, `ir_colormap`, `ir_freeze` | `_state_message` `web.py:312` | echoed after every `set_color`/`set_ir` (one-way flow) |
| `session` | `mode`(live\|replay), `source_label`, `has_live`, `recording{active,path,elapsed_s,bytes}`, `playback{is_replay,capture_name,paused,speed_fps,loop,position,total_frames}` | `build_session_message` `web.py:400` | broadcast on change **and** on the metrics cadence (so timer/position tick) |
| `captures` | `items[]{name,bytes,mtime}` (newest first) | `build_captures_message` `web.py:354` | on connect, on `list_captures`, after a recording stops |
| `slam` | `pose`[16], `follow{eye,center,up}`, `traj_tail[][3]`, `traj_len`, `fitness`, `rmse`, `tracking_lost`, `slam_ms`, `frames_integrated`, `mesh_seq`, `mesh_verts` | `build_slam_message` (web Phase 4) | every processed frame in SLAM mode; follow eye/center/up computed server-side; traj downsampled to ≤256 |
| `saved` | `items[]{name,bytes,mtime}` (newest first) | `build_saved_message` (web Phase 4) | `results/*.ply`; on connect and after a Save completes |
| `event` | `code`, `detail`, `msg` | `classify_bus_line` `web.py:142` | from a device EVENT bus line |
| `cmd` | `label`, `status`(ok\|busy\|timeout\|error), `detail` | `classify_bus_line` `web.py:151` | command-result echo; `status` via `_cmd_status` `web.py:156` |
| `log` | `line` | `classify_bus_line` `web.py:145,153` | catch-all bus line |

`event`/`cmd`/`log` are **all produced by one classifier**, `classify_bus_line` (`web.py:123`), which
reads raw reader/dispatcher bus lines and tags them. A free-text line that happens to contain ` -> `
is gated against `command_labels` (labels we actually dispatched) so it can't be mis-tagged as a `cmd`.

## Inbound — browser → server

All inbound is JSON with a `"type"`; routed by `_handle_inbound` (`web.py:942`). Unknown types warn
and are dropped. The `record`/`list_captures`/`load_capture`/`go_live`/`transport` handlers all require
a `SessionController` (`ctrl is not None`) — absent in a `--replay`-launched process with no live source.

| type | fields | effect | handler |
|------|--------|--------|---------|
| `cmd` | `name`, `param` | resolve → `CommandCode` (`resolve_command` `web.py:293`) and dispatch to the device; **in replay** publishes `"<label> -> not available in replay"` instead of a round-trip | `web.py:949` |
| `set_color` | `mode` | set point-cloud color plane (validated against `_VALID_COLOR_MODES`) → echo `state` | `web.py:1006` |
| `set_ir` | `colormap`, `freeze` | set IR colormap / freeze range (validated) → echo `state` | `web.py:1014` |
| `record` | `on` | start/stop recording to `captures/web_<ts>.bin` → echo `session` + fresh `captures` | `web.py:963` |
| `list_captures` | — | broadcast a fresh `captures` | `web.py:971` |
| `load_capture` | `name` | swap reader → replay (`sanitize_capture_name` → basename-only, `.bin`, must-exist; off-loop via `to_thread`) → echo `session` | `web.py:974` |
| `go_live` | — | swap reader → live proxy → echo `session` | `web.py:982` |
| `transport` | `action`(pause\|resume\|speed\|loop\|restart\|seek), `value` | playback control; `seek`/`restart` run off-loop via `to_thread` → echo `session` | `web.py:986` |
| `set_mode` | `mode`(realtime\|slam) | switch top-bar mode; arms/disarms the `SlamRunner` off-loop (lazy worker build) → echo `state` | web Phase 4 |
| `slam_opt` | `trajectory?`, `walls?`(solid\|split), `follow?` | SLAM display toggles → echo `state` | web Phase 4 |
| `save` | — | write full-res `mapper.mesh()` + trajectory → `results/web_<ts>.ply`/`.tum` (off-loop); toast + `saved` echo. Disabled in real-time / empty map | web Phase 4 |

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

## Frontend consumers

9 vanilla ES modules under `host/src/roomscan/static/`, wired through a hub in `app.js` (no build step,
no framework). Binary tags are demuxed in `ws.js`; each JSON `type` is routed to its module
(`metrics.js`, `sensors.js`, `capture.js`, `controls.js`, `ir.js`, `slam.js`, …). `slam.js` (web Phase 4)
renders the SLAM mesh/trajectory into `scene.js`'s single Three.js context (via a handle `app.js` passes
it) and drives the follow camera — no second WebGL context. Saved maps download from a `/results/<name>`
static mount. Element ids for driving the UI headlessly are catalogued in `docs/web-ui-testing.md`.
