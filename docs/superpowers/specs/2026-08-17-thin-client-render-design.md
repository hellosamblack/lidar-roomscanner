# LiDAR thin-client render — server side

**Status:** Approved for planning
**Date:** 2026-08-17
**Repo:** `lidar-roomscanner` (this repo)
**Companion spec:** `CrowPanelProp` repo,
`docs/superpowers/specs/2026-08-17-lidar-thin-client-crowpanel-design.md` (client side —
a cassette-futurism prop communicator built on a CrowPanel ESP32-P4 7" display). The
**Protocol contract** section below is duplicated verbatim in both specs — it's the shared
interface, not a cross-reference — keep them in sync if it changes.

## Goal

Serve a live, already-rendered view of the point cloud / SLAM / IR feed to a resource-
constrained embedded client (an ESP32-P4 panel with no GPU, no 3D pipeline, and no image
codec) that can only display raw pixels and send small JSON commands back.

## Non-goals

- No 3D data (points, mesh, triangles) is ever sent to this client — only rendered
  raster frames. The existing `/ws`/`/ws-mesh` protocol (browser clients) is unaffected
  and unchanged by this work.
- No image compression (JPEG/PNG) in v1 — raw RGB565 only. See "Open questions" for the
  deferred codec path, which would be server-side JPEG encoding once the client can
  decode it.
- No exposing ranging-profile/manual-parameter device control (`set_profile`,
  `set_manual_params`) through this endpoint in v1 — those stay reachable only via the
  existing `/ws` protocol from the browser.

## Architecture

**Thin-client model:** this server does all 3D rendering. The client (CrowPanel) is a
remote display + input relay only.

**`host/src/roomscan/thin_render.py`** (new module) — owns:

- An Open3D `OffscreenRenderer` (Open3D is already a dependency here for SLAM meshing;
  chosen over driving a headless browser against the existing Three.js UI via Playwright,
  which would need a full Chromium instance per stream and is too slow for a real-time
  orbit-control loop).
- Camera-orbit state (yaw/pitch/zoom) per connected thin client, updated by incoming
  `thin_orbit` commands.
- Current render mode (`point_cloud` / `slam` / `ir`) per connected thin client, updated
  by `thin_mode`.
- A render loop that, on each tick, pulls the latest data already computed by the
  existing pipeline — the same `pts`/`colors` arrays that feed `pack_point_cloud`
  (`web.py:1085`), the latest SLAM mesh, or the latest IR frame (`ir_image.py`) — applies
  the orbit state, renders to 480×480 RGBA, and converts to RGB565. No new sensor-side
  computation; this reuses data the existing pipeline already produces.

**New `/ws-thin` endpoint** in `web.py`, with its own broadcast loop/task (mirroring the
existing point-cloud broadcaster's task structure, `POINT_INTERVAL`-style ticking) so it
never blocks the main reader thread or the existing `/ws`/`/ws-mesh` clients. Runs
independently per-connection camera/mode state (multiple thin clients could theoretically
connect with different orbit angles).

**New mDNS advertisement.** This server does not advertise itself via mDNS today — the
existing `zeroconf` dependency in `sources.py` is used for discovering the MCU/travel
router, not for advertising the web server. Add a `zeroconf.ServiceInfo` registration for
the web server itself, started alongside the existing FastAPI/uvicorn startup, so the
CrowPanel (or any future thin client) can find it without a hardcoded IP.

## Protocol contract (`/ws-thin`)

New WebSocket endpoint: **`GET /ws-thin`**. Independent of the existing `/ws` and
`/ws-mesh` protocol (`docs/web-protocol.md`) — a thin client never receives
`POINT_CLOUD`/`MESH`/`IR_IMAGE` binary tags or the full JSON message surface. This keeps
the embedded client's parsing trivial and avoids adding a subscribe/filter layer to the
existing sockets.

### Binary frame (server → client): `THIN_FRAME`

Fixed 480×480 for v1 (no resolution negotiation — see "Open questions").

```
u32 tag        = 1
u16 width      = 480
u16 height     = 480
u8  pixels[width * height * 2]   // RGB565, little-endian, row-major
```

Target cadence: **~10 fps** (~460 KB/frame → ~4.6 MB/s per connected thin client). If a
client falls behind (its WS send buffer growing), drop to the freshest frame rather than
queueing — same decimation policy as the existing point-cloud broadcaster
(`POINT_INTERVAL`, `web.py:141`).

### JSON out: `thin_telemetry`

Sent at a lower cadence (~2 Hz) — deliberately not sent per-frame:

```json
{
  "type": "thin_telemetry",
  "fps": 9.8,
  "point_count": 2268,
  "recording": false,
  "mode": "point_cloud",
  "link": "ok",
  "roll_deg": -3.5,
  "tilt_deg": 12.8,
  "heading_deg": 117.2,
  "orientation_valid": true,
  "orientation_labels": ["Roll", "Tilt", "Heading"]
}
```

`fps`/`point_count` and the orientation block are a curated subset of the existing
`ranging`/`sensor` JSON fields already computed by `web._ranging_message` /
`build_sensor_message` — no new telemetry math, just a smaller projection of it for this
endpoint.

**Revised 2026-08-18 (owner):** `power_mode` and `i3c_airtime_pct` are **dropped** — the
operator does not want bus/power figures on this panel — and replaced by orientation and
heading, which they do. `roll_deg`/`tilt_deg`/`heading_deg` are copied **verbatim** from the
sensor message's `orientation_view` + `heading_deg`, i.e. the same numbers in the same
decomposition the web UI's Sensors panel shows; they are never recomputed from the
quaternion here (no scalar off an attitude is the bearing of an axis — the repo's
most-repeated defect). `orientation_valid` is false when the heading cannot be trusted (bad
mag reading, device accelerating, or a boresight within 10° of vertical where no compass
bearing exists at all); a client should grey the compass rather than draw a confident wrong
bearing. `heading_deg` has **no fallback** to `orientation_view["yaw_deg"]` — that slot is the
heading only under the World decomposition, and a Tait-Bryan body-axis twist under `zyx`;
`null` means no bearing exists. `orientation_labels` carries the operator's own three axis
names. `mode` and `recording` here are the **authoritative**
state the client should render; this endpoint has no ack/retry protocol, so a dropped
`thin_mode`/`thin_record` command self-corrects at the next telemetry tick.

### JSON in (client → server commands)

```json
{"type": "thin_orbit", "dyaw": 3.5, "dpitch": -1.0, "dzoom": 0.0}
{"type": "thin_mode", "mode": "point_cloud" | "slam" | "ir"}
{"type": "thin_record", "on": true}
```

- `thin_orbit`: relative deltas, applied cumulatively to this connection's camera state
  each time one arrives. No validation beyond clamping to sane bounds (e.g. pitch range)
  needed — malformed/extreme values just produce an odd-looking frame, not a crash.
- `thin_mode`: switches this connection's render source between the three modes.
- `thin_record`: reuses the **existing** record start/stop logic already wired to the
  browser's record control — do not duplicate recording logic, just add this as a second
  entry point into the same handler `web.py`'s browser `record` message already calls.

### mDNS advertisement

Register a `ServiceInfo` for the web server on startup: service type
`_roomscan._tcp.local.`, instance name `roomscan`, port `8000`. This is new — no prior
advertisement of this server exists.

## Error handling

- **Renderer init failure** (e.g. Open3D can't get a rendering context on this machine):
  `/ws-thin` should reject the connection with a JSON error message and close cleanly, not
  crash the `roomscan.web` process. The existing `/ws`/`/ws-mesh` clients must be
  completely unaffected by a thin-render failure.
- **Backpressure:** freshest-frame-wins per connection, matching the existing broadcaster
  policy — a thin client that can't keep up gets frames dropped for it specifically, never
  a global slowdown of the render loop or other connected clients.
- **Bad/malformed inbound JSON:** ignore and log, don't disconnect the client for one bad
  command — telemetry will still resync it.
- **Client disconnect:** tear down that connection's orbit/mode state; no persistence
  needed across reconnects (a fresh connection starts at a default camera angle and
  `point_cloud` mode).

## Testing / validation plan

- Unit test the `THIN_FRAME` packer (byte layout, dimensions, RGB565 conversion
  correctness) in isolation from the renderer.
- A small standalone test script (same shape as `tools/query_mdns.py` / the MCP server's
  `RigSession` in `host/src/roomscan/mcp_server/session.py`) that connects to `/ws-thin`
  as a fake client, saves received frames as PNGs for a visual sanity check, and
  round-trips `thin_orbit`/`thin_mode`/`thin_record`.
- End-to-end validation using the existing `--replay recordings/scan.bin` capability — no
  physical LiDAR hardware needed to exercise the thin-render path during development.
- Confirm the existing `/ws`/`/ws-mesh` browser clients see no behavior/performance
  regression with a thin client simultaneously connected (the whole point of the separate
  endpoint + separate broadcast task).

## Open questions / deferred work

- **Resolution/rate negotiation:** v1 hardcodes 480×480 @ ~10 fps on both sides. A
  `thin_hello` handshake to negotiate size/rate per client is plausible future work but
  adds protocol surface not needed for a single known client class today.
- **JPEG encoding:** `Pillow` (already a dependency, already used for `IR_IMAGE`-adjacent
  work) could JPEG-encode `THIN_FRAME` payloads to cut bandwidth ~5-10x, once the CrowPanel
  side has a decoder (its onboard JPEG hardware block is currently unused/unwired — see
  companion spec). Not needed for v1.
- **Exposing device parameter control** (`set_profile`/`set_manual_params`) through
  `/ws-thin`: explicitly out of scope for v1; revisit if operators want full DEVICE-panel
  parity from the CrowPanel.
