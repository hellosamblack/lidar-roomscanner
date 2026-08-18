# Thin-client render feed (`/ws-thin`)

**Issue:** [#194](https://github.com/hellosamblack/lidar-roomscanner/issues/194) ·
**Spec:** [`superpowers/specs/2026-08-17-thin-client-render-design.md`](superpowers/specs/2026-08-17-thin-client-render-design.md) ·
**Plan:** [`superpowers/plans/2026-08-17-lidar-thin-client-crowpanel.md`](superpowers/plans/2026-08-17-lidar-thin-client-crowpanel.md) ·
**Client:** `CrowPanelProp` repo (CrowPanel ESP32-P4 7" prop communicator)

`roomscan-web` renders the 3D view **on the server** and ships finished 480×480
raster frames to a client that has no GPU, no 3D pipeline and no image codec.
The client is a remote display plus an input relay; it never receives points,
meshes or triangles.

This is a **separate endpoint**, not a filter on `/ws`. The existing
`/ws` / `/ws-mesh` browser protocol ([`web-protocol.md`](web-protocol.md)) is
untouched and knows nothing about thin clients — which keeps the embedded
parser trivial and avoids adding a subscribe/filter layer to the browser
sockets.

---

## Protocol contract

> **Duplicated verbatim** in the `CrowPanelProp` companion spec. It is the
> shared interface, not a cross-reference: any change here changes both repos
> and both specs in the same commit.

### `GET /ws-thin`

### Binary frame (server → client): `THIN_FRAME`

480×480 @ 10 fps by default; negotiable via `thin_hello` (below).

```
u32 tag        = 1
u16 width      = 480
u16 height     = 480
u8  pixels[width * height * 2]   // RGB565, little-endian, row-major
```

460 808 bytes per frame at ~10 fps ≈ **4.6 MB/s per connected client**.

### Binary frame (server → client): `THIN_FRAME_JPEG` (tag 2)

Sent instead of tag 1 once a client has negotiated `format: "jpeg"` via
`thin_hello`.

```
u32 tag        = 2
u16 width
u16 height
u32 jpeg_len
u8  jpeg[jpeg_len]   // JFIF bytes, simplejpeg-encoded RGB
```

Encoding happens on the render thread, never the asyncio loop. If
`simplejpeg` is not importable on this host, `thin_hello_ack` reports
`format: "raw"` regardless of what was requested and the flow stays on
tag 1 — report what actually happened, not what was asked for — and the
server logs one warning line per connected flow (not per hello, never per
frame) so the degrade is visible in its own journal.

### JSON in: `thin_hello` / JSON out: `thin_hello_ack`

```json
{"type": "thin_hello", "format": "jpeg", "fps": 30, "width": 320, "height": 320, "quality": 80}
{"type": "thin_hello_ack", "format": "jpeg", "fps": 30.0, "width": 320, "height": 320, "quality": 80}
```

All fields optional; a field omitted keeps its current value. Clamped
server-side, and the ack echoes the CLAMPED effective values, never the
raw request: `width`/`height` to the nearest of {320, 480} (square only —
a non-square request is rejected outright and the prior width/height are
kept), `quality` to [40, 95]. A malformed or non-finite value for one
field (wrong type, `Infinity`/`NaN`, non-square pair) is a no-op for that
field only; the rest of the same message still applies, and the ack is
always sent.

`fps` stays fractional (a float) and its ceiling is COUPLED to `format`:
raw clamps to [1, 10] because a raw RGB565 480×480 frame is 460 808 bytes
(~4.6 MB/s per client at 10 fps already); jpeg clamps to [1, 60] since a
JPEG frame at the default quality is roughly 15–25 KB. The clamp is
re-applied to the STORED fps on every `thin_hello`, even one that does not
mention `fps` — a client that negotiated `jpeg@60` and later sends
`{"format": "raw"}` alone gets its rate re-clamped down to 10, not carried
over illegally.

The render loop is the single writer on the socket: a `thin_hello` is
applied at the top of a render tick and its `thin_hello_ack` is sent
before the first frame rendered under the new state, so a client never
receives a frame in the new format or size ahead of the ack.

A client that never sends `thin_hello` is unaffected — tag 1, RGB565,
480×480, 10 fps, byte-identical to before this feature.

Pacing is per-connection: each flow renders/sends at its own negotiated
interval (default `THIN_INTERVAL`, 100 ms) rather than a shared module
constant. The existing single-slot backpressure/stale-frame-resync
behavior is unchanged.

### JSON out: `thin_telemetry`

Sent every 500 ms — deliberately not per-frame.

```json
{
  "type": "thin_telemetry",
  "fps": 9.8,
  "point_count": 2268,
  "recording": false,
  "mode": "point_cloud",
  "link": "ok",
  "roll_deg": -3.5,
  "pitch_deg": 12.8,
  "tilt_deg": 12.8,
  "heading_deg": 117.2,
  "yaw_rate_dps": 0.0,
  "orientation_valid": true,
  "orientation_labels": ["Roll", "Tilt", "Heading"],
  "ir_grid": [
    12, 14, 18, 22, 25, 20, 15, 10,
    14, 28, 45, 60, 62, 48, 24, 12,
    18, 42, 85, 120, 115, 80, 38, 16,
    20, 50, 110, 180, 175, 105, 45, 18,
    22, 52, 115, 185, 180, 110, 48, 20,
    19, 44, 88, 125, 120, 85, 40, 18,
    15, 30, 48, 65, 64, 50, 26, 14,
    10, 12, 16, 20, 22, 18, 14, 10
  ]
}
```

`mode` and `recording` are **authoritative** — this endpoint has no ack/retry,
so a dropped `thin_mode`/`thin_record` self-corrects at the next tick. Any
field may be `null` before the device has been read back.

**The spatial orientation and heading block** (`roll_deg`, `pitch_deg`, `tilt_deg`,
`heading_deg`, `yaw_rate_dps`, `orientation_valid`, `orientation_labels`) is copied
**verbatim** from the sensor message the broadcaster already built — the same numbers,
in the same decomposition, that the web UI's Sensors panel displays. `yaw_rate_dps`
reports angular velocity around the body yaw axis (deg/s). It is deliberately
*not* recomputed from the quaternion at this endpoint: no scalar taken off an
attitude is the bearing of an axis, and asking for "yaw" instead of the bearing
of the axis you actually mean is the single defect this repo has shipped most
often (four times, twice in live code, pinned by the
`test_no_new_yaw_twist_consumers` AST guard). Reusing the finished projection is
what keeps this endpoint from being the fifth.

`ir_grid` carries an 8x8 matrix (64 integers in 0..255, row-major) downsampled
from the ToF reflectance array, allowing the CrowPanel sidebar to display a live
ambient/reflectance thumbnail widget even while the main viewport is in `point_cloud`
or `slam` mode.

`orientation_valid` is `false` when the heading cannot be trusted — a bad
magnetometer reading, the device accelerating (gravity reference degraded), or a
boresight within 10° of vertical, where **no compass bearing exists at all**
(BUG-058). A client should grey the compass out rather than draw a confident
wrong bearing.

`heading_deg` comes **only** from the sensor message's own `heading_deg`. There
is deliberately no fallback to `orientation_view["yaw_deg"]`: that slot is the
heading in the *World* decomposition, but under `zyx` (or any alternate
decomposition the operator selects) it is a Tait-Bryan twist about a body axis
and not a bearing at all. `null` means no bearing exists right now, and the
client is told that rather than shown a plausible wrong number. `roll_deg`,
`pitch_deg` and `tilt_deg` remain valid in that state. `orientation_labels` carries the operator's own three axis names
(default `["Roll", "Tilt", "Heading"]`).

`power_mode` and `i3c_airtime_pct` were in the first draft of this contract and
were **removed on 2026-08-18** at the owner's request — bus utilisation and
power mode are not wanted on this panel. That freed the space the orientation
and IR preview widgets now occupy.

### JSON in (client → server)

```json
{"type": "thin_orbit", "dyaw": 3.5, "dpitch": -1.0, "dzoom": 0.0}
{"type": "thin_mode", "mode": "point_cloud" | "slam" | "ir"}
{"type": "thin_record", "on": true}
```

Deltas are relative and cumulative. Pitch clamps to ±89°, zoom to [0.25, 8.0],
yaw wraps modulo 360. Malformed JSON, unknown message types, junk modes and
non-finite deltas are logged and ignored — never a disconnect.

### Errors

Sent as JSON immediately before a clean close:

| `error` | Meaning |
| --- | --- |
| `thin_render_unavailable` | No offscreen rendering context on this host |
| `thin_client_limit` | `THIN_MAX_CLIENTS` (2) already connected |

### mDNS

The server advertises `_roomscan._tcp.local.`, instance `roomscan`, at its bound
port, with TXT `path=/static/index.html` and `thin=/ws-thin`. This is the
**first time this server has advertised itself** — the pre-existing `zeroconf`
use in `sources.py` only *browses*, to find the MCU. Verify with
`host/tools/query_mdns.py`. mDNS failure is logged and ignored; it must never
break startup.

---

## Renderer backend

**Open3D `OffscreenRenderer`**, chosen after the Step 0 spike measured it on
the GPU-less dev host (Proxmox LXC, llvmpipe, EGL headless, Open3D 0.19):

| Measurement | Result |
| --- | --- |
| Context creation | OK (EGL 1.5 / OpenGL 4.5), ~4 s one-time |
| Render 480×480, 2 300 points | mean 17.2 ms (p95 18.5) |
| Full tick — churn + camera + RGB565 | 27.2 ms |
| Sustained 10 fps | 60/60 frames in 6.0 s |
| Event-loop `tick_share` (BUG-063 method) | 1.000 baseline → 1.005 rendering |
| Same, with 2 thin clients at the cap | 1.0000 → 1.0000 (+0.00 %) |

Filament renders on its own threads and releases the GIL, so a thin client
costs the asyncio event loop nothing measurable. The feared "30 % tax on every
asyncio client" did not materialise, and **the planned numpy software-projector
fallback was not needed**. `ThinScene` is still renderer-agnostic on purpose, so
the backend remains swappable.

### Two Filament constraints that shape the design

Both **abort the process** (`utils::PreconditionPanic`, `terminate called`)
rather than raising something catchable, so the code makes them unreachable
rather than handling them:

1. **One `OffscreenRenderer` per process.** A second construction aborts
   immediately. `ThinRenderer` is therefore a hard process-wide singleton, and
   every thin client shares it.
2. **Every Filament call must run on the thread that created the renderer**
   (`JobSystem::getState(): This thread has not been adopted.`). So the
   singleton owns a **dedicated render thread** that both creates the context
   and services every job from a queue; nothing else touches Open3D's rendering
   stack.

A third, quieter one: letting the interpreter garbage-collect a live renderer
from the main thread ends the process with `pure virtual method called`. The
context is destroyed on its owning thread, from `_lifespan` shutdown and an
`atexit` backstop.

---

## Client cap and backpressure

**`THIN_MAX_CLIENTS = 2`.** Connection count is a real performance variable on
this server (BUG-060), and a thin client's per-connection cost is a full render
— far above the deflate that motivated that limit. Because all clients share
the one renderer, renders serialise: 2 × 10 fps = 20 renders/s ≈ 54 % of the
render thread. If more clients are ever needed, the cap is the knob; a second
renderer is not an option.

**Backpressure is freshest-frame-wins, and structural rather than a governor.**
Each connection's task awaits its own render before pacing the next tick, so at
most one render per client is ever in flight, and a tick whose work overran the
interval resyncs instead of burst-catching-up (counted in `ThinFlow.skipped`).

It is deliberately **not** built on the send side: on this stack `send_bytes`
never blocks and queues unboundedly (BUG-061), so a send-derived "is the client
keeping up" signal would measure nothing. The guard sits on the genuinely
contended resource — the single shared render thread. A slow render costs that
client frames and never the other clients.

---

## Data taps — reuse, never recompute

The render loop draws from data the existing pipeline already produced. Each
stash is a **generation-tagged** reference assigned where the value is already
computed — no copy, no recomputation — and the thin loop **drops a stash whose
generation no longer matches** (#101 source-generation barrier), so a
live↔replay switch can never show a thin client the previous source's geometry.
The barrier block in `_broadcaster` clears them alongside `latest_mesh`.

| Mode | Source | Notes |
| --- | --- | --- |
| `point_cloud` | `state.thin_latest_pc` — the gravity-corrected `pts`/`colors` the broadcaster built for `pack_point_cloud` | Also fed by the surface path (`pg`/`cg`) |
| `slam` | `state.latest_mesh`, unpacked by `unpack_mesh_scene` | Memoised **by object identity**, never `mesh_seq` (which resets to 0 on `_reset_slam`) — unpacking a 150 k-vertex mesh per tick would dwarf the render. Decimated to `THIN_MESH_VERT_BUDGET` (120 k) — see below |
| `ir` | `state.thin_latest_ir` — the already-rotated, already-colormapped RGB | Needs no rendering context at all: a nearest-neighbour letterbox upscale, so honest sensor zones rather than an interpolated smear |

**Known limitation:** `thin_latest_pc` is only refreshed while the browser-side
`ui.display` is `point_cloud`. If the server is in SLAM display mode, a thin
client in `point_cloud` mode sees no fresh frames. That is the deliberate price
of never recomputing: fixing it would mean deprojecting and colorizing a second
time, for a client that can switch to `slam` instead.

`thin_record` routes through `_apply_record`, the same helper the browser's
`record` message uses — there is exactly one place that knows what recording
involves, so the two entry points cannot drift apart.

### The SLAM mode's mesh budget

Two things had to change before `slam` was usable, both found by actually
watching the feed rather than by reading the code:

**1. Decimation (`THIN_MESH_VERT_BUDGET = 120_000`).** A real Detailed mesh
arrives with **1.1 M vertices** — issue #190, where meshes ship far past their
own 150 k budget. Uploading that to Filament on llvmpipe takes *tens of
seconds*: measured **1 frame in 40 s, then zero in the next 90 s**. Because
every thin client shares one render thread, that does not merely stall the
client that asked for `slam` — it starves the other one too. `unpack_mesh_scene`
therefore subsamples **triangles** (not vertices — that is what keeps the result
a valid mesh) and reindexes. The step is derived from the triangle count, since
each kept triangle can pull in up to 3 distinct vertices. After: **285 frames in
40 s (7.1 fps)**.

**2. Unlit shading.** An `OffscreenRenderer` scene has **no light by default**,
so a `defaultLit` mesh renders very nearly black — the room's shape was present
but the frame was unreadable. SLAM meshes always carry vertex colours, so the
mesh path uses `defaultUnlit` when colours are present (matching how the
point-cloud mode already drew), and adds a sun only for the colourless fallback.

`slam` runs slower than the other modes (~7 fps vs 10) because a live SLAM
session emits a *new* mesh continuously, so the identity memoisation misses and
each new mesh is re-unpacked and re-uploaded. That is the expected cost, not a
fault.

---

## Probing it

`rig_thin_probe` (MCP) / `host/tools/thin_client_probe.py` (CLI) connect as a
fake thin client, decode frames back to PNG, and round-trip the commands. The
orbit check reports **how many pixels actually changed** — a frame counter or a
read-back of the camera state would prove nothing (the #106 lesson).

At a high negotiated fps, the probe's own receive buffer can hold several
frames the server already produced before a command was sent — the probe
(decode + PNG-write per frame) is a slower consumer than the server is a
producer. The orbit check accounts for this (#197): it drains the receive
buffer immediately before each control capture, and time-anchors the
post-command capture (the `after` frame must have been *received* at least one
full negotiated interval past the send, discarding anything earlier however
many frames that takes) rather than trusting a fixed count of frames to have
moved past the command.

See [`mcp-server.md`](mcp-server.md).

---

## Deferred

- **Device parameter control** (`set_profile` / `set_manual_params`) stays
  reachable only from the browser `/ws` protocol.

*(JPEG encoding and `thin_hello` resolution/rate negotiation shipped in #197 —
see the protocol sections above. The CrowPanel side — hardware JPEG decode of
tag-2 frames — lives in the `CrowPanelProp` companion repo.)*
