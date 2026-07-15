# Web Phase 1 — Core Real-Time Web Instrument

**Date:** 2026-07-15
**Status:** approved (brainstorm), pending implementation plan
**Branch:** (TBD — cut from `main`)
**Owner:** hellosamblack

## 1. Overview

`host/src/roomscan/web.py` is a minimal first cut: a FastAPI/uvicorn server
that streams a colorized point cloud to a Three.js frontend
(`host/src/roomscan/static/index.html` + `app.js`) over a single `/ws`
WebSocket. The owner's direction is a **full replacement of the desktop
panel (`panel.py`, ~3600 lines, Open3D) by the web app**, delivered in six
phases:

1. **Core real-time instrument** (this spec)
2. Sensors (IMU/env streams 9/10 — compass/gizmo/sparklines)
3. Recording & playback
4. SLAM mode (trajectory + mesh streamed to the browser)
5. Showcase mode
6. Settings persistence + retire `panel.py`

This spec covers **Phase 1 only**: turning the current minimal viewer into
the everyday real-time instrument. It has four user-facing capabilities —
working device controls with visible feedback, runtime color modes, an IR
monitor pane, and a metrics HUD — built on a backend and WebSocket protocol
designed to absorb Phases 2–6 without rework.

## 2. Goals

- Device-control buttons (Ping / Calibrate / Reinit / Usecase) actually show
  their result — success, busy, timeout, or error — instead of appearing
  dead.
- Runtime color-mode switching (depth / reflectance / confidence) without a
  page reload or device round-trip.
- A live IR (reflectance) monitor pane, independent of the 3D view.
- A metrics HUD (FPS, per-stream rate/jitter, link bandwidth) replacing the
  current 2-line overlay.
- A backend architecture and a multiplexed WebSocket message protocol that
  Phases 2–6 extend by adding message types and UI modules, not by
  rewriting the transport or reader plumbing.
- A frontend module architecture (vanilla ES modules, no build step) that
  stays maintainable as more panels/modes are added in later phases.

## 3. Non-goals (this spec)

Explicitly deferred — see §11 for the fuller phase-by-phase breakdown:

- Sensors/IMU (compass, gizmo, sparklines) — Phase 2.
- Recording & playback controls in the web UI — Phase 3.
- SLAM mode (trajectory/mesh streaming) — Phase 4.
- Showcase mode — Phase 5.
- Settings persistence, retiring `panel.py` — Phase 6.
- Exposure slider, rotate-90 / near-contrast view options (present in
  `panel.py` today; not carried over yet).
- Multi-user auth / access control.
- A JS build step or frontend framework (React, Vue, etc.) — plain ES
  modules via the existing importmap.

## 4. Current state (baseline)

`web.py` today (see file for the exact body):

- Builds a single global `TransformStage`, a latest-wins `queue.Queue(maxsize=1)`
  slot, and a `CommandKeyState` wrapping a `CommandClient`.
- Runs `viewer._reader` on a background thread, feeding the slot.
- Each WebSocket connection runs its **own** send loop that pulls from the
  shared slot — so with two browser tabs open, each `get_nowait()` steals
  the frame the other tab wanted; frames get split unpredictably across
  clients. This is a **bug the Phase 1 redesign fixes** (§5.3).
- Commands are hardcoded per-string (`if cmd == "ping": ...`) rather than
  dispatched through `CommandDispatcher`, so a command's result is never
  reported back to the client — buttons look inert.
- Only one message type exists (binary point-cloud payload); there is no
  provision for IR frames, metrics, logs, or state echoes.
- `index.html`/`app.js` are already restructured with a glassmorphic
  sidebar, an importmap for `three`/`three/addons/`, and Device
  Controls/Usecase/Reset-Camera buttons — but `app.js` is one monolithic
  file with no message-type demux beyond "assume binary = point cloud."

## 5. Backend architecture

### 5.1 Shared app state

`web.py`'s `main()` builds, once, and stores on `app.state`:

- `source` — via `sources.get_best_source(args.port, args.baud)` or
  `FileSource(args.replay)` for replay.
- `client: CommandClient | None` — `None` in replay mode (mirrors
  `panel.py`'s convention; `CommandDispatcher` already handles `client=None`
  by reporting "not available in replay").
- `stage = TransformStage(outputs=("depth", "reflectance", "confidence"))` —
  always compute all three planes; marginal cost per extra plane is
  ~zero, and it removes the current code's `stage_outputs` branching on
  `args.color` (color mode becomes a pure runtime choice, not a
  restart-required one — see §7.2).
- `slot: queue.Queue(maxsize=1)` — latest-wins raw transform output, as
  today.
- `bus = LogBus()` — device EVENT frames and command results/errors both
  funnel here.
- `metrics = MetricsRegistry(window_s=2.0)` — fed once per DATA frame by
  the reader.
- `dispatcher = CommandDispatcher(client, on_message=bus.publish)` —
  replaces the ad hoc `if cmd == "ping"` chain; every dispatch's result
  string lands on the bus and is broadcast to all clients as a `cmd`
  message (§6).
- `fault: dict` — reader-thread fatal error, as today.
- **`ui_state`** (new, small dataclass or plain dict) — the color mode and
  IR settings that Phase 1 introduces, held server-side so late-joining
  clients can be brought up to date immediately on connect:
  - `color_mode: Literal["depth", "reflectance", "confidence"] = "depth"`
  - `ir_colormap: Literal["gray", "turbo"] = "gray"`
  - `ir_freeze: bool = False`
  - `ir_freeze_range: tuple[float, float] | None = None` — set the first
    time freeze is turned on (captured from the live `ir_range` at that
    instant), held constant while frozen, cleared on unfreeze.

### 5.2 Reader thread — reuse `panel._run_reader`

Replace `viewer._reader` with `panel._run_reader(source, decoder, stage,
stats, slot, fault, bus, client, recorder, pacer, is_stopped, state=None,
metrics=metrics)`. Rationale: it already routes EVENT → `bus.publish`, ACK →
`client.offer`, feeds `metrics.record` per DATA frame, and honors a pacer +
stop flag — exactly the plumbing Phase 1 needs, and exactly what Phase 2
(sensor `state`) and Phase 3 (`recorder`) will extend. Building a second,
web-specific reader would fork this logic and rot.

Phase 1 supplies:
- `recorder = None` (no-op; Phase 3 wires a real one).
- `pacer = Pacer(interval=...)` — reused from `panel.py`'s `_Pacer` for
  replay-fps pacing; unused (`interval=0.0`) for live capture.
- `is_stopped = lambda: False` for the process lifetime (the server has no
  "stop capture" affordance yet in Phase 1; `Ctrl+C` kills the process).
- `state = None` (Phase 2 introduces `SensorState`).

`stats = Stats()` (from `viewer.py`) is kept for parity with `_run_reader`'s
signature even though Phase 1's broadcaster reads primarily from `metrics`.

### 5.3 The broadcast hub — one asyncio task, not one per connection

Today's per-connection `while True: slot.get_nowait()` loop is replaced with
a **single background asyncio task**, started in a FastAPI `startup` event
handler, that:

1. Maintains `app.state.clients: set[WebSocket]`.
2. Each tick (paced to the point-cloud rate, ~1000/28 ≈ 36 ms):
   - Pull the latest `(header, outputs)` from `slot` (non-blocking; skip if
     empty).
   - Build the POINT_CLOUD binary payload from the current `ui_state.color_mode`
     and broadcast it to every connected client.
   - On its own slower cadence (~15 Hz), build and broadcast the IR_IMAGE
     binary payload from `outputs["reflectance"]` and the current IR
     settings.
   - On its own slower cadence (~4 Hz), snapshot `metrics.snapshot(now)`
     and broadcast a `metrics` JSON message; drain `bus` and broadcast any
     new lines as `log`/`event` JSON messages.
3. A failed `send_bytes`/`send_text` on one client removes it from
   `app.state.clients` and closes it; it never aborts the broadcast to the
   others (§9).

This is the fix for the current frame-stealing bug: exactly one task reads
`slot.get_nowait()`; every connected client receives the same frame,
independently of how many tabs are open. It also gives Phase 2–5 a single
place to add new periodic broadcasts (sensor sparkline ticks, SLAM pose/mesh
pushes) without touching per-connection code.

### 5.4 Per-connection inbound task

Each `/ws` connection still runs its own `receive_commands()` coroutine
(as today), but simplified: parse inbound JSON, and route by `type`:

- `type: "cmd"` → `dispatcher.dispatch(CommandCode[...], param, label)`.
  Malformed/unknown command names are ignored with a server-side `print`/log
  (§9) — never raise into the receive loop.
- `type: "set_color"` → validate `mode` against the 3 allowed values, update
  `ui_state.color_mode`, broadcast the new `state` JSON to all clients
  (not just the sender) so every open tab's sidebar stays in sync.
- `type: "set_ir"` → validate/update `ir_colormap` / `ir_freeze`, capturing
  or clearing `ir_freeze_range` as described in §5.1; broadcast `state`.

Color/IR-setting changes are pure server-state mutations — no device round
trip, no `CommandDispatcher` involvement, so they're instant regardless of
device busy/timeout state.

On connect (`websocket.accept()`), the server immediately sends one `state`
JSON message so a newly-opened tab reflects the current color mode/IR
settings without waiting for the next change.

## 6. WebSocket message protocol

Everything is multiplexed on the single `/ws` socket, split by JS's native
`typeof event.data` (`ArrayBuffer` vs `string`) — binary for
high-frequency numeric payloads, JSON text for everything else. This
mirrors the wire protocol's own split between a compact binary frame format
and structured control messages, and is the pattern every later phase reuses
(Phase 4 adds a MESH binary type; Phase 2 adds `sensor` JSON messages).

### 6.1 Binary messages — 4-byte little-endian `uint32` type tag first

| Tag | Name | Layout | Rate |
|---|---|---|---|
| `1` | `POINT_CLOUD` | `u32 tag=1` · `f32[3N]` positions · `f32[3N]` colors | ~28 Hz (every transformed frame) |
| `2` | `IR_IMAGE` | `u32 tag=2` · `u16 width` · `u16 height` · `u8[width*height*3]` RGB | ~15 Hz |

`POINT_CLOUD` keeps today's payload shape (positions then colors,
concatenated float32) but gains the leading tag so the client can
distinguish it from `IR_IMAGE`; N is derived client-side from
`(payload.byteLength - 4) / 4 / 6`. `IR_IMAGE`'s width/height are explicit
in the header because the IR plane's resolution can differ from the
point-cloud's deprojected width/height (binning-dependent) and because a
fixed client-side constant would silently break if usecase/resolution
changes — the frontend must read the pane's canvas size from the message,
not assume it.

### 6.2 Text (JSON) messages — discriminated by `type`

- `{"type": "metrics", ...MetricsSnapshot fields...}` — sent ~4 Hz.
  Mirrors `metrics.MetricsSnapshot`: `render_fps: float`, `streams: [{stream_id,
  label, device_hz, host_hz, bytes_per_s, jitter_ms}]`, `link_bytes_per_s:
  float`, `resources: {...} | null`, `drops: int`, `gaps: int`. `resources`
  is omitted/`null` in Phase 1 (no `ResourceSampler` wired yet — optional,
  can be added without a protocol change later).
- `{"type": "event", "code": int, "detail": int, "msg": str}` — one per
  decoded device EVENT frame (from `protocol.parse_event`).
- `{"type": "log", "line": str}` — a bus line that isn't a structured event
  (e.g. an undecodable-event fallback message, or a future free-text
  status line).
- `{"type": "cmd", "label": str, "status": "ok"|"busy"|"timeout"|"error",
  "detail": str}` — a `CommandDispatcher` result. `label` matches the
  label passed to `dispatch()` (e.g. `"ping"`, `"usecase 1"`); `detail`
  carries the human-readable tail (`"OK applied=1"`, the timeout message,
  or `repr(exc)`). `status` is derived server-side by pattern-matching the
  `on_message` string (`CommandDispatcher` always emits `"{label} -> ..."`;
  the broadcaster classifies the suffix — see §7.1 for the exact mapping).
- `{"type": "state", "color_mode": str, "ir_colormap": str, "ir_freeze":
  bool}` — echoed on connect and after any `set_color`/`set_ir` inbound
  message, to every client (not just the sender).

### 6.3 Client-side demux (owned by `ws.js`, §8.3)

The single `onmessage` handler branches on `typeof event.data`: a
`"string"` payload is `JSON.parse`d and re-emitted on the internal hub
keyed by its `type` field (§6.2); an `ArrayBuffer` payload has its first 4
bytes read as a little-endian `uint32` tag and is re-emitted keyed by
`"point_cloud"` (tag 1) or `"ir_image"` (tag 2), with the raw buffer
(header included) handed to the subscriber so `scene.js`/`ir.js` can each
parse their own fixed layout (§6.1) without `ws.js` needing to know
point/pixel counts. An unrecognized tag is dropped with a console warning,
never thrown, so one skewed message can't take down the connection.

## 7. Features

### 7.1 Feature 1 — working device controls with visible feedback

**Why the buttons currently look dead:** `web.py` dispatches commands today
but never surfaces the result anywhere in the UI (no event log, no toast,
no status text) — the command *does* fire, it's just invisible. In replay
mode there is a second, structural reason: `client` is `None`, so every
dispatch is genuinely a no-op (`CommandDispatcher` reports "not available in
replay" and returns immediately) — this is correct behavior, not a bug, and
the UI must say so rather than silently doing nothing.

**Backend:** wire `dispatcher.dispatch(cmd, param, label)` from the `set`
handling in §5.4; `on_message=bus.publish`. The broadcaster classifies each
bus line as it drains it, into exactly the four `status` values §6.2's
`cmd` message defines (`ok`/`busy`/`timeout`/`error` — no fifth value):
- ends with `"not available in replay"` → `status: "error"`, `detail`
  carries that exact suffix so the toast reads distinctly from a real
  device error even though the status color is shared.
- contains `"busy, command already in flight"` → `status: "busy"`.
- starts with `"TIMEOUT"` (after the `label -> ` prefix) → `status: "timeout"`.
- starts with `"ERROR"` → `status: "error"`.
- otherwise (the `"{result.name} applied={applied}"` success line) →
  `status: "ok"`.

**Frontend:**
- A scrolling **event-log console**, docked bottom, collapsible, capped at
  ~200 lines client-side (oldest dropped first — mirrors `LogBus`'s own
  bounded-backlog discipline). Receives both `log` and `event` messages;
  each rendered as one timestamped line (client-side `Date.now()` at
  receipt, since the bus does not stamp wall-clock time either — consistent
  with `logbus.py`'s documented design).
- A **transient command toast** (top-center or near the triggering button):
  on a `cmd` message, show `label: detail` for ~2.5 s, styled by `status`
  (ok = success color, busy/timeout = warning, error = danger — using the
  existing `--success`/`--danger` CSS variables plus a new warning token).
- Buttons remain clickable while a command is in flight (the busy-guard is
  server-side per `CommandDispatcher`); a toast reports "busy" rather than
  the UI disabling the button, keeping the client simple and consistent
  with how `panel.py` already surfaces busy state.

### 7.2 Feature 2 — runtime color modes

**Backend:** `stage` always computes `depth`, `reflectance`, and
`confidence` (§5.1), so switching modes is just changing which plane the
broadcaster reads for point-cloud coloring — no stage reconfiguration, no
reader restart. The broadcaster's per-tick coloring step reuses the
validity-mask + normalize + `colors.turbo` logic already in `web.py`
(lines ~88–100 today): mask `depth` finite/positive/`< deproj.max_range_mm`,
pull the selected plane's values for the valid indices, min-max normalize,
`turbo()`. `color_mode == "depth"` colors by `pts[:, 2]` as today (depth
IS the deprojected Z).

**Missing-plane handling:** if `outputs.get(ui_state.color_mode)` is `None`
(shouldn't happen given `stage`'s fixed `outputs=(...)` tuple, but guards
against a future stage reconfiguration or a malformed frame) — fall back to
depth coloring for that tick and publish one `bus` line
(`f"color mode {mode!r} unavailable this frame, showing depth"`), not once
per tick (debounce: only log on the mode's first miss, or rate-limit to
once per few seconds, so a persistently-missing plane doesn't spam the
log).

**Frontend:** sidebar's existing "View" group grows a segmented
depth/reflectance/confidence control (three-way, like the existing
usecase grid-2 but 3 columns). Selecting a segment sends `{"type":
"set_color", "mode": "depth"|"reflectance"|"confidence"}`; the control's
active segment is driven by the `state` message, not local click state, so
a second tab's change is reflected everywhere (state round-trips through
the server — see §5.4).

### 7.3 Feature 3 — IR monitor

**Backend:** each IR tick (~15 Hz, §5.3), if `outputs.get("reflectance")`
is present: call `ir_image.reflectance_to_rgb(refl, colormap=ui_state.ir_colormap,
vmin=ui_state.ir_freeze_range[0] if frozen else None, vmax=... if frozen
else None)`. When not frozen, `vmin`/`vmax` are left `None` so
`reflectance_to_rgb` computes its own percentile range per call (2nd/98th
pct, matching `ir_image.ir_range`'s defaults) — auto-ranging exactly like
the desktop panel's IR pane. When freeze is turned on (`set_ir` with
`ir_freeze: true`), the *next* tick's `ir_range(refl)` result is captured
into `ui_state.ir_freeze_range` and held constant until unfrozen. Encode as
the `IR_IMAGE` binary message (§6.1); `upscale=1` (no server-side upscaling
— the frontend upscales via CSS, §7.3 frontend bullet, so the wire payload
stays small).

**Missing-plane handling:** if `reflectance` is absent from `outputs` for
this tick, skip sending an `IR_IMAGE` message that tick (client keeps
showing its last frame) and publish one rate-limited `bus` log line, same
debounce discipline as §7.2.

**Frontend:** a `<canvas>` corner card, bottom-left, toggleable (collapsed
by default is acceptable, or default-open — implementation's call, matching
whatever `panel.py`'s IR pane currently defaults to). Draws each `IR_IMAGE`
message via `putImageData`/`drawImage`; CSS `image-rendering: pixelated`
so the low-res reflectance grid upscales to the card's on-screen size
without blur (matches `reflectance_to_rgb`'s own nearest-neighbor
upscale philosophy, just done in CSS instead of pushing more bytes over
the wire). Card header has a gray/turbo colormap toggle and a Freeze
checkbox; both send `{"type": "set_ir", "colormap": ..., "freeze": ...}`
and reflect the server's `state` echo, same pattern as color mode.

### 7.4 Feature 4 — metrics HUD

**Backend:** the broadcaster's ~4 Hz metrics tick sends the `metrics` JSON
message (§6.2) built directly from `metrics.snapshot(time.monotonic())`.

**Frontend:** replaces the current 2-row `#metrics-overlay` with a fuller
HTML/CSS panel (top-left, `pointer-events: none` as today so it never
blocks canvas interaction):
- **VIEW fps** — client-measured render fps (frontend's own
  `requestAnimationFrame` counter, NOT from the server — this is the
  browser's actual paint rate, independent of device/network cadence; keep
  it clearly labeled distinctly from device fps to avoid confusion).
- **Device FPS** — `metrics.render_fps` from the snapshot (the reader
  thread's transform-tick rate).
- **Per-stream rows** — one per `streams[]` entry: `label` (`"ToF"`, later
  `"IMU"`/`"Env"`), `host_hz` (formatted via the same adaptive-precision
  convention as `metrics.fmt_hz`; the frontend reimplements this
  formatting in JS since `metrics.py`'s formatters are Python-only, but the
  *rule* — one decimal under 10 Hz, integer above — should match so the
  HUD's numbers read consistently with the desktop panel's), and
  `jitter_ms` (or `-` when `null`).
- **Link bandwidth bar** — a small horizontal bar/gauge sized off
  `link_bytes_per_s` (formatted via the `fmt_rate`-equivalent convention:
  B/s, KB/s, MB/s) — no fixed max is defined server-side, so pick a
  reasonable client-side scale (e.g. cap the bar's visual fill at a
  round number like 2 MB/s, clamping above that) since USB CDC/Ethernet
  bandwidth ceilings differ and Phase 1 doesn't need a precise gauge, just
  a relative-magnitude indicator.
- `resources` (CPU/RAM/GPU) is `null` in Phase 1 (§6.2) — the HUD simply
  omits that section; wiring `ResourceSampler` server-side is a trivial,
  protocol-compatible follow-up if wanted, not required for Phase 1.

## 8. UI architecture

The owner explicitly wants this section thorough: they dislike the desktop
panel's UI and want a clean, web-native architecture that scales across all
six phases without a rewrite at each step.

### 8.1 Layout regions

Full-bleed `<canvas>` for the 3D scene underneath everything; fixed
overlay regions on top, each pinned to a screen edge/corner so later
phases add *regions*, not restructure existing ones:

- **Left rail** — read-only telemetry HUD. Phase 1: metrics (§7.4). Phase
  2 adds compass/gizmo/sparklines below it. `pointer-events: none` on the
  rail itself (and every element inside it) so it never intercepts
  OrbitControls drag/zoom — this was already the working pattern for
  today's `#metrics-overlay` and must be preserved as the rail grows.
- **Right rail** — interactive control panel: a vertical stack of
  **collapsible groups**, one per concern. Phase 1 groups: Device, View
  (color mode), IR Monitor. Later phases append groups (Capture in Phase 3,
  SLAM in Phase 4, Showcase in Phase 5, Settings in Phase 6) to the same
  stack rather than inventing a new panel location — the rail becomes
  scrollable (`overflow-y: auto`, `max-height` bounded to the viewport)
  once it grows past one screenful, which Phase 1 should build in from the
  start even though Phase 1's own group count doesn't yet require
  scrolling.
- **Bottom-left** — the IR monitor canvas card (§7.3), independently
  toggleable from the right rail's IR Monitor group (the group holds the
  gray/turbo + freeze controls; the card is the pixel output).
- **Bottom** — collapsible event-log console (§7.1) docked full-width at
  the bottom edge, collapsed to a thin bar by default with an unread-count
  badge (or simply defaulting open — implementation's call, but it must be
  collapsible so it doesn't perpetually eat vertical canvas space); a
  **transient toast layer** floats above it (not inside the collapsible
  region, so toasts remain visible even when the log is collapsed).
- **Top bar** — minimal: app title/logo, a connection-status dot + text
  (reusing today's `#conn-dot`/`#conn-text`, relocated from the sidebar
  header into this bar), and a placeholder slot for the Real-Time/SLAM
  mode switch Phase 4 introduces (present in the layout grid now, even if
  Phase 1 renders nothing in it, so Phase 4 doesn't need to renegotiate top
  bar space).

### 8.2 Design system

Extend, don't replace, today's CSS custom-property tokens (`--bg-color`,
`--panel-bg`, `--panel-border`, `--text-main`, `--text-muted`, `--accent`,
`--accent-hover`, `--danger`, `--success`); add a `--warning` token for the
busy/timeout toast state (§7.1). Define a small, fixed component
vocabulary so every future phase's UI is built from the same primitives
rather than each phase inventing new CSS:

- **card** — the glassmorphic container (blur + border + radius), already
  established by `#sidebar`/`#metrics-overlay`; becomes the base class both
  rails, the IR card, and the log console share.
- **control-group** — a collapsible section within the right rail: header
  (label + chevron/disclosure), body. One per concern (§8.1).
- **segmented-control** — the 2–3-way button row already used for
  Usecase (`grid-2`) and newly for color mode (3-way); formalize it as a
  reusable class rather than one-off `grid-2` CSS.
- **toggle / checkbox** — for Freeze, and later per-overlay show/hide
  toggles (Phase 2 sensors, Phase 4 SLAM overlays).
- **button** — default and primary variants, as today.
- **toast** — transient status pill (§7.1), variants ok/busy/timeout/error.
- **log-line** — one row in the event-log console: timestamp, source tag
  (event/log/cmd), message text.

Visual language stays dark glassmorphic (blur, translucent panel-bg,
subtle borders, `Inter` for UI text, `JetBrains Mono` for numeric/telemetry
text) — consistent with what's already shipped, not a re-theme.

### 8.3 Frontend module architecture

Retire the monolithic `app.js` in favor of vanilla ES modules, loaded via
the existing importmap mechanism (no bundler, no build step):

- **`ws.js`** — owns the WebSocket connection, reconnect-with-backoff (the
  logic already in today's `app.js`), and the binary-tag/JSON demux
  (§6.3). Exposes a minimal pub/sub hub (`on(type, handler)` /
  `emit(type, payload)` / a `send(obj)` helper that JSON-stringifies and
  writes) — the frontend's mirror of the backend's `LogBus`/broadcast
  pattern. This is the **only** module that touches the raw `WebSocket`
  object.
- **`scene.js`** — Three.js scene/camera/`OrbitControls`/point-cloud
  `BufferGeometry` management (today's rendering code, extracted). Subscribes
  to `"point_cloud"` from the `ws.js` hub; owns `MAX_POINTS` and the
  render loop; publishes its own measured VIEW fps for `hud.js` to display
  (§7.4).
- **`ir.js`** — the IR monitor `<canvas>` card. Subscribes to `"ir_image"`;
  owns colormap/freeze toggle UI, sends `set_ir` via `ws.js`.
- **`hud.js`** — the left-rail metrics readout. Subscribes to `"metrics"`
  and to `scene.js`'s VIEW-fps publication; pure DOM text updates, no
  canvas.
- **`log.js`** — event-log console + toast layer. Subscribes to `"log"`,
  `"event"`, and `"cmd"`.
- **`controls.js`** — right-rail button/segmented-control bindings; turns
  DOM events into `ws.send(...)` calls (`cmd`, `set_color`, `set_ir`);
  subscribes to `"state"` to keep its own controls' active-state in sync
  with the server (so a second tab's change is reflected here too, per
  §7.2).
- **`app.js`** — now a thin composition root: constructs the `ws.js` hub,
  instantiates every other module against it, and does nothing else
  (no rendering logic, no message parsing of its own).

**State flow is one-way:** the `ws.js` hub is the single source of truth
for anything the server knows; every other module subscribes to the
message types it cares about and never reaches into another module's
state. Outbound is the mirror: any module that needs to change server
state calls `ws.send(...)` and waits for the resulting `state`/`cmd`
broadcast to update its own display, rather than optimistically
mutating its own UI first — this keeps multi-tab behavior correct for
free (§7.1, §7.2) since every tab reacts to the same broadcast.

**Why no framework:** the message surface is small and fully enumerated
(§6), DOM updates are simple text/attribute writes with no complex
component tree, and a build step would add tooling weight (bundler config,
node_modules, a dev-server proxy for `/ws`) disproportionate to the current
UI's complexity — YAGNI. The module boundaries above (one file per concern,
communicating only through the `ws.js` hub's pub/sub) are deliberately
chosen so that *if* a later phase's UI complexity (most likely Phase 4
SLAM, with mesh streaming and multiple overlay layers) justifies a
framework, the migration is mechanical: each module becomes a component,
and the `ws.js` hub becomes the framework's state store/event bus with no
change to the message protocol itself.

## 9. Error handling

- **Reader fault** (`fault["error"]` set by `_run_reader`): the broadcast
  hub checks `fault` each tick; on first observation, broadcast a `log`
  line with the error and flip the top bar's connection indicator to an
  "Offline"/fault state (distinct from the WebSocket-level connected/
  disconnected dot — this is "connected to server, but the device reader
  died"). The server does not crash; it keeps serving the last-known frame
  and lets already-connected clients see the fault message.
- **Per-client send failure**: catch the exception around `send_bytes`/
  `send_text` in the broadcast loop; remove that client from
  `app.state.clients` and let the rest of the broadcast continue
  uninterrupted (§5.3) — one dead tab must never stall or crash the
  broadcaster for everyone else.
- **Missing transform plane** (color mode or IR): fall back + rate-limited
  log, per §7.2/§7.3 — never crash the broadcast tick.
- **Malformed inbound JSON** (bad `cmd` name, missing `mode`/`colormap`
  field, JSON parse failure): ignore the message, log it server-side
  (`print`/stdlib `logging`, not the client-facing `bus`, since this is a
  client bug not a device/app event) and continue the receive loop —
  never let one bad inbound message kill that connection's task.
- **WebSocket disconnect** during either the receive or broadcast path:
  handled via `WebSocketDisconnect`/exception catch exactly as today,
  removing the client from `app.state.clients`.

## 10. Testing strategy

### 10.1 Backend (pytest)

`httpx` is **not** an installed dependency, so Starlette's `TestClient` is
unavailable — tests use the real-server approach: run
`uvicorn.Server(config).serve()` on a background thread bound to an
ephemeral port, connect with the `websockets` client library (already a
declared extra, `web = ["fastapi", "uvicorn", "websockets"]` in
`pyproject.toml`), and tear the server down after each test. Cover:

- **Protocol framing** — binary messages start with the correct 4-byte tag;
  `POINT_CLOUD` payload length is `4 + 24N` for N points; `IR_IMAGE`
  payload's declared `width`/`height` match the trailing byte count
  (`4 + width*height*3`).
- **JSON shapes** — each `type` (`metrics`, `event`, `log`, `cmd`, `state`)
  round-trips through `json.loads` with the fields §6.2 specifies;
  `metrics` matches `MetricsSnapshot`'s field set exactly (a schema/shape
  test, not a value test).
- **Color-mode selection** — sending `set_color` changes which plane
  colors subsequent `POINT_CLOUD` frames (feed a synthetic frame through
  `_run_reader`/the broadcaster with known depth/reflectance/confidence
  arrays and assert the emitted colors track the selected plane, not
  always depth).
- **IR encoding** — `reflectance_to_rgb` output shape/dtype matches what
  gets packed into the `IR_IMAGE` payload; colormap/freeze toggles change
  the encoded bytes as expected (gray vs turbo distinguishable; frozen
  range holds across two differently-ranged input frames).
- **Command dispatch routes to the bus** — a `cmd` inbound message reaches
  `CommandDispatcher.dispatch`; a fake/no-op client's result string
  produces the correctly-classified `cmd` JSON `status` per §7.1's mapping
  (ok/busy/timeout/error/unavailable).
- **Broadcaster fan-out, no frame-stealing** — connect two `websockets`
  clients simultaneously against a `FileSource` replay feed; assert both
  receive the *same* sequence of frames (matching `seq`/point counts),
  proving the single-broadcast-task fix (§5.3) — this is the regression
  test for the bug being fixed.

Where a test doesn't need the network layer at all (e.g. color-plane
selection logic, IR encoding, the `cmd`-string → `status` classifier), test
the broadcaster's pure helper functions directly rather than going through
a live socket — reserve the `uvicorn.Server` + `websockets`-client harness
for tests that are specifically about the WebSocket transport itself
(framing, multi-client fan-out, connect/disconnect).

### 10.2 Frontend (manual)

No automated browser test harness in Phase 1. Verification is manual:
run the web server against a looping `FileSource` replay
(`--replay captures/*.bin --replay-fps ...`, matching existing CLI args)
and drive the UI in a real browser, using screenshots to confirm layout
and behavior — a working screenshot/browser-driving harness for this
project already exists in the session scratchpad from earlier UI work;
reuse that approach (launch server, navigate, screenshot each region/state:
default load, a color-mode switch, a command toast, the IR pane toggled,
the metrics HUD populated, two tabs open simultaneously to eyeball the
fan-out fix) rather than building new frontend test infrastructure for
Phase 1.

## 11. Migration / rollout

- Changes are confined to `host/src/roomscan/web.py` and
  `host/src/roomscan/static/` (new module files under `static/`, replacing
  the current `app.js` monolith; `index.html` restructured for the new
  layout regions of §8.1).
- No wire-protocol changes, no firmware changes — this is purely a
  host-side web-app change. (If a future phase *does* touch the wire
  protocol, the `protocol-change` skill's checklist applies then, not
  here.)
- `panel.py` is untouched; it keeps working as the fallback/reference
  implementation until Phase 6 retires it.
- Launch remains `view-web.bat` / `view-web.sh` (already present) — no
  change to how the server is started.
- Rollout is a single PR once implemented and tested; no feature flag is
  needed since the web app is a separate entry point from `panel.py` and
  carries no risk to the desktop tool.

## 12. Explicitly deferred to later phases

| Deferred item | Target phase |
|---|---|
| IMU/env streams 9/10: compass, gizmo, sparklines | Phase 2 |
| Recording controls (start/stop, file naming) in the web UI | Phase 3 |
| Playback/replay controls in the web UI (scrub, pause, fps) | Phase 3 |
| SLAM mode: trajectory + mesh streamed to the browser, a new binary MESH message type, Real-Time/SLAM top-bar mode switch (placeholder reserved in §8.1) | Phase 4 |
| Showcase mode (reveal flow, auto-orbit) | Phase 5 |
| Settings persistence (`--save-config` equivalent for the web app) | Phase 6 |
| Retiring `panel.py` | Phase 6 |
| Exposure slider | not yet scheduled |
| Rotate-90 / near-contrast view options | not yet scheduled |
| Multi-user auth/access control | not yet scheduled |
| A JS build step or frontend framework | revisit only if Phase 4's complexity warrants it (§8.3) |
| `ResourceSampler` (CPU/RAM/GPU) wired into the metrics HUD | optional follow-up, not required for Phase 1 |
