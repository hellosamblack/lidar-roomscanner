"""Web-based real-time instrument. One reader thread (shared with the desktop
panel via `panel._run_reader`) feeds a latest-wins slot; a SINGLE asyncio
broadcast task fans every transformed frame out to all connected WebSocket
clients. This replaces the old per-connection `slot.get_nowait()` loop, whose
competing gets stole frames from one another when two tabs were open (§5.3).

The wire is multiplexed on one `/ws` socket: binary messages (point cloud, IR
image) carry a leading little-endian uint32 tag; JSON text messages carry a
`type` discriminator (metrics/event/log/cmd/state). See the design spec
docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md §6.

Pure, socket-free helpers (classify_bus_line / select_colors /
pack_point_cloud / pack_ir_image / build_metrics_message) are factored out at
module level so the protocol/coloring logic is unit-testable without a server.
"""
from __future__ import annotations

import asyncio
import json
import logging
import os
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
import webbrowser
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from . import panel
from .colors import turbo
from .control import CommandClient, CommandDispatcher
from .decoder import StreamDecoder
from .deproject import Deprojector
from .ir_image import ir_range, reflectance_to_rgb
from .logbus import LogBus
from .metrics import MetricsRegistry, MetricsSnapshot
from .pipeline import TransformStage
from .protocol import CommandCode
from .sources import FileSource, SerialSource, UdpSource, get_best_source
from .viewer import Stats, resolve_args

log = logging.getLogger("roomscan.web")

# Binary message type tags (first 4 bytes, little-endian uint32).
TAG_POINT_CLOUD = 1
TAG_IR_IMAGE = 2

# Broadcast cadences (seconds). Point cloud paces the outer loop at a 30 Hz
# target (owner, 2026-07-16) -- the cap must sit at or above the source rate so
# it never down-samples the stream; a slower source just re-sends the last frame.
# IR and metrics run on their own slower elapsed-time gates off the same task.
POINT_INTERVAL = 1.0 / 30.0
IR_INTERVAL = 1.0 / 15.0
METRICS_INTERVAL = 1.0 / 4.0
MISSING_PLANE_LOG_INTERVAL = 3.0   # debounce for missing-plane bus lines

_VALID_COLOR_MODES = ("depth", "reflectance", "confidence")
_VALID_IR_COLORMAPS = ("gray", "turbo")

# Success command results look like "OK applied=1" / "REJECTED applied=0":
# a ResultCode name (upper snake) followed by applied=<int>.
_CMD_SUCCESS_RE = re.compile(r"^[A-Z0-9_]+ applied=-?\d+$")
_EVENT_RE = re.compile(r"^\[event\] code=(-?\d+) detail=(-?\d+)(?: (.*))?$")


# --- ui state ----------------------------------------------------------------

@dataclass
class UiState:
    """Server-held view/IR settings, so a late-joining tab is brought current
    the instant it connects (§5.1). All mutations are pure server state -- no
    device round-trip -- so color/IR changes apply regardless of device busy
    state."""
    color_mode: str = "depth"
    ir_colormap: str = "gray"
    ir_freeze: bool = False
    ir_freeze_range: tuple[float, float] | None = None


# --- pure helpers (no socket, no async) -------------------------------------

def classify_bus_line(line: str, command_labels: set[str] | None = None) -> dict | None:
    """Classify one LogBus line into a `cmd`/`event`/`log` JSON dict (§7.1).

    Robustness approach: we do NOT need a live registry of in-flight commands.
    A line is a device event iff it starts with the reader's `[event] ` prefix;
    a command result iff it contains the ` -> ` marker CommandDispatcher always
    emits AND its tail matches one of the four known status shapes (replay /
    busy / TIMEOUT / ERROR / "<ResultCode> applied=<n>"). `command_labels`, when
    supplied, further gates cmd-classification to labels we actually dispatched
    (belt-and-suspenders against a free-text log line that happens to contain
    ` -> `); when None, suffix-matching alone decides. Anything else is a plain
    log line. Returns None only for an empty/None input.
    """
    if not line:
        return None

    m = _EVENT_RE.match(line)
    if m:
        code, detail, msg = m.group(1), m.group(2), m.group(3) or ""
        return {"type": "event", "code": int(code), "detail": int(detail), "msg": msg}
    if line.startswith("[event]"):
        # e.g. "[event] undecodable payload (12 B)" -- not parseable as structured
        return {"type": "log", "line": line}

    if " -> " in line:
        label, _, tail = line.partition(" -> ")
        status = _cmd_status(tail)
        if status is not None and (command_labels is None or label in command_labels):
            return {"type": "cmd", "label": label, "status": status, "detail": tail}

    return {"type": "log", "line": line}


def _cmd_status(tail: str) -> str | None:
    """Map a CommandDispatcher result tail to a status, or None if it doesn't
    look like a command result at all (so the whole line falls back to log)."""
    if tail.endswith("not available in replay"):
        return "error"
    if "busy, command already in flight" in tail:
        return "busy"
    if tail.startswith("TIMEOUT"):
        return "timeout"
    if tail.startswith("ERROR"):
        return "error"
    if _CMD_SUCCESS_RE.match(tail):
        return "ok"
    return None


def select_colors(outputs: dict, deproj: Deprojector, color_mode: str):
    """Deproject depth and colorize by the selected plane (§7.2).

    Returns (pts, colors, fell_back): pts (N,3) float32 metres, colors (N,3)
    float32 in [0,1], and fell_back True iff the requested non-depth plane was
    missing this frame and depth coloring was substituted. Coloring reuses the
    validity mask (finite, >0, < max_range) + min-max normalize + turbo, exactly
    as the classic viewer. `color_mode == "depth"` colors by deprojected Z.
    """
    depth = outputs["depth"]
    pts = deproj(depth)
    fell_back = False
    if len(pts) == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty, False

    if color_mode == "depth":
        vals = pts[:, 2].astype(np.float64, copy=False)
    else:
        plane = outputs.get(color_mode)
        if plane is None:
            fell_back = True
            vals = pts[:, 2].astype(np.float64, copy=False)
        else:
            valid = np.isfinite(depth) & (depth > 0.0) & (depth < deproj.max_range_mm)
            vals = plane[valid].astype(np.float64, copy=False)

    vn = (vals - vals.min()) / max(float(np.ptp(vals)), 1e-6)
    colors = turbo(vn)
    return pts.astype(np.float32, copy=False), colors.astype(np.float32, copy=False), fell_back


def pack_point_cloud(pts: np.ndarray, colors: np.ndarray) -> bytes:
    """POINT_CLOUD binary (tag 1): u32 tag=1 · f32[3N] positions · f32[3N]
    colors, all little-endian. Positions then colors, concatenated (§6.1)."""
    pos = np.ascontiguousarray(pts, dtype="<f4").ravel()
    col = np.ascontiguousarray(colors, dtype="<f4").ravel()
    return struct.pack("<I", TAG_POINT_CLOUD) + pos.tobytes() + col.tobytes()


def pack_ir_image(rgb: np.ndarray) -> bytes:
    """IR_IMAGE binary (tag 2): u32 tag=2 · u16 width · u16 height ·
    u8[width*height*3] RGB, little-endian. width/height read from the array
    shape (H, W, 3) (§6.1)."""
    arr = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = arr.shape[0], arr.shape[1]
    return struct.pack("<IHH", TAG_IR_IMAGE, width, height) + arr.tobytes()


def build_metrics_message(snapshot: MetricsSnapshot) -> dict:
    """MetricsSnapshot -> `metrics` JSON dict (§6.2). `resources` is null in
    Phase 1 (no ResourceSampler wired); numeric fields go raw over the wire,
    the frontend formats. device_hz/jitter_ms may be None -> JSON null."""
    return {
        "type": "metrics",
        "render_fps": float(snapshot.render_fps),
        "streams": [
            {
                "stream_id": s.stream_id,
                "label": s.label,
                "device_hz": s.device_hz,
                "host_hz": s.host_hz,
                "bytes_per_s": s.bytes_per_s,
                "jitter_ms": s.jitter_ms,
            }
            for s in snapshot.streams
        ],
        "link_bytes_per_s": float(snapshot.link_bytes_per_s),
        "resources": None,
        "drops": snapshot.drops,
        "gaps": snapshot.gaps,
    }


def resolve_command(name: str, param) -> tuple[CommandCode, int, str] | None:
    """Inbound `cmd` request -> (CommandCode, param, label), or None for an
    unknown name. usecase carries the id as both param and label suffix."""
    if name == "ping":
        return CommandCode.PING, 0, "ping"
    if name == "calib":
        return CommandCode.SEND_CALIB, 0, "calib"
    if name == "reinit":
        return CommandCode.REINIT, 0, "reinit"
    if name == "usecase":
        uid = int(param)
        return CommandCode.SET_USECASE, uid, f"usecase {uid}"
    if name == "period":
        return CommandCode.SET_FRAME_PERIOD_US, int(param), f"period {int(param)}"
    if name == "exposure":
        return CommandCode.SET_EXPOSURE_MS, int(param), f"exposure {int(param)}"
    return None


def _state_message(ui: UiState) -> dict:
    return {"type": "state", "color_mode": ui.color_mode,
            "ir_colormap": ui.ir_colormap, "ir_freeze": ui.ir_freeze}


# --- FastAPI app + broadcast hub --------------------------------------------

@asynccontextmanager
async def _lifespan(app: FastAPI):
    # Start the single broadcast task once the server begins serving. app.state
    # is fully populated by main() before uvicorn.run(); guard the (test/import)
    # case where it isn't so a bare `import roomscan.web` never spins a task.
    if getattr(app.state, "ready", False):
        app.state.broadcast_task = asyncio.create_task(_broadcaster())
    yield
    task = getattr(app.state, "broadcast_task", None)
    if task is not None:
        task.cancel()


app = FastAPI(lifespan=_lifespan)

static_dir = Path(__file__).parent / "static"
static_dir.mkdir(exist_ok=True)
app.mount("/static", StaticFiles(directory=str(static_dir), html=True), name="static")


async def _drop_client(clients: set, ws: WebSocket) -> None:
    """Remove a client and best-effort close it; never raises."""
    clients.discard(ws)
    try:
        await ws.close()
    except Exception:
        pass


async def _broadcast_bytes(clients: set, data: bytes) -> None:
    for ws in list(clients):
        try:
            await ws.send_bytes(data)
        except Exception:
            await _drop_client(clients, ws)   # one dead tab must not stall the rest (§9)


async def _broadcast_text(clients: set, text: str) -> None:
    for ws in list(clients):
        try:
            await ws.send_text(text)
        except Exception:
            await _drop_client(clients, ws)


def _log_debounced(state, bus: LogBus, key: str, message: str) -> None:
    """Publish `message` at most once per MISSING_PLANE_LOG_INTERVAL for a given
    key, so a persistently-missing plane doesn't spam the log (§7.2/§7.3)."""
    now = time.monotonic()
    last = state.debounce.get(key, 0.0)
    if now - last >= MISSING_PLANE_LOG_INTERVAL:
        state.debounce[key] = now
        bus.publish(message)


async def _broadcaster() -> None:
    """The single fan-out task (§5.3). Started once on startup; runs for the
    process lifetime. Exactly one reader of `slot`, so every client sees the
    same frames no matter how many tabs are open."""
    state = app.state
    clients: set = state.clients
    bus: LogBus = state.bus
    metrics: MetricsRegistry = state.metrics
    ui: UiState = state.ui_state

    bus_handle = bus.subscribe()
    last_item = None          # (header, outputs); kept so IR/metrics tick when slot is idle
    last_pc_key = None        # (seq, color_mode) -> cached packed point cloud
    last_pc_bytes = None
    last_ir = 0.0
    last_metrics = 0.0
    next_pc = time.monotonic()   # deadline-based pacing: sleep to the next tick,

    while True:
        # not for a fixed interval AFTER the work -- otherwise the true period is
        # POINT_INTERVAL + work_time and the stream never reaches the target rate.
        delay = next_pc - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        now = time.monotonic()
        next_pc += POINT_INTERVAL
        if next_pc <= now:       # a slow tick overran the interval: resync, don't burst-catch-up
            next_pc = now + POINT_INTERVAL

        # Latest-wins pull; a fresh frame ticks the device-fps render counter.
        try:
            item = state.slot.get_nowait()
            last_item = item
            metrics.tick_render(now)
        except queue.Empty:
            pass

        # Reader fault: surface once, flip the fault flag, keep serving.
        if state.fault and not state.fault_reported:
            state.fault_reported = True
            err = state.fault.get("error")
            print(f"\n[FATAL] reader thread stopped: {err!r}", file=sys.stderr, flush=True)
            bus.publish(f"reader stopped: {err!r}")

        if last_item is not None:
            header, outputs = last_item
            depth = outputs["depth"]
            h, w = depth.shape
            if state.deproj is None:
                state.deproj = Deprojector(w, h, state.args.fov_h, state.args.fov_v)

            # POINT_CLOUD every tick (so late joiners see data within ~36ms).
            # Cache the packed bytes; rebuild only when the frame or color mode
            # changed, so a stalled feed doesn't re-deproject 28x/s for nothing.
            key = (header.seq, ui.color_mode)
            if key != last_pc_key:
                pts, colors, fell_back = select_colors(outputs, state.deproj, ui.color_mode)
                if fell_back:
                    _log_debounced(state, bus, f"color-miss:{ui.color_mode}",
                                   f"color mode {ui.color_mode!r} unavailable this frame, showing depth")
                last_pc_bytes = pack_point_cloud(pts, colors)
                last_pc_key = key
            if last_pc_bytes is not None:
                await _broadcast_bytes(clients, last_pc_bytes)

            # IR_IMAGE on its own slower cadence.
            if now - last_ir >= IR_INTERVAL:
                last_ir = now
                refl = outputs.get("reflectance")
                if refl is not None:
                    if ui.ir_freeze:
                        if ui.ir_freeze_range is None:      # capture on the first frozen tick
                            ui.ir_freeze_range = ir_range(refl)
                        vmin, vmax = ui.ir_freeze_range
                    else:
                        vmin = vmax = None
                    rgb = reflectance_to_rgb(refl, colormap=ui.ir_colormap,
                                             vmin=vmin, vmax=vmax, upscale=1)
                    await _broadcast_bytes(clients, pack_ir_image(rgb))
                else:
                    _log_debounced(state, bus, "ir-miss",
                                   "reflectance unavailable this frame, holding IR pane")

        # Metrics + bus drain on the slowest cadence.
        if now - last_metrics >= METRICS_INTERVAL:
            last_metrics = now
            snap = metrics.snapshot(now)
            await _broadcast_text(clients, json.dumps(build_metrics_message(snap)))
            for line in bus.drain(bus_handle):
                msg = classify_bus_line(line, state.command_labels)
                if msg is not None:
                    await _broadcast_text(clients, json.dumps(msg))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    state = app.state
    clients: set = state.clients
    clients.add(websocket)

    # Bring the new tab current immediately.
    try:
        await websocket.send_text(json.dumps(_state_message(state.ui_state)))
    except Exception:
        await _drop_client(clients, websocket)
        return

    try:
        while True:
            data = await websocket.receive_text()
            try:
                await _handle_inbound(state, json.loads(data))
            except Exception as exc:  # a malformed inbound message must never kill the loop (§9)
                log.warning("bad inbound ws message: %r (%s)", data[:200], exc)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("ws receive loop error: %r", exc)
    finally:
        clients.discard(websocket)


async def _handle_inbound(state, msg: dict) -> None:
    """Route one decoded inbound JSON message by `type` (§5.4)."""
    mtype = msg.get("type")
    ui: UiState = state.ui_state

    if mtype == "cmd":
        resolved = resolve_command(msg.get("name"), msg.get("param", 0))
        if resolved is None:
            log.warning("unknown/invalid cmd request: %r", msg)
            return
        code, param, label = resolved
        state.command_labels.add(label)
        state.dispatcher.dispatch(code, param, label)   # result lands on the bus -> broadcast

    elif mtype == "set_color":
        mode = msg.get("mode")
        if mode not in _VALID_COLOR_MODES:
            log.warning("invalid set_color mode: %r", mode)
            return
        ui.color_mode = mode
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_ir":
        colormap = msg.get("colormap", ui.ir_colormap)
        if colormap not in _VALID_IR_COLORMAPS:
            log.warning("invalid set_ir colormap: %r", colormap)
            return
        freeze = bool(msg.get("freeze", ui.ir_freeze))
        ui.ir_colormap = colormap
        if freeze and not ui.ir_freeze:
            ui.ir_freeze_range = None     # arm capture: next IR tick grabs ir_range
        elif not freeze:
            ui.ir_freeze_range = None
        ui.ir_freeze = freeze
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    else:
        log.warning("unknown inbound message type: %r", mtype)


# --- CLI entry point --------------------------------------------------------

def main(argv=None) -> int:
    args = resolve_args(argv)

    source = FileSource(args.replay) if args.replay else get_best_source(args.port, args.baud)
    # Name the transport up front: the #1 "no data" question is whether we're on
    # Ethernet, serial, or a dead serial fallback. Flushed so it shows even when
    # stdout is block-buffered (not a tty).
    if isinstance(source, UdpSource):
        print(f"[source] Ethernet/UDP -> {source.target_ip}:{source.target_port}", flush=True)
    elif isinstance(source, SerialSource):
        print(f"[source] Serial CDC -> {getattr(source, 'port', '?')}", flush=True)
    elif isinstance(source, FileSource):
        print(f"[source] Replay -> {args.replay}", flush=True)

    # client is None in replay (no device to command); CommandDispatcher then
    # reports "not available in replay" for every dispatch.
    client = CommandClient(source.write) if isinstance(source, (SerialSource, UdpSource)) else None
    decoder = StreamDecoder()
    stats = Stats()
    bus = LogBus()
    metrics = MetricsRegistry(window_s=2.0)
    dispatcher = CommandDispatcher(client, on_message=bus.publish)

    # Always compute all three planes: marginal cost per plane is ~zero and it
    # makes color mode a pure runtime choice (no reader restart) -- §5.1/§7.2.
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"))
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    pacer = panel._Pacer(interval=(1.0 / args.replay_fps
                                   if (args.replay and args.replay_fps and args.replay_fps > 0) else 0.0))

    # Shared app state, built once (§5.1).
    app.state.args = args
    app.state.source = source
    app.state.client = client
    app.state.stage = stage
    app.state.slot = slot
    app.state.bus = bus
    app.state.metrics = metrics
    app.state.dispatcher = dispatcher
    app.state.fault = fault
    app.state.fault_reported = False
    app.state.stats = stats
    app.state.pacer = pacer
    app.state.ui_state = UiState()
    app.state.deproj = None
    app.state.clients = set()
    app.state.command_labels = set()
    app.state.debounce = {}
    app.state.ready = True

    # Reuse the panel's reader body -- it already routes EVENT->bus, ACK->client,
    # feeds metrics per DATA frame, and honors the pacer (§5.2). recorder=None
    # (Phase 3), is_stopped always False (no stop affordance yet), state=None
    # (Phase 2 sensors).
    threading.Thread(
        target=panel._run_reader,
        args=(source, decoder, stage, stats, slot, fault, bus, client, None,
              pacer, lambda: False),
        kwargs={"state": None, "metrics": metrics},
        daemon=True,
    ).start()

    port = 8000
    url = f"http://localhost:{port}/static/index.html"
    print("\n=== roomscan web viewer ===")
    print(f"Starting server on {url}")
    print("Press Ctrl+C to stop.")

    # Small delay to let the server start before opening the browser.
    threading.Timer(1.0, lambda: _open_browser(url)).start()

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")
    return 0


def _open_browser(url: str) -> None:
    """Open the viewer, and on Linux launch Chrome/Chromium with software-WebGL
    enabled. On a headless host (no GPU -- the whole point of this deployment)
    Chrome refuses to create a WebGL context by default, so the Three.js viewer
    dies with "Error creating WebGL context" and the page is stuck at "Offline"
    (confirmed on-box 2026-07-15: baseline Chrome -> NO-WEBGL; with the flag ->
    WEBGL-OK via SwiftShader/llvmpipe). `--enable-unsafe-swiftshader` only
    *permits* the software fallback -- a machine with a real GPU still uses it,
    so this is safe to pass unconditionally. Set ROOMSCAN_NO_BROWSER=1 to skip
    the auto-open entirely (e.g. when viewing from another machine)."""
    if os.environ.get("ROOMSCAN_NO_BROWSER"):
        print(f"[browser] auto-open disabled; open {url} yourself.", flush=True)
        return
    if sys.platform.startswith("linux"):
        for exe in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
            path = shutil.which(exe)
            if path:
                try:
                    subprocess.Popen(
                        [path, "--enable-unsafe-swiftshader", url],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[browser] opened {exe} with software WebGL enabled.", flush=True)
                    return
                except Exception as exc:
                    print(f"[browser] {exe} launch failed ({exc}); falling back.", flush=True)
                    break
        print("[browser] no Chrome/Chromium found. If the viewer shows 'Offline' "
              "with a WebGL error, launch your browser with software WebGL "
              "(Chrome: --enable-unsafe-swiftshader).", flush=True)
    webbrowser.open(url)


if __name__ == "__main__":
    sys.exit(main())
