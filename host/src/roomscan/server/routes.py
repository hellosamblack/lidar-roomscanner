"""HTTP route handlers (/thumb, /capture_video, /api/*, redirects)."""
from __future__ import annotations

import asyncio
import logging
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

from fastapi import Request
from fastapi.responses import (
    FileResponse,
    JSONResponse,
    RedirectResponse,
    Response,
    StreamingResponse,
)

from .. import thumbs as thumbs_mod
from ..camera import CameraFrame, LatestFrameSlot
from ..sources import UdpSource
from .session import *

log = logging.getLogger("roomscan.web")

REPO_ROOT = Path(__file__).resolve().parents[4]
RESTART_DELAY_S = 2.0

def _get_app_state(request: Request | None = None):
    if request is not None and hasattr(request, "app") and hasattr(request.app, "state"):
        return request.app.state
    from .app import app
    return app.state

async def root_redirect() -> RedirectResponse:
    # Temporary, not permanent: browsers cache a 308 indefinitely, which would
    # outlive any future move of the app off /static.
    return RedirectResponse(url="/static/index.html", status_code=307)




def managed_unit() -> str | None:
    """The systemd unit supervising this server, or None if unsupervised.

    Set by the unit file (`Environment=ROOMSCAN_MANAGED=roomscan-web.service`).
    When present, /api/restart must NOT spawn its own relaunch child: systemd's
    `Restart=always` already owns the process, so a second detached relaunch
    races it for port 8000 + the device stream -- a permanent double-bind
    crash-loop (#207). We just exit and let systemd bring the server back.
    """
    return os.environ.get("ROOMSCAN_MANAGED") or None


def restart_argv() -> list[str]:
    """The command that relaunches this server.

    Rebuilt as `-m roomscan.web` rather than reusing sys.argv[0]: under `python
    -m`, argv[0] is the expanded path to web.py, and re-running THAT executes
    the module as __main__ outside its package, which breaks its relative
    imports. Flags after argv[0] are preserved so a restart keeps --port etc.
    """
    return [sys.executable, "-m", "roomscan.web", *sys.argv[1:]]


def restart_command(delay_s: float = RESTART_DELAY_S) -> list[str]:
    """The detached `sh -c` argv that relaunches the server after `delay_s`.

    Split out from the endpoint so it is testable: exercising /api/restart in a
    test would run the endpoint's os._exit(0) and kill the test runner.
    """
    quoted = " ".join(shlex.quote(a) for a in restart_argv())
    return ["sh", "-c", f"sleep {delay_s:g}; exec {quoted}"]


async def api_bridge_reboot() -> JSONResponse:
    """Reboot the Pi bridge node (#209 management controls).

    One implementation, two front ends: this calls the exact same
    `tools.pi_bridge.reboot` the `bridge_reboot` MCP tool and the CLI call --
    there is no second copy of "how to reboot the bridge" living in the web
    layer. Runs off the event loop (it is an ssh hop) with the same wall-clock
    guard as the health poll, so a wedged bridge can never stall this request
    past a bounded timeout.
    """
    pi_bridge = _pi_bridge_module()
    try:
        host = await _resolve_bridge_host(_get_app_state())
        result = await asyncio.wait_for(
            asyncio.to_thread(pi_bridge.reboot, host), timeout=BRIDGE_HEALTH_WAIT_S)
    except asyncio.TimeoutError:
        result = {"ok": False, "error": f"timed out after {BRIDGE_HEALTH_WAIT_S:g}s"}
    bus = getattr(_get_app_state(), "bus", None)
    if bus is not None:
        bus.publish(f"[bridge] reboot {'ok' if result.get('ok') else 'FAILED'}: "
                    f"{result.get('error') or result.get('note') or 'requested'}")
    return JSONResponse(result)


async def api_bridge_logs(request: Request) -> JSONResponse:
    """Tail one unit's journal on the Pi bridge (#209 management controls).

    Same pure `tools.pi_bridge.logs` the `bridge_logs` MCP tool wraps. Body is
    optional JSON `{"unit": "...", "lines": 200}`; `logs()` itself rejects an
    unsafe unit name, so this endpoint does no extra validation beyond parsing.
    """
    try:
        body = await request.json()
    except Exception:
        body = {}
    unit = (body or {}).get("unit") or "roomscan-bridge"
    lines = int((body or {}).get("lines") or 200)
    pi_bridge = _pi_bridge_module()
    try:
        host = await _resolve_bridge_host(_get_app_state())
        result = await asyncio.wait_for(
            asyncio.to_thread(pi_bridge.logs, unit, lines, host), timeout=BRIDGE_HEALTH_WAIT_S)
    except asyncio.TimeoutError:
        result = {"ok": False, "error": f"timed out after {BRIDGE_HEALTH_WAIT_S:g}s", "unit": unit}
    return JSONResponse(result)


async def api_restart() -> JSONResponse:
    """Relaunch the server process.

    Under systemd supervision (`managed_unit()` set) we do NOT spawn a relaunch
    child: we just exit, and `Restart=always` respawns the server. Spawning our
    own child there would race systemd for port 8000 + the device stream and
    crash-loop the loser forever (#207).

    Unsupervised, we spawn a detached child that waits for this process to
    release port 8000, then execs a fresh server; this process exits once the
    response is on the wire. stdout/stderr are inherited so the new process keeps
    writing to whatever log the old one had (e.g. /tmp/web-live.log).
    """
    unit = managed_unit()
    if unit:
        async def _exit_soon_managed() -> None:
            await asyncio.sleep(0.3)   # let the JSON response flush
            os._exit(0)                # systemd Restart=always respawns us
        asyncio.create_task(_exit_soon_managed())
        return JSONResponse({"ok": True, "restart_via": "systemd", "unit": unit,
                             "restart_in_s": RESTART_DELAY_S})

    argv = restart_argv()
    try:
        subprocess.Popen(
            restart_command(), cwd=str(REPO_ROOT), start_new_session=True,
            stdin=subprocess.DEVNULL,
        )
    except OSError as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)

    async def _exit_soon() -> None:
        # Long enough for the JSON response to flush to the client; the child is
        # already scheduled, so the port is free well before it tries to bind.
        await asyncio.sleep(0.3)
        os._exit(0)

    asyncio.create_task(_exit_soon())
    return JSONResponse({"ok": True, "restart_in_s": RESTART_DELAY_S,
                         "argv": argv})



# --- Pi-bridge camera proxy: /api/camera/* (#226 Stage A, spec §5.4) --------
#
# Three routes over ONE upstream connection. The `CameraClient` on
# `app.state.camera` owns that connection (started by lifespan, see app.py);
# everything here reads its latest-frame slot. A second browser tab therefore
# costs one more host socket and ZERO additional Pi camera opens -- the §4.3
# invariant, and the reason the routes never construct a client of their own.
#
# Bounding the slow-viewer case needed no new mechanism, only the right one:
# unlike uvicorn's WEBSOCKET send (a bare `transport.write`, which is what left
# `/ws` unbounded until #217), uvicorn's HTTP send calls `await
# self.flow.drain()` whenever the transport's write buffer is over its 64 KiB
# high-water mark. So awaiting each part's send before selecting the next frame
# is genuine backpressure, and `wait_next` then hands back the NEWEST frame
# rather than the next one in line. A stalled viewer holds exactly one JPEG.

CAMERA_BOUNDARY = "frame"
CAMERA_MEDIA_TYPE = f"multipart/x-mixed-replace; boundary={CAMERA_BOUNDARY}"

#: Every camera response. A cached MJPEG stream or a cached "current" frame is
#: a stale picture presented as live, which is the one thing a monitor must not do.
CAMERA_NO_STORE = {"Cache-Control": "no-store, no-cache, must-revalidate",
                   "Pragma": "no-cache"}


def _camera_client(state):
    """The lifespan-owned `CameraClient`, or None when the server was built
    without one (every unit test that does not exercise the camera)."""
    return getattr(state, "camera", None)


def camera_frame_headers(frame: CameraFrame, age_ms: float | None) -> dict:
    """The `X-Roomscan-*` headers a single frame carries out of this host.

    Deliberately the SAME names and the same value spellings the Pi daemon uses
    (spec §5.2) with the host's own generation substituted, so a client --
    browser, MCP snapshot, curl -- reads one vocabulary end to end. In
    particular the flag is `X-Roomscan-Capture-Timestamp-Valid: true|false`, the
    daemon's name, and NOT a second host-invented spelling beside it: two
    headers asserting the same fact are two things that can disagree.

    `age_ms` is added here and exists nowhere upstream: it is a host-to-host
    measurement (arrival to now), named `age` and never `latency`, because no
    clock has been mapped (§4.5).
    """
    headers = {
        "X-Roomscan-Camera-Generation": str(frame.generation),
        "X-Roomscan-Camera-Seq": str(frame.seq),
        "X-Roomscan-Timestamp-Domain": frame.timestamp_domain or "unknown",
        "X-Roomscan-Capture-Timestamp-Valid": (
            "true" if frame.timestamp_valid else "false"),
    }
    if frame.pi_capture_mono_ns is not None:
        headers["X-Roomscan-Capture-Mono-Ns"] = str(int(frame.pi_capture_mono_ns))
    # Passed straight through when the daemon sent one. It is a DIAGNOSTIC, not
    # a second capture time: a consumer that fell back to it would silently be
    # reading buffer-delivery instants as exposure instants (§5.2).
    if frame.pi_dequeue_mono_ns is not None:
        headers["X-Roomscan-Capture-Dequeue-Mono-Ns"] = str(
            int(frame.pi_dequeue_mono_ns))
    if frame.source_generation is not None:
        headers["X-Roomscan-Source-Generation"] = str(frame.source_generation)
    if age_ms is not None:
        headers["X-Roomscan-Camera-Age-Ms"] = f"{age_ms:.1f}"
    return headers


def encode_mjpeg_part(frame: CameraFrame, boundary: str = CAMERA_BOUNDARY) -> bytes:
    """One `multipart/x-mixed-replace` part, byte-for-byte per spec §5.2.

    Pure, so the framing this server EMITS is tested by the same
    `MultipartParser` that reads the daemon's -- a round trip, rather than two
    hand-written layouts that agree only until one of them is edited.
    """
    head = [f"--{boundary}", "Content-Type: image/jpeg",
            f"Content-Length: {len(frame.jpeg)}"]
    for name, value in camera_frame_headers(frame, None).items():
        head.append(f"{name}: {value}")
    return ("\r\n".join(head) + "\r\n\r\n").encode("latin-1") + frame.jpeg + b"\r\n"


async def _camera_parts(slot: LatestFrameSlot, boundary: str = CAMERA_BOUNDARY):
    """Latest-only async generator over `slot`.

    The subscriber counter is incremented/decremented in a `finally`, so a
    browser that vanishes mid-frame does not leave the card reporting a viewer
    that is gone -- connection count is a real performance variable on this
    server (BUG-060) and a lying one is worse than none.
    """
    slot.subscribers += 1
    tick = 0
    try:
        while True:
            tick, frame = await slot.wait_next(tick)
            # `yield` returns only after Starlette's `await send(...)` for the
            # previous part has completed. That await IS the bound.
            yield encode_mjpeg_part(frame, boundary)
    finally:
        slot.subscribers = max(0, slot.subscribers - 1)


async def api_camera_stream(request: Request) -> Response:
    """`GET /api/camera/stream.mjpg` -- latest-only re-fanout of the Pi feed.

    503 (not an empty 200) when there is no camera ingest at all, so an `<img>`
    fails visibly and the card can say Offline rather than showing a blank
    frame that could equally mean "black room".
    """
    client = _camera_client(_get_app_state(request))
    if client is None:
        return JSONResponse({"error": "camera ingest is not running"},
                            status_code=503, headers=CAMERA_NO_STORE)
    return StreamingResponse(_camera_parts(client.slot),
                             media_type=CAMERA_MEDIA_TYPE, headers=CAMERA_NO_STORE)


async def api_camera_frame(request: Request) -> Response:
    """`GET /api/camera/frame.jpg` -- the current frame, exact original bytes.

    What Snapshot and `camera_snapshot()` fetch. No re-encode: the JPEG handed
    back is the one the C270 produced, which is what Stage B (#227) will archive
    as authoritative.
    """
    client = _camera_client(_get_app_state(request))
    if client is None:
        return JSONResponse({"error": "camera ingest is not running"},
                            status_code=503, headers=CAMERA_NO_STORE)
    _, frame = client.slot.latest()
    if frame is None:
        return JSONResponse({"error": "no camera frame yet",
                             "state": client.state()},
                            status_code=503, headers=CAMERA_NO_STORE)
    age_s = client.age_s()
    headers = dict(CAMERA_NO_STORE)
    headers.update(camera_frame_headers(
        frame, None if age_s is None else age_s * 1000.0))
    return Response(content=frame.jpeg, media_type="image/jpeg", headers=headers)


async def api_camera_status(request: Request) -> JSONResponse:
    """`GET /api/camera/status` -- host ingest state plus the Pi's own report.

    Reports what was measured, never what was configured: `width`/`height` come
    off the last JPEG's SOF marker, `host_fps` off arrival times, and the Pi's
    `/status.json` is passed through verbatim under `source` rather than
    re-summarised into fields that could drift from it.
    """
    client = _camera_client(_get_app_state(request))
    if client is None:
        return JSONResponse({"state": "offline", "running": False,
                             "error": "camera ingest is not running"},
                            headers=CAMERA_NO_STORE)
    payload = client.status()
    payload["running"] = client.running
    return JSONResponse(payload, headers=CAMERA_NO_STORE)


# --- capture thumbnails: GET /thumb/{name} (§12) ----------------------------
#
# Delivered over HTTP, deliberately NOT as a binary `/ws` tag. An `<img
# loading="lazy">` gives viewport-paced fetching, browser disk caching,
# per-image cancellation and parallelism for free; a TAG_THUMB would need
# request/response correlation bolted onto a channel that is strictly one-way
# server push, and ~34 x ~25 KB PNGs would interleave with the 30 Hz POINT_CLOUD
# and add head-of-line delay to the live render. `/results` is the precedent for
# file-shaped payloads over HTTP.
#
# `sanitize_capture_name` is the WHOLE security surface, same as `load_capture`:
# basename-only, `.bin`-suffixed, must exist under the captures dir.

_THUMB_SEM = asyncio.Semaphore(2)          # concurrent generations, see below
_THUMB_INFLIGHT: dict[str, asyncio.Future] = {}


def _captures_dir_of(state) -> str:
    ctrl = getattr(state, "controller", None)
    return getattr(ctrl, "captures_dir", None) or CAPTURES_DIR


async def _ensure_thumb(path: Path) -> str:
    """Generate `path`'s thumbnail off the event loop, coalescing duplicates.

    A grid of lazy `<img>` tags fires many requests at once and a browser will
    happily re-request the same tile. Two guards:

      * a `Semaphore(2)` so thumbnail generation cannot monopolise the shared
        default executor -- `load_capture`, `seek` and `_broadcast_captures` all
        run their filesystem work in `to_thread` too, and a 34-tile grid would
        otherwise starve them;
      * an in-flight future map so N concurrent requests for the SAME capture do
        the work once.
    """
    key = str(path)
    fut = _THUMB_INFLIGHT.get(key)
    if fut is not None:
        return await asyncio.shield(fut)
    loop = asyncio.get_running_loop()
    fut = loop.create_future()
    _THUMB_INFLIGHT[key] = fut
    try:
        async with _THUMB_SEM:
            kind = await asyncio.to_thread(thumbs_mod.make_thumbnail, path,
                                           thumbs_mod.THUMBS_DIR)
        fut.set_result(kind)
        return kind
    except Exception as exc:                          # never propagate: see below
        fut.set_result("")
        log.warning("thumbnail generation failed for %s: %s", path.name, exc)
        return ""
    finally:
        _THUMB_INFLIGHT.pop(key, None)


async def get_thumb(name: str) -> Response:
    """A capture's thumbnail PNG, generated on demand.

    NEVER 500s (the `/api/*` precedent): an unknown name, a truncated capture or
    a render failure are all 404 + `no-store`, so the browser shows the tile's
    placeholder and retries next time rather than caching a failure.

    A successful response is `immutable` because the capture's identity is baked
    into both the cache filename (`<stem>__<size>_<mtime_ns>.png`) and the
    client's `?v=` -- rewriting the capture changes the URL.
    """
    miss = Response(status_code=404, headers={"Cache-Control": "no-store"})
    try:
        path = sanitize_capture_name(name, _captures_dir_of(_get_app_state()))
        if path is None:
            return miss
        dest = thumbs_mod.thumb_path(path, thumbs_mod.THUMBS_DIR)
        if not dest.is_file():
            if await _ensure_thumb(path) == "":
                return miss
        data = await asyncio.to_thread(dest.read_bytes)
    except Exception as exc:
        log.warning("thumb route failed for %r: %s", name, exc)
        return miss
    if not data:
        return miss
    return Response(content=data, media_type="image/png",
                    headers={"Cache-Control": "public, max-age=31536000, immutable"})


async def get_capture_video(name: str) -> Response:
    """Stream a source phone video (`captures/<name>.mp4`) for in-browser preview.

    `FileResponse` is Range-capable, so the browser `<video>` seeks and streams
    without the server ever reading the multi-hundred-MB file into memory. A narrow
    route (not a `captures/` static mount) keeps the surface to `sanitize_video_name`
    and exposes no directory listing. Unknown/absent -> 404 + no-store."""
    path = sanitize_video_name(name, _captures_dir_of(_get_app_state()))
    if path is None:
        return Response(status_code=404, headers={"Cache-Control": "no-store"})
    return FileResponse(str(path), headers={"Cache-Control": "no-store"})



__all__ = [k for k in globals() if not k.startswith("__")]
