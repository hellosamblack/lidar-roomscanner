"""Web-based point-cloud viewer. Runs the same reader thread as the desktop viewer,
but serves the 3D data over a WebSocket to a Three.js frontend."""
from __future__ import annotations

import asyncio
import json
import os
import queue
import shutil
import subprocess
import sys
import threading
import time
import webbrowser
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .colors import turbo
from .control import CommandClient
from .decoder import StreamDecoder
from .deproject import Deprojector
from .pipeline import TransformStage
from .protocol import CommandCode
from .sources import FileSource, SerialSource, UdpSource, get_best_source
from .viewer import Stats, CommandKeyState, _reader, resolve_args

app = FastAPI()

# Ensure static directory exists
static_dir = Path(__file__).parent / "static"
static_dir.mkdir(exist_ok=True)
app.mount("/static", StaticFiles(directory=str(static_dir), html=True), name="static")

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    state = app.state
    deproj = None
    
    # Task to receive commands from the client
    async def receive_commands():
        try:
            while True:
                data = await websocket.receive_text()
                try:
                    msg = json.loads(data)
                    cmd = msg.get("cmd")
                    if cmd == "ping":
                        state.cmd_state.dispatch(CommandCode.PING, 0, "ping")
                    elif cmd == "calib":
                        state.cmd_state.dispatch(CommandCode.SEND_CALIB, 0, "calib")
                    elif cmd == "reinit":
                        state.cmd_state.dispatch(CommandCode.REINIT, 0, "reinit")
                    elif cmd == "usecase_0":
                        state.cmd_state.dispatch(CommandCode.SET_USECASE, 0, "usecase 0")
                    elif cmd == "usecase_1":
                        state.cmd_state.dispatch(CommandCode.SET_USECASE, 1, "usecase 1")
                except Exception as e:
                    print(f"Error handling websocket message: {e}")
        except WebSocketDisconnect:
            pass

    receiver_task = asyncio.create_task(receive_commands())

    try:
        while True:
            if state.fault:
                print(f"\nreader stopped: {state.fault['error']!r}", flush=True)
                # Tell the page instead of leaving it silently blank: a text
                # frame the frontend renders in the status line (binary frames
                # are point data; this is the one text-typed message).
                try:
                    await websocket.send_text(json.dumps(
                        {"type": "error", "message": str(state.fault["error"])}))
                except Exception:
                    pass
                break
                
            try:
                # We use a short asyncio.sleep to yield to the event loop
                item = state.slot.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.01)
                continue
                
            _hdr, outputs = item
            depth = outputs["depth"]
            h, w = depth.shape
            
            if deproj is None:
                deproj = Deprojector(w, h, state.args.fov_h, state.args.fov_v)
                
            pts = deproj(depth)
            
            if len(pts) > 0:
                plane = None if state.args.color == "depth" else outputs.get(state.args.color)
                if plane is not None:
                    valid = np.isfinite(depth) & (depth > 0.0) & (depth < deproj.max_range_mm)
                    vals = plane[valid].astype(np.float64, copy=False)
                else:
                    vals = pts[:, 2]
                
                vn = (vals - vals.min()) / max(float(np.ptp(vals)), 1e-6)
                colors = turbo(vn)
            else:
                pts = np.zeros((0, 3), dtype=np.float32)
                colors = np.zeros((0, 3), dtype=np.float32)

            # Flatten to 1D arrays
            pts_flat = pts.astype(np.float32).flatten()
            colors_flat = colors.astype(np.float32).flatten()
            
            # Concatenate positions then colors
            payload = np.concatenate([pts_flat, colors_flat]).tobytes()
            
            try:
                await websocket.send_bytes(payload)
            except WebSocketDisconnect:
                break
    finally:
        receiver_task.cancel()


def main(argv=None) -> int:
    args = resolve_args(argv)
    
    source = FileSource(args.replay) if args.replay else get_best_source(args.port, args.baud)
    # Name the transport up front: the #1 "no data" question is whether we're
    # on Ethernet, serial, or a dead serial fallback. Flushed so it shows even
    # when stdout is block-buffered (not a tty).
    if isinstance(source, UdpSource):
        print(f"[source] Ethernet/UDP -> {source.target_ip}:{source.target_port}", flush=True)
    elif isinstance(source, SerialSource):
        print(f"[source] Serial CDC -> {getattr(source, 'port', '?')}", flush=True)
    elif isinstance(source, FileSource):
        print(f"[source] Replay -> {args.replay}", flush=True)
    client = CommandClient(source.write) if isinstance(source, (SerialSource, UdpSource)) else None
    cmd_state = CommandKeyState(client)
    decoder = StreamDecoder()
    stats = Stats()
    
    stage_outputs = ("depth",) if args.color == "depth" else ("depth", args.color)
    stage = TransformStage(outputs=stage_outputs)
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}
    
    min_interval = 1.0 / args.replay_fps if (args.replay and args.replay_fps > 0) else 0.0
    
    # Store global state on the app for the websocket handler
    app.state.args = args
    app.state.cmd_state = cmd_state
    app.state.slot = slot
    app.state.fault = fault
    
    threading.Thread(target=_reader,
                     args=(source, decoder, slot, stats, args.record, fault, min_interval, stage, client),
                     daemon=True).start()

    # Watchdog: the reader swallows exceptions into `fault` (so one bad frame
    # can't kill the process); without this, a faulted reader just blanks the
    # page with no clue why. Surface it loudly to stderr the moment it happens.
    def _watch_fault():
        while True:
            if fault:
                print(f"\n[FATAL] reader thread stopped: {fault['error']!r}",
                      file=sys.stderr, flush=True)
                return
            time.sleep(0.5)
    threading.Thread(target=_watch_fault, daemon=True).start()
                     
    port = 8000
    url = f"http://localhost:{port}/static/index.html"
    print(f"\n=== roomscan web viewer ===")
    print(f"Starting server on {url}")
    print("Press Ctrl+C to stop.")

    # Small delay to let the server start before opening the browser
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
