#!/usr/bin/env python3
"""Does `roomscan-web` still exit on SIGTERM while a client holds the camera stream?

The live defect this exists to keep fixed (#226, 2026-09-01): with a browser tab
holding `<img src="/api/camera/stream.mjpg">` open, `systemctl --user restart
roomscan-web` sat in `stop-sigterm` for the whole `TimeoutStopSec=90` and was
SIGKILLed. Measured here at the time: **0.21 s** to exit with nothing attached,
**never** (>122 s) with one reading client attached.

Why it needs a real process and a real socket, and could never have been a unit
test: uvicorn's HTTP `shutdown()` only sets `cycle.keep_alive = False` when a
response is in flight and leaves the transport open
(`uvicorn/protocols/http/httptools_impl.py:349`), unlike the WebSocket path
which calls `transport.close()` unconditionally. `Server._wait_tasks_to_complete`
then spins `while self.server_state.connections`, and `timeout_graceful_shutdown`
defaults to `None`. The lifespan shutdown -- where `CameraClient.stop()` lives --
is only reached *after* that wait. So the app-level start/stop tests were green
and structurally could not see this: they call `_lifespan` directly and there is
no uvicorn, no socket and no signal anywhere in them. AGENTS.md's rule about
fakes eliding exactly the bug, again.

Hermetic: this stands up its own MJPEG daemon on loopback (`ROOMSCAN_CAMERA_HOST`
pins the server to it, skipping mDNS) and its own synthetic capture, so it needs
no Pi, no rig and no network. It binds only ephemeral ports -- never 8000.

    python host/tools/camera_shutdown_probe.py                # all arms
    python host/tools/camera_shutdown_probe.py --arm stream-open --json

Arms, and what each isolates:

    none            baseline: nothing attached
    frame-jpg       completed responses only -- proves a finished request is fine
    stream-closed   stream attached then closed BEFORE the signal
    stream-open     a client reading the stream at SIGTERM   <- the reported bug
    stream-stalled  a client that opened the stream and reads nothing; the
                    generator is blocked inside its own `send`, so only
                    `timeout_graceful_shutdown` can cut it
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HOST_DIR = REPO / "host"

#: Every arm must beat this. The owner's restart budget is a few seconds; the
#: unit's TimeoutStopSec=90 is the backstop, not the target.
DEFAULT_BUDGET_S = 5.0
ARMS = ("none", "frame-jpg", "stream-closed", "stream-open", "stream-stalled")


# --- a minimal JPEG, and the daemon that serves it ---------------------------

def _jpeg(width: int = 64, height: int = 48) -> bytes:
    """A structurally real JPEG: SOI, APP0, SOF0 carrying the size, EOI.

    Real enough that the host's `jpeg_dimensions` marker walk and its SOI/EOI
    check both accept it -- random bytes would be rejected as malformed and the
    stream would never go live.
    """
    app0 = b"JFIF\x00\x01\x01\x00" + b"\x00" * 5
    sof0 = (b"\xff\xc0" + (11).to_bytes(2, "big") + b"\x08"
            + height.to_bytes(2, "big") + width.to_bytes(2, "big")
            + b"\x01\x01\x11\x00")
    # ~200 KB of body, deliberately: the `stream-stalled` arm is only testing
    # what its name says if a single part overruns uvicorn's 64 KiB write-buffer
    # high-water mark, so the generator is blocked INSIDE its own `send` and the
    # shutdown gate cannot reach it. With 300-byte frames the arm passed in
    # 0.21 s by parking in `wait_next` instead -- a green result measuring the
    # wrong thing.
    return (b"\xff\xd8" + b"\xff\xe0" + (2 + len(app0)).to_bytes(2, "big") + app0
            + sof0 + b"\x5a" * 200_000 + b"\xff\xd9")


class _CameraDaemon(BaseHTTPRequestHandler):
    """`roomscan-cam`'s wire contract (spec §5.2), just enough of it."""

    jpeg = _jpeg()
    fps = 20.0

    def log_message(self, *_a):          # noqa: D102 - silence the stdlib logger
        return

    def do_GET(self):                    # noqa: N802 - stdlib naming
        if self.path.startswith("/status.json"):
            body = json.dumps({"state": "streaming", "generation": 1,
                               "width": 64, "height": 48, "fourcc": "MJPG",
                               "clients": 1, "last_error": None}).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if not self.path.startswith("/stream.mjpg"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        seq = 0
        try:
            while True:
                seq += 1
                head = ("--frame\r\n"
                        "Content-Type: image/jpeg\r\n"
                        f"Content-Length: {len(self.jpeg)}\r\n"
                        "X-Roomscan-Camera-Generation: 1\r\n"
                        f"X-Roomscan-Camera-Seq: {seq}\r\n"
                        f"X-Roomscan-Capture-Mono-Ns: {int(time.monotonic() * 1e9)}\r\n"
                        "X-Roomscan-Capture-Timestamp-Valid: true\r\n"
                        "X-Roomscan-Timestamp-Domain: pi_monotonic\r\n\r\n")
                self.wfile.write(head.encode() + self.jpeg + b"\r\n")
                self.wfile.flush()
                time.sleep(1.0 / self.fps)
        except (BrokenPipeError, ConnectionResetError, OSError):
            return


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _synthetic_capture(path: Path, n_frames: int = 40) -> None:
    """A tiny valid `.bin` so the server can `--replay` instead of binding the
    device -- which would compete with whatever is serving :8000."""
    sys.path.insert(0, str(HOST_DIR / "src"))
    import numpy as np

    from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

    w, h = 8, 6
    out = bytearray()
    for i in range(n_frames):
        q = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, i,
                                      i * 35000, 0, 0, len(q)), q)
        depth = np.full((h, w), 1000.0 + 5.0 * i, dtype=np.float32)
        payload = depth.astype("<f4").tobytes()
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0,
                                      i + 1, i * 35000, w, h, len(payload)),
                          payload)
    path.write_bytes(bytes(out))


def _get_json(url: str, timeout: float = 2.0):
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return json.loads(r.read().decode())
    except (urllib.error.URLError, OSError, ValueError):
        return None


# --- the probe ---------------------------------------------------------------

def run_arm(arm: str, *, budget_s: float = DEFAULT_BUDGET_S,
            startup_timeout_s: float = 60.0) -> dict:
    """Start a real server, attach `arm`'s client, SIGTERM, time the exit.

    Reports what was MEASURED -- `exited`, `exit_s`, the camera state actually
    reached, the client bytes actually read -- so an arm that never got the
    camera live cannot pass by accident with a fast exit.
    """
    if arm not in ARMS:
        raise ValueError(f"unknown arm {arm!r}; expected one of {ARMS}")

    cam_port, web_port = _free_port(), _free_port()
    daemon = ThreadingHTTPServer(("127.0.0.1", cam_port), _CameraDaemon)
    daemon.daemon_threads = True
    threading.Thread(target=daemon.serve_forever, daemon=True).start()

    tmp = Path(tempfile.mkdtemp(prefix="camshutdown-"))
    capture = tmp / "probe.bin"
    _synthetic_capture(capture)

    env = {**os.environ,
           "ROOMSCAN_NO_BROWSER": "1",
           "ROOMSCAN_WEB_PORT": str(web_port),
           "ROOMSCAN_CAMERA_HOST": "127.0.0.1",
           "ROOMSCAN_CAMERA_PORT": str(cam_port),
           "PYTHONPATH": f"{HOST_DIR / 'src'}:{HOST_DIR}"}
    proc = subprocess.Popen(
        [sys.executable, "-m", "roomscan.web", "--replay", str(capture),
         "--replay-fps", "10"],
        cwd=str(REPO), env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        start_new_session=True)

    # The server's OWN configured graceful timeout, imported rather than
    # re-stated here: a copy of the number would let this probe and the server
    # drift apart, and the whole point of reporting it is to say which of the
    # two mechanisms ended the process.
    sys.path.insert(0, str(HOST_DIR / "src"))
    from roomscan.server.app import SHUTDOWN_TIMEOUT_S

    result = {"arm": arm, "web_port": web_port, "camera_port": cam_port,
              "graceful_timeout_s": SHUTDOWN_TIMEOUT_S}
    client = None
    try:
        base = f"http://127.0.0.1:{web_port}"
        deadline = time.monotonic() + startup_timeout_s
        state = None
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                result["error"] = "server exited during startup"
                return result
            status = _get_json(f"{base}/api/camera/status")
            state = (status or {}).get("state")
            if state == "live":
                break
            time.sleep(0.25)
        result["camera_state"] = state
        if state != "live":
            result["error"] = f"camera never went live (state={state!r})"
            return result

        if arm == "frame-jpg":
            for _ in range(3):
                _get_json(f"{base}/api/camera/frame.jpg")  # not JSON; we only care it completes
        elif arm in ("stream-open", "stream-closed", "stream-stalled"):
            client = socket.create_connection(("127.0.0.1", web_port), timeout=10)
            client.sendall(b"GET /api/camera/stream.mjpg HTTP/1.1\r\n"
                           b"Host: 127.0.0.1\r\n\r\n")
            if arm == "stream-stalled":
                time.sleep(2.0)          # read NOTHING; the server blocks in send
                result["client_bytes"] = 0
            else:
                got, end = 0, time.monotonic() + 3.0
                client.settimeout(3.0)
                while time.monotonic() < end:
                    try:
                        chunk = client.recv(65536)
                    except OSError:
                        break
                    if not chunk:
                        break
                    got += len(chunk)
                result["client_bytes"] = got
                if arm == "stream-closed":
                    client.close()
                    client = None
                    time.sleep(1.0)
            result["clients_reported"] = (
                _get_json(f"{base}/api/camera/status") or {}).get("clients")

        start = time.monotonic()
        proc.send_signal(signal.SIGTERM)
        try:
            proc.wait(timeout=max(budget_s * 4, 30.0))
            result["exited"] = True
        except subprocess.TimeoutExpired:
            result["exited"] = False
        result["exit_s"] = round(time.monotonic() - start, 3)
        result["within_budget"] = bool(result["exited"]
                                       and result["exit_s"] <= budget_s)
        result["budget_s"] = budget_s
        # WHICH mechanism ended it, not merely that something did. Anything at
        # or near the graceful timeout was CUT by uvicorn; a prompt exit means
        # the response generator let go on its own. Conflating the two hides a
        # broken route behind a working backstop -- both mutants that removed
        # the route-level gate still "passed" a plain `exit_s < budget` check,
        # because the 3 s timeout is under the 5 s budget.
        result["via"] = ("graceful-timeout"
                         if result["exited"] and result["exit_s"] >= SHUTDOWN_TIMEOUT_S * 0.5
                         else "route" if result["exited"] else "none")
        return result
    finally:
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=10)
        daemon.shutdown()
        daemon.server_close()
        # Cleanup must never replace a real measurement with a setup error: a
        # leftover file, a race with the server's own writes, a read-only tmp
        # would all raise out of this `finally` and discard the exit time we
        # came here for. `rmtree(ignore_errors=True)` says the same thing in
        # one call, and a stray temp dir is a far cheaper failure than a lost
        # result.
        shutil.rmtree(tmp, ignore_errors=True)


def probe(arms=ARMS, *, budget_s: float = DEFAULT_BUDGET_S) -> dict:
    """Run each arm once. `ok` is true only if every arm exited within budget."""
    results = [run_arm(a, budget_s=budget_s) for a in arms]
    return {"ok": all(r.get("within_budget") for r in results),
            "budget_s": budget_s, "arms": results}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--arm", action="append", choices=ARMS,
                    help="run only this arm (repeatable); default: all")
    ap.add_argument("--budget", type=float, default=DEFAULT_BUDGET_S,
                    help=f"seconds each arm may take to exit (default {DEFAULT_BUDGET_S})")
    ap.add_argument("--json", action="store_true", help="machine-readable output")
    args = ap.parse_args(argv)

    report = probe(tuple(args.arm) if args.arm else ARMS, budget_s=args.budget)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        print(f"SIGTERM exit time, budget {report['budget_s']:.1f}s\n")
        for r in report["arms"]:
            if "error" in r:
                print(f"  {r['arm']:<15} SETUP FAILED: {r['error']}")
                continue
            verdict = "ok " if r["within_budget"] else "FAIL"
            exited = f"{r['exit_s']:.2f}s" if r["exited"] else f">{r['exit_s']:.0f}s (HUNG)"
            extra = f"  via {r.get('via')}"
            if "client_bytes" in r:
                extra += (f", client read {r['client_bytes']} B, "
                          f"clients={r.get('clients_reported')}")
            print(f"  {verdict} {r['arm']:<15} {exited}{extra}")
        print(f"\n{'PASS' if report['ok'] else 'FAIL'}")
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
