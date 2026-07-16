"""Headless web-UI screenshotter + driver for the `roomscan-web` viewer.

This box has **no display and no Chrome extension**, so the mcp browser tools and
any on-screen click are unavailable. This tool is how a session visually confirms
AND controls the web UI here: it launches headless Chrome (software WebGL via
SwiftShader), drives the page over the Chrome DevTools Protocol (navigate, run JS
to click/toggle/type, wait), and captures full-viewport PNGs you then Read back.

Created 2026-07-16 (Web Phase 1 verification). See docs/web-ui-testing.md for the
full recipe (start a replay server first, then run this against it).

    # 1. start the server against a replay, DETACHED, in its own shell
    #    (Bash sandbox kills listeners -> run with dangerouslyDisableSandbox):
    ROOMSCAN_NO_BROWSER=1 setsid host/.venv/bin/python -m roomscan.web \
        --replay <capture.bin> --replay-fps 20 >/tmp/web.log 2>&1 < /dev/null &

    # 2. screenshot the default load:
    host/.venv/bin/python host/tools/web_ui_shot.py --out /tmp/01.png

    # 3. drive interactions, one PNG per step (js runs in the page; click by id):
    host/.venv/bin/python host/tools/web_ui_shot.py --out /tmp/load.png --steps '[
      {"js":"document.getElementById(\"btn-ping\").click()","wait":1.2,"out":"/tmp/02-toast.png"},
      {"js":"document.querySelector(\"#seg-color button[data-mode=depth]\").click()","wait":2,"out":"/tmp/03-depth.png"}
    ]'

Each step: {"js": <expr run via Runtime.evaluate, awaited>, "wait": <s>, "out": <png>}.
Requires only stdlib + `websockets` (already in the `[web]` extra). Manages its own
Chrome unless --port points at an already-running remote-debugging instance.
"""
from __future__ import annotations

import argparse
import asyncio
import base64
import json
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

import websockets  # from the [web] extra

DEFAULT_URL = "http://localhost:8000/static/index.html"
CHROME_CANDIDATES = ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser")


def _find_chrome(explicit: str | None) -> str:
    if explicit:
        return explicit
    for exe in CHROME_CANDIDATES:
        path = shutil.which(exe)
        if path:
            return path
    sys.exit(f"[web_ui_shot] no Chrome/Chromium found (looked for {CHROME_CANDIDATES})")


def _cdp_target(port: int, timeout_s: float = 12.0) -> str:
    """Poll the CDP HTTP endpoint until a `page` target with a ws URL appears."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(f"http://localhost:{port}/json", timeout=5) as r:
                tabs = json.loads(r.read())
            page = next((t for t in tabs if t.get("type") == "page" and t.get("webSocketDebuggerUrl")), None)
            if page:
                return page["webSocketDebuggerUrl"]
        except Exception:
            pass
        time.sleep(0.25)
    sys.exit(f"[web_ui_shot] no CDP page target on port {port} within {timeout_s}s")


async def _run(ws_url: str, args, steps: list[dict]) -> None:
    _id = 0
    async with websockets.connect(ws_url, max_size=None) as ws:
        async def cmd(method: str, params: dict | None = None) -> dict:
            nonlocal _id
            _id += 1
            mid = _id
            await ws.send(json.dumps({"id": mid, "method": method, "params": params or {}}))
            while True:
                msg = json.loads(await ws.recv())
                if msg.get("id") == mid:
                    if "error" in msg:
                        print(f"[web_ui_shot] CDP {method} error: {msg['error']}")
                    return msg.get("result", {})

        async def shot(path: str) -> None:
            r = await cmd("Page.captureScreenshot", {"format": "png"})
            data = r.get("data")
            if not data:
                print(f"[web_ui_shot] screenshot failed for {path}")
                return
            with open(path, "wb") as f:
                f.write(base64.b64decode(data))
            print(f"SAVED {path}")

        await cmd("Page.enable")
        await cmd("Runtime.enable")
        await cmd("Emulation.setDeviceMetricsOverride",
                  {"width": args.width, "height": args.height, "deviceScaleFactor": 1, "mobile": False})
        await cmd("Page.navigate", {"url": args.url})
        await asyncio.sleep(args.settle)   # let WS connect + frames stream + scene paint

        await shot(args.out)
        for st in steps:
            expr = st.get("js")
            if expr:
                res = await cmd("Runtime.evaluate", {"expression": expr, "awaitPromise": True})
                exc = res.get("exceptionDetails")
                if exc:
                    print(f"[web_ui_shot] JS error in step {expr!r}: {exc.get('text')}")
            await asyncio.sleep(st.get("wait", 3))
            if st.get("out"):
                await shot(st["out"])

        # Dump the on-page diag log tail -- the fastest signal for a load failure.
        d = await cmd("Runtime.evaluate", {"expression":
            "(document.getElementById('diag-log')||{}).textContent || 'no-diag-panel'"})
        tail = (d.get("result", {}).get("value") or "")[-900:]
        print("--- diag-log tail ---")
        print(tail)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Headless-Chrome screenshotter/driver for the roomscan web UI.")
    ap.add_argument("--url", default=DEFAULT_URL)
    ap.add_argument("--out", default="/tmp/web_ui_shot.png", help="PNG path for the initial (post-settle) shot")
    ap.add_argument("--steps", default="[]",
                    help="JSON list of {js,wait,out} interaction steps, or @path to a JSON file")
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=1000)
    ap.add_argument("--settle", type=float, default=8.0, help="seconds after navigate before the first shot")
    ap.add_argument("--port", type=int, default=0,
                    help="reuse an existing Chrome remote-debugging port instead of launching one")
    ap.add_argument("--chrome", default=None, help="explicit Chrome/Chromium binary path")
    args = ap.parse_args(argv)

    raw = args.steps
    if raw.startswith("@"):
        with open(raw[1:]) as f:
            raw = f.read()
    try:
        steps = json.loads(raw)
    except json.JSONDecodeError as exc:
        sys.exit(f"[web_ui_shot] --steps is not valid JSON: {exc}")

    proc = None
    port = args.port
    profile = None
    try:
        if port == 0:
            port = 9222
            chrome = _find_chrome(args.chrome)
            profile = tempfile.mkdtemp(prefix="web-ui-shot-chrome-")
            # --enable-unsafe-swiftshader permits the software WebGL fallback a
            # GPU-less host needs; without it Three.js can't create a context.
            proc = subprocess.Popen(
                [chrome, "--headless=new", f"--remote-debugging-port={port}",
                 "--enable-unsafe-swiftshader", "--use-gl=angle", "--use-angle=swiftshader",
                 "--no-sandbox", "--disable-dev-shm-usage", f"--user-data-dir={profile}",
                 "about:blank"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ws_url = _cdp_target(port)
        asyncio.run(_run(ws_url, args, steps))
        return 0
    finally:
        if proc is not None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        if profile is not None:
            shutil.rmtree(profile, ignore_errors=True)


if __name__ == "__main__":
    sys.exit(main())
