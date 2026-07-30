"""Long-lived connections held across tool calls.

This is the whole reason the dev loop is worth exposing over MCP rather than Bash:
a Bash call is a fresh process every time, so each UI check had to relaunch Chrome
and re-settle it (~8 s). The server outlives individual tool calls, so `RigSession`
keeps one `/ws` connection open and `CdpSession` keeps one browser warm.

`CdpSession` is deliberately an interface over the browser mechanism. The initial
implementation is the raw-CDP plumbing lifted from `host/tools/web_ui_shot.py`,
which is proven to render Three.js under SwiftShader on this GPU-less host; a
Playwright backend can replace it without any tool signature changing.
"""
from __future__ import annotations

import asyncio
import base64
import json
import shutil
import subprocess
import tempfile
import time
import urllib.request

from .paths import WEB_PAGE, WEB_WS

CHROME_CANDIDATES = ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser")
# Software WebGL: without these Three.js cannot get a context on a GPU-less host.
CHROME_FLAGS = ("--headless=new", "--enable-unsafe-swiftshader", "--use-gl=angle",
                "--use-angle=swiftshader", "--no-sandbox", "--disable-dev-shm-usage")


# --- rig (/ws to roomscan-web) ----------------------------------------------

class RigSession:
    """One `/ws` connection to roomscan-web, with the latest message of each type.

    The server is the single source of truth for state (docs/web-protocol.md
    §Invariants: the client never holds optimistic state), so this caches only what
    the server last broadcast and never derives anything from it.
    """

    def __init__(self, url: str = WEB_WS) -> None:
        self.url = url
        self._ws = None
        self._reader: asyncio.Task | None = None
        self.latest: dict[str, dict] = {}
        self.binary_counts: dict[int, int] = {}
        self._events: dict[str, asyncio.Event] = {}

    @property
    def connected(self) -> bool:
        return self._ws is not None and self._reader is not None and not self._reader.done()

    async def connect(self, timeout: float = 10.0) -> bool:
        if self.connected:
            return True
        import websockets

        try:
            self._ws = await asyncio.wait_for(
                websockets.connect(self.url, max_size=None), timeout=timeout)
        except Exception:
            self._ws = None
            return False
        self.latest.clear()
        self.binary_counts.clear()
        self._reader = asyncio.create_task(self._pump())
        return True

    async def close(self) -> None:
        if self._reader:
            self._reader.cancel()
            self._reader = None
        if self._ws:
            try:
                await self._ws.close()
            finally:
                self._ws = None

    async def _pump(self) -> None:
        import struct as _struct
        try:
            async for msg in self._ws:
                if isinstance(msg, (bytes, bytearray)):
                    if len(msg) >= 4:
                        tag = _struct.unpack_from("<I", msg)[0]
                        self.binary_counts[tag] = self.binary_counts.get(tag, 0) + 1
                    continue
                try:
                    d = json.loads(msg)
                except json.JSONDecodeError:
                    continue
                t = d.get("type")
                if not t:
                    continue
                self.latest[t] = d
                ev = self._events.get(t)
                if ev:
                    ev.set()
        except Exception:
            pass  # connection dropped; `connected` goes False and callers reconnect

    async def send(self, message: dict) -> None:
        if not await self.connect():
            raise RuntimeError(f"cannot reach roomscan-web at {self.url}")
        await self._ws.send(json.dumps(message))

    async def wait_for(self, type_: str, timeout: float = 5.0) -> dict | None:
        """Wait for the *next* message of `type_` (ignores an already-cached one)."""
        ev = self._events.setdefault(type_, asyncio.Event())
        ev.clear()
        try:
            await asyncio.wait_for(ev.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return None
        return self.latest.get(type_)

    async def request(self, message: dict, expect: str, timeout: float = 5.0) -> dict | None:
        """Send `message` and return the next `expect`-typed broadcast it triggers."""
        ev = self._events.setdefault(expect, asyncio.Event())
        ev.clear()
        await self.send(message)
        try:
            await asyncio.wait_for(ev.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return None
        return self.latest.get(expect)


# --- browser (CDP to headless Chrome) ---------------------------------------

class CdpSession:
    """A headless Chrome kept alive across tool calls, driven over raw CDP.

    Lifted from `host/tools/web_ui_shot.py`, which launched and tore down a browser
    per invocation. Interface: `goto`, `evaluate`, `wait_for`, `screenshot`, `reset`.
    """

    def __init__(self, port: int = 9222) -> None:
        self.port = port
        self._proc: subprocess.Popen | None = None
        self._profile: str | None = None
        self._ws = None
        self._id = 0
        self.url: str | None = None

    # -- lifecycle
    def _find_chrome(self) -> str:
        for exe in CHROME_CANDIDATES:
            path = shutil.which(exe)
            if path:
                return path
        raise RuntimeError(f"no Chrome/Chromium found (looked for {CHROME_CANDIDATES})")

    def _cdp_target(self, timeout_s: float = 15.0) -> str:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(f"http://localhost:{self.port}/json", timeout=5) as r:
                    tabs = json.loads(r.read())
                page = next((t for t in tabs
                             if t.get("type") == "page" and t.get("webSocketDebuggerUrl")), None)
                if page:
                    return page["webSocketDebuggerUrl"]
            except Exception:
                pass
            time.sleep(0.25)
        raise RuntimeError(f"no CDP page target on port {self.port} within {timeout_s}s")

    async def start(self, width: int = 1600, height: int = 1000) -> None:
        if self._ws is not None:
            return
        import websockets

        if self._proc is None or self._proc.poll() is not None:
            self._profile = tempfile.mkdtemp(prefix="roomscan-mcp-chrome-")
            self._proc = subprocess.Popen(
                [self._find_chrome(), *CHROME_FLAGS,
                 f"--remote-debugging-port={self.port}",
                 f"--user-data-dir={self._profile}", "about:blank"],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ws_url = await asyncio.to_thread(self._cdp_target)
        self._ws = await websockets.connect(ws_url, max_size=None)
        await self.cmd("Page.enable")
        await self.cmd("Runtime.enable")
        await self._disable_http_cache()
        await self.cmd("Emulation.setDeviceMetricsOverride",
                       {"width": width, "height": height,
                        "deviceScaleFactor": 1, "mobile": False})

    async def _disable_http_cache(self) -> None:
        """Always refetch; never serve a static asset from Chrome's cache.

        This browser exists to look at files that were edited seconds ago. A
        cached `index.html` or `.js` shows the PREVIOUS edit while reporting a
        successful load, which reads as "the change did not land" -- on
        2026-07-30 a modal's copy was verified twice against a cached page
        (`renavigate=True` is a navigation, not a cache bypass) and only a
        query-string cache-buster exposed it. Non-fatal: an old CDP build
        without the Network domain still gets a working, merely cache-warm
        browser.
        """
        try:
            await self.cmd("Network.enable")
            await self.cmd("Network.setCacheDisabled", {"cacheDisabled": True})
        except Exception:
            pass

    async def stop(self) -> None:
        if self._ws is not None:
            try:
                await self._ws.close()
            finally:
                self._ws = None
        if self._proc is not None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self._proc.kill()
            self._proc = None
        if self._profile:
            shutil.rmtree(self._profile, ignore_errors=True)
            self._profile = None
        self.url = None

    # -- protocol
    async def cmd(self, method: str, params: dict | None = None) -> dict:
        self._id += 1
        mid = self._id
        await self._ws.send(json.dumps({"id": mid, "method": method, "params": params or {}}))
        while True:
            msg = json.loads(await self._ws.recv())
            if msg.get("id") == mid:
                if "error" in msg:
                    raise RuntimeError(f"CDP {method}: {msg['error']}")
                return msg.get("result", {})

    # -- interface
    async def goto(self, url: str = WEB_PAGE, settle: float = 8.0) -> None:
        await self.start()
        await self.cmd("Page.navigate", {"url": url})
        self.url = url
        await asyncio.sleep(settle)

    async def evaluate(self, expression: str, await_promise: bool = True) -> dict:
        await self.start()
        r = await self.cmd("Runtime.evaluate",
                           {"expression": expression, "awaitPromise": await_promise,
                            "returnByValue": True})
        exc = r.get("exceptionDetails")
        if exc:
            return {"ok": False, "exception": exc.get("text"),
                    "detail": (exc.get("exception") or {}).get("description")}
        return {"ok": True, "value": (r.get("result") or {}).get("value")}

    async def wait_for(self, predicate_js: str, timeout_s: float = 15.0,
                       poll_s: float = 0.25) -> dict:
        """Poll `predicate_js` until truthy. Replaces sleep-and-hope."""
        await self.start()
        deadline = time.monotonic() + timeout_s
        last: dict = {}
        while time.monotonic() < deadline:
            last = await self.evaluate(f"!!({predicate_js})")
            if last.get("ok") and last.get("value"):
                return {"ok": True, "waited_s": round(
                    timeout_s - (deadline - time.monotonic()), 2)}
            await asyncio.sleep(poll_s)
        return {"ok": False, "timeout_s": timeout_s, "last": last}

    async def screenshot(self) -> bytes:
        await self.start()
        r = await self.cmd("Page.captureScreenshot", {"format": "png"})
        data = r.get("data")
        if not data:
            raise RuntimeError("screenshot returned no data")
        return base64.b64decode(data)

    async def diag_tail(self, chars: int = 900) -> str:
        r = await self.evaluate(
            "(document.getElementById('diag-log')||{}).textContent || 'no-diag-panel'",
            await_promise=False)
        return (r.get("value") or "")[-chars:]


class PlaywrightSession:
    """Same interface as `CdpSession`, backed by Playwright.

    Preferred because waiting is native: `wait_for_function` is driven by the page's
    own event loop instead of a 4 Hz poll, so `ui_wait_for` returns as soon as the
    condition holds and cannot miss a transient one between polls.

    `channel="chrome"` drives the system /usr/bin/google-chrome, so there is no
    browser download and the SwiftShader flags are the same ones the CDP path uses.
    Verified 2026-07-29 to render the Three.js scene on this GPU-less host.
    """

    def __init__(self) -> None:
        self._pw = None
        self._browser = None
        self._page = None
        self.url: str | None = None

    async def start(self, width: int = 1600, height: int = 1000) -> None:
        if self._page is not None:
            return
        from playwright.async_api import async_playwright

        self._pw = await async_playwright().start()
        self._browser = await self._pw.chromium.launch(
            channel="chrome", args=[f for f in CHROME_FLAGS if f != "--headless=new"])
        self._page = await self._browser.new_page(
            viewport={"width": width, "height": height})
        # Same reason as `CdpSession._disable_http_cache` -- this loop looks at
        # files edited seconds ago, so a cache hit is a silent stale read.
        try:
            cdp = await self._page.context.new_cdp_session(self._page)
            await cdp.send("Network.enable")
            await cdp.send("Network.setCacheDisabled", {"cacheDisabled": True})
        except Exception:
            pass

    async def stop(self) -> None:
        for closer in (getattr(self._browser, "close", None),
                       getattr(self._pw, "stop", None)):
            if closer:
                try:
                    await closer()
                except Exception:
                    pass
        self._pw = self._browser = self._page = None
        self.url = None

    async def goto(self, url: str = WEB_PAGE, settle: float = 8.0) -> None:
        await self.start()
        await self._page.goto(url)
        self.url = url
        if settle:
            await asyncio.sleep(settle)

    async def evaluate(self, expression: str, await_promise: bool = True) -> dict:
        await self.start()
        try:
            # Wrap so a bare expression and a function body both work, matching CDP.
            return {"ok": True, "value": await self._page.evaluate(f"() => ({expression})")}
        except Exception as exc:
            return {"ok": False, "exception": str(exc).splitlines()[0], "detail": str(exc)[:600]}

    async def wait_for(self, predicate_js: str, timeout_s: float = 15.0,
                       poll_s: float = 0.25) -> dict:
        await self.start()
        t0 = time.monotonic()
        try:
            await self._page.wait_for_function(f"() => !!({predicate_js})",
                                               timeout=timeout_s * 1000)
        except Exception as exc:
            return {"ok": False, "timeout_s": timeout_s,
                    "last": {"ok": False, "exception": str(exc).splitlines()[0]}}
        return {"ok": True, "waited_s": round(time.monotonic() - t0, 2)}

    async def screenshot(self) -> bytes:
        await self.start()
        return await self._page.screenshot()

    async def diag_tail(self, chars: int = 900) -> str:
        r = await self.evaluate(
            "(document.getElementById('diag-log')||{}).textContent || 'no-diag-panel'")
        return (r.get("value") or "")[-chars:]


def _make_browser():
    """Playwright when it is installed, else the raw-CDP implementation.

    Both are verified on this host; CDP is kept as the fallback so the ui_* tools
    still work in an environment without the [mcp] extra's playwright.
    """
    try:
        import playwright.async_api  # noqa: F401
    except ImportError:
        return CdpSession()
    return PlaywrightSession()


# Module-level singletons: one rig connection and one browser per server process.
rig = RigSession()
browser = _make_browser()
