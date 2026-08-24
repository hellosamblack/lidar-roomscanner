"""Long-lived connections held across tool calls.

This is the whole reason the dev loop is worth exposing over MCP rather than Bash:
a Bash call is a fresh process every time, so each UI check had to relaunch Chrome
and re-settle it (~8 s). The server outlives individual tool calls, so `RigSession`
keeps one `/ws` connection open and `CdpSession` keeps one browser warm.

`CdpSession` is deliberately an interface over the browser mechanism. The initial
implementation is the raw-CDP plumbing lifted from `host/tools/web_ui_shot.py`,
which is proven to render Three.js under SwiftShader on this GPU-less host; a
Playwright backend can replace it without any tool signature changing.

"Warm" cannot mean "forever", though: the page is a software-rasterized WebGL scene,
and an abandoned browser burned 11 cores for days before anyone noticed (issue #219).
`_IdleReapMixin` gives both backends a reset-on-interaction idle TTL -- park to
`about:blank`, then stop -- and owns the public `goto`/`evaluate`/`wait_for`/
`screenshot` so no operation can escape that bookkeeping.
"""
from __future__ import annotations

import asyncio
import base64
import contextlib
import json
import os
import shutil
import subprocess
import tempfile
import time
import urllib.request

from . import browser_reap
from .paths import WEB_PAGE, WEB_WS

CHROME_CANDIDATES = ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser")
# Software WebGL: without these Three.js cannot get a context on a GPU-less host.
CHROME_FLAGS = ("--headless=new", "--enable-unsafe-swiftshader", "--use-gl=angle",
                "--use-angle=swiftshader", "--no-sandbox", "--disable-dev-shm-usage")

BLANK_URL = "about:blank"


def _env_seconds(name: str, default: float) -> float:
    """Non-negative seconds from `name`, falling back to `default`. `0` disables."""
    raw = os.environ.get(name)
    if raw is None or not raw.strip():
        return default
    try:
        return max(0.0, float(raw))
    except ValueError:
        return default


# Idle TTLs for the warm browser singleton (issue #219). Two stages because the two
# costs recover at different prices -- see `_IdleReapMixin`.
#
# These are deliberately LONG (owner, 2026-08-24). The failure being prevented is a
# browser that outlives the conversation and burns ~11 cores for DAYS; against that,
# 30 minutes is already a ~99.9% reduction. Anything shorter starts costing a real
# working session -- a coding session can easily go half an hour between UI checks,
# and the reaper must never be the reason a `ui_eval` is slow. Trading a large slice
# of the remaining benefit for "it is never in the way" is the right trade here,
# because the residual burn is bounded and the interruption is not.
BROWSER_PARK_S = _env_seconds("ROOMSCAN_MCP_BROWSER_PARK_S", 1800.0)     # 30 min
BROWSER_STOP_S = _env_seconds("ROOMSCAN_MCP_BROWSER_STOP_S", 14400.0)    # 4 h
# Upper bound on a restore, NOT a fixed sleep. The restore waits on a real readiness
# predicate (`RESTORE_READY_JS`) and returns as soon as it holds; this is only the
# give-up point, after which we proceed anyway rather than fail the caller's call.
# Measured: the app is genuinely live ~1.8 s after navigation, so the blind 8 s sleep
# this replaced was ~4.5x longer than needed and was most of what made parking
# noticeable at all.
BROWSER_RESTORE_TIMEOUT_S = _env_seconds("ROOMSCAN_MCP_BROWSER_RESTORE_TIMEOUT_S", 15.0)

# Readiness gate for a restored page. Degrades safely on a page that is not the
# roomscan UI: each app-specific clause is skipped when its element is absent, so an
# arbitrary URL (or `about:blank`) gates on `readyState` alone.
#
# Measured first-true times on this host, three consecutive restores:
#   readyState complete 0.83-0.89 | canvas w>0 0.89-1.29 | hud-view-fps > 0 1.68-1.84
# `hud-view-fps` going positive is the strongest of these -- it means the render loop
# has produced frames, not merely that the DOM exists. Do NOT try to gate on canvas
# PIXELS: `readPixels` after a frame reads uniform on this context
# (`preserveDrawingBuffer` is off), so that check never goes true and would burn the
# full timeout on every restore.
RESTORE_READY_JS = (
    "document.readyState === 'complete'"
    " && (() => { const c = document.querySelector('canvas');"
    "             return !c || c.width > 0; })()"
    " && (() => { const e = document.getElementById('hud-view-fps');"
    "             return !e || parseFloat(e.textContent) > 0; })()"
)


# --- rig (/ws to roomscan-web) ----------------------------------------------

class RigSession:
    """One `/ws` connection to roomscan-web, with the latest message of each type.

    The server is the single source of truth for state (docs/web-protocol.md
    §Invariants: the client never holds optimistic state), so this caches only what
    the server last broadcast and never derives anything from it.
    """

    # NOTE (#220): these sockets take the `websockets` keepalive DEFAULT -- a 20 s
    # ping with a 20 s pong deadline, which closes a healthy connection when the peer
    # is merely CPU-starved. That is what killed `CdpSession`'s CDP socket in #219.
    # Here it is masked only by the stream's constant traffic; a >20 s GIL stall in
    # roomscan-web (BUG-052/BUG-063 territory) would drop every MCP client and present
    # as "the tools lost the rig". Left as the default deliberately for now, tracked
    # in #220 -- do not "fix" it here without the reproduction that issue asks for.

    def __init__(self, url: str = WEB_WS) -> None:
        self.url = url
        self._ws = None
        self._reader: asyncio.Task | None = None
        self.latest: dict[str, dict] = {}
        self.binary_counts: dict[int, int] = {}
        self._events: dict[str, asyncio.Event] = {}
        # BUG-061 A6: MESH (tag 3) moved off `/ws` onto a dedicated `/ws-mesh`
        # socket with a one-in-flight ack contract. This is best-effort -- an
        # older server without the route must never take down the main session.
        self._ws_mesh = None
        self._mesh_reader: asyncio.Task | None = None
        self.mesh_connect_error: str | None = None

    @property
    def connected(self) -> bool:
        return self._ws is not None and self._reader is not None and not self._reader.done()

    @property
    def mesh_connected(self) -> bool:
        return (self._ws_mesh is not None and self._mesh_reader is not None
                and not self._mesh_reader.done())

    async def connect(self, timeout: float = 10.0) -> bool:
        if not self.connected:
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
        if not self.mesh_connected:
            await self._connect_mesh(timeout=timeout)
        return True

    async def _connect_mesh(self, timeout: float = 10.0) -> None:
        """Best-effort `/ws-mesh` connect. Failure here is silent to callers --
        `binary_counts` simply never gets tag-3 entries from this socket, and
        `rig_status`'s `streaming` still sees tag 3 fine as long as it arrives
        on `/ws` (older server) instead.
        """
        import websockets

        mesh_url = self.url.rsplit("/", 1)[0] + "/ws-mesh"
        try:
            self._ws_mesh = await asyncio.wait_for(
                websockets.connect(mesh_url, max_size=None), timeout=timeout)
        except Exception as exc:
            self._ws_mesh = None
            self.mesh_connect_error = f"{mesh_url}: {exc}"
            return
        self.mesh_connect_error = None
        self._mesh_reader = asyncio.create_task(self._pump_mesh())

    async def close(self) -> None:
        if self._reader:
            self._reader.cancel()
            self._reader = None
        if self._mesh_reader:
            self._mesh_reader.cancel()
            self._mesh_reader = None
        if self._ws:
            try:
                await self._ws.close()
            finally:
                self._ws = None
        if self._ws_mesh:
            try:
                await self._ws_mesh.close()
            finally:
                self._ws_mesh = None

    def _count_tag(self, msg: bytes) -> int | None:
        import struct as _struct
        if len(msg) < 4:
            return None
        tag = _struct.unpack_from("<I", msg)[0]
        self.binary_counts[tag] = self.binary_counts.get(tag, 0) + 1
        return tag

    async def _pump(self) -> None:
        try:
            async for msg in self._ws:
                if isinstance(msg, (bytes, bytearray)):
                    self._count_tag(msg)
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

    async def _pump_mesh(self) -> None:
        """Count tag 3 the same way `_pump` does, and auto-ack every mesh.

        Per the `/ws-mesh` credit contract (docs/web-protocol.md, BUG-061): at
        most one mesh in flight per connection, cleared by an app-level
        `mesh_ack` naming the `mesh_seq` at byte offset 4 of the packet. This
        session has no rendering to wait on, so it acks immediately -- it exists
        to keep `binary_counts`/`streaming` accurate, not to exercise backpressure.
        """
        import struct as _struct
        try:
            async for msg in self._ws_mesh:
                if not isinstance(msg, (bytes, bytearray)):
                    continue
                tag = self._count_tag(msg)
                if tag is None or len(msg) < 8:
                    continue
                seq = _struct.unpack_from("<I", msg, 4)[0]
                try:
                    await self._ws_mesh.send(json.dumps({"type": "mesh_ack", "seq": seq}))
                except Exception:
                    pass
        except Exception:
            pass  # mesh socket dropped; the main session is unaffected

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


# --- browser idle reaping ----------------------------------------------------

class _IdleReapMixin:
    """Reset-on-interaction idle TTL for a warm browser singleton (issue #219).

    Keeping one browser alive across tool calls is the point of exposing the UI loop
    over MCP (module docstring) -- but nothing ever closed it, and the page it sits on
    is a Three.js scene software-rasterized by SwiftShader on this GPU-less host. A
    session that ran one `ui_screenshot` and moved on left that running forever:
    measured 2026-08-24, two abandoned browsers from two other agent sessions were
    burning 1115% and 750% CPU -- 18.6 of 26 cores -- next to a `roomscan-web` using
    27.7%. They also each held `/ws` clients open, which is real per-client server
    work (BUG-060). The browser outlives the *conversation*, so nothing else was ever
    going to reclaim them.

    Two stages, because the two costs recover at different prices:

    * **Park** (`BROWSER_PARK_S`): navigate to `about:blank`. Kills the WebGL burn and
      drops the `/ws` client, but keeps the process warm, so the next call pays a
      navigation rather than a cold launch. `self.url` deliberately keeps pointing at
      the parked page -- it is still the page this session is logically on.
    * **Stop** (`BROWSER_STOP_S`): full teardown, process and profile gone.

    The trap this is built around: a parked browser must never let `ui_eval` quietly
    evaluate against `about:blank` and hand back `null` for every selector. A blank
    read that looks like a successful read is the failure mode this repo keeps paying
    for. So restore happens in `start()` -- the one choke point every interface method
    already funnels through -- and is *counted*, so a caller can tell it happened.

    Subclasses implement `_start_impl`/`_stop_impl`/`_goto_impl`/`_evaluate_impl`/
    `_wait_for_impl`/`_screenshot_impl`/`_navigate_raw`/`_alive`. The public methods
    live here so no operation can escape the idle bookkeeping, and any method added
    later inherits it by being written as an `_impl`.
    """

    park_after_s: float = BROWSER_PARK_S
    stop_after_s: float = BROWSER_STOP_S
    restore_timeout_s: float = BROWSER_RESTORE_TIMEOUT_S
    reap_tick_s: float = 15.0

    def _idle_init(self) -> None:
        self._last_used: float = time.monotonic()
        self._parked_url: str | None = None
        self._reaper: asyncio.Task | None = None
        self._busy: int = 0
        self.park_count: int = 0
        self.stop_count: int = 0
        self.restore_count: int = 0
        self.last_park_error: str | None = None
        self.last_restore: dict | None = None

    # -- clock
    def touch(self) -> None:
        self._last_used = time.monotonic()

    @property
    def idle_s(self) -> float:
        return time.monotonic() - self._last_used

    @property
    def parked(self) -> bool:
        return self._parked_url is not None

    @contextlib.asynccontextmanager
    async def _in_use(self):
        """Hold off the reaper for the duration of an operation, and reset the clock.

        The counter matters as much as the timestamp: `ui_wait_for` can legitimately
        block longer than the park TTL, and parking a page out from under a
        `wait_for_function` would fail the wait for a reason the caller cannot see.
        Re-entrant because `diag_tail` goes through `evaluate`.
        """
        self._busy += 1
        self.touch()
        try:
            yield
        finally:
            self._busy -= 1
            self.touch()

    def _cancel_park(self) -> None:
        """Forget a pending restore, for a caller that is about to navigate anyway."""
        self._parked_url = None

    def status(self) -> dict:
        return {
            "alive": self._alive(),
            "url": self.url,
            "parked": self.parked,
            "idle_s": round(self.idle_s, 1),
            "busy": self._busy,
            "park_after_s": self.park_after_s,
            "stop_after_s": self.stop_after_s,
            "restore_timeout_s": self.restore_timeout_s,
            "parks": self.park_count,
            "stops": self.stop_count,
            "restores": self.restore_count,
            "last_restore": self.last_restore,
            "last_park_error": self.last_park_error,
            "backend": type(self).__name__,
        }

    # -- reaper
    def _ensure_reaper(self) -> None:
        if self.park_after_s <= 0 and self.stop_after_s <= 0:
            return
        if not self._alive():
            return  # nothing to reap yet
        if self._reaper is not None and not self._reaper.done():
            return
        try:
            loop = asyncio.get_running_loop()
        except RuntimeError:
            return  # no loop (sync test/context); reaping is best-effort
        self._reaper = loop.create_task(self._reap_loop())

    async def _reap_loop(self) -> None:
        try:
            while True:
                await asyncio.sleep(self.reap_tick_s)
                await self._reap_once()
                if not self._alive():
                    return  # nothing left to reap; `start()` spawns a fresh reaper
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            # A reaper crash must never take down the server -- but it must not be
            # invisible either, or the browser silently reverts to immortal.
            self.last_park_error = f"reaper died: {type(exc).__name__}: {exc}"

    async def _reap_once(self) -> None:
        if self._busy or not self._alive():
            return
        idle = self.idle_s
        if self.stop_after_s > 0 and idle >= self.stop_after_s:
            try:
                await self.stop()
            finally:
                self.stop_count += 1
            return
        if (self.park_after_s > 0 and idle >= self.park_after_s
                and not self.parked and self.url and self.url != BLANK_URL):
            target = self.url
            try:
                await self._navigate_raw(BLANK_URL)
            except Exception as exc:
                # Leave it unparked -- the stop stage is the backstop -- but RECORD
                # why. A bare `return` here hid a total park failure on the CDP
                # backend behind a browser that merely looked idle-stopped later.
                self.last_park_error = f"{type(exc).__name__}: {exc}"
                return
            self.last_park_error = None
            self._parked_url = target
            self.park_count += 1

    async def _restore_if_parked(self) -> None:
        target = self._parked_url
        if target is None:
            return
        # Cleared first: a restore that throws must not leave the session looping on a
        # page it can never reach.
        self._parked_url = None
        t0 = time.monotonic()
        await self._navigate_raw(target)
        # Wait on the page telling us it is ready, not on a clock. The blind sleep
        # this replaced was ~4.5x the measured readiness time and was most of the
        # cost of parking; more importantly a fixed sleep is the wrong SHAPE, since
        # it is simultaneously too long on a fast restore and no guarantee at all on
        # a slow one. `ready=False` means the gate timed out and we proceeded anyway
        # -- the caller gets a page that may still be settling, but never a hang.
        ready = True
        if self.restore_timeout_s:
            r = await self._wait_for_impl(RESTORE_READY_JS,
                                          timeout_s=self.restore_timeout_s)
            ready = bool(r.get("ok"))
        self.restore_count += 1
        self.last_restore = {"url": target, "took_s": round(time.monotonic() - t0, 2),
                             "ready": ready}

    # -- public interface (subclasses supply the `_impl`s)
    async def start(self, width: int | None = None, height: int | None = None) -> None:
        self.touch()
        await self._start_impl(width, height)
        self._ensure_reaper()
        await self._restore_if_parked()

    async def stop(self) -> None:
        reaper, self._reaper = self._reaper, None
        if reaper is not None and reaper is not asyncio.current_task():
            # NOT when the reaper is what called us: cancelling the running task
            # raises CancelledError at the next await INSIDE `_stop_impl`, which
            # tears the browser down halfway. Measured on the live probe before this
            # guard: `stop_count` incremented, most of the tree exited, but
            # `PlaywrightSession._page` was never cleared and a chrome process
            # survived -- so `_alive()` stayed True and the session was wedged with
            # no browser behind it. `_reap_loop` returns on its own once `_alive()`
            # goes False, so there is nothing to cancel in that path.
            reaper.cancel()
        self._parked_url = None
        await self._stop_impl()

    async def goto(self, url: str = WEB_PAGE, settle: float = 8.0) -> None:
        async with self._in_use():
            self._cancel_park()  # about to navigate; a restore first would be wasted
            await self._goto_impl(url, settle)

    async def evaluate(self, expression: str, await_promise: bool = True) -> dict:
        async with self._in_use():
            return await self._evaluate_impl(expression, await_promise)

    async def wait_for(self, predicate_js: str, timeout_s: float = 15.0,
                       poll_s: float = 0.25) -> dict:
        async with self._in_use():
            return await self._wait_for_impl(predicate_js, timeout_s, poll_s)

    async def screenshot(self) -> bytes:
        async with self._in_use():
            return await self._screenshot_impl()

    async def diag_tail(self, chars: int = 900) -> str:
        r = await self.evaluate(
            "(document.getElementById('log-lines')||{}).innerText || 'no-log-console'",
            await_promise=False)
        return (r.get("value") or "")[-chars:]


# --- browser (CDP to headless Chrome) ---------------------------------------

class CdpSession(_IdleReapMixin):
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
        # Last explicitly-requested size, re-applied by every call including
        # argless internal ones (see `start()`).
        self._size: tuple[int, int] = (1600, 1000)
        self._idle_init()

    def _alive(self) -> bool:
        return self._ws is not None and self._proc is not None and self._proc.poll() is None

    async def _navigate_raw(self, url: str) -> None:
        """Navigate without touching `self.url` or the idle bookkeeping (park/restore)."""
        await self.cmd("Page.navigate", {"url": url})

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

    async def _start_impl(self, width: int | None = None, height: int | None = None) -> None:
        """Ensure the browser is up, then (re)apply the current viewport size.

        The browser is a singleton kept warm across tool calls (module docstring),
        so the viewport must be re-applied on every call, not only the first -- an
        early version returned immediately once `self._ws` existed, which made
        every call after the first one silently keep whatever size the browser
        launched with (issue #168: `width`/`height` read back unchanged after two
        different resize requests).

        `width`/`height` default to `None`, meaning "keep the last explicitly
        requested size" (`self._size`), NOT "reset to 1600x1000". This is
        deliberate: `goto`/`evaluate`/`wait_for`/`screenshot` all call
        `self.start()` with no arguments just to ensure the browser exists before
        they act -- with a hardcoded default, that argless call silently reasserted
        1600x1000 and undid whatever `ui_screenshot(width=..., height=...)` had just
        set, immediately before capturing the pixels (issue #168, second half: the
        first fix made `start()` idempotent-safe but gave teeth to those argless
        internal callers). Any *new* argless `self.start()` added later inherits
        this "preserve, don't reset" behaviour for free.
        """
        if width is not None:
            self._size = (width, self._size[1])
        if height is not None:
            self._size = (self._size[0], height)
        if self._ws is None:
            import websockets

            if self._proc is None or self._proc.poll() is not None:
                self._profile = tempfile.mkdtemp(prefix="roomscan-mcp-chrome-")
                self._proc = subprocess.Popen(
                    [self._find_chrome(), *CHROME_FLAGS,
                     f"--remote-debugging-port={self.port}",
                     f"--user-data-dir={self._profile}", "about:blank"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    # #221: the kernel kills this Chrome if we die, even on SIGKILL --
                    # the only cleanup that needs no cooperation from the dying process.
                    preexec_fn=browser_reap.pdeathsig)
            ws_url = await asyncio.to_thread(self._cdp_target)
            # `ping_interval=None`: no keepalive on the CDP socket. `websockets`
            # defaults to a 20 s ping with a 20 s pong deadline, and Chrome cannot
            # answer it while SwiftShader is saturating a dozen cores rendering the
            # live scene -- so the CLIENT tears down a perfectly good connection
            # with `1011 keepalive ping timeout`. Found via #219: the idle reaper's
            # park navigation failed every time on this backend while the same call
            # succeeded when issued by hand on a quiet page. Pre-existing, not new
            # -- it silently killed any CdpSession left idle ~20 s on a busy page.
            # Keepalive buys nothing here anyway: Chrome is our own subprocess, so
            # `self._proc.poll()` is the liveness signal that actually matters.
            self._ws = await websockets.connect(ws_url, max_size=None, ping_interval=None)
            await self.cmd("Page.enable")
            await self.cmd("Runtime.enable")
            await self._disable_http_cache()
        await self.cmd("Emulation.setDeviceMetricsOverride",
                       {"width": self._size[0], "height": self._size[1],
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

    async def _stop_impl(self) -> None:
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
    async def _goto_impl(self, url: str = WEB_PAGE, settle: float = 8.0) -> None:
        await self.start()
        await self.cmd("Page.navigate", {"url": url})
        self.url = url
        await asyncio.sleep(settle)

    async def _evaluate_impl(self, expression: str, await_promise: bool = True) -> dict:
        await self.start()
        r = await self.cmd("Runtime.evaluate",
                           {"expression": expression, "awaitPromise": await_promise,
                            "returnByValue": True})
        exc = r.get("exceptionDetails")
        if exc:
            return {"ok": False, "exception": exc.get("text"),
                    "detail": (exc.get("exception") or {}).get("description")}
        return {"ok": True, "value": (r.get("result") or {}).get("value")}

    async def _wait_for_impl(self, predicate_js: str, timeout_s: float = 15.0,
                             poll_s: float = 0.25) -> dict:
        """Poll `predicate_js` until truthy. Replaces sleep-and-hope."""
        await self.start()
        deadline = time.monotonic() + timeout_s
        last: dict = {}
        while time.monotonic() < deadline:
            last = await self._evaluate_impl(f"!!({predicate_js})")
            if last.get("ok") and last.get("value"):
                return {"ok": True, "waited_s": round(
                    timeout_s - (deadline - time.monotonic()), 2)}
            await asyncio.sleep(poll_s)
        return {"ok": False, "timeout_s": timeout_s, "last": last}

    async def _screenshot_impl(self) -> bytes:
        await self.start()
        r = await self.cmd("Page.captureScreenshot", {"format": "png"})
        data = r.get("data")
        if not data:
            raise RuntimeError("screenshot returned no data")
        return base64.b64decode(data)


class PlaywrightSession(_IdleReapMixin):
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
        # Last explicitly-requested size, re-applied by every call including
        # argless internal ones (see `start()`).
        self._size: tuple[int, int] = (1600, 1000)
        self._idle_init()

    def _alive(self) -> bool:
        return self._page is not None

    async def _navigate_raw(self, url: str) -> None:
        """Navigate without touching `self.url` or the idle bookkeeping (park/restore)."""
        await self._page.goto(url)

    async def _start_impl(self, width: int | None = None, height: int | None = None) -> None:
        """Ensure the page is up, then (re)apply the current viewport size.

        Same fix as `CdpSession.start` (issue #168): the browser/page is a
        singleton kept warm across tool calls, so an early-return once `self._page`
        existed made `width`/`height` inert after the first call. `set_viewport_size`
        is Playwright's documented way to resize an existing page's viewport.

        `width`/`height` default to `None`, meaning "keep the last explicitly
        requested size" (`self._size`), NOT "reset to 1600x1000". This matters
        because `goto`/`evaluate`/`wait_for`/`screenshot` all call `self.start()`
        with no arguments just to ensure the page exists before they act -- with a
        hardcoded default, that argless call silently reasserted 1600x1000 via
        `set_viewport_size` and undid whatever `ui_screenshot(width=..., height=...)`
        had just set, immediately before capturing the pixels (issue #168, second
        half). Any *new* argless `self.start()` added later inherits this
        "preserve, don't reset" behaviour for free.
        """
        if width is not None:
            self._size = (width, self._size[1])
        if height is not None:
            self._size = (self._size[0], height)
        if self._page is None:
            from playwright.async_api import async_playwright

            self._pw = await async_playwright().start()
            self._browser = await self._pw.chromium.launch(
                channel="chrome", args=[f for f in CHROME_FLAGS if f != "--headless=new"])
            self._page = await self._browser.new_page(
                viewport={"width": self._size[0], "height": self._size[1]})
            # Same reason as `CdpSession._disable_http_cache` -- this loop looks at
            # files edited seconds ago, so a cache hit is a silent stale read.
            try:
                cdp = await self._page.context.new_cdp_session(self._page)
                await cdp.send("Network.enable")
                await cdp.send("Network.setCacheDisabled", {"cacheDisabled": True})
            except Exception:
                pass
        else:
            await self._page.set_viewport_size(
                {"width": self._size[0], "height": self._size[1]})

    async def _stop_impl(self) -> None:
        for closer in (getattr(self._browser, "close", None),
                       getattr(self._pw, "stop", None)):
            if closer:
                try:
                    await closer()
                except Exception:
                    pass
        self._pw = self._browser = self._page = None
        self.url = None

    async def _goto_impl(self, url: str = WEB_PAGE, settle: float = 8.0) -> None:
        await self.start()
        await self._page.goto(url)
        self.url = url
        if settle:
            await asyncio.sleep(settle)

    async def _evaluate_impl(self, expression: str, await_promise: bool = True) -> dict:
        await self.start()
        try:
            # Wrap so a bare expression and a function body both work, matching CDP.
            return {"ok": True, "value": await self._page.evaluate(f"() => ({expression})")}
        except Exception as exc:
            return {"ok": False, "exception": str(exc).splitlines()[0], "detail": str(exc)[:600]}

    async def _wait_for_impl(self, predicate_js: str, timeout_s: float = 15.0,
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

    async def _screenshot_impl(self) -> bytes:
        await self.start()
        return await self._page.screenshot()


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
