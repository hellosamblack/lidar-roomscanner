"""See and drive the web UI in headless Chrome.

`docs/engineering-practices.md` requires web work to be verified visually, not just
by tests. The browser is held open across calls by `session.browser`, so only the
first call in a session pays the launch + settle cost.

It is not held open indefinitely: after `ROOMSCAN_MCP_BROWSER_PARK_S` idle it parks to
`about:blank` and after `ROOMSCAN_MCP_BROWSER_STOP_S` it exits (issue #219 -- a
forgotten browser software-rasterizes WebGL at ~11 cores). Parking is transparent to
callers here; the next call restores the page before acting, and says so in its
result. `ui_browser_status()` reports the current state.

`ui_screenshot` returns the PNG as an image block: no /tmp file, no separate read.
"""
from __future__ import annotations

from mcp.types import ImageContent, TextContent

from .paths import WEB_PAGE
from .server import mcp
from .session import browser


@mcp.tool()
async def ui_screenshot(url: str = "", settle: float = 0.0, width: int = 1600,
                        height: int = 1000, renavigate: bool = False) -> list:
    """Screenshot the web UI and return the image plus the on-page event-log tail.

    `width`/`height` set the actual browser viewport (`window.innerWidth/Height`),
    not just the returned image size -- every call resizes it, even without
    `renavigate`, so back-to-back calls at different sizes each take effect. Use
    this for the narrow-viewport checks in docs/web-ui-testing.md.

    The browser stays open between calls, so by default this shoots the page as it
    already stands -- pass `renavigate=True` (with a `settle`, ~8 s for a cold load)
    to reload first. The event-log tail is the fastest signal for a load failure.

    If the browser had been idle long enough to park (issue #219), the page is
    restored before the shot and the text block reports `restored=`.
    """
    was_parked = browser.parked
    if renavigate or browser.url is None:
        await browser.start(width=width, height=height)
        await browser.goto(url or WEB_PAGE, settle=settle or 8.0)
    else:
        await browser.start(width=width, height=height)
        if settle:
            import asyncio
            await asyncio.sleep(settle)

    png = await browser.screenshot()
    tail = await browser.diag_tail()
    import base64
    head = f"url={browser.url}"
    if was_parked:
        head += f" restored={browser.last_restore}"
    return [
        ImageContent(type="image", data=base64.b64encode(png).decode(), mimeType="image/png"),
        TextContent(type="text", text=f"{head}\n--- event-log tail ---\n{tail}"),
    ]


@mcp.tool()
async def ui_eval(js: str, await_promise: bool = True) -> dict:
    """Evaluate JavaScript in the page and return the result.

    Use this to assert real state rather than trusting a sleep. Useful readouts:
    `#pos-status` (replay "frame N / total"), `#hud-view-fps`, `#hud-device-fps`,
    `#record-status`, `#ir-frame`, `#slam-frames`. Element ids live in
    host/src/roomscan/static/index.html.

    Note `window.__diag` is the page's *logging sink function*, not a state object --
    call `ui_screenshot`, which returns the event-log tail, to read what it logged.

    A parked browser (issue #219) is restored before the expression runs, so this
    never evaluates against a blank page; `restored` appears in the result when that
    happened, and carries the settle time it cost.
    """
    was_parked = browser.parked
    r = await browser.evaluate(js, await_promise=await_promise)
    if was_parked:
        r = {**r, "restored": browser.last_restore}
    return r


@mcp.tool()
async def ui_wait_for(predicate_js: str, timeout_s: float = 15.0) -> dict:
    """Poll a JS predicate until it is truthy, or time out.

    Prefer this to a fixed wait. docs/web-ui-testing.md warns that a sleep does not
    guarantee the replay advanced, which makes fixed waits both slow and flaky.

    To wait for the replay to actually move:
        parseInt(document.getElementById('pos-status').textContent.match(/\\d+/)[0]) > 50
    """
    return await browser.wait_for(predicate_js, timeout_s=timeout_s)


@mcp.tool()
async def ui_reset(url: str = "", settle: float = 8.0, relaunch: bool = False) -> dict:
    """Renavigate to a clean page, optionally relaunching the browser.

    Server state persists across scenarios (docs/web-ui-testing.md), so reset
    between independent UI checks rather than letting one bleed into the next.
    `relaunch=True` also throws away the browser profile.
    """
    if relaunch:
        await browser.stop()
    await browser.goto(url or WEB_PAGE, settle=settle)
    return {"ok": True, "url": browser.url, "relaunched": relaunch}


@mcp.tool()
async def ui_browser_status() -> dict:
    """Report the headless browser's liveness, page, and idle-reap state (issue #219).

    The browser is kept warm across tool calls but reaped when idle: it parks to
    `about:blank` after `park_after_s` and exits after `stop_after_s`, and any `ui_*`
    call resets that clock. Use this to tell "the browser is gone/parked" apart from
    "the page is broken" -- a parked browser is not an error, and the next call
    restores it.

    `parks`/`stops`/`restores` are cumulative counters for this server process;
    `alive: false` just means the next call pays a cold launch.
    """
    return browser.status()
