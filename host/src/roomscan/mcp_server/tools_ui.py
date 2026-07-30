"""See and drive the web UI in headless Chrome.

`docs/engineering-practices.md` requires web work to be verified visually, not just
by tests. The browser is held open across calls by `session.browser`, so only the
first call in a session pays the launch + settle cost.

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
    """Screenshot the web UI and return the image plus the on-page diag-log tail.

    The browser stays open between calls, so by default this shoots the page as it
    already stands -- pass `renavigate=True` (with a `settle`, ~8 s for a cold load)
    to reload first. The diag-log tail is the fastest signal for a load failure.
    """
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
    return [
        ImageContent(type="image", data=base64.b64encode(png).decode(), mimeType="image/png"),
        TextContent(type="text", text=f"url={browser.url}\n--- diag-log tail ---\n{tail}"),
    ]


@mcp.tool()
async def ui_eval(js: str, await_promise: bool = True) -> dict:
    """Evaluate JavaScript in the page and return the result.

    Use this to assert real state rather than trusting a sleep. Useful readouts:
    `#pos-status` (replay "frame N / total"), `#hud-view-fps`, `#hud-device-fps`,
    `#record-status`, `#ir-frame`, `#slam-frames`. Element ids live in
    host/src/roomscan/static/index.html.

    Note `window.__diag` is the page's *logging sink function*, not a state object --
    call `ui_screenshot`, which returns the diag-log tail, to read what it logged.
    """
    return await browser.evaluate(js, await_promise=await_promise)


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
