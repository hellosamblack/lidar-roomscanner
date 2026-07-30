"""The `MCPServer` instance and tool registration.

Tool modules import `mcp` from here and decorate with `@mcp.tool()`; `build()`
imports them for that side effect. Keeping the instance in its own module is what
lets the tool modules stay import-cycle-free.

Note for anyone reading MCP SDK examples: in SDK 2.x the class formerly called
`FastMCP` is `mcp.server.MCPServer`.
"""
from __future__ import annotations

import contextlib
from collections.abc import AsyncIterator

from mcp.server import MCPServer

INSTRUCTIONS = """\
Tools for the roomscanner 3D room scanner (host + STM32H563 firmware).

Groups:
  rig_*      control the running `roomscan-web` instrument over its /ws channel
  ui_*       screenshot and drive the web UI in headless Chrome
  capture_*  inspect recorded captures
  fw_*       build and flash firmware
  doctor / orientation_probe / run_tests  diagnostics

Start with `rig_status()` -- it reports whether the server is up and what it is
playing. If it is down, `rig_up()` starts it (optionally against a replay).

These tools never bind the device stream directly; `roomscan-web` owns it.
"""

@contextlib.asynccontextmanager
async def _lifespan(_server: MCPServer) -> AsyncIterator[None]:
    """Tear the browser and /ws connection down on shutdown.

    Without this the Playwright Node driver is orphaned when the client exits and
    dies noisily on stderr; the Chrome process would also leak.
    """
    try:
        yield
    finally:
        from .session import browser, rig

        with contextlib.suppress(Exception):
            await rig.close()
        with contextlib.suppress(Exception):
            await browser.stop()


mcp = MCPServer(name="roomscan", instructions=INSTRUCTIONS, version="0.1.0",
                lifespan=_lifespan)


def build() -> MCPServer:
    """Import every tool module so decorators register, then hand back the server."""
    from . import tools_build, tools_data, tools_rig, tools_ui  # noqa: F401

    return mcp
