"""Entry point: `roomscan-mcp`, or `python -m roomscan.mcp_server`.

Defaults to stdio, where the server is a child of its MCP client and dies with it.
`--http` runs it as a standalone service instead, which survives across sessions so
Chrome and the /ws connection stay warm -- register that form with:

    claude mcp add --transport http roomscan http://127.0.0.1:8765/mcp -s project
"""
from __future__ import annotations

import argparse
import sys

from .server import build


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-mcp", description=__doc__.split("\n")[0])
    ap.add_argument("--http", action="store_true",
                    help="serve over streamable HTTP instead of stdio (survives across sessions)")
    ap.add_argument("--host", default="127.0.0.1", help="bind host for --http")
    ap.add_argument("--port", type=int, default=8765, help="bind port for --http")
    args = ap.parse_args(argv)

    server = build()
    if args.http:
        # SDK 2.x takes bind settings as run() kwargs, not via server.settings.
        server.run(transport="streamable-http", host=args.host, port=args.port)
    else:
        server.run(transport="stdio")
    return 0


if __name__ == "__main__":
    sys.exit(main())
