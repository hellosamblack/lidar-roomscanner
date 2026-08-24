"""Entry point: `roomscan-mcp`, or `python -m roomscan.mcp_server`.

Defaults to stdio, where the server is a child of its MCP client and dies with it --
but note the BROWSER does not die with the server unless we make it (issue #221), which
is why this sweeps for leaked browser trees on the way up and installs exit cleanup.
`--http` runs it as a standalone service instead, which survives across sessions so
Chrome and the /ws connection stay warm -- register that form with:

    claude mcp add --transport http roomscan http://127.0.0.1:8765/mcp -s project
"""
from __future__ import annotations

import argparse
import sys

from . import browser_reap
from .server import build


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-mcp", description=__doc__.split("\n")[0])
    ap.add_argument("--http", action="store_true",
                    help="serve over streamable HTTP instead of stdio (survives across sessions)")
    ap.add_argument("--host", default="127.0.0.1", help="bind host for --http")
    ap.add_argument("--port", type=int, default=8765, help="bind port for --http")
    args = ap.parse_args(argv)

    server = build()

    # #221: reap browser trees a previous server leaked (SIGKILL/crash leaves them with
    # no reaper at all), then arrange not to leak our own. Ownership-checked, so a
    # concurrent session's live browser is never touched. Never fatal -- a UI loop that
    # cannot start is worse than a leaked browser.
    try:
        report = browser_reap.sweep_orphaned_browsers()
        if report.get("killed_count"):
            print(f"roomscan-mcp: reaped {report['killed_count']} orphaned browser "
                  f"tree(s) from a previous server", file=sys.stderr)
        from .session import browser as _browser
        browser_reap.install_exit_cleanup(_browser)
    except Exception as exc:  # pragma: no cover - defensive
        print(f"roomscan-mcp: browser cleanup unavailable: {exc}", file=sys.stderr)

    if args.http:
        # SDK 2.x takes bind settings as run() kwargs, not via server.settings.
        server.run(transport="streamable-http", host=args.host, port=args.port)
    else:
        server.run(transport="stdio")
    return 0


if __name__ == "__main__":
    sys.exit(main())
