"""MCP server exposing the roomscanner dev loop as typed tools.

Why this exists: the agent-facing surface used to be ~22 scripts under `host/tools/`
plus 5 console CLIs, all printing prose that had to be re-parsed. Most of those
scripts already *computed* structured data (`analyze_capture.analyze()` builds
anomaly dicts, `Doctor` accumulates per-check results) and threw the structure away
at the `print()` boundary. These tools recover it.

Two rules hold this package together:

1. **Thin layer.** A tool never reimplements logic. Each wrapped script keeps its
   `argparse` `main()` as a prose front end; both call the same pure function.
2. **Client, never competitor.** The server must never bind the device UDP/CDC
   stream -- `roomscan-web` owns it. `capture.py --udp` and `roomscan-web` starve
   each other (see the `firmware-loop` skill), so recording goes through the
   server's own `/ws` channel. Raw `capture.py` stays CLI-only to keep this
   structural rather than a rule someone has to remember.

See docs/mcp-server.md.
"""
