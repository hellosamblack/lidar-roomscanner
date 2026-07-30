"""Repo locations, resolved once.

`host/tools/*.py` each recompute these inline (`Path(__file__).resolve().parents[2]`);
tools here import them instead so a layout change is a one-line fix.
"""
from __future__ import annotations

from pathlib import Path

# mcp_server -> roomscan -> src -> host -> repo
REPO = Path(__file__).resolve().parents[4]

HOST = REPO / "host"
VENV_PY = HOST / ".venv" / "bin" / "python"
CAPTURES = REPO / "captures"
RECORDINGS = REPO / "recordings"
RESULTS = REPO / "results"
LOGS = REPO / "logs"
TOOLS = HOST / "tools"
FIRMWARE = REPO / "firmware" / "scanner-stream"

WEB_URL = "http://localhost:8000"
WEB_PAGE = f"{WEB_URL}/static/index.html"
WEB_WS = "ws://localhost:8000/ws"


def rel(path: Path | str) -> str:
    """Repo-relative string for display, absolute if it falls outside the repo."""
    p = Path(path).resolve()
    try:
        return str(p.relative_to(REPO))
    except ValueError:
        return str(p)
