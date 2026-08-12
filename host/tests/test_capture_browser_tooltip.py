"""Regression test for #98 / BUG-087: every capture's tooltip must be its own.

The bug report: the Captures pane tile `title` was identical for every capture,
so hovering told you nothing about the specific file. Live verification against
the running UI (`ui_eval` over two real captures) found the tooltips already
differ — this test pins that behaviour by actually EXECUTING the shipped
`tileHtml()` from `browser.js` under Node, the same cross-check pattern
`test_protocol_c_crosscheck.py` uses for the C parser: extract the real source,
run it, assert on its output. A hand-reimplementation in Python would test a
copy of the logic, not the logic — it could pass while the shipped code
regressed to a single hard-coded string.

Requires `node` on PATH; skips (not fails) if absent, matching the C
cross-check fixture's handling of a missing `cc`/`gcc`/`clang`.
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest

BROWSER_JS = Path(__file__).parent.parent / "src" / "roomscan" / "static" / "browser.js"


def _extract_tile_html_source() -> str:
    """Pull `const fmtBytes = ...` through the end of `function tileHtml(c) {...}`.

    `tileHtml`'s tooltip (`tip`) depends on `fmtBytes`/`fmtTime`/`escapeHtml`/
    `THUMB_NOTE`, all defined just above it in the same closure -- slicing from
    the first of those through the function's matching closing brace pulls in
    exactly its real dependencies, no reimplementation.
    """
    text = BROWSER_JS.read_text(encoding="utf-8")
    start = text.index("const fmtBytes = (n) =>")
    fn_marker = "function tileHtml(c) {"
    fn_start = text.index(fn_marker, start)

    depth = 0
    i = fn_start
    end = None
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                end = i + 1
                break
        i += 1
    assert end is not None, "could not find matching closing brace for tileHtml()"
    return text[start:end]


def _node() -> str | None:
    return shutil.which("node")


@pytest.fixture(scope="module")
def tile_titles():
    node = _node()
    if node is None:
        pytest.skip("no node on PATH")
    if not BROWSER_JS.exists():
        pytest.skip(f"browser.js not found at {BROWSER_JS}")

    tile_html_src = _extract_tile_html_source()

    # Two captures that differ in every field the tooltip reports.
    harness = f"""
    const previewed = null;
    const playing = null;
    const prefs = {{ thumbs: true }};
    const selected = new Set();
    {tile_html_src}

    function title(html) {{
        return /title="([^"]*)"/.exec(html)[1];
    }}

    const a = tileHtml({{
        name: 'alpha.bin', duration_s: 16, frames: 494,
        bytes: 7400000, has_stream_9: true,
    }});
    const b = tileHtml({{
        name: 'officeFullScanAug6.bin', duration_s: 138, frames: 4104,
        bytes: 61600000, has_stream_9: true,
    }});
    console.log(JSON.stringify({{ a: title(a), b: title(b) }}));
    """
    proc = subprocess.run(
        [node, "-e", harness], capture_output=True, text=True, timeout=30
    )
    assert proc.returncode == 0, f"node harness failed:\n{proc.stderr}"
    return json.loads(proc.stdout)


def test_two_captures_get_different_tooltips(tile_titles):
    """The actual bug: identical tooltips regardless of which capture."""
    assert tile_titles["a"] != tile_titles["b"], (
        "tileHtml() produced the same tooltip for two different captures "
        f"(#98 / BUG-087 regressed): {tile_titles!r}"
    )


def test_tooltip_carries_this_captures_identity(tile_titles):
    """Each tooltip names its OWN capture, not a generic/shared string."""
    assert "alpha.bin" in tile_titles["a"]
    assert "494 frames" in tile_titles["a"]
    assert "officeFullScanAug6.bin" not in tile_titles["a"]

    assert "officeFullScanAug6.bin" in tile_titles["b"]
    assert "4104 frames" in tile_titles["b"]
    assert "alpha.bin" not in tile_titles["b"]
