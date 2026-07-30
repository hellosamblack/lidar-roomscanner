"""Guards on the MCP tool surface.

Two failure modes this catches, both of which quietly erode the surface:

1. A tool registered without a docstring. The docstring *is* the description the
   agent sees, so an undescribed tool is effectively invisible -- the exact
   discoverability problem the server exists to fix.
2. A new script under host/tools/ that nobody exposed and nobody decided not to
   expose. Every script must be either wrapped or listed in EXCLUDED with a reason,
   so the decision is recorded where the next session will encounter it rather than
   left implicit.
"""
from __future__ import annotations

import asyncio
import tomllib
from pathlib import Path

import pytest

from roomscan.mcp_server.server import build

REPO = Path(__file__).resolve().parents[2]
TOOLS_DIR = REPO / "host" / "tools"
PYPROJECT = REPO / "host" / "pyproject.toml"

# script -> the MCP tool(s) that cover it
EXPOSED: dict[str, tuple[str, ...]] = {
    "analyze_capture.py": ("capture_analyze",),
    "headless_doctor.py": ("doctor",),
    "orientation_probe.py": ("orientation_probe",),
    "web_ui_shot.py": ("ui_screenshot", "ui_eval", "ui_wait_for", "ui_reset"),
}

# script -> why it is deliberately CLI-only. Wrapping everything is how a tool
# surface becomes unusable; these are the considered exclusions.
EXCLUDED: dict[str, str] = {
    # Would break the client-never-competitor invariant: these bind the device
    # stream, which roomscan-web owns. Recording goes through rig_record().
    "capture.py": "binds the device stream directly; use rig_record() instead",
    "check_udp.py": "binds the device UDP stream; would starve a live roomscan-web",
    "bench_commands.py": "binds the device CDC stream; rare hardware bench",
    # Scratch tier: hardcoded IPs / Windows paths, superseded by the tools above.
    "check_cdc.py": "scratch: superseded by capture.py and rig_status()",
    "monitor_vcom.py": "scratch: hardcoded Windows ST-Link paths, retired box",
    "query_mdns.py": "scratch: one-shot mDNS lookup, covered by doctor()",
    "test_mdns.py": "scratch: one-shot mDNS browse, covered by doctor()",
    "sniff_mdns.py": "scratch: raw multicast sniff, debugging only",
    "test_udp_receive.py": "scratch: hardcoded board IP",
    "test_send.py": "scratch: hardcoded board IP, single wake byte",
    # Deprecated desktop panel (Web Phase 5).
    "panel_view.py": "deprecated panel.py tooling",
    "panel_ui_smoke.py": "deprecated panel.py tooling; needs a display",
    # Human-in-the-loop or rare one-shot rigs -- no benefit from a tool call.
    "mag_calibrate.py": "human-in-the-loop: the owner must tumble the rig",
    "slam_gpu_memory.py": "one-shot rig: thousands of GPU frames, minutes long",
    "build_flatfield.py": "one-shot: builds a calibration .npz from a panned capture",
    "roll_capture.py": "one-shot: rewrites a capture's quaternions",
    "measure_scene.py": "one-shot: physical deprojection validation",
}

CONSOLE_EXPOSED = {"roomscan-web": ("rig_up",), "roomscan-ctl": ("rig_command",)}
CONSOLE_EXCLUDED = {
    "roomscan-mcp": "this server itself; it cannot be one of its own tools",
    "roomscan-view": "desktop Open3D viewer; needs a display",
    "roomscan-panel": "deprecated (Web Phase 5)",
    "roomscan-slam": "offline batch job; the interactive path is rig_set(mode='slam')",
}


@pytest.fixture(scope="module")
def tools():
    return {t.name: t for t in asyncio.run(build().list_tools())}


def test_every_tool_has_a_description(tools):
    missing = [n for n, t in tools.items() if not (t.description or "").strip()]
    assert not missing, f"tools with no docstring (invisible to the agent): {missing}"


def test_every_tool_has_an_object_schema(tools):
    bad = [n for n, t in tools.items()
           if not isinstance(t.input_schema, dict) or t.input_schema.get("type") != "object"]
    assert not bad, f"tools with a malformed input schema: {bad}"


def test_descriptions_say_what_the_tool_does(tools):
    """First line should be a sentence, not a bare restatement of the name."""
    weak = [n for n, t in tools.items()
            if len((t.description or "").strip().splitlines()[0]) < 20]
    assert not weak, f"tools whose first description line is too thin to be useful: {weak}"


def test_every_host_tool_is_exposed_or_explicitly_excluded(tools):
    scripts = {p.name for p in TOOLS_DIR.glob("*.py") if not p.name.startswith("__")}
    accounted = set(EXPOSED) | set(EXCLUDED)
    unaccounted = scripts - accounted
    assert not unaccounted, (
        f"new script(s) under host/tools/ that are neither exposed as an MCP tool nor "
        f"listed in EXCLUDED: {sorted(unaccounted)}. Wrap it in roomscan.mcp_server, or "
        f"add it to EXCLUDED with the reason it stays CLI-only.")

    stale = accounted - scripts
    assert not stale, f"EXPOSED/EXCLUDED name scripts that no longer exist: {sorted(stale)}"


def test_console_scripts_are_exposed_or_explicitly_excluded(tools):
    data = tomllib.loads(PYPROJECT.read_text())
    scripts = set(data["project"]["scripts"])
    unaccounted = scripts - set(CONSOLE_EXPOSED) - set(CONSOLE_EXCLUDED)
    assert not unaccounted, (
        f"console script(s) neither exposed nor excluded: {sorted(unaccounted)}")


def test_claimed_tools_actually_exist(tools):
    for script, names in {**EXPOSED, **CONSOLE_EXPOSED}.items():
        for name in names:
            assert name in tools, f"{script} claims tool {name!r}, which is not registered"


def test_exclusion_reasons_are_real_sentences():
    for script, reason in {**EXCLUDED, **CONSOLE_EXCLUDED}.items():
        assert len(reason) > 15, f"{script}: exclusion reason is too vague: {reason!r}"
