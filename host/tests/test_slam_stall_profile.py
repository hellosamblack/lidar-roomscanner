"""Guards on the GIL-starvation tick accounting in
`host/tools/slam_stall_profile.py` (issue #74).

BUG-063's shape: a watchdog thread that gets almost no chance to run produces
almost no SAMPLES to sum, so summed-lateness percentages ("starved_pct_of_wall")
UNDER-report exactly the worst case -- 1 tick where ~2186 were due read as a
*lower* starvation percentage than a stage that ran freely. `tick_share`
(observed ticks / expected ticks from wall time and the watchdog period) is the
metric that can see this. These tests inject known watchdog samples and stage
wall times directly (no real thread, no capture file, no scheduling
dependency) so the math is pinned deterministically.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "tools"))

import slam_stall_profile as sp                                   # noqa: E402


def _watchdog(period_s=0.005):
    return sp.GilWatchdog(period_s=period_s)


def test_healthy_stage_reports_tick_share_near_one():
    """~95/100 expected ticks landed -> tick_share ~= 0.95, and the legacy
    fields are still there so downstream JSON consumers do not break."""
    wd = _watchdog(period_s=0.01)
    wd.late["step"] = [0.001] * 95          # 95 ticks observed
    wall_by_stage = {"step": 1.0}           # 1.0s / 0.01s period = 100 expected
    out = wd.report(1.0, wall_by_stage)

    step = out["step"]
    assert step["ticks"] == 95
    assert step["expected_ticks"] == 100
    assert step["tick_share"] == pytest.approx(0.95, abs=1e-9)
    # legacy/back-compat fields
    assert step["starved_s"] == pytest.approx(0.095, abs=1e-6)
    assert step["worst_stall_ms"] == pytest.approx(1.0, abs=1e-6)
    assert "starved_pct_of_wall" in step


def test_bug_063_near_total_starvation_has_near_zero_tick_share():
    """The reported BUG-063 failure shape: 1 tick during ~10.93s at a 5ms
    period (~2186 ticks due). Even a single fairly late sample keeps the
    legacy summed-lateness percentage looking modest -- tick_share is the
    field that reveals the near-total freeze."""
    period_s = 0.005
    wall_s = 10.93
    wd = _watchdog(period_s=period_s)
    # One tick, moderately late -- NOT itself an extreme value, so the legacy
    # percentage alone would look unremarkable.
    wd.late["mesh"] = [0.35]
    wall_by_stage = {"mesh": wall_s}
    out = wd.report(wall_s, wall_by_stage)

    mesh = out["mesh"]
    assert mesh["ticks"] == 1
    assert mesh["expected_ticks"] == int(wall_s / period_s)
    assert mesh["expected_ticks"] == 2186
    # tick_share is rounded to 4 places (matching slam_icp_bench.py), so the
    # tolerance has to be wider than the rounding step, not tighter than it.
    assert mesh["tick_share"] == pytest.approx(1 / 2186, abs=5e-5)
    assert mesh["tick_share"] < 0.001

    # The legacy metric looks "modest", i.e. does NOT itself scream total
    # starvation -- that is precisely why it is not trustworthy here.
    assert mesh["starved_pct_of_wall"] < 5.0


def test_zero_duration_stage_does_not_divide_by_zero():
    """A stage with no attributed wall time (e.g. never entered) must return a
    sensible None tick-share, not raise or silently report a nonsense ratio."""
    wd = _watchdog(period_s=0.005)
    wd.late["pack"] = [0.001, 0.002]        # ticks exist even though wall is 0
    out = wd.report(10.0, {"pack": 0.0})    # explicit zero wall for the stage

    pack = out["pack"]
    assert pack["ticks"] == 2
    assert pack["expected_ticks"] == 0
    assert pack["tick_share"] is None


def test_report_without_wall_by_stage_still_returns_legacy_fields():
    """Calling `report()` the old way (positional wall_s only) must not raise,
    and the new fields degrade to None/0 rather than crashing -- protects any
    caller that has not been updated to pass per-stage wall time."""
    wd = _watchdog(period_s=0.005)
    wd.late["idle"] = [0.0, 0.0, 0.2]
    out = wd.report(5.0)
    idle = out["idle"]
    assert idle["ticks"] == 3
    assert idle["expected_ticks"] == 0
    assert idle["tick_share"] is None
    # starved_pct_of_wall is rounded to 1 decimal place; pick a sample big
    # enough that the rounding itself does not swallow the assertion.
    assert idle["starved_pct_of_wall"] == pytest.approx(round(100.0 * 0.2 / 5.0, 1), abs=1e-6)


def test_overall_tick_headline_matches_bug_063_shape():
    """`_tick_headline` is the pure function behind the run-level `tick_share`
    (item 3 of the plan): total observed ticks across every stage (including
    idle) against wall_s / period_s."""
    period_s = 0.005
    wall_s = 10.93
    late = {"step": [0.001] * 40, "mesh": [0.35], "idle": [0.0] * 5}
    out = sp._tick_headline(late, period_s, wall_s)
    assert out["ticks"] == 46
    assert out["expected_ticks"] == 2186
    assert out["tick_share"] == pytest.approx(46 / 2186, abs=5e-5)
    assert out["tick_share"] < 0.03    # nowhere near fully sampled


def test_overall_tick_headline_zero_wall_returns_none_share():
    out = sp._tick_headline({"step": [0.001]}, 0.005, 0.0)
    assert out["ticks"] == 1
    assert out["expected_ticks"] == 0
    assert out["tick_share"] is None


def test_print_report_shows_ticks_and_share(capsys):
    """Console output must make sampling coverage visible next to the legacy
    starvation percentage (item 4 of the plan) -- grep-style check on the
    literal report text, not just the dict."""
    rep = {
        "capture": "x.bin", "width": 54, "height": 42, "device": "CPU:0",
        "block_count": 8, "decimate": False,
        "wall_s": 10.93, "frames_integrated": 300, "frames": 300,
        "effective_fps": 27.5,
        "stages": {},
        "gil_starvation": {
            "mesh": {"ticks": 1, "expected_ticks": 2186, "tick_share": 0.0005,
                     "starved_s": 0.35, "starved_pct_of_wall": 3.2,
                     "worst_stall_ms": 350.0},
        },
        "starved_pct_of_wall": 3.2,
        "worst_stall_ms": 350.0,
        "ticks": 1, "expected_ticks": 2186, "tick_share": 0.0005,
        "stalls_over_300ms": 1,
        "meshes": 60, "final_source_verts": 0, "final_sent_verts": 0,
        "final_payload_mb": 0.0, "peak_payload_mb": 0.0,
    }
    sp._print_report(rep)
    out = capsys.readouterr().out
    assert "ticks 1/2186" in out
    assert "share 0.001" in out or "share 0.0005" in out
    # TOTAL line carries the overall headline too.
    assert "TOTAL" in out and ("share" in out.split("TOTAL", 1)[1].splitlines()[0])


def test_report_math_matches_slam_icp_bench_semantics():
    """Same formula (`ticks / (stage_wall_s / period_s)`, `int()` truncation
    of expected_ticks) as the reference implementation `_Watchdog.report()` in
    slam_icp_bench.py, so the two tools cannot silently drift apart again."""
    period_s = 0.005
    wd = _watchdog(period_s=period_s)
    wd.late["prep"] = [0.001] * 37
    stage_wall = 1.001            # deliberately not a clean multiple of period
    out = wd.report(2.0, {"prep": stage_wall})
    expected = stage_wall / period_s
    assert out["prep"]["expected_ticks"] == int(expected)
    assert out["prep"]["tick_share"] == round(37 / expected, 4)
