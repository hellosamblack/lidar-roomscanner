"""Wiring tests for the triage MCP surface and the conftest marker mechanism.

These do not run the suite -- they exercise the plumbing (marker auto-application,
the two-pass merge's exit-code handling, the impacted_tests tool shape) so a silent
break in the mechanism is caught, per the "skill mechanisms need execution tests"
lesson.
"""
from __future__ import annotations

from pathlib import Path

from roomscan.mcp_server import tools_build as tb

impacted_tool = getattr(tb.impacted_tests, "fn", tb.impacted_tests)


def test_impacted_tests_tool_returns_selection_shape():
    r = impacted_tool()
    assert {"selected", "reasons", "full_run", "changed",
            "selected_count", "total_tests"} <= set(r)
    assert isinstance(r["selected"], list)


def test_merge_treats_no_tests_collected_as_neutral():
    passed = {"returncode": 0, "counts": {"passed": 3}, "failures": [],
              "failure_count": 0, "summary": "3 passed", "tail": ""}
    empty = {"returncode": 5, "counts": {}, "failures": [],
             "failure_count": 0, "summary": "no tests ran", "tail": ""}
    assert tb._merge_pytest(passed, empty)["ok"] is True     # exit 5 == neutral
    failed = {"returncode": 1, "counts": {"failed": 1}, "failures": ["x::y"],
              "failure_count": 1, "summary": "1 failed", "tail": "boom"}
    assert tb._merge_pytest(passed, failed)["ok"] is False


def test_join_markers_composes_user_and_pass_filter():
    assert tb._join_markers("", "serial") == "serial"
    assert tb._join_markers("not slow", "not serial") == "(not slow) and (not serial)"


def test_conftest_family_detection():
    # The detector drives the auto-marking; assert it on known files so a broken
    # pattern is caught (a mechanism without a test silently dies).
    from tests import conftest as cf
    tests_dir = Path(__file__).parent

    def fams(name):
        return cf._families(str(tests_dir / name))

    assert {"net", "slow"} <= fams("test_slam_remote.py")          # socket.bind
    assert {"subprocess", "slow"} <= fams("test_protocol_c_crosscheck.py")
    assert "open3d" in fams("test_slam_meshprep.py")
    assert fams("test_run_tests_triage.py") == frozenset()          # this file: none


def test_serial_quarantine_is_applied_at_collection(request):
    # The known xdist-flaky test must carry the serial marker so the parallel split
    # routes it to the serial pass. Check the applied marker on the collected item
    # when it is in this run; otherwise just pin the quarantine is non-empty.
    from tests.conftest import SERIAL_NODEIDS
    assert SERIAL_NODEIDS                                           # non-empty
    target = [it for it in request.session.items
              if it.nodeid.endswith("test_restart_resets_the_ephemeral_map")]
    if target:                       # populated only when test_web.py is collected
        assert any(m.name == "serial" for m in target[0].iter_markers())
