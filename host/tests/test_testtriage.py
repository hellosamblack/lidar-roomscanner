"""Tests for the static import-graph test triage (roomscan.testtriage).

The property that matters is correctness-first: a change the graph cannot map with
confidence must widen to a FULL run, never an empty selection -- a false negative
here silently skips a real regression.
"""
from __future__ import annotations

from roomscan import testtriage as tt


def test_graph_resolves_known_edge():
    g = tt.build_import_graph()
    # slam/mapper.py -> test_slam_mapper.py, by the naming convention.
    closure = g["test_closure"]["tests/test_slam_mapper.py"]
    assert "roomscan.slam.mapper" in closure
    # transitive: mapper pulls in tsdf.
    assert "roomscan.slam.tsdf" in closure
    assert len(g["test_closure"]) > 100          # the whole suite is in the graph


def test_leaf_module_selects_its_own_test_and_discriminates():
    r = tt.impacted_tests(changed=["host/src/roomscan/weather.py"])
    assert not r["full_run"]
    assert "tests/test_weather.py" in r["selected"]
    # a leaf must NOT drag in the whole suite.
    assert r["selected_count"] < r["total_tests"] // 2


def test_always_run_tests_always_selected():
    r = tt.impacted_tests(changed=["host/src/roomscan/weather.py"])
    for t in tt.ALWAYS_RUN:
        assert f"tests/{t}" in r["selected"], t


def test_changed_test_file_is_selected_directly():
    r = tt.impacted_tests(changed=["host/tests/test_flatfield.py"])
    assert "tests/test_flatfield.py" in r["selected"]
    assert r["reasons"]["tests/test_flatfield.py"] == "changed test file"


def test_foundational_change_forces_full_run():
    for f in ("host/src/roomscan/protocol.py",
              "host/src/roomscan/config.py",
              "host/pyproject.toml"):
        r = tt.impacted_tests(changed=[f])
        assert r["full_run"], f
        assert r["selected_count"] == r["total_tests"]


def test_shared_fixture_change_forces_full_run():
    r = tt.impacted_tests(changed=["host/tests/fixtures/golden_frame.bin"])
    assert r["full_run"]


def test_shared_test_helper_change_forces_full_run():
    # a non-test .py under tests/ is a shared helper (golden.py etc) -> full run.
    r = tt.impacted_tests(changed=["host/tests/golden.py"])
    assert r["full_run"]


def test_unmapped_source_change_forces_full_run():
    # a source .py the graph has no module for -> can't reason -> full run.
    r = tt.impacted_tests(changed=["host/src/roomscan/does_not_exist.py"])
    assert r["full_run"]
    r2 = tt.impacted_tests(changed=["host/src/roomscan/data_blob.bin"])
    assert r2["full_run"]                          # non-python source change


def test_firmware_change_has_no_host_impact():
    # firmware is CI's job; it must NOT trigger a pointless full host run.
    r = tt.impacted_tests(changed=["firmware/scanner-stream/src/main.c"])
    assert not r["full_run"]
    # only the always-run guards remain.
    assert set(r["selected"]) == {f"tests/{t}" for t in tt.ALWAYS_RUN}


def test_path_rule_maps_non_python_producer():
    r = tt.impacted_tests(changed=["host/transform/rs_transform_shim.c"])
    assert not r["full_run"]
    assert "tests/test_protocol_c_crosscheck.py" in r["selected"]

    r2 = tt.impacted_tests(changed=["pi-bridge/build_image.py"])
    assert "tests/test_pi_bridge_build.py" in r2["selected"]


def test_static_asset_change_covered_without_full_run():
    r = tt.impacted_tests(changed=["host/src/roomscan/static/app.js"])
    assert not r["full_run"]
    assert "tests/test_static_ui.py" in r["selected"]


def test_no_changes_runs_only_the_guards_never_full_run():
    r = tt.impacted_tests(changed=[])
    assert not r["full_run"]
    # No mapped modules -> only the cheap cross-cutting guards run. Never a full
    # run, and never a surprise: the selection stays honest and small.
    assert r["selected"] == [f"tests/{t}" for t in sorted(tt.ALWAYS_RUN)]
