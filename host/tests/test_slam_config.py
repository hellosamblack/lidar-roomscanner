"""Tests for SlamConfig: read-only config from [slam] table of roomscan.toml."""
from pathlib import Path

import pytest

from roomscan.slam.config import SlamConfig, preferred_device


def test_defaults():
    """Test that SlamConfig has the correct built-in defaults."""
    c = SlamConfig()
    assert c.icp_mode == "translation"
    assert c.voxel_size == 0.01
    assert c.baro_authority == 0.05
    assert c.baro_tau_frames == 900
    assert c.max_dist == 0.05
    assert c.icp_retry_dist == 0.10
    assert c.min_fitness == 0.3
    assert c.max_rmse == 0.05
    assert c.fov_h == 55.0
    assert c.fov_v == 42.0
    assert c.min_confidence == 20.0
    assert c.weight_threshold == 3.0
    assert c.device == "CPU:0"


def test_load_missing_returns_defaults(tmp_path):
    """Test that loading from a non-existent file returns defaults."""
    assert SlamConfig.load(tmp_path / "nope.toml") == SlamConfig()


def test_load_reads_slam_table(tmp_path):
    """Test that SlamConfig.load reads the [slam] table from TOML.

    Unspecified fields should retain their defaults.
    """
    p = tmp_path / "roomscan.toml"
    p.write_text('[slam]\nicp_mode = "6dof"\nvoxel_size = 0.02\n', encoding="utf-8")
    c = SlamConfig.load(p)
    assert c.icp_mode == "6dof"
    assert c.voxel_size == 0.02
    assert c.baro_authority == 0.05  # unspecified => default


def test_baro_knobs_round_trip_and_retired_weight_is_ignored(tmp_path):
    """BUG-037: the height constraint is parameterized by the barometer's
    measured characteristics (`baro_authority`, `baro_tau_frames`), not by the
    retired per-frame blend gain. A config still carrying `baro_weight` must
    load cleanly -- it is an unknown key like any other, ignored, NOT a crash
    and NOT silently reinterpreted as the new authority (their meanings differ:
    the old gain's DC authority was 1.0)."""
    p = tmp_path / "roomscan.toml"
    p.write_text('[slam]\nbaro_authority = 0.2\nbaro_tau_frames = 300\n', encoding="utf-8")
    c = SlamConfig.load(p)
    assert c.baro_authority == 0.2
    assert c.baro_tau_frames == 300

    p.write_text('[slam]\nbaro_weight = 0.9\n', encoding="utf-8")
    c = SlamConfig.load(p)
    assert not hasattr(c, "baro_weight")
    assert c.baro_authority == 0.05


def test_load_corrupt_returns_defaults(tmp_path):
    """Test that corrupt/malformed TOML returns defaults."""
    p = tmp_path / "roomscan.toml"
    p.write_text("this is not toml =====", encoding="utf-8")
    assert SlamConfig.load(p) == SlamConfig()


def test_load_reads_device_from_slam_table(tmp_path):
    """`device` is a plain string field like the other knobs -- CUDA:0 is not
    testable here (no CUDA build), but the config plumbing itself is: any
    string from the [slam] table round-trips unchanged, and an unspecified
    `device` still defaults to "CPU:0"."""
    p = tmp_path / "roomscan.toml"
    p.write_text('[slam]\ndevice = "CUDA:0"\n', encoding="utf-8")
    c = SlamConfig.load(p)
    assert c.device == "CUDA:0"
    assert c.voxel_size == 0.01   # unspecified => default, unaffected by device

    p2 = tmp_path / "roomscan2.toml"
    p2.write_text('[slam]\nicp_mode = "6dof"\n', encoding="utf-8")
    assert SlamConfig.load(p2).device == "CPU:0"


def test_preferred_device_returns_valid_string(monkeypatch):
    """preferred_device() returns a well-formed Open3D device string and
    tracks o3d.core.cuda.is_available(): CUDA:0 when CUDA is present, else
    CPU:0. Both branches exercised by faking is_available()."""
    import open3d as o3d

    monkeypatch.setattr(o3d.core.cuda, "is_available", lambda: True)
    assert preferred_device() == "CUDA:0"

    monkeypatch.setattr(o3d.core.cuda, "is_available", lambda: False)
    assert preferred_device() == "CPU:0"


def test_preferred_device_degrades_to_cpu_on_error(monkeypatch):
    """Any failure probing CUDA support degrades safely to CPU:0 (never
    raises), so a broken/partial Open3D install can't crash the panel."""
    import open3d as o3d

    def _boom():
        raise RuntimeError("cuda probe blew up")

    monkeypatch.setattr(o3d.core.cuda, "is_available", _boom)
    assert preferred_device() == "CPU:0"


def test_view_cadence_defaults():
    from roomscan.slam.config import SlamConfig
    cfg = SlamConfig()
    assert cfg.mesh_upload_hz == 3.0
    assert cfg.live_vertex_budget == 150000
    assert cfg.fps_budget_ms == 8.0


def test_view_cadence_overrides_from_toml(tmp_path):
    from roomscan.slam.config import SlamConfig
    p = tmp_path / "roomscan.toml"
    p.write_text(
        "[slam]\n"
        "mesh_upload_hz = 5.0\n"
        "live_vertex_budget = 80000\n"
        "fps_budget_ms = 4.0\n",
        encoding="utf-8")
    cfg = SlamConfig.load(p)
    assert cfg.mesh_upload_hz == 5.0
    assert cfg.live_vertex_budget == 80000
    assert cfg.fps_budget_ms == 4.0


def test_release_cache_every_default_and_toml_override(tmp_path):
    # Sub-phase 6.G: on by default (release after every extraction), and
    # overridable -- 0 restores the pre-fix behaviour for A/B measurement.
    from roomscan.slam.config import SlamConfig
    assert SlamConfig().release_cache_every == 1
    p = tmp_path / "roomscan.toml"
    p.write_text("[slam]\nrelease_cache_every = 0\n", encoding="utf-8")
    assert SlamConfig.load(p).release_cache_every == 0


def test_block_count_default_matches_tsdf_and_reads_from_toml(tmp_path):
    # BUG-035. SlamConfig spells the default as a literal so this module stays
    # importable without open3d -- pin it to the real constant so the two
    # cannot drift apart silently.
    from roomscan.slam.config import SlamConfig
    from roomscan.slam.tsdf import DEFAULT_BLOCK_COUNT
    assert SlamConfig().block_count == DEFAULT_BLOCK_COUNT
    p = tmp_path / "roomscan.toml"
    p.write_text("[slam]\nblock_count = 500000\n", encoding="utf-8")
    assert SlamConfig.load(p).block_count == 500000


def test_icp_retry_dist_is_configurable(tmp_path):
    """The retry radius must be tunable from [slam] -- including off (0),
    which restores the pre-fix single-attempt behavior."""
    f = tmp_path / "roomscan.toml"
    f.write_text("[slam]\nicp_retry_dist = 0.0\n", encoding="utf-8")
    assert SlamConfig.load(f).icp_retry_dist == 0.0

    f.write_text("[slam]\nicp_retry_dist = 0.25\n", encoding="utf-8")
    assert SlamConfig.load(f).icp_retry_dist == 0.25


def test_icp_retry_dist_is_wider_than_max_dist():
    """A retry no wider than the first attempt could never rescue anything."""
    c = SlamConfig()
    assert c.icp_retry_dist > c.max_dist
