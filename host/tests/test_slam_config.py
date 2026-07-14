"""Tests for SlamConfig: read-only config from [slam] table of roomscan.toml."""
from pathlib import Path

import pytest

from roomscan.slam.config import SlamConfig, preferred_device


def test_defaults():
    """Test that SlamConfig has the correct built-in defaults."""
    c = SlamConfig()
    assert c.icp_mode == "translation"
    assert c.voxel_size == 0.01
    assert c.baro_weight == 0.05
    assert c.max_dist == 0.05
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
    assert c.baro_weight == 0.05  # unspecified => default


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
