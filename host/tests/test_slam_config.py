"""Tests for SlamConfig: read-only config from [slam] table of roomscan.toml."""
from pathlib import Path

import pytest

from roomscan.slam.config import SlamConfig


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
