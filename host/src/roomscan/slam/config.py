"""Read-only SLAM config from the [slam] table of roomscan.toml.

Deliberately NO writer -- roomscan.config's single-table writer is off-limits
(its docstring forbids growing it). Priority for the CLI is:
  flag > this file > dataclass default.
"""
from __future__ import annotations

import tomllib
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Optional

from ..config import config_path


@dataclass
class SlamConfig:
    """SLAM configuration read from [slam] table in roomscan.toml.

    Missing or corrupt config files are tolerated -- all fields fall back to
    built-in defaults. Only recognized fields are pulled from a present
    ``[slam]`` table; anything else is ignored.
    """

    icp_mode: str = "translation"
    voxel_size: float = 0.01
    baro_weight: float = 0.05
    max_dist: float = 0.05
    min_fitness: float = 0.3
    max_rmse: float = 0.05
    fov_h: float = 55.0
    fov_v: float = 42.0
    # Task 13 (data-quality): reflectance color + noise reduction, tuned against
    # captures/phase6_motion_ref.bin -- see task-quality-report.md.
    min_confidence: float = 20.0
    weight_threshold: float = 3.0
    # Compute device for the Open3D tensor pipeline (TsdfMap/pinhole/
    # source_cloud/register). "CPU:0" today -- the installed Open3D 0.19
    # build here has no CUDA support -- but "CUDA:0" (or any other
    # o3d.core.Device string) runs unchanged once a CUDA-enabled build is
    # installed; see slam/mapper.py's docstring.
    device: str = "CPU:0"

    @classmethod
    def load(cls, path: Optional[Path] = None) -> "SlamConfig":
        """Load SLAM config from [slam] table in roomscan.toml.

        Args:
            path: Path to TOML file. If None, uses config_path() from roomscan.config.

        Returns:
            SlamConfig with values from file, or defaults if file is missing,
            unreadable, corrupt, or missing the [slam] table.
        """
        path = Path(path) if path is not None else config_path()
        try:
            raw = path.read_text(encoding="utf-8")
        except OSError:
            return cls()
        try:
            data = tomllib.loads(raw)
        except tomllib.TOMLDecodeError:
            return cls()
        table = data.get("slam")
        if not isinstance(table, dict):
            return cls()
        known = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in table.items() if k in known}
        try:
            return cls(**kwargs)
        except TypeError:
            return cls()
