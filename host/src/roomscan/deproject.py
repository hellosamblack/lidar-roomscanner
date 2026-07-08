"""Depth (perpendicular Z, mm) -> 3D points (m). FoV defaults are
datasheet-derived (VL53L9CX DS14879 rev 6, Table 3 "FoV angles" p.5 + Figure 26
p.38: 55 deg horizontal x 42 deg vertical at full 54x42 resolution) --
see docs/vl53l9cx-fov-notes.md for citations and derivation. ZAPC validation
against a real device is pending in Phase 2.5 Task 3."""
from __future__ import annotations

import numpy as np


class Deprojector:
    def __init__(self, width: int, height: int, fov_h_deg: float = 55.0,
                 fov_v_deg: float = 42.0, max_range_mm: float = 10000.0):
        ax = np.deg2rad(((np.arange(width) + 0.5) / width - 0.5) * fov_h_deg)
        ay = np.deg2rad(((np.arange(height) + 0.5) / height - 0.5) * fov_v_deg)
        self._tan_x = np.tan(ax)[None, :]   # (1, w)
        self._tan_y = np.tan(ay)[:, None]   # (h, 1)
        self.max_range_mm = max_range_mm

    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        z = depth_mm.astype(np.float64, copy=False)
        valid = np.isfinite(z) & (z > 0.0) & (z < self.max_range_mm)
        x = z * self._tan_x
        y = z * self._tan_y
        y = np.broadcast_to(y, z.shape)
        x = np.broadcast_to(x, z.shape)
        return np.stack([x[valid], y[valid], z[valid]], axis=1) / 1000.0
