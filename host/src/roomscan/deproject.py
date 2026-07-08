"""Depth (perpendicular Z, mm) -> 3D points (m). FoV defaults are
datasheet-derived (VL53L9CX DS14879 rev 6, Table 3 "FoV angles" p.5 + Figure 26
p.38: 55 deg horizontal x 42 deg vertical at full 54x42 resolution) --
see docs/vl53l9cx-fov-notes.md for citations and derivation.

ZAPC-validated (Phase 2.5 Task 3, see docs/deprojector-validation.md): the
factory-calibrated on-device point cloud (ZAPC) confirms the 55x42 defaults
under the zone-center convention already implemented here (least-squares
best fit over ~6700 valid zones across 3 golden frames: 54.65 x 42.50 deg,
within ~0.6 deg of the datasheet) -- so the linear model's *global* FoV
constants are correct as-is and were NOT changed. But the fit's residual
(RMS ~0.5 deg) is dominated by real per-zone lens distortion the linear
model can't represent (median displacement ~1% of z, but worst-case ~6.3%
of z at the extreme corners) -- a pure FoV-default tweak does not fix this
(same worst-case with the best-fit FoV), so a per-zone tan-table escape
hatch was added instead of changing the defaults: pass `zone_tan_x`/
`zone_tan_y` ((height, width) arrays, e.g. seeded from ZAPC's x/z, y/z per
zone) to bypass the separable linear model entirely. Linear (FoV-based)
stays the default -- most callers don't have per-device ZAPC data handy.
"""
from __future__ import annotations

import numpy as np


class Deprojector:
    def __init__(self, width: int, height: int, fov_h_deg: float = 55.0,
                 fov_v_deg: float = 42.0, max_range_mm: float = 10000.0,
                 zone_tan_x: np.ndarray | None = None, zone_tan_y: np.ndarray | None = None):
        """zone_tan_x/zone_tan_y: optional (height, width) per-zone tan(angle) tables
        that override the separable linear FoV model (see module docstring). Must be
        supplied together; both are validated for shape. When omitted (default), the
        linear zone-center model from fov_h_deg/fov_v_deg is used, matching prior
        behavior exactly."""
        if (zone_tan_x is None) != (zone_tan_y is None):
            raise ValueError("zone_tan_x and zone_tan_y must be provided together")
        if zone_tan_x is not None:
            if zone_tan_x.shape != (height, width) or zone_tan_y.shape != (height, width):
                raise ValueError(
                    f"zone_tan_x/zone_tan_y must both have shape ({height}, {width}), "
                    f"got {zone_tan_x.shape} / {zone_tan_y.shape}"
                )
            self._tan_x = zone_tan_x.astype(np.float64, copy=False)  # (h, w)
            self._tan_y = zone_tan_y.astype(np.float64, copy=False)  # (h, w)
        else:
            ax = np.deg2rad(((np.arange(width) + 0.5) / width - 0.5) * fov_h_deg)
            ay = np.deg2rad(((np.arange(height) + 0.5) / height - 0.5) * fov_v_deg)
            self._tan_x = np.tan(ax)[None, :]   # (1, w) -- broadcasts over rows
            self._tan_y = np.tan(ay)[:, None]   # (h, 1) -- broadcasts over cols
        self.max_range_mm = max_range_mm

    def __call__(self, depth_mm: np.ndarray) -> np.ndarray:
        z = depth_mm.astype(np.float64, copy=False)
        valid = np.isfinite(z) & (z > 0.0) & (z < self.max_range_mm)
        x = z * self._tan_x
        y = z * self._tan_y
        y = np.broadcast_to(y, z.shape)
        x = np.broadcast_to(x, z.shape)
        return np.stack([x[valid], y[valid], z[valid]], axis=1) / 1000.0
