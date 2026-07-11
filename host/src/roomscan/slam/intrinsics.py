"""Pinhole camera intrinsic from the ToF field of view.

Approximates the Deprojector's per-zone tan model (deproject.py) with a single
pinhole matrix, which is what Open3D's VoxelBlockGrid integrate/raycast consume.
The two diverge up to ~6% of z at the extreme corners (docs/deprojector-validation.md);
acceptable for TSDF fusion. Uses the SAME zone-center / FoV convention as Deprojector."""
from __future__ import annotations

import math

import numpy as np
import open3d as o3d


def _resolve_device(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


def pinhole(width: int, height: int, fov_h_deg: float = 55.0,
            fov_v_deg: float = 42.0, device: str | o3d.core.Device = "CPU:0") -> o3d.core.Tensor:
    fx = (width / 2.0) / math.tan(math.radians(fov_h_deg) / 2.0)
    fy = (height / 2.0) / math.tan(math.radians(fov_v_deg) / 2.0)
    cx = width / 2.0
    cy = height / 2.0
    k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    return o3d.core.Tensor(k, dtype=o3d.core.Dtype.Float64, device=_resolve_device(device))
