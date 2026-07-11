"""Build an Open3D tensor PointCloud (source frame) from the Deprojector's
organized (h,w,3) grid + valid mask. Points only -- point-to-plane ICP needs
normals only on the TARGET (the raycast model), not the source."""
from __future__ import annotations

import numpy as np
import open3d as o3d


def _resolve_device(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


def source_cloud(pts_hw3: np.ndarray, valid: np.ndarray,
                 intensity: np.ndarray | None = None,
                 device: str | o3d.core.Device = "CPU:0") -> o3d.t.geometry.PointCloud:
    dev = _resolve_device(device)
    mask = valid.reshape(-1)
    xyz = pts_hw3.reshape(-1, 3)[mask].astype(np.float32, copy=False)
    pc = o3d.t.geometry.PointCloud(dev)
    pc.point.positions = o3d.core.Tensor(xyz, device=dev)
    if intensity is not None:
        inten = intensity.reshape(-1)[mask].astype(np.float32, copy=False)[:, None]
        pc.point["intensity"] = o3d.core.Tensor(inten, device=dev)
    return pc
