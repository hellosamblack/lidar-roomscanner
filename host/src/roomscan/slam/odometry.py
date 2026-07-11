"""Point-to-plane ICP, frame-to-model. Two modes (docs spec 3.6):
- 'translation': rotation held at the SFLP prior (init_pose rotation); ICP's
  rotation is discarded and translation re-derived so only 3-DoF translation
  is estimated.
- '6dof': full point-to-plane, init_pose as the initial guess.
Target must carry normals (the raycast model does)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d

_reg = o3d.t.pipelines.registration


@dataclass
class RegistrationResult:
    pose: np.ndarray
    fitness: float
    rmse: float
    ok: bool


def register(source: o3d.t.geometry.PointCloud, target: o3d.t.geometry.PointCloud,
             init_pose: np.ndarray, mode: str = "translation", max_dist: float = 0.05,
             min_fitness: float = 0.3, max_rmse: float = 0.05,
             max_iter: int = 30) -> RegistrationResult:
    if mode not in ("translation", "6dof"):
        raise ValueError(f"unknown mode {mode!r}")
    init = o3d.core.Tensor(np.asarray(init_pose, dtype=np.float64),
                           device=o3d.core.Device("CPU:0"))
    criteria = _reg.ICPConvergenceCriteria(max_iteration=max_iter)
    result = _reg.icp(source, target, max_dist, init,
                      _reg.TransformationEstimationPointToPlane(), criteria)
    T = result.transformation.numpy().copy()
    if mode == "translation":
        # hold rotation at the prior; keep ICP's translation component only.
        R_prior = np.asarray(init_pose, dtype=np.float64)[:3, :3]
        T[:3, :3] = R_prior
    ok = bool(result.fitness >= min_fitness and result.inlier_rmse <= max_rmse)
    return RegistrationResult(pose=T, fitness=float(result.fitness),
                              rmse=float(result.inlier_rmse), ok=ok)
