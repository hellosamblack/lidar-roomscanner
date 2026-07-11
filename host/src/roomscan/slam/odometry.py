"""Point-to-plane ICP, frame-to-model. Two modes (docs spec 3.6):
- 'translation': rotation held at the SFLP prior (init_pose rotation); a
  genuine 3-DoF point-to-plane translation solve (Task 9.5 Lever 2) --
  NOT the full 6-DoF ICP with the rotation discarded afterward. Cheaper
  (3x3 normal equations vs 6x6 per iteration) and geometrically honest: gate
  stats (fitness/rmse) reflect the actual translation-only alignment, not a
  6-DoF fit that gets partially thrown away.
- '6dof': full point-to-plane, init_pose as the initial guess (Open3D's
  tensor ICP, unchanged).
Target must carry normals (the raycast model does)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import open3d as o3d

_reg = o3d.t.pipelines.registration
_CPU = o3d.core.Device("CPU:0")

# Condition-number ceiling for the 3x3 translation normal-equations matrix
# A = sum(n_i n_i^T). A perfectly (or near-) planar target has all normals
# pointing the same way, so A is rank-deficient in the two in-plane
# directions -- in-plane translation is genuinely unrecoverable from
# point-to-plane residuals there, not an artifact of a particular solver.
_COND_CEILING = 1e8


@dataclass
class RegistrationResult:
    pose: np.ndarray
    fitness: float
    rmse: float
    ok: bool


def _translation_icp(rotated_src: np.ndarray, tgt_pts: np.ndarray, tgt_normals: np.ndarray,
                     t0: np.ndarray, max_dist: float, max_iter: int,
                     tol: float = 1e-7) -> tuple[np.ndarray, float, float, bool]:
    """Iterated closest-point, translation-only, point-to-plane. `rotated_src`
    is the source cloud with the (held-fixed) prior rotation already applied.
    Returns (t, fitness, rmse, singular) -- fitness/rmse mirror Open3D's ICP
    result semantics (fitness = matched_fraction, rmse = RMS of the
    point-to-plane residual among matches); singular=True means the normal
    equations were too ill-conditioned to trust (e.g. constant-normal planar
    geometry) and the caller should treat this as a failed registration."""
    n_source = rotated_src.shape[0]
    tgt_t = o3d.core.Tensor(tgt_pts, device=_CPU)
    nns = o3d.core.nns.NearestNeighborSearch(tgt_t)
    nns.hybrid_index()

    t = np.asarray(t0, dtype=np.float64).copy()
    fitness, rmse = 0.0, float("inf")
    for _ in range(max_iter):
        query = rotated_src + t
        idx, _dist2, counts = nns.hybrid_search(
            o3d.core.Tensor(query, device=_CPU), max_dist, 1)
        matched = counts.numpy().reshape(-1) > 0
        n_valid = int(matched.sum())
        if n_valid == 0:
            return t, 0.0, float("inf"), False
        rows = idx.numpy().reshape(-1)[matched]
        q = tgt_pts[rows]
        n = tgt_normals[rows]
        p = query[matched]
        r = np.einsum("ij,ij->i", n, p - q)             # point-to-plane residual
        fitness = n_valid / n_source
        rmse = float(np.sqrt(np.mean(r ** 2)))

        a = n.T @ n                                       # 3x3 normal equations
        cond = np.linalg.cond(a)
        if not np.isfinite(cond) or cond > _COND_CEILING:
            return t, fitness, rmse, True
        b = -(n * r[:, None]).sum(axis=0)
        dt = np.linalg.solve(a, b)
        t = t + dt
        if np.linalg.norm(dt) < tol:
            break
    return t, fitness, rmse, False


def register(source: o3d.t.geometry.PointCloud, target: o3d.t.geometry.PointCloud,
             init_pose: np.ndarray, mode: str = "translation", max_dist: float = 0.05,
             min_fitness: float = 0.3, max_rmse: float = 0.05,
             max_iter: int = 12) -> RegistrationResult:
    if mode not in ("translation", "6dof"):
        raise ValueError(f"unknown mode {mode!r}")
    init_pose = np.asarray(init_pose, dtype=np.float64)

    if mode == "translation":
        R = init_pose[:3, :3]
        src_pts = source.point.positions.numpy().astype(np.float64, copy=False)
        tgt_pts = target.point.positions.numpy().astype(np.float64, copy=False)
        tgt_normals = target.point.normals.numpy().astype(np.float64, copy=False)
        rotated_src = (R @ src_pts.T).T
        t, fitness, rmse, singular = _translation_icp(
            rotated_src, tgt_pts, tgt_normals, init_pose[:3, 3], max_dist, max_iter)
        if singular:
            return RegistrationResult(pose=init_pose.copy(), fitness=0.0,
                                      rmse=float("inf"), ok=False)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        ok = bool(fitness >= min_fitness and rmse <= max_rmse)
        return RegistrationResult(pose=T, fitness=float(fitness), rmse=float(rmse), ok=ok)

    init = o3d.core.Tensor(init_pose, device=_CPU)
    criteria = _reg.ICPConvergenceCriteria(max_iteration=max_iter)
    try:
        result = _reg.icp(source, target, max_dist, init,
                          _reg.TransformationEstimationPointToPlane(), criteria)
    except RuntimeError:
        # Point-to-plane's 6x6 normal-equations solve is singular on large,
        # near-planar, texture-poor surfaces (e.g. a blank wall filling the
        # FOV) -- Open3D raises rather than returning a degenerate result.
        # Degrade to tracking-lost instead of crashing the mapper.
        return RegistrationResult(pose=init_pose.copy(), fitness=0.0,
                                  rmse=float("inf"), ok=False)
    T = result.transformation.numpy().copy()
    ok = bool(result.fitness >= min_fitness and result.inlier_rmse <= max_rmse)
    return RegistrationResult(pose=T, fitness=float(result.fitness),
                              rmse=float(result.inlier_rmse), ok=ok)
