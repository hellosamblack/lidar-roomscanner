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


def _resolve_device(device) -> o3d.core.Device:
    return device if isinstance(device, o3d.core.Device) else o3d.core.Device(device)


# Condition-number CAP for the 3x3 translation normal-equations matrix
# A = sum(n_i n_i^T). A perfectly (or near-) planar target has all normals
# pointing the same way, so A is rank-deficient in the two in-plane
# directions -- in-plane translation is genuinely unrecoverable from
# point-to-plane residuals there, not an artifact of a particular solver.
#
# BUG-068: this used to be a rejection threshold at 1e8, which no real frame can
# reach -- over 3018 consecutive frames of a room scan the worst conditioning
# observed is 203.5 (median 7.8, p99 39.1), so the guard fired on zero frames and
# was dead code. What it let through: at cond 10-200 the estimate slid 1.2 m in
# 3 s along the weakest-observability axis, through a wall, at fitness 0.867 and
# with no frame reported lost.
#
# Rejecting harder is the wrong fix -- a rejected frame is tracking-lost,
# `predict_pose` freezes translation at t_prev and nothing relocalizes, which is
# exactly BUG-036 (one rejected frame cost 423 frames of a circuit). So instead of
# refusing to solve, we bound HOW FAR the solve may move along directions the
# geometry cannot observe: floor the eigenvalues of A at lambda_max / _COND_CAP.
# Observable directions (eigenvalue already above the floor) are untouched, and a
# frame whose conditioning is under the cap takes the original solve path and is
# bit-identical -- see `_solve_translation_step`.
#
# 20 was chosen by a matched ensemble sweep (n=5 per point) over three captures,
# NOT by picking the value that most flatters the failing one. Max excursion,
# where the tripod capture's truth is ~0 and the other two are real travel:
#
#   cond_cap        imuTranslationError    coffeeRoomCircuitNoMnt   roomSweepFull
#   0 (pre-fix)     0.752 +/- 0.489        3.440 +/- 0.072          3.398 +/- 0.207
#   40              0.675 +/- 0.327        3.389 +/- 0.024          3.498 +/- 0.110
#   20  <- shipped  0.459 +/- 0.074        3.406 +/- 0.066          3.373 +/- 0.158
#   10              0.454 +/- 0.017        3.369 +/- 0.008          2.025 +/- 0.371
#
# The number that matters is the STANDARD DEVIATION on the tripod capture, not
# the mean: at cap 0 the run is bistable (BUG-070) -- some runs survive the
# ill-conditioned window at t~88 s and some slide 1.2 m -- and capping collapses
# that spread 0.489 -> 0.074. Real travel is untouched: both real-motion captures
# stay inside their own ensemble spread, and their path length falls (28.9 -> 21.6 m
# on the circuit) while max excursion does not, which is jitter leaving, not signal.
#
# 10 is deliberately NOT shipped even though it scores best on the tripod: it moves
# roomSweepFull's reported displacement by 40% (3.398 -> 2.025), i.e. it is eating
# real motion. It also damps ~40% of frames, where 20 damps ~10% (cond p90 = 19.7),
# so 20 stays a tail-targeted fix rather than a change to normal operation.
_COND_CAP = 20.0


def _solve_translation_step(a: np.ndarray, b: np.ndarray,
                            cond_cap: float = _COND_CAP) -> tuple[np.ndarray | None, float]:
    """Solve the 3x3 point-to-plane normal equations `a dt = b`, bounding the step
    along directions the geometry cannot observe. Returns (dt, cond); dt is None
    when `a` is genuinely singular and the caller should report a failed
    registration.

    Below `cond_cap` this is `np.linalg.solve(a, b)` verbatim -- the eigen path is
    not taken at all, so well-conditioned frames are bit-identical to the
    pre-BUG-068 solver. Above it, `a`'s eigenvalues are floored at
    `lambda_max / cond_cap`, which shrinks dt along the weak directions (dt's
    component on eigenvector i is (v_i.b)/w_i, so raising a small w_i shrinks it)
    and leaves every direction already above the floor exactly as it was.

    `cond_cap <= 0` disables the cap, restoring the original unbounded solve.
    """
    try:
        cond = float(np.linalg.cond(a))
    except np.linalg.LinAlgError:
        # `cond` runs an SVD, which raises rather than returning inf on a
        # non-finite matrix. A NaN normal-equations matrix is unsolvable, which
        # is the same answer as a non-finite condition number.
        return None, float("inf")
    if not np.isfinite(cond):
        return None, cond
    if cond_cap <= 0.0 or cond <= cond_cap:
        return np.linalg.solve(a, b), cond
    w, v = np.linalg.eigh(a)            # ascending; `a` is symmetric PSD by construction
    w_max = float(w[-1])
    if not np.isfinite(w_max) or w_max <= 0.0:
        return None, cond
    w = np.maximum(w, w_max / cond_cap)
    return v @ ((v.T @ b) / w), cond


@dataclass
class RegistrationResult:
    pose: np.ndarray
    fitness: float
    rmse: float
    ok: bool


def _translation_icp(rotated_src: np.ndarray, tgt_pts: np.ndarray, tgt_normals: np.ndarray,
                     t0: np.ndarray, max_dist: float, max_iter: int,
                     tol: float = 1e-7,
                     device: str | o3d.core.Device = "CPU:0",
                     cond_cap: float = _COND_CAP) -> tuple[np.ndarray, float, float, bool]:
    """Iterated closest-point, translation-only, point-to-plane. `rotated_src`
    is the source cloud with the (held-fixed) prior rotation already applied.
    Returns (t, fitness, rmse, singular) -- fitness/rmse mirror Open3D's ICP
    result semantics (fitness = matched_fraction, rmse = RMS of the
    point-to-plane residual among matches); singular=True means the normal
    equations were unsolvable (non-finite, or rank zero) and the caller should
    treat this as a failed registration.

    Merely ill-conditioned geometry is NOT a failure here (BUG-068): the step is
    bounded by `cond_cap` instead of rejected, because a rejection is terminal --
    it freezes the pose and nothing relocalizes (BUG-036)."""
    dev = _resolve_device(device)
    n_source = rotated_src.shape[0]
    tgt_t = o3d.core.Tensor(tgt_pts, device=dev)
    nns = o3d.core.nns.NearestNeighborSearch(tgt_t)
    # Pass the search radius at index-build time: the GPU HybridIndex REQUIRES
    # it ("radius is required for GPU HybridIndex"), while the CPU index treats
    # it as optional -- so passing max_dist here works on both devices (on CPU
    # it was previously omitted, which is equivalent).
    nns.hybrid_index(max_dist)

    t = np.asarray(t0, dtype=np.float64).copy()
    fitness, rmse = 0.0, float("inf")
    for _ in range(max_iter):
        query = rotated_src + t
        idx, _dist2, counts = nns.hybrid_search(
            o3d.core.Tensor(query, device=dev), max_dist, 1)
        matched = counts.cpu().numpy().reshape(-1) > 0
        n_valid = int(matched.sum())
        if n_valid == 0:
            return t, 0.0, float("inf"), False
        rows = idx.cpu().numpy().reshape(-1)[matched]
        q = tgt_pts[rows]
        n = tgt_normals[rows]
        p = query[matched]
        r = np.einsum("ij,ij->i", n, p - q)             # point-to-plane residual
        fitness = n_valid / n_source
        rmse = float(np.sqrt(np.mean(r ** 2)))

        a = n.T @ n                                       # 3x3 normal equations
        b = -(n * r[:, None]).sum(axis=0)
        dt, _cond = _solve_translation_step(a, b, cond_cap)
        if dt is None:
            return t, fitness, rmse, True
        t = t + dt
        if np.linalg.norm(dt) < tol:
            break
    return t, fitness, rmse, False


def register(source: o3d.t.geometry.PointCloud, target: o3d.t.geometry.PointCloud,
             init_pose: np.ndarray, mode: str = "translation", max_dist: float = 0.05,
             min_fitness: float = 0.3, max_rmse: float = 0.05,
             max_iter: int = 6, device: str | o3d.core.Device = "CPU:0",
             cond_cap: float = _COND_CAP) -> RegistrationResult:
    # max_iter=6 (Task 9.5): chosen for ACCURACY, not speed. Swept against the
    # real capture (docs/phase6-slam-validation.md "Post-optimization"): trajectory
    # drift is MONOTONICALLY WORSE with more iterations --
    #   iter=6 gap=1.095 m,  iter=12 gap=1.401,  iter=20 gap=1.872,  iter=30 gap=2.072
    # (all with 0/3184 tracking-lost). Over-iterating the translation-only solve
    # overfits each frame's noisy point-to-plane residual on the 54x42 ToF data and
    # accumulates drift, so fewer iterations track the true motion better. iter=6 is
    # the drift minimum; it also happens to sit near the ~35 ms/frame live-preview
    # target, but accuracy -- not that budget -- is why it's the default.
    if mode not in ("translation", "6dof"):
        raise ValueError(f"unknown mode {mode!r}")
    init_pose = np.asarray(init_pose, dtype=np.float64)

    if mode == "translation":
        R = init_pose[:3, :3]
        src_pts = source.point.positions.cpu().numpy().astype(np.float64, copy=False)
        tgt_pts = target.point.positions.cpu().numpy().astype(np.float64, copy=False)
        tgt_normals = target.point.normals.cpu().numpy().astype(np.float64, copy=False)
        rotated_src = (R @ src_pts.T).T
        t, fitness, rmse, singular = _translation_icp(
            rotated_src, tgt_pts, tgt_normals, init_pose[:3, 3], max_dist, max_iter,
            device=device, cond_cap=cond_cap)
        if singular:
            return RegistrationResult(pose=init_pose.copy(), fitness=0.0,
                                      rmse=float("inf"), ok=False)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        ok = bool(fitness >= min_fitness and rmse <= max_rmse)
        return RegistrationResult(pose=T, fitness=float(fitness), rmse=float(rmse), ok=ok)

    init = o3d.core.Tensor(init_pose, device=_resolve_device(device))
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
    T = result.transformation.cpu().numpy().copy()
    ok = bool(result.fitness >= min_fitness and result.inlier_rmse <= max_rmse)
    return RegistrationResult(pose=T, fitness=float(result.fitness),
                              rmse=float(result.inlier_rmse), ok=ok)


def register_escalating(source, target, init_pose, retry_dist: float = 0.0,
                        **kw) -> tuple[RegistrationResult, bool]:
    """`register`, retried once at a wider correspondence radius if the first
    attempt fails its gate. Returns (result, escalated).

    A single fixed `max_dist` has to be both accurate and robust, and it cannot
    be. The tight default (0.05) is the accuracy optimum -- measured on
    captures/coffeeRoomCircuitNoMnt.bin, widening it to 0.10 throughout degrades
    that run's loop closure from 0.150 m to 0.953 m. But a frame whose
    frame-to-model residual exceeds 0.05 finds ZERO correspondences, and because
    `predict_pose` freezes translation at t_prev on a lost frame and nothing
    relocalizes, one such frame kills the rest of the scan: on
    captures/coffeeRoomCircuitMnt.bin that cost 423 frames (22% of the capture)
    from a single failure at frame 1466, silently.

    Escalating only on failure gets both. Measured on those two captures:
    Mnt 423 lost -> 0 (drift 7.48% -> 4.80%), NoMnt bit-identical (0 escalations,
    still 0.150 m / 0.46%). Cost was ONE retry in 1889 frames, +0.5 ms p50. A
    third rung at 0.20 never fired, so one retry is all this implements.

    `retry_dist <= 0` disables the retry, restoring the single-attempt behavior
    exactly (no extra `register` call is made).
    """
    res = register(source, target, init_pose, **kw)
    if res.ok or retry_dist <= 0.0:
        return res, False
    kw = dict(kw)
    kw["max_dist"] = retry_dist
    return register(source, target, init_pose, **kw), True
