"""Point-to-plane ICP, frame-to-model. Four modes (docs spec 3.6):
- 'soft_prior' (BUG-067 fix): full 6-DoF point-to-plane, but rotation held NEAR the
  SFLP prior by a soft Tikhonov prior instead of frozen at it. This is the honest
  bridge between 'translation' (rotation frozen -> a rotation-prior error has no
  rotational DoF and is fabricated as translation, then latched into the TSDF) and
  '6dof' (rotation free -> noisier than the SFLP IMU on this 54x42 ToF, drifts far
  worse). `rot_prior_weight` (dimensionless, x the rotation block's own stiffness)
  sets how tightly rotation is pinned: -> inf reproduces 'translation' exactly,
  0 is undamped 6-DoF. Translation still gets BUG-068's conditioning cap, reused
  verbatim via the Schur complement. See `_soft_prior_icp`/`_solve_soft_prior_step`.
- 'translation' (CURRENT DEFAULT): rotation held at the SFLP prior (init_pose rotation);
  a genuine 3-DoF point-to-plane translation solve (Task 9.5 Lever 2) -- NOT the
  full 6-DoF ICP with the rotation discarded afterward. Cheaper (3x3 normal
  equations vs 6x6 per iteration) and geometrically honest: gate stats
  (fitness/rmse) reflect the actual translation-only alignment, not a 6-DoF fit
  that gets partially thrown away.
- '6dof': full point-to-plane, init_pose as the initial guess (Open3D's tensor
  ICP, unchanged). Disqualified on accuracy for live use (CUDA-ICP study: 8.0 m
  closure vs 0.67) -- kept for study/benchmarking.
- 'adaptive' (EXPERIMENTAL, opt-in, MEASURED WORSE -- do not default to it):
  LiDAR-primary/IMU-gated -- try 6dof, accept its rotation only when strongly
  confirmed AND rotationally observable, else fall back to the translation solve.
  The intent (LiDAR overrides a bad IMU rotation prior) does NOT pay off on this
  54x42 ToF: frame-to-model 6dof rotation is noisier than the SFLP IMU quaternion,
  so on real captures it accepts LiDAR ~98% of frames and drifts far worse than
  translation (17 m vs 0.63 m on captures/imuTranslationError.bin, a tripod whose
  true translation is ~0). It stays because it is the honest realisation of the
  LiDAR-primary idea and would earn its place on hardware whose ICP rotation beats
  its IMU, or a scene/IMU-failure that no capture in hand exercises. See
  docs/superpowers/plans (2026-08-05 crazySLAM) for the measurement.
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

# Soft-prior rotational damping weight (BUG-067), a DIMENSIONLESS multiple of the
# rotation block's own mean stiffness -- see `_solve_soft_prior_step`. Only read
# by mode="soft_prior". Provisional pending the matched-ensemble sweep over the
# three-capture truth set; pinned to config.SlamConfig.icp_rot_prior_weight by
# test_mapper_kwargs_defaults_match_mapper_signature.
_ROT_PRIOR_WEIGHT = 10.0


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


def _rotation_angle_deg(R: np.ndarray) -> float:
    """Geodesic magnitude of a rotation matrix, in degrees. Used by the adaptive
    mode to measure how far the 6dof ICP solve wants to move OFF the IMU prior --
    because ICP is initialised at identity in the local T_pred frame, the residual
    rotation IS the ICP-vs-IMU disagreement."""
    c = (float(np.trace(R[:3, :3])) - 1.0) / 2.0
    return float(np.degrees(np.arccos(max(-1.0, min(1.0, c)))))


def _so3_exp(omega: np.ndarray) -> np.ndarray:
    """Rotation matrix from a rotation vector (Rodrigues)."""
    theta = float(np.linalg.norm(omega))
    if theta < 1e-12:
        return np.eye(3)
    k = np.asarray(omega, dtype=np.float64) / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(theta) * K + (1.0 - np.cos(theta)) * (K @ K)


def _so3_log(R: np.ndarray) -> np.ndarray:
    """Rotation vector (axis*angle) of a rotation matrix -- the inverse of
    `_so3_exp`. Used by the soft-prior solve to measure the CURRENT deviation of
    the working rotation from the IMU prior, so the prior term pulls the total
    rotation back toward the prior rather than merely damping each step."""
    c = (float(np.trace(R)) - 1.0) / 2.0
    c = max(-1.0, min(1.0, c))
    theta = float(np.arccos(c))
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
                    dtype=np.float64)
    if theta < 1e-9:
        return 0.5 * axis                       # near identity: sin(theta) ~= theta
    return (theta / (2.0 * np.sin(theta))) * axis


def _solve_soft_prior_step(A: np.ndarray, b: np.ndarray, phi: np.ndarray,
                           rot_prior_weight: float,
                           cond_cap: float = _COND_CAP
                           ) -> tuple[np.ndarray | None, float]:
    """Solve one iteration of the anisotropically-damped 6-DoF point-to-plane
    normal equations for the increment ``xi = [omega; dt]`` (world-frame rotation
    then translation), returning (xi, translation_cond) or (None, cond) when the
    translation marginal is genuinely singular.

    `A` (6x6, symmetric PSD) and `b` (6) are the raw point-to-plane normal
    equations ``A xi = b`` with rows ordered rotation-block first. `phi` is the
    working rotation's current geodesic deviation from the IMU prior
    (`_so3_log(R @ R0^T)`), so the soft prior penalises the TOTAL rotation off the
    prior, not just this step.

    Two couplings make this the honest bridge between the two shipped modes rather
    than a third thing:

    * **Soft IMU prior on rotation.** A Tikhonov term ``lambda_r * ||omega_total||^2``
      adds ``lambda_r * I`` to the rotational block and ``-lambda_r * phi`` to its
      rhs. `lambda_r` is `rot_prior_weight` scaled by the rotational block's own
      mean stiffness (``trace(A_rr)/3``), so the knob is a dimensionless "how many
      times the geometry's own rotational constraint", robust across scenes/ranges
      rather than an absolute value that means something different every frame.
      As `rot_prior_weight -> inf` the rotation is pinned to the prior and this
      collapses onto the translation-only solve; at 0 it is undamped 6-DoF.

    * **BUG-068 cap on translation, reused verbatim.** Rotation is marginalised out
      by its Schur complement, leaving a 3x3 translation system that is handed to
      `_solve_translation_step` UNCHANGED -- so the eigenvalue-floor cap that bounds
      in-plane slides applies to translation here with identical semantics, and the
      translation block equals ``sum n n^T`` exactly (as in `_translation_icp`) in
      the large-weight limit.

    `rot_prior_weight <= 0` is not this mode's regime (that is `6dof`); callers use
    a positive weight. A non-finite/degenerate rotational block forces ``omega=0``
    (pure translation for that frame) rather than failing."""
    Arr = A[:3, :3]
    Art = A[:3, 3:]
    Att = A[3:, 3:]
    br = b[:3]
    bt = b[3:]
    scale = float(np.trace(Arr)) / 3.0
    lam = rot_prior_weight * scale if scale > 0.0 else 0.0
    Arr_d = Arr + lam * np.eye(3)
    br_d = br - lam * np.asarray(phi, dtype=np.float64)
    try:
        Arr_inv = np.linalg.inv(Arr_d)
    except np.linalg.LinAlgError:
        # Rotational block unobservable and unregularised: drop the rotational DoF
        # for this frame and solve translation alone (equivalent to lambda_r -> inf).
        dt, cond = _solve_translation_step(Att, bt, cond_cap)
        if dt is None:
            return None, cond
        xi = np.zeros(6)
        xi[3:] = dt
        return xi, cond
    # Translation Schur complement: S dt = c, rotation eliminated.
    S = Att - Art.T @ Arr_inv @ Art
    c = bt - Art.T @ (Arr_inv @ br_d)
    dt, cond = _solve_translation_step(S, c, cond_cap)
    if dt is None:
        return None, cond
    omega = Arr_inv @ (br_d - Art @ dt)
    xi = np.empty(6)
    xi[:3] = omega
    xi[3:] = dt
    return xi, cond


def rotation_observability_cond(points: np.ndarray, normals: np.ndarray) -> float:
    """Condition number of the point-to-plane ROTATION Hessian
    ``H = sum_i (p_i x n_i)(p_i x n_i)^T`` (3x3), the quantity that says whether a
    surface can constrain a full 3-DoF rotation at all.

    This is the signal fitness/rmse cannot provide. A flat wall filling the FOV has
    every normal parallel, so the cross products span only the 2 directions
    perpendicular to that normal -- H is rank-2 and rotation ABOUT the wall normal
    is unobservable, yet the plane still fits with fitness ~1.0 and near-zero rmse.
    Trusting the 6dof rotation there accumulates garbage (an adaptive ensemble that
    gated on fitness alone drifted 17.9 m on a tripod-vs-flat-wall capture whose true
    motion is ~0). A corner or a cluttered room gives normals along all three axes,
    H full rank, cond moderate -- there the LiDAR rotation is worth trusting.

    Returns +inf on a degenerate/empty set (treated as unobservable)."""
    if points.shape[0] < 3:
        return float("inf")
    cross = np.cross(points, normals)
    H = cross.T @ cross
    try:
        c = float(np.linalg.cond(H))
    except np.linalg.LinAlgError:
        return float("inf")
    return c if np.isfinite(c) else float("inf")


@dataclass
class RegistrationResult:
    pose: np.ndarray
    fitness: float
    rmse: float
    ok: bool
    # Which solver produced this result: "translation" (rotation locked to the
    # IMU prior), "6dof" (full ICP), or for adaptive mode "lidar" (6dof accepted,
    # LiDAR overrode the prior) / "imu" (6dof rejected, fell back to the locked
    # prior). Lets the Mapper count how often each source won, for tuning. New
    # field WITH a default so every existing positional constructor is unaffected.
    source: str = "translation"


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


def _soft_prior_icp(src_pts: np.ndarray, tgt_pts: np.ndarray, tgt_normals: np.ndarray,
                    R0: np.ndarray, t0: np.ndarray, rot_prior_weight: float,
                    max_dist: float, max_iter: int,
                    tol: float = 1e-7,
                    device: str | o3d.core.Device = "CPU:0",
                    cond_cap: float = _COND_CAP
                    ) -> tuple[np.ndarray, np.ndarray, float, float, bool]:
    """Iterated closest-point, full 6-DoF point-to-plane, with the rotation held
    near the IMU prior `R0` by a soft Tikhonov prior (see `_solve_soft_prior_step`).
    `src_pts` is the RAW source cloud (unrotated); `R0`/`t0` are the prior pose.
    Returns (R, t, fitness, rmse, singular).

    This is BUG-067's fix: `_translation_icp` freezes rotation at `R0` and can only
    reduce a residual by translating, so a rotation-prior error becomes fabricated
    translation and is latched into the TSDF. Here a rotation-prior error has a
    (softly bounded) rotational degree of freedom to land in instead. The prior
    weight keeps rotation IMU-dominated -- undamped 6-DoF rotation is noisier than
    the SFLP quaternion on this 54x42 ToF and drifts far worse (see the module
    docstring / CUDA-ICP study) -- while letting geometry nudge it where the scene
    strongly constrains rotation."""
    dev = _resolve_device(device)
    n_source = src_pts.shape[0]
    tgt_t = o3d.core.Tensor(tgt_pts, device=dev)
    nns = o3d.core.nns.NearestNeighborSearch(tgt_t)
    nns.hybrid_index(max_dist)

    R = np.asarray(R0, dtype=np.float64).copy()
    t = np.asarray(t0, dtype=np.float64).copy()
    fitness, rmse = 0.0, float("inf")
    for _ in range(max_iter):
        query = (R @ src_pts.T).T + t
        idx, _dist2, counts = nns.hybrid_search(
            o3d.core.Tensor(query, device=dev), max_dist, 1)
        matched = counts.cpu().numpy().reshape(-1) > 0
        n_valid = int(matched.sum())
        if n_valid == 0:
            return R, t, 0.0, float("inf"), False
        rows = idx.cpu().numpy().reshape(-1)[matched]
        q = tgt_pts[rows]
        n = tgt_normals[rows]
        p = query[matched]
        r = np.einsum("ij,ij->i", n, p - q)             # point-to-plane residual
        fitness = n_valid / n_source
        rmse = float(np.sqrt(np.mean(r ** 2)))

        # Jacobian rows [ (p x n)^T , n^T ] for the world-frame increment
        # xi = [omega; dt] applied about the origin: p -> exp([omega]x) p + dt.
        cxn = np.cross(p, n)
        J = np.concatenate([cxn, n], axis=1)            # (m, 6)
        A = J.T @ J                                     # 6x6 normal equations
        b = -(J * r[:, None]).sum(axis=0)
        phi = _so3_log(R @ R0.T)                        # current deviation from the prior
        xi, _cond = _solve_soft_prior_step(A, b, phi, rot_prior_weight, cond_cap)
        if xi is None:
            return R, t, fitness, rmse, True
        omega, dt = xi[:3], xi[3:]
        R_delta = _so3_exp(omega)
        R = R_delta @ R                                 # world-frame left update...
        t = R_delta @ t + dt                            # ...so translation rotates with it
        if np.linalg.norm(xi) < tol:
            break
    return R, t, fitness, rmse, False


def register(source: o3d.t.geometry.PointCloud, target: o3d.t.geometry.PointCloud,
             init_pose: np.ndarray, mode: str = "translation", max_dist: float = 0.05,
             min_fitness: float = 0.3, max_rmse: float = 0.05,
             max_iter: int = 6, device: str | o3d.core.Device = "CPU:0",
             cond_cap: float = _COND_CAP,
             rot_prior_weight: float = _ROT_PRIOR_WEIGHT,
             adapt_min_fitness: float = 0.6, adapt_max_rmse: float = 0.03,
             adapt_max_corr_deg: float = 20.0,
             adapt_rot_cond_cap: float = 100.0) -> RegistrationResult:
    # max_iter=6 (Task 9.5): chosen for ACCURACY, not speed. Swept against the
    # real capture (docs/phase6-slam-validation.md "Post-optimization"): trajectory
    # drift is MONOTONICALLY WORSE with more iterations --
    #   iter=6 gap=1.095 m,  iter=12 gap=1.401,  iter=20 gap=1.872,  iter=30 gap=2.072
    # (all with 0/3184 tracking-lost). Over-iterating the translation-only solve
    # overfits each frame's noisy point-to-plane residual on the 54x42 ToF data and
    # accumulates drift, so fewer iterations track the true motion better. iter=6 is
    # the drift minimum; it also happens to sit near the ~35 ms/frame live-preview
    # target, but accuracy -- not that budget -- is why it's the default.
    if mode not in ("translation", "6dof", "adaptive", "soft_prior"):
        raise ValueError(f"unknown mode {mode!r}")
    init_pose = np.asarray(init_pose, dtype=np.float64)

    if mode == "adaptive":
        # LiDAR-primary, IMU-gated (owner: "the LiDAR is our primary source of
        # truth until confidence or blur causes us to lose faith, in which case
        # the IMU takes over"). Try the full 6dof solve first: because ICP is
        # initialised at identity in the local T_pred frame, its residual rotation
        # is exactly how far the point cloud wants to move OFF the IMU prior.
        # ACCEPT it (LiDAR wins, overriding a wrong prior) only when the fit is
        # strongly confirmed -- a STRICTER bar than the base ok gate, because
        # Open3D's 6dof reports fitness >= min_fitness even on the weak-geometry
        # frames where it diverges (a pure-6dof ensemble died 2/3 of its runs on
        # 2026-08-05-crazySLAM.bin). Otherwise fall back to the robust translation
        # solve with rotation locked to the IMU prior -- the IMU takes over. The
        # per-frame correction ceiling is a divergence backstop on top of fitness.
        res6 = register(source, target, init_pose, mode="6dof", max_dist=max_dist,
                        min_fitness=min_fitness, max_rmse=max_rmse, max_iter=max_iter,
                        device=device, cond_cap=cond_cap)
        corr_deg = _rotation_angle_deg(res6.pose)
        # Rotational OBSERVABILITY is the gate fitness cannot be -- a flat wall
        # fits perfectly yet cannot constrain rotation about its normal. Only
        # trust the 6dof rotation when the target geometry actually constrains it.
        if adapt_rot_cond_cap > 0.0:
            tp = target.point.positions.cpu().numpy().astype(np.float64, copy=False)
            tn = target.point.normals.cpu().numpy().astype(np.float64, copy=False)
            observable = rotation_observability_cond(tp, tn) <= adapt_rot_cond_cap
        else:
            observable = True
        confident = (observable and res6.ok and res6.fitness >= adapt_min_fitness
                     and res6.rmse <= adapt_max_rmse and corr_deg <= adapt_max_corr_deg)
        if confident:
            res6.source = "lidar"
            return res6
        rest = register(source, target, init_pose, mode="translation", max_dist=max_dist,
                        min_fitness=min_fitness, max_rmse=max_rmse, max_iter=max_iter,
                        device=device, cond_cap=cond_cap)
        rest.source = "imu"
        return rest

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

    if mode == "soft_prior":
        R0 = init_pose[:3, :3]
        src_pts = source.point.positions.cpu().numpy().astype(np.float64, copy=False)
        tgt_pts = target.point.positions.cpu().numpy().astype(np.float64, copy=False)
        tgt_normals = target.point.normals.cpu().numpy().astype(np.float64, copy=False)
        R, t, fitness, rmse, singular = _soft_prior_icp(
            src_pts, tgt_pts, tgt_normals, R0, init_pose[:3, 3], rot_prior_weight,
            max_dist, max_iter, device=device, cond_cap=cond_cap)
        if singular:
            return RegistrationResult(pose=init_pose.copy(), fitness=0.0,
                                      rmse=float("inf"), ok=False, source="soft_prior")
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = t
        ok = bool(fitness >= min_fitness and rmse <= max_rmse)
        return RegistrationResult(pose=T, fitness=float(fitness), rmse=float(rmse),
                                  ok=ok, source="soft_prior")

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
                                  rmse=float("inf"), ok=False, source="6dof")
    T = result.transformation.cpu().numpy().copy()
    ok = bool(result.fitness >= min_fitness and result.inlier_rmse <= max_rmse)
    return RegistrationResult(pose=T, fitness=float(result.fitness),
                              rmse=float(result.inlier_rmse), ok=ok, source="6dof")


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
