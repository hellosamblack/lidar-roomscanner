import numpy as np
import open3d as o3d
import pytest

from roomscan.slam.odometry import register, register_escalating, RegistrationResult


def _plane_cloud(n=40, z=1.0, curvature=0.15):
    xs, ys = np.meshgrid(np.linspace(-0.5, 0.5, n), np.linspace(-0.4, 0.4, n))
    pts = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1).astype(np.float32)
    # Curvature is what gives ICP translational grip in x and y. It is also
    # exactly what sets the conditioning of the 3x3 normal equations, and hence
    # whether BUG-068's cap engages: 0.15 -> cond 207, 0.30 -> 52, 0.50 -> 19,
    # 0.80 -> 7.9. The 0.15 default is a WEAKLY observable plane, kept as the
    # default because several tests below depend on that marginality.
    pts[:, 2] += curvature * (pts[:, 0] ** 2 + pts[:, 1] ** 2)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def test_translation_recovered():
    # `curvature=0.8` (cond 7.9) so this pins what its name says -- the solver
    # recovers a known translation -- rather than doubling as a test of
    # BUG-068's conditioning cap. The weakly-observable case (the 0.15 default,
    # cond 207) is covered explicitly by
    # `test_weakly_observable_translation_is_damped_by_the_cap` below, which
    # pins the cap's cost instead of hiding it behind a loosened tolerance.
    target = _plane_cloud(curvature=0.8)
    src_pts = target.point.positions.numpy().copy()
    shift = np.array([0.03, -0.02, 0.04], dtype=np.float32)
    src_pts += shift
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts)
    res = register(source, target, np.eye(4), mode="translation")
    assert res.ok
    # source-to-target moves source back by -shift
    assert np.allclose(res.pose[:3, 3], -shift, atol=0.01)
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-9)   # rotation held


def test_6dof_leaves_rotation_free():
    # Strengthened beyond the brief: identical clouds registered in 6dof mode
    # from init=eye(4) must converge to (near-)identity — this genuinely
    # exercises the "6dof runs unmodified ICP" behavior instead of merely
    # checking isinstance/shape, which would pass even for a stub.
    target = _plane_cloud()
    src = _plane_cloud()
    res = register(src, target, np.eye(4), mode="6dof")
    assert isinstance(res, RegistrationResult)
    assert res.pose.shape == (4, 4)
    assert res.ok
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-2)
    assert np.allclose(res.pose[:3, 3], np.zeros(3), atol=1e-2)


def test_rotation_angle_deg_measures_geodesic_magnitude():
    from roomscan.slam.odometry import _rotation_angle_deg
    assert _rotation_angle_deg(np.eye(3)) == pytest.approx(0.0, abs=1e-9)
    th = np.radians(37.0)
    Rz = np.array([[np.cos(th), -np.sin(th), 0], [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    assert _rotation_angle_deg(Rz) == pytest.approx(37.0, abs=1e-6)


def test_adaptive_accepts_the_6dof_solve_when_confident():
    # Well-constrained corner geometry, a clean fit: the confidence bar is met, so
    # adaptive returns the LiDAR (6dof) solve itself -- the point cloud is trusted
    # to override the IMU prior.
    target = _corner_cloud()
    src = _corner_cloud()
    res = register(src, target, np.eye(4), mode="adaptive")
    assert res.source == "lidar"
    assert res.ok
    ref6 = register(src, target, np.eye(4), mode="6dof")
    assert np.allclose(res.pose, ref6.pose, atol=1e-6)     # it IS the 6dof solve


def test_adaptive_falls_back_to_the_translation_solve_when_not_confident():
    # Force the confidence gate to fail (an unreachable fitness bar): adaptive must
    # fall back to the IMU-locked translation solve, byte-for-byte.
    target = _plane_cloud(curvature=0.8)
    src_pts = target.point.positions.numpy().copy() + np.array([0.03, -0.02, 0.04], np.float32)
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts)
    res = register(source, target, np.eye(4), mode="adaptive", adapt_min_fitness=1.01)
    assert res.source == "imu"
    ref = register(source, target, np.eye(4), mode="translation")
    assert np.allclose(res.pose, ref.pose, atol=1e-9)
    assert np.allclose(res.pose[:3, :3], np.eye(3), atol=1e-9)   # rotation held at prior


def test_adaptive_correction_ceiling_forces_fallback():
    # Even a confident fit is rejected if the per-frame rotation correction exceeds
    # the ceiling (divergence backstop): ceiling 0 => any correction => IMU fallback.
    target = _corner_cloud()
    src = _corner_cloud()
    res = register(src, target, np.eye(4), mode="adaptive", adapt_max_corr_deg=-1.0)
    assert res.source == "imu"


def _corner_cloud(n=15):
    """Inside corner of a unit box: three mutually perpendicular faces
    (z=0, x=0, y=0), giving point normals along all three axes so 6dof
    point-to-plane ICP has enough structure to constrain a full 3-DoF
    rotation (unlike the single near-planar `_plane_cloud`, which suffers
    rotational ambiguity)."""
    lin = np.linspace(0.05, 0.95, n)
    a, b = np.meshgrid(lin, lin)
    a = a.ravel()
    b = b.ravel()
    zeros = np.zeros_like(a)
    floor = np.stack([a, b, zeros], axis=1)    # z=0 face, normal ~ +/-z
    wall_x = np.stack([zeros, a, b], axis=1)   # x=0 face, normal ~ +/-x
    wall_y = np.stack([a, zeros, b], axis=1)   # y=0 face, normal ~ +/-y
    pts = np.concatenate([floor, wall_x, wall_y], axis=0).astype(np.float32)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def _rotation_matrix(axis, angle):
    axis = axis / np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)


def test_mode_switch_is_discriminating():
    # Discriminates the mode branch: source is the corner cloud rotated by a
    # KNOWN, nonzero rotation about a body diagonal (constrained by all three
    # perpendicular faces), with init_pose = eye(4). 6dof ICP must recover a
    # rotation close to the true inverse rotation (not identity); translation
    # mode on the identical inputs must hold the prior's rotation exactly
    # (= identity here), regardless of what ICP itself estimates. A mutation
    # forcing the rotation-override to fire unconditionally (`if True:`)
    # would collapse the 6dof result's rotation to identity too, failing the
    # first assertion below.
    target = _corner_cloud()
    tgt_pts = target.point.positions.numpy()
    centroid = tgt_pts.mean(axis=0)

    axis = np.array([1.0, 1.0, 1.0])
    angle = np.deg2rad(7.0)
    R_true = _rotation_matrix(axis, angle)

    src_pts = (R_true @ (tgt_pts - centroid).T).T + centroid
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts.astype(np.float32))

    res_6dof = register(source, target, np.eye(4), mode="6dof", max_dist=0.2)
    assert res_6dof.ok
    # ICP aligns source (= R_true @ target, about the shared centroid) back
    # onto target, so the recovered rotation should be R_true's inverse
    # (== transpose, since it's a rotation) -- and clearly not identity.
    assert np.allclose(res_6dof.pose[:3, :3], R_true.T, atol=0.05)
    assert not np.allclose(res_6dof.pose[:3, :3], np.eye(3), atol=0.05)

    res_translation = register(source, target, np.eye(4), mode="translation", max_dist=0.2)
    assert res_translation.ok
    assert np.allclose(res_translation.pose[:3, :3], np.eye(3), atol=1e-9)


def test_low_overlap_trips_gate():
    target = _plane_cloud()
    far = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    far.point.positions = o3d.core.Tensor((target.point.positions.numpy() + 5.0).astype(np.float32))
    res = register(far, target, np.eye(4), mode="translation", max_dist=0.05)
    assert not res.ok


def _flat_plane_cloud(n=40, z=1.0):
    """Perfectly flat, coplanar grid (no curvature, unlike _plane_cloud) --
    all normals identical. Point-to-plane ICP's 6x6 normal-equations solve is
    singular on this geometry (e.g. a ToF sensor square to a blank wall)."""
    xs, ys = np.meshgrid(np.linspace(-0.5, 0.5, n), np.linspace(-0.4, 0.4, n))
    pts = np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], axis=1).astype(np.float32)
    pc = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    pc.point.positions = o3d.core.Tensor(pts)
    pc.estimate_normals()
    return pc


def test_singular_geometry_returns_not_ok():
    # Open3D's point-to-plane ICP raises RuntimeError("... Singular 6x6 linear
    # system detected, tracking failed.") on a perfectly flat, texture-poor
    # target. register() must degrade to a not-ok result instead of letting
    # the exception propagate and crash the mapper.
    target = _flat_plane_cloud()
    source = _flat_plane_cloud()
    res = register(source, target, np.eye(4), mode="translation")
    assert isinstance(res, RegistrationResult)
    assert not res.ok
    assert res.fitness == 0.0
    assert res.rmse == float("inf")
    assert np.allclose(res.pose, np.eye(4))


def test_translation_gate_reflects_genuine_translation_fit_not_stale_6dof():
    # Task 9.5 Lever 2 regression guard: translation mode must gate on the
    # ACTUAL translation-only alignment quality, not a full 6-DoF ICP result
    # whose rotation is discarded afterward. A large (40 deg) rotation about
    # the corner's body diagonal is a case where 6-DoF ICP still converges
    # perfectly (rotation absorbs all the error, fitness=1.0) but a genuine
    # translation-only fit cannot correct a rotation and should fail the
    # default gate (min_fitness=0.3). The old "run 6dof, keep its fitness,
    # override rotation after" implementation would have reported this
    # ok=True (reusing the 6dof fit's fitness=1.0) even though the returned
    # translation-only pose does not actually align the clouds well -- a
    # silently corrupt integration. Confirmed empirically before writing this
    # test (see profiling notes): 6dof fitness=1.0 vs translation fitness
    # ~0.28 at this angle.
    target = _corner_cloud()
    tgt_pts = target.point.positions.numpy()
    centroid = tgt_pts.mean(axis=0)
    axis = np.array([1.0, 1.0, 1.0])
    angle = np.deg2rad(40.0)
    R_true = _rotation_matrix(axis, angle)
    src_pts = (R_true @ (tgt_pts - centroid).T).T + centroid
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts.astype(np.float32))

    res_6dof = register(source, target, np.eye(4), mode="6dof", max_dist=0.2)
    assert res_6dof.ok
    assert res_6dof.fitness > 0.9   # 6dof genuinely converges on this case

    res_translation = register(source, target, np.eye(4), mode="translation")
    assert not res_translation.ok  # genuine translation-only fit correctly rejects it
    assert np.allclose(res_translation.pose[:3, :3], np.eye(3), atol=1e-9)


# --- register_escalating: retry a failed gate at a wider correspondence radius


def _far_pair(shift_m, axis=0):
    """A source displaced from the target by `shift_m`, so the residual is
    tunable relative to the correspondence radius. `axis=0` displaces IN-PLANE
    (a weakly observable direction); `axis=2` along the plane normal (a genuine,
    fully observable point-to-plane residual)."""
    target = _plane_cloud()
    src_pts = target.point.positions.numpy().copy()
    src_pts[:, axis] += shift_m
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(src_pts)
    return source, target


def test_escalating_no_retry_when_first_attempt_succeeds():
    source, target = _far_pair(0.02)
    res, escalated = register_escalating(source, target, np.eye(4),
                                         retry_dist=0.10, mode="translation",
                                         max_dist=0.05)
    assert res.ok
    assert escalated is False


def test_escalating_disabled_is_identical_to_register():
    """retry_dist<=0 must reproduce single-attempt behavior exactly, so the
    feature is genuinely opt-out and cannot perturb a healthy run."""
    source, target = _far_pair(0.30)
    plain = register(source, target, np.eye(4), mode="translation", max_dist=0.05)
    res, escalated = register_escalating(source, target, np.eye(4), retry_dist=0.0,
                                         mode="translation", max_dist=0.05)
    assert escalated is False
    assert res.ok == plain.ok
    assert res.fitness == pytest.approx(plain.fitness)
    assert np.allclose(res.pose, plain.pose)


def test_escalating_rescues_a_frame_the_tight_radius_loses():
    """The whole point: a displacement that finds no correspondences at 0.05
    must still register once retried wider. This is the single-frame failure
    that killed 423 frames of captures/coffeeRoomCircuitMnt.bin."""
    # Displaced along the plane NORMAL, not in-plane. An in-plane displacement
    # used to fail the tight radius only because the unbounded solve DIVERGED
    # past the target (-0.76 for a 0.70 shift) and then found zero
    # correspondences; BUG-068's cap prevents that divergence, so that fixture
    # no longer exercises escalation at all. A normal-direction displacement is
    # a genuine point-to-plane residual: it exceeds 0.05 outright, at any cap.
    source, target = _far_pair(0.10, axis=2)
    tight = register(source, target, np.eye(4), mode="translation", max_dist=0.05)
    assert not tight.ok, "fixture no longer exercises a tight-radius failure"
    assert tight.fitness == 0.0, "the failure must be zero correspondences"

    res, escalated = register_escalating(source, target, np.eye(4), retry_dist=0.20,
                                         mode="translation", max_dist=0.05)
    assert escalated is True
    assert res.ok
    assert res.pose[2, 3] == pytest.approx(-0.10, abs=0.02)


def test_escalating_reports_escalation_even_when_retry_also_fails():
    """A retry that also fails is still an escalation -- the counter measures
    how often the tight radius was insufficient, not how often it was saved."""
    source, target = _far_pair(5.0)
    res, escalated = register_escalating(source, target, np.eye(4), retry_dist=0.06,
                                         mode="translation", max_dist=0.05)
    assert escalated is True
    assert not res.ok


# ---- BUG-068: conditioning cap on the translation normal equations -----------

def _normals_with_cond(cond_target: float, n: int = 600) -> np.ndarray:
    """Unit normals whose `N.T @ N` has roughly the requested condition number:
    a dominant +Z cluster with a controllable in-plane spread."""
    rng = np.random.default_rng(7)
    spread = float(np.sqrt(1.0 / max(cond_target, 1.0 + 1e-9)))
    v = np.tile(np.array([0.0, 0.0, 1.0]), (n, 1)) + spread * rng.normal(size=(n, 3))
    return v / np.linalg.norm(v, axis=1, keepdims=True)


def test_cond_cap_is_a_no_op_below_the_cap():
    """Well-conditioned frames must be BIT-identical to the pre-BUG-068 solver --
    the fix targets a tail, and a fix that perturbs every frame is a new
    variable in every drift measurement the repo has already taken."""
    from roomscan.slam.odometry import _solve_translation_step

    n = _normals_with_cond(2.0)
    a, b = n.T @ n, np.array([0.003, -0.007, 0.011])
    assert np.linalg.cond(a) < 20.0, "fixture must sit below the shipped cap"
    dt, cond = _solve_translation_step(a, b, cond_cap=20.0)
    assert np.array_equal(dt, np.linalg.solve(a, b))
    assert cond == pytest.approx(np.linalg.cond(a))


def test_cond_cap_bounds_the_weak_axis_and_leaves_the_observable_one_exact():
    """Near-planar geometry: the step along the unobservable in-plane directions
    is suppressed, while the component along the well-observed normal direction
    is untouched. This is what stops the 1.2 m/3 s slide of BUG-068 without
    rejecting the frame (rejection is terminal -- BUG-036)."""
    from roomscan.slam.odometry import _solve_translation_step

    n = _normals_with_cond(5000.0)
    a, b = n.T @ n, np.array([0.05, -0.04, 0.02])
    assert np.linalg.cond(a) > 20.0, "fixture must sit above the shipped cap"

    uncapped = np.linalg.solve(a, b)
    capped, _ = _solve_translation_step(a, b, cond_cap=20.0)
    assert np.linalg.norm(capped) < 0.25 * np.linalg.norm(uncapped)

    strong = np.linalg.eigh(a)[1][:, -1]        # best-observed direction
    assert float(capped @ strong) == pytest.approx(float(uncapped @ strong), rel=1e-9)


def test_cond_cap_disabled_restores_the_unbounded_solve():
    """`icp_cond_cap = 0` must reproduce the pre-fix behaviour exactly, so the
    change can be bisected against and A/B'd without editing code."""
    from roomscan.slam.odometry import _solve_translation_step

    n = _normals_with_cond(5000.0)
    a, b = n.T @ n, np.array([0.05, -0.04, 0.02])
    dt, _ = _solve_translation_step(a, b, cond_cap=0.0)
    assert np.array_equal(dt, np.linalg.solve(a, b))


def test_cond_cap_still_reports_a_genuinely_singular_system():
    """Capping replaces the ill-conditioned REJECTION, not the unsolvable one:
    a rank-zero / non-finite system must still come back as a failed
    registration rather than a silent zero step."""
    from roomscan.slam.odometry import _solve_translation_step

    b = np.array([1.0, 2.0, 3.0])
    assert _solve_translation_step(np.zeros((3, 3)), b, cond_cap=20.0)[0] is None
    assert _solve_translation_step(np.full((3, 3), np.nan), b, cond_cap=20.0)[0] is None


def test_ill_conditioned_frame_is_bounded_not_rejected():
    """End-to-end through `register`: a near-planar target must still return an
    accepted pose. The old code's only response to bad conditioning was to fail
    the frame, and a failed frame freezes the pose with no relocalization."""
    target = _flat_plane_cloud()
    source = _flat_plane_cloud()
    source.point.positions = source.point.positions + o3d.core.Tensor(
        [[0.0, 0.0, 0.004]], dtype=o3d.core.float32)
    res = register(source, target, np.eye(4), mode="translation", max_dist=0.05)
    assert isinstance(res, RegistrationResult)
    # Bounded: the in-plane slide the cap exists to stop must stay small.
    assert abs(res.pose[0, 3]) < 0.05 and abs(res.pose[1, 3]) < 0.05


def test_weakly_observable_translation_is_damped_by_the_cap():
    """BUG-068's cost, pinned with numbers rather than hidden.

    On a weakly observable plane (the 0.15-curvature fixture, cond 207) the cap
    under-recovers genuine in-plane motion, because from a single frame a real
    in-plane displacement and an ICP slide are the SAME measurement. The trade is
    taken deliberately and it is asymmetric: an under-recovery is self-correcting
    (the next frame re-aligns against the map absolutely, frame-to-model), while
    an over-recovery is integrated into the TSDF and is permanent.

    The normal direction -- the one the geometry actually observes -- is exact at
    every cap. Anyone retuning `_COND_CAP` should see this number move.
    """
    target = _plane_cloud()                      # curvature 0.15 -> cond 207
    shift = np.array([0.03, -0.02, 0.04], dtype=np.float32)
    source = o3d.t.geometry.PointCloud(o3d.core.Device("CPU:0"))
    source.point.positions = o3d.core.Tensor(target.point.positions.numpy() + shift)

    uncapped = register(source, target, np.eye(4), mode="translation", cond_cap=0.0)
    assert np.allclose(uncapped.pose[:3, 3], -shift, atol=0.01)

    capped = register(source, target, np.eye(4), mode="translation")   # shipped default
    assert capped.ok
    assert capped.pose[2, 3] == pytest.approx(-0.04, abs=1e-3)   # observable: exact
    # In-plane: recovered but damped, ~60% of truth at cond 207 / cap 20.
    assert -0.024 < capped.pose[0, 3] < -0.014
    assert 0.005 < capped.pose[1, 3] < 0.015


def test_cap_prevents_the_divergence_that_used_to_present_as_a_lost_frame():
    """The cap is not purely a cost. An in-plane displacement large enough to
    make the unbounded solve overshoot the target (-0.76 for a 0.70 m shift)
    found zero correspondences afterwards and was reported tracking-lost --
    which freezes the pose with no relocalization. Bounding the step keeps the
    frame registrable."""
    source, target = _far_pair(0.70)
    assert not register(source, target, np.eye(4), mode="translation",
                        max_dist=0.05, cond_cap=0.0).ok
    assert register(source, target, np.eye(4), mode="translation",
                    max_dist=0.05, cond_cap=200.0).pose[0, 3] == pytest.approx(-0.70, abs=0.01)
