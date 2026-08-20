import math

import numpy as np
import open3d as o3d
import pytest
from roomscan.slam.intrinsics import pinhole
from roomscan.slam.mapper import Mapper, FrameStep

W, H = 54, 42

def _wall(z_m=1.0):
    return np.full((H, W), z_m * 1000.0, dtype=np.float32)

def test_first_frame_bootstraps_and_integrates():
    m = Mapper(W, H, voxel_size=0.02)
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert isinstance(step, FrameStep)
    assert not step.tracking_lost
    assert len(m.trajectory) == 1
    assert np.allclose(m.trajectory[0][:3, 3], [0, 0, 0], atol=1e-6)
    # map_point_cloud() (Open3D's extract_point_cloud()) is a known-quirky signal on
    # this synthetic axis-aligned wall -- it returns 0 points even when the map is
    # genuinely populated (see tsdf.py docstring / .superpowers/sdd/task-4-report.md,
    # reproduced by two independent agents). raycast() at the integrated pose is the
    # reliable "map grew" signal on this geometry (task-4 confirmed ~1900 pts after a
    # single wall integration at this same voxel_size), so we use it here instead.
    K = pinhole(W, H)
    model = m._tsdf.raycast(K, np.linalg.inv(m.trajectory[0]), W, H)
    assert model is not None
    assert model.point.positions.numpy().shape[0] > 100

def test_prior_smoothing_off_is_the_default_and_a_noop():
    m = Mapper(W, H, voxel_size=0.02)
    assert m._prior_smooth_alpha == 0.0
    assert m._quat_smooth is None
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert m._quat_smooth is None                  # never touched when off


def test_prior_smoothing_lags_the_prior_toward_history():
    """With alpha>0 the prior fed to the pose is a slerp EMA: after a step change
    in orientation, the smoothed prior sits BETWEEN the old and new quats, not on
    the new one. Seeds on the first sample so nothing is smoothed at startup."""
    m = Mapper(W, H, voxel_size=0.02, prior_smooth_alpha=0.5)
    q0 = (1.0, 0.0, 0.0, 0.0)
    q1 = (float(np.cos(np.radians(20.0))), 0.0, 0.0, float(np.sin(np.radians(20.0))))
    assert m._smooth_prior(q0) == q0               # seed: first sample passes through
    out = np.asarray(m._smooth_prior(q1))
    # halfway-ish between q0 (0deg) and q1 (40deg quat = 20deg rot): w between them
    assert q1[0] < out[0] < q0[0]                  # lagged toward history
    assert 0.0 < out[3] < q1[3]


def test_tracking_lost_holds_pose_and_skips_integrate():
    m = Mapper(W, H, voxel_size=0.02)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    K = pinhole(W, H)
    model_before = m._tsdf.raycast(K, np.linalg.inv(m.trajectory[0]), W, H)
    n_before = model_before.point.positions.numpy().shape[0]
    # an all-invalid (zero) depth frame => degenerate => lost, no integrate
    lost = m.step(np.zeros((H, W), dtype=np.float32), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert lost.tracking_lost
    assert m.tracking_lost_count == 1
    model_after = m._tsdf.raycast(K, np.linalg.inv(m.trajectory[0]), W, H)
    n_after = model_after.point.positions.numpy().shape[0]
    assert n_after == n_before                    # map genuinely unchanged

def _textured_wall(z_m):
    # A perfectly flat, borderless fronto-parallel wall is degenerate for full-DOF
    # point-to-plane ICP (near-constant normals => singular 6x6 normal-equations
    # solve, reproduced directly against Open3D: "gels failed in SolveCPU: singular
    # condition detected"). test_slam_odometry.py's own _plane_cloud hits the same
    # issue and works around it by adding mild curvature "so ICP has translational
    # grip in x and y too" -- same technique applied here. The curvature offset is
    # identical for both frames at a given (row, col), so the per-pixel z-shift
    # between the two frames is still exactly z_m's difference.
    rows = np.linspace(-0.4, 0.4, H)[:, None]
    cols = np.linspace(-0.5, 0.5, W)[None, :]
    curve = 0.1 * (rows ** 2 + cols ** 2)   # metres
    return ((z_m + curve) * 1000.0).astype(np.float32)

def test_recovers_after_first_frame_tracking_lost():
    m = Mapper(W, H, voxel_size=0.02)
    lost = m.step(np.zeros((H, W), dtype=np.float32), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert lost.tracking_lost
    assert len(m.trajectory) == 1
    K = pinhole(W, H)
    model_after_lost = m._tsdf.raycast(K, np.linalg.inv(m.trajectory[0]), W, H)
    assert model_after_lost is None
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert not step.tracking_lost
    model = m._tsdf.raycast(K, np.linalg.inv(step.pose), W, H)
    assert model is not None
    assert model.point.positions.numpy().shape[0] > 100

def test_reflectance_produces_non_black_varied_mesh_colors():
    # Task 13: passing reflectance through step() colors the mesh via the
    # TsdfMap color-integrate path (mirrors test_slam_tsdf.py's direct test,
    # exercised here through the full Mapper.step plumbing). weight_threshold=0
    # so a single integrated frame (weight=1) still extracts vertices.
    m = Mapper(W, H, voxel_size=0.02, weight_threshold=0.0)
    grad = (np.arange(W, dtype=np.float32) / (W - 1))
    reflectance = np.repeat(grad[None, :], H, axis=0) * 100.0   # arbitrary reflectance units
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0, reflectance=reflectance)
    colors = m.mesh().vertex.colors.numpy()
    assert len(colors) > 0
    assert colors.max() > 0.0
    assert (colors.max(axis=0) - colors.min(axis=0)).max() > 0.05


def test_no_reflectance_keeps_mesh_colors_black():
    m = Mapper(W, H, voxel_size=0.02, weight_threshold=0.0)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    colors = m.mesh().vertex.colors.numpy()
    assert len(colors) > 0
    assert np.allclose(colors, 0.0)


def test_low_confidence_gates_depth_and_causes_tracking_loss():
    # All-low-confidence => every pixel invalidated => same as an all-zero
    # depth frame => tracking lost on the (bootstrap) first frame. Confidence
    # semantics: higher = better (verified on the real capture).
    m = Mapper(W, H, voxel_size=0.02, min_confidence=50.0)
    low_confidence = np.full((H, W), 10.0, dtype=np.float32)   # below the 50.0 threshold
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0, confidence=low_confidence)
    assert step.tracking_lost


def test_high_confidence_does_not_gate_and_tracks_normally():
    m = Mapper(W, H, voxel_size=0.02, min_confidence=50.0)
    high_confidence = np.full((H, W), 200.0, dtype=np.float32)   # above the threshold
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0, confidence=high_confidence)
    assert not step.tracking_lost


def test_partial_confidence_gating_reduces_valid_points_without_losing_track():
    # Gate out half the frame (below threshold) and confirm the map still
    # only reflects the ungated half -- i.e. gating genuinely invalidates
    # those depth pixels rather than being a no-op. weight_threshold=0 so a
    # single integrated frame still extracts vertices.
    m_full = Mapper(W, H, voxel_size=0.02, min_confidence=None, weight_threshold=0.0)
    m_full.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    full_mesh_verts = len(m_full.mesh().vertex.positions)

    m_gated = Mapper(W, H, voxel_size=0.02, min_confidence=50.0, weight_threshold=0.0)
    half_confidence = np.full((H, W), 200.0, dtype=np.float32)
    half_confidence[:, : W // 2] = 10.0   # left half below threshold
    m_gated.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0, confidence=half_confidence)
    gated_mesh_verts = len(m_gated.mesh().vertex.positions)

    assert gated_mesh_verts < full_mesh_verts


def test_pose_translation_tracks_a_synthetic_shift():
    # wall moves closer by 5 cm between frames => camera moved +5cm along +z.
    # Quat = 90deg about Y, NOT identity: per docs/coordinate-frames.md's composed
    # mapping (T_WORLD_TO_CV @ R @ T_CV_TO_BODY), the ToF camera's forward axis at
    # the *identity* quat lands on Open3D-world -Y (physical "up"), not world Z --
    # verified against the documented sandwich and the sensors.py matrices. This
    # quat is the one where camera-forward truly aligns with world +Z, so checking
    # pose[2, 3] genuinely exercises the depth-ward ICP translation the test intends
    # (this is a geometry/setup fix, not a loosened assertion -- direction and
    # magnitude are unchanged and still fail if ICP recovers the wrong translation).
    q = (0.70710678, 0.0, 0.70710678, 0.0)
    m = Mapper(W, H, voxel_size=0.02, icp_mode="translation")
    m.step(_textured_wall(1.20), q, 101325.0)
    step = m.step(_textured_wall(1.15), q, 101325.0)
    assert not step.tracking_lost
    assert step.source_points > 0
    assert step.inliers == round(step.fitness * step.source_points)
    # camera translation z should be ~ +0.05 (moved toward the wall)
    assert abs(step.pose[2, 3] - 0.05) < 0.03


def test_stationary_hold_dejitters_report_without_touching_map():
    """Stationarity hold (owner: still device -> still model). Feed a noisy but
    stationary sensor (constant quat, jittery depth) to two mappers -- one with
    the hold, one without. The hold must:
      * de-jitter the REPORTED trajectory (some frames frozen -> held_count>0,
        and late-frame reported steps smaller than gate-off), while
      * leaving the map + tracking prior BYTE-IDENTICAL (accuracy untouched):
        the internal _t_prev and the extracted mesh vertex count match exactly.
    """
    q = (0.70710678, 0.0, 0.70710678, 0.0)
    rng = np.random.default_rng(7)
    base = _textured_wall(1.20)
    # Pre-generate identical noisy frames so both mappers see the same input.
    frames = [(base + rng.normal(0, 3.0, base.shape)).astype(np.float32)  # ~3 mm depth noise
              for _ in range(30)]

    m_off = Mapper(W, H, voxel_size=0.02, icp_mode="translation", stationary_hold=False)
    m_on = Mapper(W, H, voxel_size=0.02, icp_mode="translation", stationary_hold=True)
    off_pos, on_pos = [], []
    for f in frames:
        off_pos.append(m_off.step(f, q, 101325.0).pose[:3, 3].copy())
        on_pos.append(m_on.step(f, q, 101325.0).pose[:3, 3].copy())

    # Map + tracking prior: identical inputs -> identical true ICP pose, so the
    # internal translation baseline and the extracted mesh must match exactly.
    assert np.allclose(m_off._t_prev, m_on._t_prev, atol=0.0)
    assert len(m_off.mesh().vertex.positions) == len(m_on.mesh().vertex.positions)

    # The hold engaged and calmed the reported trajectory.
    assert m_on.held_count > 0
    off_pos, on_pos = np.array(off_pos), np.array(on_pos)
    off_step = np.linalg.norm(np.diff(off_pos[10:], axis=0), axis=1).mean()
    on_step = np.linalg.norm(np.diff(on_pos[10:], axis=0), axis=1).mean()
    assert on_step < off_step


def test_imu_spike_pre_gate_holds_orientation_through_a_glitch():
    """IMU spike pre-gate (2026-08-05-crazySLAM.bin): a physically-impossible
    inter-frame SFLP rotation is a glitch, and in translation mode ICP holds
    rotation at the prior and cannot disagree, so the glitch would come out as
    fabricated translation. The gate must hold the LAST GOOD orientation as the
    prior instead.

    Asserted on the pose ROTATION, which is deterministic here: in translation
    mode the ICP correction (init identity) carries rotation = identity, so the
    final pose rotation is exactly `prior_rotation(quat_prior)` whether or not
    the frame tracks -- no dependence on ICP fitness/lost behaviour.
    """
    from roomscan.slam.frames import prior_rotation

    q = (0.70710678, 0.0, 0.70710678, 0.0)          # the true, steady orientation
    q_spike = (0.0, 0.70710678, 0.0, 0.70710678)    # a glitch: 180 deg from q
    dot = abs(float(np.dot(np.array(q), np.array(q_spike))))
    assert math.degrees(2.0 * math.acos(min(1.0, dot))) > 30.0   # exceeds the ceiling

    def run(spike_deg):
        m = Mapper(W, H, voxel_size=0.02, icp_mode="translation", imu_spike_deg=spike_deg)
        m.step(_textured_wall(1.20), q, 101325.0)               # bootstrap
        m.step(_textured_wall(1.20), q, 101325.0)               # steady baseline
        s = m.step(_textured_wall(1.20), q_spike, 101325.0)     # GLITCH frame
        return m, s

    m_gated, s_gated = run(30.0)
    m_off, s_off = run(0.0)

    # Gate on: caught the glitch and held q; the disabled gate followed the spike.
    assert m_gated.imu_spike_count == 1
    assert m_off.imu_spike_count == 0
    assert np.allclose(s_gated.pose[:3, :3], prior_rotation(q), atol=1e-6)
    assert np.allclose(s_off.pose[:3, :3], prior_rotation(q_spike), atol=1e-6)
    # The held prior keeps the (unmoved) wall aligned, so the gated frame tracks
    # cleanly. (The magnitude of the fabricated translation the raw spike causes
    # is scene-dependent -- a symmetric synthetic wall maps roughly onto itself
    # under a boresight spike -- so the jump REDUCTION is measured on the real
    # capture via slam_ensemble, not asserted on this geometry.)
    assert not s_gated.tracking_lost

    # The gate did NOT advance `_quat_prev` to the glitch: a following good frame
    # sees ~0 rotation and does not re-trigger.
    s_next = m_gated.step(_textured_wall(1.20), q, 101325.0)
    assert m_gated.imu_spike_count == 1
    assert not s_next.tracking_lost


def test_imu_spike_gate_does_not_fire_on_normal_motion():
    # With the gate ENABLED (it ships off; enable explicitly here), a steady
    # in-spec orientation sequence still never trips it. ~5 deg/frame is brisk pan.
    m = Mapper(W, H, voxel_size=0.02, icp_mode="translation", imu_spike_deg=30.0)
    quats = []
    for i in range(6):
        a = math.radians(5.0 * i) / 2.0            # 5 deg/frame about Y, well under 30
        quats.append((math.cos(a), 0.0, math.sin(a), 0.0))
    for q in quats:
        m.step(_textured_wall(1.20), q, 101325.0)
    assert m.imu_spike_count == 0


def test_adaptive_mode_runs_and_classifies_each_tracked_frame():
    # icp_mode="adaptive" must run end to end through the Mapper and classify each
    # tracked (non-bootstrap, non-lost) frame as either LiDAR-accepted or
    # IMU-fallback. Totals must be consistent with the trajectory length.
    q = (0.70710678, 0.0, 0.70710678, 0.0)
    m = Mapper(W, H, voxel_size=0.02, icp_mode="adaptive")
    for _ in range(6):
        m.step(_textured_wall(1.20), q, 98579.0)
    total = m.icp_lidar_count + m.icp_imu_fallback_count
    assert total >= 1
    assert total <= len(m.trajectory)
    # translation-mode counters stay zero (no adaptive classification there)
    m2 = Mapper(W, H, voxel_size=0.02, icp_mode="translation")
    m2.step(_textured_wall(1.20), q, 98579.0)
    m2.step(_textured_wall(1.20), q, 98579.0)
    assert m2.icp_lidar_count == 0 and m2.icp_imu_fallback_count == 0


def test_mapper_accepts_explicit_cpu_device_string():
    # Device-configurability (Phase 6 follow-up): passing device="CPU:0"
    # explicitly must behave identically to the omitted-argument default --
    # CUDA:0 isn't testable without a CUDA-enabled Open3D build, but the
    # plumbing that would carry it (Mapper -> TsdfMap/pinhole/source_cloud/
    # register) is exercised here with the one device we can verify.
    m = Mapper(W, H, voxel_size=0.02, device="CPU:0")
    assert m._device == o3d.core.Device("CPU:0")
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert not step.tracking_lost
    assert m.mesh().vertex.positions.device == o3d.core.Device("CPU:0")


def test_mapper_accepts_device_as_o3d_device_instance():
    # device may also already be an o3d.core.Device (e.g. passed through
    # from a caller that resolved it itself) -- not just a string.
    m = Mapper(W, H, voxel_size=0.02, device=o3d.core.Device("CPU:0"))
    assert m._device == o3d.core.Device("CPU:0")
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert not step.tracking_lost


def test_mapper_forwards_release_cache_every_to_its_tsdf():
    # Sub-phase 6.G: the knob has to reach TsdfMap (where the extraction that
    # dirties the CUDA cache happens) -- a Mapper-only attribute would be
    # silently inert.
    assert Mapper(W, H, voxel_size=0.02)._tsdf.release_cache_every == 1
    assert Mapper(W, H, voxel_size=0.02,
                  release_cache_every=0)._tsdf.release_cache_every == 0
    assert Mapper(W, H, voxel_size=0.02,
                  release_cache_every=10)._tsdf.release_cache_every == 10


def test_mapper_forwards_block_count_to_its_tsdf():
    # BUG-035: the capacity knob has to reach TsdfMap, which owns the grid.
    from roomscan.slam.tsdf import DEFAULT_BLOCK_COUNT
    assert Mapper(W, H, voxel_size=0.02)._tsdf.block_count == DEFAULT_BLOCK_COUNT
    assert Mapper(W, H, voxel_size=0.02,
                  block_count=7777)._tsdf.block_count == 7777


# --- ICP retry plumbing + per-frame lost flags


def test_lost_flags_track_per_frame_and_match_the_count():
    """lost_flags must stay 1:1 with trajectory so metrics.tracking_stats can
    tell a recovered dropout from a run that died."""
    m = Mapper(W, H, voxel_size=0.02)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    m.step(np.zeros((H, W), dtype=np.float32), (1.0, 0.0, 0.0, 0.0), 101325.0)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert len(m.lost_flags) == len(m.trajectory) == 3
    assert m.lost_flags[1] is True                      # empty depth -> lost
    assert sum(m.lost_flags) == m.tracking_lost_count


def test_retry_disabled_makes_no_second_register_call():
    """icp_retry_dist=0 must be a true opt-out: the wider attempt is never
    even made, so a healthy run's behavior is exactly as before the fix."""
    import roomscan.slam.mapper as mod
    calls = []
    real = mod.register_escalating

    def spy(*a, **kw):
        calls.append(kw.get("retry_dist"))
        return real(*a, **kw)

    mod.register_escalating = spy
    try:
        m = Mapper(W, H, voxel_size=0.02, icp_retry_dist=0.0)
        for _ in range(3):
            m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    finally:
        mod.register_escalating = real
    assert calls and all(c == 0.0 for c in calls)
    assert m.icp_escalations == 0


def test_escalation_counter_increments_when_the_retry_is_used():
    """The counter is the diagnostic that says a scan was repeatedly on the
    edge of the terminal failure, so it must reflect real escalations."""
    import roomscan.slam.mapper as mod
    real = mod.register_escalating

    def always_escalate(*a, **kw):
        res, _ = real(*a, **kw)
        return res, True

    m = Mapper(W, H, voxel_size=0.02)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)   # bootstrap, no ICP
    mod.register_escalating = always_escalate
    try:
        m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
        m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    finally:
        mod.register_escalating = real
    assert m.icp_escalations == 2


def test_default_retry_dist_is_wider_than_default_max_dist():
    m = Mapper(W, H, voxel_size=0.02)
    assert m._retry_dist > m._gate["max_dist"]


# --- BUG-037: the barometric height constraint
#
# These drive `_apply_baro_z` directly, feeding its own output back in as the
# next frame's pose -- which is what the real loop does (the corrected pose
# becomes t_prev and seeds the next prediction), and is the feedback path the
# constraint has to account for.

def _drive_baro(m, pressures, ref_pa=101325.0):
    """Run `pressures` through m._apply_baro_z with the pose fed back, and
    return (heights, per_frame_steps) in metres along world-up."""
    from roomscan.slam.frames import world_up
    up = world_up()
    pose = np.eye(4)
    heights, steps = [], []
    for pa in pressures:
        before = float(np.dot(pose[:3, 3], up))
        pose = m._apply_baro_z(pose, pa)
        after = float(np.dot(pose[:3, 3], up))
        heights.append(after)
        steps.append(after - before)
    return np.array(heights), np.array(steps)


def _old_blend_vertical_path(pressures, weight=0.05):
    """The pre-BUG-037 constraint, verbatim: blend the pose height toward the
    RAW barometric height every frame. Used as the regression yardstick -- it
    is the thing that invented ~22 m of vertical "path" on a 32 m circuit."""
    from roomscan.slam.frames import baro_height_m
    ref, h_pose, path = None, 0.0, 0.0
    for pa in pressures:
        if ref is None:
            ref = pa
            continue
        step = weight * (baro_height_m(pa, ref) - h_pose)
        h_pose += step
        path += abs(step)
    return path


def test_baro_datum_averages_the_first_frames_not_one_sample():
    """A single pressure sample carries ~267 mm RMS of apparent altitude, so
    freezing the datum on frame 1 baked that error into the whole run. The
    datum is now the mean of the first _BARO_REF_FRAMES, and nothing is applied
    until it exists."""
    from roomscan.slam.mapper import _BARO_REF_FRAMES
    rng = np.random.default_rng(0)
    noisy = 101325.0 + rng.normal(0.0, 3.1, _BARO_REF_FRAMES)
    m = Mapper(W, H, voxel_size=0.02)
    _, steps = _drive_baro(m, noisy)
    assert m._ref_pa == pytest.approx(float(np.mean(noisy)))
    assert np.all(steps == 0.0)              # warm-up applies nothing
    assert m.baro_correction_m == 0.0


def test_baro_white_noise_no_longer_invents_vertical_path():
    """The defect: the barometer's ~3.1 Pa of per-frame white noise (~267 mm of
    apparent altitude) went straight into the pose at 5% a frame. Against the
    same noise the new constraint must move the pose orders of magnitude less."""
    rng = np.random.default_rng(1)
    n = 2000
    noisy = 101325.0 + rng.normal(0.0, 3.1, n)      # measured noise, no real motion
    m = Mapper(W, H, voxel_size=0.02)
    _, steps = _drive_baro(m, noisy)
    new_path = float(np.abs(steps).sum())
    old_path = _old_blend_vertical_path(noisy)
    assert old_path > 5.0                            # metres of invented motion
    assert new_path < old_path / 100.0
    assert abs(m.baro_correction_m) < 0.05           # and it goes nowhere


def test_baro_authority_bounds_the_lifetime_correction():
    """A *sustained* barometric disagreement must move the pose by only
    `baro_authority` of it -- the barometer contributes, it does not own the
    height. The old blend converged to 100% of any disagreement, which is why a
    barometer wandering 0.43 m dragged the pose 0.58 m off."""
    from roomscan.slam.frames import baro_height_m
    m = Mapper(W, H, voxel_size=0.02, baro_authority=0.05, baro_tau_frames=10)
    ref = 101325.0
    off = ref - 5.0                                  # ~+42 cm of apparent altitude
    n_warm = 90                                      # datum frames (nothing applied)
    heights, _ = _drive_baro(m, [ref] * n_warm + [off] * 500)
    disagreement = baro_height_m(off, ref)
    assert disagreement > 0.3
    assert heights[-1] == pytest.approx(0.05 * disagreement, rel=0.02)
    assert m.baro_correction_m == pytest.approx(heights[-1], abs=1e-9)


def test_baro_authority_zero_disables_the_constraint():
    m = Mapper(W, H, voxel_size=0.02, baro_authority=0.0)
    heights, steps = _drive_baro(m, np.linspace(101325.0, 101300.0, 500))
    assert np.all(steps == 0.0)
    assert m.baro_correction_m == 0.0


def test_baro_rejects_zero_pressure_dropout():
    """2026-08-05-crazySLAM.bin: 4 frames carried pressure == 0.0 Pa (a dropout),
    which baro_height_m(0, ref) = 44330 m turns into a ~2.45 m vertical step each
    -- the whole 8.6 m of that capture's fabricated vertical. The reject guard
    must drop a 0.0 Pa sample entirely: no datum poisoning, no filter update, no
    pose move. Without it the same dropout injects a metre-scale step."""
    ref = 101325.0
    warm = [ref] * 95                        # establish the datum (90) + a few
    seq = warm + [0.0] + [ref] * 5           # one dropout, then good again

    m_on = Mapper(W, H, voxel_size=0.02, baro_reject_m=50.0)
    heights_on, steps_on = _drive_baro(m_on, seq)
    assert m_on.baro_rejected == 1
    assert np.abs(steps_on).max() < 0.01     # dropout ignored: no vertical jump
    assert m_on._ref_pa == pytest.approx(ref)   # datum not poisoned by the 0.0

    m_off = Mapper(W, H, voxel_size=0.02, baro_reject_m=0.0)   # rejection disabled
    _, steps_off = _drive_baro(m_off, seq)
    assert m_off.baro_rejected == 0
    assert np.abs(steps_off).max() > 1.0     # unrejected dropout fabricates a step


def test_baro_rejects_datum_relative_altitude_outlier():
    # A physically-plausible pressure that still implies a huge altitude vs the
    # datum (here ~600 m from a ~7 kPa offset) is a glitch for an indoor scan and
    # is rejected; a nearby in-range value is not.
    ref = 101325.0
    m = Mapper(W, H, voxel_size=0.02, baro_reject_m=50.0)
    _drive_baro(m, [ref] * 95)               # datum
    _drive_baro(m, [ref - 7000.0])           # ~+600 m implied -> outlier
    assert m.baro_rejected == 1
    _drive_baro(m, [ref - 5.0])              # ~+0.4 m -> accepted
    assert m.baro_rejected == 1


def test_baro_longer_tau_rejects_more_noise():
    """`baro_tau_frames` is the noise filter: a longer one must let less of the
    barometer's white noise through into the pose per frame.

    Measured on the per-frame step, not on the height: the height also carries
    the filter's start-up transient (the EMA opens at the first innovation and
    decays over ~tau), which a LONGER tau necessarily drags out further -- so
    the height's spread would rank the two filters backwards."""
    rng = np.random.default_rng(2)
    noisy = 101325.0 + rng.normal(0.0, 3.1, 3000)
    short = Mapper(W, H, voxel_size=0.02, baro_tau_frames=10)
    long_ = Mapper(W, H, voxel_size=0.02, baro_tau_frames=900)
    _, s_short = _drive_baro(short, noisy)
    _, s_long = _drive_baro(long_, noisy)
    assert s_long.std() < s_short.std() / 10.0


# --- Task 8: rate-aware IMU/env behavior (baro_tau_frames tracks the APPLIED
# IMU/env poll rate, never the ToF target FPS) -------------------------------

def test_set_imu_rate_hz_pins_the_formula():
    """`baro_tau_frames = round(30 * imu_rate_hz)` -- the exact numeric
    equivalence the plan pins: 900 @ 30 Hz, 2700 @ 90 Hz."""
    m = Mapper(W, H, voxel_size=0.02)
    m.set_imu_rate_hz(30.0)
    assert m.baro_tau_frames == 900
    m.set_imu_rate_hz(90.0)
    assert m.baro_tau_frames == 2700


def test_set_imu_rate_hz_none_or_zero_is_a_noop():
    """Coupled mode's "unknown yet"/legacy-caller case: `baro_tau_frames` must
    keep whatever it was constructed with (BUG-062's "verify the knob took
    effect" -- a caller that never calls this at all must see byte-identical
    behavior)."""
    m = Mapper(W, H, voxel_size=0.02, baro_tau_frames=555)
    m.set_imu_rate_hz(None)
    assert m.baro_tau_frames == 555
    m.set_imu_rate_hz(0)
    assert m.baro_tau_frames == 555


def test_live_rate_change_does_not_reset_baro_state_or_trajectory():
    """A live IMU/env rate change (Task 7) must not reset the low-pass state,
    the barometer's own datum, or introduce a correction step -- only future
    samples are averaged differently."""
    ref = 101325.0
    m = Mapper(W, H, voxel_size=0.02, baro_authority=0.05, baro_tau_frames=900)
    m.set_imu_rate_hz(30.0)
    _drive_baro(m, [ref] * 90 + [ref - 5.0] * 200)   # warm-up + real settling

    before_correction = m.baro_correction_m
    before_lp = m._baro_lp
    before_ref_pa = m._ref_pa
    before_traj_len = len(m.trajectory)

    m.set_imu_rate_hz(90.0)

    assert m.baro_tau_frames == 2700
    assert m.baro_correction_m == before_correction     # no correction step introduced
    assert m._baro_lp == before_lp
    assert m._ref_pa == before_ref_pa
    assert len(m.trajectory) == before_traj_len         # map/trajectory untouched


def test_baro_time_constant_equivalent_in_real_seconds_at_30_and_90_hz():
    """THE equivalence test (plan step 1/5): a 30-second barometer time
    constant, sized from the APPLIED IMU/env rate, must settle to the SAME
    fraction of a sustained disagreement after 30 REAL SECONDS whether the
    applied rate -- and hence `baro_tau_frames` -- is 30 Hz (900) or 90 Hz
    (2700). Compared in seconds, not frames, per step 5: each rate drives
    its OWN sample count for the same 30 s window (900 vs 2700), and an EMA's
    exact settling fraction after k == tau_frames steps is
    1 - (1 - 1/tau_frames)**tau_frames --> 1/e, regardless of tau_frames, so
    equal REAL TIME must give equal settling regardless of which of the two
    rates was in force."""
    from roomscan.slam.frames import baro_height_m
    ref = 101325.0
    off = ref - 5.0                      # a sustained disagreement, no noise
    frac_after_one_tau = 1.0 - math.exp(-1.0)
    results = {}
    for rate_hz, tau_frames in ((30.0, 900), (90.0, 2700)):
        m = Mapper(W, H, voxel_size=0.02, baro_authority=0.05, baro_tau_frames=1)
        m.set_imu_rate_hz(rate_hz)
        assert m.baro_tau_frames == tau_frames        # the exact pinned values
        n_warm = 90
        n_settle = int(round(rate_hz * 30.0))          # 30 REAL seconds at THIS rate
        heights, _ = _drive_baro(m, [ref] * n_warm + [off] * n_settle)
        results[rate_hz] = heights[-1]
    target = 0.05 * baro_height_m(off, ref)
    for rate_hz, height in results.items():
        assert height == pytest.approx(frac_after_one_tau * target, rel=0.02), rate_hz
    # and the two rates agree with EACH OTHER, not just with the analytic target
    assert results[30.0] == pytest.approx(results[90.0], rel=0.02)


def test_baro_tau_equivalence_holds_with_the_concurrent_tof_rate_mismatched():
    """The applied IMU/env rate is what matters, not whatever the concurrent
    ToF profile happens to be running -- `Mapper` takes no ToF-rate argument
    at all, so a caller resolving a genuinely decoupled combination (e.g. a
    90 Hz ToF profile with a 30 Hz IMU/env rate, or vice versa) gets the SAME
    settling behavior as the coupled case above as long as it passes the
    correct applied IMU/env rate -- proving `set_imu_rate_hz` is driven only
    by its own argument."""
    from roomscan.slam.frames import baro_height_m
    ref = 101325.0
    off = ref - 5.0
    frac_after_one_tau = 1.0 - math.exp(-1.0)
    # A 30 Hz IMU/env rate while some unrelated (and irrelevant, since Mapper
    # never sees it) 90 Hz ToF profile is concurrently running.
    m = Mapper(W, H, voxel_size=0.02, baro_authority=0.05, baro_tau_frames=1)
    m.set_imu_rate_hz(30.0)
    assert m.baro_tau_frames == 900
    heights, _ = _drive_baro(m, [ref] * 90 + [off] * 900)
    target = 0.05 * baro_height_m(off, ref)
    assert heights[-1] == pytest.approx(frac_after_one_tau * target, rel=0.02)


# --- TSDF block-grid gauge (BUG-035, owner ask 2026-07-31) -----------------
#
# BUG-035 is the ceiling that silently killed 18% of a real sweep: map growth
# stalls near capacity and frame-to-model tracking collapses ~30 frames later
# with no error at all. The number has to be visible while the scan runs.

def test_frame_step_reports_the_block_gauge():
    m = Mapper(W, H, voxel_size=0.02, block_count=12345)
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert step.blocks_used is not None and step.blocks_used > 0
    assert step.blocks_capacity is not None and step.blocks_capacity >= step.blocks_used
    assert step.blocks_configured == 12345


def test_block_usage_is_sampled_at_the_metrics_rate_not_per_frame():
    """`TsdfMap.block_usage()` reads the hashmap size, which is a DEVICE SYNC
    on CUDA. Calling it per frame would put a pipeline stall in the hot path
    for a number nobody reads faster than 4 Hz. Drive the mapper on a frozen
    clock and count the calls: many frames, one sample.
    """
    from roomscan.slam import mapper as mapper_mod

    clock = {"t": 0.0}
    m = Mapper(W, H, voxel_size=0.02, clock=lambda: clock["t"])
    calls = []
    real = m._tsdf.block_usage

    def counting():
        calls.append(clock["t"])
        return real()

    m._tsdf.block_usage = counting

    for _ in range(8):
        m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert len(calls) == 1, f"sampled {len(calls)} times on a frozen clock"

    # Past the interval it samples again -- exactly once more.
    clock["t"] = mapper_mod._BLOCK_USAGE_INTERVAL_S + 0.001
    for _ in range(5):
        m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert len(calls) == 2


def test_block_gauge_keeps_the_last_reading_when_the_probe_fails():
    """A gauge must never be able to kill a scan, and a failed read must not
    report a fake 0 blocks (which would read as an empty map)."""
    clock = {"t": 0.0}
    m = Mapper(W, H, voxel_size=0.02, clock=lambda: clock["t"])
    good = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert good.blocks_used > 0

    def boom():
        raise RuntimeError("device sync failed")

    m._tsdf.block_usage = boom
    clock["t"] = 10.0
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert step.blocks_used == good.blocks_used     # last reading, not 0 and not a crash


# --- BUG-062: SlamConfig.mapper_kwargs() is the single source for Mapper knobs ---

def test_mapper_kwargs_are_all_accepted_by_mapper():
    """Every key the config hands out must be a real `Mapper.__init__` parameter.

    The live path now splats this dict, so a typo or a renamed Mapper parameter
    would raise TypeError at the moment SLAM arms -- on the reader thread, in
    front of the owner -- rather than here.
    """
    import inspect
    from roomscan.slam.config import SlamConfig
    accepted = set(inspect.signature(Mapper.__init__).parameters) - {"self", "width", "height"}
    assert set(SlamConfig().mapper_kwargs()) <= accepted


def test_mapper_kwargs_defaults_match_mapper_signature():
    """BUG-062's fix must be behaviour-neutral on a stock config.

    Forwarding a field whose config default differs from Mapper's default would
    silently change Live SLAM for every user who never touched `[slam]`. Only
    `device` and the FOV pair are allowed to differ: the caller overrides those
    deliberately (measured sensor FOV, resolved compute device).
    """
    import inspect
    from roomscan.slam.config import SlamConfig
    sig = inspect.signature(Mapper.__init__).parameters
    deliberate = {"device", "fov_h", "fov_v"}
    mismatched = {
        k: (v, sig[k].default)
        for k, v in SlamConfig().mapper_kwargs().items()
        if k not in deliberate and sig[k].default != v
    }
    assert not mismatched, f"config default != Mapper default: {mismatched}"


def test_mapper_kwargs_covers_every_shared_field():
    """A `[slam]` key that Mapper accepts but the dict omits is exactly the
    BUG-062 defect: honoured by the CLI/Detailed paths, ignored by Live SLAM."""
    import inspect
    from dataclasses import fields
    from roomscan.slam.config import SlamConfig
    accepted = set(inspect.signature(Mapper.__init__).parameters)
    shared = {f.name for f in fields(SlamConfig)} & accepted
    # `device` is shared by name but deliberately not sourced from the field.
    missing = shared - set(SlamConfig().mapper_kwargs()) - {"device"}
    assert not missing, f"[slam] keys Mapper accepts but Live SLAM ignores: {missing}"


# ---- stage timing (plan item 2, 2026-08-02) --------------------------------

def test_bootstrap_frame_has_no_raycast_or_icp_but_does_integrate():
    """Frame 1 accepts the prior directly (no raycast/ICP branch), then
    integrates -- so raycast_ms/icp_ms must stay exactly 0.0 while
    integrate_ms is measured. A test that only checked ">= 0" could not catch
    a wiring bug that always reports 0 for everything."""
    m = Mapper(W, H, voxel_size=0.02)
    step = m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert not step.tracking_lost
    assert step.raycast_ms == 0.0
    assert step.icp_ms == 0.0
    assert step.integrate_ms >= 0.0


def test_lost_frame_has_all_stage_timings_zero():
    m = Mapper(W, H, voxel_size=0.02)
    step = m.step(np.zeros((H, W), dtype=np.float32), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert step.tracking_lost
    assert step.raycast_ms == 0.0
    assert step.icp_ms == 0.0
    assert step.integrate_ms == 0.0     # lost frames skip integrate entirely


def test_tracked_frame_measures_raycast_and_icp(monkeypatch):
    """Strong version: patch the raycast/ICP entry points to sleep a known
    amount and assert the reported stage time is at least that long --
    discriminates against a counter that always reads back a stale 0."""
    import time as _time
    from roomscan.slam import mapper as mapper_mod

    m = Mapper(W, H, voxel_size=0.02)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)   # bootstrap frame

    real_raycast = m._tsdf.raycast
    def _slow_raycast(*a, **kw):
        _time.sleep(0.03)
        return real_raycast(*a, **kw)
    monkeypatch.setattr(m._tsdf, "raycast", _slow_raycast)

    real_register = mapper_mod.register_escalating
    def _slow_register(*a, **kw):
        _time.sleep(0.03)
        return real_register(*a, **kw)
    monkeypatch.setattr(mapper_mod, "register_escalating", _slow_register)

    step = m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert step.raycast_ms >= 25.0
    assert step.icp_ms >= 25.0


def test_mapper_device_property_matches_resolved_device():
    m_cpu = Mapper(W, H, voxel_size=0.02, device="CPU:0")
    assert m_cpu.device == "CPU:0"
    assert m_cpu.device == str(m_cpu._device)


# ---- item 5 (2026-08-02): ICP nearest-neighbour index on the host ----------
#
# `icp_device` is a device SELECTOR, and the whole failure mode this guards
# against is that a selector which silently fails to apply produces exactly the
# same output as one that applied correctly -- because the change is
# bit-identical by design. So every test below asserts the *plumbing* (what
# device object `register` was handed), never the numbers.

def test_icp_device_defaults_to_the_host_and_is_separate_from_compute_device():
    """The default splits the two devices: TSDF/intrinsic/source cloud on the
    compute device, the NN index on the host."""
    m = Mapper(W, H, voxel_size=0.02, device="CPU:0")
    assert m.icp_device == "CPU:0"
    # Constructed on a *named* CUDA device without touching CUDA: only the
    # o3d.core.Device object is built here, nothing is allocated on it.
    assert str(o3d.core.Device("CUDA:0")) == "CUDA:0"


def test_icp_device_none_follows_the_compute_device():
    """`None` restores the pre-item-5 behaviour exactly (one device for
    everything), which is what makes the change reversible without editing
    code."""
    m = Mapper(W, H, voxel_size=0.02, device="CPU:0", icp_device=None)
    assert m.icp_device == m.device == "CPU:0"
    assert m._register_device is m._device


def test_icp_device_accepts_a_device_object_as_well_as_a_string():
    m = Mapper(W, H, voxel_size=0.02, icp_device=o3d.core.Device("CPU:0"))
    assert m.icp_device == "CPU:0"


def test_icp_device_is_what_step_hands_to_register(monkeypatch):
    """The knob must reach `odometry.register`, not merely be stored.

    Proved by intercepting `register_escalating` and reading the `device` it
    was called with -- a `Mapper` that kept the field and passed `self._device`
    anyway would produce identical poses and pass every numeric test.
    """
    from roomscan.slam import mapper as mapper_mod

    seen = {}
    real = mapper_mod.register_escalating

    def _tap(*a, **kw):
        seen["device"] = kw.get("device")
        return real(*a, **kw)
    monkeypatch.setattr(mapper_mod, "register_escalating", _tap)

    m = Mapper(W, H, voxel_size=0.02, device="CPU:0",
               icp_device=o3d.core.Device("CPU:0"))
    m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)   # bootstrap
    m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert seen["device"] is m._icp_device
    assert seen["device"] is not m._device      # the two are distinct objects


def test_icp_device_is_ignored_for_6dof(monkeypatch):
    """`6dof` is Open3D's own tensor ICP and runs where its point clouds live;
    handing it a different device is a mismatch, not an optimization. A single
    `[slam] icp_device` must therefore not follow a user who also switches
    `icp_mode` -- so this asserts the compute device is used instead."""
    from roomscan.slam import mapper as mapper_mod

    seen = {}
    real = mapper_mod.register_escalating

    def _tap(*a, **kw):
        seen["device"] = kw.get("device")
        return real(*a, **kw)
    monkeypatch.setattr(mapper_mod, "register_escalating", _tap)

    m = Mapper(W, H, voxel_size=0.02, device="CPU:0", icp_mode="6dof",
               icp_device=o3d.core.Device("CPU:0"))
    m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)
    assert seen["device"] is m._device
    assert seen["device"] is not m._icp_device


def test_step_does_not_re_download_positions_to_count_them():
    """Item 5: `Mapper.step` used to call `positions.cpu().numpy()` a second
    time purely to read `.shape[0]`, a whole device->host transfer for one
    integer that `TsdfMap.raycast` already knew.

    Asserted on the source rather than by timing, because 0.016 ms/frame is
    far below this box's timing noise -- a stopwatch test here would be a
    coin flip, and the thing that must not come back is the *call*.
    """
    import inspect
    src = inspect.getsource(Mapper.step)
    assert "positions.cpu().numpy()" not in src
    assert "with_count=True" in src


def test_raycast_count_matches_the_cloud_it_returns():
    """The metadata must be the number of points actually in the cloud, on the
    real geometry -- an off-by-one or a pre-mask count would sail through the
    `< _MIN_VALID_POINTS` gate it feeds and only show up as odd tracking."""
    from roomscan.slam.tsdf import TsdfMap
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    m.integrate(_wall(1.0), K, np.eye(4))
    pc, n = m.raycast(K, np.eye(4), W, H, with_count=True)
    assert pc is not None
    assert n == pc.point.positions.cpu().numpy().shape[0] > 0
    # ...and the no-count call still returns a bare cloud (every other caller).
    assert isinstance(m.raycast(K, np.eye(4), W, H), type(pc))


def test_raycast_count_is_zero_on_every_empty_path():
    """All four early returns must answer `(None, 0)`, not `None` -- `Mapper`
    unpacks the tuple unconditionally, so a missed branch is a TypeError on a
    real frame, on the mapper thread."""
    from roomscan.slam.tsdf import TsdfMap
    m = TsdfMap(voxel_size=0.02)
    K = pinhole(W, H)
    assert m.raycast(K, np.eye(4), W, H, with_count=True) == (None, 0)   # empty map
    m.integrate(_wall(1.0), K, np.eye(4))
    # a view pointed away from everything integrated: rays hit nothing
    away = np.eye(4)
    away[2, 3] = -50.0
    pc, n = m.raycast(K, away, W, H, with_count=True)
    assert (pc, n) == (None, 0)
    # an explicitly empty block-coord set
    empty = o3d.core.Tensor(np.zeros((0, 3), np.int32))
    assert m.raycast(K, np.eye(4), W, H, block_coords=empty, with_count=True) == (None, 0)


def test_detailed_mapper_kwargs_covers_every_shared_field():
    """The Detailed preset was the THIRD place that knew the `Mapper` field
    list (item 5, 2026-08-02). BUG-062 is what a second one costs; this pins
    that Detailed cannot silently drop a `[slam]` knob the other two honour."""
    import inspect
    from roomscan.slam.config import DetailedSlamPreset, SlamConfig
    accepted = set(inspect.signature(Mapper.__init__).parameters)
    kw = DetailedSlamPreset().mapper_kwargs(SlamConfig())
    assert set(kw) <= accepted - {"self", "width", "height"}
    missing = set(SlamConfig().mapper_kwargs()) - set(kw)
    assert not missing, f"[slam] keys Detailed reconstruction ignores: {missing}"


def test_detailed_preset_overrides_win_over_the_base_config():
    """...and building on the base dict must not let a `[slam]` key shadow the
    preset's own reconstruction settings, which are what the sidecar's
    fingerprint promises were used."""
    from roomscan.slam.config import DetailedSlamPreset, SlamConfig
    base = SlamConfig(voxel_size=0.99, block_count=1, max_dist=0.99,
                      icp_retry_dist=0.99, max_iter=99, stationary_hold=True,
                      icp_device="CUDA:3")
    p = DetailedSlamPreset()
    kw = p.mapper_kwargs(base)
    assert kw["voxel_size"] == p.voxel_size
    assert kw["block_count"] == p.block_count
    assert kw["max_dist"] == p.max_dist
    assert kw["icp_retry_dist"] == p.retry_dist
    assert kw["max_iter"] == p.max_iter
    assert kw["stationary_hold"] is False
    # ...while a genuinely shared knob still comes from the config.
    assert kw["icp_device"] == "CUDA:3"


def test_the_four_stationary_knobs_are_inert_when_the_hold_is_off():
    """Detailed newly forwards `stationary_window`/`coherence`/`step_ceiling`/
    `rot_ceiling` (they come from the base dict now). That is only
    behaviour-neutral because `Mapper` builds no gate when `stationary_hold`
    is False -- assert that rather than assuming it."""
    m = Mapper(W, H, voxel_size=0.02, stationary_hold=False,
               stationary_window=3, stationary_coherence=0.01,
               stationary_step_ceiling=99.0, stationary_rot_ceiling=99.0)
    assert m._stationary_gate is None


# ---- BUG-069 accel ZUPT + BUG-031/067 quat-phase: plumbing through Mapper.step

import types


def _batch(accel_g=None, gyro_dps=None, gbias_dps=None):
    """A minimal stand-in for protocol.ImuRawBatch: Mapper reads the fields by
    getattr, so a namespace with the arrays it needs is enough."""
    n = 8
    a = np.tile(np.asarray(accel_g, float), (n, 1)) if accel_g is not None else np.zeros((0, 3))
    g = np.tile(np.asarray(gyro_dps, float), (n, 1)) if gyro_dps is not None else np.zeros((0, 3))
    b = np.tile(np.asarray(gbias_dps, float), (n, 1)) if gbias_dps is not None else np.zeros((0, 3))
    return types.SimpleNamespace(accel_g=a, gyro_dps=g, gbias_dps=b)


def test_zupt_is_on_by_default_but_a_noop_without_imu_raw():
    # ON by default (owner decision 2026-08-06), but a no-op when no accel signal
    # is supplied -- so a caller that never passes imu_raw is byte-identical.
    m = Mapper(W, H, voxel_size=0.02)
    assert m._zupt is not None
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0)   # no imu_raw
    assert m.zupt_count == 0


def test_zupt_can_be_disabled():
    m = Mapper(W, H, voxel_size=0.02, zupt_enabled=False)
    assert m._zupt is None
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
           imu_raw=_batch(accel_g=[0.0, 0.0, 1.0]))
    assert m.zupt_count == 0


def test_zupt_freezes_the_true_pose_translation_and_reaches_the_map():
    """With ZUPT on and a steady 1 g specific force, once the window fills the
    reconstruction pose's translation is held at t_prev -- the constraint reaches
    _t_prev/integration, not just the preview (BUG-069)."""
    # veto off here: this pins the FREEZE-reaches-the-map mechanism, not the
    # coherence veto (covered by test_slam_motion.py). On a static wall the tiny
    # residual ICP drift is weakly coherent, which the veto would (correctly)
    # decline to hold.
    m = Mapper(W, H, voxel_size=0.02, zupt_enabled=True, zupt_window=3, zupt_coherence=0.0)
    accel = [0.0, 0.0, 1.0]                       # 1 g along +z body, at rest
    for _ in range(8):
        m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
               imu_raw=_batch(accel_g=accel))
    assert m.zupt_count > 0
    # Once ZUPT engages (after the window fills), the RECONSTRUCTION pose stops
    # moving -- the tail is frozen bit-for-bit, and it is _t_prev (the tracking
    # prior + integration pose) that is held, not merely the display pose.
    tail = [p[:3, 3] for p in m.trajectory[-3:]]
    assert np.allclose(tail[0], tail[1]) and np.allclose(tail[1], tail[2])
    assert np.allclose(m._t_prev, m.trajectory[-1][:3, 3])
    assert np.linalg.norm(m._t_prev) < 0.05       # total fabricated drift stayed small


def test_zupt_does_not_fire_when_specific_force_is_off_one_g():
    m = Mapper(W, H, voxel_size=0.02, zupt_enabled=True, zupt_window=3)
    for _ in range(6):
        m.step(_textured_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
               imu_raw=_batch(accel_g=[0.0, 0.0, 1.5]))   # 1.5 g -> moving
    assert m.zupt_count == 0


def test_quat_phase_off_by_default_is_a_noop():
    m = Mapper(W, H, voxel_size=0.02)
    assert m._apply_quat_phase is False
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
           imu_raw=_batch(gyro_dps=[0.0, 0.0, 100.0]), quat_offset_us=7760.0)
    assert m.quat_phase_count == 0


def test_quat_phase_fires_when_enabled_with_lead_and_gyro():
    m = Mapper(W, H, voxel_size=0.02, apply_quat_phase=True)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
           imu_raw=_batch(gyro_dps=[0.0, 0.0, 100.0]), quat_offset_us=7760.0)
    assert m.quat_phase_count == 1


def test_quat_phase_noop_without_a_lead_even_when_enabled():
    m = Mapper(W, H, voxel_size=0.02, apply_quat_phase=True)
    m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
           imu_raw=_batch(gyro_dps=[0.0, 0.0, 100.0]), quat_offset_us=None)
    assert m.quat_phase_count == 0


def test_synthetic_pan_tracks_without_loss_in_both_modes():
    """A pure rotate-in-place pan (SyntheticPan) must stay tracked -- a lost
    frame freezes the pose and would mask fabrication. Smoke-level: both the
    shipped translation mode and the new soft_prior mode complete a sweep with
    zero tracking-lost frames. (Absolute drift is not asserted: the synth ray
    model is not bit-exact to Deprojector -- see SyntheticPan's docstring.)"""
    from roomscan.slam.synthscene import SyntheticPan, Room
    for mode in ("translation", "soft_prior"):
        pan = SyntheticPan(W, H, room=Room(seed=3), amp_deg=18.0, rate_deg_s=40.0)
        m = Mapper(W, H, voxel_size=0.03, icp_mode=mode, device="CPU:0")
        for _ in range(60):
            d, q = pan.next_frame()
            m.step(d, q, 101325.0)
        assert m.tracking_lost_count == 0
        assert len(m.trajectory) == 60


def test_interpolated_frames_skip_the_fixed_rollback_but_keep_zupt():
    """#155 no-double-correction pin: the offline loader hands Mapper a quat
    already SLERP-aligned to the frame instant with quat_offset_us=None, so the
    fixed gyro rollback must not apply a second correction — while the ZUPT,
    which rides the same imu_aux, must still see the raw batch."""
    m = Mapper(W, H, voxel_size=0.02, apply_quat_phase=True,
               zupt_enabled=True, zupt_window=3)
    for _ in range(8):
        m.step(_wall(1.0), (1.0, 0.0, 0.0, 0.0), 101325.0,
               imu_raw=_batch(accel_g=[0.0, 0.0, 1.0],
                              gyro_dps=[0.0, 0.0, 100.0]),
               quat_offset_us=None)
    assert m.quat_phase_count == 0
    assert m.zupt_count > 0
