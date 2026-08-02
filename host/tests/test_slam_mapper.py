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
