import numpy as np
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
