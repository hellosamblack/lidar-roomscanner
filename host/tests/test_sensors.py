import math
import struct

import numpy as np
import pytest

from roomscan.protocol import (
    Frame,
    FrameHeader,
    FrameType,
    StreamId,
)
from roomscan.protocol import ImuRawBatch
from roomscan.sensors import (
    SensorState,
    boresight_view_deg,
    graft_yaw,
    gravity_body_from_imu_raw,
    ir_gravity_rot,
    quat_mul,
    quat_pitch_alt_deg,
    quat_pitch_deg,
    quat_roll_alt_deg,
    quat_roll_deg,
    quat_to_matrix,
    quat_yaw_alt_deg,
    quat_yaw_deg,
    tilt_compensated_heading,
    tilt_from_down_deg,
    triad_roll_deg,
    wrap180,
)


def _axis_angle_quat(axis, deg):
    """Test helper: unit quat [w,x,y,z] for a rotation of `deg` about `axis`."""
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    a = math.radians(deg) / 2.0
    return (math.cos(a), *(math.sin(a) * axis))


def _frame(stream_id: int, payload: bytes) -> Frame:
    h = FrameHeader(FrameType.DATA, stream_id, 0, 1, 123, 0, 0, len(payload))
    return Frame(h, payload)


def test_quat_to_matrix_identity():
    m = quat_to_matrix(1.0, 0.0, 0.0, 0.0)
    assert np.allclose(m, np.eye(3), atol=1e-6)


def test_quat_to_matrix_90deg_about_z():
    # 90° about +Z: [w,x,y,z] = [cos45, 0, 0, sin45]
    s = np.sqrt(0.5)
    m = quat_to_matrix(s, 0.0, 0.0, s)
    # +X axis maps to +Y
    assert np.allclose(m @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-6)


def test_state_feeds_quat_and_env():
    st = SensorState()
    st.feed(_frame(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    st.feed(_frame(StreamId.ENV, struct.pack("<5f", 101325.0, 1.0, 2.0, 3.0, 20.0)))
    assert st.latest_quat() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    env = st.latest_env()
    assert env.pressure_pa == pytest.approx(101325.0)
    assert env.mag_ut == pytest.approx((1.0, 2.0, 3.0))
    assert env.temp_c == pytest.approx(20.0)


def test_state_ignores_other_streams():
    st = SensorState()
    st.feed(_frame(StreamId.RAW_3DMD, b"\x00" * 8))
    assert st.latest_quat() is None
    assert st.latest_env() is None


def test_state_history_bounded():
    st = SensorState(history=4)
    for i in range(10):
        st.feed(_frame(StreamId.ENV, struct.pack("<5f", 1000.0 + i, 0, 0, 0, float(i))))
    p = st.pressure_history()
    assert len(p) == 4
    assert p[-1] == pytest.approx(1009.0)  # newest retained


def test_tilt_compensated_heading_level_north():
    # Level device (identity), mag pointing +X (north-ish) -> heading 0
    h = tilt_compensated_heading((1.0, 0.0, 0.0, 0.0), (1.0, 0.0, 0.0))
    assert h == pytest.approx(0.0, abs=1.0) or h == pytest.approx(360.0, abs=1.0)


def test_gizmo_pose_identity():
    from roomscan.sensors import gizmo_pose
    m = gizmo_pose((1.0, 0.0, 0.0, 0.0), scale=0.2, anchor=(1.0, 2.0, 3.0))
    # True identity matrix mapping T_WORLD_TO_CV @ T_CV_TO_BODY
    expected_m = np.array([
        [-0.2, 0.0, 0.0, 1.0],
        [0.0, 0.0, -0.2, 2.0],
        [0.0, -0.2, 0.0, 3.0],
        [0.0, 0.0, 0.0, 1.0]
    ])
    assert np.allclose(m, expected_m)


def test_gizmo_pose_yaw_maps_to_y_axis_rotation():
    # 30 degree yaw (around IMU Z axis)
    import math
    from roomscan.sensors import gizmo_pose
    theta = math.radians(30.0) / 2
    quat = (math.cos(theta), 0.0, 0.0, math.sin(theta))
    m = gizmo_pose(quat, scale=1.0, anchor=(0.0, 0.0, 0.0))

    # Rotation around IMU Z axis mapped correctly through physically accurate matrices
    expected = np.array([
        [-math.cos(math.radians(30.0)), math.sin(math.radians(30.0)), 0.0],
        [0.0, 0.0, -1.0],
        [-math.sin(math.radians(30.0)), -math.cos(math.radians(30.0)), 0.0]
    ])
    assert np.allclose(m[:3, :3], expected, atol=1e-4)


def test_wrap180():
    assert wrap180(190.0) == pytest.approx(-170.0)
    assert wrap180(-190.0) == pytest.approx(170.0)
    assert wrap180(30.0) == pytest.approx(30.0)


def test_quat_yaw_of_z_rotation():
    s = np.sqrt(0.5)  # 90 deg about +Z
    assert quat_yaw_deg((s, 0.0, 0.0, s)) == pytest.approx(90.0, abs=1e-4)


def test_quat_roll_of_x_rotation():
    s = np.sqrt(0.5)  # 90 deg about +X -- pure roll, no pitch/yaw
    assert quat_roll_deg((s, s, 0.0, 0.0)) == pytest.approx(90.0, abs=1e-4)
    assert quat_pitch_deg((s, s, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-4)
    assert quat_yaw_deg((s, s, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-4)


def test_quat_roll_identity_is_zero():
    assert quat_roll_deg((1.0, 0.0, 0.0, 0.0)) == pytest.approx(0.0, abs=1e-9)


def test_graft_yaw_adds_heading_preserves_tilt():
    import math
    a = math.radians(30.0) / 2  # 30 deg pitch about +Y, no yaw
    q = (math.cos(a), 0.0, math.sin(a), 0.0)
    grafted = graft_yaw(q, 40.0)
    # pitch unchanged (tilt preserved), yaw increased by ~40 deg
    assert quat_pitch_deg(grafted) == pytest.approx(quat_pitch_deg(q), abs=0.5)
    assert wrap180(quat_yaw_deg(grafted) - quat_yaw_deg(q)) == pytest.approx(40.0, abs=0.5)


def test_graft_yaw_zero_is_noop():
    q = (0.9238795, 0.0, 0.0, 0.3826834)  # 45 deg about Z
    g = graft_yaw(q, 0.0)
    assert np.allclose(g, q, atol=1e-6)


def test_quat_mul_identity():
    q = (0.5, 0.5, 0.5, 0.5)
    assert quat_mul((1.0, 0.0, 0.0, 0.0), q) == pytest.approx(q)


def test_absolute_heading_independent_of_yaw():
    # Body-fixed mag: absolute_heading must be the same regardless of the quat's
    # (drifting) yaw, since it de-tilts with yaw stripped. This is what makes the
    # fusion reference drift-free.
    from roomscan.sensors import absolute_heading
    mag_body = (30.0, 10.0, 0.0)
    h0 = absolute_heading((1.0, 0.0, 0.0, 0.0), mag_body)          # yaw 0
    s = np.sqrt(0.5)
    h90 = absolute_heading((s, 0.0, 0.0, s), mag_body)             # yaw 90, same tilt
    assert h0 == pytest.approx(h90, abs=1e-6)


def test_fused_quat_falls_back_to_raw_without_fusion():
    st = SensorState()
    st.feed(_frame(StreamId.IMU_QUAT, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    assert st.fused_quat() == pytest.approx((1.0, 0.0, 0.0, 0.0))
    assert st.fusion_status() == "off"


def test_fused_quat_applies_yaw_correction():
    import math
    from roomscan.magcal import MagCalibration
    from roomscan.sensors import YawFusion, AXIS_CONVENTION
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    st = SensorState(fusion=YawFusion(tau_s=0.5, calibration=cal))
    target_mag = np.array([50.0 * math.cos(math.radians(60.0)), 50.0 * math.sin(math.radians(60.0)), 0.0])
    mag = tuple(AXIS_CONVENTION @ target_mag)
    for i in range(300):
        st.feed(_frame(StreamId.ENV, struct.pack("<5f", 101325.0, *mag, 20.0)))
        h = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, (i + 1) * 10_000, 0, 0, 16)
        st.feed(Frame(h, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    assert st.fusion_status() == "active"
    assert wrap180(quat_yaw_deg(st.fused_quat()) - 60.0) == pytest.approx(0.0, abs=1.5)


# ---------------------------------------------------------------------------
# ir_gravity_rot: in-plane roll snap tests
# ---------------------------------------------------------------------------

def _roll_quat(roll_deg: float) -> tuple[float, float, float, float]:
    """Quaternion representing a pure sensor roll about the depth axis.
    The sensor depth axis = SFLP body Y axis, so we rotate about Y."""
    a = math.radians(roll_deg) / 2.0
    return (math.cos(a), 0.0, math.sin(a), 0.0)  # w, x, y, z  (rot about Y)


def test_ir_gravity_rot_level():
    """Level sensor -> 0 turns."""
    from roomscan.sensors import ir_gravity_rot
    # Vertical holding = -90 roll around Y. This is the physically level (upright) orientation.
    import math
    theta = math.radians(-90.0) / 2
    q = (math.cos(theta), 0.0, math.sin(theta), 0.0)
    assert ir_gravity_rot(q) == 0


def test_ir_gravity_rot_roll_90_cw():
    """Gravity projects to image-right -> 3 CCW quarter-turns.

    Gravity sitting at image-right is 90 deg CCW of image-down on screen, so the
    content must turn 90 deg CLOCKWISE to bring it back down -- three CCW
    quarter-turns, not one. This test previously asserted 1, encoding the sign
    inversion inherited from panel.py (BUG-026 follow-up, 2026-07-29): the pane
    rotated the wrong way, so content counter-rotated at 2x the board's rate
    instead of holding still. The sign is now pinned independently, against the
    verified point-cloud path, by
    test_web.py::test_ir_gravity_angle_matches_the_point_cloud_rotation.
    """
    theta = math.radians(-90.0) / 2
    q = (math.cos(theta), math.sin(theta), 0.0, 0.0)
    assert ir_gravity_rot(q) == 3


def test_ir_gravity_rot_roll_90_ccw():
    """Gravity projects to image-left -> 1 CCW quarter-turn (was 3; see above)."""
    theta = math.radians(90.0) / 2
    q = (math.cos(theta), math.sin(theta), 0.0, 0.0)
    assert ir_gravity_rot(q) == 1


# =============================================================================
# Alternate orientation decompositions (owner ask, 2026-07-28)
# =============================================================================

def test_alt_euler_identity_is_zero():
    q = (1.0, 0.0, 0.0, 0.0)
    assert quat_roll_alt_deg(q) == pytest.approx(0.0, abs=1e-6)
    assert quat_pitch_alt_deg(q) == pytest.approx(0.0, abs=1e-6)
    assert quat_yaw_alt_deg(q) == pytest.approx(0.0, abs=1e-6)


def test_alt_euler_singularity_is_body_y_axis_not_body_x():
    """The default ZYX mode locks when body X (Up) -> world vertical
    (quat_pitch_deg -> +-90). The alt (ZXY) decomposition must NOT lock at
    that same attitude -- and must instead lock when body Y (Right) ->
    vertical, a disjoint attitude."""
    # Rotate -90 deg about Y: sends body X ([1,0,0]) to world Z ([0,0,1]).
    q_default_lock = _axis_angle_quat((0.0, 1.0, 0.0), -90.0)
    assert quat_pitch_deg(q_default_lock) == pytest.approx(-90.0, abs=1e-3)
    assert abs(quat_pitch_alt_deg(q_default_lock)) < 45.0   # alt mode nowhere near its lock

    # Rotate 90 deg about X: sends body Y ([0,1,0]) to world Z ([0,0,1]).
    q_alt_lock = _axis_angle_quat((1.0, 0.0, 0.0), 90.0)
    assert quat_pitch_alt_deg(q_alt_lock) == pytest.approx(90.0, abs=1e-3)
    assert abs(quat_pitch_deg(q_alt_lock)) < 45.0            # default mode nowhere near its lock


def test_alt_euler_recovers_a_known_rotation_away_from_lock():
    # Compose R = Rz(a) Rx(b) Ry(c) directly and confirm quat_to_matrix(q)
    # reconstructs the same matrix from the same quat, i.e. the extraction
    # functions are reading the matrix elements they claim to.
    def rz(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    def rx(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

    def ry(a):
        c, s = math.cos(a), math.sin(a)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

    a, b, c = 20.0, 10.0, -35.0
    r = rz(math.radians(a)) @ rx(math.radians(b)) @ ry(math.radians(c))
    # Build the equivalent quat via composed axis-angle multiplication.
    from roomscan.sensors import quat_mul as _mul
    qz = _axis_angle_quat((0, 0, 1), a)
    qx = _axis_angle_quat((1, 0, 0), b)
    qy = _axis_angle_quat((0, 1, 0), c)
    q = _mul(_mul(qz, qx), qy)
    assert np.allclose(quat_to_matrix(*q), r, atol=1e-5)
    assert quat_yaw_alt_deg(q) == pytest.approx(a, abs=1e-3)
    assert quat_pitch_alt_deg(q) == pytest.approx(b, abs=1e-3)
    assert quat_roll_alt_deg(q) == pytest.approx(c, abs=1e-3)


def test_boresight_view_identity_points_along_world_z():
    # Body Z (the boresight, per docs/coordinate-frames.md) = world Z at
    # identity -- elevation 90, azimuth/roll undefined-but-zero at the pole.
    az, el, roll = boresight_view_deg((1.0, 0.0, 0.0, 0.0))
    assert el == pytest.approx(90.0, abs=1e-6)


def test_boresight_view_horizontal_has_zero_elevation():
    # Rotate -90 about Y: body Z ([0,0,1]) -> world [-1,0,0] -- horizontal,
    # pointing South (azimuth 180 -- world X=North per coordinate-frames.md),
    # elevation 0, far from the singularity.
    q = _axis_angle_quat((0.0, 1.0, 0.0), -90.0)
    az, el, roll = boresight_view_deg(q)
    assert el == pytest.approx(0.0, abs=1e-3)
    assert az == pytest.approx(180.0, abs=1e-3)


def test_boresight_view_roll_tracks_twist_about_boresight():
    # Start horizontal (as above), then twist by an extra 30 deg roll about
    # the (now-horizontal) boresight -- roll should read ~30 deg while
    # azimuth/elevation stay put.
    base = _axis_angle_quat((0.0, 1.0, 0.0), -90.0)
    boresight_axis = quat_to_matrix(*base)[:, 2]
    from roomscan.sensors import quat_mul as _mul
    twist = _axis_angle_quat(boresight_axis, 30.0)
    q = _mul(twist, base)
    az, el, roll = boresight_view_deg(q)
    assert el == pytest.approx(0.0, abs=1e-2)
    assert az == pytest.approx(180.0, abs=1e-2)
    assert abs(roll) == pytest.approx(30.0, abs=1e-1)


def test_boresight_view_near_pole_roll_is_defined_as_zero_not_raising():
    az, el, roll = boresight_view_deg((1.0, 0.0, 0.0, 0.0))   # exactly at the pole
    assert roll == 0.0


# --- gravity-only (World mode) helpers --------------------------------------

def _imu_raw_batch(gravity_rows=None, accel_rows=None) -> ImuRawBatch:
    gravity = np.asarray(gravity_rows if gravity_rows is not None else [], dtype=np.float64).reshape(-1, 3)
    accel = np.asarray(accel_rows if accel_rows is not None else [], dtype=np.float64).reshape(-1, 3)
    return ImuRawBatch(
        gyro_dps=np.zeros((0, 3)), gyro_cnt=np.zeros(0, dtype=np.uint8),
        accel_g=accel, accel_cnt=np.zeros(len(accel), dtype=np.uint8),
        gravity_g=gravity, gravity_cnt=np.zeros(len(gravity), dtype=np.uint8),
        gbias_dps=np.zeros((0, 3)), gbias_cnt=np.zeros(0, dtype=np.uint8),
        timestamp_ticks=np.zeros(0, dtype=np.uint32), timestamp_cnt=np.zeros(0, dtype=np.uint8),
        n_records=len(gravity) + len(accel))


def test_sensor_state_clear_imu_raw():
    ss = SensorState()
    batch = _imu_raw_batch(gravity_rows=[[1.0, 0.0, 0.0]])
    ss._imu_raw = batch
    ss._imu_raw_hist.append((0, batch))
    assert ss.latest_imu_raw() is not None
    ss.clear_imu_raw()
    assert ss.latest_imu_raw() is None
    assert ss.imu_raw_history() == []


def test_gravity_body_from_imu_raw_none_when_no_samples():
    assert gravity_body_from_imu_raw(None) is None
    assert gravity_body_from_imu_raw(_imu_raw_batch()) is None


def test_gravity_body_from_imu_raw_averages_and_normalizes():
    # Device sitting still: SFLP gravity tag reads ~+1g "up" reaction on body
    # X (Up) -- negated, the returned down vector should point -X.
    batch = _imu_raw_batch(gravity_rows=[[1.0, 0.0, 0.0], [0.98, 0.02, 0.0]])
    down = gravity_body_from_imu_raw(batch)
    assert down is not None
    assert np.allclose(down, (-1.0, 0.0, 0.0), atol=0.02)
    assert np.linalg.norm(down) == pytest.approx(1.0, abs=1e-6)


def test_tilt_from_down_horizontal_and_vertical():
    assert tilt_from_down_deg((0.0, 1.0, 0.0), axis_body=(0.0, 0.0, 1.0)) == pytest.approx(0.0, abs=1e-6)
    assert tilt_from_down_deg((0.0, 0.0, -1.0), axis_body=(0.0, 0.0, 1.0)) == pytest.approx(90.0, abs=1e-6)
    assert tilt_from_down_deg((0.0, 0.0, 1.0), axis_body=(0.0, 0.0, 1.0)) == pytest.approx(-90.0, abs=1e-6)


def test_triad_roll_none_at_the_pole():
    # axis parallel to down -> perpendicular reference collapses.
    assert triad_roll_deg((0.0, 0.0, 1.0), axis_body=(0.0, 0.0, 1.0)) is None


def test_triad_roll_zero_when_up_ref_points_true_up():
    # Down = -Z, boresight = +Z (horizontal-ish reference), up_ref = +X:
    # true "up" is +Z, which has no component in the plane perpendicular to
    # the boresight when boresight is along Y instead -- construct a case
    # where up_ref_body is already the perpendicular projection of true up.
    down = (0.0, 1.0, 0.0)          # gravity pulls along +Y in body frame
    axis = (0.0, 0.0, 1.0)          # boresight along body Z, perpendicular to down
    up_ref = (0.0, -1.0, 0.0)       # points exactly at true "up" (-down)
    roll = triad_roll_deg(down, axis_body=axis, up_ref_body=up_ref)
    assert roll == pytest.approx(0.0, abs=1e-6)
