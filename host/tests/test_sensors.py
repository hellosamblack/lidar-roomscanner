import struct

import numpy as np
import pytest

from roomscan.protocol import (
    Frame,
    FrameHeader,
    FrameType,
    StreamId,
)
from roomscan.sensors import (
    SensorState,
    graft_yaw,
    quat_mul,
    quat_pitch_deg,
    quat_to_matrix,
    quat_yaw_deg,
    tilt_compensated_heading,
    wrap180,
)


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
    # scale on the diagonal of the rotation block
    assert m[0, 0] == pytest.approx(0.2)
    # translation column
    assert np.allclose(m[:3, 3], [1.0, 2.0, 3.0])
    assert m[3, 3] == pytest.approx(1.0)


def test_gizmo_pose_yaw_maps_to_y_axis_rotation():
    # 30 degree yaw (around IMU Z axis)
    import math
    from roomscan.sensors import gizmo_pose
    theta = math.radians(30.0) / 2
    quat = (math.cos(theta), 0.0, 0.0, math.sin(theta))
    m = gizmo_pose(quat, scale=1.0, anchor=(0.0, 0.0, 0.0))
    
    # Should correspond to a rotation around the visualizer Y axis:
    # [[ cos(30), 0, sin(30) ],
    #  [ 0,       1, 0       ],
    #  [ -sin(30),0, cos(30) ]]
    expected = np.array([
        [math.cos(math.radians(30.0)), 0.0, math.sin(math.radians(30.0))],
        [0.0, 1.0, 0.0],
        [-math.sin(math.radians(30.0)), 0.0, math.cos(math.radians(30.0))]
    ])
    assert np.allclose(m[:3, :3], expected, atol=1e-4)


def test_wrap180():
    assert wrap180(190.0) == pytest.approx(-170.0)
    assert wrap180(-190.0) == pytest.approx(170.0)
    assert wrap180(30.0) == pytest.approx(30.0)


def test_quat_yaw_of_z_rotation():
    s = np.sqrt(0.5)  # 90 deg about +Z
    assert quat_yaw_deg((s, 0.0, 0.0, s)) == pytest.approx(90.0, abs=1e-4)


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
