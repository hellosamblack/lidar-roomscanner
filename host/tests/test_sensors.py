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
    quat_to_matrix,
    tilt_compensated_heading,
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
