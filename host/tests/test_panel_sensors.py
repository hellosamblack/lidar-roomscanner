import struct

import numpy as np
import pytest

from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from roomscan.sensors import SensorState, gizmo_pose, tilt_compensated_heading
from roomscan.sensors_widgets import render_compass, render_sparkline


def _env_frame(pressure, mag, temp):
    payload = struct.pack("<5f", pressure, *mag, temp)
    return Frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, 1, 0, 0, 0, len(payload)), payload)


def test_panel_sensor_tick_pipeline():
    # Simulate the reader->UI seam without a GUI: feed frames, compute what the tick would draw.
    st = SensorState()
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0, 16),
                  struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    st.feed(_env_frame(101325.0, (1.0, 0.0, 0.0), 21.0))

    quat = st.latest_quat()
    assert quat is not None
    pose = gizmo_pose(quat, 0.15, (0.0, 0.0, 0.0))
    assert pose.shape == (4, 4)

    env = st.latest_env()
    heading = tilt_compensated_heading(quat, env.mag_ut)
    compass = render_compass(heading)
    spark = render_sparkline(st.pressure_history())
    assert compass.ndim == 3 and spark.ndim == 3


def test_panel_graceful_no_data():
    st = SensorState()
    assert st.latest_quat() is None
    assert st.latest_env() is None
    # empty history renders without error
    assert render_sparkline(st.pressure_history()).shape[2] == 3


def test_fused_quat_seam_uses_correction():
    import math
    from roomscan.magcal import MagCalibration
    from roomscan.sensors import SensorState, YawFusion, gizmo_pose, quat_yaw_deg, wrap180, AXIS_CONVENTION
    cal = MagCalibration(offset=(0.0, 0.0, 0.0),
                         matrix=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                         field_ut=50.0)
    st = SensorState(fusion=YawFusion(tau_s=0.5, calibration=cal))
    target_mag = np.array([50.0 * math.cos(math.radians(60.0)), 50.0 * math.sin(math.radians(60.0)), 0.0])
    mag = tuple(AXIS_CONVENTION @ target_mag)
    for i in range(300):
        st.feed(_env_frame(101325.0, mag, 20.0))
        h = FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, (i + 1) * 10_000, 0, 0, 16)
        st.feed(Frame(h, struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)))
    quat = st.fused_quat()                    # what the tick now draws
    assert wrap180(quat_yaw_deg(quat) - 60.0) == pytest.approx(0.0, abs=1.5)
    pose = gizmo_pose(quat, 0.15, (0.0, 0.0, 0.0))
    assert pose.shape == (4, 4)
