import struct

import numpy as np

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
