import struct

import numpy as np
import pytest

from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId
from tools.mag_calibrate import collect_mag_from_frames, calibrate


def _env_frame(mag):
    payload = struct.pack("<5f", 101325.0, *mag, 20.0)
    return Frame(FrameHeader(FrameType.DATA, StreamId.ENV, 0, 1, 0, 0, 0, len(payload)), payload)


def test_collect_pulls_mag_vectors():
    frames = [_env_frame((1.0, 2.0, 3.0)), _env_frame((4.0, 5.0, 6.0))]
    out = collect_mag_from_frames(frames)
    assert out.shape == (2, 3)
    assert np.allclose(out[1], [4.0, 5.0, 6.0])


def test_collect_ignores_non_env_frames():
    quat = Frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0, 16),
                 struct.pack("<4f", 1.0, 0.0, 0.0, 0.0))
    out = collect_mag_from_frames([quat, _env_frame((7.0, 8.0, 9.0))])
    assert out.shape == (1, 3)


def test_calibrate_writes_json_and_spherizes(tmp_path):
    rng = np.random.default_rng(1)
    dirs = rng.normal(size=(400, 3))
    dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
    raw = dirs * 45.0 @ np.array([[1.2, 0.0, 0.0], [0.0, 0.9, 0.0], [0.0, 0.0, 1.05]]).T + [4.0, -2.0, 1.0]
    out = tmp_path / "mag_cal.json"
    cal = calibrate(raw, out)
    assert out.exists()
    norms = np.linalg.norm(np.array([cal.apply(r) for r in raw]), axis=1)
    assert np.std(norms) / np.mean(norms) < 0.02
