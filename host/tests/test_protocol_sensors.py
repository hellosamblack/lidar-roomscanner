import struct

import pytest

from roomscan.protocol import (
    ENV_SIZE,
    IMU_QUAT_SIZE,
    ProtocolError,
    StreamId,
    decode_env,
    decode_imu_quat,
)


def test_stream_ids():
    assert StreamId.IMU_QUAT == 9
    assert StreamId.ENV == 10


def test_decode_imu_quat_roundtrip():
    payload = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)  # identity [w, x, y, z]
    assert len(payload) == IMU_QUAT_SIZE
    w, x, y, z = decode_imu_quat(payload)
    assert (w, x, y, z) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_decode_imu_quat_bad_length():
    with pytest.raises(ProtocolError):
        decode_imu_quat(b"\x00" * 12)


def test_decode_env_roundtrip():
    payload = struct.pack("<5f", 101325.0, 12.0, -34.0, 56.0, 21.5)
    assert len(payload) == ENV_SIZE
    pressure, mag, temp = decode_env(payload)
    assert pressure == pytest.approx(101325.0)
    assert mag == pytest.approx((12.0, -34.0, 56.0))
    assert temp == pytest.approx(21.5)


def test_decode_env_bad_length():
    with pytest.raises(ProtocolError):
        decode_env(b"\x00" * 16)
