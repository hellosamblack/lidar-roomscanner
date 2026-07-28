import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    ENV_SIZE,
    HEADER_SIZE,
    IMU_QUAT_SIZE,
    IMU_RAW_REC_SIZE,
    IMU_RAW_TICK_US,
    FrameHeader,
    FrameType,
    ImuFifoTag,
    ProtocolError,
    StreamId,
    decode_env,
    decode_imu_quat,
    decode_imu_raw,
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


# --- stream 11 (IMU_RAW) -----------------------------------------------------

def _rec(tag_sensor: int, tag_cnt: int, data6: bytes) -> bytes:
    """Build one 8-byte IMU_RAW record the way the firmware does."""
    return bytes([(tag_sensor << 3) | (tag_cnt << 1)]) + data6 + b"\x00"


def test_imu_raw_stream_id_and_record_size():
    assert StreamId.IMU_RAW == 11
    assert IMU_RAW_REC_SIZE == 8


def test_decode_imu_raw_all_tags():
    payload = b"".join([
        _rec(ImuFifoTag.GY_NC, 0, struct.pack("<3h", 1000, -2000, 3)),
        _rec(ImuFifoTag.XL_NC, 0, struct.pack("<3h", 0, 8192, -8192)),
        _rec(ImuFifoTag.TIMESTAMP, 1, struct.pack("<IH", 0x0000BEEF, 0x1234)),
        _rec(ImuFifoTag.SFLP_GBIAS, 2, struct.pack("<3h", 100, -100, 0)),
        _rec(ImuFifoTag.SFLP_GRAVITY, 3, struct.pack("<3h", 0, 0, 16384)),
    ])
    b = decode_imu_raw(payload)

    assert b.n_records == 5
    assert b.gyro_dps.shape == (1, 3)
    assert b.gyro_dps[0] == pytest.approx([17.5, -35.0, 0.0525])
    assert b.accel_g[0] == pytest.approx([0.0, 8192 * 0.000122, -8192 * 0.000122])
    assert b.gbias_dps[0] == pytest.approx([0.4375, -0.4375, 0.0])
    assert b.gravity_g[0] == pytest.approx([0.0, 0.0, 16384 * 0.000061])
    assert b.timestamp_ticks.tolist() == [0x0000BEEF]
    assert b.timestamp_us[0] == pytest.approx(0x0000BEEF * IMU_RAW_TICK_US)


def test_decode_imu_raw_preserves_tag_cnt():
    """TAG_CNT is what groups words into sample times — it must survive the wire."""
    payload = b"".join([
        _rec(ImuFifoTag.GY_NC, 2, struct.pack("<3h", 1, 2, 3)),
        _rec(ImuFifoTag.XL_NC, 2, struct.pack("<3h", 4, 5, 6)),
        _rec(ImuFifoTag.GY_NC, 3, struct.pack("<3h", 7, 8, 9)),
        _rec(ImuFifoTag.TIMESTAMP, 3, struct.pack("<IH", 7, 0)),
    ])
    b = decode_imu_raw(payload)
    assert b.gyro_cnt.tolist() == [2, 3]
    assert b.accel_cnt.tolist() == [2]
    assert b.timestamp_cnt.tolist() == [3]


def test_decode_imu_raw_skips_foreign_tags():
    """Stream 9 (game rotation, 0x13) and stream 10 (sensor hub) tags never ride stream 11,
    but an unknown tag must be counted and otherwise ignored, not crash the decoder."""
    payload = _rec(0x13, 0, b"\x01\x02\x03\x04\x05\x06") + _rec(ImuFifoTag.GY_NC, 0,
                                                               struct.pack("<3h", 1, 1, 1))
    b = decode_imu_raw(payload)
    assert b.n_records == 2
    assert b.gyro_dps.shape == (1, 3)
    assert b.gravity_g.shape == (0, 3)


def test_decode_imu_raw_bad_length():
    with pytest.raises(ProtocolError):
        decode_imu_raw(b"\x00" * 12)      # not a multiple of 8
    with pytest.raises(ProtocolError):
        decode_imu_raw(b"")               # empty batch is never sent


def test_golden_imu_raw_fixture_matches_decoder():
    """Cross-check: the fixture is built by make_fixtures.py from raw struct/zlib calls,
    independently of protocol.py's codec."""
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_raw.bin").read_bytes()
    assert zlib.crc32(golden[:-4]) == int.from_bytes(golden[-4:], "little")
    header = FrameHeader.unpack(golden[:HEADER_SIZE])
    assert header.stream_id == StreamId.IMU_RAW
    assert header.height == 0
    payload = golden[HEADER_SIZE:HEADER_SIZE + header.payload_len]
    b = decode_imu_raw(payload)
    # w carries the record count (stream-11 convention, docs/protocol.md)
    assert header.width == b.n_records == 5
    assert header.payload_len == b.n_records * IMU_RAW_REC_SIZE
    assert b.gyro_dps[0] == pytest.approx([17.5, -35.0, 0.0525])
    assert b.timestamp_ticks.tolist() == [0x0000BEEF]
    assert b.gravity_cnt.tolist() == [3]


def test_golden_imu_raw_fixture_is_reproducible():
    from make_fixtures import golden_imu_raw
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_raw.bin").read_bytes()
    assert golden_imu_raw() == golden


def test_sensor_state_feeds_imu_raw_without_touching_fused_quat():
    from roomscan.protocol import Frame, pack_frame
    from roomscan.sensors import SensorState

    st = SensorState()
    quat = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0, len(quat)), quat))
    before = st.fused_quat()

    payload = _rec(ImuFifoTag.GY_NC, 0, struct.pack("<3h", 10, 20, 30))
    raw = FrameHeader(FrameType.DATA, StreamId.IMU_RAW, 0, 1, 500, 1, 0, len(payload))
    st.feed(Frame(raw, payload))

    batch = st.latest_imu_raw()
    assert batch is not None and batch.n_records == 1
    assert [t for t, _ in st.imu_raw_history()] == [500]
    assert st.fused_quat() == before          # stream 11 must not perturb stream 9's path
    assert pack_frame(raw, payload)[:4] == b"RSCN"
