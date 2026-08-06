import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    ENV_SIZE,
    HEADER_SIZE,
    IMU_CAL_SIZE,
    IMU_QUAT_SIZE,
    IMU_RAW_REC_SIZE,
    IMU_RAW_TICK_US,
    IMU_SYNC_SIZE,
    FrameHeader,
    FrameType,
    ImuFifoTag,
    ProtocolError,
    StreamId,
    decode_env,
    decode_imu_cal,
    decode_imu_quat,
    decode_imu_raw,
    decode_imu_sync,
    decode_tof_meta,
    imu_tick_us,
)
from roomscan.protocol import TOF_META_SIZE


def _u24(v):
    return bytes((v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF))


def _make_tof_meta(*, frame_counter=42, die_temp=35, nb_step=7, binning=2, dss=2,
                   power=1, sync=2, frame_period_us=33333, step1=1170, step6=14430,
                   error_status=0, error_code=0):
    """Build a synthetic 100-byte vl53l9_meta_t block for testing."""
    m = bytearray(TOF_META_SIZE)
    m[0:4] = struct.pack("<I", frame_counter)
    m[4:6] = struct.pack("<H", die_temp)
    m[36:52] = struct.pack("<8H", 100, 200, 300, 400, 500, 600, 700, 800)
    m[52:54] = struct.pack("<H", 54)
    m[54:56] = struct.pack("<H", 42)
    m[56] = (sync & 0x3) | ((power & 0x3) << 2)
    m[57] = 4  # ambient_attenuation
    m[58:60] = struct.pack("<H", ((dss & 0x3) << 4) | ((binning & 0x1F) << 6) | ((nb_step & 0xF) << 12))
    m[60:62] = struct.pack("<H", error_code)
    m[62] = error_status
    m[68:72] = struct.pack("<I", frame_period_us)
    m[76:79] = _u24(step1)
    m[82:85] = _u24(step6)
    return bytes(m)


def test_decode_tof_meta_precision_exposure():
    # precision: step1=117*x, step6=1443*x -> x=10 ms for 1170/14430.
    md = decode_tof_meta(_make_tof_meta(nb_step=7, step1=1170, step6=14430))
    assert md["ranging_mode"] == "precision"
    assert md["exposure_ms"] == pytest.approx(10.0)          # value, not just type
    assert md["exposure_consistent"] is True
    assert md["die_temp_c"] == 35
    assert md["frame_counter"] == 42
    assert md["dss_mode"] == 2 and md["binning"] == 2 and md["nb_step"] == 7
    assert md["power_mode"] == 1
    assert md["fps"] == pytest.approx(30.0, abs=0.01)
    assert md["error_status"] == 0 and md["error_code"] == 0
    assert md["ref_channels"] == [100, 200, 300, 400, 500, 600, 700, 800]


def test_decode_tof_meta_ambient_exposure():
    # ambient: step1=231*x, step6=1418*x -> x=8 ms for 1848/11344.
    md = decode_tof_meta(_make_tof_meta(nb_step=6, step1=1848, step6=11344))
    assert md["ranging_mode"] == "ambient"
    assert md["exposure_ms"] == pytest.approx(8.0)
    assert md["exposure_consistent"] is True


def test_decode_tof_meta_reads_tail_of_full_payload():
    # decode must slice the last 100 bytes of a full RAW_3DMD payload.
    meta = _make_tof_meta(die_temp=37)
    md = decode_tof_meta(b"\x00" * 14742 + meta)
    assert md["die_temp_c"] == 37


def test_decode_tof_meta_too_short():
    assert decode_tof_meta(b"\x00" * 50) is None
    assert decode_tof_meta(None) is None


def test_stream_ids():
    assert StreamId.IMU_QUAT == 9
    assert StreamId.ENV == 10
    assert StreamId.IMU_CAL == 12
    assert StreamId.IMU_SYNC == 13


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


# --- stream 12 (IMU_CAL) -----------------------------------------------------

def test_imu_tick_us_formula():
    """AN5763 6.4: t = 1 / (46080 * (1 + 0.0013 * FREQ_FINE))."""
    assert imu_tick_us(0) == pytest.approx(1e6 / 46080.0)       # nominal ~21.70 µs
    assert imu_tick_us(0) == pytest.approx(IMU_RAW_TICK_US, abs=0.01)
    assert imu_tick_us(-23) == pytest.approx(22.3703, abs=1e-3)  # slow part -> longer tick
    assert imu_tick_us(23) == pytest.approx(21.0714, abs=1e-3)   # fast part -> shorter tick


def test_decode_imu_cal_signed_and_valid():
    cal = decode_imu_cal(struct.pack("<bBH", -23, 1, 0))
    assert cal.freq_fine == -23 and cal.valid is True
    assert cal.tick_us == pytest.approx(imu_tick_us(-23))
    # a 2.98% scale error is exactly the class of bug this stream exists to kill
    assert cal.tick_us / IMU_RAW_TICK_US == pytest.approx(1.0306, abs=1e-3)


def test_decode_imu_cal_invalid_falls_back_to_nominal():
    cal = decode_imu_cal(struct.pack("<bBH", -23, 0, 0))
    assert cal.valid is False
    assert cal.tick_us == IMU_RAW_TICK_US


def test_decode_imu_cal_bad_length():
    with pytest.raises(ProtocolError):
        decode_imu_cal(b"\x00" * 3)


def test_golden_imu_cal_fixture_matches_decoder():
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_cal.bin").read_bytes()
    assert zlib.crc32(golden[:-4]) == int.from_bytes(golden[-4:], "little")
    header = FrameHeader.unpack(golden[:HEADER_SIZE])
    assert header.stream_id == StreamId.IMU_CAL
    assert header.width == header.height == 0
    assert header.payload_len == IMU_CAL_SIZE
    cal = decode_imu_cal(golden[HEADER_SIZE:HEADER_SIZE + header.payload_len])
    assert cal.freq_fine == -23 and cal.valid is True
    assert cal.tick_us == pytest.approx(22.3703, abs=1e-3)


def test_golden_imu_cal_fixture_is_reproducible():
    from make_fixtures import golden_imu_cal
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_cal.bin").read_bytes()
    assert golden_imu_cal() == golden


# --- stream 13 (IMU_SYNC) ----------------------------------------------------

def test_decode_imu_sync_fields():
    payload = struct.pack("<IIIIHHBB", 3_931_420_041, 61, 24_109, 3_931_420_386, 44, 15, 1, 0)
    assert len(payload) == IMU_SYNC_SIZE
    s = decode_imu_sync(payload)
    assert s.lsm_ticks == 3_931_420_041
    assert s.latch_delay_us == 61 and s.drain_delay_us == 24_109
    assert s.quat_mid_ticks == 3_931_420_386 and s.quat_n == 15
    assert s.read_us == 44 and s.valid is True


def test_imu_sync_frame_ready_ticks_backs_out_the_latch_delay():
    """The edge is the latch minus its own delay, converted with the stream-12 tick."""
    s = decode_imu_sync(struct.pack("<IIIIHHBB", 1000, 217, 3000, 0, 40, 0, 1, 0))
    # 217 µs at the nominal 21.7 µs/tick is exactly 10 ticks before the latch
    assert s.frame_ready_ticks(IMU_RAW_TICK_US) == pytest.approx(990.0)
    # a slower part's tick makes the same delay fewer ticks
    assert s.frame_ready_ticks(imu_tick_us(-23)) == pytest.approx(1000 - 217 / 22.3703, abs=1e-3)


def test_decode_imu_sync_bad_length():
    with pytest.raises(ProtocolError):
        decode_imu_sync(b"\x00" * 12)


def test_golden_imu_sync_fixture_matches_decoder():
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_sync.bin").read_bytes()
    assert zlib.crc32(golden[:-4]) == int.from_bytes(golden[-4:], "little")
    header = FrameHeader.unpack(golden[:HEADER_SIZE])
    assert header.stream_id == StreamId.IMU_SYNC
    assert header.width == header.height == 0
    assert header.payload_len == IMU_SYNC_SIZE
    s = decode_imu_sync(golden[HEADER_SIZE:HEADER_SIZE + header.payload_len])
    assert s.lsm_ticks == 3_931_420_041 and s.valid is True
    assert s.latch_delay_us == 61 and s.drain_delay_us == 24_109 and s.read_us == 44
    # the averaged stream-9 quat LEADS the frame-ready edge; a negative sign here would
    # send any correction built on it the wrong way (see BUG-031 / the ODR costing note)
    assert s.quat_offset_us(imu_tick_us(-23)) == pytest.approx(7778.7, abs=1.0)


def test_golden_imu_sync_fixture_is_reproducible():
    from make_fixtures import golden_imu_sync
    golden = (Path(__file__).parent / "fixtures" / "golden_imu_sync.bin").read_bytes()
    assert golden_imu_sync() == golden


def test_sensor_state_applies_imu_cal_tick_to_later_batches():
    from roomscan.protocol import Frame
    from roomscan.sensors import SensorState

    st = SensorState()
    ts = _rec(ImuFifoTag.TIMESTAMP, 0, struct.pack("<IH", 1000, 0))

    # before any stream 12: nominal tick, i.e. exactly how a pre-2026-07-28 capture decodes
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_RAW, 0, 1, 0, 1, 0, len(ts)), ts))
    assert st.imu_tick_us == IMU_RAW_TICK_US
    assert st.latest_imu_raw().timestamp_us[0] == pytest.approx(1000 * IMU_RAW_TICK_US)

    cal = struct.pack("<bBH", -23, 1, 0)
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_CAL, 0, 1, 0, 0, 0, len(cal)), cal))
    assert st.imu_tick_us == pytest.approx(imu_tick_us(-23))

    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_RAW, 0, 2, 0, 1, 0, len(ts)), ts))
    assert st.latest_imu_raw().timestamp_us[0] == pytest.approx(1000 * imu_tick_us(-23))


def test_imu_cal_does_not_disturb_stream_9():
    """Stream 12 must be inert with respect to the orientation path."""
    from roomscan.protocol import Frame
    from roomscan.sensors import SensorState

    st = SensorState()
    quat = struct.pack("<4f", 1.0, 0.0, 0.0, 0.0)
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, 1, 0, 0, 0, len(quat)), quat))
    before = st.fused_quat()
    cal = struct.pack("<bBH", -23, 1, 0)
    st.feed(Frame(FrameHeader(FrameType.DATA, StreamId.IMU_CAL, 0, 1, 0, 0, 0, len(cal)), cal))
    assert st.fused_quat() == before
