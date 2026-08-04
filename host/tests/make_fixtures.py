"""Regenerate golden fixtures from raw struct calls — deliberately NOT via roomscan.protocol,
so a bug in protocol.py cannot hide inside its own fixture."""
import struct
import zlib
from pathlib import Path

FIXTURES = Path(__file__).parent / "fixtures"


def golden_depth_2x2() -> bytes:
    """A frozen v1 capture vector -- DO NOT bump the hardcoded version byte (1) here even
    though the host now defaults to v2. This fixture's job is to prove an old v1 recording
    still decodes after the protocol moved on (docs/protocol.md "Version history" v2 entry,
    host/tests/test_protocol.py's v1-compat tests)."""
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 0, 0, 7, 123_456_789, 2, 2, 16, 0)
    payload = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_depth_2x2_v2() -> bytes:
    """Same DATA frame as golden_depth_2x2(), but with the v2 version byte -- what
    pack_frame() produces today (VERSION == 2). The header/payload/CRC layout is
    otherwise byte-for-byte identical; only offset 4 (version) differs from the v1
    fixture above."""
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 2, 1, 0, 0, 7, 123_456_789, 2, 2, 16, 0)
    payload = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_command_manual() -> bytes:
    """One v2 SET_MANUAL_PARAMS (cmd 9) COMMAND frame, hand-packed independently of
    roomscan.protocol -- the golden-vector point. Payload: cmd(9) + ranging_mode(1 =
    PRECISION) + frame_period_us(11111, ~90 FPS) + exposure_ms(4) + power_mode(2 =
    REGULAR), 12 bytes, no padding."""
    payload = struct.pack("<IBIHB", 9, 1, 11111, 4, 2)
    # header: COMMAND(3), stream_id=0, seq=token(55), t_us=0, w=h=0, payload_len=12
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 2, 3, 0, 0, 55, 0, 0, 0, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_ack_ranging_config() -> bytes:
    """One v2 extended ACK (16-byte ranging-config shape) for cmd 10 (GET_RANGING_CONFIG),
    hand-packed independently of roomscan.protocol. Payload: cmd(10) + result(0 = OK) +
    ranging_mode(0 = AMBIENT) + frame_period_us(33333, ~30 FPS) + exposure_ms(6) +
    power_mode(0 = ULP), 16 bytes, no padding."""
    payload = struct.pack("<IIBIHB", 10, 0, 0, 33333, 6, 0)
    # header: ACK(4), stream_id=0, seq=token(55), t_us=0, w=h=0, payload_len=16
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 2, 4, 0, 0, 55, 0, 0, 0, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_imu_raw() -> bytes:
    """One stream-11 (IMU_RAW) DATA frame carrying all five FIFO tag types.

    Records are 8 B: FIFO_DATA_OUT_TAG register byte (TAG_SENSOR << 3 | TAG_CNT << 1),
    6 verbatim data bytes, 1 reserved zero. Built from raw struct calls on purpose —
    the tag/scale constants are re-stated here so a bug in protocol.py cannot hide.
    """
    def rec(tag_sensor, tag_cnt, data6):
        assert len(data6) == 6
        return bytes([(tag_sensor << 3) | (tag_cnt << 1)]) + data6 + b"\x00"

    records = b"".join([
        # GY_NC, TAG_CNT=0: +1000, -2000, +3 LSB -> 17.5, -35.0, 0.0525 dps
        rec(0x01, 0, struct.pack("<3h", 1000, -2000, 3)),
        # XL_NC, TAG_CNT=0: 0, 8192, -8192 LSB -> 0, ~0.99942 g, ~-0.99942 g
        rec(0x02, 0, struct.pack("<3h", 0, 8192, -8192)),
        # TIMESTAMP, TAG_CNT=1: tick 0x0000BEEF in the first 4 bytes, BDR meta in the last 2
        rec(0x04, 1, struct.pack("<IH", 0x0000BEEF, 0x1234)),
        # SFLP gyro bias, TAG_CNT=2: 100, -100, 0 LSB -> 0.4375, -0.4375, 0 dps
        rec(0x16, 2, struct.pack("<3h", 100, -100, 0)),
        # SFLP gravity, TAG_CNT=3: 0, 0, 16384 LSB -> 0, 0, ~0.99942 g
        rec(0x17, 3, struct.pack("<3h", 0, 0, 16384)),
    ])
    # header: DATA(1), stream 11, seq 42, t_us 1_000_000, width = record count, height = 0
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 11, 0, 42, 1_000_000,
                         len(records) // 8, 0, len(records), 0)
    return header + records + zlib.crc32(header + records).to_bytes(4, "little")


def golden_imu_cal() -> bytes:
    """One stream-12 (IMU_CAL) DATA frame: the LSM's INTERNAL_FREQ_FINE trim.

    Payload is int8 freq_fine, uint8 valid, uint16 reserved(0). freq_fine = -23 is picked
    to be negative (exercising the two's-complement decode) and to land near the ~2.98%
    host-vs-LSM scale error measured on this rig: 1/(46080 * (1 + 0.0013 * -23)) gives
    22.370 µs/tick against the nominal 21.7, i.e. +3.09%.
    """
    payload = struct.pack("<bBH", -23, 1, 0)
    # header: DATA(1), stream 12, seq 42, t_us 1_000_000, width = height = 0
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 12, 0, 42, 1_000_000,
                         0, 0, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_imu_sync() -> bytes:
    """One stream-13 (IMU_SYNC) DATA frame: the frame-ready edge on the LSM clock.

    Payload is u32 lsm_ticks, u32 latch_delay_us, u32 drain_delay_us, u32 quat_mid_ticks,
    u16 read_us, u16 quat_n, u8 valid, u8 reserved(0). The values are the shape a real rig
    produces: a large free-running tick counter, a latch 61 µs after the edge, a drain
    24.1 ms later (the gap that made the old FIFO-word inference load-dependent, BUG-031),
    and a quaternion midpoint 348 ticks (7.76 ms at this part's tick) PAST the edge — the
    averaged batch leads the depth frame, it does not lag it.
    """
    payload = struct.pack("<IIIIHHBB", 3_931_420_041, 61, 24_109, 3_931_420_386, 44, 15, 1, 0)
    # header: DATA(1), stream 13, seq 42, t_us 1_000_000, width = height = 0
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 13, 0, 42, 1_000_000,
                         0, 0, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def golden_tx_queue_stats() -> bytes:
    """One EVENT (frame_type=2) code 7 TX_QUEUE_STATS frame (Task 6,
    docs/protocol.md "TX_QUEUE_STATS (EVENT code 7) payload layout"; firmware
    rs_send_tx_queue_stats_event() in firmware/scanner-stream/Src/vl53l9_app.c).

    Payload (20 B, <IIIII> LE): code(7), packed `detail` (queue_high_water @
    bits 0-7, active_transport @ bits 8-15, pending_fragments @ bits 16-31),
    enqueue_drops, stack_stalls, emitted_bytes. Values are picked to exercise
    every field distinctly and to NOT be all-zero/all-equal (a transposition
    of two same-width fields must be detectable):
      queue_high_water   = 200   (0xC8, near the u8 ceiling)
      active_transport   = 2     (udp)
      pending_fragments  = 1000  (0x3E8, exercises the 16-bit sub-field)
      enqueue_drops      = 7
      stack_stalls       = 3
      emitted_bytes      = 123_456_789
    detail = 200 | (2 << 8) | (1000 << 16) = 0x03E802C8 (65_536_712).

    EVENT frames carry stream_id=0, width=height=0, and `seq` = the last
    captured frame's counter (not incremented by the EVENT itself, per
    docs/protocol.md's EVENT section) -- 4096 here (a 64-frame-cadence
    multiple, matching the real emission cadence).
    """
    high_water, active_transport, pending = 200, 2, 1000
    enqueue_drops, stack_stalls, emitted_bytes = 7, 3, 123_456_789
    detail = (high_water & 0xFF) | ((active_transport & 0xFF) << 8) | ((pending & 0xFFFF) << 16)
    assert detail == 0x03E802C8
    payload = struct.pack("<IIIII", 7, detail, enqueue_drops, stack_stalls, emitted_bytes)
    assert len(payload) == 20
    # header: EVENT(2), stream_id=0, seq=4096 (last captured frame's counter),
    # t_us=987_654_321, width=height=0
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 2, 2, 0, 0, 4096, 987_654_321,
                         0, 0, len(payload), 0)
    return header + payload + zlib.crc32(header + payload).to_bytes(4, "little")


def build_sensors_snippet(path):
    """A tiny capture: CALIB, then N (RAW, IMU_QUAT, ENV) triples with a rotating quaternion."""
    import numpy as np
    from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

    frames = []

    def data(stream_id, payload, seq, t_us):
        h = FrameHeader(FrameType.DATA, stream_id, 0, seq, t_us, 0, 0, len(payload))
        return pack_frame(h, payload)

    frames.append(data(StreamId.CALIB, b"\x00" * 2332, 1, 0))
    for i in range(8):
        ang = np.radians(i * 10.0)
        w, z = float(np.cos(ang / 2)), float(np.sin(ang / 2))
        raw = bytes(14842)
        frames.append(data(StreamId.RAW_3DMD, raw, i + 1, i * 35000))
        frames.append(data(StreamId.IMU_QUAT, __import__("struct").pack("<4f", w, 0.0, 0.0, z), i + 1, i * 35000))
        frames.append(data(StreamId.ENV, __import__("struct").pack("<5f", 101325.0 + i, 1.0, 0.0, 0.0, 21.0 + 0.1 * i), i + 1, i * 35000))
    with open(path, "wb") as f:
        f.write(b"".join(frames))


if __name__ == "__main__":
    FIXTURES.mkdir(exist_ok=True)
    (FIXTURES / "golden_depth_2x2.bin").write_bytes(golden_depth_2x2())
    (FIXTURES / "golden_depth_2x2_v2.bin").write_bytes(golden_depth_2x2_v2())
    (FIXTURES / "golden_command_manual.bin").write_bytes(golden_command_manual())
    (FIXTURES / "golden_ack_ranging_config.bin").write_bytes(golden_ack_ranging_config())
    (FIXTURES / "golden_imu_raw.bin").write_bytes(golden_imu_raw())
    (FIXTURES / "golden_imu_cal.bin").write_bytes(golden_imu_cal())
    (FIXTURES / "golden_imu_sync.bin").write_bytes(golden_imu_sync())
    (FIXTURES / "golden_tx_queue_stats.bin").write_bytes(golden_tx_queue_stats())
    build_sensors_snippet(FIXTURES / "golden_sensors_snippet.bin")
    print("fixtures written")
