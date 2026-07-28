"""Regenerate golden fixtures from raw struct calls — deliberately NOT via roomscan.protocol,
so a bug in protocol.py cannot hide inside its own fixture."""
import struct
import zlib
from pathlib import Path

FIXTURES = Path(__file__).parent / "fixtures"


def golden_depth_2x2() -> bytes:
    header = struct.pack("<4sBBBBIQHHII", b"RSCN", 1, 1, 0, 0, 7, 123_456_789, 2, 2, 16, 0)
    payload = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)
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
    (FIXTURES / "golden_imu_raw.bin").write_bytes(golden_imu_raw())
    build_sensors_snippet(FIXTURES / "golden_sensors_snippet.bin")
    print("fixtures written")
