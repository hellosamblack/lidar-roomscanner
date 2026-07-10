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
    build_sensors_snippet(FIXTURES / "golden_sensors_snippet.bin")
    print("fixtures written")
