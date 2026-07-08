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


if __name__ == "__main__":
    FIXTURES.mkdir(exist_ok=True)
    (FIXTURES / "golden_depth_2x2.bin").write_bytes(golden_depth_2x2())
    print("fixtures written")
