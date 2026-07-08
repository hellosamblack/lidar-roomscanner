import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    FLAG_DROPPED, HEADER_SIZE, MAGIC, VERSION,
    Frame, FrameHeader, FrameType, ProtocolError, StreamId, pack_frame,
)

FIXTURES = Path(__file__).parent / "fixtures"

GOLDEN_HEADER = FrameHeader(
    frame_type=FrameType.DATA, stream_id=StreamId.DEPTH_ZF32, flags=0,
    seq=7, t_us=123_456_789, width=2, height=2, payload_len=16,
)
GOLDEN_PAYLOAD = struct.pack("<4f", 1000.0, 2000.0, 0.0, 500.0)


def test_pack_frame_layout():
    frame = pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD)
    assert len(frame) == HEADER_SIZE + 16 + 4
    # hand-verifiable prefix: magic, version, type, stream, flags, seq
    assert frame[:8] == b"RSCN" + bytes([1, 1, 0, 0])
    assert frame[8:12] == (7).to_bytes(4, "little")
    assert frame[20:24] == (2).to_bytes(2, "little") + (2).to_bytes(2, "little")
    # CRC over everything before it
    assert frame[-4:] == zlib.crc32(frame[:-4]).to_bytes(4, "little")


def test_header_roundtrip():
    frame = pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD)
    hdr = FrameHeader.unpack(frame[:HEADER_SIZE])
    assert hdr == GOLDEN_HEADER


def test_unpack_rejects_bad_magic():
    frame = bytearray(pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD))
    frame[0] = 0x00
    with pytest.raises(ProtocolError):
        FrameHeader.unpack(bytes(frame[:HEADER_SIZE]))


def test_unpack_rejects_bad_version():
    frame = bytearray(pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD))
    frame[4] = 99
    with pytest.raises(ProtocolError):
        FrameHeader.unpack(bytes(frame[:HEADER_SIZE]))


def test_golden_fixture_matches_pack():
    golden = (FIXTURES / "golden_depth_2x2.bin").read_bytes()
    assert pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD) == golden


def test_parse_event_roundtrip():
    payload = struct.pack("<II", 2, 3) + b"trigger retries exhausted"
    from roomscan.protocol import EventCode, parse_event
    code, detail, msg = parse_event(payload)
    assert code == EventCode.TRIGGER_TIMEOUT
    assert detail == 3
    assert msg == "trigger retries exhausted"


def test_parse_event_rejects_short_payload():
    from roomscan.protocol import parse_event
    with pytest.raises(ProtocolError):
        parse_event(b"\x01\x00\x00")


def test_raw_and_calib_stream_ids():
    from roomscan.protocol import CALIB_SIZE, RAW_3DMD_SIZE_BIN2, StreamId
    assert StreamId.RAW_3DMD == 7
    assert StreamId.CALIB == 8
    assert RAW_3DMD_SIZE_BIN2 == 14842
    assert CALIB_SIZE == 2332
