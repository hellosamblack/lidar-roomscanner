import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    FLAG_DROPPED, HEADER_SIZE, MAGIC, VERSION,
    Frame, FrameHeader, FrameType, ProtocolError, StreamId, pack_frame,
    CommandCode, ResultCode, pack_command, parse_ack,
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


def test_pack_command_golden():
    """Golden bytes test for pack_command: PING with token=42."""
    frame = pack_command(CommandCode.PING, 0, token=42)
    # hand-verifiable prefix: magic, version, type=3, stream=0, flags=0, seq=42
    assert frame[:4] == b"RSCN"
    assert frame[4:5] == bytes([VERSION])
    assert frame[5:6] == bytes([FrameType.COMMAND])
    assert frame[6:8] == bytes([0, 0])  # stream_id=0, flags=0
    assert frame[8:12] == (42).to_bytes(4, "little")
    # payload_len should be 8 (cmd + param)
    assert frame[24:28] == (8).to_bytes(4, "little")
    # CRC over everything before it
    assert frame[-4:] == zlib.crc32(frame[:-4]).to_bytes(4, "little")
    # Verify payload: PING (1) + param (0)
    payload = frame[HEADER_SIZE : HEADER_SIZE + 8]
    assert struct.unpack("<II", payload) == (CommandCode.PING, 0)


def test_parse_ack_roundtrip():
    """ACK parse roundtrip: pack and decode."""
    payload = struct.pack("<III", CommandCode.SET_USECASE, ResultCode.OK, 3)
    cmd, result, applied = parse_ack(payload)
    assert cmd == CommandCode.SET_USECASE
    assert result == ResultCode.OK
    assert applied == 3


def test_parse_ack_rejects_short_payload():
    """ACK parse rejects short payloads."""
    with pytest.raises(ProtocolError):
        parse_ack(b"\x01\x00\x00")


def test_decoder_passthrough_command_and_ack():
    """Decoder passes through COMMAND and ACK frame types unchanged."""
    from roomscan.decoder import StreamDecoder

    decoder = StreamDecoder()
    # Pack a COMMAND frame
    cmd_frame = pack_command(CommandCode.PING, 0, token=100)
    # Pack an ACK frame
    ack_header = FrameHeader(
        frame_type=FrameType.ACK, stream_id=0, flags=0,
        seq=100, t_us=0, width=0, height=0, payload_len=12,
    )
    ack_payload = struct.pack("<III", CommandCode.PING, ResultCode.OK, 1)
    ack_frame = pack_frame(ack_header, ack_payload)

    # Feed both frames
    frames = decoder.feed(cmd_frame + ack_frame)
    assert len(frames) == 2
    assert frames[0].header.frame_type == FrameType.COMMAND
    assert frames[0].header.seq == 100
    assert frames[1].header.frame_type == FrameType.ACK
    assert frames[1].header.seq == 100
