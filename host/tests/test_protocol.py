import struct
import zlib
from pathlib import Path

import pytest

from roomscan.protocol import (
    HEADER_SIZE, MAGIC, SUPPORTED_VERSIONS, VERSION,
    FrameHeader, FrameType, ProtocolError, StreamId, pack_frame,
    CommandCode, ResultCode, pack_command, parse_ack,
    ACK_LEGACY_SIZE, ACK_RANGING_CONFIG_SIZE, MANUAL_PARAMS_PAYLOAD_SIZE,
    IMU_ENV_RATE_COUPLED, IMU_ENV_RATE_MAX_HZ, EXPOSURE_MS_STEP,
    ManualParams, LegacyAck, RangingConfigAck, ProfileId, RangingMode, PowerMode,
    pack_manual_command, parse_manual_command, parse_typed_ack,
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
    assert frame[:8] == b"RSCN" + bytes([VERSION, 1, 0, 0])
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
    """pack_frame() always emits the CURRENT wire version (v2) -- golden_depth_2x2_v2.bin
    is the matching hand-packed vector. golden_depth_2x2.bin (no "_v2" suffix) is a frozen
    v1 vector kept version=1 forever; see test_v1_golden_fixture_still_decodes below."""
    golden = (FIXTURES / "golden_depth_2x2_v2.bin").read_bytes()
    assert pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD) == golden
    assert golden[4] == 2  # version byte -- this fixture is deliberately v2


def test_v1_golden_fixture_is_frozen_at_version_1():
    """golden_depth_2x2.bin is a real historical v1 wire vector -- its version byte must
    NEVER be bumped even though the host default moved to v2, or the "v1 capture still
    decodes after the host moves to v2" regression this fixture exists for becomes
    untestable (make_fixtures.py's golden_depth_2x2() hardcodes the literal 1 for the
    same reason)."""
    golden = (FIXTURES / "golden_depth_2x2.bin").read_bytes()
    assert golden[4] == 1


def test_v1_golden_fixture_still_decodes():
    """The hard compatibility test: an existing v1 recording (golden_depth_2x2.bin, byte-
    for-byte the same fixture that predates v2) still decodes after the host default
    moves to v2. FrameHeader.unpack() must accept both versions in SUPPORTED_VERSIONS."""
    golden = (FIXTURES / "golden_depth_2x2.bin").read_bytes()
    assert SUPPORTED_VERSIONS == (1, 2)
    hdr = FrameHeader.unpack(golden[:HEADER_SIZE])
    assert hdr == GOLDEN_HEADER  # FrameHeader doesn't carry the version byte itself
    payload = golden[HEADER_SIZE:-4]
    assert payload == GOLDEN_PAYLOAD
    (crc,) = struct.unpack_from("<I", golden, len(golden) - 4)
    assert zlib.crc32(golden[:-4]) == crc


def test_v1_golden_fixture_decodes_through_stream_decoder():
    """Same v1 capture, but through the full incremental decoder (not just the header
    unpack) -- the actual replay path a recorded v1 capture takes."""
    from roomscan.decoder import StreamDecoder

    golden = (FIXTURES / "golden_depth_2x2.bin").read_bytes()
    decoder = StreamDecoder()
    frames = decoder.feed(golden)
    assert len(frames) == 1
    assert decoder.crc_failures == 0
    assert frames[0].header == GOLDEN_HEADER
    assert frames[0].payload == GOLDEN_PAYLOAD


def test_unpack_rejects_version_0_and_3():
    """Versions outside SUPPORTED_VERSIONS (1, 2) are still rejected -- the relaxation is
    exactly {1, 2}, not "any version"."""
    for bad_version in (0, 3, 99):
        frame = bytearray(pack_frame(GOLDEN_HEADER, GOLDEN_PAYLOAD))
        frame[4] = bad_version
        with pytest.raises(ProtocolError):
            FrameHeader.unpack(bytes(frame[:HEADER_SIZE]))


def test_parse_event_roundtrip():
    payload = struct.pack("<II", 2, 3) + b"trigger retries exhausted"
    from roomscan.protocol import EventCode, parse_event
    code, detail, msg = parse_event(payload)
    assert code == EventCode.TRIGGER_TIMEOUT
    assert detail == 3
    assert msg == "trigger retries exhausted"


def test_parse_event_auto_wake_motion():
    from roomscan.protocol import EventCode, parse_event
    wake_up_src = 0b0000_1010   # WU_IA + Y_WU set
    payload = struct.pack("<II", 6, wake_up_src)
    code, detail, msg = parse_event(payload)
    assert code == EventCode.AUTO_WAKE_MOTION
    assert detail == wake_up_src
    assert msg == ""


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
    # width/height zero at the wire level
    assert frame[20:24] == b"\x00\x00\x00\x00"
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


def test_parse_ack_rejects_long_payload():
    """ACK payloads are exactly 12 bytes; trailing bytes are a wire-format violation."""
    with pytest.raises(ProtocolError):
        parse_ack(struct.pack("<III", 1, 0, 1) + b"\x00")  # 13 bytes


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
    # payload content survives the decode intact (guards against payload-slice corruption)
    assert struct.unpack("<II", frames[0].payload) == (CommandCode.PING, 0)
    assert parse_ack(frames[1].payload) == (CommandCode.PING, ResultCode.OK, 1)


# --- v2 registry: new command codes -----------------------------------------------------

def test_new_command_codes_registry():
    assert CommandCode.SET_RANGING_PROFILE == 8
    assert CommandCode.SET_MANUAL_PARAMS == 9
    assert CommandCode.GET_RANGING_CONFIG == 10
    assert CommandCode.SET_IMU_ENV_RATE == 11
    assert CommandCode.GET_IMU_ENV_RATE == 12


def test_profile_id_enum():
    assert ProfileId.ROOM_MAPPING == 0
    assert ProfileId.PRECISION == 1
    assert ProfileId.HIGH_FRAMERATE == 2
    assert ProfileId.MANUAL == 3


def test_ranging_mode_and_power_mode_enums():
    assert RangingMode.AMBIENT == 0
    assert RangingMode.PRECISION == 1
    assert PowerMode.ULP == 0
    assert PowerMode.LP == 1
    assert PowerMode.REGULAR == 2


def test_imu_env_rate_constants():
    assert IMU_ENV_RATE_COUPLED == 0
    assert IMU_ENV_RATE_MAX_HZ == 480
    assert EXPOSURE_MS_STEP == 1


# --- v2 registry: ordinary cmd+u32 commands (8, 10, 11, 12) -----------------------------

@pytest.mark.parametrize("cmd,param", [
    (CommandCode.SET_RANGING_PROFILE, ProfileId.HIGH_FRAMERATE),
    (CommandCode.GET_RANGING_CONFIG, 0),
    (CommandCode.SET_IMU_ENV_RATE, 30),
    (CommandCode.SET_IMU_ENV_RATE, IMU_ENV_RATE_COUPLED),
    (CommandCode.GET_IMU_ENV_RATE, 0),
])
def test_new_legacy_shaped_commands_pack_as_v2_8_byte_payload(cmd, param):
    """Commands 8, 10, 11, 12 keep the legacy 8-byte cmd+param COMMAND payload -- only
    command 9 needs the new 12-byte manual shape."""
    frame = pack_command(cmd, param, token=1)
    assert frame[4] == VERSION == 2
    assert frame[24:28] == (8).to_bytes(4, "little")  # payload_len
    payload = frame[HEADER_SIZE:HEADER_SIZE + 8]
    assert struct.unpack("<II", payload) == (cmd, param)
    assert frame[-4:] == zlib.crc32(frame[:-4]).to_bytes(4, "little")


# --- v2 registry: the 12-byte SET_MANUAL_PARAMS command payload -------------------------

MANUAL_GOLDEN = ManualParams(
    ranging_mode=RangingMode.PRECISION, frame_period_us=11_111,
    exposure_ms=4, power_mode=PowerMode.REGULAR,
)


def test_pack_manual_command_layout():
    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    assert len(frame) == HEADER_SIZE + MANUAL_PARAMS_PAYLOAD_SIZE + 4
    assert MANUAL_PARAMS_PAYLOAD_SIZE == 12
    assert frame[:4] == MAGIC
    assert frame[4] == VERSION
    assert frame[5] == FrameType.COMMAND
    assert frame[8:12] == (55).to_bytes(4, "little")  # token in header seq
    assert frame[24:28] == (12).to_bytes(4, "little")  # payload_len
    payload = frame[HEADER_SIZE:HEADER_SIZE + 12]
    assert struct.unpack("<IBIHB", payload) == (
        CommandCode.SET_MANUAL_PARAMS, RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    assert frame[-4:] == zlib.crc32(frame[:-4]).to_bytes(4, "little")


def test_pack_manual_command_matches_golden_fixture():
    """Cross-check against the hand-packed (independent of protocol.py) golden vector."""
    golden = (FIXTURES / "golden_command_manual.bin").read_bytes()
    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    assert frame == golden


def test_manual_command_roundtrip():
    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    hdr = FrameHeader.unpack(frame[:HEADER_SIZE])
    assert hdr.payload_len == MANUAL_PARAMS_PAYLOAD_SIZE
    decoded = parse_manual_command(frame[HEADER_SIZE:HEADER_SIZE + MANUAL_PARAMS_PAYLOAD_SIZE])
    assert decoded == MANUAL_GOLDEN


def test_manual_command_decodes_through_stream_decoder():
    from roomscan.decoder import StreamDecoder

    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    decoder = StreamDecoder()
    frames = decoder.feed(frame)
    assert len(frames) == 1
    assert frames[0].header.frame_type == FrameType.COMMAND
    assert frames[0].header.payload_len == MANUAL_PARAMS_PAYLOAD_SIZE
    assert parse_manual_command(frames[0].payload) == MANUAL_GOLDEN


def test_parse_manual_command_rejects_wrong_length():
    with pytest.raises(ProtocolError):
        parse_manual_command(b"\x00" * 11)
    with pytest.raises(ProtocolError):
        parse_manual_command(b"\x00" * 13)


def test_parse_manual_command_rejects_wrong_cmd_word():
    """A 12-byte payload whose leading cmd word isn't SET_MANUAL_PARAMS is a decode
    error, not silently accepted -- guards against feeding it the wrong slice."""
    bad = struct.pack("<IBIHB", CommandCode.SET_RANGING_PROFILE, 1, 11_111, 4, 2)
    with pytest.raises(ProtocolError):
        parse_manual_command(bad)


# --- v2 registry: typed ACK parsing (LegacyAck / RangingConfigAck) ----------------------

def test_parse_typed_ack_legacy_shape_for_commands_1_to_8():
    payload = struct.pack("<III", CommandCode.SET_USECASE, ResultCode.OK, 2)
    ack = parse_typed_ack(CommandCode.SET_USECASE, payload)
    assert ack == LegacyAck(CommandCode.SET_USECASE, ResultCode.OK, 2)


def test_parse_typed_ack_legacy_shape_for_imu_rate_commands():
    """Commands 11/12 use the legacy 12-byte shape too; `applied` IS the rate_hz."""
    payload = struct.pack("<III", CommandCode.SET_IMU_ENV_RATE, ResultCode.OK, 30)
    ack = parse_typed_ack(CommandCode.SET_IMU_ENV_RATE, payload)
    assert isinstance(ack, LegacyAck)
    assert ack.applied == 30

    payload = struct.pack("<III", CommandCode.GET_IMU_ENV_RATE, ResultCode.OK, IMU_ENV_RATE_COUPLED)
    ack = parse_typed_ack(CommandCode.GET_IMU_ENV_RATE, payload)
    assert ack.applied == IMU_ENV_RATE_COUPLED


def test_parse_typed_ack_ranging_config_shape_for_command_9_and_10():
    payload = struct.pack("<IIBIHB", CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                          RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    ack = parse_typed_ack(CommandCode.SET_MANUAL_PARAMS, payload)
    assert ack == RangingConfigAck(CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                                   RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)

    payload = struct.pack("<IIBIHB", CommandCode.GET_RANGING_CONFIG, ResultCode.OK,
                          RangingMode.AMBIENT, 33_333, 6, PowerMode.ULP)
    ack = parse_typed_ack(CommandCode.GET_RANGING_CONFIG, payload)
    assert isinstance(ack, RangingConfigAck)
    assert ack.frame_period_us == 33_333


def test_parse_typed_ack_ranging_config_matches_golden_fixture():
    golden = (FIXTURES / "golden_ack_ranging_config.bin").read_bytes()
    payload = golden[HEADER_SIZE:HEADER_SIZE + ACK_RANGING_CONFIG_SIZE]
    ack = parse_typed_ack(CommandCode.GET_RANGING_CONFIG, payload)
    assert ack == RangingConfigAck(CommandCode.GET_RANGING_CONFIG, ResultCode.OK,
                                   RangingMode.AMBIENT, 33_333, 6, PowerMode.ULP)


def test_parse_typed_ack_ranging_config_sent_even_on_error_result():
    """The 16-byte shape is unconditional on `cmd`, not on `result` -- a BUSY ACK for
    cmd 9/10 is still 16 bytes (docs/protocol.md)."""
    payload = struct.pack("<IIBIHB", CommandCode.SET_MANUAL_PARAMS, ResultCode.BUSY, 0, 0, 0, 0)
    ack = parse_typed_ack(CommandCode.SET_MANUAL_PARAMS, payload)
    assert ack.result == ResultCode.BUSY
    assert len(payload) == ACK_RANGING_CONFIG_SIZE


def test_parse_typed_ack_rejects_wrong_length_legacy():
    with pytest.raises(ProtocolError):
        parse_typed_ack(CommandCode.PING, b"\x00" * 11)
    with pytest.raises(ProtocolError):
        parse_typed_ack(CommandCode.PING, b"\x00" * 13)


def test_parse_typed_ack_rejects_wrong_length_ranging_config():
    with pytest.raises(ProtocolError):
        parse_typed_ack(CommandCode.SET_MANUAL_PARAMS, b"\x00" * ACK_LEGACY_SIZE)  # 12, not 16
    with pytest.raises(ProtocolError):
        parse_typed_ack(CommandCode.GET_RANGING_CONFIG, b"\x00" * 17)


def test_ack_size_constants():
    assert ACK_LEGACY_SIZE == 12
    assert ACK_RANGING_CONFIG_SIZE == 16


# --- exact CRC coverage: header + payload only, never the trailing CRC itself -----------

def test_manual_command_crc_covers_header_and_payload_only():
    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    body, wire_crc = frame[:-4], frame[-4:]
    assert zlib.crc32(body) == struct.unpack("<I", wire_crc)[0]
    # flipping a payload byte must invalidate the CRC (proves the CRC isn't ignoring it)
    corrupted = bytearray(frame)
    corrupted[HEADER_SIZE] ^= 0xFF
    assert zlib.crc32(bytes(corrupted[:-4])) != struct.unpack("<I", wire_crc)[0]


def test_ranging_config_ack_crc_covers_full_16_byte_payload():
    payload = struct.pack("<IIBIHB", CommandCode.GET_RANGING_CONFIG, ResultCode.OK,
                          RangingMode.AMBIENT, 33_333, 6, PowerMode.ULP)
    header = FrameHeader(frame_type=FrameType.ACK, stream_id=0, flags=0, seq=55, t_us=0,
                         width=0, height=0, payload_len=len(payload))
    frame = pack_frame(header, payload)
    # flipping the LAST payload byte (power_mode, offset 15) must still be covered by CRC
    corrupted = bytearray(frame)
    corrupted[HEADER_SIZE + 15] ^= 0xFF
    assert zlib.crc32(bytes(corrupted[:-4])) != frame[-4:]


# --- partial reads at every byte boundary ------------------------------------------------

def test_manual_command_partial_feed_boundary_anywhere():
    from roomscan.decoder import StreamDecoder

    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    d = StreamDecoder()
    got = []
    for i in range(len(frame)):
        got += d.feed(frame[i:i + 1])
    assert len(got) == 1
    assert parse_manual_command(got[0].payload) == MANUAL_GOLDEN


def test_ranging_config_ack_partial_feed_boundary_anywhere():
    from roomscan.decoder import StreamDecoder

    payload = struct.pack("<IIBIHB", CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                          RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    header = FrameHeader(frame_type=FrameType.ACK, stream_id=0, flags=0, seq=1, t_us=0,
                         width=0, height=0, payload_len=len(payload))
    frame = pack_frame(header, payload)
    d = StreamDecoder()
    got = []
    for i in range(len(frame)):
        got += d.feed(frame[i:i + 1])
    assert len(got) == 1
    assert got[0].payload == payload


# --- garbage resynchronization ------------------------------------------------------------

def test_manual_command_resync_after_garbage():
    from roomscan.decoder import StreamDecoder

    frame = pack_manual_command(MANUAL_GOLDEN, token=55)
    noise = b"boot log noise before the first command\r\n"
    d = StreamDecoder()
    frames = d.feed(noise + frame + noise + frame)
    assert len(frames) == 2
    assert d.bytes_skipped >= len(noise)


def test_manual_and_legacy_commands_concatenated_resync_cleanly():
    """A legacy 8-byte command immediately followed by a 12-byte manual command in one
    buffer -- the decoder must not misalign on the length change."""
    from roomscan.decoder import StreamDecoder

    legacy = pack_command(CommandCode.SET_RANGING_PROFILE, ProfileId.PRECISION, token=1)
    manual = pack_manual_command(MANUAL_GOLDEN, token=2)
    d = StreamDecoder()
    frames = d.feed(legacy + manual + legacy)
    assert len(frames) == 3
    assert frames[0].header.payload_len == 8
    assert frames[1].header.payload_len == MANUAL_PARAMS_PAYLOAD_SIZE
    assert frames[2].header.payload_len == 8
    assert parse_manual_command(frames[1].payload) == MANUAL_GOLDEN


# --- invalid enum / reserved values --------------------------------------------------------

def test_manual_command_accepts_out_of_range_wire_ints_without_crashing():
    """The wire codec does not enforce enum membership (that is dispatch-layer/firmware
    validation, docs/protocol.md result codes BAD_PARAM etc.) -- an out-of-range
    ranging_mode/power_mode still round-trips as a plain int rather than raising, exactly
    like an out-of-range StandbyLevel already does via pack_command."""
    weird = ManualParams(ranging_mode=99, frame_period_us=11_111, exposure_ms=4, power_mode=200)
    frame = pack_manual_command(weird, token=1)
    decoded = parse_manual_command(frame[HEADER_SIZE:HEADER_SIZE + MANUAL_PARAMS_PAYLOAD_SIZE])
    assert decoded == weird


def test_command_frame_with_reserved_payload_len_is_rejected_by_decoder():
    """A COMMAND frame whose payload_len is neither the legacy 8 nor the manual 12 is not
    a value any v2 COMMAND shape uses. The generic StreamDecoder doesn't know about
    per-frame-type payload shapes (that's protocol.py's job for ACK; the firmware parser
    rejects it at the C layer -- see the crosscheck suite), so it still decodes as a
    syntactically valid frame; downstream command dispatch is what must reject it. This
    test pins that the decoder itself does NOT crash or hang on it."""
    header = FrameHeader(frame_type=FrameType.COMMAND, stream_id=0, flags=0, seq=1, t_us=0,
                         width=0, height=0, payload_len=10)
    frame = pack_frame(header, b"\x00" * 10)
    from roomscan.decoder import StreamDecoder
    d = StreamDecoder()
    frames = d.feed(frame)
    assert len(frames) == 1
    assert frames[0].header.payload_len == 10


# --- oversize rejection --------------------------------------------------------------------

def test_manual_command_oversize_payload_len_rejected_by_decoder():
    from roomscan.decoder import StreamDecoder

    hdr = FrameHeader(frame_type=FrameType.COMMAND, stream_id=0, flags=0, seq=1, t_us=0,
                      width=0, height=0, payload_len=1 << 30)
    raw = hdr.pack() + b"x" * 12  # lies about its length
    good = pack_manual_command(MANUAL_GOLDEN, token=2)
    d = StreamDecoder()
    frames = d.feed(raw + good)
    assert len(frames) == 1
    assert parse_manual_command(frames[0].payload) == MANUAL_GOLDEN
