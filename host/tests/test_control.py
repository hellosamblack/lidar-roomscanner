import struct
import threading
import time

import pytest

import roomscan.profiles as profiles
from roomscan.control import (
    CommandClient,
    _build_parser,
    _manual_params_to_wire,
    parse_command,
)
from roomscan.protocol import (
    CommandCode,
    Frame,
    FrameHeader,
    FrameType,
    HEADER_SIZE,
    ManualParams,
    PowerMode,
    ProfileId,
    ProtocolError,
    RangingConfigAck,
    RangingMode,
    ResultCode,
    StreamId,
)


def make_ack(token: int, cmd: int, result: int, applied: int) -> Frame:
    payload = struct.pack("<III", cmd, result, applied)
    header = FrameHeader(FrameType.ACK, 0, 0, token, 0, 0, 0, len(payload))
    return Frame(header, payload)


def make_ranging_config_ack(token: int, cmd: int, result: int, ranging_mode: int,
                            frame_period_us: int, exposure_ms: int, power_mode: int) -> Frame:
    payload = struct.pack("<IIBIHB", cmd, result, ranging_mode, frame_period_us, exposure_ms, power_mode)
    header = FrameHeader(FrameType.ACK, 0, 0, token, 0, 0, 0, len(payload))
    return Frame(header, payload)


def make_malformed_ranging_ack(token: int, cmd: int) -> Frame:
    """A cmd-9/10 ACK with the wrong (legacy 12-byte) length -- parse_typed_ack
    rejects it, exercising the "malformed extended ACK" path."""
    payload = struct.pack("<III", cmd, ResultCode.OK, 0)
    header = FrameHeader(FrameType.ACK, 0, 0, token, 0, 0, 0, len(payload))
    return Frame(header, payload)


def make_data(seq: int) -> Frame:
    payload = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
    header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, seq, 0, 2, 2, len(payload))
    return Frame(header, payload)


def make_event(seq: int) -> Frame:
    payload = struct.pack("<II", 2, 0) + b"test event"
    header = FrameHeader(FrameType.EVENT, 0, 0, seq, 0, 0, 0, len(payload))
    return Frame(header, payload)


def _wait_for_write(written: list, count: int = 1) -> None:
    for _ in range(200):
        if len(written) >= count:
            return
        time.sleep(0.01)
    raise AssertionError(f"expected {count} write(s), got {len(written)}")


def _run_send(client, cmd, param=0, timeout=1.0):
    """Run client.send() on a worker thread; return (thread, result_box)."""
    box = {}

    def worker():
        try:
            box["value"] = client.send(cmd, param, timeout=timeout)
        except Exception as exc:  # noqa: BLE001 - captured for assertion in the test
            box["error"] = exc

    t = threading.Thread(target=worker)
    t.start()
    return t, box


def test_send_writes_valid_command_frame_and_times_out_on_silence():
    written = []
    client = CommandClient(written.append)

    t, box = _run_send(client, CommandCode.PING, 0, timeout=0.2)
    t.join(timeout=2.0)
    assert not t.is_alive()

    assert len(written) == 1
    frame_bytes = written[0]
    hdr = FrameHeader.unpack(frame_bytes[:HEADER_SIZE])
    assert hdr.frame_type == FrameType.COMMAND
    cmd, param = struct.unpack("<II", frame_bytes[HEADER_SIZE:HEADER_SIZE + 8])
    assert cmd == CommandCode.PING
    assert param == 0

    assert isinstance(box.get("error"), TimeoutError)
    assert str(hdr.seq) in str(box["error"])


def test_send_writes_expected_cmd_and_param_for_usecase():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.SET_USECASE, 2, timeout=0.2)
    t.join(timeout=2.0)
    frame_bytes = written[0]
    cmd, param = struct.unpack("<II", frame_bytes[HEADER_SIZE:HEADER_SIZE + 8])
    assert cmd == CommandCode.SET_USECASE
    assert param == 2


def test_offer_delivers_ack_and_wakes_send():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.PING, 0, timeout=2.0)

    # Wait for the write to land, then build the matching ACK from the real token.
    for _ in range(200):
        if written:
            break
        time.sleep(0.01)
    assert written, "send() never called write()"
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    ack = make_ack(hdr.seq, CommandCode.PING, ResultCode.OK, 1)

    consumed = client.offer(ack)
    assert consumed is True

    t.join(timeout=2.0)
    assert not t.is_alive()
    assert box.get("value") == (ResultCode.OK, 1)


def test_offer_ignores_non_ack_frames_while_send_is_pending():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.PING, 0, timeout=2.0)

    for _ in range(200):
        if written:
            break
        time.sleep(0.01)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    # Interleaved DATA/EVENT frames must be ignored (not consumed) and must
    # not disturb the pending send.
    assert client.offer(make_data(seq=999)) is False
    assert client.offer(make_event(seq=998)) is False

    ack = make_ack(hdr.seq, CommandCode.PING, ResultCode.OK, 1)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box.get("value") == (ResultCode.OK, 1)


def test_offer_rejects_ack_with_mismatched_token():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.PING, 0, timeout=0.5)

    for _ in range(200):
        if written:
            break
        time.sleep(0.01)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    wrong_token = (hdr.seq + 12345) & 0xFFFFFFFF

    # An ACK for a token nobody is waiting on is not consumed.
    assert client.offer(make_ack(wrong_token, CommandCode.PING, ResultCode.OK, 1)) is False

    # The real send is left pending and eventually times out.
    t.join(timeout=2.0)
    assert isinstance(box.get("error"), TimeoutError)


def test_offer_duplicate_ack_on_completed_token_not_consumed():
    """A second/duplicate ACK for an already-completed token returns False, no crash."""
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.PING, 0, timeout=2.0)

    for _ in range(200):
        if written:
            break
        time.sleep(0.01)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    ack = make_ack(hdr.seq, CommandCode.PING, ResultCode.OK, 1)

    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box.get("value") == (ResultCode.OK, 1)

    # Same ACK again: token no longer pending -> not consumed, no effect.
    assert client.offer(ack) is False


def test_send_raises_timeout_error_with_token_and_count():
    client = CommandClient(lambda data: None)
    with pytest.raises(TimeoutError) as excinfo:
        client.send(CommandCode.PING, 0, timeout=0.05)
    msg = str(excinfo.value)
    assert "token=" in msg
    assert "pending" in msg


def test_concurrent_send_and_offer_thread_safety_smoke():
    """send() from a worker thread while offer() runs on the main thread —
    the exact split the CommandClient contract requires."""
    written = []
    lock = threading.Lock()

    def write(data):
        with lock:
            written.append(data)

    client = CommandClient(write)
    threads = []
    boxes = []
    for i in range(5):
        t, box = _run_send(client, CommandCode.PING, i, timeout=2.0)
        threads.append(t)
        boxes.append(box)

    # Drain writes and answer each with its matching ACK from the main thread.
    answered = set()
    deadline = time.time() + 2.0
    while len(answered) < 5 and time.time() < deadline:
        with lock:
            pending_bytes = list(written)
        for frame_bytes in pending_bytes:
            hdr = FrameHeader.unpack(frame_bytes[:HEADER_SIZE])
            if hdr.seq in answered:
                continue
            cmd, param = struct.unpack("<II", frame_bytes[HEADER_SIZE:HEADER_SIZE + 8])
            client.offer(make_ack(hdr.seq, cmd, ResultCode.OK, param))
            answered.add(hdr.seq)
        time.sleep(0.01)

    for t in threads:
        t.join(timeout=2.0)
        assert not t.is_alive()
    results = {box["value"] for box in boxes if "value" in box}
    assert results == {(ResultCode.OK, i) for i in range(5)}


def test_send_raises_timeout_on_malformed_legacy_ack():
    """A wrong-length payload for a legacy-shape command (parse_typed_ack keys
    the shape off the SENT cmd, so this can only happen from device
    corruption/bugs, not a real shape mismatch) surfaces as the same
    "arrived but unparsable" TimeoutError as before generalization."""
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.PING, 0, timeout=1.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    bad_payload = struct.pack("<II", CommandCode.PING, ResultCode.OK)  # 8 bytes, not 12
    bad_ack = Frame(FrameHeader(FrameType.ACK, 0, 0, hdr.seq, 0, 0, 0, len(bad_payload)), bad_payload)
    assert client.offer(bad_ack) is True

    t.join(timeout=2.0)
    assert isinstance(box.get("error"), TimeoutError)
    assert "unparsable" in str(box["error"])


# --- send_profile() (cmd 8, legacy ACK shape) --------------------------------

def _run_send_profile(client, profile_id, timeout=1.0):
    box = {}

    def worker():
        try:
            box["value"] = client.send_profile(profile_id, timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            box["error"] = exc

    t = threading.Thread(target=worker)
    t.start()
    return t, box


def test_send_profile_typed_readback_and_token_matching():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_profile(client, ProfileId.HIGH_FRAMERATE, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    cmd, param = struct.unpack("<II", written[0][HEADER_SIZE:HEADER_SIZE + 8])
    assert cmd == CommandCode.SET_RANGING_PROFILE
    assert param == ProfileId.HIGH_FRAMERATE

    ack = make_ack(hdr.seq, CommandCode.SET_RANGING_PROFILE, ResultCode.OK, ProfileId.HIGH_FRAMERATE)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box.get("value") == (ResultCode.OK, ProfileId.HIGH_FRAMERATE)


def test_send_profile_ignores_interleaved_data_and_event_frames():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_profile(client, ProfileId.STABILITY, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    assert client.offer(make_data(seq=1)) is False
    assert client.offer(make_event(seq=2)) is False

    ack = make_ack(hdr.seq, CommandCode.SET_RANGING_PROFILE, ResultCode.OK, ProfileId.STABILITY)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box.get("value") == (ResultCode.OK, ProfileId.STABILITY)


def test_send_profile_busy_result():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_profile(client, ProfileId.MANUAL, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    # MANUAL rejected: no candidate accepted yet -> BUSY/BAD_PARAM-style nonzero result,
    # still a normal legacy ACK, not an exception.
    ack = make_ack(hdr.seq, CommandCode.SET_RANGING_PROFILE, ResultCode.BUSY, 0)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box.get("value") == (ResultCode.BUSY, 0)


def test_send_profile_timeout():
    client = CommandClient(lambda data: None)
    with pytest.raises(TimeoutError):
        client.send_profile(ProfileId.PRECISION, timeout=0.05)


# --- send_manual_params() (cmd 9, ranging-config ACK shape) ------------------

def _run_send_manual(client, params, timeout=1.0):
    box = {}

    def worker():
        try:
            box["value"] = client.send_manual_params(params, timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            box["error"] = exc

    t = threading.Thread(target=worker)
    t.start()
    return t, box


_SAMPLE_MANUAL = ManualParams(RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)


def test_send_manual_params_typed_readback_and_token_matching():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_manual(client, _SAMPLE_MANUAL, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    assert hdr.seq  # token allocated

    ack = make_ranging_config_ack(hdr.seq, CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                                  RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    result, applied = box["value"]
    assert result == ResultCode.OK
    assert isinstance(applied, RangingConfigAck)
    assert applied.ranging_mode == RangingMode.PRECISION
    assert applied.frame_period_us == 11_111
    assert applied.exposure_ms == 4
    assert applied.power_mode == PowerMode.REGULAR


def test_send_manual_params_interleaved_data_and_event_frames():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_manual(client, _SAMPLE_MANUAL, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    assert client.offer(make_data(seq=10)) is False
    assert client.offer(make_event(seq=11)) is False

    ack = make_ranging_config_ack(hdr.seq, CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                                  RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box["value"][0] == ResultCode.OK


def test_send_manual_params_busy_result_still_carries_prior_config():
    """docs/protocol.md: a BUSY/BAD_PARAM/SENSOR_ERROR ACK is still the 16-byte
    shape, carrying the prior known-good config -- never a shorter payload."""
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_manual(client, _SAMPLE_MANUAL, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    ack = make_ranging_config_ack(hdr.seq, CommandCode.SET_MANUAL_PARAMS, ResultCode.BUSY,
                                  RangingMode.AMBIENT, 33_333, 6, PowerMode.ULP)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    result, applied = box["value"]
    assert result == ResultCode.BUSY
    assert applied.ranging_mode == RangingMode.AMBIENT  # the PRIOR config, not the rejected candidate


def test_send_manual_params_timeout():
    client = CommandClient(lambda data: None)
    with pytest.raises(TimeoutError):
        client.send_manual_params(_SAMPLE_MANUAL, timeout=0.05)


def test_send_manual_params_malformed_ack_raises_timeout():
    written = []
    client = CommandClient(written.append)
    t, box = _run_send_manual(client, _SAMPLE_MANUAL, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    bad_ack = make_malformed_ranging_ack(hdr.seq, CommandCode.SET_MANUAL_PARAMS)
    assert client.offer(bad_ack) is True  # consumed (token matched), just unparsable
    t.join(timeout=2.0)
    assert isinstance(box.get("error"), TimeoutError)
    assert "unparsable" in str(box["error"])


# --- get_ranging_config() (cmd 10, ranging-config ACK shape) -----------------

def _run_get_config(client, timeout=1.0):
    box = {}

    def worker():
        try:
            box["value"] = client.get_ranging_config(timeout=timeout)
        except Exception as exc:  # noqa: BLE001
            box["error"] = exc

    t = threading.Thread(target=worker)
    t.start()
    return t, box


def test_get_ranging_config_typed_readback_and_token_matching():
    written = []
    client = CommandClient(written.append)
    t, box = _run_get_config(client, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    cmd, param = struct.unpack("<II", written[0][HEADER_SIZE:HEADER_SIZE + 8])
    assert cmd == CommandCode.GET_RANGING_CONFIG
    assert param == 0  # ignored per docs/protocol.md, but still a real legacy 8-byte COMMAND

    ack = make_ranging_config_ack(hdr.seq, CommandCode.GET_RANGING_CONFIG, ResultCode.OK,
                                  RangingMode.AMBIENT, 33_333, 6, PowerMode.ULP)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    result, applied = box["value"]
    assert result == ResultCode.OK
    assert applied.ranging_mode == RangingMode.AMBIENT
    assert applied.frame_period_us == 33_333
    assert applied.exposure_ms == 6
    assert applied.power_mode == PowerMode.ULP


def test_get_ranging_config_error_result():
    written = []
    client = CommandClient(written.append)
    t, box = _run_get_config(client, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    ack = make_ranging_config_ack(hdr.seq, CommandCode.GET_RANGING_CONFIG, ResultCode.SENSOR_ERROR,
                                  0, 0, 0, 0)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    result, _applied = box["value"]
    assert result == ResultCode.SENSOR_ERROR


def test_get_ranging_config_timeout():
    client = CommandClient(lambda data: None)
    with pytest.raises(TimeoutError):
        client.get_ranging_config(timeout=0.05)


def test_get_ranging_config_malformed_ack_raises_timeout():
    written = []
    client = CommandClient(written.append)
    t, box = _run_get_config(client, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    bad_ack = make_malformed_ranging_ack(hdr.seq, CommandCode.GET_RANGING_CONFIG)
    assert client.offer(bad_ack) is True
    t.join(timeout=2.0)
    assert isinstance(box.get("error"), TimeoutError)
    assert "unparsable" in str(box["error"])


def test_send_raises_protocol_error_when_cmd_9_answered_via_legacy_send():
    """send() (the legacy convenience path) refuses to hand back a
    RangingConfigAck through its (ResultCode, int) contract -- calling it
    directly on cmd 9/10 is a caller bug, not a silent misparse."""
    written = []
    client = CommandClient(written.append)
    t, box = _run_send(client, CommandCode.SET_MANUAL_PARAMS, 0, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])

    ack = make_ranging_config_ack(hdr.seq, CommandCode.SET_MANUAL_PARAMS, ResultCode.OK,
                                  RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert isinstance(box.get("error"), ProtocolError)


# --- concurrent typed callers -------------------------------------------------

def test_concurrent_typed_callers_profile_manual_and_get_config():
    """send_profile()/send_manual_params()/get_ranging_config() interleaved
    from separate threads -- the same write/offer thread split the plain
    concurrency smoke test proves, but exercising every typed ACK shape at
    once so a shape mix-up would show up as a wrong-typed result, not just a
    wrong value."""
    written = []
    lock = threading.Lock()

    def write(data):
        with lock:
            written.append(data)

    client = CommandClient(write)
    threads_boxes = []
    threads_boxes.append((*_run_send_profile(client, ProfileId.PRECISION, timeout=2.0), CommandCode.SET_RANGING_PROFILE))
    threads_boxes.append((*_run_send_manual(client, _SAMPLE_MANUAL, timeout=2.0), CommandCode.SET_MANUAL_PARAMS))
    threads_boxes.append((*_run_get_config(client, timeout=2.0), CommandCode.GET_RANGING_CONFIG))
    threads_boxes.append((*_run_send_profile(client, ProfileId.HIGH_FRAMERATE, timeout=2.0), CommandCode.SET_RANGING_PROFILE))

    answered = set()
    deadline = time.time() + 2.0
    while len(answered) < len(threads_boxes) and time.time() < deadline:
        with lock:
            pending_bytes = list(written)
        for frame_bytes in pending_bytes:
            hdr = FrameHeader.unpack(frame_bytes[:HEADER_SIZE])
            if hdr.seq in answered:
                continue
            cmd = struct.unpack_from("<I", frame_bytes, HEADER_SIZE)[0]
            if cmd in (CommandCode.SET_MANUAL_PARAMS, CommandCode.GET_RANGING_CONFIG):
                ack = make_ranging_config_ack(hdr.seq, cmd, ResultCode.OK,
                                              RangingMode.PRECISION, 11_111, 4, PowerMode.REGULAR)
            else:
                param = struct.unpack_from("<I", frame_bytes, HEADER_SIZE + 4)[0]
                ack = make_ack(hdr.seq, cmd, ResultCode.OK, param)
            client.offer(ack)
            answered.add(hdr.seq)
        time.sleep(0.01)

    for t, box, _cmd in threads_boxes:
        t.join(timeout=2.0)
        assert not t.is_alive()
        assert "error" not in box, box.get("error")
        assert box["value"][0] == ResultCode.OK


# --- send_imu_env_rate() / get_imu_env_rate() (cmds 11/12, legacy ACK shape) --

def test_send_imu_env_rate_and_get_imu_env_rate_round_trip():
    written = []
    client = CommandClient(written.append)

    t, box = _run_send(client, CommandCode.SET_IMU_ENV_RATE, 30, timeout=2.0)
    _wait_for_write(written)
    hdr = FrameHeader.unpack(written[0][:HEADER_SIZE])
    ack = make_ack(hdr.seq, CommandCode.SET_IMU_ENV_RATE, ResultCode.OK, 30)
    assert client.offer(ack) is True
    t.join(timeout=2.0)
    assert box["value"] == (ResultCode.OK, 30)

    def worker():
        box2["value"] = client.get_imu_env_rate(timeout=2.0)

    box2 = {}
    t2 = threading.Thread(target=worker)
    t2.start()
    _wait_for_write(written, count=2)
    hdr2 = FrameHeader.unpack(written[1][:HEADER_SIZE])
    ack2 = make_ack(hdr2.seq, CommandCode.GET_IMU_ENV_RATE, ResultCode.OK, 30)
    assert client.offer(ack2) is True
    t2.join(timeout=2.0)
    assert box2["value"] == (ResultCode.OK, 30)


# --- profiles<->wire enum mapping (control._manual_params_to_wire) -----------

def test_manual_params_to_wire_maps_by_name_not_raw_value():
    """roomscan.profiles' RangingMode/PowerMode mirror the firmware's C enum
    ordering (VL53L9_CONTEXT_SHORT=0/Precision, VL53L9_CONTEXT_LONG=1/Ambient;
    REGULAR=0/LOW=1/ULTRA_LOW=2), while the WIRE enum (docs/protocol.md) is
    numbered independently (AMBIENT=0/PRECISION=1; ULP=0/LP=1/REGULAR=2).
    Passing `.value` straight through would silently swap ranging mode AND
    invert power mode. Pin the conversion by name for every member, not a
    single spot-check."""
    ranging_cases = [
        (profiles.RangingMode.AMBIENT, RangingMode.AMBIENT),
        (profiles.RangingMode.PRECISION, RangingMode.PRECISION),
    ]
    for host_mode, wire_mode in ranging_cases:
        params = profiles.ManualParams(host_mode, 30, 6, profiles.PowerMode.REGULAR)
        wire = _manual_params_to_wire(params)
        assert wire.ranging_mode == int(wire_mode), (host_mode, wire_mode)

    power_cases = [
        (profiles.PowerMode.ULTRA_LOW, PowerMode.ULP),
        (profiles.PowerMode.LOW, PowerMode.LP),
        (profiles.PowerMode.REGULAR, PowerMode.REGULAR),
    ]
    for host_power, wire_power in power_cases:
        params = profiles.ManualParams(profiles.RangingMode.PRECISION, 30, 6, host_power)
        wire = _manual_params_to_wire(params)
        assert wire.power_mode == int(wire_power), (host_power, wire_power)


def test_manual_params_to_wire_derives_frame_period_from_fps():
    params = profiles.ManualParams(profiles.RangingMode.PRECISION, 90, 4, profiles.PowerMode.REGULAR)
    wire = _manual_params_to_wire(params)
    assert wire.frame_period_us == profiles.fps_to_period_us(90) == 11_111
    assert wire.exposure_ms == 4


# --- CLI arg parsing (pure helper, no serial port involved) -----------------

def test_parse_command_ping():
    _, cmd, param = parse_command(["ping"])
    assert cmd == CommandCode.PING
    assert param == 0


def test_parse_command_calib():
    _, cmd, param = parse_command(["calib"])
    assert cmd == CommandCode.SEND_CALIB
    assert param == 0


def test_parse_command_reinit():
    _, cmd, param = parse_command(["reinit"])
    assert cmd == CommandCode.REINIT
    assert param == 0


def test_parse_command_usecase_value():
    args, cmd, param = parse_command(["usecase", "2"])
    assert cmd == CommandCode.SET_USECASE
    assert param == 2
    assert args.action == "usecase"


def test_parse_command_period_value():
    _, cmd, param = parse_command(["period", "50000"])
    assert cmd == CommandCode.SET_FRAME_PERIOD_US
    assert param == 50000


def test_parse_command_exposure_value():
    _, cmd, param = parse_command(["exposure", "10"])
    assert cmd == CommandCode.SET_EXPOSURE_MS
    assert param == 10


def test_parse_command_port_and_timeout_overrides():
    args, cmd, param = parse_command(["--port", "COM7", "--timeout", "5", "ping"])
    assert args.port == "COM7"
    assert args.timeout == 5.0
    assert cmd == CommandCode.PING


def test_parse_command_requires_an_action():
    with pytest.raises(SystemExit):
        parse_command([])


# --- Task 3 typed-action CLI parsing (pure, no serial port involved) --------

def test_parse_command_rejects_typed_actions():
    """profile/manual/profile-status/imu-rate/imu-rate-status don't reduce to a
    single cmd+param pair -- parse_command() says so instead of guessing."""
    for argv in (["profile", "stability"], ["manual", "--ranging", "ambient",
                 "--fps", "30", "--exposure", "6", "--power", "ulp"],
                 ["profile-status"], ["imu-rate", "coupled"], ["imu-rate-status"]):
        with pytest.raises(ValueError):
            parse_command(argv)


def test_build_parser_profile_action():
    args = _build_parser().parse_args(["profile", "high_framerate"])
    assert args.action == "profile"
    assert args.preset == "high_framerate"


def test_build_parser_profile_action_rejects_manual_as_a_preset():
    """MANUAL isn't reachable through `profile` -- it needs `manual`'s typed
    fields, so it is deliberately absent from the preset choices."""
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["profile", "manual"])


def test_build_parser_manual_action():
    args = _build_parser().parse_args(
        ["manual", "--ranging", "precision", "--fps", "90", "--exposure", "4", "--power", "regular"])
    assert args.action == "manual"
    assert args.ranging == "precision"
    assert args.fps == 90
    assert args.exposure == 4
    assert args.power == "regular"


def test_build_parser_manual_action_requires_all_fields():
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["manual", "--ranging", "ambient", "--fps", "30"])


def test_build_parser_profile_status_action():
    args = _build_parser().parse_args(["profile-status"])
    assert args.action == "profile-status"


def test_build_parser_imu_rate_action_accepts_coupled():
    args = _build_parser().parse_args(["imu-rate", "coupled"])
    assert args.action == "imu-rate"
    assert args.value == 0


def test_build_parser_imu_rate_action_accepts_case_insensitive_coupled():
    args = _build_parser().parse_args(["imu-rate", "COUPLED"])
    assert args.value == 0


def test_build_parser_imu_rate_action_accepts_explicit_hz():
    args = _build_parser().parse_args(["imu-rate", "90"])
    assert args.value == 90


def test_build_parser_imu_rate_action_rejects_garbage():
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["imu-rate", "not-a-number"])


def test_build_parser_imu_rate_status_action():
    args = _build_parser().parse_args(["imu-rate-status"])
    assert args.action == "imu-rate-status"


def test_build_parser_deprecated_actions_still_parse():
    """usecase/period/exposure are kept for one release (Task 3 step 4) -- only
    their help text changes, not their behavior."""
    args = _build_parser().parse_args(["usecase", "2"])
    assert args.action == "usecase"
    assert args.value == 2
    args = _build_parser().parse_args(["period", "50000"])
    assert args.value == 50000
    args = _build_parser().parse_args(["exposure", "10"])
    assert args.value == 10


def test_deprecated_action_help_text_says_deprecated():
    # One "deprecated" mention per marked action (usecase/period/exposure); the
    # new typed actions and the still-current ones (standby/reinit/...) must not
    # be caught up in it.
    help_text = _build_parser().format_help().lower()
    assert help_text.count("deprecated") == 3
