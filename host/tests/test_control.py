import struct
import threading
import time

import pytest

from roomscan.control import CommandClient, parse_command
from roomscan.protocol import (
    CommandCode,
    Frame,
    FrameHeader,
    FrameType,
    HEADER_SIZE,
    ResultCode,
    StreamId,
)


def make_ack(token: int, cmd: int, result: int, applied: int) -> Frame:
    payload = struct.pack("<III", cmd, result, applied)
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
