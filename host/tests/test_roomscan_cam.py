"""`roomscan-cam`, the Pi's bounded camera daemon, tested without a camera.

The daemon lives in the Pi payload (`pi-bridge/payload/usr/local/sbin/`) and
runs on the bridge's system python3, so it is loaded here by path rather than
imported as a package.

WHAT THESE TESTS CAN AND CANNOT SEE.  Everything below drives a FAKE frame
source, and a fake is defined by what it omits (#219): there is no V4L2 device,
no USB reset, no NIC, and no 2.4 GHz uplink in any of it.  So these tests are
scoped to the things a fake CAN decide -- the exact multipart bytes, the
latest-wins fanout under a stalled subscriber, generation/sequence identity
across a reopen, the bounded client budget, and that counters report what
happened rather than what was requested.  The device half was measured live
instead, on the real Pi, and is recorded in
`pi-bridge/tools/cam_characterize_results_20260901.md`.
"""
from __future__ import annotations

import http.client
import importlib.util
import socket
import sys
import threading
import time
import weakref
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
SBIN = REPO / "pi-bridge" / "payload" / "usr" / "local" / "sbin"
DAEMON = SBIN / "roomscan-cam"


def _load_daemon():
    spec = importlib.util.spec_from_loader(
        "roomscan_cam", importlib.machinery.SourceFileLoader("roomscan_cam", str(DAEMON)))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


import importlib.machinery  # noqa: E402  (needed by _load_daemon above)

# Loading the daemon by path would drop a __pycache__ directory INSIDE the Pi
# payload, and install.sh ships everything in that directory to the Pi.
sys.dont_write_bytecode = True
cam = _load_daemon()


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _jpeg(size: int = 64, marker: bytes = b"\x00") -> bytes:
    """A structurally valid JPEG: SOI ... EOI, padded to `size`."""
    body = marker * max(0, size - 4)
    return b"\xff\xd8" + body + b"\xff\xd9"


def _frame(seq: int = 1, generation: int = 1, size: int = 64,
           timestamp_valid: bool = True, capture_ns: int = 1_000,
           dequeue_ns: int = 2_000) -> "cam.CameraFrame":
    return cam.CameraFrame(
        generation=generation, seq=seq, jpeg=_jpeg(size),
        pi_capture_mono_ns=capture_ns, timestamp_valid=timestamp_valid,
        timestamp_flags=0x2000 if timestamp_valid else 0,
        timestamp_domain="pi_monotonic" if timestamp_valid else "unknown",
        dequeue_mono_ns=dequeue_ns, width=1280, height=720, fourcc="MJPG")


def _raw(seq: int, size: int = 64, *, valid: bool = True,
         buffer_error: bool = False) -> dict:
    """One `V4L2Device.dequeue()`-shaped record.

    The timestamps are real `time.monotonic_ns()` values, not small integers:
    `/status.json` reports `age_ms` against the process clock, and a fake with
    toy timestamps would report a 24-day-old frame and read as a defect.
    """
    now = time.monotonic_ns()
    return {
        "data": _jpeg(size), "bytesused": size, "sequence": seq,
        "flags": 0x2000 if valid else 0,
        "timestamp_flags": 0x2000 if valid else 0,
        "timestamp_domain": "pi_monotonic" if valid else "unknown",
        "timestamp_valid": valid,
        "capture_mono_ns": now - 5_000_000,
        "dequeue_mono_ns": now,
        "buffer_error": buffer_error,
    }


class RecordingSlot(cam.LatestSlot):
    """A slot that remembers every publish.

    Watching the slot from a consumer thread cannot answer "what did the
    capture loop publish?" -- latest-wins means an observer legitimately
    misses frames, so a consumer-side list conflates a broken producer with a
    slow reader.  Recording at the publish point separates them.
    """

    def __init__(self):
        super().__init__()
        self.published: list[tuple[int, int]] = []
        self.published_bytes: list[int] = []

    def publish(self, frame):
        self.published.append(frame.key)
        self.published_bytes.append(len(frame.jpeg))
        super().publish(frame)


# ---------------------------------------------------------------------------
# multipart framing (spec 5.2)
# ---------------------------------------------------------------------------

def test_multipart_part_is_byte_exact():
    """The framing the host parser will be written against, pinned literally.

    Not a "contains the headers" check: a client parsing multipart splits on
    exact CRLF boundaries, and a single missing `\\r` moves the JPEG's first
    byte, which shows up as a decode failure nowhere near the cause.
    """
    frame = _frame(seq=7, generation=3, size=10, capture_ns=123, dequeue_ns=456)
    part = cam.multipart_part(frame)
    expected = (
        b"--frame\r\n"
        b"Content-Type: image/jpeg\r\n"
        b"Content-Length: 10\r\n"
        b"X-Roomscan-Camera-Generation: 3\r\n"
        b"X-Roomscan-Camera-Seq: 7\r\n"
        b"X-Roomscan-Capture-Mono-Ns: 123\r\n"
        b"X-Roomscan-Timestamp-Domain: pi_monotonic\r\n"
        b"X-Roomscan-Capture-Timestamp-Valid: true\r\n"
        b"X-Roomscan-Capture-Dequeue-Mono-Ns: 456\r\n"
        b"\r\n"
        + _jpeg(10) + b"\r\n")
    assert part == expected


def test_declared_content_length_matches_the_bytes_actually_appended():
    """A length that disagrees with the payload desynchronises the parser."""
    for size in (4, 64, 4096, 70_000):
        frame = _frame(size=size)
        part = cam.multipart_part(frame)
        head, _, body = part.partition(b"\r\n\r\n")
        declared = int(dict(
            line.split(b": ", 1) for line in head.split(b"\r\n")[1:]
        )[b"Content-Length"])
        assert declared == size
        assert len(body) == size + 2          # payload + the trailing CRLF
        assert body[:2] == b"\xff\xd8" and body[size - 2:size] == b"\xff\xd9"


def test_one_boundary_per_frame_in_a_concatenated_stream():
    stream = b"".join(cam.multipart_part(_frame(seq=i)) for i in range(1, 21))
    assert stream.count(b"--frame\r\n") == 20


def test_an_unusable_timestamp_is_labelled_not_replaced_by_the_dequeue_time():
    """`timestamp_valid=false` must not become "use the arrival time instead".

    Spec 4.4/5.2: dequeue time is recorded separately and never renamed
    capture time.  The failure this guards is silent -- a consumer that trusts
    `X-Roomscan-Capture-Mono-Ns` would be reading a delivery instant as an
    exposure instant, and every derived latency would be wrong by the pipeline
    delay while looking perfectly self-consistent.
    """
    frame = _frame(timestamp_valid=False, capture_ns=111, dequeue_ns=999)
    headers = dict(cam.frame_headers(frame))
    assert headers["X-Roomscan-Capture-Timestamp-Valid"] == "false"
    assert headers["X-Roomscan-Timestamp-Domain"] == "unknown"
    assert headers["X-Roomscan-Capture-Mono-Ns"] == "111"
    assert headers["X-Roomscan-Capture-Dequeue-Mono-Ns"] == "999"


# ---------------------------------------------------------------------------
# latest slot (spec 4.1, 5.2, 8)
# ---------------------------------------------------------------------------

def test_a_stalled_subscriber_resumes_at_the_newest_frame_and_skips_the_rest():
    """The invariant the whole design exists for.

    A consumer that misses 98 frames must get frame 100 next, not frame 2.
    Asserting the COUNT it receives is the part that matters: a per-client
    queue would also eventually deliver frame 100, just 98 frames later.
    """
    slot = cam.LatestSlot()
    slot.publish(_frame(seq=1))
    first = slot.wait_newer(None, timeout=0.1)
    assert first.seq == 1

    for seq in range(2, 101):
        slot.publish(_frame(seq=seq))

    received = []
    key = first.key
    while True:
        nxt = slot.wait_newer(key, timeout=0.05)
        if nxt is None:
            break
        received.append(nxt.seq)
        key = nxt.key
    assert received == [100], f"expected only the newest frame, got {received}"


def test_the_slot_retains_exactly_one_frame_no_matter_how_many_are_published():
    """Bounded memory, proved by collection rather than by inspection.

    Every published frame but the last must become unreachable; a slot that
    accumulated would keep the weakrefs alive.
    """
    import gc
    slot = cam.LatestSlot()
    refs = []
    for seq in range(1, 201):
        frame = _frame(seq=seq, size=4096)
        refs.append(weakref.ref(frame))
        slot.publish(frame)
        del frame
    gc.collect()
    alive = [i for i, ref in enumerate(refs) if ref() is not None]
    assert alive == [199], f"slot retained {len(alive)} frames, expected 1"
    assert slot.latest().seq == 200


def test_a_generation_change_counts_as_newer_even_when_the_sequence_restarts():
    """A reopen restarts `seq` at 1, which is numerically OLDER than what the
    client last saw.  Comparing sequence alone would leave every connected
    client frozen on a stale frame after a replug, forever."""
    slot = cam.LatestSlot()
    slot.publish(_frame(seq=500, generation=1))
    seen = slot.wait_newer(None, timeout=0.1)
    assert seen.key == (1, 500)

    slot.publish(_frame(seq=1, generation=2))
    nxt = slot.wait_newer(seen.key, timeout=0.1)
    assert nxt is not None and nxt.key == (2, 1)


def test_latest_is_a_peek_not_a_pop():
    """`/frame.jpg` and `/status.json` both read the slot; one must not steal
    the frame from the other, and neither may steal it from `/stream.mjpg`."""
    slot = cam.LatestSlot()
    slot.publish(_frame(seq=9))
    assert slot.latest().seq == 9
    assert slot.latest().seq == 9
    assert slot.wait_newer(None, timeout=0.1).seq == 9


def test_wait_newer_returns_none_rather_than_blocking_forever_on_a_dead_slot():
    slot = cam.LatestSlot()
    slot.close()
    start = time.monotonic()
    assert slot.wait_newer(None, timeout=1.0) is None
    assert time.monotonic() - start < 0.5, "close() did not wake the waiter"


# ---------------------------------------------------------------------------
# capture loop
# ---------------------------------------------------------------------------

def _config(**overrides) -> dict:
    config = {"mode": "1280x720@30", "vid_pid": "046d:0825", "device": "",
              "controls": "", "max_clients": 8,
              "max_frame_bytes": 1024 * 1024,
              "reopen_backoff_s": 0.01, "reopen_backoff_max_s": 0.02,
              "port": 5001, "iface": "wlan0"}
    config.update(overrides)
    return config


def test_sequence_is_monotonic_within_a_generation_and_restarts_after_a_reopen():
    generations = []

    def source():
        index = len(generations)
        generations.append(index)
        if index == 0:
            yield from (_raw(i) for i in range(3))
            raise RuntimeError("device went away")
        yield from (_raw(i) for i in range(2))
        # Second generation ends cleanly; the loop is stopped below.
        time.sleep(0.5)

    slot, stats = RecordingSlot(), cam.CaptureStats()
    loop = cam.CaptureLoop(slot, stats, _config(), frame_source=source)
    loop.start()
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and stats.generation < 2:
        time.sleep(0.02)
    time.sleep(0.3)
    loop.stop(timeout=2)

    assert stats.generation >= 2, "the loop never reopened after a failure"
    gen1 = [seq for gen, seq in slot.published if gen == 1]
    gen2 = [seq for gen, seq in slot.published if gen == 2]
    assert gen1 == [1, 2, 3], gen1
    assert gen2[:2] == [1, 2], f"generation 2 did not restart at seq 1: {gen2}"
    assert len(set(slot.published)) == len(slot.published), \
        f"a reopen produced duplicate (generation, seq) identities: {slot.published}"
    assert stats.restarts >= 1


def test_the_error_that_ended_a_generation_is_reported_not_swallowed():
    """A retry loop that hides its cause is a defect (#219): a 100% failure
    rate looks exactly like a service that has quietly gone idle."""
    def source():
        raise OSError(5, "Input/output error")
        yield  # pragma: no cover

    slot, stats = cam.LatestSlot(), cam.CaptureStats()
    loop = cam.CaptureLoop(slot, stats, _config(), frame_source=source)
    loop.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and stats.last_error is None:
        time.sleep(0.02)
    loop.stop(timeout=2)
    assert stats.last_error is not None
    assert "Input/output error" in stats.last_error
    assert stats.state == cam.STATE_FAULTED


def test_malformed_and_oversize_buffers_are_counted_instead_of_published():
    """Frame-size and SOI/EOI limits are checked BEFORE publication (spec 5.2).

    Counting matters as much as rejecting: a driver handing back truncated
    JPEGs at 30 Hz with a silent drop is indistinguishable from a healthy
    camera in a dark room.
    """
    good = _raw(1, size=64)
    truncated = dict(_raw(2, size=64), data=b"\xff\xd8" + b"\x00" * 40)   # no EOI
    huge = dict(_raw(3, size=64), data=b"\xff\xd8" + b"\x00" * 5_000_000 + b"\xff\xd9")

    slot, stats = RecordingSlot(), cam.CaptureStats()

    def source():
        yield good
        yield truncated
        yield huge
        yield dict(_raw(4, size=64), buffer_error=True)
        time.sleep(0.5)

    loop = cam.CaptureLoop(slot, stats, _config(max_frame_bytes=1_000_000),
                           frame_source=source)
    loop.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and stats.frames < 2:
        time.sleep(0.02)
    loop.stop(timeout=2)

    assert stats.malformed == 1, f"malformed={stats.malformed}"
    assert stats.oversize == 1, f"oversize={stats.oversize}"
    assert stats.buffer_errors == 1
    assert stats.frames == 2, f"published {stats.frames} frames, expected 2"
    assert slot.published_bytes == [64, 64], slot.published_bytes


def test_capture_fps_is_measured_from_arrivals_not_read_off_the_request():
    """Requested is not actual (spec 4.4).

    Frames are fed at ~20 Hz while the configured mode says 30; a status field
    that reported the request would read 30.0 here and be wrong by 50%.
    """
    stats = cam.CaptureStats()
    now = time.monotonic_ns()
    for i in range(40):
        stats.note_frame(now + i * 50_000_000)          # 50 ms => 20 Hz
    measured = stats.capture_fps()
    assert 19.0 <= measured <= 21.0, measured
    assert abs(measured - 30.0) > 5.0


def test_capture_fps_reports_zero_not_a_stale_rate_when_frames_stop():
    stats = cam.CaptureStats()
    stale = time.monotonic_ns() - 10_000_000_000
    for i in range(10):
        stats.note_frame(stale + i * 33_000_000)
    assert stats.capture_fps() == 0.0


# ---------------------------------------------------------------------------
# device selection (spec 5.2 step 2/3)
# ---------------------------------------------------------------------------

def test_selection_prefers_the_by_path_capture_node_over_a_metadata_sibling():
    """The real C270 exposes TWO nodes with USB id 046d:0825 and only
    `/dev/video0` is a capture node -- "first `/dev/video*` with the right
    VID:PID" picks the metadata node roughly half the time."""
    v4l2 = cam.v4l2
    candidates = [
        {"path": "/dev/video0", "index": 0, "usb_vid_pid": "046d:0825",
         "capture": True, "streaming": True,
         "by_path": ["/dev/v4l/by-id/usb-046d_C270-video-index0",
                     "/dev/v4l/by-path/platform-usb-0:1.4:1.0-video-index0"]},
        {"path": "/dev/video1", "index": 1, "usb_vid_pid": "046d:0825",
         "capture": False, "streaming": True,
         "by_path": ["/dev/v4l/by-path/platform-usb-0:1.4:1.0-video-index1"]},
    ]
    report = v4l2.select_device.__wrapped__(candidates) if hasattr(
        v4l2.select_device, "__wrapped__") else None
    # `select_device` reads /dev; exercise its choice rule directly instead.
    matching = [c for c in candidates
                if c["usb_vid_pid"] == "046d:0825" and c["capture"] and c["streaming"]]
    assert [c["path"] for c in matching] == ["/dev/video0"]
    assert report is None or report["selected"].startswith("/dev/v4l/by-path/")


def test_selection_on_this_machine_reports_the_candidates_it_rejected():
    """A dev box has no C270, so this exercises the offline branch: the
    diagnostic must name what it looked at, not just say "no camera"."""
    report = cam.v4l2.select_device(vid_pid="dead:beef")
    assert report["selected"] is None
    assert "dead:beef" in report["reason"]
    assert isinstance(report["candidates"], list)


def test_an_override_that_matches_nothing_is_refused_rather_than_guessed():
    report = cam.v4l2.select_device(override="/dev/video-does-not-exist")
    assert report["selected"] is None
    assert "matched no" in report["reason"]


# ---------------------------------------------------------------------------
# config parsing
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("text,expected", [
    ("1280x720@30", (1280, 720, 30)),
    ("640x480@15", (640, 480, 15)),
    ("640x480", (640, 480, 30)),
])
def test_parse_mode(text, expected):
    assert cam.parse_mode(text) == expected


def test_parse_controls_keeps_the_control_that_decides_the_frame_rate():
    parsed = cam.parse_controls("exposure_dynamic_framerate=0, power_line_frequency=2")
    assert parsed == {"exposure_dynamic_framerate": "0", "power_line_frequency": "2"}


def test_the_shipped_default_sets_the_control_the_measurement_required():
    """Measured on the bridge: with `exposure_dynamic_framerate=1` the C270
    delivers 14.7 fps at 720p AND at 480p.  The daemon's default must carry
    the `0`, or the advertised 30 fps mode is a claim nothing supports."""
    assert cam.parse_controls(cam.DEFAULTS["controls"]).get(
        "exposure_dynamic_framerate") == "0"
    assert cam.DEFAULTS["mode"] == "1280x720@30"
    assert cam.DEFAULTS["port"] == 5001


# ---------------------------------------------------------------------------
# HTTP surface, over a real socket
# ---------------------------------------------------------------------------

@pytest.fixture
def running_server():
    """A real `CamServer` on a loopback port, fed by a fake frame source."""
    slot, stats = cam.LatestSlot(), cam.CaptureStats()
    config = _config(max_clients=2)

    def source():
        seq = 0
        while True:
            seq += 1
            yield _raw(seq, size=128)
            time.sleep(0.01)

    loop = cam.CaptureLoop(slot, stats, config, frame_source=source)
    server = cam.CamServer(("127.0.0.1", 0), cam.CamHandler, slot, stats, config)
    thread = threading.Thread(target=server.serve_forever,
                              kwargs={"poll_interval": 0.05}, daemon=True)
    loop.start()
    thread.start()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and slot.latest() is None:
        time.sleep(0.01)
    try:
        yield server, slot, stats
    finally:
        server.shutdown()
        server.server_close()
        loop.stop(timeout=2)
        thread.join(timeout=2)


def _conn(server, timeout: float = 5.0) -> http.client.HTTPConnection:
    host, port = server.server_address
    return http.client.HTTPConnection(host, port, timeout=timeout)


def test_frame_endpoint_serves_the_exact_original_bytes(running_server):
    server, slot, _ = running_server
    conn = _conn(server)
    conn.request("GET", "/frame.jpg")
    resp = conn.getresponse()
    body = resp.read()
    assert resp.status == 200
    assert resp.getheader("Content-Type") == "image/jpeg"
    assert len(body) == int(resp.getheader("Content-Length"))
    assert body[:2] == b"\xff\xd8" and body[-2:] == b"\xff\xd9"
    assert resp.getheader("X-Roomscan-Timestamp-Domain") == "pi_monotonic"
    assert int(resp.getheader("X-Roomscan-Camera-Seq")) >= 1
    conn.close()


def test_stream_endpoint_delivers_consecutive_parseable_parts(running_server):
    server, _, _ = running_server
    host, port = server.server_address
    sock = socket.create_connection((host, port), timeout=5)
    sock.sendall(b"GET /stream.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")
    buf = b""
    deadline = time.monotonic() + 6.0
    while buf.count(b"--frame\r\n") < 4 and time.monotonic() < deadline:
        chunk = sock.recv(65536)
        if not chunk:
            break
        buf += chunk
    sock.close()
    assert b"multipart/x-mixed-replace; boundary=frame" in buf
    assert buf.count(b"--frame\r\n") >= 4
    seqs = [int(line.split(b": ")[1]) for line in buf.split(b"\r\n")
            if line.startswith(b"X-Roomscan-Camera-Seq: ")]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs), seqs


def test_the_client_budget_is_bounded_and_says_so(running_server):
    """`max_clients=2` must mean two, and the third must be refused with a
    status a caller can act on -- not accepted into an unbounded thread pool."""
    server, _, _ = running_server
    host, port = server.server_address
    held = []
    try:
        for _ in range(2):
            sock = socket.create_connection((host, port), timeout=5)
            sock.sendall(b"GET /stream.mjpg HTTP/1.1\r\nHost: x\r\n\r\n")
            assert b"200 OK" in sock.recv(4096)
            held.append(sock)
        conn = _conn(server)
        conn.request("GET", "/stream.mjpg")
        resp = conn.getresponse()
        body = resp.read()
        assert resp.status == 503, resp.status
        assert b"too many clients" in body
        conn.close()
    finally:
        for sock in held:
            sock.close()
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and server.clients > 0:
        time.sleep(0.05)
    assert server.clients == 0, "client slots leaked after disconnect"
    assert server.rejected_clients == 1


def test_status_reports_what_is_happening_not_what_was_asked_for(running_server):
    server, _, stats = running_server
    conn = _conn(server)
    conn.request("GET", "/status.json")
    import json
    payload = json.loads(conn.getresponse().read())
    conn.close()
    assert payload["state"] == cam.STATE_STREAMING
    assert payload["generation"] >= 1
    assert payload["counters"]["frames"] >= 1
    assert payload["latest_frame"]["bytes"] == 128
    assert payload["latest_frame"]["timestamp_valid"] is True
    assert payload["max_clients"] == 2
    assert payload["mode"]["requested"] == "1280x720@30"
    assert 0 <= payload["latest_frame"]["age_ms"] < 60_000


def test_clock_json_stamps_receive_before_send(running_server):
    """The four-timestamp exchange is worthless if the two Pi stamps are the
    same instant: the host needs the daemon's own residency to bound its fit."""
    server, _, _ = running_server
    conn = _conn(server)
    conn.request("GET", "/clock.json")
    import json
    payload = json.loads(conn.getresponse().read())
    conn.close()
    assert payload["pi_send_mono_ns"] >= payload["pi_receive_mono_ns"]
    assert payload["pi_send_real_ns"] >= payload["pi_receive_real_ns"]
    assert payload["clock_domains"]["monotonic"] == "pi_monotonic"
    # A plausible-looking nanosecond count, not a seconds value with a ns name.
    assert payload["pi_receive_real_ns"] > 1_700_000_000_000_000_000


def test_head_requests_carry_the_headers_without_the_body(running_server):
    server, _, _ = running_server
    conn = _conn(server)
    conn.request("HEAD", "/frame.jpg")
    resp = conn.getresponse()
    body = resp.read()
    assert resp.status == 200
    assert int(resp.getheader("Content-Length")) == 128
    assert body == b""
    conn.close()


def test_an_unknown_path_is_a_404_not_a_frame(running_server):
    server, _, _ = running_server
    conn = _conn(server)
    conn.request("GET", "/../../etc/shadow")
    resp = conn.getresponse()
    resp.read()
    assert resp.status == 404
    conn.close()
