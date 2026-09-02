"""Host-side camera ingest (#226 Stage A, spec §5.4 + §9 "Automated").

What these are actually trying to catch, in the order the spec's verification
matrix lists them:

  * a multipart parser that is right for one chunking and wrong for another --
    so the framing tests replay one canned stream at EVERY split position and
    one byte at a time, and assert the frames recovered are byte-identical to
    what went in;
  * a latest-frame slot that quietly becomes a queue -- so the slot tests count
    live `CameraFrame` objects with weakrefs (a QUANTITY: exactly one retained),
    not a size heuristic;
  * a reconnect/teardown path that leaves a task or a socket behind -- so the
    lifecycle tests assert on `asyncio.all_tasks()` before and after.

Every assertion here is a quantity or an identity, never a type: "the parser
returned a list of bytes" would pass against a parser that dropped every other
frame (docs/engineering-practices.md, and BUG-050's `isinstance(x, float)`).

What these tests CANNOT see, stated plainly because a fake is defined by what
it omits (#219): there is no real socket and no real daemon here. Timing under
a loaded event loop, TCP segmentation the fake does not reproduce, and any
disagreement with the daemon's actual header spelling are all live-probe
questions, not unit-test ones.
"""
from __future__ import annotations

import asyncio
import functools
import gc
import json
import weakref
from contextlib import suppress

import pytest

from roomscan import camera as cam
from roomscan.domain import HostMono


def asyncio_test(fn):
    """Run an async test body on a fresh event loop.

    This suite has no `pytest-asyncio` (see `test_commstatus.py`, which calls
    `asyncio.run` inline); a four-line decorator keeps that convention and
    keeps each test's loop -- and therefore each test's task census -- isolated
    from every other's. `functools.wraps` preserves the signature, so pytest
    fixtures still resolve.
    """
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        return asyncio.run(fn(*args, **kwargs))
    return wrapper


# =============================================================================
# fixtures: a synthetic JPEG and a synthetic daemon
# =============================================================================

def make_jpeg(width: int = 64, height: int = 48, filler: int = 200) -> bytes:
    """A minimal but structurally real JPEG: SOI, an APP0 segment, an SOF0
    carrying `width`/`height`, some payload, EOI.

    Structurally real on purpose -- `jpeg_dimensions` walks segments, so a
    stand-in made of random bytes would test nothing about the walk. The
    dimensions are deliberately NOT round numbers a default could coincide with.
    """
    app0_payload = b"JFIF\x00\x01\x01\x00" + b"\x00" * 5
    # The segment length counts its own two bytes -- getting this wrong
    # misaligns the marker walk by one and is exactly the bug the walk must
    # survive in the wild.
    app0 = b"\xff\xe0" + (2 + len(app0_payload)).to_bytes(2, "big") + app0_payload
    sof0 = (b"\xff\xc0" + (11).to_bytes(2, "big") + b"\x08"
            + height.to_bytes(2, "big") + width.to_bytes(2, "big")
            + b"\x01\x01\x11\x00")
    return b"\xff\xd8" + app0 + sof0 + bytes([filler]) * 32 + b"\xff\xd9"


def part_bytes(jpeg: bytes, *, seq: int, generation: int = 1,
               capture_ns: int | None = 1_000_000_000,
               domain: str | None = "pi_monotonic",
               valid: str | None = None,
               dequeue_ns: int | None = None,
               boundary: str = "frame") -> bytes:
    """One part exactly as `roomscan-cam` frames it (spec §5.2).

    `valid` is the daemon's `X-Roomscan-Capture-Timestamp-Valid: true|false`
    and `dequeue_ns` its `X-Roomscan-Capture-Dequeue-Mono-Ns`. Both default to
    absent so the older-daemon path stays covered by every other test here.
    """
    head = [f"--{boundary}", "Content-Type: image/jpeg",
            f"Content-Length: {len(jpeg)}",
            f"X-Roomscan-Camera-Generation: {generation}",
            f"X-Roomscan-Camera-Seq: {seq}"]
    if capture_ns is not None:
        head.append(f"X-Roomscan-Capture-Mono-Ns: {capture_ns}")
    if valid is not None:
        head.append(f"X-Roomscan-Capture-Timestamp-Valid: {valid}")
    if dequeue_ns is not None:
        head.append(f"X-Roomscan-Capture-Dequeue-Mono-Ns: {dequeue_ns}")
    if domain is not None:
        head.append(f"X-Roomscan-Timestamp-Domain: {domain}")
    return ("\r\n".join(head) + "\r\n\r\n").encode() + jpeg + b"\r\n"


def stream_bytes(count: int, **kw) -> tuple[bytes, list[bytes]]:
    """`count` parts back to back, plus the list of JPEGs they carry."""
    jpegs = [make_jpeg(filler=100 + i) for i in range(count)]
    blob = b"".join(part_bytes(j, seq=i + 1, **kw) for i, j in enumerate(jpegs))
    return blob, jpegs


# =============================================================================
# 1. JPEG inspection -- "requested is not actual" (§4.4)
# =============================================================================

def test_jpeg_dimensions_reads_the_actual_raster_size():
    assert cam.jpeg_dimensions(make_jpeg(1280, 720)) == (1280, 720)
    assert cam.jpeg_dimensions(make_jpeg(640, 480)) == (640, 480)


def test_jpeg_dimensions_is_none_for_non_jpeg_and_for_a_headerless_jpeg():
    assert cam.jpeg_dimensions(b"not a jpeg at all") is None
    assert cam.jpeg_dimensions(b"") is None
    # SOI + immediate SOS: valid framing, no SOF, so the size is genuinely
    # unknown -- and unknown must be None, not a guess.
    assert cam.jpeg_dimensions(b"\xff\xd8\xff\xda\x00\x02\xff\xd9") is None


def test_jpeg_dimensions_does_not_mistake_a_dht_segment_for_a_frame_header():
    """0xC4 (DHT) sits inside the 0xC0-0xCF block but is NOT a frame header.

    Reading it as one lifts two arbitrary payload bytes and reports them as a
    resolution -- a wrong answer that looks exactly like a right one, which is
    why the check is here and not left to "it worked on a real JPEG".
    """
    dht = b"\xff\xc4" + (6).to_bytes(2, "big") + b"\x11\x22\x33\x44"
    sof = (b"\xff\xc0" + (11).to_bytes(2, "big") + b"\x08"
           + (480).to_bytes(2, "big") + (640).to_bytes(2, "big") + b"\x01\x01\x11\x00")
    data = b"\xff\xd8" + dht + sof + b"\xff\xd9"
    assert cam.jpeg_dimensions(data) == (640, 480)


def test_looks_like_jpeg_rejects_a_truncated_body():
    good = make_jpeg()
    assert cam.looks_like_jpeg(good)
    assert not cam.looks_like_jpeg(good[:-2])       # EOI chopped
    assert not cam.looks_like_jpeg(good[2:])        # SOI chopped


# =============================================================================
# 2. Multipart parser -- split at EVERY byte position (§9)
# =============================================================================

def test_parser_recovers_every_frame_from_a_single_chunk():
    blob, jpegs = stream_bytes(4)
    parser = cam.MultipartParser("frame")
    parts = parser.feed(blob)
    assert [p.body for p in parts] == jpegs


@pytest.mark.parametrize("count", [1, 3])
def test_parser_is_correct_at_every_two_chunk_split_position(count):
    """The real point of the parser, and the one a naive implementation fails.

    A split inside the boundary marker, inside a header name, inside the
    CRLFCRLF terminator or inside the body must all recover the identical
    frames. Failing this looks like an occasional dropped frame at runtime, not
    like a parser bug, which is why it is proven exhaustively here.
    """
    blob, jpegs = stream_bytes(count)
    for split in range(len(blob) + 1):
        parser = cam.MultipartParser("frame")
        got = parser.feed(blob[:split]) + parser.feed(blob[split:])
        assert [p.body for p in got] == jpegs, f"split at byte {split}"


def test_parser_is_correct_when_fed_one_byte_at_a_time():
    blob, jpegs = stream_bytes(3)
    parser = cam.MultipartParser("frame")
    got = []
    for i in range(len(blob)):
        got.extend(parser.feed(blob[i:i + 1]))
    assert [p.body for p in got] == jpegs


def test_parser_carries_the_headers_through_lower_cased():
    blob, _ = stream_bytes(1)
    parts = cam.MultipartParser("frame").feed(blob)
    assert parts[0].headers["x-roomscan-camera-seq"] == "1"
    assert parts[0].headers["x-roomscan-timestamp-domain"] == "pi_monotonic"
    assert parts[0].headers["content-length"] == str(len(parts[0].body))


def test_parser_skips_a_preamble_before_the_first_boundary():
    blob, jpegs = stream_bytes(2)
    parser = cam.MultipartParser("frame")
    parts = parser.feed(b"leading junk from a proxy\r\n" + blob)
    assert [p.body for p in parts] == jpegs


def test_parser_tolerates_a_missing_trailing_crlf():
    """§8 asks for tolerance in exactly the ways listed; this is one of them.

    A daemon that omits the body's trailing CRLF still frames every part
    unambiguously, because the next `--boundary` re-syncs.
    """
    jpegs = [make_jpeg(filler=1), make_jpeg(filler=2)]
    blob = b"".join(part_bytes(j, seq=i)[:-2] for i, j in enumerate(jpegs))
    parts = cam.MultipartParser("frame").feed(blob)
    assert [p.body for p in parts] == jpegs


def test_parser_rejects_a_part_with_no_content_length():
    blob = (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + make_jpeg() + b"\r\n")
    with pytest.raises(cam.CameraProtocolError, match="no Content-Length"):
        cam.MultipartParser("frame").feed(blob)


def test_parser_rejects_an_unparseable_content_length():
    blob = (b"--frame\r\nContent-Length: not-a-number\r\n\r\n" + make_jpeg())
    with pytest.raises(cam.CameraProtocolError, match="unparseable"):
        cam.MultipartParser("frame").feed(blob)


def test_parser_rejects_an_oversized_frame_before_buffering_it():
    """The limit must bite on the DECLARED length, not after the bytes arrive.

    Checking afterwards would mean a header claiming 4 GB is honoured by
    allocating 4 GB first -- exactly the memory bound the check exists to
    enforce. The assertion is on the parser's retained-byte count.
    """
    parser = cam.MultipartParser("frame", max_frame_bytes=1024)
    with pytest.raises(cam.CameraProtocolError, match="over the 1024-byte limit"):
        parser.feed(b"--frame\r\nContent-Length: 999999999\r\n\r\n")
    assert parser.buffered < 1024


def test_parser_rejects_an_oversized_header_block():
    parser = cam.MultipartParser("frame", max_header_bytes=256)
    with pytest.raises(cam.CameraProtocolError, match="part headers exceeded"):
        parser.feed(b"--frame\r\n" + b"X-Pad: " + b"p" * 400)


def test_parser_bounds_its_buffer_when_no_boundary_ever_arrives():
    """A peer that answers something other than our multipart stream must not
    be able to grow this buffer without limit."""
    parser = cam.MultipartParser("frame", max_scan_bytes=4096)
    with pytest.raises(cam.CameraProtocolError, match="no multipart boundary"):
        for _ in range(10):
            parser.feed(b"x" * 1000)


def test_parser_retains_at_most_one_frame_plus_a_chunk():
    """Steady-state memory: feeding many parts leaves the buffer near empty,
    which is the structural half of "no per-client queue"."""
    blob, _ = stream_bytes(50)
    parser = cam.MultipartParser("frame")
    peak = 0
    for i in range(0, len(blob), 977):
        parser.feed(blob[i:i + 977])
        peak = max(peak, parser.buffered)
    assert parser.buffered < 977
    assert peak < len(make_jpeg()) + 977 + cam.MAX_PART_HEADER_BYTES


def test_parse_boundary_reads_the_content_type_parameter():
    assert cam.parse_boundary("multipart/x-mixed-replace; boundary=frame") == "frame"
    assert cam.parse_boundary('multipart/x-mixed-replace; boundary="fr me"') == "fr me"
    assert cam.parse_boundary("image/jpeg") == ""


def test_a_parser_with_no_boundary_refuses_to_be_constructed():
    with pytest.raises(cam.CameraProtocolError, match="no boundary"):
        cam.MultipartParser("")


# =============================================================================
# 3. The latest-frame slot -- bounded memory, counted skips
# =============================================================================

def _frame(seq: int, size: int = 1024) -> cam.CameraFrame:
    return cam.CameraFrame(generation=1, seq=seq, jpeg=bytes(size),
                           host_recv_mono=HostMono(float(seq)))


@asyncio_test
async def test_slot_retains_exactly_one_frame_however_many_are_published():
    """The bounded-memory claim, asserted as a COUNT of live objects.

    weakrefs, not `sys.getsizeof`: the question is whether the slot holds
    references to superseded frames, and only object liveness answers that.
    """
    slot = cam.LatestFrameSlot()
    refs = []
    for i in range(50):
        f = _frame(i)
        refs.append(weakref.ref(f))
        slot.publish(f)
        del f
    gc.collect()
    alive = [r for r in refs if r() is not None]
    assert len(alive) == 1
    assert alive[0]().seq == 49


@asyncio_test
async def test_a_stalled_subscriber_resumes_at_the_newest_frame_and_counts_skips():
    """§8: "Slow host proxy client -> next selected frame is newest"."""
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(0))
    tick, first = await slot.wait_next(0)
    assert first.seq == 0

    for i in range(1, 11):          # ten frames while the subscriber is away
        slot.publish(_frame(i))
    tick2, latest = await slot.wait_next(tick)
    assert latest.seq == 10, "a stalled subscriber must not be handed frame 1"
    assert slot.skipped == 9, "the nine frames it jumped over must be counted"


@asyncio_test
async def test_two_subscribers_both_get_the_same_frame():
    """The slot does NOT pop. `MeshPrep.latest()` pops because its producer is
    finite and its consumer single; here two browser tabs must not steal frames
    from each other."""
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(7))
    a = await slot.wait_next(0)
    b = await slot.wait_next(0)
    assert a[1] is b[1]
    assert a[1].seq == 7


@asyncio_test
async def test_wait_next_blocks_until_something_newer_is_published():
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(1))
    tick, _ = await slot.wait_next(0)
    waiter = asyncio.create_task(slot.wait_next(tick))
    await asyncio.sleep(0)
    assert not waiter.done(), "wait_next returned without a newer frame"
    slot.publish(_frame(2))
    got_tick, frame = await asyncio.wait_for(waiter, timeout=1.0)
    assert frame.seq == 2
    assert got_tick > tick


@asyncio_test
async def test_a_publish_racing_a_waiter_is_not_lost():
    """The event is grabbed BEFORE the state is read. Reversing those two lines
    loses a publish that lands between them, and the symptom is a stream that
    stalls until the NEXT frame -- intermittent, and invisible at 30 fps."""
    slot = cam.LatestFrameSlot()
    waiter = asyncio.create_task(slot.wait_next(0))
    await asyncio.sleep(0)
    slot.publish(_frame(5))
    _, frame = await asyncio.wait_for(waiter, timeout=1.0)
    assert frame.seq == 5


@asyncio_test
async def test_clear_drops_the_frame_and_wakes_waiters():
    slot = cam.LatestFrameSlot()
    slot.publish(_frame(1))
    ref = weakref.ref(slot.latest()[1])
    slot.clear()
    gc.collect()
    assert slot.latest()[1] is None
    assert ref() is None


# =============================================================================
# 4. CameraClient -- a fake upstream, no real socket
# =============================================================================

class FakeReader:
    """Feeds a canned byte script in the chunk sizes a test chooses.

    Explicitly NOT a socket: it cannot show TCP segmentation, a half-open
    connection, or a peer that stops without closing. Those are live-probe
    questions (see this module's docstring).
    """

    def __init__(self, chunks: list[bytes], *, then_eof: bool = True):
        self._chunks = list(chunks)
        self._then_eof = then_eof
        self._buf = bytearray()

    async def readuntil(self, sep: bytes) -> bytes:
        while sep not in self._buf:
            if not self._chunks:
                raise asyncio.IncompleteReadError(bytes(self._buf), None)
            self._buf += self._chunks.pop(0)
        idx = self._buf.find(sep) + len(sep)
        out = bytes(self._buf[:idx])
        del self._buf[:idx]
        return out

    async def readexactly(self, n: int) -> bytes:
        while len(self._buf) < n:
            if not self._chunks:
                raise asyncio.IncompleteReadError(bytes(self._buf), n)
            self._buf += self._chunks.pop(0)
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    async def read(self, n: int = -1) -> bytes:
        if self._buf:
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out
        if self._chunks:
            return self._chunks.pop(0)
        if self._then_eof:
            return b""
        await asyncio.sleep(3600)       # a peer that never speaks again
        return b""


class FakeWriter:
    """A StreamWriter stand-in that records BOTH halves of the close.

    `close()` only requests the close; `wait_closed()` is what makes it a fact.
    The fake tracks them separately so a teardown that skips the await is
    visible here rather than only as a flaky socket in production.
    """

    def __init__(self):
        self.written = bytearray()
        self.closed = False
        self.close_awaited = False

    def write(self, data: bytes) -> None:
        self.written += data

    def close(self) -> None:
        self.closed = True

    async def wait_closed(self) -> None:
        self.close_awaited = True


def http_head(boundary: str = "frame", status: int = 200) -> bytes:
    return (f"HTTP/1.1 {status} OK\r\n"
            f"Content-Type: multipart/x-mixed-replace; boundary={boundary}\r\n"
            f"\r\n").encode()


class FakeDialler:
    """Hands out a scripted (reader, writer) per connection attempt."""

    def __init__(self, scripts):
        self._scripts = list(scripts)
        self.attempts = 0
        self.writers: list[FakeWriter] = []
        self.targets: list[tuple[str, int]] = []

    async def __call__(self, host, port):
        self.attempts += 1
        self.targets.append((host, port))
        if not self._scripts:
            raise ConnectionRefusedError("no more scripted connections")
        script = self._scripts.pop(0)
        if isinstance(script, Exception):
            raise script
        writer = FakeWriter()
        self.writers.append(writer)
        return FakeReader(script), writer


async def _no_status(host, port):
    """The status poll's dialler in the ingest tests: always refused.

    Deliberate -- it proves the status poll failing cannot disturb the frame
    path, and it keeps the scripted MJPEG connections for the ingest loop that
    is under test. `/status.json` gets its own test below.
    """
    raise ConnectionRefusedError("status poll disabled in this test")


def make_client(scripts, *, sleep=None, **cfg_kw) -> tuple[cam.CameraClient, FakeDialler]:
    cfg = cam.CameraIngestConfig(host="10.0.0.9", status_interval_s=3600.0, **cfg_kw)
    dialler = FakeDialler(scripts)
    client = cam.CameraClient(cfg, connect=dialler, status_connect=_no_status,
                              sleep=sleep)
    return client, dialler


class SleepLog:
    """Records every backoff delay the client ASKS for, then yields the loop.

    The delays requested are the schedule; wall-clock attempt counts are only a
    blurry proxy for it, and a monotone backoff is invisible in that proxy
    until the cap is reached. Sleeping ~0 keeps the test fast while leaving the
    ordering of the ingest loop intact.
    """

    def __init__(self):
        self.delays: list[float] = []

    async def __call__(self, delay: float) -> None:
        self.delays.append(float(delay))
        await asyncio.sleep(0)


async def drain(client: cam.CameraClient, *, frames: int, timeout: float = 2.0):
    """Wait until `frames` frames have been published, or fail loudly."""
    deadline = asyncio.get_running_loop().time() + timeout
    while client.frames < frames:
        if asyncio.get_running_loop().time() > deadline:
            raise AssertionError(
                f"only {client.frames}/{frames} frames after {timeout}s "
                f"(last_error={client.last_error!r})")
        await asyncio.sleep(0.005)


@asyncio_test
async def test_client_publishes_frames_byte_identical_to_the_wire():
    blob, jpegs = stream_bytes(3)
    client, dialler = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=3)
        _, frame = client.slot.latest()
        assert frame.jpeg == jpegs[-1], "the relayed JPEG must be the exact bytes"
        assert client.bytes == sum(len(j) for j in jpegs)
        assert dialler.attempts == 1, "one upstream connection, not one per frame"
    finally:
        await client.stop()


@asyncio_test
async def test_client_reads_dimensions_off_the_jpeg_not_off_a_header():
    """A daemon asserting 1280x720 while sending 640x480 must read as 640x480."""
    jpeg = make_jpeg(640, 480)
    part = part_bytes(jpeg, seq=1).replace(
        b"X-Roomscan-Camera-Seq: 1",
        b"X-Roomscan-Camera-Seq: 1\r\nX-Roomscan-Camera-Width: 1280")
    client, _ = make_client([[http_head(), part]])
    await client.start()
    try:
        await drain(client, frames=1)
        _, frame = client.slot.latest()
        assert (frame.width, frame.height) == (640, 480)
    finally:
        await client.stop()


@asyncio_test
async def test_client_counts_a_malformed_body_and_keeps_the_connection():
    """A correctly-framed part carrying a broken JPEG is a dropped FRAME, not a
    dropped CONNECTION: the stream is still in sync, so reconnecting would
    throw away good frames to punish one bad one."""
    good = make_jpeg(filler=7)
    broken = good[:-2]                       # EOI chopped, length still honest
    blob = part_bytes(broken, seq=1) + part_bytes(good, seq=2)
    client, dialler = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.malformed == 1
        assert client.frames == 1
        _, frame = client.slot.latest()
        assert frame.jpeg == good
        assert dialler.attempts == 1
    finally:
        await client.stop()


@asyncio_test
async def test_client_counts_source_sequence_gaps_as_drops():
    jpeg = make_jpeg()
    blob = (part_bytes(jpeg, seq=10) + part_bytes(jpeg, seq=11)
            + part_bytes(jpeg, seq=15))     # 12,13,14 never arrived
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=3)
        assert client.source_drops == 3
    finally:
        await client.stop()


@asyncio_test
async def test_a_source_generation_change_bumps_the_host_generation():
    """A camera reopen on the Pi is a new generation here too -- and must NOT
    be read as ~10 lost frames just because the sequence restarted."""
    jpeg = make_jpeg()
    blob = (part_bytes(jpeg, seq=99, generation=1)
            + part_bytes(jpeg, seq=0, generation=2)
            + part_bytes(jpeg, seq=1, generation=2))
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=3)
        assert client.source_drops == 0, "a reopen is not 100 lost frames"
        assert client.source_generation == 2
        # Exactly two: one for the connect, one for the reopen. Not three -- the
        # first part of a connection must not bump on top of the connect.
        assert client.generation == 2
    finally:
        await client.stop()


@asyncio_test
async def test_a_bare_sequence_reset_also_starts_a_new_generation():
    """Same reopen, but a daemon that forgot to bump its own generation. The
    host must still notice, because the browser's cache-bust keys off this."""
    jpeg = make_jpeg()
    blob = (part_bytes(jpeg, seq=42, generation=1)
            + part_bytes(jpeg, seq=1, generation=1))
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=2)
        assert client.generation == 2, (
            "the sequence going backwards must start a new host generation")
        assert client.source_drops == 0
    finally:
        await client.stop()


@asyncio_test
async def test_client_reconnects_after_the_body_ends_and_bumps_generation():
    blob1, _ = stream_bytes(2)
    blob2, _ = stream_bytes(2)
    client, dialler = make_client([[http_head(), blob1], [http_head(), blob2]],
                                  min_backoff_s=0.01, max_backoff_s=0.02)
    await client.start()
    try:
        await drain(client, frames=4, timeout=3.0)
        assert dialler.attempts == 2
        assert client.reconnects == 1
        _, frame = client.slot.latest()
        assert frame.generation >= 2
    finally:
        await client.stop()


@asyncio_test
async def test_backoff_grows_but_stays_bounded():
    """Three refused dials in a row must not busy-spin, and must not walk past
    the cap. Asserted as an ATTEMPT COUNT inside a fixed wall-clock window --
    the number a runaway retry loop would blow past."""
    refusals = [ConnectionRefusedError("nothing there") for _ in range(20)]
    client, dialler = make_client(refusals, min_backoff_s=0.02, max_backoff_s=0.05)
    await client.start()
    try:
        await asyncio.sleep(0.35)
        # min 0.02 doubling to a 0.05 cap => at most ~1 + 0.35/0.02 attempts,
        # and materially more than one (it must keep trying).
        assert 2 <= dialler.attempts <= 18, dialler.attempts
    finally:
        await client.stop()


@asyncio_test
async def test_a_non_200_response_is_reported_not_silently_retried_forever():
    """A daemon that is up but refusing must name ITS status, every attempt --
    not degrade into a generic connect error the next retry overwrites."""
    async def dial(host, port):
        return FakeReader([http_head(status=503)]), FakeWriter()

    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 min_backoff_s=0.01, max_backoff_s=0.02)
    client = cam.CameraClient(cfg, connect=dial, status_connect=_no_status)
    await client.start()
    try:
        await asyncio.sleep(0.1)
        assert client.last_error is not None
        assert "503" in client.last_error
        assert client.frames == 0
    finally:
        await client.stop()


@asyncio_test
async def test_timestamp_without_a_named_domain_is_invalid_and_unused():
    """§8 "Timestamp flag absent": the capture stamp becomes null, and arrival
    time is NOT renamed capture time."""
    jpeg = make_jpeg()
    blob = part_bytes(jpeg, seq=1, capture_ns=123456789, domain=None)
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        _, frame = client.slot.latest()
        assert frame.timestamp_valid is False
        assert frame.pi_capture_mono_ns is None
        assert frame.host_recv_mono is not None
        assert client.status()["capture_timestamp_valid"] is False
    finally:
        await client.stop()


@asyncio_test
async def test_an_unfamiliar_timestamp_domain_is_also_invalid():
    jpeg = make_jpeg()
    blob = part_bytes(jpeg, seq=1, capture_ns=5, domain="wall_clock")
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        _, frame = client.slot.latest()
        assert frame.timestamp_valid is False
        assert frame.timestamp_domain == "wall_clock"
    finally:
        await client.stop()


@asyncio_test
async def test_the_daemons_own_valid_false_revokes_a_well_formed_stamp():
    """The header name the daemon ACTUALLY sends (review round 1, 2026-09-01).

    Everything else about the part is impeccable -- `pi_monotonic` domain, a
    positive capture stamp -- so a host reading only the old
    `X-Roomscan-Timestamp-Valid` spelling would publish
    `timestamp_valid=True` and hand Stage B a capture time the daemon had
    explicitly disowned. That is the whole failure this pins: not a crash, a
    quietly wrong "true".
    """
    jpeg = make_jpeg()
    blob = part_bytes(jpeg, seq=1, capture_ns=7_000_000_000,
                      domain="pi_monotonic", valid="false")
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        _, frame = client.slot.latest()
        assert frame.timestamp_valid is False
        assert frame.pi_capture_mono_ns is None
        assert client.status()["capture_fps"] is None, (
            "a disowned stamp must not feed the capture cadence")
    finally:
        await client.stop()


@asyncio_test
async def test_the_daemons_valid_true_is_accepted_and_the_old_spelling_still_works():
    jpeg = make_jpeg()
    for header_kwargs in ({"valid": "true"}, {}):
        blob = part_bytes(jpeg, seq=1, capture_ns=7_000_000_000, **header_kwargs)
        client, _ = make_client([[http_head(), blob]])
        await client.start()
        try:
            await drain(client, frames=1)
            assert client.slot.latest()[1].timestamp_valid is True, header_kwargs
        finally:
            await client.stop()

    # the legacy alias, still honoured for a daemon of the older vintage
    blob = (part_bytes(jpeg, seq=1, capture_ns=7_000_000_000)
            .replace(b"X-Roomscan-Camera-Seq: 1",
                     b"X-Roomscan-Camera-Seq: 1\r\nX-Roomscan-Timestamp-Valid: 0"))
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.slot.latest()[1].timestamp_valid is False
    finally:
        await client.stop()


@asyncio_test
async def test_a_valid_true_cannot_grant_validity_the_evidence_does_not_support():
    """The flag revokes, never grants.

    `Valid: true` beside an unnamed timestamp domain leaves us with a number in
    no known clock. Trusting the flag over the evidence we can check ourselves
    is the invariant-preserving check that verifies nothing (BUG-059).
    """
    jpeg = make_jpeg()
    blob = part_bytes(jpeg, seq=1, capture_ns=5, domain=None, valid="true")
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.slot.latest()[1].timestamp_valid is False
    finally:
        await client.stop()


@asyncio_test
async def test_the_dequeue_stamp_is_relayed_but_never_becomes_the_capture_time():
    """§5.2: "dequeue time is recorded separately ... never renamed capture time".

    Both are plausible-looking Pi nanoseconds, so a fallback from one to the
    other would be invisible rather than wrong-looking -- which is exactly why
    the two are asserted to be different values on the same frame, and why the
    no-capture-stamp case must leave the capture time NULL rather than reaching
    for the dequeue instant that is sitting right there.
    """
    jpeg = make_jpeg()
    blob = part_bytes(jpeg, seq=1, capture_ns=1_000, dequeue_ns=9_999)
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        frame = client.slot.latest()[1]
        assert int(frame.pi_capture_mono_ns) == 1_000
        assert int(frame.pi_dequeue_mono_ns) == 9_999
    finally:
        await client.stop()

    # no capture stamp at all, dequeue present: capture time stays null
    blob = part_bytes(jpeg, seq=1, capture_ns=None, dequeue_ns=9_999)
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        frame = client.slot.latest()[1]
        assert frame.pi_capture_mono_ns is None
        assert frame.timestamp_valid is False
        assert int(frame.pi_dequeue_mono_ns) == 9_999
        assert client.status()["capture_fps"] is None
    finally:
        await client.stop()


def test_bool_or_none_distinguishes_unknown_from_false():
    """An unparseable flag is UNKNOWN, not False.

    Reading garbage as False would let a typo in one header silently revoke
    every capture timestamp on the stream, which is the loudest possible
    consequence of the quietest possible defect.
    """
    assert cam._bool_or_none("true") is True
    assert cam._bool_or_none("TRUE") is True
    assert cam._bool_or_none(" 1 ") is True
    assert cam._bool_or_none("false") is False
    assert cam._bool_or_none("0") is False
    assert cam._bool_or_none(None) is None
    assert cam._bool_or_none("") is None
    assert cam._bool_or_none("maybe") is None


@asyncio_test
async def test_capture_fps_is_measured_on_the_pi_clock_alone():
    """30 fps of Pi timestamps must read ~30, whatever the host clock does.

    The frames are fed as fast as the loop will take them, so a `capture_fps`
    derived from arrival times would read in the thousands. The assertion is an
    INTERVAL around 30, which is the number only a Pi-to-Pi difference produces.
    """
    jpeg = make_jpeg()
    base = 5_000_000_000
    blob = b"".join(part_bytes(jpeg, seq=i + 1, capture_ns=base + i * 33_333_333)
                    for i in range(30))
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=30)
        fps = client.status()["capture_fps"]
        assert fps is not None
        assert 29.0 < fps < 31.0, fps
    finally:
        await client.stop()


@asyncio_test
async def test_capture_fps_is_null_not_stale_once_frames_stop():
    """`browser_fps` (#217) read a healthy 30 while the client was seconds
    behind. A rate window that keeps reporting after arrivals stop is that
    same lie."""
    jpeg = make_jpeg()
    blob = b"".join(part_bytes(jpeg, seq=i + 1, capture_ns=i * 33_000_000)
                    for i in range(5))
    now = [1000.0]
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0, stale_after_s=2.0)
    client = cam.CameraClient(cfg, connect=FakeDialler([[http_head(), blob]]),
                              status_connect=_no_status, clock=lambda: now[0])
    await client.start()
    try:
        await drain(client, frames=5)
        assert client.status()["capture_fps"] is not None
        now[0] += 30.0                      # half a minute of silence
        assert client.status()["capture_fps"] is None
    finally:
        await client.stop()


def test_rate_window_reports_zero_once_arrivals_stop():
    """The same rule for `host_fps`, tested on `_Rate` directly.

    A windowed rate is a history, not a present-tense reading: once nothing
    arrives, it must fall to a measured 0.0 rather than keep reporting the last
    healthy window -- `browser_fps` (#217) read 30 while the client was seconds
    behind, and that is the shape of the failure.
    """
    rate = cam._Rate()
    for i in range(10):
        rate.add(100.0 + i * 0.05)          # 20 Hz
    assert rate.hz(100.45) == pytest.approx(20.0, rel=0.05)
    assert rate.hz(110.0) == 0.0            # ten seconds of silence
    assert cam._Rate().hz(0.0) is None      # nothing measured yet -> unknown


@asyncio_test
async def test_age_ms_measures_the_interval_since_arrival():
    """An INTERVAL assertion, not a type one: a clock/units error (BUG-050
    reported 1.78e9 s for a 90 s take) survives `isinstance(x, float)`."""
    blob, _ = stream_bytes(1)
    now = [500.0]
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0)
    client = cam.CameraClient(cfg, connect=FakeDialler([[http_head(), blob]]),
                              status_connect=_no_status, clock=lambda: now[0])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.status()["age_ms"] == pytest.approx(0.0, abs=1e-6)
        now[0] += 0.75
        assert client.status()["age_ms"] == pytest.approx(750.0, abs=1e-3)
    finally:
        await client.stop()


@asyncio_test
async def test_state_walks_offline_connecting_live_stale():
    blob, _ = stream_bytes(1)
    now = [10.0]
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 stale_after_s=2.0, min_backoff_s=0.01)
    # A reader that never EOFs, so the body stays open after its one frame --
    # which is exactly the "connected but frames stopped" case.
    dialler = FakeDialler([])

    async def dial(host, port):
        dialler.attempts += 1
        writer = FakeWriter()
        dialler.writers.append(writer)
        return FakeReader([http_head(), blob], then_eof=False), writer

    client = cam.CameraClient(cfg, connect=dial, status_connect=_no_status,
                              clock=lambda: now[0])
    assert client.state() == "offline", "a client that was never started is Offline"
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.state() == "live"
        now[0] += 5.0
        assert client.state() == "stale", "connected but silent is Stale, not Offline"
    finally:
        await client.stop()
    assert client.state() == "offline"


@asyncio_test
async def test_repeated_connect_failures_read_as_offline_not_forever_connecting():
    client, _ = make_client([ConnectionRefusedError("x") for _ in range(20)],
                            min_backoff_s=0.01, max_backoff_s=0.02,
                            offline_after_failures=2)
    await client.start()
    try:
        await asyncio.sleep(0.15)
        assert client.state() == "offline"
        assert client.last_error and "ConnectionRefused" in client.last_error
    finally:
        await client.stop()


@asyncio_test
async def test_a_device_absent_daemon_reads_as_offline_even_while_connected():
    blob, _ = stream_bytes(1)
    client, _ = make_client([[http_head(), blob]])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.state() == "live"
        client.source_status = {"state": "device_absent"}
        assert client.state() == "offline"
    finally:
        await client.stop()


@asyncio_test
async def test_backoff_resets_after_a_connection_that_actually_delivered_frames():
    """The delay a working camera pays after a daemon restart.

    Four refusals walk the backoff up; then a connection delivers real frames
    and drops. The NEXT delay must be the floor again. Before the fix the reset
    lived only on `_connect_and_stream`'s normal-return path -- which is
    reached only at shutdown -- so the backoff was monotone for the life of the
    process: a camera absent at boot pinned it at the 10 s cap, and every
    daemon restart after that cost 10 s of Offline on a healthy link.

    Asserted on the INTERVALS requested, not on a flag: a boolean "did we
    reset" would agree with a reset that computed the wrong number.
    """
    blob, _ = stream_bytes(3)
    log = SleepLog()
    scripts = [ConnectionRefusedError("nothing there") for _ in range(4)]
    scripts.append([http_head(), blob])                 # this one works
    scripts.extend(ConnectionRefusedError("gone again") for _ in range(3))
    client, _ = make_client(scripts, sleep=log,
                            min_backoff_s=0.5, max_backoff_s=10.0)
    await client.start()
    try:
        await drain(client, frames=3, timeout=3.0)
        for _ in range(200):                            # let the drop land
            if len(log.delays) >= 6:
                break
            await asyncio.sleep(0.002)
    finally:
        await client.stop()

    assert log.delays[:4] == [0.5, 1.0, 2.0, 4.0], log.delays
    assert log.delays[4] == 0.5, (
        f"the delay after a productive connection must be the floor, "
        f"not {log.delays[4]} -- the backoff never reset: {log.delays}")
    assert log.delays[5] == 1.0, log.delays


@asyncio_test
async def test_backoff_does_not_reset_for_a_daemon_that_accepts_then_wedges():
    """A response head is not a working camera.

    Resetting on a successful connect would turn a daemon that accepts and then
    sends nothing into a 0.5 s busy-loop. Only DELIVERED frames earn the floor.
    """
    log = SleepLog()
    heads = [[http_head()] for _ in range(5)]           # 200 OK, then EOF
    client, _ = make_client(heads, sleep=log,
                            min_backoff_s=0.5, max_backoff_s=10.0)
    await client.start()
    try:
        for _ in range(400):
            if len(log.delays) >= 4:
                break
            await asyncio.sleep(0.002)
    finally:
        await client.stop()
    assert client.frames == 0
    assert log.delays[:4] == [0.5, 1.0, 2.0, 4.0], log.delays


@asyncio_test
async def test_state_is_connecting_not_stale_before_this_connections_first_frame():
    """The daemon answered and is still opening the camera.

    `stale` is a claim about a stream that WAS running and stopped; before the
    first part there is no such stream. `age_s` alone cannot tell them apart --
    it survives across reconnects -- so a card would paint Stale the moment a
    reconnect's response head landed, which reads as a fault on a camera that
    is merely warming up.
    """
    now = [100.0]
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 stale_after_s=2.0, min_backoff_s=0.01)

    async def dial(host, port):
        # Response head only; the body never produces a part.
        return FakeReader([http_head()], then_eof=False), FakeWriter()

    client = cam.CameraClient(cfg, connect=dial, status_connect=_no_status,
                              clock=lambda: now[0])
    await client.start()
    try:
        for _ in range(200):
            if client._connected:
                break
            await asyncio.sleep(0.002)
        assert client._connected, "the fake never got as far as the response head"
        assert client.state() == "connecting"
        now[0] += 60.0                     # a long silence is still Connecting
        assert client.state() == "connecting", (
            "no frame has EVER arrived on this connection; there is no stream "
            "to have gone stale")
    finally:
        await client.stop()


@asyncio_test
async def test_state_is_stale_only_after_a_frame_then_silence():
    """The other half of the pair: once a frame HAS arrived on this
    connection, silence is Stale. Without both, either state could be a
    constant and still pass."""
    blob, _ = stream_bytes(1)
    now = [100.0]
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 stale_after_s=2.0, min_backoff_s=0.01)

    async def dial(host, port):
        return FakeReader([http_head(), blob], then_eof=False), FakeWriter()

    client = cam.CameraClient(cfg, connect=dial, status_connect=_no_status,
                              clock=lambda: now[0])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.state() == "live"
        now[0] += 60.0
        assert client.state() == "stale"
    finally:
        await client.stop()


@asyncio_test
async def test_a_second_start_cannot_leak_a_live_status_poller():
    """`start()` guards on BOTH tasks.

    With the ingest task finished but the poller alive, a guard on `_task`
    alone lets a second `start()` through, which overwrites `_status_task` and
    strands the old poller: two pollers dialling the daemon forever, and
    `stop()` able to join only the newer one. Asserted as a task census.
    """
    client, _ = make_client([], min_backoff_s=5.0)
    await client.start()
    poller = client._status_task
    client._task.cancel()                  # the ingest half dies, the poller lives
    try:
        await client._task
    except asyncio.CancelledError:
        pass
    assert not poller.done()

    before = len(asyncio.all_tasks())
    await client.start()                   # must be refused
    try:
        assert client._status_task is poller, "start() replaced the live poller"
        assert len(asyncio.all_tasks()) == before, "start() spawned a second poller"
    finally:
        await client.stop()
    assert poller.done()


def status_head(body: bytes) -> list[bytes]:
    """A complete `GET /status.json` response, as the daemon answers it."""
    return [(f"HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n"
             f"Content-Length: {len(body)}\r\n\r\n").encode() + body]


@asyncio_test
async def test_the_status_poll_turns_a_live_camera_offline_when_it_is_unplugged():
    """§8 "Camera unplugged live", seen through `/status.json`.

    The MJPEG body can stay open across an unplug -- the daemon holds the
    socket and simply stops producing -- so the frames-based signal alone would
    read `stale`, which says "something is slow". `device_absent` is the
    daemon telling us the hardware is gone, which is a different fact and the
    one a viewer needs. This drives it through the real status poll rather than
    by assigning `source_status`, so the polling path itself is exercised.
    """
    blob, _ = stream_bytes(2)
    now = [500.0]
    reports = [b'{"state": "streaming"}', b'{"state": "device_absent"}']

    async def dial(host, port):
        return FakeReader([http_head(), blob], then_eof=False), FakeWriter()

    async def status_dial(host, port):
        body = reports[0] if len(reports) == 1 else reports.pop(0)
        return FakeReader(status_head(body)), FakeWriter()

    cfg = cam.CameraIngestConfig(host="h", stale_after_s=2.0,
                                 status_interval_s=0.01, min_backoff_s=0.01)
    client = cam.CameraClient(cfg, connect=dial, status_connect=status_dial,
                              clock=lambda: now[0])
    await client.start()
    try:
        await drain(client, frames=2)
        for _ in range(300):                      # first poll: streaming
            if client.source_status == {"state": "streaming"}:
                break
            await asyncio.sleep(0.002)
        assert client.state() == "live"

        for _ in range(300):                      # second poll: unplugged
            if (client.source_status or {}).get("state") == "device_absent":
                break
            await asyncio.sleep(0.002)
        assert client.state() == "offline", (
            "a daemon reporting device_absent is Offline, not Stale -- the "
            "hardware is gone, the stream is not merely slow")
        assert client.status()["source"] == {"state": "device_absent"}, (
            "the daemon's own report must be passed through verbatim")
    finally:
        await client.stop()


@asyncio_test
async def test_a_body_that_closes_goes_offline_then_live_again_on_reconnect():
    """§8's other unplug row: the daemon drops the response body.

    Note what the state does FIRST: for `stale_after_s` after the drop it is
    still `live`, deliberately -- the picture on screen genuinely is that
    fresh, and flipping a card to Offline for a reconnect that completes in
    120 ms would be noise, not information. Only once the last frame is
    actually old AND the dials keep failing is it Offline, which is why this
    drives the clock rather than sleeping.

    Coming back must bump the host generation: the browser's cache-bust keys
    off it, so a resumed stream reusing the old generation would leave every
    viewer attached to a dead request.
    """
    blob1, _ = stream_bytes(1)
    blob2, _ = stream_bytes(1)
    now = [1000.0]
    dialler = FakeDialler([[http_head(), blob1],
                           ConnectionRefusedError("daemon gone"),
                           ConnectionRefusedError("daemon gone"),
                           [http_head(), blob2]])
    cfg = cam.CameraIngestConfig(host="h", status_interval_s=3600.0,
                                 stale_after_s=1.0, min_backoff_s=0.01,
                                 max_backoff_s=0.02, offline_after_failures=2)
    client = cam.CameraClient(cfg, connect=dialler, status_connect=_no_status,
                              clock=lambda: now[0])
    await client.start()
    try:
        await drain(client, frames=1)
        assert client.state() == "live"
        gen_before = client.slot.latest()[1].generation

        for _ in range(400):                      # the two refusals land
            if client._consecutive_failures >= 2:
                break
            await asyncio.sleep(0.002)
        assert client.state() == "live", (
            "a drop inside the freshness window is not yet Offline -- the "
            "displayed frame really is that recent")
        now[0] += 30.0
        assert client.state() == "offline", (
            "an old last frame plus repeated failed dials is a camera that is "
            "gone, not one that is still connecting")

        await drain(client, frames=2, timeout=3.0)
        assert client.state() == "live"
        gen_after = client.slot.latest()[1].generation
        assert gen_after > gen_before, (
            f"a reconnect must bump the host generation ({gen_before} -> "
            f"{gen_after}); a viewer would otherwise keep a dead <img> request")
    finally:
        await client.stop()


@asyncio_test
async def test_stop_leaves_no_task_and_closes_the_socket():
    """§9 "Lifespan: ... worker self-cleanup, no leaked task/thread/process"."""
    blob, _ = stream_bytes(1)
    before = len(asyncio.all_tasks())
    client, dialler = make_client([[http_head(), blob]], min_backoff_s=5.0)
    await client.start()
    await drain(client, frames=1)
    assert len(asyncio.all_tasks()) > before
    ingest, status = client._task, client._status_task
    await client.stop()
    # Checked with NO intervening await: `stop()` must JOIN, not merely request
    # cancellation. A cancel-and-return leaves these pending here and only
    # finishes if something else yields to the loop afterwards -- which is the
    # difference between "we asked it to stop" and "it stopped" (#219).
    assert ingest.done() and status.done(), "stop() returned before its tasks ended"
    assert len(asyncio.all_tasks()) == before, "camera tasks outlived stop()"
    assert dialler.writers and all(w.closed for w in dialler.writers)
    assert all(w.close_awaited for w in dialler.writers), (
        "close() only REQUESTS the close; without wait_closed() the transport "
        "can still be flushing when stop() returns")
    assert client.slot.latest()[1] is None
    assert client.running is False


@asyncio_test
async def test_repeated_start_stop_cycles_do_not_accumulate_tasks():
    before = len(asyncio.all_tasks())
    for _ in range(4):
        blob, _ = stream_bytes(1)
        client, _ = make_client([[http_head(), blob]], min_backoff_s=5.0)
        await client.start()
        await drain(client, frames=1)
        await client.stop()
    assert len(asyncio.all_tasks()) == before


@asyncio_test
async def test_a_second_start_does_not_create_a_second_ingest():
    blob, _ = stream_bytes(1)
    client, dialler = make_client([[http_head(), blob]], min_backoff_s=5.0)
    await client.start()
    await client.start()
    try:
        await drain(client, frames=1)
        await asyncio.sleep(0.05)
        assert dialler.attempts == 1, "a second start() opened a second connection"
    finally:
        await client.stop()


@asyncio_test
async def test_stop_is_safe_before_start_and_twice():
    client, _ = make_client([])
    await client.stop()
    await client.start()
    await client.stop()
    await client.stop()


@asyncio_test
async def test_cancelling_the_ingest_task_does_not_raise_out_of_stop():
    blob, _ = stream_bytes(1)
    client, _ = make_client([[http_head(), blob]], min_backoff_s=5.0)
    await client.start()
    await drain(client, frames=1)
    client._task.cancel()               # someone else cancelled it first
    await asyncio.sleep(0)
    await client.stop()                 # must not propagate CancelledError
    assert client.running is False


@asyncio_test
async def test_stop_reraises_a_cancellation_aimed_at_its_own_caller():
    """Swallowing the shutdown signal is not cleanup.

    `stop()` legitimately absorbs the `CancelledError` from the tasks IT
    cancelled. It must not also absorb a cancellation aimed at the task running
    `stop()` -- on a real lifespan shutdown that is the signal to stop, and
    eating it leaves the server waiting on a teardown that already decided to
    end. Cleanup must still complete first, which is what the second half
    asserts.
    """
    blob, _ = stream_bytes(1)
    client, dialler = make_client([[http_head(), blob]], min_backoff_s=5.0)
    await client.start()
    await drain(client, frames=1)

    async def stopper():
        await client.stop()

    task = asyncio.create_task(stopper())
    await asyncio.sleep(0)
    task.cancel()                       # cancel the CALLER of stop(), not the client
    with pytest.raises(asyncio.CancelledError):
        await task
    assert client._task is None and client._status_task is None
    assert all(w.closed for w in dialler.writers), (
        "the socket must be released even when the cancellation is re-raised")
    assert client.slot.latest()[1] is None


@asyncio_test
async def test_the_resolver_is_used_for_the_dial_target():
    """The bridge address is resolved per attempt, not pinned at construction:
    wlan0 moves (#209), and a client that cached the first answer would never
    find the Pi again after a DHCP change."""
    blob, _ = stream_bytes(1)
    seen = []

    async def resolve():
        seen.append(1)
        return "172.17.2.77"

    cfg = cam.CameraIngestConfig(host="unused", status_interval_s=3600.0)
    dialler = FakeDialler([[http_head(), blob]])
    client = cam.CameraClient(cfg, resolve=resolve, connect=dialler,
                              status_connect=_no_status)
    await client.start()
    try:
        await drain(client, frames=1)
        assert dialler.targets[0] == ("172.17.2.77", cam.DEFAULT_CAMERA_PORT)
        assert client.status()["bridge_addr"] == "172.17.2.77"
        assert seen
    finally:
        await client.stop()


@asyncio_test
async def test_status_reports_nulls_before_anything_has_been_measured():
    """"Null means unknown; zero means measured zero" (§5.4). A fresh client
    has measured no rate and no age; reporting 0.0 for either would be a
    fabricated reading."""
    client, _ = make_client([])
    status = client.status()
    assert status["host_fps"] is None
    assert status["capture_fps"] is None
    assert status["age_ms"] is None
    assert status["width"] is None
    assert status["frames"] == 0          # measured zero, not unknown
    assert status["source_drops"] == 0
    assert status["ingest_drops"] == 0


# =============================================================================
# 5. The `camera` /ws message
# =============================================================================

def test_camera_message_is_state_and_carries_no_pixels():
    """§3 non-goal: "No camera bytes on /ws, /ws-mesh, or /ws-thin."

    Asserted on the SERIALISED message, because that is what actually goes on
    the wire -- a field holding bytes would either blow up json.dumps or ship
    a base64 payload, and both must be impossible.
    """
    status = {"state": "live", "generation": 3, "width": 1280, "height": 720}
    msg = cam.camera_message(status)
    assert msg["type"] == "camera"
    text = json.dumps(msg)
    assert len(text) < 4096, "the camera state message must stay small"
    assert "jpeg" not in text


def test_camera_message_is_edge_triggered_on_the_keys_a_viewer_reacts_to():
    base = {"state": "live", "generation": 1, "width": 1280, "height": 720,
            "age_ms": 33.0, "host_fps": 30.0, "capture_timestamp_valid": True,
            "source_generation": 1, "bridge_addr": "10.0.0.1"}
    assert cam.camera_message_changed(None, base) is True
    assert cam.camera_message_changed(base, dict(base)) is False
    # A pure telemetry tick must NOT trigger a send: at 30 fps that would turn
    # a <=4 Hz state message into a frame-rate JSON stream.
    assert cam.camera_message_changed(base, {**base, "age_ms": 66.0}) is False
    assert cam.camera_message_changed(base, {**base, "host_fps": 29.4}) is False
    # Anything a viewer must react to does.
    assert cam.camera_message_changed(base, {**base, "state": "stale"}) is True
    assert cam.camera_message_changed(base, {**base, "generation": 2}) is True
    assert cam.camera_message_changed(base, {**base, "width": 640}) is True


# =============================================================================
# 6. Agent surface -- snapshot path resolution (§5.6)
# =============================================================================

def test_snapshot_default_path_lands_under_results(tmp_path):
    path = cam.resolve_snapshot_path("", repo=tmp_path, now=1_700_000_000.5)
    assert path.parent == (tmp_path / "results" / "camera").resolve()
    assert path.suffix == ".jpg"
    assert "1700000000500" in path.name


def test_snapshot_relative_path_resolves_against_the_repo(tmp_path):
    (tmp_path / "captures").mkdir()
    path = cam.resolve_snapshot_path("captures/shot.jpg", repo=tmp_path)
    assert path == (tmp_path / "captures" / "shot.jpg").resolve()


@pytest.mark.parametrize("bad", [
    "/etc/passwd",
    "../outside.jpg",
    "results/../../escape.jpg",
    "host/src/roomscan/camera.py",
])
def test_snapshot_refuses_a_path_outside_the_allowed_roots(tmp_path, bad):
    """Refused, not clamped: silently relocating a caller's file is how a tool
    reports what was requested instead of what happened."""
    (tmp_path / "results").mkdir()
    (tmp_path / "captures").mkdir()
    with pytest.raises(ValueError, match="outside the allowed roots"):
        cam.resolve_snapshot_path(bad, repo=tmp_path)


def test_snapshot_refuses_a_symlink_that_escapes_the_roots(tmp_path):
    """`resolve()` before the containment check, not after: a symlink inside
    `results/` pointing out of the tree passes any string-prefix test."""
    (tmp_path / "results").mkdir()
    outside = tmp_path / "elsewhere"
    outside.mkdir()
    (tmp_path / "results" / "link").symlink_to(outside)
    with pytest.raises(ValueError, match="outside the allowed roots"):
        cam.resolve_snapshot_path("results/link/shot.jpg", repo=tmp_path)


def test_fetch_camera_status_reports_an_unreachable_server_rather_than_raising(
        monkeypatch):
    def boom(*a, **k):
        raise OSError("connection refused")
    monkeypatch.setattr(cam.urllib.request, "urlopen", boom)
    out = cam.fetch_camera_status("http://localhost:9")
    assert out["ok"] is False
    assert "connection refused" in out["error"]
    assert out["url"].endswith("/api/camera/status")


def test_save_camera_snapshot_writes_the_exact_bytes_and_reports_the_real_path(
        tmp_path, monkeypatch):
    jpeg = make_jpeg(1280, 720)

    class FakeResponse:
        headers = {"X-Roomscan-Camera-Generation": "4",
                   "X-Roomscan-Camera-Seq": "912",
                   "X-Roomscan-Camera-Age-Ms": "31.5"}

        def read(self):
            return jpeg

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    monkeypatch.setattr(cam.urllib.request, "urlopen", lambda *a, **k: FakeResponse())
    out = cam.save_camera_snapshot("http://localhost:8000", "", repo=tmp_path)
    assert out["ok"] is True
    written = (tmp_path / "results" / "camera").glob("*.jpg")
    written = list(written)
    assert len(written) == 1
    assert written[0].read_bytes() == jpeg, "the snapshot must be the exact JPEG"
    assert out["path"] == str(written[0])
    assert out["bytes"] == len(jpeg)
    assert (out["width"], out["height"]) == (1280, 720)
    assert (out["generation"], out["seq"]) == (4, 912)
    assert out["age_ms"] == 31.5


def test_snapshot_refuses_an_allowed_root_itself(tmp_path):
    """`out="results"` is a directory, not a snapshot path.

    The old check accepted `candidate == root`, so this sailed through
    resolution and then hit `write_bytes` on a directory -- `IsADirectoryError`
    raised out of a tool whose entire contract is to report rather than raise.
    """
    (tmp_path / "results").mkdir()
    (tmp_path / "captures").mkdir()
    for root in ("results", "captures"):
        with pytest.raises(ValueError, match="allowed ROOT directory"):
            cam.resolve_snapshot_path(root, repo=tmp_path)


def test_save_camera_snapshot_reports_a_failed_write_rather_than_raising(
        tmp_path, monkeypatch):
    """A full disk, a read-only root and a directory destination are ordinary
    answers to "save a snapshot". Asserted by making the write genuinely fail,
    not by mocking the return value."""
    jpeg = make_jpeg(64, 48)

    class FakeResponse:
        headers: dict = {}

        def read(self):
            return jpeg

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    monkeypatch.setattr(cam.urllib.request, "urlopen", lambda *a, **k: FakeResponse())
    # A DIRECTORY where the .jpg should be: resolution allows it (it is inside
    # `results/`), and only the write can discover it is not a file.
    (tmp_path / "results" / "camera").mkdir(parents=True)
    (tmp_path / "results" / "camera" / "shot.jpg").mkdir()
    out = cam.save_camera_snapshot("http://localhost:8000",
                                   "results/camera/shot.jpg", repo=tmp_path)
    assert out["ok"] is False
    assert "IsADirectoryError" in out["error"]
    assert out["path"].endswith("shot.jpg"), "the refused path must be reported"


def test_save_camera_snapshot_refuses_a_non_jpeg_response(tmp_path, monkeypatch):
    class FakeResponse:
        headers = {}

        def read(self):
            return b'{"error": "no camera frame yet"}'

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    monkeypatch.setattr(cam.urllib.request, "urlopen", lambda *a, **k: FakeResponse())
    out = cam.save_camera_snapshot("http://localhost:8000", "", repo=tmp_path)
    assert out["ok"] is False
    assert "not a JPEG" in out["error"]
    assert not list((tmp_path / "results").rglob("*.jpg"))


@asyncio_test
async def test_a_worker_raising_an_os_timeout_is_not_reported_as_a_slow_stop(caplog):
    """Since 3.11 `asyncio.TimeoutError` IS the builtin `TimeoutError`.

    So `except asyncio.TimeoutError` around the join also catches an OS-level
    timeout raised BY the worker -- a failed socket read logged as "the task
    would not stop". Opposite diagnoses (the worker died vs the bound is too
    tight), and the one that is true decides whether anyone looks at the
    socket. `asyncio.wait` never raises for time, so the two are separated by
    what is afterwards TRUE of the task: still pending, or finished with an
    exception.
    """
    import logging

    client, _ = make_client([], min_backoff_s=5.0)
    await client.start()

    async def dies_with_a_timeout():
        raise TimeoutError("[Errno 110] Connection timed out")

    # Replace the ingest task with one that fails the way a wedged socket does.
    client._task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await client._task
    client._task = asyncio.create_task(dies_with_a_timeout())
    await asyncio.sleep(0)

    with caplog.at_level(logging.WARNING, logger="roomscan.camera"):
        await client.stop()
    text = "\n".join(r.getMessage() for r in caplog.records)
    assert "did not stop within" not in text, (
        f"a worker that raised TimeoutError was reported as a slow stop: {text}")
    assert "raised TimeoutError" in text and "Errno 110" in text, text


@asyncio_test
async def test_a_worker_that_will_not_die_is_reported_as_a_slow_stop(caplog):
    """The other arm, so neither message can be a constant.

    The stand-in absorbs its cancellation and then takes far longer than the
    stop budget to unwind -- what a worker stuck in a native call looks like.
    It does eventually finish, deliberately: a task that swallowed
    cancellation FOREVER hung `asyncio.run`'s own teardown (which cancels and
    gathers whatever is left), so the test wedged and the code under test was
    never the problem. A fixture that cannot be cleaned up does not test
    robustness, it just moves the hang.
    """
    import logging

    client, _ = make_client([], min_backoff_s=5.0, stop_timeout_s=0.05)
    await client.start()

    async def slow_to_unwind():
        try:
            await asyncio.sleep(3600)
        except asyncio.CancelledError:
            # Absorbed once, then a real (bounded) unwind. A second cancel --
            # asyncio.run's teardown -- lands here and ends it.
            await asyncio.sleep(0.4)

    client._task.cancel()
    with pytest.raises(asyncio.CancelledError):
        await client._task
    stubborn = asyncio.create_task(slow_to_unwind())
    await asyncio.sleep(0)
    client._task = stubborn

    try:
        with caplog.at_level(logging.WARNING, logger="roomscan.camera"):
            await client.stop()
        text = "\n".join(r.getMessage() for r in caplog.records)
        assert "did not stop within" in text, text
        assert "raised" not in text, (
            f"a task that merely would not finish was reported as raising: {text}")
    finally:
        stubborn.cancel()
        with suppress(asyncio.CancelledError):
            await stubborn


@asyncio_test
async def test_the_writer_close_uses_the_configured_timeout_not_the_module_default():
    """`stop_timeout_s` is a config field so a test can shrink it; reading the
    module constant instead means a caller that lowered the bound silently did
    not."""
    slow = asyncio.Event()

    class _SlowWriter(FakeWriter):
        async def wait_closed(self):
            self.close_awaited = True
            await slow.wait()             # never completes within the test

    client, _ = make_client([], min_backoff_s=5.0, stop_timeout_s=0.05)
    client._writer = _SlowWriter()
    loop = asyncio.get_running_loop()
    t0 = loop.time()
    await client.stop()
    elapsed = loop.time() - t0
    assert elapsed < 1.0, (
        f"the writer close took {elapsed:.2f}s against a 0.05s configured "
        f"timeout -- it is reading the module constant instead")
    assert client._writer is None
