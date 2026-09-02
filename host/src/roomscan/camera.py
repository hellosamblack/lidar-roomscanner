"""Host ingest for the Pi-bridge webcam (#226 Stage A, spec §5.4).

One `CameraClient` per server process holds **one** TCP connection to the
`roomscan-cam` daemon on the Pi (port 5001 on its wlan0 address), parses its
`multipart/x-mixed-replace` MJPEG body incrementally, and publishes each
complete JPEG into a single immutable latest-frame slot. Every consumer -- the
`/api/camera/stream.mjpg` re-fanout, `/api/camera/frame.jpg`, the `camera`
WebSocket message, the MCP snapshot -- reads that one slot. Browser count
therefore never multiplies Pi camera opens (spec invariant §4.3), and the
memory the host holds for the camera is one JPEG, not one queue per viewer
(§4.1).

Design constraints this file exists to satisfy, each of which has a repo lesson
behind it:

* **Latest wins, never a backlog.** `LatestFrameSlot` stores exactly one frame.
  A subscriber that falls behind resumes at the newest frame and counts what it
  skipped; there is no per-client deque to grow. #217's "a growing multi-second
  lag is queueing, not loss" applies verbatim -- the failure mode a preview
  feed invites is latency, not dropout, so the bound is structural rather than
  a rate governor metered against an assumed drain rate.
* **Nothing here holds the GIL or blocks the loop.** The whole path is
  `asyncio` socket reads plus `bytes` slicing. There is no decode, no numpy, no
  native call: the JPEG is relayed byte-identical (which is also what Stage B
  needs it to be), and the only inspection performed is a marker walk over the
  JPEG header to read its ACTUAL dimensions.
* **Requested is not actual (§4.4).** Width/height come from the JPEG's own SOF
  marker, not from a header the daemon asserts and not from a configured mode.
  A daemon that negotiated 640x480 while advertising 720p is visible here.
* **Clock domains stay named (§4.5).** The Pi's capture timestamp is a
  `PiMonoNanos`, a third monotonic origin that shares no epoch with `HostMono`.
  `capture_fps` is derived from Pi-to-Pi deltas only; `age_ms` from host-to-host
  deltas only. The two are never subtracted from one another, and Stage A
  publishes no cross-clock number at all.

Wire contract consumed (spec §5.2) -- read defensively throughout, because the
daemon is written on a different branch:

    GET /stream.mjpg  -> 200, Content-Type: multipart/x-mixed-replace; boundary=frame
      --frame\\r\\n
      Content-Type: image/jpeg\\r\\n
      Content-Length: N\\r\\n
      X-Roomscan-Camera-Generation: G\\r\\n
      X-Roomscan-Camera-Seq: S\\r\\n
      X-Roomscan-Capture-Mono-Ns: T\\r\\n
      X-Roomscan-Capture-Timestamp-Valid: true|false\\r\\n
      X-Roomscan-Capture-Dequeue-Mono-Ns: D\\r\\n
      X-Roomscan-Timestamp-Domain: pi_monotonic\\r\\n
      \\r\\n
      <N bytes>\\r\\n

    `X-Roomscan-Capture-Dequeue-Mono-Ns` is when the V4L2 buffer was DEQUEUED,
    not when the sensor exposed. It is relayed untouched and never folded into
    the capture stamp or into `capture_fps`: §5.2 is explicit that "dequeue
    time is recorded separately for diagnostics and is never renamed capture
    time", and the whole reason the daemon sends both is that they are
    different facts. Both are plausible-looking Pi nanoseconds, so a fallback
    from one to the other would be invisible rather than wrong-looking.

    GET /frame.jpg    -> 200 image/jpeg + the same X-Roomscan-* headers
    GET /status.json  -> 200 JSON; passed through VERBATIM as `source` and read
                         only through `.get()`, so a field the daemon renames
                         or adds costs this module nothing.

`/clock.json` is deliberately NOT consumed: it exists for Stage B's (#227)
four-timestamp exchange, and Stage A makes no synchronization claim.
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
import urllib.error
import urllib.request
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path

from .domain import HostMono, PiMonoNanos, now_mono, pi_mono_delta_s

log = logging.getLogger("roomscan.camera")

#: The daemon's port on the bridge's wlan0 address (spec §5.3). Not the
#: scanner-facing eth0 side.
DEFAULT_CAMERA_PORT = 5001

#: Hard ceiling on one relayed JPEG. A C270 720p MJPEG frame measures in the
#: low tens of KB; 4 MiB is ~50x the p99 and still small enough that a garbage
#: `Content-Length` cannot make the host allocate its way out of memory. A part
#: claiming more than this is a protocol error, not a big frame.
MAX_FRAME_BYTES = 4 * 1024 * 1024

#: Ceiling on one part's header block, and on how far we will scan for a
#: boundary before concluding the body is not the multipart stream we asked
#: for. Both bound the parser's buffer independently of the peer's behaviour.
MAX_PART_HEADER_BYTES = 8 * 1024
MAX_BOUNDARY_SCAN_BYTES = 64 * 1024

#: No frame for this long, while the body is still open, reads as `stale` --
#: the camera is nominally 30 fps, so 2 s is ~60 missed frames, well past any
#: jitter and comfortably short of a human wondering why the picture is frozen.
STALE_AFTER_S = 2.0

#: Reconnect backoff bounds. Starts fast (a daemon restart is ~1 s) and caps
#: low enough that a camera plugged back in is picked up within ~10 s.
MIN_BACKOFF_S = 0.5
MAX_BACKOFF_S = 10.0

#: Consecutive failed connect attempts before the state stops saying
#: `connecting` and says `offline`. One blip is a reconnect; two in a row is an
#: absent daemon, and the card should say so rather than spinning forever.
OFFLINE_AFTER_FAILURES = 2

#: Ceiling on joining ONE camera task during `stop()`.
STOP_TIMEOUT_S = 1.0

CONNECT_TIMEOUT_S = 5.0
RESPONSE_HEADER_TIMEOUT_S = 5.0
#: A read that yields nothing for this long is a dead connection. TCP alone
#: will not tell us: a Pi that lost power leaves an ESTABLISHED socket that
#: never errors, which is exactly how a preview freezes without going Offline.
READ_TIMEOUT_S = 8.0
STATUS_POLL_INTERVAL_S = 5.0
STATUS_TIMEOUT_S = 4.0

#: Window for the host-side arrival rate. 64 frames is ~2 s at 30 fps: long
#: enough to be steady, short enough to fall when the stream actually stops.
RATE_WINDOW = 64


class CameraProtocolError(Exception):
    """The peer's body is not the multipart stream we asked for.

    Distinct from a socket error on purpose: a transport failure is retried
    quietly, a protocol failure is retried too but is worth reporting as
    `last_error`, because it means the daemon and this parser disagree.
    """


# --- frame record -------------------------------------------------------------


@dataclass(frozen=True, slots=True, weakref_slot=True)
class CameraFrame:
    """One immutable JPEG plus the facts that came with it.

    Frozen and slotted because it is shared, unsynchronised, between the ingest
    task and every subscriber: a mutable frame handed to two consumers is the
    kind of aliasing bug that only shows up under load.

    `weakref_slot=True` (one pointer per frame, and at most a handful are ever
    alive) exists so the bounded-memory claim can be tested as OBJECT
    LIVENESS -- "the slot retains exactly one frame after fifty publishes" --
    rather than as a size heuristic that would pass against a slot quietly
    holding a list.

    `jpeg` is the daemon's exact bytes -- no re-encode, no re-wrap. Stage B
    (#227) archives these verbatim, so any transformation here would silently
    become the recorded "original".
    """

    generation: int              # HOST ingest generation (see CameraClient)
    seq: int                     # source sequence within `source_generation`
    jpeg: bytes
    host_recv_mono: HostMono
    source_generation: int | None = None
    pi_capture_mono_ns: PiMonoNanos | None = None
    #: When the V4L2 buffer was DEQUEUED on the Pi -- a diagnostic, relayed
    #: untouched. A separate field, never a fallback for the capture stamp.
    pi_dequeue_mono_ns: PiMonoNanos | None = None
    timestamp_valid: bool = False
    timestamp_domain: str | None = None
    width: int | None = None
    height: int | None = None

    @property
    def nbytes(self) -> int:
        return len(self.jpeg)


# --- JPEG inspection ----------------------------------------------------------

_SOI = b"\xff\xd8"
_EOI = b"\xff\xd9"
# Start-of-frame markers carry the real raster size. C0/C1/C2 are the ones a
# webcam emits; the rest are listed so a progressive/arithmetic variant is read
# rather than reported as unknown. C4/C8/CC are NOT frame headers (DHT, JPG,
# DAC) and must not be matched -- that is the classic off-by-one in this walk.
_SOF_MARKERS = frozenset(
    {0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF}
)


def looks_like_jpeg(data: bytes) -> bool:
    """SOI/EOI sanity, the same check the daemon does before publishing.

    Cheap and worth repeating on this side: a truncated part that still
    satisfied `Content-Length` (a daemon bug, a proxy) would otherwise be
    relayed to the browser as a broken image with nothing counting it.
    """
    return len(data) >= 4 and data[:2] == _SOI and data[-2:] == _EOI


def jpeg_dimensions(data: bytes) -> tuple[int, int] | None:
    """(width, height) read out of the JPEG's SOF segment, or None.

    The ACTUAL raster size (§4.4), not a header the daemon asserts and not the
    mode someone requested. Pure marker walk over the header segments -- it
    never touches entropy-coded data, so it costs microseconds and stops at the
    first SOF.
    """
    n = len(data)
    if n < 4 or data[:2] != _SOI:
        return None
    i = 2
    while i + 3 < n:
        if data[i] != 0xFF:
            # Fill bytes are legal between segments; anything else means we
            # have lost marker alignment and should stop rather than guess.
            i += 1
            continue
        marker = data[i + 1]
        if marker == 0xFF:          # fill byte
            i += 1
            continue
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7:
            i += 2                  # standalone markers carry no length
            continue
        if marker == 0xDA or marker == 0xD9:
            return None             # reached scan data with no SOF
        if i + 3 >= n:
            return None
        seg_len = (data[i + 2] << 8) | data[i + 3]
        if seg_len < 2:
            return None
        if marker in _SOF_MARKERS:
            # SOF payload: precision(1) height(2) width(2)
            if i + 9 > n:
                return None
            height = (data[i + 5] << 8) | data[i + 6]
            width = (data[i + 7] << 8) | data[i + 8]
            return (width, height) if width and height else None
        i += 2 + seg_len
    return None


# --- incremental multipart parser ---------------------------------------------


@dataclass
class MultipartPart:
    """One parsed part: its header block and its exact body bytes."""

    headers: dict[str, str]
    body: bytes


class MultipartParser:
    """Incremental `multipart/x-mixed-replace` reader.

    Correct when the input is split at ANY byte position -- that is the whole
    point of it, and `test_camera.py` proves it by replaying a canned stream one
    byte at a time and at every single split index of a two-chunk division.
    The naive implementation ("find the boundary in this chunk") is wrong at
    exactly the offsets a real TCP stream produces, and the failure looks like
    an occasional dropped frame rather than a parser bug.

    Bounds, all enforced regardless of what the peer sends:

      * a part header block over `max_header_bytes` -> `CameraProtocolError`;
      * a missing/unparseable/negative `Content-Length` -> `CameraProtocolError`
        (we never fall back to "scan forward for the next boundary", which is
        what turns one malformed part into an unbounded buffer);
      * a `Content-Length` over `max_frame_bytes` -> `CameraProtocolError`;
      * more than `max_scan_bytes` of junk before a boundary -> error.

    So the retained buffer is bounded by `max(max_frame_bytes,
    max_scan_bytes) + one chunk` at all times.
    """

    _SEEK, _HEADERS, _BODY, _TRAILER = 0, 1, 2, 3

    def __init__(
        self,
        boundary: str,
        *,
        max_frame_bytes: int = MAX_FRAME_BYTES,
        max_header_bytes: int = MAX_PART_HEADER_BYTES,
        max_scan_bytes: int = MAX_BOUNDARY_SCAN_BYTES,
    ) -> None:
        if not boundary:
            raise CameraProtocolError("multipart response declared no boundary")
        self._marker = b"--" + boundary.encode("latin-1")
        self._buf = bytearray()
        self._scanned = 0            # bytes discarded looking for a boundary
        self._state = self._SEEK
        self._headers: dict[str, str] = {}
        self._need = 0
        self.max_frame_bytes = max_frame_bytes
        self.max_header_bytes = max_header_bytes
        self.max_scan_bytes = max_scan_bytes

    @property
    def buffered(self) -> int:
        """Bytes currently retained. Asserted on in the bounded-memory test."""
        return len(self._buf)

    def feed(self, chunk: bytes) -> list[MultipartPart]:
        """Append `chunk` and return every part it completed (possibly none).

        Raises `CameraProtocolError` on a framing violation; the caller drops
        the connection, because a parser that has lost boundary alignment
        cannot recover it in-band.
        """
        if chunk:
            self._buf += chunk
        out: list[MultipartPart] = []
        while True:
            part = self._step()
            if part is None:
                return out
            if part is not _PROGRESS:
                out.append(part)

    def _step(self):
        if self._state == self._SEEK:
            idx = self._buf.find(self._marker)
            if idx < 0:
                # Retain only what could still be a partial marker. Everything
                # earlier is preamble and can never become one -- so the buffer
                # stays bounded even against an endless non-multipart body.
                keep = len(self._marker) - 1
                if len(self._buf) > keep:
                    dropped = len(self._buf) - keep
                    del self._buf[:dropped]
                    self._scanned += dropped
                # The CUMULATIVE count, not the retained one: trimming already
                # bounds memory, so a buffer-size check here could never fire
                # and a peer streaming junk forever would be scanned forever.
                if self._scanned > self.max_scan_bytes:
                    raise CameraProtocolError(
                        f"no multipart boundary in the first {self.max_scan_bytes} bytes")
                return None
            del self._buf[: idx + len(self._marker)]
            self._scanned = 0
            self._state = self._HEADERS
            return _PROGRESS

        if self._state == self._HEADERS:
            # The boundary line's own CRLF (and a `--` terminator) sit at the
            # head of the buffer; the header block ends at the first blank line.
            end = self._buf.find(b"\r\n\r\n")
            if end < 0:
                if len(self._buf) > self.max_header_bytes:
                    raise CameraProtocolError(
                        f"part headers exceeded {self.max_header_bytes} bytes")
                return None
            raw = bytes(self._buf[:end])
            del self._buf[: end + 4]
            self._headers = _parse_headers(raw)
            self._need = _content_length(self._headers, self.max_frame_bytes)
            self._state = self._BODY
            return _PROGRESS

        if self._state == self._BODY:
            if len(self._buf) < self._need:
                return None
            body = bytes(self._buf[: self._need])
            del self._buf[: self._need]
            self._state = self._TRAILER
            return MultipartPart(headers=self._headers, body=body)

        # _TRAILER: consume the CRLF that follows the body if it is there.
        # Tolerated-absent on purpose (§8's "be tolerant in exactly these
        # ways"): the next `--boundary` search re-syncs either way, so a daemon
        # that omits it degrades to a slower scan, not a broken stream.
        if len(self._buf) < 2:
            return None
        if self._buf[:2] == b"\r\n":
            del self._buf[:2]
        self._state = self._SEEK
        return _PROGRESS


#: Sentinel meaning "the state machine advanced but produced no part". Distinct
#: from None ("need more bytes") so `feed`'s loop terminates on the right one.
_PROGRESS = MultipartPart(headers={}, body=b"")


def _parse_headers(raw: bytes) -> dict[str, str]:
    """Lower-cased header dict from a part's header block. Never raises."""
    headers: dict[str, str] = {}
    for line in raw.split(b"\r\n"):
        line = line.strip()
        if not line or b":" not in line:
            continue
        name, _, value = line.partition(b":")
        headers[name.strip().decode("latin-1").lower()] = value.strip().decode("latin-1")
    return headers


def _content_length(headers: dict[str, str], limit: int) -> int:
    """The part's declared body length, or `CameraProtocolError`.

    A missing or unparseable length is fatal rather than "scan for the next
    boundary": scanning is what lets one bad part consume unbounded memory, and
    the daemon's own contract (§5.2) makes the header mandatory.
    """
    raw = headers.get("content-length")
    if raw is None:
        raise CameraProtocolError("part had no Content-Length")
    try:
        n = int(raw)
    except ValueError:
        raise CameraProtocolError(f"unparseable Content-Length {raw!r}") from None
    if n < 0:
        raise CameraProtocolError(f"negative Content-Length {n}")
    if n > limit:
        raise CameraProtocolError(f"part claims {n} bytes, over the {limit}-byte limit")
    return n


def parse_boundary(content_type: str) -> str:
    """The `boundary=` parameter of a multipart Content-Type, or "" if absent."""
    for token in content_type.split(";")[1:]:
        name, _, value = token.strip().partition("=")
        if name.strip().lower() == "boundary":
            return value.strip().strip('"')
    return ""


# --- the latest-frame slot ----------------------------------------------------


class LatestFrameSlot:
    """One immutable frame, any number of latest-only subscribers (§4.1/§4.3).

    `publish` overwrites; it never appends. A subscriber blocked on its socket
    holds a reference to whatever frame it last took and nothing else, so N
    stalled browsers cost N JPEGs, not N queues -- and the slot itself always
    retains exactly one, which `test_camera.py` asserts with weakrefs rather
    than with a size heuristic.

    Skips are COUNTED, not silent. A bounded queue that drops quietly reads as
    healthy from every dashboard (#217's lesson, and `_count_backpressure_drops`
    on `/ws` is the same fix); `skipped` is what the `camera` message reports as
    `ingest_drops`.

    `skipped` is charged when a subscriber RESUMES, not while it is blocked --
    a stalled viewer is inside its `send`, not inside `wait_next`, so nothing
    has selected a frame to skip yet. It is therefore a lagging total, honest
    but late: read it as "frames viewers have jumped over", never as a live
    gauge of how far behind someone is right now. The present-tense signal for
    that is the transport's own write-buffer depth.
    """

    def __init__(self) -> None:
        self._frame: CameraFrame | None = None
        self._tick = 0
        self._event = asyncio.Event()
        self.published = 0
        self.skipped = 0
        self.subscribers = 0

    def publish(self, frame: CameraFrame) -> None:
        self._frame = frame
        self._tick += 1
        self.published += 1
        event, self._event = self._event, asyncio.Event()
        event.set()

    def clear(self) -> None:
        """Drop the retained frame (a reconnect/teardown), keeping counters.

        The tick still advances so a waiter does not read the cleared slot as
        "nothing has happened".
        """
        self._frame = None
        self._tick += 1
        event, self._event = self._event, asyncio.Event()
        event.set()

    def latest(self) -> tuple[int, CameraFrame | None]:
        """(tick, frame) without waiting. Does NOT pop -- unlike `MeshPrep`,
        this producer is infinite and every consumer is latest-only, so a
        popping slot would make two viewers steal frames from each other."""
        return self._tick, self._frame

    async def wait_next(self, since_tick: int) -> tuple[int, CameraFrame]:
        """Block until a frame newer than `since_tick` exists, then return it.

        Counts everything published between `since_tick` and what is returned
        as skipped by this subscriber, which is precisely the "slow host proxy
        client ... next selected frame is newest" row of the §8 failure table.
        """
        while True:
            event = self._event          # grab BEFORE reading state
            tick, frame = self._tick, self._frame
            if frame is not None and tick > since_tick:
                missed = tick - since_tick - 1
                if since_tick > 0 and missed > 0:
                    self.skipped += missed
                return tick, frame
            await event.wait()


# --- the client ---------------------------------------------------------------


@dataclass
class CameraIngestConfig:
    """Where and how to reach the daemon. Every bound is a field so a test can
    shrink it rather than sleep for the production value."""

    host: str = ""
    port: int = DEFAULT_CAMERA_PORT
    stream_path: str = "/stream.mjpg"
    status_path: str = "/status.json"
    connect_timeout_s: float = CONNECT_TIMEOUT_S
    header_timeout_s: float = RESPONSE_HEADER_TIMEOUT_S
    read_timeout_s: float = READ_TIMEOUT_S
    status_interval_s: float = STATUS_POLL_INTERVAL_S
    status_timeout_s: float = STATUS_TIMEOUT_S
    stale_after_s: float = STALE_AFTER_S
    min_backoff_s: float = MIN_BACKOFF_S
    max_backoff_s: float = MAX_BACKOFF_S
    offline_after_failures: int = OFFLINE_AFTER_FAILURES
    max_frame_bytes: int = MAX_FRAME_BYTES
    #: Per-task ceiling on `stop()`. Short: both workers are plain asyncio and
    #: cancel promptly, so anything approaching this is a bug worth logging.
    stop_timeout_s: float = STOP_TIMEOUT_S


@dataclass
class _Rate:
    """Windowed rate over a bounded deque of instants. Returns None until it
    has two samples -- an unknown rate is null, never a fabricated 0.0 (§5.4:
    "Null means unknown; zero means measured zero")."""

    times: deque = field(default_factory=lambda: deque(maxlen=RATE_WINDOW))

    def add(self, t: float) -> None:
        self.times.append(float(t))

    def reset(self) -> None:
        self.times.clear()

    def hz(self, now: float | None = None, *, idle_after_s: float = 3.0) -> float | None:
        if len(self.times) < 2:
            return None
        span = self.times[-1] - self.times[0]
        if span <= 0:
            return None
        if now is not None and (now - self.times[-1]) > idle_after_s:
            # The window is a history, not a present-tense reading. Once
            # arrivals stop, reporting the old rate is the `browser_fps` trap
            # from #217 -- a healthy-looking counter beside a frozen picture.
            return 0.0
        return (len(self.times) - 1) / span


class CameraClient:
    """Lifespan-owned ingest: one upstream connection, one latest-frame slot.

    Started and stopped by FastAPI lifespan, never opportunistically by a route
    or a session message (spec §5.4) -- an ingest that a viewer can start is an
    ingest that N viewers can start N times.

    Teardown (`stop`) closes the socket, cancels and AWAITS its worker, and
    leaves no liveness flag set. The cancel carries an
    `asyncio.current_task()` guard because `stop` is reachable from inside the
    worker's own error path in principle, and a task cancelling itself lands
    the `CancelledError` mid-teardown with nowhere to go (#219).
    """

    def __init__(
        self,
        config: CameraIngestConfig | None = None,
        *,
        resolve=None,
        connect=None,
        status_connect=None,
        clock=None,
        sleep=None,
    ) -> None:
        self.config = config or CameraIngestConfig()
        #: async () -> str, the bridge address. Injected so the server can share
        #: `commstatus.resolve_bridge_target`'s TTL cache with the ICMP loop and
        #: the ssh health poll rather than adding a third resolution policy.
        self._resolve = resolve
        #: async (host, port) -> (reader, writer). Injected only by tests.
        self._connect = connect or _open_connection
        #: The same dialler for the status poll in production, and a SEPARATE
        #: injection point in tests -- the two loops open independent
        #: short-lived/long-lived connections, and a test that scripts the
        #: MJPEG body must not have the status poll eat its first script.
        self._status_connect = status_connect or self._connect
        self._now = clock or (lambda: float(now_mono()))
        #: `asyncio.sleep`, injected only by tests -- so the backoff SCHEDULE
        #: can be asserted as the intervals actually requested rather than
        #: inferred from wall-clock attempt counts, which is how a monotone
        #: backoff hid here in the first place.
        self._sleep = sleep or asyncio.sleep

        self.slot = LatestFrameSlot()
        self.generation = 0             # HOST ingest generation
        self.source_generation: int | None = None
        self.last_seq: int | None = None
        self.frames = 0
        self.bytes = 0
        self.reconnects = 0
        self.malformed = 0
        self.source_drops = 0
        self.last_error: str | None = None
        self._logged_error: str | None = None
        self.bridge_addr: str | None = None
        self.source_status: dict | None = None
        self.source_status_error: str | None = None

        self._running = False
        self._connected = False
        self._consecutive_failures = 0
        self._last_frame_at: float | None = None
        #: Frames delivered by the CURRENT connection. Distinct from `frames`
        #: (process lifetime) because both `state()` and the backoff reset ask
        #: "has THIS connection produced anything yet?", and the lifetime
        #: counter answers a different question.
        self._frames_this_connection = 0
        self._host_rate = _Rate()
        self._capture_rate = _Rate()
        self._last_capture_ns: PiMonoNanos | None = None
        self._capture_base_ns: PiMonoNanos | None = None
        self._task: asyncio.Task | None = None
        self._status_task: asyncio.Task | None = None
        self._writer = None

    # -- lifecycle -------------------------------------------------------

    async def start(self) -> None:
        """Idempotent: a second `start` on a running client is a no-op, so a
        repeated lifespan (the app factory is exercised many times in tests)
        can never leave two ingest tasks racing for one slot."""
        # BOTH tasks, not just the ingest one. Guarding on `_task` alone means
        # an ingest task that has finished (a cancel from elsewhere, a bug that
        # let `_run` return) lets a second `start()` through, which then
        # overwrites `_status_task` with a fresh poller and leaks the live one
        # -- two pollers dialling the daemon forever, and `stop()` able to join
        # only the newer of them.
        if any(t is not None and not t.done()
               for t in (self._task, self._status_task)):
            return
        self._running = True
        self._task = asyncio.create_task(self._run(), name="camera-ingest")
        self._status_task = asyncio.create_task(
            self._status_loop(), name="camera-status-poll")

    async def stop(self) -> None:
        """Close the socket, cancel and JOIN both tasks, clear the slot.

        Bounded: the join is awaited, not fire-and-forget, so `stop` returning
        means there is genuinely no task left holding the socket -- the
        difference between "we asked it to stop" and "it stopped" (#219).
        """
        self._running = False
        self._connected = False
        current = asyncio.current_task()
        cancelled_from_outside = False
        try:
            for attr in ("_task", "_status_task"):
                task = getattr(self, attr)
                setattr(self, attr, None)
                # `is current` is the self-cancel guard: a task that cancels
                # itself lands the CancelledError inside its own cleanup, with
                # nowhere to go, and half-tears-down whatever it was holding
                # (#219).
                if task is None or task is current:
                    continue
                task.cancel()
                try:
                    # BOUNDED. A join is only as bounded as the thing it joins:
                    # a worker wedged somewhere uncancellable would turn
                    # "teardown" into "hang", which is exactly the failure mode
                    # #226 round 4 was reported for. Past the timeout the task
                    # is abandoned rather than waited on, and that is said out
                    # loud below instead of being swallowed.
                    await asyncio.wait_for(task, timeout=self.config.stop_timeout_s)
                except asyncio.TimeoutError:
                    log.warning("camera %s did not stop within %.1fs; abandoning it",
                                attr.lstrip("_"), self.config.stop_timeout_s)
                except asyncio.CancelledError:
                    # Two different events raise here and they must not be
                    # conflated: the task WE cancelled finishing (expected,
                    # swallowed) versus THIS task being cancelled while we
                    # waited -- which happens on a real lifespan shutdown.
                    # Swallowing the second one silently eats the shutdown
                    # signal, so it is re-raised after cleanup completes.
                    if _being_cancelled(current):
                        cancelled_from_outside = True
                except Exception:               # noqa: BLE001 - a dead worker
                    pass
        finally:
            # Runs even when the re-raise above is pending, so the socket and
            # the slot are released either way.
            try:
                await self._aclose_writer()
            except Exception:                   # noqa: BLE001
                pass
            self.slot.clear()
        if cancelled_from_outside:
            raise asyncio.CancelledError

    async def _aclose_writer(self) -> None:
        """Close the upstream socket and WAIT for it to finish closing.

        `close()` alone only requests it; without the `wait_closed()` the
        transport can still be flushing when `stop()` returns, so "the lifespan
        returned" would not actually mean the socket is gone -- the same
        asked-versus-happened distinction the task join exists for.
        """
        writer, self._writer = self._writer, None
        if writer is None:
            return
        try:
            writer.close()
        except Exception:                       # noqa: BLE001 - already dead
            return
        wait_closed = getattr(writer, "wait_closed", None)
        if wait_closed is None:
            return                              # a test double, or an old loop
        try:
            await asyncio.wait_for(wait_closed(), timeout=STOP_TIMEOUT_S)
        except Exception:                       # noqa: BLE001 - it is closing,
            pass                                # or it never will; either way we go

    # -- the ingest loop -------------------------------------------------

    async def _run(self) -> None:
        backoff = self.config.min_backoff_s
        while self._running:
            productive = False
            # Zeroed HERE, at the top of the attempt, not inside
            # `_connect_and_stream` -- a dial that fails never reaches that
            # function, so the count from the PREVIOUS connection would survive
            # and every subsequent failure would re-earn the reset. One owner,
            # and it means exactly "frames delivered by the attempt just made".
            self._frames_this_connection = 0
            try:
                await self._connect_and_stream()
                backoff = self.config.min_backoff_s
            except asyncio.CancelledError:
                raise
            except Exception as exc:            # noqa: BLE001 - every failure retries
                self.last_error = f"{type(exc).__name__}: {exc}"
                self._consecutive_failures += 1
                # NOT a bare `except: continue`. A reconnect loop that swallows
                # its reason hides a 100 % failure rate behind a stream that
                # merely looks quiet (#219); this is the log line that makes
                # "the camera never came back" diagnosable.
                #
                # Logged at INFO only when the reason CHANGES. A bridge with no
                # camera daemon fails identically every backoff period forever,
                # and a line every few seconds saying the same thing is how a
                # log stops being read at all. The repeat still goes to DEBUG,
                # and `last_error` + `reconnects` carry the full picture.
                level = log.info if self.last_error != self._logged_error else log.debug
                self._logged_error = self.last_error
                level("camera ingest attempt failed (%s); retry in %.1fs",
                      self.last_error, backoff)
            finally:
                productive = self._frames_this_connection > 0
                self._connected = False
                await self._aclose_writer()
            if not self._running:
                break
            # A connection that actually DELIVERED frames earns a fresh
            # backoff. Without this the reset lived only on the normal-return
            # path -- which `_connect_and_stream` reaches only at shutdown --
            # so every real drop doubled and the delay was monotone for the
            # life of the process. The visible symptom is nasty and delayed: a
            # camera absent at boot walks the backoff to its 10 s cap, and from
            # then on a daemon restart costs 10 s of Offline forever, on a link
            # that is otherwise perfectly healthy.
            #
            # Keyed on frames delivered, NOT on a successful response head: a
            # daemon that accepts the connection and then wedges would
            # otherwise reset the backoff every attempt and turn this into a
            # 0.5 s busy-loop against a broken peer.
            if productive:
                backoff = self.config.min_backoff_s
            await self._sleep(backoff)
            backoff = min(backoff * 2.0, self.config.max_backoff_s)

    async def _connect_and_stream(self) -> None:
        cfg = self.config
        host = await self._target()
        self.bridge_addr = host
        reader, writer = await asyncio.wait_for(
            self._connect(host, cfg.port), timeout=cfg.connect_timeout_s)
        self._writer = writer
        writer.write(_http_get(host, cfg.port, cfg.stream_path,
                               accept="multipart/x-mixed-replace"))
        head = await asyncio.wait_for(
            reader.readuntil(b"\r\n\r\n"), timeout=cfg.header_timeout_s)
        status, headers = _parse_response_head(head)
        if status != 200:
            raise CameraProtocolError(f"{cfg.stream_path} returned HTTP {status}")
        boundary = parse_boundary(headers.get("content-type", ""))
        parser = MultipartParser(boundary, max_frame_bytes=cfg.max_frame_bytes)

        # A reconnect is a NEW host generation even when the Pi's own
        # generation is unchanged: the browser's cache-busted <img> URL keys off
        # this, and the frames either side of a gap are not one continuous
        # sequence from the viewer's point of view.
        self.generation += 1
        self.reconnects += 1 if self.frames else 0
        self._connected = True
        self._consecutive_failures = 0
        self.last_error = None
        self._logged_error = None
        self._host_rate.reset()
        self._capture_rate.reset()
        self._last_capture_ns = None
        self._capture_base_ns = None
        self.last_seq = None

        while self._running:
            chunk = await asyncio.wait_for(reader.read(65536),
                                           timeout=cfg.read_timeout_s)
            if not chunk:
                raise CameraProtocolError("upstream closed the MJPEG body")
            for part in parser.feed(chunk):
                self._publish(part)

    async def _target(self) -> str:
        if self._resolve is not None:
            addr = await self._resolve()
            if addr:
                return addr
        if not self.config.host:
            raise CameraProtocolError("no camera host configured and no resolver")
        return self.config.host

    def _publish(self, part: MultipartPart) -> None:
        """Turn one parsed part into a published frame, or count it malformed.

        A malformed BODY is not a framing failure -- the part was correctly
        delimited, so the connection stays in sync and only the frame is
        dropped. That distinction is why `looks_like_jpeg` counts rather than
        raises.
        """
        body = part.body
        if not looks_like_jpeg(body):
            self.malformed += 1
            self.last_error = f"part {len(body)}B failed the JPEG SOI/EOI check"
            return

        headers = part.headers
        seq = _int_or_none(headers.get("x-roomscan-camera-seq"))
        src_gen = _int_or_none(headers.get("x-roomscan-camera-generation"))
        domain = headers.get("x-roomscan-timestamp-domain")
        cap_ns = _int_or_none(headers.get("x-roomscan-capture-mono-ns"))
        # A capture timestamp counts only when the daemon named the domain it
        # is in. Anything else -- absent header, empty value, an unfamiliar
        # domain -- is `timestamp_valid=False` and the number is not used
        # (§4.5, §8's "Timestamp flag absent" row). Arrival time is recorded
        # separately and is never renamed capture time.
        valid = domain == "pi_monotonic" and cap_ns is not None and cap_ns >= 0
        # The daemon's own flag, whatever it decided. `roomscan-cam` sends
        # `X-Roomscan-Capture-Timestamp-Valid: true|false`; the shorter
        # `X-Roomscan-Timestamp-Valid` is kept as an alias so either spelling is
        # read. It can only ever REVOKE validity, never grant it: a `true`
        # alongside an absent stamp or an unnamed domain still leaves no usable
        # capture time, and taking the daemon's word over evidence we can check
        # ourselves is how an invariant-preserving check verifies nothing.
        explicit = _bool_or_none(headers.get("x-roomscan-capture-timestamp-valid"))
        if explicit is None:
            explicit = _bool_or_none(headers.get("x-roomscan-timestamp-valid"))
        if explicit is False:
            valid = False
        # Relayed, never consumed -- see `CameraFrame.pi_dequeue_mono_ns`.
        dequeue_ns = _int_or_none(headers.get("x-roomscan-capture-dequeue-mono-ns"))

        now = self._now()
        self._track_sequence(src_gen, seq)
        if valid:
            # Pi-to-Pi ONLY, and expressed as an offset from the first capture
            # stamp of this connection. The window holds Pi-domain seconds;
            # `now` (host monotonic) never enters it, so `capture_fps` cannot
            # become a disguised cross-clock difference.
            if self._capture_base_ns is None:
                self._capture_base_ns = PiMonoNanos(cap_ns)
            self._capture_rate.add(
                pi_mono_delta_s(self._capture_base_ns, PiMonoNanos(cap_ns)))
            self._last_capture_ns = PiMonoNanos(cap_ns)

        dims = jpeg_dimensions(body)
        frame = CameraFrame(
            generation=self.generation,
            seq=seq if seq is not None else self.frames,
            jpeg=body,
            host_recv_mono=HostMono(now),
            source_generation=src_gen,
            pi_capture_mono_ns=PiMonoNanos(cap_ns) if valid else None,
            pi_dequeue_mono_ns=(PiMonoNanos(dequeue_ns)
                                if dequeue_ns is not None and dequeue_ns >= 0
                                else None),
            timestamp_valid=valid,
            timestamp_domain=domain,
            width=dims[0] if dims else None,
            height=dims[1] if dims else None,
        )
        self.frames += 1
        self._frames_this_connection += 1
        self.bytes += len(body)
        self._last_frame_at = now
        self._host_rate.add(now)
        self.slot.publish(frame)

    def _track_sequence(self, src_gen: int | None, seq: int | None) -> None:
        """Count frames the source produced that we never saw, and notice a
        reset. A gap inside one source generation is loss; a new source
        generation (or a sequence going backwards) is a camera reopen, which
        starts a fresh HOST generation rather than reading as ~2^31 lost
        frames."""
        if seq is None:
            return
        first = self.last_seq is None
        # The FIRST part of a connection only adopts the source generation; it
        # never bumps. The connect already bumped, and bumping again here would
        # make every browser cache-bust its stream URL one frame after opening
        # it -- a reconnect loop that looked like correct generation tracking.
        if (src_gen is not None and self.source_generation is not None
                and src_gen != self.source_generation):
            self.source_generation = src_gen
            self.last_seq = seq
            self.generation += 1
            self.slot.clear()
            return
        if src_gen is not None:
            self.source_generation = src_gen
        if not first:
            if seq <= self.last_seq:
                # Sequence reset without a generation bump: still a reopen, and
                # it must be treated as one even though the daemon did not say
                # so -- the browser's cache-bust keys off the host generation.
                self.generation += 1
                self.slot.clear()
            elif seq > self.last_seq + 1:
                self.source_drops += seq - self.last_seq - 1
        self.last_seq = seq

    # -- status poll -----------------------------------------------------

    async def _status_loop(self) -> None:
        """Poll `/status.json` on its OWN slow cadence.

        A separate task, not a branch of the ingest loop and emphatically not
        the scanner broadcaster's tick (§5.4): a wedged HTTP request must not
        be able to stall frame parsing, and frame parsing must not be able to
        delay the status the card shows when frames have stopped -- which is
        exactly when the status matters most.
        """
        cfg = self.config
        while self._running:
            try:
                host = self.bridge_addr or await self._target()
                self.source_status = await asyncio.wait_for(
                    self._fetch_status(host), timeout=cfg.status_timeout_s)
                self.source_status_error = None
            except asyncio.CancelledError:
                raise
            except Exception as exc:                    # noqa: BLE001
                self.source_status = None
                self.source_status_error = f"{type(exc).__name__}: {exc}"
            await asyncio.sleep(cfg.status_interval_s)

    async def _fetch_status(self, host: str) -> dict:
        cfg = self.config
        reader, writer = await asyncio.wait_for(
            self._status_connect(host, cfg.port), timeout=cfg.connect_timeout_s)
        try:
            writer.write(_http_get(host, cfg.port, cfg.status_path,
                                   accept="application/json", close=True))
            head = await reader.readuntil(b"\r\n\r\n")
            status, headers = _parse_response_head(head)
            if status != 200:
                raise CameraProtocolError(f"{cfg.status_path} returned HTTP {status}")
            length = headers.get("content-length")
            body = (await reader.readexactly(int(length))) if length else await reader.read(
                MAX_PART_HEADER_BYTES * 8)
            return json.loads(body.decode("utf-8"))
        finally:
            try:
                writer.close()
            except Exception:                           # noqa: BLE001
                pass

    # -- observation -----------------------------------------------------

    @property
    def running(self) -> bool:
        return self._running

    def age_s(self, now: float | None = None) -> float | None:
        """Seconds since the last frame ARRIVED here, or None if none has.

        A host-to-host difference, and named so. It is not latency: the camera's
        exposure instant lives on the Pi's clock, and Stage A maps no clocks.
        """
        if self._last_frame_at is None:
            return None
        return max(0.0, (self._now() if now is None else now) - self._last_frame_at)

    def state(self, now: float | None = None) -> str:
        """`offline` | `connecting` | `live` | `stale` -- see the card (§5.5)."""
        if not self._running:
            return "offline"
        source_state = (self.source_status or {}).get("state")
        if source_state == "device_absent":
            return "offline"
        age = self.age_s(now)
        if age is not None and age <= self.config.stale_after_s:
            return "live"
        if self._connected:
            # `stale` is a claim about a stream that WAS running and stopped.
            # Before this connection's first part there is no such stream: the
            # daemon accepted us and is opening the camera, which is exactly
            # what Connecting means. `age_s` cannot tell the two apart on its
            # own -- it survives across reconnects -- so the per-connection
            # counter is what separates "not started" from "stopped".
            return "stale" if self._frames_this_connection else "connecting"
        if self._consecutive_failures >= self.config.offline_after_failures:
            return "offline"
        return "connecting"

    def status(self, now: float | None = None) -> dict:
        """Structured host-ingest + Pi-reported status.

        The pure reporting function behind `/api/camera/status`, the `camera`
        WebSocket message and the `camera_status` MCP tool -- one
        implementation, three front ends. Every number is something measured:
        `None` where nothing has been measured yet, `0` where zero was.
        """
        t = self._now() if now is None else now
        _, frame = self.slot.latest()
        age = self.age_s(t)
        return {
            "state": self.state(t),
            "bridge_addr": self.bridge_addr,
            "port": self.config.port,
            "generation": self.generation,
            "source_generation": self.source_generation,
            "seq": frame.seq if frame is not None else None,
            "width": frame.width if frame is not None else None,
            "height": frame.height if frame is not None else None,
            "fourcc": "MJPG" if frame is not None else None,
            # Unknown, not stale-but-plausible: once frames stop arriving here
            # we have no evidence about what the Pi is capturing, and reporting
            # the last window would be `browser_fps` all over again (#217) -- a
            # healthy number beside a frozen picture.
            "capture_fps": (self._capture_rate.hz()
                            if age is not None and age <= self.config.stale_after_s
                            else None),
            "host_fps": self._host_rate.hz(t),
            "capture_timestamp_valid": frame.timestamp_valid if frame is not None else None,
            "capture_timestamp_domain": frame.timestamp_domain if frame is not None else None,
            "age_ms": None if age is None else round(age * 1000.0, 1),
            "frames": self.frames,
            "bytes": self.bytes,
            "frame_bytes": frame.nbytes if frame is not None else None,
            "reconnects": self.reconnects,
            "malformed": self.malformed,
            "source_drops": self.source_drops,
            "ingest_drops": self.slot.skipped,
            "clients": self.slot.subscribers,
            "last_error": self.last_error,
            # Verbatim pass-through, read only via `.get()` elsewhere: a field
            # the daemon adds or renames costs this module nothing.
            "source": self.source_status,
            "source_error": self.source_status_error,
        }


# --- the `camera` WebSocket message (§5.4) ------------------------------------

#: Keys whose change makes the message worth sending immediately. Everything
#: else (counters, fps, age) rides the heartbeat: a message every time `age_ms`
#: ticks would be a 30 Hz JSON stream dressed up as a state echo.
CAMERA_EDGE_KEYS = (
    "state", "generation", "source_generation", "width", "height",
    "capture_timestamp_valid", "bridge_addr",
)


def camera_message(status: dict) -> dict:
    """The `/ws` `camera` message: state, never pixels (§5.4).

    **No `recording {state, writer_drops, last_error}` sub-object, and no
    Recording/Partial state word -- deferred to Stage B (#227) on purpose.**
    §5.4 lists both, but they describe a capture-bundle writer that does not
    exist yet: Stage A has nothing to write and therefore nothing truthful to
    say about `writer_drops` or `camera_partial`. Emitting the keys as
    permanent nulls would let a browser render a recording badge from a field
    no code populates -- a `state` a viewer could read as "healthy dual
    capture" while only the scanner is being written, which is precisely the
    failure §6.3 exists to prevent. The keys arrive with the writer.
    """
    msg = {"type": "camera"}
    msg.update(status)
    return msg


def camera_message_changed(previous: dict | None, current: dict) -> bool:
    """Whether `current` is worth an immediate send.

    Edge-triggered on the keys a viewer reacts to; the periodic heartbeat
    carries everything else. This is what keeps the cadence human-rate (<=4 Hz)
    without making the card wait a whole heartbeat to notice the camera died.
    """
    if previous is None:
        return True
    return any(previous.get(k) != current.get(k) for k in CAMERA_EDGE_KEYS)


# --- agent surface: pure functions (§5.6) -------------------------------------

#: Roots a snapshot may be written under. Anything else is refused rather than
#: clamped, so a caller learns its path was wrong instead of finding the file
#: somewhere it did not ask for.
SNAPSHOT_ROOTS = ("results", "captures")


def resolve_snapshot_path(out: str, *, repo: Path, now: float | None = None) -> Path:
    """Where a snapshot may be written. Pure; raises `ValueError` if refused.

    An empty `out` lands in `results/camera/<unix-ms>.jpg`. A relative path is
    taken against the repo root. Every result is `resolve()`d and checked to be
    STRICTLY INSIDE one of `SNAPSHOT_ROOTS` -- symlink and `..` traversal
    included, because resolving first is the only check that sees either.

    Strictly inside, not "inside or equal": `out="results"` resolving to the
    root directory itself passed the old check and then reached `write_bytes`
    on a directory, which raises `IsADirectoryError` out of a tool whose whole
    contract is to report rather than raise. A directory is not a snapshot
    path, so it is refused here where the refusal can carry a reason.
    """
    repo = Path(repo).resolve()
    if not out:
        stamp = int((time.time() if now is None else now) * 1000)
        candidate = repo / "results" / "camera" / f"camera-{stamp}.jpg"
    else:
        candidate = Path(out).expanduser()
        if not candidate.is_absolute():
            candidate = repo / candidate
    candidate = Path(candidate).resolve()
    allowed = [(repo / r).resolve() for r in SNAPSHOT_ROOTS]
    if candidate in allowed:
        raise ValueError(
            f"snapshot path {candidate} is an allowed ROOT directory, not a file "
            f"inside one -- name the .jpg to write")
    if not any(root in candidate.parents for root in allowed):
        raise ValueError(
            f"snapshot path {candidate} is outside the allowed roots "
            f"{[str(a) for a in allowed]}")
    return candidate


def fetch_camera_status(base_url: str, *, timeout: float = 5.0) -> dict:
    """GET `<base_url>/api/camera/status`. Never raises; reports what happened.

    Goes through `roomscan-web`, never straight to the Pi: the MCP server must
    not open a second upstream camera connection any more than it may bind the
    device stream (the client-never-competitor invariant).
    """
    url = base_url.rstrip("/") + "/api/camera/status"
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            payload = json.loads(response.read().decode("utf-8"))
    except urllib.error.URLError as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}", "url": url,
                "hint": "is roomscan-web running on this host?"}
    except Exception as exc:                            # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}", "url": url}
    payload["ok"] = True
    payload["url"] = url
    return payload


def save_camera_snapshot(base_url: str, out: str = "", *, repo: Path,
                         timeout: float = 5.0) -> dict:
    """GET `<base_url>/api/camera/frame.jpg` and write it under an allowed root.

    Reports the path ACTUALLY written and the bytes actually on disk, plus the
    frame's generation/sequence/age from the response headers -- not the
    arguments it was handed.
    """
    url = base_url.rstrip("/") + "/api/camera/frame.jpg"
    try:
        dest = resolve_snapshot_path(out, repo=repo)
    except ValueError as exc:
        return {"ok": False, "error": str(exc), "requested": out}
    try:
        with urllib.request.urlopen(url, timeout=timeout) as response:
            data = response.read()
            headers = {k.lower(): v for k, v in response.headers.items()}
    except urllib.error.HTTPError as exc:
        return {"ok": False, "error": f"HTTP {exc.code}", "url": url,
                "hint": "no camera frame available; check camera_status()"}
    except Exception as exc:                            # noqa: BLE001
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}", "url": url}
    if not looks_like_jpeg(data):
        return {"ok": False, "error": f"response was not a JPEG ({len(data)} bytes)",
                "url": url}
    # The WRITE is inside the try too, not just the fetch. A full disk, a
    # read-only root, a permission denial and a destination that is a directory
    # are all ordinary answers to "save a snapshot", and a tool that raises
    # `OSError` at its caller instead of returning `{"ok": false, ...}` has
    # stopped reporting what actually happened for exactly the cases where that
    # matters most.
    try:
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_bytes(data)
        written = dest.stat().st_size
    except OSError as exc:
        return {"ok": False, "error": f"{type(exc).__name__}: {exc}",
                "path": str(dest), "url": url}
    dims = jpeg_dimensions(data)
    return {
        "ok": True,
        "path": str(dest),
        "bytes": written,
        "width": dims[0] if dims else None,
        "height": dims[1] if dims else None,
        "generation": _int_or_none(headers.get("x-roomscan-camera-generation")),
        "seq": _int_or_none(headers.get("x-roomscan-camera-seq")),
        "age_ms": _float_or_none(headers.get("x-roomscan-camera-age-ms")),
        "url": url,
    }


# --- small helpers ------------------------------------------------------------


async def _open_connection(host: str, port: int):
    # `limit` bounds what `readuntil` will buffer looking for the header
    # terminator; overrunning it raises rather than growing without bound.
    return await asyncio.open_connection(host, port, limit=MAX_PART_HEADER_BYTES * 8)


def _http_get(host: str, port: int, path: str, *, accept: str,
              close: bool = False) -> bytes:
    lines = [
        f"GET {path} HTTP/1.1",
        f"Host: {host}:{port}",
        f"Accept: {accept}",
        "User-Agent: roomscan-host/1 (camera ingest)",
        "Connection: close" if close else "Connection: keep-alive",
        "", "",
    ]
    return "\r\n".join(lines).encode("latin-1")


def _parse_response_head(head: bytes) -> tuple[int, dict[str, str]]:
    text = head.split(b"\r\n\r\n", 1)[0]
    lines = text.split(b"\r\n")
    if not lines or not lines[0].startswith(b"HTTP/"):
        raise CameraProtocolError("peer did not answer with an HTTP status line")
    parts = lines[0].split(None, 2)
    try:
        status = int(parts[1])
    except (IndexError, ValueError):
        raise CameraProtocolError(f"unparseable status line {lines[0]!r}") from None
    return status, _parse_headers(b"\r\n".join(lines[1:]))


def _being_cancelled(task) -> bool:
    """Whether `task` has a cancellation request pending against it.

    `Task.cancelling()` (3.11+, and this package requires >=3.11) counts
    outstanding `cancel()` calls on the task itself. It is the only signal that
    separates "the coroutine I awaited was cancelled" from "I am being
    cancelled while awaiting it" -- inspecting the awaited task cannot, because
    in both cases it ends up cancelled.
    """
    if task is None:
        return False
    cancelling = getattr(task, "cancelling", None)
    if cancelling is None:                      # pragma: no cover - <3.11
        return False
    try:
        return cancelling() > 0
    except Exception:                           # noqa: BLE001
        return False


def _int_or_none(raw: str | None) -> int | None:
    if raw is None:
        return None
    try:
        return int(raw.strip())
    except (ValueError, AttributeError):
        return None


#: Every spelling of a boolean this wire has to survive. The daemon sends
#: `true`/`false`, this host emits the same, and a `1`/`0` from anything else
#: still reads correctly. Anything unrecognised is UNKNOWN (None), which is not
#: the same as False -- an unparseable flag must not silently revoke a capture
#: timestamp that is otherwise well-formed.
_TRUE_WORDS = frozenset({"1", "true", "yes", "on"})
_FALSE_WORDS = frozenset({"0", "false", "no", "off"})


def _bool_or_none(raw: str | None) -> bool | None:
    if raw is None:
        return None
    token = raw.strip().lower()
    if token in _TRUE_WORDS:
        return True
    if token in _FALSE_WORDS:
        return False
    return None


def _float_or_none(raw: str | None) -> float | None:
    if raw is None:
        return None
    try:
        return float(raw.strip())
    except (ValueError, AttributeError):
        return None
