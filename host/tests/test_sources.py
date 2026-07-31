import socket
import struct
import time

import pytest

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.sources import FileSource, Recorder, UdpSource, get_best_source, pump

HDR = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 16)
FRAME = pack_frame(HDR, struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))


# --- UdpSource._resolve_target: mDNS-first, broadcast-fallback --------------

class _FakeInfo:
    def __init__(self, addrs): self._addrs = addrs
    def parsed_addresses(self): return self._addrs


class _FakeZeroconf:
    """Injectable stand-in for zeroconf.Zeroconf -- no real network I/O."""
    _answer = None   # class-level knob the test sets before constructing UdpSource

    def get_service_info(self, service_type, name, timeout=1500):
        return self._answer

    def close(self): pass


def test_resolve_target_uses_mdns_address_when_found():
    _FakeZeroconf._answer = _FakeInfo(["10.1.2.3"])
    try:
        src = UdpSource(port=0, zeroconf_factory=_FakeZeroconf)
        try:
            assert src.target_ip == "10.1.2.3"
        finally:
            src.close()
    finally:
        _FakeZeroconf._answer = None


def test_resolve_target_falls_back_to_broadcast_when_mdns_finds_nothing():
    _FakeZeroconf._answer = None   # zeroconf found no matching service
    src = UdpSource(port=0, zeroconf_factory=_FakeZeroconf)
    try:
        assert src.target_ip == "255.255.255.255"
        # SO_BROADCAST must actually be enabled, or a broadcast sendto() would fail
        assert src.sock.getsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST) != 0
    finally:
        src.close()


def test_resolve_target_falls_back_to_broadcast_on_zeroconf_error():
    class _BrokenZeroconf:
        def __init__(self): raise OSError("no multicast interface")

    src = UdpSource(port=0, zeroconf_factory=_BrokenZeroconf)
    try:
        assert src.target_ip == "255.255.255.255"
    finally:
        src.close()


# --- get_best_source: real retry loop, not a single blocking read -----------

def test_get_best_source_resends_wake_packet_and_returns_promptly_on_data(monkeypatch):
    """Regression (owner, 2026-07-15): the old code set the *socket's* own
    timeout to the full probe window before the retry loop, so the very
    first `udp.read()` call blocked for the whole window internally -- the
    outer `while` never got a real second iteration, so exactly one wake
    packet was ever sent and a single dropped UDP packet silently killed
    Ethernet preference for the whole launch ("we had comms over ethernet
    working prior to this... it's supposed to prefer ethernet"). Now: short
    per-read timeout, real polling, periodic resend, and an immediate return
    the moment data arrives (no full-window wait)."""
    import roomscan.sources as sources

    class _FakeUdp:
        def __init__(self, *a, **k):
            self.sock = _FakeSock()
            self.writes = 0
            self.closed = False
            self._t0 = time.time()

        def write(self, data):
            self.writes += 1

        def read(self):
            # "device" answers only once a couple of resend windows have
            # genuinely elapsed -- proves the loop is really polling/resending,
            # not just blocking once on the first call.
            if time.time() - self._t0 >= 0.12 and self.writes >= 2:
                return b"frame-data"
            return b""

        def close(self):
            self.closed = True

    class _FakeSock:
        def gettimeout(self): return 0.05
        def settimeout(self, v): pass

    fake_holder = {}

    def _make_fake(*a, **k):
        fake_holder["fake"] = _FakeUdp()
        return fake_holder["fake"]

    monkeypatch.setattr(sources, "UdpSource", _make_fake)
    t0 = time.time()
    result = sources.get_best_source(probe_s=2.0, resend_s=0.05)
    elapsed = time.time() - t0

    fake = fake_holder["fake"]
    assert result is fake
    assert not fake.closed                # never fell back to Serial
    assert fake.writes >= 2               # resent the wake packet, not just once
    assert elapsed < 1.0                  # returned promptly, did not wait out probe_s


def test_udp_source_does_not_retarget_on_a_short_loopback_datagram():
    """Regression (owner, 2026-07-28): with no mDNS the source broadcasts its
    wake, Linux loops that 1-byte datagram back into the same socket, and
    `read()` used to set `target_ip` from *any* sender -- so the host adopted
    itself as the device and every subsequent wake/keepalive went nowhere near
    the board. Only a datagram long enough to be a real fragment may retarget."""
    _FakeZeroconf._answer = None              # no mDNS -> broadcast fallback
    src = UdpSource(port=0, timeout=0.01, zeroconf_factory=_FakeZeroconf)
    real_sock, src.sock = src.sock, None
    real_sock.close()
    assert src.target_ip == "255.255.255.255"

    class _Loopback:
        def recvfrom(self, n): return b"\x00", ("172.17.2.54", 5000)
        def settimeout(self, v): pass
        def close(self): pass

    src.sock = _Loopback()
    src.keepalive_s = 0                       # keepalive would need a real socket
    assert src.read() == b""
    assert src.target_ip == "255.255.255.255"  # unchanged -- still broadcasting


class _SilentUdp:
    """A UdpSource stand-in that never hears the device (probe times out)."""
    def __init__(self, *a, **k):
        self.sock = _FakeSock()
        self.target_port = 5000
        self.closed = False

    def write(self, data): pass
    def read(self): return b""
    def close(self): self.closed = True


class _FakeSock:
    def gettimeout(self): return 0.05
    def settimeout(self, v): pass


def test_get_best_source_keeps_udp_when_there_is_no_serial_port(monkeypatch, capsys):
    """Regression (owner, 2026-07-28): with the ST-Link unplugged and the board
    powered from USB_USER there IS no CDC port, so the serial fallback raised
    `no scanner serial port found among [...]` and killed the launch of an
    app that only ever wanted Ethernet. A missing port must degrade to the UDP
    source -- its keepalive re-wakes the board, so the stream starts on its own
    once the device appears."""
    import roomscan.sources as sources

    holder = {}

    def _make(*a, **k):
        holder["udp"] = _SilentUdp()
        return holder["udp"]

    class _NoPort:
        def __init__(self, *a, **k):
            raise RuntimeError("no scanner serial port found among ['/dev/ttyACM0']")

    monkeypatch.setattr(sources, "UdpSource", _make)
    monkeypatch.setattr(sources, "SerialSource", _NoPort)
    result = sources.get_best_source(probe_s=0.05, resend_s=0.01)

    assert result is holder["udp"]
    assert not result.closed              # still usable -- socket left open
    assert "no scanner serial port" in capsys.readouterr().err


def test_get_best_source_still_raises_for_a_busy_serial_port(monkeypatch):
    """A *busy* port is a different story: a real scanner port exists and
    something else holds it, which panel._open_source resolves interactively.
    Swallowing that would hide the one case the user can actually fix."""
    import roomscan.sources as sources

    holder = {}

    def _make(*a, **k):
        holder["udp"] = _SilentUdp()
        return holder["udp"]

    class _BusyPort:
        def __init__(self, *a, **k):
            raise PermissionError(13, "Access is denied.")

    monkeypatch.setattr(sources, "UdpSource", _make)
    monkeypatch.setattr(sources, "SerialSource", _BusyPort)
    with pytest.raises(PermissionError):
        sources.get_best_source(probe_s=0.05, resend_s=0.01)
    assert holder["udp"].closed           # no socket leak on the raising path


def test_file_source_replays_all_frames(tmp_path):
    p = tmp_path / "cap.bin"
    p.write_bytes(b"boot noise\r\n" + FRAME * 5)
    frames = list(pump(FileSource(p), StreamDecoder()))
    assert len(frames) == 5


def test_pump_records_raw_bytes(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(FRAME * 3)
    rec = tmp_path / "rec.bin"
    frames = list(pump(FileSource(src_file), StreamDecoder(), record_path=rec))
    assert len(frames) == 3
    assert rec.read_bytes() == FRAME * 3   # byte-exact tee


def test_pump_flushes_recording_on_early_close(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(FRAME * 3)
    rec = tmp_path / "rec.bin"
    gen = pump(FileSource(src_file, chunk=len(FRAME)), StreamDecoder(), record_path=rec)
    next(gen)          # consume one frame, generator suspended at yield
    gen.close()        # early termination — finally must close/flush rec
    assert rec.read_bytes() == FRAME  # exactly the one chunk read so far


def test_recorded_capture_replays_identically(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(b"junk" + FRAME * 2)
    rec = tmp_path / "rec.bin"
    first = list(pump(FileSource(src_file), StreamDecoder(), record_path=rec))
    second = list(pump(FileSource(rec), StreamDecoder()))
    assert [f.payload for f in first] == [f.payload for f in second]


def test_file_source_write_raises(tmp_path):
    import pytest
    p = tmp_path / "cap.bin"
    p.write_bytes(FRAME)
    src = FileSource(p)
    try:
        with pytest.raises(NotImplementedError):
            src.write(b"\x00")   # replay is read-only: no device to write to
    finally:
        src.close()


def test_recorder_start_write_stop_roundtrip(tmp_path):
    p = tmp_path / "rec.bin"
    rec = Recorder()
    assert not rec.active
    assert rec.path is None
    rec.start(p)
    assert rec.active
    assert rec.path == p
    rec.write(b"hello ")
    rec.write(b"world")
    rec.stop()
    assert not rec.active
    assert rec.path is None
    assert p.read_bytes() == b"hello world"   # closed => flushed to disk


def test_recorder_mid_stream_start_stop_only_captures_active_segment(tmp_path):
    p = tmp_path / "rec.bin"
    rec = Recorder()
    rec.write(b"before-not-recorded")   # inactive: no-op
    rec.start(p)
    rec.write(b"middle-recorded")
    rec.stop()
    rec.write(b"after-not-recorded")    # inactive again: no-op
    assert p.read_bytes() == b"middle-recorded"


def test_recorder_stop_when_inactive_is_noop(tmp_path):
    rec = Recorder()
    rec.stop()          # must not raise
    rec.stop()           # idempotent
    assert not rec.active


def test_recorder_write_when_inactive_is_noop(tmp_path):
    rec = Recorder()
    rec.write(b"ignored")  # must not raise, must not create a file
    assert not rec.active


def test_recorder_start_while_active_switches_files(tmp_path):
    p1 = tmp_path / "rec1.bin"
    p2 = tmp_path / "rec2.bin"
    rec = Recorder()
    rec.start(p1)
    rec.write(b"segment-one")
    rec.start(p2)   # documented behavior: closes p1, switches to p2 (no exception)
    rec.write(b"segment-two")
    rec.stop()
    assert p1.read_bytes() == b"segment-one"
    assert p2.read_bytes() == b"segment-two"


def test_recorder_close_is_idempotent_and_flushes(tmp_path):
    p = tmp_path / "rec.bin"
    rec = Recorder()
    rec.start(p)
    rec.write(b"data")
    rec.close()
    rec.close()   # safe to call again
    assert p.read_bytes() == b"data"


def test_pump_tees_raw_chunks_into_recorder(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(FRAME * 3)
    rec_path = tmp_path / "rec.bin"
    rec = Recorder()
    rec.start(rec_path)
    frames = list(pump(FileSource(src_file), StreamDecoder(), recorder=rec))
    assert len(frames) == 3
    assert rec_path.read_bytes() == FRAME * 3
    # pump does not own the recorder's lifecycle: still active after pump returns.
    assert rec.active
    rec.close()


def test_pump_leaves_inactive_recorder_untouched_when_not_started(tmp_path):
    src_file = tmp_path / "cap.bin"
    src_file.write_bytes(FRAME * 2)
    rec = Recorder()
    frames = list(pump(FileSource(src_file), StreamDecoder(), recorder=rec))
    assert len(frames) == 2
    assert not rec.active   # pump never called start(); write() was a no-op


# --- UdpSource fragment reassembly: order-independent, and counted ----------
#
# The device splits a frame into 1400-byte fragments carrying
# {u32 seq, u8 frag_idx, u8 total_frags}. Reassembly used to require
# frag_idx == expected and append, so a REORDERED datagram destroyed the frame
# exactly like a lost one -- and silently. These pin the fix.

def _frag(seq: int, idx: int, total: int, payload: bytes) -> bytes:
    return struct.pack("<IBB", seq, idx, total) + payload


class _ScriptedSocket:
    """Feeds a fixed list of datagrams to UdpSource.read(), then times out."""
    def __init__(self, datagrams):
        self._queue = list(datagrams)

    def recvfrom(self, n):
        if not self._queue:
            raise socket.timeout()
        return self._queue.pop(0), ("10.0.0.9", 5000)

    def sendto(self, *a, **k): pass
    def close(self): pass
    def setsockopt(self, *a, **k): pass
    def settimeout(self, *a, **k): pass


def _udp_with(datagrams):
    src = UdpSource(port=0, zeroconf_factory=_FakeZeroconf)
    src.sock.close()
    src.sock = _ScriptedSocket(datagrams)
    src._maybe_keepalive = lambda: None   # no wake traffic in tests
    return src


def _drain(src, n_reads):
    return [r for r in (src.read() for _ in range(n_reads)) if r]


def test_reassembly_accepts_out_of_order_fragments():
    """The regression that motivated this: fragments 0,2,1 must still yield the
    frame. Under the old append-if-expected logic this returned nothing at all
    and the frame was lost -- for a reorder, not a loss."""
    body = [b"A" * 1400, b"B" * 1400, b"C" * 20]
    src = _udp_with([_frag(5, 0, 3, body[0]),
                     _frag(5, 2, 3, body[2]),
                     _frag(5, 1, 3, body[1])])
    try:
        assert _drain(src, 3) == [b"".join(body)]
        assert src.frags_reordered == 1     # frag 1 arrived after frag 2
        assert src.frames_incomplete == 0   # nothing was actually lost
    finally:
        src.close()


def test_reassembly_still_handles_in_order_fragments():
    body = [b"x" * 1400, b"y" * 7]
    src = _udp_with([_frag(1, 0, 2, body[0]), _frag(1, 1, 2, body[1])])
    try:
        assert _drain(src, 2) == [b"".join(body)]
        assert src.frags_reordered == 0
    finally:
        src.close()


def test_reassembly_counts_a_genuinely_lost_fragment():
    """A hole that never fills must be COUNTED, not just silently dropped --
    otherwise transport loss is invisible (the frame never reaches the decoder,
    so the only downstream trace is a header seq gap)."""
    src = _udp_with([_frag(7, 0, 3, b"a" * 1400),
                     _frag(7, 2, 3, b"c" * 10),        # frag 1 never arrives
                     _frag(8, 0, 1, b"next frame")])
    try:
        assert _drain(src, 3) == [b"next frame"]
        assert src.frames_incomplete == 1
        assert src.frags_lost == 1
    finally:
        src.close()


def test_reassembly_ignores_duplicate_fragment():
    src = _udp_with([_frag(3, 0, 2, b"p" * 1400),
                     _frag(3, 0, 2, b"p" * 1400),   # echo / retransmit
                     _frag(3, 1, 2, b"q" * 5)])
    try:
        assert _drain(src, 3) == [b"p" * 1400 + b"q" * 5]
        assert src.frags_duplicate == 1
    finally:
        src.close()


def test_reassembly_rejects_inconsistent_total_frags():
    """Corrupt framing must not resize the frame mid-flight and silently
    truncate it."""
    src = _udp_with([_frag(4, 0, 3, b"a" * 1400),
                     _frag(4, 1, 9, b"b" * 1400),   # disagrees with the seq
                     _frag(4, 1, 3, b"b" * 1400),
                     _frag(4, 2, 3, b"c" * 3)])
    try:
        assert _drain(src, 4) == [b"a" * 1400 + b"b" * 1400 + b"c" * 3]
        assert src.frags_invalid == 1
    finally:
        src.close()
