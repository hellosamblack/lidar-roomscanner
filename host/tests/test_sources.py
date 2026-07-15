import socket
import struct
import time

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
