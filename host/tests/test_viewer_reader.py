import queue
import struct
from types import SimpleNamespace

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.viewer import Stats, _reader


class ExplodingSource:
    def read(self):
        raise OSError("device gone")

    def close(self):
        pass


def test_reader_surfaces_fault():
    fault: dict = {}
    _reader(ExplodingSource(), StreamDecoder(), queue.Queue(maxsize=1), Stats(), None, fault)
    assert isinstance(fault["error"], OSError)


def test_reader_counts_stats():
    frame = pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 5, 0, 2, 2, 16),
                       struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))

    class OneShotThenStop:
        def __init__(self):
            self._sent = False

        def read(self):
            if self._sent:
                raise StopIteration  # any exception ends _reader via the fault path
            self._sent = True
            return frame

        def close(self):
            pass

    fault: dict = {}
    stats = Stats()
    _reader(OneShotThenStop(), StreamDecoder(), queue.Queue(maxsize=1), stats, None, fault)
    assert stats.frames == 1 and stats._last_seq == 5


def test_reader_prints_event_frames(capsys):
    event_payload = struct.pack("<II", 2, 3) + b"trigger retries exhausted"
    frame = pack_frame(FrameHeader(FrameType.EVENT, 0, 0, 1, 0, 0, 0, len(event_payload)),
                       event_payload)

    class OneShotThenStop:
        def __init__(self):
            self._sent = False

        def read(self):
            if self._sent:
                raise StopIteration  # any exception ends _reader via the fault path
            self._sent = True
            return frame

        def close(self):
            pass

    fault: dict = {}
    stats = Stats()
    _reader(OneShotThenStop(), StreamDecoder(), queue.Queue(maxsize=1), stats, None, fault)
    out = capsys.readouterr().out
    assert "code=2" in out
    assert "trigger retries exhausted" in out
    assert stats.frames == 0


def test_reader_paces_frames_with_min_interval():
    import struct
    import time

    from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

    frames = b"".join(
        pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i, 0, 2, 2, 16),
                   struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))
        for i in range(1, 4)
    )

    class ThreeThenStop:
        def __init__(self):
            self._sent = False

        def read(self):
            if self._sent:
                raise StopIteration
            self._sent = True
            return frames

        def close(self):
            pass

    fault: dict = {}
    stats = Stats()
    slot = queue.Queue()          # unbounded here: we want all 3 paced puts to land
    t0 = time.monotonic()
    _reader(ThreeThenStop(), StreamDecoder(), slot, stats, None, fault, min_interval=0.05)
    elapsed = time.monotonic() - t0
    assert stats.frames == 3
    # Frames 2 and 3 each waited ~50 ms, so the theoretical minimum is 0.10 s —
    # but OS sleeps can return marginally early, making an exact >= 0.10 flaky.
    # 0.08 keeps a scheduling-jitter margin while still discriminating
    # unambiguously: with pacing off, elapsed is ~0.001 s.
    assert elapsed >= 0.08


def test_stats_new_stream_forgets_the_sequence_without_losing_the_totals():
    """A source swap is not a gap. Sequence numbers are per-source, so the first
    frame after live->replay->live differs from the last by however far apart the
    two numberings are -- booked as lost frames it read 1,529,274 gaps in a
    session with 0 drops streaming cleanly at 30 fps (BUG-057). The running
    totals must survive: BUG-049's transport-loss work reads them."""
    s = Stats()
    for seq in (1, 2, 4):                 # one real gap of 1
        s.update(SimpleNamespace(seq=seq, flags=0))
    assert (s.frames, s.seq_gaps) == (3, 1)

    s.new_stream()
    s.update(SimpleNamespace(seq=900000, flags=0))     # other source's numbering
    assert s.seq_gaps == 1, "the swap itself must not count as ~900k lost frames"
    assert s.frames == 4, "totals are session-level and must not reset"

    s.update(SimpleNamespace(seq=900002, flags=0))     # real gaps still counted
    assert s.seq_gaps == 2
