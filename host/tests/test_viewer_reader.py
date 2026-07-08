import queue
import struct

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
