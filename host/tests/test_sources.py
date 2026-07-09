import struct

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame
from roomscan.sources import FileSource, Recorder, pump

HDR = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 16)
FRAME = pack_frame(HDR, struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))


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
