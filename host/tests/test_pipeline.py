import numpy as np
import pytest

from roomscan.native import Transform
from roomscan.pipeline import TransformStage
from roomscan.protocol import (
    Frame, FrameHeader, FrameType, StreamId, pack_frame,
)
from tests.golden import load_golden_pairs

needs_dll = pytest.mark.skipif(not Transform.available(), reason="native transform DLL not built")


def _calib_frame(payload: bytes, seq: int = 1) -> Frame:
    return Frame(FrameHeader(FrameType.DATA, StreamId.CALIB, 0, seq, 0, 0, 0, len(payload)), payload)


def _raw_frame(payload: bytes, seq: int = 1) -> Frame:
    return Frame(FrameHeader(FrameType.DATA, StreamId.RAW_3DMD, 0, seq, 0, 0, 0, len(payload)), payload)


@needs_dll
def test_raw_before_calib_counted_and_dropped():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    stage = TransformStage()
    result = stage.feed(_raw_frame(raw))

    assert result is None
    assert stage.raw_skipped_awaiting_calib == 1
    assert stage.raw_transformed == 0
    assert not stage.active


@needs_dll
def test_calib_then_raw_matches_direct_transform():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    stage = TransformStage()
    assert stage.feed(_calib_frame(calib)) is None
    assert stage.active

    result = stage.feed(_raw_frame(raw, seq=2))
    assert result is not None
    header, depth = result
    assert header.stream_id == StreamId.RAW_3DMD
    assert depth.shape == (42, 54) and depth.dtype == np.float32
    assert stage.raw_transformed == 1

    # Fresh Transform instance fed the same single raw frame -- deterministic:
    # both start from the same just-reset TNR state, so direct process() output
    # must match the stage's output exactly.
    direct = Transform(calib).process(raw)
    assert np.array_equal(depth, direct)


def test_depth_passthrough_without_dll():
    # No DLL gating: DEPTH_ZF32 is a pure decode, independent of Transform.available().
    payload = np.array([[1.0, 2.0], [3.0, 4.0]], dtype="<f4").tobytes()
    header = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 9, 123, 2, 2, len(payload))
    packed = pack_frame(header, payload)

    from roomscan.decoder import StreamDecoder
    frames = StreamDecoder().feed(packed)
    assert len(frames) == 1

    stage = TransformStage()
    result = stage.feed(frames[0])

    assert result is not None
    out_header, depth = result
    assert out_header == header
    assert depth.shape == (2, 2) and depth.dtype == np.float32
    np.testing.assert_array_equal(depth, np.array([[1.0, 2.0], [3.0, 4.0]], dtype=np.float32))
    assert not stage.active  # DEPTH passthrough never touches the DLL/Transform


def test_unknown_stream_returns_none():
    stage = TransformStage()
    frame = Frame(FrameHeader(FrameType.DATA, StreamId.STATUS, 0, 1, 0, 0, 0, 4), b"\x00" * 4)
    assert stage.feed(frame) is None
    assert not stage.active


@needs_dll
def test_different_calib_replaces_transform():
    calib, pairs = load_golden_pairs()
    raw, _ = pairs[0]

    mutated = bytearray(calib)
    mutated[0] ^= 0xFF
    mutated_calib = bytes(mutated)
    assert mutated_calib != calib

    stage = TransformStage()
    stage.feed(_calib_frame(calib, seq=1))
    assert stage.active
    stage.feed(_raw_frame(raw, seq=2))
    assert stage.raw_transformed == 1

    stage.feed(_calib_frame(mutated_calib, seq=65))
    assert stage.active  # rebuilt, not just kept

    result = stage.feed(_raw_frame(raw, seq=66))
    assert result is not None
    assert stage.raw_transformed == 2
