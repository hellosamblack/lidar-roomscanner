"""Capture thumbnails (§12) -- `roomscan.thumbs`.

The headline test here is `test_sampling_cost_is_independent_of_capture_size`:
the whole reason this module exists as a header-walk + selective read rather
than "decode the capture" is that the library holds >1 GB of files and a tile
must not cost more for a big one. That is proved with a counting wrapper on the
file object, not by reading the code.
"""
from __future__ import annotations

import struct
from pathlib import Path

import numpy as np
import pytest

from roomscan import thumbs
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame


def _capture(path: Path, n_frames: int, *, with_quat: bool = True,
             w: int = 54, h: int = 42) -> None:
    """A DEPTH_ZF32 capture (no native transform needed) with one IMU_QUAT
    before each depth frame, so the sampler has an orientation to rotate by.
    Depth is a slanted plane so the deprojection isn't degenerate."""
    yy, xx = np.mgrid[0:h, 0:w]
    out = bytearray()
    for i in range(n_frames):
        if with_quat:
            ang = 2.0 * np.pi * i / max(1, n_frames)
            q = struct.pack("<4f", float(np.cos(ang / 2)), 0.0, 0.0, float(np.sin(ang / 2)))
            out += pack_frame(FrameHeader(FrameType.DATA, StreamId.IMU_QUAT, 0, i, i * 35000,
                                          0, 0, len(q)), q)
        depth = (800.0 + 6.0 * xx + 4.0 * yy + 3.0 * i).astype(np.float32)
        payload = depth.astype("<f4").tobytes()
        out += pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, i + 1,
                                      i * 35000, w, h, len(payload)), payload)
    path.write_bytes(bytes(out))


class _CountingFile:
    """Wraps a real file object and counts bytes read + seeks."""

    def __init__(self, f):
        self._f = f
        self.bytes_read = 0
        self.reads = 0

    def read(self, n=-1):
        data = self._f.read(n)
        self.bytes_read += len(data)
        self.reads += 1
        return data

    def __getattr__(self, name):
        return getattr(self._f, name)

    def __enter__(self):
        self._f.__enter__()
        return self

    def __exit__(self, *exc):
        return self._f.__exit__(*exc)


# --- sampling ---------------------------------------------------------------

def test_sample_returns_frames_and_orientation(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 120)
    s = thumbs.sample_capture_frames(cap, 10)
    assert s["total_depth_frames"] == 120
    assert s["has_stream_9"] is True
    assert (s["width"], s["height"]) == (54, 42)
    assert 1 <= len(s["frames"]) <= 10
    assert all(q is not None for _d, q in s["frames"])


def test_sample_without_orientation_reports_no_stream_9(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 30, with_quat=False)
    s = thumbs.sample_capture_frames(cap, 10)
    assert s["has_stream_9"] is False
    assert all(q is None for _d, q in s["frames"])


def test_sampling_cost_is_independent_of_capture_size(tmp_path, monkeypatch):
    """A 1 GB capture must cost the same as a 10 MB one.

    Proved by counting, not inspection: the small and large captures here differ
    by 20x in bytes on disk, and the PAYLOAD bytes read (everything past the
    32-byte header walk) must be within a frame of each other. The header walk
    itself does scale -- it is one 32 B read + one seek per frame -- so this
    asserts on `payload_bytes`, and separately that total bytes read is a small
    fraction of the file.
    """
    small, large = tmp_path / "small.bin", tmp_path / "large.bin"
    _capture(small, 60)
    _capture(large, 1200)
    assert large.stat().st_size > 15 * small.stat().st_size

    got = {}
    real_open = open

    def counting_open(path, *a, **kw):
        f = _CountingFile(real_open(path, *a, **kw))
        got[Path(path).name] = f
        return f

    monkeypatch.setattr("roomscan.thumbs.open", counting_open, raising=False)
    s_small = thumbs.sample_capture_frames(small, 40)
    s_large = thumbs.sample_capture_frames(large, 40)

    # The payload read is set by the SAMPLE COUNT, not the file size.
    assert s_small["payload_bytes"] == pytest.approx(s_large["payload_bytes"], rel=0.05)
    # And the large capture is never anywhere near fully read.
    total_large = got["large.bin"].bytes_read
    assert total_large < 0.25 * large.stat().st_size, (
        f"read {total_large} of {large.stat().st_size} bytes -- the walk is not header-only")


def test_sample_stops_safely_on_a_truncated_capture(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 20)
    data = cap.read_bytes()
    cap.write_bytes(data[: len(data) // 2] + b"\x00" * 64)
    s = thumbs.sample_capture_frames(cap, 10)
    assert s["total_depth_frames"] >= 1        # what survived, no exception


def test_sample_of_an_empty_file_is_empty(tmp_path):
    cap = tmp_path / "empty.bin"
    cap.write_bytes(b"")
    s = thumbs.sample_capture_frames(cap, 10)
    assert s == {"frames": [], "width": None, "height": None,
                 "total_depth_frames": 0, "has_stream_9": False, "payload_bytes": 0}


# --- rendering --------------------------------------------------------------

def test_render_floorplan_is_a_square_rgb_image(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 80)
    s = thumbs.sample_capture_frames(cap, 20)
    img = thumbs.render_floorplan(s["frames"], s["width"], s["height"], size=64)
    assert img.shape == (64, 64, 3) and img.dtype == np.uint8
    assert img.max() > 0                      # something was actually drawn


def test_render_floorplan_returns_none_without_orientation(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 30, with_quat=False)
    s = thumbs.sample_capture_frames(cap, 10)
    assert thumbs.render_floorplan(s["frames"], s["width"], s["height"], size=64) is None


def test_render_depth_fallback_upscales_one_frame(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 30, with_quat=False)
    s = thumbs.sample_capture_frames(cap, 10)
    img = thumbs.render_depth_fallback(s["frames"], size=96)
    assert img.shape == (96, 96, 3) and img.dtype == np.uint8


# --- make_thumbnail ---------------------------------------------------------

def test_make_thumbnail_writes_a_floorplan_and_caches_it(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 80)
    tdir = tmp_path / "thumbs"
    assert thumbs.make_thumbnail(cap, tdir, size=64) == "floorplan"
    dest = thumbs.thumb_path(cap, tdir, size=64)
    assert dest.is_file() and dest.stat().st_size > 0
    assert dest.name == f"a__64_{cap.stat().st_mtime_ns}.png"
    assert thumbs.make_thumbnail(cap, tdir, size=64) == "cached"
    # Nothing half-written is ever left behind.
    assert not list(tdir.glob("*.tmp.png"))


def test_make_thumbnail_falls_back_to_depth_without_orientation(tmp_path):
    cap = tmp_path / "a.bin"
    _capture(cap, 30, with_quat=False)
    assert thumbs.make_thumbnail(cap, tmp_path / "thumbs", size=64) == "depth"


@pytest.mark.parametrize("content", [b"", b"not a capture at all", b"RSCN" + b"\x00" * 200])
def test_make_thumbnail_never_raises_on_junk(tmp_path, content):
    """It is called from a GET route serving lazy <img> loads: a truncated
    capture must degrade to "no tile", never to a 500."""
    cap = tmp_path / "junk.bin"
    cap.write_bytes(content)
    assert thumbs.make_thumbnail(cap, tmp_path / "thumbs", size=64) == ""


def test_make_thumbnail_never_raises_on_a_missing_file(tmp_path):
    assert thumbs.make_thumbnail(tmp_path / "nope.bin", tmp_path / "thumbs") == ""


def test_thumb_path_identity_changes_when_the_capture_does(tmp_path):
    """The cache key is in the FILENAME, exactly like `_CAPTURE_INFO_CACHE`'s
    `(path, size, mtime_ns)` -- which is what makes the HTTP response safely
    `immutable`."""
    cap = tmp_path / "a.bin"
    _capture(cap, 10)
    first = thumbs.thumb_path(cap, tmp_path / "t")
    import os
    st = cap.stat()
    os.utime(cap, ns=(st.st_atime_ns, st.st_mtime_ns + 1_000_000))
    assert thumbs.thumb_path(cap, tmp_path / "t") != first
