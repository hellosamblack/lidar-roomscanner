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


def _rot_toward(direction: tuple[float, float, float]) -> np.ndarray:
    """Orthonormal 3x3 whose 3rd column is `direction` (unit-normalised) --
    the other two columns are an arbitrary right-handed completion. Enough
    for a synthetic near-zero-FOV frame, where only where the boresight ray
    lands matters."""
    d = np.asarray(direction, dtype=np.float64)
    d = d / np.linalg.norm(d)
    helper = np.array([1.0, 0.0, 0.0]) if abs(d[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    col0 = np.cross(helper, d)
    col0 /= np.linalg.norm(col0)
    col1 = np.cross(d, col0)
    return np.stack([col0, col1, d], axis=1)


def test_floorplan_height_band_excludes_grazing_vertical_contamination(monkeypatch):
    """#109: the reported symptom ("vertical point cloud instead of a
    horizontal floorplan") comes from tilted frames' rays -- ceiling/floor,
    grazing-angle -- landing far from the room's real footprint AND far from
    the shared origin's own height, flooding the top-down histogram.
    `_floorplan_ground_points`'s height band should keep the level "wall"
    content and drop that contamination.

    24 level frames sweep a 2 m-radius circle (Y == 0 exactly -- a small
    room's walls). 4 contaminant frames are pitched 60 deg off level at an
    8 m range: real ToF grazing-angle clutter looks exactly like this, a long
    return from a ray nearly parallel to the surface it hits. Their
    horizontal reach (8 * cos(60 deg) = 4 m) is well outside the room's real
    2 m footprint, and their height (8 * sin(60 deg) = 6.9 m) is well outside
    any plausible band.

    `display_rotation` is monkeypatched to an exact lookup -- this test is
    about the height-band arithmetic, not the SFLP quaternion convention
    (that's `test_sensors.py` / `docs/coordinate-frames.md`).
    """
    w = h = 8
    rots: dict[int, np.ndarray] = {}
    frames = []
    depth_core = np.full((h, w), 2000.0, dtype=np.float32)     # 2 m "wall"
    for i in range(24):
        theta = 2.0 * np.pi * i / 24
        rots[i] = _rot_toward((np.sin(theta), 0.0, np.cos(theta)))
        frames.append((depth_core, i))

    depth_far = np.full((h, w), 8000.0, dtype=np.float32)      # 8 m grazing return
    pitch = np.deg2rad(60.0)
    for j in range(4):
        idx = 100 + j
        az = 2.0 * np.pi * j / 4
        d = (np.sin(az) * np.cos(pitch), np.sin(pitch), np.cos(az) * np.cos(pitch))
        rots[idx] = _rot_toward(d)
        frames.append((depth_far, idx))

    monkeypatch.setattr(thumbs, "display_rotation", lambda q: rots[q])

    # With the fix (height-band filtering on): the contaminant frames are
    # excluded, so the footprint stays near the 2 m room radius.
    x, z = thumbs._floorplan_ground_points(frames, w, h, fov_h=2.0, fov_v=2.0,
                                           height_band_m=0.4)
    radius = np.hypot(x, z)
    assert radius.max() < 2.5, f"grazing contamination leaked through: max radius {radius.max():.2f} m"

    # Reintroducing the pre-fix defect (no height filter at all) must let the
    # contamination back in -- proving the assertion above exercises the
    # band, not some other clamp.
    x_nf, z_nf = thumbs._floorplan_ground_points(frames, w, h, fov_h=2.0, fov_v=2.0,
                                                 height_band_m=None)
    radius_nf = np.hypot(x_nf, z_nf)
    assert radius_nf.max() > 3.5, (
        f"expected the unfiltered defect to inflate the footprint, got max radius {radius_nf.max():.2f} m")


def test_render_floorplan_applies_the_height_band_by_default(monkeypatch, tmp_path):
    """`render_floorplan`'s default argument must actually reach
    `_floorplan_ground_points` as a real (non-None, non-infinite) band --
    the helper-level test above would not catch a caller that forgot to wire
    the parameter through."""
    cap = tmp_path / "a.bin"
    _capture(cap, 40)
    s = thumbs.sample_capture_frames(cap, 20)
    seen = {}
    real = thumbs._floorplan_ground_points

    def spy(frames, width, height, fov_h, fov_v, height_band_m):
        seen["height_band_m"] = height_band_m
        return real(frames, width, height, fov_h, fov_v, height_band_m)

    monkeypatch.setattr(thumbs, "_floorplan_ground_points", spy)
    thumbs.render_floorplan(s["frames"], s["width"], s["height"], size=64)
    assert seen["height_band_m"] == thumbs.DEFAULT_HEIGHT_BAND_M
    assert seen["height_band_m"] is not None


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
