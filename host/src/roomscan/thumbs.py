"""Capture thumbnails for the View page's file browser (§12).

A pure module: it imports `numpy`, `PIL.Image` and the decoding half of the
package (`protocol` / `decoder` / `pipeline` / `deproject` / `colors` /
`sensors`) and **never** imports `web`. `web` imports the whole server, the SLAM
stack, and `sensors` -- importing it back from here would be a cycle. That is
also why `display_rotation` was moved to `sensors.py`.

The cost model is the point of this module. A capture library holds ~34 files
totalling >1 GB; some single captures are 200 MB. Generating a tile must not
depend on which one you picked:

  * `sample_capture_frames` walks **headers only** (32 B read + a seek past each
    payload -- the `scan_capture_metadata` loop), building an index of frame
    offsets. Nothing but headers is read in that pass.
  * It then seeks to and reads **only** the ~40 chosen depth frames, the CALIB
    that governs each, and the IMU_QUAT preceding each. That is ~40 payloads,
    not the file.

So a 1 GB capture costs the same payload bytes as a 10 MB one, and the header
walk (the only part that scales) is a sequential seek chain. `test_thumbs.py`
proves this with a counting wrapper on the file object rather than by
inspection.

Two renderings:

  * `render_floorplan` -- **not a map.** With no translation estimate every
    sampled frame is deprojected about the same origin, so what you see is the
    *rotational sweep* of the scan: how much of the room the operator aimed at,
    in which directions. It reads like a floor plan for a room-sweep capture
    only because the operator turned in place. The tile's tooltip says so; do
    not let it be read as a map.
  * `render_depth_fallback` -- one colorized depth raster from ~10% in, for a
    capture with no orientation stream (nothing to rotate by).
"""
from __future__ import annotations

import os
from pathlib import Path

import numpy as np

from .colors import normalize, turbo
from .decoder import StreamDecoder
from .deproject import Deprojector
from .pipeline import TransformStage
from .protocol import HEADER_SIZE, FrameHeader, FrameType, ProtocolError, StreamId, decode_imu_quat
from .sensors import display_rotation

DEFAULT_SAMPLES = 40
DEFAULT_SIZE = 256
THUMBS_DIR = "thumbs"

_DEPTH_STREAMS = (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32)


# --- index walk + selective read --------------------------------------------

def _index_frames(f) -> tuple[list[tuple[int, int]], list[tuple[int, int]], list[tuple[int, int]]]:
    """Header-only walk -> (depth_spans, calib_spans, quat_spans).

    Each span is (byte_offset, byte_length) of a whole wire frame (header +
    payload + CRC), so a caller can re-read it verbatim and hand it to
    `StreamDecoder`. Stops safely at the first malformed/truncated frame, like
    `scan_capture_metadata`.
    """
    depth: list[tuple[int, int]] = []
    calib: list[tuple[int, int]] = []
    quat: list[tuple[int, int]] = []
    off = 0
    f.seek(0)
    while True:
        raw = f.read(HEADER_SIZE)
        if len(raw) < HEADER_SIZE:
            break
        try:
            hdr = FrameHeader.unpack(raw)
        except ProtocolError:
            break
        if hdr.payload_len < 0:
            break
        total = HEADER_SIZE + hdr.payload_len + 4
        if hdr.frame_type == FrameType.DATA:
            if hdr.stream_id in _DEPTH_STREAMS:
                depth.append((off, total))
            elif hdr.stream_id == StreamId.CALIB:
                calib.append((off, total))
            elif hdr.stream_id == StreamId.IMU_QUAT:
                quat.append((off, total))
        off += total
        f.seek(off)
    return depth, calib, quat


def _last_before(spans: list[tuple[int, int]], offset: int) -> tuple[int, int] | None:
    """The last span starting strictly before `offset` (spans are in file order)."""
    lo, hi = 0, len(spans)
    while lo < hi:
        mid = (lo + hi) // 2
        if spans[mid][0] < offset:
            lo = mid + 1
        else:
            hi = mid
    return spans[lo - 1] if lo > 0 else None


def sample_capture_frames(path, n: int = DEFAULT_SAMPLES) -> dict:
    """Decode ~`n` evenly-spaced depth frames from `path` plus their orientation.

    Returns ``{"frames": [(depth_mm(h, w), quat|None), ...], "width", "height",
    "total_depth_frames", "has_stream_9", "payload_bytes"}``. `payload_bytes` is
    what was actually read past the header walk -- the number the cost claim in
    the module docstring is about.

    Never reads the whole file: see the module docstring.
    """
    n = max(1, int(n))
    out: list[tuple[np.ndarray, tuple | None]] = []
    width = height = None
    payload_bytes = 0
    with open(path, "rb") as f:
        depth_spans, calib_spans, quat_spans = _index_frames(f)
        total = len(depth_spans)
        if total == 0:
            return {"frames": [], "width": None, "height": None,
                    "total_depth_frames": 0, "has_stream_9": bool(quat_spans),
                    "payload_bytes": 0}

        # Evenly spaced over the whole capture, endpoints included.
        if total <= n:
            picks = list(range(total))
        else:
            picks = sorted({int(round(i * (total - 1) / (n - 1))) if n > 1 else 0
                            for i in range(n)})

        # Gather every span we will actually read: the picked depth frames, plus
        # the CALIB governing each and the IMU_QUAT preceding each. Deduped and
        # sorted by offset so the decoder sees them in file order (CALIB before
        # the RAW frames it calibrates) and the reads are a forward seek chain.
        needed: set[tuple[int, int]] = set()
        for i in picks:
            off = depth_spans[i][0]
            needed.add(depth_spans[i])
            c = _last_before(calib_spans, off)
            if c is not None:
                needed.add(c)
            q = _last_before(quat_spans, off)
            if q is not None:
                needed.add(q)

        dec = StreamDecoder()
        stage = TransformStage(outputs=("depth",))
        last_quat = None
        for off, size in sorted(needed):
            f.seek(off)
            blob = f.read(size)
            payload_bytes += len(blob)
            for frame in dec.feed(blob):
                h = frame.header
                if h.frame_type != FrameType.DATA:
                    continue
                if h.stream_id == StreamId.IMU_QUAT:
                    try:
                        last_quat = decode_imu_quat(frame.payload)
                    except Exception:
                        last_quat = None
                    continue
                res = stage.feed(frame)
                if res is None:
                    continue
                header, arrays = res
                d = arrays.get("depth")
                if d is None:
                    continue
                width, height = header.width, header.height
                out.append((np.asarray(d, dtype=np.float32), last_quat))

    return {"frames": out, "width": width, "height": height,
            "total_depth_frames": total, "has_stream_9": bool(quat_spans),
            "payload_bytes": payload_bytes}


# --- rendering ---------------------------------------------------------------

def render_floorplan(frames, width: int, height: int, *, size: int = DEFAULT_SIZE,
                     fov_h: float = 55.0, fov_v: float = 42.0) -> np.ndarray | None:
    """Orientation-only rotational sweep as an RGB uint8 (size, size, 3) image.

    Every frame is deprojected about the SAME origin (there is no translation
    estimate here -- that costs a full SLAM run) and rotated into the
    gravity-aligned world by `display_rotation(quat)`. World (X, Z) -- the
    ground plane, since the Open3D CV world is Y-down -- is 2-D histogrammed and
    log1p-normalised, because a stationary dwell otherwise saturates one cell
    and blacks out the rest of the sweep.

    Returns None when no frame carried an orientation (the caller falls back to
    `render_depth_fallback`).
    """
    deproj = Deprojector(width, height, fov_h, fov_v)
    xs: list[np.ndarray] = []
    zs: list[np.ndarray] = []
    for depth, quat in frames:
        rot = display_rotation(quat)
        if rot is None:
            continue
        pts = deproj(depth)                    # (N, 3) metres, CV camera frame
        if pts.size == 0:
            continue
        world = pts @ rot.T
        xs.append(world[:, 0])
        zs.append(world[:, 2])
    if not xs:
        return None
    x = np.concatenate(xs)
    z = np.concatenate(zs)
    if x.size == 0:
        return None

    # Square, centred, 2nd/98th-percentile extent so one stray far return can't
    # shrink the room to a dot.
    lo = float(np.percentile(np.concatenate([x, z]), 1.0))
    hi = float(np.percentile(np.concatenate([x, z]), 99.0))
    if not np.isfinite(lo) or not np.isfinite(hi) or hi <= lo:
        lo, hi = -1.0, 1.0
    pad = 0.05 * (hi - lo)
    rng = (lo - pad, hi + pad)

    hist, _, _ = np.histogram2d(z, x, bins=size, range=[rng, rng])
    dens = np.log1p(hist)
    peak = float(dens.max())
    dens = dens / peak if peak > 0 else dens
    # Row 0 of a histogram is the LOW edge of the first axis; flip so +Z (the
    # sensor's forward) renders upward, the way a plan view reads.
    rgb = turbo(np.flipud(dens))
    return (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)


def render_depth_fallback(frames, *, size: int = DEFAULT_SIZE) -> np.ndarray | None:
    """One colorized depth raster (~10% into the capture) as RGB uint8.

    Used when a capture has no orientation stream: with no rotation there is
    nothing to sweep, and a single frame at least says what the scene was.
    """
    if not frames:
        return None
    idx = min(len(frames) - 1, max(0, int(round(0.1 * (len(frames) - 1)))))
    depth = frames[idx][0]
    if depth is None or depth.size == 0:
        return None
    rgb = turbo(normalize(depth))
    img = (np.clip(rgb, 0.0, 1.0) * 255.0).astype(np.uint8)
    return _resize_nearest(img, size)


def _resize_nearest(img: np.ndarray, size: int) -> np.ndarray:
    """Nearest-neighbour upscale of a tiny (42x54) raster. Deliberately not
    smoothed: at 54x42 the zones ARE the data, and interpolating them invents
    detail the sensor never measured."""
    h, w = img.shape[:2]
    if h == 0 or w == 0:
        return img
    ry = np.minimum((np.arange(size) * h) // size, h - 1)
    rx = np.minimum((np.arange(size) * w) // size, w - 1)
    return img[ry][:, rx]


# --- cache + public entry point ---------------------------------------------

def thumb_path(capture, thumbs_dir=THUMBS_DIR, *, size: int = DEFAULT_SIZE) -> Path:
    """`thumbs/<stem>__<size>_<mtime_ns>.png`.

    The capture's identity is IN the filename, exactly like `_CAPTURE_INFO_CACHE`'s
    `(path, size, mtime_ns)` key -- so a capture that is rewritten or renamed gets
    a different path and the stale tile is simply never asked for again. It also
    makes the HTTP response safely `immutable`.
    """
    p = Path(capture)
    try:
        mtime_ns = p.stat().st_mtime_ns
    except OSError:
        mtime_ns = 0
    return Path(thumbs_dir) / f"{p.stem}__{int(size)}_{mtime_ns}.png"


def make_thumbnail(capture, thumbs_dir=THUMBS_DIR, *, size: int = DEFAULT_SIZE,
                   samples: int = DEFAULT_SAMPLES, fov_h: float = 55.0,
                   fov_v: float = 42.0) -> str:
    """Generate (or reuse) a capture's thumbnail. Returns the KIND rendered --
    ``"floorplan"``, ``"depth"``, or ``""`` when nothing could be rendered.

    **Never raises.** It is called from a GET route serving a browser's lazy
    `<img>` loads; a truncated capture, an unbuildable native transform, or an
    unwritable thumbs dir must all degrade to "no tile", not to a 500.

    Writes atomically (`.tmp.png` -> `os.replace`, the `write_manifest_atomic`
    pattern) so a concurrent request can never read a half-written PNG.
    """
    try:
        from PIL import Image
    except Exception:
        return ""
    dest = thumb_path(capture, thumbs_dir, size=size)
    try:
        if dest.is_file() and dest.stat().st_size > 0:
            return "cached"
    except OSError:
        pass

    try:
        sampled = sample_capture_frames(capture, samples)
        frames = sampled["frames"]
        if not frames or not sampled["width"]:
            return ""
        img = None
        kind = ""
        if sampled["has_stream_9"]:
            img = render_floorplan(frames, sampled["width"], sampled["height"],
                                   size=size, fov_h=fov_h, fov_v=fov_v)
            kind = "floorplan" if img is not None else ""
        if img is None:
            img = render_depth_fallback(frames, size=size)
            kind = "depth" if img is not None else ""
        if img is None:
            return ""
        dest.parent.mkdir(parents=True, exist_ok=True)
        tmp = dest.with_suffix(".tmp.png")
        Image.fromarray(img, mode="RGB").save(tmp, format="PNG")
        os.replace(tmp, dest)
        return kind
    except Exception:
        try:
            tmp = dest.with_suffix(".tmp.png")
            if tmp.exists():
                tmp.unlink()
        except OSError:
            pass
        return ""
