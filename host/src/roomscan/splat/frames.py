"""Extract downscaled still frames from a video for COLMAP.

Uses the ffmpeg binary bundled by ``imageio-ffmpeg`` (no system ffmpeg / root
required).  Frames are sampled at a fixed rate, downscaled so the long edge is
``long_edge`` px, and evenly thinned to ``max_frames`` -- COLMAP matching cost
grows fast with image count, and a room walkthrough at 3 fps already gives the
overlap SfM needs.
"""
from __future__ import annotations

import subprocess
from pathlib import Path


def _ffmpeg_exe() -> str:
    import imageio_ffmpeg
    return imageio_ffmpeg.get_ffmpeg_exe()


def extract_frames(video: str | Path, out_dir: str | Path, *, fps: float = 3.0,
                   long_edge: int = 1600, max_frames: int = 300,
                   log=lambda m: None) -> list[Path]:
    """Write ``frame_00001.jpg`` … to ``out_dir``; return the kept frame paths.

    Two-step so the count is bounded deterministically: ffmpeg samples at ``fps``
    and downscales, then we evenly drop frames until at most ``max_frames``
    remain (keeping the first and last).  Raises ``RuntimeError`` if ffmpeg
    produced nothing (unreadable video / bad codec).
    """
    video = Path(video)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    # -2 keeps the short edge even (required by the jpeg/h264 encoders) while
    # scaling the long edge to `long_edge`; the if/gt picks which edge is long.
    vf = (f"fps={fps},"
          f"scale='if(gt(iw,ih),{long_edge},-2)':'if(gt(iw,ih),-2,{long_edge})'")
    cmd = [_ffmpeg_exe(), "-nostdin", "-hide_banner", "-loglevel", "error",
           "-i", str(video), "-vf", vf, "-q:v", "2",
           str(out_dir / "frame_%05d.jpg")]
    log(f"[frames] ffmpeg fps={fps} long_edge={long_edge}")
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed ({proc.returncode}): {proc.stderr.strip()[:500]}")

    frames = sorted(out_dir.glob("frame_*.jpg"))
    if not frames:
        raise RuntimeError("ffmpeg produced no frames")
    if len(frames) > max_frames:
        # Even stride keeping endpoints: indices round(i*(n-1)/(k-1)).
        n, k = len(frames), max_frames
        keep_idx = sorted({round(i * (n - 1) / (k - 1)) for i in range(k)})
        keep = {frames[i] for i in keep_idx}
        for f in frames:
            if f not in keep:
                f.unlink()
        frames = sorted(keep)
    log(f"[frames] kept {len(frames)} frames")
    return frames
