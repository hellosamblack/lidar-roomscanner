"""Build a per-zone reflectance flat-field (FPN) correction from a capture.

The capture MUST be a flat-field recording: aim the sensor at a uniform matte
surface (a blank wall is fine) and slowly PAN/sweep it across the surface while
recording. Panning is essential -- it averages real surface texture across zones
so only the sensor's fixed per-zone response (FPN) remains. A static capture is
NOT valid input (it bakes scene texture into the "correction").

Record one with the native-CDC capturer, then build:

    cd host
    python -m tools.capture --seconds 20 --out flatfield_pan.bin   # pan while this runs
    python -m tools.build_flatfield flatfield_pan.bin --out flatfield.npz

Then point roomscan.toml at it to enable correction everywhere:

    [viewer]
    flatfield_path = "flatfield.npz"

See docs/flatfield-calibration.md and roomscan.flatfield.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from roomscan.decoder import StreamDecoder
from roomscan.flatfield import build_flatfield
from roomscan.pipeline import TransformStage
from roomscan.protocol import StreamId


def reflectance_frames_from_capture(path) -> np.ndarray:
    """Decode a .bin recording -> (N, H, W) reflectance stack via the transform
    (no flat-field applied -- we're measuring the raw FPN)."""
    stage = TransformStage(outputs=("reflectance",))
    dec = StreamDecoder()
    frames = []
    with open(path, "rb") as f:
        data = f.read()
    for fr in dec.feed(data):
        result = stage.feed(fr)
        if result is not None and fr.header.stream_id == StreamId.RAW_3DMD:
            frames.append(result[1]["reflectance"])
    if not frames:
        raise SystemExit(
            f"{path}: no RAW_3DMD frames decoded (need a raw-stream capture with CALIB)")
    return np.asarray(frames, dtype=np.float64)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Build a reflectance flat-field (FPN) correction")
    ap.add_argument("capture", help="path to a flat-field .bin recording (panned uniform surface)")
    ap.add_argument("--out", default="flatfield.npz", help="output .npz map path")
    ap.add_argument("--sigma", type=float, default=2.5,
                    help="illumination smoothing sigma in zones (default 2.5)")
    ap.add_argument("--note", default="", help="free-text provenance note stored in the map")
    args = ap.parse_args(argv)

    frames = reflectance_frames_from_capture(args.capture)
    ff = build_flatfield(frames, smooth_sigma=args.sigma,
                         note=args.note or f"built from {Path(args.capture).name}")
    ff.save(args.out)

    g = ff.gain
    print(f"decoded {frames.shape[0]} reflectance frames of shape {frames.shape[1:]}")
    print(f"measured per-zone FPN residual: {ff.meta['residual_pct']:.1f}% of signal")
    print(f"gain map: mean={g.mean():.3f} min={g.min():.3f} max={g.max():.3f} "
          f"std={g.std()*100:.1f}%")
    if frames.shape[0] < 30:
        print("WARNING: few frames -- make sure you PANNED across a uniform surface; "
              "a static capture bakes scene texture into the map and is invalid.")
    print(f"saved {args.out}  ->  set [viewer] flatfield_path = \"{args.out}\" to enable")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
