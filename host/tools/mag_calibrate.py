"""Interactive magnetometer calibration: collect ENV-stream mag samples while
rotating the rig through all orientations, fit hard/soft-iron correction, save.

Usage:  cd host && python -m tools.mag_calibrate --seconds 30 --out mag_cal.json
Rotate the rig slowly through as many orientations as possible during the window."""
from __future__ import annotations

import argparse
import time
from pathlib import Path
from typing import Iterable

import numpy as np

from roomscan.decoder import StreamDecoder
from roomscan.magcal import MagCalibration, fit_ellipsoid
from roomscan.protocol import Frame, FrameType, StreamId, decode_env


def collect_mag_from_frames(frames: Iterable[Frame]) -> np.ndarray:
    out = []
    for fr in frames:
        if fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.ENV:
            _, mag, _ = decode_env(fr.payload)
            out.append(mag)
    return np.asarray(out, dtype=np.float64).reshape(-1, 3)


def calibrate(samples: np.ndarray, out_path) -> MagCalibration:
    cal = fit_ellipsoid(np.asarray(samples, dtype=np.float64))
    cal.save(out_path)
    return cal


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Magnetometer hard/soft-iron calibration")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--out", default="mag_cal.json")
    ap.add_argument("--port", default=None)
    args = ap.parse_args(argv)

    from roomscan.sources import SerialSource  # deferred: no pyserial in tests
    src = SerialSource(port=args.port)
    dec = StreamDecoder()
    print(f"Rotate the rig through ALL orientations for {args.seconds:.0f} s...")
    samples: list[tuple[float, float, float]] = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < args.seconds:
        for fr in dec.feed(src.read()):
            if fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.ENV:
                _, mag, _ = decode_env(fr.payload)
                samples.append(mag)
    src.close()
    arr = np.asarray(samples, dtype=np.float64).reshape(-1, 3)
    print(f"collected {arr.shape[0]} mag samples")
    cal = calibrate(arr, args.out)
    norms = np.linalg.norm(np.array([cal.apply(r) for r in arr]), axis=1)
    print(f"field_ut={cal.field_ut:.2f}  residual(std/mean)={np.std(norms)/np.mean(norms):.4f}")
    print(f"saved -> {Path(args.out).resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
