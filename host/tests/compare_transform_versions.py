"""A/B two builds of vl53l9-transform-c over the same capture.

One-off script, NOT a pytest test: it needs two shared libraries built from
different vendor packages, and captures that CI won't have. The gate that runs
in pytest is test_equivalence.py (PC vs MCU); this is the gate for "did
swapping the vendor library change what we produce".

Build the two arms first (RS_TRANSFORM_PKG selects the source package, see
host/transform/CMakeLists.txt):

    cmake -S host/transform -B /tmp/x131 -DCMAKE_BUILD_TYPE=Release \
        -DRS_TRANSFORM_PKG=53L9A1 && cmake --build /tmp/x131
    cmake -S host/transform -B /tmp/x150 -DCMAKE_BUILD_TYPE=Release \
        -DRS_TRANSFORM_PKG=STSW-IMG053 && cmake --build /tmp/x150

    host/.venv/bin/python host/tests/compare_transform_versions.py \
        captures/coffeeRoomCircuitNoMnt.bin \
        /tmp/x131/libroomscan_transform.so /tmp/x150/libroomscan_transform.so

Each arm runs in its own subprocess: roomscan.native caches the CDLL at import
and both builds export identical symbols, so they cannot share a process.
Frames go through in capture order because TNR is stateful — and note that a
TNR-related difference shows up as frame 0 identical and frame 1 onward
divergent, since TNR passes the first frame through.

Reports, per output plane: what fraction of values are bit-identical, the
|Δ| distribution, and the tail above a set of absolute thresholds. Exits
nonzero if any plane differs, so it can gate an upgrade.
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

PLANES = ("depth", "reflectance", "confidence", "ambient")
THRESHOLDS = (1e-3, 1e-2, 0.1, 1.0, 10.0, 100.0)
DEFAULT_FRAMES = 300


def _dump(capture: Path, out: Path, limit: int) -> None:
    """Child-process arm: run `limit` frames through whichever DLL is selected."""
    from roomscan.decoder import StreamDecoder
    from roomscan.native import Transform
    from roomscan.protocol import StreamId

    frames = StreamDecoder().feed(capture.read_bytes())
    calib = next(f.payload for f in frames if f.header.stream_id == StreamId.CALIB)
    raws = [f.payload for f in frames if f.header.stream_id == StreamId.RAW_3DMD][:limit]
    if not raws:
        raise SystemExit(f"{capture}: no RAW_3DMD frames")

    t = Transform(calib, outputs=PLANES)
    acc: dict[str, list[np.ndarray]] = {p: [] for p in PLANES}
    for raw in raws:
        result = t.process(raw)
        for p in PLANES:
            acc[p].append(result[p])
    np.savez(out, **{p: np.stack(acc[p]) for p in PLANES})


def main() -> int:
    if os.environ.get("_RS_XFORM_AB_ARM"):
        _dump(Path(sys.argv[1]), Path(sys.argv[2]), int(sys.argv[3]))
        return 0

    if len(sys.argv) < 4:
        print(__doc__)
        return 2
    capture, lib_a, lib_b = (Path(a) for a in sys.argv[1:4])
    frames = int(sys.argv[4]) if len(sys.argv) > 4 else DEFAULT_FRAMES

    with tempfile.TemporaryDirectory() as td:
        arms = {}
        for tag, lib in (("A", lib_a), ("B", lib_b)):
            if not lib.is_file():
                print(f"missing library: {lib}")
                return 1
            out = Path(td) / f"{tag}.npz"
            subprocess.run(
                [sys.executable, __file__, str(capture), str(out), str(frames)],
                env=dict(os.environ, _RS_XFORM_AB_ARM="1", ROOMSCAN_TRANSFORM_DLL=str(lib)),
                check=True,
            )
            arms[tag] = np.load(out)

        print(f"{capture.name}: {frames} frames\n  A = {lib_a}\n  B = {lib_b}")
        differing = []
        for p in PLANES:
            a, b = arms["A"][p], arms["B"][p]
            # NaN is a legitimate "no return" value; NaN-vs-NaN is a match, but a
            # NaN appearing on only one side is a difference the |Δ| can't show.
            nan_only_one_side = int((np.isnan(a) ^ np.isnan(b)).sum())
            d = np.abs(a.astype(np.float64) - b.astype(np.float64))
            d[np.isnan(a) & np.isnan(b)] = 0.0
            d = d[np.isfinite(d)]
            p50, p90, p99 = np.percentile(d, [50, 90, 99]) if d.size else (0.0, 0.0, 0.0)
            print(f"  {p:12s} identical={100 * (d == 0).mean():6.2f}%  "
                  f"p50={p50:.4g} p90={p90:.4g} p99={p99:.4g} max={d.max() if d.size else 0.0:.4g}  "
                  f"nan_only_one_side={nan_only_one_side}")
            print("      tail: " + "  ".join(
                f">{t:g}: {100 * (d > t).mean():.3f}%" for t in THRESHOLDS))
            if d.size and (d.max() > 0 or nan_only_one_side):
                differing.append(p)

        if differing:
            per_frame = [float(np.nanmax(np.abs(arms["A"]["depth"][i] - arms["B"]["depth"][i])))
                         for i in range(arms["A"]["depth"].shape[0])]
            first = next((i for i, v in enumerate(per_frame) if v > 0), None)
            print(f"\nDIFFER: {', '.join(differing)}  (first depth-differing frame: {first})")
            return 1

    print("\nidentical on every plane")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
