"""Full-capture equivalence sweep: PC transform vs MCU output over ALL pairs
in the 65 s hardware capture (not just the 3-pair committed fixture).

One-off script, NOT a pytest test: it needs captures/golden_pairs.bin, a
17.5 MB gitignored file that CI and fresh clones won't have. Run manually:

    host/.venv/Scripts/python host/tests/sweep_golden_capture.py

Processes pairs in capture order (seq ascending, starting at the stream's
true frame 1) since the on-device TNR (temporal noise reduction) filter is
stateful and must be fed frames in the same order the MCU saw them.

Reports: pair count, exact-match count/%, max abs diff overall, a max-abs-diff
per-frame distribution (p50/p90/p99/max), and any pair exceeding the plan's
0.01 mm tolerance.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.native import Transform  # noqa: E402
from roomscan.protocol import StreamId  # noqa: E402

CAPTURE = Path(__file__).parent.parent.parent / "captures" / "golden_pairs.bin"
ATOL_MM = 0.01


def load_full_capture(path: Path) -> tuple[bytes, list[tuple[int, bytes, bytes]]]:
    """Returns (calib, [(seq, raw, depth_mcu), ...]) sorted by seq (capture order)."""
    frames = StreamDecoder().feed(path.read_bytes())
    calib = next(f.payload for f in frames if f.header.stream_id == StreamId.CALIB)
    raws = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.RAW_3DMD}
    depths = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.DEPTH_ZF32}
    seqs = sorted(raws.keys() & depths.keys())
    return calib, [(s, raws[s], depths[s]) for s in seqs]


def main() -> int:
    if not Transform.available():
        print("SKIP: native transform DLL not built — see roomscan.native._BUILD_HINT")
        return 1
    if not CAPTURE.is_file():
        print(f"SKIP: full capture not found at {CAPTURE}")
        return 1

    calib, triples = load_full_capture(CAPTURE)
    print(f"loaded {len(triples)} (raw, depth) pairs from {CAPTURE} "
          f"(seq {triples[0][0]}..{triples[-1][0]})")

    t = Transform(calib)
    exact = 0
    max_diffs: list[float] = []
    violations: list[tuple[int, float]] = []
    overall_max = 0.0

    for seq, raw, depth_mcu in triples:
        depth_pc = t.process(raw)
        mcu = np.frombuffer(depth_mcu, dtype="<f4").reshape(42, 54)
        if np.array_equal(depth_pc, mcu):
            exact += 1
        diff = np.abs(depth_pc - mcu)
        # nanmax over an all-NaN slice would warn/error; guard it.
        finite = diff[~np.isnan(diff)]
        frame_max = float(finite.max()) if finite.size else 0.0
        max_diffs.append(frame_max)
        overall_max = max(overall_max, frame_max)
        if frame_max > ATOL_MM:
            violations.append((seq, frame_max))

    max_diffs_arr = np.array(max_diffs)
    n = len(triples)
    p50 = float(np.percentile(max_diffs_arr, 50))
    p90 = float(np.percentile(max_diffs_arr, 90))
    p99 = float(np.percentile(max_diffs_arr, 99))

    print()
    print("=== full-capture equivalence sweep ===")
    print(f"pairs processed:        {n}")
    print(f"exact-match pairs:      {exact}/{n} ({100.0 * exact / n:.2f}%)")
    print(f"max abs diff overall:   {overall_max:.6f} mm")
    print("per-frame max-abs-diff distribution (mm):")
    print(f"  p50: {p50:.6f}")
    print(f"  p90: {p90:.6f}")
    print(f"  p99: {p99:.6f}")
    print(f"  max: {max(max_diffs):.6f}")
    print(f"tolerance (atol):       {ATOL_MM} mm")
    print(f"violations (> atol):    {len(violations)}")
    if violations:
        print("  first 20 violations (seq, max abs diff mm):")
        for seq, d in violations[:20]:
            print(f"    seq={seq}: {d:.6f}")
        print(f"VERDICT: FAIL — {len(violations)} pair(s) exceed {ATOL_MM} mm tolerance")
        return 1
    print(f"VERDICT: PASS — all {n} pairs within {ATOL_MM} mm tolerance")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
