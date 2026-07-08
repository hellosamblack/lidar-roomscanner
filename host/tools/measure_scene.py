"""Physical scene measurement: validate deprojection against known object geometry.

Decodes a raw capture (RAW_3DMD + CALIB), runs the native transform to depth,
takes a mid-capture window of frames (TNR settled), segments the near field into
clusters, and reports each cluster's distance and vertical extent in millimetres
against user-supplied ground truth (e.g. "a 6-inch statue about 1 foot away").

    host/.venv/Scripts/python host/tools/measure_scene.py captures/crow_scene.bin \
        --expected-distance-mm 305 --expected-height-mm 152.4

Method
------
- Temporal median depth per zone over the analysis window (robust to flicker),
  plus per-frame numbers for spread.
- Near/far split: fixed --near-mm threshold (default 500) — the scenes this
  validates have a near object against a background metres away, so any value
  in the gap works.
- Clustering: 8-connected components on the near mask, then a column-overlap
  merge (components whose grid-column ranges overlap and whose median z agree
  within --merge-z-mm belong to one physical object — handles objects split by
  a weak-return band, e.g. a glossy statue's mid-section).
- Height: max(y) - min(y) over the cluster's deprojected points. y = z*tan(ay),
  positive DOWN (row-major top-to-bottom) per the Deprojector convention.
- Honest limits printed with the numbers: vertical zone pitch at the cluster's
  distance (quantization is +/- 1-2 zones), and FoV-edge clipping flags (a
  cluster touching the top/bottom row extends beyond the FoV: its extent is a
  lower bound and must not be scored as a height measurement).
"""
from __future__ import annotations

import argparse
import sys
import warnings
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.pipeline import TransformStage  # noqa: E402


def load_depths(capture: Path) -> list[np.ndarray]:
    dec = StreamDecoder()
    stage = TransformStage(outputs=("depth",))
    depths: list[np.ndarray] = []
    with open(capture, "rb") as f:
        for frame in dec.feed(f.read()):
            result = stage.feed(frame)
            if result is not None:
                depths.append(result[1]["depth"].copy())
    print(f"decoded {dec.frames_decoded} frames ({len(depths)} depth), "
          f"crc failures {dec.crc_failures}, skipped {dec.bytes_skipped} B")
    return depths


def connected_components(mask: np.ndarray) -> list[list[tuple[int, int]]]:
    """8-connected components of a small boolean grid (pure-python BFS)."""
    h, w = mask.shape
    seen = np.zeros_like(mask, dtype=bool)
    comps = []
    for r0 in range(h):
        for c0 in range(w):
            if not mask[r0, c0] or seen[r0, c0]:
                continue
            queue = [(r0, c0)]
            seen[r0, c0] = True
            comp = []
            while queue:
                r, c = queue.pop()
                comp.append((r, c))
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < h and 0 <= cc < w and mask[rr, cc] and not seen[rr, cc]:
                            seen[rr, cc] = True
                            queue.append((rr, cc))
            comps.append(comp)
    return comps


def merge_by_column_overlap(comps: list[list[tuple[int, int]]], zmed: np.ndarray,
                            merge_z_mm: float) -> list[list[tuple[int, int]]]:
    """Merge components whose grid-column ranges overlap and whose median z agree
    within merge_z_mm — one physical object split by a weak-return band."""
    def stats(comp):
        rows, cols = zip(*comp)
        zs = np.array([zmed[r, c] for r, c in comp])
        return min(cols), max(cols), float(np.median(zs))

    merged = [list(c) for c in comps]
    changed = True
    while changed:
        changed = False
        for i in range(len(merged)):
            for j in range(i + 1, len(merged)):
                c0lo, c0hi, z0 = stats(merged[i])
                c1lo, c1hi, z1 = stats(merged[j])
                if c0lo <= c1hi and c1lo <= c0hi and abs(z0 - z1) <= merge_z_mm:
                    merged[i].extend(merged[j])
                    del merged[j]
                    changed = True
                    break
            if changed:
                break
    return merged


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture", type=Path, help="raw capture file (RAW_3DMD + CALIB stream)")
    ap.add_argument("--expected-distance-mm", type=float, default=305.0)
    ap.add_argument("--expected-height-mm", type=float, default=152.4)
    ap.add_argument("--frames", type=int, default=20, help="analysis window size (mid-capture)")
    ap.add_argument("--near-mm", type=float, default=500.0, help="near/far split threshold")
    ap.add_argument("--merge-z-mm", type=float, default=50.0,
                    help="max median-z difference for column-overlap cluster merge")
    ap.add_argument("--min-zones", type=int, default=6, help="drop clusters smaller than this")
    ap.add_argument("--split-col", type=int, default=None,
                    help="manual x-separation: treat grid columns >= N as a separate object "
                         "(for scenes where two adjacent objects touch at 8-connectivity)")
    ap.add_argument("--fov-h", type=float, default=55.0)
    ap.add_argument("--fov-v", type=float, default=42.0)
    ap.add_argument("--tolerance", type=float, default=0.15, help="pass/fail band (fraction)")
    args = ap.parse_args(argv)

    depths = load_depths(args.capture)
    if len(depths) < args.frames:
        print(f"error: only {len(depths)} depth frames, need {args.frames}", file=sys.stderr)
        return 2
    lo = max(0, (len(depths) - args.frames) // 2)
    window = np.stack(depths[lo:lo + args.frames])
    print(f"analysis window: frames {lo}..{lo + args.frames - 1} of {len(depths)}")

    h, w = window.shape[1:]
    valid = np.isfinite(window) & (window > 0) & (window < 10000)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # all-NaN zones are expected
        zmed = np.nanmedian(np.where(valid, window, np.nan), axis=0)  # temporal median per zone

    # Zone-center angle tables (Deprojector convention: y positive DOWN).
    tan_x = np.tan(np.deg2rad(((np.arange(w) + 0.5) / w - 0.5) * args.fov_h))[None, :]
    tan_y = np.tan(np.deg2rad(((np.arange(h) + 0.5) / h - 0.5) * args.fov_v))[:, None]

    near = np.isfinite(zmed) & (zmed < args.near_mm)
    n_near = int(near.sum())
    all_z = zmed[near]
    print(f"\nnear field (< {args.near_mm:.0f} mm): {n_near} zones, "
          f"combined median z = {np.median(all_z):.0f} mm "
          f"(p10 {np.percentile(all_z, 10):.0f}, p90 {np.percentile(all_z, 90):.0f})")

    if args.split_col is not None:
        left, right = near.copy(), near.copy()
        left[:, args.split_col:] = False
        right[:, :args.split_col] = False
        comps = ([c for c in connected_components(left) if len(c) >= args.min_zones]
                 + [c for c in connected_components(right) if len(c) >= args.min_zones])
        # merge within each side only (column ranges won't overlap across the split)
    else:
        comps = [c for c in connected_components(near) if len(c) >= args.min_zones]
    clusters = merge_by_column_overlap(comps, zmed, args.merge_z_mm)
    clusters.sort(key=len, reverse=True)

    # Per-frame spread of the combined near-cluster median (noise estimate).
    per_frame_med = []
    for k in range(window.shape[0]):
        zk = window[k][near]  # this frame's values under the temporal-median near mask
        zk = zk[np.isfinite(zk) & (zk > 0) & (zk < args.near_mm + 100)]
        if zk.size:
            per_frame_med.append(np.median(zk))
    print(f"per-frame combined median z over the window: "
          f"mean {np.mean(per_frame_med):.1f} mm, std {np.std(per_frame_med):.1f} mm")

    print(f"\n{len(clusters)} near cluster(s) after merge (>= {args.min_zones} zones):")
    scored = []
    for i, comp in enumerate(clusters):
        rows = np.array([r for r, _ in comp])
        cols = np.array([c for _, c in comp])
        zs = zmed[rows, cols]
        ys = zs * tan_y[rows, 0]
        xs = zs * tan_x[0, cols]
        z_c = float(np.median(zs))
        height = float(ys.max() - ys.min())
        width_mm = float(xs.max() - xs.min())
        pitch = z_c * np.tan(np.deg2rad(args.fov_v / h))
        clip_top = rows.min() == 0
        clip_bot = rows.max() == h - 1
        clip_v = clip_top or clip_bot
        clip_h = cols.min() == 0 or cols.max() == w - 1
        clip_note = "+".join(s for s, f in (("top", clip_top), ("bottom", clip_bot)) if f)
        print(f"  [{i}] {len(comp):3d} zones | rows {rows.min():2d}..{rows.max():2d} "
              f"cols {cols.min():2d}..{cols.max():2d} | median z {z_c:6.1f} mm | "
              f"y-extent {height:6.1f} mm | x-extent {width_mm:5.1f} mm | "
              f"zone pitch {pitch:.1f} mm/zone"
              + (f" | CLIPPED {clip_note} (extent = lower bound)" if clip_v else "")
              + (" | clipped left/right" if clip_h else ""))
        scored.append((i, z_c, height, pitch, clip_v))

    # Verdict: score the unclipped cluster whose (z, height) best matches expectations.
    candidates = [s for s in scored if not s[4]] or scored
    best = min(candidates, key=lambda s: abs(s[1] - args.expected_distance_mm) / args.expected_distance_mm
               + abs(s[2] - args.expected_height_mm) / args.expected_height_mm)
    i, z_c, height, pitch, clip_v = best

    # Row profile of the verdict cluster: zones-per-row + median z per row, so a
    # human can spot structure (weak-return bands, a foreground slab at the bottom,
    # FoV clipping) that a single extent number hides.
    comp = clusters[i]
    rows_arr = np.array([r for r, _ in comp])
    print(f"\nrow profile of cluster [{i}] (row: n zones, median z, y at median z):")
    for r in sorted(set(rows_arr.tolist())):
        cs = [c for rr, c in comp if rr == r]
        zs_r = zmed[r, cs]
        z_r = float(np.median(zs_r))
        print(f"    row {r:2d}: n={len(cs):2d}  z={z_r:6.1f} mm  y={z_r * tan_y[r, 0]:+7.1f} mm")

    tol = args.tolerance
    d_err = (z_c - args.expected_distance_mm) / args.expected_distance_mm
    h_err = (height - args.expected_height_mm) / args.expected_height_mm
    h_err_q = (height - 2 * pitch - args.expected_height_mm) / args.expected_height_mm  # quantization allowance
    print(f"\nverdict target: cluster [{i}]"
          + (" (WARNING: vertically clipped -- height is a lower bound)" if clip_v else ""))
    print(f"  distance: {z_c:.1f} mm vs {args.expected_distance_mm:.1f} mm expected "
          f"({d_err:+.1%})  ->  {'PASS' if abs(d_err) <= tol else 'FAIL'} (band +/-{tol:.0%})")
    print(f"  height:   {height:.1f} mm vs {args.expected_height_mm:.1f} mm expected "
          f"({h_err:+.1%}; {h_err_q:+.1%} after -2-zone quantization allowance of {2*pitch:.1f} mm)"
          f"  ->  {'PASS' if abs(h_err) <= tol or abs(h_err_q) <= tol else 'FAIL'} (band +/-{tol:.0%})")
    ok = abs(d_err) <= tol and (abs(h_err) <= tol or abs(h_err_q) <= tol) and not clip_v
    print(f"\nprojection {'ACCURATE within tolerance' if ok else 'NOT confirmed within tolerance'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
