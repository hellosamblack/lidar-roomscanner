# Flat-field study — 2026-08-04

This directory is the preserved archive for the nine stationary flat-field
captures taken against the blank grey ceiling at 55 inches. It intentionally
lives outside `captures/`, alongside the derived calibration maps, diagnostics,
MCP summary, and reproducible analysis tool.

This archive now holds **two** studies:

1. The original **stationary** ceiling study — start with
   [analysis_20260804_report.md](analysis_20260804_report.md). Short version:
   Precision and Ambient require different flat-field maps; the maps are
   engineering candidates and are **not enabled globally**.
2. The **panned** flat-field study (DC-D2 candidate set) — see
   [ffpan_20260804_report.md](ffpan_20260804_report.md). Six slow multi-height
   pans: 5 clean candidates, 1 contaminated (`ambientRegular8or16` — recapture).
   Confirms mode-family-specificity (milder than the static study) and shows a
   single-height map does not transfer across distance — a full multi-height pan
   is required.
3. The **cross-room (held-out-scene) validation** — see
   [crossroom_20260804_report.md](crossroom_20260804_report.md). Two pans in a
   larger second room (`{precision,ambient}Regular8msFFpanLarge.bin`) prove
   scene-independence: a room-A map flattens the unseen room to ~2.4% (Ambient) /
   ~3.0% (Precision), symmetric, after rejecting wall frames at the turnarounds.
   Honest cross-scene floor ~2.4–3.0%, not the ~1.3% self-map. Scene-independence
   gate CLOSED; remaining work is mode-aware map selection.

`ffpan_20260804_*` files (report, `metrics.json`, per-capture `.npz`/`.png`, and
`analyze_20260804_ffpan.py`) are the panned study; `analysis_20260804_*` and the
per-mode `flatfield_*`/`maps_*` files are the stationary study.

Contents:

- `*.bin` — original raw captures; preserved unchanged from the former
  `captures/2026-08-04/` directory.
- `flatfield_*.npz` — per-capture reflectance gain maps.
- `maps_*.npz` — compact depth/reflectance diagnostic maps.
- `analysis_20260804_report.md`, `analysis_20260804_metrics.{json,csv}` —
  findings and numeric results.
- `analysis_20260804_flatfield_maps.png` — visual map summary.
- `mcp_20260804_summary.json` — MCP integrity/rate/motion/skew summary.
- `analyze_20260804_flat_field.py` — reproducible analysis script.
- `SHA256SUMS.txt` — preservation checksums for every archived file except
  this checksum manifest and README.

From the repository root, rerun the analysis with:

```sh
MPLCONFIGDIR=/tmp/mpl PYTHONPATH=host/src:host \
  host/.venv/bin/python calibration/flatfield/2026-08-04/analyze_20260804_flat_field.py
```
