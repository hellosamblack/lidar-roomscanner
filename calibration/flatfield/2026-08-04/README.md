# Flat-field study — 2026-08-04

This directory is the preserved archive for the nine stationary flat-field
captures taken against the blank grey ceiling at 55 inches. It intentionally
lives outside `captures/`, alongside the derived calibration maps, diagnostics,
MCP summary, and reproducible analysis tool.

Start with [analysis_20260804_report.md](analysis_20260804_report.md). The
short version is that Precision and Ambient require different flat-field maps;
the maps here are engineering candidates and are **not enabled globally**.
The official production calibration still needs a slow pan over a matte wall
and a held-out validation distance.

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
