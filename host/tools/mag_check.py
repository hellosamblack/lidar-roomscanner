"""Check a magnetometer calibration against a RECORDED capture.

Created 2026-07-30 to validate the BUG-030 re-fit. The calibration modal scores
a tumble while you collect it; nothing scored the result afterwards, against
data the fit never saw -- which is the only evidence that actually generalises.

    host/.venv/bin/python host/tools/mag_check.py captures/roomSweepFull20260730.bin
    host/.venv/bin/python host/tools/mag_check.py <capture> --cal /path/to/candidate.json
    host/.venv/bin/python host/tools/mag_check.py <capture> --compare mag_cal.json old.json
    host/.venv/bin/python host/tools/mag_check.py <capture> --json

What to read, in order:

  1. `attitude` -- THE verdict on a moving capture. |B| detrended of the slow
     spatial drift, then the part that is a function of body-frame attitude.
     That is the calibration's own error; see `magsweep.attitude_locked_error`.
     It is a LOWER bound (the detrend absorbs error from any attitude held
     longer than the window), so it is only trustworthy next to (2).
  2. `tilt` -- |B| binned by boresight tilt. BUG-030's signature was a
     monotonic ramp across this table (40 -> 110 uT); a good fit is flat.
     Detrend-free, so this is what keeps (1) honest -- read them together.
  3. `field` -- `magsweep.field_consistency`. Correct for a stationary tumble,
     but on a walk its `bias_pct` absorbs the room's field level and it will
     under-rate a good calibration. Kept because it is the tumble-time metric
     and the numbers must be comparable.

Both a heading check and a tumble-coverage check are deliberately absent:
  * heading needs ground truth this capture does not carry. |B| flatness proves
    magnitude, not direction -- an ellipsoid fit is ambiguous up to a rotation
    (DT0103). The test for that is a braced, fixed-heading tilt sweep.
  * coverage here is of the CAPTURE's attitudes, not the tumble's, so a low
    number means "this scan did not visit much of the sphere", not "the
    calibration is under-determined". Reported, not verdicted.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.decoder import StreamDecoder                        # noqa: E402
from roomscan.magcal import MagCalibration                        # noqa: E402
from roomscan.magsweep import (assign_cells, attitude_locked_error,  # noqa: E402
                               calibrated_directions, calibrated_norms,
                               coverage_stats, field_consistency)
from roomscan.protocol import (FrameType, StreamId, decode_env,   # noqa: E402
                               decode_imu_quat)
from roomscan.sensors import quat_to_matrix, tilt_from_down_deg   # noqa: E402

# Boresight tilt away from straight-up, in degrees: 0 = pointing at the ceiling,
# 90 = horizontal (the wall-scanning attitude BUG-030 was worst in), 180 = floor.
TILT_EDGES = (0, 15, 30, 45, 60, 75, 90, 105, 120, 150, 180)
MIN_BIN = 5


def default_cal_path() -> Path:
    """The calibration a `roomscan-web` on this box would actually load.

    `ViewerConfig.mag_cal_path` is relative, so it resolves against the server's
    cwd -- the repo root in practice. A stale copy under host/ used to shadow
    this and silently apply a superseded fit to anything run from there
    (BUG-030); prefer the root file, fall back to cwd.
    """
    root = REPO / "mag_cal.json"
    return root if root.exists() else Path("mag_cal.json")


def read_mag_frames(path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Capture -> (mag uT (N,3), t_s (N,), paired quats (N,4)).

    Each ENV sample is paired with the nearest-in-time IMU_QUAT. Quats are all
    identity when the capture carries no stream 9, which makes every tilt land
    in one bin -- `check_capture` reports that rather than pretending.
    """
    dec = StreamDecoder()
    mags: list[tuple[float, float, float]] = []
    mag_t: list[int] = []
    quats: list[tuple[float, float, float, float]] = []
    quat_t: list[int] = []
    with open(path, "rb") as f:
        while chunk := f.read(1 << 20):
            for fr in dec.feed(chunk):
                if fr.header.frame_type != FrameType.DATA:
                    continue
                if fr.header.stream_id == StreamId.ENV:
                    _p, m, _t = decode_env(fr.payload)
                    mags.append(m)
                    mag_t.append(fr.header.t_us)
                elif fr.header.stream_id == StreamId.IMU_QUAT:
                    quats.append(decode_imu_quat(fr.payload))
                    quat_t.append(fr.header.t_us)
    m = np.asarray(mags, dtype=np.float64).reshape(-1, 3)
    mt = np.asarray(mag_t, dtype=np.float64)
    t_s = (mt - mt[0]) / 1e6 if mt.size else mt
    if quats:
        qt = np.asarray(quat_t, dtype=np.float64)
        q = np.asarray(quats, dtype=np.float64)[
            np.searchsorted(qt, mt).clip(0, len(qt) - 1)] if mt.size else np.zeros((0, 4))
    else:
        q = np.tile(np.array([1.0, 0.0, 0.0, 0.0]), (m.shape[0], 1))
    return m, t_s, q


def boresight_tilt_deg(quats) -> np.ndarray:
    """(N,4) quats -> boresight angle from straight up, 0..180 deg."""
    q = np.asarray(quats, dtype=np.float64).reshape(-1, 4)
    out = np.empty(q.shape[0])
    down = np.array([0.0, 0.0, -1.0])
    for i, qi in enumerate(q):
        out[i] = 90.0 - tilt_from_down_deg(tuple(quat_to_matrix(*qi).T @ down))
    return out


# A ramp of |B| across tilt is BUG-030's signature and is DETREND-FREE, so it
# catches what `attitude_locked_error` structurally cannot: an error in an
# attitude family the operator held for longer than the detrend window gets
# absorbed into the trend and scores clean. Measured references: the 2026-07-30
# fit ramps 1.04x across this table, the superseded 2026-07-15 fit 2.72x.
# Some spread is the room, not the fit -- hence 1.10, not 1.0.
TILT_RAMP_GOOD = 1.10
TILT_RAMP_MARGINAL = 1.25


def tilt_table(norms, tilts) -> list[dict]:
    """|B| binned by boresight tilt -- BUG-030's signature test."""
    rows = []
    for lo, hi in zip(TILT_EDGES, TILT_EDGES[1:]):
        sel = (tilts >= lo) & (tilts < hi)
        n = int(sel.sum())
        if n < MIN_BIN:
            continue
        b = norms[sel]
        rows.append({"tilt_lo_deg": lo, "tilt_hi_deg": hi, "n": n,
                     "mean_ut": float(b.mean()), "std_ut": float(b.std()),
                     "std_pct": float(100.0 * b.std() / b.mean()),
                     "min_ut": float(b.min()), "max_ut": float(b.max())})
    return rows


def tilt_ramp(rows) -> dict:
    """Spread of the tilt table's bin means -- the detrend-free half of the verdict."""
    means = [r["mean_ut"] for r in rows]
    if len(means) < 3:
        return {"ratio": None, "verdict": "unknown",
                "reason": f"only {len(means)} tilt bins hold >={MIN_BIN} samples"}
    lo, hi = min(means), max(means)
    ratio = (hi / lo) if lo > 1e-9 else float("inf")
    verdict = ("good" if ratio < TILT_RAMP_GOOD else
               "marginal" if ratio < TILT_RAMP_MARGINAL else "bad")
    return {"ratio": ratio, "min_ut": lo, "max_ut": hi, "bins": len(means),
            "verdict": verdict}


def _combine(*verdicts: str) -> str:
    """Worst-wins, with `unknown` blocking a clean pass.

    Same principle as `field_consistency`'s spread/bias pairing: two independent
    ways to be wrong, neither allowed to hide behind the other. A MEASURED
    failure still outranks an unmeasured component -- "bad" beats "unknown".
    """
    for v in ("bad", "marginal", "unknown"):
        if v in verdicts:
            return v
    return "good"


def check_capture(path, cal_path=None, window_s: float = 5.0) -> dict:
    """Score `cal_path` against the capture at `path`. See the module docstring."""
    p = Path(path)
    if not p.exists():
        return {"error": f"no such capture: {p}", "path": str(p)}
    cal_p = Path(cal_path) if cal_path else default_cal_path()
    cal = MagCalibration.load(cal_p)
    if cal is None:
        return {"error": f"no readable calibration at {cal_p}", "path": str(p),
                "cal_path": str(cal_p)}

    mags, t_s, quats = read_mag_frames(p)
    if mags.shape[0] == 0:
        return {"error": "capture carries no stream 10 (ENV) samples -- no magnetometer data",
                "path": str(p), "cal_path": str(cal_p)}
    has_quat = bool(np.any(np.abs(quats[:, 1:]) > 0))
    norms = calibrated_norms(mags, cal)
    tilts = boresight_tilt_deg(quats)
    cov = coverage_stats(assign_cells(calibrated_directions(mags, cal)))
    att = attitude_locked_error(mags, t_s, cal, window_s=window_s)
    rows = tilt_table(norms, tilts) if has_quat else []
    ramp = tilt_ramp(rows)
    raw = np.linalg.norm(mags, axis=1)
    dur = float(t_s[-1] - t_s[0]) if t_s.size > 1 else 0.0
    return {
        "path": str(p),
        "cal_path": str(cal_p),
        "cal": {"offset_ut": [float(v) for v in cal.offset],
                "hard_iron_ut": float(np.linalg.norm(cal.offset)),
                "field_ut": float(cal.field_ut),
                "axis_gain_ratio": float(_axis_gain(cal))},
        "samples": int(mags.shape[0]),
        "duration_s": dur,
        "rate_hz": (mags.shape[0] / dur) if dur > 0 else 0.0,
        "raw_norm_ut": {"min": float(raw.min()), "max": float(raw.max())},
        "has_orientation": has_quat,
        "field": field_consistency(mags, cal),
        "attitude": att,
        "tilt": rows,
        "tilt_ramp": ramp,
        "coverage": {"occupied": cov["occupied"], "cells": cov["cells"],
                     "fraction": cov["fraction"]},
        # Worst of the detrended attitude error and the detrend-free tilt ramp;
        # see the module docstring for why `field.verdict` is neither.
        "verdict": _combine((att or {}).get("verdict", "unknown"), ramp["verdict"]),
    }


def _axis_gain(cal: MagCalibration) -> float:
    ev = np.linalg.eigvalsh(np.asarray(cal.matrix, dtype=np.float64))
    return float(ev.max() / ev.min()) if ev.min() > 1e-9 else float("inf")


def format_report(r: dict) -> str:
    if "error" in r:
        return f"ERROR: {r['error']}"
    L = [f"{r['path']}  ({r['samples']} mag samples, {r['duration_s']:.1f} s, "
         f"{r['rate_hz']:.1f} Hz)  raw |B| {r['raw_norm_ut']['min']:.1f}..{r['raw_norm_ut']['max']:.1f} uT",
         f"calibration {r['cal_path']}  hard-iron {r['cal']['hard_iron_ut']:.2f} uT  "
         f"field_ut {r['cal']['field_ut']:.2f}  axis-gain {r['cal']['axis_gain_ratio']:.3f}"]
    a = r.get("attitude")
    if a and a.get("attitude_locked_ut") is not None:
        L.append(f"ATTITUDE-LOCKED ERROR  {a['attitude_locked_ut']:.2f} uT "
                 f"({a['attitude_locked_pct']:.2f}% of |B|)  -> {a['verdict'].upper()}"
                 f"   [{a['cells_used']} cells, {a['window_s']:.0f}s detrend]")
        L.append(f"  |B| std {a['total_std_ut']:.2f} uT = spatial/room {a['spatial_std_ut']:.2f} "
                 f"+ residual {a['residual_std_ut']:.2f}")
    elif a:
        L.append(f"ATTITUDE-LOCKED ERROR  not measurable: {a.get('reason')}")
    f = r.get("field")
    if f:
        L.append(f"field_consistency (tumble-time metric): std {f['std_pct']:.2f}%  "
                 f"bias {f['bias_pct']:+.2f}%  ratio {f['ratio']:.2f}  -> {f['verdict']}")
    if r["tilt"]:
        ramp = r["tilt_ramp"]
        L.append(f"TILT RAMP  {ramp['ratio']:.3f}x across {ramp['bins']} bins "
                 f"-> {ramp['verdict'].upper()}   (detrend-free cross-check)")
        L.append("|B| vs boresight tilt from vertical (0=ceiling, 90=horizontal):")
        for row in r["tilt"]:
            L.append(f"  {row['tilt_lo_deg']:3d}-{row['tilt_hi_deg']:3d} deg  n={row['n']:6d}  "
                     f"{row['mean_ut']:6.2f} +-{row['std_ut']:5.2f} uT ({row['std_pct']:4.1f}%)")
    elif not r["has_orientation"]:
        L.append("no stream 9 in this capture -- tilt table unavailable")
    c = r["coverage"]
    L.append(f"attitudes visited by this capture: {c['occupied']}/{c['cells']} cells "
             f"({100 * c['fraction']:.0f}%)  [not a tumble-coverage verdict]")
    return "\n".join(L)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--cal", default=None,
                    help="calibration JSON (default: the one roomscan-web would load)")
    ap.add_argument("--compare", nargs="+", default=None,
                    help="score several calibrations against the same capture")
    ap.add_argument("--window", type=float, default=5.0,
                    help="rolling-median detrend window, seconds (default 5)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    cals = args.compare if args.compare else [args.cal]
    reports = [check_capture(args.capture, c, window_s=args.window) for c in cals]
    if args.json:
        print(json.dumps(reports if args.compare else reports[0], indent=2))
    else:
        print("\n\n".join(format_report(r) for r in reports))
    return 1 if any("error" in r for r in reports) else 0


if __name__ == "__main__":
    raise SystemExit(main())
