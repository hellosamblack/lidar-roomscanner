"""Is the reported heading a HEADING, or is it wearing another axis's clothes?

    host/.venv/bin/python host/tools/heading_check.py captures/NorthFacingRoll.bin
    host/.venv/bin/python host/tools/heading_check.py <capture> --cal candidate.json --json

Written 2026-08-01 after BUG-058, which is the fourth time a scalar taken off
the attitude was used as a bearing (BUG-039/048/051/058) and the second time the
owner found it by eye. `capture_magcheck` scores the magnetometer's *magnitude*
and cannot see this class at all: on `NorthFacingRoll.bin` it returns
`attitude_locked 0.18%` -- an excellent calibration -- while the heading built on
top of it swung 154 deg for 0 deg of real turn.

## What it measures

Two independent estimates of where the sensor points exist in every stream-9
capture, and a correct heading is the one that agrees with the other:

  * `boresight_bearing_deg(quat)` -- from the orientation alone. Its zero is the
    SFLP world frame's arbitrary +X datum, which drifts.
  * `absolute_heading(quat, mag)` -- from the orientation AND the magnetometer,
    referenced to magnetic north.

They differ by that datum, so over a capture their difference must be a
CONSTANT. Regress heading on (bearing, roll) together -- not on either alone,
because on a normal sweep the operator's roll and bearing are correlated and a
single-variable fit hands the confounder's slope to whichever is regressed:

    bearing_coef  ->  1  (heading tracks where the device points)
    roll_coef     ->  0  (rolling the device in the hand is not a turn)

Proved against a known-bad input rather than only against a working one: fed
the pre-BUG-058 heading over `NorthFacingRoll.bin` it returns `bad`, roll
coefficient **-0.984, 95% CI [-1.036, -0.944]**; fed the fixed heading over the
same capture, `good` at **+0.016, [-0.036, +0.056]**.

Each axis is judged separately and only against what the capture can resolve --
a bootstrap interval that straddles the band reads `inconclusive`, not `good`
and not `bad`. Three of the four captures on hand can only certify ONE of the
two axes, which is the honest answer, not a defect in the tool.

## What it does NOT measure

**Absolute direction.** Both estimates share the quaternion and the calibration,
so a magnetometer whose fit is rotated (DT0103, still open) moves them together
and this tool sees nothing. It says the heading is a heading; it does not say it
points at north. That needs the braced fixed-bearing tilt sweep, ROADMAP DC-E.

That is not hypothetical: hours after this tool was written, BUG-059 turned out
to be a flat **180 deg** offset -- the field vector was anti-parallel -- and this
tool scores the capture IDENTICALLY before and after, because it regresses slopes
and a constant offset lives in the intercept it discards. The check that catches
THAT one is the dip: rotate the calibrated field into the world frame and its Z
must be negative. `check_capture` reports `inclination_deg` for exactly this
reason -- read it alongside the coefficients, not instead of them.

**Anything, on a capture that does not move.** Each axis reports its own
`range_deg` for that reason: a regression over 3 deg of roll cannot exonerate
the roll axis, and that axis says `inconclusive` rather than `good` when the
capture never exercised it.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))
sys.path.insert(0, str(REPO / "host"))       # `tools` is a top-level package rooted at host/

from roomscan.magcal import MagCalibration                        # noqa: E402
from roomscan.sensors import (AXIS_CONVENTION, absolute_heading,  # noqa: E402
                              boresight_bearing_deg, boresight_view_deg,
                              quat_to_matrix)
from tools.mag_check import default_cal_path, read_mag_frames     # noqa: E402

# Below this an axis simply was not exercised, so its coefficient is fitted on
# noise. Chosen from the captures on hand: a deliberate roll demo spans ~150 deg
# and a room sweep ~360, while an incidental wobble is a few degrees.
MIN_RANGE_DEG = 20.0

# BUG-058 scored a roll coefficient of -0.978 and BUG-051 a bearing coefficient
# well off 1; a clean capture scores within a few hundredths. 0.10 sits far from
# both, and the bootstrap interval below decides whether a given capture can
# actually resolve that much.
COEF_TOL = 0.10

# Moving-block bootstrap. The residual here is dominated by yaw drift and by the
# room's own field varying with position -- both drift over SECONDS, so plain
# resampling would treat autocorrelated wander as independent noise and report
# an interval ~10x too tight. That is not academic: it is the difference between
# `coffeeRoomCircuitNoMnt` reading "BAD, roll coefficient 0.181" and reading
# "inconclusive, this capture only rolled 36 deg and drifts 5.8 deg".
BOOTSTRAP_BLOCK_S = 3.0
BOOTSTRAP_N = 300


def _unwrap(a: np.ndarray) -> np.ndarray:
    return np.degrees(np.unwrap(np.radians(np.asarray(a, dtype=np.float64))))


def _fit(design: np.ndarray, h: np.ndarray) -> np.ndarray:
    coefs, *_ = np.linalg.lstsq(design, h, rcond=None)
    return coefs


def _block_bootstrap_ci(design, h, block: int, n_boot: int = BOOTSTRAP_N):
    """(lo, hi) 95% intervals for each coefficient, resampling whole blocks.

    Deterministic (`default_rng(0)`) so a report is reproducible and two runs of
    the same capture can be compared.
    """
    n = h.size
    block = max(1, min(block, n))
    starts = np.arange(n - block + 1)
    k = int(np.ceil(n / block))
    rng = np.random.default_rng(0)
    draws = np.empty((n_boot, design.shape[1]))
    for i in range(n_boot):
        picks = rng.choice(starts, size=k)
        idx = np.concatenate([np.arange(s, s + block) for s in picks])[:n]
        draws[i] = _fit(design[idx], h[idx])
    return (np.percentile(draws, 2.5, axis=0), np.percentile(draws, 97.5, axis=0))


def _axis_verdict(coef, lo, hi, target, range_deg, name, symptom):
    """Judge one coefficient against its target, given what the capture can resolve."""
    d = {"coef": float(coef), "ci95": [float(lo), float(hi)],
         "range_deg": float(range_deg), "target": float(target)}
    if range_deg < MIN_RANGE_DEG:
        return {**d, "verdict": "inconclusive",
                "reason": f"{name} only spans {range_deg:.1f} deg (<{MIN_RANGE_DEG:.0f}) -- "
                          f"this capture does not exercise that axis"}
    good_lo, good_hi = target - COEF_TOL, target + COEF_TOL
    if lo >= good_lo and hi <= good_hi:
        return {**d, "verdict": "good", "reason": None}
    if hi < good_lo or lo > good_hi:
        return {**d, "verdict": "bad",
                "reason": f"heading moves {coef:.3f} deg per deg of {name} "
                          f"(95% CI {lo:.3f}..{hi:.3f}, want {target:.1f}) -- {symptom}"}
    return {**d, "verdict": "inconclusive",
            "reason": f"{coef:.3f} with a 95% CI of {lo:.3f}..{hi:.3f} straddles the "
                      f"{target:.1f}+-{COEF_TOL:.2f} band -- this capture cannot resolve it"}


def score(bearing_deg, heading_deg, roll_deg, rate_hz: float = 30.0) -> dict:
    """The pure scorer: three same-length angle series -> per-axis coefficients,
    bootstrap intervals and verdicts.

    Separated from the capture reader so it can be driven with synthetic series
    whose answer is known -- including a deliberately defective heading, which
    is the only way to show the instrument can fail (see the tests).
    """
    b, h, r = (_unwrap(x) for x in (bearing_deg, heading_deg, roll_deg))
    n = b.size
    out = {"n": int(n)}
    if n < 30:
        return {**out, "bearing": None, "roll": None, "verdict": "inconclusive",
                "reason": "fewer than 30 usable samples"}

    design = np.column_stack([b, r, np.ones(n)])
    coefs = _fit(design, h)
    lo, hi = _block_bootstrap_ci(design, h, block=int(round(BOOTSTRAP_BLOCK_S * rate_hz)))
    out["offset_deg"] = float(coefs[2])
    out["residual_sd_deg"] = float((h - design @ coefs).std())
    out["bearing"] = _axis_verdict(
        coefs[0], lo[0], hi[0], 1.0, b.max() - b.min(), "bearing",
        "it is not tracking where the sensor points")
    out["roll"] = _axis_verdict(
        coefs[1], lo[1], hi[1], 0.0, r.max() - r.min(), "ROLL",
        "rolling the device in the hand is being reported as a turn (BUG-058)")

    axes = (out["bearing"], out["roll"])
    verdict = ("bad" if any(a["verdict"] == "bad" for a in axes)
               else "inconclusive" if any(a["verdict"] == "inconclusive" for a in axes)
               else "good")
    reasons = [a["reason"] for a in axes if a["reason"]]
    return {**out, "verdict": verdict, "reason": "; ".join(reasons) if reasons else None}


def inclination_deg(world_field) -> float:
    """Mean dip of a world-frame field, in degrees: POSITIVE means it points
    below the horizon, which is what Earth's field does in the northern
    hemisphere (~70 deg here).

    The constant-offset check `score()` is structurally blind to: an
    anti-parallel vector puts magnetic north 180 deg out and reverses every
    heading (BUG-059), while leaving both regression coefficients untouched.
    Averaged first, then measured -- the field is world-fixed, so the mean over
    a capture is a better estimate of it than any single noisy sample.
    """
    wm = np.asarray(world_field, dtype=np.float64).reshape(-1, 3).mean(axis=0)
    return math.degrees(math.atan2(-wm[2], math.hypot(wm[0], wm[1])))


def check_capture(path, cal_path=None) -> dict:
    """Score the heading reported over the capture at `path`."""
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
        return {"error": "capture carries no stream 10 (ENV) samples", "path": str(p),
                "cal_path": str(cal_p)}
    if not bool(np.any(np.abs(quats[:, 1:]) > 0)):
        return {"error": "capture carries no stream 9 (IMU_QUAT) -- heading needs orientation",
                "path": str(p), "cal_path": str(cal_p)}

    bearing, heading, roll, world, undefined = [], [], [], [], 0
    for q, m in zip(quats, mags):
        qt = tuple(float(v) for v in q)
        cal_mag = tuple(AXIS_CONVENTION @ cal.apply(tuple(float(v) for v in m)))
        world.append(quat_to_matrix(*qt) @ np.asarray(cal_mag))
        b = boresight_bearing_deg(qt)
        h = absolute_heading(qt, cal_mag)
        if b is None or h is None:
            undefined += 1          # aimed within 10 deg of vertical: no bearing exists
            continue
        bearing.append(b)
        heading.append(h)
        roll.append(boresight_view_deg(qt)[2])

    inclination = inclination_deg(world)

    dur = float(t_s[-1] - t_s[0]) if t_s.size > 1 else 0.0
    rate = (mags.shape[0] / dur) if dur > 0 else 30.0
    return {
        "path": str(p),
        "cal_path": str(cal_p),
        "samples": int(mags.shape[0]),
        "duration_s": dur,
        "undefined_frac": (undefined / mags.shape[0]) if mags.shape[0] else 0.0,
        "inclination_deg": inclination,
        "inclination_verdict": ("good" if inclination > 0 else "bad"),
        "inclination_note": (None if inclination > 0 else
                             "the calibrated field points UP -- anti-parallel to Earth's, so "
                             "magnetic north is 180 deg out and every heading is reversed (BUG-059)"),
        **score(bearing, heading, roll, rate_hz=rate),
    }


def format_report(r: dict) -> str:
    if "error" in r:
        return f"{r.get('path', '?')}: ERROR -- {r['error']}"
    lines = [f"{r['path']}",
             f"  cal {r['cal_path']}",
             f"  {r['samples']} samples over {r['duration_s']:.1f} s; "
             f"{r['undefined_frac']:.1%} of frames aimed within 10 deg of vertical (no bearing)",
             f"  field inclination: {r['inclination_deg']:+.1f} deg "
             f"({'below' if r['inclination_deg'] > 0 else 'ABOVE'} horizontal)  "
             f"{r['inclination_verdict'].upper()}"]
    for key, label, want in (("bearing", "bearing", "+1.000"), ("roll", "ROLL   ", " 0.000")):
        a = r.get(key)
        if not a:
            continue
        lines.append(f"  per deg of {label}: {a['coef']:+.3f} "
                     f"[{a['ci95'][0]:+.3f}, {a['ci95'][1]:+.3f}]  want {want}   "
                     f"(swept {a['range_deg']:.1f} deg)  {a['verdict'].upper()}")
    if "residual_sd_deg" in r:
        lines.append(f"  residual sd: {r['residual_sd_deg']:.2f} deg")
    lines.append(f"  verdict: {r['verdict'].upper()}" + (f" -- {r['reason']}" if r.get("reason") else ""))
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--cal", default=None,
                    help="calibration JSON (default: the one roomscan-web would load)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    r = check_capture(args.capture, args.cal)
    print(json.dumps(r, indent=2) if args.json else format_report(r))
    return 1 if ("error" in r or r.get("verdict") == "bad") else 0


if __name__ == "__main__":
    raise SystemExit(main())
