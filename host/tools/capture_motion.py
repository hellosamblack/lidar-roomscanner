"""What the operator physically DID during a recorded capture, from stream 9.

    host/.venv/bin/python host/tools/capture_motion.py captures/DebugCapF.bin
    host/.venv/bin/python host/tools/capture_motion.py <capture> --json

Several acceptance gates in ROADMAP.md's data-collection queue are conditions on
the operator's motion, not on the data's integrity, and until now nothing could
check them: DC-F asks for "3 takes x ~60 s ... 10 s stationary at both ends of
every take", DC-E for "sweep tilt level -> 45 deg -> vertical, holding each ~15 s.
Two cycles", DC-D for "slowly panning the whole time" (a static flat-field capture
is invalid -- it bakes scene texture into the correction), DC-A for "3-4 deliberate
fast whips". Whether a capture actually contains those is a question about angular
rate over time, which is exactly what stream 9 plus the TIM2 `t_us` clock encode.

WHAT IT REPORTS

`segments` alternates `hold` and `move` runs: the take/bookend structure. A DC-F
capture holding three takes with bookends should read hold/move/hold/move/hold...
`holds` and `moves` summarise those runs, and `takes` counts move runs long enough
to be a deliberate pan rather than a twitch.

`rate_deg_s` percentiles characterise the motion band -- p95 is what says whether
DC-F's "slow ~20 / medium ~50 / fast ~100 deg/s" takes are present and distinct.
`fast_events` counts excursions above `--fast-deg-s`, which is DC-A's gate.

`tilt_deg` is boresight elevation above horizontal, so DC-E's level/45/vertical
cycles show up directly as its per-hold values.

MEASUREMENT NOTES

* Rate is the geodesic angle between consecutive quaternions over the measured
  `dt`, NOT a fixed 1/30 s. Frames are lost on this link (see analyze_capture's
  continuity census), and dividing a two-frame rotation by one frame period would
  report a phantom doubling of speed exactly where the data is worst.
* A gap longer than `--max-dt` breaks the segment rather than being interpolated
  across: during a 7 s dropout the operator's motion is simply unmeasured, and
  inventing a smooth interpolation there would fabricate the very thing the
  DC gates are checking.
* Stream 9's quaternion is a batch MEAN and is valid ~7.8 ms after the frame stamp
  (BUG-031). That offset is constant, so it shifts every segment boundary equally
  and does not affect hold/move structure or rate magnitudes.
* Hold detection is hysteretic (`--hold-deg-s` to enter, 2x to leave) because the
  fp16 SFLP quantisation floor means a truly stationary device still reports
  ~0.02-0.06 deg/frame of dither; a single threshold chatters on it.

Only reads the capture; never writes.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))
sys.path.insert(0, str(REPO / "host"))   # so `tools.mag_check` resolves as a script too

from roomscan.decoder import StreamDecoder                            # noqa: E402
from roomscan.protocol import FrameType, StreamId, decode_imu_quat    # noqa: E402
from roomscan.sources import FileSource, pump                         # noqa: E402

# Stationary if below this, moving if above 2x it (hysteresis -- see module docstring).
DEFAULT_HOLD_DEG_S = 8.0
# A "fast whip": DC-A asks for ~1.5 m/s of travel, which on a handheld sweep comes
# with rotation well above the ~30 deg/s of an ordinary pan.
DEFAULT_FAST_DEG_S = 100.0
# Shortest run that counts as a deliberate hold or take rather than a pause/twitch.
# A take is short by design: DC-F's pans between two braced endpoints run 1-4 s, so
# a multi-second floor here would report a correctly-collected capture as empty.
DEFAULT_MIN_HOLD_S = 1.5
DEFAULT_MIN_TAKE_S = 0.75
# Beyond this the frame-to-frame rotation is unmeasured, not slow.
DEFAULT_MAX_DT_S = 0.25


def read_orientations(path: str | Path) -> tuple[np.ndarray, np.ndarray]:
    """Return (t_s, quats) for every stream-9 frame, in capture order.

    `t_s` is seconds from the first IMU_QUAT frame, on the TIM2 clock carried in
    the frame header (`docs/protocol.md`), so it survives frame loss intact.
    """
    src = FileSource(str(path))
    dec = StreamDecoder()
    ts: list[int] = []
    qs: list[tuple[float, float, float, float]] = []
    try:
        for frame in pump(src, dec):
            h = frame.header
            if h.frame_type != FrameType.DATA or h.stream_id != StreamId.IMU_QUAT:
                continue
            ts.append(h.t_us)
            qs.append(decode_imu_quat(frame.payload))
    finally:
        src.close()
    if not ts:
        return np.zeros(0), np.zeros((0, 4))
    t = np.asarray(ts, dtype=np.float64)
    return (t - t[0]) / 1e6, np.asarray(qs, dtype=np.float64)


def angular_rate(t_s: np.ndarray, quats: np.ndarray,
                 max_dt_s: float = DEFAULT_MAX_DT_S) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Geodesic angular rate between consecutive orientations.

    Returns (t_mid, rate_deg_s, valid) where `valid` is False across sample gaps
    longer than `max_dt_s` -- there the motion is unmeasured, not slow.
    """
    if t_s.size < 2:
        return np.zeros(0), np.zeros(0), np.zeros(0, dtype=bool)
    a, b = quats[:-1], quats[1:]
    # |<a,b>| -> geodesic angle; abs() folds the double cover so q and -q agree.
    dot = np.clip(np.abs(np.einsum("ij,ij->i", a, b)), 0.0, 1.0)
    ang = np.degrees(2.0 * np.arccos(dot))
    dt = np.diff(t_s)
    valid = (dt > 0) & (dt <= max_dt_s)
    rate = np.zeros_like(ang)
    np.divide(ang, dt, out=rate, where=valid)
    return t_s[:-1] + dt / 2.0, rate, valid


# Reused, not reimplemented: this is the convention BUG-030's tilt table was
# validated against. Note the SFLP quaternion's world is Z-up (world down =
# (0,0,-1)), NOT the renderer's Y-up `slam.frames.world_up()` -- deriving tilt in
# the wrong world frame reports a 90-degree tilt sweep as 0.7 degrees of nothing.
from tools.mag_check import boresight_tilt_deg                       # noqa: E402


def _runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous [start, end) index runs where `mask` is True."""
    out = []
    i = 0
    n = mask.size
    while i < n:
        if mask[i]:
            j = i
            while j < n and mask[j]:
                j += 1
            out.append((i, j))
            i = j
        else:
            i += 1
    return out


def _classify(rate: np.ndarray, valid: np.ndarray, hold_deg_s: float) -> np.ndarray:
    """Hysteretic still/moving state per sample. True = moving."""
    moving = np.zeros(rate.size, dtype=bool)
    state = False
    high, low = 2.0 * hold_deg_s, hold_deg_s
    for i in range(rate.size):
        if not valid[i]:
            moving[i] = state          # a dropout holds the previous state
            continue
        if state and rate[i] < low:
            state = False
        elif not state and rate[i] > high:
            state = True
        moving[i] = state
    return moving


def segment(t_mid: np.ndarray, rate: np.ndarray, valid: np.ndarray, tilt: np.ndarray, *,
            hold_deg_s: float = DEFAULT_HOLD_DEG_S,
            min_hold_s: float = DEFAULT_MIN_HOLD_S) -> list[dict]:
    """Split the capture into alternating `hold` and `move` segments."""
    if t_mid.size == 0:
        return []
    moving = _classify(rate, valid, hold_deg_s)

    # A pan slows through the middle of its arc and dips under the threshold; that
    # is one take, not three. Absorb sub-`min_hold_s` stills into the surrounding
    # motion FIRST, so the runs extracted below are already merged. Doing this after
    # extraction instead would leave a single pan reported as several tiny takes.
    for i, j in _runs(~moving):
        if float(t_mid[j - 1] - t_mid[i]) < min_hold_s:
            moving[i:j] = True

    segs = []
    for kind, mask in (("move", moving), ("hold", ~moving)):
        for i, j in _runs(mask):
            dur = float(t_mid[j - 1] - t_mid[i])
            sl = slice(i, j)
            seg = {
                "kind": kind,
                "start_s": round(float(t_mid[i]), 2),
                "end_s": round(float(t_mid[j - 1]), 2),
                "duration_s": round(dur, 2),
                "mean_rate_deg_s": round(float(rate[sl][valid[sl]].mean()), 2)
                if valid[sl].any() else None,
                "p95_rate_deg_s": round(float(np.percentile(rate[sl][valid[sl]], 95)), 2)
                if valid[sl].any() else None,
                "mean_tilt_deg": round(float(tilt[sl].mean()), 1),
                "dropout_frac": round(float(1.0 - valid[sl].mean()), 3),
            }
            segs.append(seg)
    return sorted(segs, key=lambda s: s["start_s"])


def describe(path: str | Path, *, hold_deg_s: float = DEFAULT_HOLD_DEG_S,
             fast_deg_s: float = DEFAULT_FAST_DEG_S,
             min_hold_s: float = DEFAULT_MIN_HOLD_S,
             min_take_s: float = DEFAULT_MIN_TAKE_S,
             max_dt_s: float = DEFAULT_MAX_DT_S) -> dict:
    """Characterise the operator's motion over a capture. Pure: returns a dict."""
    t_s, quats = read_orientations(path)
    if t_s.size < 2:
        return {"path": str(path), "error": "capture carries no usable stream 9 (IMU_QUAT)",
                "frames": int(t_s.size)}

    t_mid, rate, valid = angular_rate(t_s, quats, max_dt_s)
    tilt = boresight_tilt_deg(quats)
    segs = segment(t_mid, rate, valid, tilt[:-1], hold_deg_s=hold_deg_s, min_hold_s=min_hold_s)

    good = rate[valid]
    holds = [s for s in segs if s["kind"] == "hold"]
    moves = [s for s in segs if s["kind"] == "move"]
    takes = [s for s in moves if s["duration_s"] >= min_take_s]

    # A "fast event" is a contiguous excursion, not a sample count: one 0.5 s whip
    # is one event, and DC-A asks for 3-4 of them.
    fast_runs = _runs(valid & (rate > fast_deg_s))

    return {
        "path": str(path),
        "frames": int(t_s.size),
        "duration_s": round(float(t_s[-1]), 1),
        "unmeasured_frac": round(float(1.0 - valid.mean()), 3),
        "rate_deg_s": {
            "mean": round(float(good.mean()), 2) if good.size else None,
            "median": round(float(np.median(good)), 2) if good.size else None,
            "p95": round(float(np.percentile(good, 95)), 2) if good.size else None,
            "max": round(float(good.max()), 2) if good.size else None,
        },
        "tilt_deg": {
            "min": round(float(tilt.min()), 1),
            "max": round(float(tilt.max()), 1),
            "range": round(float(tilt.max() - tilt.min()), 1),
        },
        "moving_frac": round(float(np.mean([s["kind"] == "move" for s in segs])), 3)
        if segs else 0.0,
        "takes": len(takes),
        "holds": len(holds),
        "hold_total_s": round(sum(s["duration_s"] for s in holds), 1),
        "longest_hold_s": max((s["duration_s"] for s in holds), default=0.0),
        "fast_events": len(fast_runs),
        "fast_deg_s": fast_deg_s,
        "starts_with_hold": bool(segs and segs[0]["kind"] == "hold"),
        "ends_with_hold": bool(segs and segs[-1]["kind"] == "hold"),
        "segments": segs,
        "thresholds": {"hold_deg_s": hold_deg_s, "min_hold_s": min_hold_s,
                       "min_take_s": min_take_s, "max_dt_s": max_dt_s},
    }


def format_report(r: dict) -> str:
    if "error" in r:
        return f"=== {r['path']} ===\n  {r['error']}"
    out = [f"=== {r['path']} ===",
           f"  {r['frames']} orientation frames over {r['duration_s']} s "
           f"({r['unmeasured_frac'] * 100:.1f}% unmeasured across dropouts)",
           f"  rate deg/s: mean {r['rate_deg_s']['mean']}  median {r['rate_deg_s']['median']}  "
           f"p95 {r['rate_deg_s']['p95']}  max {r['rate_deg_s']['max']}",
           f"  boresight tilt: {r['tilt_deg']['min']}..{r['tilt_deg']['max']} deg "
           f"(range {r['tilt_deg']['range']})",
           f"  takes {r['takes']}   holds {r['holds']} totalling {r['hold_total_s']} s "
           f"(longest {r['longest_hold_s']} s)",
           f"  fast events > {r['fast_deg_s']} deg/s: {r['fast_events']}",
           f"  bookends: starts with hold {r['starts_with_hold']}, "
           f"ends with hold {r['ends_with_hold']}",
           "  segments:"]
    for s in r["segments"]:
        out.append(f"    {s['kind']:5} {s['start_s']:>7.2f}-{s['end_s']:>7.2f}s "
                   f"({s['duration_s']:>6.2f}s)  mean {s['mean_rate_deg_s']:>7} deg/s  "
                   f"p95 {s['p95_rate_deg_s']:>7}  tilt {s['mean_tilt_deg']:>6} deg")
    return "\n".join(out)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="capture_motion",
        description="Angular-rate and hold/move structure of a recorded capture.")
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--hold-deg-s", type=float, default=DEFAULT_HOLD_DEG_S)
    ap.add_argument("--fast-deg-s", type=float, default=DEFAULT_FAST_DEG_S)
    ap.add_argument("--min-hold-s", type=float, default=DEFAULT_MIN_HOLD_S)
    ap.add_argument("--min-take-s", type=float, default=DEFAULT_MIN_TAKE_S)
    ap.add_argument("--max-dt", type=float, default=DEFAULT_MAX_DT_S, dest="max_dt_s")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    results = [describe(p, hold_deg_s=args.hold_deg_s, fast_deg_s=args.fast_deg_s,
                        min_hold_s=args.min_hold_s, min_take_s=args.min_take_s,
                        max_dt_s=args.max_dt_s) for p in args.captures]
    if args.json:
        print(json.dumps(results if len(results) > 1 else results[0], indent=2))
    else:
        print("\n\n".join(format_report(r) for r in results))
    return 0


if __name__ == "__main__":
    sys.exit(main())
