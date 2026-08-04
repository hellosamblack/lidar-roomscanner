"""Measure the ToF↔IMU frame-timing skew in a RECORDED capture (BUG-031).

    host/.venv/bin/python host/tools/skew_check.py captures/web_20260730_175304.bin
    host/.venv/bin/python host/tools/skew_check.py <capture> --json

The question: a depth frame's header `t_us` is its FRAME_READY edge on the MCU's
TIM2 clock, but the rotation used to orient that frame comes from the LSM, on the
LSM's clock. How well do we know where one instant sits on the other's clock?

Two estimators, both reported, because they answer that question differently:

  `fifo`  -- the INFERENCE, and the only thing possible before stream 13: pair
            each ToF frame with the last TIMESTAMP word of the stream-11 FIFO
            batch sent alongside it. That word is a sample the LSM took at some
            point before the firmware's drain, which runs 24.3 ms later in the
            acquisition loop (measured -- see `drain_delay_us`), after the DMA
            readout and the RAW send. Its residual against a linear
            clock fit is therefore frame-stamp error PLUS drain lag PLUS up to
            one sample period of FIFO phase. This is the estimator that produced
            the 1072 µs RMS recorded in BUG-031.
  `sync`  -- the MEASUREMENT (stream 13, 2026-07-30): the LSM TIMESTAMP register
            read at the FRAME_READY edge itself, corrected for the read's own
            delay. Absent from any capture older than that firmware.

`calib_load` is the causal test that separated the two. Every 64th frame also
carries the 2332-byte CALIB blob, sent BEFORE the drain -- a natural load
experiment nobody has to set up. If the drain lag is real, those frames' `fifo`
residual must sit systematically lower, and it does.

MEASUREMENT NOTES

* Fit windows, not one global fit. The MCU and LSM oscillators wander against
  each other by ~±1.8 ms over three minutes, so a single fit over a long capture
  reports that wander as if it were per-frame jitter (1628 µs global vs 1070 µs
  windowed on the same 176 s file).
* State the window, because it stopped being a detail once stream 13 landed. The
  `fifo` estimator is insensitive to it -- 1070 / 1073 / 1085 µs RMS at 2 / 5 /
  20 s -- because its error is white. `sync` is not: 18 / 38 / 150 µs over the
  same windows on the same capture. That difference is itself the result. What
  survives the fix is the two oscillators drifting against each other (lag-1
  autocorrelation 0.992), which any local clock model removes, and not per-frame
  skew. `frame_to_frame_us`, the std of the residual's first difference, is the
  window-free version of the same number and is the one to quote when in doubt.
  `window_s` defaults to 2 s: 60 frames at 30 fps, enough for a stable slope.
* Centre before fitting. Raw values are ~1e10 µs and the naive [x, 1] design
  matrix is badly conditioned enough to return a nonsense slope (0.044 instead
  of 1.003) without any warning.
* Scale ticks with stream 12, never the nominal 21.7 µs -- this part is trimmed
  ~3% off nominal (see `roomscan.protocol.imu_tick_us`).
* The `fifo` estimator has a floor of ~617 µs RMS it can never beat: the last
  FIFO word's phase is uniform over one 2.083 ms sample period. Quoting an
  improvement against it below that floor would be measuring the metric, not
  the firmware.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.decoder import StreamDecoder                          # noqa: E402
from roomscan.protocol import (FrameType, StreamId, decode_imu_cal,  # noqa: E402
                               decode_imu_raw, decode_imu_sync)
from roomscan.sources import FileSource, pump                        # noqa: E402

# One LSM sample period at the 480 Hz ODR the firmware runs (rs_lsm.c). The `fifo`
# estimator's phase noise is uniform over this, i.e. a std of PERIOD/sqrt(12).
LSM_SAMPLE_PERIOD_US = 96 * 21.7 / 1.0  # 96 ticks/sample; ~2083 µs nominal
DEFAULT_WINDOW_S = 2.0


def _fit_residual(x: np.ndarray, y: np.ndarray) -> tuple[float, np.ndarray]:
    """Least-squares y = a*x + b, returning (a, residuals). Centred: see module docstring."""
    x0 = x - x.mean()
    denom = float((x0 * x0).sum())
    if denom <= 0.0:
        return float("nan"), np.zeros_like(y)
    a = float((x0 * (y - y.mean())).sum() / denom)
    return a, y - (y.mean() + a * x0)


def windowed_residuals(x_us: np.ndarray, y_us: np.ndarray, window_s: float,
                       min_points: int = 30) -> tuple[np.ndarray, float, np.ndarray]:
    """Residuals of y against x from independent per-window linear fits.

    Windows are cut on y (the MCU clock) and fitted independently, so slow
    oscillator wander between the two clocks lands in the per-window slope/offset
    instead of inflating the residual. Returns (residuals, mean slope, used-mask) --
    the mask matters because a short trailing window is dropped, so the residual
    array is shorter than the input and any per-frame covariate must be masked the
    same way before it can be compared against it.
    """
    used = np.zeros(x_us.size, dtype=bool)
    if x_us.size < min_points:
        return np.zeros(0), float("nan"), used
    t_rel = (y_us - y_us[0]) / 1e6
    out, slopes = [], []
    edge = 0.0
    while edge < t_rel[-1]:
        m = (t_rel >= edge) & (t_rel < edge + window_s)
        edge += window_s
        if m.sum() < min_points:
            continue
        a, r = _fit_residual(x_us[m], y_us[m])
        out.append(r)
        slopes.append(a)
        used |= m
    if not out:
        return np.zeros(0), float("nan"), used
    return np.concatenate(out), float(np.mean(slopes)), used


def summarize_us(resid: np.ndarray) -> dict:
    """RMS / p95 / max of a residual series, in µs. p95 and max are on |residual|.

    `frame_to_frame_us` is the std of the residual's first difference: the part of
    the error that changes between adjacent frames, which is window-free. A series
    dominated by slow drift has a large RMS and a small `frame_to_frame_us`; white
    error has `frame_to_frame_us` ≈ sqrt(2) × RMS.
    """
    if resid.size == 0:
        return {"n": 0}
    a = np.abs(resid)
    out = {
        "n": int(resid.size),
        "rms_us": round(float(np.sqrt((resid ** 2).mean())), 1),
        "p95_us": round(float(np.percentile(a, 95)), 1),
        "max_us": round(float(a.max()), 1),
        "std_us": round(float(resid.std()), 1),
    }
    if resid.size > 2:
        out["frame_to_frame_us"] = round(float(np.diff(resid).std()), 1)
    return out


def _welch_t(a: np.ndarray, b: np.ndarray) -> float:
    if a.size < 2 or b.size < 2:
        return float("nan")
    se = np.sqrt(a.var(ddof=1) / a.size + b.var(ddof=1) / b.size)
    return float((a.mean() - b.mean()) / se) if se > 0 else float("nan")


def collect_frames(path: str | Path) -> tuple[list[dict], float, bool]:
    """Walk a capture, returning one row per ToF frame plus the stream-12 tick.

    Row keys: `t_us` (RAW header), `ts_last_us` (last stream-11 TIMESTAMP word,
    LSM clock), `n_ts`, `calib` (this frame also carried CALIB), and, when the
    capture has stream 13, `sync_us` / `latch_delay_us` / `drain_delay_us` /
    `read_us`.

    IMU_RAW (stream 11) accumulates a LIST of payloads per seq, not a single
    overwritten one (Task 7, high-framerate plan step 7): a decoupled IMU/env
    rate (SET_IMU_ENV_RATE, cmd 11) can drain several times off its own TIM2-paced
    schedule while one ToF seq is still current (`g_last_seq`, frozen — see
    docs/protocol.md and rs_lsm_service_tick() in vl53l9_app.c), so more than one
    IMU_RAW frame legitimately shares one seq. The old code kept `row["imu_raw"] =
    frame.payload` — a plain overwrite — so every payload but the LAST one sharing
    a seq was silently discarded (fine for coupled mode, where at most one ever
    arrives per seq, but wrong the moment a decoupled rate is in force). All
    payloads sharing a seq are now decoded and their TIMESTAMP words concatenated,
    in arrival order (== chronological order: drains run to completion strictly
    one at a time on the single-threaded firmware loop, so payload N's words
    always predate payload N+1's) — `ts_last_us`/`n_ts` are computed from the
    FULL concatenated series, not just the final payload. IMU_SYNC (stream 13)
    stays a single scalar per seq on purpose: the firmware only ever sends it
    from the one drain that is genuinely coincident with that seq's own
    FRAME_READY edge (never from an off-cycle decoupled drain), so at most one
    can exist per seq in EITHER mode.
    """
    src = FileSource(str(path))
    dec = StreamDecoder()
    tick_us = 21.7
    tick_from_device = False
    rows: dict[int, dict] = {}
    order: list[int] = []
    try:
        for frame in pump(src, dec):
            h = frame.header
            if h.frame_type != FrameType.DATA:
                continue
            if h.stream_id == StreamId.IMU_CAL:
                cal = decode_imu_cal(frame.payload)
                tick_us, tick_from_device = cal.tick_us, True
                continue
            row = rows.get(h.seq)
            if row is None:
                row = rows[h.seq] = {"seq": h.seq, "imu_raw_payloads": []}
                order.append(h.seq)
            if h.stream_id == StreamId.RAW_3DMD:
                row["t_us"] = h.t_us
            elif h.stream_id == StreamId.CALIB:
                row["calib"] = True
            elif h.stream_id == StreamId.IMU_RAW:
                row["imu_raw_payloads"].append(frame.payload)
            elif h.stream_id == StreamId.IMU_SYNC:
                row["sync"] = decode_imu_sync(frame.payload)
    finally:
        src.close()

    out = []
    for seq in order:
        row = rows[seq]
        if "t_us" not in row:
            continue
        rec: dict = {"seq": seq, "t_us": float(row["t_us"]),
                     "calib": bool(row.get("calib", False))}
        payloads = row.get("imu_raw_payloads") or []
        if payloads:
            rec["n_imu_raw_sends"] = len(payloads)  # >1 only under a decoupled rate
            ticks_parts = []
            for payload in payloads:
                batch = decode_imu_raw(payload, tick_us=tick_us)
                if batch.timestamp_ticks.size:
                    ticks_parts.append(batch.timestamp_ticks)
            if ticks_parts:
                all_ticks = np.concatenate(ticks_parts)
                rec["ts_last_us"] = float(all_ticks[-1]) * tick_us
                rec["n_ts"] = int(all_ticks.size)
        sync = row.get("sync")
        if sync is not None and sync.valid:
            rec["sync_us"] = sync.frame_ready_ticks(tick_us) * tick_us
            rec["latch_delay_us"] = float(sync.latch_delay_us)
            rec["drain_delay_us"] = float(sync.drain_delay_us)
            rec["read_us"] = float(sync.read_us)
        out.append(rec)
    return out, tick_us, tick_from_device


def check_capture(path: str | Path, window_s: float = DEFAULT_WINDOW_S) -> dict:
    """Full skew report for one capture. Pure aside from reading the file."""
    rows, tick_us, tick_from_device = collect_frames(path)
    report: dict = {
        "path": str(path),
        "frames": len(rows),
        "tick_us": round(tick_us, 5),
        "tick_from_device": tick_from_device,
        "window_s": window_s,
        "fifo_floor_us": round(LSM_SAMPLE_PERIOD_US / np.sqrt(12.0), 1),
    }
    if len(rows) < 30:
        report["error"] = "too few ToF frames to fit a clock"
        return report

    t_us = np.array([r["t_us"] for r in rows])
    report["span_s"] = round(float((t_us[-1] - t_us[0]) / 1e6), 1)
    report["fps"] = round(float((len(rows) - 1) / ((t_us[-1] - t_us[0]) / 1e6)), 2)

    # --- fifo estimator (the pre-stream-13 inference) --------------------------------
    have = np.array([("ts_last_us" in r) for r in rows])
    if have.sum() >= 30:
        tf = t_us[have]
        ts = np.array([r["ts_last_us"] for r in rows if "ts_last_us" in r])
        resid, slope, used = windowed_residuals(ts, tf, window_s)
        report["fifo"] = summarize_us(resid)
        report["fifo"]["clock_ppm"] = (round((slope - 1.0) * 1e6)
                                       if np.isfinite(slope) else None)

        # causal test: does the drain lag move with processing load?
        calib = np.array([r["calib"] for r in rows if "ts_last_us" in r])[used]
        if resid.size == calib.size and calib.any() and (~calib).any():
            report["calib_load"] = {
                "n_calib": int(calib.sum()),
                "n_plain": int((~calib).sum()),
                "calib_mean_us": round(float(resid[calib].mean()), 1),
                "plain_mean_us": round(float(resid[~calib].mean()), 1),
                "shift_us": round(float(resid[calib].mean() - resid[~calib].mean()), 1),
                "welch_t": round(_welch_t(resid[calib], resid[~calib]), 2),
            }

    # --- sync estimator (stream 13) ---------------------------------------------------
    have_s = np.array([("sync_us" in r) for r in rows])
    report["has_stream_13"] = bool(have_s.any())
    if have_s.sum() >= 30:
        tf = t_us[have_s]
        sy = np.array([r["sync_us"] for r in rows if "sync_us" in r])
        resid, slope, _used = windowed_residuals(sy, tf, window_s)
        report["sync"] = summarize_us(resid)
        report["sync"]["clock_ppm"] = (round((slope - 1.0) * 1e6)
                                       if np.isfinite(slope) else None)
        report["sync"]["coverage_pct"] = round(100.0 * have_s.sum() / len(rows), 1)
        for key in ("latch_delay_us", "drain_delay_us", "read_us"):
            v = np.array([r[key] for r in rows if key in r])
            report[key] = {
                "mean": round(float(v.mean()), 1),
                "median": round(float(np.median(v)), 1),
                "std": round(float(v.std()), 1),
                "p95": round(float(np.percentile(v, 95)), 1),
                "max": round(float(v.max()), 1),
            }
    return report


def _print(rep: dict) -> None:
    print(f"capture: {rep['path']}")
    if "error" in rep:
        print(f"  ERROR: {rep['error']}")
        return
    print(f"  {rep['frames']} ToF frames, {rep['span_s']} s, {rep['fps']} fps; "
          f"LSM tick {rep['tick_us']} µs "
          f"({'stream 12' if rep['tick_from_device'] else 'NOMINAL — no stream 12'})")
    print(f"  windowed clock fits, window = {rep['window_s']} s")
    f = rep.get("fifo")
    if f:
        print("\n  fifo  (inference from the last FIFO word — the BUG-031 estimator)")
        print(f"    RMS {f['rms_us']} µs   p95 {f['p95_us']} µs   max {f['max_us']} µs   "
              f"[{f['n']} frames, {f['clock_ppm']:+} ppm]")
        print(f"    frame-to-frame {f['frame_to_frame_us']} µs (window-free)")
        print(f"    this estimator cannot beat {rep['fifo_floor_us']} µs RMS "
              f"(one sample period of FIFO phase)")
    c = rep.get("calib_load")
    if c:
        print("\n  calib_load  (do CALIB-carrying frames drain later? the causal test)")
        print(f"    CALIB frames {c['calib_mean_us']:+} µs vs plain {c['plain_mean_us']:+} µs "
              f"-> shift {c['shift_us']:+} µs  (Welch t = {c['welch_t']:+}, "
              f"n = {c['n_calib']}/{c['n_plain']})")
    s = rep.get("sync")
    if s:
        print("\n  sync  (stream 13 — LSM clock read AT the frame-ready edge)")
        print(f"    RMS {s['rms_us']} µs   p95 {s['p95_us']} µs   max {s['max_us']} µs   "
              f"[{s['n']} frames, {s['clock_ppm']:+} ppm, {s['coverage_pct']}% of frames]")
        print(f"    frame-to-frame {s['frame_to_frame_us']} µs (window-free)")
        for key, label in (("latch_delay_us", "edge -> latch"),
                           ("read_us", "latch read cost"),
                           ("drain_delay_us", "edge -> FIFO drain")):
            v = rep[key]
            print(f"    {label:<20} mean {v['mean']:>8} µs  median {v['median']:>8}  "
                  f"std {v['std']:>7}  max {v['max']:>8}")
    elif not rep.get("has_stream_13"):
        print("\n  sync: capture predates stream 13 (no direct measurement available)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--window", type=float, default=DEFAULT_WINDOW_S,
                    help="clock-fit window in seconds (default 20)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    path = Path(args.capture)
    if not path.is_absolute():
        path = REPO / path
    if not path.is_file():
        print(f"no such capture: {path}", file=sys.stderr)
        return 2
    rep = check_capture(path, window_s=args.window)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print(rep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
