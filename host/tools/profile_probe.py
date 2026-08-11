"""Pure analyzer over a recorded capture: what rate did a ranging profile actually run at?

    host/.venv/bin/python host/tools/profile_probe.py captures/web_20260803_121735.bin
    host/.venv/bin/python host/tools/profile_probe.py <capture> --requested-fps 30 --json

Task 1 of docs/superpowers/plans/completed/2026-07-31-high-framerate-and-manual-ranging-modes.md:
`roomscan.profiles` says what a requested configuration is EXPECTED to do; this tool
says what a recorded capture ACTUALLY did, so a "90 Hz" claim can be checked against
the plan's +-2% acceptance gate instead of taken on faith. It will be reused, unchanged
in shape, by every later hardware task in that plan (Tasks 4-6, 12) to grade a capture
against the profile that was requested.

Reports, purely from decoded frame headers/payloads:

  * requested vs measured FPS: median/p05/p95 RAW_3DMD inter-frame interval (from the
    device's own TIM2 `t_us`, not wall clock), and whether it is within +-2% of
    `--requested-fps`;
  * RAW seq gaps: span/received/missing/loss_pct on the reference (RAW_3DMD) stream;
  * CRC failures and skipped bytes, via the same `StreamDecoder` accounting
    `analyze_capture.py`/`skew_check.py` use;
  * per-stream pairing: how many RAW_3DMD frames have a same-seq IMU_QUAT/ENV/
    IMU_SYNC/IMU_RAW sibling. This assumes TODAY's coupled 1:1 emission (one sample
    per ToF trigger) -- Task 7 will introduce a decoupled N:1 mode (multiple
    independent sends sharing one frozen `seq`), which this tool does NOT yet handle;
    `paired_with_raw` is a lower bound in that mode, not a bug, until Task 7 lands its
    own N:1-aware successor (see the plan's `skew_check.collect_frames()` note);
  * stream-11 (IMU_RAW) effective sample rate: total distinct sample-time slots
    (TIMESTAMP FIFO words) decoded, divided by capture duration, compared against the
    480 Hz XL/GY/SFLP ODR ceiling.

UDP fragment health (incomplete/lost/reordered) is NOT recoverable from a .bin file
after the fact -- reassembly already happened by the time bytes reach the recorder, and
a frame that lost a fragment is dropped before it is ever written (see
`roomscan.sources.UdpSource`). Pass `--udp-stats-json` (a JSON object or a path to one)
with a live source's own counters -- e.g. from `rig_status()`'s metrics, which
`roomscan-web` already exposes without this tool ever binding the stream itself -- to
merge them into the report; absent that the field is `None`, not guessed.

Pure stdlib + numpy + roomscan (StreamDecoder/FileSource/pump/protocol). Only reads.
Never binds UDP/CDC directly -- `roomscan-web` owns the live device stream.
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
from roomscan.protocol import FrameType, StreamId, decode_imu_raw    # noqa: E402
from roomscan.sources import FileSource, pump                        # noqa: E402

RAW_STREAM = StreamId.RAW_3DMD
FPS_TOLERANCE_PCT = 2.0  # the plan's own 90 Hz acceptance-gate tolerance

# Streams paired against RAW_3DMD's seq, today's coupled (1:1) assumption only.
_PAIRED_STREAMS = (
    (StreamId.IMU_QUAT, "IMU_QUAT"),
    (StreamId.ENV, "ENV"),
    (StreamId.IMU_SYNC, "IMU_SYNC"),
    (StreamId.IMU_RAW, "IMU_RAW"),
)


def _interval_stats_ms(intervals_us: np.ndarray) -> dict:
    if intervals_us.size == 0:
        return {"n": 0}
    ms = intervals_us / 1000.0
    return {
        "n": int(intervals_us.size),
        "median_ms": round(float(np.median(ms)), 3),
        "p05_ms": round(float(np.percentile(ms, 5)), 3),
        "p95_ms": round(float(np.percentile(ms, 95)), 3),
        "mean_ms": round(float(ms.mean()), 3),
        "min_ms": round(float(ms.min()), 3),
        "max_ms": round(float(ms.max()), 3),
    }


def collect(path: str | Path) -> dict:
    """Walk `path` once, returning per-stream (seq, t_us) rows and decoder counters.

    Pure aside from reading the file. Kept separate from `probe()` so a future
    caller (e.g. a live-frame-timestamp path, per the plan's "or decoded frame
    timestamps" wording) can build the same `rows_by_stream` shape without a file.
    """
    src = FileSource(str(path))
    dec = StreamDecoder()
    rows_by_stream: dict[int, list[tuple[int, int]]] = {}
    imu_raw_samples = 0
    try:
        for frame in pump(src, dec):
            h = frame.header
            if h.frame_type != FrameType.DATA:
                continue
            rows_by_stream.setdefault(h.stream_id, []).append((h.seq, h.t_us))
            if h.stream_id == StreamId.IMU_RAW:
                try:
                    batch = decode_imu_raw(frame.payload)
                except Exception:
                    continue
                imu_raw_samples += int(batch.timestamp_ticks.size)
    finally:
        src.close()
    return {
        "rows_by_stream": rows_by_stream,
        "imu_raw_samples": imu_raw_samples,
        "crc_failures": dec.crc_failures,
        "bytes_skipped": dec.bytes_skipped,
        "frames_decoded": dec.frames_decoded,
    }


def probe(path: str | Path, *, requested_fps: float | None = None,
         udp_stats: dict | None = None) -> dict:
    """Full profile-rate report for one capture. Pure: reads the file, returns a
    dict, prints nothing. `main()` renders this as prose; the MCP wrapper returns
    it directly."""
    data = collect(path)
    rows = data["rows_by_stream"]
    report: dict = {
        "path": str(path),
        "frames_decoded": data["frames_decoded"],
        "crc_failures": data["crc_failures"],
        "bytes_skipped": data["bytes_skipped"],
        "requested_fps": requested_fps,
        "udp_stats": udp_stats,
    }

    raw_rows = rows.get(RAW_STREAM, [])
    if len(raw_rows) < 2:
        report["error"] = "fewer than 2 RAW_3DMD frames decoded; cannot measure rate"
        return report

    order = sorted(range(len(raw_rows)), key=lambda i: raw_rows[i][1])
    seqs = np.array([raw_rows[i][0] for i in order])
    t_us = np.array([raw_rows[i][1] for i in order], dtype=np.float64)
    intervals_us = np.diff(t_us)
    intervals_us = intervals_us[intervals_us > 0]  # guard t_us ties (should not happen)

    duration_s = float((t_us[-1] - t_us[0]) / 1e6)
    measured_fps = round((len(raw_rows) - 1) / duration_s, 3) if duration_s > 0 else 0.0

    report.update({
        "raw_frames": len(raw_rows),
        "duration_s": round(duration_s, 2),
        "measured_fps": measured_fps,
        "interval_ms": _interval_stats_ms(intervals_us),
    })

    if requested_fps:
        pct_error = 100.0 * (measured_fps - requested_fps) / requested_fps
        report["fps_pct_error"] = round(pct_error, 2)
        report["fps_within_tolerance"] = abs(pct_error) <= FPS_TOLERANCE_PCT
        report["fps_tolerance_pct"] = FPS_TOLERANCE_PCT

    seq_list = seqs.tolist()
    seq_set = set(seq_list)
    span = int(max(seq_list) - min(seq_list) + 1) if seq_list else 0
    missing = span - len(seq_set)
    report["raw_seq_gaps"] = {
        "span": span,
        "received": len(seq_set),
        "missing": missing,
        "loss_pct": round(100.0 * missing / span, 3) if span else 0.0,
    }

    pairing = {}
    for stream_id, label in _PAIRED_STREAMS:
        stream_rows = rows.get(stream_id, [])
        stream_seq_set = {s for s, _ in stream_rows}
        paired = len(seq_set & stream_seq_set)
        pairing[label] = {
            "frames": len(stream_rows),
            "paired_with_raw": paired,
            "paired_pct": round(100.0 * paired / len(seq_set), 1) if seq_set else 0.0,
        }
    report["stream_pairing"] = pairing

    imu_raw_samples = data["imu_raw_samples"]
    report["imu_raw"] = {
        "timestamp_samples": imu_raw_samples,
        "effective_hz": round(imu_raw_samples / duration_s, 1) if duration_s > 0 else 0.0,
        "nominal_odr_hz": 480,
    }

    return report


def _print(rep: dict) -> None:
    print(f"capture: {rep['path']}")
    if "error" in rep:
        print(f"  ERROR: {rep['error']}")
        return
    print(f"  {rep['raw_frames']} RAW_3DMD frames over {rep['duration_s']} s -> "
          f"measured {rep['measured_fps']} fps")
    if rep.get("requested_fps"):
        status = "OK" if rep.get("fps_within_tolerance") else "OUT OF TOLERANCE"
        print(f"  requested {rep['requested_fps']} fps -> "
              f"{rep['fps_pct_error']:+.2f}% [{status}, +-{rep['fps_tolerance_pct']}%]")
    iv = rep["interval_ms"]
    print(f"  interval: median {iv['median_ms']} ms  p05 {iv['p05_ms']} ms  "
          f"p95 {iv['p95_ms']} ms  [{iv['n']} intervals]")
    print(f"  crc_failures={rep['crc_failures']}  bytes_skipped={rep['bytes_skipped']}")
    g = rep["raw_seq_gaps"]
    print(f"  RAW seq: span={g['span']} received={g['received']} missing={g['missing']} "
          f"({g['loss_pct']}%)")
    print("  stream pairing (against RAW_3DMD seq, coupled/1:1 assumption):")
    for label, p in rep["stream_pairing"].items():
        print(f"    {label:10} frames={p['frames']:>6}  paired={p['paired_with_raw']:>6} "
              f"({p['paired_pct']}%)")
    ir = rep["imu_raw"]
    print(f"  stream-11 effective rate: {ir['effective_hz']} Hz "
          f"({ir['timestamp_samples']} samples; nominal ODR {ir['nominal_odr_hz']} Hz)")
    if rep.get("udp_stats") is not None:
        print(f"  udp_stats (supplied, not measured here): {rep['udp_stats']}")
    else:
        print("  udp_stats: None (not recoverable from a .bin file; pass "
              "--udp-stats-json from a live source)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--requested-fps", type=float, default=None,
                    help="compare the measured rate against this target (the plan's "
                         "own +-2%% tolerance)")
    ap.add_argument("--udp-stats-json", default=None,
                    help="a JSON object, or a path to one, with a live UdpSource's "
                         "frames_incomplete/frags_lost/frags_reordered -- this tool "
                         "cannot recover them from the capture file itself")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    path = Path(args.capture)
    if not path.is_absolute():
        path = REPO / path
    if not path.is_file():
        print(f"no such capture: {path}", file=sys.stderr)
        return 2

    udp_stats = None
    if args.udp_stats_json:
        raw = args.udp_stats_json
        candidate = Path(raw)
        text = candidate.read_text() if candidate.is_file() else raw
        udp_stats = json.loads(text)

    rep = probe(path, requested_fps=args.requested_fps, udp_stats=udp_stats)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print(rep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
