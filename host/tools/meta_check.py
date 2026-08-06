"""Report the ToF per-frame METADATA a recorded capture actually ran at.

    host/.venv/bin/python host/tools/meta_check.py captures/precisionRegular8msFFpanLarge.bin
    host/.venv/bin/python host/tools/meta_check.py <capture> --json

Every RAW_3DMD payload carries a 100-byte `vl53l9_meta_t` tail that the firmware
writes and the host has always archived but never decoded (see
`roomscan.protocol.decode_tof_meta`). This tool walks a capture, decodes that tail
per frame, and aggregates it — so a `.bin` whose exposure was only ever guessed from
its filename now reports the exposure the device *actually* used, plus the ToF die
temperature (ranging-drift relevant, datasheet ±0.1 mm/°C), the per-frame
error_status/error_code health flag, the DSS/binning/nb_step config readback, and the
internal reference-SPAD channels.

Exposure is inverted from the nb_shot_step counts via the AN6522 ratios and is
cross-checked (step1 vs step6); `exposure_consistent_frac` < 1.0 flags frames where
the two disagree. Pure: reads the file, returns a dict, prints nothing; `main()`
renders it as prose for the CLI and `roomscan.mcp_server` returns it as JSON.
"""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.decoder import StreamDecoder                          # noqa: E402
from roomscan.protocol import FrameType, StreamId, decode_tof_meta  # noqa: E402
from roomscan.sources import FileSource, pump                       # noqa: E402


def check_capture(path: str | Path) -> dict:
    """Decode + aggregate the RAW_3DMD metadata tail across a capture. Pure."""
    src = FileSource(str(path))
    dec = StreamDecoder()
    metas: list[dict] = []
    try:
        for frame in pump(src, dec):
            h = frame.header
            if h.frame_type != FrameType.DATA or h.stream_id != StreamId.RAW_3DMD:
                continue
            md = decode_tof_meta(frame.payload)
            if md is not None:
                metas.append(md)
    finally:
        src.close()

    if not metas:
        return {"n_frames": 0, "note": "no RAW_3DMD frames with a decodable metadata tail"}

    def col(key):
        return np.array([m[key] for m in metas], dtype=float)

    die = col("die_temp_c")
    exp = np.array([m["exposure_ms"] for m in metas if m["exposure_ms"] is not None], dtype=float)
    consistent = [m["exposure_consistent"] for m in metas if m["exposure_consistent"] is not None]
    fps = col("frame_period_us")
    fps = fps[fps > 0]
    modes = Counter(m["ranging_mode"] for m in metas)
    status_nz = sum(1 for m in metas if m["error_status"] != 0)
    code_nz = sum(1 for m in metas if m["error_code"] != 0)
    codes = sorted({m["error_code"] for m in metas if m["error_code"] != 0})
    refs = np.array([m["ref_channels"] for m in metas], dtype=float)

    def uniq(key):
        return sorted({m[key] for m in metas})

    return {
        "n_frames": len(metas),
        "ranging_mode": modes.most_common(1)[0][0],
        "ranging_mode_counts": dict(modes),
        "exposure_ms": {
            "median": float(np.median(exp)) if exp.size else None,
            "min": float(exp.min()) if exp.size else None,
            "max": float(exp.max()) if exp.size else None,
            "varied": bool(exp.size and (exp.max() - exp.min()) > 0.1),
            "consistent_frac": float(np.mean(consistent)) if consistent else None,
        },
        "die_temp_c": {"min": float(die.min()), "max": float(die.max()), "mean": float(die.mean())},
        "fps_from_frame_period": {
            "median": float(1_000_000.0 / np.median(fps)) if fps.size else None,
            "frame_period_us": uniq("frame_period_us"),
        },
        "config": {
            "dss_mode": uniq("dss_mode"),
            "binning": uniq("binning"),
            "nb_step": uniq("nb_step"),
            "power_mode": uniq("power_mode"),
        },
        "errors": {"status_nonzero_frames": status_nz, "code_nonzero_frames": code_nz, "codes_seen": codes},
        "ref_channels_mean": [round(float(v), 1) for v in refs.mean(axis=0)],
        "frame_counter": {"first": metas[0]["frame_counter"], "last": metas[-1]["frame_counter"]},
    }


def _print(rep: dict) -> None:
    if rep.get("n_frames", 0) == 0:
        print(rep.get("note", "no frames"))
        return
    e = rep["exposure_ms"]
    d = rep["die_temp_c"]
    print(f"frames:      {rep['n_frames']}  (frame_counter {rep['frame_counter']['first']}..{rep['frame_counter']['last']})")
    print(f"ranging:     {rep['ranging_mode']}  {rep['ranging_mode_counts']}")
    exp_med = f"{e['median']:.2f} ms" if e["median"] is not None else "unknown"
    warn = "" if not e["varied"] else "  (VARIED across capture)"
    cons = "" if e["consistent_frac"] in (None, 1.0) else f"  step1/step6 agree in {100 * e['consistent_frac']:.0f}% of frames"
    print(f"exposure:    {exp_med}{warn}{cons}")
    print(f"die temp:    {d['mean']:.1f} C mean (min {d['min']:.0f}, max {d['max']:.0f})  [raw u16, ~C; ST-undocumented scale]")
    fps = rep["fps_from_frame_period"]["median"]
    print(f"fps:         {fps:.1f}" if fps else "fps:         unknown")
    c = rep["config"]
    print(f"config:      dss={c['dss_mode']} binning={c['binning']} nb_step={c['nb_step']} power_mode={c['power_mode']}")
    er = rep["errors"]
    print(f"health:      error_status set in {er['status_nonzero_frames']} frames, error_code in {er['code_nonzero_frames']} (codes {er['codes_seen'] or 'none'})")
    print(f"ref chans:   {rep['ref_channels_mean']}  [amp/dist ch1L,ch2L,ch1S,ch2S]")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    path = Path(args.capture)
    if not path.is_absolute():
        path = REPO / path
    if not path.is_file():
        print(f"no such capture: {path}", file=sys.stderr)
        return 2
    rep = check_capture(path)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print(rep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
