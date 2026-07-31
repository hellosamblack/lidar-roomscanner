"""Capture inspection, host diagnostics, and offline SLAM re-rendering.

Most functions here delegate to the corresponding `host/tools/` script's pure
half and contribute no analysis logic of their own. `slam_rerender` is the one
exception in shape: it shells out to the `roomscan-slam` console script rather
than calling it in-process, because that job runs for many minutes and would
otherwise block the event loop and pull CUDA into the server process. It reads
the run's `--json` report instead of scraping stdout, so the prose and the
structured output stay one implementation with two front ends.
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

from .paths import CAPTURES, HOST, RECORDINGS, REPO, VENV_PY, WEB_WS, rel
from .server import mcp

sys.path.insert(0, str(HOST))  # `tools` is a top-level package rooted at host/


@mcp.tool()
def slam_loop_closure_gate(baseline: list[dict], loop_closure: list[dict]) -> dict:
    """Apply the required paired 95% loop-closure acceptance gate.

    Pass the ten matched runs from each circuit separately. Every run needs
    `horizontal_closure_m`, `lost`, and optional `died`; the tool returns the
    deterministic paired bootstrap interval and an acceptance decision. A
    positive mean alone is never enough: the lower 95% bound must be positive
    and the global pass may not add loss or die.
    """
    from roomscan.slam.validation import paired_loop_gate
    return paired_loop_gate(baseline, loop_closure)


def _survey(path: Path, max_frames: int = 400) -> dict:
    """Cheap bounded header walk: which streams a capture carries, and roughly how many.

    Deliberately not `analyze_capture.scan()` -- that decodes and CRCs the whole
    file, which is far too slow to run across a whole directory just to answer
    "does this one have stream 9?".
    """
    from tools.analyze_capture import _HEADER, HEADER_SIZE, MAGIC, MAX_PAYLOAD, STREAMS

    counts: dict[str, int] = {}
    frames = 0
    truncated = False
    with open(path, "rb") as f:
        data = f.read(8 << 20)  # a bounded prefix is enough to survey stream presence
    n = len(data)
    pos = 0
    while pos < n and frames < max_frames:
        idx = data.find(MAGIC, pos)
        if idx < 0:
            break
        pos = idx
        if n - pos < HEADER_SIZE:
            truncated = True
            break
        _m, ver, _ft, stream, _fl, _seq, _t, _w, _h, plen, _r = _HEADER.unpack(
            data[pos:pos + HEADER_SIZE])
        if ver != 1 or plen > MAX_PAYLOAD:
            pos += 1
            continue
        total = HEADER_SIZE + plen + 4
        if n - pos < total:
            truncated = True
            break
        name = STREAMS.get(stream, str(stream))
        counts[name] = counts.get(name, 0) + 1
        frames += 1
        pos += total
    return {"streams": counts, "frames_sampled": frames, "prefix_truncated": truncated}


@mcp.tool()
def capture_list(surveyed: bool = True) -> dict:
    """List recorded captures in captures/ and recordings/, newest first.

    Reports size, mtime and -- when `surveyed` -- which streams each file carries,
    including `has_stream_9` (IMU_QUAT). SLAM and orientation work both require a
    stream-9 capture, and answering that question is otherwise a manual decode.

    Set `surveyed=False` for a fast listing that only stats the files.
    """
    out = []
    for d in (CAPTURES, RECORDINGS):
        if not d.is_dir():
            continue
        for p in d.glob("*.bin"):
            st = p.stat()
            entry = {
                "path": rel(p),
                "size_bytes": st.st_size,
                "size_mb": round(st.st_size / 1e6, 1),
                "mtime": int(st.st_mtime),
            }
            if surveyed:
                s = _survey(p)
                entry.update(s)
                entry["has_stream_9"] = "IMU_QUAT" in s["streams"]
            out.append(entry)
    out.sort(key=lambda e: e["mtime"], reverse=True)
    return {"count": len(out), "captures": out}


@mcp.tool()
def capture_analyze(path: str, min_zero_run: int = 50, zero_scan_frames: int = 8,
                    dump_bytes: int = 0, include_frame_log: bool = False) -> dict:
    """Byte-exact forensics over a capture: CRC failures, skip runs, truncation.

    Every anomaly is pinned to a file offset and carries the decoded header fields.
    `include_frame_log` adds the full per-frame inventory, which is thousands of
    entries on a real capture -- leave it off unless you need it.

    Wraps `host/tools/analyze_capture.py::scan()`.
    """
    from tools.analyze_capture import scan

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    r = scan(str(p), min_zero_run=min_zero_run, zero_scan_frames=zero_scan_frames,
             dump_bytes=dump_bytes)
    if not include_frame_log:
        r["frame_log_len"] = len(r.pop("frame_log"))
    r["path"] = rel(p)
    return r


@mcp.tool()
def capture_magcheck(path: str, cal_path: str = "", window_s: float = 5.0,
                     compare: list[str] | None = None) -> dict:
    """Score a magnetometer calibration against a recorded capture it never saw.

    Read `attitude.attitude_locked_pct` first -- on a moving capture that is the
    calibration's own error, with the room's slowly-varying field detrended out.
    It is a LOWER bound, so confirm it against the detrend-free `tilt` table:
    a good fit is flat across boresight tilt, BUG-030's bad fit ramped 40->110 uT.
    `field` is `magsweep.field_consistency`, correct for a stationary tumble but
    it under-rates a good calibration on a walk (its bias term absorbs the room).

    `cal_path` defaults to the calibration a roomscan-web on this box would load.
    `compare` scores several calibrations against the same capture.

    Wraps `host/tools/mag_check.py::check_capture()`.
    """
    from tools.mag_check import check_capture

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}

    def _resolve(c: str) -> str | None:
        if not c:
            return None
        cp = Path(c)
        return str(cp if cp.is_absolute() else REPO / cp)

    if compare:
        return {"path": rel(p),
                "reports": [check_capture(p, _resolve(c), window_s=window_s) for c in compare]}
    return check_capture(p, _resolve(cal_path), window_s=window_s)


@mcp.tool()
def capture_skew(path: str, window_s: float = 2.0) -> dict:
    """Measure how well a capture pins its ToF frames onto the IMU's clock (BUG-031).

    Two estimators. `sync` is the direct measurement -- stream 13, the LSM
    TIMESTAMP register read at the FRAME_READY edge -- and is absent from any
    capture older than 2026-07-30. `fifo` is the older inference from stream 11's
    last FIFO word, which also absorbs the firmware's drain lag and up to one
    sample period of FIFO phase; it cannot beat ~601 us RMS however good the
    timing gets, so read it as a bound, not a skew.

    `calib_load` is the causal test: CALIB-carrying frames drain later, so a
    negative `shift_us` with a large |welch_t| means the pairing moves with
    processing load.

    Wraps `host/tools/skew_check.py::check_capture()`.
    """
    from tools.skew_check import check_capture

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    rep = check_capture(p, window_s=window_s)
    rep["path"] = rel(p)
    return rep


@mcp.tool()
def doctor(build: bool = False, net: bool = True) -> dict:
    """Run the headless-host bring-up checks and return each verdict.

    Checks vendored 53L9A1 sources, the native transform .so, the live board UDP
    stream + mDNS, vendored three.js, and a WebGL-capable browser. `build=True`
    builds the native library if it is missing; `net=False` skips the board probe.

    Wraps `host/tools/headless_doctor.py::Doctor`.
    """
    from tools.headless_doctor import Doctor

    d = Doctor(quiet=True)
    failed = d.run(build=build, net=net)
    return {"failed": failed, "ok": failed == 0, "checks": d.results}


@mcp.tool()
async def orientation_probe(mode: str = "jitter", seconds: float = 15.0,
                            label: str = "", url: str = "") -> dict:
    """Measure orientation noise or stream health against a running roomscan-web.

    `mode="jitter"` reports per-frame rotation change (mean/median/p95/max degrees
    plus edge motion at 3 m); `mode="health"` reports per-stream Hz, drops and gaps.
    Needs the server up -- call `rig_status()` first.

    Wraps `host/tools/orientation_probe.py`.
    """
    from tools import orientation_probe as op

    url = url or WEB_WS
    if mode == "jitter":
        dirs = await op.collect_directions(url, seconds)
        if not dirs:
            return {"error": "no POINT_CLOUD messages seen — is the rig streaming?",
                    "url": url, "seconds": seconds}
        return op.summarize_jitter(dirs, seconds=seconds, label=label)
    if mode == "health":
        msgs = await op.collect_metrics(url, seconds)
        if not msgs:
            return {"error": "no metrics messages seen — is the server up?",
                    "url": url, "seconds": seconds}
        return op.summarize_health(msgs, seconds=seconds)
    return {"error": f"unknown mode {mode!r} (expected 'jitter' or 'health')"}



@mcp.tool()
def slam_rerender(capture: str, voxel_size: float = 0.0, block_count: int = 0,
                  device: str = "", max_frames: int = 0, icp_mode: str = "",
                  out_mesh: str = "", out_traj: str = "", timeout_s: int = 1800) -> dict:
    """Re-run SLAM over a recorded capture at a chosen resolution, offline.

    A live scan is only a preview: the capture stores raw ToF frames, not a map, so
    the whole pipeline can be re-run afterwards at any voxel size. This is the
    high-detail post-processing pass -- `voxel_size=0.005` roughly doubles map
    detail over the 10 mm default.

    Two limits worth knowing before picking a voxel size. The sensor samples about
    36 mm between adjacent rays at 2 m, so below ~5 mm the extra detail comes only
    from multi-view fusion (dense back-and-forth sweeping), never from a single
    view. And blocks scale as 1/voxel_size^2, so halving the voxel needs roughly 4x
    the `block_count`. Give it real headroom: a scan that ran at ~97% of its
    capacity stalled and lost tracking (BUG-035), so check the returned
    `map.saturated`. Past ~6 GiB of grid, pass `device="CPU:0"`, where system RAM
    rather than VRAM is the limit.

    Before quoting `trajectory.start_end_gap_m` as drift, read `tracking.died`.
    Once a frame is lost the pose freezes and nothing relocalizes, so a run that
    ended in a sustained lost streak reports a plausible-looking gap that is really
    just where the estimate stood when it quit -- one real circuit reported 2.05 m
    of "drift" whose last 22% was fabricated (BUG-036). `tracking.trailing_lost`
    and `longest_lost_run` say how much of the tail to distrust, and
    `icp_escalations` counts frames the tight ICP radius could not handle alone
    (~0 on a clean scan; many means the run was repeatedly near that failure).

    `baro.correction_m` says how much of the reported height came from the
    barometer rather than from ICP. Expect ~10 mm; a large value means a drifting
    barometer dragged the run. Note that `trajectory.path_length_m` from before
    2026-07-30 is not comparable with this one: the old height constraint injected
    the barometer's per-frame noise straight into the pose, so ~35% of the reported
    path was vertical motion that never happened, and every "% of path" drift
    figure computed against it was flattered by the same factor (BUG-037).

    Runs `roomscan-slam` as a subprocess (this is a long batch job -- many minutes on
    a full-length capture, and it must not block the server's event loop or pull CUDA
    into this process). Bound it with `max_frames` for a quick check. Zero/empty
    arguments mean "use the [slam] config default".
    """
    import json
    import tempfile

    cap = Path(capture)
    if not cap.is_absolute():
        for base in (REPO, CAPTURES, RECORDINGS):
            if (base / capture).exists():
                cap = base / capture
                break
    if not cap.exists():
        return {"ok": False, "error": f"capture not found: {capture}"}

    with tempfile.TemporaryDirectory() as tmp:
        report_path = Path(tmp) / "slam.json"
        cmd = [str(VENV_PY), "-m", "roomscan.slam.cli", str(cap), "--json", str(report_path)]
        for flag, value in (("--voxel-size", voxel_size), ("--block-count", block_count),
                            ("--device", device), ("--max-frames", max_frames),
                            ("--icp-mode", icp_mode), ("--out-mesh", out_mesh),
                            ("--out-traj", out_traj)):
            if value:
                cmd += [flag, str(value)]
        try:
            p = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                               capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            return {"ok": False, "cmd": " ".join(cmd),
                    "error": f"timed out after {timeout_s}s -- raise timeout_s, bound the run "
                             "with max_frames, or use a coarser voxel_size"}
        if p.returncode != 0 or not report_path.exists():
            return {"ok": False, "cmd": " ".join(cmd), "returncode": p.returncode,
                    "stdout": p.stdout[-2000:], "stderr": p.stderr[-2000:]}
        report = json.loads(report_path.read_text())

    # Surface saturation at the top level: it is the difference between "this map is
    # finished" and "this map stopped partway and the trajectory after that is junk".
    result = {"ok": True, "cmd": " ".join(cmd), **report}
    chosen = report["modes"][report["chosen_mode"]]
    if chosen["map"]["saturated"]:
        result["warning"] = (
            f"grid filled at {chosen['map']['blocks']}/{chosen['map']['capacity']} blocks: the map "
            "stopped accepting geometry partway through and tracking will have collapsed after "
            "that point. Re-run with a larger block_count (BUG-035).")
    # Same class of silent failure as saturation: the run reports a trajectory and a
    # start/end gap even though its tail is a frozen dead-reckoned pose (BUG-036).
    tracking = chosen.get("tracking") or {}
    if tracking.get("died"):
        result["warning"] = (
            f"tracking never recovered: the last {tracking['trailing_lost']} of "
            f"{tracking['n']} frames are dead-reckoned from a frozen pose, so the "
            f"trajectory tail and start_end_gap_m are not measurements. The map is "
            f"only valid up to that point.")
    return result
