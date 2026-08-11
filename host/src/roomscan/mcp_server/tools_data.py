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

from .paths import CAPTURES, HOST, RECORDINGS, REPO, RESULTS, VENV_PY, WEB_WS, rel
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
    """Byte-exact forensics plus stream continuity: CRC, skip runs, truncation, lost frames.

    Every anomaly is pinned to a file offset and carries the decoded header fields.
    `include_frame_log` adds the full per-frame inventory, which is thousands of
    entries on a real capture -- leave it off unless you need it.

    **`clean` and `continuity.complete` are different questions.** `clean` says the
    bytes that arrived decode end to end; `continuity` says whether everything the
    device sent actually arrived. A capture can be `clean: true` and still be missing
    seconds of frames -- the three 2026-07-31 multi-room captures were byte-perfect
    while losing 2.3% / 4.3% / 9.4% of RAW frames, one in a single 215-frame (7.1 s)
    hole. Check `continuity.complete` before trusting a capture's coverage, and
    before quoting any SLAM result computed over it.

    `continuity.whole_group_lost` vs `partial_group_lost` separates two real faults:
    a seq absent from every stream is a link outage, while one absent only from
    RAW_3DMD is fragment loss on the ~15 KB datagram (its 20-byte siblings survived).
    `device_fps` (seq span over elapsed `t_us`) against `received_fps` shows what the
    device produced versus what the recorder kept. CALIB/IMU_CAL are censused against
    their 64-frame cadence instead, under `continuity.cadenced`.

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
def capture_meta(path: str) -> dict:
    """Report the ToF per-frame metadata a capture actually ran at.

    Every RAW_3DMD payload carries a 100-byte vl53l9_meta_t tail the host has
    always archived but never decoded. This reports the exposure the device
    *actually* used (inverted from nb_shot_step via AN6522 ratios, so a capture
    whose exposure was only guessed from its filename is now answerable), the ToF
    die temperature (ranging-drift relevant), the per-frame error_status/code
    health flag, the DSS/binning/nb_step config readback, and the internal
    reference-SPAD channels -- aggregated across the capture.

    `exposure_ms.consistent_frac` < 1.0 flags frames where the step1- and
    step6-derived exposures disagree; `die_temp_c` is a raw u16 that reads as C
    empirically (ST does not document the scale).

    Wraps `host/tools/meta_check.py::check_capture()`.
    """
    from tools.meta_check import check_capture

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    rep = check_capture(p)
    rep["path"] = rel(p)
    return rep


@mcp.tool()
def capture_heading(path: str, cal_path: str = "") -> dict:
    """Is the heading a HEADING, or is it wearing another axis's clothes (BUG-058)?

    Regresses the mag-referenced `absolute_heading` on the quat's OWN boresight
    bearing AND its roll, together -- the two must differ only by the SFLP
    frame's arbitrary datum, so `bearing.coef` -> 1 and `roll.coef` -> 0. On the
    same real capture this returns +0.016 for today's heading and **-0.984** for
    the pre-BUG-058 one, which is the class of defect the owner has now caught by
    eye twice.

    Read the per-axis `verdict`, not the point estimate: each is judged against
    a block-bootstrap 95% interval, so an axis the capture never exercised, or a
    drifty capture that cannot resolve the band, reads `inconclusive` rather than
    `good` or `bad`. Most captures certify only one of the two axes.

    It says the heading IS a heading; it cannot say it points at true north --
    both estimates share the quaternion and the calibration, so a rotated mag fit
    (DT0103) moves them together. That still needs ROADMAP DC-E's braced sweep.

    `cal_path` defaults to the calibration a roomscan-web on this box would load.

    Wraps `host/tools/heading_check.py::check_capture()`.
    """
    from tools.heading_check import check_capture

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    cp = Path(cal_path) if cal_path else None
    if cp is not None and not cp.is_absolute():
        cp = REPO / cp
    rep = check_capture(p, cp)
    rep["path"] = rel(p)
    return rep


@mcp.tool()
def slam_ensemble(capture: str, n: int = 10, device: str = "", voxel_size: float = 0.0,
                  block_count: int = 0, icp_mode: str = "", max_frames: int = 0,
                  include_runs: bool = True, timeout_s: int = 3600) -> dict:
    """Score a capture with an ensemble of SLAM runs -- one run is NOT a measurement.

    Frame-to-model tracking is chaotic at centimetre scale: a deliberate 3 mm
    height nudge moves a real circuit's loop closure by 0.37 m (BUG-037), and this
    tool's own validation run spread 0.477-0.966 m across ten numerically innocuous
    perturbations of the SAME capture. `ROADMAP.md` therefore requires drift figures
    be ensemble means +/- sd over ~10 perturbations. Use this, not `slam_rerender`,
    whenever a number is going to be quoted or compared.

    Perturbations are deterministic and innocuous: start one frame later, and nudge
    the ICP correspondence radius by +/-1e-4 m (0.2% of 0.05 m). Device is not
    perturbed -- mixing CPU and CUDA adds no chaos coverage and confounds timing.

    `summary.horizontal_closure_m` is the headline, and is the field
    `slam_loop_closure_gate` consumes. It is the start-to-end gap projected onto the
    horizontal plane (up is -Y here), kept separate from `vertical_error_m` because
    the two have different error sources -- vertical is barometer plus ICP drift,
    horizontal is pure odometry drift.

    Two things to check before quoting any of it. `runs_died` > 0 means at least one
    run froze its pose and dead-reckoned the tail, so its closure is where the
    estimate quit rather than drift (BUG-036). `any_saturated` means a run outgrew
    its block grid and map growth stalled mid-scan (BUG-035). And closure is only
    DRIFT if the operator actually returned to the start pose -- on a non-closing
    capture it is just the distance between two different places.

    Cost: roughly (frames x n x 7 ms) on CUDA:0, so a 5000-frame capture at n=10 is
    about 5 minutes. Drop `n` to 5 for a triage pass; keep 10 for anything a formal
    decision rides on.

    Wraps `host/tools/slam_ensemble.py::run_ensemble()`.
    """
    import json
    import tempfile

    p = (REPO / capture) if not Path(capture).is_absolute() else Path(capture)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    # Subprocess, not in-process: the ensemble holds a TSDF grid on the compute
    # device for minutes, and the MCP server must not carry that allocation.
    with tempfile.TemporaryDirectory() as td:
        out = Path(td) / "ensemble.json"
        cmd = [str(VENV_PY), str(HOST / "tools" / "slam_ensemble.py"), str(p),
               "-n", str(n), "--json", str(out), "--quiet"]
        for flag, val in (("--device", device), ("--icp-mode", icp_mode)):
            if val:
                cmd += [flag, val]
        for flag, val in (("--voxel-size", voxel_size), ("--block-count", block_count),
                          ("--max-frames", max_frames)):
            if val:
                cmd += [flag, str(val)]
        try:
            proc = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                                  capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            return {"ok": False, "cmd": " ".join(cmd),
                    "error": f"timed out after {timeout_s}s -- lower n, or bound the run "
                             "with max_frames"}
        if proc.returncode != 0 or not out.exists():
            return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                    "stdout": proc.stdout[-2000:], "stderr": proc.stderr[-2000:]}
        r = json.loads(out.read_text())

    r["ok"] = True
    r["capture"] = rel(p)
    if not include_runs:
        r.pop("runs", None)
    if r.get("runs_died"):
        r["warning"] = (
            f"{r['runs_died']}/{r['n']} runs died (worst trailing lost "
            f"{r['worst_trailing_lost']}): those runs' closure is where the estimate "
            f"froze, not measured drift (BUG-036).")
    elif r.get("worst_longest_lost_run", 0) >= 30:
        # `died` is trailing-only. A run that freezes mid-scan and then re-registers
        # passes it while carrying a dead-reckoned hole: DebugCapB2 froze for 628
        # frames (21.2 s, 15.5% of the run) and still reported died=False.
        r["warning"] = (
            f"no run died, but the worst froze for {r['worst_longest_lost_run']} frames "
            f"MID-RUN before recovering. `died` is trailing-only and does not catch that; "
            f"the frozen segment is dead-reckoned, so path length and closure are "
            f"contaminated even though the tail is healthy.")
    elif r.get("any_saturated"):
        r["warning"] = ("a run saturated its block grid: map growth stalled mid-scan "
                        "and tracking collapses after that point (BUG-035).")
    return r


@mcp.tool()
def capture_motion(path: str, hold_deg_s: float = 0.0, fast_deg_s: float = 0.0,
                   min_hold_s: float = 0.0, min_take_s: float = 0.0,
                   include_segments: bool = True) -> dict:
    """What the operator physically DID during a capture: holds, pans, whips, tilt.

    Several ROADMAP data-collection gates are conditions on motion rather than on
    data integrity -- DC-F wants "3 takes ... 10 s stationary at both ends of every
    take", DC-E "level -> 45 deg -> vertical, holding each ~15 s. Two cycles", DC-D
    "slowly panning the whole time" (a static flat-field capture is invalid: it
    bakes scene texture into the correction), DC-A "3-4 deliberate fast whips".
    This answers those directly instead of by eyeballing a replay.

    `segments` alternates `hold` and `move` runs, which IS the take/bookend
    structure; `starts_with_hold`/`ends_with_hold` are the bookend check, `takes`
    counts deliberate pans, and `fast_events` counts excursions above `fast_deg_s`
    (DC-A's gate). Per-hold `mean_tilt_deg` is what shows DC-E's tilt cycles.

    Two things it will not do. Rate is computed over the MEASURED dt, not a nominal
    1/30 s, because frames are lost on this link and a two-frame step would
    otherwise report a phantom doubling of speed. And a gap longer than `max_dt`
    is marked unmeasured rather than interpolated across -- `unmeasured_frac`
    says how much of the capture's motion is simply unknown, which on a lossy
    capture is the caveat on everything else here.

    Tilt is degrees from straight up in the SFLP quaternion's own Z-up world, the
    same convention as `capture_magcheck`'s tilt table -- 0 = pointing at the
    ceiling, 90 = horizontal. Zero-valued numeric arguments mean "use the default".

    Wraps `host/tools/capture_motion.py::describe()`.
    """
    from tools.capture_motion import (DEFAULT_FAST_DEG_S, DEFAULT_HOLD_DEG_S,
                                      DEFAULT_MIN_HOLD_S, DEFAULT_MIN_TAKE_S, describe)

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    r = describe(p,
                 hold_deg_s=hold_deg_s or DEFAULT_HOLD_DEG_S,
                 fast_deg_s=fast_deg_s or DEFAULT_FAST_DEG_S,
                 min_hold_s=min_hold_s or DEFAULT_MIN_HOLD_S,
                 min_take_s=min_take_s or DEFAULT_MIN_TAKE_S)
    r["path"] = rel(p)
    if not include_segments:
        r.pop("segments", None)
    return r


@mcp.tool()
def capture_profile_probe(path: str, requested_fps: float = 0.0,
                          udp_stats: dict | None = None) -> dict:
    """Grade a capture against the ranging profile it was supposed to run at.

    `roomscan.profiles` says what a requested configuration is EXPECTED to do;
    this is the other half — what a recorded capture ACTUALLY did. Reports
    measured median/p05/p95 RAW_3DMD inter-frame interval and the resulting fps
    against `requested_fps` at the plan's own +/-2% acceptance tolerance
    (`fps_within_tolerance`), RAW seq gaps, CRC failures, per-stream pairing of
    IMU_QUAT/ENV/IMU_SYNC/IMU_RAW against RAW_3DMD's seq (today's coupled 1:1
    assumption — a lower bound once Task 7 ships decoupled N:1 draining), and
    stream-11's effective sample rate against the 480 Hz ODR ceiling.

    `udp_stats` is NOT recoverable from the capture file itself — a datagram
    that lost a fragment is dropped before it ever reaches the recording, so
    the file alone cannot see it. Pass the live `UdpSource` counters from
    `rig_status()`'s metrics (never bind the stream to get them yourself) to
    merge `frames_incomplete`/`frags_lost`/`frags_reordered` into the report;
    omit it and the field reports `None` rather than a guess.

    Wraps `host/tools/profile_probe.py::probe()`.
    """
    from tools.profile_probe import probe

    p = (REPO / path) if not Path(path).is_absolute() else Path(path)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    rep = probe(p, requested_fps=requested_fps or None, udp_stats=udp_stats)
    rep["path"] = rel(p)
    return rep


@mcp.tool()
def profile_tuning(ranging_mode: str = "Precision", power_config: str = "Regular",
                   resolution: str = "54x42", dss: bool = True,
                   output_interface: str = "I3C", fps: int = 30,
                   exposure_ms: int = 10, ambient_lux: float = 100.0) -> dict:
    """Compare ST's ProfileTuning estimate with the rig's full-map measured model.

    Returns both sources separately and names every disagreement, including the
    unverified DSS-off I3C frame-layout assumption. It is offline only; use
    capture_profile_probe() to report what a recording actually delivered.

    `ranging_mode` is Precision|Ambient, `power_config` Regular|Low Power|Ultralow
    Power, `output_interface` I3C|CSI2|I2C_FM+|I2C_FM (only I3C is a rig
    prediction — the others are ST planning estimates for hardware paths this
    board does not wire). `ambient_lux` feeds the power model only.
    `roomscanner_full_map` is the same estimate shape `profile_estimate()`
    returns; where the two sources disagree, the measured one wins.
    """
    from tools.profile_tuning import analyze_profile
    return analyze_profile(ranging_mode=ranging_mode, power_config=power_config,
                           resolution=resolution, dss=dss,
                           output_interface=output_interface, fps=fps,
                           exposure_ms=exposure_ms, ambient_lux=ambient_lux)


def _estimate_config(profile: str, ranging_mode: str, fps: int, exposure_ms: int,
                     power_mode: str, imu_env_rate_hz: int, transport: str,
                     ambient_lux: float) -> dict:
    """Pure half of `profile_estimate` -- resolve names, then hand the whole
    thing to `roomscan.profiles`, which owns every number in the answer."""
    from roomscan import profiles

    manual_fields = {"ranging_mode": ranging_mode, "fps": fps,
                     "exposure_ms": exposure_ms, "power_mode": power_mode}
    given = {k: v for k, v in manual_fields.items() if v}

    if not profile and not given:
        # No arguments at all: describe every preset button, which is the
        # question "what do the four profiles actually do?" in one call.
        return {"ok": True, "transport": transport, "presets": {
            profiles.PROFILE_ID_TO_STR[pid]: profiles.estimate_to_json(
                profiles.estimate_preset(pid, transport=transport,
                                         imu_env_rate_hz=imu_env_rate_hz or None,
                                         ambient_lux=ambient_lux))
            for pid in profiles.PRESETS}}

    if profile and profile not in profiles.STR_TO_PROFILE_ID:
        return {"ok": False, "errors": [
            f"unknown profile {profile!r}; expected one of "
            f"{sorted(profiles.STR_TO_PROFILE_ID)}"]}

    pid = profiles.STR_TO_PROFILE_ID.get(profile)
    if pid is not None and pid is not profiles.ProfileId.MANUAL and not given:
        est = profiles.estimate_preset(pid, transport=transport,
                                       imu_env_rate_hz=imu_env_rate_hz or None,
                                       ambient_lux=ambient_lux)
        return {"ok": est.ok, "kind": "preset", "profile": profile,
                "transport": transport, "estimate": profiles.estimate_to_json(est),
                "errors": list(est.errors), "warnings": list(est.warnings)}

    # Manual: every field is required, because an estimate over a config the
    # caller only half specified would be an estimate of a config nobody asked
    # for. (A preset name plus overrides is deliberately NOT supported --
    # `rig_profile` cannot send that either; the device takes whole candidates.)
    missing = [k for k, v in manual_fields.items() if not v]
    if missing:
        return {"ok": False, "errors": [
            f"manual estimate needs all of ranging_mode/fps/exposure_ms/power_mode; "
            f"missing {missing}"]}
    rm = profiles.STR_TO_RANGING_MODE.get(ranging_mode)
    pm = profiles.STR_TO_POWER_MODE.get(power_mode)
    if rm is None or pm is None:
        return {"ok": False, "errors": [
            f"ranging_mode must be one of {sorted(profiles.STR_TO_RANGING_MODE)} and "
            f"power_mode one of {sorted(profiles.STR_TO_POWER_MODE)}; "
            f"got {ranging_mode!r}/{power_mode!r}"]}

    params = profiles.ManualParams(rm, int(fps), int(exposure_ms), pm,
                                   imu_env_rate_hz or None)
    est = profiles.estimate_manual(params, transport=transport, ambient_lux=ambient_lux)
    return {"ok": est.ok, "kind": "manual", "profile": "manual", "transport": transport,
            "estimate": profiles.estimate_to_json(est),
            "errors": list(est.errors), "warnings": list(est.warnings)}


@mcp.tool()
def profile_estimate(profile: str = "", ranging_mode: str = "", fps: int = 0,
                     exposure_ms: int = 0, power_mode: str = "",
                     imu_env_rate_hz: int = 0, transport: str = "ethernet",
                     ambient_lux: float = 100.0) -> dict:
    """Predict what a ranging configuration would do, without touching the device.

    The offline half of `rig_profile()`: same `roomscan.profiles` model the
    server and the web UI use, so what this predicts is exactly what the Device
    card would show. Call it BEFORE applying a config to see whether it is even
    valid and what it costs.

    Pass `profile` (stability|precision|high_framerate) for a preset, or all
    four manual fields — `ranging_mode` (ambient|precision), `fps` 1-100,
    `exposure_ms` 1-16, `power_mode` (ulp|lp|regular) — for a candidate. With no
    arguments it estimates every preset at once. `imu_env_rate_hz` is 0/omitted
    for coupled-to-ToF, else 1-480. `transport` is ethernet|cdc|replay and only
    feeds the non-blocking CDC-above-60-fps warning; `ambient_lux` feeds the
    power model and nothing else.

    Read `estimate.expected_delivered_fps`, not `fps`: above an exposure's
    measured 1x ceiling the sensor ACCEPTS the request and then delivers
    period-multiples (measured 2026-08-03), so a 90 fps request at 2 ms is
    really ~45 fps. `errors` mean the config is invalid and would be refused
    host-side; `warnings` mean it is applicable but will not do what it looks
    like it does. What a recording ACTUALLY delivered is a different question --
    that is `capture_profile_probe()`.
    """
    return _estimate_config(profile, ranging_mode, fps, exposure_ms, power_mode,
                            imu_env_rate_hz, transport, ambient_lux)


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


@mcp.tool()
def slam_stall_profile(capture: str, frames: int = 1500, device: str = "",
                       decimate: bool = False, mesh_every: int = 5,
                       block_count: int = 0, timeout_s: float = 1800.0) -> dict:
    """Find which live-SLAM stage freezes the server, and for how long (BUG-060).

    Replays a capture through the real pipeline (Mapper.step -> mapper.mesh() ->
    prepare_packet -> pack_mesh) and reports, per stage, both the wall time and
    the GIL starvation a watchdog thread measured. Read the starvation numbers,
    not the wall time: these are native calls and Open3D holds the GIL, so a
    stage's starvation IS how long `roomscan-web`'s asyncio loop was frozen --
    and the two differ by an order of magnitude. `prepare_packet` costs 178 ms
    p50 but only 11.9% of wall in starvation (mostly numpy, which releases the
    GIL); with `decimate=True` the same stage costs 2440 ms p50 and 94.3%.

    Run it at SCALE. 1200 frames of a near-static capture showed zero stalls on
    code that freezes for 1261 ms on a real room sweep -- every cost here grows
    with map size, so pick a capture with real operator motion.

    Runs in a subprocess (it holds a TSDF grid on the GPU for minutes) and never
    binds the device, so it is safe beside a live `roomscan-web` -- though both
    will be slower while it runs.

    Wraps `host/tools/slam_stall_profile.py::profile_capture()`.
    """
    import json

    p = (REPO / capture) if not Path(capture).is_absolute() else Path(capture)
    if not p.is_file():
        return {"error": f"no such capture: {rel(p)}"}
    cmd = [str(VENV_PY), str(HOST / "tools" / "slam_stall_profile.py"), str(p),
           "--frames", str(frames), "--mesh-every", str(mesh_every), "--json"]
    if device:
        cmd += ["--device", device]
    if block_count:
        cmd += ["--block-count", str(block_count)]
    if decimate:
        cmd += ["--decimate"]
    try:
        proc = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return {"ok": False, "cmd": " ".join(cmd),
                "error": f"timed out after {timeout_s}s -- lower `frames`"}
    if proc.returncode != 0:
        return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                "stdout": proc.stdout[-2000:], "stderr": proc.stderr[-2000:]}
    rep = json.loads(proc.stdout)
    rep["ok"] = True
    rep["capture"] = rel(p)
    if rep.get("worst_stall_ms", 0) >= 300:
        worst = max(rep["gil_starvation"].items(),
                    key=lambda kv: kv[1]["starved_s"])[0]
        rep["warning"] = (
            f"worst single freeze {rep['worst_stall_ms']} ms, {rep['starved_pct_of_wall']}% "
            f"of wall starved; the biggest contributor is the {worst!r} stage. A freeze "
            f"this long stops the broadcaster, plain HTTP, and the SLAM feed alike.")
    return rep


@mcp.tool()
def slam_icp_bench(capture: str = "", what: str = "all", frames: int = 400,
                   raycast_frames: int = 200, ensemble_n: int = 10,
                   device: str = "", ab_pairs: int = 4, ab_frames: int = 0,
                   baseline_icp_device: str = "", candidate_icp_device: str = "CPU:0",
                   timeout_s: float = 5400.0) -> dict:
    """Matched device benchmark for the SLAM ICP solve and the raycast round-trips.

    Answers "would moving this to the GPU (or off it) help?" with measurements
    rather than intuition, on ONE device over the SAME recorded ICP inputs.
    `what` selects the pass:

    * `api` -- which Open3D 0.19 tensor ops force a host synchronization. Run
      this FIRST before designing any device-resident code. On the installed
      build `sum(dim=)`, boolean-mask indexing, `nonzero()`, `.item()`, every
      linalg entry point AND `nns.hybrid_search` all sync; elementwise ops,
      `matmul`, `T()`, gathers, `concatenate` and uploads do not. A call
      returning a device tensor is NOT evidence that no sync happened, which is
      why this probes against a deep CUDA queue instead of just timing the op.
    * `icp` -- four solvers on identical (source, model, init) triples recorded
      from a real replay: the shipped `translation`, Open3D tensor `6dof`, a
      fully GPU-resident translation solve, and the shipped translation with its
      NN index on the host. Reports wall time, GIL-held blocking cost, and a
      per-call equivalence check against the shipped solver (round-off is
      ~1e-16 m; a real algorithmic difference lands many orders above that).
    * `raycast` -- the cost of `TsdfMap.raycast()`'s download/mask/re-upload and
      of `Mapper.step()`'s second download of the same positions just to count
      them, against a device-resident alternative.
    * `ensemble` -- accuracy, not speed: a matched perturbation ensemble per
      variant plus a NON-INFERIORITY gate (paired bootstrap CI inside one
      standard deviation of the BASELINE ensemble's own closure). Minutes per
      variant; this is the pass a decision rides on.
    * `ab` -- interleaved, paired, WHOLE-PIPELINE A/B of `Mapper.icp_device`
      (item 5): replays the entire capture through the shipped `Mapper` once
      per arm, `ab_pairs` times, alternating which arm runs first so warm-up
      and box drift bias both arms equally. Use this, not `icp`, to size the
      change -- the isolated microbenchmark swung 43% between sessions on
      identical inputs. Reports the per-pair spread (never one number), a
      whole-trajectory equivalence check, `tick_share`, and a CPU-load + GPU
      sample around every arm. `baseline_icp_device` empty = "follow the
      compute device", i.e. the pre-item-5 behaviour.

    The GPU-resident variant lives in the tool, not in `roomscan.slam.odometry`
    -- the shipped ICP path is deliberately untouched so a later before/after
    stays interpretable.

    Runs in a subprocess and never binds the device, but it does allocate a TSDF
    on the compute device, so it competes with a live `roomscan-web` for the GPU.

    Wraps `host/tools/slam_icp_bench.py::run()`.
    """
    import json

    cmd = [str(VENV_PY), str(HOST / "tools" / "slam_icp_bench.py")]
    if capture:
        p = (REPO / capture) if not Path(capture).is_absolute() else Path(capture)
        if not p.is_file():
            return {"error": f"no such capture: {rel(p)}"}
        cmd.append(str(p))
    elif what != "api":
        return {"error": f"what={what!r} needs a capture; only what='api' runs without one"}
    cmd += ["--what", what, "--frames", str(frames),
            "--raycast-frames", str(raycast_frames), "-n", str(ensemble_n),
            "--ab-pairs", str(ab_pairs),
            "--candidate-icp-device", candidate_icp_device]
    if ab_frames:
        cmd += ["--ab-frames", str(ab_frames)]
    if baseline_icp_device:
        cmd += ["--baseline-icp-device", baseline_icp_device]
    if device:
        cmd += ["--device", device]
    try:
        proc = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return {"ok": False, "cmd": " ".join(cmd),
                "error": f"timed out after {timeout_s}s -- lower `frames` or `ensemble_n`"}
    if proc.returncode != 0:
        return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                "stdout": proc.stdout[-2000:], "stderr": proc.stderr[-2000:]}
    # The tool prints progress lines before the JSON body on the ensemble pass.
    body = proc.stdout[proc.stdout.index("{"):]
    rep = json.loads(body)
    rep["ok"] = True
    return rep


@mcp.tool()
def splat_list() -> dict:
    """List the offline Gaussian-splat room reconstructions built under results/splats/.

    Each entry has the display `name`, `slug`, gaussian count, byte size, and the
    `ply_url` the web viewer loads. These are the splats the Live/View/Splat
    source offers. Light and torch-free -- it only reads manifests off disk.
    """
    from roomscan.splat import list_splats
    return {"ok": True, "splats": list_splats(str(RESULTS))}


@mcp.tool()
def splat_build(video: str, name: str, force: bool = False, fps: float = 0.0,
                iters: int = 0, max_gaussians: int = 0, timeout_s: int = 7200) -> dict:
    """Reconstruct a navigable Gaussian splat from a video (frames -> COLMAP -> 3DGS).

    This is the OFFLINE, standalone reconstruction: a *rough* room from phone video
    alone, with no ToF fusion or hand-eye calibration (that is the blocked full
    Phase 7). The result lands in `results/splats/<slug>/` and appears as the third
    Live/View/Splat source in `roomscan-web`. `name` is the display name (e.g.
    "Sam Office"); the slug is derived from it, so re-running the same name rebuilds
    that splat (a matching, current build is skipped unless `force`).

    It is a long, heavy, host-only job: COLMAP structure-from-motion runs on CPU and
    3DGS training runs on the GPU (gsplat/CUDA), typically many minutes -- so it runs
    as a subprocess (`roomscan-splat`), never in the server process, and reads the
    run's `--json` report rather than scraping stdout. gsplat JIT-compiles its CUDA
    kernels the first time on a fresh machine, which needs a system `nvcc` + `ninja`
    on PATH (`CUDA_HOME=/usr/local/cuda`); this tool adds those best-effort.

    Zero/omitted `fps`/`iters`/`max_gaussians` mean "use the [splat] preset default".
    A returned `built: false` with `ok: true` means a current splat already existed;
    pass `force=true` to rebuild. Check `stats.registered_ratio`: a low value means
    COLMAP could register few frames (textureless scene / too-fast motion) and the
    splat will be sparse.
    """
    import json
    import os
    import tempfile

    vid = Path(video)
    if not vid.is_absolute():
        for base in (REPO, CAPTURES, RECORDINGS):
            if (base / video).exists():
                vid = base / video
                break
    if not vid.exists():
        return {"ok": False, "error": f"video not found: {video}"}

    env = dict(os.environ)
    cuda = next((p for p in ("/usr/local/cuda", "/usr/local/cuda-13.3") if Path(p).exists()), None)
    if cuda:
        env["CUDA_HOME"] = cuda
        env["PATH"] = f"{Path(VENV_PY).parent}:{cuda}/bin:" + env.get("PATH", "")

    with tempfile.TemporaryDirectory() as tmp:
        report_path = Path(tmp) / "splat.json"
        cmd = [str(VENV_PY), "-m", "roomscan.splat.cli", "--results-dir", str(RESULTS),
               "--json", str(report_path), "build", str(vid), "--name", name]
        if force:
            cmd.append("--force")
        for flag, value in (("--fps", fps), ("--iters", iters), ("--max-gaussians", max_gaussians)):
            if value:
                cmd += [flag, str(value)]
        try:
            p = subprocess.run(cmd, cwd=str(REPO), env=env, timeout=timeout_s,
                               capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            return {"ok": False, "cmd": " ".join(cmd),
                    "error": f"timed out after {timeout_s}s -- raise timeout_s or lower iters/fps"}
        if not report_path.exists():
            return {"ok": False, "cmd": " ".join(cmd), "returncode": p.returncode,
                    "stdout": p.stdout[-2000:], "stderr": p.stderr[-2000:]}
        report = json.loads(report_path.read_text())
    return report


@mcp.tool()
def splat_compare(capture: str, reference: str = "", opacity_min: float = 0.5,
                  voxel: float = 0.03, allow_scale: bool = False,
                  timeout_s: int = 900) -> dict:
    """Diff a capture's SLAM reconstruction against a ground-truth splat (reverse Phase 7).

    Instead of using the splat to improve the lidar, this treats a metric splat as
    GROUND TRUTH to expose where our SLAM got the room SHAPE wrong -- e.g. BUG-084's
    map fork on `officeFullScanAug6`, where the ceiling drops mid-scan and a second
    displaced room appears. It rigidly aligns the two (both are metric, so a
    scale/extent mismatch is a finding, not fit away) and reports alignment
    fitness/RMSE, per-axis bounding-box extents + ratio, floor footprint, bidirectional
    cloud-to-cloud distance, and a vertical/ceiling-fork analysis.

    Needs the capture's Detailed-SLAM mesh (`results/<stem>.ply`) to exist -- build it
    first with `slam_rerender`/Detailed SLAM if missing. With `reference` empty the best
    *imported* ground-truth splat for this capture is auto-selected by name match (NOT
    hardcoded to one splat; see `report.reference_selection`) -- pass a `.ply` path or a
    `results/splats/` slug to override. Ground truth means an imported external splat
    (e.g. Scaniverse); our own video builds are too rough to be truth.
    `opacity_min` drops low-opacity splat floaters; `voxel` sets the downsample. Writes
    `results/compare/<stem>__vs__<ref>/` (overlay.ply, error_heatmap.ply, floorplan.png,
    elevation.png, report.json) -- the PNGs are served at `/results/compare/...`.

    Runs as a subprocess (open3d, minutes on a big map), reading the `--json` report.
    Key fields: `alignment.fitness` (low = the scan does not rigidly match truth),
    `vertical.fork_suspected` / `vertical.scan_height_modes` (BUG-084), and
    `extent_obb_m.ratio_scan_over_ref` (how far the scanned dimensions drift).
    """
    import json
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        report_path = Path(tmp) / "compare.json"
        cmd = [str(VENV_PY), "-m", "roomscan.splat.cli", "--results-dir", str(RESULTS),
               "--json", str(report_path), "compare", capture,
               "--opacity-min", str(opacity_min), "--voxel", str(voxel)]
        if reference:
            cmd += ["--reference", reference]
        if allow_scale:
            cmd.append("--allow-scale")
        try:
            p = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                               capture_output=True, text=True)
        except subprocess.TimeoutExpired:
            return {"ok": False, "cmd": " ".join(cmd),
                    "error": f"timed out after {timeout_s}s -- raise timeout_s or voxel"}
        if not report_path.exists():
            return {"ok": False, "cmd": " ".join(cmd), "returncode": p.returncode,
                    "stdout": p.stdout[-2000:], "stderr": p.stderr[-2000:]}
        return json.loads(report_path.read_text())


@mcp.tool()
def splat_vram_sweep(video: str = "", model_dir: str = "", image_dir: str = "",
                     budget_gib: float = 0.0, margin_gib: float = 0.8,
                     reserve_gib: float = 0.0, ladder: str = "", sh_degree: int = 3,
                     long_edge: int = 0, depth_lambda: float = 0.0, worst_k: int = 0,
                     safety_factor: float = 2.0, timeout_s: int = 3600) -> dict:
    """Find the max MCMC cap_max (gaussian count) that fits VRAM on this 8 GB card.

    Answers "how dense a splat can we get away with on THIS capture?" by MEASURING
    peak VRAM on the REAL COLMAP model + REAL frames at forced gaussian counts,
    cycling every view to catch the worst-case frame's backward -- the thing that
    OOMs. This is the honest replacement for the discredited synthetic probe, which
    built a uniform cube and under-reported the true peak by ~2x. The count is
    forced to exactly N (never grown by MCMC) using the real cloud; fit is decided on
    reserved / device-wide NVML, never max_memory_allocated. IMPORTANT: the clone-at-N
    measurement is itself a LOWER BOUND on the real TRAINING peak (it skips the MCMC
    densification that fragments the allocator and raises tile overlap -- calibrated
    ~2x low against a real build), so the recommendation multiplies by `safety_factor`
    (default 2.0). The ground truth is still a real build's `vram=` log.

    Give a `video` (runs frames+SfM once) OR an existing `model_dir` + `image_dir`
    (skips SfM). Zero args mean the preset default. `budget_gib=0` -> NVML total
    minus `margin_gib` (CUDA context + fragmentation) minus `reserve_gib` (headroom
    for a co-resident process; 0 = the build runs isolated, which it does). Match
    `long_edge`/`sh_degree`/`depth_lambda` to the target build or the cap is
    meaningless. `worst_k>0` measures only the k nearest-depth views -- faster but
    optimistic-risk (it can miss the true worst frame); default 0 = all views.

    Read `recommended_cap`, `capture_limited`, and `caveats`. A `capture_limited`
    scene (low `registered_ratio`) is SfM-bound, not VRAM-bound: a higher cap adds
    no gaussians -- run `splat_sfm_probe` instead. Heavy/GPU/minutes -> runs as a
    subprocess, never in the server process; the returned cap is the ISOLATED cap.

    Wraps `host/tools/splat_vram_sweep.py`.
    """
    import json
    import os

    cmd = [str(VENV_PY), str(HOST / "tools" / "splat_vram_sweep.py")]
    if model_dir:
        if not image_dir:
            return {"ok": False, "error": "model_dir needs image_dir"}
        cmd += ["--model-dir", model_dir, "--image-dir", image_dir]
    elif video:
        vid = Path(video)
        if not vid.is_absolute():
            for base in (REPO, CAPTURES, RECORDINGS):
                if (base / video).exists():
                    vid = base / video
                    break
        if not vid.exists():
            return {"ok": False, "error": f"video not found: {video}"}
        cmd.append(str(vid))
    else:
        return {"ok": False, "error": "give a video, or model_dir + image_dir"}

    if budget_gib:
        cmd += ["--budget-gib", str(budget_gib)]
    cmd += ["--margin-gib", str(margin_gib), "--reserve-gib", str(reserve_gib),
            "--sh-degree", str(sh_degree), "--depth-lambda", str(depth_lambda),
            "--worst-k", str(worst_k), "--safety-factor", str(safety_factor)]
    if ladder:
        cmd += ["--ladder", ladder]
    if long_edge:
        cmd += ["--long-edge", str(long_edge)]
    cmd.append("--json")

    env = dict(os.environ)
    cuda = next((p for p in ("/usr/local/cuda", "/usr/local/cuda-13.3") if Path(p).exists()), None)
    if cuda:
        env["CUDA_HOME"] = cuda
        env["PATH"] = f"{Path(VENV_PY).parent}:{cuda}/bin:" + env.get("PATH", "")
    try:
        proc = subprocess.run(cmd, cwd=str(REPO), env=env, timeout=timeout_s,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return {"ok": False, "cmd": " ".join(cmd),
                "error": f"timed out after {timeout_s}s -- shorten the ladder or raise timeout_s"}
    if proc.returncode != 0 or "{" not in proc.stdout:
        return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                "stdout": proc.stdout[-2000:], "stderr": proc.stderr[-2000:]}
    rep = json.loads(proc.stdout[proc.stdout.index("{"):])
    return rep


@mcp.tool()
def splat_sfm_probe(video: str, configs: str = "", fps: float = 0.0, max_frames: int = 0,
                    long_edge: int = 0, timeout_s: int = 5400) -> dict:
    """Find why a splat is sparse: how many frames/points SfM can actually register.

    Density has two ceilings -- VRAM (`splat_vram_sweep`) and REGISTRATION -- and
    for a room walkthrough the binding one is usually registration. `run_sfm`
    matches sequentially and keeps only the largest connected sub-model, silently
    discarding the rest; a long video thinned to 300 frames gets large inter-frame
    baselines, sequential overlap breaks, the reconstruction fragments, and most
    frames are thrown away (the new Sam Office 2 video registered only 16%).

    Extracts frames ONCE, then runs SfM under several configs on the SAME frames
    and reports, per config, `registered_ratio`, `n_submodels`, `largest_ratio`
    (largest sub-model / total -- the fragmentation this exposes), `points3D`,
    track length, reprojection error, and wall seconds. Recommends the config that
    maximizes single-connected registration. `configs` is a comma-separated subset
    of the built-in names (default: all); the built-ins vary matcher
    (sequential/exhaustive), sequential overlap, SIFT feature count, and frame
    density. Heavy CPU (COLMAP), minutes -> subprocess, never in the server process.

    Wraps `host/tools/splat_sfm_probe.py::probe_sfm()`.
    """
    import json

    vid = Path(video)
    if not vid.is_absolute():
        for base in (REPO, CAPTURES, RECORDINGS):
            if (base / video).exists():
                vid = base / video
                break
    if not vid.exists():
        return {"ok": False, "error": f"video not found: {video}"}

    cmd = [str(VENV_PY), str(HOST / "tools" / "splat_sfm_probe.py"), str(vid), "--json"]
    if configs:
        cmd += ["--configs", configs]
    for flag, value in (("--fps", fps), ("--max-frames", max_frames), ("--long-edge", long_edge)):
        if value:
            cmd += [flag, str(value)]
    try:
        proc = subprocess.run(cmd, cwd=str(REPO), timeout=timeout_s,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return {"ok": False, "cmd": " ".join(cmd),
                "error": f"timed out after {timeout_s}s -- fewer configs or raise timeout_s"}
    if proc.returncode != 0 or "{" not in proc.stdout:
        return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                "stdout": proc.stdout[-2000:], "stderr": proc.stderr[-2000:]}
    return json.loads(proc.stdout[proc.stdout.index("{"):])


@mcp.tool()
def splat_render(ply: str, out: str, transform: str = "", azimuth: float = 35.0,
                 elevation: float = 35.0, opacity_min: float = 0.0, core_pct: float = 100.0,
                 iso_scale: float = 0.0, width: int = 1280, height: int = 960,
                 views: int = 1, timeout_s: int = 600) -> dict:
    """Render a Gaussian-splat .ply to a PNG on the GPU, headless -- so results are SEEN.

    This box has no display and llvmpipe can't drive the browser splat viewer at these
    counts, so this rasterizes the splat directly with gsplat on CUDA (base/DC color)
    and writes a PNG the caller then Reads. Use it to SHOW a splat build's result
    rather than only reporting gaussian counts.

    `ply` is a splat point_cloud.ply (repo-relative ok); `out` the PNG path. Pass a
    splat `transform` (its manifest.json) for a correctly-levelled camera. The camera
    auto-frames the cloud; `azimuth`/`elevation` orbit it and `views>1` writes
    `out_<i>.png` around the scene. For a clean COVERAGE comparison (how much of the
    room reconstructed, floaters/needles removed) set `iso_scale` ~0.015 (renders every
    gaussian as a round blob), `opacity_min` ~0.3 and `core_pct` ~95 (drops the floater
    halo). Runs as a subprocess on the GPU.

    Wraps `host/tools/splat_render.py`.
    """
    import json
    import os

    p = (REPO / ply) if not Path(ply).is_absolute() else Path(ply)
    if not p.is_file():
        return {"ok": False, "error": f"no such ply: {ply}"}
    outp = (REPO / out) if not Path(out).is_absolute() else Path(out)
    cmd = [str(VENV_PY), str(HOST / "tools" / "splat_render.py"), str(p), str(outp),
           "--azimuth", str(azimuth), "--elevation", str(elevation),
           "--opacity-min", str(opacity_min), "--core-pct", str(core_pct),
           "--iso-scale", str(iso_scale), "--width", str(width), "--height", str(height),
           "--views", str(views)]
    if transform:
        tp = (REPO / transform) if not Path(transform).is_absolute() else Path(transform)
        cmd += ["--transform", str(tp)]
    env = dict(os.environ)
    cuda = next((c for c in ("/usr/local/cuda", "/usr/local/cuda-13.3") if Path(c).exists()), None)
    if cuda:
        env["CUDA_HOME"] = cuda
        env["PATH"] = f"{Path(VENV_PY).parent}:{cuda}/bin:" + env.get("PATH", "")
    try:
        proc = subprocess.run(cmd, cwd=str(REPO), env=env, timeout=timeout_s,
                              capture_output=True, text=True)
    except subprocess.TimeoutExpired:
        return {"ok": False, "cmd": " ".join(cmd), "error": f"timed out after {timeout_s}s"}
    if proc.returncode != 0:
        return {"ok": False, "cmd": " ".join(cmd), "returncode": proc.returncode,
                "stderr": proc.stderr[-2000:]}
    outs = ([str(outp.with_name(f"{outp.stem}_{i}{outp.suffix}")) for i in range(views)]
            if views > 1 else [str(outp)])
    return {"ok": True, "outputs": [rel(Path(o)) for o in outs], "gaussians_ply": rel(p)}
