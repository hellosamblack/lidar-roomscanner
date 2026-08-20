"""roomscan-slam: run frame-to-model SLAM over a recorded capture and report
trajectory + timing, optionally comparing ICP modes / KISS-ICP."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from ..decoder import StreamDecoder
from ..flatfield import FlatField
from ..pipeline import TransformStage
from ..protocol import (StreamId, FrameType, IMU_RAW_TICK_US, decode_imu_quat,
                        decode_env, decode_imu_raw, decode_imu_cal, decode_imu_sync)
from ..sensor_time import TimestampedQuaternionBuffer, signed_tick_delta
from .config import SlamConfig
from .mapper import Mapper
from . import metrics

#: Widest stream-9 gap the #155 interpolation will bridge. Beyond ~4 lost frames
#: a bracket still exists but interpolating across it is low-pass smoothing of
#: the prior, which BUG-067 showed scores as "improvement" on a tripod regardless
#: of correctness — exactly the misreading this mechanism must not reintroduce.
_INTERP_MAX_SPAN_US = 150_000.0


class _ImuAuxList(list):
    """imu_aux with an `interp_stats` side-channel (#155): still a plain list of
    (ImuRawBatch|None, quat_offset_us|None) for every existing consumer, plus a
    dict reporting what the timestamp interpolation actually did — so "flag on
    but zero usable stream-13 pairs" is visible instead of silently identical."""
    interp_stats: dict | None = None


def _load_frames(path, max_frames=None, with_imu=False, quat_interp=False):
    """Return (frames, width, height) where frames is a list of
    (depth_mm(h,w), reflectance(h,w)|None, confidence(h,w)|None, quat(4),
    pressure_pa|None, t_s). Depth/reflectance/confidence come from
    TransformStage; quat/pressure are carried forward from the latest 9/10.
    reflectance/confidence are None for sources that don't provide them (the
    on-device DEPTH_ZF32 passthrough path only ever returns "depth").

    `with_imu=True` returns a FOURTH value, `imu_aux`: a per-depth-frame list of
    (ImuRawBatch|None, quat_offset_us|None) carried forward from the latest
    stream 11 / 13 (with the stream-12 tick applied to the offset). It powers the
    accel ZUPT (BUG-069) and the quat-phase lever (BUG-067); it is OPT-IN and off
    by default so the 6-tuple frame shape and 3-value return that every existing
    caller unpacks are unchanged. `quat_offset_us` is None where no stream-13
    frame preceded (older captures), so those levers degrade to no-ops there.

    `quat_interp` (#155, requires `with_imu`) replaces the carried-forward quat
    with one SLERP-interpolated AT the depth frame's own frame-ready instant,
    from stream-9 samples timestamped by their exact-group stream 13
    (`quat_mid_ticks`, LSM clock, wrap-safe). Pairing is by exact
    (seq, t_us) — never seq alone (decoupled mode freezes seq) and never
    "latest sync". Where interpolation succeeds that frame's imu_aux offset
    becomes None so Mapper's fixed gyro rollback cannot correct it a second
    time; where it fails (no bracket, missing/invalid 13, legacy capture) the
    frame keeps the legacy carried-forward quat AND the legacy offset, so the
    old mechanism remains that frame's fallback. `quat_interp="reflected"` is a
    VALIDATION-ONLY null: it queries at the mirror image of the frame time on
    the far side of the quat midpoint — a deliberately wrong-direction shift of
    equal magnitude. If that scores as well as the real thing, a metric is
    rewarding smoothing, not phase correction (BUG-067's trap). Never ship it.
    Interpolation coverage is reported on the returned list's `interp_stats`."""
    dec = StreamDecoder()
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"),
                           flatfield=FlatField.load_configured())
    with open(path, "rb") as f:
        data = f.read()
    interp = bool(quat_interp) and with_imu
    mode = "reflected" if quat_interp == "reflected" else ("on" if interp else "off")
    frames = []
    imu_aux = _ImuAuxList()
    frame_keys = []                  # per depth frame: its (seq, t_us) group key
    quats_by_key = {}                # (seq, t_us) -> stream-9 quat of that exact group
    syncs_by_key = {}                # (seq, t_us) -> valid ImuSync of that exact group
    sample_order = []                # keys in wire order (13 is last in its group)
    stop_key = None                  # set once max_frames is reached (interp only)
    last_quat = (1.0, 0.0, 0.0, 0.0)
    last_pa = None
    last_raw = None
    last_sync = None
    last_tick_us = None
    width = height = None
    for frame in dec.feed(data):
        h = frame.header
        if h.frame_type != FrameType.DATA:
            continue
        if h.stream_id == StreamId.IMU_QUAT:
            last_quat = decode_imu_quat(frame.payload)
            if interp:
                quats_by_key.setdefault((h.seq, h.t_us), last_quat)
            continue
        if h.stream_id == StreamId.ENV:
            last_pa = decode_env(frame.payload)[0]
            continue
        if with_imu and h.stream_id == StreamId.IMU_RAW:
            try:
                last_raw = decode_imu_raw(frame.payload,
                                          tick_us=last_tick_us or decode_imu_raw.__defaults__[0])
            except Exception:
                pass
            continue
        if with_imu and h.stream_id == StreamId.IMU_CAL:
            try:
                last_tick_us = decode_imu_cal(frame.payload).tick_us
            except Exception:
                pass
            continue
        if with_imu and h.stream_id == StreamId.IMU_SYNC:
            try:
                sync = decode_imu_sync(frame.payload)
            except Exception:
                continue
            last_sync = sync
            if interp and sync.valid and sync.quat_n and sync.quat_mid_ticks:
                key = (h.seq, h.t_us)
                if key not in syncs_by_key:
                    syncs_by_key[key] = sync
                    sample_order.append(key)
            continue
        if stop_key is not None:
            # max_frames reached: the sensor handlers above keep draining the
            # final depth frame's own trailing stream 9/13 (they arrive AFTER
            # the depth payload in the group); the first transform-bound frame
            # of a different group means that group is complete.
            if (h.seq, h.t_us) != stop_key:
                break
            continue
        out = stage.feed(frame)
        if out is None:
            continue
        header, arrays = out
        depth = arrays.get("depth")
        if depth is None:
            continue
        reflectance = arrays.get("reflectance")
        confidence = arrays.get("confidence")
        width, height = header.width, header.height
        frames.append((
            depth.astype(np.float32),
            reflectance.astype(np.float32) if reflectance is not None else None,
            confidence.astype(np.float32) if confidence is not None else None,
            last_quat, last_pa, header.t_us / 1e6,
        ))
        frame_keys.append((header.seq, header.t_us))
        if with_imu:
            tick = last_tick_us if last_tick_us is not None else 21.7
            offset = last_sync.quat_offset_us(tick) if last_sync is not None else None
            imu_aux.append((last_raw, offset))
        if max_frames and len(frames) >= max_frames:
            if not interp:
                break
            stop_key = (header.seq, header.t_us)
    if interp:
        imu_aux.interp_stats = _align_quats_to_frame_time(
            frames, frame_keys, imu_aux, quats_by_key, syncs_by_key, sample_order,
            last_tick_us if last_tick_us is not None else IMU_RAW_TICK_US, mode)
    if with_imu:
        return frames, width, height, imu_aux
    return frames, width, height


def _align_quats_to_frame_time(frames, frame_keys, imu_aux, quats_by_key,
                               syncs_by_key, sample_order, tick_us, mode):
    """Post-pass of `_load_frames` (#155): walk the exact-group timed quaternion
    samples through a `TimestampedQuaternionBuffer` in wire order and, for every
    depth frame whose own group carried a valid stream 13, query the orientation
    at that frame's frame-ready instant. Mutates `frames`/`imu_aux` in place;
    returns the coverage stats dict.

    A post-pass rather than in-loop state because the group's stream 9/13 arrive
    AFTER its depth payload: at depth time the carried-forward quat belongs to
    the PREVIOUS group. The frame-ready instant sits ~5-8 ms BEFORE the paired
    quat midpoint (the lead BUG-031 measured), so frame N's target is bracketed
    by groups N-1 and N — queries and samples are both monotone, one sweep."""
    samples = [(syncs_by_key[k].quat_mid_ticks, quats_by_key[k])
               for k in sample_order if k in quats_by_key]
    stats = {"mode": mode, "frames": len(frames), "timed_samples": len(samples),
             "eligible": 0, "applied": 0}
    buf = TimestampedQuaternionBuffer(
        capacity=8, max_span_ticks=_INTERP_MAX_SPAN_US / tick_us)
    si = 0
    last_added = None
    for i, key in enumerate(frame_keys):
        sync = syncs_by_key.get(key)
        if sync is None:
            continue
        stats["eligible"] += 1
        target = sync.frame_ready_ticks(tick_us)
        if mode == "reflected":
            target = target + 2.0 * signed_tick_delta(target, sync.quat_mid_ticks)
        while si < len(samples) and (last_added is None
                                     or signed_tick_delta(last_added, target) > 0.0):
            if buf.add(samples[si][0], samples[si][1]):
                last_added = samples[si][0]
            si += 1
        quat = buf.at(target)
        if quat is None:
            continue
        f = frames[i]
        frames[i] = (f[0], f[1], f[2], quat, f[4], f[5])
        if i < len(imu_aux):
            imu_aux[i] = (imu_aux[i][0], None)
        stats["applied"] += 1
    return stats


def _load_frames_maybe_imu(path, max_frames=None, need_imu=False, quat_interp=False):
    """`_load_frames`, returning a 4th `imu_aux` value, tolerant of a caller (or
    a test double) that hands back only the legacy 3-tuple. Centralises the
    "load the raw IMU only when a lever needs it" choice for the CLI, the
    ensemble, and Detailed, so those three cannot drift. Returns
    (frames, width, height, imu_aux|None). `quat_interp` selects #155's
    timestamp alignment (see `_load_frames`); it implies nothing unless
    `need_imu` is also set by the lever that wants it."""
    if need_imu:
        # Pass quat_interp only when the lever is actually on: legacy test
        # doubles patch `_load_frames` without the #155 kwarg and must keep
        # working for the (default-off) paths they exercise.
        kwargs = {"with_imu": True}
        if quat_interp:
            kwargs["quat_interp"] = quat_interp
        loaded = _load_frames(path, max_frames, **kwargs)
    else:
        loaded = _load_frames(path, max_frames)
    if len(loaded) == 4:
        return loaded
    frames, width, height = loaded
    return frames, width, height, None


def _run(frames, width, height, cfg, mode, device=None, imu_aux=None, trace_window=None):
    # `cfg.mapper_kwargs()` is the single source for the Mapper field list
    # (BUG-062). This used to re-list all eighteen knobs by hand, which is the
    # second-construction-site shape that bug is about -- item 5 (2026-08-02)
    # added `icp_device` and it would have had to be remembered here too.
    #
    # Two deliberate overrides, both pre-existing CLI semantics:
    #   `icp_mode`: this function is called once per mode by --compare-modes.
    #   `device`:   --device, else `[slam] device` -- NOT `preferred_device()`.
    #               The offline CLI honours the configured device (default
    #               CPU:0); only the live/Detailed paths auto-select CUDA.
    kwargs = cfg.mapper_kwargs()
    kwargs.update(icp_mode=mode,
                  device=device if device is not None else cfg.device)
    mapper = Mapper(width, height, **kwargs)
    timings, ts = [], []
    trace_steps = [] if trace_window is not None else None
    for i, (depth, reflectance, confidence, quat, pa, t_s) in enumerate(frames):
        # imu_aux (BUG-067/069 levers) is opt-in and index-aligned to `frames`;
        # None everywhere when a caller did not load it, so step() sees the same
        # no-op defaults it always has.
        imu_raw = offset = None
        if imu_aux is not None and i < len(imu_aux):
            imu_raw, offset = imu_aux[i]
        step = mapper.step(depth, quat, pa, reflectance=reflectance, confidence=confidence,
                           imu_raw=imu_raw, quat_offset_us=offset)
        timings.append(step.slam_ms)
        ts.append(t_s)
        if trace_steps is not None:
            trace_steps.append(step)
    if trace_window is not None:
        mapper.icp_trace = metrics.icp_trace(
            trace_steps, ts, mapper.trajectory, *trace_window)
    return mapper, timings, ts


def _icp_trace_window(value: str) -> tuple[float, float]:
    try:
        start_s, end_s = (float(part) for part in value.split(":", 1))
    except (TypeError, ValueError):
        raise argparse.ArgumentTypeError("expected START:END seconds") from None
    if start_s < 0 or end_s < start_s:
        raise argparse.ArgumentTypeError("expected 0 <= START <= END")
    return start_s, end_s


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="roomscan-slam")
    ap.add_argument("capture")
    ap.add_argument("--icp-mode", choices=["translation", "6dof", "adaptive", "soft_prior"],
                    default=None)
    ap.add_argument("--device", default=None,
                    help='Open3D compute device, e.g. "CPU:0" or "CUDA:0" '
                         "(default: [slam].device in roomscan.toml, else CPU:0). "
                         "Only CPU:0 is testable until a CUDA-enabled Open3D build "
                         "is installed.")
    ap.add_argument("--icp-device", default=None,
                    help='Open3D device for ICP\'s nearest-neighbour index only, overriding '
                         '[slam] icp_device (default "CPU:0"). Everything else -- TSDF '
                         "integrate, raycast, the source cloud -- stays on --device. The "
                         "translation solve is already all-numpy, so a host index removes a "
                         "device round-trip rather than adding one, and is bit-identical: pass "
                         '"CUDA:0" here to restore the pre-2026-08-02 behaviour. Ignored by '
                         "--icp-mode 6dof, whose ICP must run where its point clouds live.")
    ap.add_argument("--compare-modes", action="store_true")
    ap.add_argument("--benchmark", action="store_true")
    ap.add_argument("--out-mesh", default="slam_map.ply")
    ap.add_argument("--out-traj", default="slam_traj.tum")
    ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--json", default=None, metavar="PATH",
                    help="also write the run's stats to PATH as JSON. The prose above is for "
                         "humans; this is the machine-readable front end (roomscan-mcp's "
                         "slam_rerender reads it) so nothing has to scrape stdout.")
    ap.add_argument(
        "--icp-trace", type=_icp_trace_window, metavar="START:END",
        help="include per-frame ICP fitness, RMSE, exact inlier/source counts, and vertical "
             "motion for this relative-time window in --json. The capture prefix is still "
             "processed because frame-to-model ICP depends on it; only output is windowed.",
    )
    ap.add_argument("--voxel-size", type=float, default=None,
                    help="TSDF voxel size in metres, overriding [slam] voxel_size. The live "
                         "scan is only a preview -- the capture holds raw frames, so re-running "
                         "offline at a finer voxel is how you get a high-detail map. Note the "
                         "sensor samples ~36 mm between rays at 2 m, so below ~5 mm the extra "
                         "detail comes only from multi-view fusion, not from a single view.")
    ap.add_argument("--block-count", type=int, default=None,
                    help="TSDF grid capacity, overriding [slam] block_count. Blocks scale as "
                         "1/voxel_size^2, so halving the voxel needs ~4x this. Give it real "
                         # NB "%%": argparse %-expands help strings, and a bare "% o" is a
                         # valid octal conversion -- this line alone made --help crash with
                         # "%o format: an integer is required, not dict" (found 2026-07-30).
                         "headroom: a scan that ran at ~97%% of its capacity stalled and lost "
                         "tracking (BUG-035). Beyond ~6 GiB use --device CPU:0, where system "
                         "RAM rather than VRAM is the limit.")
    ap.add_argument("--baro-authority", type=float, default=None,
                    help="barometer's share of the low-passed height disagreement, overriding "
                         "[slam] baro_authority; 0 turns the height constraint off. Provided so "
                         "the default can be RE-measured rather than argued about -- but note "
                         "single-run comparisons below ~0.3 m are chaos, not signal (a 3 mm "
                         "one-shot height nudge moves the final height error by 146 mm and the "
                         "loop closure by 0.37 m on a real circuit). Sweep it across an ensemble "
                         "of innocuous perturbations, not one run. See BUG-037.")
    ap.add_argument("--max-iter", type=int, default=None,
                    help="ICP iterations for this run. Values above the measured six-iteration "
                         "baseline must be ensemble-validated before Detailed SLAM adopts them.")
    args = ap.parse_args(argv)

    cfg = SlamConfig.load()
    if args.baro_authority is not None:
        cfg.baro_authority = args.baro_authority
    if args.voxel_size is not None:
        cfg.voxel_size = args.voxel_size
    if args.block_count is not None:
        cfg.block_count = args.block_count
    if args.max_iter is not None:
        cfg.max_iter = max(1, int(args.max_iter))
    if args.icp_device is not None:
        cfg.icp_device = args.icp_device
    # Load the raw IMU (streams 11/13) only when a lever consumes it (ZUPT is on
    # by default), so the CLI's ZUPT is not a silent no-op while Live/Detailed use it.
    need_imu = bool(cfg.zupt_enabled or cfg.apply_quat_phase)
    frames, width, height, imu_aux = _load_frames_maybe_imu(
        args.capture, args.max_frames, need_imu, quat_interp=cfg.apply_quat_phase)
    if not frames:
        print("[slam] no depth frames decoded from capture", file=sys.stderr)
        return 1
    print(f"[slam] {len(frames)} frames, {width}x{height}, "
          f"voxel {cfg.voxel_size * 1000:g} mm, capacity {cfg.block_count} blocks")
    interp_stats = getattr(imu_aux, "interp_stats", None)
    if interp_stats is not None:
        # #155: say what the timestamp alignment actually did — a lever that is
        # on but found zero usable stream-13 pairs must not read as applied.
        print(f"[slam] quat interpolation ({interp_stats['mode']}): "
              f"{interp_stats['applied']}/{interp_stats['eligible']} frames aligned, "
              f"{interp_stats['timed_samples']} timed samples")

    modes = ["translation", "6dof"] if args.compare_modes else [args.icp_mode or cfg.icp_mode]
    results = {}
    report = {"capture": args.capture, "frames": len(frames),
              "width": width, "height": height,
              "voxel_size": cfg.voxel_size, "block_count": cfg.block_count,
              "device": args.device or cfg.device,
              # Reported, not just applied: a run that used a different ICP
              # index device is not comparable with one that did not, and the
              # JSON is what `slam_rerender` and any later A/B reads.
              "icp_device": cfg.icp_device, "quat_interp": interp_stats, "modes": {}}
    for mode in modes:
        trace_kw = {"trace_window": args.icp_trace} if args.icp_trace is not None else {}
        mapper, timings, ts = _run(frames, width, height, cfg, mode, device=args.device,
                                   imu_aux=imu_aux, **trace_kw)
        tstats = metrics.trajectory_stats(mapper.trajectory)
        divergence = metrics.baro_divergence_stats(
            mapper.trajectory, [frame[4] for frame in frames], ts)
        mstats = metrics.timing_stats(timings)
        results[mode] = (mapper, tstats, mstats, ts)
        print(f"\n=== mode={mode} ===")
        print(f"  trajectory: n={tstats['n']} path={tstats['path_length_m']:.3f} m "
              f"gap={tstats['start_end_gap_m']:.3f} m "
              f"(horizontal={tstats['horizontal_gap_m']:.3f} m, "
              f"vertical={tstats['vertical_gap_m']:+.3f} m) "
              f"max_step={tstats['max_step_m']:.3f} m")
        if divergence["diverged"]:
            print(f"  vertical drift: FLAGGED at {divergence['first_trigger_s']:.1f} s "
                  f"(peak disagreement {divergence['peak_abs_m']:.3f} m)")
        print(f"  timing: median={mstats['median_ms']:.1f} ms p90={mstats['p90_ms']:.1f} "
              f"p99={mstats['p99_ms']:.1f} max={mstats['max_ms']:.1f} "
              f"over35ms={mstats['over_budget_frac']*100:.1f}% lost={mapper.tracking_lost_count}")
        kstats = metrics.tracking_stats(mapper.lost_flags)
        died = ("  <-- THE RUN DIED: the tail is a frozen dead-reckoned pose, "
                "not a measured trajectory" if kstats["died"] else "")
        # BUG-037: say how much of the reported height came from the barometer
        # rather than from ICP, so a run that was pulled by a drifting baro
        # says so instead of quietly reporting it as measured motion.
        print(f"  baro: correction={mapper.baro_correction_m * 1000:+.0f} mm "
              f"(authority={cfg.baro_authority:g}, tau={cfg.baro_tau_frames} frames)")
        print(f"  tracking: lost={kstats['lost']}/{kstats['n']} "
              f"({kstats['lost_frac']*100:.1f}%) longest_run={kstats['longest_lost_run']} "
              f"trailing={kstats['trailing_lost']} "
              f"icp_escalations={mapper.icp_escalations}{died}")
        # BUG-035: report against the CONFIGURED capacity, not the live one --
        # the grid rehashes to grow, so live capacity always looks roomy; what
        # predicted the failure was running near the value it was built with.
        used, live_cap = mapper._tsdf.block_usage()
        cap = cfg.block_count
        saturated = used >= 0.97 * cap
        note = ("  <-- outgrew the configured capacity; give it headroom via --block-count"
                if saturated else "")
        print(f"  map: {used} blocks, {100.0 * used / cap:.0f}% of the configured {cap} "
              f"(live grid capacity {live_cap}){note}")
        mode_report = {
            "trajectory": dict(tstats), "timing": dict(mstats),
            "vertical_divergence": divergence,
            "tracking_lost": mapper.tracking_lost_count,
            "tracking": dict(kstats),
            "icp_escalations": mapper.icp_escalations,
            "baro": {"correction_m": mapper.baro_correction_m,
                     "authority": cfg.baro_authority,
                     "tau_frames": cfg.baro_tau_frames},
            "map": {"blocks": used, "capacity": cap, "live_capacity": live_cap,
                    "percent_of_capacity": round(100.0 * used / cap, 1),
                    "saturated": bool(saturated)},
        }
        if args.icp_trace is not None:
            mode_report["icp_trace"] = mapper.icp_trace
        report["modes"][mode] = mode_report

    chosen = modes[0]
    mapper, _, _, ts = results[chosen]
    import open3d as o3d
    # mesh() may live on a non-CPU compute device (--device); write_triangle_mesh
    # is host-side I/O -- .cpu() is a no-op when it's already on CPU.
    o3d.t.io.write_triangle_mesh(args.out_mesh, mapper.mesh().cpu())
    metrics.write_tum(args.out_traj, ts, mapper.trajectory)
    print(f"\n[slam] wrote {args.out_mesh} and {args.out_traj} (mode={chosen})")

    report["chosen_mode"] = chosen
    report["out_mesh"], report["out_traj"] = args.out_mesh, args.out_traj

    if args.benchmark:
        depths = [d for d, _, _, _, _, _ in frames]
        kiss = metrics.compare_kiss(depths, mapper._intr, cfg.fov_h, cfg.fov_v)
        if kiss:
            print(f"[slam] KISS-ICP: path={kiss['path_length_m']:.3f} m "
                  f"gap={kiss['start_end_gap_m']:.3f} m")
            report["kiss_icp"] = dict(kiss)

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(f"[slam] wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
