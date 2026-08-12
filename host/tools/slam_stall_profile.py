"""Where does a live-SLAM second go? Stage timing + GIL starvation (BUG-060).

    host/.venv/bin/python host/tools/slam_stall_profile.py captures/roomSweepFull20260730.bin
    host/.venv/bin/python host/tools/slam_stall_profile.py <capture> --frames 1500 --json

Replays a capture through the exact live pipeline -- `Mapper.step` ->
`mapper.mesh()` -> `meshprep.prepare_packet` -> `web.pack_mesh` -- and times each
stage, while a watchdog thread records how late its own fixed-interval ticks
fire. That lateness is the number that matters: every one of these stages is a
native call, and Open3D holds the GIL throughout, so a stage's lateness IS how
long `roomscan-web`'s asyncio event loop was frozen. Requested by ROADMAP 6.I
("the GIL-starvation watchdog ... what turns 'the UI feels frozen' into a
number").

WHY LATENESS AND NOT WALL TIME. They are different by an order of magnitude and
the difference is the whole diagnosis. On 1500 frames of roomSweepFull,
`prepare_packet` costs 178 ms p50 of wall time but only 11.9% of wall in
starvation with a 38.8 ms worst stall -- because most of it is numpy, which
releases the GIL. Wall time alone would have condemned it; it is fine. Turn
`--decimate` on and the same stage goes to 2440 ms p50 (max 8.4 s) and **94.3%
starvation** with an 8.2 s worst stall, because `simplify_quadric_decimation` is
C++ that never lets go. Same pipeline, same capture; one of those two freezes
the server and the other does not, and only the watchdog can tell you which.

READ `tick_share`, NOT JUST `starved_pct_of_wall` (BUG-063). The watchdog's own
lateness SAMPLES can go missing exactly when starvation is worst: a stage that
holds the GIL almost completely leaves the watchdog thread almost no chance to
tick at all, so there is almost nothing to sum into `starved_s` -- measured on
the twin instrument (`slam_icp_bench.py`), 1 tick landed in 10.93 s where
~2186 were due, and the summed-lateness percentage read *lower* than a stage
that ran freely. `tick_share` = observed ticks / expected ticks
(`stage_wall_s / _WATCHDOG_PERIOD_S`) is the fix: high `tick_share` (near 1.0)
means the watchdog was regularly scheduled and `starved_pct_of_wall` can be
trusted; low `tick_share` means other Python threads (this one, the asyncio
loop, the reader) were prevented from running, and a low legacy starvation
percentage in that regime is not evidence of health -- it is evidence the
instrument barely sampled. Both the per-stage `gil_starvation[...]` entries and
the top-level report carry `ticks`/`expected_ticks`/`tick_share` alongside the
legacy fields.

MEASUREMENT NOTES

* Run it at SCALE. 1200 frames of a near-static capture showed max lateness
  32 ms and zero stalls on code that stalls for 1261 ms on a real room sweep --
  every one of these costs grows with map size, so a small map proves nothing.
  Prefer a capture with real operator motion and >=1500 frames.
* The rig is serial on purpose. `roomscan-web` runs these stages on three
  threads, but the GIL serialises them anyway, so a serial rig attributes each
  millisecond of starvation to the stage that caused it. Use it to find WHICH
  stage is expensive, not to predict end-to-end throughput.
* Nothing here binds the device. It replays a file and builds its own `Mapper`,
  so it is safe to run beside a live `roomscan-web` -- though it will compete
  for the same GPU, so expect both to be a little slower while it runs.
* `--device CPU:0` is the useful A/B when a result looks CUDA-specific.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

_DEFAULT_FRAMES = 1500
_DEFAULT_MESH_EVERY = 5          # == slam.worker._MESH_EVERY, the live cadence
_WATCHDOG_PERIOD_S = 0.005
_STALL_S = 0.3                   # "a stall" = a freeze a human would see
_STAGES = ("step", "mesh", "prep", "pack")


class GilWatchdog:
    """Records how late its own fixed-interval ticks fire, tagged by stage.

    A pure-Python thread cannot run while another thread holds the GIL, so its
    tick lateness is a direct measurement of GIL starvation -- which for
    `roomscan-web` is exactly how long the asyncio event loop could not run.
    `stage` is written by the profiling thread with no lock: it is a single
    attribute store, and a tick landing on the boundary between two stages
    mis-attributes at most one 5 ms sample.
    """

    def __init__(self, period_s: float = _WATCHDOG_PERIOD_S):
        self.period_s = float(period_s)
        self.stage = "idle"
        self.late: dict[str, list[float]] = {}
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self) -> None:
        nxt = time.monotonic() + self.period_s
        while not self._stop.is_set():
            now = time.monotonic()
            if now < nxt:
                time.sleep(nxt - now)
                now = time.monotonic()
            self.late.setdefault(self.stage, []).append(max(0.0, now - nxt))
            nxt = now + self.period_s

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def report(self, wall_s: float, wall_by_stage: dict[str, float] | None = None) -> dict:
        """Per-stage starvation, including tick-coverage (BUG-063).

        `tick_share` is the field to read, NOT `starved_pct_of_wall`. Summing
        tick lateness silently UNDER-reports the worst case: if a stage holds
        the GIL so completely that this thread barely gets to run at all,
        there are almost no samples to sum, so a near-total freeze can report
        a *lower* starvation percentage than a stage that ran freely --
        measured on `slam_icp_bench.py`'s twin watchdog: **1 tick in 10.93 s**
        where ~2186 were due. `tick_share` = observed ticks / expected ticks
        (`stage_wall_s / period_s`) reports how many of the expected ticks
        actually landed: near 1.0 means other Python threads (this one, the
        asyncio loop, the reader) got to run; near 0 means they did not,
        whatever `starved_pct_of_wall` says -- a low legacy percentage is not
        trustworthy when `tick_share` is low. `wall_by_stage` is optional only
        for backward compatibility with old callers; without it every stage's
        `expected_ticks`/`tick_share` degrade to 0/None rather than raising.
        Same semantics and rounding as `slam_icp_bench.py`'s `_Watchdog.report`.
        """
        wall_by_stage = wall_by_stage or {}
        out = {}
        for stage, samples in self.late.items():
            total = sum(samples)
            stage_wall = wall_by_stage.get(stage, 0.0)
            expected = stage_wall / self.period_s if stage_wall else 0.0
            out[stage] = {
                "ticks": len(samples),
                "expected_ticks": int(expected),
                "tick_share": round(len(samples) / expected, 4) if expected else None,
                "starved_s": round(total, 3),
                "starved_pct_of_wall": round(100.0 * total / wall_s, 1) if wall_s else 0.0,
                "worst_stall_ms": round(max(samples) * 1000.0, 1) if samples else 0.0,
            }
        return out


def _tick_headline(late: dict[str, list[float]], period_s: float, wall_s: float) -> dict:
    """Overall tick-coverage headline (plan item 3): total watchdog ticks
    across every stage (including idle) versus `wall_s / period_s`. This is
    the run-level companion to `GilWatchdog.report()`'s per-stage
    `tick_share` -- the number that answers "was the starvation measurement
    itself adequately sampled", independent of which stage did it."""
    total_ticks = sum(len(v) for v in late.values())
    expected = wall_s / period_s if wall_s else 0.0
    return {
        "ticks": total_ticks,
        "expected_ticks": int(expected),
        "tick_share": round(total_ticks / expected, 4) if expected else None,
    }


def _pct(sorted_vals: list[float], q: float) -> float:
    if not sorted_vals:
        return 0.0
    return sorted_vals[min(len(sorted_vals) - 1, int(q * len(sorted_vals)))]


def profile_capture(path, *, frames: int = _DEFAULT_FRAMES,
                    mesh_every: int = _DEFAULT_MESH_EVERY,
                    device: str | None = None,
                    block_count: int | None = None,
                    decimate: bool = False) -> dict:
    """Replay `path` through the live SLAM pipeline; return stage timings, GIL
    starvation per stage, and the payload the browser would have received.

    `decimate` forces MeshPrep's quadric decimation on -- the A/B that showed it
    costs 14x what it saves. Pure apart from reading the capture; builds and
    tears down its own `Mapper`, and never touches the device."""
    from roomscan.slam.cli import _load_frames
    from roomscan.slam.config import SlamConfig, preferred_device
    from roomscan.slam.mapper import Mapper
    from roomscan.slam.meshprep import prepare_packet
    from roomscan.web import pack_mesh

    cfg = SlamConfig.load()
    device = device or preferred_device()
    block_count = cfg.block_count if block_count is None else int(block_count)

    all_frames, width, height = _load_frames(str(path))
    used = all_frames[:frames] if frames else all_frames
    if not used:
        return {"error": f"no depth frames decoded from {path}"}

    # Hand-picking four `[slam]` keys here is exactly BUG-062's shape: this rig
    # exists to profile the SHIPPED pipeline, so it must be built from the same
    # single source the shipped paths use (item 5, 2026-08-02). `device` and
    # `block_count` stay as this function's own arguments.
    mapper_kwargs = cfg.mapper_kwargs()
    mapper_kwargs.update(device=device, block_count=block_count)
    mapper = Mapper(width, height, **mapper_kwargs)

    wd = GilWatchdog()
    wd.start()
    timings: dict[str, list[float]] = {k: [] for k in _STAGES}
    payloads: list[tuple[int, int, int]] = []     # (source_verts, sent_verts, bytes)
    integrated = 0
    t_start = time.monotonic()
    try:
        for depth, refl, conf, quat, pressure, _t_s in used:
            wd.stage = "step"
            t0 = time.monotonic()
            step = mapper.step(depth, quat, pressure, reflectance=refl, confidence=conf)
            timings["step"].append(time.monotonic() - t0)
            if step.tracking_lost:
                wd.stage = "idle"
                continue
            integrated += 1
            if not (integrated == 1 or integrated % mesh_every == 0):
                wd.stage = "idle"
                continue

            wd.stage = "mesh"
            t0 = time.monotonic()
            mesh = mapper.mesh()
            timings["mesh"].append(time.monotonic() - t0)

            wd.stage = "prep"
            t0 = time.monotonic()
            pkt = prepare_packet(mesh, wall_mode="split", glow_origin=None,
                                 mesh_seq=integrated,
                                 vertex_budget=cfg.live_vertex_budget,
                                 decimate=decimate)
            timings["prep"].append(time.monotonic() - t0)

            wd.stage = "pack"
            t0 = time.monotonic()
            wire = pack_mesh(pkt)
            timings["pack"].append(time.monotonic() - t0)
            payloads.append((pkt.source_vertex_count,
                             len(pkt.non_wall_verts) + len(pkt.wall_verts),
                             len(wire)))
            wd.stage = "idle"
    finally:
        wd.stage = "idle"
        wall = time.monotonic() - t_start
        wd.stop()

    stages = {}
    for name in _STAGES:
        vals = sorted(timings[name])
        if not vals:
            continue
        stages[name] = {
            "n": len(vals),
            "p50_ms": round(_pct(vals, 0.5) * 1000.0, 1),
            "p90_ms": round(_pct(vals, 0.9) * 1000.0, 1),
            "max_ms": round(vals[-1] * 1000.0, 1),
            "total_s": round(sum(vals), 2),
            "pct_of_wall": round(100.0 * sum(vals) / wall, 1) if wall else 0.0,
        }

    # Per-stage wall time straight off the SAME timing samples used for
    # `stages` above (plan item 2), plus `idle` as the remaining session wall
    # time -- not "idle ticks vs the whole run", but "idle ticks vs the time
    # actually spent idle", so its tick coverage means the same thing as every
    # other stage's.
    wall_by_stage = {name: sum(timings[name]) for name in _STAGES if timings[name]}
    wall_by_stage["idle"] = max(0.0, wall - sum(wall_by_stage.values()))

    starvation = wd.report(wall, wall_by_stage)
    all_late = [x for samples in wd.late.values() for x in samples]
    stalls = [x for x in all_late if x > _STALL_S]
    headline = _tick_headline(wd.late, wd.period_s, wall)
    return {
        "capture": str(path),
        "device": device,
        # Read off the built Mapper, not off `cfg`: "what was configured" and
        # "what was constructed" are different claims (BUG-062's lesson), and a
        # profile taken with a different ICP index device is not comparable.
        "icp_device": mapper.icp_device,
        "block_count": block_count,
        "decimate": bool(decimate),
        "mesh_every": int(mesh_every),
        "frames": len(used),
        "frames_integrated": integrated,
        "width": width,
        "height": height,
        "wall_s": round(wall, 1),
        "effective_fps": round(integrated / wall, 2) if wall else 0.0,
        "stages": stages,
        "gil_starvation": starvation,
        # The headline pair. `starved_pct_of_wall` is how much of the session
        # the event loop could not run at all; `worst_stall_ms` is the longest
        # single freeze a user would have seen. NEITHER is trustworthy on its
        # own when `tick_share` (below) is low: BUG-063 measured a case where
        # near-total starvation produced almost no samples to sum, so the
        # summed-lateness percentage read LOWER than a stage that ran freely.
        "starved_pct_of_wall": round(100.0 * sum(all_late) / wall, 1) if wall else 0.0,
        "worst_stall_ms": round(max(all_late) * 1000.0, 1) if all_late else 0.0,
        # Overall tick-coverage headline (BUG-063 / issue #74): observed vs.
        # expected watchdog ticks across the whole run. High tick_share means
        # the watchdog (and everything else Python -- asyncio, the reader) was
        # regularly scheduled; low tick_share means other Python threads were
        # prevented from running, whatever `starved_pct_of_wall` says.
        "ticks": headline["ticks"],
        "expected_ticks": headline["expected_ticks"],
        "tick_share": headline["tick_share"],
        "stalls_over_300ms": len(stalls),
        "meshes": len(payloads),
        "final_source_verts": payloads[-1][0] if payloads else 0,
        "final_sent_verts": payloads[-1][1] if payloads else 0,
        "final_payload_mb": round(payloads[-1][2] / 1e6, 2) if payloads else 0.0,
        "peak_payload_mb": round(max(p[2] for p in payloads) / 1e6, 2) if payloads else 0.0,
    }


def _print_report(rep: dict) -> None:
    if "error" in rep:
        print(rep["error"], file=sys.stderr)
        return
    print(f"{rep['capture']}  {rep['width']}x{rep['height']}  device={rep['device']}  "
          f"block_count={rep['block_count']}  decimate={rep['decimate']}")
    print(f"wall {rep['wall_s']}s   integrated {rep['frames_integrated']}/{rep['frames']}   "
          f"effective {rep['effective_fps']} fps")
    print(f"\n{'stage':6} {'n':>5} {'p50 ms':>9} {'p90 ms':>9} {'max ms':>9} {'%wall':>7}")
    for name, s in rep["stages"].items():
        print(f"{name:6} {s['n']:5d} {s['p50_ms']:9.1f} {s['p90_ms']:9.1f} "
              f"{s['max_ms']:9.1f} {s['pct_of_wall']:6.1f}%")
    print("\nGIL starvation (= how long roomscan-web's event loop could not run):")
    print("  high tick_share ~= watchdog was regularly scheduled; low tick_share ~= other "
          "Python threads (asyncio, the reader) were prevented from running -- a low "
          "starved_pct_of_wall is NOT trustworthy when tick share is low (BUG-063).")
    for name, g in sorted(rep["gil_starvation"].items(),
                          key=lambda kv: -kv[1]["starved_s"]):
        share = g.get("tick_share")
        share_str = f"{share:.3f}" if share is not None else "n/a"
        print(f"  {name:6} starved {g['starved_s']:7.2f}s ({g['starved_pct_of_wall']:5.1f}% wall)"
              f"   worst stall {g['worst_stall_ms']:8.1f} ms"
              f"   ticks {g['ticks']}/{g.get('expected_ticks', 0)}, share {share_str}")
    total_share = rep.get("tick_share")
    total_share_str = f"{total_share:.3f}" if total_share is not None else "n/a"
    print(f"\nTOTAL starved {rep['starved_pct_of_wall']}% of wall; "
          f"worst single stall {rep['worst_stall_ms']} ms; "
          f"{rep['stalls_over_300ms']} stalls over 300 ms; "
          f"ticks {rep.get('ticks', 0)}/{rep.get('expected_ticks', 0)}, share {total_share_str}")
    print(f"payload: {rep['meshes']} meshes, final {rep['final_source_verts']} source verts "
          f"-> {rep['final_sent_verts']} sent, {rep['final_payload_mb']} MB "
          f"(peak {rep['peak_payload_mb']} MB)")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("capture")
    ap.add_argument("--frames", type=int, default=_DEFAULT_FRAMES,
                    help="frames to replay (0 = all); small maps prove nothing")
    ap.add_argument("--mesh-every", type=int, default=_DEFAULT_MESH_EVERY)
    ap.add_argument("--device", default=None, help="CUDA:0 / CPU:0 (default: auto)")
    ap.add_argument("--block-count", type=int, default=None)
    ap.add_argument("--decimate", action="store_true",
                    help="force MeshPrep quadric decimation (the 14x-worse A/B)")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    path = Path(args.capture)
    if not path.is_file():
        print(f"no such capture: {path}", file=sys.stderr)
        return 1
    rep = profile_capture(path, frames=args.frames, mesh_every=args.mesh_every,
                          device=args.device, block_count=args.block_count,
                          decimate=args.decimate)
    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print_report(rep)
    return 1 if "error" in rep else 0


if __name__ == "__main__":
    sys.exit(main())
