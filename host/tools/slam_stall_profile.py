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

    def report(self, wall_s: float) -> dict:
        out = {}
        for stage, samples in self.late.items():
            total = sum(samples)
            out[stage] = {
                "ticks": len(samples),
                "starved_s": round(total, 3),
                "starved_pct_of_wall": round(100.0 * total / wall_s, 1) if wall_s else 0.0,
                "worst_stall_ms": round(max(samples) * 1000.0, 1) if samples else 0.0,
            }
        return out


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

    mapper = Mapper(width, height, device=device, block_count=block_count,
                    release_cache_every=cfg.release_cache_every,
                    icp_retry_dist=cfg.icp_retry_dist,
                    baro_authority=cfg.baro_authority,
                    baro_tau_frames=cfg.baro_tau_frames)

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

    starvation = wd.report(wall)
    all_late = [x for samples in wd.late.values() for x in samples]
    stalls = [x for x in all_late if x > _STALL_S]
    return {
        "capture": str(path),
        "device": device,
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
        # single freeze a user would have seen.
        "starved_pct_of_wall": round(100.0 * sum(all_late) / wall, 1) if wall else 0.0,
        "worst_stall_ms": round(max(all_late) * 1000.0, 1) if all_late else 0.0,
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
    for name, g in sorted(rep["gil_starvation"].items(),
                          key=lambda kv: -kv[1]["starved_s"]):
        print(f"  {name:6} starved {g['starved_s']:7.2f}s ({g['starved_pct_of_wall']:5.1f}% wall)"
              f"   worst stall {g['worst_stall_ms']:8.1f} ms")
    print(f"\nTOTAL starved {rep['starved_pct_of_wall']}% of wall; "
          f"worst single stall {rep['worst_stall_ms']} ms; "
          f"{rep['stalls_over_300ms']} stalls over 300 ms")
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
