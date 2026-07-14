"""Replay a recorded capture through RemoteSlamWorker (GPU container) and a
local CPU SlamWorker, and compare per-frame slam_ms -- both the absolute speed
(median/mean/p90 + CPU/GPU speedup) and the DEGRADATION-OVER-TIME trend (median
slam_ms of the first third of the run vs the last third), which answers whether
per-step cost climbs as the TSDF map grows and whether the GPU flattens that
slope. Run with the container started (tools/slam-container/start.ps1).

Usage: python tools/slam-container/verify_e2e.py <capture.bin> [--addr 127.0.0.1:5555]
"""
import argparse, statistics, time
from roomscan.slam.cli import _load_frames
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.worker import SlamWorker


def _pctl(xs, q):
    s = sorted(xs)
    return s[min(len(s) - 1, int(q * len(s)))]


def summarize(label, ms):
    """Print absolute + trend stats for one backend's per-frame slam_ms."""
    if not ms:
        print(f"{label}: no samples")
        return None
    n = len(ms)
    third = max(1, n // 3)
    early = statistics.median(ms[:third])
    late = statistics.median(ms[-third:])
    drift = (late / early) if early else float("nan")
    print(f"{label}: n={n}  median={statistics.median(ms):.2f}ms  "
          f"mean={statistics.mean(ms):.2f}ms  p90={_pctl(ms, 0.9):.2f}ms")
    print(f"{label} trend: first-third median={early:.2f}ms  "
          f"last-third median={late:.2f}ms  degradation={drift:.2f}x")
    return {"median": statistics.median(ms), "early": early, "late": late, "drift": drift}


def drive(worker, frames):
    ms = []
    is_remote = not hasattr(worker, "run_once")
    last = None  # last published result tuple (identity), remote only
    for depth, refl, conf, quat, pa, t in frames:
        worker.submit(depth, quat, pa, reflectance=refl, confidence=conf)
        if is_remote:
            # _out_slot persists across reads (never cleared), so a plain
            # "non-None" poll would re-read the previous frame's stale result
            # and break immediately. Wait for a genuinely NEW published tuple
            # instead -- each publish in _recv_loop is a fresh tuple object,
            # so `is not last` distinguishes new from stale. This also paces
            # us to the server's true round-trip cadence, so the single-slot
            # _in_slot mailbox is drained before the next submit() (no frames
            # silently dropped). remote_ms below is therefore the
            # container-reported per-frame slam_ms (GPU compute time),
            # sampled 1:1 with submitted frames.
            got = None
            for _ in range(400):                 # wait up to ~2s for THIS frame's result
                time.sleep(0.005)
                cur = worker.latest()
                if cur is not None and cur is not last:
                    got = cur
                    last = cur
                    break
            # if no new result arrived (timeout), skip this frame rather than
            # recording a stale duplicate
        else:
            worker.run_once()
            got = worker.latest()
        if got is not None:
            ms.append(got[2].slam_ms)
    return ms


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("capture")
    ap.add_argument("--addr", default="127.0.0.1:5555")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="cap frames (e.g. to stay under the GPU-memory wall on long scans)")
    args = ap.parse_args()
    frames, w, h = _load_frames(args.capture, max_frames=args.max_frames)
    print(f"{len(frames)} frames {w}x{h}")

    rw = RemoteSlamWorker(w, h, addr=args.addr, fov_h=55.0, fov_v=42.0)
    assert rw.connect(), f"no service at {args.addr} -- run start.ps1"
    rw.start()
    t0 = time.time(); remote_ms = drive(rw, frames); gpu_wall = time.time() - t0
    rw.stop()

    lw = SlamWorker(w, h, fov_h=55.0, fov_v=42.0, device="CPU:0")
    t0 = time.time(); local_ms = drive(lw, frames); cpu_wall = time.time() - t0

    print()
    gpu = summarize("remote(GPU/CUDA)", remote_ms)
    cpu = summarize("local(CPU)      ", local_ms)
    print(f"\nwall-clock: GPU pass {gpu_wall:.1f}s  CPU pass {cpu_wall:.1f}s")
    if gpu and cpu and gpu["median"]:
        print(f"per-step speedup (CPU median / GPU median) = {cpu['median'] / gpu['median']:.1f}x")
        print(f"degradation over run: CPU {cpu['drift']:.2f}x  vs  GPU {gpu['drift']:.2f}x "
              f"(lower = flatter; >1 means each step got slower late in the run)")


if __name__ == "__main__":
    raise SystemExit(main())
