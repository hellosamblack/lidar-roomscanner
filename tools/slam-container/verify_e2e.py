"""Replay a recorded capture through RemoteSlamWorker (GPU container) and a
local CPU SlamWorker, and compare median per-frame slam_ms. Run with the
container started (tools/slam-container/start.ps1).

Usage: python tools/slam-container/verify_e2e.py <capture.bin> [--addr 127.0.0.1:5555]
"""
import argparse, statistics, time
from roomscan.slam.cli import _load_frames
from roomscan.slam.remote import RemoteSlamWorker
from roomscan.slam.worker import SlamWorker


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
    args = ap.parse_args()
    frames, w, h = _load_frames(args.capture)
    print(f"{len(frames)} frames {w}x{h}")

    rw = RemoteSlamWorker(w, h, addr=args.addr, fov_h=55.0, fov_v=42.0)
    assert rw.connect(), f"no service at {args.addr} -- run start.ps1"
    rw.start()
    remote_ms = drive(rw, frames); rw.stop()

    lw = SlamWorker(w, h, fov_h=55.0, fov_v=42.0, device="CPU:0")
    local_ms = drive(lw, frames)

    print(f"remote(GPU) median slam_ms = {statistics.median(remote_ms):.2f} (n={len(remote_ms)})")
    print(f"local(CPU)  median slam_ms = {statistics.median(local_ms):.2f} (n={len(local_ms)})")


if __name__ == "__main__":
    raise SystemExit(main())
