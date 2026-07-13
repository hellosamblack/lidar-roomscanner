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
    for depth, refl, conf, quat, pa, t in frames:
        worker.submit(depth, quat, pa, reflectance=refl, confidence=conf)
        # remote: poll latest until a new result arrives; local: run_once
        if hasattr(worker, "run_once"):
            worker.run_once()
            got = worker.latest()
        else:
            got = None
            for _ in range(200):
                time.sleep(0.005); got = worker.latest()
                if got is not None:
                    break
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
