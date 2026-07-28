"""Measure orientation noise as the renderer actually sees it, over a live `/ws`.

Created 2026-07-28 while chasing point-cloud edge shimmer (docs/iks4a1-stacking.md
"Orientation-noise pass"). The web server gravity-aligns the cloud by the fused SFLP
quaternion, so orientation noise is multiplied by the scene's lever arm -- a 3 m wall
turns 0.03 deg of wobble into 1.7 mm of edge motion. These subcommands are the
before/after instrument for any change to that path, firmware or host.

    # server must already be running (see docs/web-ui-testing.md); needs
    # dangerouslyDisableSandbox for the socket:
    host/.venv/bin/python host/tools/orientation_probe.py jitter --label "after fix"
    host/.venv/bin/python host/tools/orientation_probe.py coherence
    host/.venv/bin/python host/tools/orientation_probe.py health
    host/.venv/bin/python host/tools/orientation_probe.py frame

Subcommands:
  jitter     frame-to-frame angular change of the rotation applied to the cloud
  coherence  is the residual incoherent (noise/dither) or coherent (real motion)?
  health     per-stream rates + drops/gaps -- prove the quat is fresh, not stale
  frame      confirm the cloud is gravity-aligned and by exactly the reported `rot`

THREE MEASUREMENT TRAPS, each of which produced a wrong answer once:

1. Do NOT measure noise off the `sensor` JSON message. `build_sensor_message` rounds
   `rot` to 5 decimals, censoring sub-0.0006 deg changes to exactly zero. That reads
   as a spurious ~40x improvement. `jitter` reads the float32 cloud instead.
2. Do NOT compare points index-wise between frames. POINT_CLOUD is a variable-length
   list of VALID points, so index i is a different ray when the valid set changes;
   that inflates the number ~100x. `jitter` uses the mean unit ray direction, which
   averages ~2000 directions and is insensitive to membership. Its floor is
   0.0004 deg, verified against a no-IMU replay (rotation identically disabled).
3. Bypass the host smoother when measuring FIRMWARE effects -- temporarily set
   `floor_alpha=1.0` in `web.OrientationSmoother.__init__`. It is a coherence-gated
   low-pass and will mask exactly what you are trying to measure.

Requires only stdlib + numpy + `websockets` (already in the `[web]` extra).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import struct
import sys

import numpy as np
import websockets

TAG_POINT_CLOUD = 1
DEFAULT_URL = "ws://localhost:8000/ws"

# Lever arm used to translate an angle into the edge motion a viewer perceives.
# 3 m is a representative room-scale wall distance for this scanner.
LEVER_ARM_M = 3.0


def _mean_direction(buf: bytes) -> np.ndarray | None:
    """Mean unit ray direction of a POINT_CLOUD payload, or None if degenerate.

    This rotates exactly with the applied orientation but does not care which points
    were valid this frame -- see trap 2 above.
    """
    n = (len(buf) - 4) // 24          # f32[3N] positions + f32[3N] colors
    if n <= 0:
        return None
    p = np.frombuffer(buf, dtype="<f4", count=3 * n, offset=4).reshape(n, 3).astype(float)
    r = np.linalg.norm(p, axis=1)
    ok = r > 1e-6
    if not ok.any():
        return None
    d = (p[ok] / r[ok, None]).mean(axis=0)
    norm = np.linalg.norm(d)
    return None if norm < 1e-9 else d / norm


async def _collect(url: str, seconds: float, want_rot: bool = False):
    """Collect (mean directions, latest rot) from `/ws` for `seconds`."""
    dirs, rot = [], None
    async with websockets.connect(url, max_size=None) as ws:
        t0 = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t0 < seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=10)
            if isinstance(msg, bytes):
                if struct.unpack_from("<I", msg)[0] == TAG_POINT_CLOUD:
                    d = _mean_direction(msg)
                    if d is not None:
                        dirs.append(d)
            elif want_rot:
                d = json.loads(msg)
                if d.get("type") == "sensor" and d.get("rot"):
                    rot = np.array(d["rot"], dtype=float).reshape(3, 3)
    return dirs, rot


def _angles_deg(dirs) -> np.ndarray:
    a = np.array(dirs)
    dots = np.clip(np.einsum("ij,ij->i", a[:-1], a[1:]), -1.0, 1.0)
    return np.degrees(np.arccos(dots))


async def cmd_jitter(args) -> int:
    dirs, _ = await _collect(args.url, args.seconds)
    if len(dirs) < 2:
        print("not enough point clouds -- is the server streaming?", file=sys.stderr)
        return 1
    a = _angles_deg(dirs)
    mm = lambda deg: np.radians(deg) * LEVER_ARM_M * 1000.0   # noqa: E731
    label = f"{args.label}: " if args.label else ""
    print(f"{label}{len(dirs)} clouds over {args.seconds:.0f}s")
    print(f"  applied-rotation change per frame (deg): "
          f"mean={a.mean():.4f} p95={np.percentile(a, 95):.4f} max={a.max():.4f}")
    print(f"  => edge motion at {LEVER_ARM_M:.0f} m (mm):        "
          f"mean={mm(a.mean()):.2f} p95={mm(np.percentile(a, 95)):.2f}")
    print("  (metric floor is 0.0004 deg; compare only against runs measured this way)")
    return 0


async def cmd_coherence(args) -> int:
    """Directional coherence separates dither/noise from genuine movement.

    ~1/sqrt(window) (0.32 at window 10) = incoherent zero-mean jitter.
    ~1.0 = consistent real motion.
    BELOW 1/sqrt(window) = anti-correlated, the signature of quantization dither
    (which is how the SFLP fp16 FIFO floor was identified).
    """
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[1] / "src"))
    from roomscan.motion import coherence     # shared with the SLAM stationarity gate

    dirs, _ = await _collect(args.url, args.seconds)
    if len(dirs) < args.window + 2:
        print("not enough point clouds", file=sys.stderr)
        return 1
    inc = np.diff(np.array(dirs), axis=0)
    c = np.array([coherence(inc[i:i + args.window])
                  for i in range(0, len(inc) - args.window)])
    print(f"windows={len(c)}  coherence: mean={c.mean():.3f} "
          f"p50={np.percentile(c, 50):.3f} p90={np.percentile(c, 90):.3f}")
    print(f"  fraction above the 0.5 gate = {float((c > 0.5).mean()):.2f}")
    print(f"  ~{1/np.sqrt(args.window):.2f} = incoherent jitter · ~1.0 = real motion · "
          f"below that = quantization dither")
    return 0


async def cmd_health(args) -> int:
    """Per-stream rates + drops/gaps. Run this after ANY firmware reflash before
    believing a noise improvement: a stalled LSM leaves the host holding a stale
    quaternion, which looks like a spectacular noise reduction."""
    latest = None
    async with websockets.connect(args.url, max_size=None) as ws:
        t0 = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t0 < args.seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=10)
            if isinstance(msg, bytes):
                continue
            d = json.loads(msg)
            if d.get("type") == "metrics":
                latest = d
    if latest is None:
        print("no metrics message seen", file=sys.stderr)
        return 1
    for s in latest.get("streams", []):
        print(f"  stream {s['stream_id']:>2} {s['label']:<12} "
              f"device_hz={s.get('device_hz')} host_hz={s.get('host_hz')}")
    print(f"  drops={latest.get('drops')} gaps={latest.get('gaps')} "
          f"render_fps={latest.get('render_fps')}")
    print("  healthy full stack = streams 7/9/10 all present at ~30 Hz, 0 drops, 0 gaps")
    return 0


async def cmd_frame(args) -> int:
    """Confirm the cloud is gravity-aligned, and by (approximately) the reported `rot`.

    A raw deprojected cloud comes off the 54x42 ray grid, so px/pz takes ~54 distinct
    values and py/pz ~42. Any non-trivial rotation destroys that structure; applying
    rot^-1 collapses it back. This is what proved the alignment was really applied.

    EXPECT A COLLAPSE, NOT AN EXACT MATCH. The cloud is rotated by the SMOOTHED display
    quaternion while the `sensor` message's `rot` is deliberately raw, so rot^-1 does
    not invert it exactly -- you get a few hundred distinct values rather than 54x42.
    A ~10x collapse still proves the cloud carries essentially that rotation. To see the
    exact 54x42, bypass the smoother (`floor_alpha=1.0` in web.OrientationSmoother).
    """
    def signature(p):
        ok = np.abs(p[:, 2]) > 1e-6
        return (len(np.unique(np.round(p[ok, 0] / p[ok, 2], 4))),
                len(np.unique(np.round(p[ok, 1] / p[ok, 2], 4))))

    rot = None
    async with websockets.connect(args.url, max_size=None) as ws:
        t0 = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - t0 < args.seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=10)
            if isinstance(msg, bytes):
                if struct.unpack_from("<I", msg)[0] == TAG_POINT_CLOUD and rot is not None:
                    n = (len(msg) - 4) // 24
                    p = np.frombuffer(msg, dtype="<f4", count=3 * n,
                                      offset=4).reshape(n, 3).astype(float)
                    bcast, unrot = signature(p), signature(p @ rot)
                    collapse = (bcast[0] * bcast[1]) / max(unrot[0] * unrot[1], 1)
                    print(f"  rot is identity : {np.allclose(rot, np.eye(3), atol=1e-3)}")
                    print(f"  as broadcast    : {bcast} distinct (u, v)")
                    print(f"  un-rotated      : {unrot} distinct (u, v)")
                    print(f"  collapse factor : {collapse:.0f}x  "
                          f"({'PASS' if collapse > 5 else 'FAIL'} -- want >5x)")
                    print("  (raw grid is 54x42; exact only with the smoother bypassed)")
                    return 0 if collapse > 5 else 1
            else:
                d = json.loads(msg)
                if d.get("type") == "sensor" and d.get("rot"):
                    rot = np.array(d["rot"], dtype=float).reshape(3, 3)
    print("no sensor `rot` seen -- ToF-only session? then no alignment is applied",
          file=sys.stderr)
    return 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--url", default=DEFAULT_URL, help=f"default {DEFAULT_URL}")
    ap.add_argument("--seconds", type=float, default=15.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("jitter", help="frame-to-frame change of the applied rotation")
    p.add_argument("--label", default="", help="tag the run, e.g. 'before' / 'after'")
    p.set_defaults(fn=cmd_jitter)
    p = sub.add_parser("coherence", help="noise/dither vs real motion")
    p.add_argument("--window", type=int, default=10, help="trailing window (gate uses 10)")
    p.set_defaults(fn=cmd_coherence)
    sub.add_parser("health", help="stream rates + drops/gaps").set_defaults(fn=cmd_health)
    sub.add_parser("frame", help="verify the cloud frame of reference").set_defaults(fn=cmd_frame)
    args = ap.parse_args()
    return asyncio.run(args.fn(args))


if __name__ == "__main__":
    raise SystemExit(main())
