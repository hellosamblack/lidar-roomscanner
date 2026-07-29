"""Orientation-noise + stream-health instrument for a live roomscan-web `/ws`.

Created 2026-07-28 while chasing point-cloud edge shimmer (docs/iks4a1-stacking.md
"Orientation-noise pass", BUG-027). The web server gravity-aligns the cloud by the
fused SFLP quaternion, so orientation noise is multiplied by the scene's lever arm --
a 3 m wall turns 0.03 deg of wobble into 1.7 mm of edge motion. This is the
before/after instrument for any change to that path, firmware or host.

    # server must already be running (see docs/web-ui-testing.md); needs
    # dangerouslyDisableSandbox for the socket:
    host/.venv/bin/python host/tools/orientation_probe.py jitter --label "after fix"
    host/.venv/bin/python host/tools/orientation_probe.py jitter --seconds 60 --window 10 --json
    host/.venv/bin/python host/tools/orientation_probe.py health
    host/.venv/bin/python host/tools/orientation_probe.py health --json
    host/.venv/bin/python host/tools/orientation_probe.py frame

Subcommands:
  jitter     frame-to-frame angular change of the rotation applied to the cloud,
             plus directional coherence of the residual (noise vs. real motion).
             Prints a report and one machine-parseable `JITTER_SUMMARY k=v ...`
             line (or a single JSON object with `--json`).
  health     per-stream rates + drops/gaps -- prove the quat is fresh, not stale.
             Reports both the cumulative counters and their DELTA across the
             window (the delta says whether the link is degrading NOW). Prints
             a `HEALTH_SUMMARY k=v ...` line (or JSON with `--json`).
  frame      confirm the cloud is gravity-aligned and by exactly the reported `rot`.

Reference values for interpreting a run (keep these current):

  * metric floor        = 0.0004 deg/frame -- measured against a no-IMU replay
    (rotation identically disabled); below this you are reading the metric's
    own resolution, not the sensor.
  * firmware baseline    = 0.0118 deg/frame -- post-BUG-027 firmware (FIFO-batch
    averaging + gyro LPF1 + LIS2MDL OFF_CANC|LPF), stationary rig, smoother
    bypassed. The remaining floor is the SFLP FIFO's fp16 encoding
    (~0.056 deg/step), not the sensor.
  * coherence ~ 1/sqrt(window) (0.32 at window 10) = incoherent zero-mean
    jitter (a directional random walk); ~1.0 = consistent real motion;
    independent perturbations about a fixed truth (MA(1)) read ~0.10 --
    anti-correlated, still below the white-noise gate, the signature of
    quantization dither (this is how the fp16 floor was identified).

FOUR MEASUREMENT TRAPS, each of which produced a wrong answer once
(docs/engineering-practices.md "Reporting a measured improvement"):

1. Do NOT measure noise off the `sensor` JSON message. `build_sensor_message`
   rounds `rot` to 5 decimals, censoring sub-0.0006 deg changes to exactly
   zero. That reads as a spurious ~40x improvement. `jitter` reads the
   float32 cloud instead.
2. Do NOT compare points index-wise between frames. POINT_CLOUD is a
   variable-length list of VALID points, so index i is a different ray when
   the valid set changes; that inflates the number ~100x. `jitter` uses the
   mean unit ray direction, which averages ~2000 directions and is
   insensitive to membership. Its floor is 0.0004 deg, verified against a
   no-IMU replay (rotation identically disabled).
3. Bypass the host smoother when measuring FIRMWARE effects -- temporarily
   set `floor_alpha=1.0` in `web.OrientationSmoother.__init__`. It is a
   coherence-gated low-pass and will mask exactly what you are trying to
   measure. Before/after runs must have the smoother in the same state.
4. Run `health` after ANY firmware reflash before believing a jitter number --
   a stalled LSM leaves the host holding a stale quaternion, which looks like
   a spectacular noise reduction.

For a raw device-link rate check without a server (straight off the UDP
socket), use `host/tools/check_udp.py --seconds N` instead.

The pure math (no socket) is unit-tested in host/tests/test_orientation_probe.py.
Requires only stdlib + numpy; the socket path additionally needs `websockets`
(already in the `[web]` extra).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.motion import coherence  # noqa: E402  shared with the display/SLAM gates

TAG_POINT_CLOUD = 1
DEFAULT_URL = "ws://localhost:8000/ws"

# Verified floor of THIS metric (no-IMU replay, rotation identically disabled).
METRIC_FLOOR_DEG = 0.0004
# Post-BUG-027 firmware on a stationary rig, smoother bypassed (2026-07-28).
FIRMWARE_BASELINE_DEG = 0.0118
# Lever arm used to translate an angle into perceived edge motion: 3 m is a
# representative room-scale wall distance for this scanner.
LEVER_ARM_M = 3.0
# Trailing-window length for the coherence statistic; 10 is what both the
# display smoother and the SLAM stationarity hold use, so numbers compare.
DEFAULT_WINDOW = 10


# --- pure math (unit-tested, no socket) -------------------------------------

def mean_ray_direction(message: bytes) -> np.ndarray | None:
    """Mean unit ray direction of a POINT_CLOUD `/ws` message, or None.

    Layout (docs/web-protocol.md): ``u32 tag | f32[3N] positions | f32[3N]
    colors``. Points closer than 1e-6 m to the origin are dropped (no defined
    ray). Returns a unit 3-vector, or None for an empty/degenerate cloud.
    """
    n = (len(message) - 4) // 24
    if n <= 0:
        return None
    p = np.frombuffer(message, dtype="<f4", count=3 * n, offset=4)
    p = p.reshape(n, 3).astype(np.float64)
    r = np.linalg.norm(p, axis=1)
    ok = r > 1e-6
    if not ok.any():
        return None
    d = (p[ok] / r[ok, None]).mean(axis=0)
    norm = np.linalg.norm(d)
    return None if norm < 1e-9 else d / norm


def frame_angles_deg(dirs) -> np.ndarray:
    """Angle (deg) between each consecutive pair of unit directions."""
    a = np.asarray(dirs, dtype=np.float64)
    dots = np.clip(np.einsum("ij,ij->i", a[:-1], a[1:]), -1.0, 1.0)
    return np.degrees(np.arccos(dots))


def coherence_series(dirs, window: int = DEFAULT_WINDOW) -> np.ndarray:
    """Directional coherence of the direction increments, per trailing window.

    ``coherence`` (roomscan.motion) is ||sum(inc)|| / sum(||inc||): ~1.0 for
    consistent motion, ~1/sqrt(window) for zero-mean jitter, lower still for
    anti-correlated dither. Empty result if there are not enough frames
    (< window + 2).
    """
    inc = np.diff(np.asarray(dirs, dtype=np.float64), axis=0)
    if len(inc) <= window:
        return np.empty(0)
    return np.array([coherence(inc[i:i + window]) for i in range(len(inc) - window)])


def summarize_jitter(dirs, window: int = DEFAULT_WINDOW, seconds: float | None = None,
                      label: str = "") -> dict:
    """Full jitter summary for a sequence of >= 2 mean directions."""
    a = frame_angles_deg(dirs)
    c = coherence_series(dirs, window)
    to_mm = lambda deg: float(np.radians(deg) * LEVER_ARM_M * 1000.0)  # noqa: E731
    return {
        "label": label,
        "frames": len(dirs),
        "seconds": seconds,
        "mean_deg": float(a.mean()),
        "median_deg": float(np.percentile(a, 50)),
        "p95_deg": float(np.percentile(a, 95)),
        "max_deg": float(a.max()),
        "edge_mm_at_3m_mean": to_mm(a.mean()),
        "edge_mm_at_3m_p95": to_mm(np.percentile(a, 95)),
        "coherence_window": window,
        "coherence_mean": float(c.mean()) if len(c) else None,
        "coherence_p50": float(np.percentile(c, 50)) if len(c) else None,
        "coherence_p90": float(np.percentile(c, 90)) if len(c) else None,
        "white_noise_ref": float(1.0 / np.sqrt(window)),
        "metric_floor_deg": METRIC_FLOOR_DEG,
        "firmware_baseline_deg": FIRMWARE_BASELINE_DEG,
    }


def format_jitter_summary_line(s: dict) -> str:
    """One grep-able `JITTER_SUMMARY key=value ...` line from `summarize_jitter()`."""
    def fmt(v):
        if v is None:
            return "nan"
        if isinstance(v, float):
            return f"{v:.6f}"
        return str(v)
    keys = ["label", "frames", "seconds", "mean_deg", "median_deg", "p95_deg",
            "max_deg", "coherence_window", "coherence_mean", "coherence_p50",
            "white_noise_ref"]
    return "JITTER_SUMMARY " + " ".join(f"{k}={fmt(s.get(k))}" for k in keys)


def summarize_health(messages: list[dict], seconds: float | None = None) -> dict:
    """Reduce a window of `metrics` JSON messages to a health summary.

    Rates come from the LAST message (the server's freshest estimate); the
    cumulative `drops`/`gaps` counters are reported both as their final value
    and as the delta across the window (last - first) -- the delta is what
    tells you whether the link is degrading NOW, not whether it ever hiccuped
    since server start.
    """
    if not messages:
        raise ValueError("no metrics messages")
    first, last = messages[0], messages[-1]

    def delta(key):
        a, b = first.get(key), last.get(key)
        return (b - a) if isinstance(a, (int, float)) and isinstance(b, (int, float)) else None

    streams = {}
    for s in last.get("streams", []):
        streams[int(s["stream_id"])] = {
            "label": s.get("label"),
            "device_hz": s.get("device_hz"),
            "host_hz": s.get("host_hz"),
        }
    return {
        "seconds": seconds,
        "metrics_msgs": len(messages),
        "streams": streams,
        "drops": last.get("drops"),
        "drops_delta": delta("drops"),
        "gaps": last.get("gaps"),
        "gaps_delta": delta("gaps"),
        "render_fps": last.get("render_fps"),
    }


def format_health_summary_line(s: dict) -> str:
    """One grep-able `HEALTH_SUMMARY key=value ...` line from `summarize_health()`.

    Per-stream host-side rates appear as `stream<id>_hz=<host_hz>` for every
    stream id seen (host_hz is always populated; device_hz may be null).
    """
    def fmt(v):
        if v is None:
            return "nan"
        if isinstance(v, float):
            return f"{v:.2f}"
        return str(v)
    parts = [f"metrics_msgs={s['metrics_msgs']}", f"seconds={fmt(s['seconds'])}"]
    for sid in sorted(s["streams"]):
        parts.append(f"stream{sid}_hz={fmt(s['streams'][sid]['host_hz'])}")
    for k in ("drops", "drops_delta", "gaps", "gaps_delta", "render_fps"):
        parts.append(f"{k}={fmt(s.get(k))}")
    return "HEALTH_SUMMARY " + " ".join(parts)


# --- socket collection (needs `websockets`, not exercised by unit tests) ----

async def collect_directions(url: str, seconds: float) -> list[np.ndarray]:
    """Mean ray direction of every POINT_CLOUD seen on `/ws` for `seconds`."""
    import websockets  # deferred: [web] extra, not needed by the pure math

    dirs: list[np.ndarray] = []
    async with websockets.connect(url, max_size=None) as ws:
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        while loop.time() - t0 < seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=10)
            if isinstance(msg, bytes) and struct.unpack_from("<I", msg)[0] == TAG_POINT_CLOUD:
                d = mean_ray_direction(msg)
                if d is not None:
                    dirs.append(d)
    return dirs


async def collect_metrics(url: str, seconds: float) -> list[dict]:
    """Every `metrics` JSON message seen on `/ws` for `seconds`."""
    import websockets  # deferred: [web] extra, not needed by the pure math

    messages: list[dict] = []
    async with websockets.connect(url, max_size=None) as ws:
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        while loop.time() - t0 < seconds:
            msg = await asyncio.wait_for(ws.recv(), timeout=10)
            if isinstance(msg, bytes):
                continue
            d = json.loads(msg)
            if d.get("type") == "metrics":
                messages.append(d)
    return messages


# --- reports + subcommands ---------------------------------------------------

def report_jitter(s: dict) -> None:
    label = f"{s['label']}: " if s["label"] else ""
    print(f"{label}{s['frames']} clouds over {s['seconds']:.0f}s")
    print(f"  rotation change per frame (deg): mean={s['mean_deg']:.4f} "
          f"median={s['median_deg']:.4f} p95={s['p95_deg']:.4f} max={s['max_deg']:.4f}")
    print(f"  => edge motion at {LEVER_ARM_M:.0f} m (mm):    "
          f"mean={s['edge_mm_at_3m_mean']:.2f} p95={s['edge_mm_at_3m_p95']:.2f}")
    if s["coherence_mean"] is not None:
        print(f"  coherence (window {s['coherence_window']}): mean={s['coherence_mean']:.3f} "
              f"p50={s['coherence_p50']:.3f} p90={s['coherence_p90']:.3f}")
        print(f"  ~{s['white_noise_ref']:.2f} = incoherent jitter · ~1.0 = real motion · "
              f"below = quantization dither")
    print(f"  context: metric floor {METRIC_FLOOR_DEG} deg · "
          f"firmware baseline {FIRMWARE_BASELINE_DEG} deg (2026-07-28)")
    print(format_jitter_summary_line(s))


async def cmd_jitter(args) -> int:
    dirs = await collect_directions(args.url, args.seconds)
    if len(dirs) < 2:
        print("not enough point clouds -- is the server streaming?", file=sys.stderr)
        return 1
    s = summarize_jitter(dirs, window=args.window, seconds=args.seconds, label=args.label)
    if args.json:
        print(json.dumps(s))
    else:
        report_jitter(s)
    return 0


def report_health(s: dict) -> None:
    print(f"{s['metrics_msgs']} metrics messages over {s['seconds']:.0f}s")
    for sid in sorted(s["streams"]):
        st = s["streams"][sid]
        print(f"  stream {sid:>2} {str(st['label']):<12} "
              f"device_hz={st['device_hz']} host_hz={st['host_hz']}")
    print(f"  drops={s['drops']} (delta {s['drops_delta']})  "
          f"gaps={s['gaps']} (delta {s['gaps_delta']})  render_fps={s['render_fps']}")
    print("  healthy full stack = streams 7/9/10 all ~30 Hz, drops_delta=0, gaps_delta=0")
    print(format_health_summary_line(s))


async def cmd_health(args) -> int:
    """Per-stream rates + drops/gaps. Run this after ANY firmware reflash before
    believing a noise improvement: a stalled LSM leaves the host holding a
    stale quaternion, which looks like a spectacular noise reduction."""
    messages = await collect_metrics(args.url, args.seconds)
    if not messages:
        print("no metrics message seen -- is the server running and streaming?",
              file=sys.stderr)
        return 1
    s = summarize_health(messages, seconds=args.seconds)
    if args.json:
        print(json.dumps(s))
    else:
        report_health(s)
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
    import websockets  # deferred: [web] extra, not needed by the pure math

    def signature(p):
        ok = np.abs(p[:, 2]) > 1e-6
        return (len(np.unique(np.round(p[ok, 0] / p[ok, 2], 4))),
                len(np.unique(np.round(p[ok, 1] / p[ok, 2], 4))))

    rot = None
    async with websockets.connect(args.url, max_size=None) as ws:
        loop = asyncio.get_event_loop()
        t0 = loop.time()
        while loop.time() - t0 < args.seconds:
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
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        epilog="Full method + measurement traps in the module docstring.")
    ap.add_argument("--url", default=DEFAULT_URL, help=f"default {DEFAULT_URL}")
    ap.add_argument("--seconds", type=float, default=15.0, help="collection window")
    ap.add_argument("--json", action="store_true",
                     help="print a single JSON summary object instead of the report")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("jitter", help="frame-to-frame change of the applied rotation")
    p.add_argument("--label", default="", help="tag the run, e.g. 'before' / 'after'")
    p.add_argument("--window", type=int, default=DEFAULT_WINDOW,
                    help=f"coherence trailing window (default {DEFAULT_WINDOW}, "
                         "matches the smoother/SLAM gates)")
    p.set_defaults(fn=cmd_jitter)

    sub.add_parser("health", help="stream rates + drops/gaps (deltas across the window)"
                   ).set_defaults(fn=cmd_health)

    sub.add_parser("frame", help="verify the cloud frame of reference"
                   ).set_defaults(fn=cmd_frame)

    args = ap.parse_args()
    return asyncio.run(args.fn(args))


if __name__ == "__main__":
    raise SystemExit(main())
