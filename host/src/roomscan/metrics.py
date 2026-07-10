"""Live viewer metrics: per-sensor data rates, rendered FPS, system resources.

Pure/testable core (no Open3D imports) consumed by the control panel's HUD
overlay. See docs/superpowers/specs/2026-07-10-viewer-metrics-hud-design.md.

Rate model (owner-confirmed 2026-07-10): every wire frame carries an on-device
microsecond timestamp ``t_us``. For each sensor stream we report two rates:

* **device_hz** (``->hub``) = ``1e6 / mean(delta t_us)`` — the cadence the MCU /
  sensor-hub *produced* samples at, independent of the link. ``None`` when a
  stream's ``t_us`` is absent/zero/non-monotonic (firmware not stamping it).
* **host_hz** (``->host``) = frames received / wall-clock window — the rate that
  actually reached the PC.

host < device reveals frames lost/decimated on the USB CDC link, per sensor.

Threading: ``RateMeter``/``MetricsRegistry`` are fed from the reader thread and
read from the UI thread; each guards its own state with a lock (mirrors
``SensorState``). ``ResourceSampler`` runs its own daemon thread and publishes a
latest-snapshot via an atomic reference swap.
"""
from __future__ import annotations

import subprocess
import threading
import time
from collections import deque
from dataclasses import dataclass

from .protocol import FrameHeader, FrameType, StreamId

WINDOW_S = 2.0                 # sliding window for rate/bandwidth estimates

# stream_id -> short HUD label. DEPTH_ZF32 (Phase-1 replay) and RAW_3DMD
# (raw-only live) are both the ToF depth source and never co-occur in practice.
SENSOR_LABELS: dict[int, str] = {
    StreamId.DEPTH_ZF32: "ToF",
    StreamId.RAW_3DMD: "ToF",
    StreamId.IMU_QUAT: "IMU",
    StreamId.ENV: "Env",
}
# fixed display order (ToF, IMU, Env); anything else sorts last
_SENSOR_ORDER: dict[int, int] = {
    StreamId.DEPTH_ZF32: 0,
    StreamId.RAW_3DMD: 0,
    StreamId.IMU_QUAT: 1,
    StreamId.ENV: 2,
}


class RateMeter:
    """Sliding-window rate/bandwidth tracker for one stream. Pure aside from a
    lock guarding the sample deque (reader writes, UI reads)."""

    def __init__(self, window_s: float = WINDOW_S):
        self.window_s = window_s
        self._lock = threading.Lock()
        self._samples: deque[tuple[float, int, int]] = deque()  # (arrival_s, t_us, nbytes)

    def record(self, t_us: int, arrival_s: float, nbytes: int) -> None:
        with self._lock:
            self._samples.append((arrival_s, int(t_us), int(nbytes)))
            self._trim(arrival_s)

    def _trim(self, now: float) -> None:
        s = self._samples
        while s and now - s[0][0] > self.window_s:
            s.popleft()

    def host_hz(self, now: float) -> float:
        with self._lock:
            self._trim(now)
            s = self._samples
            if len(s) < 2:
                return 0.0
            span = s[-1][0] - s[0][0]
            return (len(s) - 1) / span if span > 0 else 0.0

    def device_hz(self, now: float) -> float | None:
        with self._lock:
            self._trim(now)
            ts = [x[1] for x in self._samples]
        if len(ts) < 2:
            return None
        deltas = [b - a for a, b in zip(ts, ts[1:])]
        if any(d <= 0 for d in deltas):     # zero/flat or non-monotonic t_us
            return None
        mean_us = sum(deltas) / len(deltas)
        return 1e6 / mean_us if mean_us > 0 else None

    def bytes_per_s(self, now: float) -> float:
        with self._lock:
            self._trim(now)
            s = self._samples
            if len(s) < 2:
                return 0.0
            span = s[-1][0] - s[0][0]
            if span <= 0:
                return 0.0
            # bytes that arrived after the window-start reference frame
            total = sum(x[2] for x in list(s)[1:])
            return total / span


@dataclass(frozen=True)
class StreamRate:
    stream_id: int
    label: str
    device_hz: float | None
    host_hz: float
    bytes_per_s: float


@dataclass(frozen=True)
class ResourceSnapshot:
    cpu_percent: list[float]        # per logical core
    cpu_overall: float
    ram_used: int
    ram_total: int
    gpu_util: float | None
    vram_used: int | None
    vram_total: int | None
    gpu_source: str                 # "pynvml" | "nvidia-smi" | "n/a" | test tag
    net_bytes_per_s: float


@dataclass(frozen=True)
class MetricsSnapshot:
    render_fps: float
    streams: list[StreamRate]
    link_bytes_per_s: float
    resources: ResourceSnapshot | None


class MetricsRegistry:
    """One RateMeter per stream_id seen + a rendered-frame counter. Fed from the
    reader thread (``record``/``tick_render``); the UI thread reads ``snapshot``.

    Optional ``sampler`` is a ResourceSampler whose latest snapshot is folded into
    ``MetricsSnapshot.resources`` (None if absent)."""

    def __init__(self, window_s: float = WINDOW_S, sampler: "ResourceSampler | None" = None):
        self.window_s = window_s
        self.sampler = sampler
        self._lock = threading.Lock()
        self._meters: dict[int, RateMeter] = {}
        self._render_ticks: deque[float] = deque()

    def record(self, header: FrameHeader, nbytes: int, now: float) -> None:
        """Record one decoded DATA frame. Non-DATA frames are ignored (their
        stream_id field is meaningless and would pollute the ToF meter)."""
        if header.frame_type != FrameType.DATA:
            return
        sid = header.stream_id
        with self._lock:
            m = self._meters.get(sid)
            if m is None:
                m = RateMeter(self.window_s)
                self._meters[sid] = m
        m.record(header.t_us, now, nbytes)

    def tick_render(self, now: float) -> None:
        with self._lock:
            self._render_ticks.append(now)
            while self._render_ticks and now - self._render_ticks[0] > self.window_s:
                self._render_ticks.popleft()

    def render_fps(self, now: float) -> float:
        with self._lock:
            ticks = self._render_ticks
            if len(ticks) < 2:
                return 0.0
            span = ticks[-1] - ticks[0]
            return (len(ticks) - 1) / span if span > 0 else 0.0

    def snapshot(self, now: float) -> MetricsSnapshot:
        with self._lock:
            meters = list(self._meters.items())
        streams: list[StreamRate] = []
        link_bps = 0.0
        for sid, m in meters:
            bps = m.bytes_per_s(now)
            link_bps += bps
            label = SENSOR_LABELS.get(sid)
            if label is not None:
                streams.append(StreamRate(sid, label, m.device_hz(now), m.host_hz(now), bps))
        streams.sort(key=lambda s: _SENSOR_ORDER.get(s.stream_id, 99))
        resources = self.sampler.latest() if self.sampler is not None else None
        return MetricsSnapshot(self.render_fps(now), streams, link_bps, resources)


# --- GPU probing (fallback chain: pynvml -> nvidia-smi -> n/a) ----------------

def _probe_gpu_pynvml() -> tuple[float | None, int | None, int | None, str]:
    import pynvml  # optional dep; ImportError falls through to the next probe
    pynvml.nvmlInit()
    try:
        h = pynvml.nvmlDeviceGetHandleByIndex(0)
        util = float(pynvml.nvmlDeviceGetUtilizationRates(h).gpu)
        mem = pynvml.nvmlDeviceGetMemoryInfo(h)
        return util, int(mem.used), int(mem.total), "pynvml"
    finally:
        try:
            pynvml.nvmlShutdown()
        except Exception:
            pass


def _probe_gpu_smi() -> tuple[float | None, int | None, int | None, str]:
    out = subprocess.run(
        ["nvidia-smi",
         "--query-gpu=utilization.gpu,memory.used,memory.total",
         "--format=csv,noheader,nounits"],
        capture_output=True, text=True, timeout=2.0, check=True,
    )
    line = out.stdout.strip().splitlines()[0]
    util_s, used_s, total_s = (p.strip() for p in line.split(","))
    return (float(util_s), int(used_s) * 1_048_576, int(total_s) * 1_048_576, "nvidia-smi")


_DEFAULT_GPU_PROBES = [_probe_gpu_pynvml, _probe_gpu_smi]


def probe_gpu(probes=None) -> tuple[float | None, int | None, int | None, str]:
    """Try each probe in order; first that returns without raising wins. All
    failures (no NVIDIA GPU, no driver, no pynvml) degrade to ``(None, None,
    None, "n/a")``. ``probes`` is injectable for tests."""
    for fn in (probes if probes is not None else _DEFAULT_GPU_PROBES):
        try:
            return fn()
        except Exception:
            continue
    return None, None, None, "n/a"


# --- HUD text formatting (pure) ----------------------------------------------

def fmt_hz(hz: float | None) -> str:
    """Rate for the HUD: ``—`` when unknown (device rate with no usable t_us),
    else adaptive precision (finer under 10 Hz)."""
    if hz is None:
        return "—"
    return f"{hz:.1f}" if hz < 10 else f"{hz:.0f}"


def fmt_rate(n_bytes_per_s: float) -> str:
    """Bytes/second -> compact B/s, KB/s, MB/s (1024-based)."""
    x = float(n_bytes_per_s)
    for unit in ("B", "KB", "MB", "GB"):
        if x < 1024 or unit == "GB":
            return f"{x:.0f} {unit}/s" if unit == "B" else f"{x:.1f} {unit}/s"
        x /= 1024
    return f"{x:.1f} GB/s"


def fmt_bytes(n: int | None) -> str:
    """Absolute byte count -> GB/MB for RAM/VRAM readouts. ``?`` when unknown."""
    if n is None:
        return "?"
    x = float(n)
    for unit in ("B", "KB", "MB", "GB", "TB"):
        if x < 1024 or unit == "TB":
            return f"{x:.1f} {unit}"
        x /= 1024
    return f"{x:.1f} TB"


def fmt_stream_line(sr: "StreamRate") -> str:
    """One sensor row: ``ToF   ->hub 28.1  ->host 28.0   431.0 KB/s``."""
    return (f"{sr.label:<4} →hub {fmt_hz(sr.device_hz):>5}  "
            f"→host {fmt_hz(sr.host_hz):>5}   {fmt_rate(sr.bytes_per_s)}")


def fmt_gpu_line(res: "ResourceSnapshot") -> str:
    if res.gpu_source == "n/a" or res.gpu_util is None:
        return "GPU  n/a"
    return (f"GPU  {res.gpu_util:.0f}%  VRAM {fmt_bytes(res.vram_used)}/"
            f"{fmt_bytes(res.vram_total)} ({res.gpu_source})")


def fmt_cpu_line(res: "ResourceSnapshot") -> str:
    cores = " ".join(f"{c:2.0f}" for c in res.cpu_percent)
    return f"CPU  avg {res.cpu_overall:2.0f}%  [{cores}]"


class ResourceSampler:
    """Background daemon sampling CPU/RAM/NIC (psutil) + GPU/VRAM (probe chain)
    at ``interval`` seconds, publishing the latest ResourceSnapshot for lockless
    reads. A slow ``nvidia-smi`` therefore never touches the render loop.

    ``gpu_probe`` is injectable (defaults to ``probe_gpu``) so tests need no GPU.
    """

    def __init__(self, interval: float = 0.7, gpu_probe=None):
        self.interval = interval
        self._gpu_probe = gpu_probe if gpu_probe is not None else probe_gpu
        self._latest: ResourceSnapshot | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="resource-sampler", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
            self._thread = None

    def is_running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def latest(self) -> ResourceSnapshot | None:
        return self._latest       # atomic reference read; no lock needed

    def _run(self) -> None:
        import psutil
        psutil.cpu_percent(percpu=True)     # prime (first call is a 0.0 baseline)
        last_net = psutil.net_io_counters()
        last_t = time.monotonic()
        while not self._stop.wait(self.interval):
            now = time.monotonic()
            cpu = psutil.cpu_percent(percpu=True)
            vm = psutil.virtual_memory()
            net = psutil.net_io_counters()
            dt = now - last_t
            if dt <= 0:
                dt = self.interval or 1e-9
            net_delta = (net.bytes_sent + net.bytes_recv) - (last_net.bytes_sent + last_net.bytes_recv)
            net_bps = max(net_delta / dt, 0.0)
            last_net, last_t = net, now
            gpu_util, vram_used, vram_total, src = self._gpu_probe()
            self._latest = ResourceSnapshot(
                cpu_percent=list(cpu),
                cpu_overall=(sum(cpu) / len(cpu)) if cpu else 0.0,
                ram_used=int(vm.used),
                ram_total=int(vm.total),
                gpu_util=gpu_util,
                vram_used=vram_used,
                vram_total=vram_total,
                gpu_source=src,
                net_bytes_per_s=net_bps,
            )
