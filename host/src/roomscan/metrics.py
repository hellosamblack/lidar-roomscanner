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

    def jitter_ms(self, now: float) -> float | None:
        with self._lock:
            self._trim(now)
            s = self._samples
        if len(s) < 2:
            return None
        j_sum = 0.0
        for i in range(1, len(s)):
            dh = s[i][0] - s[i-1][0]
            dd = (s[i][1] - s[i-1][1]) / 1e6
            if dd > 0:
                j_sum += abs(dh - dd)
        return (j_sum / (len(s) - 1)) * 1000.0


@dataclass(frozen=True)
class StreamRate:
    stream_id: int
    label: str
    device_hz: float | None
    host_hz: float
    bytes_per_s: float
    jitter_ms: float | None


@dataclass(frozen=True)
class ResourceSnapshot:
    """Resource usage of THIS process (not the whole system), so the HUD shows
    what our app consumes. ``proc_cpu_percent`` is summed across cores
    (100% == one full core); divide by 100 for core-equivalents. ``proc_vram``
    is None where the platform can't attribute GPU memory per process (Windows
    WDDM), in which case the HUD omits the VRAM bar."""
    proc_cpu_percent: float
    n_cores: int
    proc_rss: int                   # our process resident set size (bytes)
    ram_total: int                  # system RAM (the capacity the bar fills toward)
    gpu_util: float | None          # our process SM utilization %, None if no NVML
    proc_vram: int | None           # our process VRAM (bytes), None if unavailable
    vram_total: int | None
    gpu_name: str | None
    gpu_source: str                 # "pynvml" | "n/a" | test tag


@dataclass(frozen=True)
class MetricsSnapshot:
    render_fps: float
    streams: list[StreamRate]
    link_bytes_per_s: float
    resources: ResourceSnapshot | None
    drops: int = 0
    gaps: int = 0


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
                streams.append(StreamRate(sid, label, m.device_hz(now), m.host_hz(now), bps, m.jitter_ms(now)))
        streams.sort(key=lambda s: _SENSOR_ORDER.get(s.stream_id, 99))
        resources = self.sampler.latest() if self.sampler is not None else None
        return MetricsSnapshot(self.render_fps(now), streams, link_bps, resources)


# --- text formatting (pure) --------------------------------------------------

def fmt_hz(hz: float | None) -> str:
    """Rate for the HUD: ``-`` when unknown (device rate with no usable t_us),
    else adaptive precision (finer under 10 Hz)."""
    if hz is None:
        return "-"
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


# --- GPU probing (per-process, NVML only) ------------------------------------
# nvidia-smi can't report per-process GPU utilization, so the per-project GPU
# readout needs NVML (optional `monitor` extra). Without it, GPU shows n/a.

def _nvml_recent_us() -> int:
    """Timestamp (us) ~2 s ago: the lower bound for nvmlDeviceGetProcessUtilization
    so it returns only recent SM-utilization samples."""
    return int(time.time() * 1e6) - 2_000_000


def _probe_gpu_pynvml(pid: int):
    import pynvml as N   # optional dep; ImportError -> caught by probe_gpu_process
    N.nvmlInit()
    try:
        h = N.nvmlDeviceGetHandleByIndex(0)
        name = N.nvmlDeviceGetName(h)
        if isinstance(name, bytes):
            name = name.decode("ascii", "replace")
        vram_total = int(N.nvmlDeviceGetMemoryInfo(h).total)
        proc_vram = None                      # WDDM (Windows) reports None per process
        try:
            running = (list(N.nvmlDeviceGetGraphicsRunningProcesses(h))
                       + list(N.nvmlDeviceGetComputeRunningProcesses(h)))
            for p in running:
                if p.pid == pid and getattr(p, "usedGpuMemory", None):
                    proc_vram = int(p.usedGpuMemory)
        except N.NVMLError:
            pass
        util = 0.0                            # NVML up but our pid idle -> 0, not n/a
        try:
            for s in N.nvmlDeviceGetProcessUtilization(h, _nvml_recent_us()):
                if s.pid == pid:
                    util = float(s.smUtil)
        except N.NVMLError:
            util = 0.0                        # no recent samples == no GPU activity
        return util, proc_vram, vram_total, name, "pynvml"
    finally:
        try:
            N.nvmlShutdown()
        except Exception:
            pass


def probe_gpu_process(pid: int, probes=None):
    """Per-process GPU sample ``(util%, proc_vram, vram_total, name, source)``.
    Tries each probe; first that returns wins; all failures -> the n/a tuple.
    ``probes`` is injectable for tests (each called with ``pid``)."""
    for fn in (probes if probes is not None else [_probe_gpu_pynvml]):
        try:
            return fn(pid)
        except Exception:
            continue
    return None, None, None, None, "n/a"


class ResourceSampler:
    """Background daemon sampling THIS process's CPU/RAM (psutil) + per-process
    GPU (NVML) at ``interval`` seconds, publishing the latest ResourceSnapshot
    for lockless reads. Off the render loop so a slow probe never stalls it.

    ``gpu_probe`` is injectable (defaults to the NVML per-process probe) so tests
    need no GPU; ``pid`` defaults to the current process.
    """

    def __init__(self, interval: float = 0.7, gpu_probe=None, pid: int | None = None):
        import os
        self.interval = interval
        self._pid = pid if pid is not None else os.getpid()
        self._gpu_probe = gpu_probe if gpu_probe is not None else (lambda: probe_gpu_process(self._pid))
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
        proc = psutil.Process(self._pid)
        proc.cpu_percent(None)              # prime (first call is a 0.0 baseline)
        n_cores = psutil.cpu_count() or 1
        while not self._stop.wait(self.interval):
            try:
                cpu = proc.cpu_percent(None)
                rss = int(proc.memory_info().rss)
            except psutil.Error:
                continue                    # process vanished mid-sample; try again
            ram_total = int(psutil.virtual_memory().total)
            gpu_util, proc_vram, vram_total, gpu_name, src = self._gpu_probe()
            self._latest = ResourceSnapshot(
                proc_cpu_percent=float(cpu),
                n_cores=int(n_cores),
                proc_rss=rss,
                ram_total=ram_total,
                gpu_util=gpu_util,
                proc_vram=proc_vram,
                vram_total=vram_total,
                gpu_name=gpu_name,
                gpu_source=src,
            )
