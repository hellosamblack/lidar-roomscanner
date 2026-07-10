"""Unit tests for roomscan.metrics — the per-sensor rate meters, the metrics
registry, GPU probe fallback chain, and the resource sampler.

All GPU access is mocked (no real GPU dependency); psutil is real (installed)
and exercised lightly through the ResourceSampler integration test.
"""
import time

from roomscan.metrics import (
    MetricsRegistry,
    RateMeter,
    ResourceSampler,
    fmt_bytes,
    fmt_hz,
    fmt_rate,
    probe_gpu_process,
)
from roomscan.protocol import FrameHeader, FrameType, StreamId


# --- RateMeter ---------------------------------------------------------------

def test_ratemeter_host_hz_from_arrival_span():
    m = RateMeter(window_s=10.0)
    # 5 frames, 0.10 s apart in arrival -> 4 gaps over 0.4 s -> 10 Hz
    for i in range(5):
        m.record(t_us=i * 100_000, arrival_s=i * 0.1, nbytes=100)
    assert abs(m.host_hz(now=0.4) - 10.0) < 1e-6


def test_ratemeter_device_hz_from_t_us_deltas():
    m = RateMeter(window_s=10.0)
    # device stamps 2000 us apart -> 500 Hz, regardless of arrival jitter
    arrivals = [0.0, 0.05, 0.30, 0.31]     # deliberately uneven arrival
    for i, a in enumerate(arrivals):
        m.record(t_us=i * 2000, arrival_s=a, nbytes=10)
    assert abs(m.device_hz(now=0.31) - 500.0) < 1e-6


def test_ratemeter_device_hz_none_when_t_us_flat_or_zero():
    m = RateMeter(window_s=10.0)
    for i in range(4):
        m.record(t_us=0, arrival_s=i * 0.1, nbytes=10)   # firmware not stamping t_us
    assert m.device_hz(now=0.3) is None


def test_ratemeter_device_hz_none_when_t_us_nonmonotonic():
    m = RateMeter(window_s=10.0)
    for i, tu in enumerate([1000, 2000, 1500, 3000]):    # a backwards step
        m.record(t_us=tu, arrival_s=i * 0.1, nbytes=10)
    assert m.device_hz(now=0.3) is None


def test_ratemeter_device_and_host_diverge_reveals_loss():
    # device produced at 100 Hz (10 ms apart) but only every other frame arrived,
    # 0.02 s apart -> host 50 Hz. host < device == link loss made visible.
    m = RateMeter(window_s=10.0)
    for i in range(6):
        m.record(t_us=i * 20_000, arrival_s=i * 0.02, nbytes=10)
    assert abs(m.device_hz(now=0.1) - 50.0) < 1e-6
    assert abs(m.host_hz(now=0.1) - 50.0) < 1e-6
    # (same numbers here; the point is both are computed independently — see
    #  the registry test for a genuine divergence through record())


def test_ratemeter_window_trims_old_samples():
    m = RateMeter(window_s=1.0)
    for i in range(5):
        m.record(t_us=i * 100_000, arrival_s=i * 0.1, nbytes=10)   # 0.0..0.4
    # now advance well past the window with a query -> everything stale -> 0 / None
    assert m.host_hz(now=5.0) == 0.0
    assert m.device_hz(now=5.0) is None


def test_ratemeter_bytes_per_s():
    m = RateMeter(window_s=10.0)
    # 5 frames of 1000 B, 0.1 s apart. bytes after the window-start reference
    # = 4 * 1000 over the 0.4 s span = 10_000 B/s.
    for i in range(5):
        m.record(t_us=i * 100_000, arrival_s=i * 0.1, nbytes=1000)
    assert abs(m.bytes_per_s(now=0.4) - 10_000.0) < 1e-6


def test_ratemeter_empty_is_zero():
    m = RateMeter()
    assert m.host_hz(now=1.0) == 0.0
    assert m.device_hz(now=1.0) is None
    assert m.bytes_per_s(now=1.0) == 0.0


# --- MetricsRegistry ---------------------------------------------------------

def _hdr(stream_id, seq, t_us):
    return FrameHeader(FrameType.DATA, stream_id, 0, seq, t_us, 54, 42, 0)


def test_registry_lists_only_sensor_streams():
    reg = MetricsRegistry(window_s=10.0)
    for i in range(4):
        reg.record(_hdr(StreamId.RAW_3DMD, i, i * 30_000), nbytes=14842, now=i * 0.03)
        reg.record(_hdr(StreamId.IMU_QUAT, i, i * 2_000), nbytes=16, now=i * 0.03)
        reg.record(_hdr(StreamId.CALIB, i, i * 30_000), nbytes=2332, now=i * 0.03)
    snap = reg.snapshot(now=0.09)
    labels = {s.label for s in snap.streams}
    assert labels == {"ToF", "IMU"}          # CALIB tracked for bandwidth, not listed
    # CALIB bytes still count toward the link total
    assert snap.link_bytes_per_s > 0.0


def test_registry_sensor_ordering_stable():
    reg = MetricsRegistry(window_s=10.0)
    order = [StreamId.ENV, StreamId.IMU_QUAT, StreamId.RAW_3DMD]
    for i in range(3):
        for sid in order:
            reg.record(_hdr(sid, i, i * 1000), nbytes=10, now=i * 0.1)
    snap = reg.snapshot(now=0.2)
    assert [s.label for s in snap.streams] == ["ToF", "IMU", "Env"]


def test_registry_render_fps():
    reg = MetricsRegistry(window_s=10.0)
    for i in range(11):
        reg.tick_render(now=i * 0.02)        # 10 gaps over 0.2 s -> 50 fps
    snap = reg.snapshot(now=0.2)
    assert abs(snap.render_fps - 50.0) < 1e-6


def test_registry_link_bytes_is_sum_across_streams():
    reg = MetricsRegistry(window_s=10.0)
    for i in range(5):
        reg.record(_hdr(StreamId.RAW_3DMD, i, i * 100_000), nbytes=1000, now=i * 0.1)
        reg.record(_hdr(StreamId.IMU_QUAT, i, i * 100_000), nbytes=500, now=i * 0.1)
    snap = reg.snapshot(now=0.4)
    tof = next(s for s in snap.streams if s.label == "ToF")
    imu = next(s for s in snap.streams if s.label == "IMU")
    assert abs(snap.link_bytes_per_s - (tof.bytes_per_s + imu.bytes_per_s)) < 1e-6


# --- formatters (pure) -------------------------------------------------------

def test_fmt_hz_precision_switches_at_10():
    assert fmt_hz(4.25) == "4.2"       # finer under 10 Hz
    assert fmt_hz(479.6) == "480"      # whole numbers above


def test_fmt_rate_units():
    assert fmt_rate(500) == "500 B/s"
    assert fmt_rate(1536) == "1.5 KB/s"
    assert fmt_rate(2 * 1024 * 1024) == "2.0 MB/s"


def test_fmt_bytes_units_and_unknown():
    assert fmt_bytes(None) == "?"
    assert fmt_bytes(8 * 1024 * 1024 * 1024) == "8.0 GB"


def test_fmt_hz_dash_when_none():
    assert fmt_hz(None) == "-"


# --- probe_gpu_process (per-process, NVML) -----------------------------------

def _boom(exc=RuntimeError("gpu probe failed")):
    def _p(pid):
        raise exc
    return _p


def test_probe_gpu_process_first_probe_wins():
    def good(pid):
        assert pid == 4321
        return (42.0, None, 12_000_000_000, "RTX", "pynvml")
    out = probe_gpu_process(4321, probes=[good, _boom()])
    assert out == (42.0, None, 12_000_000_000, "RTX", "pynvml")


def test_probe_gpu_process_falls_through_on_error():
    def ok(pid):
        return (7.0, 1_000, 2_000, "GPU", "pynvml")
    out = probe_gpu_process(1, probes=[_boom(ImportError("no pynvml")), ok])
    assert out[0] == 7.0 and out[4] == "pynvml"


def test_probe_gpu_process_all_fail_returns_na():
    out = probe_gpu_process(1, probes=[_boom(), _boom()])
    assert out == (None, None, None, None, "n/a")


# --- ResourceSampler (real psutil for THIS process, mocked GPU) --------------

def test_resource_sampler_produces_process_snapshot():
    def fake_gpu():
        return (55.0, 3_000_000, 8_000_000, "Fake GPU", "fake")
    s = ResourceSampler(interval=0.05, gpu_probe=fake_gpu)
    s.start()
    try:
        snap = None
        deadline = time.monotonic() + 3.0
        while snap is None and time.monotonic() < deadline:
            snap = s.latest()
            time.sleep(0.02)
        assert snap is not None, "sampler never produced a snapshot"
        assert snap.n_cores >= 1
        assert snap.proc_cpu_percent >= 0.0
        assert snap.ram_total > 0 and 0 < snap.proc_rss <= snap.ram_total
        assert snap.gpu_util == 55.0 and snap.gpu_source == "fake"
        assert snap.proc_vram == 3_000_000 and snap.vram_total == 8_000_000
    finally:
        s.stop()


def test_resource_sampler_gpu_na_when_probe_fails():
    def na_probe():
        return probe_gpu_process(999999, probes=[_boom()])

    s = ResourceSampler(interval=0.05, gpu_probe=na_probe)
    s.start()
    try:
        deadline = time.monotonic() + 3.0
        snap = None
        while snap is None and time.monotonic() < deadline:
            snap = s.latest()
            time.sleep(0.02)
        assert snap is not None
        assert snap.gpu_source == "n/a"
        assert snap.gpu_util is None
    finally:
        s.stop()


def test_resource_sampler_stop_is_idempotent_and_joins():
    def fake_gpu():
        return (0.0, None, 1, "g", "fake")
    s = ResourceSampler(interval=0.05, gpu_probe=fake_gpu)
    s.start()
    s.stop()
    s.stop()   # second stop must not raise
    assert not s.is_running()
