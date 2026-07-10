"""Unit tests for roomscan.metrics_hud.render_hud — the image HUD with bars.

Pure image rendering (Pillow); asserts shape/dtype, that the value/bar content
actually changes with the data, and that partial/absent data degrades cleanly.
"""
import numpy as np

from roomscan.metrics import (
    MetricsSnapshot,
    ResourceSnapshot,
    StreamRate,
)
from roomscan.metrics_hud import render_hud


def _res(**over):
    base = dict(proc_cpu_percent=150.0, n_cores=8, proc_rss=512_000_000,
                ram_total=32_000_000_000, gpu_util=23.0, proc_vram=None,
                vram_total=12_000_000_000, gpu_name="Test GPU", gpu_source="pynvml")
    base.update(over)
    return ResourceSnapshot(**base)


def _snap(streams=None, link=400_000.0, fps=27.0, resources="default"):
    if streams is None:
        streams = [StreamRate(7, "ToF", 28.0, 28.0, 400_000.0)]
    if resources == "default":
        resources = _res()
    return MetricsSnapshot(fps, streams, link, resources)


def test_render_hud_shape_and_dtype():
    img = render_hud(_snap(), width=320)
    assert img.dtype == np.uint8
    assert img.ndim == 3 and img.shape[2] == 3
    assert img.shape[1] == 320
    assert img.shape[0] > 0


def test_render_hud_width_fixed_height_grows_with_rows():
    # width is fixed; height scales with the number of rows (more sensors / more
    # CPU-core bars -> taller). The overlay frame tracks this each render.
    few = render_hud(_snap(streams=[], resources=None), width=320)
    many = render_hud(_snap(streams=[StreamRate(7, "ToF", 28.0, 28.0, 4e5),
                                     StreamRate(9, "IMU", 480.0, 476.0, 8000.0)]),
                      width=320)
    assert few.shape[1] == many.shape[1] == 320
    assert many.shape[0] > few.shape[0]


def test_render_hud_one_bar_per_core_in_use():
    # ~2.4 cores -> 3 CPU rows; ~0.5 core -> 1 CPU row. More cores == taller HUD.
    one_core = render_hud(_snap(streams=[], resources=_res(proc_cpu_percent=50.0)))
    three_core = render_hud(_snap(streams=[], resources=_res(proc_cpu_percent=240.0)))
    assert three_core.shape[0] > one_core.shape[0]


def test_render_hud_bar_fill_reflects_utilization():
    # a near-empty USB bar vs a near-full one must differ in pixel content
    low = render_hud(_snap(link=10_000.0, streams=[], resources=None))
    high = render_hud(_snap(link=1_150_000.0, streams=[], resources=None))
    assert not np.array_equal(low, high)


def test_render_hud_handles_missing_resources():
    img = render_hud(_snap(resources=None))     # sampler hasn't produced yet
    assert img.dtype == np.uint8 and img.shape[2] == 3


def test_render_hud_handles_device_hz_none():
    # no usable t_us -> host-only row, no ratio bar; must not raise
    img = render_hud(_snap(streams=[StreamRate(9, "IMU", None, 476.0, 8000.0)]))
    assert img.shape[2] == 3


def test_render_hud_gpu_na_when_util_none():
    img = render_hud(_snap(resources=_res(gpu_util=None, gpu_source="n/a")))
    assert img.shape[2] == 3


def test_render_hud_includes_vram_row_only_when_available():
    # proc_vram None (Windows/WDDM) -> no VRAM row; a value present -> a VRAM row
    # is drawn. The two renders must differ (an extra populated row).
    without = render_hud(_snap(resources=_res(proc_vram=None)))
    with_vram = render_hud(_snap(resources=_res(proc_vram=2_000_000_000)))
    assert not np.array_equal(without, with_vram)


def test_render_hud_cpu_multicore_segments_differ_from_single():
    one = render_hud(_snap(resources=_res(proc_cpu_percent=40.0)))    # ~0.4 core, 1 seg
    two = render_hud(_snap(resources=_res(proc_cpu_percent=175.0)))   # ~1.75 cores, 2 seg
    assert not np.array_equal(one, two)
