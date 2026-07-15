"""Render the metrics HUD as an (H, W, 3) uint8 RGB image with drawn capacity
bars, shown over the 3D scene via an Open3D gui.ImageWidget.

Why an image and not gui.Labels: Open3D's gui font can't render many glyphs
(arrows, block-drawing chars all show as '?'), and it has no real bar widget.
Rendering with Pillow sidesteps both — we draw filled rectangles for the bars
and plain ASCII text, so it looks the same on every box. Same pattern as
sensors_widgets.render_compass / ir_image.reflectance_to_rgb.

Each row is `label   [====----]  value`, where the bar visualizes utilization
against a capacity:

* sensor rows (ToF/IMU/Env): bar = host_hz / device_hz — the fraction of what
  the sensor produced that actually reached the host (full == keeping up).
* USB: link throughput / USB Full-Speed practical ceiling.
* CPU: our process, drawn as one utilization bar per core in use (each 0..100%).
* RAM: our process RSS / system RAM.
* GPU: our process SM utilization (0..100%).
* VRAM: our process VRAM / total — only when the platform exposes it.

All figures are for THIS process, not the whole system.
"""
from __future__ import annotations

import math

import numpy as np

from .metrics import MetricsSnapshot, fmt_bytes, fmt_hz, fmt_rate

# USB Full-Speed CDC practical bulk ceiling. FS line rate is 12 Mbit/s
# (~1.5 MB/s); usable bulk throughput is ~1.0-1.2 MB/s. The bar fills toward
# this so a glance shows headroom before the link saturates.
USB_FS_CAPACITY_BPS = 1_200_000.0
FPS_TARGET = 60.0

# palette (RGB)
_BG = (16, 17, 22)
_TEXT = (232, 233, 240)
_MUTED = (150, 152, 165)
_TRACK = (44, 46, 55)
_GREEN = (95, 200, 125)
_AMBER = (232, 182, 72)
_RED = (226, 92, 80)
_BLUE = (96, 162, 232)

def _font(size: int):
    from PIL import ImageFont
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size)   # bundled with Pillow
    except Exception:
        try:
            return ImageFont.load_default(size)             # Pillow >= 10.1
        except Exception:
            return ImageFont.load_default()


def _bar_color(frac: float, good_high: bool) -> tuple[int, int, int]:
    """Green/amber/red by fraction. good_high=True means a HIGH value is good
    (sensor delivery ratio); False means high == near capacity (utilization)."""
    f = max(0.0, min(1.0, frac))
    if good_high:
        return _GREEN if f >= 0.9 else _AMBER if f >= 0.7 else _RED
    return _GREEN if f < 0.7 else _AMBER if f < 0.9 else _RED


class _Row:
    __slots__ = ("label", "value", "frac", "good_high", "accent")

    def __init__(self, label, value, frac=None, good_high=False, accent=None):
        self.label = label
        self.value = value
        self.frac = frac                # None -> empty track (unknown capacity)
        self.good_high = good_high
        self.accent = accent            # fixed color override (else green/amber/red)


def _rows(snap: MetricsSnapshot, link_capacity_bps: float, fps_target: float,
          view_fps: float = 0.0, link_label: str = "USB") -> list[_Row]:
    rows: list[_Row] = []
    rows.append(_Row("VIEW", f"{view_fps:.0f}",
                     frac=view_fps / fps_target if fps_target > 0 else None,
                     accent=_BLUE))
    rows.append(_Row("FPS", f"{snap.render_fps:.0f}",
                     frac=snap.render_fps / fps_target if fps_target > 0 else None,
                     accent=_BLUE))
    for s in snap.streams:
        if s.device_hz and s.device_hz > 0:
            frac = s.host_hz / s.device_hz
            val = f"{fmt_hz(s.host_hz)}/{fmt_hz(s.device_hz)} Hz"
            if s.jitter_ms is not None:
                val += f" j:{s.jitter_ms:.1f}ms"
            rows.append(_Row(s.label, val, frac=frac, good_high=True))
        else:                            # no usable device timestamp -> host only
            rows.append(_Row(s.label, f"{fmt_hz(s.host_hz)} Hz", frac=None))
    rows.append(_Row(link_label, fmt_rate(snap.link_bytes_per_s),
                     frac=snap.link_bytes_per_s / link_capacity_bps if link_capacity_bps > 0 else None))
    if snap.drops > 0 or snap.gaps > 0:
        rows.append(_Row("DROP", f"{snap.drops} frm, {snap.gaps} net", frac=None, accent=_RED))
    res = snap.resources
    if res is not None:
        # One utilization bar per core our app is using (not the whole system).
        # We can't map a process to physical cores per-core, so distribute the
        # process's total CPU into core-equivalents: N = ceil(cores used), each
        # bar filled 0..100%. Labelled CPU on the first row, blank under it.
        cores_used = res.proc_cpu_percent / 100.0
        n_bars = max(1, min(res.n_cores, math.ceil(cores_used) if cores_used > 0 else 1))
        for i in range(n_bars):
            fill = max(0.0, min(1.0, cores_used - i))
            rows.append(_Row("CPU" if i == 0 else "", f"{fill * 100:.0f}%", frac=fill))
        rows.append(_Row("RAM", fmt_bytes(res.proc_rss),
                         frac=res.proc_rss / res.ram_total if res.ram_total else None))
        if res.gpu_util is None:
            rows.append(_Row("GPU", "n/a (needs pynvml)", frac=None))
        else:
            rows.append(_Row("GPU", f"{res.gpu_util:.0f}%", frac=res.gpu_util / 100.0))
            if res.proc_vram is not None and res.vram_total:
                rows.append(_Row("VRAM", fmt_bytes(res.proc_vram),
                                 frac=res.proc_vram / res.vram_total))
    return rows


def render_hud(snap: MetricsSnapshot, *, view_fps: float = 0.0, width: int = 320, row_h: int = 22,
               link_capacity_bps: float = USB_FS_CAPACITY_BPS,
               link_label: str = "USB",
               fps_target: float = FPS_TARGET) -> np.ndarray:
    """Pure: MetricsSnapshot -> (H, W, 3) uint8 RGB HUD image."""
    from PIL import Image, ImageDraw

    rows = _rows(snap, link_capacity_bps, fps_target, view_fps=view_fps, link_label=link_label)
    top, bottom = 8, 6
    height = top + max(1, len(rows)) * row_h + bottom   # grows with the CPU-core rows
    img = Image.new("RGB", (width, height), _BG)
    d = ImageDraw.Draw(img)
    font = _font(13)

    label_x = 8
    label_w = 42
    val_w = 96
    bar_x = label_x + label_w
    bar_w = width - bar_x - val_w - 8
    bar_h = 9

    for i, row in enumerate(rows):
        cy = top + i * row_h + row_h // 2
        by = cy - bar_h // 2
        d.text((label_x, cy - 7), row.label, font=font, fill=_TEXT)
        # bar track
        d.rectangle([bar_x, by, bar_x + bar_w, by + bar_h], fill=_TRACK)
        if row.frac is not None:
            fillw = bar_w * max(0.0, min(1.0, row.frac))
            if fillw > 0:
                color = row.accent or _bar_color(row.frac, row.good_high)
                d.rectangle([bar_x, by, bar_x + fillw, by + bar_h], fill=color)
        # value text, right-aligned in the value column
        vw = d.textlength(row.value, font=font)
        d.text((width - 8 - vw, cy - 7), row.value, font=font,
               fill=_TEXT if row.frac is not None else _MUTED)

    return np.asarray(img, dtype=np.uint8)
