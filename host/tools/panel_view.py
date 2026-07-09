"""Headless snapshotter for the Open3D gui control panel (roomscan.panel).

WHY: the panel can only be *seen* on a live display, and Open3D's own offscreen
path (Filament) fails on this box with "EGL Headless is not supported" (needs a
real GPU/display; the machine is locked). This tool renders what the panel shows
to a PNG using pure CPU (numpy + Pillow), so the agent can open the image and
visually verify the cloud, the IR monitor pane, and the control state without a
display.

It is faithful because it runs the SAME data path the panel does -- FileSource ->
StreamDecoder -> TransformStage -> Deprojector -> the identical turbo/aux-plane
coloring and the identical `reflectance_to_rgb` IR render. The only substitution
is the 3D *renderer*: instead of Open3D's SceneWidget (GPU), it orthographically
projects the real deprojected points in numpy and rasterizes them with Pillow.
The control widgets (buttons/combos/sliders/log) are drawn as a static mock
reflecting the resolved state.

API
    frame = compute_frame(capture, frame_index=..., fov_h=..., fov_v=...)
    png   = render_snapshot(frame, color=..., ir_colormap=..., ir_freeze=...,
                            out=...)                      # -> Path
    png   = snapshot_from_replay(capture, ..., out=...)   # convenience one-shot
    png   = contact_sheet(capture, out=...)               # color x IR grid

CLI
    python tools/panel_view.py --replay ../captures/e2e_p2.bin --frame 150 \
        --color reflectance --ir turbo --out /tmp/panel.png
    python tools/panel_view.py --replay ../captures/e2e_p2.bin --contact --out /tmp/sheet.png
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

# roomscan is importable as an installed package; add src for direct `python tools/...` runs too.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

from roomscan.deproject import Deprojector              # noqa: E402
from roomscan.decoder import StreamDecoder              # noqa: E402
from roomscan.ir_image import ir_range, reflectance_to_rgb  # noqa: E402
from roomscan.native import Transform                   # noqa: E402
from roomscan.panel import _COLOR_MODES, _IR_UPSCALE, _USECASES, _rot_xy  # noqa: E402
from roomscan.pipeline import TransformStage            # noqa: E402
from roomscan.shading import MODES as NEAR_MODES        # noqa: E402
from roomscan.shading import cloud_colors               # noqa: E402
from roomscan.sources import FileSource, pump           # noqa: E402

W, H = 1280, 800
PANEL_W = 340
CLOUD_W = W - PANEL_W
BG = (13, 13, 20)
PANEL_BG = (24, 24, 32)
GROUP_BG = (34, 34, 44)
FG = (220, 220, 228)
DIM = (150, 150, 160)
ACCENT = (90, 170, 255)


@dataclass
class Frame:
    """Real per-frame render inputs pulled from the pipeline."""
    seq: int
    depth: np.ndarray                      # (h, w) f32, mm
    outputs: dict                          # name -> (h, w) plane
    frames: int = 0
    raw: int = 0
    gaps: int = 0
    crc: int = 0
    events: list[str] = field(default_factory=list)
    dll: bool = True


# ---- pipeline (the real data path) -----------------------------------------
def compute_frame(capture, frame_index: int = 120, fov_h: float = 55.0,
                  fov_v: float = 42.0) -> Frame:
    """Decode `capture` and return the `frame_index`-th transformed DATA frame,
    running the exact panel pipeline (transform via the native DLL when built)."""
    src = FileSource(str(capture))
    dec = StreamDecoder()
    dll = Transform.available()
    stage = TransformStage(("depth", "reflectance", "confidence") if dll else ("depth",))
    got = 0
    last = None
    frames = 0
    last_seq = None
    gaps = 0
    events: list[str] = []
    try:
        for fr in pump(src, dec):
            res = stage.feed(fr)
            if res is None:
                continue
            header, outputs = res
            frames += 1
            if last_seq is not None and header.seq > last_seq + 1:
                gaps += header.seq - last_seq - 1
            last_seq = header.seq
            last = (header, outputs)
            got += 1
            if got >= frame_index:
                break
    finally:
        src.close()
    if last is None:
        raise RuntimeError(f"{capture}: no transformed frames (need the DLL for RAW-only captures)")
    header, outputs = last
    return Frame(seq=header.seq, depth=outputs["depth"], outputs=outputs, frames=frames,
                 raw=stage.raw_transformed, gaps=gaps, crc=dec.crc_failures, events=events, dll=dll)


def _cloud_vals(depth, plane, max_range_mm, pts):
    """The per-point values to colorize -- mirrors roomscan.panel._render_frame
    (aux plane filtered by the identical validity mask so it stays point-aligned)."""
    if plane is not None:
        valid = np.isfinite(depth) & (depth > 0.0) & (depth < max_range_mm)
        return plane[valid].astype(np.float64, copy=False)
    return pts[:, 2]


# ---- cloud projection + raster (CPU stand-in for the GPU SceneWidget) -------
def _project(pts, width, height, azim=-35.0, elev=20.0, margin=0.10):
    """Orthographic 3/4 projection of the (N,3) cloud into an image box.
    Returns (sx, sy, order) where order draws far->near (painter's algorithm)."""
    c = pts.mean(0)
    q = pts - c
    az, el = math.radians(azim), math.radians(elev)
    ry = np.array([[math.cos(az), 0, math.sin(az)], [0, 1, 0], [-math.sin(az), 0, math.cos(az)]])
    rx = np.array([[1, 0, 0], [0, math.cos(el), -math.sin(el)], [0, math.sin(el), math.cos(el)]])
    r = q @ ry.T @ rx.T
    x, y, z = r[:, 0], r[:, 1], r[:, 2]
    span = max(float(np.ptp(x)), float(np.ptp(y)), 1e-6)
    scale = (1.0 - 2 * margin) * min(width, height) / span
    sx = width / 2 + x * scale
    sy = height / 2 - y * scale        # image y is down
    order = np.argsort(z)              # far (small z after rotation) first
    return sx, sy, order


def _draw_cloud(draw, pts, colors, x0, y0, w, h, point_size=6):
    draw.rectangle([x0, y0, x0 + w, y0 + h], fill=BG)
    if len(pts) == 0:
        draw.text((x0 + w // 2 - 40, y0 + h // 2), "(no points)", fill=DIM)
        return
    sx, sy, order = _project(pts, w, h)
    r = max(1, int(round(point_size)))       # square half-extent; raise to close gaps
    for i in order:
        px, py = x0 + sx[i], y0 + sy[i]
        col = tuple(int(255 * v) for v in colors[i])
        draw.rectangle([px - r, py - r, px + r, py + r], fill=col)


# ---- widget-panel mock ------------------------------------------------------
def _group(draw, x, y, w, title, lines, height):
    draw.rectangle([x, y, x + w, y + height], fill=GROUP_BG)
    draw.text((x + 8, y + 5), title, fill=ACCENT)
    ty = y + 22
    for ln, col in lines:
        draw.text((x + 12, ty), ln, fill=col)
        ty += 15
    return y + height + 8


def _button(draw, x, y, w, h, label, on=False):
    fill = (70, 110, 160) if on else (55, 60, 72)
    draw.rectangle([x, y, x + w, y + h], fill=fill, outline=(90, 96, 110))
    draw.text((x + 7, y + h // 2 - 6), label, fill=FG)
    return x + w + 6


def render_snapshot(frame: Frame, *, color: str = "depth", ir_colormap: str = "gray",
                    ir_freeze: bool = False, fov_h: float = 55.0, fov_v: float = 42.0,
                    usecase: int = 1, point_size: int = 6, near_mode: str = "window",
                    near_cutoff_m: float = 1.5, near_emphasis: float = 0.5, rot: int = 0,
                    out="panel_snapshot.png") -> Path:
    """Compose the panel snapshot PNG from a real Frame + the chosen view state."""
    depth = frame.depth
    h, w = depth.shape
    deproj = Deprojector(w, h, fov_h, fov_v)
    pts = deproj(depth)
    plane = None if color == "depth" else frame.outputs.get(color)
    if len(pts):
        vals = _cloud_vals(depth, plane, deproj.max_range_mm, pts)
        colors = cloud_colors(vals, pts[:, 2], mode=near_mode, cutoff_m=near_cutoff_m,
                              emphasis=near_emphasis)
        pts = _rot_xy(pts, rot)          # Rotate-90 button: roll cloud + IR together
    else:
        colors = np.zeros((0, 3))
    color_available = color == "depth" or plane is not None

    img = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(img)
    _draw_cloud(draw, pts, colors, 0, 0, CLOUD_W, H, point_size)
    near_txt = {"window": f"window<{near_cutoff_m:.1f}m", "emphasis": f"emphasis {near_emphasis:.2f}",
                "equalize": "equalize", "off": "off"}.get(near_mode, near_mode)
    draw.text((10, 10), f"3D cloud  |  {len(pts)} pts  |  color: {color}  |  near: {near_txt}"
              f"  |  pt {point_size}"
              + ("" if color_available else "  (plane absent -> depth fallback)"),
              fill=FG if color_available else (255, 180, 90))
    draw.text((10, H - 22), "CPU orthographic stand-in for the Open3D SceneWidget (Filament offscreen"
              " unavailable on this locked box)", fill=DIM)

    # right panel
    px = CLOUD_W
    draw.rectangle([px, 0, W, H], fill=PANEL_BG)
    gx, gw, y = px + 8, PANEL_W - 16, 8

    uc_name = _USECASES[usecase][1]
    y = _group(draw, gx, y, gw, "Status", [
        (f"replay | seq {frame.seq}", FG),
        (f"frames {frame.frames} | gaps {frame.gaps} | crc {frame.crc}", DIM),
        (f"raw {frame.raw} | dll {'on' if frame.dll else 'off'}", DIM),
        (f"usecase {uc_name.split()[0]} | color {color}", FG),
    ], 92)

    # Device
    draw.rectangle([gx, y, gx + gw, y + 96], fill=GROUP_BG)
    draw.text((gx + 8, y + 5), "Device", fill=ACCENT)
    bx = _button(draw, gx + 8, y + 24, 60, 20, "Ping")
    bx = _button(draw, bx, y + 24, 96, 20, "Req CALIB")
    _button(draw, bx, y + 24, 66, 20, "Reinit")
    draw.text((gx + 12, y + 52), f"usecase: {uc_name}", fill=FG)
    draw.text((gx + 12, y + 70), "exposure: 5 ms  [====------]", fill=FG)
    y += 104

    # View
    near_ctl = {"window": f"cutoff {near_cutoff_m:.1f} m", "emphasis": f"strength {near_emphasis:.2f}",
                "equalize": "(auto)", "off": "(off)"}.get(near_mode, "")
    y = _group(draw, gx, y, gw, "View", [
        ("color:  " + "  ".join(("[%s]" % m if m == color else m) for m in _COLOR_MODES), FG),
        (f"point size {point_size}   dark bg [x]   [Reset view] [Help]", DIM),
        ("near:  " + "  ".join(("[%s]" % m if m == near_mode else m) for m in NEAR_MODES), FG),
        (f"       {near_ctl}", DIM),
    ], 90)

    # IR Monitor (real image)
    ir_h = 150
    draw.rectangle([gx, y, gx + gw, y + ir_h + 44], fill=GROUP_BG)
    draw.text((gx + 8, y + 5), "IR Monitor", fill=ACCENT)
    refl = frame.outputs.get("reflectance")
    iy = y + 22
    if refl is not None:
        rng = ir_range(refl)
        rgb = reflectance_to_rgb(refl, colormap=ir_colormap, vmin=rng[0], vmax=rng[1], upscale=_IR_UPSCALE)
        if rot:
            rgb = np.rot90(rgb, rot)
        ir_img = Image.fromarray(np.ascontiguousarray(rgb)).resize((gw - 16, ir_h), Image.NEAREST)
        img.paste(ir_img, (gx + 8, iy))
        draw.text((gx + 12, y + ir_h + 24),
                  f"map: {ir_colormap}  freeze: {'on' if ir_freeze else 'off'}  "
                  f"range {rng[0]:.0f}-{rng[1]:.0f}", fill=DIM)
    else:
        draw.rectangle([gx + 8, iy, gx + gw - 8, iy + ir_h], fill=(40, 10, 10))
        draw.text((gx + 20, iy + ir_h // 2 - 6), "IR unavailable (no reflectance)", fill=(200, 120, 120))
    y += ir_h + 52

    # Capture + Events
    y = _group(draw, gx, y, gw, "Capture", [("[Record]   replay: [Pause] fps[==== ]", DIM)], 40)
    ev = frame.events[-4:] or ["connected: replay", f"transform dll {'on' if frame.dll else 'off'}"]
    _group(draw, gx, y, gw, "Events", [(e[:44], DIM) for e in ev], 24 + 15 * len(ev))

    out = Path(out)
    img.save(out)
    return out


def snapshot_from_replay(capture, *, frame_index=120, color="depth", ir_colormap="gray",
                         ir_freeze=False, fov_h=55.0, fov_v=42.0, usecase=1, point_size=6,
                         near_mode="window", near_cutoff_m=1.5, near_emphasis=0.5, rot=0,
                         out="panel_snapshot.png") -> Path:
    frame = compute_frame(capture, frame_index, fov_h, fov_v)
    return render_snapshot(frame, color=color, ir_colormap=ir_colormap, ir_freeze=ir_freeze,
                           fov_h=fov_h, fov_v=fov_v, usecase=usecase, point_size=point_size,
                           near_mode=near_mode, near_cutoff_m=near_cutoff_m,
                           near_emphasis=near_emphasis, rot=rot, out=out)


def contact_sheet(capture, *, frame_index=120, out="panel_contact.png") -> Path:
    """A 2x3 grid: {depth, reflectance, confidence} x {IR gray, IR turbo}."""
    frame = compute_frame(capture, frame_index)
    tiles = []
    for color in _COLOR_MODES:
        for ir in ("gray", "turbo"):
            p = render_snapshot(frame, color=color, ir_colormap=ir, out=f"_tile_{color}_{ir}.png")
            tiles.append(Image.open(p))
    tw, th = tiles[0].size
    scale = 0.5
    tw2, th2 = int(tw * scale), int(th * scale)
    sheet = Image.new("RGB", (tw2 * 2, th2 * 3), (0, 0, 0))
    for i, t in enumerate(tiles):
        r, c = divmod(i, 2)
        sheet.paste(t.resize((tw2, th2)), (c * tw2, r * th2))
    out = Path(out)
    sheet.save(out)
    for t in tiles:
        Path(t.filename).unlink(missing_ok=True)
    return out


def _main(argv=None) -> int:
    ap = argparse.ArgumentParser(prog="panel_view",
                                 description="Render a headless PNG snapshot of the roomscan gui panel.")
    ap.add_argument("--replay", required=True, help="RAW+CALIB (or depth) capture to render")
    ap.add_argument("--frame", type=int, default=120, help="which transformed frame to show")
    ap.add_argument("--color", choices=_COLOR_MODES, default="depth")
    ap.add_argument("--ir", choices=("gray", "turbo"), default="gray")
    ap.add_argument("--freeze", action="store_true")
    ap.add_argument("--fov-h", type=float, default=55.0)
    ap.add_argument("--fov-v", type=float, default=42.0)
    ap.add_argument("--usecase", type=int, default=1)
    ap.add_argument("--point-size", type=int, default=6, help="cloud point square size (px)")
    ap.add_argument("--near-mode", choices=NEAR_MODES, default="window",
                    help="near-contrast mode (more colormap on close targets)")
    ap.add_argument("--near-cutoff", type=float, default=1.5, help="window-mode cutoff (m)")
    ap.add_argument("--near-emphasis", type=float, default=0.5, help="emphasis-mode strength 0..1")
    ap.add_argument("--rot", type=int, default=0, help="90-deg turns to roll cloud + IR (0..3)")
    ap.add_argument("--contact", action="store_true", help="render the color x IR grid instead")
    ap.add_argument("--out", default="panel_snapshot.png")
    a = ap.parse_args(argv)
    if a.contact:
        p = contact_sheet(a.replay, frame_index=a.frame, out=a.out)
    else:
        p = snapshot_from_replay(a.replay, frame_index=a.frame, color=a.color, ir_colormap=a.ir,
                                 ir_freeze=a.freeze, fov_h=a.fov_h, fov_v=a.fov_v, usecase=a.usecase,
                                 point_size=a.point_size, near_mode=a.near_mode,
                                 near_cutoff_m=a.near_cutoff, near_emphasis=a.near_emphasis,
                                 rot=a.rot, out=a.out)
    print(f"wrote {p}")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
