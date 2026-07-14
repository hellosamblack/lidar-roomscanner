"""Floating in-scene HUD (spec §5.1): each primary control renders to an RGBA
image in the depth-scope instrument language; a pure HudLayout maps clicks to
controls. No Open3D imports -- unit-tested like theme.py/cards.py.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import instrument

MODE_SWITCH = "mode_switch"
VIEW_TOGGLE = "view_toggle"
ACTION_CLUSTER = "action_cluster"
IR_CONTROL = "ir_control"
STATUS_CHIP = "status_chip"

SIZES = {
    MODE_SWITCH: (220, 34),
    VIEW_TOGGLE: (190, 34),
    ACTION_CLUSTER: (300, 44),
    IR_CONTROL: (220, 40),
    STATUS_CHIP: (200, 28),
}

_ALPHA = 235   # interior opacity; margin stays 0 (transparent)


def _canvas(w, h):
    """New RGBA image: transparent, then an opaque _PANEL card inset 1px with a
    hairline frame + cyan corner ticks. Returns (np_rgba, PIL_img, draw)."""
    from PIL import Image, ImageDraw
    img = Image.new("RGBA", (w, h), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)
    d.rectangle([1, 1, w - 2, h - 2], fill=instrument.PANEL + (_ALPHA,),
                outline=instrument.HAIR + (_ALPHA,))
    instrument.corner_ticks(d, 1, 1, w - 2, h - 2, instrument.ACCENT_U8 + (_ALPHA,),
                            ln=8, inset=3, width=2)
    return img, d


def _finish(img):
    return np.asarray(img, dtype=np.uint8)


def _segmented(labels, active_idx, size):
    w, h = size
    img, d = _canvas(w, h)
    font = instrument.load_font(12, bold=True)
    seg_w = (w - 4) / len(labels)
    for i, label in enumerate(labels):
        x0 = 2 + i * seg_w
        if i == active_idx:
            d.rectangle([x0 + 1, 3, x0 + seg_w - 1, h - 4],
                        fill=instrument.ACCENT_U8 + (_ALPHA,))
            fill = instrument.PANEL + (255,)
        else:
            fill = instrument.MUTED + (255,)
        tw = d.textlength(label, font=font)
        d.text((x0 + (seg_w - tw) / 2, (h - 14) / 2), label, font=font, fill=fill)
    return _finish(img)


def render_mode_switch(active: str) -> np.ndarray:
    return _segmented(["REAL-TIME", "SLAM"], 0 if active == "real_time" else 1,
                      SIZES[MODE_SWITCH])


def render_view_toggle(active: str) -> np.ndarray:
    return _segmented(["1ST-PERSON", "ORBIT"], 0 if active == "first_person" else 1,
                      SIZES[VIEW_TOGGLE])


def render_action_cluster(phase: str, is_replay: bool) -> np.ndarray:
    # Buttons vary by phase: idle -> [Record][Load][Clear];
    # recording -> [Stop][Clear]; processing -> [ ...processing ]; final -> [Load][Clear].
    labels = {
        "idle": ["REC", "LOAD", "CLR"],
        "recording": ["STOP", "CLR"],
        "processing": ["PROCESSING"],
        "final": ["LOAD", "CLR"],
    }.get(phase, ["REC", "LOAD", "CLR"])
    w, h = SIZES[ACTION_CLUSTER]
    img, d = _canvas(w, h)
    font = instrument.load_font(13, bold=True)
    seg_w = (w - 4) / len(labels)
    for i, label in enumerate(labels):
        x0 = 2 + i * seg_w
        hot = label in ("REC", "STOP")
        d.rectangle([x0 + 2, 4, x0 + seg_w - 2, h - 5],
                    outline=(instrument.ACCENT_U8 if hot else instrument.HAIR) + (_ALPHA,))
        tw = d.textlength(label, font=font)
        col = instrument.ACCENT_U8 if hot else instrument.TEXT
        d.text((x0 + (seg_w - tw) / 2, (h - 15) / 2), label, font=font, fill=col + (255,))
    return _finish(img)


def render_ir_control(enabled: bool, opacity: float) -> np.ndarray:
    w, h = SIZES[IR_CONTROL]
    img, d = _canvas(w, h)
    font = instrument.load_font(11, bold=True)
    d.text((10, (h - 12) / 2), "IR", font=font,
           fill=(instrument.ACCENT_U8 if enabled else instrument.MUTED) + (255,))
    # slider track + fill (fraction == opacity), right of the label
    tx0, tx1 = 40, w - 12
    ty = h // 2
    d.line([(tx0, ty), (tx1, ty)], fill=instrument.HAIR + (_ALPHA,), width=3)
    frac = float(np.clip(opacity, 0.0, 1.0))
    fx = tx0 + (tx1 - tx0) * frac
    d.line([(tx0, ty), (fx, ty)], fill=instrument.ACCENT_U8 + (_ALPHA,), width=3)
    d.ellipse([fx - 4, ty - 4, fx + 4, ty + 4], fill=instrument.ACCENT_U8 + (255,))
    return _finish(img)


def render_status_chip(tracking: str, fps: float) -> np.ndarray:
    w, h = SIZES[STATUS_CHIP]
    img, d = _canvas(w, h)
    font = instrument.load_font(11, mono=True)
    dot = {"ok": instrument.ACCENT_U8, "lost": (224, 96, 96), "--": instrument.MUTED}.get(
        tracking, instrument.MUTED)
    d.ellipse([9, h // 2 - 4, 17, h // 2 + 4], fill=dot + (255,))
    d.text((24, (h - 12) / 2), f"{tracking.upper():<5} {fps:4.0f} fps", font=font,
           fill=instrument.TEXT + (255,))
    return _finish(img)


MARGIN = 12
_IR_TRACK_X0 = 40          # must match render_ir_control's track start
_IR_TRACK_PAD_RIGHT = 12


@dataclass(frozen=True)
class ControlHit:
    control: str
    segment: int | None = None
    fraction: float | None = None


class HudLayout:
    def __init__(self, scene_x, scene_y, scene_w, scene_h, *,
                 is_replay: bool = False, mode: str = "slam"):
        self.x, self.y, self.w, self.h = scene_x, scene_y, scene_w, scene_h
        self.is_replay = is_replay
        self.mode = mode

    def rects(self):
        top = self.y + MARGIN
        bottom = lambda ch: self.y + self.h - MARGIN - SIZES[ch][1]
        mw, mh = SIZES[MODE_SWITCH]
        vw, vh = SIZES[VIEW_TOGGLE]
        sw, sh = SIZES[STATUS_CHIP]
        aw, ah = SIZES[ACTION_CLUSTER]
        iw, ih = SIZES[IR_CONTROL]
        out = {
            MODE_SWITCH: (self.x + (self.w - mw) // 2, top, mw, mh),
            VIEW_TOGGLE: (self.x + self.w - MARGIN - vw, top, vw, vh),
            STATUS_CHIP: (self.x + MARGIN, bottom(STATUS_CHIP), sw, sh),
            IR_CONTROL: (self.x + self.w - MARGIN - iw, bottom(IR_CONTROL), iw, ih),
        }
        if self.mode == "slam":
            out[ACTION_CLUSTER] = (self.x + (self.w - aw) // 2, bottom(ACTION_CLUSTER), aw, ah)
        return out

    @staticmethod
    def _in(rect, px, py):
        x, y, w, h = rect
        return x <= px < x + w and y <= py < y + h

    def hit_test(self, px, py):
        rects = self.rects()
        # Fixed 2-segment controls -> segment index by x.
        for ch in (MODE_SWITCH, VIEW_TOGGLE):
            if ch in rects and self._in(rects[ch], px, py):
                x, _, w, _ = rects[ch]
                seg = min(1, max(0, int((px - x) / (w / 2))))
                return ControlHit(ch, segment=seg)
        # Action cluster: variable button count (phase-driven) -> return a
        # fraction; the panel maps it to a button index by the current labels.
        if ACTION_CLUSTER in rects and self._in(rects[ACTION_CLUSTER], px, py):
            x, _, w, _ = rects[ACTION_CLUSTER]
            frac = float(np.clip((px - x) / max(1, w), 0.0, 1.0))
            return ControlHit(ACTION_CLUSTER, fraction=frac)
        if IR_CONTROL in rects and self._in(rects[IR_CONTROL], px, py):
            x, _, w, _ = rects[IR_CONTROL]
            local = px - x
            if local < _IR_TRACK_X0:
                return ControlHit(IR_CONTROL, segment=0)          # label -> toggle
            tx0, tx1 = _IR_TRACK_X0, w - _IR_TRACK_PAD_RIGHT
            frac = float(np.clip((local - tx0) / max(1, tx1 - tx0), 0.0, 1.0))
            return ControlHit(IR_CONTROL, fraction=frac)
        if STATUS_CHIP in rects and self._in(rects[STATUS_CHIP], px, py):
            return ControlHit(STATUS_CHIP)
        return None
