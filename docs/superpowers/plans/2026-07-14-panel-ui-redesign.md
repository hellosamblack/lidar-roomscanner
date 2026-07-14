# Panel UI Redesign Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure `roomscan-panel` from a sidebar-driven, multi-mode window into a two-mode (Real-Time / SLAM), first-person-by-default, HUD-driven instrument with menu-driven settings — without changing any SLAM/TSDF math, protocol, or firmware.

**Architecture:** Extract three focused, GUI-free, unit-tested modules — `hud.py` (floating-control RGBA renders + a pure hit-test/layout model), `ir_overlay.py` (a camera-locked billboard quad for the first-person IR overlay), and `settings_dialog.py` (the old sidebar's grouped controls as one menu-opened dialog). `panel.py` shrinks to an orchestrator that owns threads/state/scene, holds two orthogonal axes (`mode ∈ {REAL_TIME, SLAM}`, `camera ∈ {FIRST_PERSON, ORBIT}`), builds a menubar, positions the HUD/overlay widgets in `_on_layout`, and routes scene clicks through `hud.hit_test` before camera nav. The SLAM and Showcase code paths are merged behind the single SLAM mode.

**Tech Stack:** Python 3.11–3.12, NumPy, Pillow (PIL, already a dep of `cards.py`), Open3D 0.19 (`visualization.gui` / `rendering`, tensor + legacy geometry), pytest.

## Global Constraints

Copied verbatim from the spec (`docs/superpowers/specs/2026-07-13-panel-ui-redesign-design.md`); every task's requirements implicitly include these.

- **No change to the SLAM/TSDF math, protocol, or firmware.** This is presentation-layer only.
- **No change to the reveal card / wavefront / stage / mesh pipeline already shipped** — they are reused as-is (`cards.render_scan_complete_card`, `_upload_slam_mesh`, `_upload_mesh_packet`, `MeshPrep`, the Showcase `PostProcessWorker`).
- **No new telemetry design** — the metrics HUD stays an optional overlay, toggled from the menu.
- **The classic keyboard-only `roomscan-view` window is out of scope** and must stay untouched (`viewer.py`).
- **One clear mode switch; no separate Showcase concept in the UI.** SLAM mode = the former SLAM view *and* the Showcase record→process→reveal flow, merged.
- **First-person is the default camera in both modes**, gizmo hidden and not flickering in first-person.
- **Settings live in a menubar dialog, out of the 3D view**; the always-visible sidebar (`_build_panel`) is retired.
- **Primary controls float in-scene, custom-drawn** in the instrument language (cyan accent `theme.ACCENT`, mono, corner ticks — reuse `theme.py` / the `cards.py` primitives).
- **Tests are GPU-free and GUI-free** for unit coverage. Pure helpers (`hud.py`, `ir_overlay.py`, state predicates, config) are unit-tested headless; use `pytest.importorskip("open3d")` only where a real tensor/legacy mesh is needed. Drive `ControlPanel` methods **unbound on a lightweight stand-in** (the established `test_panel_walls.py` / `test_panel_ux.py` / `test_panel_showcase.py` pattern), never a real window. GUI wiring stays **supervised-run verified** on the dev box (Filament fails headless).
- **All rendered strings must be pure ASCII** (`.isascii()`), so the GUI font never renders tofu — this is an existing regression guard (`test_panel_ux.py::test_showcase_banner_static_strings_are_ascii`); extend it, don't break it.
- **Existing panel unit tests stay green:** `test_panel_walls.py`, `test_panel_ux.py`, `test_panel_showcase.py`, `test_panel_meshpacket.py`, `test_panel_viewfps.py`, `test_cards.py`, `test_theme.py`, `test_config.py`. Only tests tied to the *retired sidebar widgets* may be updated/removed (Task 13).

**Test runner (all tasks):** from `F:\git\personal\lidar\roomscanner\host`, run
`.venv\Scripts\python.exe -m pytest <path>::<test> -v`
(`pythonpath = ["src", "."]` is set in `host/pyproject.toml`, so no editable install is needed). On a Git-Bash shell use `.venv/Scripts/python.exe -m pytest ...`.

**Commit discipline:** commit at the end of each task (DRY, YAGNI, TDD, frequent commits). Do not merge to `main`; this work is on `feature/phase6-slam`.

---

## File Structure

**New modules:**
- `host/src/roomscan/instrument.py` — shared instrument-drawing primitives (`u8`, `load_font`, `tracked_text`, `corner_ticks`, `fmt_count`) lifted out of `cards.py` so both `cards.py` and the new `hud.py` reuse one source. One responsibility: pure Pillow/numpy drawing helpers in the "depth-scope" language. GUI-free, unit-tested.
- `host/src/roomscan/hud.py` — floating-HUD control value-objects, pure `render_*` RGBA renders, and the `HudLayout` hit-test/layout model. No Open3D imports. GUI-free, unit-tested.
- `host/src/roomscan/ir_overlay.py` — pure `camera_locked_quad(...)` geometry for the first-person IR billboard. No Open3D imports. GUI-free, unit-tested.
- `host/src/roomscan/settings_dialog.py` — `build_settings_dialog(panel, *, section=None) -> gui.Dialog`: the old sidebar's grouped controls as one collapsible dialog, reusing the panel's existing `_on_*` handlers. GUI module (imports `open3d.visualization.gui`); its pure section list is unit-tested, the dialog build is supervised-run verified.

**Modified:**
- `host/src/roomscan/cards.py` — re-point its private primitives at `instrument.py` (keep the public `render_scan_complete_card` / `fmt_count` surface byte-identical).
- `host/src/roomscan/config.py` — add `mode`, `camera`, `ir_overlay`, `ir_opacity` to `ViewerConfig`.
- `host/src/roomscan/panel.py` — the orchestrator changes: state model, menubar, HUD/overlay widget positioning + mouse routing, mode/camera switching, first-person IR overlay, gizmo-flicker fix, config persistence, sidebar retirement.

**Tests (new):**
- `host/tests/test_instrument.py` (Task 1)
- `host/tests/test_hud.py` (Tasks 2–3)
- `host/tests/test_ir_overlay.py` (Task 4)
- `host/tests/test_panel_modes.py` (Tasks 5, 7, 10 — pure state predicates + unbound-stand-in wiring)
- `host/tests/test_config_ui.py` (Task 6)
- `host/tests/test_settings_dialog.py` (Task 8 — pure section list)
- `host/tools/panel_hud_probe.py` (Task 0 — supervised probe script)
- `host/tools/panel_ui_smoke.py` (Task 13 — supervised smoke script)

---

## Reference facts (verified against the code — use these exact names)

- `theme.ACCENT = (0.18, 0.88, 0.82)` (RGB float 0–1). `theme.vertical_gradient(w,h,top,bottom) -> (h,w,3) uint8`. `theme.floor_grid_lines`, `theme.trajectory_ramp` exist. `theme.BG_CLEAR_DARK`, `STAGE_TOP_DARK`, etc.
- `cards.py` private primitives to lift: `_u8(rgb_float)->(int,int,int)`, `_font(size,*,mono=False,bold=False)->PIL.ImageFont`, `_tracked(draw,xy,text,font,fill,spacing=3)->x_cursor`, `_corner_ticks(d,x0,y0,x1,y1,color,ln=11,inset=6,width=2)->None`, `fmt_count(n)->str` (already public). Chrome colors `_PANEL=(16,19,26)`, `_HAIR=(46,52,64)`, `_TEXT=(232,234,242)`, `_MUTED=(126,132,148)`, `_ACCENT=_u8(theme.ACCENT)=(46,224,209)`.
- `FrameStep` (`slam/mapper.py:33`): dataclass with `pose: np.ndarray` (4×4), `fitness`, `rmse`, `tracking_lost: bool`, `slam_ms: float`.
- `SlamWorker.latest() -> (mesh, trajectory, step) | None`; `step` is a `FrameStep`. `step.pose[:3,3]` is the sensor position.
- `ShowcasePhase` (`slam/showcase.py`): `IDLE, RECORDING, PROCESSING, FINAL` (`enum.auto()`, 1–4). `next_phase(phase,*,record_pressed=False,stop_pressed=False,processing_done=False,cleared=False)`. `Progress(fraction, mesh, trajectory, done, stats=None)`.
- Panel geometry-name constants already exist: `_GEOM="cloud"`, `_MESH_GEOM`, `_MESH_WALLS_GEOM`, `_SLAM_TRAJ_GEOM`, `_TRAJ_HEAD_GEOM`, `_FLOOR_GRID_GEOM`, `_FOV_GEOM`, `_CAPTURE_SQUARE_GEOM`, `_GIZMO_GEOM`.
- Panel helpers to reuse: `_np_to_o3d(rgb)`, `_apply_follow_camera(pose)`, `follow_camera_target(pose)`, `_hide_first_person_clutter()`, `_remove_live_view_geometries()`, `_remove_slam_geometries()`, `_update_camera_gizmo(quat)`, `_render_frame(item)`, `reflectance_to_rgb`, `ir_range`.
- Config plumbing: `ViewerConfig` dataclass (`config.py`), `apply_config_defaults`, `_PANEL_FIELDS` + `_fill_panel_fields` (`panel.py:2824`), `_persist_config` (`panel.py:1071`), `save()` iterates `fields(self)` so new dataclass fields persist automatically — but `_persist_config` builds a `ViewerConfig(...)` with an explicit subset, so new fields must be added there too.
- Existing camera-follow machinery the SLAM first-person path reuses: `self.follow_camera_enabled` (bool), `_follow_eye`/`_follow_center` (smoothing state), `_apply_follow_camera`, and `_on_mouse` early-returns `CONSUMED` when `follow_camera_enabled` so manual nav can't fight the followed camera.

---

## Task 0: Feasibility probe — do scene mouse events fall through a floating `ImageWidget`?

**Why first:** Spec §5.1 / §8 name this the highest risk; it gates the HUD interaction approach for Task 9. If a floating `gui.ImageWidget` **consumes** clicks (so `SceneWidget.set_on_mouse`/`_on_mouse` never sees them over a control), Task 9 must use the invisible-`gui.Button`-per-region fallback instead of pure image + hit-test.

**Files:**
- Create: `host/tools/panel_hud_probe.py`

**Interfaces:**
- Produces: a recorded decision (a one-line note appended to this plan file under Task 9, plus a commit message) — `PROBE RESULT: image-transparent` (scene handler sees clicks → pure hit-test) or `PROBE RESULT: image-opaque` (→ button-layer fallback).

- [ ] **Step 1: Write the probe script**

Create `host/tools/panel_hud_probe.py`:

```python
"""Task 0 probe (spec §5.1): does a floating gui.ImageWidget over a SceneWidget
let clicks reach the scene's set_on_mouse handler? Run ON THE DEV BOX (Filament
needs a display). Click INSIDE the cyan rectangle, then OUTSIDE it; read the
[probe] console lines.

  cd host && .venv\\Scripts\\python.exe tools\\panel_hud_probe.py
"""
import numpy as np
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering


def main():
    gui.Application.instance.initialize()
    w = gui.Application.instance.create_window("hud probe", 800, 600)
    scene = gui.SceneWidget()
    scene.scene = rendering.Open3DScene(w.renderer)
    box = o3d.geometry.TriangleMesh.create_box(1, 1, 1)
    box.compute_vertex_normals()
    mat = rendering.MaterialRecord(); mat.shader = "defaultUnlit"
    scene.scene.add_geometry("box", box, mat)
    scene.setup_camera(60.0, scene.scene.bounding_box, [0, 0, 0])

    def on_mouse(e):
        if e.type == gui.MouseEvent.Type.BUTTON_DOWN:
            print(f"[probe] scene handler GOT click at ({e.x},{e.y}) -- "
                  "if this fires when clicking INSIDE the cyan box, image is transparent")
        return gui.SceneWidget.EventCallbackResult.IGNORED

    scene.set_on_mouse(on_mouse)
    w.add_child(scene)

    img = np.zeros((80, 220, 4), dtype=np.uint8)
    img[..., :3] = [46, 224, 209]; img[..., 3] = 200   # cyan, semi-opaque
    overlay = gui.ImageWidget(o3d.geometry.Image(np.ascontiguousarray(img)))
    w.add_child(overlay)

    def on_layout(ctx):
        r = w.content_rect
        scene.frame = gui.Rect(r.x, r.y, r.width, r.height)
        overlay.frame = gui.Rect(r.x + 40, r.y + 40, 220, 80)
    w.set_on_layout(on_layout)
    gui.Application.instance.run()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the probe on the dev box and read the console**

Run: `cd host && .venv\Scripts\python.exe tools\panel_hud_probe.py`
Click once inside the cyan rectangle, once outside it, then close the window.
Expected outcomes:
- `[probe] scene handler GOT click ...` prints for **both** clicks → the ImageWidget is click-transparent → **`PROBE RESULT: image-transparent`** (Task 9 uses pure image + `hit_test`).
- The line prints only for the click **outside** the rectangle → the ImageWidget consumes clicks → **`PROBE RESULT: image-opaque`** (Task 9 adds an invisible `gui.Button` click layer per control region, sized/positioned identically to each HUD image, whose `set_on_clicked` calls the same hit-test action).

If the agent cannot inject synthetic clicks and no display is available, this is the one supervised step in the plan: ask the owner to run the two clicks and report the console lines.

- [ ] **Step 3: Record the decision + commit**

Append the outcome to Task 9's "Probe gate" line in this plan file (edit the doc), then:

```bash
git add host/tools/panel_hud_probe.py docs/superpowers/plans/2026-07-14-panel-ui-redesign.md
git commit -m "chore(panel): HUD mouse-fallthrough probe + recorded decision"
```

---

## Task 1: Extract shared instrument-drawing primitives into `instrument.py`

**Files:**
- Create: `host/src/roomscan/instrument.py`
- Modify: `host/src/roomscan/cards.py` (re-point private primitives at `instrument.py`)
- Test: `host/tests/test_instrument.py`
- Keep green: `host/tests/test_cards.py`

**Interfaces:**
- Produces (Tasks 2, 8 rely on these exact names/signatures):
  - `u8(rgb_float) -> tuple[int, int, int]` — RGB float 0–1 → 0–255 ints via `int(round(c*255))`.
  - `load_font(size: int, *, mono: bool = False, bold: bool = False)` — PIL `ImageFont`, Windows→DejaVu→default fallback chain (moved verbatim from `cards._font`).
  - `tracked_text(draw, xy, text, font, fill, spacing: int = 3) -> float` — letter-spaced draw; returns the x cursor after the last glyph.
  - `corner_ticks(d, x0, y0, x1, y1, color, ln: int = 11, inset: int = 6, width: int = 2) -> None` — L-shaped instrument-frame corners.
  - `fmt_count(n: int) -> str` — compact integer (`273789 -> "273k"`, `1_200_000 -> "1.2M"`).
  - Color constants: `PANEL = (16, 19, 26)`, `HAIR = (46, 52, 64)`, `TEXT = (232, 234, 242)`, `MUTED = (126, 132, 148)`, `ACCENT_U8 = u8(theme.ACCENT)  # (46, 224, 209)`.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_instrument.py`:

```python
"""Pure-function tests for the shared instrument-drawing primitives."""
import numpy as np
from PIL import Image, ImageDraw

from roomscan import instrument, theme


def test_u8_rounds_accent():
    assert instrument.u8(theme.ACCENT) == (46, 224, 209)


def test_fmt_count_thresholds():
    assert instrument.fmt_count(42) == "42"
    assert instrument.fmt_count(273789) == "273k"
    assert instrument.fmt_count(1_200_000) == "1.2M"
    assert instrument.fmt_count(0) == "0"


def test_accent_u8_constant():
    assert instrument.ACCENT_U8 == (46, 224, 209)


def test_corner_ticks_draws_accent_pixels():
    img = Image.new("RGB", (60, 40), instrument.PANEL)
    d = ImageDraw.Draw(img)
    instrument.corner_ticks(d, 0, 0, 59, 39, instrument.ACCENT_U8)
    arr = np.asarray(img)
    matches = np.all(arr.reshape(-1, 3) == np.array(instrument.ACCENT_U8), axis=1)
    assert matches.sum() > 8          # 4 corners * 2 strokes, several px each


def test_tracked_text_advances_cursor():
    img = Image.new("RGB", (200, 40), instrument.PANEL)
    d = ImageDraw.Draw(img)
    end = instrument.tracked_text(d, (5, 5), "ABC", instrument.load_font(13), instrument.TEXT)
    assert end > 5                     # cursor moved right of the start x


def test_load_font_returns_font():
    f = instrument.load_font(12, mono=True, bold=True)
    assert f is not None
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_instrument.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.instrument'`.

- [ ] **Step 3: Create `instrument.py`**

Create `host/src/roomscan/instrument.py`. Move the bodies of `_u8`, `_font`, `_tracked`, `_corner_ticks`, `fmt_count` out of `cards.py` verbatim, renamed to the public names above; add the color constants:

```python
"""Shared "depth-scope" instrument-drawing primitives (pure Pillow/numpy).

Lifted out of cards.py so both cards.py and hud.py draw in one visual
language from one source. No Open3D, no IO. Accent cyan == theme.ACCENT.
"""
from __future__ import annotations

from . import theme

PANEL = (16, 19, 26)
HAIR = (46, 52, 64)
TEXT = (232, 234, 242)
MUTED = (126, 132, 148)


def u8(rgb_float) -> tuple[int, int, int]:
    return tuple(int(round(c * 255)) for c in rgb_float)


ACCENT_U8 = u8(theme.ACCENT)   # (46, 224, 209)


def load_font(size: int, *, mono: bool = False, bold: bool = False):
    from PIL import ImageFont
    if mono and bold:
        candidates = ["C:/Windows/Fonts/consolab.ttf", "DejaVuSansMono.ttf"]
    elif mono:
        candidates = ["C:/Windows/Fonts/consola.ttf", "DejaVuSansMono.ttf"]
    elif bold:
        candidates = ["C:/Windows/Fonts/seguisb.ttf", "DejaVuSans.ttf"]
    else:
        candidates = ["C:/Windows/Fonts/segoeui.ttf", "DejaVuSans.ttf"]
    for c in candidates:
        try:
            return ImageFont.truetype(c, size)
        except Exception:
            continue
    try:
        return ImageFont.load_default(size)
    except Exception:
        return ImageFont.load_default()


def tracked_text(draw, xy, text, font, fill, spacing: int = 3) -> float:
    x, y = xy
    for ch in text:
        draw.text((x, y), ch, font=font, fill=fill)
        x += draw.textlength(ch, font=font) + spacing
    return x


def corner_ticks(d, x0, y0, x1, y1, color, ln: int = 11, inset: int = 6, width: int = 2) -> None:
    x0 += inset; y0 += inset; x1 -= inset; y1 -= inset
    for (cx, cy, sx, sy) in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                             (x0, y1, 1, -1), (x1, y1, -1, -1)):
        d.line([(cx, cy), (cx + sx * ln, cy)], fill=color, width=width)
        d.line([(cx, cy), (cx, cy + sy * ln)], fill=color, width=width)


def fmt_count(n: int) -> str:
    a = abs(n)
    if a >= 1_000_000:
        return f"{n / 1_000_000:.1f}M"
    if a >= 1_000:
        return f"{n // 1000}k"
    return str(n)
```

> Copy `load_font`/`tracked_text`/`corner_ticks` bodies from the current `cards.py` implementations to match them exactly; the versions above reproduce the behavior the explorer captured, but the source of truth is `cards.py`.

- [ ] **Step 4: Re-point `cards.py` at `instrument.py`**

In `host/src/roomscan/cards.py`, replace the private primitive definitions and chrome constants with imports, keeping the public API identical:

```python
from . import theme
from .instrument import (
    ACCENT_U8 as _ACCENT,
    HAIR as _HAIR,
    MUTED as _MUTED,
    PANEL as _PANEL,
    TEXT as _TEXT,
    corner_ticks as _corner_ticks,
    fmt_count,
    load_font as _font,
    tracked_text as _tracked,
    u8 as _u8,
)
```

Delete the old `_u8` / `_font` / `_tracked` / `_corner_ticks` / `fmt_count` bodies and the `_PANEL/_HAIR/_TEXT/_MUTED/_ACCENT` literals from `cards.py`. `render_scan_complete_card` still calls `_font(...)`, `_tracked(...)`, `_corner_ticks(...)`, `fmt_count(...)` — now the re-exported aliases — so its body is unchanged.

- [ ] **Step 5: Run both test files to verify green**

Run: `.venv\Scripts\python.exe -m pytest tests/test_instrument.py tests/test_cards.py -v`
Expected: PASS (all). `test_cards.py` proves the extraction was behavior-preserving.

- [ ] **Step 6: Commit**

```bash
git add host/src/roomscan/instrument.py host/src/roomscan/cards.py host/tests/test_instrument.py
git commit -m "refactor(panel): extract instrument-drawing primitives into instrument.py"
```

---

## Task 2: `hud.py` — control value-objects + `render_*` RGBA renders

**Files:**
- Create: `host/src/roomscan/hud.py`
- Test: `host/tests/test_hud.py`

**Interfaces:**
- Consumes: `roomscan.instrument.{load_font, tracked_text, corner_ticks, u8, ACCENT_U8, PANEL, HAIR, TEXT, MUTED}`.
- Produces (Task 3 + Task 9 rely on these):
  - Control-id constants: `MODE_SWITCH = "mode_switch"`, `VIEW_TOGGLE = "view_toggle"`, `ACTION_CLUSTER = "action_cluster"`, `IR_CONTROL = "ir_control"`, `STATUS_CHIP = "status_chip"`.
  - Fixed control pixel sizes (a control's rendered image is always this size): `SIZES: dict[str, tuple[int, int]]` (w, h) = `{MODE_SWITCH:(220,34), VIEW_TOGGLE:(190,34), ACTION_CLUSTER:(300,44), IR_CONTROL:(220,40), STATUS_CHIP:(200,28)}`.
  - `render_mode_switch(active: str) -> np.ndarray` — `active in ("real_time","slam")`; `(34,220,4)` uint8 RGBA.
  - `render_view_toggle(active: str) -> np.ndarray` — `active in ("first_person","orbit")`; `(34,190,4)`.
  - `render_action_cluster(phase: str, is_replay: bool) -> np.ndarray` — `phase in ("idle","recording","processing","final")`; `(44,300,4)`.
  - `render_ir_control(enabled: bool, opacity: float) -> np.ndarray` — `opacity in [0,1]`; `(40,220,4)`.
  - `render_status_chip(tracking: str, fps: float) -> np.ndarray` — `tracking in ("--","ok","lost")`; `(28,200,4)`.
  - All renders are pure and deterministic; RGBA with an **opaque interior** (`_PANEL`, alpha 235) and a fully-transparent 1px margin, so the control reads as a card whether or not the widget honors alpha.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_hud.py`:

```python
"""Pure tests for the floating HUD renders + hit-test model (no GUI)."""
import numpy as np

from roomscan import hud, instrument


def _accent_pixels(img):
    rgb = img[..., :3].reshape(-1, 3)
    return int(np.all(rgb == np.array(instrument.ACCENT_U8), axis=1).sum())


def test_render_mode_switch_shape_and_dtype():
    img = hud.render_mode_switch("slam")
    assert img.shape == (34, 220, 4)
    assert img.dtype == np.uint8


def test_render_mode_switch_uses_accent():
    assert _accent_pixels(hud.render_mode_switch("slam")) > 10


def test_render_mode_switch_active_differs():
    a = hud.render_mode_switch("real_time")
    b = hud.render_mode_switch("slam")
    assert not np.array_equal(a, b)          # the highlighted segment moved


def test_render_mode_switch_deterministic():
    np.testing.assert_array_equal(hud.render_mode_switch("slam"),
                                  hud.render_mode_switch("slam"))


def test_render_view_toggle_shape():
    assert hud.render_view_toggle("first_person").shape == (34, 190, 4)


def test_render_action_cluster_phases_differ():
    idle = hud.render_action_cluster("idle", is_replay=False)
    rec = hud.render_action_cluster("recording", is_replay=False)
    assert idle.shape == (44, 300, 4)
    assert not np.array_equal(idle, rec)


def test_render_ir_control_opacity_changes_render():
    lo = hud.render_ir_control(True, 0.1)
    hi = hud.render_ir_control(True, 0.9)
    assert lo.shape == (40, 220, 4)
    assert not np.array_equal(lo, hi)        # the slider fill width moved


def test_render_status_chip_shape_and_ascii_only_inputs():
    img = hud.render_status_chip("ok", 58.0)
    assert img.shape == (28, 200, 4)


def test_renders_have_transparent_margin():
    img = hud.render_mode_switch("slam")
    # top-left corner pixel is in the 1px transparent margin
    assert img[0, 0, 3] == 0
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_hud.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.hud'`.

- [ ] **Step 3: Implement the renders**

Create `host/src/roomscan/hud.py` (renders only — `HudLayout`/`hit_test` land in Task 3):

```python
"""Floating in-scene HUD (spec §5.1): each primary control renders to an RGBA
image in the depth-scope instrument language; a pure HudLayout maps clicks to
controls. No Open3D imports -- unit-tested like theme.py/cards.py.
"""
from __future__ import annotations

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
```

- [ ] **Step 4: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_hud.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/hud.py host/tests/test_hud.py
git commit -m "feat(panel): hud.py floating-control RGBA renders"
```

---

## Task 3: `hud.py` — `HudLayout` + `hit_test`

**Files:**
- Modify: `host/src/roomscan/hud.py`
- Test: `host/tests/test_hud.py` (add to it)

**Interfaces:**
- Consumes: `SIZES`, control-id constants from Task 2.
- Produces (Task 9 relies on these):
  - `ControlHit` dataclass: `control: str`, `segment: int | None = None`, `fraction: float | None = None`.
  - `class HudLayout:` constructed `HudLayout(scene_x: int, scene_y: int, scene_w: int, scene_h: int, *, is_replay: bool = False, mode: str = "slam")`.
    - `rects() -> dict[str, tuple[int,int,int,int]]` — control-id → absolute `(x, y, w, h)` window rect. Includes `ACTION_CLUSTER` only when `mode == "slam"`; includes `IR_CONTROL` only when `mode` first-person-capable (always present here — visibility is the panel's call).
    - `hit_test(px: int, py: int) -> ControlHit | None` — absolute pixel → control + sub-region. `MODE_SWITCH`/`VIEW_TOGGLE` are fixed 2-segment → `segment ∈ {0,1}` by x. `ACTION_CLUSTER`'s button count varies by phase (the layout is phase-agnostic), so it returns `fraction` (clamped 0..1 across the cluster width); the panel maps `fraction` → button index using the current label count. For `IR_CONTROL`, `fraction` = clamped slider fraction from x within the track; `segment=0` marks a click on the "IR" label region (toggle) vs `fraction` set for the track. `STATUS_CHIP` is read-only → returns `ControlHit(STATUS_CHIP)` with no sub-region.
  - Positions (absolute, relative to scene rect): `MODE_SWITCH` top-center; `VIEW_TOGGLE` top-right; `STATUS_CHIP` bottom-left; `ACTION_CLUSTER` bottom-center; `IR_CONTROL` bottom-right. Margin `MARGIN = 12` px from scene edges; top row `y = scene_y + MARGIN`; bottom row `y = scene_y + scene_h - MARGIN - h`.

- [ ] **Step 1: Write the failing tests** (append to `host/tests/test_hud.py`)

```python
from roomscan.hud import ControlHit, HudLayout


def test_layout_rects_present_for_slam():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    r = lay.rects()
    assert set(r) >= {hud.MODE_SWITCH, hud.VIEW_TOGGLE, hud.STATUS_CHIP,
                      hud.ACTION_CLUSTER, hud.IR_CONTROL}


def test_layout_real_time_hides_action_cluster():
    lay = HudLayout(0, 0, 1000, 700, mode="real_time")
    assert hud.ACTION_CLUSTER not in lay.rects()


def test_mode_switch_is_top_center():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.MODE_SWITCH]
    assert y == 12                                   # top row
    assert abs((x + w / 2) - 500) <= 1               # horizontally centered


def test_hit_test_mode_switch_second_segment():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.MODE_SWITCH]
    hit = lay.hit_test(int(x + w * 0.75), int(y + h / 2))
    assert hit == ControlHit(hud.MODE_SWITCH, segment=1)


def test_hit_test_miss_returns_none():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    assert lay.hit_test(500, 350) is None            # dead center of the scene


def test_hit_test_ir_slider_fraction():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.IR_CONTROL]
    # click ~ the far right of the track region -> fraction near 1.0
    hit = lay.hit_test(int(x + w - 14), int(y + h / 2))
    assert hit.control == hud.IR_CONTROL
    assert hit.fraction is not None and hit.fraction > 0.8


def test_hit_test_status_chip_readonly():
    lay = HudLayout(0, 0, 1000, 700, mode="slam")
    x, y, w, h = lay.rects()[hud.STATUS_CHIP]
    hit = lay.hit_test(int(x + w / 2), int(y + h / 2))
    assert hit == ControlHit(hud.STATUS_CHIP)
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_hud.py -k "layout or hit_test" -v`
Expected: FAIL with `ImportError: cannot import name 'ControlHit'`.

- [ ] **Step 3: Implement `ControlHit` + `HudLayout`** (append to `hud.py`)

```python
from dataclasses import dataclass

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
```

> The panel (Task 9/10) maps `ControlHit.fraction` for `ACTION_CLUSTER` to a button index via the current phase's label list (`render_action_cluster`'s label table): `idx = min(len(labels)-1, int(fraction * len(labels)))`. Keeping the cluster fraction-based lets `HudLayout` stay phase-agnostic (it never learns the button count).

- [ ] **Step 4: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_hud.py -v`
Expected: PASS (all Task 2 + Task 3 tests).

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/hud.py host/tests/test_hud.py
git commit -m "feat(panel): HudLayout hit-test model for the floating HUD"
```

---

## Task 4: `ir_overlay.py` — camera-locked IR billboard quad

**Files:**
- Create: `host/src/roomscan/ir_overlay.py`
- Test: `host/tests/test_ir_overlay.py`

**Interfaces:**
- Produces (Task 11 relies on this):
  - `camera_locked_quad(eye, forward, up, fov_h_deg: float, fov_v_deg: float, dist: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]` returning `(verts (4,3) f64, uvs (4,2) f32, tris (2,3) i32)`: a planar rectangle `dist` metres in front of `eye`, centered on the view ray, spanning the FoV at that distance, facing back toward `eye`. Corner order top-left / top-right / bottom-right / bottom-left (matching `capture_square_corners`). `up` is the world-up used to orient the quad's vertical; the quad's own right axis is `normalize(cross(forward, up))` and its up axis `cross(right, forward)`. `uvs` map the IR texture with (0,0) at top-left. `tris = [[0,1,2],[0,2,3]]`.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_ir_overlay.py`:

```python
"""Pure geometry tests for the first-person IR overlay quad."""
import numpy as np

from roomscan.ir_overlay import camera_locked_quad

_WORLD_UP = np.array([0.0, -1.0, 0.0])


def test_quad_shapes():
    v, uv, t = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    assert v.shape == (4, 3)
    assert uv.shape == (4, 2)
    assert t.shape == (2, 3)


def test_quad_is_planar():
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    n = np.cross(v[1] - v[0], v[2] - v[0]); n /= np.linalg.norm(n)
    assert abs(np.dot(v[3] - v[0], n)) < 1e-9


def test_quad_center_is_dist_ahead():
    dist = 0.8
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, 55.0, 42.0, dist)
    center = v.mean(axis=0)
    assert np.allclose(center, [0, 0, dist], atol=1e-9)


def test_quad_faces_the_eye():
    # The quad normal should point back toward the eye (dot with forward < 0
    # or > 0 consistently) -- i.e. the ray from center to eye is ~antiparallel
    # to forward.
    eye = np.array([0.0, 0.0, 0.0]); fwd = np.array([0.0, 0.0, 1.0])
    v, _, _ = camera_locked_quad(eye, fwd, _WORLD_UP, 55.0, 42.0, 1.0)
    center = v.mean(axis=0)
    to_eye = eye - center
    assert np.dot(to_eye, fwd) < 0


def test_quad_size_matches_fov():
    dist, fov_h, fov_v = 1.0, 60.0, 40.0
    v, _, _ = camera_locked_quad([0, 0, 0], [0, 0, 1], _WORLD_UP, fov_h, fov_v, dist)
    width = np.linalg.norm(v[1] - v[0])       # top-left -> top-right
    height = np.linalg.norm(v[2] - v[1])      # top-right -> bottom-right
    np.testing.assert_allclose(width, 2 * dist * np.tan(np.deg2rad(fov_h) / 2), rtol=1e-6)
    np.testing.assert_allclose(height, 2 * dist * np.tan(np.deg2rad(fov_v) / 2), rtol=1e-6)


def test_quad_translated_eye():
    v, _, _ = camera_locked_quad([5, 2, -3], [0, 0, 1], _WORLD_UP, 55.0, 42.0, 1.0)
    assert np.allclose(v.mean(axis=0), [5, 2, -2], atol=1e-9)
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_ir_overlay.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.ir_overlay'`.

- [ ] **Step 3: Implement**

Create `host/src/roomscan/ir_overlay.py`:

```python
"""Pure geometry for the first-person IR overlay: a camera-locked billboard
quad spanning the sensor FoV at a fixed distance in front of the eye (spec
§5.2). No Open3D imports -- unit-tested. The panel textures this with the live
IR image and material defaultUnlitTransparency at base_color alpha == opacity.
"""
from __future__ import annotations

import numpy as np


def camera_locked_quad(eye, forward, up, fov_h_deg, fov_v_deg, dist):
    eye = np.asarray(eye, dtype=np.float64)
    fwd = np.asarray(forward, dtype=np.float64)
    fwd = fwd / (np.linalg.norm(fwd) + 1e-12)
    up = np.asarray(up, dtype=np.float64)
    right = np.cross(fwd, up)
    right /= (np.linalg.norm(right) + 1e-12)
    quad_up = np.cross(right, fwd)
    quad_up /= (np.linalg.norm(quad_up) + 1e-12)
    half_w = dist * np.tan(np.deg2rad(fov_h_deg) / 2.0)
    half_v = dist * np.tan(np.deg2rad(fov_v_deg) / 2.0)
    center = eye + dist * fwd
    # Match capture_square_corners' vertical convention exactly (panel.py): its
    # camera-y is physically DOWN (x-right, y-down, z-forward), so its "top"
    # corners use -half_v along camera-y == +half_v along quad_up (quad_up here
    # points physically UP when up==_WORLD_UP). Order TL, TR, BR, BL.
    tl = center - half_w * right + half_v * quad_up
    tr = center + half_w * right + half_v * quad_up
    br = center + half_w * right - half_v * quad_up
    bl = center - half_w * right - half_v * quad_up
    verts = np.vstack([tl, tr, br, bl]).astype(np.float64)
    uvs = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float32)
    tris = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
    return verts, uvs, tris
```

- [ ] **Step 4: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_ir_overlay.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/ir_overlay.py host/tests/test_ir_overlay.py
git commit -m "feat(panel): ir_overlay.py camera-locked IR billboard quad"
```

---

## Task 5: Panel state model — mode/camera constants + pure transition & gizmo-gating predicates

**Files:**
- Modify: `host/src/roomscan/panel.py` (module-level additions only)
- Test: `host/tests/test_panel_modes.py`

**Interfaces:**
- Produces (Tasks 7, 9, 10 rely on these exact names):
  - Module constants: `VIEW_REAL_TIME = "real_time"`, `VIEW_SLAM = "slam"`, `CAM_FIRST_PERSON = "first_person"`, `CAM_ORBIT = "orbit"`.
  - `def follow_active(mode: str, camera: str) -> bool` — True iff `mode == VIEW_SLAM and camera == CAM_FIRST_PERSON` (the SLAM pose-follow path).
  - `def gizmo_should_update(camera: str, imu_gizmo: bool) -> bool` — True iff `camera == CAM_ORBIT and imu_gizmo` (spec §5.3 flicker fix: never add/re-add the gizmo in first-person).
  - `def real_time_first_person(mode: str, camera: str) -> bool` — True iff `mode == VIEW_REAL_TIME and camera == CAM_FIRST_PERSON` (the fixed sensor-origin camera).
  - `def load_kind(path: str) -> str` — `".bin" -> "capture"`, `".ply" -> "mesh"`, else `"unknown"` (case-insensitive on the suffix).

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_panel_modes.py`:

```python
"""Pure state-model predicates for the two-mode / two-camera panel redesign."""
import roomscan.panel as p


def test_follow_active_only_slam_first_person():
    assert p.follow_active(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is True
    assert p.follow_active(p.VIEW_SLAM, p.CAM_ORBIT) is False
    assert p.follow_active(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is False


def test_gizmo_should_update_only_orbit():
    assert p.gizmo_should_update(p.CAM_ORBIT, True) is True
    assert p.gizmo_should_update(p.CAM_FIRST_PERSON, True) is False
    assert p.gizmo_should_update(p.CAM_ORBIT, False) is False


def test_real_time_first_person():
    assert p.real_time_first_person(p.VIEW_REAL_TIME, p.CAM_FIRST_PERSON) is True
    assert p.real_time_first_person(p.VIEW_SLAM, p.CAM_FIRST_PERSON) is False


def test_load_kind_by_suffix():
    assert p.load_kind("captures/panel_x.bin") == "capture"
    assert p.load_kind("results/showcase_y.PLY") == "mesh"
    assert p.load_kind("foo.txt") == "unknown"
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -v`
Expected: FAIL with `AttributeError: module 'roomscan.panel' has no attribute 'VIEW_SLAM'`.

- [ ] **Step 3: Implement the constants + predicates**

In `host/src/roomscan/panel.py`, add after the existing `_FOLLOW_SMOOTH = 0.12` block (near line 133):

```python
# --- Two-mode / two-camera redesign (spec 2026-07-13) --------------------------
VIEW_REAL_TIME = "real_time"
VIEW_SLAM = "slam"
CAM_FIRST_PERSON = "first_person"
CAM_ORBIT = "orbit"


def follow_active(mode: str, camera: str) -> bool:
    """The SLAM pose-follow path (rides step.pose via _apply_follow_camera) is
    on only in SLAM mode + first-person. Real-Time first-person is a separate
    fixed sensor-origin camera (real_time_first_person), not this path."""
    return mode == VIEW_SLAM and camera == CAM_FIRST_PERSON


def gizmo_should_update(camera: str, imu_gizmo: bool) -> bool:
    """Spec §5.3 flicker fix: only add/refresh the IMU gizmo in ORBIT. In
    first-person the gizmo is never added, so nothing removes+re-adds it every
    tick -> no flicker."""
    return camera == CAM_ORBIT and imu_gizmo


def real_time_first_person(mode: str, camera: str) -> bool:
    return mode == VIEW_REAL_TIME and camera == CAM_FIRST_PERSON


def load_kind(path: str) -> str:
    """Dispatch a Load target by suffix: .bin -> capture (process pipeline),
    .ply -> mesh (display only), else unknown."""
    low = str(path).lower()
    if low.endswith(".bin"):
        return "capture"
    if low.endswith(".ply"):
        return "mesh"
    return "unknown"
```

- [ ] **Step 4: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_modes.py
git commit -m "feat(panel): mode/camera state-model constants + pure predicates"
```

---

## Task 6: Config — `mode`, `camera`, `ir_overlay`, `ir_opacity` fields + persistence

**Files:**
- Modify: `host/src/roomscan/config.py`
- Modify: `host/src/roomscan/panel.py` (`_PANEL_FIELDS`, `_persist_config`)
- Test: `host/tests/test_config_ui.py`

**Interfaces:**
- Consumes: `VIEW_SLAM`/`CAM_FIRST_PERSON` (Task 5) as defaults' string values.
- Produces (Tasks 9–11 read these off `args`): `ViewerConfig.mode: str = "slam"`, `.camera: str = "first_person"`, `.ir_overlay: bool = False`, `.ir_opacity: float = 0.5`.

- [ ] **Step 1: Write the failing tests**

Create `host/tests/test_config_ui.py`:

```python
"""Round-trip + default tests for the redesign's new config fields."""
from roomscan.config import ViewerConfig


def test_new_fields_defaults():
    c = ViewerConfig()
    assert c.mode == "slam"
    assert c.camera == "first_person"
    assert c.ir_overlay is False
    assert c.ir_opacity == 0.5


def test_new_fields_round_trip(tmp_path):
    path = tmp_path / "roomscan.toml"
    ViewerConfig(mode="real_time", camera="orbit", ir_overlay=True,
                 ir_opacity=0.8).save(path)
    back = ViewerConfig.load(path)
    assert back.mode == "real_time"
    assert back.camera == "orbit"
    assert back.ir_overlay is True
    assert back.ir_opacity == 0.8


def test_unknown_keys_ignored_still_loads(tmp_path):
    path = tmp_path / "roomscan.toml"
    path.write_text("[viewer]\nmode = \"slam\"\nbogus = 1\n", encoding="utf-8")
    assert ViewerConfig.load(path).mode == "slam"
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_config_ui.py -v`
Expected: FAIL with `AssertionError` (fields absent → `AttributeError`/default mismatch).

- [ ] **Step 3: Add the dataclass fields**

In `host/src/roomscan/config.py`, add to the `ViewerConfig` dataclass (after `metrics_overlay`, before the `yaw_*` block, ~line 58):

```python
    mode: str = "slam"                 # UI redesign: "real_time" | "slam"
    camera: str = "first_person"       # UI redesign: "first_person" | "orbit"
    ir_overlay: bool = False           # first-person IR billboard overlay on/off
    ir_opacity: float = 0.5            # IR overlay opacity 0..1
```

`save()` iterates `fields(self)`, so these persist automatically; `load()` already filters to known fields. No writer change needed (all scalar types the writer handles).

- [ ] **Step 4: Wire the panel's load + persist paths**

In `host/src/roomscan/panel.py`, add the four names to `_PANEL_FIELDS` (line 2824) so `_fill_panel_fields` pulls them from config when no CLI flag set:

```python
_PANEL_FIELDS = ("point_size", "ir_colormap", "ir_freeze_range", "panel_width",
                 "near_mode", "near_cutoff_m", "near_emphasis",
                 "surface_enabled", "surface_mode", "surface_threshold_pct",
                 "imu_gizmo", "sensors_panel", "gizmo_scale", "metrics_overlay",
                 "yaw_fusion", "yaw_fusion_tau", "mag_cal_path",
                 "yaw_anomaly_frac", "yaw_motion_rate_dps", "yaw_gimbal_margin_deg",
                 "mode", "camera", "ir_overlay", "ir_opacity")
```

And extend `_persist_config` (line 1071) to include the runtime values in the `ViewerConfig(...)` it builds:

```python
                imu_gizmo=self.imu_gizmo, sensors_panel=self.sensors_panel,
                gizmo_scale=self.gizmo_scale, metrics_overlay=self.metrics_overlay,
                mode=self.mode, camera=self.camera,
                ir_overlay=self.ir_overlay_enabled, ir_opacity=self.ir_opacity)
```

(`self.mode`, `self.camera`, `self.ir_overlay_enabled`, `self.ir_opacity` are initialized in Task 9 — this edit lands with Task 9's `__init__` additions; if running Task 6 standalone, guard with `getattr(self, "mode", "slam")` etc. and tighten in Task 9.)

- [ ] **Step 5: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_config_ui.py tests/test_config.py -v`
Expected: PASS (new + existing config tests).

- [ ] **Step 6: Commit**

```bash
git add host/src/roomscan/config.py host/src/roomscan/panel.py host/tests/test_config_ui.py
git commit -m "feat(panel): persist mode/camera/ir_overlay/ir_opacity in ViewerConfig"
```

---

## Task 7: Gizmo-flicker fix (spec §5.3)

**Files:**
- Modify: `host/src/roomscan/panel.py` (`_update_sensors`, `_render_frame`'s gizmo call site)
- Test: `host/tests/test_panel_modes.py` (add an unbound-stand-in test)

**Interfaces:**
- Consumes: `gizmo_should_update(camera, imu_gizmo)` (Task 5); `self.camera_mode` (set in Task 9 `__init__`; for this task, read via `getattr(self, "camera_mode", CAM_ORBIT)`).

**Root cause (verified):** `_render_frame` calls `_update_camera_gizmo(quat_display)` every frame (line 1191), and `_update_sensors` calls it again on the ≤4 Hz tick (line 2573–2574). The first-person path calls `_hide_first_person_clutter()` (removes the gizmo, resets `_gizmo_added`) every processed SLAM frame while the gizmo code keeps re-adding it → visible flicker. Fix: gate every gizmo add/refresh on `gizmo_should_update(...)` so in first-person the gizmo is simply never added.

- [ ] **Step 1: Write the failing test** (append to `host/tests/test_panel_modes.py`)

```python
import numpy as np
import roomscan.panel as panel_mod


class _FakeGizmoScene:
    def __init__(self):
        self.geoms = {}

    def has_geometry(self, n):
        return n in self.geoms

    def add_geometry(self, n, g, m):
        self.geoms[n] = g

    def remove_geometry(self, n):
        self.geoms.pop(n, None)

    def set_geometry_transform(self, n, t):
        pass


class _FakeGizmoPanel:
    def __init__(self, camera):
        import open3d as o3d
        self._o3d = o3d
        self.imu_gizmo = True
        self.camera_mode = camera
        self.gizmo_scale = 0.15
        self._gizmo_added = False
        self.mesh_material = "M"
        self.scene_widget = type("SW", (), {"scene": _FakeGizmoScene()})()


def test_gizmo_not_added_in_first_person():
    pytest_o3d = __import__("pytest").importorskip("open3d")
    fake = _FakeGizmoPanel(panel_mod.CAM_FIRST_PERSON)
    quat = (1.0, 0.0, 0.0, 0.0)
    panel_mod.ControlPanel._update_camera_gizmo(fake, quat)
    assert panel_mod._GIZMO_GEOM not in fake.scene_widget.scene.geoms
    assert fake._gizmo_added is False


def test_gizmo_added_in_orbit():
    __import__("pytest").importorskip("open3d")
    fake = _FakeGizmoPanel(panel_mod.CAM_ORBIT)
    panel_mod.ControlPanel._update_camera_gizmo(fake, (1.0, 0.0, 0.0, 0.0))
    assert panel_mod._GIZMO_GEOM in fake.scene_widget.scene.geoms
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k gizmo -v`
Expected: FAIL — the current `_update_camera_gizmo` gates only on `self.imu_gizmo`, so it adds the gizmo even in first-person.

- [ ] **Step 3: Gate the gizmo on camera mode**

In `_update_camera_gizmo` (line 2538), change the guard from `if self.imu_gizmo and quat_display is not None:` to:

```python
    def _update_camera_gizmo(self, quat_display):
        if gizmo_should_update(getattr(self, "camera_mode", CAM_ORBIT), self.imu_gizmo) \
                and quat_display is not None:
```

(body unchanged). This single gate covers both call sites (`_render_frame` line 1191 and `_update_sensors` line 2573–2574), since both funnel through this method. Additionally, in `_update_sensors` (line 2573), change the pre-check `if self.imu_gizmo and quat_display is not None:` to `if gizmo_should_update(getattr(self, "camera_mode", CAM_ORBIT), self.imu_gizmo) and quat_display is not None:` for symmetry (avoids a redundant call in first-person).

- [ ] **Step 4: Run to verify it passes**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_modes.py
git commit -m "fix(panel): gate IMU gizmo on orbit camera -> no first-person flicker"
```

---

## Task 8: `settings_dialog.py` + menubar (retire the sidebar)

**Files:**
- Create: `host/src/roomscan/settings_dialog.py`
- Modify: `host/src/roomscan/panel.py` (`__init__` build sequence, `_build_menubar`, remove `_build_panel` from the build path)
- Test: `host/tests/test_settings_dialog.py`

**Interfaces:**
- Produces:
  - `SECTIONS: list[str] = ["View", "Surface", "IR Monitor", "Device", "Capture"]` — the section order, unit-testable without a GUI.
  - `build_settings_dialog(panel, *, section: str | None = None) -> gui.Dialog` — builds one `CollapsableVert`-per-section dialog that reads/writes `panel` state and wires each control to the panel's existing `_on_*` handlers; opens with `section` expanded (others collapsed) when given. GUI-only; supervised-run verified.
  - `def build_menubar(panel) -> None` (a panel method `_build_menubar` — see below) creating `View`, `Device`, `Overlays`, `Help` menus.

**Design:** The heavy control-building logic currently in `_build_panel` (lines 772–996) moves into `settings_dialog.build_settings_dialog`, reusing `panel._group` / `panel._labeled_grid` (kept as panel methods) and the panel's existing `_on_*` handlers unchanged. The panel no longer calls `_build_panel`; instead it calls `_build_menubar`. The `Status` and `Events` groups (live readouts) are dropped from the settings dialog — Status is surfaced by the HUD status chip (Task 9); Events becomes an `Overlays → Events` toggle of a floating log widget (kept minimal: reuse the existing `lv_events` in a small dialog opened from the menu, OR leave `lv_events` construction in place but unparented and shown via a dialog). Keep it simple: put `Events` (the `lv_events` ListView) as a collapsible section at the bottom of the settings dialog too, so no live log widget floats. Update the `Status` labels (`lbl_conn`/`lbl_counts`) to still be constructed (the tick updates them) but placed in the settings dialog's top, non-collapsible — they remain harmless if the dialog is closed.

- [ ] **Step 1: Write the failing test** (pure section list only)

Create `host/tests/test_settings_dialog.py`:

```python
"""The GUI dialog build is supervised-run verified; here we only assert the
pure section ordering contract that the menubar relies on."""
from roomscan import settings_dialog


def test_sections_order():
    assert settings_dialog.SECTIONS[0] == "View"
    assert "Device" in settings_dialog.SECTIONS
    assert "IR Monitor" in settings_dialog.SECTIONS
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_settings_dialog.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'roomscan.settings_dialog'`.

- [ ] **Step 3: Create `settings_dialog.py`**

Create `host/src/roomscan/settings_dialog.py`. Move the group-building blocks out of `panel._build_panel` verbatim (View, Surface, IR Monitor, Sensors, Device, Capture, Events), rehomed as functions that take `panel` and append to a dialog `Vert`. Sketch (fill each section by lifting the exact widget-construction lines from `_build_panel`, lines 786–994, changing `self.` → `panel.` and `self._group(...)` → a local `_group(panel, dlg_parent, title)` that adds to the dialog instead of the retired side panel):

```python
"""One menu-opened settings dialog carrying the former sidebar's grouped
controls (spec §5.4). Reuses the panel's existing _on_* handlers; the panel
holds no always-visible side panel anymore.
"""
from __future__ import annotations

SECTIONS = ["View", "Surface", "IR Monitor", "Device", "Capture"]


def build_settings_dialog(panel, *, section: str | None = None):
    gui = panel._gui
    em = panel.window.theme.font_size
    dlg = gui.Dialog("Settings")
    root = gui.Vert(0.25 * em, gui.Margins(em, em, em, em))
    # ... build View / Surface / IR Monitor / Device / Capture / Events groups by
    # lifting the widget-construction lines from the old ControlPanel._build_panel,
    # with self -> panel. Each group is a gui.CollapsableVert added to `root`,
    # opened iff its title == section.
    close = gui.Button("Close")
    close.set_on_clicked(panel.window.close_dialog)
    row = gui.Horiz(); row.add_stretch(); row.add_child(close)
    root.add_child(row)
    dlg.add_child(root)
    return dlg
```

> Implementation note: the control widgets (`panel.cb_color`, `panel.sl_point`, `panel.cb_near`, `panel.sl_near`, `panel.chk_surface`, `panel.cb_ir`, `panel.chk_freeze`, `panel.cb_usecase`, `panel.sl_exposure`, `panel.btn_record`, `panel.sl_fps`, `panel.lv_events`, etc.) must still be assigned onto `panel` (the tick and handlers reference them by attribute). Build them here and assign to `panel` exactly as `_build_panel` did. Because a `gui.Dialog` is rebuilt each open, construct the widgets **once** in `panel._build_settings_widgets()` (called from `__init__`) and have `build_settings_dialog` only *parent* them into fresh `CollapsableVert`s — OR accept rebuilding per open. Simplest correct approach: build widgets once in `__init__` into a detached `gui.Vert` stored as `panel._settings_root`, and `build_settings_dialog` wraps that single root in a `Dialog`. Keep whichever the supervised run shows Open3D allows (a widget can only have one parent — if re-parenting throws, keep the persistent-root approach).

- [ ] **Step 4: Add the menubar to the panel**

In `host/src/roomscan/panel.py`, add a `_build_menubar` method and call it from `__init__` in place of `self._build_panel()` (line 688). Menubar per spec §5.4:

```python
    def _build_menubar(self):
        gui = self._gui
        app = gui.Application.instance
        menubar = gui.Menu()

        view_menu = gui.Menu()
        view_menu.add_item("Settings...", 100)
        device_menu = gui.Menu()
        device_menu.add_item("Settings...", 101)
        overlays = gui.Menu()
        overlays.add_item("Metrics HUD", 110)
        overlays.set_checked(110, self.metrics_overlay)
        overlays.add_item("Sensors", 111)
        overlays.set_checked(111, self.sensors_panel)
        help_menu = gui.Menu()
        help_menu.add_item("Help", 120)

        menubar.add_menu("View", view_menu)
        menubar.add_menu("Device", device_menu)
        menubar.add_menu("Overlays", overlays)
        menubar.add_menu("Help", help_menu)
        app.menubar = menubar

        self.window.set_on_menu_item_activated(100, lambda: self._open_settings("View"))
        self.window.set_on_menu_item_activated(101, lambda: self._open_settings("Device"))
        self.window.set_on_menu_item_activated(110, self._toggle_metrics_menu)
        self.window.set_on_menu_item_activated(111, self._toggle_sensors_menu)
        self.window.set_on_menu_item_activated(120, self._show_help)

    def _open_settings(self, section):
        from . import settings_dialog
        self.window.show_dialog(settings_dialog.build_settings_dialog(self, section=section))

    def _toggle_metrics_menu(self):
        self.metrics_overlay = not self.metrics_overlay
        self.overlay.visible = self.metrics_overlay
        self.bus.publish(f"metrics overlay -> {'on' if self.metrics_overlay else 'off'}")

    def _toggle_sensors_menu(self):
        self.sensors_panel = not self.sensors_panel
        self.bus.publish(f"sensors -> {'on' if self.sensors_panel else 'off'}")
```

Then: build the settings widgets once in `__init__` (extract `_build_settings_widgets` from the old `_build_panel` body — the widget construction, minus the `self.panel`/`ScrollableVert` scaffolding), remove the `self.panel`/`_build_panel` construction, and update `_on_layout` (line 998) to give the `scene_widget` the **full** content rect (no `panel_w` split) since there's no sidebar:

```python
    def _on_layout(self, ctx):
        gui = self._gui
        r = self.window.content_rect
        if r.width <= 0 or r.height <= 0:
            return
        self.scene_widget.frame = gui.Rect(r.x, r.y, r.width, r.height)
        # ... metrics HUD / banner / progress / reveal-card positioning unchanged ...
        # ... HUD control widgets positioned here in Task 9 ...
```

Keep the metrics-HUD/banner/progress/reveal-card frame math from the existing `_on_layout` (lines 1013–1036), just sourced from the full-width scene.

- [ ] **Step 5: Run pure test + supervised build check**

Run: `.venv\Scripts\python.exe -m pytest tests/test_settings_dialog.py -v`
Expected: PASS (pure section list).

Supervised (dev box): `cd host && .venv\Scripts\python.exe -m roomscan.panel --panel --replay <a capture.bin>` — confirm the window opens with **no sidebar**, a menubar with View/Device/Overlays/Help, and `View → Settings...` opens the grouped dialog with working controls. (No headless assertion — Filament.)

- [ ] **Step 6: Commit**

```bash
git add host/src/roomscan/settings_dialog.py host/src/roomscan/panel.py host/tests/test_settings_dialog.py
git commit -m "feat(panel): menu-driven settings dialog; retire the sidebar"
```

---

## Task 9: Floating HUD widgets — positioning + mouse routing

**Probe gate (from Task 0):** `PROBE RESULT: <fill in from Task 0>`. If `image-transparent`, implement the pure image + `hit_test` path (Steps below). If `image-opaque`, additionally place an invisible/borderless `gui.Button` per control rect (same rects from `HudLayout.rects()`), whose `set_on_clicked` runs the same action the hit-test dispatch would — the image widgets then serve as visuals only.

**Files:**
- Modify: `host/src/roomscan/panel.py` (`__init__` state + HUD ImageWidgets, `_build_overlay`, `_on_layout`, `_on_mouse`, new `_dispatch_hud_hit`, `_refresh_hud_images`)
- Test: `host/tests/test_panel_modes.py` (add hit-dispatch stand-in tests)

**Interfaces:**
- Consumes: `hud.{HudLayout, ControlHit, render_*, SIZES, MODE_SWITCH, VIEW_TOGGLE, ACTION_CLUSTER, IR_CONTROL, STATUS_CHIP}` (Tasks 2–3); `follow_active`, `CAM_*`, `VIEW_*` (Task 5).
- Produces (Task 10/11 read/set these): `self.mode`, `self.camera_mode`, `self.ir_overlay_enabled`, `self.ir_opacity`, and the dispatch method `_dispatch_hud_hit(hit: ControlHit) -> bool` (True if it consumed the click).

- [ ] **Step 1: Write the failing tests** (append to `host/tests/test_panel_modes.py`)

Test the dispatch logic on an unbound stand-in (no GUI): a click on the mode switch's SLAM segment sets `mode`; on the view toggle's ORBIT segment sets `camera_mode`; on the IR track sets `ir_opacity`.

```python
class _FakeHudPanel:
    def __init__(self):
        self.mode = panel_mod.VIEW_REAL_TIME
        self.camera_mode = panel_mod.CAM_FIRST_PERSON
        self.ir_overlay_enabled = False
        self.ir_opacity = 0.5
        self._mode_calls = []
        self._cam_calls = []

    # stubs the dispatch calls into (Task 10 supplies the real ones)
    def _set_mode(self, m): self.mode = m; self._mode_calls.append(m)
    def _set_camera(self, c): self.camera_mode = c; self._cam_calls.append(c)
    def _do_action(self, seg): pass
    def _toggle_ir_overlay(self): self.ir_overlay_enabled = not self.ir_overlay_enabled
    def _set_ir_opacity(self, f): self.ir_opacity = f
    def _hud_action_labels(self):
        return ["REC", "LOAD", "CLR"]


def test_dispatch_mode_switch_sets_slam():
    from roomscan.hud import ControlHit, MODE_SWITCH
    fake = _FakeHudPanel()
    consumed = panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(MODE_SWITCH, segment=1))
    assert consumed is True
    assert fake.mode == panel_mod.VIEW_SLAM


def test_dispatch_view_toggle_sets_orbit():
    from roomscan.hud import ControlHit, VIEW_TOGGLE
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(VIEW_TOGGLE, segment=1))
    assert fake.camera_mode == panel_mod.CAM_ORBIT


def test_dispatch_ir_fraction_sets_opacity():
    from roomscan.hud import ControlHit, IR_CONTROL
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(IR_CONTROL, fraction=0.75))
    assert fake.ir_opacity == 0.75


def test_dispatch_ir_label_toggles():
    from roomscan.hud import ControlHit, IR_CONTROL
    fake = _FakeHudPanel()
    panel_mod.ControlPanel._dispatch_hud_hit(fake, ControlHit(IR_CONTROL, segment=0))
    assert fake.ir_overlay_enabled is True
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k dispatch -v`
Expected: FAIL — `_dispatch_hud_hit` doesn't exist yet.

- [ ] **Step 3: Add HUD state + widgets in `__init__`**

In `ControlPanel.__init__`, after the existing view/config-backed state (~line 549), add:

```python
        # Two-mode / two-camera redesign state (spec 2026-07-13).
        self.mode = args.mode if getattr(args, "mode", None) in (VIEW_REAL_TIME, VIEW_SLAM) else VIEW_SLAM
        self.camera_mode = args.camera if getattr(args, "camera", None) in (CAM_FIRST_PERSON, CAM_ORBIT) else CAM_FIRST_PERSON
        self.ir_overlay_enabled = bool(getattr(args, "ir_overlay", False))
        self.ir_opacity = float(getattr(args, "ir_opacity", 0.5) or 0.5)
        self._hud_layout = None
        self._hud_img_state = {}    # control-id -> last (state-key) rendered, to skip re-render
```

Keep `self.follow_camera_enabled` but derive it from the mode/camera state at each set (Task 10). Note: `slam_enabled`/`showcase_enabled` become internal reflections of `mode == VIEW_SLAM` (Task 10 wires the toggles); leave their initializers as-is for now.

- [ ] **Step 4: Build the HUD ImageWidgets in `_build_overlay`**

Append to `_build_overlay` (after the reveal card, line 754):

```python
        # Floating HUD controls (spec §5.1): one ImageWidget per control, drawn
        # by hud.render_* and positioned in _on_layout. Interaction is routed
        # through _on_mouse -> HudLayout.hit_test (see _dispatch_hud_hit).
        from . import hud as _hud
        self._hud = _hud
        self.hud_widgets = {}
        for cid in (_hud.MODE_SWITCH, _hud.VIEW_TOGGLE, _hud.ACTION_CLUSTER,
                    _hud.IR_CONTROL, _hud.STATUS_CHIP):
            w = gui.ImageWidget(self._np_to_o3d_rgba(np.zeros((*_hud.SIZES[cid][::-1], 4), np.uint8)))
            self.window.add_child(w)
            self.hud_widgets[cid] = w
```

Add an RGBA image helper next to `_np_to_o3d` (line 2499):

```python
    def _np_to_o3d_rgba(self, rgba: np.ndarray):
        """(H,W,4) uint8 RGBA -> o3d Image for a floating HUD control."""
        return self._o3d.geometry.Image(np.ascontiguousarray(rgba))
```

- [ ] **Step 5: Position + refresh HUD images in `_on_layout` / on the tick**

In `_on_layout` (after the scene frame is set, Task 8's version), position each HUD widget from a fresh `HudLayout`:

```python
        self._hud_layout = self._hud.HudLayout(
            r.x, r.y, r.width, r.height, is_replay=self.is_replay, mode=self.mode)
        rects = self._hud_layout.rects()
        for cid, w in self.hud_widgets.items():
            if cid in rects:
                x, y, cw, ch = rects[cid]
                w.frame = gui.Rect(x, y, cw, ch)
                w.visible = self._hud_control_visible(cid)
            else:
                w.visible = False
```

Add `_hud_control_visible` and `_refresh_hud_images` (called from `_on_tick`'s ≤4 Hz block):

```python
    def _hud_control_visible(self, cid):
        if cid == self._hud.ACTION_CLUSTER:
            return self.mode == VIEW_SLAM
        if cid == self._hud.IR_CONTROL:
            return self.camera_mode == CAM_FIRST_PERSON
        return True

    def _refresh_hud_images(self):
        """Re-render each HUD control image only when its state changes."""
        h = self._hud
        phase = "idle"
        if self.mode == VIEW_SLAM and self.showcase_phase is not None:
            phase = self.showcase_phase.name.lower()
        tracking = getattr(self, "_hud_tracking", "--")
        states = {
            h.MODE_SWITCH: ("m", self.mode),
            h.VIEW_TOGGLE: ("v", self.camera_mode),
            h.ACTION_CLUSTER: ("a", phase, self.is_replay),
            h.IR_CONTROL: ("i", self.ir_overlay_enabled, round(self.ir_opacity, 2)),
            h.STATUS_CHIP: ("s", tracking, round(self._fps)),
        }
        renders = {
            h.MODE_SWITCH: lambda: h.render_mode_switch(self.mode),
            h.VIEW_TOGGLE: lambda: h.render_view_toggle(self.camera_mode),
            h.ACTION_CLUSTER: lambda: h.render_action_cluster(phase, self.is_replay),
            h.IR_CONTROL: lambda: h.render_ir_control(self.ir_overlay_enabled, self.ir_opacity),
            h.STATUS_CHIP: lambda: h.render_status_chip(tracking, self._fps),
        }
        for cid, key in states.items():
            if self._hud_img_state.get(cid) != key and self.hud_widgets[cid].visible:
                self.hud_widgets[cid].update_image(self._np_to_o3d_rgba(renders[cid]()))
                self._hud_img_state[cid] = key
```

Call `self._refresh_hud_images()` inside `_on_tick`'s `if now - self._last_ui >= _UI_PERIOD:` block (line 1131), and set `self._hud_tracking` where the SLAM tracking label is computed (Task 10).

- [ ] **Step 6: Route mouse through the hit-test**

At the very top of `_on_mouse` (line 2414), before the `follow_camera_enabled` swallow, intercept HUD hits on BUTTON_DOWN/DRAG:

```python
    def _on_mouse(self, e):
        gui = self._gui
        res = gui.SceneWidget.EventCallbackResult
        if self._hud_layout is not None and e.type in (
                gui.MouseEvent.Type.BUTTON_DOWN, gui.MouseEvent.Type.DRAG):
            hit = self._hud_layout.hit_test(int(e.x), int(e.y))
            if hit is not None:
                if self._dispatch_hud_hit(hit):
                    return res.CONSUMED
        # ... existing body (cam_target None guard, follow swallow, orbit/pan/zoom) ...
```

Add `_dispatch_hud_hit`:

```python
    def _dispatch_hud_hit(self, hit) -> bool:
        h = self._hud
        if hit.control == h.MODE_SWITCH:
            self._set_mode(VIEW_REAL_TIME if hit.segment == 0 else VIEW_SLAM)
            return True
        if hit.control == h.VIEW_TOGGLE:
            self._set_camera(CAM_FIRST_PERSON if hit.segment == 0 else CAM_ORBIT)
            return True
        if hit.control == h.ACTION_CLUSTER:
            labels = self._hud_action_labels()
            idx = min(len(labels) - 1, max(0, int((hit.fraction or 0.0) * len(labels))))
            self._do_action(idx)
            return True
        if hit.control == h.IR_CONTROL:
            if hit.fraction is not None:
                self._set_ir_opacity(hit.fraction)
            else:
                self._toggle_ir_overlay()
            return True
        if hit.control == h.STATUS_CHIP:
            return True    # read-only, but consume so it doesn't orbit the camera
        return False
```

`_set_mode`, `_set_camera`, `_do_action`, `_toggle_ir_overlay`, `_set_ir_opacity` are implemented in Task 10/11. For this task, add minimal stubs so the module imports and the dispatch tests pass:

```python
    def _set_mode(self, m): self.mode = m
    def _set_camera(self, c): self.camera_mode = c
    def _do_action(self, seg): pass
    def _toggle_ir_overlay(self): self.ir_overlay_enabled = not self.ir_overlay_enabled
    def _set_ir_opacity(self, f): self.ir_opacity = float(f)
```

- [ ] **Step 7: Run the dispatch tests**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k dispatch -v`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_modes.py
git commit -m "feat(panel): floating HUD widgets + mouse-routed hit-test dispatch"
```

---

## Task 10: Mode & camera switching — merge SLAM+Showcase, first-person in both modes, Load

**Files:**
- Modify: `host/src/roomscan/panel.py` (`_set_mode`, `_set_camera`, `_do_action`, `_apply_camera_mode`, `_load_dialog`, Real-Time first-person camera; reuse `_on_slam_toggle`/`_on_showcase_toggle` internals)
- Test: `host/tests/test_panel_modes.py` (Load-dispatch stand-in test)

**Interfaces:**
- Consumes: `follow_active`, `real_time_first_person`, `load_kind` (Task 5); the existing `_on_slam_toggle`, `_on_showcase_toggle`, `_enter_showcase_recording`, `_on_record`, `_on_clear_scan`, `_apply_follow_camera`, `follow_camera_target`.
- Produces: the real `_set_mode`/`_set_camera`/`_do_action` (replacing Task 9 stubs), `_hud_action_labels`.

**Semantics (from spec §3):**
- `_set_mode(VIEW_SLAM)`: enable the SLAM path (reuse `_on_slam_toggle(True)` internals — lazily builds `SlamWorker`+`MeshPrep`, removes the live cloud). SLAM sub-state (record→process→reveal) is the merged Showcase flow, so SLAM mode uses the Showcase state machine: set `self.showcase_enabled = True` and drive `showcase_phase`. **Decision:** merge by routing SLAM-mode rendering through the Showcase path (`_render_showcase_frame`), because that path already unifies live-preview (RECORDING), processing, and reveal. Live pose+map view before any recording = a lightweight "IDLE preview": in IDLE, run the same live `SlamWorker` preview the RECORDING phase uses (so the map shows live), without a `Recorder` writing. Simplest faithful wiring: keep `slam_enabled` driving `_render_slam_frame` for the live map, and layer the Showcase actions (Record/Load/Clear) on top via the action cluster — i.e. SLAM mode = `slam_enabled=True`; pressing Record additionally flips into the Showcase record→process→reveal flow. Implement `_set_mode(VIEW_SLAM)` as `_on_slam_toggle(True)` and `_do_action` Record as today's `_on_record` + `_enter_showcase_recording` bridge.
- `_set_mode(VIEW_REAL_TIME)`: `_on_slam_toggle(False)` and `_on_showcase_toggle(False)` (tears down workers, restores the raw cloud).
- `_set_camera(...)`: set `self.camera_mode`, recompute `self.follow_camera_enabled = follow_active(self.mode, self.camera_mode)`, and reuse `_on_follow_camera_toggle`'s restore-orbit branch when leaving first-person. In Real-Time first-person, apply the fixed sensor-origin camera (`_apply_real_time_first_person`).
- `_do_action(segment)`: map the segment to the phase-appropriate label from `_hud_action_labels()` and call: `REC`→`_on_record`(+showcase bridge), `STOP`→`_on_record` (toggles off, enters processing), `LOAD`→`_load_dialog`, `CLR`→`_on_clear_scan`.

- [ ] **Step 1: Write the failing test** (Load dispatch, append to `test_panel_modes.py`)

```python
def test_load_dialog_dispatches_by_kind(monkeypatch):
    calls = {}

    class _FakeLoadPanel:
        def _process_capture(self, path): calls["capture"] = path
        def _display_mesh_file(self, path): calls["mesh"] = path
        bus = type("B", (), {"publish": lambda self, m: None})()

    fake = _FakeLoadPanel()
    panel_mod.ControlPanel._load_path(fake, "captures/a.bin")
    panel_mod.ControlPanel._load_path(fake, "results/b.ply")
    panel_mod.ControlPanel._load_path(fake, "x.txt")
    assert calls == {"capture": "captures/a.bin", "mesh": "results/b.ply"}
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k load_dialog -v`
Expected: FAIL — `_load_path` doesn't exist.

- [ ] **Step 3: Implement mode/camera/action wiring** (replace the Task 9 stubs)

```python
    def _set_mode(self, m):
        if m == self.mode:
            return
        self.mode = m
        if m == VIEW_SLAM:
            self.chk_slam_checked = True                 # (no sidebar checkbox now)
            self._on_slam_toggle(True)
        else:
            if self.showcase_enabled:
                self._on_showcase_toggle(False)
            self._on_slam_toggle(False)
        self._apply_camera_mode()
        self.window.set_needs_layout()                   # ACTION_CLUSTER visibility changed
        self.bus.publish(f"mode -> {m}")

    def _set_camera(self, c):
        if c == self.camera_mode:
            return
        self.camera_mode = c
        self._apply_camera_mode()
        self.window.set_needs_layout()                   # IR_CONTROL visibility changed
        self.bus.publish(f"camera -> {c}")

    def _apply_camera_mode(self):
        """Reconcile the SLAM pose-follow flag + Real-Time fixed camera with the
        current (mode, camera)."""
        want_follow = follow_active(self.mode, self.camera_mode)
        if want_follow != self.follow_camera_enabled:
            self.follow_camera_enabled = want_follow
            self._follow_eye = None
            self._follow_center = None
            if not want_follow:
                self._apply_camera()                     # restore free-orbit
        if real_time_first_person(self.mode, self.camera_mode):
            self._apply_real_time_first_person()

    def _apply_real_time_first_person(self):
        """Real-Time first-person: a fixed sensor-origin camera looking along
        the sensor +Z (the raw cloud is already in the sensor frame), per spec
        §3. Eye slightly behind the origin; center one metre ahead."""
        if self.scene_widget.frame.width <= 0 or self.scene_widget.frame.height <= 0:
            return
        eye = np.array([0.0, 0.0, -_FOLLOW_BACK_OFF_M], dtype=np.float32)
        center = np.array([0.0, 0.0, _FOLLOW_LOOK_AHEAD_M], dtype=np.float32)
        self.scene_widget.look_at(center, eye, _WORLD_UP)

    def _hud_action_labels(self):
        phase = self.showcase_phase.name.lower() if (self.mode == VIEW_SLAM and self.showcase_phase) else "idle"
        return {
            "idle": ["REC", "LOAD", "CLR"],
            "recording": ["STOP", "CLR"],
            "processing": ["PROCESSING"],
            "final": ["LOAD", "CLR"],
        }.get(phase, ["REC", "LOAD", "CLR"])

    def _do_action(self, segment):
        labels = self._hud_action_labels()
        if segment is None or segment >= len(labels):
            return
        label = labels[segment]
        if label in ("REC", "STOP"):
            # Toggle the record button + run the existing handler (which bridges
            # into the Showcase record->process flow when showcase_enabled).
            self.btn_record.is_on = not self.btn_record.is_on
            self._on_record()
        elif label == "LOAD":
            self._load_dialog()
        elif label == "CLR":
            self._on_clear_scan()

    def _load_dialog(self):
        gui = self._gui
        dlg = gui.FileDialog(gui.FileDialog.OPEN, "Load capture or mesh", self.window.theme)
        dlg.add_filter(".bin", "Capture (.bin)")
        dlg.add_filter(".ply", "Mesh (.ply)")
        dlg.set_on_cancel(self.window.close_dialog)

        def _chosen(path):
            self.window.close_dialog()
            self._load_path(path)
        dlg.set_on_done(_chosen)
        self.window.show_dialog(dlg)

    def _load_path(self, path):
        kind = load_kind(path)
        if kind == "capture":
            self._process_capture(path)
        elif kind == "mesh":
            self._display_mesh_file(path)
        else:
            self.bus.publish(f"load: unsupported file {path!r}")

    def _process_capture(self, path):
        """Run the existing Showcase record->process->reveal pipeline on a saved
        .bin (spec §3): ensure SLAM mode + Showcase on, then reuse
        _enter_showcase_processing's loader against the given path."""
        if self.mode != VIEW_SLAM:
            self._set_mode(VIEW_SLAM)
        if not self.showcase_enabled:
            self._on_showcase_toggle(True)
        from .slam.showcase import next_phase
        self.showcase_phase = next_phase(self.showcase_phase, record_pressed=True)   # -> RECORDING
        self._enter_showcase_processing(path)          # tears into PROCESSING on `path`
        self.bus.publish(f"load capture -> {path}")

    def _display_mesh_file(self, path):
        """Display a saved .ply mesh (orbit, no reprocessing), spec §3."""
        o3d = self._o3d
        try:
            mesh = o3d.io.read_triangle_mesh(path)
        except Exception as exc:
            self.bus.publish(f"load mesh failed: {exc!r}")
            return
        if len(mesh.triangles) == 0:
            self.bus.publish("load mesh: empty")
            return
        self._set_camera(CAM_ORBIT)
        mesh.compute_vertex_normals()
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        sc.add_geometry(_MESH_GEOM, mesh, self.mesh_material)
        self._camera_set = False
        self._last_all_pts = np.asarray(mesh.vertices)
        self._reset_camera()
        self.bus.publish(f"load mesh -> {path}")
```

> `_process_capture` reuses the exact Showcase loader path (`_enter_showcase_processing` → `_start_showcase_post_process`), so `.bin` load and the Record→Stop flow share one code path (spec §3: "Load `.bin` → runs the existing capture→process→reveal pipeline").

- [ ] **Step 4: Run the Load test + full modes suite**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -v`
Expected: PASS.

- [ ] **Step 5: Supervised run (dev box)**

`cd host && .venv\Scripts\python.exe -m roomscan.panel --panel --replay <capture.bin>`
Confirm: mode switch flips Real-Time↔SLAM; view toggle flips first-person↔orbit (first-person rides the pose in SLAM, sits at the sensor origin in Real-Time); the action cluster shows REC/LOAD/CLR and Record→Stop→reveal works; `LOAD` opens a file dialog and a `.ply` displays in orbit.

- [ ] **Step 6: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_modes.py
git commit -m "feat(panel): mode/camera switching, SLAM+Showcase merge, Load .bin/.ply"
```

---

## Task 11: First-person IR overlay

**Files:**
- Modify: `host/src/roomscan/panel.py` (`__init__` material, `_update_ir_overlay`, call sites in `_render_slam_frame`/`_render_showcase_recording` and the Real-Time first-person path, `_toggle_ir_overlay`/`_set_ir_opacity` real impls, teardown)
- Test: `host/tests/test_panel_modes.py` (overlay geometry stand-in test)

**Interfaces:**
- Consumes: `ir_overlay.camera_locked_quad` (Task 4); `reflectance_to_rgb`, `ir_range` (existing); `follow_camera_target` (existing).
- Produces: `_IR_OVERLAY_GEOM = "__ir_overlay__"` geometry; `_update_ir_overlay(eye, forward)`; `_remove_ir_overlay()`.

- [ ] **Step 1: Write the failing test** (stand-in, append to `test_panel_modes.py`)

```python
def test_ir_overlay_builds_and_removes_geometry():
    __import__("pytest").importorskip("open3d")
    import numpy as np
    import open3d as o3d

    class _Scene:
        def __init__(self): self.geoms = {}
        def has_geometry(self, n): return n in self.geoms
        def add_geometry(self, n, g, m): self.geoms[n] = g
        def remove_geometry(self, n): self.geoms.pop(n, None)

    class _Fake:
        def __init__(self):
            self._o3d = o3d
            self.scene_widget = type("SW", (), {"scene": _Scene()})()
            self.args = type("A", (), {"fov_h": 55.0, "fov_v": 42.0})()
            self.ir_opacity = 0.5
            self._latest_outputs = {"reflectance": np.full((42, 54), 0.5, np.float32)}
            self.ir_colormap = "gray"
            self.ir_overlay_material = "M"

    fake = _Fake()
    panel_mod.ControlPanel._update_ir_overlay(fake, [0, 0, 0], [0, 0, 1])
    assert panel_mod._IR_OVERLAY_GEOM in fake.scene_widget.scene.geoms
    panel_mod.ControlPanel._remove_ir_overlay(fake)
    assert panel_mod._IR_OVERLAY_GEOM not in fake.scene_widget.scene.geoms
```

- [ ] **Step 2: Run to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k ir_overlay -v`
Expected: FAIL — `_IR_OVERLAY_GEOM`/`_update_ir_overlay` don't exist.

- [ ] **Step 3: Implement the overlay**

Add the geometry-name constant near the others (line 84):

```python
_IR_OVERLAY_GEOM = "__ir_overlay__"    # first-person IR billboard (spec §5.2)
```

Add a transparent-unlit material in `__init__` (next to `wall_translucent_material`, line 662):

```python
        self.ir_overlay_material = rendering.MaterialRecord()
        self.ir_overlay_material.shader = "defaultUnlitTransparency"
        self.ir_overlay_material.base_color = [1.0, 1.0, 1.0, self.ir_opacity]
```

Add the overlay methods:

```python
    def _update_ir_overlay(self, eye, forward):
        """Build/refresh the first-person IR billboard quad (spec §5.2) from the
        latest reflectance frame. Only meaningful in first-person; callers gate
        on that. No-op (removes) when there's no IR frame."""
        from . import ir_overlay
        outputs = self._latest_outputs or {}
        refl = outputs.get("reflectance")
        if refl is None:
            self._remove_ir_overlay()
            return
        auto = ir_range(refl)
        rgb = reflectance_to_rgb(refl, colormap=self.ir_colormap, vmin=auto[0], vmax=auto[1],
                                 upscale=1)
        verts, uvs, tris = ir_overlay.camera_locked_quad(
            eye, forward, _WORLD_UP, self.args.fov_h, self.args.fov_v, dist=1.0)
        o3d = self._o3d
        m = o3d.geometry.TriangleMesh()
        m.vertices = o3d.utility.Vector3dVector(verts)
        m.triangles = o3d.utility.Vector3iVector(tris)
        m.triangle_uvs = o3d.utility.Vector2dVector(uvs[tris].reshape(-1, 2).astype(np.float64))
        m.textures = [o3d.geometry.Image(np.ascontiguousarray(rgb))]
        m.triangle_material_ids = o3d.utility.IntVector(np.zeros(len(tris), np.int32))
        self.ir_overlay_material.base_color = [1.0, 1.0, 1.0, float(self.ir_opacity)]
        sc = self.scene_widget.scene
        if sc.has_geometry(_IR_OVERLAY_GEOM):
            sc.remove_geometry(_IR_OVERLAY_GEOM)
        sc.add_geometry(_IR_OVERLAY_GEOM, m, self.ir_overlay_material)

    def _remove_ir_overlay(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_IR_OVERLAY_GEOM):
            sc.remove_geometry(_IR_OVERLAY_GEOM)

    def _toggle_ir_overlay(self):
        self.ir_overlay_enabled = not self.ir_overlay_enabled
        if not self.ir_overlay_enabled:
            self._remove_ir_overlay()
        self.bus.publish(f"IR overlay -> {'on' if self.ir_overlay_enabled else 'off'}")

    def _set_ir_opacity(self, fraction):
        self.ir_opacity = float(np.clip(fraction, 0.0, 1.0))
```

> The texture-mapping call shape (`triangle_uvs` / `textures` / `triangle_material_ids`) is Open3D-legacy-mesh-specific; confirm on the supervised run and adjust if Open3D 0.19 wants `o3d.t.geometry` instead. The unit test only asserts geometry presence/removal, so it's texture-API-agnostic.

- [ ] **Step 4: Wire the per-frame call**

In `_render_slam_frame`, inside `if self.follow_camera_enabled:` (line 1317, after `_apply_follow_camera`), add:

```python
            if self.ir_overlay_enabled:
                eye, _, _ = follow_camera_target(step.pose)
                fwd = step.pose[:3, 2]
                self._update_ir_overlay(eye, fwd)
            else:
                self._remove_ir_overlay()
```

Mirror the same in `_render_showcase_recording` (line 1896 block). For the Real-Time first-person path, call `_update_ir_overlay([0,0,-_FOLLOW_BACK_OFF_M], [0,0,1])` from `_render_frame` when `real_time_first_person(self.mode, self.camera_mode) and self.ir_overlay_enabled` (add a small block near the top of `_render_frame`'s non-SLAM branch). Ensure `_remove_ir_overlay()` is called when leaving first-person (add to `_apply_camera_mode`'s `if not want_follow` branch and to `_on_slam_toggle(False)`).

- [ ] **Step 5: Run the overlay test**

Run: `.venv\Scripts\python.exe -m pytest tests/test_panel_modes.py -k ir_overlay -v`
Expected: PASS.

- [ ] **Step 6: Supervised run (dev box)**

Confirm the IR overlay appears as a billboard in front of the first-person camera, that the HUD IR slider changes its opacity, and that it disappears in orbit / when toggled off / when no IR frame is present.

- [ ] **Step 7: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_panel_modes.py
git commit -m "feat(panel): first-person IR billboard overlay with opacity control"
```

---

## Task 12: Config persistence end-to-end + defaults on startup

**Files:**
- Modify: `host/src/roomscan/panel.py` (verify `__init__` reads `args.mode/camera/ir_overlay/ir_opacity`; ensure the startup mode/camera are actually *applied*, not just stored)
- Test: `host/tests/test_config_ui.py` (add a startup-apply stand-in test)

**Interfaces:**
- Consumes: everything from Tasks 6, 9, 10, 11.

- [ ] **Step 1: Write the failing test** (append to `test_config_ui.py`)

```python
import roomscan.panel as panel_mod


def test_startup_applies_saved_mode_camera():
    # A minimal args namespace with saved SLAM+orbit should leave the panel's
    # mode/camera fields matching (the __init__ normalizer, tested unbound).
    class _Args:
        mode = "slam"; camera = "orbit"; ir_overlay = True; ir_opacity = 0.7
    # emulate the normalizer __init__ runs (Task 9 block)
    a = _Args()
    mode = a.mode if a.mode in (panel_mod.VIEW_REAL_TIME, panel_mod.VIEW_SLAM) else panel_mod.VIEW_SLAM
    camera = a.camera if a.camera in (panel_mod.CAM_FIRST_PERSON, panel_mod.CAM_ORBIT) else panel_mod.CAM_FIRST_PERSON
    assert (mode, camera) == (panel_mod.VIEW_SLAM, panel_mod.CAM_ORBIT)
    assert bool(a.ir_overlay) is True
    assert float(a.ir_opacity) == 0.7
```

(This guards the normalizer contract without a window; the real apply-on-startup is supervised.)

- [ ] **Step 2: Ensure startup applies the mode/camera**

At the end of `__init__` (after `_build_menubar`/overlay build, before `self.bus.publish("connected...")`, ~line 694), add:

```python
        # Apply the saved/CLI mode + camera so the window opens in the right
        # state (spec §3: both default to first-person; mode defaults to SLAM).
        if self.mode == VIEW_SLAM:
            self._on_slam_toggle(True)
        self._apply_camera_mode()
```

Guard `_apply_camera_mode` against a not-yet-framed camera (`_cam_target is None`): the existing `_apply_camera`/`look_at` guards already early-return on a degenerate viewport, so this is safe pre-first-frame.

- [ ] **Step 3: Run the test + full suite**

Run: `.venv\Scripts\python.exe -m pytest tests/test_config_ui.py -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add host/src/roomscan/panel.py host/tests/test_config_ui.py
git commit -m "feat(panel): apply saved mode/camera on startup"
```

---

## Task 13: Retire stale sidebar tests + live smoke script + full-suite green

**Files:**
- Create: `host/tools/panel_ui_smoke.py`
- Modify/remove: any test asserting on retired sidebar widgets (search first — see Step 1)
- Test: whole `host/tests` suite

- [ ] **Step 1: Find tests tied to retired sidebar widgets**

Run (Grep, not shell): search `host/tests` for references to the retired always-visible panel: `self.panel`, `_build_panel`, `chk_slam`, `chk_showcase`, `chk_follow_camera`, `chk_trajectory`, `panel_width` layout. For each hit, decide: (a) still valid (widget still built in the settings dialog / still an attribute) → leave; (b) asserts the old sidebar layout/visibility → update to the new menu/HUD reality or delete. Do **not** weaken a test that still checks real behavior.

- [ ] **Step 2: Write the supervised smoke script**

Create `host/tools/panel_ui_smoke.py` — drives the redesigned panel through both modes and both cameras via `run(args, smoke_ticks=N)` and screenshots, for supervised review on the dev box:

```python
"""Supervised UI smoke for the panel redesign. Dev box only (Filament needs a
display). Cycles Real-Time/SLAM x first-person/orbit and asserts no crash.

  cd host && .venv\\Scripts\\python.exe tools\\panel_ui_smoke.py <capture.bin>
"""
import sys

from roomscan.panel import _resolve, run


def main():
    argv = ["--panel", "--replay", sys.argv[1]] if len(sys.argv) > 1 else ["--panel"]
    args = _resolve(argv)
    rc = run(args, smoke_ticks=60)     # opens, ticks, tears down cleanly
    print(f"[smoke] exit {rc}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Run the full unit suite headless**

Run: `.venv\Scripts\python.exe -m pytest -q`
Expected: PASS (all green; count should be ≥ the pre-redesign 506 plus the new tests, minus any deleted stale sidebar tests).

- [ ] **Step 4: Supervised smoke on the dev box**

Run: `cd host && .venv\Scripts\python.exe tools\panel_ui_smoke.py <a capture.bin>`
Expected: `[smoke] exit 0`, no traceback. Then a manual open (`-m roomscan.panel --panel --replay <bin>`) to eyeball: no sidebar, menubar present, HUD controls visible and clickable, first-person default, IR overlay + opacity work, gizmo doesn't flicker, mode/camera/ir settings persist across a `--save-config` run.

- [ ] **Step 5: Run status-sync + commit**

Invoke the `status-sync` skill (mandatory at ship time — docs move with the code), update `CLAUDE.md`/`ROADMAP.md`/memory as it directs, then:

```bash
git add host/tools/panel_ui_smoke.py host/tests
git commit -m "test(panel): retire stale sidebar tests; add UI smoke script"
```

---

## Self-Review

**1. Spec coverage** (each spec section → task):
- §1/§3 two modes (REAL_TIME/SLAM), SLAM=SLAM+Showcase merged → Tasks 5, 10.
- §3 first-person default both modes; SLAM rides `step.pose`; Real-Time fixed sensor-origin +Z → Tasks 5, 10 (`_apply_real_time_first_person`, `follow_active`).
- §3 SLAM sub-state = Showcase `ShowcasePhase` state machine; FINAL auto-orbit reuse → Task 10 (reuses `_render_showcase_*`, `next_phase`).
- §3 Load `.bin`→pipeline, `.ply`→display → Task 10 (`_load_path`, `_process_capture`, `_display_mesh_file`).
- §3 mode-switch teardown reuse `_remove_live_view_geometries`/`_remove_slam_geometries` → Task 10 (via `_on_slam_toggle`/`_on_showcase_toggle`).
- §4 extract `hud.py`, `ir_overlay.py`, `settings_dialog.py`; panel = orchestrator → Tasks 2–4, 8, 9.
- §5.1 floating HUD (approach B), interaction via `set_on_mouse`+`hit_test`, plan-task-0 probe + fallback → Tasks 0, 2, 3, 9.
- §5.2 IR overlay (first-person only, `camera_locked_quad`, `defaultUnlitTransparency`, per-frame rebuild, remove when off/orbit/no-frame) → Tasks 4, 11.
- §5.3 gizmo-flicker fix (gate on ORBIT) → Tasks 5, 7.
- §5.4 menubar View/Device/Overlays/Help + single settings dialog; sliders in dialog not menu → Task 8.
- §6 config persistence adds mode/camera/ir_overlay/ir_opacity → Tasks 6, 12.
- §7 pure/unit tests (hud renders+hit-test, quad, transitions, gizmo predicate) + live smoke + retire old tests → each task's tests + Task 13.
- §8 risks: mouse fallthrough (Task 0 gate + Task 9 fallback), IR texture-per-frame perf (Task 11 supervised check), panel.py churn (extraction Tasks 2–4, 8).

No spec requirement is left without a task.

**2. Placeholder scan:** The wiring tasks (8–11) contain real code plus a few explicit "lift these exact lines from `_build_panel`/adjust to Open3D 0.19 on the supervised run" notes — these are pointers to *existing verbatim source*, not vague "handle it" placeholders, and each is bounded to a named method/line range. Acceptable given Filament can't be exercised headless (matches the repo's established "GUI wiring is supervised-run verified" norm). Pure tasks (1–7 predicates, hud, ir_overlay, config) have complete code and real assertions.

**3. Type consistency:** `follow_active`/`gizmo_should_update`/`real_time_first_person`/`load_kind` names match across Tasks 5/7/9/10. `ControlHit(control, segment, fraction)` and control-id constants (`MODE_SWITCH`…) match across Tasks 2/3/9. `SIZES` dims match between renders (Task 2) and layout (Task 3). `camera_locked_quad` signature matches Task 4 ↔ Task 11 call. Config field names (`mode`,`camera`,`ir_overlay`,`ir_opacity`) match Tasks 6/12 and `_persist_config`/`_PANEL_FIELDS`. `_IR_OVERLAY_GEOM`, `_update_ir_overlay`, `_remove_ir_overlay` consistent in Task 11. `showcase_phase.name.lower()` phase strings (`idle/recording/processing/final`) match `render_action_cluster`'s keys (Task 2) and `_hud_action_labels` (Task 10).
