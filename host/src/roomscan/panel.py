"""Open3D `visualization.gui` control panel for the roomscan viewer (Phase 3.5).

A real control panel to replace the classic keyboard-only Open3D window: a
`SceneWidget` renders the live point cloud on the left; a settings panel on the
right carries Status / Device / View / IR-Monitor / Capture / Events groups.

ALL data plumbing is reused unchanged from the classic viewer -- `TransformStage`,
`CommandClient`/`CommandDispatcher`, `Deprojector`, `sources`/`pump`, `config`,
`Stats`/`StreamDecoder`. This module is presentation only. The classic
`roomscan-view` window stays available (this is opt-in via `--panel` /
`roomscan-panel`).

Threading model (hard rules, mirrors the classic viewer's hard-won contract):
  * A single reader thread owns the source+decoder+transform, routes device
    EVENT/ACK to the log bus / CommandClient, and drops each rendered frame into
    a latest-wins slot (`queue.Queue(maxsize=1)`).
  * ALL scene/UI mutation happens on the GUI main thread, driven by
    `Window.set_on_tick_event` -- the tick polls the slot (cloud every frame it
    changes) and refreshes labels / IR pane / event log at <=4 Hz.
  * Serial writes (commands) run on `CommandDispatcher`'s short-lived worker
    threads, never on the reader or UI thread; their results come back as log-bus
    messages, drained on the UI thread. This keeps `SerialSource.write`'s "never
    on the reader thread" invariant intact.
"""
from __future__ import annotations

import os
import queue
import sys
import threading
import time
from pathlib import Path

import numpy as np

from .config import ViewerConfig, apply_config_defaults
from .control import CommandClient, CommandDispatcher
from .decoder import StreamDecoder
from .deproject import Deprojector
from .ir_image import ir_range, reflectance_to_rgb
from .logbus import LogBus
from .native import Transform
from . import portguard
from .pipeline import TransformStage
from .protocol import CommandCode, FrameType, ProtocolError, parse_event
from .shading import MODES as _NEAR_MODES
from .shading import cloud_colors
from .sources import FileSource, Recorder, SerialSource, pump
from .viewer import Stats, _build_arg_parser

# Usecase id -> label (only binning-2 profiles are usable at full res; see ROADMAP
# Phase 3 table -- AF_RANGE/AF are binning-4 and get REJECTED_BINNING by firmware).
_USECASES = [(0, "AR_RANGE (~32 fps)"), (1, "AR_PRECISION (~28 fps)")]
_COLOR_MODES = ("depth", "reflectance", "confidence")
_IR_COLORMAPS = ("gray", "turbo")
_IR_UPSCALE = 6                 # 54x42 zones -> 324x252 px, nearest-neighbor
_GEOM = "cloud"
_UI_PERIOD = 0.25               # <=4 Hz label / IR / log refresh
_EXPOSURE_DEBOUNCE = 0.4        # s to settle before sending a dragged exposure value
_BG_DARK = [0.05, 0.05, 0.08, 1.0]
_BG_LIGHT = [0.90, 0.90, 0.92, 1.0]

_HELP_LINES = [
    "",
    "Mouse:  left-drag orbit  |  ctrl/middle-drag pan  |  wheel zoom",
    "Key:    H  this help",
    "",
    "Status   fps, frame/seq-gap/drop/crc/raw counters, current usecase + color.",
    "Device   Ping / Request CALIB / Reinit; usecase; exposure (ms, sent on release).",
    "         (device controls are inactive in replay.)",
    "View     color mode (depth / reflectance IR / confidence);",
    "         point size (raise it to close the gaps between zones);",
    "         Near contrast (see below); dark background; Reset view.",
    "",
    "Near contrast -- spend more of the colormap on close targets (e.g. a face",
    "in front of a wall) so facial relief stands out:",
    "  window   : color only points within the cutoff distance and grey the rest",
    "             (slider = cutoff metres). Best for isolating a person.",
    "  emphasis : nonlinear boost of near depths (slider = strength); wall stays",
    "             colored but compressed.",
    "  equalize : auto histogram-equalize -- dense surfaces stretch, flat compress.",
    "  off      : plain linear depth coloring.",
    "",
    "IR Monitor  live 2D reflectance image; gray/turbo; Freeze holds the range.",
    "Capture     Record to captures/*.bin; replay adds Pause + fps.",
    "Events      device EVENTs, command results, connect/disconnect.",
    "",
    "Run with --save-config to persist the current view/IR/near settings.",
]


def _ir_freeze_range(freeze, frozen, auto):
    """Resolve the IR pane's display range for one frame (pure, unit-tested).

    `auto` is this frame's percentile auto-range; `frozen` is the currently
    captured frozen range or None. Returns `(vmin, vmax, frozen_out)`.

    When `freeze` is set, reuse `frozen` if present, else lazily capture this
    frame's `auto` as the frozen range and return it -- this is what makes freeze
    engage when it was set from config (the checkbox `.checked=True` never fires
    the toggle handler) or toggled before any reflectance frame arrived (nothing
    to freeze yet). When not frozen, pass `frozen` through untouched so a later
    re-freeze via the toggle handler still has the last value to fall back on.
    """
    if freeze:
        frozen = auto if frozen is None else frozen
        return frozen[0], frozen[1], frozen
    return auto[0], auto[1], frozen


class _Pacer:
    """Mutable replay-pacing + pause control shared with the reader thread.

    `interval` (seconds/frame, 0 = as-fast-as-decoded) is read live so the fps
    slider takes effect immediately; `paused` (an Event) blocks the reader
    between frames. Live capture leaves interval 0 and never pauses.
    """

    def __init__(self, interval: float = 0.0):
        self.interval = interval
        self.paused = threading.Event()


def _run_reader(source, decoder, stage, stats, slot, fault, bus, client, recorder,
                pacer, is_stopped):
    """Reader-thread body (module-level so it's unit-testable without a window).

    Owns source+decoder+transform; routes device EVENT -> log bus, ACK ->
    CommandClient, and each transformed DATA frame -> the latest-wins render
    slot. Honors the pacer's live `interval` (replay fps) and `paused` gate, and
    tees raw bytes into `recorder`. Any exception is surfaced via `fault` (unless
    we're stopping) exactly like the classic viewer's reader.
    """
    last_pace = 0.0
    last_paced_seq = None
    try:
        for frame in pump(source, decoder, recorder=recorder):
            if is_stopped():
                break
            ft = frame.header.frame_type
            if ft == FrameType.EVENT:
                try:
                    code, detail, msg = parse_event(frame.payload)
                    bus.publish(f"[event] code={code} detail={detail} {msg}")
                except ProtocolError:
                    bus.publish(f"[event] undecodable payload ({len(frame.payload)} B)")
                continue
            if ft == FrameType.ACK:
                if client is not None:
                    client.offer(frame)
                continue
            if ft != FrameType.DATA:
                continue
            result = stage.feed(frame)
            if result is None:
                continue
            header, outputs = result
            stats.update(header)
            while pacer.paused.is_set() and not is_stopped():
                time.sleep(0.05)
            if is_stopped():
                break
            interval = pacer.interval
            if interval > 0.0 and header.seq != last_paced_seq:
                wait = last_pace + interval - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                last_pace = time.monotonic()
                last_paced_seq = header.seq
            try:
                slot.get_nowait()
            except queue.Empty:
                pass
            slot.put((header, outputs))
    except Exception as exc:  # surface, don't vanish
        if not is_stopped():
            fault["error"] = exc


class ControlPanel:
    def __init__(self, args, source, client, stage, bus, recorder, pacer):
        import open3d as o3d
        import open3d.visualization.gui as gui
        import open3d.visualization.rendering as rendering
        self._o3d = o3d
        self._gui = gui

        self.args = args
        self.source = source
        self.client = client
        self.stage = stage
        self.bus = bus
        self.recorder = recorder
        self.pacer = pacer
        self.is_replay = isinstance(source, FileSource)

        self.decoder = StreamDecoder()
        self.stats = Stats()
        self.slot: queue.Queue = queue.Queue(maxsize=1)
        self.fault: dict = {}
        self._stop = False
        self._reader_thread: threading.Thread | None = None

        # render state
        self.deproj: Deprojector | None = None
        self.pcd = o3d.geometry.PointCloud()
        self._camera_set = False
        self._latest_outputs: dict | None = None
        self._color_fallback_warned = False
        self._shown = 0
        self._fps = 0.0
        self._fps_mark = (time.monotonic(), 0)
        self._fault_reported = False
        self._last_ui = 0.0

        # view/config-backed state (normalize out-of-set values so a hand-edited
        # config can't feed an unknown colormap into reflectance_to_rgb every tick)
        self.color_mode = args.color if args.color in _COLOR_MODES else "depth"
        self.ir_colormap = args.ir_colormap if getattr(args, "ir_colormap", None) in _IR_COLORMAPS else "gray"
        self.ir_freeze = bool(getattr(args, "ir_freeze_range", False))
        # near-contrast state
        self.near_mode = args.near_mode if getattr(args, "near_mode", None) in _NEAR_MODES else "window"
        self.near_cutoff_m = float(getattr(args, "near_cutoff_m", 1.5) or 1.5)
        self.near_emphasis = float(getattr(args, "near_emphasis", 0.5) or 0.5)
        self._ir_last_auto: tuple[float, float] | None = None
        self._ir_frozen: tuple[float, float] | None = None
        self._ir_unavailable_shown = False

        # command state
        self._pending_exposure: tuple[int, float] | None = None
        self._last_sent_exposure: int | None = None
        self.dispatcher = CommandDispatcher(client, on_message=self._on_cmd_message)

        # log pane state
        self._log_sub = bus.subscribe()
        self._log_lines: list[str] = []

        self.rendering = rendering
        self.window = gui.Application.instance.create_window("roomscan panel", 1280, 800)
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"
        self.material.point_size = float(getattr(args, "point_size", 5.0))
        self._dark_bg = True

        self._build_scene()
        self._build_panel()
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close)
        self.window.set_on_tick_event(self._on_tick)
        self.window.set_on_key(self._on_key)   # H -> help dialog
        self.bus.publish(f"connected: {'replay ' + str(args.replay) if self.is_replay else 'live ' + str(getattr(source, 'port', '?'))}")

    # ---- construction -------------------------------------------------------
    def _build_scene(self):
        gui = self._gui
        self.scene_widget = gui.SceneWidget()
        self.scene_widget.scene = self.rendering.Open3DScene(self.window.renderer)
        self.scene_widget.scene.set_background(_BG_DARK)
        self.window.add_child(self.scene_widget)

    def _build_panel(self):
        gui = self._gui
        em = self.window.theme.font_size
        margin = gui.Margins(em, em, em, em)
        self.panel = gui.ScrollableVert(0.5 * em, margin)

        # --- Status ---
        st = gui.CollapsableVert("Status", 0.25 * em, gui.Margins(em, 0, 0, 0))
        self.lbl_conn = gui.Label("connecting...")
        self.lbl_fps = gui.Label("fps: --")
        self.lbl_counts = gui.Label("frames 0 | gaps 0 | drops 0 | crc 0")
        self.lbl_mode = gui.Label(f"usecase --  |  color {self.color_mode}")
        for w in (self.lbl_conn, self.lbl_fps, self.lbl_counts, self.lbl_mode):
            st.add_child(w)
        self.panel.add_child(st)

        # --- Device ---
        dev = gui.CollapsableVert("Device", 0.25 * em, gui.Margins(em, 0, 0, 0))
        row = gui.Horiz(0.25 * em)
        for text, cmd, param, label in (
            ("Ping", CommandCode.PING, 0, "ping"),
            ("Request CALIB", CommandCode.SEND_CALIB, 0, "calib"),
            ("Reinit", CommandCode.REINIT, 0, "reinit"),
        ):
            b = gui.Button(text)
            b.horizontal_padding_em = 0.5
            b.set_on_clicked(lambda c=cmd, p=param, lb=label: self.dispatcher.dispatch(c, p, lb))
            row.add_child(b)
        dev.add_child(row)

        uc_row = gui.Horiz(0.25 * em)
        uc_row.add_child(gui.Label("Usecase"))
        self.cb_usecase = gui.Combobox()
        for _id, name in _USECASES:
            self.cb_usecase.add_item(name)
        self.cb_usecase.selected_index = 1 if str(getattr(self.args, "usecase", "")) != "0" else 0
        self.cb_usecase.set_on_selection_changed(self._on_usecase)
        uc_row.add_child(self.cb_usecase)
        dev.add_child(uc_row)

        ex_row = gui.Horiz(0.25 * em)
        ex_row.add_child(gui.Label("Exposure ms"))
        self.sl_exposure = gui.Slider(gui.Slider.INT)
        self.sl_exposure.set_limits(1, 30)
        self.sl_exposure.int_value = 5
        self.sl_exposure.set_on_value_changed(self._on_exposure_changed)
        ex_row.add_child(self.sl_exposure)
        self.lbl_exposure = gui.Label("5")
        ex_row.add_child(self.lbl_exposure)
        dev.add_child(ex_row)
        if self.is_replay:
            dev.add_child(gui.Label("(device controls inactive in replay)"))
        self.panel.add_child(dev)

        # --- View ---
        view = gui.CollapsableVert("View", 0.25 * em, gui.Margins(em, 0, 0, 0))
        c_row = gui.Horiz(0.25 * em)
        c_row.add_child(gui.Label("Color"))
        self.cb_color = gui.Combobox()
        for m in _COLOR_MODES:
            self.cb_color.add_item(m)
        self.cb_color.selected_index = _COLOR_MODES.index(self.color_mode)
        self.cb_color.set_on_selection_changed(self._on_color)
        c_row.add_child(self.cb_color)
        view.add_child(c_row)

        ps_row = gui.Horiz(0.25 * em)
        ps_row.add_child(gui.Label("Point size"))
        self.sl_point = gui.Slider(gui.Slider.INT)
        self.sl_point.set_limits(1, 20)     # wide enough to close the inter-zone gaps
        self.sl_point.int_value = int(self.material.point_size)
        self.sl_point.set_on_value_changed(self._on_point_size)
        ps_row.add_child(self.sl_point)
        view.add_child(ps_row)

        # Near contrast: give close targets (a person) more of the colormap so
        # facial relief stands out instead of washing into the wall's range.
        nc_row = gui.Horiz(0.25 * em)
        nc_row.add_child(gui.Label("Near contrast"))
        self.cb_near = gui.Combobox()
        for m in _NEAR_MODES:
            self.cb_near.add_item(m)
        self.cb_near.selected_index = _NEAR_MODES.index(self.near_mode)
        self.cb_near.set_on_selection_changed(self._on_near_mode)
        nc_row.add_child(self.cb_near)
        view.add_child(nc_row)

        na_row = gui.Horiz(0.25 * em)
        self.lbl_near = gui.Label("cutoff m")
        na_row.add_child(self.lbl_near)
        self.sl_near = gui.Slider(gui.Slider.DOUBLE)
        self.sl_near.set_on_value_changed(self._on_near_value)
        na_row.add_child(self.sl_near)
        self.lbl_near_val = gui.Label("")
        na_row.add_child(self.lbl_near_val)
        view.add_child(na_row)
        self._sync_near_slider()

        self.chk_bg = gui.Checkbox("Dark background")
        self.chk_bg.checked = True
        self.chk_bg.set_on_checked(self._on_bg)
        view.add_child(self.chk_bg)

        vb_row = gui.Horiz(0.25 * em)
        reset = gui.Button("Reset view")
        reset.set_on_clicked(self._on_reset_view)
        vb_row.add_child(reset)
        help_btn = gui.Button("Help (?)")
        help_btn.set_on_clicked(self._show_help)
        vb_row.add_child(help_btn)
        view.add_child(vb_row)
        self.panel.add_child(view)

        # --- IR Monitor ---
        ir = gui.CollapsableVert("IR Monitor", 0.25 * em, gui.Margins(em, 0, 0, 0))
        blank = self._o3d.geometry.Image(
            np.zeros((42 * _IR_UPSCALE, 54 * _IR_UPSCALE, 3), dtype=np.uint8))
        self.ir_widget = gui.ImageWidget(blank)
        ir.add_child(self.ir_widget)
        map_row = gui.Horiz(0.25 * em)
        map_row.add_child(gui.Label("Map"))
        self.cb_ir = gui.Combobox()
        for m in _IR_COLORMAPS:
            self.cb_ir.add_item(m)
        self.cb_ir.selected_index = _IR_COLORMAPS.index(self.ir_colormap) if self.ir_colormap in _IR_COLORMAPS else 0
        self.cb_ir.set_on_selection_changed(self._on_ir_colormap)
        map_row.add_child(self.cb_ir)
        ir.add_child(map_row)
        self.chk_freeze = gui.Checkbox("Freeze range")
        self.chk_freeze.checked = self.ir_freeze
        self.chk_freeze.set_on_checked(self._on_ir_freeze)
        ir.add_child(self.chk_freeze)
        self.panel.add_child(ir)

        # --- Capture ---
        cap = gui.CollapsableVert("Capture", 0.25 * em, gui.Margins(em, 0, 0, 0))
        self.btn_record = gui.Button("Record")
        self.btn_record.toggleable = True
        self.btn_record.set_on_clicked(self._on_record)
        cap.add_child(self.btn_record)
        if self.is_replay:
            self.btn_pause = gui.Button("Pause")
            self.btn_pause.toggleable = True
            self.btn_pause.set_on_clicked(self._on_pause)
            cap.add_child(self.btn_pause)
            fps_row = gui.Horiz(0.25 * em)
            fps_row.add_child(gui.Label("Replay fps"))
            self.sl_fps = gui.Slider(gui.Slider.INT)
            self.sl_fps.set_limits(0, 60)
            self.sl_fps.int_value = int(1.0 / self.pacer.interval) if self.pacer.interval > 0 else 0
            self.sl_fps.set_on_value_changed(self._on_fps)
            fps_row.add_child(self.sl_fps)
            cap.add_child(fps_row)
        self.panel.add_child(cap)

        # --- Events ---
        ev = gui.CollapsableVert("Events", 0.25 * em, gui.Margins(em, 0, 0, 0))
        self.lv_events = gui.ListView()
        self.lv_events.set_items([])
        ev.add_child(self.lv_events)
        self.panel.add_child(ev)

        self.window.add_child(self.panel)

    def _on_layout(self, ctx):
        gui = self._gui
        r = self.window.content_rect
        panel_w = int(getattr(self.args, "panel_width", 340))
        panel_w = min(panel_w, r.width - 100)
        self.scene_widget.frame = gui.Rect(r.x, r.y, r.width - panel_w, r.height)
        self.panel.frame = gui.Rect(r.x + r.width - panel_w, r.y, panel_w, r.height)

    # ---- lifecycle ----------------------------------------------------------
    def start(self):
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _on_close(self):
        self._stop = True
        self.pacer.paused.clear()          # unblock a paused reader so it can exit
        try:
            self.source.close()            # unblocks a blocking read; pump's finally re-closes harmlessly
        except Exception:
            pass
        if self._reader_thread is not None:
            self._reader_thread.join(timeout=1.5)   # don't leave a live thread into interpreter teardown
        try:
            self.recorder.close()
        except Exception:
            pass
        if getattr(self.args, "save_config", False):
            self._persist_config()
        self._gui.Application.instance.quit()
        return True

    def _persist_config(self):
        """Honor --save-config in panel mode: write the effective settings,
        including the runtime-adjusted panel fields, back to roomscan.toml."""
        try:
            cfg = ViewerConfig(
                color=self.color_mode, fov_h=self.args.fov_h, fov_v=self.args.fov_v,
                replay_fps=self.args.replay_fps, port=self.args.port,
                point_size=self.material.point_size, ir_colormap=self.ir_colormap,
                ir_freeze_range=self.ir_freeze, panel_width=int(getattr(self.args, "panel_width", 340)),
                near_mode=self.near_mode, near_cutoff_m=self.near_cutoff_m,
                near_emphasis=self.near_emphasis)
            path = cfg.save()
            self.bus.publish(f"saved config to {path}")
        except Exception as exc:  # never let a config write block window close
            self.bus.publish(f"config save failed: {exc!r}")

    # ---- reader thread ------------------------------------------------------
    def _reader_loop(self):
        _run_reader(self.source, self.decoder, self.stage, self.stats, self.slot,
                    self.fault, self.bus, self.client, self.recorder, self.pacer,
                    lambda: self._stop)

    # ---- tick (GUI main thread) --------------------------------------------
    def _on_tick(self):
        redraw = False
        try:
            item = self.slot.get_nowait()
        except queue.Empty:
            item = None
        if self.fault and not self._fault_reported:
            self.bus.publish(f"reader stopped: {self.fault['error']!r}")
            self._fault_reported = True
        if item is not None:
            self._render_frame(item)
            redraw = True
        now = time.monotonic()
        # debounced exposure send
        if self._pending_exposure is not None:
            val, ts = self._pending_exposure
            if now - ts >= _EXPOSURE_DEBOUNCE:
                self._pending_exposure = None
                if val != self._last_sent_exposure:
                    self._last_sent_exposure = val
                    self.dispatcher.dispatch(CommandCode.SET_EXPOSURE_MS, val, f"exposure {val}ms")
        if now - self._last_ui >= _UI_PERIOD:
            self._last_ui = now
            self._update_status()
            self._update_ir()
            self._drain_log()
            redraw = True
        return redraw

    def _render_frame(self, item):
        o3d = self._o3d
        header, outputs = item
        self._latest_outputs = outputs
        depth = outputs["depth"]
        h, w = depth.shape
        if self.deproj is None:
            self.deproj = Deprojector(w, h, self.args.fov_h, self.args.fov_v)
        pts = self.deproj(depth)
        self.pcd.points = o3d.utility.Vector3dVector(pts)
        if len(pts):
            plane = None if self.color_mode == "depth" else outputs.get(self.color_mode)
            if plane is not None:
                valid = np.isfinite(depth) & (depth > 0.0) & (depth < self.deproj.max_range_mm)
                vals = plane[valid].astype(np.float64, copy=False)
            else:
                if self.color_mode != "depth" and not self._color_fallback_warned:
                    self.bus.publish(f"no '{self.color_mode}' plane in stream — coloring by depth")
                    self._color_fallback_warned = True
                vals = pts[:, 2]
            colors = cloud_colors(vals, pts[:, 2], mode=self.near_mode,
                                  cutoff_m=self.near_cutoff_m, emphasis=self.near_emphasis)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)
        else:
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
        self._show_cloud()
        self._shown += 1

    def _show_cloud(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        sc.add_geometry(_GEOM, self.pcd, self.material)
        if not self._camera_set and len(self.pcd.points):
            self._reset_camera()

    def _reset_camera(self):
        bounds = self.pcd.get_axis_aligned_bounding_box()
        if bounds.get_extent().max() <= 0:
            return
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())
        self._camera_set = True

    def _update_status(self):
        now, mark = time.monotonic(), self._shown
        t0, m0 = self._fps_mark
        dt = now - t0
        if dt >= 0.5:
            self._fps = (mark - m0) / dt
            self._fps_mark = (now, mark)
        self.lbl_conn.text = ("replay: " + str(self.args.replay)) if self.is_replay \
            else ("live: " + str(getattr(self.source, "port", "?")))
        self.lbl_fps.text = f"fps: {self._fps:5.1f}"
        line = (f"frames {self.stats.frames} | gaps {self.stats.seq_gaps} "
                f"| drops {self.stats.dropped_flags} | crc {self.decoder.crc_failures} "
                f"| raw {self.stage.raw_transformed}")
        if self.stage.raw_skipped_awaiting_calib:
            line += f" | raw-skip {self.stage.raw_skipped_awaiting_calib}"
        self.lbl_counts.text = line
        uc = _USECASES[self.cb_usecase.selected_index][1].split()[0]
        self.lbl_mode.text = f"usecase {uc}  |  color {self.color_mode}"

    def _update_ir(self):
        outputs = self._latest_outputs
        if outputs is None:
            return
        refl = outputs.get("reflectance")
        if refl is None:
            if not self._ir_unavailable_shown:
                self.ir_widget.update_image(self._ir_placeholder())
                self._ir_unavailable_shown = True
            return
        self._ir_unavailable_shown = False
        auto = ir_range(refl)
        self._ir_last_auto = auto
        vmin, vmax, self._ir_frozen = _ir_freeze_range(self.ir_freeze, self._ir_frozen, auto)
        rgb = reflectance_to_rgb(refl, colormap=self.ir_colormap,
                                 vmin=vmin, vmax=vmax, upscale=_IR_UPSCALE)
        self.ir_widget.update_image(self._o3d.geometry.Image(np.ascontiguousarray(rgb)))

    def _ir_placeholder(self):
        img = np.zeros((42 * _IR_UPSCALE, 54 * _IR_UPSCALE, 3), dtype=np.uint8)
        img[:, :, 0] = 40  # dim maroon = "no IR"
        return self._o3d.geometry.Image(img)

    def _drain_log(self):
        new = self.bus.drain(self._log_sub)
        if not new:
            return
        self._log_lines.extend(new)
        if len(self._log_lines) > 200:
            self._log_lines = self._log_lines[-200:]
        self.lv_events.set_items(self._log_lines)

    # ---- callbacks ----------------------------------------------------------
    def _on_cmd_message(self, msg: str):
        self.bus.publish(f"[cmd] {msg}")

    def _on_usecase(self, text, index):
        self.dispatcher.dispatch(CommandCode.SET_USECASE, _USECASES[index][0], f"usecase {_USECASES[index][0]}")

    def _on_exposure_changed(self, value):
        v = int(value)
        self.lbl_exposure.text = str(v)
        self._pending_exposure = (v, time.monotonic())

    def _on_color(self, text, index):
        self.color_mode = text
        self.bus.publish(f"color -> {text}")

    def _on_point_size(self, value):
        self.material.point_size = float(int(value))
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.modify_geometry_material(_GEOM, self.material)

    def _on_bg(self, checked):
        self._dark_bg = checked
        self.scene_widget.scene.set_background(_BG_DARK if checked else _BG_LIGHT)

    def _on_reset_view(self):
        self._camera_set = False
        self._reset_camera()

    def _sync_near_slider(self):
        """Point the shared near-contrast slider at the control the current mode
        uses: distance cutoff (window), strength (emphasis), or disabled."""
        if self.near_mode == "window":
            self.lbl_near.text = "cutoff m"
            self.sl_near.enabled = True
            self.sl_near.set_limits(0.3, 5.0)
            self.sl_near.double_value = self.near_cutoff_m
            self.lbl_near_val.text = f"{self.near_cutoff_m:.1f}"
        elif self.near_mode == "emphasis":
            self.lbl_near.text = "strength"
            self.sl_near.enabled = True
            self.sl_near.set_limits(0.0, 1.0)
            self.sl_near.double_value = self.near_emphasis
            self.lbl_near_val.text = f"{self.near_emphasis:.2f}"
        else:                                    # off / equalize -> no scalar to tune
            self.lbl_near.text = "(no control)" if self.near_mode == "equalize" else "(off)"
            self.sl_near.enabled = False
            self.lbl_near_val.text = ""

    def _on_near_mode(self, text, index):
        self.near_mode = text
        self._sync_near_slider()
        self.bus.publish(f"near contrast -> {text}")

    def _on_near_value(self, value):
        if self.near_mode == "window":
            self.near_cutoff_m = float(value)
            self.lbl_near_val.text = f"{self.near_cutoff_m:.1f}"
        elif self.near_mode == "emphasis":
            self.near_emphasis = float(value)
            self.lbl_near_val.text = f"{self.near_emphasis:.2f}"

    def _show_help(self, *_):
        gui = self._gui
        em = self.window.theme.font_size
        dlg = gui.Dialog("Help")
        v = gui.Vert(0.3 * em, gui.Margins(em, em, em, em))
        v.add_child(gui.Label("roomscan control panel"))
        for line in _HELP_LINES:
            v.add_child(gui.Label(line))
        ok = gui.Button("Close")
        ok.set_on_clicked(self.window.close_dialog)
        row = gui.Horiz()
        row.add_stretch()
        row.add_child(ok)
        v.add_child(row)
        dlg.add_child(v)
        self.window.show_dialog(dlg)

    def _on_key(self, event):
        # H toggles the help dialog; let everything else fall through to the scene.
        if event.type == self._gui.KeyEvent.DOWN and event.key == self._gui.KeyName.H:
            self._show_help()
            return True
        return False

    def _on_ir_colormap(self, text, index):
        self.ir_colormap = text

    def _on_ir_freeze(self, checked):
        self.ir_freeze = checked
        if checked:
            self._ir_frozen = self._ir_last_auto  # freeze the most recent auto-range
            self.bus.publish("IR range frozen")
        else:
            self.bus.publish("IR range auto")

    def _on_record(self, *_):
        if self.btn_record.is_on:
            Path("captures").mkdir(parents=True, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = str(Path("captures") / f"panel_{ts}.bin")
            self.recorder.start(path)
            self.btn_record.text = "Recording..."
            self.bus.publish(f"recording -> {path}")
        else:
            self.recorder.stop()
            self.btn_record.text = "Record"
            self.bus.publish("recording stopped")

    def _on_pause(self, *_):
        if self.btn_pause.is_on:
            self.pacer.paused.set()
            self.btn_pause.text = "Resume"
            self.bus.publish("replay paused")
        else:
            self.pacer.paused.clear()
            self.btn_pause.text = "Pause"
            self.bus.publish("replay resumed")

    def _on_fps(self, value):
        v = int(value)
        self.pacer.interval = 1.0 / v if v > 0 else 0.0


# ---- entry points -----------------------------------------------------------
_PANEL_FIELDS = ("point_size", "ir_colormap", "ir_freeze_range", "panel_width",
                 "near_mode", "near_cutoff_m", "near_emphasis")


def _fill_panel_fields(args) -> None:
    """Fill the panel-only config-backed fields (no CLI flags for these) from
    roomscan.toml when absent. Idempotent: a value already set (e.g. by
    ``_resolve``) is left untouched, so calling this again in ``run()`` for
    viewer-delegated args is safe."""
    cfg = ViewerConfig.load()
    for name in _PANEL_FIELDS:
        if getattr(args, name, None) is None:
            setattr(args, name, getattr(cfg, name))


def _resolve(argv):
    ap = _build_arg_parser()
    if not any(a.dest == "panel" for a in ap._actions):
        ap.add_argument("--panel", action="store_true")
    args = ap.parse_args(argv)
    apply_config_defaults(args, ViewerConfig.load())
    _fill_panel_fields(args)
    return args


def _open_source(args):
    """Open the frame source, distinguishing a busy port (locked by another
    program -- offer to close it and retry) from a missing one (no scanner /
    bad path). Returns the source, or None after printing a clean message."""
    try:
        return FileSource(args.replay) if args.replay else SerialSource(args.port, args.baud)
    except Exception as exc:
        if args.replay:
            print(f"error: could not open replay file {args.replay!r}: {exc}", file=sys.stderr)
            return None
        kind = portguard.classify_open_error(exc)
        if kind == "missing":
            print(f"error: scanner not found: {exc}\n"
                  "       Check the USER USB cable (CDC CAFE:4001) and press the board's RESET button.",
                  file=sys.stderr)
            return None
        # busy (or unknown-but-permission): offer to close the holder, then retry once
        print(f"error: the scanner port is in use: {exc}", file=sys.stderr)
        if sys.stdin is not None and sys.stdin.isatty():
            if portguard.offer_to_close_holders(exclude_pid=os.getpid()):
                time.sleep(0.6)   # let Windows release the handle
                try:
                    return SerialSource(args.port, args.baud)
                except Exception as exc2:
                    print(f"error: port still in use after closing: {exc2}", file=sys.stderr)
                    return None
        else:
            print("       Close any other roomscan viewer/panel window (only one can hold the "
                  "port), then retry.", file=sys.stderr)
        return None


def run(args, *, smoke_ticks: int = 0) -> int:
    import open3d.visualization.gui as gui

    _fill_panel_fields(args)   # viewer-delegated args arrive without the panel-only fields

    source = _open_source(args)
    if source is None:
        return 1
    client = CommandClient(source.write) if isinstance(source, SerialSource) else None
    dll = Transform.available()
    outputs = ("depth", "reflectance", "confidence") if dll else ("depth",)
    stage = TransformStage(outputs=outputs)   # all three computed by one instance; ~zero marginal cost
    bus = LogBus()
    recorder = Recorder()
    interval = 1.0 / args.replay_fps if (args.replay and args.replay_fps and args.replay_fps > 0) else 0.0
    pacer = _Pacer(interval)
    if not dll:
        bus.publish("transform DLL absent — depth only, IR/reflectance unavailable")

    gui.Application.instance.initialize()
    panel = ControlPanel(args, source, client, stage, bus, recorder, pacer)
    panel.start()
    if smoke_ticks > 0:
        for _ in range(smoke_ticks):
            gui.Application.instance.run_one_tick()
            time.sleep(0.01)
        panel._on_close()
        return 0
    gui.Application.instance.run()
    return 0


def main(argv=None) -> int:
    args = _resolve(argv)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
