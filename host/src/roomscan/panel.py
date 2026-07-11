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
    `Window.set_on_tick_event` -- the tick polls the slot and renders the cloud
    AND the IR pane every frame it changes; labels / sensors / metrics / event
    log refresh at <=4 Hz.
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
from .metrics import MetricsRegistry, ResourceSampler
from .metrics_hud import render_hud
from .native import Transform
from . import portguard
from .pipeline import TransformStage
from .protocol import HEADER_SIZE, CommandCode, FrameType, ProtocolError, parse_event
from .magcal import MagCalibration
from .sensors import (
    AXIS_CONVENTION,
    SensorState,
    YawFusion,
    absolute_heading,
    gizmo_pose,
    tilt_compensated_heading,
)
from .sensors_widgets import render_compass, render_sparkline
from .shading import MODES as _NEAR_MODES
from .shading import cloud_colors
from .sources import FileSource, Recorder, SerialSource, pump
from .surface import grid_triangles, grid_triangles_3d
from .viewer import Stats, _build_arg_parser

# Usecase id -> label (only binning-2 profiles are usable at full res; see ROADMAP
# Phase 3 table -- AF_RANGE/AF are binning-4 and get REJECTED_BINNING by firmware).
_USECASES = [(0, "AR_RANGE (~32 fps)"), (1, "AR_PRECISION (~28 fps)")]
_COLOR_MODES = ("depth", "reflectance", "confidence")
_IR_COLORMAPS = ("gray", "turbo")
_SURFACE_MODES = ("grid", "spatial")
_IR_UPSCALE = 6                 # 54x42 zones -> 324x252 px, nearest-neighbor
_GEOM = "cloud"
_MESH_GEOM = "surface"
_MESH_WALLS_GEOM = "__slam_walls__"   # see-through-walls split, SLAM/Showcase only
_WALL_MODES = ("solid", "translucent", "wireframe")
_SLAM_TRAJ_GEOM = "__slam_trajectory__"
_GIZMO_GEOM = "__imu_gizmo__"
_GIZMO_ANCHOR = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # fixed scene anchor; calibrate later
_UI_PERIOD = 0.25               # <=4 Hz label / sensors / metrics / log refresh
                                # (the IR pane is NOT throttled here -- it renders
                                #  per frame, in lockstep with the point cloud)
_EXPOSURE_DEBOUNCE = 0.4        # s to settle before sending a dragged exposure value
_BG_DARK = [0.05, 0.05, 0.08, 1.0]
_BG_LIGHT = [0.90, 0.90, 0.92, 1.0]

# Constrained turntable camera: azimuth-only orbit (no elevation/tilt term exists)
# + pan + zoom, world-up fixed in look_at so the view can never roll or pitch --
# it always faces the scene the same way the flat IR monitor does. Sensitivities
# are in radians (orbit) / fraction (pan, zoom) per pixel or wheel-notch.
#
# Up is -Y, not +Y: the sensor's own frame is a standard CV/camera convention
# (x-right, y-DOWN, z-forward -- ZAPC-validated, docs/deprojector-validation.md
# "y axis increases monotonically with row", row 0 = top = negative y), so +Y
# in point-cloud space is physically down. az=0 also looks along +Z (the
# sensor's own forward/depth axis), not -Z, so the default view is from the
# sensor's own side, not from behind it. Both were verified empirically against
# an Open3D view matrix: with fwd=-Z/up=+Y (the old values) a corner at
# (row0,col0) -- top-left in the IR monitor -- rendered bottom-left (a vertical
# flip); fwd=+Z/up=-Y renders it top-left, matching the IR monitor exactly on
# all four corners.
_WORLD_UP = np.array([0.0, -1.0, 0.0], dtype=np.float32)
_ORBIT_K = 0.008          # rad per pixel drag (~0.46 deg/px)
_PAN_K = 0.0015           # pan fraction of radius per pixel
_ZOOM_STEP = 0.9          # radius *= 0.9**wheel_dy

# Showcase mode (Task 12): FINAL-phase auto-orbit rate and camera ease-in
# duration. See "Fade" note on _advance_showcase_orbit -- Filament has no
# practical per-geometry alpha fade, so the PROCESSING->FINAL transition is a
# clean geometry swap plus this eased camera move, not a literal cross-fade.
_SHOWCASE_ORBIT_STEP = 0.003    # rad per rendered frame -- a gentle few-deg/sec orbit
_SHOWCASE_EASE_S = 1.5          # seconds to ease the camera into the final framing

_HELP_LINES = [
    "",
    "Mouse:  left-drag orbit (yaw only)  |  ctrl / middle-drag pan  |  wheel zoom",
    "        (camera is tilt-locked: it spins level and pans but never tips)",
    "Key:    H  this help    M  toggle metrics overlay    G  camera model    C  clear scan",
    "",
    "Metrics overlay (top-left of the 3D view): capacity bars for our app.",
    "  Sensor rows show host/hub rate — the bar is the fraction of what the",
    "  sensor produced that reached the host (full = keeping up). Plus FPS,",
    "  USB link use, and this process's CPU (cores), RAM, and GPU.",
    "",
    "Status   fps, frame/seq-gap/drop/crc/raw counters, current usecase + color.",
    "Device   Ping / Request CALIB / Reinit; usecase; exposure (ms, sent on release).",
    "         (device controls are inactive in replay.)",
    "View     color mode (depth / reflectance IR / confidence);",
    "         point size (raise it to close the gaps between zones);",
    "         Near contrast (see below); dark background;",
    "         Rotate 90 (turns the cloud AND the IR pane, e.g. sideways mount);",
    "         Reset view; Clear scan.",
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
    "SLAM        live pose + map view (Phase 6): mesh + trajectory replace the",
    "            raw cloud, off the GUI/reader threads; Clear resets the map too.",
    "Capture     Record to captures/*.bin; replay adds Pause + fps.",
    "Events      device EVENTs, command results, connect/disconnect.",
    "",
    "Run with --save-config to persist the current view/IR/near settings.",
]


def _orbit_eye(target, az, radius):
    """Camera eye position for a level turntable orbit: azimuth `az` (radians)
    at `radius` from `target`, always at the target's height -- there is no
    elevation term, so no drag input can ever tilt the camera. az=0 puts the
    eye on the -Z side looking along +Z, the sensor's own forward/depth axis,
    so the default view faces the scene the same way the sensor (and the IR
    monitor) does. With a fixed world-up in look_at, this also can never
    introduce roll. Pure — unit-tested."""
    d = np.array([-np.sin(az), 0.0, -np.cos(az)])
    return np.asarray(target, dtype=np.float64) + radius * d


def _rot_xy(pts, k):
    """Rotate (N,3) points by k*90 deg CCW about the viewing (z) axis, leaving z
    (depth) untouched so coloring/near-contrast are unaffected. Used to upright a
    sideways-mounted sensor; kept in lockstep with the IR pane's np.rot90."""
    k %= 4
    if k == 0 or len(pts) == 0:
        return pts
    x, y, z = pts[:, 0].copy(), pts[:, 1].copy(), pts[:, 2]
    for _ in range(k):
        x, y = -y, x
    return np.stack([x, y, z], axis=1)


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


def _wall_submesh(verts, colors, tris):
    """Build a new legacy `TriangleMesh` from a subset of `tris` (rows into
    the parent mesh's triangle array), remapping vertex indices to a dense
    0..N-1 range and carrying over `verts`/`colors` for the referenced
    vertices only. Module-level (not a method) both because it's pure --
    doesn't touch `self` -- and so it's directly callable from the
    `_upload_slam_mesh` unit tests (test_panel_walls.py), which run that
    method unbound on a lightweight stand-in object (see
    test_panel_showcase.py's established pattern) rather than a real
    `ControlPanel`.

    `TriangleMesh.select_by_index` selects *vertices*, not triangles
    (verified empirically -- it drops any triangle that isn't fully inside
    the selected vertex set), so it can't be used to pull out an arbitrary
    triangle subset; this does it by hand instead."""
    import open3d as o3d
    uniq, remap = np.unique(tris.reshape(-1), return_inverse=True)
    new_tris = remap.reshape(tris.shape).astype(np.int32)
    m = o3d.geometry.TriangleMesh()
    m.vertices = o3d.utility.Vector3dVector(verts[uniq])
    m.triangles = o3d.utility.Vector3iVector(new_tris)
    m.vertex_colors = o3d.utility.Vector3dVector(colors[uniq])
    return m


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
                pacer, is_stopped, state=None, metrics=None):
    """Reader-thread body (module-level so it's unit-testable without a window).

    Owns source+decoder+transform; routes device EVENT -> log bus, ACK ->
    CommandClient, and each transformed DATA frame -> the latest-wins render
    slot. Honors the pacer's live `interval` (replay fps) and `paused` gate, and
    tees raw bytes into `recorder`. Any exception is surfaced via `fault` (unless
    we're stopping) exactly like the classic viewer's reader. `state` (a
    SensorState, optional -- defaults to None for callers that don't care about
    IMU/env streams, e.g. existing tests) is fed every DATA frame; it ignores
    any stream that isn't IMU_QUAT/ENV, mirroring `stage.feed`'s own filtering.
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
            if metrics is not None:
                # Feed every DATA frame (RAW/DEPTH/CALIB/IMU/ENV) so per-sensor
                # rates and link bandwidth see the full stream, not just the
                # frames that survive stage.feed's RAW->depth filter. Wire size
                # = header + payload + CRC32.
                metrics.record(frame.header, HEADER_SIZE + frame.header.payload_len + 4,
                               time.monotonic())
            if state is not None:
                try:
                    state.feed(frame)   # streams 9/10 -> SensorState; ignores others
                except Exception:
                    pass  # a malformed IMU/ENV payload must never kill the reader (ToF continues)
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
        self.imu_gizmo = bool(getattr(args, "imu_gizmo", True))
        self.sensors_panel = bool(getattr(args, "sensors_panel", True))
        self.gizmo_scale = float(getattr(args, "gizmo_scale", 0.15) or 0.15)
        self.yaw_fusion = bool(getattr(args, "yaw_fusion", True))
        self._mag_cal = None
        fusion = None
        if self.yaw_fusion:
            self._mag_cal = MagCalibration.load(
                getattr(args, "mag_cal_path", "mag_cal.json") or "mag_cal.json")
            fusion = YawFusion(
                tau_s=float(getattr(args, "yaw_fusion_tau", 20.0) or 20.0),
                calibration=self._mag_cal,
                anomaly_frac=float(getattr(args, "yaw_anomaly_frac", 0.3) or 0.3),
                motion_rate_dps=float(getattr(args, "yaw_motion_rate_dps", 40.0) or 40.0),
                gimbal_margin_deg=float(getattr(args, "yaw_gimbal_margin_deg", 15.0) or 15.0),
            )
        self.sensor_state = SensorState(fusion=fusion)
        # seed with "off" so the disabled case never publishes a status line;
        # real transitions (init/active/gated:*) still log once each.
        self._last_fusion_status = "off"
        self._gizmo_added = False
        self._baseline_yaw = None
        self.persistence = False  # only show currently perceived image, no persistence (for now)
        self.stats = Stats()
        # metrics HUD: per-sensor rate meters + a background resource sampler.
        # Fed from the reader thread; read on the UI tick. Overlay is toggleable.
        self.resource_sampler = ResourceSampler()
        self.metrics = MetricsRegistry(sampler=self.resource_sampler)
        self.metrics_overlay = bool(getattr(args, "metrics_overlay", True))
        self.slot: queue.Queue = queue.Queue(maxsize=1)
        self.fault: dict = {}
        self._stop = False
        self._reader_thread: threading.Thread | None = None

        # render state
        self.deproj: Deprojector | None = None
        self.pcd = o3d.geometry.PointCloud()
        self.mesh = o3d.geometry.TriangleMesh()
        self._accumulated_points = []
        self._accumulated_colors = []
        self._accumulated_mesh = self._o3d.geometry.TriangleMesh()
        self._last_all_pts: np.ndarray | None = None      # full valid-point set, for camera framing
        self._camera_set = False
        self._cam_target = None             # turntable camera state (world-up locked, no tilt)
        self._cam_az = 0.0
        self._cam_radius = 1.0
        self._drag = None
        self._rot = 0                       # 90 deg CCW turns applied to cloud + IR pane
        self._last_item = None              # last (header, outputs) rendered — reused on rotate
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

        # surface-interpolation state (opt-in: adjacent points close enough
        # get covered by a mesh instead of drawn as dots -- see docs/
        # superpowers/plans/2026-07-09-surface-interpolation-design.md)
        self.surface_enabled = bool(getattr(args, "surface_enabled", False))
        self.surface_mode = args.surface_mode if getattr(args, "surface_mode", None) in _SURFACE_MODES else "grid"
        self.surface_threshold_pct = float(getattr(args, "surface_threshold_pct", 4.0) or 4.0)

        # SLAM view state (Task 10) -- off by default; the worker is created
        # lazily (first SLAM-mode frame) so no depth shape is needed up front.
        # See slam/worker.py for the threading contract.
        self.slam_enabled = False
        self.slam_worker = None
        self._slam_last_mesh_obj = None   # identity check: skip re-upload of an unchanged mesh

        # "See-through walls" (owner request, Phase 6): classify each SLAM/TSDF
        # mesh triangle as wall (vertical surface) vs floor/ceiling (see
        # slam/shading.wall_triangle_mask) and render walls translucent or as
        # wireframe so a near wall doesn't occlude the room's interior. Shared
        # by the classic SLAM view (_render_slam_frame) and Showcase mode
        # (_show_showcase_mesh) via _upload_slam_mesh. Default "translucent"
        # per the owner: they want to see the contents, not the shell.
        self.wall_mode = "translucent"

        # Showcase mode (Task 12): record -> live preview -> post-process -> final
        # reveal. Off by default; layered on top of / mutually exclusive with the
        # SLAM view above (see _on_showcase_toggle / _on_slam_toggle). The heavy
        # `slam.showcase` import (pulls in open3d's tensor geometry stack) is
        # deferred to first actual use, same as slam.worker.SlamWorker above.
        self.showcase_enabled = False
        self.showcase_phase = None   # set to ShowcasePhase.IDLE on first toggle-on
        self._showcase_preview_worker = None   # SlamWorker, live during RECORDING
        self._showcase_post_worker = None      # PostProcessWorker, live during PROCESSING/FINAL
        self._showcase_loader_thread = None    # loads the .bin + builds the PostProcessWorker
        # Bumped by _join_showcase_workers (teardown) and _start_showcase_post_process
        # (each new load) so a slow-loading loader thread that's still in flight
        # when the panel moves on (new recording, Clear, mode switch, window
        # close) can tell it's been superseded and must not publish its
        # (running) PostProcessWorker into self._showcase_post_worker.
        self._showcase_generation = 0
        self._showcase_last_mesh_obj = None
        self._showcase_rec_frames = 0
        self._showcase_orbit_enabled = False
        self._showcase_ease = None             # camera ease-in state (see _advance_showcase_orbit)
        self._showcase_process_start_ts = None

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
        self.mesh_material = rendering.MaterialRecord()
        self.mesh_material.shader = "defaultUnlit"
        self.slam_line_material = rendering.MaterialRecord()
        self.slam_line_material.shader = "unlitLine"
        self.slam_line_material.line_width = 3.0
        # See-through walls (Phase 6): translucent-mesh + wall-wireframe materials.
        # "defaultUnlitTransparency" is Open3D 0.19's unlit alpha-blend shader
        # (confirmed present: <open3d install>/resources/defaultUnlitTransparency.filamat);
        # alpha lives in base_color's 4th channel, RGB left at [1,1,1] (no tint)
        # so the mesh's own baked shade_colors() vertex colors show through.
        self.wall_translucent_material = rendering.MaterialRecord()
        self.wall_translucent_material.shader = "defaultUnlitTransparency"
        self.wall_translucent_material.base_color = [1.0, 1.0, 1.0, 0.3]
        self.wall_wire_material = rendering.MaterialRecord()
        self.wall_wire_material.shader = "unlitLine"
        self.wall_wire_material.line_width = 2.0
        self._dark_bg = True

        self._build_scene()
        self._build_panel()
        self._build_overlay()
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
        # Own the camera nav so it can't tilt: our set_on_mouse handles orbit/pan/
        # zoom and returns CONSUMED, replacing the built-in arcball entirely (a
        # HANDLED return still lets the arcball run afterward -- that's what let
        # it fight our leveling and produce the jitter; CONSUMED stops it).
        self.scene_widget.set_on_mouse(self._on_mouse)
        self.window.add_child(self.scene_widget)

    def _build_overlay(self):
        """A floating metrics HUD image drawn over the top-left of the 3D scene
        (a Window child positioned in _on_layout, NOT inside the side panel).
        Rendered by metrics_hud.render_hud into an ImageWidget so bars and text
        look identical on every box (Open3D's gui font can't draw bars/arrows).
        Hidden/shown by the Metrics-overlay checkbox / M key."""
        gui = self._gui
        blank = np.zeros((10, 10, 3), dtype=np.uint8)
        self.overlay = gui.ImageWidget(self._np_to_o3d(blank))
        self._overlay_size = (10, 10)            # (w, h) tracked for _on_layout
        self.overlay.visible = self.metrics_overlay
        self.window.add_child(self.overlay)

        # Showcase mode's phase banner (Task 12) -- a plain floating Label,
        # stacked just below the metrics HUD image (see _on_layout), hidden
        # unless Showcase mode is on.
        self.showcase_banner = gui.Label("")
        self.showcase_banner.visible = False
        self.window.add_child(self.showcase_banner)

    def _group(self, title, *, open=True):
        """A collapsable group added to the panel, with consistent margins."""
        gui = self._gui
        em = self.window.theme.font_size
        g = gui.CollapsableVert(title, 0.15 * em, gui.Margins(0.5 * em, 0.15 * em, 0, 0.15 * em))
        g.set_is_open(open)
        self.panel.add_child(g)
        return g

    def _labeled_grid(self):
        """A 2-column label|control grid — the columns size to content so the
        label and its control never overlap (the cause of the old crowding)."""
        gui = self._gui
        em = self.window.theme.font_size
        return gui.VGrid(2, 0.5 * em, gui.Margins(0, 0.15 * em, 0, 0.15 * em))

    def _build_panel(self):
        gui = self._gui
        em = self.window.theme.font_size
        self.panel = gui.ScrollableVert(0.15 * em, gui.Margins(0.4 * em, 0.4 * em, 0.4 * em, 0.4 * em))

        # --- Status (live readout only — no usecase/color echo; those live in the
        #     View/Device controls that already show them) ---
        st = self._group("Status")
        self.lbl_conn = gui.Label("connecting...")
        self.lbl_counts = gui.Label("frames 0")
        self.lbl_counts2 = gui.Label("")
        for w in (self.lbl_conn, self.lbl_counts, self.lbl_counts2):
            st.add_child(w)

        # --- View (used most -> near the top) ---
        view = self._group("View")
        vg = self._labeled_grid()
        vg.add_child(gui.Label("Color"))
        self.cb_color = gui.Combobox()
        for m in _COLOR_MODES:
            self.cb_color.add_item(m)
        self.cb_color.selected_index = _COLOR_MODES.index(self.color_mode)
        self.cb_color.set_on_selection_changed(self._on_color)
        vg.add_child(self.cb_color)
        vg.add_child(gui.Label("Point size"))
        self.sl_point = gui.Slider(gui.Slider.INT)
        self.sl_point.set_limits(1, 20)          # wide enough to close the inter-zone gaps
        self.sl_point.int_value = int(self.material.point_size)
        self.sl_point.set_on_value_changed(self._on_point_size)
        vg.add_child(self.sl_point)
        vg.add_child(gui.Label("Near contrast"))
        self.cb_near = gui.Combobox()
        for m in _NEAR_MODES:
            self.cb_near.add_item(m)
        self.cb_near.selected_index = _NEAR_MODES.index(self.near_mode)
        self.cb_near.set_on_selection_changed(self._on_near_mode)
        vg.add_child(self.cb_near)
        self.lbl_near = gui.Label("cutoff m")    # relabeled per mode; slider shows the value
        vg.add_child(self.lbl_near)
        self.sl_near = gui.Slider(gui.Slider.DOUBLE)
        self.sl_near.set_on_value_changed(self._on_near_value)
        vg.add_child(self.sl_near)
        view.add_child(vg)
        self._sync_near_slider()
        self.chk_bg = gui.Checkbox("Dark background")
        self.chk_bg.checked = True
        self.chk_bg.set_on_checked(self._on_bg)
        view.add_child(self.chk_bg)
        self.chk_metrics = gui.Checkbox("Metrics overlay (M)")
        self.chk_metrics.checked = self.metrics_overlay
        self.chk_metrics.set_on_checked(self._on_metrics_overlay)
        view.add_child(self.chk_metrics)
        vrow = gui.Horiz(0.25 * em)
        for text, cb in (("Rotate 90", self._on_rotate), ("Reset", self._on_reset_view),
                         ("Clear", self._on_clear_scan), ("Help", self._show_help)):
            b = gui.Button(text)
            b.horizontal_padding_em = 0.4
            b.set_on_clicked(cb)
            vrow.add_child(b)
        view.add_child(vrow)

        # --- Surface (opt-in: interpolate adjacent points into a mesh) ---
        surf = self._group("Surface", open=False)
        self.chk_surface = gui.Checkbox("Enable surface interpolation")
        self.chk_surface.checked = self.surface_enabled
        self.chk_surface.set_on_checked(self._on_surface_enabled)
        surf.add_child(self.chk_surface)
        sg = self._labeled_grid()
        sg.add_child(gui.Label("Adjacency"))
        self.cb_surface_mode = gui.Combobox()
        for m in _SURFACE_MODES:
            self.cb_surface_mode.add_item(m)
        self.cb_surface_mode.selected_index = _SURFACE_MODES.index(self.surface_mode)
        self.cb_surface_mode.set_on_selection_changed(self._on_surface_mode)
        sg.add_child(self.cb_surface_mode)
        sg.add_child(gui.Label("Threshold %"))
        self.sl_surface_threshold = gui.Slider(gui.Slider.DOUBLE)
        self.sl_surface_threshold.set_limits(0.5, 15.0)
        self.sl_surface_threshold.double_value = self.surface_threshold_pct
        self.sl_surface_threshold.set_on_value_changed(self._on_surface_threshold)
        sg.add_child(self.sl_surface_threshold)
        surf.add_child(sg)

        # --- IR Monitor ---
        ir = self._group("IR Monitor")
        blank = self._np_to_o3d(np.zeros((42 * _IR_UPSCALE, 54 * _IR_UPSCALE, 3), dtype=np.uint8))
        self.ir_widget = gui.ImageWidget(blank)
        ir.add_child(self.ir_widget)
        ig = self._labeled_grid()
        ig.add_child(gui.Label("Map"))
        self.cb_ir = gui.Combobox()
        for m in _IR_COLORMAPS:
            self.cb_ir.add_item(m)
        self.cb_ir.selected_index = _IR_COLORMAPS.index(self.ir_colormap) if self.ir_colormap in _IR_COLORMAPS else 0
        self.cb_ir.set_on_selection_changed(self._on_ir_colormap)
        ig.add_child(self.cb_ir)
        ir.add_child(ig)
        self.chk_freeze = gui.Checkbox("Freeze range")
        self.chk_freeze.checked = self.ir_freeze
        self.chk_freeze.set_on_checked(self._on_ir_freeze)
        ir.add_child(self.chk_freeze)

        # --- SLAM (Phase 6, Task 10): live pose + map view -------------------
        slam = self._group("SLAM", open=False)
        self.chk_slam = gui.Checkbox("SLAM view (mesh + trajectory)")
        self.chk_slam.checked = False
        self.chk_slam.set_on_checked(self._on_slam_toggle)
        slam.add_child(self.chk_slam)
        self.lbl_slam_tracking = gui.Label("tracking: --")
        slam.add_child(self.lbl_slam_tracking)
        self.lbl_slam_ms = gui.Label("slam_ms: --")
        slam.add_child(self.lbl_slam_ms)
        slam.add_child(gui.Label("(View -> Clear also resets the SLAM map)"))
        # See-through walls (owner request): governs both this view and
        # Showcase mode's mesh render (they share _upload_slam_mesh).
        wg = self._labeled_grid()
        wg.add_child(gui.Label("Walls"))
        self.cb_wall_mode = gui.Combobox()
        for m in _WALL_MODES:
            self.cb_wall_mode.add_item(m)
        self.cb_wall_mode.selected_index = _WALL_MODES.index(self.wall_mode)
        self.cb_wall_mode.set_on_selection_changed(self._on_wall_mode)
        wg.add_child(self.cb_wall_mode)
        slam.add_child(wg)

        # --- Showcase (Task 12): record -> live preview -> post-process -> reveal.
        # Mutually exclusive with the SLAM view above (see _on_showcase_toggle /
        # _on_slam_toggle); the phase banner + stats render inside the 3D view
        # itself (a floating gui.Label -- see _build_overlay/_on_layout), not here.
        show = self._group("Showcase", open=False)
        self.chk_showcase = gui.Checkbox("Showcase mode (record -> live preview -> reveal)")
        self.chk_showcase.checked = False
        self.chk_showcase.set_on_checked(self._on_showcase_toggle)
        show.add_child(self.chk_showcase)
        show.add_child(gui.Label("Record to scan, Stop to process. Clear resets to idle."))

        # --- Sensors (LSM6DSV16X: tilt-compensated heading + pressure/temp) ---
        if self.sensors_panel:
            sg = self._group("Sensors")
            self.compass_widget = gui.ImageWidget(self._np_to_o3d(render_compass(0.0)))
            sg.add_child(gui.Label("Heading (tilt-compensated)"))
            sg.add_child(self.compass_widget)
            self.press_widget = gui.ImageWidget(self._np_to_o3d(render_sparkline(np.zeros(2))))
            sg.add_child(gui.Label("Pressure (Pa)"))
            sg.add_child(self.press_widget)
            self.temp_widget = gui.ImageWidget(self._np_to_o3d(render_sparkline(np.zeros(2))))
            sg.add_child(gui.Label("Temperature (°C)"))
            sg.add_child(self.temp_widget)
            self.btn_reset_orientation = gui.Button("Reset Baseline")
            self.btn_reset_orientation.set_on_clicked(self._on_reset_orientation)
            sg.add_child(self.btn_reset_orientation)

        # --- Device ---
        dev = self._group("Device")
        row = gui.Horiz(0.25 * em)
        for text, cmd, param, label in (
            ("Ping", CommandCode.PING, 0, "ping"),
            ("CALIB", CommandCode.SEND_CALIB, 0, "calib"),
            ("Reinit", CommandCode.REINIT, 0, "reinit"),
        ):
            b = gui.Button(text)
            b.horizontal_padding_em = 0.5
            b.set_on_clicked(lambda c=cmd, p=param, lb=label: self.dispatcher.dispatch(c, p, lb))
            row.add_child(b)
        dev.add_child(row)
        dg = self._labeled_grid()
        dg.add_child(gui.Label("Usecase"))
        self.cb_usecase = gui.Combobox()
        for _id, name in _USECASES:
            self.cb_usecase.add_item(name)
        self.cb_usecase.selected_index = 1 if str(getattr(self.args, "usecase", "")) != "0" else 0
        self.cb_usecase.set_on_selection_changed(self._on_usecase)
        dg.add_child(self.cb_usecase)
        dg.add_child(gui.Label("Exposure ms"))
        self.sl_exposure = gui.Slider(gui.Slider.INT)
        self.sl_exposure.set_limits(1, 30)
        self.sl_exposure.int_value = 5
        self.sl_exposure.set_on_value_changed(self._on_exposure_changed)
        dg.add_child(self.sl_exposure)
        dev.add_child(dg)
        if self.is_replay:
            dev.add_child(gui.Label("(inactive in replay)"))

        # --- Capture ---
        cap = self._group("Capture", open=not self.is_replay)
        self.btn_record = gui.Button("Record")
        self.btn_record.toggleable = True
        self.btn_record.set_on_clicked(self._on_record)
        cap.add_child(self.btn_record)
        if self.is_replay:
            self.btn_pause = gui.Button("Pause")
            self.btn_pause.toggleable = True
            self.btn_pause.set_on_clicked(self._on_pause)
            cap.add_child(self.btn_pause)
            fg = self._labeled_grid()
            fg.add_child(gui.Label("Replay fps"))
            self.sl_fps = gui.Slider(gui.Slider.INT)
            self.sl_fps.set_limits(0, 60)
            self.sl_fps.int_value = int(1.0 / self.pacer.interval) if self.pacer.interval > 0 else 0
            self.sl_fps.set_on_value_changed(self._on_fps)
            fg.add_child(self.sl_fps)
            cap.add_child(fg)

        # --- Events (collapsed by default — expand to watch the log) ---
        ev = self._group("Events", open=False)
        self.lv_events = gui.ListView()
        self.lv_events.set_items([])
        ev.add_child(self.lv_events)

        self.window.add_child(self.panel)

    def _on_layout(self, ctx):
        gui = self._gui
        r = self.window.content_rect
        # Return early if window is minimized or has degenerate dimensions
        if r.width <= 0 or r.height <= 0:
            return
        panel_w = int(getattr(self.args, "panel_width", 340))
        # Prevent panel_w from going negative or exceeding window bounds
        panel_w = max(0, min(panel_w, r.width - 100))
        scene_w = r.width - panel_w
        # Prevent degenerate 3D viewport frames
        if scene_w <= 0:
            return
        self.scene_widget.frame = gui.Rect(r.x, r.y, scene_w, r.height)
        self.panel.frame = gui.Rect(r.x + r.width - panel_w, r.y, panel_w, r.height)
        # metrics HUD image: pinned to the scene's top-left at its native size
        w, h = self._overlay_size
        pad = int(0.5 * self.window.theme.font_size)
        self.overlay.frame = gui.Rect(r.x + pad, r.y + pad, w, h)
        # Showcase banner: stacked just below the metrics HUD image (if visible),
        # spanning most of the scene's width. Computed unconditionally (cheap;
        # mirrors the overlay's own always-computed frame above) -- visibility
        # alone gates whether it's actually drawn.
        em = self.window.theme.font_size
        banner_h = int(1.6 * em)
        banner_y = r.y + pad + (h + pad if self.overlay.visible else 0)
        self.showcase_banner.frame = gui.Rect(r.x + pad, banner_y,
                                              min(scene_w - 2 * pad, int(32 * em)), banner_h)

    # ---- lifecycle ----------------------------------------------------------
    def start(self):
        self.resource_sampler.start()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _on_close(self):
        self._stop = True
        if self.slam_worker is not None:
            self.slam_worker.stop()        # join the SLAM worker thread before teardown
        self._join_showcase_workers()      # join Showcase's preview/post-process/loader threads
        self.resource_sampler.stop()       # join the sampler thread before teardown
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
                near_emphasis=self.near_emphasis,
                imu_gizmo=self.imu_gizmo, sensors_panel=self.sensors_panel,
                gizmo_scale=self.gizmo_scale, metrics_overlay=self.metrics_overlay)
            path = cfg.save()
            self.bus.publish(f"saved config to {path}")
        except Exception as exc:  # never let a config write block window close
            self.bus.publish(f"config save failed: {exc!r}")

    # ---- reader thread ------------------------------------------------------
    def _reader_loop(self):
        _run_reader(self.source, self.decoder, self.stage, self.stats, self.slot,
                    self.fault, self.bus, self.client, self.recorder, self.pacer,
                    lambda: self._stop, self.sensor_state, self.metrics)

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
            self._update_ir()      # IR pane in lockstep with the cloud (per frame),
            redraw = True          #   not batched into the <=4 Hz UI refresh below
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
            self._update_sensors()
            self._update_metrics()
            self._drain_log()
            redraw = True
        return redraw

    def _render_frame(self, item):
        o3d = self._o3d
        header, outputs = item
        self._last_item = item
        self._latest_outputs = outputs
        depth = outputs["depth"]
        h, w = depth.shape
        if self.deproj is None:
            self.deproj = Deprojector(w, h, self.args.fov_h, self.args.fov_v)

        if self.showcase_enabled:
            # Showcase mode (Task 12): the 4-phase record/preview/process/reveal
            # experience replaces both the raw cloud AND the classic SLAM view
            # below (mutually exclusive -- see _on_showcase_toggle). Additive and
            # unreachable unless the Showcase checkbox is on.
            self._render_showcase_frame(depth)
            self._shown += 1
            self.metrics.tick_render(time.monotonic())
            return

        if self.slam_enabled:
            # SLAM view (Task 10): mesh + trajectory replace the raw cloud.
            # Everything below this block (the normal-mode cloud/surface path)
            # is untouched and unreachable while this flag is set.
            self._render_slam_frame(depth)
            self._shown += 1
            self.metrics.tick_render(time.monotonic())
            return

        pts = self.deproj(depth)

        # Retrieve fused orientation
        quat = self.sensor_state.fused_quat()
        quat_display = quat
        if quat is not None and self._baseline_yaw is not None:
            from .sensors import graft_yaw
            quat_display = graft_yaw(quat, -self._baseline_yaw)

        if quat_display is not None:
            from .sensors import quat_to_matrix, T_WORLD_TO_CV, T_CV_TO_BODY
            # Apply quaternion rotation using the true physical coordinate mappings
            r = quat_to_matrix(*quat_display)
            # Transform rotation back to CV frame for Open3D rendering
            r_mapped = T_WORLD_TO_CV @ r @ T_CV_TO_BODY
        else:
            r_mapped = np.eye(3)

        # Update the visual camera entity's transform at frame rate
        self._update_camera_gizmo(quat_display)

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
            colors = cloud_colors(vals, pts[:, 2], mode=self.near_mode,   # z-based, so pre-rotation
                                  cutoff_m=self.near_cutoff_m, emphasis=self.near_emphasis)
            
            # Align points to sensor physical mounting orientation
            rot_pts = _rot_xy(pts, self._rot)
            
            # Map points into the fixed world using camera orientation
            world_pts = (r_mapped @ rot_pts.T).T

            # Reset accumulation if camera set flag was cleared
            if not self._camera_set:
                self._accumulated_points = []
                self._accumulated_colors = []
                self._accumulated_mesh = self._o3d.geometry.TriangleMesh()

            if self.surface_enabled:
                self._render_surface(depth, world_pts, colors, r_mapped)
            else:
                self._remove_mesh_geometry()
                if self.persistence:
                    self._accumulated_points.append(world_pts)
                    self._accumulated_colors.append(colors)
                else:
                    self.pcd.points = o3d.utility.Vector3dVector(world_pts)
                    self.pcd.colors = o3d.utility.Vector3dVector(colors)

            # Concatenate accumulated points if persistence is enabled
            if self.persistence:
                if len(self._accumulated_points) > 0:
                    flat_pts = np.concatenate(self._accumulated_points, axis=0)
                    flat_cols = np.concatenate(self._accumulated_colors, axis=0)

                    # Voxel downsample if too large to preserve interactive performance
                    if len(flat_pts) > 20000:
                        pcd_temp = self._o3d.geometry.PointCloud()
                        pcd_temp.points = self._o3d.utility.Vector3dVector(flat_pts)
                        pcd_temp.colors = self._o3d.utility.Vector3dVector(flat_cols)
                        pcd_temp = pcd_temp.voxel_down_sample(voxel_size=0.02)
                        self._accumulated_points = [np.asarray(pcd_temp.points)]
                        self._accumulated_colors = [np.asarray(pcd_temp.colors)]
                        flat_pts = self._accumulated_points[0]
                        flat_cols = self._accumulated_colors[0]
                else:
                    flat_pts = np.zeros((0, 3))
                    flat_cols = np.zeros((0, 3))

                self.pcd.points = o3d.utility.Vector3dVector(flat_pts)
                self.pcd.colors = o3d.utility.Vector3dVector(flat_cols)
                self._show_geometries(flat_pts)
            else:
                self._show_geometries(world_pts)
        else:
            self._remove_mesh_geometry()
            if not self._camera_set:
                self._accumulated_points = []
                self._accumulated_colors = []
                self._accumulated_mesh = self._o3d.geometry.TriangleMesh()
            self.pcd.points = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self.pcd.colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
            self._show_geometries(np.zeros((0, 3)))
        self._shown += 1
        self.metrics.tick_render(time.monotonic())   # rendered-FPS counter

    # ---- SLAM view (Task 10) --------------------------------------------------
    def _render_slam_frame(self, depth):
        """SLAM view: feed the background `SlamWorker` and render its latest
        mesh + trajectory instead of the raw cloud. Only ever called from
        `_render_frame` when `self.slam_enabled`, on the GUI thread; the
        worker's own thread does the actual `Mapper.step` work, so this stays
        cheap (a lock-guarded submit + a lock-guarded read). No serial writes
        happen here or on the worker thread, per the module's threading
        contract (see panel.py's module docstring and slam/worker.py)."""
        if self.slam_worker is None:
            from .slam.worker import SlamWorker
            h, w = depth.shape
            self.slam_worker = SlamWorker(w, h, fov_h=self.args.fov_h, fov_v=self.args.fov_v)
            self.slam_worker.start()

        quat = self.sensor_state.fused_quat()
        if quat is not None:
            env = self.sensor_state.latest_env()
            pressure = env.pressure_pa if env is not None else None
            self.slam_worker.submit(depth, quat, pressure)

        latest = self.slam_worker.latest()
        if latest is None:
            self.lbl_slam_tracking.text = "tracking: waiting for IMU..."
            self.lbl_slam_ms.text = "slam_ms: --"
            return
        mesh, trajectory, step = latest
        self.lbl_slam_tracking.text = f"tracking: {'LOST' if step.tracking_lost else 'ok'}"
        self.lbl_slam_ms.text = f"slam_ms: {step.slam_ms:.1f}"

        sc = self.scene_widget.scene
        # mesh extraction is already throttled inside the worker (every K
        # processed frames); the identity check here avoids re-uploading the
        # *same* mesh object to the renderer every GUI tick in between.
        if (mesh is not None and mesh is not self._slam_last_mesh_obj
                and len(mesh.vertex.positions) > 0):
            self._upload_slam_mesh(mesh)
            self._slam_last_mesh_obj = mesh

        # BUG-009 (fixed): a LineSet with < 2 points has 0 line segments and
        # hard-crashes Filament ("vertexCount cannot be 0", then a native
        # segfault) on add_geometry -- skip the upload entirely until there's
        # a real segment to draw. Same guard `_show_showcase_trajectory`
        # (Showcase mode) already uses.
        if trajectory and len(trajectory) >= 2:
            pts = np.array([T[:3, 3] for T in trajectory], dtype=np.float64)
            lines = self._o3d.geometry.LineSet()
            lines.points = self._o3d.utility.Vector3dVector(pts)
            idx = np.stack([np.arange(len(pts) - 1), np.arange(1, len(pts))], axis=1)
            lines.lines = self._o3d.utility.Vector2iVector(idx)
            lines.colors = self._o3d.utility.Vector3dVector(
                np.tile([[0.1, 0.9, 0.3]], (len(idx), 1)))
            if sc.has_geometry(_SLAM_TRAJ_GEOM):
                sc.remove_geometry(_SLAM_TRAJ_GEOM)
            sc.add_geometry(_SLAM_TRAJ_GEOM, lines, self.slam_line_material)

        self._slam_camera_frame(mesh, trajectory)

    def _slam_camera_frame(self, mesh, trajectory):
        """First-time camera framing for the SLAM view -- mirrors
        `_reset_camera`'s bounds-from-points approach, sourced from the mesh
        vertices + trajectory instead of the raw cloud. No-op once
        `_camera_set` (cleared by the SLAM checkbox / Clear map, like the
        normal view's camera-set flag)."""
        if self._camera_set:
            return
        if self.scene_widget.frame.width <= 0 or self.scene_widget.frame.height <= 0:
            return
        pts_list = [np.zeros((1, 3))]
        if mesh is not None and len(mesh.vertex.positions) > 0:
            pts_list.append(mesh.vertex.positions.numpy())
        if trajectory:
            pts_list.append(np.array([T[:3, 3] for T in trajectory]))
        all_pts = np.vstack(pts_list)
        bounds = self._o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            self._o3d.utility.Vector3dVector(all_pts))
        ext = float(bounds.get_extent().max())
        if ext <= 0:
            return
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())
        self._cam_target = np.asarray(bounds.get_center(), dtype=np.float64)
        self._cam_radius = ext * 1.8
        self._cam_az = 0.0
        self._apply_camera()
        self._camera_set = True

    def _remove_slam_geometries(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_SLAM_TRAJ_GEOM):
            sc.remove_geometry(_SLAM_TRAJ_GEOM)
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if sc.has_geometry(_MESH_WALLS_GEOM):
            sc.remove_geometry(_MESH_WALLS_GEOM)

    def _on_wall_mode(self, text, index):
        """Combobox handler for the "see-through walls" control (owner
        request): governs both the classic SLAM view and Showcase mode,
        since both funnel through `_upload_slam_mesh`. Clearing the identity
        caches forces the next tick to re-run the upload (and, for
        translucent/wireframe, the wall/non-wall split) even though the
        worker's latest mesh object hasn't changed."""
        self.wall_mode = text
        self._slam_last_mesh_obj = None
        self._showcase_last_mesh_obj = None
        self.bus.publish(f"wall mode -> {text}")

    def _upload_slam_mesh(self, mesh):
        """Shade + upload a SLAM/TSDF tensor `mesh` (non-None, non-empty --
        callers gate on that already). Shared by the classic SLAM view
        (`_render_slam_frame`) and Showcase mode (`_show_showcase_mesh`) so
        the "see-through walls" split (owner request) lives in one place.

        `self.wall_mode`:
          * "solid" -- unchanged pre-existing behavior: the whole mesh
            uploaded as one opaque `_MESH_GEOM`.
          * "translucent"/"wireframe" -- triangles are classified wall vs.
            floor/ceiling by face-normal orientation (`wall_triangle_mask`,
            not camera-facing, so it holds from any orbit angle). Non-wall
            triangles upload as the normal opaque `_MESH_GEOM`; wall
            triangles upload as a second geometry, `_MESH_WALLS_GEOM` --
            alpha-blended for translucent, a `LineSet` of mesh edges for
            wireframe -- so a near wall doesn't occlude the room's interior.

        Always removes any stale `_MESH_GEOM`/`_MESH_WALLS_GEOM` first so a
        mode switch (or a wall-only mesh) never leaves a leftover geometry
        from the previous upload.
        """
        from .slam.shading import shade_colors, wall_triangle_mask
        sc = self.scene_widget.scene
        legacy_mesh = mesh.to_legacy()
        legacy_mesh.compute_vertex_normals()
        # The TSDF mesh's vertex colors are always [0,0,0] (integrate() is
        # depth-only, see tsdf.py) and this material is defaultUnlit with no
        # scene lights -- bake a fixed-light shade in so it's not invisible
        # black (see slam/shading.py's module docstring).
        legacy_mesh.vertex_colors = self._o3d.utility.Vector3dVector(
            shade_colors(np.asarray(legacy_mesh.vertex_normals)))

        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if sc.has_geometry(_MESH_WALLS_GEOM):
            sc.remove_geometry(_MESH_WALLS_GEOM)

        if self.wall_mode == "solid" or len(legacy_mesh.triangles) == 0:
            sc.add_geometry(_MESH_GEOM, legacy_mesh, self.mesh_material)
            return

        legacy_mesh.compute_triangle_normals()
        wall_mask = wall_triangle_mask(np.asarray(legacy_mesh.triangle_normals))
        tris = np.asarray(legacy_mesh.triangles)
        verts = np.asarray(legacy_mesh.vertices)
        colors = np.asarray(legacy_mesh.vertex_colors)

        non_wall_tris = tris[~wall_mask]
        if non_wall_tris.shape[0] > 0:
            sc.add_geometry(_MESH_GEOM, _wall_submesh(verts, colors, non_wall_tris),
                             self.mesh_material)

        wall_tris = tris[wall_mask]
        if wall_tris.shape[0] == 0:
            return
        wall_submesh = _wall_submesh(verts, colors, wall_tris)
        if self.wall_mode == "translucent":
            sc.add_geometry(_MESH_WALLS_GEOM, wall_submesh, self.wall_translucent_material)
        else:   # "wireframe"
            wire = self._o3d.geometry.LineSet.create_from_triangle_mesh(wall_submesh)
            # BUG-009 (see _render_slam_frame's trajectory upload): a LineSet
            # with < 2 points / 0 segments hard-crashes Filament -- guard it
            # here too, even though a non-empty triangle selection should
            # always produce >= 3 points/edges.
            if len(wire.points) >= 2:
                wire.colors = self._o3d.utility.Vector3dVector(
                    np.tile([[0.45, 0.60, 0.75]], (len(wire.lines), 1)))  # muted blue-gray
                sc.add_geometry(_MESH_WALLS_GEOM, wire, self.wall_wire_material)

    def _on_slam_toggle(self, checked):
        if checked and self.showcase_enabled:
            # Mutually exclusive with Showcase mode (see _on_showcase_toggle's
            # symmetric guard) -- this one added line is the only change to this
            # method's pre-existing body below.
            self.chk_showcase.checked = False
            self._on_showcase_toggle(False)
        self.slam_enabled = checked
        if not checked:
            if self.slam_worker is not None:
                self.slam_worker.stop()
                self.slam_worker = None
            self._remove_slam_geometries()
            self._slam_last_mesh_obj = None
            self._camera_set = False
            if self._last_item is not None:   # re-render the last frame as a normal cloud
                self._render_frame(self._last_item)
        else:
            self._camera_set = False          # reframe onto SLAM content once it appears
        self.bus.publish(f"SLAM view -> {'on' if checked else 'off'}")

    # ---- Showcase mode (Task 12): record -> live preview -> post-process -> reveal ----
    def _render_showcase_frame(self, depth):
        """Showcase mode's per-frame render -- dispatches on the current
        `ShowcasePhase` (slam/showcase.py). Called from `_render_frame` only
        when `self.showcase_enabled`; mirrors `_render_slam_frame`'s shape but
        drives the 4-phase state machine instead of a single live view."""
        from .slam.showcase import ShowcasePhase
        phase = self.showcase_phase
        if phase is ShowcasePhase.RECORDING:
            self._render_showcase_recording(depth)
        elif phase is ShowcasePhase.PROCESSING:
            self._render_showcase_processing()
        elif phase is ShowcasePhase.FINAL:
            self._advance_showcase_orbit()
        else:
            self._set_showcase_banner("Press Record to scan the room.")

    def _render_showcase_recording(self, depth):
        """RECORDING phase: feed the live preview `SlamWorker` (same engine
        Task 10's classic SLAM view uses, preview quality/translation-only ICP)
        from this frame's depth + fused quat/pressure, and show its latest
        rough mesh + trajectory. The panel's `Recorder` (started by the Record
        button, see _on_record) is simultaneously writing the raw .bin
        alongside this -- entirely independent of this preview."""
        from .slam.worker import SlamWorker
        if self._showcase_preview_worker is None:
            h, w = depth.shape
            self._showcase_preview_worker = SlamWorker(w, h, fov_h=self.args.fov_h,
                                                        fov_v=self.args.fov_v)
            self._showcase_preview_worker.start()

        quat = self.sensor_state.fused_quat()
        tracking_txt = "waiting for IMU..."
        if quat is not None:
            env = self.sensor_state.latest_env()
            pressure = env.pressure_pa if env is not None else None
            self._showcase_preview_worker.submit(depth, quat, pressure)
            self._showcase_rec_frames += 1

        latest = self._showcase_preview_worker.latest()
        if latest is not None:
            mesh, trajectory, step = latest
            tracking_txt = "lost" if step.tracking_lost else "ok"
            self._show_showcase_mesh(mesh)
            self._show_showcase_trajectory(trajectory)
            self._slam_camera_frame(mesh, trajectory)
        self._set_showcase_banner(
            f"● Recording — scanning… ({self._showcase_rec_frames} frames "
            f"· tracking: {tracking_txt})")

    def _render_showcase_processing(self):
        """PROCESSING phase: render whatever the background `PostProcessWorker`
        has published so far -- the scene stays interactive (orbitable) the
        whole time; each newer, more-complete mesh simply swaps in over the
        last one (see _show_showcase_mesh's identity check). On its terminal
        (done=True) publish, hands off to _enter_showcase_final."""
        worker = self._showcase_post_worker
        if worker is None:
            self._set_showcase_banner("Processing… loading capture…")
            return
        latest = worker.latest()
        if latest is None:
            self._set_showcase_banner("Processing… 0%")
            return
        self._show_showcase_mesh(latest.mesh)
        self._show_showcase_trajectory(latest.trajectory)
        self._slam_camera_frame(latest.mesh, latest.trajectory)
        if latest.done:
            self._enter_showcase_final(latest)
        else:
            self._set_showcase_banner(f"Processing… {latest.fraction * 100:.0f}%")

    def _show_showcase_mesh(self, mesh):
        """Upload `mesh` if it's new (identity check -- mesh extraction is
        already throttled inside the worker) and non-empty. Shares the
        classic SLAM view's geometry name/material (via `_upload_slam_mesh`)
        since the two modes are mutually exclusive and never rendered in the
        same frame."""
        if mesh is None or mesh is self._showcase_last_mesh_obj or len(mesh.vertex.positions) == 0:
            return
        self._upload_slam_mesh(mesh)
        self._showcase_last_mesh_obj = mesh

    def _show_showcase_trajectory(self, trajectory):
        # NOTE: intentionally stricter than _render_slam_frame's equivalent
        # block (classic SLAM view, which uploads a LineSet even for a
        # single-point trajectory). Reproduced live (replaying
        # captures/phase6_motion_ref.bin, ticking the panel through the
        # classic SLAM view too): uploading a LineSet with 1 point / 0 line
        # segments as the first "unlitLine"-shaded geometry ever added to a
        # fresh scene hard-crashes Filament ("VertexBuffer ... vertexCount
        # cannot be 0", then a native segfault) -- a pre-existing bug in
        # Task 10's code (filed in BUGS.md), not something to inherit into
        # this new method just because the shape matches. Skipping until
        # there are >= 2 points sidesteps it entirely for Showcase mode.
        if len(trajectory) < 2:
            return
        sc = self.scene_widget.scene
        pts = np.array([T[:3, 3] for T in trajectory], dtype=np.float64)
        lines = self._o3d.geometry.LineSet()
        lines.points = self._o3d.utility.Vector3dVector(pts)
        idx = np.stack([np.arange(len(pts) - 1), np.arange(1, len(pts))], axis=1)
        lines.lines = self._o3d.utility.Vector2iVector(idx)
        lines.colors = self._o3d.utility.Vector3dVector(
            np.tile([[0.1, 0.9, 0.3]], (len(idx), 1)))
        if sc.has_geometry(_SLAM_TRAJ_GEOM):
            sc.remove_geometry(_SLAM_TRAJ_GEOM)
        sc.add_geometry(_SLAM_TRAJ_GEOM, lines, self.slam_line_material)

    def _showcase_target_camera(self, mesh, trajectory):
        """(center, radius) to frame `mesh` + `trajectory`'s bounds -- the same
        approach as `_slam_camera_frame`, duplicated (not shared) so that
        pre-existing method, used by the classic SLAM view, stays untouched.
        Returns None on a still-degenerate (zero-extent) scan."""
        pts_list = [np.zeros((1, 3))]
        if mesh is not None and len(mesh.vertex.positions) > 0:
            pts_list.append(mesh.vertex.positions.numpy())
        if trajectory:
            pts_list.append(np.array([T[:3, 3] for T in trajectory]))
        all_pts = np.vstack(pts_list)
        bounds = self._o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            self._o3d.utility.Vector3dVector(all_pts))
        ext = float(bounds.get_extent().max())
        if ext <= 0:
            return None
        return np.asarray(bounds.get_center(), dtype=np.float64), ext * 1.8

    def _enter_showcase_final(self, progress):
        """PROCESSING -> FINAL: the mesh/trajectory are already the final ones
        (just uploaded by the caller); auto-frame + start the slow orbit, and
        show the stats banner. Fade note: Filament (Open3D's rendering
        backend) has no practical per-geometry alpha fade, so rather than fight
        it for a literal cross-fade, the "transition" is a clean geometry swap
        (already done) plus this eased camera move toward the final framing
        (smoothstepped over _SHOWCASE_EASE_S, in _advance_showcase_orbit) --
        the fallback the brief explicitly sanctions."""
        from .slam.showcase import next_phase
        self.showcase_phase = next_phase(self.showcase_phase, processing_done=True)
        target = self._showcase_target_camera(progress.mesh, progress.trajectory)
        now = time.monotonic()
        if target is not None:
            to_target, to_radius = target
            if self._cam_target is not None:
                self._showcase_ease = {
                    "t0": now, "duration": _SHOWCASE_EASE_S,
                    "from_target": self._cam_target.copy(), "from_radius": self._cam_radius,
                    "to_target": to_target, "to_radius": to_radius,
                }
            else:   # nothing framed yet (e.g. RECORDING never got a valid mesh) -- snap once
                self._cam_target, self._cam_radius, self._cam_az = to_target, to_radius, 0.0
                self._camera_set = True
                self._apply_camera()
                self._showcase_ease = None
        self._showcase_orbit_enabled = True
        elapsed = now - (self._showcase_process_start_ts or now)
        stats = progress.stats or {}
        self._set_showcase_banner(
            f"Scan complete — {stats.get('frames', 0)} frames · "
            f"drift {stats.get('gap_m', 0.0):.2f} m · {stats.get('verts', 0)} verts · "
            f"{elapsed:.1f}s")
        self.bus.publish("showcase: scan complete")

    def _advance_showcase_orbit(self):
        """FINAL phase, called every rendered frame: advance the camera-ease
        (if one is in flight) and the slow auto-orbit -- both stop dead the
        moment the user takes the mouse (checking `self._drag`, the same drag
        state `_on_mouse` already tracks), so this never fights manual
        orbit/pan."""
        if self._drag is not None:
            return
        if self._showcase_ease is not None:
            e = self._showcase_ease
            frac = min(1.0, (time.monotonic() - e["t0"]) / e["duration"])
            s = frac * frac * (3 - 2 * frac)   # smoothstep
            self._cam_target = e["from_target"] + (e["to_target"] - e["from_target"]) * s
            self._cam_radius = e["from_radius"] + (e["to_radius"] - e["from_radius"]) * s
            if frac >= 1.0:
                self._showcase_ease = None
        if self._showcase_orbit_enabled:
            self._cam_az += _SHOWCASE_ORBIT_STEP
        self._apply_camera()

    def _enter_showcase_recording(self):
        """Record button pressed while in Showcase mode: fresh preview worker
        (lazily built on the first depth frame, once its shape is known),
        fresh camera framing, fresh geometry. `next_phase`'s record_pressed
        rule fires from ANY phase (not just IDLE) -- pressing Record again
        while looking at a FINAL reveal, or mid-PROCESSING, restarts rather
        than being ignored, so this must tear down (not just orphan) whatever
        the interrupted phase was using: `_join_showcase_workers()` stops and
        joins a still-running PostProcessWorker/loader thread exactly like
        leaving Showcase mode does.

        Also re-requests CALIB (same command the Device group's "CALIB"
        button sends): the device only streams CALIB once, near stream
        start, so a Recorder capture that starts well into an already-running
        session (the normal case -- Showcase can be turned on and Record
        pressed at any point) would otherwise be missing it entirely. Without
        it, `_load_frames`/`TransformStage` can't transform a single raw
        frame in the just-recorded .bin, so the PostProcessWorker's Mapper
        never even gets real width/height (see showcase.py's
        `_publish_construction_failure`, which keeps that case from hanging
        PROCESSING forever -- but the fix here is what makes the recording
        actually processable). `dispatch()` itself already reports "not
        available in replay" harmlessly when there's no live device, so this
        is unconditional."""
        from .slam.showcase import next_phase
        self.showcase_phase = next_phase(self.showcase_phase, record_pressed=True)
        self._join_showcase_workers()
        self.dispatcher.dispatch(CommandCode.SEND_CALIB, 0, "calib (showcase)")
        self._showcase_rec_frames = 0
        self._showcase_last_mesh_obj = None
        self._showcase_orbit_enabled = False
        self._showcase_ease = None
        self._camera_set = False
        self._remove_slam_geometries()
        self._set_showcase_banner("● Recording — scanning…")

    def _enter_showcase_processing(self, path):
        """Stop button pressed while RECORDING: tear down the live preview
        worker (its job is done) and kick off the full-quality re-process on a
        background thread -- both the capture load (`_load_frames`, tens of
        MB) and the `Mapper` run happen off the GUI thread, so Stop returns
        immediately and the banner flips to "Processing..." right away."""
        from .slam.showcase import next_phase
        self.showcase_phase = next_phase(self.showcase_phase, stop_pressed=True)
        if self._showcase_preview_worker is not None:
            self._showcase_preview_worker.stop()
            self._showcase_preview_worker = None
        self._showcase_post_worker = None
        self._showcase_process_start_ts = time.monotonic()
        self._set_showcase_banner("Processing… loading capture…")
        if not path:
            self.bus.publish("showcase: no capture path to process")
            return
        self._start_showcase_post_process(path)

    def _start_showcase_post_process(self, path):
        """Load `path` + build/start the `PostProcessWorker` on a dedicated
        background thread (file IO + the full Mapper run; NOT the serial
        reader -- no second serial reader, per the module contract). Publishes
        the finished worker to `self._showcase_post_worker` (a simple
        reference assignment, safe to read from the GUI thread without a lock,
        same as `self.slam_worker`'s lazy construction elsewhere in this
        file).

        Race guard: this bumps `self._showcase_generation` and the loader
        closure captures that value. `_load_frames` (inside `from_capture`)
        can take a while on a large capture; if the panel moves on in the
        meantime -- a new recording, Clear, leaving Showcase mode, or window
        close, all of which call `_join_showcase_workers` (which also bumps
        the generation) -- the generation the loader captured no longer
        matches `self._showcase_generation` by the time it checks, so a
        superseded load quietly drops its (possibly still-running) worker
        instead of assigning it into the live `self._showcase_post_worker`
        slot out from under whatever the panel is doing now."""
        self._showcase_generation += 1
        gen = self._showcase_generation

        def _loader():
            if gen != self._showcase_generation:
                return   # superseded before loading even started
            from .slam.showcase import PostProcessWorker
            try:
                worker = PostProcessWorker.from_capture(
                    path, mesh_every=25, icp_mode="translation",
                    fov_h=self.args.fov_h, fov_v=self.args.fov_v)
            except Exception as exc:
                self.bus.publish(f"showcase: failed to load capture {path!r}: {exc!r}")
                return
            if gen != self._showcase_generation:
                return   # superseded while `_load_frames`/Mapper setup ran
            worker.start()
            if gen != self._showcase_generation:
                # superseded in the gap between start() and this check --
                # don't leave the worker's thread running unsupervised
                worker.stop()
                return
            self._showcase_post_worker = worker
        self._showcase_loader_thread = threading.Thread(target=_loader, daemon=True)
        self._showcase_loader_thread.start()

    def _join_showcase_workers(self):
        """Stop + join every Showcase-mode thread (preview SlamWorker, the
        PostProcessWorker, and the capture-loader thread) -- called on window
        close, on leaving Showcase mode, and on Clear, so no Showcase thread
        ever outlives the panel or a mode switch. Safe to call repeatedly /
        when nothing is running.

        Bumps `_showcase_generation` first so any loader thread still in
        flight from a superseded `_start_showcase_post_process` call (see
        that method's docstring) can tell it's stale and must not publish
        into `self._showcase_post_worker`."""
        self._showcase_generation += 1
        if self._showcase_preview_worker is not None:
            self._showcase_preview_worker.stop()
            self._showcase_preview_worker = None
        if self._showcase_post_worker is not None:
            self._showcase_post_worker.stop()
            self._showcase_post_worker = None
        if self._showcase_loader_thread is not None:
            self._showcase_loader_thread.join(timeout=5.0)
            self._showcase_loader_thread = None

    def _set_showcase_banner(self, text):
        self.showcase_banner.text = text

    def _on_showcase_toggle(self, checked):
        from .slam.showcase import next_phase
        if checked and self.slam_enabled:
            # Mutually exclusive with the classic SLAM view (symmetric guard
            # to the one added in _on_slam_toggle).
            self.chk_slam.checked = False
            self._on_slam_toggle(False)
        self.showcase_enabled = checked
        if checked:
            self.showcase_phase = next_phase(self.showcase_phase, cleared=True)   # -> IDLE
            self._showcase_orbit_enabled = False
            self._showcase_ease = None
            self._camera_set = False
            self._showcase_last_mesh_obj = None
            self.showcase_banner.visible = True
            self._set_showcase_banner("Press Record to scan the room.")
        else:
            self._join_showcase_workers()
            self.showcase_phase = next_phase(self.showcase_phase, cleared=True)   # -> IDLE
            self._showcase_orbit_enabled = False
            self._showcase_ease = None
            self._remove_slam_geometries()
            self._showcase_last_mesh_obj = None
            self.showcase_banner.visible = False
            self._camera_set = False
            if self._last_item is not None:   # re-render the last frame as a normal cloud
                self._render_frame(self._last_item)
        self.bus.publish(f"Showcase mode -> {'on' if checked else 'off'}")

    def _show_geometries(self, all_pts):
        """Push the dot cloud to the scene and (re)frame the camera from the
        FULL valid point set for this frame -- `all_pts` is every valid point
        before the covered/lone split, so framing doesn't shrink once most
        points move into the mesh. The mesh geometry itself is managed
        separately by _show_mesh_geometry/_remove_mesh_geometry, called from
        _render_surface, since only surface mode touches it."""
        self._last_all_pts = all_pts
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        sc.add_geometry(_GEOM, self.pcd, self.material)
        if not self._camera_set and len(all_pts):
            self._reset_camera()

    def _show_mesh_geometry(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        mesh_to_show = self._accumulated_mesh if self.persistence else self.mesh
        if mesh_to_show is not None and len(mesh_to_show.triangles) > 0:
            sc.add_geometry(_MESH_GEOM, mesh_to_show, self.mesh_material)

    def _remove_mesh_geometry(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)

    def _reset_camera(self):
        # Do not configure camera if the viewport is minimized or degenerate
        if self.scene_widget.frame.width <= 0 or self.scene_widget.frame.height <= 0:
            return
        all_pts = self._last_all_pts
        if all_pts is None or len(all_pts) == 0:
            return
        # Frame both the camera center [0.0, 0.0, 0.0] and the painted points
        pts_for_bounds = np.vstack([all_pts, [0.0, 0.0, 0.0]])
        if self.persistence and self._accumulated_mesh is not None and len(self._accumulated_mesh.vertices) > 0:
            mesh_verts = np.asarray(self._accumulated_mesh.vertices)
            if len(mesh_verts):
                pts_for_bounds = np.vstack([pts_for_bounds, mesh_verts])
        bounds = self._o3d.geometry.AxisAlignedBoundingBox.create_from_points(
            self._o3d.utility.Vector3dVector(pts_for_bounds))
        ext = float(bounds.get_extent().max())
        if ext <= 0:
            return
        self.scene_widget.setup_camera(60.0, bounds, bounds.get_center())  # projection + near/far
        self._cam_target = np.asarray(bounds.get_center(), dtype=np.float64)
        self._cam_radius = ext * 1.8
        self._cam_az = 0.0
        self._apply_camera()
        self._camera_set = True

    def _render_surface(self, depth, rot_pts, colors, r_mapped=None):
        """Split this frame's points into covered (hidden, drawn by the mesh)
        and lone (still dots), per the selected adjacency mode. Accumulates
        the current mesh and the lone points into the global scan."""
        h, w = depth.shape
        pts_grid, valid_grid = self.deproj.grid(depth)

        if self.surface_mode == "spatial":
            # Spatial mode: grid adjacency with a 3D distance threshold (in meters)
            mean_z = float(np.mean(pts_grid[valid_grid, 2])) if np.any(valid_grid) else 1.0
            threshold_m = max((self.surface_threshold_pct / 100.0) * mean_z, 1e-6)
            triangles, covered_grid = grid_triangles_3d(pts_grid, valid_grid, threshold_m)
        else:
            # Grid mode: grid adjacency with relative depth percentage threshold
            triangles, covered_grid = grid_triangles(pts_grid, valid_grid, self.surface_threshold_pct)

        covered = covered_grid[valid_grid.ravel()]
        mesh_verts = _rot_xy(pts_grid.reshape(-1, 3), self._rot)
        if r_mapped is not None:
            mesh_verts = mesh_verts @ r_mapped.T
            
        colors_grid = np.zeros((h * w, 3), dtype=np.float64)
        colors_grid[valid_grid.ravel()] = colors
        
        if self.persistence:
            current_mesh = self._o3d.geometry.TriangleMesh()
            current_mesh.vertices = self._o3d.utility.Vector3dVector(mesh_verts)
            current_mesh.vertex_colors = self._o3d.utility.Vector3dVector(colors_grid)
            current_mesh.triangles = self._o3d.utility.Vector3iVector(triangles.astype(np.int32))
            
            if self._accumulated_mesh is None or len(self._accumulated_mesh.triangles) == 0:
                self._accumulated_mesh = current_mesh
            else:
                self._accumulated_mesh += current_mesh
                
            if len(self._accumulated_mesh.triangles) > 50000:
                self._accumulated_mesh = self._accumulated_mesh.simplify_vertex_clustering(0.02)
                self._accumulated_mesh.remove_duplicated_vertices()
                self._accumulated_mesh.remove_duplicated_triangles()
                self._accumulated_mesh.remove_degenerate_triangles()
                
            self._show_mesh_geometry()

            lone_pts = rot_pts[~covered]
            lone_colors = colors[~covered]
            self._accumulated_points.append(lone_pts)
            self._accumulated_colors.append(lone_colors)
        else:
            self.mesh.vertices = self._o3d.utility.Vector3dVector(mesh_verts)
            self.mesh.vertex_colors = self._o3d.utility.Vector3dVector(colors_grid)
            self.mesh.triangles = self._o3d.utility.Vector3iVector(triangles.astype(np.int32))
            self._show_mesh_geometry()
            
            self.pcd.points = self._o3d.utility.Vector3dVector(rot_pts[~covered])
            self.pcd.colors = self._o3d.utility.Vector3dVector(colors[~covered])


    def _apply_camera(self):
        # Do not apply camera look-at if the viewport is minimized or degenerate
        if self.scene_widget.frame.width <= 0 or self.scene_widget.frame.height <= 0:
            return
        if self._cam_target is None:
            return
        eye = _orbit_eye(self._cam_target, self._cam_az, self._cam_radius)
        self.scene_widget.look_at(self._cam_target.astype(np.float32),
                                  eye.astype(np.float32), _WORLD_UP)   # fixed up -> never tilts/rolls

    def _on_mouse(self, e):
        gui = self._gui
        res = gui.SceneWidget.EventCallbackResult
        if self._cam_target is None:
            return res.IGNORED
        et = e.type
        if et == gui.MouseEvent.Type.WHEEL:
            if e.wheel_dy:
                self._cam_radius = max(self._cam_radius * (_ZOOM_STEP ** e.wheel_dy), 1e-3)
                self._apply_camera()
            return res.CONSUMED
        if et == gui.MouseEvent.Type.BUTTON_DOWN:
            self._drag = (e.x, e.y)
            # Showcase FINAL's camera ease (_advance_showcase_orbit) computes
            # its progress from an un-paused wall clock -- it only *pauses*
            # while `_drag` is set, so an ease that's still in flight when a
            # drag starts would otherwise resume (and can jump straight to
            # 100%) the moment the drag ends, teleporting the camera and
            # discarding the user's manual positioning. Cancel outright on
            # drag start instead: a manual drag always wins.
            self._showcase_ease = None
            return res.CONSUMED
        if et == gui.MouseEvent.Type.BUTTON_UP:
            self._drag = None
            return res.CONSUMED
        if et == gui.MouseEvent.Type.DRAG and self._drag is not None:
            dx, dy = e.x - self._drag[0], e.y - self._drag[1]
            self._drag = (e.x, e.y)
            pan = (e.is_modifier_down(gui.KeyModifier.CTRL)
                   or e.is_button_down(gui.MouseButton.MIDDLE)
                   or e.is_button_down(gui.MouseButton.RIGHT))
            if pan:
                self._pan(dx, dy)
            else:                                   # yaw only -- no elevation/tilt term exists
                self._cam_az -= dx * _ORBIT_K
            self._apply_camera()
            return res.CONSUMED
        return res.IGNORED

    def _pan(self, dx, dy):
        eye = _orbit_eye(self._cam_target, self._cam_az, self._cam_radius)
        fwd = self._cam_target - eye
        fwd /= np.linalg.norm(fwd) + 1e-9
        right = np.cross(fwd, _WORLD_UP.astype(np.float64))
        right /= np.linalg.norm(right) + 1e-9
        cam_up = np.cross(right, fwd)
        self._cam_target = self._cam_target + (-dx * right + dy * cam_up) * (self._cam_radius * _PAN_K)

    def _update_status(self):
        now, mark = time.monotonic(), self._shown
        t0, m0 = self._fps_mark
        dt = now - t0
        if dt >= 0.5:
            self._fps = (mark - m0) / dt
            self._fps_mark = (now, mark)
        where = str(self.args.replay) if self.is_replay else str(getattr(self.source, "port", "?"))
        self.lbl_conn.text = f"{'replay' if self.is_replay else 'live'}: {where}    {self._fps:.1f} fps"
        self.lbl_counts.text = (f"frames {self.stats.frames}  raw {self.stage.raw_transformed}  "
                                f"gaps {self.stats.seq_gaps}")
        line2 = f"drops {self.stats.dropped_flags}  crc {self.decoder.crc_failures}"
        if self.stage.raw_skipped_awaiting_calib:
            line2 += f"  raw-skip {self.stage.raw_skipped_awaiting_calib}"
        self.lbl_counts2.text = line2

    def _update_metrics(self):
        """Render the HUD image from a metrics snapshot and push it to the
        overlay ImageWidget (UI thread, <=4 Hz). No-op past setting visibility
        when the overlay is hidden."""
        self.overlay.visible = self.metrics_overlay
        if not self.metrics_overlay:
            return
        snap = self.metrics.snapshot(time.monotonic())
        img = render_hud(snap)
        h, w = img.shape[:2]
        self.overlay.update_image(self._np_to_o3d(img))
        if (w, h) != self._overlay_size:      # fixed-size render -> fires once
            self._overlay_size = (w, h)
            self.window.set_needs_layout()

    def _np_to_o3d(self, rgb: np.ndarray):
        """(H,W,3) uint8 RGB -> o3d.geometry.Image, the shape gui.ImageWidget /
        update_image expect. Shared by the IR monitor and the Sensors widgets."""
        return self._o3d.geometry.Image(np.ascontiguousarray(rgb))

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
        # Use the raw fused quaternion (no yaw baseline — gravity is absolute) to
        # determine the in-plane sensor roll so IR "down" matches the 3D view.
        quat_raw = self.sensor_state.fused_quat()
        if quat_raw is not None:
            from .sensors import ir_gravity_rot
            gravity_steps = ir_gravity_rot(quat_raw)
        else:
            gravity_steps = 0
        total_rot = (self._rot + gravity_steps) % 4
        if total_rot:
            rgb = np.rot90(rgb, total_rot)     # keep the IR pane aligned with gravity + manual rot
        self.ir_widget.update_image(self._np_to_o3d(rgb))

    def _ir_placeholder(self):
        img = np.zeros((42 * _IR_UPSCALE, 54 * _IR_UPSCALE, 3), dtype=np.uint8)
        img[:, :, 0] = 40  # dim maroon = "no IR"
        return self._np_to_o3d(img)

    def _update_camera_gizmo(self, quat_display):
        if self.imu_gizmo and quat_display is not None:
            sc = self.scene_widget.scene
            if not self._gizmo_added:
                # Construct 3D camera geometry (unlit body + lens + red shutter button)
                body = self._o3d.geometry.TriangleMesh.create_box(1.0, 0.6, 0.4)
                body.translate([-0.5, -0.3, -0.4])
                body.paint_uniform_color([0.2, 0.2, 0.22])
                
                lens = self._o3d.geometry.TriangleMesh.create_cylinder(radius=0.25, height=0.3)
                lens.rotate(lens.get_rotation_matrix_from_xyz([np.pi/2, 0, 0]), center=[0, 0, 0])
                lens.translate([0, 0, 0.15])
                lens.paint_uniform_color([0.1, 0.3, 0.6])
                
                button = self._o3d.geometry.TriangleMesh.create_cylinder(radius=0.08, height=0.1)
                button.translate([0.3, -0.35, -0.2])
                button.paint_uniform_color([0.7, 0.2, 0.2])
                
                self._gizmo = body + lens + button
                self._gizmo.compute_vertex_normals()
                sc.add_geometry(_GIZMO_GEOM, self._gizmo, self.mesh_material)
                self._gizmo_added = True
            pose = gizmo_pose(quat_display, self.gizmo_scale, _GIZMO_ANCHOR)
            sc.set_geometry_transform(_GIZMO_GEOM, pose)

    def _update_sensors(self):
        """Called on the <=4 Hz UI tick: refresh the scene gizmo's transform
        from the latest orientation quaternion, and (if the
        Sensors panel group is enabled) the compass + pressure/temp sparklines.
        Graceful no-data: quietly does nothing until IMU_QUAT/ENV frames arrive."""
        quat = self.sensor_state.fused_quat()
        quat_display = quat
        if quat is not None and self._baseline_yaw is not None:
            from .sensors import graft_yaw
            quat_display = graft_yaw(quat, -self._baseline_yaw)
        if self.imu_gizmo and quat_display is not None:
            self._update_camera_gizmo(quat_display)
        status = self.sensor_state.fusion_status()
        if status != self._last_fusion_status:
            self._last_fusion_status = status
            self.bus.publish(f"yaw-fusion -> {status}")
        if not self.sensors_panel:
            return
        env = self.sensor_state.latest_env()
        if env is not None and quat is not None:
            mag = env.mag_ut
            if self._mag_cal is not None:
                mag = tuple(AXIS_CONVENTION @ self._mag_cal.apply(mag))
            heading = absolute_heading(quat, mag)
            self.compass_widget.update_image(self._np_to_o3d(render_compass(heading)))
        self.press_widget.update_image(self._np_to_o3d(render_sparkline(self.sensor_state.pressure_history())))
        self.temp_widget.update_image(self._np_to_o3d(render_sparkline(self.sensor_state.temp_history())))

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
        self._pending_exposure = (int(value), time.monotonic())   # slider shows the value itself

    def _on_color(self, text, index):
        self.color_mode = text
        self.bus.publish(f"color -> {text}")

    def _on_rotate(self, *_):
        self._rot = (self._rot + 1) % 4
        self.bus.publish(f"rotated {self._rot * 90} deg")
        if self._last_item is not None:   # re-apply now (also covers a paused replay)
            self._render_frame(self._last_item)
        self._update_ir()

    def _on_point_size(self, value):
        self.material.point_size = float(int(value))
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.modify_geometry_material(_GEOM, self.material)

    def _on_bg(self, checked):
        self._dark_bg = checked
        self.scene_widget.scene.set_background(_BG_DARK if checked else _BG_LIGHT)

    def _on_metrics_overlay(self, checked):
        self.metrics_overlay = checked
        self.overlay.visible = checked
        self.bus.publish(f"metrics overlay -> {'on' if checked else 'off'}")

    def _on_reset_view(self):
        self._camera_set = False
        self._reset_camera()

    def _on_reset_orientation(self):
        quat = self.sensor_state.fused_quat()
        if quat is not None:
            from .sensors import quat_yaw_deg
            self._baseline_yaw = quat_yaw_deg(quat)
            self.bus.publish(f"yaw-fusion -> baseline reset (yaw = {self._baseline_yaw:.1f} deg)")
            self._on_clear_scan()

    def _on_clear_scan(self):
        self._accumulated_points = []
        self._accumulated_colors = []
        self._accumulated_mesh = self._o3d.geometry.TriangleMesh()
        self.pcd.points = self._o3d.utility.Vector3dVector(np.zeros((0, 3)))
        self.pcd.colors = self._o3d.utility.Vector3dVector(np.zeros((0, 3)))
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        sc.add_geometry(_GEOM, self.pcd, self.material)
        self._remove_mesh_geometry()
        self._camera_set = False
        if self.slam_enabled and self.slam_worker is not None:
            # "Clear" doubles as "Clear map" in SLAM view: drop the worker (and
            # its Mapper/TSDF) so the next SLAM frame lazily builds a fresh one.
            self.slam_worker.stop()
            self.slam_worker = None
            self._remove_slam_geometries()
            self._slam_last_mesh_obj = None
            self.lbl_slam_tracking.text = "tracking: --"
            self.lbl_slam_ms.text = "slam_ms: --"
            self.bus.publish("SLAM map cleared")
        if self.showcase_enabled:
            from .slam.showcase import ShowcasePhase, next_phase
            if self.showcase_phase is ShowcasePhase.RECORDING and self.btn_record.is_on:
                # Clear during an in-progress recording also stops it -- don't
                # leave a dangling capture the panel no longer tracks.
                self.recorder.stop()
                self.btn_record.is_on = False
                self.btn_record.text = "Record"
            self._join_showcase_workers()
            self.showcase_phase = next_phase(self.showcase_phase, cleared=True)
            self._showcase_orbit_enabled = False
            self._showcase_ease = None
            self._remove_slam_geometries()
            self._showcase_last_mesh_obj = None
            self._set_showcase_banner("Press Record to scan the room.")
            self.bus.publish("showcase cleared -> idle")
        self.bus.publish("scan cleared")

    def _sync_near_slider(self):
        """Point the shared near-contrast slider at the control the current mode
        uses: distance cutoff (window), strength (emphasis), or disabled."""
        if self.near_mode == "window":
            self.lbl_near.text = "cutoff m"
            self.sl_near.enabled = True
            self.sl_near.set_limits(0.3, 5.0)
            self.sl_near.double_value = self.near_cutoff_m
        elif self.near_mode == "emphasis":
            self.lbl_near.text = "strength"
            self.sl_near.enabled = True
            self.sl_near.set_limits(0.0, 1.0)
            self.sl_near.double_value = self.near_emphasis
        else:                                    # off / equalize -> no scalar to tune
            self.lbl_near.text = "near " + ("(auto)" if self.near_mode == "equalize" else "(off)")
            self.sl_near.enabled = False

    def _on_near_mode(self, text, index):
        self.near_mode = text
        self._sync_near_slider()
        self.bus.publish(f"near contrast -> {text}")

    def _on_near_value(self, value):
        if self.near_mode == "window":
            self.near_cutoff_m = float(value)
        elif self.near_mode == "emphasis":
            self.near_emphasis = float(value)

    def _on_surface_enabled(self, checked):
        self.surface_enabled = checked
        self.bus.publish(f"surface interpolation -> {'on' if checked else 'off'}")
        self._camera_set = False
        if not checked:
            self._remove_mesh_geometry()

    def _on_surface_mode(self, text, index):
        self.surface_mode = text
        self.bus.publish(f"surface adjacency -> {text}")
        self._camera_set = False


    def _on_surface_threshold(self, value):
        self.surface_threshold_pct = float(value)

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
        # H toggles the help dialog, G the orientation gizmo, C clears the scan; everything else
        # falls through to the scene.
        gui = self._gui
        if event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.H:
            self._show_help()
            return True
        if event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.M:
            self.metrics_overlay = not self.metrics_overlay
            self.overlay.visible = self.metrics_overlay
            self.chk_metrics.checked = self.metrics_overlay
            self.bus.publish(f"metrics overlay -> {'on' if self.metrics_overlay else 'off'}")
            return True
        if event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.G:
            self.imu_gizmo = not self.imu_gizmo
            if not self.imu_gizmo and self._gizmo_added:
                self.scene_widget.scene.remove_geometry(_GIZMO_GEOM)
                self._gizmo_added = False
            self.bus.publish(f"IMU gizmo -> {'on' if self.imu_gizmo else 'off'}")
            return True
        if event.type == gui.KeyEvent.DOWN and event.key == gui.KeyName.C:
            self._on_clear_scan()
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
            if self.showcase_enabled:
                self._enter_showcase_recording()
        else:
            path = self.recorder.path   # captured before stop() clears it
            self.recorder.stop()
            self.btn_record.text = "Record"
            self.bus.publish("recording stopped")
            from .slam.showcase import ShowcasePhase
            if self.showcase_enabled and self.showcase_phase is ShowcasePhase.RECORDING:
                self._enter_showcase_processing(path)

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
                 "near_mode", "near_cutoff_m", "near_emphasis",
                 "surface_enabled", "surface_mode", "surface_threshold_pct",
                 "imu_gizmo", "sensors_panel", "gizmo_scale", "metrics_overlay",
                 "yaw_fusion", "yaw_fusion_tau", "mag_cal_path",
                 "yaw_anomaly_frac", "yaw_motion_rate_dps", "yaw_gimbal_margin_deg")


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
