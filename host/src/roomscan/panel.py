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
from . import theme
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
_TRAJ_HEAD_GEOM = "__slam_traj_head__"  # glowing marker at the sensor's current pose
_FLOOR_GRID_GEOM = "__floor_grid__"    # grounded stage grid, SLAM/Showcase only
_FOV_GEOM = "__fov_indicator__"        # faint camera FoV frustum, RECORDING/SLAM only
_FOV_RANGE_M = 0.5                     # how far out the frustum rays/edges are drawn
_CAPTURE_SQUARE_GEOM = "__capture_square__"  # bright planar capture-area quad, RECORDING/SLAM
_CAPTURE_SQUARE_DEPTH_M = 0.75          # fixed depth (m) the capture square is drawn at
_GIZMO_GEOM = "__imu_gizmo__"
_RESULTS_DIR = "results"               # Showcase FINAL save target (mesh + trajectory)
_GIZMO_ANCHOR = np.array([0.0, 0.0, 0.0], dtype=np.float64)  # fixed scene anchor; calibrate later
_UI_PERIOD = 0.25               # <=4 Hz label / sensors / metrics / log refresh
                                # (the IR pane is NOT throttled here -- it renders
                                #  per frame, in lockstep with the point cloud)
_EXPOSURE_DEBOUNCE = 0.4        # s to settle before sending a dragged exposure value

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

# Camera-follow ("first-person") mode (owner request): default OFF, toggled
# per-view. `_FOLLOW_BACK_OFF_M` pulls the eye a hair behind the sensor along
# -forward for a touch of context (0 would put the eye exactly at the sensor);
# `_FOLLOW_LOOK_AHEAD_M` is how far ahead of the sensor the look-at center
# sits. `_FOLLOW_SMOOTH` is the per-tick lerp fraction toward the target
# eye/center so per-frame pose noise (SLAM jitter) doesn't jitter the view.
# Lowered 0.25 -> 0.12 (owner report: view was visibly shaky even with the
# sensor stationary on a tripod). An EMA at alpha attenuates the steady-state
# noise std by sqrt(alpha/(2-alpha)): 0.25 -> 0.38x, 0.12 -> 0.25x. It cannot
# fully remove the jitter (the root is per-frame ICP translation noise on the
# 54x42 depth), but 0.12 takes it from "shaky" to "gently floating" while the
# added lag (~8 ticks / ~0.3 s at 28 fps) stays acceptable for a live preview.
_FOLLOW_BACK_OFF_M = 0.3
_FOLLOW_LOOK_AHEAD_M = 1.0
_FOLLOW_SMOOTH = 0.12

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
    "            A bright square shows exactly what the sensor is capturing now;",
    "            Follow camera (first-person) rides the sensor's pose instead",
    "            of free-orbit -- off by default, toggle back off to resume orbit.",
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


def _fov_frustum_lines(pose, fov_h_deg: float, fov_v_deg: float,
                       range_m: float = _FOV_RANGE_M):
    """(points (5,3), lines (8,2)) outlining a small pyramid frustum for the
    camera FoV at `pose` (4x4 world<-camera, Open3D CV convention: x-right,
    y-down, z-forward -- the same convention `slam/frames.py`'s poses use):
    the camera origin plus its four FoV corners at `range_m` along the view
    direction, and the 8 edges (4 rays from the origin + the far rectangle)
    that outline it. Pure -- unit-tested; feeds the faint FoV `LineSet`
    (`_FOV_GEOM`) drawn during RECORDING/SLAM (owner request)."""
    pose = np.asarray(pose, dtype=np.float64)
    origin = pose[:3, 3]
    r = pose[:3, :3]
    half_h = np.tan(np.deg2rad(fov_h_deg) / 2.0)
    half_v = np.tan(np.deg2rad(fov_v_deg) / 2.0)
    corners_cam = np.array([
        [-half_h, -half_v, 1.0],   # "upper" (y-down -> negative y is up)
        [half_h, -half_v, 1.0],
        [half_h, half_v, 1.0],
        [-half_h, half_v, 1.0],
    ])
    corners_cam /= np.linalg.norm(corners_cam, axis=1, keepdims=True)
    corners_world = origin + range_m * (corners_cam @ r.T)
    points = np.vstack([origin[None, :], corners_world])
    lines = np.array([
        [0, 1], [0, 2], [0, 3], [0, 4],   # 4 rays from the camera origin
        [1, 2], [2, 3], [3, 4], [4, 1],   # far rectangle
    ], dtype=np.int64)
    return points, lines


def capture_square_corners(pose, fov_h_deg: float, fov_v_deg: float,
                           depth: float = _CAPTURE_SQUARE_DEPTH_M):
    """(4,3) ndarray: the camera FoV footprint at `pose`, as a genuinely
    PLANAR rectangle at `depth` metres in front of the sensor -- what the
    sensor is capturing RIGHT NOW (owner request: "show a square indicating
    the capture area"). Corner order top-left/top-right/bottom-right/
    bottom-left (matching `_fov_frustum_lines`'s corner ordering), each:

        corner = apex + depth*forward +/- (depth*tan(fov_h/2))*right
                                       +/- (depth*tan(fov_v/2))*up

    computed directly in camera space (all 4 corners share z_cam == depth)
    and then rotated+translated by the rigid `pose` -- a rotation+translation
    of a planar set of points is still planar, so this is coplanar by
    construction. Unlike `_fov_frustum_lines` (whose far corners are
    normalized onto a sphere of radius `range_m` -- a cosmetic
    simplification that's fine for a faint hint-only indicator), this is the
    shape we actually claim is "the capture area", so it must be exact.

    `pose` is a 4x4 world<-camera matrix in the Open3D CV convention
    (x-right, y-down, z-forward -- same as `slam/frames.py`'s poses and
    `_fov_frustum_lines`). Pure -- unit-tested."""
    pose = np.asarray(pose, dtype=np.float64)
    apex = pose[:3, 3]
    r = pose[:3, :3]
    half_h = depth * np.tan(np.deg2rad(fov_h_deg) / 2.0)
    half_v = depth * np.tan(np.deg2rad(fov_v_deg) / 2.0)
    corners_cam = np.array([
        [-half_h, -half_v, depth],   # top-left (y-down -> negative y is up)
        [half_h, -half_v, depth],    # top-right
        [half_h, half_v, depth],     # bottom-right
        [-half_h, half_v, depth],    # bottom-left
    ])
    return apex + corners_cam @ r.T


def follow_camera_target(pose, back_off: float = _FOLLOW_BACK_OFF_M,
                         look_ahead: float = _FOLLOW_LOOK_AHEAD_M, up=None):
    """(eye, center, up) camera placement for camera-follow mode (owner
    request: "make SLAM mode be from the perspective of the camera"). `eye`
    sits `back_off` metres BEHIND the sensor along -forward (a hair of
    context so the view isn't pinned exactly to the sensor's nose; pass 0 to
    put the eye exactly at the sensor position), `center` sits `look_ahead`
    metres AHEAD of the sensor along +forward (what `look_at` aims the
    camera at, so the view translates+rotates with the sensor as it's
    carried around), and `up` is the fixed world-up convention (`_WORLD_UP`
    == `slam.frames.world_up()`, `[0,-1,0]`) unless overridden.

    `pose` is a 4x4 world<-camera matrix, same convention as
    `capture_square_corners`/`_fov_frustum_lines`. Pure -- unit-tested; feeds
    `_apply_follow_camera`, which additionally smooths eye/center across
    ticks so per-frame pose noise doesn't jitter the view."""
    pose = np.asarray(pose, dtype=np.float64)
    sensor_pos = pose[:3, 3]
    forward = pose[:3, 2]
    if up is None:
        up = _WORLD_UP
    eye = sensor_pos - back_off * forward
    center = sensor_pos + look_ahead * forward
    return eye, center, np.asarray(up, dtype=np.float64)


def _eta_seconds(elapsed_s: float, fraction: float) -> float | None:
    """Estimated remaining seconds from elapsed wall time and completion
    `fraction` in [0, 1]: eta = elapsed * (1 - frac) / frac. None when
    `fraction` is too small to safely extrapolate from (would blow up) or
    already done -- callers should show no ETA in that case. Pure --
    unit-tested."""
    if fraction is None or fraction <= 1e-3 or fraction >= 1.0:
        return None
    return elapsed_s * (1.0 - fraction) / fraction


def _format_eta(seconds: float | None) -> str:
    """'~M:SS left' from a remaining-seconds estimate, or '' when `seconds`
    is None (too early to estimate) or negative. Pure -- unit-tested; ASCII
    only (no unicode tilde/ellipsis substitutes -- see Issue #4)."""
    if seconds is None or seconds < 0:
        return ""
    total = int(round(seconds))
    m, s = divmod(total, 60)
    return f"~{m}:{s:02d} left"


def _showcase_result_paths(ts: str, results_dir: str = _RESULTS_DIR) -> tuple[str, str]:
    """(mesh_path, trajectory_path) for a Showcase FINAL save at timestamp
    `ts` (e.g. ``time.strftime("%Y%m%d_%H%M%S")``) -- pure, unit-tested.
    Mirrors `_on_record`'s ``captures/panel_<ts>.bin`` naming, but in a
    separate ``results/`` dir since these are processed OUTPUTS, not raw
    device captures."""
    base = f"{results_dir}/showcase_{ts}"
    return f"{base}.ply", f"{base}.tum"


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

        # Component A (off-thread adaptive mesh): MeshPrep does the O(map-size)
        # shading/decimation/wall-split/floor work off the GUI thread; the tick
        # only uploads its ready packet at `_mesh_upload_period` s, feeding the
        # measured upload wall-time back for adaptive decimation. See
        # slam/meshprep.py + docs/superpowers/plans/2026-07-13-live-view-fps.md.
        from .slam.config import SlamConfig as _SlamCfg
        _view_cfg = _SlamCfg.load()
        self.mesh_prep = None
        self._mesh_prep_seq = 0
        self._last_mesh_upload_t = 0.0
        self._mesh_upload_period = (1.0 / _view_cfg.mesh_upload_hz
                                    if _view_cfg.mesh_upload_hz > 0 else 0.0)
        self._live_vertex_budget = _view_cfg.live_vertex_budget
        self._fps_budget_ms = _view_cfg.fps_budget_ms

        # FoV indicator (owner request): faint camera-frustum LineSet, shown
        # during RECORDING/SLAM, recomputed only when the pose actually moves.
        # The bright capture-area square (_CAPTURE_SQUARE_GEOM) shares this
        # same pose-change gate (see _update_fov_geometry).
        self._fov_last_pose = None

        # Camera-follow ("first-person") mode (owner request): off by default,
        # toggled by chk_follow_camera. `_follow_eye`/`_follow_center` hold the
        # smoothed camera state across ticks (see _apply_follow_camera); reset
        # to None whenever follow is (re-)enabled so it snaps to the first pose
        # instead of lerping in from a stale/zero position.
        self.follow_camera_enabled = False
        self._follow_eye = None
        self._follow_center = None

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
        self._showcase_save_thread = None      # short-lived thread: final mesh+trajectory -> disk

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
        self.slam_line_material.line_width = 4.0   # trajectory ribbon (Phase 6 UX)
        self.traj_head_material = rendering.MaterialRecord()
        self.traj_head_material.shader = "defaultUnlit"
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
        # FoV indicator (owner request): faint thin lines, muted gray so it
        # reads as a hint rather than competing with the mesh/trajectory.
        self.fov_material = rendering.MaterialRecord()
        self.fov_material.shader = "unlitLine"
        self.fov_material.line_width = 1.0
        # Capture-area square (owner request): bright + thick so it reads
        # clearly, distinct from the faint FoV frustum (gray, 1px) and the
        # green trajectory line.
        self.capture_square_material = rendering.MaterialRecord()
        self.capture_square_material.shader = "unlitLine"
        self.capture_square_material.line_width = 3.0
        # Floor grid ("the stage"): a single faint hairline, quieter than the
        # FoV frustum so it grounds the room without competing with it.
        self.floor_material = rendering.MaterialRecord()
        self.floor_material.shader = "unlitLine"
        self.floor_material.line_width = 1.0
        self._floor_last_bounds = None   # (min,max) the current grid was built for
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
        # Graded "stage" background (Phase 6 UX): a vertical gradient reads as a
        # horizon and gives the scene depth, vs the old flat void. Filament
        # stretches the image to the viewport, so a tall thin gradient suffices;
        # the paired clear color matches its midpoint (theme.BG_CLEAR_*).
        self._bg_grad_dark = self._np_to_o3d(
            theme.vertical_gradient(2, 512, theme.STAGE_TOP_DARK, theme.STAGE_BOTTOM_DARK))
        self._bg_grad_light = self._np_to_o3d(
            theme.vertical_gradient(2, 512, theme.STAGE_TOP_LIGHT, theme.STAGE_BOTTOM_LIGHT))
        self.scene_widget.scene.set_background(theme.BG_CLEAR_DARK, self._bg_grad_dark)
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

        # PROCESSING-phase progress bar (Issue #3): a real gui.ProgressBar
        # (0..1 = the PostProcessWorker's latest().fraction), stacked just
        # below the banner text; hidden outside PROCESSING.
        self.progress_bar = gui.ProgressBar()
        self.progress_bar.value = 0.0
        self.progress_bar.visible = False
        self.window.add_child(self.progress_bar)

        # Showcase FINAL "reveal card" (Phase 6 UX): a rendered instrument card
        # (cards.render_scan_complete_card) shown as a lower-third over the 3D
        # scene the moment a scan completes, replacing the system-font banner
        # for that moment. A separate ImageWidget from the transient banner so
        # the plain REC/PROCESSING text plumbing stays untouched; positioned in
        # _on_layout, sized from its rendered image (mirrors the metrics HUD).
        self.reveal_card = gui.ImageWidget(self._np_to_o3d(blank))
        self._reveal_card_size = (10, 10)
        self.reveal_card.visible = False
        self.window.add_child(self.reveal_card)

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
        # Camera-follow (owner request): governs both this view and Showcase
        # mode's RECORDING phase (see _render_slam_frame/_render_showcase_
        # recording) since both share the SLAM/Showcase mesh pipeline. Off by
        # default -- free-orbit stays the default camera behavior.
        self.chk_follow_camera = gui.Checkbox("Follow camera (first-person)")
        self.chk_follow_camera.checked = False
        self.chk_follow_camera.set_on_checked(self._on_follow_camera_toggle)
        slam.add_child(self.chk_follow_camera)

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
        banner_w = min(scene_w - 2 * pad, int(32 * em))
        self.showcase_banner.frame = gui.Rect(r.x + pad, banner_y, banner_w, banner_h)
        # Progress bar: stacked directly below the banner text, same width.
        pb_h = int(0.4 * em)
        pb_y = banner_y + banner_h + int(0.15 * em)
        self.progress_bar.frame = gui.Rect(r.x + pad, pb_y, banner_w, pb_h)
        # Reveal card: centered horizontally in the scene, in the lower third,
        # at its native rendered size. Computed unconditionally (cheap); its
        # own `.visible` gates whether it's actually drawn.
        cw, ch = self._reveal_card_size
        card_x = r.x + max(pad, (scene_w - cw) // 2)
        card_y = r.y + r.height - ch - int(3 * em)
        self.reveal_card.frame = gui.Rect(card_x, card_y, cw, ch)

    # ---- lifecycle ----------------------------------------------------------
    def start(self):
        self.resource_sampler.start()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _on_close(self):
        self._stop = True
        if self.slam_worker is not None:
            self.slam_worker.stop()        # join the SLAM worker thread before teardown
        if self.mesh_prep is not None:
            self.mesh_prep.stop()          # join the off-thread mesh-prep worker
        self._join_showcase_workers()      # join Showcase's preview/post-process/loader threads
        if self._showcase_save_thread is not None:
            self._showcase_save_thread.join(timeout=10.0)   # let an in-flight result save finish
            self._showcase_save_thread = None
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
        contract (see panel.py's module docstring and slam/worker.py).

        Issue #1 (reflectance-in-live): `self._latest_outputs` is the same
        dict `_render_frame` just decoded this frame (set before dispatching
        here) -- when the native transform is available it already carries
        "reflectance"/"confidence" planes (`run()` requests all three
        outputs whenever the DLL is present), so forwarding them to
        `submit()` needs no extra transform call, just reading what's
        already there. `.get(...)` degrades to None for a depth-only source
        (Phase 1 DEPTH_ZF32 passthrough, or no transform DLL), exactly
        `Mapper.step`'s pre-existing default."""
        if self.slam_worker is None:
            from .slam.worker import SlamWorker
            from .slam.config import preferred_device
            from .slam.backend import make_slam_worker
            from .slam.meshprep import MeshPrep
            h, w = depth.shape
            self.slam_worker = make_slam_worker(w, h, fov_h=self.args.fov_h,
                                                fov_v=self.args.fov_v,
                                                device=preferred_device())
            self.slam_worker.start()
            self.mesh_prep = MeshPrep(vertex_budget=self._live_vertex_budget,
                                      fps_budget_ms=self._fps_budget_ms)
            self.mesh_prep.start()

        quat = self.sensor_state.fused_quat()
        if quat is not None:
            env = self.sensor_state.latest_env()
            pressure = env.pressure_pa if env is not None else None
            outputs = self._latest_outputs or {}
            self.slam_worker.submit(depth, quat, pressure,
                                    reflectance=outputs.get("reflectance"),
                                    confidence=outputs.get("confidence"))

        latest = self.slam_worker.latest()
        if latest is None:
            self.lbl_slam_tracking.text = "tracking: waiting for IMU..."
            self.lbl_slam_ms.text = "slam_ms: --"
            return
        mesh, trajectory, step = latest
        self.lbl_slam_tracking.text = f"tracking: {'LOST' if step.tracking_lost else 'ok'}"
        self.lbl_slam_ms.text = f"slam_ms: {step.slam_ms:.1f}"
        self._update_fov_geometry(step.pose)
        if self.follow_camera_enabled:
            self._apply_follow_camera(step.pose)
            # First-person (owner request): hide the stray IMU gizmo ("camera
            # icon", frozen in the scene since entering SLAM) and the green
            # trajectory "trail" -- both clutter/occlude the first-person view.
            # Removing the gizmo resets `_gizmo_added` so the classic view
            # re-adds it cleanly on exit; the trail is simply not re-added
            # below while following, and reappears the next frame once follow
            # is turned off.
            self._hide_first_person_clutter()

        # Feed MeshPrep only when the worker publishes a genuinely NEW mesh
        # object (identity check) -- all the O(map-size) work happens on its
        # thread, never here.
        if (mesh is not None and mesh is not self._slam_last_mesh_obj
                and len(mesh.vertex.positions) > 0):
            self._mesh_prep_seq += 1
            self.mesh_prep.submit(mesh, mesh_seq=self._mesh_prep_seq,
                                  glow_origin=step.pose[:3, 3], wall_mode=self.wall_mode)
            self._slam_last_mesh_obj = mesh

        # Upload a ready packet at most `mesh_upload_hz` times/sec; measure the
        # upload wall-time and feed it back to MeshPrep's adaptive controller.
        now = time.monotonic()
        if now - self._last_mesh_upload_t >= self._mesh_upload_period:
            packet = self.mesh_prep.latest()
            if packet is not None:
                t0 = time.monotonic()
                self._upload_mesh_packet(packet)
                self.mesh_prep.note_upload_ms((time.monotonic() - t0) * 1000.0)
                self._last_mesh_upload_t = now

        # Trajectory ribbon + head marker (Phase 6 UX) -- skipped entirely in
        # follow/first-person mode (the eye is at the sensor). The <2-point
        # BUG-009 guard now lives inside `_upload_trajectory`.
        if not self.follow_camera_enabled:
            self._upload_trajectory(trajectory)

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
            # mesh may live on a non-CPU compute device (Mapper(device=...));
            # .cpu() is a no-op when it's already on CPU.
            pts_list.append(mesh.vertex.positions.cpu().numpy())
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
        if sc.has_geometry(_TRAJ_HEAD_GEOM):
            sc.remove_geometry(_TRAJ_HEAD_GEOM)
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if sc.has_geometry(_MESH_WALLS_GEOM):
            sc.remove_geometry(_MESH_WALLS_GEOM)
        self._remove_floor_grid()
        self._remove_fov_geometry()

    def _remove_floor_grid(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_FLOOR_GRID_GEOM):
            sc.remove_geometry(_FLOOR_GRID_GEOM)
        self._floor_last_bounds = None

    def _update_floor_grid(self, verts):
        """(Re)build the grounded 'stage' floor grid from the current map's
        bounds (`verts`, an (N,3) array of the just-uploaded mesh's vertices).
        Only rebuilds when the bounds shift by more than the grid pitch -- the
        grid needn't track every sub-pitch mesh growth, and rebuilding a
        LineSet every mesh extraction would churn the renderer needlessly.
        Placed at the map's floor plane via the pure `theme.floor_grid_lines`;
        shared by the SLAM view and Showcase mode (both call `_upload_slam_mesh`)."""
        if verts is None or len(verts) == 0:
            return
        from .slam.frames import world_up
        mn = verts.min(axis=0)
        mx = verts.max(axis=0)
        spacing = 0.5
        if self._floor_last_bounds is not None:
            omn, omx = self._floor_last_bounds
            if (np.abs(mn - omn).max() < spacing and np.abs(mx - omx).max() < spacing
                    and self.scene_widget.scene.has_geometry(_FLOOR_GRID_GEOM)):
                return
        pts, lines = theme.floor_grid_lines(mn, mx, up=world_up(), spacing=spacing)
        sc = self.scene_widget.scene
        if sc.has_geometry(_FLOOR_GRID_GEOM):
            sc.remove_geometry(_FLOOR_GRID_GEOM)
        if len(pts) >= 2 and len(lines) > 0:   # BUG-009: never upload an empty LineSet
            ls = self._o3d.geometry.LineSet()
            ls.points = self._o3d.utility.Vector3dVector(pts)
            ls.lines = self._o3d.utility.Vector2iVector(lines)
            ls.colors = self._o3d.utility.Vector3dVector(
                np.tile([list(theme.FLOOR_GRID)], (len(lines), 1)))
            sc.add_geometry(_FLOOR_GRID_GEOM, ls, self.floor_material)
            self._floor_last_bounds = (mn, mx)

    def _upload_trajectory(self, trajectory, *, with_head=True):
        """(Re)upload the trajectory as a fading cyan 'ribbon' (dim oldest ->
        bright newest, via `theme.trajectory_ramp`) plus an optional glowing
        head marker at the sensor's current pose -- the motion read that
        replaces the old flat lime debug line. Shared by the SLAM view and
        Showcase mode. `with_head` is False in camera-follow/first-person mode
        (the eye is AT the sensor, so a marker in front of it just occludes).

        BUG-009: a LineSet with < 2 points has 0 segments and hard-crashes
        Filament -- callers already gate on `len(trajectory) >= 2`, and this
        removes both geometries and returns for anything shorter."""
        sc = self.scene_widget.scene
        if trajectory is None or len(trajectory) < 2:
            if sc.has_geometry(_SLAM_TRAJ_GEOM):
                sc.remove_geometry(_SLAM_TRAJ_GEOM)
            if sc.has_geometry(_TRAJ_HEAD_GEOM):
                sc.remove_geometry(_TRAJ_HEAD_GEOM)
            return
        pts = np.array([T[:3, 3] for T in trajectory], dtype=np.float64)
        lines = self._o3d.geometry.LineSet()
        lines.points = self._o3d.utility.Vector3dVector(pts)
        idx = np.stack([np.arange(len(pts) - 1), np.arange(1, len(pts))], axis=1)
        lines.lines = self._o3d.utility.Vector2iVector(idx)
        lines.colors = self._o3d.utility.Vector3dVector(theme.trajectory_ramp(len(idx)))
        if sc.has_geometry(_SLAM_TRAJ_GEOM):
            sc.remove_geometry(_SLAM_TRAJ_GEOM)
        sc.add_geometry(_SLAM_TRAJ_GEOM, lines, self.slam_line_material)

        if sc.has_geometry(_TRAJ_HEAD_GEOM):
            sc.remove_geometry(_TRAJ_HEAD_GEOM)
        if with_head:
            extent = float(np.ptp(pts, axis=0).max())
            radius = max(extent * 0.02, 0.01)
            head = self._o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            head.translate(pts[-1])
            head.paint_uniform_color(list(theme.ACCENT))
            sc.add_geometry(_TRAJ_HEAD_GEOM, head, self.traj_head_material)

    def _update_fov_geometry(self, pose):
        """Faint camera-FoV frustum indicator (owner request), shown during
        RECORDING/SLAM. Recomputed only when `pose` has actually moved (a
        plain array-equality check against the last pose used -- cheap, and
        this is only ever called once per processed frame, not per GUI
        tick), via the pure `_fov_frustum_lines` helper. `pose` is None-safe
        (removes the geometry instead) even though no current caller passes
        None, matching the module's general defensiveness.

        Also (re)draws the bright capture-area square (`_update_capture_square`,
        owner request: "show a square indicating the capture area") -- it
        shares this method's pose/None gating since both indicators are
        derived from the same per-frame pose."""
        sc = self.scene_widget.scene
        if pose is None:
            self._remove_fov_geometry()
            return
        pose = np.asarray(pose, dtype=np.float64)
        if self._fov_last_pose is not None and np.array_equal(pose, self._fov_last_pose):
            return
        self._fov_last_pose = pose.copy()
        points, lines = _fov_frustum_lines(pose, self.args.fov_h, self.args.fov_v)
        if sc.has_geometry(_FOV_GEOM):
            sc.remove_geometry(_FOV_GEOM)
        # BUG-009 (see _render_slam_frame's trajectory upload): a LineSet
        # with < 2 points hard-crashes Filament -- guard it here too, even
        # though `_fov_frustum_lines` always returns a fixed 5-point/8-line
        # shape today.
        if len(points) >= 2 and len(lines) > 0:
            ls = self._o3d.geometry.LineSet()
            ls.points = self._o3d.utility.Vector3dVector(points)
            ls.lines = self._o3d.utility.Vector2iVector(lines)
            ls.colors = self._o3d.utility.Vector3dVector(
                np.tile([[0.55, 0.55, 0.6]], (len(lines), 1)))   # faint gray, not attention-grabbing
            sc.add_geometry(_FOV_GEOM, ls, self.fov_material)
        self._update_capture_square(pose)

    def _update_capture_square(self, pose):
        """Bright planar quad outlining exactly what the sensor is capturing
        RIGHT NOW at `_CAPTURE_SQUARE_DEPTH_M` (owner request) -- 4 corners
        from the pure `capture_square_corners` helper + the closing edge
        (4 line segments, a closed loop), distinct in color from both the
        faint FoV frustum (muted gray) and the green trajectory line. Shown
        whenever there's a live pose (RECORDING/SLAM), independent of
        camera-follow mode -- it indicates the capture area either way."""
        sc = self.scene_widget.scene
        corners = capture_square_corners(pose, self.args.fov_h, self.args.fov_v,
                                         _CAPTURE_SQUARE_DEPTH_M)
        lines = np.array([[0, 1], [1, 2], [2, 3], [3, 0]], dtype=np.int64)
        if sc.has_geometry(_CAPTURE_SQUARE_GEOM):
            sc.remove_geometry(_CAPTURE_SQUARE_GEOM)
        # BUG-009 (see _render_slam_frame's trajectory upload): a LineSet with
        # < 2 points hard-crashes Filament -- guard it here too, even though
        # `capture_square_corners` always returns a fixed 4-point shape today.
        if len(corners) < 2 or len(lines) == 0:
            return
        ls = self._o3d.geometry.LineSet()
        ls.points = self._o3d.utility.Vector3dVector(corners)
        ls.lines = self._o3d.utility.Vector2iVector(lines)
        ls.colors = self._o3d.utility.Vector3dVector(
            np.tile([list(theme.ACCENT)], (len(lines), 1)))   # accent cyan -- the live capture beam
        sc.add_geometry(_CAPTURE_SQUARE_GEOM, ls, self.capture_square_material)

    def _remove_fov_geometry(self):
        sc = self.scene_widget.scene
        if sc.has_geometry(_FOV_GEOM):
            sc.remove_geometry(_FOV_GEOM)
        if sc.has_geometry(_CAPTURE_SQUARE_GEOM):
            sc.remove_geometry(_CAPTURE_SQUARE_GEOM)
        self._fov_last_pose = None

    def _apply_follow_camera(self, pose):
        """Camera-follow mode (owner request: "make SLAM mode be from the
        perspective of the camera"): each time this is called (once per
        processed frame with a live pose, from `_render_slam_frame` /
        `_render_showcase_recording` -- same call sites `_update_fov_geometry`
        uses), ease the render camera toward `follow_camera_target(pose)`
        instead of the free-orbit turntable, so the view translates+rotates
        as the sensor is carried/aimed around the room.

        Light exponential smoothing (`_FOLLOW_SMOOTH` per call) keeps
        per-frame SLAM pose noise from jittering the view -- not a
        lag-behind follow, just enough to take the edge off. The free-orbit
        state (`_cam_target`/`_cam_az`/`_cam_radius`) is deliberately left
        untouched here, so toggling follow back off (`_on_follow_camera_toggle`)
        resumes free-orbit exactly where it was, via one `_apply_camera()`
        call -- it never gets stuck on the last follow-mode frame."""
        if self.scene_widget.frame.width <= 0 or self.scene_widget.frame.height <= 0:
            return
        eye, center, up = follow_camera_target(pose)
        if self._follow_eye is None:
            self._follow_eye, self._follow_center = eye, center
        else:
            self._follow_eye = self._follow_eye + _FOLLOW_SMOOTH * (eye - self._follow_eye)
            self._follow_center = self._follow_center + _FOLLOW_SMOOTH * (center - self._follow_center)
        self.scene_widget.look_at(self._follow_center.astype(np.float32),
                                  self._follow_eye.astype(np.float32),
                                  up.astype(np.float32))

    def _hide_first_person_clutter(self):
        """Remove the IMU gizmo ("camera icon") and the trajectory trail while
        camera-follow is active (owner request). The gizmo is removed here
        (and `_gizmo_added` cleared so the classic view re-adds it on exit);
        the trail is gated off at its add site in `_render_slam_frame`, so we
        only need to remove any copy already in the scene."""
        sc = self.scene_widget.scene
        if sc.has_geometry(_GIZMO_GEOM):
            sc.remove_geometry(_GIZMO_GEOM)
            self._gizmo_added = False
        if sc.has_geometry(_SLAM_TRAJ_GEOM):
            sc.remove_geometry(_SLAM_TRAJ_GEOM)
        if sc.has_geometry(_TRAJ_HEAD_GEOM):
            sc.remove_geometry(_TRAJ_HEAD_GEOM)

    def _on_follow_camera_toggle(self, checked):
        self.follow_camera_enabled = checked
        self._follow_eye = None
        self._follow_center = None
        if not checked:
            # Restore free-orbit immediately -- _cam_target/_cam_az/_cam_radius
            # were never touched while following, so this snaps straight back
            # to where free-orbit was left, not stuck on the last followed frame.
            self._apply_camera()
        self.bus.publish(f"Follow camera -> {'on' if checked else 'off'}")

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

    def _upload_slam_mesh(self, mesh, glow_origin=None):
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

        Issue #1 (reflectance-in-live): `mesh.vertex.colors` is a real
        reflectance-derived grayscale image once `TsdfMap.integrate()` was
        fed a `color` array (Task 13, now wired from the live panel too --
        see `_render_slam_frame`/`_render_showcase_recording`), or the
        uniform all-[0,0,0] black a depth-only integration always produces
        (`mesh_colors_are_meaningful` tells the two apart). When meaningful,
        that color is used as the base and MODULATED by `shade_brightness`
        (the scalar Lambert term `shade_colors` itself bakes into its fixed
        base color) so the surface still reads as 3D; otherwise this falls
        back to plain `shade_colors`, byte-identical to this method's
        pre-Task-14 behavior.
        """
        from .slam.shading import (height_base_colors, height_tint_hue,
                                    mesh_colors_are_meaningful,
                                    shade_brightness, shade_colors, wall_triangle_mask,
                                    wavefront_glow)
        from .slam.frames import world_up
        sc = self.scene_widget.scene
        # mesh may live on a non-CPU compute device (Mapper(device=...)) --
        # Filament (the GUI renderer) only ever renders CPU geometry, and
        # to_legacy() itself requires a CPU tensor mesh; .cpu() is a no-op
        # when it's already on CPU.
        legacy_mesh = mesh.cpu().to_legacy()
        legacy_mesh.compute_vertex_normals()
        normals = np.asarray(legacy_mesh.vertex_normals)
        raw_colors = np.asarray(legacy_mesh.vertex_colors)
        if mesh_colors_are_meaningful(raw_colors):
            # Live reflectance mesh: keep its own grey texture but grade it by
            # height ("the stage") so the room reads cool-low / warm-high instead
            # of a flat grey -- height_tint_hue is a luma-preserving multiplier.
            brightness = shade_brightness(normals)
            hue = height_tint_hue(np.asarray(legacy_mesh.vertices), world_up())
            final_colors = np.clip(raw_colors * brightness[:, None] * hue, 0.0, 1.0)
        else:
            # The TSDF mesh's vertex colors are always [0,0,0] here (no color
            # was integrated) and this material is defaultUnlit with no scene
            # lights -- bake a fixed-light shade in so it's not invisible
            # black (see slam/shading.py's module docstring). The base albedo
            # is height-cued ("the stage"): cool at the floor, warm up high, so
            # depth reads at a glance instead of a flat clay monochrome.
            base = height_base_colors(np.asarray(legacy_mesh.vertices), world_up())
            final_colors = shade_colors(normals, base=base)
        # Materialization wavefront (live scanning only): glow the surface near
        # the sensor's current position. Blended in BEFORE the wall split so
        # both the opaque and wall submeshes inherit it (they copy vertex
        # colors). None on the finished PROCESSING/FINAL mesh -> no glow.
        if glow_origin is not None:
            final_colors = wavefront_glow(np.asarray(legacy_mesh.vertices),
                                          glow_origin, final_colors)
        legacy_mesh.vertex_colors = self._o3d.utility.Vector3dVector(final_colors)

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

    def _upload_mesh_packet(self, packet):
        """Build + add_geometry from a `MeshPrep.MeshPacket` (Component A). The
        O(map-size) shading/decimation/wall-split already ran off the GUI thread
        in MeshPrep; this only materializes Open3D geometry and uploads it. Twin
        of `_upload_slam_mesh`'s upload half, sourced from packet arrays; that
        method stays for Showcase PROCESSING/FINAL."""
        o3d = self._o3d
        sc = self.scene_widget.scene
        if sc.has_geometry(_MESH_GEOM):
            sc.remove_geometry(_MESH_GEOM)
        if sc.has_geometry(_MESH_WALLS_GEOM):
            sc.remove_geometry(_MESH_WALLS_GEOM)

        if len(packet.non_wall_tris) > 0:
            m = o3d.geometry.TriangleMesh()
            m.vertices = o3d.utility.Vector3dVector(packet.non_wall_verts)
            m.triangles = o3d.utility.Vector3iVector(packet.non_wall_tris)
            m.vertex_colors = o3d.utility.Vector3dVector(packet.non_wall_colors)
            sc.add_geometry(_MESH_GEOM, m, self.mesh_material)

        if len(packet.wall_tris) > 0:
            wm = o3d.geometry.TriangleMesh()
            wm.vertices = o3d.utility.Vector3dVector(packet.wall_verts)
            wm.triangles = o3d.utility.Vector3iVector(packet.wall_tris)
            wm.vertex_colors = o3d.utility.Vector3dVector(packet.wall_colors)
            if packet.wall_mode == "translucent":
                sc.add_geometry(_MESH_WALLS_GEOM, wm, self.wall_translucent_material)
            else:   # "wireframe"
                wire = o3d.geometry.LineSet.create_from_triangle_mesh(wm)
                # BUG-009: a <2-point / 0-segment LineSet hard-crashes Filament.
                if len(wire.points) >= 2:
                    wire.colors = o3d.utility.Vector3dVector(
                        np.tile([[0.45, 0.60, 0.75]], (len(wire.lines), 1)))
                    sc.add_geometry(_MESH_WALLS_GEOM, wire, self.wall_wire_material)

        self._upload_floor_grid_from_packet(packet.floor_pts, packet.floor_lines)

    def _upload_floor_grid_from_packet(self, pts, lines):
        """Upload the pre-extracted floor grid from a packet. Replaces the
        per-tick `mesh.vertex.positions.cpu().numpy()` copy that used to live in
        `_update_floor_grid` -- MeshPrep already did that O(size) copy + bounds
        off-thread. BUG-009: never upload a <2-point / 0-segment LineSet."""
        o3d = self._o3d
        sc = self.scene_widget.scene
        if sc.has_geometry(_FLOOR_GRID_GEOM):
            sc.remove_geometry(_FLOOR_GRID_GEOM)
        if len(pts) >= 2 and len(lines) > 0:
            ls = o3d.geometry.LineSet()
            ls.points = o3d.utility.Vector3dVector(pts)
            ls.lines = o3d.utility.Vector2iVector(lines)
            ls.colors = o3d.utility.Vector3dVector(
                np.tile([list(theme.FLOOR_GRID)], (len(lines), 1)))
            sc.add_geometry(_FLOOR_GRID_GEOM, ls, self.floor_material)

    def _on_slam_toggle(self, checked):
        if checked and self.showcase_enabled:
            # Mutually exclusive with Showcase mode (see _on_showcase_toggle's
            # symmetric guard) -- this one added line is the only change to this
            # method's pre-existing body below.
            self.chk_showcase.checked = False
            self._on_showcase_toggle(False)
        self.slam_enabled = checked
        if not checked:
            if self.follow_camera_enabled:
                # Follow-camera only ever drives the view from a SLAM/Showcase
                # pose (_render_slam_frame/_render_showcase_recording) -- leaving
                # SLAM without also releasing it would strand the classic view
                # with mouse nav swallowed (_on_mouse) and nothing left to drive
                # the camera. Force it off so free-orbit is always reachable.
                self.chk_follow_camera.checked = False
                self._on_follow_camera_toggle(False)
            if self.slam_worker is not None:
                self.slam_worker.stop()
                self.slam_worker = None
            if self.mesh_prep is not None:
                self.mesh_prep.stop()          # join the off-thread mesh-prep worker
                self.mesh_prep = None
            self._last_mesh_upload_t = 0.0
            self._remove_slam_geometries()
            self._slam_last_mesh_obj = None
            self._camera_set = False
            if self._last_item is not None:   # re-render the last frame as a normal cloud
                self._render_frame(self._last_item)
        else:
            self._camera_set = False          # reframe onto SLAM content once it appears
            self._remove_live_view_geometries()   # owner report: stale cloud must not show through
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
        alongside this -- entirely independent of this preview.

        Issue #1: forwards `self._latest_outputs`' reflectance/confidence
        planes to `submit()` -- see `_render_slam_frame`'s docstring for why
        that's already enough (no extra transform call needed). Issue #2:
        updates the faint FoV indicator from this step's pose."""
        from .slam.worker import SlamWorker
        from .slam.config import preferred_device
        from .slam.backend import make_slam_worker
        if self._showcase_preview_worker is None:
            h, w = depth.shape
            self._showcase_preview_worker = make_slam_worker(w, h, fov_h=self.args.fov_h,
                                                             fov_v=self.args.fov_v,
                                                             device=preferred_device())
            self._showcase_preview_worker.start()

        quat = self.sensor_state.fused_quat()
        tracking_txt = "waiting for IMU..."
        if quat is not None:
            env = self.sensor_state.latest_env()
            pressure = env.pressure_pa if env is not None else None
            outputs = self._latest_outputs or {}
            self._showcase_preview_worker.submit(depth, quat, pressure,
                                                 reflectance=outputs.get("reflectance"),
                                                 confidence=outputs.get("confidence"))
            self._showcase_rec_frames += 1

        latest = self._showcase_preview_worker.latest()
        if latest is not None:
            mesh, trajectory, step = latest
            tracking_txt = "lost" if step.tracking_lost else "ok"
            self._show_showcase_mesh(mesh, glow_origin=step.pose[:3, 3])
            if not self.follow_camera_enabled:
                self._show_showcase_trajectory(trajectory)
            self._slam_camera_frame(mesh, trajectory)
            self._update_fov_geometry(step.pose)
            if self.follow_camera_enabled:
                self._apply_follow_camera(step.pose)
                self._hide_first_person_clutter()   # hide gizmo + trail (owner request)
        self._set_showcase_banner(
            f"REC Recording - scanning... ({self._showcase_rec_frames} frames "
            f"| tracking: {tracking_txt})")

    def _render_showcase_processing(self):
        """PROCESSING phase: render whatever the background `PostProcessWorker`
        has published so far -- the scene stays interactive (orbitable) the
        whole time; each newer, more-complete mesh simply swaps in over the
        last one (see _show_showcase_mesh's identity check). On its terminal
        (done=True) publish, hands off to _enter_showcase_final.

        Issue #3: drives the real `gui.ProgressBar` (0..1 = `latest.fraction`)
        and an ETA computed from elapsed wall time (`_showcase_process_start_ts`,
        set in `_enter_showcase_processing`) via `_eta_seconds`/`_format_eta`.
        The bar is only visible while there's an actual in-progress fraction
        to show (not while still loading the capture, nor once FINAL)."""
        worker = self._showcase_post_worker
        if worker is None:
            self.progress_bar.visible = False
            self._set_showcase_banner("Processing... loading capture...")
            return
        latest = worker.latest()
        if latest is None:
            self.progress_bar.visible = False
            self._set_showcase_banner("Processing  0%")
            return
        self._show_showcase_mesh(latest.mesh)
        self._show_showcase_trajectory(latest.trajectory)
        self._slam_camera_frame(latest.mesh, latest.trajectory)
        if latest.done:
            self.progress_bar.visible = False
            self._enter_showcase_final(latest)
        else:
            frac = latest.fraction
            self.progress_bar.value = frac
            self.progress_bar.visible = True
            elapsed = time.monotonic() - (self._showcase_process_start_ts or time.monotonic())
            eta_str = _format_eta(_eta_seconds(elapsed, frac))
            text = f"Processing  {frac * 100:.0f}%"
            if eta_str:
                text += f"  {eta_str}"
            self._set_showcase_banner(text)

    def _show_showcase_mesh(self, mesh, glow_origin=None):
        """Upload `mesh` if it's new (identity check -- mesh extraction is
        already throttled inside the worker) and non-empty. Shares the
        classic SLAM view's geometry name/material (via `_upload_slam_mesh`)
        since the two modes are mutually exclusive and never rendered in the
        same frame. `glow_origin` (the sensor's current position) enables the
        materialization wavefront during RECORDING; None (PROCESSING/FINAL)
        leaves the finished mesh unglowed."""
        if mesh is None or mesh is self._showcase_last_mesh_obj or len(mesh.vertex.positions) == 0:
            return
        self._upload_slam_mesh(mesh, glow_origin=glow_origin)
        self._update_floor_grid(mesh.vertex.positions.cpu().numpy())
        self._showcase_last_mesh_obj = mesh

    def _show_showcase_trajectory(self, trajectory):
        # Trajectory ribbon + head marker (Phase 6 UX), shared with the SLAM
        # view via `_upload_trajectory`. The <2-point guard (BUG-009: a 0-segment
        # LineSet hard-crashes Filament) lives inside that helper.
        self._upload_trajectory(trajectory)

    def _showcase_target_camera(self, mesh, trajectory):
        """(center, radius) to frame `mesh` + `trajectory`'s bounds -- the same
        approach as `_slam_camera_frame`, duplicated (not shared) so that
        pre-existing method, used by the classic SLAM view, stays untouched.
        Returns None on a still-degenerate (zero-extent) scan."""
        pts_list = [np.zeros((1, 3))]
        if mesh is not None and len(mesh.vertex.positions) > 0:
            # mesh may live on a non-CPU compute device (Mapper(device=...));
            # .cpu() is a no-op when it's already on CPU.
            pts_list.append(mesh.vertex.positions.cpu().numpy())
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
        # The reveal moment (Phase 6 UX): a rendered instrument card instead of
        # the system-font debug banner. _show_reveal_card hides the text banner
        # and falls back to it if the render ever fails.
        self._show_reveal_card(stats.get("frames", 0), stats.get("gap_m", 0.0),
                               stats.get("verts", 0), elapsed)
        self.bus.publish("showcase: scan complete")
        self._save_showcase_result(progress.mesh, progress.trajectory)

    def _save_showcase_result(self, mesh, trajectory):
        """Issue #6: persist the final fused mesh (`.ply`) + trajectory
        (`.tum`) to `results/showcase_<ts>.{ply,tum}` when a Showcase scan
        reaches FINAL, so the owner's scans aren't lost. Runs the actual
        writes on a short-lived daemon thread -- a large mesh (~100 MB) can
        take a moment and this must not stall the GUI tick -- and logs the
        saved paths via `self.bus` (drained onto the log pane on the UI
        tick, so the bus is the safe cross-thread channel). The `mesh.cpu()`
        device->host copy runs INSIDE the save thread (not on the GUI tick):
        on a CUDA build that copy of a ~100 MB mesh is a real D2H transfer
        that must not stall the UI; the mesh is the final, no-longer-mutated
        result at FINAL, so reading it from the save thread is safe (a no-op
        on CPU).

        No-op on an empty mesh (a degenerate/failed scan -- nothing worth
        writing). Never raises into the caller; any write failure is logged."""
        if mesh is None or len(mesh.vertex.positions) == 0:
            self.bus.publish("showcase: nothing to save (empty mesh)")
            return
        o3d = self._o3d
        ts = time.strftime("%Y%m%d_%H%M%S")
        mesh_path, traj_path = _showcase_result_paths(ts)
        traj = list(trajectory or [])

        def _save():
            from .slam import metrics as slam_metrics
            try:
                Path(_RESULTS_DIR).mkdir(parents=True, exist_ok=True)
                mesh_cpu = mesh.cpu()   # D2H copy off the GUI thread (no-op on CPU)
                o3d.t.io.write_triangle_mesh(mesh_path, mesh_cpu)
                # TUM needs a timestamp per pose; we don't carry per-pose wall
                # times to FINAL, so use a monotone synthetic index -- fine for
                # a relative trajectory dump (the poses are what matter).
                timestamps = [float(i) for i in range(len(traj))]
                slam_metrics.write_tum(traj_path, timestamps, traj)
                self.bus.publish(f"showcase: saved {mesh_path} + {traj_path}")
            except Exception as exc:
                self.bus.publish(f"showcase: save failed: {exc!r}")

        self._showcase_save_thread = threading.Thread(target=_save, daemon=True)
        self._showcase_save_thread.start()

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
        self._remove_slam_geometries()   # also clears any stale FoV indicator (Issue #2)
        self.progress_bar.visible = False
        self._set_showcase_banner("REC Recording - scanning...")

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
        self._remove_fov_geometry()   # FoV indicator is RECORDING/SLAM-only (Issue #2)
        self.progress_bar.visible = False
        self._set_showcase_banner("Processing... loading capture...")
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
            from .slam.config import preferred_device
            try:
                worker = PostProcessWorker.from_capture(
                    path, mesh_every=25, icp_mode="translation",
                    fov_h=self.args.fov_h, fov_v=self.args.fov_v,
                    device=preferred_device())
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
        # Any transient status (IDLE prompt / REC / PROCESSING) means we are NOT
        # in the FINAL reveal -- drop the reveal card and show the text banner.
        self.showcase_banner.text = text
        self._hide_reveal_card()

    def _hide_reveal_card(self):
        self.reveal_card.visible = False

    def _show_reveal_card(self, frames, drift_m, verts, elapsed_s):
        """Render + show the FINAL 'scan complete' reveal card and hide the
        transient text banner. Re-lays-out the window so the card picks up its
        freshly-rendered native size (see `_on_layout`). Never raises into the
        FINAL transition -- a card-render failure must not lose the scan."""
        from . import cards
        try:
            img = cards.render_scan_complete_card(frames, drift_m, verts, elapsed_s)
        except Exception as exc:   # fall back to the plain banner on any render error
            self.bus.publish(f"reveal card render failed: {exc!r}")
            self.showcase_banner.text = (
                f"Scan complete - {int(frames)} frames | drift {drift_m:.2f} m | "
                f"{verts} verts | {elapsed_s:.1f}s")
            self.showcase_banner.visible = True
            return
        h, w = img.shape[:2]
        self.reveal_card.update_image(self._np_to_o3d(img))
        self._reveal_card_size = (w, h)
        self.reveal_card.visible = True
        self.showcase_banner.visible = False
        self.window.set_needs_layout()

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
            self._remove_live_view_geometries()   # owner report: stale cloud must not show through
            self.showcase_banner.visible = True
            self.progress_bar.visible = False
            self._set_showcase_banner("Press Record to scan the room.")
        else:
            if self.follow_camera_enabled:
                # See the matching guard in _on_slam_toggle: leaving Showcase
                # without releasing follow would strand the classic view with
                # mouse nav swallowed and nothing left to drive the camera.
                self.chk_follow_camera.checked = False
                self._on_follow_camera_toggle(False)
            self._join_showcase_workers()
            self.showcase_phase = next_phase(self.showcase_phase, cleared=True)   # -> IDLE
            self._showcase_orbit_enabled = False
            self._showcase_ease = None
            self._remove_slam_geometries()
            self._showcase_last_mesh_obj = None
            self.showcase_banner.visible = False
            self.progress_bar.visible = False
            self._hide_reveal_card()
            self._camera_set = False
            if self._last_item is not None:   # re-render the last frame as a normal cloud
                self._render_frame(self._last_item)
        self.bus.publish(f"Showcase mode -> {'on' if checked else 'off'}")

    def _remove_live_view_geometries(self):
        """Remove the classic point-cloud/surface-mesh view's geometries from
        the scene -- called when ENTERING SLAM/Showcase mode (owner report:
        the previous turbo-colored point cloud + point grid otherwise
        persisted behind the SLAM/Showcase mesh). Leaves the SLAM/
        Showcase-only geometries (`_MESH_GEOM`/`_MESH_WALLS_GEOM`/
        `_SLAM_TRAJ_GEOM`/`_FOV_GEOM`) alone -- those are managed by
        `_upload_slam_mesh`/`_remove_slam_geometries` and populate on the
        very next SLAM/Showcase frame. The IMU gizmo is left alone
        deliberately: it shows the live camera orientation regardless of
        view mode, not a "previous view" artifact."""
        sc = self.scene_widget.scene
        if sc.has_geometry(_GEOM):
            sc.remove_geometry(_GEOM)
        self._remove_mesh_geometry()

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
        if self.follow_camera_enabled:
            # Camera-follow mode owns the view every tick (_apply_follow_camera)
            # -- swallow manual nav instead of letting it mutate the free-orbit
            # state (_cam_az/_cam_radius/pan) behind the scenes, which would
            # otherwise cause a jump the moment follow is toggled back off.
            return res.CONSUMED
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
        if checked:
            self.scene_widget.scene.set_background(theme.BG_CLEAR_DARK, self._bg_grad_dark)
        else:
            self.scene_widget.scene.set_background(theme.BG_CLEAR_LIGHT, self._bg_grad_light)

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
            self._remove_slam_geometries()   # also clears the FoV indicator (Issue #2)
            self._showcase_last_mesh_obj = None
            self.progress_bar.visible = False
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
