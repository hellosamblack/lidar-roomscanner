"""Web-based real-time instrument. One reader thread (the neutral
`reader._run_reader`, shared with the desktop panel) feeds a latest-wins slot; a SINGLE asyncio
broadcast task fans every transformed frame out to all connected WebSocket
clients. This replaces the old per-connection `slot.get_nowait()` loop, whose
competing gets stole frames from one another when two tabs were open (§5.3).

The wire is multiplexed on one `/ws` socket: binary messages (point cloud, IR
image) carry a leading little-endian uint32 tag; JSON text messages carry a
`type` discriminator (metrics/event/log/cmd/state). See the design spec
docs/superpowers/specs/2026-07-15-web-phase1-core-instrument-design.md §6.

Pure, socket-free helpers (classify_bus_line / select_colors /
pack_point_cloud / pack_ir_image / build_metrics_message) are factored out at
module level so the protocol/coloring logic is unit-testable without a server.
"""
from __future__ import annotations

import asyncio
import json
import logging
import math
import os
import queue
import re
import shlex
import shutil
import struct
import subprocess
import sys
import threading
import time
from collections import deque
import webbrowser
import zlib
from contextlib import asynccontextmanager
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from .colors import gray, turbo
from .config import ViewerConfig
from .control import CommandClient, CommandDispatcher
from .decoder import StreamDecoder
from .deproject import Deprojector
from .ir_image import ir_range, reflectance_to_rgb
from .logbus import LogBus
from .magcal import MagCalibration
from .magsweep import (
    MagSweepSession,
    assign_cells,
    build_report as build_magcal_report,
    calibrated_directions,
    calibrated_norms,
    motion_state,
    view_calibration,
)
from .metrics import MetricsRegistry, MetricsSnapshot
from .flatfield import FlatField
from .pipeline import TransformStage
from .reader import _Pacer, _run_reader, follow_camera_target
from .protocol import (
    HEADER_SIZE,
    MAGIC,
    CommandCode,
    FrameHeader,
    FrameType,
    ProtocolError,
    StandbyLevel,
    StreamId,
)
from .sensors import (
    AXIS_CONVENTION,
    SensorState,
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    YawFusion,
    absolute_heading,
    boresight_view_deg,
    graft_yaw,
    gravity_body_from_imu_raw,
    ir_gravity_residual_deg,
    ir_gravity_rot,
    quat_mul,
    quat_pitch_alt_deg,
    quat_pitch_deg,
    quat_roll_alt_deg,
    quat_roll_deg,
    quat_to_matrix,
    quat_yaw_alt_deg,
    quat_yaw_deg,
    tilt_from_down_deg,
    triad_roll_deg,
    wrap180,
)
from .motion import coherence
from .sources import FileSource, Recorder, SerialSource, UdpSource, get_best_source
from .slam.config import DetailedSlamPreset, preferred_device
from .slam.detailed import (build_manifest, estimate_seconds, sidecar_paths,
                            sidecar_status, write_manifest_atomic)
from .slam.meshprep import MeshPrep
from .slam.metrics import write_tum
from .slam.showcase import PostProcessWorker
from .viewer import Stats, resolve_args

log = logging.getLogger("roomscan.web")

# Binary message type tags (first 4 bytes, little-endian uint32).
TAG_POINT_CLOUD = 1
TAG_IR_IMAGE = 2
TAG_MESH = 3               # SLAM reconstruction mesh (web Phase 4)
TAG_SURFACE = 4            # surface-interpolated point cloud (grid + triangles)
TAG_MAGPOSE = 5            # 30 Hz magcal pose/field sample (magcal 3D feedback)

# Broadcast cadences (seconds). Point cloud paces the outer loop at a 30 Hz
# target (owner, 2026-07-16) -- the cap must sit at or above the source rate so
# it never down-samples the stream; a slower source just re-sends the last frame.
# IR and metrics run on their own slower elapsed-time gates off the same task.
POINT_INTERVAL = 1.0 / 30.0
IR_INTERVAL = 1.0 / 15.0
METRICS_INTERVAL = 1.0 / 4.0
# Sensor (streams 9/10) broadcast cadence: 15 Hz is smooth for a handheld gizmo
# and well above the ~4 Hz sparkline need. History rides every message so a
# late-joining tab's sparklines are instantly full (Phase-1 late-joiner rule).
SENSOR_INTERVAL = 1.0 / 15.0
# Rolling window for the orientation jitter stats (roll/pitch/yaw/heading/overall):
# ~5s of `sensor` messages at SENSOR_INTERVAL (15 Hz) => ~75 frame-to-frame samples.
SENSOR_JITTER_WINDOW_S = 5.0
# Magnetometer-sweep (`magcal` modal) TRUTH cadence -- the JSON report: cell
# counts, verdicts, coverage, guidance. 5 Hz (raised from 4 when `cell_dirs`
# stopped riding every message: 4490 B -> 1982 B, so the raise is still ~2x
# cheaper than before). This is data that changes at HUMAN speed; the 30 Hz
# render payload rides the binary MAGPOSE channel instead (docs/web-protocol.md
# "Framing": high-rate render payloads are tagged binary, human-rate state is
# JSON). Sent ONLY to tabs that have the modal open -- `state.magcal_clients`.
MAGCAL_INTERVAL = 1.0 / 5.0
MISSING_PLANE_LOG_INTERVAL = 3.0   # debounce for missing-plane bus lines

# Recording & playback (web Phase 3).
CAPTURES_DIR = "captures"          # where Record writes + the library browses
# SLAM mode (web Phase 4).
RESULTS_DIR = "results"            # where Save writes the full-res map + trajectory
_TRAJ_TAIL_MAX = 256               # trajectory positions shipped in each `slam` message
_VALID_MODES = ("realtime", "slam")  # legacy inbound compatibility
_VALID_SOURCES = ("live", "view")
_VALID_DISPLAYS = ("point_cloud", "slam", "detailed")
_VALID_WALL_MODES = ("solid", "split")
# The playback speed segmented control maps ×0.5/×1/×2/Max onto these fps; a
# capture's own cadence is ~28 Hz, so ×1 (30 fps) plays it near-native and Max
# (0 -> interval 0) drains as fast as it decodes.
_SPEED_BASE_FPS = 30.0

_VALID_COLOR_MODES = ("depth", "reflectance", "confidence")
_VALID_IR_COLORMAPS = ("gray", "turbo")
_VALID_VIEW_COLORMAPS = ("turbo", "gray")
_VALID_SURFACE_MODES = ("grid", "spatial")
_VALID_IDLE_LEVELS = ("soft", "hard")
# Real-time view modes (owner ask, 2026-07-29). Which frame the live cloud is
# shipped in -- see `view_rotation`. Real-time only; SLAM has its own camera.
_VALID_VIEW_MODES = ("world", "fpv", "mirror")

# --- camera framing per view mode (owner ask, 2026-07-30) --------------------
# All three views are described as an offset from ONE baseline: the FPV ground
# truth, i.e. a camera sitting exactly at the sensor looking down its boresight.
# All-zero offsets reproduce that camera exactly, in every mode. `distance_m`
# backs the eye off along the view axis, `height_m` lifts it, and
# `rotation_deg` swings it about the vertical axis through the aim point (which
# stays on the axis, so the subject stays framed).
#
# The modes differ only in what "the axis" is: in fpv/mirror it is the live
# boresight, because the server has already rotated the cloud into the
# boresight view frame (`view_rotation`); in world it is the fixed world
# forward. Those coincide exactly when the board is held in the reference pose,
# which is what makes the FPV baseline a common reference rather than three
# unrelated cameras. Ranges are validated on the wire AND on config load.
_CAM_DISTANCE_RANGE = (0.0, 15.0)      # metres back along the view axis
_CAM_HEIGHT_RANGE = (-5.0, 10.0)       # metres up (negative dips below)
_CAM_ROTATION_RANGE = (-180.0, 180.0)  # degrees; positive swings the eye right
# Auto-orbit is WORLD-ONLY: fpv/mirror are locked to the sensor, so there is no
# scene to circle. Azimuth only -- elevation and distance are untouched, which
# is why the client hands it to OrbitControls' own `autoRotate` rather than
# animating `rotation_deg` (that would also churn the persisted value at 30 Hz).
_ORBIT_SPEED_RANGE = (-60.0, 60.0)     # deg/s; negative reverses


@dataclass
class ViewCam:
    """Camera framing for one view mode, as an offset from the FPV baseline."""
    distance_m: float = 0.0
    height_m: float = 0.0
    rotation_deg: float = 0.0


# Defaults: world is an elevated establishing shot, fpv/mirror sit just off the
# optical centre. A camera exactly AT the centre (all zeros) reproduces the
# depth image's own projection and renders flat -- the small default offset is
# what supplies the parallax that makes depth legible.
_DEFAULT_VIEW_CAM = {
    "world": ViewCam(4.2, 2.6, 0.0),
    "fpv": ViewCam(0.30, 0.20, 0.0),
    "mirror": ViewCam(0.30, 0.20, 0.0),
}

# `[viewer]` keys per mode, in (distance, height, rotation) order. The TOML
# writer is flat/scalar-only, so the nine values are nine plain floats.
_VIEW_CAM_CONFIG_KEYS = {
    m: (f"web_cam_{m}_distance_m", f"web_cam_{m}_height_m", f"web_cam_{m}_rotation_deg")
    for m in _VALID_VIEW_MODES
}
_VIEW_CAM_FIELDS = (("distance_m", _CAM_DISTANCE_RANGE),
                    ("height_m", _CAM_HEIGHT_RANGE),
                    ("rotation_deg", _CAM_ROTATION_RANGE))


def default_view_cam() -> dict[str, ViewCam]:
    """A fresh, independent copy of the per-mode defaults (never share the
    module-level `ViewCam` instances -- they are mutable and would leak edits
    across `UiState`s, which in tests reads as one test corrupting the next)."""
    return {m: replace(c) for m, c in _DEFAULT_VIEW_CAM.items()}

# --- orientation decomposition modes (owner ask, 2026-07-28) -----------------
# Presentation-only: every mode is a different VIEW of the same
# `sensor_state.fused_quat()` -- none of this feeds `display_rotation`, the
# point-cloud rotation path, or SLAM. See `sensors.py`'s
# "Alternate orientation decompositions" section for the per-mode math and
# exactly where each one's gimbal lock sits.
_VALID_ORIENTATION_MODES = ("zyx", "zxy", "boresight", "world")
DEFAULT_AXIS_LABELS = ("Roll", "Pitch", "Yaw")
_MAX_LABEL_LEN = 24

# "Zero yaw here" (owner ask, 2026-07-29): which modes have a free-running,
# SFLP-derived yaw-like slot that a user offset even applies to. "world" is
# deliberately absent -- its yaw slot is `heading_full`, an ABSOLUTE magnetic
# bearing computed independently of the fused quat's own yaw (see
# `absolute_heading`), so grafting it would corrupt its meaning rather than
# just re-zero a drifting reference. `_YAW_GRAFT_SIGN` is the sign to apply
# when CAPTURING an offset from a given mode's current `yaw_deg` (i.e. the
# multiplier `s` such that `graft_yaw(quat, s * yaw_deg)` reports that mode's
# yaw-like slot as ~0 immediately after capture): zyx/zxy report MORE positive
# as `graft_yaw`'s delta grows (so capturing needs the negative), boresight's
# azimuth is a compass bearing with the opposite handedness and reports LESS
# positive for the same delta (so capturing needs the positive) -- verified
# numerically against the actual decompositions, not derived by hand, because
# ZYX/ZXY/boresight are three genuinely different functions of the same
# rotation, not sign-flipped copies of one formula.
_YAW_GRAFT_SIGN = {"zyx": -1.0, "zxy": -1.0, "boresight": 1.0}
# Precedent: `YawFusion.gimbal_margin_deg=15.0` gates the yaw-fusion filter
# near its own gimbal lock; reused verbatim as the "near singularity" warning
# threshold for every decomposition mode below.
ORIENTATION_SINGULARITY_MARGIN_DEG = 15.0
# Precedent: `YawFusion.anomaly_frac=0.3` gates its heading update on the
# calibrated mag magnitude; reused verbatim for the World mode's validity
# indicator (owner ask: "reuse that notion rather than inventing a new one").
WORLD_MODE_MAG_ANOMALY_FRAC = 0.3
# World mode's gravity reference is corrupted by linear acceleration; flag it
# when the stream-11 accel batch's mean norm strays this far from 1 g.
WORLD_MODE_ACCEL_TOL_G = 0.15


def idle_standby_level(name: str) -> int:
    """Map an `idle_level` name to its SET_STANDBY param. Anything but "hard"
    (including an unrecognized value) is treated as soft standby -- the safer,
    instant-resume default."""
    return int(StandbyLevel.HARD if name == "hard" else StandbyLevel.SOFT)

# Success command results look like "OK applied=1" / "REJECTED applied=0":
# a ResultCode name (upper snake) followed by applied=<int>.
_CMD_SUCCESS_RE = re.compile(r"^[A-Z0-9_]+ applied=-?\d+$")
_EVENT_RE = re.compile(r"^\[event\] code=(-?\d+) detail=(-?\d+)(?: (.*))?$")


# --- ui state ----------------------------------------------------------------

@dataclass
class UiState:
    """Server-held view/IR settings, so a late-joining tab is brought current
    the instant it connects (§5.1). All mutations are pure server state -- no
    device round-trip -- so color/IR changes apply regardless of device busy
    state."""
    color_mode: str = "depth"
    ir_colormap: str = "gray"
    ir_freeze: bool = False
    ir_freeze_range: tuple[float, float] | None = None
    # SLAM mode (web Phase 4). `mode` gates the whole SLAM pipeline: the worker
    # is only fed (and only constructed) while mode == "slam", so real-time mode
    # burns no GPU. The three display toggles ride the same one-way `state` echo
    # as color/IR, so a late-joining tab is brought current on connect.
    # `mode` is retained as a short-lived compatibility alias for older MCP
    # clients. The Live/View model is source + display; only `display == slam`
    # arms the latest-wins live runner.
    mode: str = "realtime"
    source: str = "live"              # "live" | "view" (not persisted)
    display: str = "point_cloud"      # "point_cloud" | "slam" | "detailed"
    selected_capture: str | None = None
    slam_trajectory: bool = True
    slam_walls: str = "split"          # "solid" | "split" -> MeshPrep wall_mode
    slam_follow: bool = True
    # Laser-wear auto-idle (SET_STANDBY): when enabled, the server idles the ToF
    # sensor while no tab is connected and wakes it on connect. `idle_level` picks
    # the depth (soft = FSM standby, hard = XSHUT power-down). These ride the same
    # persisted-pref + `state` echo path as the display toggles so a settings UI
    # (or roomscan.toml) can drive them; the debounce delay is config-only.
    view_colormap: str = "turbo"       # 3D viewport colormap: "turbo" | "gray"
    point_size: float = 0.025          # Three.js point size, world units (sizeAttenuation); when
                                       # point_size_auto it is the size at 1 m of RANGE instead
    point_size_auto: bool = True       # scale each point by its range from the sensor, so every
                                       # zone's splat subtends the same solid angle (the zone
                                       # pitch grows as r*dtheta, so one fixed size cannot cover
                                       # a near and a far surface at once)
    surface_enabled: bool = False      # surface interpolation (triangulated mesh)
    surface_mode: str = "grid"         # "grid" (relative depth %) | "spatial" (3D Euclidean)
    surface_threshold_pct: float = 4.0 # grid: max depth gap %; spatial: % of mean depth -> metres
    # Real-time view mode (owner ask, 2026-07-29): "world" orbits a
    # gravity-aligned scene (the original behaviour), "fpv" ships the cloud in
    # the sensor's own frame for a camera locked to the sensor, "mirror" is fpv
    # with X negated so the user can look at themselves. See `view_rotation`.
    view_mode: str = "world"
    # Camera framing per view mode, all referenced to the FPV baseline --
    # see `_DEFAULT_VIEW_CAM`. Keyed by view mode; always all three keys.
    view_cam: dict[str, ViewCam] = field(default_factory=default_view_cam)
    # Slow auto-orbit of the world view (owner ask, 2026-07-30). Azimuth only.
    orbit_enabled: bool = False
    orbit_speed_deg_s: float = 6.0     # 60 s per revolution
    idle_enabled: bool = True
    idle_level: str = "soft"           # "soft" | "hard"
    # Orientation decomposition (owner ask, 2026-07-28): which VIEW of the
    # orientation quaternion the Sensors panel reports, plus user-renamable
    # axis labels applied positionally (slot 1/2/3) whichever mode is active.
    # Presentation-only -- see the `_VALID_ORIENTATION_MODES` comment above.
    orientation_mode: str = "zyx"
    orientation_labels: tuple[str, str, str] = DEFAULT_AXIS_LABELS
    # "Zero yaw here" (owner ask, 2026-07-29): a world-Z `graft_yaw` delta
    # applied to the relative yaw-like slot of zyx/zxy/boresight ONLY, so the
    # Sensors card can read 0 at whatever attitude the user pressed the
    # button. World mode's `yaw_deg` is the absolute magnetic heading and is
    # NEVER offset -- see `_YAW_GRAFT_SIGN` and `build_sensor_message`.
    # Presentation-only, same guarantee as `orientation_mode` above: never
    # touches `fused_quat()`/`display_rotation`/the point-cloud rotation/SLAM.
    yaw_offset_deg: float = 0.0


# --- pure helpers (no socket, no async) -------------------------------------

def classify_bus_line(line: str, command_labels: set[str] | None = None) -> dict | None:
    """Classify one LogBus line into a `cmd`/`event`/`log` JSON dict (§7.1).

    Robustness approach: we do NOT need a live registry of in-flight commands.
    A line is a device event iff it starts with the reader's `[event] ` prefix;
    a command result iff it contains the ` -> ` marker CommandDispatcher always
    emits AND its tail matches one of the four known status shapes (replay /
    busy / TIMEOUT / ERROR / "<ResultCode> applied=<n>"). `command_labels`, when
    supplied, further gates cmd-classification to labels we actually dispatched
    (belt-and-suspenders against a free-text log line that happens to contain
    ` -> `); when None, suffix-matching alone decides. Anything else is a plain
    log line. Returns None only for an empty/None input.
    """
    if not line:
        return None

    m = _EVENT_RE.match(line)
    if m:
        code, detail, msg = m.group(1), m.group(2), m.group(3) or ""
        return {"type": "event", "code": int(code), "detail": int(detail), "msg": msg}
    if line.startswith("[event]"):
        # e.g. "[event] undecodable payload (12 B)" -- not parseable as structured
        return {"type": "log", "line": line}

    if " -> " in line:
        label, _, tail = line.partition(" -> ")
        status = _cmd_status(tail)
        if status is not None and (command_labels is None or label in command_labels):
            return {"type": "cmd", "label": label, "status": status, "detail": tail}

    return {"type": "log", "line": line}


def _cmd_status(tail: str) -> str | None:
    """Map a CommandDispatcher result tail to a status, or None if it doesn't
    look like a command result at all (so the whole line falls back to log)."""
    if tail.endswith("not available in replay"):
        return "error"
    if "busy, command already in flight" in tail:
        return "busy"
    if tail.startswith("TIMEOUT"):
        return "timeout"
    if tail.startswith("ERROR"):
        return "error"
    if _CMD_SUCCESS_RE.match(tail):
        return "ok"
    return None


def _apply_colormap(vn: np.ndarray, colormap: str) -> np.ndarray:
    return gray(vn) if colormap == "gray" else turbo(vn)


def display_rotation(quat) -> np.ndarray | None:
    """Body -> Open3D-CV-world rotation used to gravity-align the live display,
    or None when there is no orientation yet (ToF-only session).

    This is the one composed mapping from `docs/coordinate-frames.md`
    (`T_WORLD_TO_CV @ R @ T_CV_TO_BODY`) — the same matrix the desktop panel
    applied to its orbit-mode cloud (`panel.py:1337`) and the same one shipped to
    the client as the gizmo's `rot`. Never re-derive it locally."""
    if quat is None:
        return None
    return T_WORLD_TO_CV @ quat_to_matrix(*quat) @ T_CV_TO_BODY


# A horizontal (left-right) flip of the live cloud. The ToF/CV frame is
# X=Right, Y=Down, Z=Forward (`docs/coordinate-frames.md`), so negating X alone
# mirrors about the vertical axis -- a selfie flip -- and leaves up/down and
# range untouched. The IR raster shares that index space, which is why the same
# idea shows up client-side as one `scaleX(-1)` on the IR pane.
_MIRROR_X = np.diag([-1.0, 1.0, 1.0])
_WORLD_DOWN_CV = np.array([0.0, 1.0, 0.0])   # Open3D CV world is Y-DOWN
_CV_FORWARD = np.array([0.0, 0.0, 1.0])
_CV_UP = np.array([0.0, -1.0, 0.0])


def boresight_view_frame(grav_rot: np.ndarray) -> np.ndarray:
    """World -> a CV camera frame aimed along the sensor's boresight but *level*.

    Rows are the frame's right/down/forward axes in the gravity-aligned world:
    forward is the boresight, and right is forced perpendicular to world down,
    which is what keeps the horizon flat. Composed with `grav_rot` (below) the
    net effect on the cloud is a pure roll about the boresight -- the same
    un-rolling the IR pane already gets from `ir_gravity_rot`, so the FPV view
    and the IR monitor finally agree on which way is up.
    """
    z = grav_rot @ _CV_FORWARD
    z = z / np.linalg.norm(z)
    x = np.cross(_WORLD_DOWN_CV, z)
    nx = float(np.linalg.norm(x))
    if nx < 1e-6:
        # Aimed straight up or down: "level" is undefined about a vertical
        # boresight, so fall back to the sensor's own up axis. Without this the
        # frame would blow up exactly when a handheld scanner looks at the
        # ceiling or the floor.
        x = np.cross(grav_rot @ _CV_UP, z)
        nx = float(np.linalg.norm(x))
        if nx < 1e-6:                       # unreachable for a real rotation
            return np.eye(3)
    x = x / nx
    return np.stack([x, np.cross(z, x), z])


def view_rotation(grav_rot: np.ndarray | None, view_mode: str) -> np.ndarray | None:
    """The rotation baked into the live cloud for the selected real-time view mode.

    The mode is resolved HERE, server-side, rather than by moving the client's
    camera: the cloud is rotated by the smoothed display quat at the broadcast
    rate, while the client's only orientation feed (`sensor.rot`) is raw and
    half as fast, so a client-rotated camera would lag and slosh against the
    geometry it is supposed to be locked to. Baking the frame in instead makes
    the client camera a *static* pose, matched by construction.

    - "world"  -> `grav_rot`: gravity-aligned, free orbit (the original behaviour)
    - "fpv"    -> look down the boresight, still gravity-levelled (owner: FPV and
                  Mirror "need to respect gravity the same way world does" --
                  the camera follows the sensor's aim, but the scene never rolls)
    - "mirror" -> fpv, then flipped left-right
    """
    if view_mode == "world":
        return grav_rot
    if grav_rot is None:
        # No orientation yet (ToF-only session): the cloud is already in the
        # sensor's frame, which IS the boresight view -- there is simply no
        # gravity to level against. Mirror still mirrors.
        return _MIRROR_X if view_mode == "mirror" else None
    rot = boresight_view_frame(grav_rot) @ grav_rot
    return _MIRROR_X @ rot if view_mode == "mirror" else rot


def rotate_points(pts: np.ndarray, rot: np.ndarray | None) -> np.ndarray:
    """Rotate an (..., 3) array of CV-frame points by `rot`, returning float32.
    A no-op (same array back) when `rot` is None or there are no points, so the
    un-oriented path stays allocation-free."""
    if rot is None or pts.size == 0:
        return pts
    return np.ascontiguousarray(pts @ rot.T, dtype=np.float32)


def rotation_key(rot: np.ndarray | None):
    """Cache key for `rot`: quantized so a stationary sensor keeps hitting the
    cached point-cloud bytes, while real motion invalidates them."""
    return None if rot is None else tuple(np.round(rot.ravel(), 3).tolist())


def quat_slerp(a, b, t: float):
    """Shortest-arc spherical interpolation between unit quats [w,x,y,z]."""
    qa = np.asarray(a, dtype=np.float64)
    qb = np.asarray(b, dtype=np.float64)
    dot = float(qa @ qb)
    if dot < 0.0:            # take the short way round (q and -q are the same rotation)
        qb, dot = -qb, -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:         # nearly parallel: lerp is numerically safer than slerp
        q = qa + t * (qb - qa)
    else:
        theta = math.acos(dot)
        s = math.sin(theta)
        q = (math.sin((1.0 - t) * theta) / s) * qa + (math.sin(t * theta) / s) * qb
    n = float(np.linalg.norm(q))
    return tuple(qa) if n < 1e-12 else tuple(q / n)


def quat_angle_deg(a, b) -> float:
    """Angle in degrees between two unit quats (rotation-aware, sign-agnostic)."""
    dot = abs(float(np.asarray(a, dtype=np.float64) @ np.asarray(b, dtype=np.float64)))
    return float(math.degrees(2.0 * math.acos(min(1.0, max(-1.0, dot)))))


def quat_rotvec(a, b) -> np.ndarray:
    """Rotation vector (axis × angle, in degrees) taking unit quat `a` to `b`."""
    qa = np.asarray(a, dtype=np.float64)
    qb = np.asarray(b, dtype=np.float64)
    if qa @ qb < 0.0:
        qb = -qb                                   # shortest arc
    conj = np.array([qa[0], -qa[1], -qa[2], -qa[3]])
    rel = np.asarray(quat_mul(tuple(conj), tuple(qb)), dtype=np.float64)
    if rel[0] < 0.0:
        rel = -rel
    v = rel[1:]
    n = float(np.linalg.norm(v))
    if n < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(n, float(rel[0]))
    return math.degrees(angle) * (v / n)


class OrientationSmoother:
    """Coherence-gated low-pass on the gravity-alignment quaternion.

    Measured on a *stationary* rig (2026-07-28, live Ethernet stream): the fused
    orientation carries ~0.14 deg mean / 0.25 deg p95 of zero-mean noise per
    update — net rotation 0.14 deg over 15 s against 22.9 deg of summed absolute
    change, i.e. essentially pure jitter. Rotating the cloud by that raw signal
    swings a 3 m lever arm, which is why it reads as shimmer at the cloud edges.

    A magnitude deadband cannot fix this, for the same reason it could not fix
    the SLAM translation jitter (slam/motion.py): 0.25 deg/frame of noise
    overlaps a slow deliberate pan (~20 deg/s = 0.7 deg/frame at 28 fps), so any
    threshold that suppresses the noise also drags on real aiming. The
    discriminator that separates them is directional COHERENCE — real rotation
    accumulates in a consistent direction (-> 1), noise cancels (-> 1/sqrt(N)).
    So this reuses `slam.motion.coherence` on the per-update rotation vectors:
    incoherent history damps by `floor_alpha`, coherent motion passes 1:1, and a
    single large step short-circuits the window so a fast flick never lags.

    Strictly display-only. SLAM feeds off `sensor_state.fused_quat()` directly,
    so a smoothed display quat can never reach the reconstruction — the same
    invariant that made the SLAM stationarity hold safe.
    """

    def __init__(self, window: int = 10, coherence_thresh: float = 0.5,
                 snap_deg: float = 2.0, floor_alpha: float = 0.05):
        self.window = int(window)
        self.coherence_thresh = float(coherence_thresh)
        self.snap_deg = float(snap_deg)
        self.floor_alpha = float(floor_alpha)
        self._held: tuple[float, float, float, float] | None = None
        self._prev_raw: tuple[float, float, float, float] | None = None
        self._hist: deque = deque(maxlen=self.window)

    def _alpha(self, delta_deg: float) -> float:
        # A single large step is motion whatever the history says -- act now.
        if delta_deg >= self.snap_deg:
            return 1.0
        # Never suppress before there's enough evidence to call it jitter.
        if len(self._hist) < self.window:
            return 1.0
        coh = coherence(np.array(self._hist))
        if coh <= self.coherence_thresh:
            return self.floor_alpha
        span = max(1.0 - self.coherence_thresh, 1e-9)
        ramp = (coh - self.coherence_thresh) / span
        return self.floor_alpha + (1.0 - self.floor_alpha) * ramp

    @property
    def held(self):
        """The current smoothed quat (None before the first sample). Read-only
        view for consumers that need the display orientation outside the tick
        that produced it -- e.g. the `sensor` message's `ir_roll_deg`, which must
        agree with the IR pane's snap."""
        return self._held

    def update(self, quat):
        """Feed the newest fused quat; return the smoothed one to display."""
        if quat is None:
            return None
        quat = tuple(float(v) for v in quat)
        if self._held is None:
            # First sample: adopt outright. Easing in from identity would swing
            # the whole scene through a large arc on the first frame.
            self._held = self._prev_raw = quat
            return self._held
        # Always measure increments RAW-to-RAW, never against the held value, or
        # damping would feed back into the motion estimate and lock the gate shut.
        self._hist.append(quat_rotvec(self._prev_raw, quat))
        self._prev_raw = quat
        self._held = quat_slerp(self._held, quat,
                                self._alpha(quat_angle_deg(self._held, quat)))
        return self._held


def _sanitize_axis_labels(labels) -> tuple[str, str, str]:
    """Coerce an inbound labels value to exactly 3 short, non-empty strings,
    falling back to `DEFAULT_AXIS_LABELS` slot-by-slot on anything malformed
    (wrong length/type, empty/whitespace-only entry, absurdly long) -- a
    stray WS message must never leave a blank or runaway-length label stuck
    in the UI (or persisted to disk)."""
    out = list(DEFAULT_AXIS_LABELS)
    if isinstance(labels, (list, tuple)):
        for i, v in enumerate(labels[:3]):
            if isinstance(v, str) and v.strip():
                out[i] = v.strip()[:_MAX_LABEL_LEN]
    return (out[0], out[1], out[2])


def _mag_validity(mag_ut, mag_cal: MagCalibration | None):
    """(valid, reason, measured_ut, expected_ut) for the World mode's heading
    component. `valid` gates on the calibrated+axis-corrected mag magnitude
    matching the fitted field strength within `WORLD_MODE_MAG_ANOMALY_FRAC` --
    the same anomaly notion `YawFusion` already gates its heading update on
    (owner ask: reuse it, don't invent a second threshold). No calibration at
    all is reported as invalid with its own reason, not silently "valid"."""
    if mag_cal is None or mag_ut is None:
        return False, "no magnetometer calibration", None, None
    cal_mag = AXIS_CONVENTION @ mag_cal.apply(mag_ut)
    measured = float(np.linalg.norm(cal_mag))
    expected = float(mag_cal.field_ut)
    ok = abs(measured - expected) <= WORLD_MODE_MAG_ANOMALY_FRAC * expected
    reason = None if ok else (
        f"mag field {measured:.1f} uT vs calibrated {expected:.1f} uT "
        f"(>{WORLD_MODE_MAG_ANOMALY_FRAC:.0%} off -- local magnetic interference?)")
    return ok, reason, measured, expected


def _accel_motion_flag(batch) -> bool | None:
    """True if the stream-11 accel batch's mean norm strays more than
    `WORLD_MODE_ACCEL_TOL_G` from 1 g -- a cheap "is the device accelerating"
    proxy. Linear acceleration is indistinguishable from gravity tilt to a
    3-axis accelerometer, so this is precisely when the World mode's
    gravity-only tilt/roll degrade (that dynamic weakness is the whole reason
    the gyro + complementary filter exist for the primary orientation).
    None when no IMU_RAW batch is available -- motion state is simply
    unknown, not "assumed stationary"."""
    if batch is None or batch.accel_g.shape[0] == 0:
        return None
    norm = float(np.linalg.norm(batch.accel_g.mean(axis=0)))
    return abs(norm - 1.0) > WORLD_MODE_ACCEL_TOL_G


def orientation_view(mode: str, quat, mag_ut_raw=None, heading_full: float | None = None,
                      mag_cal: MagCalibration | None = None, imu_raw_batch=None) -> dict:
    """Decompose `quat` into the selected presentation MODE's three named
    slots (`roll_deg`/`pitch_deg`/`yaw_deg` -- generic positional names; what
    each one physically means depends on `mode`, see `sensors.py`), plus how
    close the CURRENT attitude is to that mode's own singularity.

    Presentation-only: reads `sensor_state.fused_quat()` (passed in as
    `quat`) and never writes anything back -- `display_rotation`/
    `fused_quat()`/the SLAM path are untouched by which mode is selected.

    Returns a dict always shaped {roll_deg, pitch_deg, yaw_deg,
    singularity_margin_deg, near_singularity, valid, reason, ...mode extras}.
    `valid`/`reason` gate the MAG-dependent yaw slot only (World mode); the
    gravity-only roll/pitch slots are reported regardless (their own
    trustworthiness is `near_singularity`, not `valid`)."""
    if quat is None:
        return {"roll_deg": None, "pitch_deg": None, "yaw_deg": None,
                "singularity_margin_deg": None, "near_singularity": False,
                "valid": False, "reason": "no orientation yet"}

    if mode == "zxy":
        roll = quat_roll_alt_deg(quat)
        pitch = quat_pitch_alt_deg(quat)
        yaw = quat_yaw_alt_deg(quat)
        margin = 90.0 - abs(pitch)
        return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
                "singularity_margin_deg": margin,
                "near_singularity": margin < ORIENTATION_SINGULARITY_MARGIN_DEG,
                "valid": True, "reason": None}

    if mode == "boresight":
        azimuth, elevation, roll = boresight_view_deg(quat)
        margin = 90.0 - abs(elevation)
        return {"roll_deg": roll, "pitch_deg": elevation, "yaw_deg": azimuth,
                "singularity_margin_deg": margin,
                "near_singularity": margin < ORIENTATION_SINGULARITY_MARGIN_DEG,
                "valid": True, "reason": None}

    if mode == "world":
        raw_down = gravity_body_from_imu_raw(imu_raw_batch)
        gravity_source = "imu_raw"
        if raw_down is None:
            r = quat_to_matrix(*quat)
            raw_down = tuple((r.T @ np.array([0.0, 0.0, -1.0])).tolist())
            gravity_source = "quat"
        tilt = tilt_from_down_deg(raw_down)
        roll = triad_roll_deg(raw_down)
        heading = heading_full   # reuse the caller's absolute_heading(quat, calibrated_mag)
        mag_valid, mag_reason, mag_norm, mag_expected = _mag_validity(mag_ut_raw, mag_cal)
        motion_flag = _accel_motion_flag(imu_raw_batch)
        reasons = [r for r in (mag_reason,) if r]
        if motion_flag:
            reasons.append("device is accelerating -- gravity tilt reference degraded")
        margin = 90.0 - abs(tilt)
        return {"roll_deg": roll, "pitch_deg": tilt, "yaw_deg": heading,
                "singularity_margin_deg": margin,
                "near_singularity": margin < ORIENTATION_SINGULARITY_MARGIN_DEG,
                "valid": mag_valid and not motion_flag,
                "reason": "; ".join(reasons) if reasons else None,
                "gravity_source": gravity_source,
                "mag_norm_ut": mag_norm, "mag_expected_ut": mag_expected,
                "motion_stable": (None if motion_flag is None else not motion_flag)}

    # default: "zyx" (and any unrecognized value -- fail safe to the original,
    # always-on-by-default decomposition rather than raising)
    roll = quat_roll_deg(quat)
    pitch = quat_pitch_deg(quat)
    yaw = quat_yaw_deg(quat)
    margin = 90.0 - abs(pitch)
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
            "singularity_margin_deg": margin,
            "near_singularity": margin < ORIENTATION_SINGULARITY_MARGIN_DEG,
            "valid": True, "reason": None}


def _unit_quat(q) -> tuple[float, float, float, float]:
    """Re-normalize a [w,x,y,z] quaternion to unit length (float64).

    Device quats are float32 and not always exactly unit-norm. Skipping this
    before a dot product lets `|a . b|` exceed 1.0, get clamped to exactly
    1.0 by the acos guard, and silently report a ZERO angle for the smallest
    real steps -- the bug that produced a bogus zero-jitter reading on
    2026-07-28. Only needed for the dot-product angle; `quat_yaw_deg` /
    `quat_pitch_deg` / `quat_roll_deg` tolerate the float32 slop fine."""
    arr = np.asarray(q, dtype=np.float64)
    n = float(np.linalg.norm(arr))
    if n < 1e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(arr / n)


_JITTER_SIGNALS = ("roll", "pitch", "yaw", "heading", "orientation")


def _jitter_stats(samples: list[dict[str, float]]) -> dict:
    """[{signal: abs frame-to-frame delta_deg}, ...] -> per-signal {mean_deg,
    p95_deg, n}. mean/p95 of the ABSOLUTE step, not the signed value (a jitter
    magnitude, not a drift). p95 is the headline statistic (measured CV 1.45%,
    the most stable of mean/median/p95 on this signal); mean is secondary.
    MEDIAN is deliberately not reported: quantization ties pile up at
    near-zero steps and make it the least stable (CV 4.41%). `n == 0` (no diff computed yet --
    fewer than two raw samples fed) reports None, not 0.0 -- a flat line is not the same claim
    as "no data". A single diff (n=1) is a well-defined mean/p95 of one value."""
    out = {}
    for key in _JITTER_SIGNALS:
        vals = [s[key] for s in samples if key in s]
        if not vals:
            out[key] = {"mean_deg": None, "p95_deg": None, "n": 0}
        else:
            arr = np.asarray(vals, dtype=np.float64)
            out[key] = {
                "mean_deg": float(arr.mean()),
                "p95_deg": float(np.percentile(arr, 95)),
                "n": len(vals),
            }
    return out


class OrientationJitter:
    """Rolling-window frame-to-frame noise stats for the RAW (unsmoothed)
    fused orientation -- roll/pitch/yaw/heading plus the overall angular step
    between consecutive quaternions (the quaternion dot-product angle).

    Computed server-side from FULL-PRECISION internal values, fed once per
    `sensor` message (never from the rounded wire fields -- `rot` is rounded
    to 5dp / ~0.0006 deg, which would censor exactly the noise floor this
    exists to measure). Feeds off `sensor_state.fused_quat()` directly, the
    same pre-`OrientationSmoother` signal SLAM sees -- this reports real
    sensor noise, not the display-smoothed shimmer-suppressed one.

    Angular signals (roll/yaw/heading) diff with `wrap180` so a heading
    crossing 360->0 doesn't register as a 360 deg jump; pitch is bounded
    already but wrapped too for uniformity. The overall-orientation angle
    re-normalizes both quats first (`_unit_quat`) -- see its docstring.

    `orientation` (the quat dot-product angle) and `heading` (magnetic, via
    `absolute_heading`) are convention-INDEPENDENT -- they mean the same thing
    regardless of the selected decomposition mode, so they are always tracked
    and are the trustworthy signals when comparing across a mode switch.
    `roll`/`pitch`/`yaw`, by contrast, are whatever the ACTIVE mode's slots
    report (owner ask, 2026-07-28: "jitter must follow the selected mode") --
    on a mode change those three are reset (history purged, not just the
    running `_prev_*`) so a diff is never taken across two incompatible
    labelings of the same rotation."""

    def __init__(self, window_s: float = SENSOR_JITTER_WINDOW_S):
        self.window_s = float(window_s)
        self._samples: deque[tuple[float, dict[str, float]]] = deque()
        self._prev_quat: tuple[float, float, float, float] | None = None
        self._prev_roll: float | None = None
        self._prev_pitch: float | None = None
        self._prev_yaw: float | None = None
        self._prev_heading: float | None = None
        self._mode: str = "zyx"

    def update(self, quat, heading_deg: float | None, now: float | None = None,
               mode: str = "zyx", roll_deg: float | None = None,
               pitch_deg: float | None = None, yaw_deg: float | None = None) -> dict:
        """Feed the newest RAW quat (+ full-precision heading, or None before
        env/mag data arrives). `roll_deg`/`pitch_deg`/`yaw_deg`, when given,
        are the ACTIVE `mode`'s decomposition of `quat` (from
        `orientation_view`); omitted, they default to the ZYX Tait-Bryan
        values (`quat_roll_deg` etc.) for backward compatibility with direct
        callers that don't care about mode. Returns `{"window_s": ..,
        <signal>: {mean_deg, p95_deg, n}, ...}` for
        roll/pitch/yaw/heading/orientation."""
        now = time.monotonic() if now is None else now
        if mode != self._mode:
            # Mode switch: the numeric MEANING of roll/pitch/yaw just changed,
            # so a frame-to-frame diff spanning the switch is nonsense -- purge
            # only those three signals (orientation/heading are mode-agnostic
            # and stay valid) from both the running "previous value" state and
            # the buffered window.
            self._samples = deque(
                (t, {k: v for k, v in d.items() if k not in ("roll", "pitch", "yaw")})
                for t, d in self._samples)
            self._prev_roll = self._prev_pitch = self._prev_yaw = None
            self._mode = mode
        diffs: dict[str, float] = {}
        if quat is not None:
            quat = tuple(float(v) for v in quat)
            roll = quat_roll_deg(quat) if roll_deg is None else float(roll_deg)
            pitch = quat_pitch_deg(quat) if pitch_deg is None else float(pitch_deg)
            yaw = quat_yaw_deg(quat) if yaw_deg is None else float(yaw_deg)
            if self._prev_roll is not None:
                diffs["roll"] = abs(wrap180(roll - self._prev_roll))
                diffs["pitch"] = abs(wrap180(pitch - self._prev_pitch))
                diffs["yaw"] = abs(wrap180(yaw - self._prev_yaw))
            self._prev_roll, self._prev_pitch, self._prev_yaw = roll, pitch, yaw
            if self._prev_quat is not None:
                diffs["orientation"] = quat_angle_deg(_unit_quat(self._prev_quat), _unit_quat(quat))
            self._prev_quat = quat
        if heading_deg is not None:
            if self._prev_heading is not None:
                diffs["heading"] = abs(wrap180(heading_deg - self._prev_heading))
            self._prev_heading = float(heading_deg)
        if diffs:
            self._samples.append((now, diffs))
        cutoff = now - self.window_s
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        out = _jitter_stats([d for _, d in self._samples])
        out["window_s"] = self.window_s
        return out


def select_colors(outputs: dict, deproj: Deprojector, color_mode: str,
                  colormap: str = "turbo"):
    """Deproject depth and colorize by the selected plane (§7.2).

    Returns (pts, colors, fell_back): pts (N,3) float32 metres, colors (N,3)
    float32 in [0,1], and fell_back True iff the requested non-depth plane was
    missing this frame and depth coloring was substituted. Coloring reuses the
    validity mask (finite, >0, < max_range) + min-max normalize + turbo/gray,
    exactly as the classic viewer. `color_mode == "depth"` colors by deprojected Z.
    """
    depth = outputs["depth"]
    pts = deproj(depth)
    fell_back = False
    if len(pts) == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return empty, empty, False

    if color_mode == "depth":
        vals = pts[:, 2].astype(np.float64, copy=False)
    else:
        plane = outputs.get(color_mode)
        if plane is None:
            fell_back = True
            vals = pts[:, 2].astype(np.float64, copy=False)
        else:
            valid = np.isfinite(depth) & (depth > 0.0) & (depth < deproj.max_range_mm)
            vals = plane[valid].astype(np.float64, copy=False)

    vn = (vals - vals.min()) / max(float(np.ptp(vals)), 1e-6)
    colors = _apply_colormap(vn, colormap)
    return pts.astype(np.float32, copy=False), colors.astype(np.float32, copy=False), fell_back


def pack_point_cloud(pts: np.ndarray, colors: np.ndarray) -> bytes:
    """POINT_CLOUD binary (tag 1): u32 tag=1 · f32[3N] positions · f32[3N]
    colors, all little-endian. Positions then colors, concatenated (§6.1)."""
    pos = np.ascontiguousarray(pts, dtype="<f4").ravel()
    col = np.ascontiguousarray(colors, dtype="<f4").ravel()
    return struct.pack("<I", TAG_POINT_CLOUD) + pos.tobytes() + col.tobytes()


def select_surface(outputs: dict, deproj: Deprojector, color_mode: str,
                   colormap: str = "turbo", surface_mode: str = "grid",
                   threshold_pct: float = 4.0):
    """Grid-structured coloring + triangulation for surface mode.

    Returns (pts_grid, colors_grid, valid, triangles, covered, fell_back):
    pts_grid (h,w,3) f32, colors_grid (h,w,3) f32, valid (h,w) bool,
    triangles (T,3) int64, covered (h*w,) bool, fell_back bool."""
    from .surface import grid_triangles, grid_triangles_3d

    depth = outputs["depth"]
    pts_grid, valid = deproj.grid(depth)
    fell_back = False

    if color_mode == "depth":
        vals = pts_grid[..., 2]
    else:
        plane = outputs.get(color_mode)
        if plane is None:
            fell_back = True
            vals = pts_grid[..., 2]
        else:
            vals = plane.astype(np.float64, copy=False)

    valid_vals = vals[valid]
    if valid_vals.size > 0:
        lo, hi = float(valid_vals.min()), float(valid_vals.max())
    else:
        lo, hi = 0.0, 1.0
    rng = max(hi - lo, 1e-6)
    vn = np.clip((vals - lo) / rng, 0.0, 1.0)
    colors_grid = _apply_colormap(vn, colormap)
    colors_grid[~valid] = 0.0

    if surface_mode == "spatial":
        mean_z = float(np.mean(pts_grid[valid, 2])) if np.any(valid) else 1.0
        threshold_m = max((threshold_pct / 100.0) * mean_z, 1e-6)
        triangles, covered = grid_triangles_3d(pts_grid, valid, threshold_m)
    else:
        triangles, covered = grid_triangles(pts_grid, valid, threshold_pct)

    return (pts_grid.astype(np.float32, copy=False),
            colors_grid.astype(np.float32, copy=False),
            valid, triangles, covered, fell_back)


def pack_surface_cloud(pts_grid: np.ndarray, colors_grid: np.ndarray,
                       valid: np.ndarray, triangles: np.ndarray,
                       covered: np.ndarray) -> bytes:
    """SURFACE binary (tag 4): grid-ordered positions + colors + triangle mesh.

    Layout: u32 tag=4 · u16 w · u16 h · u32 n_tris ·
    f32[3*W*H] positions · f32[3*W*H] colors · u8[W*H] valid ·
    u32[3*T] tri_indices · u8[W*H] covered."""
    h, w = pts_grid.shape[:2]
    n_tris = len(triangles)
    pos = np.ascontiguousarray(pts_grid.reshape(-1, 3), dtype="<f4").ravel()
    col = np.ascontiguousarray(colors_grid.reshape(-1, 3), dtype="<f4").ravel()
    val = np.ascontiguousarray(valid.ravel(), dtype=np.uint8)
    tri = (np.ascontiguousarray(triangles.ravel(), dtype="<u4")
           if n_tris > 0 else np.array([], dtype="<u4"))
    cov = np.ascontiguousarray(covered, dtype=np.uint8)
    header = struct.pack("<IHHI", TAG_SURFACE, w, h, n_tris)
    return (header + pos.tobytes() + col.tobytes() + val.tobytes()
            + tri.tobytes() + cov.tobytes())


def pack_ir_image(rgb: np.ndarray) -> bytes:
    """IR_IMAGE binary (tag 2): u32 tag=2 · u16 width · u16 height ·
    u8[width*height*3] RGB, little-endian. width/height read from the array
    shape (H, W, 3) (§6.1)."""
    arr = np.ascontiguousarray(rgb, dtype=np.uint8)
    height, width = arr.shape[0], arr.shape[1]
    return struct.pack("<IHH", TAG_IR_IMAGE, width, height) + arr.tobytes()


def pack_mesh(packet) -> bytes:
    """MESH binary (tag 3): a SLAM `MeshPacket` (slam/meshprep.py) flattened to
    one self-describing little-endian frame (web Phase 4, docs/web-protocol.md).

    Counts up front so the client allocates once; then non-wall (verts f32·3,
    colors f32·3, tris u32·3), wall (same), floor (verts f32·3, lines u32·2).
    Positions/colors cast f64->f32; indices to u32. `flags` bit0=decimated,
    bit1=walls_split (a split packet carries a non-empty wall submesh)."""
    nw_v = np.ascontiguousarray(packet.non_wall_verts, dtype="<f4").ravel()
    nw_c = np.ascontiguousarray(packet.non_wall_colors, dtype="<f4").ravel()
    nw_t = np.ascontiguousarray(packet.non_wall_tris, dtype="<u4").ravel()
    w_v = np.ascontiguousarray(packet.wall_verts, dtype="<f4").ravel()
    w_c = np.ascontiguousarray(packet.wall_colors, dtype="<f4").ravel()
    w_t = np.ascontiguousarray(packet.wall_tris, dtype="<u4").ravel()
    f_v = np.ascontiguousarray(packet.floor_pts, dtype="<f4").ravel()
    f_l = np.ascontiguousarray(packet.floor_lines, dtype="<u4").ravel()

    flags = (1 if packet.decimated else 0) | (2 if packet.wall_mode == "split" else 0)
    header = struct.pack(
        "<IIIIIIIII", TAG_MESH, int(packet.mesh_seq), flags,
        len(packet.non_wall_verts), len(packet.non_wall_tris),
        len(packet.wall_verts), len(packet.wall_tris),
        len(packet.floor_pts), len(packet.floor_lines))
    return (header + nw_v.tobytes() + nw_c.tobytes() + nw_t.tobytes()
            + w_v.tobytes() + w_c.tobytes() + w_t.tobytes()
            + f_v.tobytes() + f_l.tobytes())


def build_slam_message(step, trajectory, *, frames_integrated: int, mesh_seq: int,
                       source_vertex_count: int) -> dict:
    """FrameStep + trajectory -> `slam` JSON (web Phase 4). Follow-camera
    eye/center/up are computed server-side (reader.follow_camera_target) per the
    web-protocol "server-side math stays server-side" rule -- the browser just
    places its camera. `traj_tail` is downsampled to <=_TRAJ_TAIL_MAX positions
    so the JSON stays small on a long scan; `traj_len` carries the true length."""
    pose = np.asarray(step.pose, dtype=np.float64)
    eye, center, up = follow_camera_target(pose)
    n = len(trajectory)
    if n > _TRAJ_TAIL_MAX:
        idx = np.linspace(0, n - 1, _TRAJ_TAIL_MAX).astype(int)
        tail = [trajectory[i] for i in idx]
    else:
        tail = trajectory
    traj_tail = [[round(float(p[0, 3]), 4), round(float(p[1, 3]), 4), round(float(p[2, 3]), 4)]
                 for p in tail]
    return {
        "type": "slam",
        "pose": [round(float(v), 5) for v in pose.reshape(-1)],   # row-major 16
        "follow": {"eye": [round(float(v), 4) for v in eye],
                   "center": [round(float(v), 4) for v in center],
                   "up": [round(float(v), 4) for v in up]},
        "traj_tail": traj_tail,
        "traj_len": n,
        "fitness": round(float(step.fitness), 4),
        "rmse": round(float(step.rmse), 5),
        "tracking_lost": bool(step.tracking_lost),
        "slam_ms": round(float(step.slam_ms), 2),
        "frames_integrated": int(frames_integrated),
        "mesh_seq": int(mesh_seq),
        "mesh_verts": int(source_vertex_count),
    }


def sanitize_result_name(name, results_dir) -> Path | None:
    """A results filename from the client -> a safe existing path, or None.
    Basename only, `.ply`/`.tum` allow-list, must exist under results_dir
    (same discipline as sanitize_capture_name)."""
    if not name or not isinstance(name, str):
        return None
    base = Path(name).name
    if base != name or Path(name).suffix not in (".ply", ".tum"):
        return None
    p = Path(results_dir) / base
    return p if p.is_file() else None


def list_results(results_dir) -> list[dict]:
    """`results/*.ply` as {name, bytes, mtime}, newest first (the saved-maps
    library; mirrors list_captures)."""
    d = Path(results_dir)
    if not d.is_dir():
        return []
    items = []
    for p in sorted(d.glob("*.ply")):
        try:
            st = p.stat()
        except OSError:
            continue
        items.append({"name": p.name, "bytes": st.st_size, "mtime": round(st.st_mtime, 1)})
    items.sort(key=lambda x: x["mtime"], reverse=True)
    return items


def build_saved_message(results_dir) -> dict:
    return {"type": "saved", "items": list_results(results_dir)}


def build_metrics_message(snapshot: MetricsSnapshot) -> dict:
    """MetricsSnapshot -> `metrics` JSON dict (§6.2). `resources` is null in
    Phase 1 (no ResourceSampler wired); numeric fields go raw over the wire,
    the frontend formats. device_hz/jitter_ms may be None -> JSON null."""
    return {
        "type": "metrics",
        "render_fps": float(snapshot.render_fps),
        "streams": [
            {
                "stream_id": s.stream_id,
                "label": s.label,
                "device_hz": s.device_hz,
                "host_hz": s.host_hz,
                "bytes_per_s": s.bytes_per_s,
                "jitter_ms": s.jitter_ms,
            }
            for s in snapshot.streams
        ],
        "link_bytes_per_s": float(snapshot.link_bytes_per_s),
        "resources": None,
        "drops": snapshot.drops,
        "gaps": snapshot.gaps,
    }


_FUSION_LABELS = {
    "off": "Off",
    "init": "Initializing",
    "active": "Active",
    "gated:no-cal": "No mag calibration",
    "gated:gimbal": "Gimbal lock",
    "gated:motion": "Fast motion",
    "gated:anomaly": "Mag anomaly",
}


def build_sensor_message(sensor_state: SensorState, mag_cal: MagCalibration | None,
                          jitter: OrientationJitter | None = None,
                          orientation_mode: str = "zyx",
                          axis_labels=DEFAULT_AXIS_LABELS,
                          yaw_offset_deg: float = 0.0,
                          ir_display_quat=None) -> dict | None:
    """SensorState -> `sensor` JSON dict (streams 9/10), or None when there is no
    sensor data at all (so the broadcaster stays silent on a ToF-only session).

    The load-bearing math is reused verbatim from the desktop panel: the gizmo
    `rot` is the same display rotation `gizmo_pose` builds
    (T_WORLD_TO_CV @ R @ T_CV_TO_BODY, sensors.py:183-192), and `heading` is
    `absolute_heading` over the calibrated mag (panel.py:3172-3178). Computing
    them here keeps the sign/permutation matrices in exactly one place (Python),
    so the frontend never re-derives them.

    `orientation_raw` and `jitter` are additive fields (owner ask, 2026-07-28):
    the RAW numeric orientation (quat + Euler + heading, un-rounded relative to
    `rot`/`heading` above -- those are rounded for the wire and would censor
    the very noise `jitter` measures) and rolling-window frame-to-frame jitter
    stats (mean_deg/p95_deg/n) for roll/pitch/yaw/heading/orientation. Both are
    computed off `sensor_state.fused_quat()` -- pre-`OrientationSmoother`, the
    same raw signal SLAM sees -- never off the smoothed display quat, so they
    read real sensor noise. `jitter` is None-safe: pass no tracker (e.g. from a
    test) and every signal reports `n=0` / `None` stats rather than raising.

    `orientation_view` (owner ask, 2026-07-28) adds the SELECTED decomposition
    mode's own take on the same quat -- ZYX Tait-Bryan by default (identical
    numbers to `orientation_raw`), or an alternate Euler sequence / boresight
    az-el-roll / gravity+mag "world" reference when `orientation_mode` picks
    one (see `orientation_view()` and `sensors.py`'s "Alternate orientation
    decompositions" section for the math + where each mode's singularity
    sits). `axis_labels` are carried through verbatim for the frontend to use
    instead of hardcoded "Roll"/"Pitch"/"Yaw" strings. This is purely
    presentation: `orientation_raw`/`rot`/`heading` above are computed exactly
    as before, unaffected by `orientation_mode`.

    `yaw_offset_deg` ("Zero yaw here", owner ask, 2026-07-29) is a further
    presentation-only graft applied ONLY to the decomposition fed into
    `orientation_view` -- never to `quat` itself, so `rot`/`heading`/
    `orientation_raw`/`jitter`'s "orientation" signal above are computed from
    the untouched quat exactly as before. It is applied via `graft_yaw`
    (a world-Z rotation that provably preserves roll/pitch/tilt -- see its
    docstring) rather than by adding/subtracting degrees from a Euler
    component, so the decomposition it feeds is still a valid rotation. It is
    skipped entirely for World mode: that mode's `yaw_deg` is the absolute
    magnetic `heading_full` computed above, not a function of this graft, so
    applying it would be a no-op at best and confusing at worst -- the skip
    just documents that explicitly. A constant offset cancels exactly in any
    frame-to-frame diff, so `jitter`'s roll/pitch/yaw magnitudes are
    unaffected by it (see `test_yaw_offset_does_not_change_jitter_magnitude`).
    """
    quat = sensor_state.fused_quat()
    env = sensor_state.latest_env()
    press_spark = sensor_state.pressure_spark_history()
    temp_spark = sensor_state.temp_spark_history()
    if quat is None and env is None and press_spark.size == 0:
        return None

    rot = None
    r = display_rotation(quat)
    if r is not None:
        rot = [round(float(v), 5) for v in r.reshape(-1)]   # row-major 9

    heading = None
    heading_full = None
    mag_out = None
    if env is not None:
        mag = env.mag_ut
        if mag_cal is not None:
            mag = tuple(float(v) for v in AXIS_CONVENTION @ mag_cal.apply(mag))
        mag_out = [round(float(v), 2) for v in mag]
        if quat is not None:
            heading_full = absolute_heading(quat, tuple(mag))
            heading = round(heading_full, 1)

    orientation_raw = {
        "quat": [round(float(v), 6) for v in quat] if quat is not None else None,
        "roll_deg": round(quat_roll_deg(quat), 4) if quat is not None else None,
        "pitch_deg": round(quat_pitch_deg(quat), 4) if quat is not None else None,
        "yaw_deg": round(quat_yaw_deg(quat), 4) if quat is not None else None,
        "heading_deg": round(heading_full, 4) if heading_full is not None else None,
    }

    mode = orientation_mode if orientation_mode in _VALID_ORIENTATION_MODES else "zyx"
    # "Zero yaw here": graft the user's offset onto a DISPLAY-ONLY copy of the
    # quat, never `quat` itself -- World mode is excluded, its yaw slot is the
    # absolute `heading_full` above, not a function of this quat's yaw.
    display_quat = quat
    if quat is not None and mode != "world" and yaw_offset_deg:
        display_quat = graft_yaw(quat, float(yaw_offset_deg))
    view = orientation_view(mode, display_quat,
                             mag_ut_raw=(env.mag_ut if env is not None else None),
                             heading_full=heading_full, mag_cal=mag_cal,
                             imu_raw_batch=sensor_state.latest_imu_raw())
    orientation_view_out = {
        **{k: (round(v, 4) if isinstance(v, float) else v) for k, v in view.items()},
        "mode": mode,
        "labels": list(axis_labels),
        # Echoed for the frontend to show/hide "Clear offset" -- 0.0 (not the
        # raw setting) whenever mode == "world", since it never applied there.
        "yaw_offset_deg": round(float(yaw_offset_deg), 2) if mode != "world" else 0.0,
    }

    jitter_out = (jitter.update(quat, heading_full, mode=mode, roll_deg=view["roll_deg"],
                                 pitch_deg=view["pitch_deg"], yaw_deg=view["yaw_deg"])
                  if jitter is not None
                  else _jitter_stats([]) | {"window_s": SENSOR_JITTER_WINDOW_S})

    raw_status = sensor_state.fusion_status()
    return {
        "type": "sensor",
        "have_quat": quat is not None,
        "rot": rot,
        "heading": heading,
        "pressure_pa": round(float(env.pressure_pa), 1) if env is not None else None,
        "temp_c": round(float(env.temp_c), 2) if env is not None else None,
        "mag_ut": mag_out,
        "fusion": _FUSION_LABELS.get(raw_status, raw_status),
        "fusion_key": raw_status,
        "has_mag_cal": mag_cal is not None,
        "pressure_hist": [round(float(v), 1) for v in press_spark.tolist()],
        "temp_hist": [round(float(v), 2) for v in temp_spark.tolist()],
        "orientation_raw": orientation_raw,
        "orientation_view": orientation_view_out,
        "jitter": jitter_out,
        # Residual in-plane roll the IR pane's server-side 90-deg snap leaves
        # behind, for the client to finish with a CSS transform (see
        # `ir_gravity_residual_deg`). CCW-positive, so CSS must negate it.
        # Computed from the SMOOTHED display quat -- the same one the snap uses --
        # or the two would disagree near a 45-deg boundary and the pane would jump.
        "ir_roll_deg": (None if ir_display_quat is None
                        else round(ir_gravity_residual_deg(ir_display_quat), 2)),
    }


def resolve_command(name: str, param) -> tuple[CommandCode, int, str] | None:
    """Inbound `cmd` request -> (CommandCode, param, label), or None for an
    unknown name. usecase carries the id as both param and label suffix."""
    if name == "ping":
        return CommandCode.PING, 0, "ping"
    if name == "calib":
        return CommandCode.SEND_CALIB, 0, "calib"
    if name == "reinit":
        return CommandCode.REINIT, 0, "reinit"
    if name == "usecase":
        uid = int(param)
        return CommandCode.SET_USECASE, uid, f"usecase {uid}"
    if name == "period":
        return CommandCode.SET_FRAME_PERIOD_US, int(param), f"period {int(param)}"
    if name == "exposure":
        return CommandCode.SET_EXPOSURE_MS, int(param), f"exposure {int(param)}"
    return None


def _state_message(ui: UiState, controller=None, detailed=None) -> dict:
    """Authoritative presentation state.

    ``mode`` is emitted for one release so existing automation remains usable;
    new clients must use the unambiguous Live/View ``source`` + ``display``.
    """
    detail = None
    if controller is not None and controller.mode == "replay" and controller.replay_path:
        try:
            detail = sidecar_status(controller.replay_path, RESULTS_DIR,
                                    getattr(detailed, "preset", None))
        except OSError:
            detail = None
    slam_available = not (controller is not None and controller.mode == "replay" and
                          not controller.index.get("has_stream_9", False))
    return {"type": "state", "color_mode": ui.color_mode,
            "ir_colormap": ui.ir_colormap, "ir_freeze": ui.ir_freeze,
            "view_colormap": ui.view_colormap, "point_size": ui.point_size,
            "point_size_auto": ui.point_size_auto,
            "surface_enabled": ui.surface_enabled,
            "surface_mode": ui.surface_mode,
            "surface_threshold_pct": ui.surface_threshold_pct,
            "view_mode": ui.view_mode,
            "view_cam": {m: asdict(c) for m, c in ui.view_cam.items()},
            "orbit_enabled": ui.orbit_enabled,
            "orbit_speed_deg_s": ui.orbit_speed_deg_s,
            "mode": ui.mode, "source": ui.source, "display": ui.display,
            "selected_capture": ui.selected_capture, "detailed": detail,
            "slam_available": slam_available,
            "slam_trajectory": ui.slam_trajectory,
            "slam_walls": ui.slam_walls, "slam_follow": ui.slam_follow,
            "idle_enabled": ui.idle_enabled, "idle_level": ui.idle_level,
            "orientation_mode": ui.orientation_mode,
            "orientation_labels": list(ui.orientation_labels),
            "yaw_offset_deg": ui.yaw_offset_deg}


# --- settings persistence (Web Phase 5) -------------------------------------
#
# The web UI's display preferences live in the SAME `roomscan.toml` [viewer]
# table the desktop viewer/panel uses, so a single config follows the user
# across both frontends. Only the web-owned prefs below (the six display toggles
# plus the two sensor auto-idle prefs) are written; every other [viewer] field
# (fov/port/near-mode/yaw-fusion/...) is preserved verbatim because we mutate and
# re-save the whole loaded `ViewerConfig`.
#
# `mode` is deliberately NOT persisted/restored: the SLAM worker is armed lazily
# on the first `set_mode slam` (no GPU burned until then), so a server restart
# always comes up in real-time regardless of the last session -- restoring into
# SLAM would silently spin up the GPU on launch. The desktop panel keeps its own
# `mode` in the file; the web app leaves that field untouched.

def ui_from_config(cfg: ViewerConfig) -> UiState:
    """Seed a fresh `UiState` from a loaded `ViewerConfig`, validating each
    field against the web app's allowed values and falling back to the UiState
    default on anything unrecognized. `mode` is not restored (see note above)."""
    ui = UiState()
    if cfg.color in _VALID_COLOR_MODES:
        ui.color_mode = cfg.color
    if cfg.ir_colormap in _VALID_IR_COLORMAPS:
        ui.ir_colormap = cfg.ir_colormap
    ui.ir_freeze = bool(cfg.ir_freeze_range)
    if cfg.view_colormap in _VALID_VIEW_COLORMAPS:
        ui.view_colormap = cfg.view_colormap
    if 0.001 <= float(cfg.web_point_size) <= 1.0:   # same range `set_view` enforces
        ui.point_size = float(cfg.web_point_size)
    ui.point_size_auto = bool(cfg.web_point_size_auto)
    ui.surface_enabled = bool(cfg.surface_enabled)
    if cfg.surface_mode in _VALID_SURFACE_MODES:
        ui.surface_mode = cfg.surface_mode
    ui.surface_threshold_pct = float(cfg.surface_threshold_pct)
    if cfg.web_view_mode in _VALID_VIEW_MODES:
        ui.view_mode = cfg.web_view_mode
    for mode, keys in _VIEW_CAM_CONFIG_KEYS.items():
        cam = ui.view_cam[mode]
        for key, (attr, (lo, hi)) in zip(keys, _VIEW_CAM_FIELDS):
            try:
                v = float(getattr(cfg, key))
            except (TypeError, ValueError):
                continue                      # corrupt value: keep the default
            if lo <= v <= hi:
                setattr(cam, attr, v)
    ui.orbit_enabled = bool(cfg.web_orbit_enabled)
    try:
        speed = float(cfg.web_orbit_speed_deg_s)
    except (TypeError, ValueError):
        speed = None
    if speed is not None and _ORBIT_SPEED_RANGE[0] <= speed <= _ORBIT_SPEED_RANGE[1]:
        ui.orbit_speed_deg_s = speed
    ui.slam_trajectory = bool(cfg.slam_trajectory)
    if cfg.slam_walls in _VALID_WALL_MODES:
        ui.slam_walls = cfg.slam_walls
    ui.slam_follow = bool(cfg.slam_follow)
    ui.idle_enabled = bool(cfg.sensor_idle_enabled)
    if cfg.sensor_idle_level in _VALID_IDLE_LEVELS:
        ui.idle_level = cfg.sensor_idle_level
    if cfg.orientation_mode in _VALID_ORIENTATION_MODES:
        ui.orientation_mode = cfg.orientation_mode
    ui.orientation_labels = _sanitize_axis_labels(cfg.orientation_labels.split(","))
    try:
        ui.yaw_offset_deg = float(cfg.yaw_offset_deg)
    except (TypeError, ValueError):
        pass   # keep the UiState default (0.0) on a corrupt config value
    return ui


def apply_ui_to_config(ui: UiState, cfg: ViewerConfig) -> None:
    """Copy the web-owned prefs from `ui` into `cfg` in place (the six display
    toggles + the two sensor auto-idle prefs; leaving `mode` and every non-web
    field alone), ready to `cfg.save()`."""
    cfg.color = ui.color_mode
    cfg.ir_colormap = ui.ir_colormap
    cfg.ir_freeze_range = bool(ui.ir_freeze)
    cfg.view_colormap = ui.view_colormap
    cfg.web_point_size = float(ui.point_size)
    cfg.web_point_size_auto = bool(ui.point_size_auto)
    cfg.surface_enabled = bool(ui.surface_enabled)
    cfg.surface_mode = ui.surface_mode
    cfg.surface_threshold_pct = float(ui.surface_threshold_pct)
    cfg.web_view_mode = ui.view_mode
    for mode, keys in _VIEW_CAM_CONFIG_KEYS.items():
        cam = ui.view_cam[mode]
        for key, (attr, _range) in zip(keys, _VIEW_CAM_FIELDS):
            setattr(cfg, key, float(getattr(cam, attr)))
    cfg.web_orbit_enabled = bool(ui.orbit_enabled)
    cfg.web_orbit_speed_deg_s = float(ui.orbit_speed_deg_s)
    cfg.slam_trajectory = bool(ui.slam_trajectory)
    cfg.slam_walls = ui.slam_walls
    cfg.slam_follow = bool(ui.slam_follow)
    cfg.sensor_idle_enabled = bool(ui.idle_enabled)
    cfg.sensor_idle_level = ui.idle_level
    cfg.orientation_mode = ui.orientation_mode
    cfg.orientation_labels = ",".join(ui.orientation_labels)
    cfg.yaw_offset_deg = float(ui.yaw_offset_deg)


def _persist_ui(state) -> None:
    """Best-effort write of the current UiState display prefs to roomscan.toml.
    A no-op when no `ViewerConfig` is attached (tests build state directly), and
    a swallowed-with-a-warning failure on any write error (read-only fs, etc.) --
    a viewer must never crash a color click because the config dir is unwritable.

    Re-loads the file first so any non-web field a concurrent editor changed is
    preserved (we only ever own the six display prefs); `ViewerConfig.load`
    tolerates a missing/corrupt file by returning defaults, so this never raises
    on the read side."""
    cfg = getattr(state, "config", None)
    if cfg is None:
        return
    cfg = ViewerConfig.load()
    apply_ui_to_config(state.ui_state, cfg)
    try:
        cfg.save()
    except OSError as exc:
        log.warning("could not persist settings to roomscan.toml: %s", exc)
        return
    state.config = cfg


# --- recording & playback pure helpers (no socket, no thread) ---------------

def speed_to_interval(speed_fps: float) -> float:
    """Playback fps -> per-frame pacer interval; 0 (or <=0) means as-fast-as-decoded."""
    return 1.0 / speed_fps if speed_fps and speed_fps > 0 else 0.0


def sanitize_capture_name(name, captures_dir) -> Path | None:
    """Resolve an inbound capture name to a real file under `captures_dir`, or
    None. Basename-only (no path separators / traversal), must end in `.bin`,
    must exist. The frontend only ever sends names we handed it, but a WS peer
    is untrusted, so this is the load path's whole security surface."""
    if not name or not isinstance(name, str):
        return None
    base = os.path.basename(name)
    if base != name or not base.endswith(".bin"):
        return None
    p = Path(captures_dir) / base
    return p if p.is_file() else None


def sanitize_new_capture_name(name, captures_dir) -> str | None:
    """User-typed rename target (post-recording naming modal) -> a safe basename
    ending in `.bin`, or None if empty/traversal/separator/already-exists.
    `.bin` is appended if the user didn't type it. Does not check the source
    file exists — that is the caller's job (it knows which file it's renaming)."""
    if not isinstance(name, str):
        return None
    stripped = name.strip()
    if not stripped:
        return None
    base = stripped if stripped.endswith(".bin") else stripped + ".bin"
    if os.path.basename(base) != base:
        return None                        # path separators / traversal
    if (Path(captures_dir) / base).exists():
        return None
    return base


_CAPTURE_INFO_CACHE: dict[tuple[str, int, int], dict] = {}


def scan_capture_metadata(path) -> dict:
    """Header-only library scan; avoids loading a multi-hundred-MB capture.

    The selected capture still receives the CRC-verified ``build_capture_index``
    needed for seek. This scan is display metadata only and stops safely at the
    first malformed/truncated frame.
    """
    frames, times, have_imu = 0, [], False
    with open(path, "rb") as f:
        while True:
            raw = f.read(HEADER_SIZE)
            if len(raw) < HEADER_SIZE:
                break
            try:
                hdr = FrameHeader.unpack(raw)
            except ProtocolError:
                break
            if hdr.payload_len < 0:
                break
            f.seek(hdr.payload_len + 4, os.SEEK_CUR)
            if hdr.frame_type != FrameType.DATA:
                continue
            have_imu = have_imu or hdr.stream_id == StreamId.IMU_QUAT
            if hdr.stream_id in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32):
                frames += 1
                times.append(hdr.t_us)
    valid = len(times) >= 2 and times[-1] > times[0] and all(b >= a for a, b in zip(times, times[1:]))
    return {"frames": frames, "has_stream_9": have_imu,
            "duration_s": round((times[-1] - times[0]) / 1e6 if valid else frames / 30.0, 3),
            "timestamped": valid}


def _capture_info(path) -> dict:
    """Cached lightweight metadata for the View library."""
    p = Path(path)
    st = p.stat()
    key = (str(p), st.st_size, st.st_mtime_ns)
    cached = _CAPTURE_INFO_CACHE.get(key)
    if cached is not None:
        return dict(cached)
    out = scan_capture_metadata(p)
    _CAPTURE_INFO_CACHE[key] = out
    return dict(out)


def list_captures(captures_dir) -> list[dict]:
    """View-library entries including SLAM capability and real duration."""
    d = Path(captures_dir)
    if not d.is_dir():
        return []
    items = []
    for p in sorted(d.glob("*.bin")):
        try:
            st = p.stat()
        except OSError:
            continue
        try:
            info = _capture_info(p)
        except (OSError, ProtocolError):
            info = {"frames": 0, "has_stream_9": False, "duration_s": 0.0, "timestamped": False}
        items.append({"name": p.name, "bytes": st.st_size, "mtime": round(st.st_mtime, 1), **info})
    items.sort(key=lambda x: x["mtime"], reverse=True)
    return items


def build_captures_message(captures_dir) -> dict:
    return {"type": "captures", "items": list_captures(captures_dir)}


def build_capture_index(path) -> dict:
    """Linear, CRC-verified scan of a capture's frame boundaries (§3).

    Returns {n_frames, offsets, seqs, calib_spans}: byte offsets + device seqs of
    each DATA depth frame (RAW_3DMD / DEPTH_ZF32), and the byte spans of CALIB
    frames in file order (a seek pre-feeds the governing CALIB so the transform
    stage has calibration). Frames are self-delimiting; the CRC check rejects a
    MAGIC that happens to fall inside a payload."""
    offsets: list[int] = []
    seqs: list[int] = []
    timestamps_us: list[int] = []
    calib_spans: list[tuple[int, int]] = []
    has_stream_9 = False
    with open(path, "rb") as f:
        data = f.read()
    n = len(data)
    i = 0
    while True:
        j = data.find(MAGIC, i)
        if j < 0 or j + HEADER_SIZE > n:
            break
        try:
            hdr = FrameHeader.unpack(data[j:j + HEADER_SIZE])
        except ProtocolError:
            i = j + 1
            continue
        total = HEADER_SIZE + hdr.payload_len + 4
        if j + total > n:
            break                                   # truncated tail frame
        (crc,) = struct.unpack_from("<I", data, j + total - 4)
        if zlib.crc32(data[j:j + total - 4]) != crc:
            i = j + 1                                # false magic inside a payload
            continue
        if hdr.frame_type == FrameType.DATA:
            has_stream_9 = has_stream_9 or hdr.stream_id == StreamId.IMU_QUAT
            if hdr.stream_id == StreamId.CALIB:
                calib_spans.append((j, j + total))
            elif hdr.stream_id in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32):
                offsets.append(j)
                seqs.append(hdr.seq)
                timestamps_us.append(hdr.t_us)
        i = j + total
    return {"n_frames": len(offsets), "offsets": offsets, "seqs": seqs,
            "timestamps_us": timestamps_us, "has_stream_9": has_stream_9,
            "calib_spans": calib_spans}


def build_session_message(mode, source_label, has_live, *, rec_active, rec_path,
                          rec_elapsed_s, rec_bytes, is_replay, capture_name,
                          paused, speed_fps, loop, position, total_frames,
                          rec_last_name=None, elapsed_s=None, duration_s=None,
                          timestamped=False) -> dict:
    """Assemble the `session` message (§4) from primitives (pure, unit-tested)."""
    return {
        "type": "session",
        "mode": mode,
        "source_label": source_label,
        "has_live": has_live,
        "recording": {
            "active": rec_active,
            "path": rec_path,
            "elapsed_s": rec_elapsed_s,
            "bytes": rec_bytes,
            "last_name": rec_last_name,
        },
        "playback": {
            "is_replay": is_replay,
            "capture_name": capture_name,
            "paused": paused,
            "speed_fps": speed_fps,
            "loop": loop,
            "position": position,
            "total_frames": total_frames,
            "elapsed_s": elapsed_s,
            "duration_s": duration_s,
            "timestamped": bool(timestamped),
        },
    }


# --- runtime source-swap + session controller (§2) --------------------------

class _NoCloseSource:
    """Delegating proxy whose `close()` is a no-op, so `pump()`'s
    `finally: source.close()` never closes the persistent live device when the
    reader is swapped to replay. Go Live re-uses the same open source (no UDP
    re-probe / serial re-open). Real teardown calls the underlying close."""

    def __init__(self, inner):
        self._inner = inner

    def read(self) -> bytes:
        return self._inner.read()

    def write(self, data: bytes) -> None:
        self._inner.write(data)

    def close(self) -> None:
        pass


class _PrefixSource:
    """Yields `prefix` bytes once, then delegates to an inner FileSource. Used
    for scrub-seek: the prefix is the governing CALIB frame(s) so the transform
    stage has calibration before the first RAW frame at the seek offset. Carries
    `eof_on_empty` so `pump()` stops at the inner file's EOF (it is not itself a
    FileSource)."""

    eof_on_empty = True

    def __init__(self, prefix: bytes, inner):
        self._prefix = prefix
        self._inner = inner

    def read(self) -> bytes:
        if self._prefix:
            p = self._prefix
            self._prefix = b""
            return p
        return self._inner.read()

    def write(self, data: bytes) -> None:
        pass

    def close(self) -> None:
        self._inner.close()


class SlamRunner:
    """Owns the SLAM compute for the web app (web Phase 4). Wraps the reused
    `make_slam_worker` (local CUDA:0 worker here; remote SlamService if
    configured) + `MeshPrep`, both off-thread, and turns their output into the
    `slam` JSON + MESH binary the broadcaster ships. Feeds/polls run on the
    async broadcaster; enter/leave/reset/save run off the event loop (to_thread)
    under a lock so they never race a poll.

    Lifecycle: `set_active(True)` arms it; the worker+meshprep are built lazily
    on the first `submit()` (which is when the frame width/height are known) so
    real-time mode constructs no Open3D/GPU state. `set_active(False)` and
    `reset()` tear the pipeline down; `reset()` is called on a source-swap so a
    new capture / Go Live starts a fresh map."""

    def __init__(self, *, bus: LogBus, fov_h: float = 55.0, fov_v: float = 42.0):
        self._bus = bus
        self._fov_h = float(fov_h)
        self._fov_v = float(fov_v)
        self._lock = threading.Lock()
        self._active = False
        self._worker = None
        self._meshprep = None
        self._wh = None                 # (width, height) once known
        self._mesh_seq = 0
        self._last_mesh = object()      # identity sentinel; never == a real mesh
        self._last_source_verts = 0

    # ---- lifecycle (inbound thread, via to_thread) --------------------------
    def set_active(self, on: bool) -> None:
        with self._lock:
            if on == self._active:
                return
            self._active = on
            if not on:
                self._teardown_locked()

    def reset(self) -> None:
        """Drop the current map (fresh worker on the next frame). Called on a
        source-swap; safe whether or not SLAM is active."""
        with self._lock:
            self._teardown_locked()

    def _teardown_locked(self) -> None:
        for obj in (self._worker, self._meshprep):
            if obj is not None:
                try:
                    obj.stop()
                except Exception:
                    pass
        self._worker = None
        self._meshprep = None
        self._wh = None
        self._mesh_seq = 0
        self._last_mesh = object()
        self._last_source_verts = 0

    def _build_locked(self, width: int, height: int) -> None:
        # Mirror panel._maybe_start_slam (panel.py:1539): fov from args, device
        # auto (CUDA:0 here), backend picked by make_slam_worker from [slam].
        # MeshPrep budgets from the [slam] view config, same as the desktop.
        from .slam.backend import make_slam_worker
        from .slam.config import SlamConfig, preferred_device
        from .slam.meshprep import MeshPrep
        cfg = SlamConfig.load()
        device = preferred_device()
        worker = make_slam_worker(width, height, fov_h=self._fov_h,
                                  fov_v=self._fov_v, device=device,
                                  release_cache_every=cfg.release_cache_every,
                                  block_count=cfg.block_count,
                                  icp_retry_dist=cfg.icp_retry_dist,
                                  baro_authority=cfg.baro_authority,
                                  baro_tau_frames=cfg.baro_tau_frames)
        worker.start()
        meshprep = MeshPrep(vertex_budget=cfg.live_vertex_budget,
                            fps_budget_ms=cfg.fps_budget_ms)
        meshprep.start()
        self._worker, self._meshprep, self._wh = worker, meshprep, (width, height)
        self._bus.publish(f"[slam] worker started on {device} ({width}x{height})")

    # ---- feed + poll (broadcaster / async task) -----------------------------
    def submit(self, depth, quat, pressure, reflectance=None, confidence=None) -> None:
        """Forward the newest frame to the worker (latest-wins drop). No-op when
        inactive or when there is no orientation prior yet (SLAM needs the quat;
        without it the mapper loses tracking immediately -- see the 07-08
        no-stream-9 capture note in docs/…web-phase4…)."""
        if quat is None:
            return
        with self._lock:
            if not self._active:
                return
            if self._worker is None:
                h, w = np.asarray(depth).shape
                self._build_locked(w, h)
            worker = self._worker
        worker.submit(depth, quat, pressure, reflectance=reflectance, confidence=confidence)

    def poll(self, wall_mode: str) -> tuple[dict | None, bytes | None]:
        """Latest (`slam` message, MESH bytes-or-None). MESH is emitted only when
        the worker published a new mesh (identity check) and MeshPrep has a
        packet ready; the `slam` message ticks every processed frame."""
        with self._lock:
            worker, meshprep = self._worker, self._meshprep
        if worker is None or meshprep is None:
            return None, None
        res = worker.latest()
        if res is None:
            return None, None
        mesh, trajectory, step = res
        if mesh is not None and mesh is not self._last_mesh:
            self._mesh_seq += 1
            meshprep.submit(mesh, mesh_seq=self._mesh_seq, glow_origin=None,
                            wall_mode=wall_mode)
            self._last_mesh = mesh
        mesh_bytes = None
        pkt = meshprep.latest()
        if pkt is not None:
            self._last_source_verts = pkt.source_vertex_count
            mesh_bytes = pack_mesh(pkt)
        frames_integrated = max(0, len(trajectory) - worker.tracking_lost_count)
        msg = build_slam_message(
            step, trajectory, frames_integrated=frames_integrated,
            mesh_seq=self._mesh_seq, source_vertex_count=self._last_source_verts)
        return msg, mesh_bytes

    @property
    def has_map(self) -> bool:
        with self._lock:
            worker = self._worker
        if worker is None:
            return False
        res = worker.latest()
        return bool(res is not None and res[0] is not None)

    # ---- save (inbound thread, via to_thread) -------------------------------
    def save(self, ply_path, tum_path) -> int:
        """Write the full-res map + trajectory. Returns the mesh vertex count;
        raises ValueError on an empty map. Uses the worker's latest published
        mesh (full-res -- MeshPrep decimation is display-only and never touches
        it), so it works identically for the local and remote workers."""
        import open3d as o3d
        from .slam.metrics import write_tum
        with self._lock:
            worker = self._worker
        if worker is None:
            raise ValueError("SLAM is not running")
        res = worker.latest()
        if res is None or res[0] is None:
            raise ValueError("map is empty (no frames integrated yet)")
        mesh, trajectory, _step = res
        legacy = mesh.cpu().to_legacy() if hasattr(mesh, "to_legacy") else mesh
        if len(legacy.vertices) == 0:
            raise ValueError("map is empty (no vertices)")
        Path(ply_path).parent.mkdir(parents=True, exist_ok=True)
        o3d.io.write_triangle_mesh(str(ply_path), legacy)
        # Synthetic monotonic timestamps at the ~28 Hz frame cadence, matching
        # the roomscan-slam CLI's --out-traj.
        ts = [i / 28.0 for i in range(len(trajectory))]
        write_tum(str(tum_path), ts, trajectory)
        return len(legacy.vertices)

    def close(self) -> None:
        with self._lock:
            self._teardown_locked()


class DetailedRunner:
    """Server-owned offline reconstruction job for one immutable capture.

    Unlike ``SlamRunner`` this never receives the broadcaster's latest-wins
    frames: it owns a ``PostProcessWorker`` loaded from the selected capture,
    processes every frame, and exposes the same MESH presentation path.  The
    job intentionally survives a source/display switch; only shutdown stops it.
    """

    def __init__(self, *, bus: LogBus, results_dir=RESULTS_DIR):
        self._bus, self.results_dir = bus, Path(results_dir)
        self._lock = threading.Lock()
        self._worker = None
        self._meshprep = None
        self._capture = None
        self._cached_mesh = None
        self._last_mesh = object()
        self._mesh_seq = 0
        self._committed = False
        self.preset = DetailedSlamPreset.load()

    def start(self, capture, *, force: bool = False) -> dict:
        capture = Path(capture)
        with self._lock:
            if self._worker is not None:
                latest = self._worker.latest()
                if latest is None or not latest.done:
                    return {"started": False, "reason": "another Detailed build is running"}
                # A completed job is only presentation state; release its
                # worker so a manual Regenerate can start immediately.
                for obj in (self._worker, self._meshprep):
                    if obj is not None:
                        obj.stop()
                self._worker = self._meshprep = None
            status = sidecar_status(capture, self.results_dir, self.preset)
            if status["current"] and not force:
                return {"started": False, "reason": "current sidecar exists", "status": status}
            kw = self.preset.mapper_kwargs()
            worker = PostProcessWorker.from_capture(str(capture), mesh_every=self.preset.mesh_every, **kw)
            worker.start()
            prep = MeshPrep(vertex_budget=150000, fps_budget_ms=8.0)
            prep.start()
            self._worker, self._meshprep, self._capture = worker, prep, capture
            self._cached_mesh = None
            self._last_mesh, self._mesh_seq, self._committed = object(), 0, False
        self._bus.publish(f"[detailed] started {capture.name} ({self.preset.fingerprint()})")
        return {"started": True, "estimate": estimate_seconds(
            len(worker.timestamps), self.preset, cuda=preferred_device().startswith("CUDA"))}

    def load_cached(self, capture) -> bool:
        """Load a saved Detailed mesh for immediate View rendering."""
        capture = Path(capture)
        paths = sidecar_paths(capture, self.results_dir)
        if not paths["ply"].is_file():
            return False
        import open3d as o3d
        with self._lock:
            if self._worker is not None:
                return False
            mesh = o3d.t.io.read_triangle_mesh(str(paths["ply"]))
            prep = MeshPrep(vertex_budget=150000, fps_budget_ms=8.0)
            prep.start()
            self._meshprep, self._capture, self._cached_mesh = prep, capture, mesh
            self._last_mesh, self._mesh_seq = object(), 0
        return True

    def status(self) -> dict | None:
        with self._lock:
            worker, capture = self._worker, self._capture
        if worker is None or capture is None:
            return None
        progress = worker.latest()
        if progress is None:
            return {"capture": capture.name, "phase": "frames", "processed": 0,
                    "total": len(worker.timestamps), "done": False}
        return {"capture": capture.name, "phase": "offline_only" if progress.done else "frames",
                "processed": round(progress.fraction * len(worker.timestamps)),
                "total": len(worker.timestamps), "done": progress.done, "stats": progress.stats}

    def poll(self, wall_mode: str) -> tuple[dict | None, bytes | None]:
        with self._lock:
            worker, prep, capture = self._worker, self._meshprep, self._capture
        if prep is None or capture is None:
            return None, None
        if worker is None and self._cached_mesh is not None:
            if self._cached_mesh is not self._last_mesh:
                self._mesh_seq += 1
                prep.submit(self._cached_mesh, mesh_seq=self._mesh_seq, glow_origin=None, wall_mode=wall_mode)
                self._last_mesh = self._cached_mesh
            packet = prep.latest()
            return ({"type": "detailed", "capture": capture.name, "phase": "cached", "done": True,
                     "mesh_seq": self._mesh_seq}, pack_mesh(packet) if packet is not None else None)
        progress = worker.latest()
        if progress is None:
            return self.status(), None
        if progress.mesh is not self._last_mesh:
            self._mesh_seq += 1
            prep.submit(progress.mesh, mesh_seq=self._mesh_seq, glow_origin=None, wall_mode=wall_mode)
            self._last_mesh = progress.mesh
        packet = prep.latest()
        mesh = pack_mesh(packet) if packet is not None else None
        state = self.status() or {}
        state.update({"type": "detailed", "mesh_seq": self._mesh_seq})
        if progress.done and not self._committed:
            self._commit(capture, worker, progress)
            self._committed = True
        return state, mesh

    def _commit(self, capture: Path, worker: PostProcessWorker, progress) -> None:
        if progress.mesh is None:
            self._bus.publish("[detailed] no mesh; sidecar not written")
            return
        import open3d as o3d
        paths = sidecar_paths(capture, self.results_dir)
        self.results_dir.mkdir(parents=True, exist_ok=True)
        # Keep the final suffix on temporaries: Open3D picks the encoder from
        # the extension and refuses an opaque `.tmp` path.
        ply_tmp = paths["ply"].with_name(paths["ply"].stem + ".tmp.ply")
        tum_tmp = paths["tum"].with_name(paths["tum"].stem + ".tmp.tum")
        mesh = progress.mesh.cpu() if hasattr(progress.mesh, "cpu") else progress.mesh
        o3d.t.io.write_triangle_mesh(str(ply_tmp), mesh)
        write_tum(str(tum_tmp), worker.timestamps, progress.trajectory)
        os.replace(ply_tmp, paths["ply"])
        os.replace(tum_tmp, paths["tum"])
        est = estimate_seconds(len(worker.timestamps), self.preset,
                               cuda=preferred_device().startswith("CUDA"))
        manifest = build_manifest(capture, self.preset, stats=progress.stats or {}, estimate=est)
        write_manifest_atomic(paths["manifest"], manifest)  # commit marker is deliberately last
        self._bus.publish(f"[detailed] saved {paths['ply'].name}")

    def close(self) -> None:
        with self._lock:
            worker, prep = self._worker, self._meshprep
            self._worker = self._meshprep = None
            self._cached_mesh = None
        for obj in (worker, prep):
            if obj is not None:
                try:
                    obj.stop()
                except Exception:
                    pass


class SessionController:
    """Owns the reader-thread lifecycle so the source can be swapped live<->replay
    at runtime without disturbing the single broadcaster or the shared `slot`
    (§2). One reader thread runs at a time; swaps stop it, retarget, and respawn
    under a lock. The broadcaster reads `.mode`/`.index` for the `session`
    message; everything mutating runs off the event loop via `asyncio.to_thread`.
    """

    def __init__(self, *, live_source, live_label, stage, stats, slot, fault, bus,
                 client, recorder, pacer, sensor_state, metrics,
                 captures_dir=CAPTURES_DIR, initial_replay_path=None,
                 initial_speed_fps=0.0):
        self._live_underlying = live_source
        self._live_proxy = _NoCloseSource(live_source) if live_source is not None else None
        self.live_label = live_label
        self.has_live = live_source is not None
        self.stage = stage
        self.stats = stats
        self.slot = slot
        self.fault = fault
        self.bus = bus
        self.client = client
        self.recorder = recorder
        self.pacer = pacer
        self.sensor_state = sensor_state
        self.metrics = metrics
        self.captures_dir = str(captures_dir)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._record_started = 0.0
        self._last_recorded_name = None
        self._seek_prefix = b""
        self._seek_offset = 0
        self.loop = False

        if initial_replay_path is not None:
            self.mode = "replay"
            self.replay_path = str(initial_replay_path)
            self.index = build_capture_index(self.replay_path)
            self.speed_fps = float(initial_speed_fps or 0.0)
        else:
            self.mode = "live"
            self.replay_path = None
            self.index = None
            self.speed_fps = 0.0

    # ---- lifecycle ----

    @property
    def source_label(self) -> str:
        if self.mode == "replay" and self.replay_path:
            return f"Replay · {os.path.basename(self.replay_path)}"
        return self.live_label

    def start(self) -> None:
        self.pacer.interval = speed_to_interval(self.speed_fps) if self.mode == "replay" else 0.0
        self._spawn()

    def _spawn(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _open_source(self):
        if self.mode == "live":
            return self._live_proxy
        if self._seek_prefix:
            return _PrefixSource(self._seek_prefix,
                                 FileSource(self.replay_path, start=self._seek_offset))
        return FileSource(self.replay_path, start=self._seek_offset)

    def _run(self) -> None:
        """Reader loop: (re)build decoder+source, run the shared reader body, then
        loop on natural replay EOF or exit on manual stop (§2)."""
        while True:
            decoder = StreamDecoder()
            source = self._open_source()
            client = self.client if self.mode == "live" else None
            _run_reader(
                source, decoder, self.stage, self.stats, self.slot, self.fault,
                self.bus, client, self.recorder, self.pacer, self._stop.is_set,
                state=self.sensor_state, metrics=self.metrics)
            if self._stop.is_set():
                return                                    # manual stop / swap
            if self.mode == "replay" and self.loop:
                self._seek_prefix = b""
                self._seek_offset = 0
                self.bus.publish("replay looping")
                continue
            if self.mode == "replay":
                self.bus.publish("replay finished")
            return                                        # park at EOF

    def _stop_reader(self) -> None:
        self._stop.set()
        self.pacer.paused.clear()                         # unblock a paused reader
        t = self._thread
        if t is not None:
            t.join(timeout=2.0)
        self._thread = None

    # ---- swaps (run under _lock, off the event loop) ----

    def switch_to_replay(self, path) -> None:
        with self._lock:
            self._stop_reader()
            if self.recorder.active:
                self.recorder.stop()                      # never record a replay
            # Drop any stream-11 batch from whatever source was active before
            # (owner ask, 2026-07-28 "World" orientation mode) -- a capture
            # with no stream 11 must fall back to the quat-derived gravity
            # vector, not silently inherit the old source's real one.
            self.sensor_state.clear_imu_raw()
            self.mode = "replay"
            self.replay_path = str(path)
            self.index = build_capture_index(self.replay_path)
            self._seek_prefix = b""
            self._seek_offset = 0
            self.pacer.interval = speed_to_interval(self.speed_fps)
            self.pacer.paused.clear()
            self._spawn()
            self.bus.publish(f"loaded capture {os.path.basename(self.replay_path)}")

    def switch_to_live(self) -> None:
        with self._lock:
            if not self.has_live:
                self.bus.publish("go live -> no device source available")
                return
            self._stop_reader()
            try:                                          # drop stale serial RX bytes
                ser = getattr(self._live_underlying, "_ser", None)
                if ser is not None:
                    ser.reset_input_buffer()
            except Exception:
                pass
            self.sensor_state.clear_imu_raw()              # see switch_to_replay's comment
            self.mode = "live"
            self.replay_path = None
            self.index = None
            self._seek_prefix = b""
            self._seek_offset = 0
            self.pacer.interval = 0.0
            self.pacer.paused.clear()
            self._spawn()
            self.bus.publish("switched to live device")

    def seek(self, frac: float) -> None:
        with self._lock:
            if self.mode != "replay" or not self.index or self.index["n_frames"] == 0:
                return
            self._stop_reader()
            n = self.index["n_frames"]
            i = max(0, min(n - 1, int(round(frac * (n - 1)))))
            off = self.index["offsets"][i]
            prefix = b""
            spans = [s for s in self.index["calib_spans"] if s[0] <= off]
            if spans:
                # Read only the governing CALIB span bytes (not the whole file):
                # a long recording can be hundreds of MB, and we just need the
                # ~2 KB calib blob to seed the transform stage.
                with open(self.replay_path, "rb") as f:
                    parts = []
                    for (s, e) in spans:
                        f.seek(s)
                        parts.append(f.read(e - s))
                prefix = b"".join(parts)
            self._seek_offset = off
            self._seek_prefix = prefix
            self.pacer.paused.clear()
            self._spawn()

    def restart(self) -> None:
        with self._lock:
            if self.mode != "replay":
                return
            self._stop_reader()
            self._seek_prefix = b""
            self._seek_offset = 0
            self.pacer.paused.clear()
            self._spawn()

    # ---- lightweight transport (no reader restart) ----

    def pause(self) -> None:
        self.pacer.paused.set()

    def resume(self) -> None:
        self.pacer.paused.clear()

    def set_speed(self, fps: float) -> None:
        self.speed_fps = float(fps)
        self.pacer.interval = speed_to_interval(self.speed_fps)

    def set_loop(self, on: bool) -> None:
        self.loop = bool(on)

    # ---- recording ----

    def start_record(self) -> None:
        if self.mode != "live":
            self.bus.publish("record -> not available in replay")
            return
        Path(self.captures_dir).mkdir(parents=True, exist_ok=True)
        path = str(Path(self.captures_dir) / f"web_{time.strftime('%Y%m%d_%H%M%S')}.bin")
        self.recorder.start(path)
        self._record_started = time.monotonic()
        self._last_recorded_name = None    # clear the previous take's "just finished" name
        self.bus.publish(f"recording -> {path}")

    def stop_record(self) -> None:
        if not self.recorder.active:
            return
        path = self.recorder.path
        self.recorder.stop()
        self._last_recorded_name = os.path.basename(path) if path else None
        self.bus.publish(f"recording stopped -> {path}")

    def rename_last_recording(self, new_name: str) -> str | None:
        """Rename the just-finished recording (the Web UI's post-stop naming
        modal) to `new_name` under `captures_dir`. Returns the resolved
        basename on success, None if there is nothing to rename or the target
        name is invalid/already taken."""
        old_name = self._last_recorded_name
        if not old_name:
            return None
        new_base = sanitize_new_capture_name(new_name, self.captures_dir)
        if new_base is None:
            return None
        old_path = Path(self.captures_dir) / old_name
        new_path = Path(self.captures_dir) / new_base
        if not old_path.is_file() or new_path.exists():
            return None
        old_path.rename(new_path)
        self._last_recorded_name = new_base
        return new_base

    def close(self) -> None:
        self._stop_reader()
        try:
            self.recorder.close()
        except Exception:
            pass
        if self._live_underlying is not None:
            try:
                self._live_underlying.close()
            except Exception:
                pass

    # ---- session snapshot ----

    def session_message(self, position, now) -> dict:
        rec_active = self.recorder.active
        rec_path = self.recorder.path
        rec_bytes = 0
        rec_elapsed = 0.0
        if rec_active:
            rec_elapsed = max(0.0, now - self._record_started)
            try:
                rec_bytes = os.path.getsize(rec_path) if rec_path else 0
            except OSError:
                rec_bytes = 0
        is_replay = self.mode == "replay"
        total = self.index["n_frames"] if (is_replay and self.index) else 0
        times = self.index.get("timestamps_us", []) if self.index else []
        timestamped = (len(times) >= 2 and times[-1] > times[0] and
                       all(b >= a for a, b in zip(times, times[1:])))
        duration_s = ((times[-1] - times[0]) / 1e6 if timestamped else total / 30.0) if is_replay else None
        elapsed_s = (None if position is None else round(float(position) * (duration_s or 0.0), 3))
        return build_session_message(
            self.mode, self.source_label, self.has_live,
            rec_active=rec_active, rec_path=rec_path,
            rec_elapsed_s=round(rec_elapsed, 1), rec_bytes=rec_bytes,
            rec_last_name=self._last_recorded_name,
            is_replay=is_replay,
            capture_name=(os.path.basename(self.replay_path) if self.replay_path else None),
            paused=self.pacer.paused.is_set(), speed_fps=self.speed_fps, loop=self.loop,
            position=position, total_frames=total, elapsed_s=elapsed_s,
            duration_s=duration_s, timestamped=timestamped)


def _replay_position(ctrl: SessionController, last_item) -> float | None:
    """Current replay progress from TIM2 header time, then sequence fallback."""
    if ctrl is None or ctrl.mode != "replay" or not ctrl.index or last_item is None:
        return None
    times = ctrl.index.get("timestamps_us", [])
    if len(times) >= 2 and times[-1] > times[0] and all(b >= a for a, b in zip(times, times[1:])):
        return max(0.0, min(1.0, (last_item[0].t_us - times[0]) / (times[-1] - times[0])))
    seqs = ctrl.index["seqs"]
    if not seqs:
        return None
    lo, hi = seqs[0], seqs[-1]
    if hi <= lo:
        return 0.0
    seq = last_item[0].seq
    return max(0.0, min(1.0, (seq - lo) / (hi - lo)))


# --- FastAPI app + broadcast hub --------------------------------------------

@asynccontextmanager
async def _lifespan(app: FastAPI):
    # Start the single broadcast task once the server begins serving. app.state
    # is fully populated by main() before uvicorn.run(); guard the (test/import)
    # case where it isn't so a bare `import roomscan.web` never spins a task.
    if getattr(app.state, "ready", False):
        app.state.broadcast_task = asyncio.create_task(_broadcaster())
    yield
    task = getattr(app.state, "broadcast_task", None)
    if task is not None:
        task.cancel()
    detailed = getattr(app.state, "detailed_runner", None)
    if detailed is not None:
        await asyncio.to_thread(detailed.close)


app = FastAPI(lifespan=_lifespan)

static_dir = Path(__file__).parent / "static"
static_dir.mkdir(exist_ok=True)
app.mount("/static", StaticFiles(directory=str(static_dir), html=True), name="static")

# Saved SLAM maps (web Phase 4): served for download from the browser. Created
# lazily so a first Save has somewhere to land; the dir is process-cwd-relative,
# same as CAPTURES_DIR.
_results_dir = Path(RESULTS_DIR)
_results_dir.mkdir(exist_ok=True)
app.mount("/results", StaticFiles(directory=str(_results_dir)), name="results")


@app.get("/", include_in_schema=False)
async def root_redirect() -> RedirectResponse:
    # Temporary, not permanent: browsers cache a 308 indefinitely, which would
    # outlive any future move of the app off /static.
    return RedirectResponse(url="/static/index.html", status_code=307)


# --- admin endpoints: FileHub bridge mode + server restart ------------------
#
# Two owner-facing maintenance actions surfaced as top-bar buttons. Both are
# POST-only so a stray GET (prefetch, crawler, refresh) can never fire them.
#
# DELIBERATELY UNAUTHENTICATED (owner's call, 2026-07-31), and the server binds
# 0.0.0.0 -- so anyone who can reach port 8000 can run the bridge script or
# bounce the process. That is acceptable on this isolated rig LAN; it would not
# be on a shared network. If this ever moves, gate on request.client.host.

REPO_ROOT = Path(__file__).resolve().parents[3]
BRIDGE_SCRIPT = REPO_ROOT / "filehub-bridgemode.sh"
BRIDGE_TIMEOUT_S = 45.0
RESTART_DELAY_S = 2.0


def transport_counters(state) -> dict | None:
    """UDP fragment-level health, or None when the source isn't UDP.

    `gaps` says a frame went missing; these say *why*, which the seq counter
    structurally cannot. `reordered` is the one that used to be indistinguishable
    from loss -- before BUG-042 a reordered datagram destroyed its frame, so it
    showed up as a gap and looked exactly like a dropped packet. Splitting them
    is what makes "did the pacer help?" answerable: pacing changes loss, not
    reordering.

    Reported here rather than on a HUD row because it is a diagnostic surface
    (rig_status / MCP), and the left dock is already at its height budget.
    """
    controller = getattr(state, "controller", None)
    src = getattr(controller, "_live_underlying", None) if controller else None
    if not isinstance(src, UdpSource):
        return None
    return {
        "frames_incomplete": src.frames_incomplete,
        "frags_lost": src.frags_lost,
        "frags_reordered": src.frags_reordered,
        "frags_duplicate": src.frags_duplicate,
        "frags_invalid": src.frags_invalid,
    }


def run_bridge_mode(script: Path = BRIDGE_SCRIPT,
                    timeout_s: float = BRIDGE_TIMEOUT_S) -> dict:
    """Run the FileHub bridge-mode script and report what actually happened.

    Pure-ish helper (no FastAPI types) so it is testable and reusable. Returns
    {ok, returncode, output, error}. Never raises: every failure mode -- missing
    script, missing `expect`, timeout, non-zero exit -- comes back as a dict the
    UI can render, because the whole point of the button is to tell the owner
    whether the router is bridged, not to 500 at them.

    NOTE this is only step 3 of the owner's 4-step recovery (unplug Ethernet ->
    power-cycle the FileHub -> run this -> replug). Running it out of order
    makes the FileHub treat the port as WAN; the UI states the sequence.
    """
    if not script.is_file():
        return {"ok": False, "returncode": None, "output": "",
                "error": f"script not found: {script}"}
    if shutil.which("expect") is None:
        return {"ok": False, "returncode": None, "output": "",
                "error": "'expect' is not installed on this host (apt install expect)"}
    try:
        proc = subprocess.run(
            ["bash", str(script)], cwd=str(REPO_ROOT), capture_output=True,
            text=True, timeout=timeout_s,
        )
    except subprocess.TimeoutExpired:
        return {"ok": False, "returncode": None, "output": "",
                "error": f"script did not finish within {timeout_s:g}s "
                         f"(is the FileHub powered on and reachable?)"}
    except OSError as exc:
        return {"ok": False, "returncode": None, "output": "", "error": str(exc)}
    output = (proc.stdout or "") + (proc.stderr or "")
    return {"ok": proc.returncode == 0, "returncode": proc.returncode,
            "output": output.strip(), "error": None}


def restart_argv() -> list[str]:
    """The command that relaunches this server.

    Rebuilt as `-m roomscan.web` rather than reusing sys.argv[0]: under `python
    -m`, argv[0] is the expanded path to web.py, and re-running THAT executes
    the module as __main__ outside its package, which breaks its relative
    imports. Flags after argv[0] are preserved so a restart keeps --port etc.
    """
    return [sys.executable, "-m", "roomscan.web", *sys.argv[1:]]


def restart_command(delay_s: float = RESTART_DELAY_S) -> list[str]:
    """The detached `sh -c` argv that relaunches the server after `delay_s`.

    Split out from the endpoint so it is testable: exercising /api/restart in a
    test would run the endpoint's os._exit(0) and kill the test runner.
    """
    quoted = " ".join(shlex.quote(a) for a in restart_argv())
    return ["sh", "-c", f"sleep {delay_s:g}; exec {quoted}"]


@app.post("/api/bridge-mode", include_in_schema=False)
async def api_bridge_mode() -> JSONResponse:
    """Put the RavPower FileHub back into transparent-bridge mode."""
    result = await asyncio.to_thread(run_bridge_mode)
    bus = getattr(app.state, "bus", None)
    if bus is not None:
        bus.publish(f"[bridge] {'ok' if result['ok'] else 'FAILED'}: "
                    f"{result['error'] or 'bridge mode applied'}")
    return JSONResponse(result)


@app.post("/api/restart", include_in_schema=False)
async def api_restart() -> JSONResponse:
    """Relaunch the server process.

    Spawns a detached child that waits for this process to release port 8000,
    then execs a fresh server; this process exits once the response is on the
    wire. stdout/stderr are inherited so the new process keeps writing to
    whatever log the old one had (e.g. /tmp/web-live.log).
    """
    argv = restart_argv()
    try:
        subprocess.Popen(
            restart_command(), cwd=str(REPO_ROOT), start_new_session=True,
            stdin=subprocess.DEVNULL,
        )
    except OSError as exc:
        return JSONResponse({"ok": False, "error": str(exc)}, status_code=500)

    async def _exit_soon() -> None:
        # Long enough for the JSON response to flush to the client; the child is
        # already scheduled, so the port is free well before it tries to bind.
        await asyncio.sleep(0.3)
        os._exit(0)

    asyncio.create_task(_exit_soon())
    return JSONResponse({"ok": True, "restart_in_s": RESTART_DELAY_S,
                         "argv": argv})


# --- sensor auto-idle (SET_STANDBY, laser-wear reduction) -------------------
#
# The device streams continuously once a host attaches, firing the VCSEL every
# frame even when nobody is looking. To spare the laser, the server idles it
# whenever the viewer count hits zero and wakes it the instant a tab connects.
# The idle is DEBOUNCED (sensor_idle_delay_s) so a tab reload doesn't thrash the
# sensor FSM. It acts only on a live device we're actually streaming from -- not
# during a replay excursion, where the command-ACK path is the capture file, not
# the device (so a standby command would never be acknowledged). The device only
# ever idles when commanded, so a headless capture.py session (no web server)
# keeps streaming exactly as before.


def _auto_idle_active(state) -> bool:
    """True iff auto-idle should act now: enabled, a live device exists, and the
    controller is streaming from it (mode == "live", not a replay excursion)."""
    ui = getattr(state, "ui_state", None)
    if ui is None or not getattr(ui, "idle_enabled", False):
        return False
    ctrl = getattr(state, "controller", None)
    return ctrl is not None and ctrl.has_live and ctrl.mode == "live"


def _cancel_idle_timer(state) -> None:
    timer = getattr(state, "idle_timer", None)
    if timer is not None:
        timer.cancel()
        state.idle_timer = None


def _dispatch_standby(state, level: int, label: str) -> None:
    """Fire-and-forget a SET_STANDBY at `level` via the shared dispatcher (which
    spawns its own worker thread, so this never blocks the event loop). The
    result/ACK lands on the LogBus like any other command."""
    dispatcher = getattr(state, "dispatcher", None)
    if dispatcher is None:
        return
    state.command_labels.add(label)
    dispatcher.dispatch(int(CommandCode.SET_STANDBY), int(level), label)


async def _viewer_arrived(state) -> None:
    """A tab connected. Cancel any pending idle and, if we had idled the sensor,
    wake it back to streaming. Safe to call on every connect: a no-op unless the
    sensor was actually idled."""
    if not hasattr(state, "sensor_idled"):
        return  # partially-built app.state (unit tests) -- nothing to manage
    _cancel_idle_timer(state)
    if state.sensor_idled:
        _dispatch_standby(state, int(StandbyLevel.ACTIVE), "auto-wake")
        state.sensor_idled = False


async def _viewer_left(state) -> None:
    """A tab disconnected. If it was the last one, arm the debounced idle timer;
    when it fires (and the viewer set is still empty), idle the sensor."""
    if not hasattr(state, "sensor_idled"):
        return
    if state.clients:                       # other tabs still watching
        return
    _cancel_idle_timer(state)
    if not _auto_idle_active(state):
        return

    def _fire() -> None:
        state.idle_timer = None
        # Re-check under the event loop at fire time: a tab may have reconnected
        # during the debounce, or the source swapped to replay.
        if state.clients or not _auto_idle_active(state):
            return
        level = idle_standby_level(state.ui_state.idle_level)
        _dispatch_standby(state, level, f"auto-idle ({state.ui_state.idle_level})")
        state.sensor_idled = True

    loop = asyncio.get_event_loop()
    state.idle_timer = loop.call_later(float(getattr(state, "idle_delay_s", 5.0)), _fire)


async def _drop_client(clients: set, ws: WebSocket) -> None:
    """Remove a client and best-effort close it; never raises. Also arms the
    debounced sensor idle if that was the last viewer (covers tabs that die
    without a clean disconnect, caught by a failed broadcast send)."""
    clients.discard(ws)
    try:
        await ws.close()
    except Exception:
        pass
    await _viewer_left(app.state)


async def _broadcast_bytes(clients: set, data: bytes) -> None:
    for ws in list(clients):
        try:
            await ws.send_bytes(data)
        except Exception:
            await _drop_client(clients, ws)   # one dead tab must not stall the rest (§9)


async def _broadcast_text(clients: set, text: str) -> None:
    for ws in list(clients):
        try:
            await ws.send_text(text)
        except Exception:
            await _drop_client(clients, ws)


async def _broadcast_session(state) -> None:
    """Push a fresh `session` immediately after a state-changing control so every
    tab updates now rather than waiting for the ~4 Hz broadcaster tick. Position
    is left None here (the next tick fills it from the latest frame)."""
    ctrl = getattr(state, "controller", None)
    if ctrl is not None:
        await _broadcast_text(state.clients, json.dumps(ctrl.session_message(None, time.time())))


async def _broadcast_state(state) -> None:
    await _broadcast_text(state.clients, json.dumps(_state_message(
        state.ui_state, getattr(state, "controller", None),
        getattr(state, "detailed_runner", None))))


async def _reset_slam(state) -> None:
    """Drop the SLAM map (off the event loop) after a source-swap, so a new
    capture / Go Live rebuilds a fresh map. No-op if SLAM was never armed."""
    slam = getattr(state, "slam_runner", None)
    if slam is not None:
        await asyncio.to_thread(slam.reset)


# --- magnetometer sweep / calibration modal (owner ask, 2026-07-29) ---------
#
# Additive diagnostic layer: nothing here touches `display_rotation`, the
# point-cloud path, `fused_quat()`, or anything SLAM reads. The ONE thing that
# reaches into the live system is `install_mag_calibration`, and only on an
# explicit user Save -- which is the whole point of the feature. Collecting,
# fitting and previewing are pure observation (guarded by
# `test_magcal_preview_does_not_touch_display_path`).


def _magcal_session(state) -> MagSweepSession:
    """Lazily-created sweep session, same pattern as `orientation_smoother` --
    a hand-built `app.state` (tests) needn't know about it."""
    session = getattr(state, "magcal_session", None)
    if session is None:
        session = state.magcal_session = MagSweepSession()
    return session


def _magcal_clients(state) -> set:
    clients = getattr(state, "magcal_clients", None)
    if clients is None:
        clients = state.magcal_clients = set()
    return clients


def _magcal_report(state, full: bool = False) -> dict:
    """The slow TRUTH channel. `full=True` (only on `open`) adds the two
    deterministic constants the client caches -- `cell_dirs` + `t_world_to_cv`."""
    return build_magcal_report(
        _magcal_session(state), getattr(state, "mag_cal", None),
        view=getattr(state, "magcal_view", "current"),
        saved_path=getattr(state, "mag_cal_path", "mag_cal.json"),
        full=full)


async def _broadcast_magcal(state) -> None:
    """Push a fresh `magcal` to the tabs with the modal open, right after a
    state-changing action rather than waiting for the next 5 Hz tick."""
    clients = _magcal_clients(state)
    if clients:
        await _broadcast_text(clients, json.dumps(_magcal_report(state)))


# --- MAGPOSE (binary tag 5): the fast half of the magcal channel -------------
#
# WHY A SECOND CHANNEL. The `magcal` JSON is 1982 B; at 30 Hz that would be
# 60 kB/s and 30 x JSON.parse per second on the UI thread, to move data (cell
# counts, verdicts, coverage) that changes at HUMAN speed. Split by rate of
# change instead, exactly as the rest of the app does: high-rate render payloads
# are tagged binary, human-rate state is JSON.
#
# `filled_cell` is the trick that keeps the JSON slow: the 30 Hz channel carries
# the DELTA ("this sample just lit cell 47"), so a cell goes solid the instant it
# fills, while the 5 Hz JSON remains the truth that reconciles counts and
# verdicts (and corrects the delta -- `MagSweepSession.sync_occupied`).
#
# Binary rather than a small 30 Hz JSON because orientation must not be measured
# off a rounded decimal (`sensor.rot` is 5 dp); f32 avoids inventing a second
# rounding policy and matches `pack_point_cloud`'s precedent.
MAGPOSE_COLLECTING = 1 << 0
MAGPOSE_STATIONARY = 1 << 1
MAGPOSE_ANOMALY = 1 << 2        # reserved: anomalous-sample detection is Phase 2
MAGPOSE_HAVE_QUAT = 1 << 3
MAGPOSE_PROVISIONAL = 1 << 4    # binning on a provisional/raw estimate, not a real cal
MAGPOSE_REJECTED = 1 << 5

# u32 tag - u32 seq - f32[4] quat(w,x,y,z) - f32[3] field_dir_body -
# f32[3] gravity_body - f32 field_ut - f32 dev_pct - f32 dip_deg -
# i16 live_cell - i16 filled_cell - u16 flags - u16 pad   => 68 bytes.
_MAGPOSE = struct.Struct("<II13fhhHH")
MAGPOSE_SIZE = _MAGPOSE.size


def pack_magpose(seq: int, quat, field_dir, gravity, field_ut: float,
                 dev_pct: float, dip_deg: float, live_cell: int,
                 filled_cell: int, flags: int) -> bytes:
    """Pack one MAGPOSE frame. Pure; every value already resolved by
    `build_magpose`. `live_cell`/`filled_cell` use -1 for "none"."""
    return _MAGPOSE.pack(
        TAG_MAGPOSE, int(seq) & 0xFFFFFFFF,
        float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]),
        float(field_dir[0]), float(field_dir[1]), float(field_dir[2]),
        float(gravity[0]), float(gravity[1]), float(gravity[2]),
        float(field_ut), float(dev_pct), float(dip_deg),
        int(live_cell), int(filled_cell), int(flags) & 0xFFFF, 0)


def build_magpose(session: MagSweepSession, current, sensor_state, seq: int,
                  view: str = "current", stored: bool = False) -> bytes | None:
    """One 30 Hz pose+field sample for the open magcal modals, or None when
    there is no magnetometer sample to report (a ToF-only source, or a replay
    with no stream 10 -- the modal then says *nothing is arriving* rather than
    animating a convincingly empty sphere).

    ISOLATION: reads `fused_quat()` / `latest_imu_raw()` and writes nothing to
    them. The only mutation is `session.mark_occupied`, which is the magcal
    session's own state. Guarded by
    `test_magcal_preview_does_not_touch_display_path`."""
    raw = session.live_raw
    if raw is None:
        return None
    bin_cal = session.binning_calibration(current)
    dirs = calibrated_directions([raw], bin_cal)
    if dirs.shape[0] == 0:
        return None
    d = dirs[0]
    live_cell = int(assign_cells(dirs, session.n)[0])

    view_cal, _ = view_calibration(session, current, view)
    if view_cal is not None:
        field_ut = float(calibrated_norms([raw], view_cal)[0])
        expected = float(view_cal.field_ut)
        dev_pct = 100.0 * (field_ut - expected) / expected if expected > 1e-9 else math.nan
    else:
        # No calibration at all: |B| of the raw vector is a meaningless number
        # to plot against a ramp, so say "unknown" rather than draw a colour.
        field_ut = float(np.linalg.norm(np.asarray(raw, dtype=np.float64)))
        dev_pct = math.nan

    quat = sensor_state.fused_quat() if sensor_state is not None else None
    have_quat = quat is not None
    if not have_quat:
        quat = (1.0, 0.0, 0.0, 0.0)

    # Gravity: stream-11 SFLP gravity when the device streams it (~16x finer in
    # tilt than the fp16 quaternion), else the quat-derived down vector -- the
    # same order `build_sensor_message` already uses. Do not re-derive.
    gravity = None
    if sensor_state is not None:
        gravity = gravity_body_from_imu_raw(sensor_state.latest_imu_raw())
    if gravity is None and have_quat:
        gravity = tuple(float(v) for v in
                        (quat_to_matrix(*quat).T @ np.array([0.0, 0.0, -1.0])))
    if gravity is None:
        gravity = (0.0, 0.0, 0.0)
        dip_deg = math.nan
    else:
        # The dip arc: angle(B, g). For a CORRECT calibration this is a constant
        # of the location (90 deg + magnetic dip) -- both vectors are fixed in
        # the world, so their mutual angle cannot depend on attitude. It is also
        # immune to scale error, so it catches the soft-iron / axis-misalignment
        # faults a self-consistent-but-wrong-magnitude calibration sails past.
        dip_deg = math.degrees(math.acos(
            float(np.clip(np.dot(d, np.asarray(gravity, dtype=np.float64)), -1.0, 1.0))))

    filled = live_cell if (stored and session.mark_occupied(live_cell)) else -1

    flags = 0
    if session.collecting:
        flags |= MAGPOSE_COLLECTING
    if have_quat:
        flags |= MAGPOSE_HAVE_QUAT
    if bin_cal is None or (session.candidate is None and current is None):
        flags |= MAGPOSE_PROVISIONAL
    if session.last_rejected:
        flags |= MAGPOSE_REJECTED
    if motion_state(session.recent, bin_cal)["stationary"]:
        flags |= MAGPOSE_STATIONARY

    return pack_magpose(seq, quat, d, gravity, field_ut, dev_pct, dip_deg,
                        live_cell, filled, flags)


def install_mag_calibration(state, cal: MagCalibration) -> None:
    """Adopt a newly-saved calibration WITHOUT a server restart.

    Three consumers hold a reference to the calibration and all three are
    updated here, which is what makes hot-reload real rather than partial:
      * `state.mag_cal` -- read every tick by `build_sensor_message` /
        `_mag_validity` / `orientation_view`'s World mode (heading, mag-validity
        gate, the mag readout);
      * the `YawFusion` filter's own `cal` -- it captured the object at
        construction, so reassigning `state.mag_cal` alone would leave the
        FUSED heading running on the old calibration indefinitely;
      * the accumulated yaw offset, cleared via `reset_fusion()` so the filter
        re-snaps to the new calibration's heading instead of low-passing from a
        delta computed under the old one over the next ~20 s time constant.

    Deliberately does NOT touch `display_rotation`/the point cloud/SLAM: those
    consume `fused_quat()`'s tilt, which is SFLP gravity, not magnetic."""
    state.mag_cal = cal
    fusion = getattr(state, "fusion", None)
    if fusion is not None:
        fusion.cal = cal
    sensor_state = getattr(state, "sensor_state", None)
    if sensor_state is not None:
        sensor_state.reset_fusion()


def _log_debounced(state, bus: LogBus, key: str, message: str) -> None:
    """Publish `message` at most once per MISSING_PLANE_LOG_INTERVAL for a given
    key, so a persistently-missing plane doesn't spam the log (§7.2/§7.3)."""
    now = time.monotonic()
    last = state.debounce.get(key, 0.0)
    if now - last >= MISSING_PLANE_LOG_INTERVAL:
        state.debounce[key] = now
        bus.publish(message)


async def _broadcaster() -> None:
    """The single fan-out task (§5.3). Started once on startup; runs for the
    process lifetime. Exactly one reader of `slot`, so every client sees the
    same frames no matter how many tabs are open."""
    state = app.state
    clients: set = state.clients
    bus: LogBus = state.bus
    metrics: MetricsRegistry = state.metrics
    ui: UiState = state.ui_state

    bus_handle = bus.subscribe()
    last_item = None          # (header, outputs); kept so IR/metrics tick when slot is idle
    last_pc_key = None        # (seq, color_mode) -> cached packed point cloud
    last_pc_bytes = None
    # Display-only de-jitter of the gravity alignment. Lazily created like
    # `state.deproj`, so a hand-built state (tests) doesn't have to know about it.
    smoother = getattr(state, "orientation_smoother", None)
    if smoother is None:
        smoother = state.orientation_smoother = OrientationSmoother()
    # Frame-to-frame jitter stats (owner ask, 2026-07-28) -- same lazy-create
    # pattern as the smoother above, off the RAW quat (fused_quat(), never the
    # smoothed display one).
    jitter = getattr(state, "orientation_jitter", None)
    if jitter is None:
        jitter = state.orientation_jitter = OrientationJitter()
    last_ir = 0.0
    last_metrics = 0.0
    last_sensor = 0.0
    last_magcal = 0.0
    magpose_seq = 0
    next_pc = time.monotonic()   # deadline-based pacing: sleep to the next tick,

    while True:
        # not for a fixed interval AFTER the work -- otherwise the true period is
        # POINT_INTERVAL + work_time and the stream never reaches the target rate.
        delay = next_pc - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        now = time.monotonic()
        next_pc += POINT_INTERVAL
        if next_pc <= now:       # a slow tick overran the interval: resync, don't burst-catch-up
            next_pc = now + POINT_INTERVAL

        # Latest-wins pull; a fresh frame ticks the device-fps render counter.
        try:
            item = state.slot.get_nowait()
            last_item = item
            metrics.tick_render(now)
        except queue.Empty:
            pass

        # Reader fault: surface once, flip the fault flag, keep serving.
        if state.fault and not state.fault_reported:
            state.fault_reported = True
            err = state.fault.get("error")
            print(f"\n[FATAL] reader thread stopped: {err!r}", file=sys.stderr, flush=True)
            bus.publish(f"reader stopped: {err!r}")

        if last_item is not None:
            header, outputs = last_item
            depth = outputs["depth"]
            h, w = depth.shape
            if state.deproj is None:
                state.deproj = Deprojector(w, h, state.args.fov_h, state.args.fov_v)

            # Gravity-align the live display with the fused orientation, so the
            # scene reads upright however the board is held (desktop-panel orbit
            # parity, panel.py:1332-1337). Raw fused quat, no yaw baseline --
            # gravity is absolute. None on a ToF-only session => sensor frame.
            # Smoothed display-side only: the raw quat's ~0.14 deg/update noise
            # becomes visible shimmer once it swings a 3 m lever arm.
            grav_quat = smoother.update(state.sensor_state.fused_quat())
            grav_rot = display_rotation(grav_quat)
            # ...unless the user picked FPV/Mirror, which ship the cloud in the
            # sensor's own frame instead. `grav_quat`/`grav_rot` themselves stay
            # gravity-true below: the IR pane's quarter-turn snap and the
            # `sensor` message are deliberately unaffected by the view mode
            # (owner: "IR view is unchanged for all but mirror", and Mirror's IR
            # flip is a client-side CSS transform, not a re-rotation).
            view_rot = view_rotation(grav_rot, ui.view_mode)

            # POINT_CLOUD (or SURFACE) every tick (so late joiners see data
            # within ~36ms), but only in real-time mode -- SLAM mode replaces
            # the cloud with the reconstructed mesh.  Cache the packed bytes;
            # rebuild only when the frame, orientation, color mode, colormap, or
            # surface settings changed.
            if ui.display == "point_cloud":
                surf_key = ((ui.surface_enabled, ui.surface_mode,
                             ui.surface_threshold_pct) if ui.surface_enabled
                            else None)
                key = (header.seq, ui.color_mode, ui.view_colormap, surf_key,
                       rotation_key(view_rot))
                if key != last_pc_key:
                    if ui.surface_enabled:
                        pg, cg, val, tris, cov, fell_back = select_surface(
                            outputs, state.deproj, ui.color_mode,
                            ui.view_colormap, ui.surface_mode,
                            ui.surface_threshold_pct)
                        last_pc_bytes = pack_surface_cloud(
                            rotate_points(pg, view_rot), cg, val, tris, cov)
                    else:
                        pts, colors, fell_back = select_colors(
                            outputs, state.deproj, ui.color_mode, ui.view_colormap)
                        last_pc_bytes = pack_point_cloud(
                            rotate_points(pts, view_rot), colors)
                    if fell_back:
                        _log_debounced(state, bus, f"color-miss:{ui.color_mode}",
                                       f"color mode {ui.color_mode!r} unavailable this frame, showing depth")
                    last_pc_key = key
                if last_pc_bytes is not None:
                    await _broadcast_bytes(clients, last_pc_bytes)

            # SLAM mode (web Phase 4): feed the newest frame to the worker and
            # ship the latest `slam` message + (throttled) MESH. The feed/poll
            # touch off-thread workers; nothing blocks the event loop here.
            slam = getattr(state, "slam_runner", None)
            if ui.display == "slam" and slam is not None:
                quat = state.sensor_state.fused_quat()
                env = state.sensor_state.latest_env()
                pressure = env.pressure_pa if env is not None else None
                slam.submit(depth, quat, pressure,
                            reflectance=outputs.get("reflectance"),
                            confidence=outputs.get("confidence"))
                smsg, mesh_bytes = slam.poll(ui.slam_walls)
                if mesh_bytes is not None:
                    await _broadcast_bytes(clients, mesh_bytes)
                if smsg is not None:
                    await _broadcast_text(clients, json.dumps(smsg))

            # Detailed uses an offline worker, but publishes its progressive
            # mesh through the same MESH channel. It deliberately does not
            # consume the replay reader's latest frame here.
            detailed = getattr(state, "detailed_runner", None)
            if ui.display == "detailed" and detailed is not None:
                dmsg, mesh_bytes = detailed.poll(ui.slam_walls)
                if mesh_bytes is not None:
                    await _broadcast_bytes(clients, mesh_bytes)
                if dmsg is not None:
                    await _broadcast_text(clients, json.dumps(dmsg))

            # IR_IMAGE on its own slower cadence.
            if now - last_ir >= IR_INTERVAL:
                last_ir = now
                refl = outputs.get("reflectance")
                if refl is not None:
                    if ui.ir_freeze:
                        if ui.ir_freeze_range is None:      # capture on the first frozen tick
                            ui.ir_freeze_range = ir_range(refl)
                        vmin, vmax = ui.ir_freeze_range
                    else:
                        vmin = vmax = None
                    rgb = reflectance_to_rgb(refl, colormap=ui.ir_colormap,
                                             vmin=vmin, vmax=vmax, upscale=1)
                    # Roll the pane to the nearest 90 deg so its "down" is
                    # physical down, matching the gravity-aligned cloud. Same
                    # smoothed quat, so the pane can't flap between turns when
                    # the sensor sits near a 45 deg snap boundary.
                    steps = ir_gravity_rot(grav_quat) if grav_quat is not None else 0
                    if steps:
                        rgb = np.rot90(rgb, steps)
                    await _broadcast_bytes(clients, pack_ir_image(rgb))
                else:
                    _log_debounced(state, bus, "ir-miss",
                                   "reflectance unavailable this frame, holding IR pane")

        # Sensor (streams 9/10) on its own cadence; silent until 9/10 arrives.
        if now - last_sensor >= SENSOR_INTERVAL:
            last_sensor = now
            smsg = build_sensor_message(state.sensor_state, state.mag_cal, jitter,
                                         orientation_mode=state.ui_state.orientation_mode,
                                         axis_labels=state.ui_state.orientation_labels,
                                         yaw_offset_deg=state.ui_state.yaw_offset_deg,
                                         ir_display_quat=smoother.held)
            if smsg is not None:
                await _broadcast_text(clients, json.dumps(smsg))

        # Magnetometer sweep (owner ask, 2026-07-29). Feed at the full loop
        # rate so a tumble is sampled as densely as the env stream allows, but
        # only while a tab has the modal open or a collection is running -- a
        # normal session does no work here at all. The feed POLLS
        # `latest_env()` (de-duplicated on `t_us`) rather than tapping the
        # reader thread, so the shared sensor path is untouched; at 30 Hz loop
        # vs ~28 Hz env that captures effectively every sample.
        mag_session = getattr(state, "magcal_session", None)
        magcal_clients = getattr(state, "magcal_clients", None) or set()
        if mag_session is not None and (magcal_clients or mag_session.collecting):
            env = state.sensor_state.latest_env()
            stored = False
            if env is not None:
                stored = mag_session.add(env.mag_ut, env.t_us)
            if magcal_clients:
                # MAGPOSE every tick (30 Hz): the render payload. `stored` gates
                # the newly-filled-cell delta so a de-duplicated env sample can
                # never claim to have filled anything.
                pose = build_magpose(mag_session, getattr(state, "mag_cal", None),
                                     state.sensor_state, magpose_seq,
                                     view=getattr(state, "magcal_view", "current"),
                                     stored=stored)
                if pose is not None:
                    magpose_seq += 1
                    await _broadcast_bytes(magcal_clients, pose)
            if magcal_clients and now - last_magcal >= MAGCAL_INTERVAL:
                last_magcal = now
                await _broadcast_text(magcal_clients, json.dumps(_magcal_report(state)))

        # Metrics + session + bus drain on the slowest cadence.
        if now - last_metrics >= METRICS_INTERVAL:
            last_metrics = now
            snap = metrics.snapshot(now)
            # MetricsRegistry knows nothing about frame sequencing, so drops/gaps
            # come from the reader's Stats (panel.py did this; the web path never
            # did, leaving the HUD's Drops/Gaps rows pinned at the dataclass
            # default 0). That is not a cosmetic gap: a UDP fragment loss makes
            # the host discard the whole frame (sources.py reassembly), which
            # shows up ONLY as a header seq gap -- so without this the web UI
            # cannot see transport loss at all.
            stats = getattr(state, "stats", None)
            if stats is not None:
                snap = replace(snap, drops=stats.dropped_flags, gaps=stats.seq_gaps)
            msg = build_metrics_message(snap)
            msg["transport"] = transport_counters(state)
            await _broadcast_text(clients, json.dumps(msg))
            ctrl = getattr(state, "controller", None)
            if ctrl is not None:
                pos = _replay_position(ctrl, last_item)
                await _broadcast_text(clients, json.dumps(ctrl.session_message(pos, time.time())))
            for line in bus.drain(bus_handle):
                msg = classify_bus_line(line, state.command_labels)
                if msg is not None:
                    await _broadcast_text(clients, json.dumps(msg))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    state = app.state
    clients: set = state.clients
    clients.add(websocket)
    await _viewer_arrived(state)   # cancel any pending idle; wake the sensor if idled

    # Bring the new tab current immediately.
    try:
        await websocket.send_text(json.dumps(_state_message(
            state.ui_state, getattr(state, "controller", None),
            getattr(state, "detailed_runner", None))))
        ctrl = getattr(state, "controller", None)
        if ctrl is not None:
            await websocket.send_text(json.dumps(ctrl.session_message(None, time.time())))
            await websocket.send_text(json.dumps(build_captures_message(ctrl.captures_dir)))
        await websocket.send_text(json.dumps(build_saved_message(RESULTS_DIR)))
    except Exception:
        await _drop_client(clients, websocket)
        return

    try:
        while True:
            data = await websocket.receive_text()
            try:
                await _handle_inbound(state, json.loads(data), websocket)
            except Exception as exc:  # a malformed inbound message must never kill the loop (§9)
                log.warning("bad inbound ws message: %r (%s)", data[:200], exc)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("ws receive loop error: %r", exc)
    finally:
        clients.discard(websocket)
        _magcal_clients(state).discard(websocket)   # stop paying for a closed modal
        await _viewer_left(state)   # arm the debounced sensor idle if that was the last tab


async def _handle_inbound(state, msg: dict, ws=None) -> None:
    """Route one decoded inbound JSON message by `type` (§5.4).

    `ws` is the originating socket, needed only by `magcal` (whose report
    stream is per-tab, not a global broadcast). Optional so every existing
    caller/test that routes a message without one keeps working."""
    mtype = msg.get("type")
    ui: UiState = state.ui_state

    ctrl = getattr(state, "controller", None)

    if mtype == "cmd":
        resolved = resolve_command(msg.get("name"), msg.get("param", 0))
        if resolved is None:
            log.warning("unknown/invalid cmd request: %r", msg)
            return
        code, param, label = resolved
        state.command_labels.add(label)
        # In replay there is no device; report it the same way the dispatcher
        # would (classified `error` -> toast) instead of a real round-trip.
        if ctrl is not None and ctrl.mode == "replay":
            state.bus.publish(f"{label} -> not available in replay")
            return
        state.dispatcher.dispatch(code, param, label)   # result lands on the bus -> broadcast

    elif mtype == "record" and ctrl is not None:
        if bool(msg.get("on")):
            ctrl.start_record()
        else:
            ctrl.stop_record()
        await _broadcast_session(state)
        await _broadcast_text(state.clients, json.dumps(build_captures_message(ctrl.captures_dir)))

    elif mtype == "list_captures" and ctrl is not None:
        await _broadcast_text(state.clients, json.dumps(build_captures_message(ctrl.captures_dir)))

    elif mtype == "rename_capture" and ctrl is not None:
        new_name = await asyncio.to_thread(ctrl.rename_last_recording, msg.get("name"))
        if new_name is None:
            state.bus.publish("rename -> invalid name or already exists")
        await _broadcast_session(state)
        await _broadcast_text(state.clients, json.dumps(build_captures_message(ctrl.captures_dir)))

    elif mtype == "load_capture" and ctrl is not None:
        path = sanitize_capture_name(msg.get("name"), ctrl.captures_dir)
        if path is None:
            log.warning("load_capture: unknown/invalid name %r", msg.get("name"))
            return
        await asyncio.to_thread(ctrl.switch_to_replay, path)
        await _reset_slam(state)          # fresh map for the new source
        ui.source, ui.selected_capture = "view", path.name
        if ui.display == "detailed":
            ui.display = "point_cloud"
        ui.mode = "realtime"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "go_live" and ctrl is not None:
        await asyncio.to_thread(ctrl.switch_to_live)
        await _reset_slam(state)
        ui.source, ui.selected_capture = "live", None
        if ui.display == "detailed":
            ui.display = "point_cloud"
        ui.mode = "realtime" if ui.display == "point_cloud" else "slam"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "transport" and ctrl is not None:
        action = msg.get("action")
        value = msg.get("value", 0)
        if action == "pause":
            ctrl.pause()
        elif action == "resume":
            ctrl.resume()
        elif action == "speed":
            ctrl.set_speed(float(value))
        elif action == "loop":
            ctrl.set_loop(bool(value))
        elif action == "restart":
            await asyncio.to_thread(ctrl.restart)
        elif action == "seek":
            await asyncio.to_thread(ctrl.seek, float(value))
        else:
            log.warning("unknown transport action: %r", action)
            return
        await _broadcast_session(state)

    elif mtype == "set_color":
        mode = msg.get("mode")
        if mode not in _VALID_COLOR_MODES:
            log.warning("invalid set_color mode: %r", mode)
            return
        ui.color_mode = mode
        _persist_ui(state)
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_ir":
        colormap = msg.get("colormap", ui.ir_colormap)
        if colormap not in _VALID_IR_COLORMAPS:
            log.warning("invalid set_ir colormap: %r", colormap)
            return
        freeze = bool(msg.get("freeze", ui.ir_freeze))
        ui.ir_colormap = colormap
        if freeze and not ui.ir_freeze:
            ui.ir_freeze_range = None     # arm capture: next IR tick grabs ir_range
        elif not freeze:
            ui.ir_freeze_range = None
        ui.ir_freeze = freeze
        _persist_ui(state)
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_view":
        changed = False
        if "colormap" in msg:
            if msg["colormap"] not in _VALID_VIEW_COLORMAPS:
                log.warning("invalid set_view colormap: %r", msg.get("colormap"))
                return
            ui.view_colormap = msg["colormap"]
            changed = True
        if "point_size" in msg:
            ps = float(msg["point_size"])
            if 0.001 <= ps <= 1.0:
                ui.point_size = ps
                changed = True
        if "point_size_auto" in msg:
            ui.point_size_auto = bool(msg["point_size_auto"])
            changed = True
        if "surface" in msg:
            ui.surface_enabled = bool(msg["surface"])
            changed = True
        if "surface_mode" in msg:
            if msg["surface_mode"] not in _VALID_SURFACE_MODES:
                log.warning("invalid set_view surface_mode: %r", msg.get("surface_mode"))
                return
            ui.surface_mode = msg["surface_mode"]
            changed = True
        if "surface_threshold" in msg:
            t = float(msg["surface_threshold"])
            if 0.1 <= t <= 50.0:
                ui.surface_threshold_pct = t
                changed = True
        if "view_mode" in msg:
            if msg["view_mode"] not in _VALID_VIEW_MODES:
                log.warning("invalid set_view view_mode: %r", msg.get("view_mode"))
                return
            ui.view_mode = msg["view_mode"]
            changed = True
        # Camera framing always edits the CURRENTLY SELECTED mode's slot -- the
        # three sliders show that mode, so there is exactly one thing they can
        # mean. Handled after `view_mode` above so a combined message lands on
        # the newly selected mode, not the one being left.
        cam = ui.view_cam.setdefault(ui.view_mode, ViewCam())
        if msg.get("cam_reset"):
            ui.view_cam[ui.view_mode] = cam = replace(_DEFAULT_VIEW_CAM[ui.view_mode])
            changed = True
        for key, attr, (lo, hi) in (("cam_distance", "distance_m", _CAM_DISTANCE_RANGE),
                                    ("cam_height", "height_m", _CAM_HEIGHT_RANGE),
                                    ("cam_rotation", "rotation_deg", _CAM_ROTATION_RANGE)):
            if key not in msg:
                continue
            try:
                v = float(msg[key])
            except (TypeError, ValueError):
                log.warning("invalid set_view %s: %r", key, msg.get(key))
                continue
            if lo <= v <= hi:                 # out of range: drop, keep the current value
                setattr(cam, attr, v)
                changed = True
        if "orbit" in msg:
            ui.orbit_enabled = bool(msg["orbit"])
            changed = True
        if "orbit_speed" in msg:
            try:
                s = float(msg["orbit_speed"])
            except (TypeError, ValueError):
                log.warning("invalid set_view orbit_speed: %r", msg.get("orbit_speed"))
                s = None
            if s is not None and _ORBIT_SPEED_RANGE[0] <= s <= _ORBIT_SPEED_RANGE[1]:
                ui.orbit_speed_deg_s = s
                changed = True
        if changed:
            _persist_ui(state)
            await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_source" and ctrl is not None:
        source = msg.get("source")
        if source not in _VALID_SOURCES:
            log.warning("invalid set_source: %r", source)
            return
        if source == "live":
            await asyncio.to_thread(ctrl.switch_to_live)
            ui.source, ui.selected_capture = "live", None
        elif ui.selected_capture:
            path = sanitize_capture_name(ui.selected_capture, ctrl.captures_dir)
            if path is not None:
                await asyncio.to_thread(ctrl.switch_to_replay, path)
                ui.source = "view"
        else:
            state.bus.publish("View -> select a capture first")
            return
        await _reset_slam(state)
        ui.display, ui.mode = "point_cloud", "realtime"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "set_display":
        display = msg.get("display")
        if display not in _VALID_DISPLAYS:
            log.warning("invalid set_display: %r", display)
            return
        if display != "point_cloud":
            if ctrl is not None and ctrl.mode == "replay" and not ctrl.index.get("has_stream_9"):
                state.bus.publish("SLAM -> unavailable: this legacy capture has no stream 9 orientation")
                return
            if display == "detailed" and (ctrl is None or ctrl.mode != "replay"):
                state.bus.publish("Detailed SLAM -> select a capture in View first")
                return
        ui.display = display
        ui.mode = "slam" if display == "slam" else "realtime"
        slam = getattr(state, "slam_runner", None)
        if slam is not None:
            await asyncio.to_thread(slam.set_active, display == "slam")
        if display == "detailed" and ctrl is not None and ctrl.replay_path:
            runner = getattr(state, "detailed_runner", None)
            if runner is not None:
                await asyncio.to_thread(runner.load_cached, ctrl.replay_path)
        await _broadcast_state(state)

    elif mtype in ("generate_detailed", "regenerate_detailed") and ctrl is not None:
        if ctrl.mode != "replay" or not ctrl.replay_path or not ctrl.index.get("has_stream_9"):
            state.bus.publish("Detailed SLAM -> unavailable without a stream-9 capture in View")
            return
        runner = getattr(state, "detailed_runner", None)
        if runner is None:
            return
        result = await asyncio.to_thread(runner.start, ctrl.replay_path,
                                         force=(mtype == "regenerate_detailed"))
        await _broadcast_text(state.clients, json.dumps({"type": "detailed", **result}))
        await _broadcast_state(state)

    elif mtype == "set_mode":
        mode = msg.get("mode")
        if mode not in _VALID_MODES:
            log.warning("invalid set_mode: %r", mode)
            return
        # Compatibility for existing rig clients: mode maps exactly onto the
        # shared display control, never to Detailed.
        ui.display = "slam" if mode == "slam" else "point_cloud"
        ui.mode = mode
        slam = getattr(state, "slam_runner", None)
        if slam is not None:
            # Arming is a cheap flag; disarming stops+joins the worker threads,
            # so do it off the event loop.
            await asyncio.to_thread(slam.set_active, mode == "slam")
        await _broadcast_state(state)

    elif mtype == "slam_opt":
        if "trajectory" in msg:
            ui.slam_trajectory = bool(msg["trajectory"])
        if "walls" in msg:
            if msg["walls"] not in _VALID_WALL_MODES:
                log.warning("invalid slam_opt walls: %r", msg.get("walls"))
                return
            ui.slam_walls = msg["walls"]
        if "follow" in msg:
            ui.slam_follow = bool(msg["follow"])
        _persist_ui(state)
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_idle":
        # Runtime control of the sensor auto-idle (persisted like the display
        # prefs). `enabled` toggles the whole feature; `level` picks soft/hard.
        changed = False
        if "enabled" in msg:
            ui.idle_enabled = bool(msg["enabled"])
            changed = True
        if "level" in msg:
            if msg["level"] not in _VALID_IDLE_LEVELS:
                log.warning("invalid set_idle level: %r", msg.get("level"))
                return
            ui.idle_level = msg["level"]
            changed = True
        if changed:
            _persist_ui(state)
            if not ui.idle_enabled:
                _cancel_idle_timer(state)   # a pending idle must not fire once disabled
            await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "set_orientation":
        # Orientation decomposition mode + custom axis labels (owner ask,
        # 2026-07-28). Presentation-only -- see `orientation_view()`.
        changed = False
        if "mode" in msg:
            if msg["mode"] not in _VALID_ORIENTATION_MODES:
                log.warning("invalid set_orientation mode: %r", msg.get("mode"))
                return
            ui.orientation_mode = msg["mode"]
            changed = True
        if "labels" in msg:
            ui.orientation_labels = _sanitize_axis_labels(msg["labels"])
            changed = True
        if changed:
            _persist_ui(state)
            await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "zero_yaw":
        # "Zero yaw here" (owner ask, 2026-07-29): the SFLP-derived yaw has an
        # arbitrary power-on origin and free-runs with gyro drift (no mag
        # input -- unlike roll/pitch, which are gravity-referenced and
        # absolute). Capture the CURRENTLY DISPLAYED mode's yaw-like slot at
        # THIS attitude and store the `graft_yaw` delta that zeroes it, so the
        # Sensors card reads 0 here and tracks relative motion from this pose
        # afterward. No-op in World mode: its yaw slot is the absolute
        # magnetic `heading_full`, not a function of the quat's own yaw, so
        # there is nothing sensible to zero (also disabled client-side; see
        # sensors.js's `isWorld` guard on the button).
        if ui.orientation_mode == "world":
            log.warning("zero_yaw ignored: World mode's heading is absolute magnetic north")
            return
        quat = state.sensor_state.fused_quat()
        if quat is None:
            log.warning("zero_yaw ignored: no orientation yet")
            return
        raw_yaw = orientation_view(ui.orientation_mode, quat).get("yaw_deg")
        if raw_yaw is None:
            return
        ui.yaw_offset_deg = _YAW_GRAFT_SIGN[ui.orientation_mode] * float(raw_yaw)
        _persist_ui(state)
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "clear_yaw_offset":
        # Explicit reset back to the raw (un-offset) SFLP yaw.
        ui.yaw_offset_deg = 0.0
        _persist_ui(state)
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

    elif mtype == "save":
        state.bus.publish("save -> Detailed SLAM owns persistent sidecars; Realtime SLAM is preview-only")

    elif mtype == "reset_fusion":
        state.sensor_state.reset_fusion()
        state.bus.publish("heading fusion reset")

    elif mtype == "magcal":
        await _handle_magcal(state, msg, ws)

    else:
        log.warning("unknown inbound message type: %r", mtype)


_MAGCAL_ACTIONS = ("open", "close", "start", "stop", "reset", "save", "discard", "view")


async def _handle_magcal(state, msg: dict, ws=None) -> None:
    """Route one `{"type": "magcal", "action": ...}` request.

    The workflow is deliberately two-stage — a fit is only ever a CANDIDATE
    until `save`, and `save` refuses a candidate that doesn't exist — so the
    saved calibration can never be replaced by a fit the user hasn't seen the
    quality of. That is the process failure this feature is fixing: the
    2026-07-15 calibration was accepted with no measurement of its
    field-magnitude consistency and stayed in production for two weeks.
    """
    action = msg.get("action")
    if action not in _MAGCAL_ACTIONS:
        log.warning("unknown/invalid magcal action: %r", action)
        return
    session = _magcal_session(state)
    clients = _magcal_clients(state)

    if action == "open":
        if ws is not None:
            clients.add(ws)
        # Answer this tab immediately rather than making it wait up to 200 ms
        # for the next broadcaster tick (Phase-1 late-joiner rule). This is the
        # ONE report that carries `cell_dirs` + `t_world_to_cv`; the client
        # caches them and every later tick omits them.
        if ws is not None:
            await ws.send_text(json.dumps(_magcal_report(state, full=True)))
        return

    if action == "close":
        if ws is not None:
            clients.discard(ws)
        return

    if action == "start":
        session.start()
        state.bus.publish("mag calibration: collecting")
    elif action == "stop":
        session.stop()          # fits a candidate; never raises
        if session.candidate is not None:
            q = quality_report_verdict(session, state)
            state.bus.publish(f"mag calibration: fit {q}")
        else:
            state.bus.publish(f"mag calibration: fit failed ({session.fit_error})")
    elif action == "reset":
        session.reset()
        state.bus.publish("mag calibration: samples cleared")
    elif action == "discard":
        # Keep the samples: the usual reason to reject a candidate is "not
        # enough coverage yet", and the fix is to tumble MORE into the same
        # cloud, not to throw away what's already there.
        session.candidate = None
        session.fit_error = None
        state.magcal_view = "current"
        state.bus.publish("mag calibration: candidate discarded")
    elif action == "view":
        state.magcal_view = msg.get("cal") if msg.get("cal") in ("current", "candidate") else "current"
    elif action == "save":
        cal = session.candidate
        if cal is None:
            state.bus.publish("mag calibration: nothing to save (stop collection first)")
        else:
            path = Path(getattr(state, "mag_cal_path", "mag_cal.json"))
            try:
                cal.save(path)
            except OSError as exc:
                state.bus.publish(f"mag calibration: save FAILED {exc}")
            else:
                install_mag_calibration(state, cal)
                session.candidate = None
                state.magcal_view = "current"
                state.bus.publish(
                    f"mag calibration: saved -> {path} (field {cal.field_ut:.2f} uT), applied live")
    await _broadcast_magcal(state)


def quality_report_verdict(session: MagSweepSession, state) -> str:
    """Short human summary of the candidate for the event log."""
    report = _magcal_report(state)
    cand = report.get("candidate") or {}
    field = cand.get("field") or {}
    std = field.get("std_pct")
    cov = (cand.get("coverage") or {}).get("fraction")
    return (f"{cand.get('verdict', '?')} — |B| spread "
            f"{'?' if std is None else f'{std:.1f}%'}, coverage "
            f"{'?' if cov is None else f'{100 * cov:.0f}%'}")


# --- CLI entry point --------------------------------------------------------

def main(argv=None) -> int:
    args = resolve_args(argv)

    # Web Phase 3: the reader lifecycle is owned by a SessionController so the
    # source can be swapped live<->replay at runtime. Launched with --replay we
    # start with NO live device (has_live False, Go Live disabled); otherwise we
    # open the live source once and keep it for the whole process, reused across
    # replay excursions via the _NoCloseSource proxy.
    live_source = None if args.replay else get_best_source(args.port, args.baud)
    # Name the transport up front: the #1 "no data" question is whether we're on
    # Ethernet, serial, or a dead serial fallback. Flushed so it shows even when
    # stdout is block-buffered (not a tty).
    if isinstance(live_source, UdpSource):
        live_label = f"Ethernet/UDP · {live_source.target_ip}:{live_source.target_port}"
    elif isinstance(live_source, SerialSource):
        live_label = f"Serial CDC · {getattr(live_source, 'port', '?')}"
    else:
        live_label = "no device"
    if live_source is not None:
        print(f"[source] {live_label}", flush=True)
    else:
        print(f"[source] Replay -> {args.replay}", flush=True)

    # client is None in replay (no device to command); the reader passes it only
    # in live mode, and the cmd handler reports "not available in replay" itself.
    client = CommandClient(live_source.write) if isinstance(live_source, (SerialSource, UdpSource)) else None
    stats = Stats()
    bus = LogBus()
    metrics = MetricsRegistry(window_s=2.0)
    dispatcher = CommandDispatcher(client, on_message=bus.publish)

    # Always compute all three planes: marginal cost per plane is ~zero and it
    # makes color mode a pure runtime choice (no reader restart) -- §5.1/§7.2.
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"),
                           flatfield=FlatField.load_configured())
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}

    # Sensor state (streams 9/10) -- built exactly like the desktop panel
    # (panel.py:525-541), reusing SensorState + YawFusion + MagCalibration.
    # getattr defaults cover viewer.resolve_args not defining the panel's sensor
    # flags; a missing mag_cal.json just leaves fusion in gated:no-cal.
    mag_cal = None
    fusion = None
    # Resolved ONCE so the web calibration modal saves to exactly the file the
    # loader (and panel.py:440) reads -- same expression, no second convention.
    mag_cal_path = getattr(args, "mag_cal_path", "mag_cal.json") or "mag_cal.json"
    if getattr(args, "yaw_fusion", True):
        mag_cal = MagCalibration.load(mag_cal_path)
        fusion = YawFusion(
            tau_s=float(getattr(args, "yaw_fusion_tau", 20.0) or 20.0),
            calibration=mag_cal,
            anomaly_frac=float(getattr(args, "yaw_anomaly_frac", 0.3) or 0.3),
            motion_rate_dps=float(getattr(args, "yaw_motion_rate_dps", 40.0) or 40.0),
            gimbal_margin_deg=float(getattr(args, "yaw_gimbal_margin_deg", 15.0) or 15.0),
        )
    sensor_state = SensorState(fusion=fusion)

    initial_speed_fps = float(args.replay_fps) if (args.replay and args.replay_fps and args.replay_fps > 0) else 0.0
    pacer = _Pacer(interval=speed_to_interval(initial_speed_fps) if args.replay else 0.0)
    recorder = Recorder()

    controller = SessionController(
        live_source=live_source, live_label=live_label, stage=stage, stats=stats,
        slot=slot, fault=fault, bus=bus, client=client, recorder=recorder, pacer=pacer,
        sensor_state=sensor_state, metrics=metrics, captures_dir=CAPTURES_DIR,
        initial_replay_path=args.replay, initial_speed_fps=initial_speed_fps)

    # SLAM mode (web Phase 4): armed lazily on the first `set_mode slam`; builds
    # no Open3D/GPU state until then, so real-time launches are unaffected.
    slam_runner = SlamRunner(bus=bus, fov_h=args.fov_h, fov_v=args.fov_v)

    # Shared app state, built once (§5.1).
    app.state.args = args
    app.state.source = live_source
    app.state.controller = controller
    app.state.recorder = recorder
    app.state.client = client
    app.state.stage = stage
    app.state.slot = slot
    app.state.bus = bus
    app.state.metrics = metrics
    app.state.dispatcher = dispatcher
    app.state.fault = fault
    app.state.fault_reported = False
    app.state.stats = stats
    app.state.pacer = pacer
    # Settings persistence (Web Phase 5): seed the UI from the shared
    # roomscan.toml [viewer] table and keep the loaded config around so runtime
    # display-pref changes write straight back to it. `mode` is not restored --
    # SLAM is armed lazily, so a restart always comes up in real-time.
    config = ViewerConfig.load()
    app.state.config = config
    app.state.ui_state = ui_from_config(config)
    if controller.mode == "replay":
        app.state.ui_state.source = "view"
        app.state.ui_state.selected_capture = os.path.basename(controller.replay_path)
    app.state.sensor_state = sensor_state
    app.state.mag_cal = mag_cal
    # Magnetometer sweep modal (owner ask, 2026-07-29). `fusion` is held so a
    # Save can hot-reload the filter's calibration too (install_mag_calibration);
    # `magcal_clients` is the per-tab subscriber set (empty => zero cost).
    app.state.mag_cal_path = mag_cal_path
    app.state.fusion = fusion
    app.state.magcal_session = MagSweepSession()
    app.state.magcal_clients = set()
    app.state.magcal_view = "current"
    app.state.slam_runner = slam_runner
    app.state.detailed_runner = DetailedRunner(bus=bus, results_dir=RESULTS_DIR)
    app.state.deproj = None
    app.state.orientation_smoother = OrientationSmoother()
    app.state.orientation_jitter = OrientationJitter()
    app.state.clients = set()
    app.state.command_labels = set()
    app.state.debounce = {}
    # Sensor auto-idle (SET_STANDBY): idle_enabled/idle_level live in ui_state
    # (persisted); the debounce delay and the runtime bookkeeping live here.
    app.state.idle_delay_s = float(getattr(config, "sensor_idle_delay_s", 5.0) or 5.0)
    app.state.sensor_idled = False   # whether we've commanded the device into standby
    app.state.idle_timer = None      # asyncio TimerHandle for the debounced idle
    app.state.ready = True

    # The controller owns the reader thread now (Web Phase 3): it runs the same
    # reader._run_reader body, but can stop+respawn it against a new source for
    # capture load / Go Live / seek, and tees raw bytes into the Recorder.
    controller.start()

    # A previous server may have left the sensor in standby (SET_STANDBY persists
    # on the device across a host restart). Guarantee it is streaming now, whatever
    # this launch's idle setting is -- a harmless no-op if the device is already
    # active (firmware acks without touching the sensor). Fire-and-forget; the ACK
    # (or a startup-race timeout) just lands on the log bus.
    if live_source is not None:
        _dispatch_standby(app.state, int(StandbyLevel.ACTIVE), "startup-wake")

    port = 8000
    url = f"http://localhost:{port}/static/index.html"
    print("\n=== roomscan web viewer ===")
    print(f"Starting server on {url}")
    print("Press Ctrl+C to stop.")

    # Small delay to let the server start before opening the browser.
    threading.Timer(1.0, lambda: _open_browser(url)).start()

    uvicorn.run(app, host="0.0.0.0", port=port, log_level="warning")
    return 0


def _open_browser(url: str) -> None:
    """Open the viewer, and on Linux launch Chrome/Chromium with software-WebGL
    enabled. On a headless host (no GPU -- the whole point of this deployment)
    Chrome refuses to create a WebGL context by default, so the Three.js viewer
    dies with "Error creating WebGL context" and the page is stuck at "Offline"
    (confirmed on-box 2026-07-15: baseline Chrome -> NO-WEBGL; with the flag ->
    WEBGL-OK via SwiftShader/llvmpipe). `--enable-unsafe-swiftshader` only
    *permits* the software fallback -- a machine with a real GPU still uses it,
    so this is safe to pass unconditionally. Set ROOMSCAN_NO_BROWSER=1 to skip
    the auto-open entirely (e.g. when viewing from another machine)."""
    if os.environ.get("ROOMSCAN_NO_BROWSER"):
        print(f"[browser] auto-open disabled; open {url} yourself.", flush=True)
        return
    if sys.platform.startswith("linux"):
        for exe in ("google-chrome", "google-chrome-stable", "chromium", "chromium-browser"):
            path = shutil.which(exe)
            if path:
                try:
                    subprocess.Popen(
                        [path, "--enable-unsafe-swiftshader", url],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    print(f"[browser] opened {exe} with software WebGL enabled.", flush=True)
                    return
                except Exception as exc:
                    print(f"[browser] {exe} launch failed ({exc}); falling back.", flush=True)
                    break
        print("[browser] no Chrome/Chromium found. If the viewer shows 'Offline' "
              "with a WebGL error, launch your browser with software WebGL "
              "(Chrome: --enable-unsafe-swiftshader).", flush=True)
    webbrowser.open(url)


if __name__ == "__main__":
    sys.exit(main())
