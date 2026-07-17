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
import os
import queue
import re
import shutil
import struct
import subprocess
import sys
import threading
import time
import webbrowser
import zlib
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from .colors import turbo
from .config import ViewerConfig
from .control import CommandClient, CommandDispatcher
from .decoder import StreamDecoder
from .deproject import Deprojector
from .ir_image import ir_range, reflectance_to_rgb
from .logbus import LogBus
from .magcal import MagCalibration
from .metrics import MetricsRegistry, MetricsSnapshot
from .pipeline import TransformStage
from .reader import _Pacer, _run_reader, follow_camera_target
from .protocol import (
    HEADER_SIZE,
    MAGIC,
    CommandCode,
    FrameHeader,
    FrameType,
    ProtocolError,
    StreamId,
)
from .sensors import (
    AXIS_CONVENTION,
    SensorState,
    T_CV_TO_BODY,
    T_WORLD_TO_CV,
    YawFusion,
    absolute_heading,
    quat_to_matrix,
)
from .sources import FileSource, Recorder, SerialSource, UdpSource, get_best_source
from .viewer import Stats, resolve_args

log = logging.getLogger("roomscan.web")

# Binary message type tags (first 4 bytes, little-endian uint32).
TAG_POINT_CLOUD = 1
TAG_IR_IMAGE = 2
TAG_MESH = 3               # SLAM reconstruction mesh (web Phase 4)

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
MISSING_PLANE_LOG_INTERVAL = 3.0   # debounce for missing-plane bus lines

# Recording & playback (web Phase 3).
CAPTURES_DIR = "captures"          # where Record writes + the library browses
# SLAM mode (web Phase 4).
RESULTS_DIR = "results"            # where Save writes the full-res map + trajectory
_TRAJ_TAIL_MAX = 256               # trajectory positions shipped in each `slam` message
_VALID_MODES = ("realtime", "slam")
_VALID_WALL_MODES = ("solid", "split")
# The playback speed segmented control maps ×0.5/×1/×2/Max onto these fps; a
# capture's own cadence is ~28 Hz, so ×1 (30 fps) plays it near-native and Max
# (0 -> interval 0) drains as fast as it decodes.
_SPEED_BASE_FPS = 30.0

_VALID_COLOR_MODES = ("depth", "reflectance", "confidence")
_VALID_IR_COLORMAPS = ("gray", "turbo")

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
    mode: str = "realtime"
    slam_trajectory: bool = True
    slam_walls: str = "split"          # "solid" | "split" -> MeshPrep wall_mode
    slam_follow: bool = True


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


def select_colors(outputs: dict, deproj: Deprojector, color_mode: str):
    """Deproject depth and colorize by the selected plane (§7.2).

    Returns (pts, colors, fell_back): pts (N,3) float32 metres, colors (N,3)
    float32 in [0,1], and fell_back True iff the requested non-depth plane was
    missing this frame and depth coloring was substituted. Coloring reuses the
    validity mask (finite, >0, < max_range) + min-max normalize + turbo, exactly
    as the classic viewer. `color_mode == "depth"` colors by deprojected Z.
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
    colors = turbo(vn)
    return pts.astype(np.float32, copy=False), colors.astype(np.float32, copy=False), fell_back


def pack_point_cloud(pts: np.ndarray, colors: np.ndarray) -> bytes:
    """POINT_CLOUD binary (tag 1): u32 tag=1 · f32[3N] positions · f32[3N]
    colors, all little-endian. Positions then colors, concatenated (§6.1)."""
    pos = np.ascontiguousarray(pts, dtype="<f4").ravel()
    col = np.ascontiguousarray(colors, dtype="<f4").ravel()
    return struct.pack("<I", TAG_POINT_CLOUD) + pos.tobytes() + col.tobytes()


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


def build_sensor_message(sensor_state: SensorState, mag_cal: MagCalibration | None) -> dict | None:
    """SensorState -> `sensor` JSON dict (streams 9/10), or None when there is no
    sensor data at all (so the broadcaster stays silent on a ToF-only session).

    The load-bearing math is reused verbatim from the desktop panel: the gizmo
    `rot` is the same display rotation `gizmo_pose` builds
    (T_WORLD_TO_CV @ R @ T_CV_TO_BODY, sensors.py:183-192), and `heading` is
    `absolute_heading` over the calibrated mag (panel.py:3172-3178). Computing
    them here keeps the sign/permutation matrices in exactly one place (Python),
    so the frontend never re-derives them.
    """
    quat = sensor_state.fused_quat()
    env = sensor_state.latest_env()
    press_hist = sensor_state.pressure_history()
    temp_hist = sensor_state.temp_history()
    if quat is None and env is None and press_hist.size == 0:
        return None

    rot = None
    if quat is not None:
        r = T_WORLD_TO_CV @ quat_to_matrix(*quat) @ T_CV_TO_BODY
        rot = [round(float(v), 5) for v in r.reshape(-1)]   # row-major 9

    heading = None
    mag_out = None
    if env is not None:
        mag = env.mag_ut
        if mag_cal is not None:
            mag = tuple(float(v) for v in AXIS_CONVENTION @ mag_cal.apply(mag))
        mag_out = [round(float(v), 2) for v in mag]
        if quat is not None:
            heading = round(absolute_heading(quat, tuple(mag)), 1)

    return {
        "type": "sensor",
        "have_quat": quat is not None,
        "rot": rot,
        "heading": heading,
        "pressure_pa": round(float(env.pressure_pa), 1) if env is not None else None,
        "temp_c": round(float(env.temp_c), 2) if env is not None else None,
        "mag_ut": mag_out,
        "fusion": sensor_state.fusion_status(),
        "pressure_hist": [round(float(v), 1) for v in press_hist.tolist()],
        "temp_hist": [round(float(v), 2) for v in temp_hist.tolist()],
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


def _state_message(ui: UiState) -> dict:
    return {"type": "state", "color_mode": ui.color_mode,
            "ir_colormap": ui.ir_colormap, "ir_freeze": ui.ir_freeze,
            "mode": ui.mode, "slam_trajectory": ui.slam_trajectory,
            "slam_walls": ui.slam_walls, "slam_follow": ui.slam_follow}


# --- settings persistence (Web Phase 5) -------------------------------------
#
# The web UI's display preferences live in the SAME `roomscan.toml` [viewer]
# table the desktop viewer/panel uses, so a single config follows the user
# across both frontends. Only the six display prefs below are web-owned; every
# other [viewer] field (fov/port/near-mode/yaw-fusion/...) is preserved
# verbatim because we mutate and re-save the whole loaded `ViewerConfig`.
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
    ui.slam_trajectory = bool(cfg.slam_trajectory)
    if cfg.slam_walls in _VALID_WALL_MODES:
        ui.slam_walls = cfg.slam_walls
    ui.slam_follow = bool(cfg.slam_follow)
    return ui


def apply_ui_to_config(ui: UiState, cfg: ViewerConfig) -> None:
    """Copy the six web-owned display prefs from `ui` into `cfg` in place
    (leaving `mode` and every non-web field alone), ready to `cfg.save()`."""
    cfg.color = ui.color_mode
    cfg.ir_colormap = ui.ir_colormap
    cfg.ir_freeze_range = bool(ui.ir_freeze)
    cfg.slam_trajectory = bool(ui.slam_trajectory)
    cfg.slam_walls = ui.slam_walls
    cfg.slam_follow = bool(ui.slam_follow)


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


def list_captures(captures_dir) -> list[dict]:
    """`captures/*.bin` as [{name, bytes, mtime}], newest first. Missing dir -> []."""
    d = Path(captures_dir)
    if not d.is_dir():
        return []
    items = []
    for p in sorted(d.glob("*.bin")):
        try:
            st = p.stat()
        except OSError:
            continue
        items.append({"name": p.name, "bytes": st.st_size, "mtime": round(st.st_mtime, 1)})
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
    calib_spans: list[tuple[int, int]] = []
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
            if hdr.stream_id == StreamId.CALIB:
                calib_spans.append((j, j + total))
            elif hdr.stream_id in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32):
                offsets.append(j)
                seqs.append(hdr.seq)
        i = j + total
    return {"n_frames": len(offsets), "offsets": offsets, "seqs": seqs,
            "calib_spans": calib_spans}


def build_session_message(mode, source_label, has_live, *, rec_active, rec_path,
                          rec_elapsed_s, rec_bytes, is_replay, capture_name,
                          paused, speed_fps, loop, position, total_frames) -> dict:
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
        },
        "playback": {
            "is_replay": is_replay,
            "capture_name": capture_name,
            "paused": paused,
            "speed_fps": speed_fps,
            "loop": loop,
            "position": position,
            "total_frames": total_frames,
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
                                  fov_v=self._fov_v, device=device)
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
        self.bus.publish(f"recording -> {path}")

    def stop_record(self) -> None:
        if not self.recorder.active:
            return
        path = self.recorder.path
        self.recorder.stop()
        self.bus.publish(f"recording stopped -> {path}")

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
        return build_session_message(
            self.mode, self.source_label, self.has_live,
            rec_active=rec_active, rec_path=rec_path,
            rec_elapsed_s=round(rec_elapsed, 1), rec_bytes=rec_bytes,
            is_replay=is_replay,
            capture_name=(os.path.basename(self.replay_path) if self.replay_path else None),
            paused=self.pacer.paused.is_set(), speed_fps=self.speed_fps, loop=self.loop,
            position=position, total_frames=total)


def _replay_position(ctrl: SessionController, last_item) -> float | None:
    """Current replay progress in [0,1] from the latest frame's seq vs the
    capture index's seq range, or None when not applicable (§3)."""
    if ctrl is None or ctrl.mode != "replay" or not ctrl.index or last_item is None:
        return None
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


async def _drop_client(clients: set, ws: WebSocket) -> None:
    """Remove a client and best-effort close it; never raises."""
    clients.discard(ws)
    try:
        await ws.close()
    except Exception:
        pass


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


async def _reset_slam(state) -> None:
    """Drop the SLAM map (off the event loop) after a source-swap, so a new
    capture / Go Live rebuilds a fresh map. No-op if SLAM was never armed."""
    slam = getattr(state, "slam_runner", None)
    if slam is not None:
        await asyncio.to_thread(slam.reset)


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
    last_ir = 0.0
    last_metrics = 0.0
    last_sensor = 0.0
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

            # POINT_CLOUD every tick (so late joiners see data within ~36ms),
            # but only in real-time mode -- SLAM mode replaces the cloud with the
            # reconstructed mesh, so skip the deproject+send entirely there.
            # Cache the packed bytes; rebuild only when the frame or color mode
            # changed, so a stalled feed doesn't re-deproject 28x/s for nothing.
            if ui.mode == "realtime":
                key = (header.seq, ui.color_mode)
                if key != last_pc_key:
                    pts, colors, fell_back = select_colors(outputs, state.deproj, ui.color_mode)
                    if fell_back:
                        _log_debounced(state, bus, f"color-miss:{ui.color_mode}",
                                       f"color mode {ui.color_mode!r} unavailable this frame, showing depth")
                    last_pc_bytes = pack_point_cloud(pts, colors)
                    last_pc_key = key
                if last_pc_bytes is not None:
                    await _broadcast_bytes(clients, last_pc_bytes)

            # SLAM mode (web Phase 4): feed the newest frame to the worker and
            # ship the latest `slam` message + (throttled) MESH. The feed/poll
            # touch off-thread workers; nothing blocks the event loop here.
            slam = getattr(state, "slam_runner", None)
            if ui.mode == "slam" and slam is not None:
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
                    await _broadcast_bytes(clients, pack_ir_image(rgb))
                else:
                    _log_debounced(state, bus, "ir-miss",
                                   "reflectance unavailable this frame, holding IR pane")

        # Sensor (streams 9/10) on its own cadence; silent until 9/10 arrives.
        if now - last_sensor >= SENSOR_INTERVAL:
            last_sensor = now
            smsg = build_sensor_message(state.sensor_state, state.mag_cal)
            if smsg is not None:
                await _broadcast_text(clients, json.dumps(smsg))

        # Metrics + session + bus drain on the slowest cadence.
        if now - last_metrics >= METRICS_INTERVAL:
            last_metrics = now
            snap = metrics.snapshot(now)
            await _broadcast_text(clients, json.dumps(build_metrics_message(snap)))
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

    # Bring the new tab current immediately.
    try:
        await websocket.send_text(json.dumps(_state_message(state.ui_state)))
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
                await _handle_inbound(state, json.loads(data))
            except Exception as exc:  # a malformed inbound message must never kill the loop (§9)
                log.warning("bad inbound ws message: %r (%s)", data[:200], exc)
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("ws receive loop error: %r", exc)
    finally:
        clients.discard(websocket)


async def _handle_inbound(state, msg: dict) -> None:
    """Route one decoded inbound JSON message by `type` (§5.4)."""
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

    elif mtype == "load_capture" and ctrl is not None:
        path = sanitize_capture_name(msg.get("name"), ctrl.captures_dir)
        if path is None:
            log.warning("load_capture: unknown/invalid name %r", msg.get("name"))
            return
        await asyncio.to_thread(ctrl.switch_to_replay, path)
        await _reset_slam(state)          # fresh map for the new source
        await _broadcast_session(state)

    elif mtype == "go_live" and ctrl is not None:
        await asyncio.to_thread(ctrl.switch_to_live)
        await _reset_slam(state)
        await _broadcast_session(state)

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

    elif mtype == "set_mode":
        mode = msg.get("mode")
        if mode not in _VALID_MODES:
            log.warning("invalid set_mode: %r", mode)
            return
        ui.mode = mode
        slam = getattr(state, "slam_runner", None)
        if slam is not None:
            # Arming is a cheap flag; disarming stops+joins the worker threads,
            # so do it off the event loop.
            await asyncio.to_thread(slam.set_active, mode == "slam")
        await _broadcast_text(state.clients, json.dumps(_state_message(ui)))

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

    elif mtype == "save":
        slam = getattr(state, "slam_runner", None)
        if slam is None or ui.mode != "slam":
            state.bus.publish("save -> not available (enter SLAM mode first)")
            return
        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        ply = Path(RESULTS_DIR) / f"web_{stamp}.ply"
        tum = Path(RESULTS_DIR) / f"web_{stamp}.tum"
        try:
            n = await asyncio.to_thread(slam.save, ply, tum)
        except Exception as exc:
            state.bus.publish(f"save -> ERROR {exc}")
            return
        state.bus.publish(f"saved {ply.name} ({n} verts)")
        await _broadcast_text(state.clients, json.dumps(build_saved_message(RESULTS_DIR)))

    else:
        log.warning("unknown inbound message type: %r", mtype)


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
    stage = TransformStage(outputs=("depth", "reflectance", "confidence"))
    slot: queue.Queue = queue.Queue(maxsize=1)
    fault: dict = {}

    # Sensor state (streams 9/10) -- built exactly like the desktop panel
    # (panel.py:525-541), reusing SensorState + YawFusion + MagCalibration.
    # getattr defaults cover viewer.resolve_args not defining the panel's sensor
    # flags; a missing mag_cal.json just leaves fusion in gated:no-cal.
    mag_cal = None
    fusion = None
    if getattr(args, "yaw_fusion", True):
        mag_cal = MagCalibration.load(
            getattr(args, "mag_cal_path", "mag_cal.json") or "mag_cal.json")
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
    app.state.sensor_state = sensor_state
    app.state.mag_cal = mag_cal
    app.state.slam_runner = slam_runner
    app.state.deproj = None
    app.state.clients = set()
    app.state.command_labels = set()
    app.state.debounce = {}
    app.state.ready = True

    # The controller owns the reader thread now (Web Phase 3): it runs the same
    # reader._run_reader body, but can stop+respawn it against a new source for
    # capture load / Go Live / seek, and tees raw bytes into the Recorder.
    controller.start()

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
