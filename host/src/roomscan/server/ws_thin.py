"""Thin-client /ws-thin WebSocket handler and frame pump (#194, #197)."""
from __future__ import annotations

import asyncio
import base64
import itertools
import json
import logging
import math
import os
import queue
import re
import socket
import struct
import sys
import time
import weakref
import zlib
from collections import deque
from contextlib import suppress
from dataclasses import asdict, dataclass, field, replace
from pathlib import Path

import numpy as np
from fastapi import WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState

from ..thin_render import (
    DEFAULT_JPEG_QUALITY,
    THIN_HEIGHT,
    THIN_POINT_SIZE_MAX,
    THIN_POINT_SIZE_MIN,
    THIN_WIDTH,
    ThinCamera,
    ThinRenderer,
    ThinRenderUnavailable,
    ThinViewConfig,
    extract_ir_full,
    extract_ir_grid,
    image_scene,
    jpeg_available,
    points_scene,
    unpack_mesh_scene,
    warm_jpeg_import,
)
from .session import *
from .ws_broadcaster import (
    _advance_due,
    _drop_client,
    _get_app_state,
    _ws_is_connected,
    _ws_transport,
    ws_backlog_bytes,
)

log = logging.getLogger("roomscan.web")

THIN_INTERVAL = 1.0 / 10.0
#: `thin_telemetry` cadence for a flow that has never negotiated one (#216).
#: Every v1 client, and any flow whose `thin_hello` omits
#: `telemetry_interval_s`, keeps exactly this -- 2 Hz, byte-identical to
#: pre-#216 behaviour.
THIN_TELEMETRY_INTERVAL = 0.5
#: default cadence granted to a flow the moment it ratchets to `proto: 2`
#: without naming one (spec 2026-08-23 §2B: "default 0.1s for v2 flows").
#: The IR grid rides `thin_telemetry`, so at 0.5 s the CrowPanel's thermal
#: feed is pinned at 2 FPS no matter what the sensor can do; 10 Hz is
#: fluid thermal video for ~30 KB/s of JSON on a ~187 KB/s link.
THIN_TELEMETRY_INTERVAL_V2 = 0.1
#: negotiation clamps for `telemetry_interval_s`. The floor is 20 Hz: the
#: ToF source itself tops out near 30 fps, so anything faster re-sends
#: numbers that have not changed, and the message is JSON (the full-res
#: `ir_grid_b64` alone is ~3 KB) -- 20 Hz is already ~60 KB/s. The ceiling
#: is the old default, so no client can make itself *slower* than v1.
THIN_TELEMETRY_INTERVAL_MIN = 0.05
THIN_TELEMETRY_INTERVAL_MAX = 1.0
# `thin_hello` negotiation clamps (#197) -- deliberately generous but bounded:
# a client that never sends `thin_hello` is unaffected (`ThinFlow` defaults
# reproduce THIN_INTERVAL / THIN_WIDTH / THIN_HEIGHT exactly).
THIN_HELLO_FPS_MIN = 1
# The fps ceiling is COUPLED to format (#197 review finding 5): a raw
# RGB565 480x480 frame is 460 800 bytes, so even today's 10 fps default is
# ~4.6 MB/s per client (see docs/thin-client.md); 60 fps raw would be
# ~27.6 MB/s and ~160% of the SINGLE shared render thread for just one
# client, starving the other (THIN_MAX_CLIENTS=2) even before the network is
# considered. A JPEG frame at the default quality is roughly 15-25 KB, so
# 60 fps JPEG (~1-1.5 MB/s) is cheaper than 10 fps raw -- raw keeps today's
# ceiling, jpeg gets the higher one.
THIN_HELLO_FPS_MAX_RAW = 10
THIN_HELLO_FPS_MAX_JPEG = 60
#: square resolutions only -- a non-square request is rejected outright and
#: the flow's prior width/height are kept, rather than distorting the image.
#:
#: 160 is the THUMBNAIL tier (#216): the CrowPanel's dashboard scanner page
#: embeds a 132x132 voxel preview, so serving it 480x480 spends ~85% of a
#: ~1.5 Mbit/s uplink on pixels the client immediately downscales away. A
#: 160x160 JPEG is ~1.5-2.5 KB, which fits 30 fps into ~60-120 KB/s.
#: Ordered ascending, which matters for `_nearest`: an exact midpoint (240)
#: ties, and `min` keeps the FIRST minimum, so a tie resolves to the
#: cheaper resolution rather than the more expensive one.
THIN_HELLO_RESOLUTIONS = (160, 320, 480)
THIN_HELLO_QUALITY_MIN = 40
THIN_HELLO_QUALITY_MAX = 95
DEFAULT_THIN_FPS = 1.0 / THIN_INTERVAL   # 10.0, matches today's cadence exactly
# `/ws-thin` v2 flow control (#202, CrowPanel bandwidth spec 2026-08-18). The
# server never has more than the granted credits outstanding to a v2 client and
# DROPS (never queues) at zero credit -- on the CrowPanel's ~1.5 Mbit/s link,
# pushing faster than the client drains only fills TCP buffers until pings hit
# seconds and the client's staleness timeout flaps the session. Depth 2 = one
# frame on the wire while the client works on the previous one; deeper just
# recreates the queue this exists to remove, so the clamp ceiling stays small.
THIN_CREDITS_DEFAULT = 2
THIN_CREDITS_MIN = 1
THIN_CREDITS_MAX = 8
#: floor for a client-advertised `max_frame_bytes` -- below this even a
#: quality-40 320x320 JPEG cannot reliably fit, so the request is nonsense.
THIN_MAX_FRAME_BYTES_MIN = 4096
# `ir_cells`: how many IR zones a client says it can render. The 8x8 thumbnail the
# CrowPanel shipped with is a ~97% throwaway of a 54x42 = 2268-zone sensor, so a client
# that can hold the real thing asks for it here. Floor is the old 64 (asking for less
# than the v1 shape buys nothing); ceiling is generous headroom over 2268 for a future
# sensor, bounded so one client cannot ask the telemetry message to become enormous.
THIN_IR_CELLS_MIN = 64
THIN_IR_CELLS_MAX = 4096
#: window for the per-client `tx_fps` / `tx_bytes_per_s` telemetry figures.
THIN_TX_WINDOW_S = 5.0
# Hard cap on concurrent thin clients. Connection count is a real performance
# variable on this server (BUG-060), and a thin client costs a full render --
# far above a deflate. All clients share the one renderer (a second
# `OffscreenRenderer` aborts the process), so renders serialise: 2 clients x
# 10 fps = 20 renders/s, about 54 % of the render thread.
THIN_MAX_CLIENTS = 2
# Sensor (streams 9/10) broadcast cadence: 15 Hz is smooth for a handheld gizmo
# and well above the ~4 Hz sparkline need. History rides every message so a


_VALID_THIN_COLORMAPS = ("amber", "turbo", "gray")
# Thin voxel point size, in pixels (Open3D's material scalar). Distinct from the
# browser's fractional `point_size` (world units) -- a different renderer, a
# different unit -- so it has its own range (#215).
_THIN_POINT_SIZE_RANGE = (THIN_POINT_SIZE_MIN, THIN_POINT_SIZE_MAX)


# --------------------------------------------------------------------------
# `/ws-thin` -- server-rendered raster feed for GPU-less thin clients (#194)
# --------------------------------------------------------------------------


@dataclass
class ThinFlow:
    """Per-connection `/ws-thin` state.

    Camera and mode are deliberately NOT in `UiState`: that is server-held and
    shared by every browser tab, whereas two thin clients must be able to orbit
    independently. A disconnect drops this object and with it all the state.
    """

    camera: ThinCamera = field(default_factory=ThinCamera)
    frames_sent: int = 0
    #: ticks skipped because the previous render had not finished
    skipped: int = 0
    render_errors: int = 0
    #: negotiated via `thin_hello` (#197). Defaults reproduce today's wire
    #: format exactly for a client that never sends one: tag-1 RGB565
    #: 480x480 at 10 fps.
    format: str = "raw"                       # "raw" | "jpeg"
    fps: float = DEFAULT_THIN_FPS
    width: int = THIN_WIDTH
    height: int = THIN_HEIGHT
    quality: int = DEFAULT_JPEG_QUALITY
    #: a raw `thin_hello` dict awaiting application by the render loop
    #: (#197 review finding 2) -- the receive task only ever STASHES it here,
    #: never applies it or sends the ack itself, so the render loop remains
    #: the single writer on this websocket.
    pending_hello: dict | None = None
    #: set once a jpeg request has been silently degraded to raw for this
    #: flow, so the one-per-flow warning (#197 review finding 9) fires only
    #: the first time, not on every subsequent hello/frame.
    jpeg_degraded_logged: bool = False
    #: protocol generation (#202): 1 until a `thin_hello` with `proto >= 2`
    #: arrives, then 2. Credit accounting applies only at proto 2 -- a v1
    #: client (or the pre-spec #197 hello shape) keeps free-running frames.
    proto: int = 1
    #: send credits currently held (proto 2 only). Each sent frame consumes
    #: one; each `thin_ready` restores one, up to `credits_max`. At zero the
    #: tick's frame is DISCARDED (counted in `dropped`), never queued.
    credits: int = 0
    credits_max: int = THIN_CREDITS_DEFAULT
    #: last frame sequence number SENT (tag-2 `seq` field); 0 = none yet.
    seq: int = 0
    #: frames discarded for this client since connect -- zero-credit ticks
    #: plus oversize-frame drops. Growing `dropped` with a healthy `tx_fps`
    #: means flow control is working, not failing.
    dropped: int = 0
    #: client-advertised RX buffer ceiling for one whole frame message, or
    #: None if never advertised. A packed frame larger than this is dropped
    #: for this client (it could not buffer it anyway) and counted.
    max_frame_bytes: int | None = None
    #: client-advertised IR zone budget. None (never advertised) keeps the v1
    #: 8x8 `ir_grid`; a value >= the sensor's native zone count upgrades this
    #: flow to the full-resolution `ir_grid_b64` instead.
    ir_cells: int | None = None
    #: negotiated `thin_telemetry` cadence in seconds, or None if this flow
    #: has never negotiated one -- None means THIN_TELEMETRY_INTERVAL (0.5 s),
    #: so a v1 client's cadence is untouched by #216. Read it through the
    #: `telemetry_interval` property, never directly.
    telemetry_interval_s: float | None = None
    #: (monotonic_time, frame_bytes) per sent frame, trimmed to the last
    #: THIN_TX_WINDOW_S by the telemetry builder -- feeds tx_fps/tx_bytes_per_s.
    tx_log: deque = field(default_factory=deque)
    connected_at: float = field(default_factory=time.monotonic)
    oversize_logged: bool = False

    @property
    def interval(self) -> float:
        """Per-flow render/send pacing interval, in seconds."""
        return (1.0 / self.fps) if self.fps > 0 else THIN_INTERVAL

    @property
    def telemetry_interval(self) -> float:
        """Per-flow `thin_telemetry` cadence, in seconds (#216).

        Resolves the "never negotiated" sentinel so no caller has to: an
        un-negotiated flow reports the module default, which is what every
        v1 client and every pre-#216 flow gets.
        """
        if self.telemetry_interval_s is None:
            return THIN_TELEMETRY_INTERVAL
        return self.telemetry_interval_s


def _thin_clients(state) -> dict:
    """Lazily-created `{WebSocket: ThinFlow}` map, same pattern as
    `_mesh_clients` -- hand-built `app.state` objects in tests needn't know."""
    clients = getattr(state, "thin_clients", None)
    if clients is None:
        clients = state.thin_clients = {}
    return clients


def _thin_mesh_scene(state, generation):
    """Renderable geometry for the SLAM mode, unpacked from the cached MESH
    bytes and memoised **by object identity**.

    Unpacking a 150 k-vertex MESH costs real numpy work; doing it per tick at
    10 fps would dwarf the render itself. `_cache_latest_mesh` already treats
    identity (never `mesh_seq`, which resets to 0 on `_reset_slam`) as the
    definition of newness, so this cache uses the same test.
    """
    mesh_bytes = getattr(state, "latest_mesh", None)
    if mesh_bytes is None:
        return None
    cached = getattr(state, "thin_mesh_cache", None)
    if cached is not None and cached[0] is mesh_bytes:
        return cached[1]
    scene = unpack_mesh_scene(mesh_bytes, generation=generation)
    state.thin_mesh_cache = (mesh_bytes, scene)
    return scene


def thin_scene_for(state, mode: str):
    """The `ThinScene` to draw for `mode`, or None if there is no fresh data.

    Every stash is generation-tagged and a mismatch is dropped rather than
    drawn (#101): a live->replay switch must never show a thin client the
    previous source's geometry.
    """
    ctrl = getattr(state, "controller", None)
    cur_gen = getattr(ctrl, "generation", None)

    if mode == "ir":
        stash = getattr(state, "thin_latest_ir", None)
        if stash is None:
            return None
        gen, rgb = stash
        if gen != cur_gen or rgb is None:
            return None
        return image_scene(rgb, generation=gen)

    if mode == "slam":
        return _thin_mesh_scene(state, cur_gen)

    stash = getattr(state, "thin_latest_pc", None)
    if stash is None:
        return None
    gen, pts, colors = stash
    if gen != cur_gen:
        return None
    if pts is None or len(pts) == 0:
        return None
    return points_scene(pts, colors, generation=gen)



def thin_telemetry_message(state, flow: ThinFlow) -> dict:
    """The curated `thin_telemetry` projection, sent at `flow.telemetry_interval`.

    A strict subset of numbers the existing `ranging`/`sensor`/`metrics`
    builders already compute -- no new telemetry math. `mode` and `recording`
    are authoritative: `/ws-thin` has no ack protocol, so a dropped
    `thin_mode`/`thin_record` self-corrects at the next tick.

    Cadence is 2 Hz unless negotiated (#216). Note the ACHIEVED rate is
    bounded by the render tick rate as well as by the negotiated interval:
    this message is built once per pass of `_thin_render_loop`, so a flow
    at 10 fps cannot exceed 10 Hz telemetry however short an interval it
    asked for. Ask for the fps first.
    """
    fps = None
    if getattr(state, "ranging_state", None) is not None:
        try:
            ranging = _ranging_message(state)
        except Exception:  # noqa: BLE001 - telemetry must never kill the feed
            ranging = {}
        fps = ranging.get("measured_fps")
        if fps is None:
            fps = (ranging.get("applied") or {}).get("fps")

    # Orientation + heading (owner ask, 2026-08-18). Taken VERBATIM from the
    # sensor message the broadcaster already built -- the same numbers, in the
    # same decomposition, that the web UI's Sensors panel displays.
    #
    # Deliberately not recomputed from the quaternion here. No scalar taken off
    # an attitude is the bearing of an axis; asking for "yaw" instead of the
    # bearing of the axis you mean is the single defect this repo has shipped
    # most often (four times, two of them in live code). Reusing the finished
    # projection means this endpoint cannot be the fifth.
    roll = tilt = pitch = heading = None
    orientation_valid = False
    orientation_labels = list(DEFAULT_AXIS_LABELS)
    smsg = getattr(state, "thin_latest_sensor", None)
    if smsg:
        heading = smsg.get("heading_deg")
        if heading is None:
            heading = smsg.get("heading")
        view = smsg.get("orientation_view") or {}
        roll = view.get("roll_deg")
        tilt = view.get("pitch_deg")
        pitch = tilt
        # NO fallback to `orientation_view["yaw_deg"]`. In the World
        # decomposition that slot happens to be the heading, but under `zyx`
        # (or any alternate decomposition) it is a Tait-Bryan twist about a
        # body axis -- not a bearing at all. Publishing it as `heading_deg`
        # would be this repo's most-repeated defect for the fifth time. If
        # `heading_deg` is absent there is no bearing, and the client is told
        # so rather than shown a plausible wrong number.
        orientation_valid = bool(view.get("valid", False)) and heading is not None
        labels = smsg.get("labels")
        if labels:
            orientation_labels = list(labels)

    # Angular rate around body yaw axis (deg/s) from stream 11 gyro
    yaw_rate_dps = 0.0
    sensor_st = getattr(state, "sensor_state", None)
    if sensor_st is not None:
        try:
            imu_raw = getattr(sensor_st, "latest_imu_raw", lambda: None)()
            if imu_raw is not None and getattr(imu_raw, "gyro_dps", None) is not None and len(imu_raw.gyro_dps) > 0:
                w = np.asarray(imu_raw.gyro_dps, dtype=np.float64)
                gbias = getattr(imu_raw, "gbias_dps", None)
                if gbias is not None and len(gbias) > 0:
                    w = w - np.asarray(gbias, dtype=np.float64)
                mean_w = np.mean(w, axis=0)
                yaw_rate_dps = round(float(mean_w[2]), 2)
        except Exception:  # noqa: BLE001
            yaw_rate_dps = 0.0

    point_count = 0
    stash = getattr(state, "thin_latest_pc", None)
    if stash is not None and stash[1] is not None:
        point_count = int(len(stash[1]))

    ctrl = getattr(state, "controller", None)
    recording = bool(getattr(ctrl, "recording", False)) if ctrl is not None else False

    # IR thumbnail for the sidebar widget. A client that advertised an `ir_cells`
    # budget big enough for the sensor's native zone grid (54x42 on the 3DMD) gets
    # that, base64'd, plus its shape -- the 8x8 below discards ~97% of it and was
    # only ever a bandwidth compromise. Everyone else keeps the exact v1 8x8 array.
    ir_grid = None
    ir_grid_b64 = None
    ir_grid_w = ir_grid_h = None
    ir_stash = getattr(state, "thin_latest_ir", None)
    if ir_stash is not None and ir_stash[1] is not None:
        try:
            full = (extract_ir_full(ir_stash[1], flow.ir_cells)
                    if flow.ir_cells is not None else None)
            if full is not None:
                ir_grid_w, ir_grid_h, cells = full
                ir_grid_b64 = base64.b64encode(cells).decode("ascii")
            else:
                ir_grid = extract_ir_grid(ir_stash[1])
        except Exception:  # noqa: BLE001
            ir_grid = ir_grid_b64 = None

    tx_fps, tx_bytes_per_s = thin_tx_stats(flow)

    return {
        "type": "thin_telemetry",
        # `fps` is the RIG's internal render rate, NOT this client's frame
        # rate -- the tx_* fields below are the per-client truth (#202).
        "fps": fps,
        "tx_fps": tx_fps,
        "tx_bytes_per_s": tx_bytes_per_s,
        "dropped": flow.dropped,
        "point_count": point_count,
        "recording": recording,
        "mode": flow.camera.mode,
        "link": "ok",
        # Spatial Orientation & Heading
        "heading_deg": heading,
        "pitch_deg": pitch,
        "roll_deg": roll,
        "yaw_rate_dps": yaw_rate_dps,
        "tilt_deg": tilt,
        "orientation_valid": orientation_valid,
        "orientation_labels": orientation_labels,
        # IR Matrix Array. Exactly one of these is populated: `ir_grid` is the v1
        # 8x8 list of ints; `ir_grid_b64` is `ir_grid_w * ir_grid_h` uint8 zones,
        # row-major, base64'd, for a client that negotiated `ir_cells`. The shape
        # is sent every time and is NOT constant -- the pane rotates with gravity,
        # which transposes it.
        "ir_grid": ir_grid,
        "ir_grid_b64": ir_grid_b64,
        "ir_grid_w": ir_grid_w,
        "ir_grid_h": ir_grid_h,
    }


def thin_tx_stats(flow: ThinFlow, now: float | None = None) -> tuple[float, int]:
    """Per-client `(tx_fps, tx_bytes_per_s)` over the last THIN_TX_WINDOW_S.

    Trims `flow.tx_log` in place. The divisor is the real observation window
    (capped by time-since-connect, floored at 0.5 s) so a connection two
    seconds old reports its true rate rather than a fifth of it. These are
    the spec's per-client truth (#202): the rig-internal `fps` reads ~30
    while a throttled CrowPanel receives ~0.4, and displaying the former as
    the latter was "the single most misleading number in the whole system".
    """
    if now is None:
        now = time.monotonic()
    cutoff = now - THIN_TX_WINDOW_S
    tx_log = flow.tx_log
    while tx_log and tx_log[0][0] < cutoff:
        tx_log.popleft()
    window = max(0.5, min(THIN_TX_WINDOW_S, now - flow.connected_at))
    tx_fps = round(len(tx_log) / window, 2)
    tx_bytes_per_s = int(sum(nbytes for _, nbytes in tx_log) / window)
    return tx_fps, tx_bytes_per_s


def _nearest(value: float, choices) -> int:
    return min(choices, key=lambda c: abs(c - value))


def _finite_float(value) -> float | None:
    """`float(value)` if that succeeds AND is finite, else `None`.

    JSON permits the literal tokens `Infinity`/`-Infinity`/`NaN` (Python's
    `json` module accepts them on both ends by default), and `int(float("inf"))`
    raises an uncaught `OverflowError` -- which used to escape
    `negotiate_thin_hello` entirely, skipping the rest of the message AND the
    ack (#197 review finding 1). Every numeric field below is parsed through
    here first, so a non-finite value is simply indistinguishable from a
    missing one: left at the flow's current value, never a raise.
    """
    if value is None:
        return None
    try:
        f = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return f if math.isfinite(f) else None


def negotiate_thin_hello(msg: dict, flow: ThinFlow) -> dict:
    """Pure clamping logic for one `thin_hello` message (#197).

    Returns the dict of EFFECTIVE (clamped) values -- what the caller applies
    to `flow` and echoes back as `thin_hello_ack`, "report what actually
    happened, not what was requested". Never raises: a field this cannot make
    sense of (wrong type, missing, non-finite, non-square width/height) is
    simply left at `flow`'s current value, so one bad field cannot corrupt the
    others in the same message, and a fully malformed hello is a genuine
    no-op -- see `_finite_float`.

    `fps` stays a FLOAT throughout (`ThinFlow.fps` is a float): an earlier
    version ran requested fps through `int()` before clamping, which both
    truncated a fractional request (0.5 fps became 1 fps for the wrong
    reason -- 0, then clamped up to the floor) and rejected a numeric STRING
    like `"30.5"` while accepting `"30"` (#197 review finding 1). The fps
    ceiling is COUPLED to `format` (finding 5) and is re-applied to the
    EFFECTIVE fps on every call, whether or not this message requested a new
    one -- so a client that negotiated jpeg@60 and later asks only for
    `format: "raw"` gets its stored 60 fps re-clamped down to the raw
    ceiling, rather than carrying an now-illegal rate forward silently.

    v2 (#202, CrowPanel spec 2026-08-18) adds `proto`, `accept` (a preference
    list of "jpeg"/"rgb565" -- the spec's spelling of the format field),
    `credits` and `max_frame_bytes`. `accept` and `format` are two spellings
    of the same choice; an explicit `format` wins when both appear. `proto`
    only ever ratchets UP to 2 -- a later hello without it does not demote
    the flow back to free-running.

    #216 adds the 160 thumbnail tier to `THIN_HELLO_RESOLUTIONS` and a
    negotiable `telemetry_interval_s`. Both are re-negotiable mid-connection
    (the CrowPanel switches between a 132x132 dashboard preview and a full
    LiDAR page), which needs nothing new here -- `pending_hello` already
    routes every hello through this function, first or fiftieth.
    """
    proto = flow.proto
    requested_proto = _finite_float(msg.get("proto"))
    if requested_proto is not None and requested_proto >= 2:
        proto = 2

    fmt = flow.format
    accept = msg.get("accept")
    if isinstance(accept, (list, tuple)):
        # Preference order is the client's: first entry this server can
        # actually serve wins ("rgb565" is the spec's name for tag-1 raw).
        for choice in accept:
            if choice == "jpeg" and jpeg_available():
                fmt = "jpeg"
                break
            if choice in ("rgb565", "raw"):
                fmt = "raw"
                break
    requested_fmt = msg.get("format")
    if requested_fmt in ("raw", "jpeg"):
        fmt = requested_fmt
    if fmt == "jpeg" and not jpeg_available():
        # Degrade gracefully (#197 requirement 5) -- never promise a codec
        # this process cannot actually run.
        fmt = "raw"

    credits = flow.credits_max
    requested_credits = _finite_float(msg.get("credits"))
    if requested_credits is not None:
        credits = int(round(max(float(THIN_CREDITS_MIN),
                                min(float(THIN_CREDITS_MAX), requested_credits))))

    max_frame_bytes = flow.max_frame_bytes
    requested_mfb = _finite_float(msg.get("max_frame_bytes"))
    if requested_mfb is not None:
        max_frame_bytes = max(THIN_MAX_FRAME_BYTES_MIN, int(requested_mfb))

    ir_cells = flow.ir_cells
    requested_ir = _finite_float(msg.get("ir_cells"))
    if requested_ir is not None:
        ir_cells = int(round(max(float(THIN_IR_CELLS_MIN),
                                 min(float(THIN_IR_CELLS_MAX), requested_ir))))

    # `telemetry_interval_s` (#216). Three cases, in order:
    #   explicit request  -> clamp it;
    #   v2 and never negotiated -> the v2 default (0.1 s / 10 Hz);
    #   anything else     -> keep what the flow already has, which for a v1
    #                        flow is the None sentinel = 0.5 s.
    # Note the middle case reads `proto` (the value this call is ratcheting
    # TO), not `flow.proto`, so the v2 default lands on the same hello that
    # opts the flow into v2 rather than one hello later. Once applied, the
    # flow holds a concrete float, so a later re-negotiation that omits the
    # field keeps the client's chosen cadence instead of snapping back.
    telemetry_interval_s = flow.telemetry_interval_s
    requested_ti = _finite_float(msg.get("telemetry_interval_s"))
    if requested_ti is not None:
        telemetry_interval_s = max(THIN_TELEMETRY_INTERVAL_MIN,
                                   min(THIN_TELEMETRY_INTERVAL_MAX, requested_ti))
    elif telemetry_interval_s is None and proto >= 2:
        telemetry_interval_s = THIN_TELEMETRY_INTERVAL_V2

    fps_max = THIN_HELLO_FPS_MAX_JPEG if fmt == "jpeg" else THIN_HELLO_FPS_MAX_RAW

    fps = flow.fps
    requested_fps = _finite_float(msg.get("fps"))
    if requested_fps is not None:
        fps = requested_fps
    # Re-clamp unconditionally -- not only when this message requested a new
    # fps -- because a format change alone can invalidate a stored rate.
    fps = max(float(THIN_HELLO_FPS_MIN), min(float(fps_max), fps))

    width, height = flow.width, flow.height
    rw = _finite_float(msg.get("width"))
    rh = _finite_float(msg.get("height"))
    if rw is not None and rh is not None:
        if rw == rh:
            width = height = _nearest(rw, THIN_HELLO_RESOLUTIONS)
        # else: non-square is rejected outright -- keep the prior size

    quality = flow.quality
    rq = _finite_float(msg.get("quality"))
    if rq is not None:
        quality = int(round(max(float(THIN_HELLO_QUALITY_MIN),
                                min(float(THIN_HELLO_QUALITY_MAX), rq))))

    return {"format": fmt, "fps": fps, "width": width, "height": height,
            "quality": quality, "proto": proto, "credits": credits,
            "max_frame_bytes": max_frame_bytes, "ir_cells": ir_cells,
            "telemetry_interval_s": telemetry_interval_s}


def _apply_pending_hello(flow: ThinFlow) -> dict | None:
    """Pop and apply `flow.pending_hello`, if any; return the
    `thin_hello_ack` JSON dict to send, or `None` if nothing was pending.

    The ONLY caller is `_thin_render_loop`, at the top of a tick (#197 review
    finding 2). The receive task (`_handle_thin_inbound`) only ever stashes
    the raw message on `flow.pending_hello` -- it never applies it and never
    writes to the websocket -- so this connection has exactly one writer, and
    the ack is guaranteed to reach the client before any frame rendered under
    the new settings (same tick, right after this call, using the
    already-updated `flow`).
    """
    if flow.pending_hello is None:
        return None
    msg, flow.pending_hello = flow.pending_hello, None
    effective = negotiate_thin_hello(msg, flow)
    requested_jpeg = (msg.get("format") == "jpeg"
                      or (isinstance(msg.get("accept"), (list, tuple))
                          and "jpeg" in msg["accept"]))
    was_proto = flow.proto
    flow.format = effective["format"]
    flow.fps = effective["fps"]
    flow.width = effective["width"]
    flow.height = effective["height"]
    flow.quality = effective["quality"]
    flow.proto = effective["proto"]
    flow.credits_max = effective["credits"]
    flow.max_frame_bytes = effective["max_frame_bytes"]
    flow.ir_cells = effective["ir_cells"]
    flow.telemetry_interval_s = effective["telemetry_interval_s"]
    if flow.proto >= 2:
        if was_proto < 2:
            # The handshake itself is the initial grant (spec: "server starts
            # each connection with the credit count from the handshake").
            flow.credits = flow.credits_max
        else:
            # Re-negotiation never grants free frames -- only shrink to a
            # lowered ceiling; outstanding credit is the client's to restore.
            flow.credits = min(flow.credits, flow.credits_max)
    if requested_jpeg and effective["format"] != "jpeg" and not flow.jpeg_degraded_logged:
        # One line per FLOW, not per frame (#197 review finding 9) -- the
        # observable that finding 8's venv-staleness migration was missing.
        flow.jpeg_degraded_logged = True
        log.warning("thin_hello requested format=jpeg but simplejpeg is "
                    "unavailable; flow stays on format=raw")
    # The ack is authoritative and reports the CLAMPED effective values. The
    # spec's field is `encoding` ("jpeg"/"rgb565"); `format`/`fps` ride along
    # for the pre-spec #197 hello shape so both client generations read the
    # same message.
    ack = {"type": "thin_hello_ack", "proto": flow.proto,
           "encoding": "jpeg" if flow.format == "jpeg" else "rgb565",
           **{k: effective[k] for k in
              ("format", "fps", "width", "height", "quality")},
           # Always the RESOLVED cadence, never the None sentinel: a client
           # that omitted the field still gets told what it will actually
           # receive, which is the whole point of an authoritative ack.
           "telemetry_interval_s": flow.telemetry_interval}
    if flow.proto >= 2:
        ack["credits"] = flow.credits_max
    if flow.ir_cells is not None:
        ack["ir_cells"] = flow.ir_cells
    return ack


async def _handle_thin_inbound(state, msg: dict, flow: ThinFlow) -> None:
    """Apply one `/ws-thin` command. Unknown types are ignored, not fatal.

    `thin_hello` is deliberately NOT applied here -- see `_apply_pending_hello`.
    """
    mtype = msg.get("type")
    if mtype == "thin_orbit":
        flow.camera.apply_orbit(
            float(msg.get("dyaw", 0.0) or 0.0),
            float(msg.get("dpitch", 0.0) or 0.0),
            float(msg.get("dzoom", 0.0) or 0.0))
    elif mtype == "thin_mode":
        flow.camera.set_mode(msg.get("mode"))
    elif mtype == "thin_hello":
        flow.pending_hello = msg
    elif mtype == "thin_ready":
        # One credit back, capped at the negotiated depth (#202). The echoed
        # `seq` is informational -- the grant itself is the accounting event,
        # so a raw-format v2 flow (tag 1 carries no seq) still flow-controls.
        # Harmless before a v2 hello: `credits` only gates sends at proto 2.
        if flow.credits < flow.credits_max:
            flow.credits += 1
    elif mtype == "thin_record":
        ctrl = getattr(state, "controller", None)
        if ctrl is not None:
            await _apply_record(state, ctrl, bool(msg.get("on")))
    else:
        log.warning("unknown /ws-thin message type %r", mtype)


def thin_tick_may_send(flow: ThinFlow) -> bool:
    """Credit gate for one render tick (#202). Pure decision + counting.

    A v1 flow (no spec-shaped hello) always sends -- free-running is exactly
    v1's contract. A v2 flow at zero credit has this tick's frame DISCARDED
    (counted in `flow.dropped`) before it is even rendered: the spec's "drop
    on render completion" is satisfied one step earlier, which also spares
    the single shared render thread work no one can receive. Nothing is ever
    queued, so when a `thin_ready` restores credit the next tick renders a
    genuinely fresh frame -- "the newest frame it has" by construction.
    """
    if flow.proto < 2:
        return True
    if flow.credits <= 0:
        flow.dropped += 1
        return False
    return True


def thin_frame_fits(flow: ThinFlow, frame_len: int) -> bool:
    """False if a packed frame exceeds the client's advertised RX buffer.

    The CrowPanel sizes its receive buffer to `max_frame_bytes`; a bigger
    message could never be reassembled there, so it is dropped for this
    client (counted, logged once per flow) rather than sent to certain
    failure. No ceiling advertised = nothing to enforce.
    """
    if flow.max_frame_bytes is None or frame_len <= flow.max_frame_bytes:
        return True
    flow.dropped += 1
    if not flow.oversize_logged:
        flow.oversize_logged = True
        log.warning(
            "thin frame of %d B exceeds the client's max_frame_bytes=%d; "
            "dropping (lower quality/resolution via thin_hello)",
            frame_len, flow.max_frame_bytes)
    return False


def _thin_next_tick(flow: ThinFlow, next_tick: float, now: float) -> tuple[float, bool]:
    """Advance `next_tick` by `flow.interval` (per-flow pacing, #197).

    Pure so pacing is testable with a literal "fake clock" rather than a
    monkeypatched `time.monotonic` driving a real asyncio loop. Resyncs
    (rather than burst-catching-up) when the previous tick's work overran --
    returns `skipped=True` in that case, mirroring the old inline logic that
    used the module-level `THIN_INTERVAL` unconditionally.
    """
    interval = flow.interval
    next_tick += interval
    if next_tick <= now:
        return now + interval, True
    return next_tick, False


async def _thin_render_loop(state, websocket, flow: ThinFlow) -> None:
    """Render and push frames to one thin client, paced at the flow's own
    negotiated interval (`flow.interval`; `THIN_INTERVAL`/10 fps by default).

    This task is the ONLY writer on `websocket` (#197 review finding 2): a
    pending `thin_hello` is applied and acked here, at the top of a tick,
    before that tick's telemetry/frame -- never from the receive task, which
    only stashes it (`_handle_thin_inbound` -> `flow.pending_hello`). That
    guarantees the ack reaches the client before any frame rendered under the
    new settings, and rules out two tasks racing to write the same socket.

    Backpressure is freshest-frame-wins, and it is **structural** rather than a
    governor: this task awaits its own render before pacing the next tick, so
    exactly one render per client can ever be in flight, and a tick whose work
    overran the interval resyncs instead of burst-catching-up. That is the
    single-slot guard. It is deliberately not built on the send side -- on this
    stack `send_bytes` never blocks and queues unboundedly (BUG-061), so a
    send-derived signal would measure nothing. A slow render costs this client
    frames (counted in `flow.skipped`) and never the other clients.

    (#217 corrects the last clause: a send-derived signal measures nothing only
    if you read the send's RETURN value. The queue itself is plainly visible --
    see `ws_backlog_bytes` -- and `/ws` is now gated on it. The structural guard
    here is still the right design for THIS endpoint, because a thin client's
    cost is the render, not the bytes; the point is only that "unbounded" was
    never the same claim as "unmeasurable".)
    """
    renderer = ThinRenderer.instance()
    loop = asyncio.get_running_loop()
    next_tick = time.monotonic()
    next_telemetry = 0.0
    client_id = _resolve_client_id(websocket)

    while True:
        delay = next_tick - time.monotonic()
        if delay > 0:
            await asyncio.sleep(delay)
        now = time.monotonic()
        # Self-touch every tick (not just on inbound messages): a v1 thin
        # client never sends anything after its initial hello, so waiting on
        # `_handle_thin_inbound` would let a perfectly live connection age out
        # past `sensor_idle_activity_timeout_s` and idle the sensor under it.
        _touch_client_active(state, client_id)

        ack = _apply_pending_hello(flow)
        if ack is not None:
            try:
                await websocket.send_text(json.dumps(ack))
            except Exception:  # noqa: BLE001 - disconnect; the receive loop wins
                return
            # Force this tick's telemetry (#216). `next_telemetry` was
            # scheduled under the PREVIOUS cadence, so a client shortening
            # 1.0 s -> 0.05 s would otherwise wait out the full old interval
            # before the new rate began -- and a re-negotiating client wants
            # the server's authoritative mode/recording view now anyway.
            next_telemetry = 0.0

        next_tick, skipped = _thin_next_tick(flow, next_tick, now)
        if skipped:
            flow.skipped += 1

        if now >= next_telemetry:
            # Advance the schedule by whole intervals from the DUE time, not
            # by re-anchoring it to `now` (#216). `now + interval` re-phases
            # to the render tick on every send, which quantizes the achieved
            # cadence up to a multiple of the tick period: measured on a
            # 41 ms tick, a 0.1 s request landed at 8.0 Hz and a 0.05 s
            # request at 13.6 Hz, because each send pushed the next due time
            # a full interval past a tick boundary. Advancing absolutely
            # keeps the average honest: the same two requests measure
            # 9.85 Hz and 18.49 Hz on a 29 fps tick after this change.
            next_telemetry += flow.telemetry_interval
            if next_telemetry <= now:
                # First tick (seeded 0.0), a cadence change, or a stall
                # longer than one interval. Resync instead of bursting to
                # catch up -- backlogged telemetry is worthless, and on this
                # transport a burst is exactly the queue growth /ws-thin's
                # credit scheme exists to prevent.
                next_telemetry = now + flow.telemetry_interval
            try:
                await websocket.send_text(
                    json.dumps(thin_telemetry_message(state, flow)))
            except Exception:  # noqa: BLE001 - disconnect; the receive loop wins
                return

        scene = thin_scene_for(state, flow.camera.mode)
        if scene is None:  # no fresh data for this mode yet -- send nothing
            continue

        # Credit gate (#202): a v2 flow with no credit has this tick's frame
        # dropped -- never queued -- so a slow client bounds its own queue at
        # `credits_max` frames outstanding, and slowness is not a fault (there
        # is deliberately no timeout-based eviction on this endpoint).
        if not thin_tick_may_send(flow):
            continue

        # Resolve the FPV pose for this tick (auto-orbit sweep, or a held manual
        # drag) from the server-global thin-view config, then snapshot: an orbit
        # command arriving mid-render must not mutate the state the render thread
        # is reading (#194/#215).
        flow.camera.resolve(
            getattr(getattr(state, "ui_state", None), "thin_view", None), now)
        try:
            frame = await asyncio.wrap_future(
                renderer.submit(scene, replace(flow.camera),
                               fmt=flow.format, quality=flow.quality,
                               out_width=flow.width, out_height=flow.height,
                               seq=flow.seq + 1),
                loop=loop)
        except ThinRenderUnavailable:
            raise
        except Exception as exc:  # noqa: BLE001 - one bad frame is not fatal
            flow.render_errors += 1
            if flow.render_errors <= 3:
                log.warning("thin render failed: %s: %s", type(exc).__name__, exc)
            continue

        if not thin_frame_fits(flow, len(frame)):
            continue

        try:
            await websocket.send_bytes(frame)
        except Exception:  # noqa: BLE001 - disconnect; the receive loop wins
            return
        flow.frames_sent += 1
        flow.seq += 1
        if flow.proto >= 2:
            flow.credits -= 1
        flow.tx_log.append((time.monotonic(), len(frame)))



async def websocket_thin_endpoint(websocket: WebSocket):
    """Server-rendered raster feed for a GPU-less thin client (#194).

    Deliberately its own endpoint rather than a filter on `/ws`: a thin client
    never parses POINT_CLOUD/MESH/IR_IMAGE or the full JSON surface, and this
    way the browser protocol keeps zero knowledge of it. See
    `docs/thin-client.md`.

    A renderer that cannot start is reported as JSON and closed cleanly --
    `/ws` and `/ws-mesh` clients must be entirely unaffected by it.
    """
    await websocket.accept()
    state = _get_app_state(websocket)
    flows = _thin_clients(state)

    if len(flows) >= THIN_MAX_CLIENTS:
        await websocket.send_text(json.dumps({
            "type": "error",
            "error": "thin_client_limit",
            "message": (f"at most {THIN_MAX_CLIENTS} thin clients "
                        f"are supported; {len(flows)} connected"),
        }))
        await websocket.close()
        return

    try:
        # Context creation is ~4 s the first time and blocking, so it goes to a
        # thread; every later connection gets the cached answer immediately.
        await asyncio.to_thread(ThinRenderer.instance().ensure_available)
    except ThinRenderUnavailable as exc:
        await websocket.send_text(json.dumps({
            "type": "error",
            "error": "thin_render_unavailable",
            "message": str(exc),
        }))
        await websocket.close()
        return

    # Warm the (possibly first-ever-in-this-process, dlopen-costly) `import
    # simplejpeg` HERE, off the loop -- not inline in `thin_hello` negotiation
    # (#197 review finding 4). Best-effort: any failure just means
    # `jpeg_available()` keeps reporting unavailable and negotiation degrades
    # to raw as usual; it must never affect the raw path.
    await asyncio.to_thread(warm_jpeg_import)

    flow = ThinFlow()
    flows[websocket] = flow
    if not hasattr(state, "ws_client_id"):
        state.ws_client_id = {}
    client_id = _resolve_client_id(websocket)
    state.ws_client_id[websocket] = client_id
    # A connected thin client is a live viewer exactly like a `/ws` tab --
    # cancel any pending idle and wake the sensor if it was already idled.
    # `_thin_render_loop` re-touches this every tick so the connection never
    # ages out on its own (a v1 client never sends anything after its hello).
    _touch_client_active(state, client_id)
    await _viewer_arrived(state)

    render_task = asyncio.create_task(
        _thin_render_loop(state, websocket, flow))
    try:
        while True:
            data = await websocket.receive_text()
            try:
                await _handle_thin_inbound(state, json.loads(data), flow)
            except Exception as exc:  # noqa: BLE001 - one bad command is not fatal
                log.warning("bad /ws-thin message: %r (%s)", data[:200], exc)
    except WebSocketDisconnect:
        pass
    except Exception as exc:  # noqa: BLE001
        log.warning("ws-thin receive loop error: %r", exc)
    finally:
        render_task.cancel()
        with suppress(asyncio.CancelledError, Exception):
            await render_task
        flows.pop(websocket, None)
        cid = state.ws_client_id.pop(websocket, None)
        if cid is not None:
            getattr(state, "client_active", {}).pop(cid, None)
        await _viewer_left(state)   # arm the debounced sensor idle if that was the last active viewer


__all__ = [k for k in globals() if not k.startswith("__")]
