"""Primary /ws and /ws-mesh binary telemetry and command dispatchers."""
from __future__ import annotations

import asyncio
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
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from fastapi import WebSocket, WebSocketDisconnect
from starlette.websockets import WebSocketState

from .. import commstatus
from ..control import CommandClient, CommandDispatcher
from ..magcal import MagCalibration
from ..metrics import MetricsRegistry, MetricsSnapshot
from ..pipeline import TransformStage
from ..protocol import (
    CommandCode,
    EventCode,
    FrameHeader,
    FrameType,
    ProtocolError,
    RangingConfigAck,
    ResultCode,
    StandbyLevel,
    StreamId,
)
from ..domain import CV, Cloud
from ..sensors import (
    SensorState,
    display_frame_rotation,
    graft_yaw,
    quat_pitch_alt_deg,
    quat_pitch_deg,
    quat_roll_alt_deg,
    quat_roll_deg,
    quat_yaw_alt_deg,
    quat_yaw_deg,
    wrap180,
)
from ..sources import FileSource, Recorder, SerialSource, UdpSource
from ..thin_render import ThinRenderer
from .session import *

log = logging.getLogger("roomscan.web")

def _get_app_state(websocket: WebSocket | None = None):
    if websocket is not None and hasattr(websocket, "app") and hasattr(websocket.app, "state"):
        return websocket.app.state
    from .app import app
    return app.state

def _ws_is_connected(ws: WebSocket) -> bool:
    if getattr(ws, "closed", False):
        return False
    client_state = getattr(ws, "client_state", None)
    app_state = getattr(ws, "application_state", None)
    if client_state is not None and client_state != WebSocketState.CONNECTED:
        return False
    if app_state is not None and app_state != WebSocketState.CONNECTED:
        return False
    return True


async def _drop_client(clients: set, ws: WebSocket) -> None:
    """Remove a client and best-effort close it; never raises. Also arms the
    debounced sensor idle if that was the last active viewer (covers tabs
    that die without a clean disconnect, caught by a failed broadcast send).

    `clients` is whatever set `_broadcast_bytes`/`_broadcast_text` were handed
    for iteration -- since Phase A.5 that can be a filtered (parked-excluded)
    *copy* of `app.state.clients`, not the canonical set itself, so removal
    always also targets `app.state.clients` directly or a dead engaged tab
    would never actually leave membership."""
    clients.discard(ws)
    state = _get_app_state(ws)
    state.clients.discard(ws)
    _magcal_clients(state).discard(ws)
    cid = getattr(state, "ws_client_id", {}).pop(ws, None)
    if cid is not None:
        getattr(state, "client_active", {}).pop(cid, None)
    if _ws_is_connected(ws):
        try:
            await ws.close()
        except Exception:
            pass
    await _viewer_left(state)


def _ws_transport(ws):
    """The asyncio transport underneath a Starlette `WebSocket`, or None.

    There is no public accessor for this. uvicorn's ASGI `send` is wrapped by
    Starlette's exception middleware, so the protocol instance is reachable
    only by walking bound-method `__self__` and function closures out of
    `ws._send` until something exposes a `.transport` with a write buffer.
    Measured shape on uvicorn 0.51 / websockets 16.1: the walk lands on
    `WebSocketsSansIOProtocol`, whose transport is a `_SelectorSocketTransport`.

    Deliberately fail-open: a shape this walk cannot follow (a future uvicorn,
    the wsproto impl, a hand-built test double) returns None and every caller
    then behaves exactly as it did before #217 -- an unbounded queue is bad,
    but silently dropping every frame because we could not find a transport
    would be worse. `test_ws_backpressure.py` asserts the walk still resolves
    against a REAL uvicorn server, so a vendor bump that breaks it fails loudly
    in CI rather than quietly disabling backpressure here.
    """
    try:
        cached = _WS_TRANSPORT_CACHE.get(ws)
    except TypeError:
        # Not weak-referenceable -- a bare `object()` stand-in in a test, a
        # `__slots__` class. Skip the cache entirely rather than raising: this
        # helper runs inside `ws_flow_counters`, where an exception would take
        # out the whole metrics broadcast over a client we merely cannot
        # measure.
        cached = None
    if cached is not None:
        return cached
    seen: set[int] = set()

    def walk(obj, depth: int):
        if depth > 6 or id(obj) in seen:
            return None
        seen.add(id(obj))
        tr = getattr(obj, "transport", None)
        if tr is not None and hasattr(tr, "get_write_buffer_size"):
            return tr
        owner = getattr(obj, "__self__", None)
        if owner is not None:
            found = walk(owner, depth + 1)
            if found is not None:
                return found
        for cell in (getattr(obj, "__closure__", None) or ()):
            try:
                content = cell.cell_contents
            except ValueError:      # an empty cell (recursive closure)
                continue
            found = walk(content, depth + 1)
            if found is not None:
                return found
        return None

    try:
        tr = walk(getattr(ws, "_send", None), 0)
    except Exception:               # noqa: BLE001 - introspection must never raise
        tr = None
    if tr is not None:
        try:
            _WS_TRANSPORT_CACHE[ws] = tr
        except TypeError:
            pass        # see above: uncacheable is fine, just slower
    return tr


def ws_backlog_bytes(ws) -> int | None:
    """Bytes handed to this client's socket that have NOT reached the kernel yet,
    or None when the transport cannot be resolved.

    This is the queue BUG-061 named and never bounded on `/ws`. uvicorn's
    websocket send path is `transport.write(...)` with no drain (verified in
    `websockets_sansio_impl.py`), so `send_bytes` returns immediately and the
    surplus accrues in the asyncio transport's write buffer. Measured against a
    stalled client at 54 KB/frame: 0 -> 10.9 MB in 280 sends, still climbing.

    So the docstring on `_thin_render_loop` -- "on this stack `send_bytes` never
    blocks and queues unboundedly, so a send-derived signal would measure
    nothing" -- is right about *blocking* and wrong about *observing*: the queue
    is invisible to the send's return value but perfectly visible here.
    """
    tr = _ws_transport(ws)
    if tr is None:
        return None
    try:
        return int(tr.get_write_buffer_size())
    except Exception:               # noqa: BLE001 - transport closing mid-read
        return None


def writable_clients(clients, limit: int = WS_BACKLOG_LIMIT_BYTES):
    """`clients` minus those already holding more than `limit` unsent bytes.

    Latest-wins backpressure for the heavy per-frame payloads: a client that
    cannot drain at the rate we produce loses THIS frame rather than being
    handed one more to stack behind the others. Dropping is the correct
    behaviour for a live view -- 12 fps of now beats 30 fps of seven seconds
    ago, which is exactly what #217 reported.

    Only the heavy binary payloads are gated. Small JSON (`state`, `session`,
    `sensor`) still goes to everyone: it is a rounding error against a 54 KB
    point cloud, and skipping a `state` echo would desynchronise the one-way
    state flow. Letting the big payloads drop is also what lets the socket
    drain far enough for that JSON to move again.

    A client whose transport cannot be resolved is always writable, so an
    environment where `_ws_transport` fails behaves exactly as it did before.
    """
    return {ws for ws in clients
            if (backlog := ws_backlog_bytes(ws)) is None or backlog <= limit}


def _advance_due(due: float, now: float, interval: float) -> float:
    """Next due time for a periodic sub-cadence, advanced ABSOLUTELY (#217).

    `due + interval`, not `now + interval`. Re-anchoring to `now` on each send
    re-phases the schedule to whatever tick happened to notice it was due, so
    the achieved period becomes a multiple of the tick period rather than the
    requested interval -- a 66.7 ms (15 Hz) request on this loop's 33.3 ms tick
    lands on 3 ticks far more often than 2 and measures 11.75 Hz.

    A due time that is still in the past after one interval means the loop
    stalled longer than an interval (or this is the seeded first tick):
    resync to `now + interval` instead of bursting to catch up, since backlogged
    telemetry is worthless and a burst is the queue growth `writable_clients`
    exists to prevent. Same rule, same reason, as `_thin_render_loop`'s.
    """
    nxt = due + interval
    return nxt if nxt > now else now + interval


def _count_backpressure_drops(state, n: int) -> None:
    """Record heavy-payload sends skipped for slow clients (#217).

    A bounded queue that drops silently reads as "everything is fine" from
    every dashboard -- the same trap as a silently truncated work list. This
    surfaces in `metrics.ws.backpressure_drops`, so a client being held back is
    visible rather than merely quiet.
    """
    if n > 0:
        state.ws_backpressure_drops = getattr(state, "ws_backpressure_drops", 0) + n


async def _broadcast_bytes(clients: set, data: bytes) -> None:
    for ws in list(clients):
        if not _ws_is_connected(ws):
            await _drop_client(clients, ws)
            continue
        try:
            await ws.send_bytes(data)
        except Exception:
            await _drop_client(clients, ws)   # one dead tab must not stall the rest (§9)


async def _broadcast_telemetry_text(state, clients: set, text: str) -> None:
    """`_broadcast_text` for high-rate JSON that is latest-wins by nature (#217).

    `/ws` is one ordered socket, so bounding only the binary payloads bounds
    only the bytes -- a client can still fall behind on message COUNT, and
    everything behind the backlog is late regardless of how small it is.
    Measured after gating the binaries alone: `backlog_bytes_max` held at 0
    while a slow client still read 17 s of latency, because `sensor` (15 Hz)
    plus `metrics` (4 Hz) alone out-ran it.

    Gating is by kind, not by size. `sensor` and `metrics` are periodic
    telemetry: a dropped sample is superseded by the next one a few tens of ms
    later, exactly like a dropped frame. Edge-triggered state -- `state`,
    `session`, `saved`, `captures`, `splats`, `comm`, log lines -- is NEVER
    gated: those are sent once when something changes, and a client that missed
    one would stay wrong until the next unrelated change. That asymmetry is the
    whole reason this is a separate function rather than a flag on
    `_broadcast_text`.
    """
    engaged = writable_clients(clients)
    _count_backpressure_drops(state, len(clients) - len(engaged))
    await _broadcast_text(engaged, text)


async def _broadcast_text(clients: set, text: str) -> None:
    for ws in list(clients):
        if not _ws_is_connected(ws):
            await _drop_client(clients, ws)
            continue
        try:
            await ws.send_text(text)
        except Exception:
            await _drop_client(clients, ws)


# --- /ws-mesh: credit-gated MESH transport (BUG-061 Part A) ------------------


# --- /ws-mesh: credit-gated MESH transport (BUG-061 Part A) ------------------
#
# `/ws` pose JSON must never sit behind a whole-map MESH resend on a slow link
# -- an open-loop broadcast of a 31 MB mesh on Wi-Fi is seconds of head-of-line
# blocking, which is what turned into the reported 15 s display lag. MESH
# moves to its own socket with one-in-flight, latest-wins, ack-gated delivery:
# a client's GPU upload finishes, it acks, and only then does the server hand
# it whatever is newest -- never a queue of stale intermediate meshes. A
# client that never acks (an old cached tab, a diagnostic tool) still gets the
# latest mesh, just throttled to a bounded cadence instead of an open loop.
#
# Newness is ALWAYS by bytes-object identity, never by `mesh_seq` -- the seq
# resets to 0 on `_reset_slam`, so a seq comparison would wedge after a map
# teardown (a fresh map's first mesh has a "smaller" seq than the last one a
# flow saw). An object identity has no such wraparound.

MESH_ACK_TIMEOUT_S = 60.0        # clears an in-flight credit nothing ever acked
LEGACY_MESH_INTERVAL_S = 5.0     # bounded cadence for a client that never acks


@dataclass
class MeshFlow:
    """Per-`/ws-mesh`-client delivery state (one credit in flight, latest-wins)."""
    in_flight: bool = False
    sent_at: float = 0.0
    last_sent_obj: object = None
    acked_ever: bool = False
    last_legacy_send: float = 0.0
    sent: int = 0
    acks: int = 0
    superseded: int = 0
    ack_timeouts: int = 0
    last_ack_lag_s: float | None = None


def _mesh_clients(state) -> dict:
    """Lazily-created `{WebSocket: MeshFlow}` map, same pattern as
    `_magcal_clients` -- a hand-built `app.state` (tests) needn't know about
    it, and the broadcaster/pump/endpoint all share one dict via `state`."""
    clients = getattr(state, "mesh_clients", None)
    if clients is None:
        clients = state.mesh_clients = {}
    return clients


def _cache_latest_mesh(state, mesh_bytes: bytes) -> None:
    """Cache the newest MESH bytes for `/ws-mesh` to pump out -- replaces
    broadcasting MESH on `/ws` (BUG-061). `MeshPrep.latest()` POPS, so this is
    the only place the bytes are held for a resend.

    Any flow that had not yet sent the mesh being overwritten missed it
    entirely (it will never get sent -- `latest_mesh` just moved past it):
    that flow's `superseded` counts up. This is the number that should climb
    under load, not a growing queue."""
    prev = getattr(state, "latest_mesh", None)
    state.latest_mesh = mesh_bytes
    state.latest_mesh_seq = struct.unpack_from("<I", mesh_bytes, 4)[0]
    if prev is None:
        return
    for flow in _mesh_clients(state).values():
        if flow.last_sent_obj is not prev:
            flow.superseded += 1


async def _drop_mesh_client(state, ws) -> None:
    """Mirror `_drop_client`, but for `/ws-mesh` -- deliberately WITHOUT the
    `_viewer_left` idle coupling, since viewer-idle stays keyed to `/ws` (a
    dead mesh socket alone must not idle the sensor)."""
    _mesh_clients(state).pop(ws, None)
    getattr(state, "ws_client_id", {}).pop(ws, None)
    if _ws_is_connected(ws):
        try:
            await ws.close()
        except Exception:
            pass


async def _pump_mesh(state, now: float) -> None:
    """Run every broadcaster tick: give each `/ws-mesh` flow its one credit
    if it is free (acked, or a legacy client whose throttle window is open)
    and there is something newer than what it last got. A parked client (Phase
    A.5, correlated via `client_id`) gets skipped entirely -- zero MESH
    traffic while idled, not just BUG-061's already-bounded legacy trickle."""
    latest = getattr(state, "latest_mesh", None)
    if latest is None:
        return
    ids = getattr(state, "ws_client_id", {})
    for ws, flow in list(_mesh_clients(state).items()):
        cid = ids.get(ws)
        if cid is not None and not _client_is_active(state, cid, now):
            continue
        if flow.in_flight and now - flow.sent_at > MESH_ACK_TIMEOUT_S:
            flow.in_flight = False
            flow.ack_timeouts += 1
        if latest is flow.last_sent_obj:
            continue
        ready = (not flow.in_flight if flow.acked_ever
                 else now - flow.last_legacy_send >= LEGACY_MESH_INTERVAL_S)
        if not ready:
            continue
        if not _ws_is_connected(ws):
            await _drop_mesh_client(state, ws)
            continue
        try:
            await ws.send_bytes(latest)
        except Exception:
            await _drop_mesh_client(state, ws)
            continue
        flow.in_flight = True
        flow.sent_at = now
        flow.last_sent_obj = latest
        flow.sent += 1
        flow.last_legacy_send = now


async def _broadcast_mesh_reset(state, generation: int) -> None:
    """Text control frame on `/ws-mesh` marking a source-generation boundary
    (issue #101 step 5): every mesh client must clear its cached
    geometry/pose so a mesh already in flight for the SUPERSEDED generation
    can never render against the new source.

    Ordering: the caller (`_broadcaster`'s generation-change handling) clears
    `state.latest_mesh` BEFORE calling this, and this runs before that same
    tick's `_pump_mesh` -- so nothing queued for the old generation can be
    sent after this control frame goes out. `flow.last_sent_obj`/`in_flight`
    are reset too, so a flow that was mid-ack on the old mesh doesn't sit
    blocked for `MESH_ACK_TIMEOUT_S` before the new generation's first mesh
    can reach it."""
    msg = json.dumps({"type": "mesh_reset", "generation": generation})
    for ws, flow in list(_mesh_clients(state).items()):
        flow.in_flight = False
        flow.last_sent_obj = None
        if not _ws_is_connected(ws):
            continue
        try:
            await ws.send_text(msg)
        except Exception:
            await _drop_mesh_client(state, ws)


def ws_flow_counters(state) -> dict:
    """`/ws` + `/ws-mesh` connection/credit health (BUG-061 Part A4) -- lands
    in `metrics.ws`, and from there in `rig_status` for free. Cheap: derived
    from the two client collections already held in `state`, no I/O."""
    clients = getattr(state, "clients", None) or set()
    flows = list((getattr(state, "mesh_clients", None) or {}).values())
    lags = [f.last_ack_lag_s for f in flows if f.last_ack_lag_s is not None]
    latest = getattr(state, "latest_mesh", None)
    return {
        "clients": len(clients),
        "mesh_clients": len(flows),
        "mesh_in_flight": sum(1 for f in flows if f.in_flight),
        "legacy_mesh_clients": sum(1 for f in flows if not f.acked_ever),
        "mesh_superseded_total": sum(f.superseded for f in flows),
        "mesh_ack_lag_s_max": max(lags) if lags else None,
        "mesh_ack_timeouts_total": sum(f.ack_timeouts for f in flows),
        "latest_mesh_bytes": 0 if latest is None else len(latest),
        "thin_clients": len(getattr(state, "thin_clients", None) or {}),
        # #217. `backpressure_drops` is cumulative heavy-payload sends skipped
        # because a client was already over `WS_BACKLOG_LIMIT_BYTES`;
        # `backlog_bytes_max` is the deepest write buffer right now, which is
        # the number that should stay bounded. A rising drop count with a
        # bounded backlog is the fix working, not a fault.
        "backpressure_drops": getattr(state, "ws_backpressure_drops", 0),
        "backlog_bytes_max": max(
            [b for b in (ws_backlog_bytes(ws) for ws in clients) if b is not None],
            default=0),
    }



async def _broadcast_session(state) -> None:
    """Push a fresh `session` immediately after a state-changing control so every
    tab updates now rather than waiting for the ~4 Hz broadcaster tick. Position
    is left None here (the next tick fills it from the latest frame)."""
    ctrl = getattr(state, "controller", None)
    if ctrl is not None:
        await _broadcast_text(state.clients, json.dumps(ctrl.session_message(None, time.time())))


async def _broadcast_state(state) -> None:
    """Push the authoritative `state`, *with* the controller + Detailed context.

    Every echo must carry it. `slam_available` and `detailed` are derived from
    the loaded capture, so a context-free `_state_message(ui)` reports the
    permissive defaults (SLAM available, no sidecar) -- and because the client
    drives its disabled state purely from this echo (one-way flow), an unrelated
    colour or point-size change would silently re-enable the SLAM segments on a
    legacy stream-9-less capture and clear the stale badge."""
    await _broadcast_text(state.clients, json.dumps(_state_message(
        state.ui_state, getattr(state, "controller", None),
        getattr(state, "detailed_runner", None),
        calibration_available=getattr(getattr(state, "stage", None), "has_flatfield", False))))


async def _broadcast_captures(state, ctrl) -> None:
    """Push a fresh `captures` list, scanned OFF the event loop.

    `list_captures` now header-walks every capture for frame count, duration and
    stream-9 capability. Measured 501 ms cold over the current 25-file / 1.06 GB
    library (0.4 ms warm, cached on size+mtime) -- half a second of stalled
    broadcaster, and a just-stopped recording is guaranteed cache-cold."""
    msg = await asyncio.to_thread(build_captures_message, ctrl.captures_dir)
    await _broadcast_text(state.clients, json.dumps(msg))


async def _broadcast_splats(state) -> None:
    """Push a fresh `splats` list (scanned off the event loop) to every tab."""
    msg = await asyncio.to_thread(build_splats_message, RESULTS_DIR)
    await _broadcast_text(state.clients, json.dumps(msg))


async def _broadcast_source_videos(state) -> None:
    """Push a fresh `source_videos` list (mp4/mov + their splats), scanned off the
    event loop (a manifest scan over every splat dir)."""
    msg = await asyncio.to_thread(build_source_videos_message,
                                  _captures_dir_of(state), RESULTS_DIR)
    await _broadcast_text(state.clients, json.dumps(msg))


async def _handle_delete_captures(state, ctrl, msg: dict) -> dict:
    """`delete_captures {"names": [...], "sidecars": true}` -- **the only
    destructive action in the app**.

    Returns (and broadcasts) what was ACTUALLY deleted, not what was requested.
    A batch delete that silently drops one file is worse than one that refuses
    it loudly, so every rejection carries a reason.

    Order matters, and this is why:

      1. `names` must be a list; the batch is capped (a WS peer is untrusted).
      2. Each name resolves through `sanitize_capture_name` -- the same whole
         security surface as `load_capture`. An unresolvable entry (traversal,
         separator, non-`.bin`, already gone) is logged and **dropped**, never
         fatal to the rest of the batch.
      3. The **currently-replaying** capture is switched away from FIRST.
         Unlinking under a running replay reader gives a broken stream, not a
         clean state: on POSIX the reader's fd keeps the inode alive and it
         plays a file that no longer exists, and any later seek re-opens by
         path and fails. If there is NO live source to switch to (a
         `--replay`-launched process), that one file is refused -- there is
         nowhere safe to go.
      4. The **actively-recording** file is refused outright: the `Recorder`
         holds an open write handle, so deleting it deletes a file still being
         appended to.
      5. Only then is anything unlinked, off the event loop.

    Sidecars and thumbnails go with the capture by default (`sidecars: false`
    keeps them): they are derived artifacts of a file that no longer exists. An
    orphaned `.ply` would keep appearing in the Saved-maps list pointing at
    nothing, and an orphaned thumb is unreachable anyway (`/thumb/<name>` 404s).
    """
    ui: UiState = state.ui_state
    names = msg.get("names")
    if not isinstance(names, list):
        log.warning("delete_captures: `names` must be a list, got %r", type(names).__name__)
        state.bus.publish("delete -> malformed request")
        return {"deleted": [], "refused": [], "bytes": 0}
    if len(names) > _MAX_DELETE_BATCH:
        log.warning("delete_captures: batch of %d exceeds cap %d", len(names), _MAX_DELETE_BATCH)
        state.bus.publish(f"delete -> refused, batch larger than {_MAX_DELETE_BATCH}")
        return {"deleted": [], "refused": [], "bytes": 0}
    want_sidecars = msg.get("sidecars", True)
    if not isinstance(want_sidecars, bool):
        want_sidecars = True

    refused: list[dict] = []
    resolved: list[Path] = []
    for n in names:
        p = sanitize_capture_name(n, ctrl.captures_dir)
        if p is None:
            log.warning("delete_captures: unknown/invalid name %r", n)
            refused.append({"name": str(n)[:120], "reason": "not found"})
            continue
        if p not in resolved:
            resolved.append(p)

    rec_path = ctrl.recorder.path if ctrl.recorder.active else None
    rec_resolved = Path(rec_path).resolve() if rec_path else None
    replay_resolved = Path(ctrl.replay_path).resolve() if ctrl.replay_path else None

    deletable: list[Path] = []
    switch_away = False
    for p in resolved:
        rp = p.resolve()
        if rec_resolved is not None and rp == rec_resolved:
            refused.append({"name": p.name, "reason": "currently recording"})
            continue
        if replay_resolved is not None and rp == replay_resolved:
            if not ctrl.has_live:
                refused.append({"name": p.name, "reason": "currently playing and there is no live source to switch to"})
                continue
            switch_away = True
        deletable.append(p)

    if switch_away:
        await _stop_slam_auto_record(state, ctrl)
        await asyncio.to_thread(ctrl.switch_to_live)   # resets SLAM internally
        ui.source, ui.selected_capture = "live", None
        if ui.display in ("preview", "detailed"):
            ui.display = "point_cloud"
        ui.mode = "realtime" if ui.display == "point_cloud" else "slam"
        state.bus.publish("switched to live: the capture being played was deleted")

    def _unlink_all() -> list[dict]:
        out = []
        for p in deletable:
            res = ctrl.delete_capture(p.name, sidecars=want_sidecars)
            if res is not None:
                out.append(res)
        return out

    deleted = await asyncio.to_thread(_unlink_all)
    done = {d["name"] for d in deleted}
    for p in deletable:
        if p.name not in done:
            refused.append({"name": p.name, "reason": "delete failed"})
    total = sum(d["bytes"] for d in deleted)
    result = {"type": "deleted", "deleted": deleted, "refused": refused, "bytes": total}
    if deleted:
        state.bus.publish(f"deleted {len(deleted)} capture(s), {total} bytes freed")

    await _broadcast_text(state.clients, json.dumps(result))
    await _broadcast_captures(state, ctrl)
    saved = await asyncio.to_thread(build_saved_message, RESULTS_DIR)
    await _broadcast_text(state.clients, json.dumps(saved))
    await _broadcast_session(state)
    await _broadcast_state(state)
    return result



async def _broadcast_magcal(state) -> None:
    """Push a fresh `magcal` to the tabs with the modal open, right after a
    state-changing action rather than waiting for the next 5 Hz tick."""
    clients = _magcal_clients(state)
    if clients:
        await _broadcast_text(clients, json.dumps(_magcal_report(state)))



async def _broadcast_ranging(state) -> None:
    await _broadcast_text(state.clients, json.dumps(_ranging_message(state)))


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
    state = _get_app_state()
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
    # TX_QUEUE_STATS (EVENT code 7) event-log heartbeat filter (issue #211) --
    # same lazy-create pattern as the two above.
    tx_stats_filter = getattr(state, "tx_stats_filter", None)
    if tx_stats_filter is None:
        tx_stats_filter = state.tx_stats_filter = TxQueueStatsLogFilter()
    # Elevation readout (owner ask, 2026-07-31): the EMA that makes a ~1.2 ft
    # noise band readable, and the cached sea-level reference it is measured
    # against. Same lazy-create pattern as the two above, so a hand-built
    # state (tests) doesn't have to know about either.
    elevation = getattr(state, "elevation_smoother", None)
    if elevation is None:
        elevation = state.elevation_smoother = ElevationSmoother()
    msl = getattr(state, "msl_pressure", None)
    if msl is None:
        cfg = getattr(state, "config", None)
        msl = state.msl_pressure = MslPressure(
            getattr(cfg, "latitude", 45.014060) if cfg else 45.014060,
            getattr(cfg, "longitude", -93.245526) if cfg else -93.245526,
            refresh_s=float(getattr(cfg, "msl_refresh_s", 1800.0) or 1800.0) if cfg else 1800.0)
    # Sub-cadence schedules as absolute DUE TIMES, advanced by whole intervals
    # rather than re-anchored to `now` on each send (#217, the same defect #216
    # fixed for `/ws-thin` telemetry). `now - last_x >= INTERVAL; last_x = now`
    # quantizes the achieved rate UP to a multiple of this loop's 33.3 ms tick,
    # because every send pushes the next due time a full interval past a tick
    # boundary: IR measured 11.75 Hz against an intended 15 Hz, and `sensor` --
    # which the IMU/device rate readouts are drawn from -- the same, which is
    # why the owner read it as dropped frames. Seeded 0.0 so the first tick of
    # each fires immediately, exactly as before.
    due_ir = 0.0
    due_metrics = 0.0
    due_sensor = 0.0
    due_magcal = 0.0
    magpose_seq = 0
    next_pc = time.monotonic()   # deadline-based pacing: sleep to the next tick,
    # issue #101 step 2: sentinel that never equals a real generation (an int,
    # starting at 0), so the very first tick with a controller attached always
    # runs the reset-on-generation-change branch once (harmless -- everything
    # it resets is already at its fresh-start value).
    last_seen_generation = None

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

        ctrl = getattr(state, "controller", None)
        cur_gen = getattr(ctrl, "generation", None)
        if cur_gen != last_seen_generation:
            # issue #101 step 2: a live/replay/capture switch just landed --
            # drop every piece of source-owned broadcaster state so the FIRST
            # tick of the new generation can't show a leftover frame, a
            # colour-keyed point-cloud cache built from the old source, or
            # smoothed/jittered orientation eased in from the old source's
            # last quat. `mesh_reset` similarly clears the cached MESH and
            # tells every `/ws-mesh` flow to drop its geometry (step 5).
            last_seen_generation = cur_gen
            last_item = None
            last_pc_key = None
            last_pc_bytes = None
            smoother.reset()
            jitter.reset()
            elevation.reset()
            tx_stats_filter.reset()
            ui.ir_freeze_range = None
            state.latest_mesh = None
            # A live<->replay switch must never leave a thin client looking at
            # the previous source's geometry (#101 barrier).
            state.thin_latest_pc = None
            state.thin_latest_ir = None
            state.thin_latest_sensor = None
            if cur_gen is not None:
                await _broadcast_mesh_reset(state, cur_gen)
        ready = ui.source != "view" or _view_ready(ui, ctrl)

        # Latest-wins pull; a fresh frame ticks the device-fps render counter.
        # Each entry is generation-tagged by `_GenerationSlot` whenever a
        # `SessionController` is attached (real server + most tests); a bare
        # `queue.Queue` fed directly by `reader._run_reader` (a few tests that
        # bypass the controller) still yields the pre-#101 2-tuple, handled by
        # the `else` branch for backward compatibility.
        try:
            item = state.slot.get_nowait()
        except queue.Empty:
            pass
        else:
            if ctrl is not None and isinstance(item, tuple) and len(item) == 3:
                gen, header, outputs = item
                if gen == ctrl.generation:
                    last_item = (header, outputs)
                    metrics.tick_render(now)
                # else: a superseded reader's frame -- drop it, `last_item`
                # (already reset above on the generation change) stays as-is.
            else:
                last_item = item
                metrics.tick_render(now)

        # Reader fault: surface once, flip the fault flag, keep serving.
        if state.fault and not state.fault_reported:
            state.fault_reported = True
            err = state.fault.get("error")
            print(f"\n[FATAL] reader thread stopped: {err!r}", file=sys.stderr, flush=True)
            bus.publish(f"reader stopped: {err!r}")

        # issue #101 step 3: while View has no ready capture (browsing, or a
        # swap still in flight), emit no point-cloud / SLAM / IR data at all --
        # even though `last_item` may hold a perfectly good frame, because on
        # `set_source view` with nothing selected the reader is deliberately
        # left running against whatever it was already playing (see
        # `set_source`'s comment). Live/Splat are unaffected: `ready` is True
        # for them unconditionally.
        if ready and last_item is not None:
            header, outputs = last_item
            tof_meta = outputs.get("tof_meta")
            if tof_meta is not None:
                state.tof_meta = tof_meta
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
            grav_rot = display_frame_rotation(grav_quat)   # Rotation[CvWorld, CV] | None
            # ...unless the user picked FPV/Mirror, which ship the cloud in the
            # sensor's own frame instead. `grav_quat`/`grav_rot` themselves stay
            # gravity-true below: the IR pane's quarter-turn snap and the
            # `sensor` message are deliberately unaffected by the view mode
            # (owner: "IR view is unchanged for all but mirror", and Mirror's IR
            # flip is a client-side CSS transform, not a re-rotation).
            view_rot = view_frame_rotation(grav_rot, ui.view_mode)   # Rotation[View, CV] | None
            view_mat = None if view_rot is None else view_rot.matrix

            # POINT_CLOUD (or SURFACE) every tick (so late joiners see data
            # within ~36ms), but only in real-time mode -- SLAM mode replaces
            # the cloud with the reconstructed mesh.  Cache the packed bytes;
            # rebuild only when the frame, orientation, color mode, colormap, or
            # surface settings changed.
            if ui.display == "point_cloud":
                surf_key = ((ui.surface_enabled, ui.surface_mode,
                             ui.surface_threshold_pct) if ui.surface_enabled
                            else None)
                # `ui.thin_view.colormap` is in the key so a thin-only tone
                # change (which never touches the browser cloud) still rebuilds
                # and refreshes the `/ws-thin` stash (#215).
                thin_cmap = ui.thin_view.colormap
                key = (header.seq, ui.color_mode, ui.view_colormap, surf_key,
                       rotation_key(view_mat), thin_cmap)
                if key != last_pc_key:
                    if ui.surface_enabled:
                        pg, cg, val, tris, cov, fell_back, thin_cg = select_surface(
                            outputs, state.deproj, ui.color_mode,
                            ui.view_colormap, ui.surface_mode,
                            ui.surface_threshold_pct, thin_colormap=thin_cmap)
                        # Frame tags on (#225): `pg` is the deprojector's CV
                        # grid; only a `Cloud[View]` may reach the packer.
                        view_cloud = rotate_cloud(Cloud(pg, CV), view_rot)
                        rot = view_cloud.xyz
                        last_pc_bytes = pack_surface_cloud(
                            rot, cg, val, tris, cov)
                        # `/ws-thin` renders the *gravity-corrected* grid so
                        # its own orbit camera is the only thing moving; the
                        # thin stash carries the amber-toned copy (#215).
                        state.thin_latest_pc = (cur_gen, rot, thin_cg)
                    else:
                        # One deproject + normalize feeds two tones: the browser
                        # cloud (turbo/gray) and the thin voxel view (amber by
                        # default). The thin colours reuse the browser array when
                        # the tones match, so nothing extra is computed then.
                        pts, vn, fell_back = deproject_normalize(
                            outputs, state.deproj, ui.color_mode)
                        colors = _apply_colormap(vn, ui.view_colormap).astype(
                            np.float32, copy=False)
                        view_cloud = rotate_cloud(Cloud(pts, CV), view_rot)
                        rot = view_cloud.xyz
                        last_pc_bytes = pack_point_cloud(rot, colors)
                        thin_colors = (colors if thin_cmap == ui.view_colormap
                                       else _apply_colormap(vn, thin_cmap).astype(
                                           np.float32, copy=False))
                        # Stash for `/ws-thin` (#194): a generation-tagged
                        # reference to the arrays this tick already built. The
                        # thin loop drops a stash whose generation no longer
                        # matches.
                        state.thin_latest_pc = (cur_gen, rot, thin_colors)
                    if fell_back:
                        _log_debounced(state, bus, f"color-miss:{ui.color_mode}",
                                       f"color mode {ui.color_mode!r} unavailable this frame, showing depth")
                    last_pc_key = key
                if last_pc_bytes is not None:
                    engaged = _engaged_clients(state, now)
                    writable = writable_clients(engaged)
                    _count_backpressure_drops(state, len(engaged) - len(writable))
                    await _broadcast_bytes(writable, last_pc_bytes)
                    # Task 9: the literal "browser frames sent" counter -- see
                    # MetricsRegistry.tick_browser's docstring for why this is
                    # not the same thing as `render_fps`.
                    metrics.tick_browser(now)

            # SLAM mode (web Phase 4): ship the latest `slam` message, then
            # cache the (throttled) MESH for `/ws-mesh` to pump out. The FEED
            # is not here -- it runs on the reader thread (`make_slam_feed`) so
            # reconstruction rate is independent of this loop's health
            # (BUG-060). Poll only reads published slots. Pose goes out FIRST
            # (BUG-061 A2) and MESH no longer goes on `/ws` at all (A3) -- a
            # multi-MB mesh must never sit ahead of pose in the same socket's
            # send queue.
            slam = getattr(state, "slam_runner", None)
            if ui.display == "slam" and slam is not None:
                smsg, mesh_bytes = slam.poll(ui.slam_walls)
                if smsg is not None:
                    await _broadcast_text(clients, json.dumps(smsg))
                if mesh_bytes is not None:
                    _cache_latest_mesh(state, mesh_bytes)

            # IR_IMAGE on its own slower cadence.
            if now >= due_ir:
                due_ir = _advance_due(due_ir, now, IR_INTERVAL)
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
                    # Stash the colorized frame for `/ws-thin` (#194) --
                    # already rotated and colormapped, nothing to recompute.
                    state.thin_latest_ir = (cur_gen, rgb)
                    engaged = _engaged_clients(state, now)
                    writable = writable_clients(engaged)
                    _count_backpressure_drops(state, len(engaged) - len(writable))
                    await _broadcast_bytes(writable, pack_ir_image(rgb))
                else:
                    _log_debounced(state, bus, "ir-miss",
                                   "reflectance unavailable this frame, holding IR pane")

        # Detailed uses an offline worker, but shares the same `/ws-mesh` MESH
        # cache/pump (BUG-061 A3).  It deliberately does not consume the
        # replay reader's latest frame, and must keep reporting while replay is
        # paused (Detailed has no valid playback transport).
        detailed = getattr(state, "detailed_runner", None)
        if ui.display == "detailed" and detailed is not None:
            dmsg, mesh_bytes = detailed.poll(ui.slam_walls)
            if dmsg is not None:
                await _broadcast_text(clients, json.dumps(dmsg))
            # Detailed RE-OFFERS its retained bytes every tick (#186), so de-dupe on
            # identity: re-caching the same object at tick rate would count a
            # `superseded` miss per tick against every flow that had not yet sent it.
            # A non-identical object here is either a genuinely new mesh or the same
            # mesh after a generation reset cleared `latest_mesh` -- both need caching.
            if mesh_bytes is not None and mesh_bytes is not getattr(state, "latest_mesh", None):
                _cache_latest_mesh(state, mesh_bytes)

        # Splat build progress. Polled every tick REGARDLESS of display (a build
        # runs while the user sits on source==splat / display==point_cloud), but
        # `poll()` is a cheap file stat that returns None unless there's a throttled
        # update or a one-shot completion, so this costs ~nothing when idle.
        splat_runner = getattr(state, "splat_runner", None)
        if splat_runner is not None:
            smsg = splat_runner.poll()
            if smsg is not None:
                await _broadcast_text(clients, json.dumps(smsg))
                if smsg.get("done"):
                    await _broadcast_source_videos(state)  # refresh the just-built row
                    await _broadcast_splats(state)

        # Give every `/ws-mesh` flow its one credit if it's free and there is
        # something newer cached than what it last got (BUG-061 A3). Every
        # tick, regardless of display mode, so a flow doesn't wait on the next
        # SLAM/Detailed poll to notice an ack.
        await _pump_mesh(state, now)

        # Sensor (streams 9/10) on its own cadence; silent until 9/10 arrives.
        if now >= due_sensor:
            due_sensor = _advance_due(due_sensor, now, SENSOR_INTERVAL)
            # issue #101 step 3: no sensor data while View isn't ready --
            # `state.sensor_state` is still being fed by whatever the reader
            # is doing (possibly live), and a browsing tab must see nothing.
            if ready:
                # One outbound HTTP GET per 30 min, fire-and-forget off the loop
                # (weather.py). A box with no uplink just keeps reporting
                # msl_source="fallback"; nothing here can raise or block.
                msl.maybe_refresh()
                snap = msl.snapshot()
                smsg = build_sensor_message(state.sensor_state, state.mag_cal, jitter,
                                             orientation_mode=state.ui_state.orientation_mode,
                                             axis_labels=state.ui_state.orientation_labels,
                                             yaw_offset_deg=state.ui_state.yaw_offset_deg,
                                             ir_display_quat=smoother.held,
                                             msl_pa=snap["msl_pa"],
                                             msl_source=snap["msl_source"],
                                             msl_age_s=snap["msl_age_s"],
                                             elevation_datum_ft=state.ui_state.elevation_datum_ft,
                                             altitude_smoother=elevation,
                                             tof_meta=state.tof_meta)
                if smsg is not None:
                    # Stash for `/ws-thin` (#194, owner ask 2026-08-18):
                    # orientation and heading come from this message verbatim.
                    # Never re-derive them -- no scalar off an attitude is the
                    # bearing of an axis, and four shipped defects were exactly
                    # that mistake.
                    state.thin_latest_sensor = smsg
                    await _broadcast_telemetry_text(state, clients, json.dumps(smsg))

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
            if magcal_clients and now >= due_magcal:
                due_magcal = _advance_due(due_magcal, now, MAGCAL_INTERVAL)
                await _broadcast_text(magcal_clients, json.dumps(_magcal_report(state)))

        # Metrics + session + bus drain on the slowest cadence.
        if now >= due_metrics:
            due_metrics = _advance_due(due_metrics, now, METRICS_INTERVAL)
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
            # issue #101 step 3: "stream rows identify only capture streams" --
            # while View isn't ready, the reader may still be the live device,
            # and its rates must never reach a browsing tab.
            if ready:
                msg = build_metrics_message(snap)
                msg["transport"] = transport_counters(state)
                msg["ws"] = ws_flow_counters(state)
                await _broadcast_telemetry_text(state, clients, json.dumps(msg))
            ctrl = getattr(state, "controller", None)
            if ctrl is not None:
                pos = _replay_position(ctrl, last_item)
                await _broadcast_text(clients, json.dumps(ctrl.session_message(pos, time.time())))
            for line in bus.drain(bus_handle):
                msg = classify_bus_line(line, state.command_labels)
                if msg is not None and _should_broadcast_bus_msg(msg, tx_stats_filter, now):
                    await _broadcast_text(clients, json.dumps(msg))

            # Ranging profile / IMU-env poll rate (Task 10): measured_fps reuses
            # THIS same metrics snapshot's device_hz for the ToF stream -- "reuse
            # the existing metrics (device rate) rather than inventing a new
            # measurement" -- and the `ranging` message rides the same slow
            # cadence as `metrics`/`session` above.
            rs = getattr(state, "ranging_state", None)
            if rs is not None:
                tof_stream = next((s for s in snap.streams
                                   if s.stream_id in (StreamId.RAW_3DMD, StreamId.DEPTH_ZF32)), None)
                rs.measured_fps = tof_stream.device_hz if tof_stream is not None else None
                rs.transport = _transport_kind(ctrl)
                # Task 8: stream 9's own measured cadence -- see
                # ImuEnvRateState.measured_quat_hz's docstring. Computed the
                # same way as `rs.measured_fps` above (this snapshot's
                # per-stream device_hz), so it costs nothing extra and is
                # truthful on a replay of any recorded rate.
                ie = getattr(state, "imu_env_state", None)
                if ie is not None:
                    quat_stream = next((s for s in snap.streams
                                        if s.stream_id == StreamId.IMU_QUAT), None)
                    ie.measured_quat_hz = quat_stream.device_hz if quat_stream is not None else None
                await _broadcast_ranging(state)



async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    state = _get_app_state(websocket)
    clients: set = state.clients
    clients.add(websocket)
    if not hasattr(state, "ws_client_id"):
        state.ws_client_id = {}
    client_id = _resolve_client_id(websocket)
    state.ws_client_id[websocket] = client_id
    _touch_client_active(state, client_id)
    await _viewer_arrived(state)   # cancel any pending idle; wake the sensor if idled

    # Bring the new tab current immediately.
    try:
        await websocket.send_text(json.dumps(_state_message(
            state.ui_state, getattr(state, "controller", None),
            getattr(state, "detailed_runner", None),
            calibration_available=getattr(getattr(state, "stage", None), "has_flatfield", False))))
        ctrl = getattr(state, "controller", None)
        if ctrl is not None:
            await websocket.send_text(json.dumps(ctrl.session_message(None, time.time())))
            caps = await asyncio.to_thread(build_captures_message, ctrl.captures_dir)
            await websocket.send_text(json.dumps(caps))
        saved = await asyncio.to_thread(build_saved_message, RESULTS_DIR)
        await websocket.send_text(json.dumps(saved))
        # Splat-source picker: the built-splat list, so the Splat page is
        # populated on first paint rather than after a `list_splats` round trip.
        await websocket.send_text(json.dumps(
            await asyncio.to_thread(build_splats_message, RESULTS_DIR)))
        # Ranging profile/IMU-env state (Task 10): a second/late-joining tab must
        # see the SAME server-authoritative state immediately, not wait for the
        # next tick or an unrelated control's echo.
        if getattr(state, "ranging_state", None) is not None:
            await websocket.send_text(json.dumps(_ranging_message(state)))
        # Comm indicator: the last probe result, so the top-bar link dots are
        # populated on the first paint rather than after a full probe interval.
        last_comm = getattr(state, "last_comm", None)
        if last_comm is not None:
            await websocket.send_text(json.dumps(last_comm))
        # Camera monitor (#226): same reason as `comm` above -- the card paints
        # Offline/Connecting/Live/Stale on first frame instead of sitting blank
        # for up to a heartbeat, and a late-joining tab sees the same
        # server-authoritative state as the one already open.
        last_camera = getattr(state, "last_camera", None)
        if last_camera is not None:
            await websocket.send_text(json.dumps(last_camera))
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
        cid = state.ws_client_id.pop(websocket, None)
        if cid is not None:
            getattr(state, "client_active", {}).pop(cid, None)
        await _viewer_left(state)   # arm the debounced sensor idle if that was the last active viewer


async def websocket_mesh_endpoint(websocket: WebSocket):
    """Credit-gated MESH delivery (BUG-061 Part A). Deliberately no pairing
    with `/ws`: MESH is identical shared display state for every viewer, so
    if this socket drops, the mesh just holds stale while pose keeps flowing
    on `/ws` -- correct degradation, not an error.

    Protocol: binary MESH (tag 3) pushed at most once per outstanding credit;
    the client acks with `{"type": "mesh_ack"}` once its GPU upload finishes,
    which both frees the credit and (if a newer mesh is cached) triggers an
    immediate resend rather than waiting for the next broadcaster tick."""
    await websocket.accept()
    state = _get_app_state(websocket)
    flows = _mesh_clients(state)
    flow = MeshFlow()
    flows[websocket] = flow
    if not hasattr(state, "ws_client_id"):
        state.ws_client_id = {}
    state.ws_client_id[websocket] = _resolve_client_id(websocket)   # correlates
    # with this tab's `/ws` connection for Phase A.5 parking; the shared
    # `client_active` entry's lifecycle belongs to the `/ws` socket, not this one.

    # Late joiner: push whatever is cached right now rather than waiting for
    # the next SLAM/Detailed poll to produce something "new".
    latest = getattr(state, "latest_mesh", None)
    if latest is not None:
        try:
            await websocket.send_bytes(latest)
        except Exception:
            flows.pop(websocket, None)
            state.ws_client_id.pop(websocket, None)
            return
        now = time.monotonic()
        flow.in_flight = True
        flow.sent_at = now
        flow.last_sent_obj = latest
        flow.sent += 1
        flow.last_legacy_send = now

    try:
        while True:
            data = await websocket.receive_text()
            try:
                msg = json.loads(data)
            except Exception as exc:
                log.warning("bad inbound ws-mesh message: %r (%s)", data[:200], exc)
                continue
            if msg.get("type") != "mesh_ack":
                continue
            now = time.monotonic()
            flow.in_flight = False
            flow.acked_ever = True
            flow.acks += 1
            flow.last_ack_lag_s = now - flow.sent_at
            latest = getattr(state, "latest_mesh", None)
            if latest is not None and latest is not flow.last_sent_obj:
                try:
                    await websocket.send_bytes(latest)
                except Exception:
                    break
                flow.in_flight = True
                flow.sent_at = now
                flow.last_sent_obj = latest
                flow.sent += 1
    except WebSocketDisconnect:
        pass
    except Exception as exc:
        log.warning("ws-mesh receive loop error: %r", exc)
    finally:
        flows.pop(websocket, None)
        state.ws_client_id.pop(websocket, None)


async def _start_slam_auto_record(state, ctrl) -> bool:
    """Start Live SLAM's automatic recording, if it is enabled and possible.

    A no-op -- never an error -- when there is no controller, no live device,
    a take is already running, or `[viewer] slam_auto_record` is off. Publishes
    a bus line and re-broadcasts `session` only when it actually started one,
    so the UI's Record indicator and the log agree with reality.
    """
    if ctrl is None:
        return False
    cfg = getattr(state, "config", None)
    if not bool(getattr(cfg, "slam_auto_record", True)):
        return False
    if not await asyncio.to_thread(ctrl.start_auto_record):
        return False
    state.bus.publish("Live SLAM -> auto-recording this scan (a live scan is unrepeatable)")
    await _broadcast_session(state)
    await _broadcast_captures(state, ctrl)
    return True


async def _stop_slam_auto_record(state, ctrl) -> bool:
    """Stop Live SLAM's automatic recording if one is running. A manually
    started take is untouched. The client's existing falling-edge latch then
    pops the rename modal -- the intended ending, no new client code."""
    if ctrl is None:
        return False
    if not await asyncio.to_thread(ctrl.stop_auto_record):
        return False
    state.bus.publish("Live SLAM ended -> recording stopped")
    await _broadcast_session(state)
    await _broadcast_captures(state, ctrl)
    return True


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
        await _apply_record(state, ctrl, bool(msg.get("on")))

    elif mtype == "list_captures" and ctrl is not None:
        await _broadcast_captures(state, ctrl)

    elif mtype == "rename_capture" and ctrl is not None:
        # `from` is OPTIONAL and additive (§12). Absent => byte-identical to the
        # pre-2026-07-31 behaviour, which the post-recording modal and the
        # `rig_*` MCP tools depend on: "rename the take that just stopped".
        # Present => rename that specific capture, from the View-page browser.
        src = msg.get("from")
        if src is None:
            new_name = await asyncio.to_thread(ctrl.rename_last_recording, msg.get("name"))
        elif not isinstance(src, str):
            log.warning("rename_capture: non-string `from` %r", src)
            new_name = None
        else:
            new_name = await asyncio.to_thread(ctrl.rename_capture, src, msg.get("name"))
        if new_name is None:
            state.bus.publish("rename -> invalid name or already exists")
        await _broadcast_session(state)
        await _broadcast_captures(state, ctrl)
        await _broadcast_state(state)      # replay_path/label may have moved

    elif mtype == "delete_captures" and ctrl is not None:
        await _handle_delete_captures(state, ctrl, msg)

    elif mtype == "set_browser":
        # No PARTIAL mutation on a bad enum (the set_idle / set_orientation
        # shape): validate everything first, then apply, so a malformed message
        # leaves the persisted prefs exactly as they were.
        sort = msg.get("sort")
        view = msg.get("view")
        thumbs = msg.get("thumbs")
        if sort is not None and sort not in _VALID_BROWSER_SORTS:
            log.warning("set_browser: invalid sort %r", sort)
            return
        if view is not None and view not in _VALID_BROWSER_VIEWS:
            log.warning("set_browser: invalid view %r", view)
            return
        if thumbs is not None and not isinstance(thumbs, bool):
            log.warning("set_browser: non-bool thumbs %r", thumbs)
            return
        if sort is not None:
            ui.browser_sort = sort
        if view is not None:
            ui.browser_view = view
        if thumbs is not None:
            ui.browser_thumbs = thumbs
        _persist_ui(state)
        await _broadcast_state(state)

    elif mtype == "load_capture" and ctrl is not None:
        path = sanitize_capture_name(msg.get("name"), ctrl.captures_dir)
        if path is None:
            log.warning("load_capture: unknown/invalid name %r", msg.get("name"))
            return
        # Leaving live ends Live SLAM's auto-take (before switch_to_replay
        # closes the recorder itself, which would strand the file unnamed).
        await _stop_slam_auto_record(state, ctrl)
        # `switch_to_replay` resets SLAM itself, inside its no-reader window
        # (`_begin_new_timeline`). Resetting again out here would run AFTER the
        # new reader is already spawned and could drop its first frames.
        await asyncio.to_thread(ctrl.switch_to_replay, path)
        ui.source, ui.selected_capture = "view", path.name
        if ui.display in ("preview", "detailed"):
            ui.display = "point_cloud"
        # Derive the compat alias from `display`, like every other site that
        # moves either one -- this used to hard-code "realtime", so loading a
        # capture while the display was `slam` broadcast the contradiction
        # `mode: "realtime", display: "slam"`. `slam.js` reads `display` and
        # only falls back to `mode`, so nothing was visibly broken, but the
        # alias is exactly the kind of stale field a later reader trusts.
        ui.mode = "realtime" if ui.display == "point_cloud" else "slam"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "go_live" and ctrl is not None:
        # Stop Live SLAM's auto-take BEFORE the swap: switch_to_live tears the
        # reader down, and a recording closed by that path would never reach
        # `stop_record` -- so the file would be orphaned with no `last_name`
        # and no rename modal.
        await _stop_slam_auto_record(state, ctrl)
        await asyncio.to_thread(ctrl.switch_to_live)   # resets SLAM internally
        ui.source, ui.selected_capture = "live", None
        if ui.display == "detailed":
            ui.display = "point_cloud"
        ui.mode = "realtime" if ui.display == "point_cloud" else "slam"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "load_splat":
        # Select which built splat the Splat source renders. Validated against the
        # slugs actually on disk (basename-only, no path) so a stale/typo'd name
        # can't point the client at an arbitrary URL. Selecting one implies the
        # Splat source, so a click in the picker works from any source.
        slug = msg.get("slug") or msg.get("name")
        avail = {s["slug"] for s in await asyncio.to_thread(list_splats_on_disk, RESULTS_DIR)}
        if slug not in avail:
            log.warning("load_splat: unknown splat %r", slug)
            return
        ui.selected_splat, ui.source = slug, "splat"
        await _broadcast_state(state)

    elif mtype == "list_splats":
        await _broadcast_splats(state)

    elif mtype == "list_source_videos":
        await _broadcast_source_videos(state)

    elif mtype == "build_splat":
        # Kick off an offline splat build (subprocess-isolated) for a source video.
        # `video` is validated to a real .mp4/.mov under captures/; `params` is
        # allow-listed to SplatPreset field names (a WS peer is untrusted, and the
        # values become CLI flags). Only one build runs at a time (8 GB card).
        runner = getattr(state, "splat_runner", None)
        video = sanitize_video_name(msg.get("video"), _captures_dir_of(state))
        name = (msg.get("name") or "").strip()
        if runner is None or video is None or not name:
            log.warning("build_splat: bad request video=%r name=%r", msg.get("video"), name)
            return
        allowed = set(splat_defaults())
        params = {k: v for k, v in (msg.get("params") or {}).items()
                  if k in allowed and v is not None}
        result = await asyncio.to_thread(runner.start, video, name, params,
                                         force=bool(msg.get("force")))
        await _broadcast_text(state.clients, json.dumps({"type": "splat_build", **result}))
        await _broadcast_source_videos(state)

    elif mtype == "transport" and ctrl is not None:
        # Split by whether the action moves the read position (BUG-091):
        # `restart`/`seek` begin a new replay timeline and reset ephemeral SLAM
        # + sensor state inside the controller's no-reader window;
        # pause/resume/speed/loop do not touch the position, so they keep the
        # current map. Documented in docs/web-protocol.md.
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
        await _broadcast_state(state)

    elif mtype == "set_calibrated":
        # "Calibrated / Uncalibrated" toggle: apply the configured flat-field
        # correction or show raw sensor reflectance. Updates the live stage
        # immediately (a plain attribute the reader thread reads each frame).
        ui.calibrated = bool(msg.get("enabled", ui.calibrated))
        st = getattr(state, "stage", None)
        if st is not None:
            st.flatfield_enabled = ui.calibrated
        _persist_ui(state)
        await _broadcast_state(state)

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
        await _broadcast_state(state)

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
        if "see_through" in msg:
            try:
                st = float(msg["see_through"])
            except (TypeError, ValueError):
                log.warning("invalid set_view see_through: %r", msg.get("see_through"))
                st = None
            if st is not None and _SEE_THROUGH_RANGE[0] <= st <= _SEE_THROUGH_RANGE[1]:
                ui.see_through = st
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
        if "orbit_mode" in msg:
            if msg["orbit_mode"] not in _VALID_ORBIT_MODES:
                log.warning("invalid set_view orbit_mode: %r", msg.get("orbit_mode"))
                return
            ui.orbit_mode = msg["orbit_mode"]
            changed = True
        if "orbit_amplitude" in msg:
            try:
                a = float(msg["orbit_amplitude"])
            except (TypeError, ValueError):
                log.warning("invalid set_view orbit_amplitude: %r", msg.get("orbit_amplitude"))
                a = None
            if a is not None and _ORBIT_AMPLITUDE_RANGE[0] <= a <= _ORBIT_AMPLITUDE_RANGE[1]:
                ui.orbit_amplitude_deg = a
                changed = True
        if changed:
            _persist_ui(state)
            await _broadcast_state(state)

    elif mtype == "set_thin_view":
        # Thin-client voxel view framing/tone (#215), edited from the CrowPanel
        # modal. Every field is optional; out-of-range/invalid values are
        # dropped (the current value survives) rather than rejecting the whole
        # message, mirroring `set_view`.
        tv = ui.thin_view
        changed = False
        if "colormap" in msg:
            if msg["colormap"] not in _VALID_THIN_COLORMAPS:
                log.warning("invalid set_thin_view colormap: %r", msg.get("colormap"))
                return
            tv.colormap = msg["colormap"]
            changed = True
        for key, attr, (lo, hi) in (
                ("distance", "distance_m", _CAM_DISTANCE_RANGE),
                ("height", "height_m", _CAM_HEIGHT_RANGE),
                ("rotation", "rotation_deg", _CAM_ROTATION_RANGE),
                ("orbit_speed", "orbit_speed_deg_s", _ORBIT_SPEED_RANGE),
                ("orbit_amplitude", "orbit_amplitude_deg", _ORBIT_AMPLITUDE_RANGE),
                ("point_size", "point_size", _THIN_POINT_SIZE_RANGE)):
            if key not in msg:
                continue
            try:
                v = float(msg[key])
            except (TypeError, ValueError):
                log.warning("invalid set_thin_view %s: %r", key, msg.get(key))
                continue
            if lo <= v <= hi:
                setattr(tv, attr, v)
                changed = True
        if "orbit" in msg:
            tv.orbit_enabled = bool(msg["orbit"])
            changed = True
        if "point_size_auto" in msg:
            tv.point_size_auto = bool(msg["point_size_auto"])
            changed = True
        if changed:
            _persist_ui(state)
            await _broadcast_state(state)

    elif mtype == "set_source" and ctrl is not None:
        source = msg.get("source")
        if source not in _VALID_SOURCES:
            log.warning("invalid set_source: %r", source)
            return
        # Leaving Live SLAM by switching source ends its auto-take, and must do
        # so before switch_to_replay stops the recorder behind our back (which
        # would strand the file without a `last_name` for the rename modal).
        await _stop_slam_auto_record(state, ctrl)
        if source == "splat":
            # Splat is static geometry the client draws -- no device/replay reader
            # to swap. The live/replay reader is left exactly as it was (as the
            # empty-View branch does), so returning here is instant. The client
            # hides the point cloud / SLAM objects and shows the loaded splat.
            ui.source = "splat"
            ui.display, ui.mode = "point_cloud", "realtime"
            await _reset_slam(state)               # drop any live SLAM GPU work
            await _broadcast_splats(state)         # populate the picker
            await _broadcast_source_videos(state)  # populate the capture viewer
            await _broadcast_state(state)
            return
        # Both swapping branches reset SLAM inside the controller's no-reader
        # window, so there is no `_reset_slam` after this block -- see
        # `_begin_new_timeline`. The one path that reaches here without swapping
        # is an unresolvable `selected_capture`, and that leaves the reader
        # untouched, so leaving its map alone is the consistent answer.
        if source == "live":
            await asyncio.to_thread(ctrl.switch_to_live)
            ui.source, ui.selected_capture = "live", None
        elif ui.selected_capture:
            path = sanitize_capture_name(ui.selected_capture, ctrl.captures_dir)
            if path is not None:
                await asyncio.to_thread(ctrl.switch_to_replay, path)
            ui.source = "view"
        else:
            # View with nothing selected is a legitimate state: it is the
            # capture BROWSER (§12). Before the browser existed this returned
            # early with "select a capture first" -- which became a dead end the
            # moment the library moved onto the View page, since the only way to
            # select a capture is a card that only renders when source == view.
            # So the page opens; the reader is deliberately left alone (still
            # live, `is_replay` false, so the Playback card stays hidden) until
            # a `load_capture` from the browser actually swaps it.
            ui.source = "view"
            state.bus.publish("View -> pick a capture from the browser")
            await _broadcast_state(state)
            return
        ui.display, ui.mode = "point_cloud", "realtime"
        await _broadcast_session(state)
        await _broadcast_state(state)

    elif mtype == "set_display":
        display = msg.get("display")
        if display not in _VALID_DISPLAYS:
            log.warning("invalid set_display: %r", display)
            return
        if display == "preview":
            # issue #101 step 3: Preview is View-only, and readiness means the
            # ONE predicate -- not just "a name is selected", which used to
            # pass even mid-swap, before `replay_path` had caught up.
            if not _view_ready(ui, ctrl):
                state.bus.publish("Preview -> load a capture in View first")
                return
        elif display != "point_cloud":
            # SLAM/Detailed while browsing View (no capture, or a swap still
            # in flight) must refuse, same as Preview -- but this must NOT
            # reject Live SLAM (`ui.source == "live"`), which is a real,
            # separate feature `_view_ready` says nothing about.
            if ui.source == "view" and not _view_ready(ui, ctrl):
                state.bus.publish(f"{display} -> select a capture in View first")
                return
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
            # Detailed owns its own immutable offline reader.  Keeping the
            # replay pacer moving here makes the playback card look active even
            # though replay frames cannot affect the Detailed map.
            ctrl.pause()
            runner = getattr(state, "detailed_runner", None)
            if runner is not None:
                runner.begin_load_cached(ctrl.replay_path)
        # Auto-record the scan (owner ask, 2026-07-31): entering Live SLAM
        # starts a take, leaving it stops one -- but only the take Live SLAM
        # itself started (see SessionController._auto_recording). Deliberately
        # after set_active, so a scan is only recorded once the pipeline it is
        # recording for is actually armed.
        if display == "slam" and ui.source == "live":
            await _start_slam_auto_record(state, ctrl)
        elif ui.source == "live":
            await _stop_slam_auto_record(state, ctrl)
        await _broadcast_state(state)
        if display == "detailed":
            runner = getattr(state, "detailed_runner", None)
            if runner is not None:
                status = runner.status()
                if status is not None:
                    await _broadcast_text(state.clients, json.dumps(status))

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
        # issue #101 step 3: same View-readiness gate as `set_display` -- this
        # legacy alias is still how the rig_* MCP tools arm SLAM, and it must
        # not bypass the check `set_display` enforces.
        if mode == "slam" and ui.source == "view" and not _view_ready(ui, ctrl):
            state.bus.publish("slam -> select a capture in View first")
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
        # Same auto-record rule as `set_display` -- this alias is still how the
        # rig_* MCP tools enter SLAM, and a scan started that way is just as
        # unrepeatable.
        if mode == "slam" and ui.source == "live":
            await _start_slam_auto_record(state, ctrl)
        else:
            await _stop_slam_auto_record(state, ctrl)
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
        await _broadcast_state(state)

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
            await _broadcast_state(state)

    elif mtype == "idle_state":
        # Client-side activity heartbeat/park signal (Phase C, `idle.js`).
        # `active: true` -- sent on user activity and as a periodic heartbeat
        # while a human is actually there -- refreshes this connection's
        # activity clock and fast-paths a wake if it was the one bringing the
        # active count back above zero. `active: false` parks immediately
        # rather than waiting out `sensor_idle_activity_timeout_s`, and
        # fast-paths the idle decision instead of waiting for the next
        # `_idle_reconcile_loop` tick. No `_broadcast_state` -- this is
        # per-connection bookkeeping, not shared UI state.
        cid = getattr(state, "ws_client_id", {}).get(ws) if ws is not None else None
        if cid is not None:
            if bool(msg.get("active", True)):
                _touch_client_active(state, cid)
                await _viewer_arrived(state)
            else:
                _mark_client_inactive(state, cid)
                await _viewer_left(state)

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
            await _broadcast_state(state)

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
        await _broadcast_state(state)

    elif mtype == "clear_yaw_offset":
        # Explicit reset back to the raw (un-offset) SFLP yaw.
        ui.yaw_offset_deg = 0.0
        _persist_ui(state)
        await _broadcast_state(state)

    elif mtype == "set_elevation_datum":
        # Delta button (owner ask, 2026-07-31): capture the CURRENT smoothed
        # elevation as a datum, so the Sensors card reads change-since instead
        # of an absolute height. Same shape as zero_yaw/clear_yaw_offset above
        # -- presentation-only, persisted, echoed back as the single source of
        # the button's pressed state (one-way flow; the client never toggles
        # itself). Deliberately captures the SMOOTHED value, not the raw one:
        # a datum taken from a single sample would bake in ~1.2 ft of
        # barometer noise as a constant offset for the whole session -- the
        # exact mistake BUG-037 found in the SLAM height datum.
        on = msg.get("on")
        if not isinstance(on, bool):
            log.warning("invalid set_elevation_datum: %r (expected a bool `on`)", on)
            return
        if on:
            smoother = getattr(state, "elevation_smoother", None)
            current_ft = smoother.value_ft if smoother is not None else None
            if current_ft is None:
                state.bus.publish("elevation datum -> no barometer reading yet")
                return
            ui.elevation_datum_ft = round(float(current_ft), 1)
        else:
            ui.elevation_datum_ft = None
        _persist_ui(state)
        await _broadcast_state(state)

    elif mtype == "save":
        # Save survives ONLY for Live SLAM (owner decision, 2026-07-31). A live
        # scan is unrepeatable -- if Record wasn't running, the frames are gone
        # the moment the map is dropped -- so the one-shot export has to stay.
        # Replay SLAM is explicitly preview-only (re-run it, or build Detailed,
        # which is the capture-keyed sidecar), and Detailed writes its own.
        slam = getattr(state, "slam_runner", None)
        if ui.display != "slam" or slam is None:
            state.bus.publish("save -> not available (switch the display to SLAM first)")
            return
        if ui.source != "live":
            state.bus.publish("save -> replay SLAM is a preview; use Detailed to write a sidecar")
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
        saved = await asyncio.to_thread(build_saved_message, RESULTS_DIR)
        await _broadcast_text(state.clients, json.dumps(saved))

    elif mtype == "reset_fusion":
        state.sensor_state.reset_fusion()
        state.bus.publish("heading fusion reset")

    elif mtype == "set_profile":
        pid = _STR_TO_PROFILE_ID.get(msg.get("profile"))
        if pid is None:
            log.warning("set_profile: unknown profile %r", msg.get("profile"))
            return
        await _set_profile(state, int(pid))

    elif mtype == "set_manual_params":
        await _set_manual_params(
            state,
            ranging_mode=msg.get("ranging_mode"),
            fps=msg.get("fps"),
            exposure_ms=msg.get("exposure_ms"),
            power_mode=msg.get("power_mode"),
        )

    elif mtype == "set_imu_env_rate":
        rate = msg.get("rate_hz", 0)
        if not isinstance(rate, int) or isinstance(rate, bool):
            log.warning("set_imu_env_rate: non-integer rate_hz %r", rate)
            return
        await _set_imu_env_rate(state, int(rate))

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


__all__ = [k for k in globals() if not k.startswith("__")]
