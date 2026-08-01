"""Control the running `roomscan-web` instrument over its `/ws` channel.

These tools are a *client* of roomscan-web and never touch the device stream
themselves -- see the package docstring. Recording therefore goes through the
server's own `record` message rather than `capture.py`, which would fight it for
the UDP socket.

Message names and payloads follow docs/web-protocol.md §"Inbound".
"""
from __future__ import annotations

import asyncio
import os
import subprocess
import urllib.request

from .paths import LOGS, REPO, VENV_PY, WEB_PAGE, WEB_URL
from .server import mcp
from .session import rig

# tool kwarg -> (/ws message type, field in that message, field in the `state` echo).
# The third entry is what makes verification possible: the server broadcasts `state`
# on a timer as well as on change, so the first `state` seen after a set may predate
# it. Poll the echoed field instead of trusting arrival.
DISPLAY_SETTERS = {
    "color": ("set_color", "mode", "color_mode"),
    "ir_colormap": ("set_ir", "colormap", "ir_colormap"),
    "ir_freeze": ("set_ir", "freeze", "ir_freeze"),
    "point_size": ("set_view", "point_size", "point_size"),
    "see_through": ("set_view", "see_through", "see_through"),
    "surface": ("set_view", "surface", "surface_enabled"),
    "mode": ("set_mode", "mode", "mode"),
    "trajectory": ("slam_opt", "trajectory", "slam_trajectory"),
    "walls": ("slam_opt", "walls", "slam_walls"),
    "follow": ("slam_opt", "follow", "slam_follow"),
    "orientation": ("set_orientation", "mode", "orientation_mode"),
}


def _server_procs() -> list:
    """PIDs running roomscan.web.

    Uses psutil rather than pkill: `pkill -f roomscan.web` matches its own shell,
    a trap the firmware-loop skill calls out explicitly.
    """
    import psutil

    out = []
    for p in psutil.process_iter(["pid", "cmdline"]):
        cmd = " ".join(p.info.get("cmdline") or [])
        if "roomscan.web" in cmd or "roomscan-web" in cmd:
            out.append(p)
    return out


def _http_ok(timeout: float = 2.0) -> bool:
    try:
        with urllib.request.urlopen(WEB_PAGE, timeout=timeout) as r:
            return r.status == 200
    except Exception:
        return False


@mcp.tool()
async def rig_status(reconnect: bool = True) -> dict:
    """Whether roomscan-web is up, what it is playing, and how it is streaming.

    Merges the server's latest `state`, `metrics`, `session` and `captures`
    broadcasts. Call this first -- every other rig_* tool needs the server running.
    """
    up = _http_ok()
    procs = _server_procs()
    out = {"server_up": up, "url": WEB_URL, "pids": [p.info["pid"] for p in procs]}
    if not up:
        out["hint"] = "server not responding — call rig_up() to start it"
        return out

    if reconnect:
        out["ws_connected"] = await rig.connect()
    if not rig.connected:
        out["hint"] = "HTTP is up but /ws did not connect"
        return out

    # Give the broadcaster a beat to push at least one round of state.
    if not rig.latest:
        await rig.wait_for("state", timeout=5.0)
        await asyncio.sleep(1.0)

    state, metrics = rig.latest.get("state", {}), rig.latest.get("metrics", {})
    session = rig.latest.get("session", {})
    playback = session.get("playback") or {}
    out.update({
        "mode": state.get("mode"),
        "color": state.get("color_mode"),
        "source": session.get("source_label"),
        "is_replay": playback.get("is_replay"),
        # `position` is a 0..1 fraction, not a frame index -- resolve it so callers
        # don't have to know that, and so it matches the UI's "frame N / total".
        "playback_frame": (round(playback["position"] * playback["total_frames"])
                           if playback.get("total_frames") and
                           playback.get("position") is not None else None),
        "playback_total_frames": playback.get("total_frames"),
        "paused": playback.get("paused"),
        "recording": (session.get("recording") or {}).get("active"),
        "render_fps": metrics.get("render_fps"),
        "state": state or None,
        "metrics": metrics or None,
        "session": session or None,
        "captures": (rig.latest.get("captures") or {}).get("captures"),
        "binary_tags_seen": rig.binary_counts,
        "streaming": bool(rig.binary_counts),
    })
    return out


@mcp.tool()
async def rig_up(replay: str = "", replay_fps: float = 0.0, wait_s: float = 30.0) -> dict:
    """Start roomscan-web detached and wait until it serves the UI.

    `replay` points at a capture (repo-relative) to play instead of the live device
    -- the usual choice for verification, since it is deterministic and does not
    need the board. Returns the server's first `state` broadcast.
    """
    if _http_ok():
        return {"ok": True, "already_running": True,
                "pids": [p.info["pid"] for p in _server_procs()],
                "status": await rig_status()}

    cmd = [str(VENV_PY), "-m", "roomscan.web"]
    if replay:
        cmd += ["--replay", replay]
        if replay_fps:
            cmd += ["--replay-fps", str(replay_fps)]

    env = dict(os.environ)
    env["ROOMSCAN_NO_BROWSER"] = "1"        # headless host: nothing to open into
    LOGS.mkdir(exist_ok=True)
    log = LOGS / "mcp_web.log"
    with open(log, "ab") as fh:
        proc = subprocess.Popen(cmd, cwd=str(REPO), env=env, stdout=fh, stderr=fh,
                                stdin=subprocess.DEVNULL, start_new_session=True)

    deadline = asyncio.get_event_loop().time() + wait_s
    while asyncio.get_event_loop().time() < deadline:
        if _http_ok():
            await asyncio.sleep(1.0)
            return {"ok": True, "pid": proc.pid, "replay": replay or None,
                    "log": str(log), "status": await rig_status()}
        if proc.poll() is not None:
            tail = log.read_text(errors="replace")[-2000:] if log.exists() else ""
            return {"ok": False, "error": f"server exited with {proc.returncode}",
                    "log": str(log), "tail": tail}
        await asyncio.sleep(0.5)
    return {"ok": False, "error": f"server did not answer within {wait_s}s",
            "pid": proc.pid, "log": str(log)}


@mcp.tool()
async def rig_down() -> dict:
    """Stop roomscan-web and drop the /ws connection."""
    await rig.close()
    procs = _server_procs()
    pids = [p.info["pid"] for p in procs]
    for p in procs:
        try:
            p.terminate()
        except Exception:
            pass
    await asyncio.sleep(1.5)
    survivors = []
    for p in procs:
        try:
            if p.is_running():
                p.kill()
                survivors.append(p.info["pid"])
        except Exception:
            pass
    return {"ok": not _http_ok(), "terminated": pids, "killed": survivors}


@mcp.tool()
async def rig_command(name: str, param: int | None = None, timeout: float = 5.0) -> dict:
    """Send a device COMMAND and return its ACK.

    `name` is one of ping, calib, reinit, usecase, period, exposure, standby.
    usecase/period/exposure/standby take `param`. Returns "not available in replay"
    if the rig is playing a capture rather than talking to the board.
    """
    msg = {"type": "cmd", "name": name}
    if param is not None:
        msg["param"] = param
    ack = await rig.request(msg, expect="cmd", timeout=timeout)
    if ack is None:
        return {"ok": False, "error": f"no ACK within {timeout}s", "sent": msg}
    # An ACK arriving is not the same as the device accepting it. web.py's
    # _cmd_status emits ok | error | busy | timeout -- only "ok" is success, and
    # "timeout" in particular means the device never answered at all.
    status = ack.get("status")
    out = {"ok": status == "ok", "status": status,
           "detail": ack.get("detail"), "sent": msg, "ack": ack}
    if status == "timeout":
        out["error"] = ("device did not ACK — it is unresponsive or not streaming; "
                        "check rig_status() and consider fw_flash() to reset it")
    return out


@mcp.tool()
async def rig_record(on: bool, timeout: float = 10.0) -> dict:
    """Start or stop recording through the server, returning the capture path.

    This is the correct way to record while the UI is live: `capture.py --udp` and
    roomscan-web both bind the device stream and starve each other.
    """
    await rig.send({"type": "record", "on": on})
    # Poll rather than trusting the first `session` to arrive: the server also
    # broadcasts it on a timer, so the first one can predate the change. Observed
    # on-rig 2026-07-29 -- a recording that produced 734 clean frames was reported
    # as a failure to start.
    session = await _await_session(
        lambda s: bool((s.get("recording") or {}).get("active")) == on, timeout=timeout)
    if session is None:
        return {"ok": False, "error": f"no session update within {timeout}s"}
    rec = session.get("recording") or {}
    # While recording, `path` is the file being written; on stop the server clears
    # it and reports the finished file as `last_name`.
    active = bool(rec.get("active"))
    out = {"ok": active == on, "active": active,
           "capture": rec.get("path") or rec.get("last_name"),
           "elapsed_s": rec.get("elapsed_s"), "bytes": rec.get("bytes"),
           "recording": rec}
    if on and not active:
        # start_record() is a documented no-op unless the source is the live device.
        playback = session.get("playback") or {}
        out["error"] = ("recording did not start — "
                        + ("the rig is playing a replay, not the live device; "
                           "call rig_playback('go_live') first"
                           if playback.get("is_replay") else "see logs/app.log"))
    return out


@mcp.tool()
async def rig_set(mode: str = "", color: str = "", ir_colormap: str = "",
                  ir_freeze: bool | None = None, point_size: float | None = None,
                  see_through: float | None = None,
                  surface: bool | None = None, orientation: str = "",
                  trajectory: bool | None = None, walls: bool | None = None,
                  follow: bool | None = None, timeout: float = 5.0) -> dict:
    """Set display and mode options; only the arguments you pass are sent.

    `mode` is realtime|slam. `color` picks the point-cloud color plane.
    `see_through` (0..1, 0 = off) blends geometry hidden behind other geometry
    back over its occluder, so a near wall stops hiding the room behind it. The
    SLAM display toggles (trajectory/walls/follow) apply in slam mode. Returns the
    server's echoed `state` -- the server is authoritative, so trust that over the
    values you sent.
    """
    values = {"mode": mode, "color": color, "ir_colormap": ir_colormap,
              "ir_freeze": ir_freeze, "point_size": point_size,
              "see_through": see_through, "surface": surface,
              "orientation": orientation, "trajectory": trajectory,
              "walls": walls, "follow": follow}
    requested = {k: v for k, v in values.items() if v != "" and v is not None}
    if not requested:
        return {"ok": False, "error": "nothing to set — pass at least one option"}

    grouped: dict[str, dict] = {}
    for key, val in requested.items():
        msg_type, field, _state_field = DISPLAY_SETTERS[key]
        grouped.setdefault(msg_type, {"type": msg_type})[field] = val

    sent = []
    for msg in grouped.values():
        await rig.send(msg)
        sent.append(msg)

    expected = {DISPLAY_SETTERS[k][2]: v for k, v in requested.items()}
    state = await _await_state(expected, timeout=timeout)
    applied = {f: (state or {}).get(f) for f in expected}
    mismatched = {f: {"wanted": v, "got": applied.get(f)}
                  for f, v in expected.items() if applied.get(f) != v}
    out = {"ok": not mismatched and state is not None, "sent": sent,
           "applied": applied, "state": state}
    if mismatched:
        out["error"] = f"server state does not match what was set: {mismatched}"
    return out


async def _await_state(expected: dict, timeout: float) -> dict | None:
    """Wait until the server's `state` echo matches `expected`, not merely arrives.

    roomscan-web broadcasts `state` on a timer as well as on change, so the first
    one after a set can predate it -- which reads as a successful set that silently
    reported the old value.
    """
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout
    state = rig.latest.get("state")
    while loop.time() < deadline:
        if state and all(state.get(f) == v for f, v in expected.items()):
            return state
        state = await rig.wait_for("state", timeout=max(0.1, deadline - loop.time()))
        if state is None:
            break
    return state or rig.latest.get("state")


async def _await_session(matches, timeout: float) -> dict | None:
    """Wait for a `session` broadcast satisfying `matches`, not merely the next one."""
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout
    session = rig.latest.get("session")
    while loop.time() < deadline:
        if session and matches(session):
            return session
        session = await rig.wait_for("session", timeout=max(0.1, deadline - loop.time()))
        if session is None:
            break
    return session or rig.latest.get("session")


@mcp.tool()
async def rig_playback(action: str, value: str | float | None = None,
                       timeout: float = 10.0) -> dict:
    """Control the source: go_live, load_capture, or a transport action.

    `action="load_capture"` takes the capture name in `value`; `action="go_live"`
    takes none. Anything else is a transport action (pause, resume, speed, loop,
    restart, seek), where `value` is that action's argument.
    """
    if action == "go_live":
        msg = {"type": "go_live"}
    elif action == "load_capture":
        if not value:
            return {"ok": False, "error": "load_capture needs the capture name in `value`"}
        msg = {"type": "load_capture", "name": str(value)}
    else:
        msg = {"type": "transport", "action": action}
        if value is not None:
            msg["value"] = value
    # What the requested action should look like once it has landed. Poll for it
    # rather than reading the first `session` to arrive, which the server also
    # broadcasts on a timer and can therefore predate the change.
    settled = {
        "go_live": lambda s: not (s.get("playback") or {}).get("is_replay"),
        "load_capture": lambda s: (s.get("playback") or {}).get("capture_name") == str(value),
        "pause": lambda s: (s.get("playback") or {}).get("paused") is True,
        "resume": lambda s: (s.get("playback") or {}).get("paused") is False,
    }.get(action, lambda _s: True)

    await rig.send(msg)
    session = await _await_session(settled, timeout=timeout)
    if session is None:
        return {"ok": False, "error": f"no session update within {timeout}s", "sent": msg}

    # Arrival is not the same as effect -- go_live is silently a no-op when the
    # server was started with --replay, since it has no live source to switch to.
    pb = session.get("playback") or {}
    out = {"ok": True, "sent": msg, "source": session.get("source_label"),
           "is_replay": pb.get("is_replay"), "paused": pb.get("paused"),
           "session": session}
    if action == "go_live" and pb.get("is_replay"):
        out["ok"] = False
        out["error"] = ("still on a replay — "
                        + ("this server has no live source (started with --replay); "
                           "restart it with rig_down() then rig_up()"
                           if not session.get("has_live") else "see logs/app.log"))
    elif action == "load_capture" and pb.get("capture_name") != str(value):
        out["ok"] = False
        out["error"] = f"source is {pb.get('capture_name')!r}, expected {value!r}"
    elif action in ("pause", "resume") and pb.get("paused") is not (action == "pause"):
        out["ok"] = False
        out["error"] = f"paused={pb.get('paused')} after {action}"
    return out


@mcp.tool()
async def rig_view(source: str = "", display: str = "", regenerate: bool = False,
                   timeout: float = 15.0) -> dict:
    """Select Live/View and Point cloud/Preview/SLAM/Detailed, verifying server state.

    ``source`` is live|view; View reuses the selected capture. ``display`` is
    point_cloud|preview|slam|detailed. Preview requires the selected View
    capture. Set ``regenerate=True`` to start an explicit
    Detailed sidecar rebuild for the selected View capture. The returned state,
    not the requested value, is the result.
    """
    sent = []
    if source:
        msg = {"type": "set_source", "source": source}
        await rig.send(msg)
        sent.append(msg)
    if display:
        msg = {"type": "set_display", "display": display}
        await rig.send(msg)
        sent.append(msg)
    if regenerate:
        msg = {"type": "regenerate_detailed"}
        await rig.send(msg)
        sent.append(msg)
    if not sent:
        return {"ok": False, "error": "pass source, display, or regenerate"}
    expected = {k: v for k, v in (("source", source), ("display", display)) if v}
    state = await _await_state(expected, timeout)
    return {"ok": state is not None and all(state.get(k) == v for k, v in expected.items()),
            "sent": sent, "state": state}


@mcp.tool()
async def rig_save(timeout: float = 120.0) -> dict:
    """Save the LIVE SLAM map: full-res .ply mesh plus .tum trajectory.

    Live SLAM only -- `source == "live"` and `display == "slam"`. A live scan is
    unrepeatable, so if Record wasn't running its frames are gone the moment the
    map is dropped, which is why this one-shot export exists.

    For a recorded capture this is the wrong tool: replay SLAM is a preview, and
    the persistent artifact is the capture-keyed sidecar built by
    `rig_view(display="detailed", regenerate=True)`.
    """
    saved = await rig.request({"type": "save"}, expect="saved", timeout=timeout)
    if saved is None:
        # The server refuses with a bus line rather than a `saved` echo, so a
        # timeout here is usually "wrong source/display", not a slow write.
        state = rig.latest.get("state") or {}
        where = f"source={state.get('source')!r} display={state.get('display')!r}"
        return {"ok": False, "state": state or None,
                "error": f"no save confirmation within {timeout}s ({where}); save is "
                         "Live SLAM only -- for a capture use "
                         "rig_view(display='detailed', regenerate=True)"}
    return {"ok": True, "saved": saved}
