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

from .paths import HOST, LOGS, REPO, VENV_PY, WEB_PAGE, WEB_URL
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
    # Ranging profile + IMU/env poll rate: what the sensor is actually configured
    # to do is part of "what is this rig doing right now", so it belongs here
    # rather than only in rig_profile()/rig_imu_env_rate().
    ranging = rig.latest.get("ranging") or {}
    imu_env = ranging.get("imu_env") or {}
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
        # `applied` is null until a real device readback -- never a guessed default.
        "ranging_profile": (ranging.get("applied") or {}).get("profile"),
        "ranging_measured_fps": ranging.get("measured_fps"),
        "imu_env_rate_hz": imu_env.get("applied_rate_hz"),
        "imu_env_coupled": imu_env.get("coupled"),
        "ranging": ranging or None,
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


# Per-command minimum wait, keyed by `resolve_command()`'s command NAME (not its
# label -- "standby 1"/"standby 0" share one entry). Only `standby` needs one
# today: SET_STANDBY powers the ToF laser down/up and did not ACK within the
# 5s default on-rig, but did within 20s (#171) -- the 5s default made a working
# command look broken. Explicit `timeout=` still overrides this either way.
_DEFAULT_COMMAND_TIMEOUT_S = 5.0
_COMMAND_TIMEOUT_S = {"standby": 20.0}


@mcp.tool()
async def rig_command(name: str, param: int | None = None,
                      timeout: float | None = None) -> dict:
    """Send a device COMMAND and return its ACK, matched to the command sent.

    `name` is one of ping, calib, reinit, usecase, period, exposure, standby.
    usecase/period/exposure/standby take `param`. Returns "not available in replay"
    if the rig is playing a capture rather than talking to the board.

    `timeout` defaults to 5s, except `standby` (20s) -- SET_STANDBY powers the
    ToF laser down/up and legitimately takes longer to ACK than a plain command
    (#171: it did not ACK within 5s but did within 20s). Pass an explicit value
    to override either default.

    The ACK is matched by `resolve_command()`'s label (e.g. "standby 1", "ping")
    to the command THIS call sent -- never merely the next `cmd` broadcast the
    bus produces. Without that check, a differently-named command's late ACK
    (still in flight from an earlier, already-timed-out call) can arrive while
    this call is waiting and get reported as THIS call's result: #171 saw a
    timed-out `standby` call followed by a `ping` call that came back
    `ok: true` carrying the standby ACK. A `cmd` broadcast seen with a
    non-matching label while waiting is therefore never treated as this call's
    answer; it is reported under `unmatched_acks` instead of being silently
    dropped (a real result vanishing would just be a different flavour of the
    same bug) -- this session keeps no per-caller queue, so it cannot be
    re-delivered to its rightful caller, only surfaced.

    Label matching alone cannot disambiguate two in-flight commands that share
    a label (e.g. two `ping`s close together) -- the wire has no per-request id.
    That is a real remaining gap; the fix here targets the reported cross-command
    misattribution, not that same-label case.
    """
    from roomscan.web import resolve_command

    msg = {"type": "cmd", "name": name}
    if param is not None:
        msg["param"] = param
    resolved = resolve_command(name, param if param is not None else 0)
    label = resolved[2] if resolved is not None else name
    wait_s = timeout if timeout is not None else _COMMAND_TIMEOUT_S.get(
        name, _DEFAULT_COMMAND_TIMEOUT_S)

    await rig.send(msg)
    loop = asyncio.get_event_loop()
    deadline = loop.time() + wait_s
    unmatched: list[dict] = []
    ack: dict | None = None
    while loop.time() < deadline:
        candidate = await rig.wait_for("cmd", timeout=max(0.0, deadline - loop.time()))
        if candidate is None:
            break
        if candidate.get("label") == label:
            ack = candidate
            break
        unmatched.append(candidate)

    if ack is None:
        out = {"ok": False, "error": f"no ACK within {wait_s}s", "sent": msg}
        if unmatched:
            out["unmatched_acks"] = unmatched
            out["error"] += (f" (saw {len(unmatched)} ACK(s) for a different command "
                             "while waiting -- a late reply to an earlier call, not "
                             "this one)")
        return out
    # An ACK arriving is not the same as the device accepting it. web.py's
    # _cmd_status emits ok | error | busy | timeout -- only "ok" is success, and
    # "timeout" in particular means the device never answered at all.
    status = ack.get("status")
    out = {"ok": status == "ok", "status": status,
           "detail": ack.get("detail"), "sent": msg, "ack": ack}
    if unmatched:
        out["unmatched_acks"] = unmatched
    if status == "timeout":
        out["error"] = ("device did not ACK — it is unresponsive or not streaming; "
                        "check rig_status() and consider fw_flash() to reset it")
    return out


async def _await_tof_live(timeout: float, threshold_hz: float = 10.0
                          ) -> tuple[bool, float | None]:
    """Wait for the ToF stream to resume above `threshold_hz`, proving the laser
    woke. When idled, stream 7's device_hz is null/0; awake it is ~fps. Returns
    (live, measured_hz)."""
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout

    def _tof_hz(m: dict | None) -> float | None:
        for s in (m or {}).get("streams") or []:
            if s.get("stream_id") == 7:  # metrics labels stream 7 "ToF"
                return s.get("device_hz") or s.get("host_hz")
        return None

    hz = _tof_hz(rig.latest.get("metrics"))
    while loop.time() < deadline:
        if hz is not None and hz > threshold_hz:
            return True, hz
        m = await rig.wait_for("metrics", timeout=max(0.1, deadline - loop.time()))
        if m is None:
            break
        hz = _tof_hz(m)
    return (hz is not None and hz > threshold_hz), hz


@mcp.tool()
async def rig_idle(auto_idle: bool | None = None, level: str = "",
                   wake: bool = True, timeout: float = 12.0) -> dict:
    """Wake the ToF sensor and/or control the laser-wear auto-idle.

    The server idles the ToF LASER (SET_STANDBY) when no browser tab is actively
    watching, and the firmware self-wakes only on real MOTION. So on a STATIC scene
    the laser parks and stays parked: a recording then captures only IMU (RAW_3DMD
    stops, the IMU streams drop to the ~18 Hz parked rate) while `capture_analyze`
    continuity still reads `loss 0%`, because an absent frame never gets a seq slot.
    This is the host-side wake a real viewer triggers on focus, exposed for headless
    work -- the device command channel's SET_STANDBY(ACTIVE) does not reliably ACK
    from cold standby, but `idle_state{active:true}` -> `_viewer_arrived` does.

    `wake=True` (default) sends the same `idle_state{active:true}` a viewer sends:
    it wakes the laser if idled AND marks this connection active so it does not
    immediately re-idle. `auto_idle` True/False persistently enables/disables the
    whole auto-idle feature (persisted to roomscan.toml, shared with the live UI) --
    set it False before recording a static scene, True to restore. `level` is
    soft|hard (idle depth). Verified against the device: a wake waits for ToF frames
    to resume (stream 7 device_hz), an `auto_idle` change waits for the `state` echo
    to carry it. `ok=false` on an unreachable server, a bad `level`, an `auto_idle`
    that did not take, or ToF not resuming within `timeout` (a replay, or genuinely
    down).
    """
    if not await rig.connect():
        return {"ok": False,
                "error": "cannot reach roomscan-web — call rig_status()/rig_up() first"}
    if level and level not in ("soft", "hard"):
        return {"ok": False, "error": f"level must be soft|hard, got {level!r}"}
    if auto_idle is None and not level and not wake:
        return {"ok": False, "error": "nothing to do — pass auto_idle, level, or wake"}

    sent: list[dict] = []
    idle_state_msg: dict | None = None
    # 1) Auto-idle feature control (persisted, shared with the UI).
    if auto_idle is not None or level:
        msg: dict = {"type": "set_idle"}
        if auto_idle is not None:
            msg["enabled"] = bool(auto_idle)
        if level:
            msg["level"] = level
        await rig.send(msg)
        sent.append(msg)
        expected: dict = {}
        if auto_idle is not None:
            expected["idle_enabled"] = bool(auto_idle)
        if level:
            expected["idle_level"] = level
        idle_state_msg = await _await_state(expected, timeout=timeout)

    # 2) Wake now (also refreshes this connection's activity so it won't re-idle).
    tof_hz = None
    woke = None
    if wake:
        wmsg = {"type": "idle_state", "active": True}
        await rig.send(wmsg)
        sent.append(wmsg)
        woke, tof_hz = await _await_tof_live(timeout=timeout)

    st = idle_state_msg or rig.latest.get("state") or {}
    out: dict = {"ok": True, "sent": sent,
                 "idle_enabled": st.get("idle_enabled"),
                 "idle_level": st.get("idle_level")}
    if wake:
        out["tof_live"] = bool(woke)
        out["tof_hz"] = tof_hz
        if not woke:
            out["ok"] = False
            out["error"] = (f"ToF did not resume within {timeout}s — the rig may be on a "
                            "replay (rig_playback('go_live')) or the device is not streaming "
                            "(rig_status()).")
    if auto_idle is not None and st.get("idle_enabled") != bool(auto_idle):
        out["ok"] = False
        out["error"] = (f"auto-idle did not change to {auto_idle} "
                        f"(state says {st.get('idle_enabled')})")
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


# --- ranging profile / IMU-env poll rate (plan Task 11) ----------------------
#
# Both halves ride ONE outbound message (`ranging`, docs/web-protocol.md), which
# the server also re-broadcasts on its ~4 Hz metrics tick. So the same trap
# `_await_state` documents applies twice over: the cached `ranging`, and even a
# freshly-arrived one, can predate the command. Verification here is therefore
# "saw our half go pending, then saw it settle onto the config we asked for" --
# never "a message arrived", and never the log line the UI prints.

_APPLIED_FIELDS = ("ranging_mode", "fps", "exposure_ms", "power_mode")


def _ranging_half(msg: dict, half: str) -> tuple[bool, str | None]:
    """(pending, error) for `half` -- "profile" or "imu_env" -- of one `ranging`
    message. The two are INDEPENDENT pending commands, so a waiter that read the
    whole message would settle on the other control's echo."""
    if half == "imu_env":
        ie = msg.get("imu_env") or {}
        return bool(ie.get("pending")), ie.get("error")
    return bool(msg.get("pending")), msg.get("error")


async def _await_ranging(half: str, matches, timeout: float,
                         prior_error: str | None) -> tuple[dict | None, str]:
    """Wait for `half` to go pending and then settle, returning (message, verdict).

    `verdict` is "ok" (settled onto a config `matches` accepted), "error" (the
    server/device reported a failure for this half), or "timeout".

    `prior_error` is that half's error BEFORE the command was sent: a metrics-tick
    broadcast can slip between the send and the server acting on it, carrying the
    previous command's error, and reporting that as this command's result would be
    a lie. An unchanged error string is therefore ignored until the pending echo
    proves the server has started our command.
    """
    loop = asyncio.get_event_loop()
    deadline = loop.time() + timeout
    saw_pending = False
    last: dict | None = None
    while loop.time() < deadline:
        msg = await rig.wait_for("ranging", timeout=max(0.1, deadline - loop.time()))
        if msg is None:
            break
        last = msg
        pending, error = _ranging_half(msg, half)
        if error and (saw_pending or error != prior_error):
            return msg, "error"
        if pending:
            saw_pending = True
            continue
        if saw_pending and matches(msg):
            return msg, "ok"
    return last, "timeout"


def _ranging_result(msg: dict | None, requested: dict) -> dict:
    """The common answer shape: what was asked for, what the DEVICE says is
    applied, what the model predicts, and what is actually being measured."""
    msg = msg or {}
    estimate = msg.get("estimate") or {}
    warnings = list(estimate.get("warnings") or [])
    if estimate.get("transport_warning") and estimate["transport_warning"] not in warnings:
        warnings.append(estimate["transport_warning"])
    imu_env = msg.get("imu_env") or {}
    if imu_env.get("warning"):
        warnings.append(imu_env["warning"])
    return {
        "requested": requested,
        "applied": msg.get("applied"),
        "imu_env": imu_env or None,
        "estimate": estimate or None,
        # The honest rate: what the model says this config DELIVERS (not what it
        # was asked for), beside what the link is actually seeing right now.
        "expected_delivered_fps": estimate.get("expected_delivered_fps"),
        "measured_fps": msg.get("measured_fps"),
        "transport": msg.get("transport"),
        "warnings": warnings,
        "initialized": msg.get("initialized"),
        "ranging": msg or None,
    }


async def _ranging_preflight(half: str, timeout: float = 5.0) -> tuple[dict | None, str | None]:
    """A FRESH `ranging` message plus the reason not to send, if there is one.

    Fresh, not cached: the cached message can be arbitrarily old, and "is another
    command already in flight?" is exactly the question a stale answer gets wrong.
    """
    if not await rig.connect():
        return None, "cannot reach roomscan-web — call rig_status()/rig_up() first"
    msg = await rig.wait_for("ranging", timeout=timeout)
    if msg is None:
        return None, ("no `ranging` broadcast within "
                      f"{timeout}s — is roomscan-web running with a device?")
    if msg.get("transport") == "replay":
        return msg, "ranging control is unavailable in replay — rig_playback('go_live') first"
    pending, _error = _ranging_half(msg, half)
    if pending:
        return msg, (f"busy: a {half} command is already pending on the server "
                     "(wait for it, then retry)")
    return msg, None


@mcp.tool()
async def rig_profile(profile: str = "", ranging_mode: str = "", fps: int = 0,
                      exposure_ms: int = 0, power_mode: str = "", force: bool = False,
                      timeout: float = 20.0) -> dict:
    """Read or set the ranging profile, verified against the device's own readback.

    With no arguments this QUERIES: it returns the server's current ranging state
    (applied config, model estimate, measured fps, transport, IMU/env rate).

    To set, pass either `profile` — stability|precision|high_framerate, or
    `manual` to reapply the device's last accepted manual candidate — or all four
    manual fields: `ranging_mode` (ambient|precision), `fps` 1-100, `exposure_ms`
    1-16, `power_mode` (ulp|lp|regular). Manual candidates are validated
    host-side first, so an invalid one never reaches the device.

    `ok=true` means the DEVICE read back the configuration that was asked for --
    this waits for the `ranging` echo to go pending and then settle onto a
    matching `applied`, because that message is also re-broadcast on a timer and
    a merely newly-arrived one proves nothing. `ok=false` covers a busy server, a
    replay source, host-side validation failure, a device error (BUSY, BAD_PARAM,
    a timeout), an unsupported CDC rate, and an applied-vs-requested mismatch.

    Above 60 fps over USB CDC is refused before the device is touched; pass
    `force=True` to send it anyway (Task 12 exercises exactly that case) and the
    warning comes back in `warnings`. Leave ≥2 s between reconfigurations: a
    faster one can produce no ACK at all (BUG-073).

    Read `expected_delivered_fps` rather than the requested `fps` — above an
    exposure's measured 1x ceiling the sensor accepts the request and delivers
    period-multiples. `profile_estimate()` answers the same question offline.
    """
    from roomscan import profiles

    manual_given = {"ranging_mode": ranging_mode, "fps": fps,
                    "exposure_ms": exposure_ms, "power_mode": power_mode}
    given = {k: v for k, v in manual_given.items() if v}

    # -- query
    if not profile and not given:
        if not await rig.connect():
            return {"ok": False, "error": "cannot reach roomscan-web — call rig_up() first"}
        msg = await rig.wait_for("ranging", timeout=5.0) or rig.latest.get("ranging")
        if msg is None:
            return {"ok": False, "error": "no `ranging` state from the server"}
        out = _ranging_result(msg, {"kind": "query"})
        out["ok"] = bool(msg.get("initialized"))
        if not out["ok"]:
            out["error"] = ("the server has no device readback yet (`initialized: false`) — "
                            "it never guesses a firmware default")
        return out

    if profile and profile not in profiles.STR_TO_PROFILE_ID:
        return {"ok": False, "error": f"unknown profile {profile!r}; expected one of "
                                      f"{sorted(profiles.STR_TO_PROFILE_ID)}"}
    if given and profile and profile != "manual":
        return {"ok": False, "error": f"profile={profile!r} is a preset; drop the manual "
                                      "fields, or pass profile='' with all four of them"}

    is_manual_params = bool(given)
    if is_manual_params:
        missing = [k for k, v in manual_given.items() if not v]
        if missing:
            return {"ok": False, "error": "a manual change needs all four of "
                                          f"ranging_mode/fps/exposure_ms/power_mode; missing {missing}"}
        rm = profiles.STR_TO_RANGING_MODE.get(ranging_mode)
        pm = profiles.STR_TO_POWER_MODE.get(power_mode)
        if rm is None or pm is None:
            return {"ok": False, "error":
                    f"ranging_mode must be one of {sorted(profiles.STR_TO_RANGING_MODE)} and "
                    f"power_mode one of {sorted(profiles.STR_TO_POWER_MODE)}; "
                    f"got {ranging_mode!r}/{power_mode!r}"}
        requested = {"kind": "manual", "ranging_mode": ranging_mode, "fps": int(fps),
                     "exposure_ms": int(exposure_ms), "power_mode": power_mode}
        expected = dict(requested)
        expected.pop("kind")
        target_fps: int | None = int(fps)
    else:
        pid = profiles.STR_TO_PROFILE_ID[profile]
        requested = {"kind": "profile", "profile": profile}
        if pid is profiles.ProfileId.MANUAL:
            # Reapplies whatever candidate the device last accepted, which the
            # host does not know -- so the readback IS the expectation here.
            expected, target_fps = None, None
        else:
            preset = profiles.PRESETS[pid]
            expected = {"ranging_mode": profiles.RANGING_MODE_TO_STR[preset.ranging_mode],
                        "fps": preset.fps, "exposure_ms": preset.exposure_ms,
                        "power_mode": profiles.POWER_MODE_TO_STR[preset.power_mode]}
            target_fps = preset.fps
            requested.update(expected)

    pre, blocked = await _ranging_preflight("profile")
    if blocked:
        out = _ranging_result(pre, requested)
        out.update({"ok": False, "error": blocked, "sent": None})
        return out
    transport = (pre or {}).get("transport") or "none"

    # Host-side validation BEFORE the device is touched -- a request this model
    # rejects should never have been sent (and the device would reject it too).
    if is_manual_params:
        params = profiles.ManualParams(rm, int(fps), int(exposure_ms), pm)
        validation = profiles.validate_manual_params(params)
        if not validation.ok:
            out = _ranging_result(pre, requested)
            out.update({"ok": False, "sent": None, "error": "; ".join(validation.errors),
                        "warnings": list(validation.warnings)})
            return out

    if target_fps is not None and not force:
        tw = profiles.transport_warning_message(transport, target_fps)
        if tw:
            out = _ranging_result(pre, requested)
            out.update({"ok": False, "sent": None, "error": tw + " Pass force=True to "
                        "apply it anyway.", "warnings": [tw]})
            return out

    if is_manual_params:
        sent = {"type": "set_manual_params", "ranging_mode": ranging_mode, "fps": int(fps),
                "exposure_ms": int(exposure_ms), "power_mode": power_mode}
    else:
        sent = {"type": "set_profile", "profile": profile}

    def matches(msg: dict) -> bool:
        applied = msg.get("applied")
        if applied is None:
            return False
        if expected is None:
            return True     # MANUAL reapply: the device chose; the readback is the answer
        return all(applied.get(f) == expected[f] for f in _APPLIED_FIELDS)

    prior_error = (pre or {}).get("error")
    await rig.send(sent)
    msg, verdict = await _await_ranging("profile", matches, timeout, prior_error)

    out = _ranging_result(msg, requested)
    out["sent"] = sent
    out["ok"] = verdict == "ok"
    if verdict == "error":
        out["error"] = _ranging_half(msg or {}, "profile")[1]
    elif verdict == "timeout":
        applied = (msg or {}).get("applied")
        out["error"] = (f"no confirmed readback within {timeout}s "
                        f"(last applied: {applied}); the command may still be in flight, "
                        "or another one was already pending")
        last_error = _ranging_half(msg or {}, "profile")[1]
        if last_error:
            out["last_error"] = last_error
    elif expected is None:
        out["note"] = ("profile='manual' reapplies the device's last accepted candidate, "
                       "which the host does not know — `applied` is the device's own "
                       "readback, not a match against a requested config")
    return out


@mcp.tool()
async def rig_imu_env_rate(rate_hz: int | None = None, coupled: bool = False,
                           require_full_env: bool = False, timeout: float = 20.0) -> dict:
    """Read or set the IMU/env poll rate, verified against the device's readback.

    Streams 9 (quat), 10 (env) and 11 (raw IMU) are drained at this rate. It is a
    SECOND, independent command from `rig_profile()` — neither blocks the other,
    and each is verified only against its own half of the `ranging` echo.

    With no arguments this QUERIES. `coupled=True` returns the streams to the ToF
    trigger (the default: one sample per depth frame); `rate_hz` 1-480 decouples
    them at an explicit rate. Above the 60 Hz sensor-hub cycle, stream 10 (env)
    sub-samples while 9 and 11 keep the full rate — that is reported as a warning,
    not refused, unless you pass `require_full_env=True`, which makes it an error
    before anything is sent.

    `ok=true` means the device read the rate back — this waits for `imu_env` to go
    pending and then settle on the applied value, never on a log line or a merely
    newly-arrived broadcast. `ok=false` covers busy, replay, a rate outside 1-480,
    an unreachable-for-your-requirement rate, a device error, and a timeout.
    """
    from roomscan import profiles

    if not await rig.connect():
        return {"ok": False, "error": "cannot reach roomscan-web — call rig_up() first"}

    if rate_hz is None and not coupled:
        msg = await rig.wait_for("ranging", timeout=5.0) or rig.latest.get("ranging")
        if msg is None:
            return {"ok": False, "error": "no `ranging` state from the server"}
        out = _ranging_result(msg, {"kind": "query"})
        ie = msg.get("imu_env") or {}
        out["ok"] = bool(ie.get("initialized"))
        if not out["ok"]:
            out["error"] = ("the server has no IMU/env rate readback yet "
                            "(`imu_env.initialized: false`) — it never guesses one")
        return out

    if rate_hz is not None and coupled and rate_hz != 0:
        return {"ok": False, "error": f"pass coupled=True or rate_hz={rate_hz}, not both"}

    wanted = 0 if coupled else int(rate_hz or 0)
    requested = {"kind": "imu_env_rate", "rate_hz": wanted, "coupled": wanted == 0}

    validation = profiles.validate_imu_env_rate(wanted or None)
    if not validation.ok:
        return {"ok": False, "requested": requested, "sent": None,
                "error": "; ".join(validation.errors)}
    warnings = list(validation.warnings)
    if require_full_env and wanted > profiles.IMU_ENV_HUB_CYCLE_HZ:
        return {"ok": False, "requested": requested, "sent": None, "warnings": warnings,
                "error": (f"rate_hz={wanted} cannot deliver un-sub-sampled env: stream 10 "
                          f"rides the {profiles.IMU_ENV_HUB_CYCLE_HZ} Hz sensor-hub cycle. "
                          "Drop require_full_env to accept sub-sampled env.")}

    pre, blocked = await _ranging_preflight("imu_env")
    if blocked:
        out = _ranging_result(pre, requested)
        out.update({"ok": False, "error": blocked, "sent": None})
        return out

    sent = {"type": "set_imu_env_rate", "rate_hz": wanted}
    prior_error = ((pre or {}).get("imu_env") or {}).get("error")

    def matches(msg: dict) -> bool:
        ie = msg.get("imu_env") or {}
        return bool(ie.get("initialized")) and ie.get("applied_rate_hz") == wanted

    await rig.send(sent)
    msg, verdict = await _await_ranging("imu_env", matches, timeout, prior_error)

    out = _ranging_result(msg, requested)
    out["sent"] = sent
    out["ok"] = verdict == "ok"
    out["warnings"] = list(dict.fromkeys(warnings + out["warnings"]))
    if verdict == "error":
        out["error"] = _ranging_half(msg or {}, "imu_env")[1]
    elif verdict == "timeout":
        applied = ((msg or {}).get("imu_env") or {}).get("applied_rate_hz")
        out["error"] = (f"no confirmed readback within {timeout}s (last applied rate: "
                        f"{applied}); the command may still be in flight, or another "
                        "IMU/env rate change was already pending")
        last_error = _ranging_half(msg or {}, "imu_env")[1]
        if last_error:
            out["last_error"] = last_error
    return out


@mcp.tool()
async def rig_playback(action: str, value: str | float | None = None,
                       timeout: float = 10.0) -> dict:
    """Control the source: go_live, load_capture, or a transport action.

    `action="load_capture"` takes the capture name in `value`; `action="go_live"`
    takes none. Anything else is a transport action (pause, resume, speed, loop,
    restart, seek), where `value` is that action's argument.

    **`go_live`, `load_capture`, `seek` and `restart` DISCARD the ephemeral SLAM
    map** (BUG-091): they begin a new replay timeline, so the server resets the
    SLAM worker, TSDF, trajectory, cached mesh and all sensor state. Seeking
    mid-scan to "look at" a different part of a capture therefore throws the
    scan away and starts over from the seek point. `pause`, `resume`, `speed`
    and `loop` keep the current map. Reaching EOF with `loop` on wraps to frame
    0 and likewise starts a fresh map each lap.
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
    detailed = None
    if regenerate:
        msg = {"type": "regenerate_detailed"}
        # The server broadcasts `DetailedRunner.start()`'s own result -- await
        # it the same way `rig_save` awaits `saved`, rather than firing the
        # request and reading whatever `state` happens to have on hand, which
        # for a Detailed rebuild is the state from *before* the rebuild ran.
        detailed = await rig.request(msg, expect="detailed", timeout=timeout)
        sent.append(msg)
    if not sent:
        return {"ok": False, "error": "pass source, display, or regenerate"}
    expected = {k: v for k, v in (("source", source), ("display", display)) if v}
    state = await _await_state(expected, timeout)
    ok = state is not None and all(state.get(k) == v for k, v in expected.items())
    out = {"ok": ok, "sent": sent, "state": state}
    if regenerate:
        out["detailed"] = detailed
        if detailed is None:
            out["ok"] = False
            out["error"] = f"no detailed rebuild confirmation within {timeout}s"
        elif not detailed.get("started"):
            out["ok"] = False
            out["error"] = detailed.get("reason", "detailed rebuild did not start")
    return out


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


@mcp.tool()
async def rig_ws_probe(seconds: float = 10.0, url: str = "") -> dict:
    """Split "nothing rendered" into the three questions it actually collapses.

    Watches the same `/ws` + `/ws-mesh` a browser tab uses and reports, as
    `verdict`: whether the SERVER is computing (the `slam` message's
    `frames_integrated` / `mesh_seq` / `blocks_used`), whether the TRANSPORT is
    delivering (per-type JSON counts, binary tag counts, MESH bytes actually
    received), and whether the PAYLOAD is well formed (one MESH re-parsed with
    `slam.js`'s exact layout). All three green means the fault is in the browser
    -- geometry, camera, or something drawing over the viewport -- and no amount
    of further server reading will find it.

    `payload_ok` is the one people skip. `slam.js` walks the packet with bare
    `new Float32Array(buffer, off, n)` views, so a header that disagrees with the
    payload by one element does not degrade: the constructor throws, the handler
    dies, and the map silently stops updating. `slack_bytes != 0` in
    `mesh_decode` means the packer and the reader have drifted apart.

    This acks every mesh it receives, because `/ws-mesh` is credit-gated and a
    silent client gets one mesh and then a 1-per-5-s trickle, which looks exactly
    like a server that stopped sending.

    Keep `seconds` small. Connection count is a performance variable on this
    server -- `/ws` has no backpressure and `_broadcast_bytes` awaits per client
    on the event loop (BUG-060, BUG-061) -- so a long probe changes what it
    measures. Never binds the device stream.

    Wraps `host/tools/ws_probe.py::probe_async()`.
    """
    import sys

    if str(HOST) not in sys.path:
        sys.path.insert(0, str(HOST))
    from tools.ws_probe import DEFAULT_URL, probe_async

    return await probe_async(seconds=seconds, url=url or DEFAULT_URL)


@mcp.tool()
async def rig_thin_probe(frames: int = 5, url: str = "", out_dir: str = "",
                         orbit_yaw: float = 120.0,
                         modes: str = "point_cloud,slam,ir",
                         record: bool = False, timeout: float = 15.0,
                         format: str = "", fps: int = 0,
                         width: int = 0, height: int = 0,
                         quality: int = 0, v2: bool = False,
                         credits: int = 0, max_frame_bytes: int = 0) -> dict:
    """Look at what `/ws-thin` is actually drawing, as PNGs, and prove the
    commands move it.

    Stands in for the CrowPanel thin client we cannot put on the bench: connects
    to the running roomscan-web's `/ws-thin`, decodes the binary frame it sends
    -- tag 1 (`THIN_FRAME`: u32 tag=1, u16 w, u16 h, then exactly w*h*2 bytes of
    RGB565) by default, or tag 2 (`THIN_FRAME_JPEG`, #202 v2 layout: u32 tag=2,
    u16 w, u16 h, u32 seq, u32 payload_len, then payload_len bytes of baseline
    4:2:0 JFIF, decoded via `simplejpeg`) once a `thin_hello` has negotiated
    jpeg (#197/#202) -- writes each frame to
    `results/thin_probe/<timestamp>/*.png` (served at `/results/...`), and
    round-trips the inbound commands. Read the PNGs -- that is the point of it.

    `v2=True` sends the CrowPanel spec's proto-2 hello (#202): an `accept`
    preference list plus optional `credits` (1-8) and `max_frame_bytes`, and
    the probe then grants a `thin_ready` per consumed frame -- exercising the
    server's credit-based flow control (frames are DROPPED, never queued, when
    the client is slow; watch `telemetry.tx_fps` / `tx_bytes_per_s` /
    `dropped` for the per-client truth vs the rig-internal `fps`).

    `format`/`fps`/`width`/`height`/`quality` negotiate `thin_hello` before
    anything else, when any is given (non-empty/non-zero); leaving all five at
    their defaults sends no hello at all, so a default call still proves the
    un-negotiated path -- today's tag-1 RGB565 480x480 @ 10 fps, byte-identical
    to before #197. `width` alone (no `height`) is sent as a square request
    (this protocol is square-only). The result's `hello` key reports the exact
    request, the ACKED (clamped) effective values, and `degraded` (True only
    when `format="jpeg"` was requested but the server acked `format="raw"` --
    `simplejpeg` unavailable server-side). `ok` REQUIRES the ack to have
    arrived and every collected frame's tag to match the acked format -- a
    silently-degraded or never-acked hello no longer reports `ok: true`.

    THE OBSERVABLE IS PIXELS. `thin_orbit` is judged by `orbit.changed_frac`,
    the fraction of pixels that moved by more than 8 levels, measured **against
    a control**: `orbit.control_changed_frac` is the same measure on two
    consecutive frames with no command sent, so scene motion, replay advance and
    render dither are subtracted by construction and a null result cannot be
    explained away as "the scene was static". `orbit.moved_pixels` requires the
    orbit to change >=10% of pixels AND beat that control 3x. A frame counter or
    a read-back of the camera would prove nothing (the #106 lesson).

    `thin_mode` is confirmed the same way, per mode: the tool waits for the
    SERVER's own `thin_telemetry` to report the new mode (telemetry goes out at
    the top of a render tick and the frame at the bottom of the same tick, so
    the next frame is genuinely that mode) and then requires a frame.
    `modes_with_frames` is the answer. A confirmed mode with no frame is a real
    finding, not a failure: the render loop deliberately sends nothing when its
    generation-tagged stash for that mode is empty or stale (#101).

    `frame_stats` per frame includes `distinct_colors` -- a silently failed
    render is a uniformly filled buffer that every mean/variance check calls a
    picture, so check that number before believing a frame. `measured_fps` and
    `measured_mbps` cover the initial decoded receive batch and are timestamped
    before the probe computes those statistics or writes PNGs; the probe's own
    observability work therefore cannot masquerade as server/network latency.

    `record=False` by default and must be passed explicitly: `thin_record`
    starts a REAL capture. Everything else here is read-only.

    Never raises for the states you actually hit -- a server that is down, at its
    2-client cap (`thin_client_limit`), or without an offscreen context
    (`thin_render_unavailable`) all come back as `server_error`/`errors` in the
    result. Never binds the device stream; each connected thin client costs the
    server a full render, so this connects, works and disconnects.

    Wraps `host/tools/thin_client_probe.py::probe_async()`.
    """
    import sys

    if str(HOST) not in sys.path:
        sys.path.insert(0, str(HOST))
    from tools.thin_client_probe import DEFAULT_URL, probe_async

    return await probe_async(frames=frames, url=url or DEFAULT_URL,
                             out_dir=out_dir or None, orbit_yaw=orbit_yaw,
                             modes=modes, record=record, timeout=timeout,
                             format=format or None, fps=fps or None,
                             width=width or None, height=height or None,
                             quality=quality or None,
                             proto=2 if v2 else None, credits=credits or None,
                             max_frame_bytes=max_frame_bytes or None)
