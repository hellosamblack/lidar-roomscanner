"""Sensor auto-idle (SET_STANDBY, laser-wear reduction) — protocol opcode,
config/UiState plumbing, and the viewer-count-driven idle/wake state machine."""
import asyncio
import time
import types

import pytest

import roomscan.web as web
from roomscan.config import ViewerConfig
from roomscan.control import parse_command
from roomscan.protocol import (
    CommandCode,
    StandbyLevel,
    pack_command,
)


# --- protocol ---------------------------------------------------------------

def test_standby_opcode_and_levels():
    assert int(CommandCode.SET_STANDBY) == 7
    assert (int(StandbyLevel.ACTIVE), int(StandbyLevel.SOFT), int(StandbyLevel.HARD)) == (0, 1, 2)


@pytest.mark.parametrize("level", [0, 1, 2])
def test_pack_command_standby_roundtrips(level):
    frame = pack_command(int(CommandCode.SET_STANDBY), level, token=0xABCD)
    # header seq carries the token; payload is cmd(u32)+param(u32) LE
    from roomscan.protocol import FrameHeader, HEADER_SIZE
    hdr = FrameHeader.unpack(frame[:HEADER_SIZE])
    assert hdr.seq == 0xABCD
    import struct
    cmd, param = struct.unpack_from("<II", frame, HEADER_SIZE)
    assert cmd == int(CommandCode.SET_STANDBY)
    assert param == level


def test_control_cli_parses_standby():
    _args, cmd, param = parse_command(["standby", "2"])
    assert cmd == CommandCode.SET_STANDBY
    assert param == 2


# --- config + UiState plumbing ----------------------------------------------

def test_config_idle_defaults():
    c = ViewerConfig()
    assert c.sensor_idle_enabled is True
    assert c.sensor_idle_level == "soft"
    assert c.sensor_idle_delay_s == 5.0
    assert c.sensor_idle_activity_timeout_s == 60.0
    assert c.browser_idle_timeout_s == 300.0


def test_config_idle_round_trip(tmp_path):
    path = tmp_path / "roomscan.toml"
    ViewerConfig(sensor_idle_enabled=False, sensor_idle_level="hard",
                 sensor_idle_delay_s=2.5, sensor_idle_activity_timeout_s=30.0,
                 browser_idle_timeout_s=120.0).save(path)
    back = ViewerConfig.load(path)
    assert back.sensor_idle_enabled is False
    assert back.sensor_idle_level == "hard"
    assert back.sensor_idle_delay_s == 2.5
    assert back.sensor_idle_activity_timeout_s == 30.0
    assert back.browser_idle_timeout_s == 120.0


def test_idle_standby_level_maps_soft_and_hard():
    assert web.idle_standby_level("soft") == int(StandbyLevel.SOFT)
    assert web.idle_standby_level("hard") == int(StandbyLevel.HARD)
    assert web.idle_standby_level("nonsense") == int(StandbyLevel.SOFT)  # safe default


def test_ui_from_config_and_apply_round_trip():
    cfg = ViewerConfig(sensor_idle_enabled=False, sensor_idle_level="hard",
                       browser_idle_timeout_s=90.0)
    ui = web.ui_from_config(cfg)
    assert ui.idle_enabled is False
    assert ui.idle_level == "hard"
    assert ui.browser_idle_timeout_s == 90.0
    out = ViewerConfig()
    web.apply_ui_to_config(ui, out)
    assert out.sensor_idle_enabled is False
    assert out.sensor_idle_level == "hard"
    assert out.browser_idle_timeout_s == 90.0


def test_ui_from_config_rejects_bad_level():
    cfg = ViewerConfig(sensor_idle_level="bogus")
    assert web.ui_from_config(cfg).idle_level == "soft"  # falls back to UiState default


def test_state_message_carries_idle_fields():
    ui = web.UiState(idle_enabled=False, idle_level="hard", browser_idle_timeout_s=45.0)
    msg = web._state_message(ui)
    assert msg["idle_enabled"] is False
    assert msg["idle_level"] == "hard"
    assert msg["browser_idle_timeout_s"] == 45.0


# --- auto-idle state machine ------------------------------------------------

class _FakeDispatcher:
    def __init__(self):
        self.calls = []

    def dispatch(self, cmd, param, label):
        self.calls.append((int(cmd), int(param), label))


def _make_state(*, enabled=True, level="soft", has_live=True, mode="live",
                clients=(), delay=0.02, recording=False, activity_timeout=60.0,
                ws_client_id=None, client_active=None):
    return types.SimpleNamespace(
        ui_state=web.UiState(idle_enabled=enabled, idle_level=level),
        controller=types.SimpleNamespace(
            has_live=has_live, mode=mode,
            recorder=types.SimpleNamespace(active=recording)),
        dispatcher=_FakeDispatcher(),
        command_labels=set(),
        clients=set(clients),
        sensor_idled=False,
        idle_timer=None,
        idle_delay_s=delay,
        activity_timeout_s=activity_timeout,
        ws_client_id=dict(ws_client_id) if ws_client_id else {},
        client_active=dict(client_active) if client_active else {},
    )


def test_auto_idle_active_gating():
    assert web._auto_idle_active(_make_state()) is True
    assert web._auto_idle_active(_make_state(enabled=False)) is False
    assert web._auto_idle_active(_make_state(has_live=False)) is False
    assert web._auto_idle_active(_make_state(mode="replay")) is False


def test_last_viewer_leaving_idles_after_debounce():
    async def scenario():
        st = _make_state(level="hard")
        await web._viewer_left(st)                 # last tab gone -> arm timer
        assert st.idle_timer is not None
        assert st.dispatcher.calls == []           # not yet -- debounced
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.sensor_idled is True
    assert st.dispatcher.calls == [(int(CommandCode.SET_STANDBY), int(StandbyLevel.HARD),
                                    "auto-idle (hard)")]

    async def wake():
        st.clients.add(object())                   # a tab connects
        await web._viewer_arrived(st)
        return st

    st = asyncio.run(wake())
    assert st.sensor_idled is False
    assert st.dispatcher.calls[-1] == (int(CommandCode.SET_STANDBY),
                                       int(StandbyLevel.ACTIVE), "auto-wake")


def test_reconnect_during_debounce_cancels_idle():
    async def scenario():
        st = _make_state()
        await web._viewer_left(st)                  # arm
        st.clients.add(object())                    # reconnect before it fires
        await web._viewer_arrived(st)               # cancels the timer
        assert st.idle_timer is None
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.sensor_idled is False
    assert st.dispatcher.calls == []                # never idled -> nothing to wake either


def test_no_idle_when_other_viewers_present():
    async def scenario():
        st = _make_state(clients=(object(),))       # one tab still watching
        await web._viewer_left(st)
        assert st.idle_timer is None
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.dispatcher.calls == []


def test_no_idle_in_replay_or_disabled():
    for st_kwargs in (dict(mode="replay"), dict(enabled=False)):
        async def scenario():
            st = _make_state(**st_kwargs)
            await web._viewer_left(st)
            await asyncio.sleep(st.idle_delay_s + 0.05)
            return st

        st = asyncio.run(scenario())
        assert st.idle_timer is None
        assert st.dispatcher.calls == []


# --- activity-based "actively engaged" viewer accounting (Phase A/A.5) ------
#
# The reported failure this section fixes: a `/ws` connection that never
# engages (an agent's one-shot probe, a tab left open unattended) used to keep
# `state.clients` non-empty forever, so `_viewer_left` never armed the idle
# timer at all -- see the module comment above `_resolve_client_id` in web.py.


def test_client_is_active_default_and_timeout():
    st = _make_state(activity_timeout=0.05)
    # Untracked client_id: fail-open (treated as active) rather than crash.
    assert web._client_is_active(st, "never-seen") is True
    web._touch_client_active(st, "a")
    assert web._client_is_active(st, "a") is True
    time.sleep(0.08)
    assert web._client_is_active(st, "a") is False   # aged out
    web._touch_client_active(st, "a")
    assert web._client_is_active(st, "a") is True    # heartbeat refreshes it


def test_mark_client_inactive_forces_immediately_stale():
    st = _make_state(activity_timeout=60.0)
    web._touch_client_active(st, "a")
    assert web._client_is_active(st, "a") is True
    web._mark_client_inactive(st, "a")
    assert web._client_is_active(st, "a") is False   # doesn't wait out the timeout


def test_active_viewer_count_ignores_stale_but_counts_untracked_and_fresh():
    ws_tracked_stale = object()
    ws_tracked_fresh = object()
    ws_untracked = object()
    st = _make_state(
        clients=(ws_tracked_stale, ws_tracked_fresh, ws_untracked),
        ws_client_id={ws_tracked_stale: "stale", ws_tracked_fresh: "fresh"},
        client_active={"stale": 0.0},   # aged out
        activity_timeout=60.0,
    )
    web._touch_client_active(st, "fresh")
    # stale -> excluded, untracked -> fail-open counted, fresh -> counted.
    assert web._active_viewer_count(st) == 2


def test_active_viewer_count_recording_overrides_engagement():
    st = _make_state(clients=(), recording=True)
    assert web._active_viewer_count(st) == 1   # a live take is never interrupted


def test_engaged_clients_excludes_only_stale_tracked_sockets():
    ws_active, ws_parked, ws_untracked = object(), object(), object()
    st = _make_state(
        clients=(ws_active, ws_parked, ws_untracked),
        ws_client_id={ws_active: "active-cid", ws_parked: "parked-cid"},
        client_active={"parked-cid": 0.0},
        activity_timeout=60.0,
    )
    web._touch_client_active(st, "active-cid")
    engaged = web._engaged_clients(st)
    assert engaged == {ws_active, ws_untracked}


def test_idle_state_false_parks_immediately_and_arms_idle():
    """A client that explicitly parks (Phase C's idle.js) idles on the normal
    debounce, not on `sensor_idle_activity_timeout_s` -- it doesn't wait to be
    aged out."""
    async def scenario():
        ws = object()
        st = _make_state(clients=(ws,), ws_client_id={ws: "cid"}, level="hard")
        web._touch_client_active(st, "cid")
        assert st.dispatcher.calls == []
        await web._handle_inbound(st, {"type": "idle_state", "active": False}, ws)
        assert st.idle_timer is not None      # armed immediately, not on a reconcile tick
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.sensor_idled is True
    assert st.dispatcher.calls == [(int(CommandCode.SET_STANDBY), int(StandbyLevel.HARD),
                                    "auto-idle (hard)")]


def test_idle_state_true_heartbeat_prevents_idle_and_wakes():
    async def scenario():
        ws = object()
        st = _make_state(clients=(ws,), ws_client_id={ws: "cid"})
        await web._handle_inbound(st, {"type": "idle_state", "active": False}, ws)
        assert st.idle_timer is not None
        # The heartbeat arrives before the debounce fires -> cancels it, and a
        # subsequent reconcile tick must not idle a heartbeating client either.
        await web._handle_inbound(st, {"type": "idle_state", "active": True}, ws)
        assert st.idle_timer is None
        await web._reconcile_idle_once(st)
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.sensor_idled is False
    assert st.dispatcher.calls == []


def test_reconcile_idle_once_ages_out_a_silently_stale_connection():
    """The actual reported scenario: a socket stays open (never disconnects,
    never sends idle_state) but nobody has done anything with it in a long
    time. Nothing event-driven ever re-evaluates that -- only the periodic
    reconcile tick does."""
    async def scenario():
        ws = object()
        st = _make_state(clients=(ws,), ws_client_id={ws: "cid"},
                          activity_timeout=0.05, level="hard")
        web._touch_client_active(st, "cid")
        await web._reconcile_idle_once(st)     # still fresh -> no-op
        assert st.idle_timer is None
        time.sleep(0.08)                       # ages out with no traffic at all
        await web._reconcile_idle_once(st)
        assert st.idle_timer is not None
        await asyncio.sleep(st.idle_delay_s + 0.05)
        return st

    st = asyncio.run(scenario())
    assert st.sensor_idled is True
    assert st.dispatcher.calls == [(int(CommandCode.SET_STANDBY), int(StandbyLevel.HARD),
                                    "auto-idle (hard)")]


def test_reconcile_idle_once_does_not_redispatch_while_already_idled():
    async def scenario():
        st = _make_state(clients=(), activity_timeout=0.05, level="hard")
        await web._reconcile_idle_once(st)
        await asyncio.sleep(st.idle_delay_s + 0.05)
        first_call_count = len(st.dispatcher.calls)
        # Repeated ticks while still nobody's here must not re-send SET_STANDBY.
        await web._reconcile_idle_once(st)
        await web._reconcile_idle_once(st)
        return st, first_call_count

    st, first_call_count = asyncio.run(scenario())
    assert st.sensor_idled is True
    assert len(st.dispatcher.calls) == first_call_count == 1
