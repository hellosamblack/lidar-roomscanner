"""Sensor auto-idle (SET_STANDBY, laser-wear reduction) — protocol opcode,
config/UiState plumbing, and the viewer-count-driven idle/wake state machine."""
import asyncio
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


def test_config_idle_round_trip(tmp_path):
    path = tmp_path / "roomscan.toml"
    ViewerConfig(sensor_idle_enabled=False, sensor_idle_level="hard",
                 sensor_idle_delay_s=2.5).save(path)
    back = ViewerConfig.load(path)
    assert back.sensor_idle_enabled is False
    assert back.sensor_idle_level == "hard"
    assert back.sensor_idle_delay_s == 2.5


def test_idle_standby_level_maps_soft_and_hard():
    assert web.idle_standby_level("soft") == int(StandbyLevel.SOFT)
    assert web.idle_standby_level("hard") == int(StandbyLevel.HARD)
    assert web.idle_standby_level("nonsense") == int(StandbyLevel.SOFT)  # safe default


def test_ui_from_config_and_apply_round_trip():
    cfg = ViewerConfig(sensor_idle_enabled=False, sensor_idle_level="hard")
    ui = web.ui_from_config(cfg)
    assert ui.idle_enabled is False
    assert ui.idle_level == "hard"
    out = ViewerConfig()
    web.apply_ui_to_config(ui, out)
    assert out.sensor_idle_enabled is False
    assert out.sensor_idle_level == "hard"


def test_ui_from_config_rejects_bad_level():
    cfg = ViewerConfig(sensor_idle_level="bogus")
    assert web.ui_from_config(cfg).idle_level == "soft"  # falls back to UiState default


def test_state_message_carries_idle_fields():
    ui = web.UiState(idle_enabled=False, idle_level="hard")
    msg = web._state_message(ui)
    assert msg["idle_enabled"] is False
    assert msg["idle_level"] == "hard"


# --- auto-idle state machine ------------------------------------------------

class _FakeDispatcher:
    def __init__(self):
        self.calls = []

    def dispatch(self, cmd, param, label):
        self.calls.append((int(cmd), int(param), label))


def _make_state(*, enabled=True, level="soft", has_live=True, mode="live",
                clients=(), delay=0.02):
    return types.SimpleNamespace(
        ui_state=web.UiState(idle_enabled=enabled, idle_level=level),
        controller=types.SimpleNamespace(has_live=has_live, mode=mode),
        dispatcher=_FakeDispatcher(),
        command_labels=set(),
        clients=set(clients),
        sensor_idled=False,
        idle_timer=None,
        idle_delay_s=delay,
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
