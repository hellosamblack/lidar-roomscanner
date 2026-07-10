import queue
import struct
import time

from roomscan.config import ViewerConfig
from roomscan.protocol import CommandCode, FrameHeader, FrameType, ResultCode, StreamId, pack_frame
from roomscan.viewer import CommandKeyState, _reader, resolve_args


# --- CommandKeyState ---------------------------------------------------------

class StubClient:
    """Records send() calls; returns/raises whatever the test configures."""

    def __init__(self, result=(ResultCode.OK, 1), raise_timeout=False, delay=0.0):
        self.calls = []
        self._result = result
        self._raise_timeout = raise_timeout
        self._delay = delay

    def send(self, cmd, param=0, timeout=2.0):
        self.calls.append((cmd, param))
        if self._delay:
            time.sleep(self._delay)
        if self._raise_timeout:
            raise TimeoutError(f"no ACK for cmd={cmd}")
        return self._result


def _wait_for(predicate, timeout=2.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def test_dispatch_replay_mode_prints_not_available(capsys):
    state = CommandKeyState(client=None)
    state.dispatch(CommandCode.PING, 0, "ping")
    out = capsys.readouterr().out
    assert "not available in replay" in out


def test_dispatch_success_prints_result_and_clears_busy(capsys):
    client = StubClient(result=(ResultCode.OK, 1))
    state = CommandKeyState(client)
    state.dispatch(CommandCode.SET_USECASE, 1, "usecase 1")
    assert _wait_for(lambda: not state._busy)
    out = capsys.readouterr().out
    assert "usecase 1" in out
    assert "OK" in out
    assert "applied=1" in out
    assert client.calls == [(CommandCode.SET_USECASE, 1)]


def test_dispatch_timeout_prints_timeout_and_clears_busy(capsys):
    client = StubClient(raise_timeout=True)
    state = CommandKeyState(client)
    state.dispatch(CommandCode.PING, 0, "ping")
    assert _wait_for(lambda: not state._busy)
    out = capsys.readouterr().out
    assert "TIMEOUT" in out


def test_dispatch_busy_guard_drops_second_press(capsys):
    client = StubClient(delay=0.2)
    state = CommandKeyState(client)
    state.dispatch(CommandCode.PING, 0, "ping")
    # second press immediately, while the first is still in flight
    state.dispatch(CommandCode.PING, 0, "ping")
    out = capsys.readouterr().out
    assert "busy" in out
    assert _wait_for(lambda: not state._busy)
    assert client.calls == [(CommandCode.PING, 0)]  # second press never called send()


def test_dispatch_allows_a_new_command_once_previous_completes(capsys):
    client = StubClient(result=(ResultCode.OK, 0))
    state = CommandKeyState(client)
    state.dispatch(CommandCode.PING, 0, "ping")
    assert _wait_for(lambda: not state._busy)
    capsys.readouterr()
    state.dispatch(CommandCode.REINIT, 0, "reinit")
    assert _wait_for(lambda: not state._busy)
    assert client.calls == [(CommandCode.PING, 0), (CommandCode.REINIT, 0)]


# --- _reader routes ACK frames to the CommandClient, not the render slot ----

class StubOfferClient:
    def __init__(self):
        self.offered = []

    def offer(self, frame):
        self.offered.append(frame)
        return True


def make_ack_frame(token: int, cmd: int, result: int, applied: int):
    payload = struct.pack("<III", cmd, result, applied)
    header = FrameHeader(FrameType.ACK, 0, 0, token, 0, 0, 0, len(payload))
    return pack_frame(header, payload)


class OneShotThenStop:
    def __init__(self, data: bytes):
        self._data = data
        self._sent = False

    def read(self):
        if self._sent:
            raise StopIteration
        self._sent = True
        return self._data

    def close(self):
        pass


def test_reader_offers_ack_frames_to_client_not_the_slot():
    from roomscan.decoder import StreamDecoder

    frame_bytes = make_ack_frame(42, CommandCode.PING, ResultCode.OK, 1)
    client = StubOfferClient()
    fault: dict = {}
    slot: queue.Queue = queue.Queue(maxsize=1)
    from roomscan.viewer import Stats
    _reader(OneShotThenStop(frame_bytes), StreamDecoder(), slot, Stats(), None, fault,
            client=client)
    assert len(client.offered) == 1
    assert client.offered[0].header.frame_type == FrameType.ACK
    assert slot.empty()  # ACK never reaches the render slot


def test_reader_ack_with_no_client_is_silently_dropped():
    from roomscan.decoder import StreamDecoder
    from roomscan.viewer import Stats

    frame_bytes = make_ack_frame(42, CommandCode.PING, ResultCode.OK, 1)
    fault: dict = {}
    slot: queue.Queue = queue.Queue(maxsize=1)
    _reader(OneShotThenStop(frame_bytes), StreamDecoder(), slot, Stats(), None, fault)
    assert slot.empty()  # ACK frames never populate the render slot, client or not


# --- resolve_args: config load/save/priority --------------------------------

def test_resolve_args_defaults_from_builtin_when_no_config_file(tmp_path):
    cfg_path = tmp_path / "roomscan.toml"
    args = resolve_args(["--replay", "dummy.bin"], config_path=cfg_path)
    assert args.color == "reflectance"
    assert args.fov_h == 55.0
    assert args.fov_v == 42.0
    assert args.replay_fps == 0.0
    assert args.port is None


def test_resolve_args_pulls_from_existing_config_file(tmp_path):
    cfg_path = tmp_path / "roomscan.toml"
    ViewerConfig(color="reflectance", fov_h=54.65, fov_v=42.50, replay_fps=25.0, port="COM7").save(cfg_path)
    args = resolve_args(["--replay", "dummy.bin"], config_path=cfg_path)
    assert args.color == "reflectance"
    assert args.fov_h == 54.65
    assert args.fov_v == 42.50
    assert args.replay_fps == 25.0
    assert args.port == "COM7"


def test_resolve_args_cli_flag_overrides_config_file(tmp_path):
    cfg_path = tmp_path / "roomscan.toml"
    ViewerConfig(color="reflectance", fov_h=54.65).save(cfg_path)
    args = resolve_args(["--replay", "dummy.bin", "--color", "confidence"], config_path=cfg_path)
    assert args.color == "confidence"   # CLI wins
    assert args.fov_h == 54.65          # config still fills the untouched flag


def test_resolve_args_save_config_writes_effective_settings(tmp_path, capsys):
    cfg_path = tmp_path / "roomscan.toml"
    resolve_args(["--replay", "dummy.bin", "--color", "confidence", "--fov-h", "50.0",
                  "--save-config"], config_path=cfg_path)
    assert cfg_path.exists()
    saved = ViewerConfig.load(cfg_path)
    assert saved.color == "confidence"
    assert saved.fov_h == 50.0
    out = capsys.readouterr().out
    assert "saved config" in out
    assert str(cfg_path) in out


def test_resolve_args_save_config_then_reload_is_the_new_default(tmp_path):
    cfg_path = tmp_path / "roomscan.toml"
    resolve_args(["--replay", "dummy.bin", "--color", "reflectance", "--save-config"], config_path=cfg_path)
    # A later run with no --color flag should now pick up "reflectance".
    args = resolve_args(["--replay", "dummy.bin"], config_path=cfg_path)
    assert args.color == "reflectance"
