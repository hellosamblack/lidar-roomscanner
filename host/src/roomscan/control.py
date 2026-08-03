"""Host->device command channel: CommandClient + the roomscan-ctl CLI.

CommandClient does NOT own a serial port or a read loop. It is fed decoded
frames by whatever loop already owns the StreamDecoder (call `offer()` from
that loop) and it writes command frames through a caller-supplied callable
(call `send()` from any OTHER thread). This split is deliberate: Phase 3
Task 2 proved on hardware that a blocking write of any size, issued from the
thread draining reads, starves that read loop and causes the device to abort
an in-flight send (>100 ms of read starvation trips its bounded best-effort
policy). Keeping "write" and "observe decoded frames" as separate entry
points makes that bug structurally impossible to reintroduce, whether
CommandClient is driven by the roomscan-ctl CLI (own throwaway reader thread)
or later wired into the viewer's existing reader thread (Task 7).
"""
from __future__ import annotations

import argparse
import itertools
import random
import sys
import threading
from typing import Callable

from . import profiles
from . import protocol
from .protocol import (
    CommandCode,
    Frame,
    FrameType,
    LegacyAck,
    ProtocolError,
    RangingConfigAck,
    ResultCode,
    pack_command,
    pack_manual_command,
    parse_event,
    parse_typed_ack,
)


class CommandClient:
    """Send COMMAND frames and await their ACK.

    Thread-contract: `offer()` is called by the single loop that owns the
    decoder (matches decoded frames against pending sends); `send()` may be
    called concurrently from any other thread(s) and blocks until its ACK
    arrives or `timeout` elapses.
    """

    def __init__(self, write: Callable[[bytes], None]):
        self._write = write
        self._tokens = itertools.count(random.getrandbits(32))
        self._lock = threading.Lock()
        # token -> (event, slot, cmd). `cmd` is what makes offer() able to pick
        # the right ACK shape (parse_typed_ack dispatches on it) -- v2 has two
        # incompatible ACK payload lengths (12-byte legacy vs 16-byte ranging-
        # config) selected by which command they answer, not by their own length.
        self._pending: dict[int, tuple[threading.Event, list, int]] = {}

    def offer(self, frame: Frame) -> bool:
        """Feed one decoded frame in. Returns True iff it was consumed as the
        awaited ACK for a pending send (matched by token == header.seq).
        Everything else — DATA, EVENT, an ACK with no matching pending token,
        or an ACK whose payload fails to parse — returns False untouched so
        the caller's own DATA/EVENT handling keeps working undisturbed.
        """
        if frame.header.frame_type != FrameType.ACK:
            return False
        token = frame.header.seq
        with self._lock:
            entry = self._pending.pop(token, None)
        if entry is None:
            return False
        event, slot, cmd = entry
        try:
            slot.append(parse_typed_ack(cmd, frame.payload))
        except ProtocolError:
            slot.append(None)
        event.set()
        return True

    def _next_token(self) -> int:
        return next(self._tokens) & 0xFFFFFFFF

    def _send_and_await(self, cmd: int, frame_bytes: bytes, token: int,
                        timeout: float) -> "LegacyAck | RangingConfigAck":
        """Register `token` as pending for `cmd`'s ACK shape, write the already-
        packed `frame_bytes` (which must encode `token` as its header seq), and
        block for the matching typed ACK. This is the one thread-contract
        chokepoint every public send*/get* method funnels through: `_pending`
        is populated BEFORE the write so an ACK racing in on the reader thread
        can never arrive before anyone is waiting for it, and the write itself
        happens on the CALLER's thread, never the reader thread that drives
        `offer()` (see module docstring)."""
        event = threading.Event()
        slot: list = []
        with self._lock:
            self._pending[token] = (event, slot, cmd)
        try:
            self._write(frame_bytes)
            if not event.wait(timeout):
                raise TimeoutError(
                    f"no ACK for cmd={cmd} token={token} within {timeout}s "
                    f"({len(self._pending)} command(s) still pending)"
                )
        finally:
            with self._lock:
                self._pending.pop(token, None)
        result = slot[0] if slot else None
        if result is None:
            raise TimeoutError(f"ACK for cmd={cmd} token={token} arrived but its payload was unparsable")
        return result

    def send(self, cmd: int, param: int = 0, timeout: float = 2.0) -> tuple[ResultCode, int]:
        """Write a legacy 8-byte cmd+param COMMAND frame and block for its
        12-byte ACK. Covers commands 1-8, 11, 12 (every command that keeps the
        v1 payload shape -- see docs/protocol.md). Raises TimeoutError on
        silence within `timeout` seconds, or ProtocolError if `cmd` turns out
        to use the extended ranging-config ACK shape (9/10 -- use
        `send_manual_params`/`get_ranging_config` for those)."""
        token = self._next_token()
        ack = self._send_and_await(cmd, pack_command(cmd, param, token), token, timeout)
        if not isinstance(ack, LegacyAck):
            raise ProtocolError(
                f"cmd={cmd} returned a {type(ack).__name__} ACK; send() only handles the "
                "legacy shape -- use send_manual_params()/get_ranging_config() for cmd 9/10")
        return ResultCode(ack.result), ack.applied

    def send_profile(self, profile_id: int, timeout: float = 2.0) -> tuple[ResultCode, int]:
        """SET_RANGING_PROFILE (cmd 8): legacy shape, param/applied = the
        `ProfileId` enum value. Presets 0-2 apply immediately; MANUAL (3)
        reapplies the last accepted `send_manual_params()` candidate and is
        rejected (BAD_PARAM) until one exists."""
        return self.send(CommandCode.SET_RANGING_PROFILE, int(profile_id), timeout=timeout)

    def send_manual_params(self, params: "protocol.ManualParams",
                           timeout: float = 2.0) -> tuple[ResultCode, RangingConfigAck]:
        """SET_MANUAL_PARAMS (cmd 9): pack+send the 12-byte manual COMMAND
        shape, block for the 16-byte ranging-config ACK. `params` is the
        wire-shaped `protocol.ManualParams` (raw wire ints), not
        `roomscan.profiles.ManualParams` -- callers validate through
        `roomscan.profiles` and convert (see `roomscan-ctl manual`'s
        `_manual_params_to_wire`) before calling this. The returned
        `RangingConfigAck` is always the device's actual applied/readback
        config (docs/protocol.md: sent with this 16-byte shape regardless of
        `result` -- a BUSY/BAD_PARAM/SENSOR_ERROR ACK still carries the prior
        known-good config, never a shorter payload), so a caller must check
        `result` before trusting the config as newly applied."""
        token = self._next_token()
        ack = self._send_and_await(
            CommandCode.SET_MANUAL_PARAMS, pack_manual_command(params, token), token, timeout)
        if not isinstance(ack, RangingConfigAck):
            raise ProtocolError(
                f"SET_MANUAL_PARAMS returned a {type(ack).__name__} ACK, expected RangingConfigAck")
        return ResultCode(ack.result), ack

    def get_ranging_config(self, timeout: float = 2.0) -> tuple[ResultCode, RangingConfigAck]:
        """GET_RANGING_CONFIG (cmd 10): no parameter, blocks for the 16-byte
        ranging-config ACK -- the device's authoritative current config, used
        to restore host state after a web-server restart or a second client
        attaching (never assumed from a firmware default)."""
        token = self._next_token()
        ack = self._send_and_await(
            CommandCode.GET_RANGING_CONFIG,
            pack_command(CommandCode.GET_RANGING_CONFIG, 0, token), token, timeout)
        if not isinstance(ack, RangingConfigAck):
            raise ProtocolError(
                f"GET_RANGING_CONFIG returned a {type(ack).__name__} ACK, expected RangingConfigAck")
        return ResultCode(ack.result), ack

    def send_imu_env_rate(self, rate_hz: int, timeout: float = 2.0) -> tuple[ResultCode, int]:
        """SET_IMU_ENV_RATE (cmd 11): legacy shape; `applied` IS the applied
        rate_hz (0 = coupled to the ToF trigger, the default)."""
        return self.send(CommandCode.SET_IMU_ENV_RATE, rate_hz, timeout=timeout)

    def get_imu_env_rate(self, timeout: float = 2.0) -> tuple[ResultCode, int]:
        """GET_IMU_ENV_RATE (cmd 12): legacy shape, no parameter; `applied` IS
        the device's current applied rate_hz (0 = coupled)."""
        return self.send(CommandCode.GET_IMU_ENV_RATE, 0, timeout=timeout)


class CommandDispatcher:
    """Fire-and-forget command dispatch, shared by the viewer's key bindings
    and the Phase 3.5 GUI control panel.

    Generalizes ``roomscan.viewer.CommandKeyState``: identical busy-guard and
    worker-thread mechanics (a single in-flight-guard flag under a lock,
    set/checked synchronously in `dispatch()` before any thread is spawned;
    the actual `CommandClient.send()` call -- up to its 2 s ACK timeout --
    runs on a short-lived daemon worker thread so the caller never blocks).
    The only difference: results/errors go through an injected `on_message`
    callback instead of a hardcoded `print`, so a GUI panel can route them
    to its event-log bus while the classic viewer just passes `print`.

    Message convention: this class always emits bare `f"{label} -> ..."`
    strings -- no leading marker. Callers that want one (e.g. the viewer's
    "[cmd] " prefix) add it in their `on_message` callback; that keeps this
    class's wording identical to `CommandKeyState`'s today, so a caller can
    swap `CommandKeyState` for `CommandDispatcher(client, lambda m: print(f"\\n[cmd] {m}"))`
    with no observable change in output.

    `client is None` means replay mode (no live device to command): every
    dispatch just reports that commands aren't available and returns (no
    worker thread, `busy` stays False).
    """

    def __init__(self, client: CommandClient | None, on_message: Callable[[str], None] = print):
        self.client = client
        self.on_message = on_message
        self._lock = threading.Lock()
        self._busy = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._busy

    def dispatch(self, cmd: int, param: int, label: str) -> None:
        if self.client is None:
            self.on_message(f"{label} -> not available in replay")
            return
        with self._lock:
            if self._busy:
                self.on_message(f"{label} -> busy, command already in flight")
                return
            self._busy = True

        def worker() -> None:
            try:
                result, applied = self.client.send(cmd, param)
                self.on_message(f"{label} -> {result.name} applied={applied}")
            except TimeoutError as exc:
                self.on_message(f"{label} -> TIMEOUT {exc}")
            except Exception as exc:  # e.g. SerialException on a dead port: report, don't traceback
                self.on_message(f"{label} -> ERROR {exc!r}")
            finally:
                with self._lock:
                    self._busy = False

        threading.Thread(target=worker, daemon=True).start()


# --- roomscan-ctl CLI --------------------------------------------------------

_ACTION_COMMANDS = {
    "ping": CommandCode.PING,
    "calib": CommandCode.SEND_CALIB,
    "reinit": CommandCode.REINIT,
}
_ACTION_COMMANDS_WITH_VALUE = {
    "usecase": CommandCode.SET_USECASE,
    "period": CommandCode.SET_FRAME_PERIOD_US,
    "exposure": CommandCode.SET_EXPOSURE_MS,
    "standby": CommandCode.SET_STANDBY,
}

# Deprecated for one release (plan Task 3 step 4): non-atomic, low-level diagnostics.
# `period` is additionally inert under old manual-sync firmware (docs/protocol.md cmd 4:
# "has no observable effect while the app uses VL53L9_SYNC_MANUAL"). `profile`/`manual`
# below are the atomic, typed-readback replacements.
_DEPRECATED_LEGACY_ACTIONS = frozenset({"usecase", "period", "exposure"})

# Typed ranging-profile actions (this task). Each has its own dispatch function below --
# none of them reduce to parse_command()'s single (cmd, param) shape, either because they
# need roomscan.profiles validation first (manual, imu-rate) or a typed ACK (all of them).
_TYPED_ACTIONS = frozenset({"profile", "manual", "profile-status", "imu-rate", "imu-rate-status"})

_PROFILE_PRESET_CHOICES: dict[str, protocol.ProfileId] = {
    "room_mapping": protocol.ProfileId.ROOM_MAPPING,
    "precision": protocol.ProfileId.PRECISION,
    "high_framerate": protocol.ProfileId.HIGH_FRAMERATE,
}
_MANUAL_RANGING_CHOICES: dict[str, profiles.RangingMode] = {
    "ambient": profiles.RangingMode.AMBIENT,
    "precision": profiles.RangingMode.PRECISION,
}
_MANUAL_POWER_CHOICES: dict[str, profiles.PowerMode] = {
    "ulp": profiles.PowerMode.ULTRA_LOW,
    "lp": profiles.PowerMode.LOW,
    "regular": profiles.PowerMode.REGULAR,
}

# roomscan.profiles' RangingMode/PowerMode enums deliberately MIRROR the firmware's C
# enums (vl53l9_context_t / vl53l9_power_mode_t), while protocol.py's wire enums use their
# OWN independent numbering fixed by docs/protocol.md's registries -- e.g. wire AMBIENT=0/
# PRECISION=1 vs profiles' PRECISION=0/AMBIENT=1; wire ULP=0/LP=1/REGULAR=2 vs profiles'
# REGULAR=0/LOW=1/ULTRA_LOW=2. Passing `.value` straight through would silently send the
# wrong ranging mode or power mode. Map by NAME, always through these tables, never by
# raw int -- see test_manual_params_to_wire_maps_by_name_not_raw_value in test_control.py.
_RANGING_MODE_TO_WIRE: dict[profiles.RangingMode, protocol.RangingMode] = {
    profiles.RangingMode.AMBIENT: protocol.RangingMode.AMBIENT,
    profiles.RangingMode.PRECISION: protocol.RangingMode.PRECISION,
}
_POWER_MODE_TO_WIRE: dict[profiles.PowerMode, protocol.PowerMode] = {
    profiles.PowerMode.REGULAR: protocol.PowerMode.REGULAR,
    profiles.PowerMode.LOW: protocol.PowerMode.LP,
    profiles.PowerMode.ULTRA_LOW: protocol.PowerMode.ULP,
}


def _manual_params_to_wire(params: "profiles.ManualParams") -> "protocol.ManualParams":
    """Convert a validated `roomscan.profiles.ManualParams` (host validation model,
    firmware-C-enum-shaped) into the wire-shaped `protocol.ManualParams` `send_manual_params`
    sends. Pure / no I/O -- the mapping tables above are the single place the two enum
    spaces are bridged; nothing else in this module should convert one to the other."""
    return protocol.ManualParams(
        ranging_mode=int(_RANGING_MODE_TO_WIRE[params.ranging_mode]),
        frame_period_us=profiles.fps_to_period_us(params.fps),
        exposure_ms=params.exposure_ms,
        power_mode=int(_POWER_MODE_TO_WIRE[params.power_mode]),
    )


def _imu_rate_value(raw: str) -> int:
    """argparse `type=` for `imu-rate <hz|coupled>`. Format-level parsing only:
    "coupled" (any case) -> 0, else an integer. Range validation (1-480, and the
    >60 Hz sensor-hub-cycle warning) happens through `roomscan.profiles` at dispatch
    time so the error text matches the shared model instead of a second copy of it."""
    if raw.strip().lower() == "coupled":
        return 0
    try:
        return int(raw)
    except ValueError:
        raise argparse.ArgumentTypeError(f"must be 'coupled' or an integer Hz, got {raw!r}") from None


def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="roomscan-ctl", description="Send one command to a live roomscanner board.")
    ap.add_argument("--port", help="serial port override (default: auto-detect CDC CAFE:4001)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds to wait for the ACK")
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("ping", help="PING -> ack.applied == firmware protocol version")
    sub.add_parser("calib", help="request an on-demand CALIB frame")
    p_usecase = sub.add_parser(
        "usecase", help="deprecated diagnostic: SET_USECASE <id> -- non-atomic; use `profile`/`manual`")
    p_usecase.add_argument("value", type=int)
    p_period = sub.add_parser(
        "period", help="deprecated diagnostic: SET_FRAME_PERIOD_US <microseconds> -- non-atomic and INERT "
                       "under old manual-sync firmware; use `manual --fps`")
    p_period.add_argument("value", type=int)
    p_exposure = sub.add_parser(
        "exposure", help="deprecated diagnostic: SET_EXPOSURE_MS <milliseconds> -- non-atomic; use "
                         "`manual --exposure`")
    p_exposure.add_argument("value", type=int)
    p_standby = sub.add_parser("standby", help="SET_STANDBY <0=wake|1=soft|2=hard> — idle the ToF laser")
    p_standby.add_argument("value", type=int, choices=[0, 1, 2])
    sub.add_parser("reinit", help="full sensor re-init cycle")

    p_profile = sub.add_parser(
        "profile", help="SET_RANGING_PROFILE <preset> -- one atomic preset switch "
                        "(room_mapping|precision|high_framerate)")
    p_profile.add_argument("preset", choices=sorted(_PROFILE_PRESET_CHOICES))

    p_manual = sub.add_parser(
        "manual", help="SET_MANUAL_PARAMS -- one atomic manual ranging command "
                       "(validated through roomscan.profiles before sending)")
    p_manual.add_argument("--ranging", choices=sorted(_MANUAL_RANGING_CHOICES), required=True)
    p_manual.add_argument("--fps", type=int, required=True)
    p_manual.add_argument("--exposure", type=int, required=True,
                          help="integer milliseconds (EXPOSURE_STEP_MS -- no sub-ms step)")
    p_manual.add_argument("--power", choices=sorted(_MANUAL_POWER_CHOICES), required=True)

    sub.add_parser(
        "profile-status", help="GET_RANGING_CONFIG -- read back the device's authoritative applied "
                               "ranging config")

    p_imu_rate = sub.add_parser(
        "imu-rate", help="SET_IMU_ENV_RATE <hz|coupled> -- poll streams 9/10/11 at their own rate "
                         "(coupled/0 = one sample per ToF trigger, the default)")
    p_imu_rate.add_argument("value", type=_imu_rate_value)

    sub.add_parser(
        "imu-rate-status", help="GET_IMU_ENV_RATE -- read back the device's applied IMU/env poll rate")
    return ap


def parse_command(argv=None) -> tuple[argparse.Namespace, CommandCode, int]:
    """Parse argv -> (args, cmd, param). Pure / no I/O — the testable seam
    between CLI arg handling and everything that touches a serial port.

    Only covers the legacy single-cmd/single-param actions (1-8, 11, 12's plain
    form, plus the deprecated diagnostics). The typed actions added in Task 3
    (`profile`/`manual`/`profile-status`/`imu-rate`/`imu-rate-status`) don't reduce
    to one cmd+param pair — they need `roomscan.profiles` validation and/or the
    extended ranging-config ACK — so this raises ValueError for them; `main()`
    routes those to `_main_typed`/`_TYPED_DISPATCH` instead, before ever calling
    this."""
    args = _build_parser().parse_args(argv)
    if args.action in _ACTION_COMMANDS:
        return args, _ACTION_COMMANDS[args.action], 0
    if args.action in _ACTION_COMMANDS_WITH_VALUE:
        return args, _ACTION_COMMANDS_WITH_VALUE[args.action], args.value
    if args.action in _TYPED_ACTIONS:
        raise ValueError(
            f"{args.action!r} is a typed action with no single cmd/param pair; "
            "use main()'s typed dispatch, not parse_command()")
    raise ValueError(f"unhandled action {args.action!r}")  # pragma: no cover - argparse restricts choices


def _start_reader_thread(source, decoder, client: CommandClient, stop: threading.Event):
    """The CLI's own throwaway reader loop (see module docstring's thread
    contract): feeds every decoded frame to `client.offer()` first — what it
    doesn't consume as a pending ACK is DATA (counted) or EVENT (counted +
    printed). Shared by every roomscan-ctl action; this loop, and only this
    loop, may call `source.read()` — `send()`/`send_manual_params()`/etc. write
    from the caller's own thread and never touch it."""
    counts = {"data": 0, "event": 0}

    def reader() -> None:
        while not stop.is_set():
            try:
                chunk = source.read()
            except Exception:
                return
            if not chunk:
                continue
            for frame in decoder.feed(chunk):
                if client.offer(frame):
                    continue
                if frame.header.frame_type == FrameType.EVENT:
                    counts["event"] += 1
                    try:
                        code, detail, msg = parse_event(frame.payload)
                        print(f"[device event] code={code} detail={detail} {msg}")
                    except ProtocolError:
                        print("[device event] undecodable payload")
                elif frame.header.frame_type == FrameType.DATA:
                    counts["data"] += 1

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    return thread, counts


def _do_profile(client: CommandClient, args: argparse.Namespace) -> int:
    preset = _PROFILE_PRESET_CHOICES[args.preset]
    print(f"requested: profile={args.preset} ({int(preset)})")
    result, applied = client.send_profile(preset, timeout=args.timeout)
    try:
        applied_name = protocol.ProfileId(applied).name
    except ValueError:
        applied_name = f"unknown({applied})"
    print(f"applied:   result={result.name} profile={applied_name} ({applied})")
    return 0 if result == ResultCode.OK else 1


def _do_manual(client: CommandClient, args: argparse.Namespace) -> int:
    params = profiles.ManualParams(
        ranging_mode=_MANUAL_RANGING_CHOICES[args.ranging],
        fps=args.fps,
        exposure_ms=args.exposure,
        power_mode=_MANUAL_POWER_CHOICES[args.power],
    )
    validation = profiles.validate_manual_params(params)
    for warning in validation.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    if not validation.ok:
        for error in validation.errors:
            print(f"error: {error}", file=sys.stderr)
        return 1

    # This CLI only ever talks over a serial CDC port (SerialSource, below) -- there is
    # no Ethernet path here, so the transport is truthfully always "cdc".
    transport_warning = profiles.transport_warning_message("cdc", params.fps)
    if transport_warning:
        print(f"warning: {transport_warning}", file=sys.stderr)

    period_us = profiles.fps_to_period_us(params.fps)
    print(f"requested: ranging={args.ranging} fps={params.fps} (frame_period_us={period_us}) "
          f"exposure_ms={params.exposure_ms} power={args.power}")

    result, ack = client.send_manual_params(_manual_params_to_wire(params), timeout=args.timeout)
    print(f"applied:   result={result.name} ranging_mode={protocol.RangingMode(ack.ranging_mode).name} "
          f"frame_period_us={ack.frame_period_us} exposure_ms={ack.exposure_ms} "
          f"power_mode={protocol.PowerMode(ack.power_mode).name}")

    if result != ResultCode.OK:
        return 1
    if transport_warning:
        return 1
    return 0


def _do_profile_status(client: CommandClient, args: argparse.Namespace) -> int:
    result, ack = client.get_ranging_config(timeout=args.timeout)
    print(f"applied: result={result.name} ranging_mode={protocol.RangingMode(ack.ranging_mode).name} "
          f"frame_period_us={ack.frame_period_us} exposure_ms={ack.exposure_ms} "
          f"power_mode={protocol.PowerMode(ack.power_mode).name}")
    return 0 if result == ResultCode.OK else 1


def _do_imu_rate(client: CommandClient, args: argparse.Namespace) -> int:
    rate_hz = args.value  # 0 (coupled) or an int Hz, already parsed by _imu_rate_value
    validation = profiles.validate_imu_env_rate(rate_hz or None)
    for warning in validation.warnings:
        print(f"warning: {warning}", file=sys.stderr)
    if not validation.ok:
        for error in validation.errors:
            print(f"error: {error}", file=sys.stderr)
        return 1

    label = "coupled" if rate_hz == 0 else f"{rate_hz} Hz"
    print(f"requested: imu_env_rate={label}")
    result, applied = client.send_imu_env_rate(rate_hz, timeout=args.timeout)
    applied_label = "coupled" if applied == 0 else f"{applied} Hz"
    print(f"applied:   result={result.name} imu_env_rate={applied_label}")
    return 0 if result == ResultCode.OK else 1


def _do_imu_rate_status(client: CommandClient, args: argparse.Namespace) -> int:
    result, applied = client.get_imu_env_rate(timeout=args.timeout)
    applied_label = "coupled" if applied == 0 else f"{applied} Hz"
    print(f"applied: result={result.name} imu_env_rate={applied_label}")
    return 0 if result == ResultCode.OK else 1


_TYPED_DISPATCH = {
    "profile": _do_profile,
    "manual": _do_manual,
    "profile-status": _do_profile_status,
    "imu-rate": _do_imu_rate,
    "imu-rate-status": _do_imu_rate_status,
}


def _main_typed(args: argparse.Namespace) -> int:
    """Open the command channel and run one typed ranging action. Mirrors the
    legacy branch of `main()` below (same source/reader-thread lifecycle) but
    dispatches to `_TYPED_DISPATCH` instead of a single `client.send()`, and
    additionally treats a `ProtocolError` (a malformed/wrong-shape ACK) and a
    non-empty transport warning as the "protocol mismatch" / "unsupported
    transport" nonzero-exit cases the plan calls for, alongside the existing
    timeout/device-rejection ones each `_do_*` already returns 1 for."""
    from .decoder import StreamDecoder  # deferred: keep CLI parsing importable without pyserial
    from .sources import SerialSource

    try:
        source = SerialSource(args.port, args.baud)
    except Exception as exc:  # port missing/busy/broken: report cleanly, no traceback
        print(f"error: {exc}", file=sys.stderr)
        return 1
    decoder = StreamDecoder()
    client = CommandClient(source.write)
    stop = threading.Event()
    reader_thread, _counts = _start_reader_thread(source, decoder, client, stop)
    try:
        return _TYPED_DISPATCH[args.action](client, args)
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    except ProtocolError as exc:
        print(f"error: protocol mismatch: {exc}", file=sys.stderr)
        return 1
    finally:
        stop.set()
        reader_thread.join(timeout=1.0)
        source.close()


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    if args.action in _TYPED_ACTIONS:
        return _main_typed(args)

    args, cmd, param = parse_command(argv)

    from .decoder import StreamDecoder  # deferred: keep CLI parsing importable without pyserial
    from .sources import SerialSource

    try:
        source = SerialSource(args.port, args.baud)
    except Exception as exc:  # port missing/busy/broken: report cleanly, no traceback
        print(f"error: {exc}", file=sys.stderr)
        return 1
    decoder = StreamDecoder()
    client = CommandClient(source.write)
    stop = threading.Event()
    reader_thread, _counts = _start_reader_thread(source, decoder, client, stop)
    try:
        result, applied = client.send(cmd, param, timeout=args.timeout)
    except TimeoutError as exc:
        print(f"error: {exc}", file=sys.stderr)
        stop.set()
        reader_thread.join(timeout=1.0)
        source.close()
        return 1
    stop.set()
    reader_thread.join(timeout=1.0)
    source.close()
    print(f"result={result.name} applied={applied}")
    return 0 if result == ResultCode.OK else 1


if __name__ == "__main__":
    sys.exit(main())
