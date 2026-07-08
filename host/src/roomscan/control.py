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

from .protocol import (
    CommandCode,
    Frame,
    FrameType,
    ProtocolError,
    ResultCode,
    pack_command,
    parse_ack,
    parse_event,
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
        self._pending: dict[int, tuple[threading.Event, list]] = {}

    def offer(self, frame: Frame) -> bool:
        """Feed one decoded frame in. Returns True iff it was consumed as the
        awaited ACK for a pending send() (matched by token == header.seq).
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
        event, slot = entry
        try:
            slot.append(parse_ack(frame.payload))
        except ProtocolError:
            slot.append(None)
        event.set()
        return True

    def send(self, cmd: int, param: int = 0, timeout: float = 2.0) -> tuple[ResultCode, int]:
        """Write a COMMAND frame and block for its ACK. Raises TimeoutError on
        silence within `timeout` seconds."""
        token = next(self._tokens) & 0xFFFFFFFF
        event = threading.Event()
        slot: list = []
        with self._lock:
            self._pending[token] = (event, slot)
        try:
            self._write(pack_command(cmd, param, token))
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
        _cmd, result_code, applied = result
        return ResultCode(result_code), applied


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
}


def _build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(prog="roomscan-ctl", description="Send one command to a live roomscanner board.")
    ap.add_argument("--port", help="serial port override (default: auto-detect CDC CAFE:4001)")
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--timeout", type=float, default=2.0, help="seconds to wait for the ACK")
    sub = ap.add_subparsers(dest="action", required=True)
    sub.add_parser("ping", help="PING -> ack.applied == firmware protocol version")
    sub.add_parser("calib", help="request an on-demand CALIB frame")
    p_usecase = sub.add_parser("usecase", help="SET_USECASE <id>")
    p_usecase.add_argument("value", type=int)
    p_period = sub.add_parser("period", help="SET_FRAME_PERIOD_US <microseconds>")
    p_period.add_argument("value", type=int)
    p_exposure = sub.add_parser("exposure", help="SET_EXPOSURE_MS <milliseconds>")
    p_exposure.add_argument("value", type=int)
    sub.add_parser("reinit", help="full sensor re-init cycle")
    return ap


def parse_command(argv=None) -> tuple[argparse.Namespace, CommandCode, int]:
    """Parse argv -> (args, cmd, param). Pure / no I/O — the testable seam
    between CLI arg handling and everything that touches a serial port."""
    args = _build_parser().parse_args(argv)
    if args.action in _ACTION_COMMANDS:
        return args, _ACTION_COMMANDS[args.action], 0
    if args.action in _ACTION_COMMANDS_WITH_VALUE:
        return args, _ACTION_COMMANDS_WITH_VALUE[args.action], args.value
    raise ValueError(f"unhandled action {args.action!r}")  # pragma: no cover - argparse restricts choices


def main(argv=None) -> int:
    args, cmd, param = parse_command(argv)

    from .decoder import StreamDecoder  # deferred: keep CLI parsing importable without pyserial
    from .sources import SerialSource

    source = SerialSource(args.port, args.baud)
    decoder = StreamDecoder()
    client = CommandClient(source.write)
    stop = threading.Event()
    data_frames = 0
    event_frames = 0

    def reader() -> None:
        nonlocal data_frames, event_frames
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
                    event_frames += 1
                    try:
                        code, detail, msg = parse_event(frame.payload)
                        print(f"[device event] code={code} detail={detail} {msg}")
                    except ProtocolError:
                        print("[device event] undecodable payload")
                elif frame.header.frame_type == FrameType.DATA:
                    data_frames += 1

    reader_thread = threading.Thread(target=reader, daemon=True)
    reader_thread.start()
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
