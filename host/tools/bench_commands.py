"""[HW] command-channel bench, promoted from the Phase 3 Task 2 throwaway script
(``host/tests/bench_commands.py``, kept as-is for historical reference) into a
proper ``host/tools/`` CLI, rebased on ``roomscan.control.CommandClient``.

Exercises the CDC command channel (docs/protocol.md COMMAND/ACK) against a live,
already-streaming board while a background reader thread keeps decoding the
concurrent RAW/CALIB stream, so every scenario reports its stream-continuity cost
(seq gaps, CRC failures, bytes skipped, fps) in the same style as the original
Task 2 bench.

    host/.venv/Scripts/python host/tools/bench_commands.py --port COM15 ping
    host/.venv/Scripts/python host/tools/bench_commands.py calib --count 2
    host/.venv/Scripts/python host/tools/bench_commands.py burst 3
    host/.venv/Scripts/python host/tools/bench_commands.py corrupted-frame
    host/.venv/Scripts/python host/tools/bench_commands.py mixed-burst
    host/.venv/Scripts/python host/tools/bench_commands.py all

Writes off the reader thread (the hard-won Task 2 rule)
---------------------------------------------------------
``CommandClient.send()`` already encodes it: writes happen on whatever thread
calls ``send()``, decoded-frame matching happens via ``offer()`` on the thread
that owns the decoder -- never the same thread. `ping`/`calib` here go through
`CommandClient.send()` directly. `burst`/`corrupted-frame`/`mixed-burst` write
MULTIPLE raw command frames in ONE ``write()`` call BY DESIGN (that concatenation
is the scenario under test -- it doesn't fit `send()`'s one-shot request/response
shape), so they build frames with `roomscan.protocol.pack_command` and match ACKs
by token directly; those writes still run on a dedicated writer thread, never the
reader thread, same rule, just not funneled through `CommandClient`'s wrapper.

CALIB cadence-vs-on-demand ambiguity (Task 2 review minor, now fixed)
-----------------------------------------------------------------------
SEND_CALIB's immediate CALIB frame carries a `seq` built the same way as a
periodic 64-frame retransmit (docs/protocol.md), so a raw count can't tell them
apart. `CalibClassifier` discriminates primarily by `seq`: the first CALIB frame
seen establishes a residue (`seq % 64`); periodic retransmits land on that residue
by construction, so anything off it is unambiguously on-demand. The rare case
where an on-demand frame's `seq` coincides with the periodic residue by chance is
flagged "ambiguous" using send-time correlation (arrived within 1s of a SEND_CALIB
request) as a tiebreaker, rather than silently mis-attributed either way.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.control import CommandClient  # noqa: E402
from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.protocol import (  # noqa: E402
    CommandCode,
    FrameType,
    ProtocolError,
    ResultCode,
    StreamId,
    pack_command,
    parse_ack,
)
from roomscan.sources import SerialSource  # noqa: E402

CALIB_CADENCE = 64
BASELINE_DRAIN_S = 2.0


class CalibClassifier:
    """Discriminate periodic (every-64-frame) CALIB retransmits from on-demand
    (SEND_CALIB-triggered) ones. See the module docstring for the rationale."""

    def __init__(self) -> None:
        self.residue: int | None = None
        self.periodic: list[tuple[int, float]] = []
        self.on_demand: list[tuple[int, float]] = []
        self.ambiguous: list[tuple[int, float]] = []

    def observe(self, seq: int, t: float, recent_send_calib_t: float | None) -> str:
        if self.residue is None:
            self.residue = seq % CALIB_CADENCE
            self.periodic.append((seq, t))
            return "periodic (baseline)"
        aligned = (seq % CALIB_CADENCE) == self.residue
        recently_requested = recent_send_calib_t is not None and (t - recent_send_calib_t) < 1.0
        if aligned and not recently_requested:
            self.periodic.append((seq, t))
            return "periodic"
        if not aligned:
            self.on_demand.append((seq, t))
            return "on-demand"
        self.ambiguous.append((seq, t))
        return "ambiguous (seq aligns with the periodic residue but arrived within 1s of a SEND_CALIB request)"


class Bench:
    """Owns the port, the decoder, the CommandClient, and a background reader
    thread. `window()` gives every scenario the same before/after stream-
    continuity accounting the original Task 2 bench used."""

    def __init__(self, port: str | None, baud: int):
        self.source = SerialSource(port, baud)
        self.decoder = StreamDecoder()
        self.client = CommandClient(self.source.write)
        self.calib = CalibClassifier()
        self.raw_seqs: list[int] = []
        self.calib_events: list[tuple[int, str]] = []  # (seq, classification)
        self.acks: dict[int, list[tuple[int, int, int]]] = {}
        self.last_send_calib_t: float | None = None
        self.baseline_crc = 0
        self.baseline_skipped = 0
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._reader, daemon=True)

    def start(self) -> None:
        self._thread.start()
        print(f"draining {BASELINE_DRAIN_S}s of baseline stream ...")
        time.sleep(BASELINE_DRAIN_S)
        with self._lock:
            self.baseline_crc = self.decoder.crc_failures
            self.baseline_skipped = self.decoder.bytes_skipped
        print(f"baseline (connect transient window): crc_failures={self.baseline_crc}, "
              f"bytes_skipped={self.baseline_skipped}")

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.source.close()

    def _reader(self) -> None:
        while not self._stop.is_set():
            try:
                data = self.source.read()
            except Exception:
                return
            if not data:
                continue
            for frame in self.decoder.feed(data):
                self.client.offer(frame)  # completes any pending CommandClient.send()
                ft = frame.header.frame_type
                with self._lock:
                    if ft == FrameType.DATA and frame.header.stream_id == StreamId.RAW_3DMD:
                        self.raw_seqs.append(frame.header.seq)
                    elif ft == FrameType.DATA and frame.header.stream_id == StreamId.CALIB:
                        cls = self.calib.observe(frame.header.seq, time.monotonic(),
                                                  self.last_send_calib_t)
                        self.calib_events.append((frame.header.seq, cls))
                    elif ft == FrameType.ACK:
                        try:
                            self.acks.setdefault(frame.header.seq, []).append(
                                parse_ack(frame.payload))
                        except ProtocolError:
                            pass

    def window(self, label: str, fn):
        """Run `fn()`, then report this window's stream-continuity cost relative
        to a snapshot taken just before `fn()` -- Task-2-style per-window
        accounting (seq gaps, CRC failures, bytes skipped, fps)."""
        with self._lock:
            crc0 = self.decoder.crc_failures
            skip0 = self.decoder.bytes_skipped
            seqs_before = list(self.raw_seqs)
        t0 = time.monotonic()
        result = fn()
        elapsed = time.monotonic() - t0
        with self._lock:
            crc1 = self.decoder.crc_failures
            skip1 = self.decoder.bytes_skipped
            seqs_after = list(self.raw_seqs)
        new_seqs = seqs_after[len(seqs_before):]
        check_seqs = (seqs_before[-1:] if seqs_before else []) + new_seqs
        gaps = sum(1 for a, b in zip(check_seqs, check_seqs[1:]) if b != a + 1)
        fps = len(new_seqs) / elapsed if elapsed > 0 else 0.0
        print(f"\n--- window: {label} ({elapsed:.2f}s) ---")
        print(f"  RAW frames: {len(new_seqs)}  fps: {fps:.1f}  seq gaps: {gaps}  "
              f"crc_failures: {crc1 - crc0}  bytes_skipped: {skip1 - skip0}")
        return result


def act_ping(bench: Bench, count: int) -> bool:
    def fn():
        return [bench.client.send(CommandCode.PING, 0, timeout=2.0) for _ in range(count)]

    results = bench.window(f"ping x{count}", fn)
    ok = all(r == (ResultCode.OK, 1) for r in results)
    print(f"  ping results: {results} -> {'OK' if ok else 'FAIL'}")
    return ok


def act_calib(bench: Bench, count: int) -> bool:
    def fn():
        results = []
        for _ in range(count):
            bench.last_send_calib_t = time.monotonic()
            results.append(bench.client.send(CommandCode.SEND_CALIB, 0, timeout=2.0))
        time.sleep(0.3)  # let the triggered CALIB frame(s) arrive and get classified
        return results

    before = len(bench.calib_events)
    results = bench.window(f"calib x{count}", fn)
    new_events = bench.calib_events[before:]
    ok = all(r[0] == ResultCode.OK for r in results) and len(new_events) >= count
    print(f"  calib results: {results}")
    print(f"  CALIB frames observed: {new_events}")
    return ok


def act_burst(bench: Bench, n: int) -> bool:
    tokens = [9000 + i for i in range(n)]
    frame_bytes = b"".join(pack_command(CommandCode.PING, 0, t) for t in tokens)

    def fn():
        writer = threading.Thread(target=bench.source.write, args=(frame_bytes,), daemon=True)
        writer.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0 and sum(1 for t in tokens if t in bench.acks) < n:
            time.sleep(0.02)
        writer.join(timeout=1.0)

    bench.window(f"burst {n}x PING in one write ({len(frame_bytes)} B)", fn)
    got = {t: bench.acks.get(t) for t in tokens}
    ok = all(v and v[0] == (int(CommandCode.PING), int(ResultCode.OK), 1) for v in got.values())
    print(f"  burst acks: {got} -> {'OK' if ok else 'FAIL'}")
    return ok


def act_corrupted(bench: Bench) -> bool:
    token = 9500
    good = pack_command(CommandCode.PING, 0, token)
    bad = bytearray(good)
    bad[32] ^= 0xFF  # inside the cmd field, well before the CRC tail (matches the Task 2 bench)
    bad = bytes(bad)

    def fn():
        writer = threading.Thread(target=bench.source.write, args=(bad,), daemon=True)
        writer.start()
        time.sleep(1.0)
        writer.join(timeout=1.0)

    bench.window("corrupted PING (flipped cmd byte)", fn)
    ok = token not in bench.acks
    print(f"  ack for corrupted token {token}: {bench.acks.get(token)} -> "
          f"{'OK (correctly absent)' if ok else 'FAIL (should be absent)'}")
    return ok


def act_mixed_burst(bench: Bench) -> bool:
    spec = [(CommandCode.PING, 8001), (CommandCode.SEND_CALIB, 8002), (CommandCode.PING, 8003),
             (97, 8004), (CommandCode.PING, 8005), (CommandCode.SEND_CALIB, 8006),
             (CommandCode.PING, 8007), (98, 8008), (CommandCode.PING, 8009), (CommandCode.PING, 8010)]
    frame_bytes = b"".join(pack_command(c, 0, t) for c, t in spec)
    tokens = [t for _, t in spec]

    def fn():
        bench.last_send_calib_t = time.monotonic()
        writer = threading.Thread(target=bench.source.write, args=(frame_bytes,), daemon=True)
        writer.start()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0 and sum(1 for t in tokens if t in bench.acks) < len(tokens):
            time.sleep(0.02)
        writer.join(timeout=1.0)
        time.sleep(0.3)  # let any triggered CALIB frames arrive and get classified

    calib_before = len(bench.calib_events)
    bench.window(f"mixed burst {len(spec)}x in one write ({len(frame_bytes)} B)", fn)
    got = {t: bench.acks.get(t) for _, t in spec}
    ok = True
    for c, t in spec:
        v = got.get(t)
        expect = ResultCode.UNKNOWN_CMD if c in (97, 98) else ResultCode.OK
        ok = ok and bool(v) and v[0][1] == expect
    new_calib = bench.calib_events[calib_before:]
    on_demand = sum(1 for _, cls in new_calib if cls == "on-demand")
    print(f"  mixed burst acks: {got}")
    print(f"  CALIB frames triggered: {new_calib} (classified on-demand: {on_demand}, expect >= 2)")
    ok = ok and on_demand >= 2
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="bench_commands",
        description="[HW] bench for the CDC command channel; run against a live, streaming board.")
    ap.add_argument("--port", help="serial port override (default: auto-detect CDC CAFE:4001)")
    ap.add_argument("--baud", type=int, default=921600, help="no-op on the native CDC port")
    sub = ap.add_subparsers(dest="action", required=True)
    p_ping = sub.add_parser("ping", help="PING, expect applied == protocol version")
    p_ping.add_argument("--count", type=int, default=1)
    p_calib = sub.add_parser("calib", help="SEND_CALIB, expect an on-demand CALIB frame")
    p_calib.add_argument("--count", type=int, default=1)
    p_burst = sub.add_parser("burst", help="N PINGs in one write() call")
    p_burst.add_argument("n", type=int, nargs="?", default=3)
    sub.add_parser("corrupted-frame", help="CRC-corrupted command must produce NO ack")
    sub.add_parser("mixed-burst", help="10 mixed valid/invalid commands in one write() call")
    sub.add_parser("all", help="run every scenario in sequence (matches the original Task 2 bench)")
    args = ap.parse_args(argv)

    try:
        bench = Bench(args.port, args.baud)
    except Exception as exc:  # port missing/busy/broken: report cleanly, no traceback
        print(f"error: {exc}", file=sys.stderr)
        return 1

    bench.start()
    try:
        if args.action == "ping":
            ok = act_ping(bench, args.count)
        elif args.action == "calib":
            ok = act_calib(bench, args.count)
        elif args.action == "burst":
            ok = act_burst(bench, args.n)
        elif args.action == "corrupted-frame":
            ok = act_corrupted(bench)
        elif args.action == "mixed-burst":
            ok = act_mixed_burst(bench)
        elif args.action == "all":
            results = [
                act_ping(bench, 1),
                act_calib(bench, 1),
                act_burst(bench, 3),
                act_corrupted(bench),
                act_mixed_burst(bench),
            ]
            ok = all(results)
        else:  # pragma: no cover - argparse restricts choices
            ok = False
    finally:
        bench.stop()

    print(f"\n{'ALL OK' if ok else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
