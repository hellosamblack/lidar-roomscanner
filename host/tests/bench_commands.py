"""Throwaway [HW] bench script for Phase 3 Task 2 (CDC command channel).

NOT a pytest test -- run directly against live hardware while the board is streaming:

    host/.venv/Scripts/python host/tests/bench_commands.py [PORT]

Sends PING, SEND_CALIB, an unknown command, a CRC-corrupted command, a 3-command
back-to-back burst (one write, exceeds the firmware's 128 B rx_buf), and a 10-command
mixed burst over the CDC control port while decoding the concurrent RAW/CALIB/ACK
stream, then reports stream continuity (seq gaps, CRC failures, fps) and whether each
command's expected ACK/behavior was observed.
"""
from __future__ import annotations

import sys
import threading
import time

import serial

from roomscan.decoder import StreamDecoder
from roomscan.protocol import CommandCode, FrameType, ResultCode, StreamId, pack_command, parse_ack
from roomscan.sources import SerialSource

PORT = sys.argv[1] if len(sys.argv) > 1 else SerialSource.find_port()
BAUD = 921600  # no-op on the native CDC port, kept for parity with SerialSource


def corrupt(frame: bytes) -> bytes:
    """Flip a payload byte so the CRC no longer validates -- the frame must vanish
    entirely (no ACK, not even an UNKNOWN_CMD one) per the malformed-input contract."""
    b = bytearray(frame)
    b[32] ^= 0xFF  # inside the cmd field, well before the CRC tail
    return bytes(b)


def main() -> int:
    print(f"opening {PORT} ...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    decoder = StreamDecoder()

    def drain(seconds: float):
        t0 = time.time()
        while time.time() - t0 < seconds:
            data = ser.read(4096)
            if data:
                decoder.feed(data)

    def send_and_collect(frame: bytes, wait_s: float = 1.0):
        # Write from a separate thread: on Windows, ser.write() of a multi-command burst
        # BLOCKS until the device accepts every OUT byte, and the firmware deliberately
        # paces command intake (128 B rx_buf, 2 dispatches/poll, ~36 ms/poll) -- a 440 B
        # burst takes ~150 ms to be accepted. Blocking this read loop that long starves
        # the device's TX for >100 ms and its bounded best-effort send policy aborts one
        # RAW frame (by design: DROPPED flag, self-healing; measured as exactly 1 gap +
        # 1 crc + 1186 skipped bytes when written inline). A real host (CommandClient,
        # Task 3) writes from a thread other than the reader for the same reason.
        writer = threading.Thread(target=ser.write, args=(frame,), daemon=True)
        writer.start()
        frames = []
        t0 = time.time()
        while time.time() - t0 < wait_s:
            data = ser.read(4096)
            if data:
                frames.extend(decoder.feed(data))
        writer.join(timeout=1.0)
        return frames

    print("draining 2 s of baseline stream ...")
    drain(2.0)
    # Snapshot decoder health AFTER the baseline drain: the known connect-time
    # transient (Task 6 scope: ~1 corrupted frame at port open, self-healing) lands in
    # the drain window and must not be attributed to command handling.
    crc_at_baseline = decoder.crc_failures
    skipped_at_baseline = decoder.bytes_skipped
    print(f"baseline (connect transient window): crc_failures={crc_at_baseline}, "
          f"bytes_skipped={skipped_at_baseline}")

    raw_seqs = []
    calib_events = []
    acks = {}

    def observe(frames):
        for fr in frames:
            if fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.RAW_3DMD:
                raw_seqs.append(fr.header.seq)
            elif fr.header.frame_type == FrameType.DATA and fr.header.stream_id == StreamId.CALIB:
                calib_events.append(fr.header.seq)
            elif fr.header.frame_type == FrameType.ACK:
                cmd, result, applied = parse_ack(fr.payload)
                acks.setdefault(fr.header.seq, []).append((cmd, result, applied))

    bench_t0 = time.time()

    # --- PING ---
    token_ping = 1001
    frame = pack_command(CommandCode.PING, 0, token_ping)
    observe(send_and_collect(frame))

    # --- SEND_CALIB ---
    calib_before = len(calib_events)
    token_calib = 1002
    frame = pack_command(CommandCode.SEND_CALIB, 0, token_calib)
    observe(send_and_collect(frame))
    calib_arrived = len(calib_events) > calib_before

    # --- unknown command ---
    token_unknown = 1003
    frame = pack_command(99, 0, token_unknown)
    observe(send_and_collect(frame))

    # --- CRC-corrupted command (must produce NO ack) ---
    token_corrupt = 1004
    good = pack_command(CommandCode.PING, 0, token_corrupt)
    bad = corrupt(good)
    observe(send_and_collect(bad))

    # --- burst: 3 back-to-back commands in ONE write (132 B > the fw's 128 B rx_buf;
    # exercises parse-while-draining -- all 3 must ACK with correct tokens) ---
    burst3_tokens = [3001, 3002, 3003]
    burst3 = b"".join(pack_command(CommandCode.PING, 0, t) for t in burst3_tokens)
    observe(send_and_collect(burst3, wait_s=1.5))

    # --- 10-command mixed burst in one write: PINGs, SEND_CALIBs, unknowns.
    # Dispatch cap is 2/poll, so this drains over ~5 polls (~200 ms of frame periods);
    # all 10 must ack while the RAW stream keeps flowing gap-free. ---
    calib_before_burst = len(calib_events)
    burst10_spec = [
        (CommandCode.PING, 4001), (CommandCode.SEND_CALIB, 4002), (CommandCode.PING, 4003),
        (97, 4004), (CommandCode.PING, 4005), (CommandCode.SEND_CALIB, 4006),
        (CommandCode.PING, 4007), (98, 4008), (CommandCode.PING, 4009), (CommandCode.PING, 4010),
    ]
    burst10 = b"".join(pack_command(c, 0, t) for c, t in burst10_spec)
    observe(send_and_collect(burst10, wait_s=3.0))

    print("draining 2 more seconds of stream ...")
    t0 = time.time()
    extra = []
    while time.time() - t0 < 2.0:
        data = ser.read(4096)
        if data:
            extra.extend(decoder.feed(data))
    observe(extra)
    bench_elapsed = time.time() - bench_t0

    ser.close()

    # --- report ---
    def ack_report(token, name, expect_result=None, expect_applied=None):
        got = acks.get(token)
        ok = got is not None
        detail = f"{got}" if got else "NO ACK"
        print(f"{name}: token={token} -> {detail}")
        return ok, got

    print("\n--- results ---")
    ok_ping, got_ping = ack_report(token_ping, "PING")
    ok_calib, got_calib = ack_report(token_calib, "SEND_CALIB")
    ok_unknown, got_unknown = ack_report(token_unknown, "UNKNOWN(99)")
    ok_corrupt, got_corrupt = ack_report(token_corrupt, "CORRUPTED(should be absent)")

    print(f"CALIB arrived on SEND_CALIB request: {calib_arrived}")

    burst3_ok = all(
        acks.get(t) and acks[t][0] == (int(CommandCode.PING), int(ResultCode.OK), 1)
        for t in burst3_tokens
    )
    print(f"burst3 (3x44 B one write): tokens {burst3_tokens} -> "
          f"{[acks.get(t) for t in burst3_tokens]}")

    burst10_results = {t: acks.get(t) for _, t in burst10_spec}
    burst10_acked = sum(1 for v in burst10_results.values() if v)
    burst10_ok = burst10_acked == len(burst10_spec)
    for (c, t) in burst10_spec:
        got = acks.get(t)
        if got:
            _, result, _ = got[0]
            expect = ResultCode.UNKNOWN_CMD if c in (97, 98) else ResultCode.OK
            burst10_ok = burst10_ok and result == expect
    burst10_calib_count = len(calib_events) - calib_before_burst
    print(f"burst10 (10x44 B one write, mixed): {burst10_acked}/10 acked, "
          f"{burst10_calib_count} on-demand CALIB frames -> {burst10_results}")

    gaps = 0
    for a, b in zip(raw_seqs, raw_seqs[1:]):
        if b != a + 1:
            gaps += 1
            print(f"  seq gap: {a} -> {b} (position {raw_seqs.index(b)}/{len(raw_seqs)})")
    cmd_window_crc = decoder.crc_failures - crc_at_baseline
    cmd_window_skipped = decoder.bytes_skipped - skipped_at_baseline
    approx_fps = len(raw_seqs) / bench_elapsed if bench_elapsed > 0 else 0.0
    print(f"RAW frames observed: {len(raw_seqs)}, seq gaps: {gaps}; command window: "
          f"crc_failures={cmd_window_crc}, bytes_skipped={cmd_window_skipped} "
          f"(cumulative since connect: {decoder.crc_failures}/{decoder.bytes_skipped})")
    print(f"approx fps across the {bench_elapsed:.1f} s command-handling window "
          f"(includes command round-trip pauses): {approx_fps:.1f}")

    print("\n--- pass/fail ---")
    checks = [
        ("PING acked with applied==1", ok_ping and got_ping[0][2] == 1 and got_ping[0][1] == ResultCode.OK),
        ("SEND_CALIB acked OK", ok_calib and got_calib[0][1] == ResultCode.OK),
        ("CALIB frame arrived promptly", calib_arrived),
        ("Unknown cmd -> UNKNOWN_CMD", ok_unknown and got_unknown[0][1] == ResultCode.UNKNOWN_CMD),
        ("Corrupted cmd -> no ack", not ok_corrupt),
        ("Burst 3x one write: 3 ACKs, correct tokens", burst3_ok),
        ("Burst 10x mixed one write: 10 ACKs, correct results", burst10_ok),
        ("Burst SEND_CALIBs delivered CALIB frames", burst10_calib_count >= 2),
        ("No RAW seq gaps in command window", gaps == 0),
        ("No CRC failures in command window", cmd_window_crc == 0),
    ]
    all_ok = True
    for label, cond in checks:
        print(f"  [{'OK' if cond else 'FAIL'}] {label}")
        all_ok = all_ok and cond
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
