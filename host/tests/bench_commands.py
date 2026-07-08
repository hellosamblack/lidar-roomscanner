"""Throwaway [HW] bench script for Phase 3 Task 2 (CDC command channel).

NOT a pytest test -- run directly against live hardware while the board is streaming:

    host/.venv/Scripts/python host/tests/bench_commands.py [PORT]

Sends PING, SEND_CALIB, an unknown command, and a CRC-corrupted command over the CDC
control port while decoding the concurrent RAW/CALIB/ACK stream, then reports stream
continuity (seq gaps, CRC failures, fps) and whether each command's expected
ACK/behavior was observed.
"""
from __future__ import annotations

import sys
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
        ser.write(frame)
        ser.flush()
        frames = []
        t0 = time.time()
        while time.time() - t0 < wait_s:
            data = ser.read(4096)
            if data:
                frames.extend(decoder.feed(data))
        return frames

    print("draining 2 s of baseline stream ...")
    drain(2.0)

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

    gaps = 0
    for a, b in zip(raw_seqs, raw_seqs[1:]):
        if b != a + 1:
            gaps += 1
    approx_fps = len(raw_seqs) / bench_elapsed if bench_elapsed > 0 else 0.0
    print(f"RAW frames observed: {len(raw_seqs)}, seq gaps: {gaps}, crc_failures: {decoder.crc_failures}, "
          f"bytes_skipped: {decoder.bytes_skipped}")
    print(f"approx fps across the {bench_elapsed:.1f} s command-handling window "
          f"(includes command round-trip pauses): {approx_fps:.1f}")

    print("\n--- pass/fail ---")
    checks = [
        ("PING acked with applied==1", ok_ping and got_ping[0][2] == 1 and got_ping[0][1] == ResultCode.OK),
        ("SEND_CALIB acked OK", ok_calib and got_calib[0][1] == ResultCode.OK),
        ("CALIB frame arrived promptly", calib_arrived),
        ("Unknown cmd -> UNKNOWN_CMD", ok_unknown and got_unknown[0][1] == ResultCode.UNKNOWN_CMD),
        ("Corrupted cmd -> no ack", not ok_corrupt),
        ("No RAW seq gaps", gaps == 0),
        ("No CRC failures on stream", decoder.crc_failures == 0),
    ]
    all_ok = True
    for label, cond in checks:
        print(f"  [{'OK' if cond else 'FAIL'}] {label}")
        all_ok = all_ok and cond
    return 0 if all_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
