"""Byte-exact capture forensics: locate and characterize every decode anomaly.

Runs the same magic-scan/CRC/resync policy as roomscan.decoder.StreamDecoder over a
recorded capture, but tracks absolute file offsets so anomalies can be pinned to
bytes. Written for the Phase 3 Task 6 connect-transient investigation
(docs/connect-transient-forensics.md); kept as a general capture-forensics tool.

    host/.venv/Scripts/python host/tools/analyze_capture.py captures/e2e_p2.bin

Reports:
- summary counters (frames decoded / CRC failures / bytes skipped) — these match
  what StreamDecoder would report for the same bytes;
- every anomaly in file order: CRC-failing header candidates (with the header's
  decoded fields — a well-formed header whose frame body fails CRC is the signature
  of a truncated/aborted send), skip runs (contiguous byte ranges that produced no
  frame, with the previous/next good frame for context), and a frame truncated by
  end-of-capture;
- zero-run detection (>= --min-zero-run contiguous 0x00 bytes) inside anomalous
  regions AND inside the first --zero-scan-frames good RAW payloads — sensor warm-up
  frames legitimately contain large zero blocks (see the forensics doc), so a zero
  run inside a truncated frame is NOT by itself evidence of garbage;
- optionally (--frames) a per-frame inventory table: offset, type, stream, seq,
  flags, payload_len, t_us.

Only reads the capture; never writes. Pure stdlib.
"""
from __future__ import annotations

import argparse
import struct
import sys
import zlib

MAGIC = b"RSCN"
HEADER_SIZE = 32
_HEADER = struct.Struct("<4sBBBBIQHHII")  # magic, ver, type, stream, flags, seq, t_us, w, h, plen, reserved
MAX_PAYLOAD = 1 << 20  # decoder policy (docs/protocol.md)

FRAME_TYPES = {1: "DATA", 2: "EVENT", 3: "COMMAND", 4: "ACK"}
STREAMS = {0: "DEPTH_ZF32", 1: "DEPTH_ZAPC", 2: "AMBIENT", 3: "AMPLITUDE", 4: "CONFIDENCE",
           5: "REFLECTANCE", 6: "STATUS", 7: "RAW_3DMD", 8: "CALIB"}


def zero_runs(buf: bytes, min_len: int) -> list[tuple[int, int]]:
    """Return (offset, length) of every run of >= min_len contiguous 0x00 bytes."""
    runs = []
    i = 0
    n = len(buf)
    while i < n:
        if buf[i] == 0:
            j = i
            while j < n and buf[j] == 0:
                j += 1
            if j - i >= min_len:
                runs.append((i, j - i))
            i = j
        else:
            i += 1
    return runs


def hexdump(data: bytes, base_off: int, width: int = 16) -> str:
    lines = []
    for i in range(0, len(data), width):
        chunk = data[i:i + width]
        hexs = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"  {base_off + i:10d} (0x{base_off + i:08x}): {hexs:<{width * 3}} {asc}")
    return "\n".join(lines)


def analyze(path: str, *, min_zero_run: int, zero_scan_frames: int,
            show_frames: bool, dump_bytes: int) -> None:
    with open(path, "rb") as f:
        data = f.read()
    n = len(data)
    print(f"=== {path} ===")
    print(f"file size: {n} bytes")

    pos = 0
    frames_decoded = 0
    crc_failures = 0
    bytes_skipped = 0
    anomalies: list[dict] = []
    frame_log: list[tuple] = []  # (offset, type, stream, seq, flags, plen, t_us)
    skip_run_start: int | None = None

    def mark_skip(off: int, count: int) -> None:
        nonlocal bytes_skipped, skip_run_start
        bytes_skipped += count
        if skip_run_start is None:
            skip_run_start = off

    while pos < n:
        idx = data.find(MAGIC, pos)
        if idx < 0:
            if n - pos:
                mark_skip(pos, n - pos)
            break
        if idx > pos:
            mark_skip(pos, idx - pos)
            pos = idx
        if n - pos < HEADER_SIZE:
            mark_skip(pos, n - pos)
            break
        magic, ver, ftype, stream, flags, seq, t_us, w, h, plen, _res = _HEADER.unpack(
            data[pos:pos + HEADER_SIZE])
        if ver != 1 or plen > MAX_PAYLOAD:
            mark_skip(pos, 1)
            pos += 1
            continue
        total = HEADER_SIZE + plen + 4
        if n - pos < total:
            anomalies.append({
                "kind": "TRUNCATED_AT_EOF", "offset": pos,
                "frame_type": FRAME_TYPES.get(ftype, ftype),
                "stream_id": STREAMS.get(stream, stream), "seq": seq,
                "declared_payload_len": plen, "bytes_available": n - pos,
            })
            mark_skip(pos, n - pos)
            break
        body = data[pos:pos + total]
        (crc,) = struct.unpack_from("<I", body, total - 4)
        computed = zlib.crc32(body[:-4])
        if computed != crc:
            crc_failures += 1
            anomalies.append({
                "kind": "CRC_FAIL", "offset": pos,
                "frame_type": FRAME_TYPES.get(ftype, ftype),
                "stream_id": STREAMS.get(stream, stream), "seq": seq,
                "flags": flags, "declared_payload_len": plen,
                "t_us": t_us, "w": w, "h": h,
                "computed_crc": computed, "wire_crc": crc,
            })
            mark_skip(pos, 1)
            pos += 1
            continue
        if skip_run_start is not None:
            anomalies.append({
                "kind": "SKIP_RUN", "run_start": skip_run_start,
                "run_end": pos, "run_len": pos - skip_run_start,
            })
            skip_run_start = None
        frames_decoded += 1
        frame_log.append((pos, FRAME_TYPES.get(ftype, ftype),
                          STREAMS.get(stream, stream), seq, flags, plen, t_us))
        pos += total
    if skip_run_start is not None:
        anomalies.append({
            "kind": "SKIP_RUN", "run_start": skip_run_start,
            "run_end": n, "run_len": n - skip_run_start,
        })

    print(f"frames_decoded={frames_decoded} crc_failures={crc_failures} "
          f"bytes_skipped={bytes_skipped}")

    print("\n--- anomalies (file order) ---")
    if not anomalies:
        print("  none — capture decodes clean end to end")
    for a in anomalies:
        print(f"  {a}")

    print("\n--- skip-run context ---")
    for a in anomalies:
        if a["kind"] != "SKIP_RUN":
            continue
        start, end = a["run_start"], a["run_end"]
        prev = next_ = None
        for rec in frame_log:
            if rec[0] < start:
                prev = rec
            elif rec[0] >= end:
                next_ = rec
                break
        print(f"\n  SKIP RUN [{start}, {end}) len={end - start}")
        if prev:
            off, ft, sid, seq, fl, pl, _t = prev
            frame_end = off + HEADER_SIZE + pl + 4
            print(f"    prev good frame: off={off} {ft}/{sid} seq={seq} flags=0x{fl:02x} "
                  f"plen={pl} ends_at={frame_end} (gap to run: {start - frame_end} B)")
        else:
            print("    prev good frame: none (run at capture start)")
        if next_:
            off, ft, sid, seq, fl, pl, _t = next_
            print(f"    next good frame: off={off} {ft}/{sid} seq={seq} flags=0x{fl:02x} plen={pl}")
        else:
            print("    next good frame: none (run extends to EOF)")
        zr = zero_runs(data[start:end], min_zero_run)
        print(f"    zero-runs >= {min_zero_run} B inside run (offsets relative to run start): {zr}")
        if dump_bytes > 0:
            d0 = max(0, start - 32)
            d1 = min(n, start + dump_bytes)
            print(f"    hexdump [{d0}, {d1}):")
            print(hexdump(data[d0:d1], d0))

    print(f"\n--- zero-runs >= {min_zero_run} B in the first {zero_scan_frames} good RAW payloads ---")
    shown = 0
    for off, ft, sid, seq, fl, pl, _t in frame_log:
        if sid != "RAW_3DMD":
            continue
        payload = data[off + HEADER_SIZE:off + HEADER_SIZE + pl]
        zr = zero_runs(payload, min_zero_run)
        print(f"  RAW seq={seq:6d} flags=0x{fl:02x} zero-runs (payload offsets): {zr if zr else 'none'}")
        shown += 1
        if shown >= zero_scan_frames:
            break

    if show_frames:
        print("\n--- frame inventory ---")
        print(f"  {'offset':>10}  {'type':8}  {'stream':12}  {'seq':>8}  flags  {'plen':>6}  t_us")
        for off, ft, sid, seq, fl, pl, t_us in frame_log:
            print(f"  {off:>10}  {ft:8}  {str(sid):12}  {seq:>8}  0x{fl:02x}   {pl:>6}  {t_us}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="analyze_capture",
        description="Byte-exact forensics over a roomscanner wire-protocol capture.")
    ap.add_argument("captures", nargs="+", help="capture file(s) to analyze")
    ap.add_argument("--min-zero-run", type=int, default=50,
                    help="minimum contiguous 0x00 run length to report (default 50)")
    ap.add_argument("--zero-scan-frames", type=int, default=8,
                    help="how many leading good RAW payloads to scan for zero-runs (default 8)")
    ap.add_argument("--frames", action="store_true",
                    help="print the full per-frame inventory table")
    ap.add_argument("--dump", type=int, default=0, metavar="N",
                    help="hexdump the first N bytes of each skip run (default 0 = off)")
    args = ap.parse_args(argv)
    for i, path in enumerate(args.captures):
        if i:
            print("\n" + "=" * 90 + "\n")
        analyze(path, min_zero_run=args.min_zero_run,
                zero_scan_frames=args.zero_scan_frames,
                show_frames=args.frames, dump_bytes=args.dump)
    return 0


if __name__ == "__main__":
    sys.exit(main())
