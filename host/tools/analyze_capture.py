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
           5: "REFLECTANCE", 6: "STATUS", 7: "RAW_3DMD", 8: "CALIB", 9: "IMU_QUAT",
           10: "ENV", 11: "IMU_RAW", 12: "IMU_CAL"}  # keep in sync with protocol.StreamId


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


def scan(path: str, *, min_zero_run: int = 50, zero_scan_frames: int = 8,
         dump_bytes: int = 0) -> dict:
    """Decode `path` frame by frame and return every finding as data.

    Pure: reads the file, returns a dict, prints nothing. `analyze()` renders this
    as prose for the CLI; `roomscan.mcp_server` returns it as JSON. Raw file bytes
    are deliberately not included -- any hexdump is rendered here into a string so
    callers never carry megabytes around.
    """
    with open(path, "rb") as f:
        data = f.read()
    n = len(data)

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

    skip_context = []
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
        ctx: dict = {"run_start": start, "run_end": end, "run_len": end - start,
                     "prev_frame": None, "next_frame": None,
                     "zero_runs": zero_runs(data[start:end], min_zero_run)}
        if prev:
            off, ft, sid, seq, fl, pl, _t = prev
            ctx["prev_frame"] = {"offset": off, "frame_type": ft, "stream_id": sid, "seq": seq,
                                 "flags": fl, "declared_payload_len": pl,
                                 "ends_at": off + HEADER_SIZE + pl + 4,
                                 "gap_to_run": start - (off + HEADER_SIZE + pl + 4)}
        if next_:
            off, ft, sid, seq, fl, pl, _t = next_
            ctx["next_frame"] = {"offset": off, "frame_type": ft, "stream_id": sid, "seq": seq,
                                 "flags": fl, "declared_payload_len": pl}
        if dump_bytes > 0:
            d0 = max(0, start - 32)
            d1 = min(n, start + dump_bytes)
            ctx["hexdump"] = {"start": d0, "end": d1, "text": hexdump(data[d0:d1], d0)}
        skip_context.append(ctx)

    raw_zero_runs = []
    for off, ft, sid, seq, fl, pl, _t in frame_log:
        if sid != "RAW_3DMD":
            continue
        payload = data[off + HEADER_SIZE:off + HEADER_SIZE + pl]
        raw_zero_runs.append({"seq": seq, "flags": fl,
                              "zero_runs": zero_runs(payload, min_zero_run)})
        if len(raw_zero_runs) >= zero_scan_frames:
            break

    return {
        "path": path,
        "size_bytes": n,
        "frames_decoded": frames_decoded,
        "crc_failures": crc_failures,
        "bytes_skipped": bytes_skipped,
        "clean": not anomalies,
        "anomalies": anomalies,
        "skip_context": skip_context,
        "raw_zero_runs": raw_zero_runs,
        "min_zero_run": min_zero_run,
        "zero_scan_frames": zero_scan_frames,
        "frame_log": [
            {"offset": off, "frame_type": ft, "stream_id": sid, "seq": seq,
             "flags": fl, "declared_payload_len": pl, "t_us": t_us}
            for off, ft, sid, seq, fl, pl, t_us in frame_log
        ],
    }


def analyze(path: str, *, min_zero_run: int, zero_scan_frames: int,
            show_frames: bool, dump_bytes: int) -> None:
    """Print `scan()`'s findings as the prose report the CLI has always emitted."""
    r = scan(path, min_zero_run=min_zero_run, zero_scan_frames=zero_scan_frames,
             dump_bytes=dump_bytes)
    print(f"=== {path} ===")
    print(f"file size: {r['size_bytes']} bytes")
    print(f"frames_decoded={r['frames_decoded']} crc_failures={r['crc_failures']} "
          f"bytes_skipped={r['bytes_skipped']}")

    print("\n--- anomalies (file order) ---")
    if not r["anomalies"]:
        print("  none — capture decodes clean end to end")
    for a in r["anomalies"]:
        print(f"  {a}")

    print("\n--- skip-run context ---")
    for c in r["skip_context"]:
        print(f"\n  SKIP RUN [{c['run_start']}, {c['run_end']}) len={c['run_len']}")
        p = c["prev_frame"]
        if p:
            print(f"    prev good frame: off={p['offset']} {p['frame_type']}/{p['stream_id']} "
                  f"seq={p['seq']} flags=0x{p['flags']:02x} "
                  f"plen={p['declared_payload_len']} ends_at={p['ends_at']} "
                  f"(gap to run: {p['gap_to_run']} B)")
        else:
            print("    prev good frame: none (run at capture start)")
        nx = c["next_frame"]
        if nx:
            print(f"    next good frame: off={nx['offset']} {nx['frame_type']}/{nx['stream_id']} "
                  f"seq={nx['seq']} flags=0x{nx['flags']:02x} plen={nx['declared_payload_len']}")
        else:
            print("    next good frame: none (run extends to EOF)")
        print(f"    zero-runs >= {min_zero_run} B inside run "
              f"(offsets relative to run start): {c['zero_runs']}")
        if "hexdump" in c:
            print(f"    hexdump [{c['hexdump']['start']}, {c['hexdump']['end']}):")
            print(c["hexdump"]["text"])

    print(f"\n--- zero-runs >= {min_zero_run} B in the first {zero_scan_frames} good RAW payloads ---")
    for z in r["raw_zero_runs"]:
        print(f"  RAW seq={z['seq']:6d} flags=0x{z['flags']:02x} "
              f"zero-runs (payload offsets): {z['zero_runs'] if z['zero_runs'] else 'none'}")

    if show_frames:
        print("\n--- frame inventory ---")
        print(f"  {'offset':>10}  {'type':8}  {'stream':12}  {'seq':>8}  flags  {'plen':>6}  t_us")
        for f_ in r["frame_log"]:
            print(f"  {f_['offset']:>10}  {f_['frame_type']:8}  {str(f_['stream_id']):12}  "
                  f"{f_['seq']:>8}  0x{f_['flags']:02x}   {f_['declared_payload_len']:>6}  {f_['t_us']}")


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
