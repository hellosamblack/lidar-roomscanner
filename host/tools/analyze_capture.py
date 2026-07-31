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
- stream continuity: per-stream `seq` gap accounting, i.e. frames the device sent
  that never reached the recorder. This is a DIFFERENT property from `clean`, which
  only means the bytes that arrived decode end to end. A capture can be perfectly
  byte-clean and still be missing seconds of frames — three 2026-07-31 multi-room
  captures were `clean: true` while losing 2.3% / 4.3% / 9.4% of RAW frames, one of
  them in a single 215-frame (7.1 s) hole. Read `continuity.complete`, not `clean`,
  before trusting a capture's coverage;
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
           10: "ENV", 11: "IMU_RAW", 12: "IMU_CAL",
           13: "IMU_SYNC"}  # keep in sync with protocol.StreamId


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


# Streams carrying exactly one frame per capture frame-group, so a seq gap in any of
# them is a lost frame. CALIB/IMU_CAL ride a 64-frame cadence (docs/protocol.md) and
# are censused separately -- their "gaps" of 64 are the design, not loss.
CONTINUOUS_STREAMS = ("DEPTH_ZF32", "RAW_3DMD", "IMU_QUAT", "ENV", "IMU_RAW", "IMU_SYNC")
CADENCED_STREAMS = ("CALIB", "IMU_CAL")
CALIB_CADENCE = 64
# Preference order for the stream whose timeline defines the capture's duration/fps.
_REFERENCE_ORDER = ("RAW_3DMD", "DEPTH_ZF32", "IMU_QUAT")


def _gap_histogram(sizes: list[int]) -> dict[str, int]:
    """Bucket gap sizes so one 215-frame hole reads differently from 215 singletons."""
    buckets = {"1": 0, "2-5": 0, "6-30": 0, "31-90": 0, "91+": 0}
    for s in sizes:
        if s == 1:
            buckets["1"] += 1
        elif s <= 5:
            buckets["2-5"] += 1
        elif s <= 30:
            buckets["6-30"] += 1
        elif s <= 90:
            buckets["31-90"] += 1
        else:
            buckets["91+"] += 1
    return buckets


def _stream_continuity(records: list[tuple[int, int]], t0: int | None) -> dict:
    """Census one stream's seq coverage. `records` is [(seq, t_us)], any order."""
    recs = sorted(set(records))  # by seq; dedupe exact retransmits
    seqs = [s for s, _ in recs]
    lo, hi = seqs[0], seqs[-1]
    span = hi - lo + 1
    received = len(seqs)
    gaps = []
    for (a, t_a), (b, _t_b) in zip(recs, recs[1:]):
        if b - a > 1:
            gaps.append({"after_seq": a, "missing": b - a - 1, "t_us": t_a,
                         "t_s": None if t0 is None else round((t_a - t0) / 1e6, 2)})
    sizes = [g["missing"] for g in gaps]
    missing = sum(sizes)
    return {
        "first_seq": lo, "last_seq": hi, "span": span, "received": received,
        "missing": missing,
        "loss_pct": round(100.0 * missing / span, 3) if span else 0.0,
        "gap_events": len(gaps),
        "max_gap": max(sizes) if sizes else 0,
        "gap_histogram": _gap_histogram(sizes),
        "worst_gaps": sorted(gaps, key=lambda g: -g["missing"])[:5],
    }


def _cadence_continuity(records: list[tuple[int, int]]) -> dict:
    """CALIB/IMU_CAL ride a fixed cadence; a spacing above it means one was lost."""
    seqs = sorted({s for s, _ in records})
    spacings = [b - a for a, b in zip(seqs, seqs[1:])]
    over = [s for s in spacings if s > CALIB_CADENCE]
    return {
        "received": len(seqs),
        "cadence": CALIB_CADENCE,
        "spacings_seen": sorted(set(spacings)),
        "missed": sum(s // CALIB_CADENCE - 1 for s in over),
    }


def stream_continuity(frame_log: list[tuple]) -> dict:
    """Which frames the device sent that never made it into this capture.

    Independent of `clean`: `clean` says the arrived bytes decode, this says whether
    everything arrived. Only DATA frames are censused -- EVENT/ACK carry their own
    seq space and would otherwise read as one enormous gap.
    """
    by_stream: dict[str, list[tuple[int, int]]] = {}
    for _off, ftype, sid, seq, _fl, _pl, t_us in frame_log:
        if ftype != "DATA":
            continue
        by_stream.setdefault(str(sid), []).append((seq, t_us))
    if not by_stream:
        return {"complete": True, "streams": {}, "cadenced": {}, "reference_stream": None,
                "duration_s": 0.0, "frames_lost": 0, "loss_pct": 0.0,
                "whole_group_lost": 0, "partial_group_lost": 0}

    reference = next((s for s in _REFERENCE_ORDER if s in by_stream), None)
    t0 = duration = None
    if reference:
        ts = [t for _s, t in by_stream[reference]]
        t0, t1 = min(ts), max(ts)
        duration = (t1 - t0) / 1e6

    streams = {name: _stream_continuity(recs, t0)
               for name, recs in by_stream.items() if name in CONTINUOUS_STREAMS}
    cadenced = {name: _cadence_continuity(recs)
                for name, recs in by_stream.items() if name in CADENCED_STREAMS}

    # Whole-group loss (the link went away) vs single-stream loss (a big RAW datagram
    # lost a fragment while its 20-byte siblings arrived) are different faults.
    whole = partial = 0
    if streams:
        lo = max(s["first_seq"] for s in streams.values())
        hi = min(s["last_seq"] for s in streams.values())
        seen = {name: {sq for sq, _t in by_stream[name]} for name in streams}
        for seq in range(lo, hi + 1):
            absent = sum(1 for got in seen.values() if seq not in got)
            if absent == len(seen):
                whole += 1
            elif absent:
                partial += 1

    ref = streams.get(reference) if reference else None
    return {
        "complete": all(s["missing"] == 0 for s in streams.values())
                    and all(c["missed"] == 0 for c in cadenced.values()),
        "reference_stream": reference,
        "duration_s": round(duration, 1) if duration else 0.0,
        "device_fps": (round(ref["span"] / duration, 2)
                       if ref and duration else 0.0),
        "received_fps": (round(ref["received"] / duration, 2)
                         if ref and duration else 0.0),
        "frames_lost": ref["missing"] if ref else 0,
        "loss_pct": ref["loss_pct"] if ref else 0.0,
        "whole_group_lost": whole,
        "partial_group_lost": partial,
        "streams": streams,
        "cadenced": cadenced,
    }


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
        "continuity": stream_continuity(frame_log),
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

    c = r["continuity"]
    print("\n--- stream continuity (frames sent but never received) ---")
    if not c["streams"]:
        print("  no DATA frames to census")
    elif c["complete"]:
        print(f"  complete — every seq present on {len(c['streams'])} stream(s), "
              f"{c['duration_s']} s @ {c['device_fps']} fps")
    else:
        print(f"  INCOMPLETE — {c['frames_lost']} {c['reference_stream']} frames lost "
              f"({c['loss_pct']}%) over {c['duration_s']} s")
        print(f"  device produced {c['device_fps']} fps; recorder kept {c['received_fps']} fps")
        print(f"  whole-group losses: {c['whole_group_lost']} (link outage)   "
              f"single-stream: {c['partial_group_lost']} (e.g. fragment loss)")
        for name, s in sorted(c["streams"].items()):
            print(f"    {name:12} recv {s['received']:>6}/{s['span']:<6} "
                  f"lost {s['missing']:>5} ({s['loss_pct']:>6}%)  "
                  f"gaps={s['gap_events']:>4} max={s['max_gap']:>4}  {s['gap_histogram']}")
        for name, cad in sorted(c["cadenced"].items()):
            print(f"    {name:12} {cad['received']} seen, cadence {cad['cadence']}, "
                  f"spacings {cad['spacings_seen']}, missed {cad['missed']}")
        ref = c["streams"].get(c["reference_stream"], {})
        if ref.get("worst_gaps"):
            print("  worst gaps:", ", ".join(
                f"{g['missing']}@{g['t_s']}s" for g in ref["worst_gaps"]))

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
