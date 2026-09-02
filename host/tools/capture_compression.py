"""Re-runnable measurement behind the Phase 5.5 compression go/no-go (issue #130).

    host/.venv/bin/python host/tools/capture_compression.py captures/roomSweepFull20260730.bin
    host/.venv/bin/python host/tools/capture_compression.py <capture...> --json

`docs/roadmap-history.md`'s Phase 5.5 entry quotes two numbers from a one-off
2026-07-31 investigation: LZ4 on a 2-byte de-interleaved RAW_3DMD payload gives a
"54.3% pooled" ratio (in-transit, recommended), and zstd-9 "strictly dominates"
zlib-6 at rest (1.868 vs 1.808). Both existed only as prose. This tool makes them
a re-runnable measurement so a firmware go/no-go rests on a command, not a
paragraph next time the payload shape or a codec version changes.

WHAT IT MEASURES, per stream type present in each capture

* raw payload bytes (n_frames sampled, total + per-frame mean)
* LZ4 block-mode ratio (`lz4.block`, default/fast mode -- the realistic MCU
  choice, not high-compression), with and without a 2-byte de-interleave
  (`deinterleave16`: even-offset bytes then odd-offset bytes, contiguous --
  ToF payloads are 16-bit LE, so this un-interleaves the low-entropy high byte
  from the high-entropy low byte, matching the byte-lane-entropy finding that
  motivated it)
* zstd ratio at a configurable level (default 9) and zlib ratio at a
  configurable level (default 6) -- the "at rest" comparison, measured on the
  WHOLE stream's sampled payloads concatenated into one blob (not per-frame:
  at typical ~14 KB block sizes zstd's frame overhead can make it lose to
  zlib on small/lightly-redundant blocks, which is not what the roadmap's
  whole-corpus "1.868 vs 1.808" figure measured)
* compress/decompress throughput in MB/s on THIS host, for every codec above,
  measured on a representative frame (repeated until timing is stable, never a
  single too-fast-to-time call)
* an LZ4 working-set ESTIMATE relevant to a Cortex-M33: the documented LZ4
  stream-state size (`LZ4_STREAMSIZE` in lz4.h, 16416 bytes -- the Python
  binding does not expose the C struct directly, so this is read from the
  library's own published constant, not measured) plus one frame's input
  buffer and `LZ4_COMPRESSBOUND` output buffer. This is an ESTIMATE, nothing
  built on-target -- flagged the same way the roadmap entry flags its own
  ms/frame estimate.

Ratio is reported two ways because the roadmap prose uses both conventions in
the same section: `pct_of_original` = compressed/raw*100 (smaller is better --
the in-transit "54.3%" phrasing) and `factor` = raw/compressed (bigger is
better -- the at-rest "1.868x" phrasing).

Every codec's output is round-tripped against the original bytes; a mismatch is
a hard error (`RuntimeError`), not a silently wrong ratio.

`measure_captures()` is pure: reads capture files, returns a dict. `main()`
renders it as a table; `--json` for the raw dict. `roomscan.mcp_server` wraps
the same function (`capture_compression` tool) -- one implementation, two front
ends.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import zlib
from pathlib import Path

import lz4.block
import zstandard as zstd

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

from roomscan.decoder import StreamDecoder            # noqa: E402
from roomscan.protocol import FrameType, StreamId      # noqa: E402
from roomscan.sources import FileSource, pump           # noqa: E402

DEFAULT_ZSTD_LEVEL = 9
DEFAULT_ZLIB_LEVEL = 6
DEFAULT_MAX_FRAMES_PER_STREAM = 500
# lz4.h: "#define LZ4_STREAMSIZE 16416" -- the compression context's documented
# size. Not measurable from the Python binding (it wraps the opaque C struct),
# so this is the library's own published constant, cited rather than probed.
LZ4_STREAM_STATE_BYTES = 16416
# Minimum wall time a throughput timing loop must accumulate before trusting
# the rate -- guards against a single too-fast-to-time call reading as infinite.
MIN_TIMING_S = 0.02
MAX_TIMING_ITERS = 200

_STREAM_NAMES = {s.value: s.name for s in StreamId}


def deinterleave16(payload: bytes) -> bytes:
    """Split 16-bit-LE payload bytes into (all-low-bytes, all-high-bytes).

    Even offsets (the low byte of each little-endian u16) first, then odd
    offsets (the high byte). Length-preserving for any length, including odd
    (the trailing byte, if any, lands in the low-byte plane). Reversible by
    the caller if it also knows the split point (len(payload) - len(payload)//2
    high bytes) -- not implemented here since this tool never needs to invert
    it, only measure the compression effect.
    """
    return bytes(payload[0::2]) + bytes(payload[1::2])


def _lz4_compress(data: bytes) -> bytes:
    return lz4.block.compress(data, mode="default", store_size=False)


def _lz4_decompress(data: bytes, size: int) -> bytes:
    return lz4.block.decompress(data, uncompressed_size=size)


def _zstd_compressor(level: int):
    return zstd.ZstdCompressor(level=level)


def _timed(fn, *args, min_s: float = MIN_TIMING_S, max_iters: int = MAX_TIMING_ITERS):
    """Run `fn(*args)` repeatedly until >= min_s elapsed; return (result, iters/s * len).

    Returns (last_result, mb_per_s). `mb_per_s` is None if `args[0]` (the bytes
    being processed) is empty -- there is nothing to rate.
    """
    payload = args[0]
    if not payload:
        return fn(*args), None
    start = time.perf_counter()
    result = None
    iters = 0
    elapsed = 0.0
    while iters < max_iters:
        result = fn(*args)
        iters += 1
        elapsed = time.perf_counter() - start
        if elapsed >= min_s:
            break
    mb_per_s = (len(payload) * iters / (1024 * 1024)) / elapsed if elapsed > 0 else None
    return result, mb_per_s


def lz4_compress_bound(input_size: int) -> int:
    """LZ4_COMPRESSBOUND(inputSize), the documented worst-case output size."""
    return input_size + (input_size // 255) + 16


def _codec_report(name: str, payloads: list[bytes], compress, decompress, sample: bytes) -> dict:
    """Compress every payload, sum ratio, round-trip check, time on `sample`."""
    raw_total = 0
    comp_total = 0
    for p in payloads:
        c = compress(p)
        d = decompress(c, len(p))
        if d != p:
            raise RuntimeError(f"{name}: round-trip mismatch on a {len(p)}-byte payload")
        raw_total += len(p)
        comp_total += len(c)

    comp_sample, compress_mb_s = _timed(compress, sample)
    decompress_mb_s = None
    if sample:
        decompress_mb_s = _timed(lambda c: decompress(c, len(sample)), comp_sample)[1]

    ratio = (comp_total / raw_total) if raw_total else None
    return {
        "raw_bytes": raw_total,
        "compressed_bytes": comp_total,
        "pct_of_original": round(ratio * 100.0, 2) if ratio is not None else None,
        "factor": round(raw_total / comp_total, 3) if comp_total else None,
        "compress_mb_s": round(compress_mb_s, 1) if compress_mb_s is not None else None,
        "decompress_mb_s": round(decompress_mb_s, 1) if decompress_mb_s is not None else None,
    }


def _stream_report(payloads: list[bytes], zstd_level: int, zlib_level: int) -> dict:
    sample = payloads[0] if payloads else b""
    deinterleaved = [deinterleave16(p) for p in payloads]
    dsample = deinterleave16(sample)

    # LZ4 is measured per-frame, block-independent: that is how the firmware
    # would actually use it, compressing one outgoing frame at a time with no
    # cross-frame dictionary (the "in transit" methodology).
    #
    # zstd/zlib are measured on the WHOLE stream's payloads concatenated into
    # one blob: that is the "at rest" methodology (an archived file, not a
    # per-packet block), and matters here because at typical per-frame block
    # sizes (~14 KB) zstd's frame overhead can make it LOSE to zlib on small
    # or lightly-redundant blocks -- the roadmap's "zstd-9 strictly dominates
    # zlib-6" figure was a whole-corpus measurement, not a per-frame one.
    blob = b"".join(payloads)

    zc = _zstd_compressor(zstd_level)
    zd = zstd.ZstdDecompressor()

    report = {
        "n_frames": len(payloads),
        "raw_bytes_total": sum(len(p) for p in payloads),
        "raw_bytes_per_frame_mean": (sum(len(p) for p in payloads) / len(payloads)
                                     if payloads else None),
        "lz4": _codec_report("lz4", payloads, _lz4_compress, _lz4_decompress, sample),
        "lz4_deinterleaved": _codec_report(
            "lz4_deinterleaved", deinterleaved, _lz4_compress, _lz4_decompress, dsample),
        "zstd": _codec_report(
            f"zstd-{zstd_level}", [blob] if blob else [],
            lambda d: zc.compress(d), lambda c, n: zd.decompress(c, max_output_size=n), blob),
        "zlib": _codec_report(
            f"zlib-{zlib_level}", [blob] if blob else [],
            lambda d: zlib.compress(d, zlib_level), lambda c, n: zlib.decompress(c), blob),
    }
    if payloads:
        frame_size = len(sample)
        report["m33_footprint_estimate"] = {
            "note": "ESTIMATE -- LZ4_STREAMSIZE is lz4.h's documented constant, "
                    "not measured on-target; nothing built or flashed.",
            "state_bytes": LZ4_STREAM_STATE_BYTES,
            "input_buffer_bytes": frame_size,
            "output_buffer_bytes": lz4_compress_bound(frame_size),
            "total_bytes": (LZ4_STREAM_STATE_BYTES + frame_size
                            + lz4_compress_bound(frame_size)),
        }
    return report


def _iter_stream_payloads(path: str | Path, max_frames_per_stream: int,
                          wanted: set[int] | None) -> dict[int, list[bytes]]:
    by_stream: dict[int, list[bytes]] = {}
    src = FileSource(str(path))
    dec = StreamDecoder()
    try:
        for frame in pump(src, dec):
            h = frame.header
            if h.frame_type != FrameType.DATA or not frame.payload:
                continue
            if wanted is not None and h.stream_id not in wanted:
                continue
            bucket = by_stream.setdefault(h.stream_id, [])
            if len(bucket) >= max_frames_per_stream:
                continue
            bucket.append(frame.payload)
    finally:
        src.close()
    return by_stream


def measure_captures(paths: list[str | Path], streams: list[str] | None = None,
                     zstd_level: int = DEFAULT_ZSTD_LEVEL, zlib_level: int = DEFAULT_ZLIB_LEVEL,
                     max_frames_per_stream: int = DEFAULT_MAX_FRAMES_PER_STREAM) -> dict:
    """Measure compression codecs per stream type across one or more captures.

    Pure: reads the capture files, returns a dict, prints nothing. `streams`,
    if given, is a list of `StreamId` names (case-insensitive, e.g.
    ["RAW_3DMD"]); default is every stream type with a non-empty DATA payload.
    """
    wanted: set[int] | None = None
    if streams:
        name_to_id = {s.name: s.value for s in StreamId}
        wanted = set()
        for s in streams:
            key = s.strip().upper()
            if key not in name_to_id:
                raise ValueError(f"unknown stream {s!r}; known: {sorted(name_to_id)}")
            wanted.add(name_to_id[key])

    per_capture: dict[str, dict] = {}
    pooled_payloads: dict[int, list[bytes]] = {}

    for p in paths:
        pp = Path(p)
        by_stream = _iter_stream_payloads(pp, max_frames_per_stream, wanted)
        cap_report = {}
        for sid, payloads in by_stream.items():
            name = _STREAM_NAMES.get(sid, f"STREAM_{sid}")
            cap_report[name] = _stream_report(payloads, zstd_level, zlib_level)
            pooled_payloads.setdefault(sid, []).extend(payloads)
        per_capture[str(pp)] = cap_report

    pooled = {
        _STREAM_NAMES.get(sid, f"STREAM_{sid}"): _stream_report(payloads, zstd_level, zlib_level)
        for sid, payloads in pooled_payloads.items()
    }

    return {
        "zstd_level": zstd_level,
        "zlib_level": zlib_level,
        "max_frames_per_stream": max_frames_per_stream,
        "captures": per_capture,
        "pooled": pooled,
    }


def _fmt_pct(v):
    return f"{v:.1f}%" if v is not None else "n/a"


def _fmt_factor(v):
    return f"{v:.3f}x" if v is not None else "n/a"


def _print(report: dict) -> None:
    for path, streams in report["captures"].items():
        print(f"\n== {path} ==")
        for name, s in streams.items():
            print(f"  {name}: {s['n_frames']} frames, {s['raw_bytes_total']} raw bytes")
            for codec in ("lz4", "lz4_deinterleaved", "zstd", "zlib"):
                c = s[codec]
                print(f"    {codec:20s} {_fmt_pct(c['pct_of_original']):>8s} of original "
                     f"({_fmt_factor(c['factor'])})  "
                     f"compress={c['compress_mb_s']} MB/s  decompress={c['decompress_mb_s']} MB/s")

    print("\n== pooled across all captures ==")
    for name, s in report["pooled"].items():
        print(f"  {name}: {s['n_frames']} frames, {s['raw_bytes_total']} raw bytes")
        for codec in ("lz4", "lz4_deinterleaved", "zstd", "zlib"):
            c = s[codec]
            print(f"    {codec:20s} {_fmt_pct(c['pct_of_original']):>8s} of original "
                 f"({_fmt_factor(c['factor'])})  "
                 f"compress={c['compress_mb_s']} MB/s  decompress={c['decompress_mb_s']} MB/s")
        m = s.get("m33_footprint_estimate")
        if m:
            print(f"    lz4 M33 working set (estimate): state={m['state_bytes']}B "
                 f"+ in={m['input_buffer_bytes']}B + out={m['output_buffer_bytes']}B "
                 f"= {m['total_bytes']}B")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("captures", nargs="+")
    ap.add_argument("--streams", default=None,
                    help="comma-separated StreamId names, e.g. RAW_3DMD (default: all present)")
    ap.add_argument("--zstd-level", type=int, default=DEFAULT_ZSTD_LEVEL)
    ap.add_argument("--zlib-level", type=int, default=DEFAULT_ZLIB_LEVEL)
    ap.add_argument("--max-frames-per-stream", type=int, default=DEFAULT_MAX_FRAMES_PER_STREAM)
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    paths = []
    for c in args.captures:
        p = Path(c)
        if not p.is_absolute():
            p = REPO / p
        if not p.is_file():
            print(f"no such capture: {p}", file=sys.stderr)
            return 2
        paths.append(p)

    streams = args.streams.split(",") if args.streams else None
    report = measure_captures(paths, streams=streams, zstd_level=args.zstd_level,
                              zlib_level=args.zlib_level,
                              max_frames_per_stream=args.max_frames_per_stream)
    if args.json:
        print(json.dumps(report, indent=2))
    else:
        _print(report)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
