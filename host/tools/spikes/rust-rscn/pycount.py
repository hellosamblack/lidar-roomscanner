"""Python reference: count frames per (type,stream) + CRC failures with roomscan's StreamDecoder; time protocol-only parse."""
import sys, time, zlib
from collections import Counter
sys.path.insert(0, "/home/sam/git/personal/lidar-roomscanner/host/src")
from roomscan.decoder import StreamDecoder
from roomscan.sources import FileSource, pump

path = sys.argv[1]; reps = int(sys.argv[2]) if len(sys.argv) > 2 else 1
data = open(path, "rb").read()

# (a) whole-file feed (mirrors the Rust spike's single-buffer parse)
best = None; counts = None
for _ in range(reps):
    dec = StreamDecoder(); t0 = time.perf_counter(); frames = dec.feed(data); dt = time.perf_counter() - t0
    best = dt if best is None else min(best, dt)
    counts = Counter((f.header.frame_type, f.header.stream_id) for f in frames)
n = dec.frames_decoded
print(f"file={path} bytes={len(data)} frames={n} crc_failures={dec.crc_failures} bytes_skipped={dec.bytes_skipped}")
for (ft, sid), c in sorted(counts.items()): print(f"  type={ft} stream={sid} count={c}")
print(f"whole-file feed: {best*1e3:.1f} ms  {n/best:.0f} frames/s  {best*1e3/n:.4f} ms/frame (best of {reps})")

# (b) production path: FileSource 4 KiB chunks -> pump -> StreamDecoder (how replay actually runs)
t0 = time.perf_counter(); dec2 = StreamDecoder(); k = sum(1 for _ in pump(FileSource(path), dec2)); dt = time.perf_counter() - t0
print(f"pump(FileSource 4KiB): frames={k} crc_failures={dec2.crc_failures} {dt*1e3:.1f} ms  {k/dt:.0f} frames/s  {dt*1e3/k:.4f} ms/frame")

# (c) CRC cost alone
t0 = time.perf_counter(); zlib.crc32(data); dt = time.perf_counter() - t0
print(f"zlib.crc32 whole file: {dt*1e3:.2f} ms -> {len(data)/dt/1e6:.0f} MB/s")
