"""Is the board streaming over Ethernet, and how fast?

The one-liner reached for after every firmware flash on the headless box, where
CDC does not exist and `capture.py` (which is CDC-first) is the wrong tool. Two
modes:

    python host/tools/check_udp.py              # smoke test: decode a few frames, exit
    python host/tools/check_udp.py --seconds 12 # rate report over a timed window

The rate report prints per-stream counts and **both** fps conventions, labeled,
per the firmware-loop skill -- reporting one bare "fps" number caused real
confusion in the P2.5-era reports. Note the totals count *protocol* frames
across all streams, so with the IKS4A1 stacked the total runs ~3x the ToF frame
rate (RAW + IMU_QUAT + ENV, plus CALIB every 64th).
"""
import argparse
import sys
import time
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.decoder import StreamDecoder
from roomscan.sources import UdpSource


def smoke(udp, decoder, want=5):
    frames_decoded = 0
    for _ in range(50):
        data = udp.read()
        if not data:
            continue
        print(f"Received {len(data)} bytes via UDP from {udp.target_ip}")
        for frame in decoder.feed(data):
            frames_decoded += 1
            print(f"Decoded frame: {frame}")
        if frames_decoded >= want:
            print(f"Successfully received and decoded {want} UDP frames!")
            return frames_decoded
    return frames_decoded


def rate(udp, decoder, seconds):
    per_stream = Counter()
    first_us = {}
    last_us = {}
    total = 0
    t0 = time.time()
    while time.time() - t0 < seconds:
        data = udp.read()
        if not data:
            continue
        for f in decoder.feed(data):
            sid = f.header.stream_id
            per_stream[sid] += 1
            total += 1
            last_us[sid] = f.header.t_us
            first_us.setdefault(sid, f.header.t_us)
    wall = time.time() - t0

    print(f"\nsource {udp.target_ip}:{udp.target_port} · {wall:.1f} s window")
    if not total:
        print("NO FRAMES. Check the board's boot LEDs (firmware-loop skill -> Observe "
              "checklist): LD1 green solid + LD2 yellow blinking is healthy.")
        return 0
    print(f"{total} frames total · {total / wall:.2f} fps (wall-clock, all streams)")
    print(f"{'stream':>7}  {'count':>6}  {'interval fps':>12}  {'wall fps':>9}")
    for sid, n in sorted(per_stream.items()):
        span_s = (last_us[sid] - first_us[sid]) / 1e6
        interval = (n - 1) / span_s if n > 1 and span_s > 0 else float("nan")
        print(f"{sid:>7}  {n:>6}  {interval:>12.2f}  {n / wall:>9.2f}")
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=None,
                    help="measure the frame rate for this long instead of smoke-testing")
    args = ap.parse_args()

    print("Listening for UDP frames on port 5000...")
    udp = UdpSource(timeout=2.0 if args.seconds is None else 0.2)
    decoder = StreamDecoder()
    try:
        if args.seconds is None:
            n = smoke(udp, decoder)
            print(f"Done. Decoded {n} frames.")
        else:
            rate(udp, decoder, args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        udp.close()


if __name__ == "__main__":
    main()
