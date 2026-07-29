"""Rewrite a capture's stream-9 quaternions with an extra roll about the sensor
boresight, so a replay exercises orientation-dependent display code.

Created 2026-07-28. The web server rolls the IR pane to gravity in 90 deg steps
(`ir_gravity_rot`) and gravity-aligns the point cloud, but most captures were taken
with the board held roughly upright -- so replaying them exercises the code with a
zero-step, near-identity rotation and proves nothing. This synthesizes the attitude
you could not be bothered to hold during a capture.

    host/.venv/bin/python host/tools/roll_capture.py \
        host/captures/verify_slam.bin /tmp/rolled90.bin --degrees 90

A 90 deg roll is the useful default: it makes the IR pane arrive PORTRAIT (42x54
instead of 54x42), which is visible end-to-end without needing to eyeball an image.
180 deg tests the upside-down case but leaves the dimensions unchanged, so verify
that one by content rather than by shape.

Only stream 9 (IMU_QUAT) payloads are touched; every other frame is re-emitted
byte-for-byte with a recomputed CRC, so the result stays a valid capture.

**LIMITATION — this cannot verify that content is STABILISED.** Only the
quaternion is rewritten, so the ToF/reflectance data does not co-rotate the way it
would if you had physically rolled the board. Replaying a synthetic roll therefore
*should* make the displayed content rotate; that is correct behaviour here, not a
bug. Use this to exercise the mechanism — dimension swaps, transform plumbing, the
square-frame fit, cache keys — and to hit angles you can't be bothered to hold.
To check that a target stays upright as the board turns, you need a real physical
boresight roll: ask the owner to record one (aim at an asymmetric, high-contrast
target, roll about the lens axis only, pause at 0/±45/±90). Trusting a synthetic
roll for this is how an inverted rotation sign survived two verification rounds
(BUG-026 follow-up 2, `docs/engineering-practices.md` → "Verifying a rotation").
"""
from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.decoder import StreamDecoder          # noqa: E402
from roomscan.protocol import FrameType, StreamId, pack_frame   # noqa: E402
from roomscan.sensors import quat_mul               # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("src", type=Path)
    ap.add_argument("dst", type=Path)
    ap.add_argument("--degrees", type=float, default=90.0,
                    help="roll about the sensor boresight (body Z); default 90")
    args = ap.parse_args()

    if args.dst.exists():
        print(f"refusing to overwrite {args.dst}", file=sys.stderr)
        return 1

    half = np.radians(args.degrees) / 2.0
    q_roll = (float(np.cos(half)), 0.0, 0.0, float(np.sin(half)))   # about body Z

    dec = StreamDecoder()
    n_quat = n_total = 0
    with open(args.src, "rb") as fin, open(args.dst, "wb") as fout:
        while True:
            chunk = fin.read(65536)
            if not chunk:
                break
            for frame in dec.feed(chunk):
                payload = frame.payload
                if (frame.header.frame_type == FrameType.DATA
                        and frame.header.stream_id == StreamId.IMU_QUAT):
                    q = struct.unpack("<4f", payload[:16])
                    # Post-multiply: a rotation expressed in the BODY frame.
                    payload = struct.pack("<4f", *quat_mul(q, q_roll)) + payload[16:]
                    n_quat += 1
                fout.write(pack_frame(frame.header, payload))
                n_total += 1

    print(f"rolled {n_quat} stream-9 frames by {args.degrees:g} deg "
          f"({n_total} frames total) -> {args.dst}")
    if n_quat == 0:
        print("WARNING: no stream-9 frames -- this capture predates the IMU, so a "
              "replay of it applies no rotation at all", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
