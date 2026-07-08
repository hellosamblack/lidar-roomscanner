"""Cut host/tests/fixtures/golden_pairs_snippet.bin from a full dual-stream capture.

Regenerate with (from repo root, after a fresh captures/golden_pairs.bin exists):
    host/.venv/Scripts/python host/tests/make_golden_snippet.py

Selects the first CALIB frame present in the capture plus the first 3 complete
RAW_3DMD/DEPTH_ZF32 seq-matched pairs starting at the earliest captured seq. The
transform's TNR stage is stateful, so Task 4's replay must start at the stream's
true first processed frame: in dual-stream mode (CONF_STREAM_RAW) the firmware
holds acquisition until a host asserts DTR on the CDC port, so a from-boot capture
begins at sensor frame-counter 1 with CALIB first in the byte stream. This script
asserts nothing about absolute seq values, but a valid golden capture starts at
seq 1 -- if the reported "earliest captured seq" is higher, the capture missed the
boot window and must be redone.

Each selected frame is re-emitted via roomscan.protocol.pack_frame(header, payload).
This is NOT a re-encode from scratch -- pack_frame deterministically reproduces the
same header layout + CRC32 the firmware wrote, so pack_frame(f.header, f.payload)
is byte-identical to the original wire bytes. That equivalence is exactly what the
Phase 1 golden-fixture tests already pin (see test_protocol.py / make_fixtures.py),
so re-packing here keeps the snippet a faithful, minimal wire sample instead of a
hand-rolled approximation.
"""
from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.protocol import StreamId, pack_frame

CAPTURE = Path(__file__).parent.parent.parent / "captures" / "golden_pairs.bin"
OUT = Path(__file__).parent / "fixtures" / "golden_pairs_snippet.bin"
NUM_PAIRS = 3


def main():
    data = CAPTURE.read_bytes()
    dec = StreamDecoder()
    frames = dec.feed(data)
    assert dec.crc_failures == 0, f"capture has {dec.crc_failures} CRC failures"

    calib = [f for f in frames if f.header.stream_id == StreamId.CALIB]
    assert calib, "no CALIB frame found in capture"
    first_calib = calib[0]

    raw = {f.header.seq: f for f in frames if f.header.stream_id == StreamId.RAW_3DMD}
    depth = {f.header.seq: f for f in frames if f.header.stream_id == StreamId.DEPTH_ZF32}
    matched_seqs = sorted(set(raw) & set(depth))
    assert len(matched_seqs) >= NUM_PAIRS, (
        f"only {len(matched_seqs)} matched RAW/DEPTH pairs available, need {NUM_PAIRS}"
    )
    earliest_seqs = matched_seqs[:NUM_PAIRS]

    out = bytearray()
    out += pack_frame(first_calib.header, first_calib.payload)
    for seq in earliest_seqs:
        out += pack_frame(raw[seq].header, raw[seq].payload)
        out += pack_frame(depth[seq].header, depth[seq].payload)

    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_bytes(out)
    print(f"wrote {len(out)} bytes to {OUT}")
    print(f"  CALIB seq={first_calib.header.seq}")
    print(f"  RAW/DEPTH pairs at seqs={earliest_seqs} "
          f"(earliest captured seq overall={matched_seqs[0]})")


if __name__ == "__main__":
    main()
