"""Load the hardware golden-pair fixture: one CALIB payload + seq-matched (raw, depth) pairs."""
from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.protocol import StreamId

FIXTURE = Path(__file__).parent / "fixtures" / "golden_pairs_snippet.bin"


def load_golden_pairs() -> tuple[bytes, list[tuple[bytes, bytes]]]:
    frames = StreamDecoder().feed(FIXTURE.read_bytes())
    calib = next(f.payload for f in frames if f.header.stream_id == StreamId.CALIB)
    raws = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.RAW_3DMD}
    depths = {f.header.seq: f.payload for f in frames if f.header.stream_id == StreamId.DEPTH_ZF32}
    pairs = [(raws[s], depths[s]) for s in sorted(raws.keys() & depths.keys())]
    return calib, pairs
