from pathlib import Path

from roomscan.decoder import StreamDecoder
from roomscan.protocol import StreamId

FIXTURE = Path(__file__).parent / "fixtures" / "hw_capture_snippet.bin"


def test_real_firmware_capture_decodes():
    d = StreamDecoder()
    frames = d.feed(FIXTURE.read_bytes())
    assert len(frames) >= 3
    assert d.crc_failures == 0
    for f in frames:
        assert f.header.stream_id == StreamId.DEPTH_ZF32
        assert f.header.payload_len == f.header.width * f.header.height * 4
    seqs = [f.header.seq for f in frames]
    assert seqs == sorted(seqs)
