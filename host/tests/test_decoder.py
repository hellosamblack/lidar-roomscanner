import struct

from roomscan.decoder import StreamDecoder
from roomscan.protocol import FrameHeader, FrameType, StreamId, pack_frame

HDR = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 1000, 2, 2, 16)
PAYLOAD = struct.pack("<4f", 1.0, 2.0, 3.0, 4.0)
FRAME = pack_frame(HDR, PAYLOAD)


def test_single_frame():
    d = StreamDecoder()
    frames = d.feed(FRAME)
    assert len(frames) == 1
    assert frames[0].header == HDR and frames[0].payload == PAYLOAD


def test_partial_feed_boundary_anywhere():
    d = StreamDecoder()
    got = []
    for i in range(len(FRAME)):          # feed one byte at a time
        got += d.feed(FRAME[i:i + 1])
    assert len(got) == 1


def test_resync_after_ascii_garbage():
    d = StreamDecoder()
    noise = b"streams_inspect: depth ZF32 54x42\r\n"
    frames = d.feed(noise + FRAME + noise + FRAME)
    assert len(frames) == 2
    assert d.bytes_skipped >= len(noise)


def test_corrupt_crc_dropped_then_recovers():
    bad = bytearray(FRAME)
    bad[40] ^= 0xFF                       # flip a payload byte
    d = StreamDecoder()
    frames = d.feed(bytes(bad) + FRAME)
    assert len(frames) == 1
    assert d.crc_failures == 1
    assert d.bytes_skipped == len(FRAME)  # every byte of the corrupted frame is discarded


def test_oversize_payload_rejected():
    hdr = FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 1 << 30)
    raw = hdr.pack() + b"x" * 8           # lies about its length; would stall a naive decoder
    d = StreamDecoder()
    frames = d.feed(raw + FRAME)          # must skip the liar and still decode the real frame
    assert len(frames) == 1
    assert frames[0].payload == PAYLOAD


def test_unknown_stream_and_frame_type_pass_through():
    hdr = FrameHeader(frame_type=2, stream_id=42, flags=0, seq=9, t_us=0,
                      width=0, height=0, payload_len=12)
    payload = struct.pack("<II", 1, 0) + b"boot"
    frame_bytes = pack_frame(hdr, payload)
    d = StreamDecoder()
    frames = d.feed(frame_bytes)
    assert len(frames) == 1 and d.crc_failures == 0
    assert frames[0].header.frame_type == 2 and frames[0].header.stream_id == 42


def test_max_payload_boundary():
    small = StreamDecoder(max_payload=16)
    ok = pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 1, 0, 2, 2, 16),
                    struct.pack("<4f", 1.0, 2.0, 3.0, 4.0))
    assert len(small.feed(ok)) == 1            # == max_payload decodes
    big = pack_frame(FrameHeader(FrameType.DATA, StreamId.DEPTH_ZF32, 0, 2, 0, 2, 2, 20),
                     struct.pack("<5f", 1.0, 2.0, 3.0, 4.0, 5.0))
    assert small.feed(big) == []               # max_payload+4 rejected
    assert small.bytes_skipped > 0
