import struct
import zlib

import pytest

from roomscan.decoder import StreamDecoder
from roomscan.protocol import Frame, FrameHeader, FrameType, StreamId, pack_frame
from tools.pcap2capture import PcapFormatError, Reassembler, convert

MAGIC_US_LE = 0xA1B2C3D4
MAGIC_US_BE = 0xD4C3B2A1
MAGIC_NS_LE = 0xA1B23C4D
MAGIC_NS_BE = 0x4D3CB2A1
MAGIC_PCAPNG = 0x0A0D0D0A

LINKTYPE_ETHERNET = 1

FRAG_SIZE = 1400
SRC_MAC = bytes.fromhex("aabbccddeeff")
DST_MAC = bytes.fromhex("112233445566")
SRC_IP = "10.0.0.5"
DST_IP = "10.0.0.9"
SCANNER_PORT = 5000
HOST_PORT = 54321


def _ip4(dst_ip: str) -> bytes:
    return bytes(int(o) for o in dst_ip.split("."))


def _udp_packet(src_port, dst_port, payload: bytes) -> bytes:
    """Build one Ethernet(IPv4/UDP) link-layer frame carrying `payload`."""
    udp_len = 8 + len(payload)
    udp_hdr = struct.pack(">HHHH", src_port, dst_port, udp_len, 0)
    udp = udp_hdr + payload

    total_len = 20 + len(udp)
    ip_hdr = struct.pack(
        ">BBHHHBBH4s4s",
        0x45, 0, total_len, 0, 0, 64, 17, 0,
        _ip4(SRC_IP), _ip4(DST_IP),
    )
    # IP header checksum -- not validated by the tool, zero is fine, but compute a
    # real one for realism.
    ip_hdr = _with_ip_checksum(ip_hdr)

    eth = DST_MAC + SRC_MAC + struct.pack(">H", 0x0800)
    return eth + ip_hdr + udp


def _with_ip_checksum(hdr: bytes) -> bytes:
    def checksum(data: bytes) -> int:
        if len(data) % 2:
            data += b"\x00"
        s = sum(struct.unpack(f">{len(data) // 2}H", data))
        s = (s & 0xFFFF) + (s >> 16)
        s = (s & 0xFFFF) + (s >> 16)
        return (~s) & 0xFFFF

    hdr = hdr[:10] + b"\x00\x00" + hdr[12:]
    return hdr[:10] + struct.pack(">H", checksum(hdr)) + hdr[12:]


# (real on-wire magic value, struct endian prefix) for each of the four detection
# constants above -- e.g. a BE file's raw bytes decode to MAGIC_US_BE only when read
# with "<I" (see pcap2capture._detect_format); to WRITE that file we must instead pack
# the canonical magic 0xa1b2c3d4 with ">I".
_MAGIC_WRITE = {
    MAGIC_US_LE: (0xA1B2C3D4, "<"),
    MAGIC_US_BE: (0xA1B2C3D4, ">"),
    MAGIC_NS_LE: (0xA1B23C4D, "<"),
    MAGIC_NS_BE: (0xA1B23C4D, ">"),
}


def _pcap(packets, magic=MAGIC_US_LE, linktype=LINKTYPE_ETHERNET, snap_last_to=None) -> bytes:
    """Build a classic-pcap byte string. `packets` is a list of
    (ts_seconds, link_layer_frame_bytes). `snap_last_to` optionally truncates the
    incl_len of the last record to simulate a snaplen-limited capture."""
    real_magic, endian = _MAGIC_WRITE[magic]
    ns = magic in (MAGIC_NS_LE, MAGIC_NS_BE)

    out = struct.pack(endian + "IHHiIII", real_magic, 2, 4, 0, 0, 65535, linktype)
    n = len(packets)
    for i, (ts, frame) in enumerate(packets):
        ts_sec = int(ts)
        frac = ts - ts_sec
        ts_frac = int(round(frac * (1e9 if ns else 1e6)))
        orig_len = len(frame)
        incl_len = orig_len
        data = frame
        if snap_last_to is not None and i == n - 1:
            incl_len = snap_last_to
            data = frame[:snap_last_to]
        out += struct.pack(endian + "IIII", ts_sec, ts_frac, incl_len, orig_len)
        out += data
    return out


def _real_frame() -> bytes:
    """One real protocol frame (header + payload + CRC), built via roomscan.protocol
    exactly as the firmware/host would produce it."""
    payload = bytes(range(256)) * 8  # 2048 bytes, arbitrary content
    header = FrameHeader(FrameType.DATA, StreamId.RAW_3DMD, 0, 42, 1_000_000, 0, 0, len(payload))
    return pack_frame(header, payload)


def _fragment(frame: bytes, frag_size=FRAG_SIZE):
    """Split a frame into (seq, frag_idx, total_frags, chunk) fragment payloads with the
    6-byte <IBB header UdpSource expects."""
    seq = 7
    chunks = [frame[i:i + frag_size] for i in range(0, len(frame), frag_size)]
    total = len(chunks)
    frags = []
    for idx, chunk in enumerate(chunks):
        hdr = struct.pack("<IBB", seq, idx, total)
        frags.append(hdr + chunk)
    return frags


def _write_pcap(tmp_path, name, packets, **kw):
    p = tmp_path / name
    p.write_bytes(_pcap(packets, **kw))
    return p


def test_clean_frame_byte_exact(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(1000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["frames_incomplete"] == 0
    assert stats["frags_lost"] == 0
    assert stats["frags_reordered"] == 0
    assert stats["frags_duplicate"] == 0
    assert stats["frags_invalid"] == 0
    assert stats["frames_written"] == 1
    assert stats["frame_loss_pct"] == 0.0


def test_reordered_fragments_byte_exact(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    assert len(frags) >= 2
    reordered = [frags[1], frags[0]] + frags[2:]
    packets = [(2000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(reordered)]
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["frags_reordered"] > 0
    assert stats["frames_incomplete"] == 0


def test_dropped_fragment_counts_incomplete(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    assert len(frags) >= 2
    dropped = frags[:-1]  # drop the last fragment
    packets = [(3000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(dropped)]
    # A subsequent frame with a different seq to force retirement of the partial one.
    next_frame_frag = struct.pack("<IBB", 8, 0, 1) + b"x" * 10
    packets.append((3999.0, _udp_packet(SCANNER_PORT, HOST_PORT, next_frame_frag)))
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert stats["frames_incomplete"] == 1
    assert stats["frags_lost"] == 1
    denom = stats["frames_written"] + stats["frames_incomplete"]
    expected_pct = stats["frames_incomplete"] / denom * 100.0
    assert stats["frame_loss_pct"] == pytest.approx(expected_pct)


def test_duplicate_fragment_byte_exact(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    with_dup = [frags[0], frags[0]] + frags[1:]
    packets = [(4000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(with_dup)]
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["frags_duplicate"] == 1


def test_invalid_fragment_counted(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    total = len(frags)
    # frag_idx >= total_frags is invalid.
    bad = struct.pack("<IBB", 7, total + 5, total) + b"garbage"
    packets = [(5000.0, _udp_packet(SCANNER_PORT, HOST_PORT, bad))]
    packets += [(5000.0 + (i + 1) * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert stats["frags_invalid"] >= 1
    assert out.read_bytes() == frame


def test_output_decodes_with_stream_decoder(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(6000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    convert([str(pcap)], out_path=out)

    dec = StreamDecoder()
    decoded = dec.feed(out.read_bytes())
    assert dec.crc_failures == 0
    assert len(decoded) == 1
    assert isinstance(decoded[0], Frame)
    assert decoded[0].header.stream_id == StreamId.RAW_3DMD


@pytest.mark.parametrize("magic", [MAGIC_US_LE, MAGIC_US_BE, MAGIC_NS_LE, MAGIC_NS_BE])
def test_all_magics_parse_identically(tmp_path, magic):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(7000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    pcap = _write_pcap(tmp_path, f"ring_{magic:x}.pcap", packets, magic=magic)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["frames_written"] == 1


def test_keepalives_and_wrong_port_ignored(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(8000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    # 1-byte keepalive TO port 5000 (host -> scanner direction).
    packets.append((8500.0, _udp_packet(HOST_PORT, SCANNER_PORT, b"\x00")))
    # Unrelated UDP flow, nothing to do with the scanner.
    packets.append((8501.0, _udp_packet(9999, 9998, b"hello world unrelated flow")))
    pcap = _write_pcap(tmp_path, "ring.pcap", packets)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["packets_selected"] == len(frags)


def test_snapped_packet_skipped(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(9000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    pcap_bytes = _pcap(packets, snap_last_to=20)
    pcap = tmp_path / "ring.pcap"
    pcap.write_bytes(pcap_bytes)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)

    assert stats["packets_snapped"] == 1
    # The snapped fragment (likely the tail) never lands -> frame incomplete or,
    # if the retirement never triggers within this single file, simply not written.
    assert stats["frames_written"] == 0


def test_multi_file_ordering_and_cross_file_reassembly(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    assert len(frags) >= 2
    split = len(frags) // 2 or 1
    first_half = frags[:split]
    second_half = frags[split:]

    # File B (alphabetically later) has earlier timestamps and the first fragments;
    # file A (alphabetically earlier) has later timestamps and the rest.
    packets_b = [(1000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(first_half)]
    packets_a = [(2000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(second_half)]

    pcap_a = _write_pcap(tmp_path, "a_ring.pcap", packets_a)
    pcap_b = _write_pcap(tmp_path, "b_ring.pcap", packets_b)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap_a), str(pcap_b)], out_path=out)

    assert out.read_bytes() == frame
    assert stats["files_processed"] == 2
    assert stats["out_of_order_files"] == 0


def test_out_of_order_files_flagged_when_alphabetical_order_used(tmp_path):
    # Two independent single-fragment frames, so no cross-file reassembly is needed --
    # this isolates the out_of_order_files bookkeeping.
    frame1 = struct.pack("<IBB", 100, 0, 1) + b"x" * 20
    frame2 = struct.pack("<IBB", 101, 0, 1) + b"y" * 20
    packets_later_name_earlier_ts = [(1000.0, _udp_packet(SCANNER_PORT, HOST_PORT, frame1))]
    packets_earlier_name_later_ts = [(2000.0, _udp_packet(SCANNER_PORT, HOST_PORT, frame2))]

    pcap_a = _write_pcap(tmp_path, "a_ring.pcap", packets_earlier_name_later_ts)
    pcap_z = _write_pcap(tmp_path, "z_ring.pcap", packets_later_name_earlier_ts)

    stats = convert([str(pcap_a), str(pcap_z)])
    # timestamp order should put z_ring (ts=1000) before a_ring (ts=2000).
    assert stats["inputs"][0].endswith("z_ring.pcap")
    assert stats["inputs"][1].endswith("a_ring.pcap")


def test_truncated_trailing_record_does_not_raise(tmp_path):
    frame = _real_frame()
    frags = _fragment(frame)
    packets = [(10000.0 + i * 0.001, _udp_packet(SCANNER_PORT, HOST_PORT, f)) for i, f in enumerate(frags)]
    good = _pcap(packets)
    # Chop off the tail of the file mid-record to simulate tcpdump being killed.
    truncated_bytes = good[:-5]
    pcap = tmp_path / "ring.pcap"
    pcap.write_bytes(truncated_bytes)

    out = tmp_path / "out.bin"
    stats = convert([str(pcap)], out_path=out)  # must not raise

    assert stats["packets_truncated_record"] == 1


def test_pcapng_magic_raises_clear_error(tmp_path):
    p = tmp_path / "ring.pcapng"
    p.write_bytes(struct.pack("<I", MAGIC_PCAPNG) + b"\x00" * 20)

    with pytest.raises(PcapFormatError) as exc_info:
        convert([str(p)])
    assert "pcapng" in str(exc_info.value).lower()


def test_reassembler_matches_udpsource_counter_names():
    r = Reassembler()
    assert hasattr(r, "frames_incomplete")
    assert hasattr(r, "frags_lost")
    assert hasattr(r, "frags_reordered")
    assert hasattr(r, "frags_duplicate")
    assert hasattr(r, "frags_invalid")
