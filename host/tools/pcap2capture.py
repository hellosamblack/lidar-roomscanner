"""Convert a Pi-bridge tee pcap (or ring of pcaps) into a capture .bin, byte-identical
to what host/tools/capture.py writes -- so every existing capture_* tool consumes it
unmodified.

    host/.venv/bin/python host/tools/pcap2capture.py ring.pcap -o captures/recovered.bin
    host/.venv/bin/python host/tools/pcap2capture.py ring.pcap ring.pcap0 ring.pcap1 --json

The Pi bridge runs `tcpdump -i eth0 'udp port 5000' -w ring.pcap -C 100 -W 20` between the
STM32 scanner and the PC host, teeing every scanner packet to a bounded ring of classic
pcap files. When the Wi-Fi hop loses frames, the pcap is the recovery source: this tool
parses the pcap itself (stdlib only, no scapy/dpkt), reassembles the UDP fragments exactly
as `roomscan.sources.UdpSource.read()` does, and concatenates the resulting frames into a
headerless .bin -- exactly what `roomscan.decoder.StreamDecoder` eats.
"""
from __future__ import annotations

import argparse
import json
import struct
import sys
from pathlib import Path
from typing import Iterator

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

# Classic pcap global-header magics (both endiannesses, us and ns resolution).
_MAGIC_US_LE = 0xA1B2C3D4
_MAGIC_US_BE = 0xD4C3B2A1
_MAGIC_NS_LE = 0xA1B23C4D
_MAGIC_NS_BE = 0x4D3CB2A1
_MAGIC_PCAPNG = 0x0A0D0D0A

_LINKTYPE_ETHERNET = 1
_LINKTYPE_RAW = 101
_LINKTYPE_LINUX_SLL = 113

_SCANNER_PORT = 5000


class PcapFormatError(Exception):
    pass


def _detect_format(magic_bytes: bytes) -> tuple[str, bool, bool]:
    """Return (kind, byteorder_is_le, is_ns) for a 4-byte magic, or raise PcapFormatError."""
    magic_le = struct.unpack("<I", magic_bytes)[0]
    if magic_le == _MAGIC_PCAPNG:
        raise PcapFormatError(
            "pcapng format is not supported (only classic pcap) -- "
            "re-capture with tcpdump's classic -w, or convert with `editcap -F pcap`")
    if magic_le == _MAGIC_US_LE:
        return "classic", True, False
    if magic_le == _MAGIC_US_BE:
        return "classic", False, False
    if magic_le == _MAGIC_NS_LE:
        return "classic", True, True
    if magic_le == _MAGIC_NS_BE:
        return "classic", False, True
    raise PcapFormatError(f"unrecognized pcap magic: {magic_bytes!r}")


def iter_pcap_packets(path) -> Iterator[tuple[float, bytes]]:
    """Yield (timestamp_s, link_layer_frame_bytes) for every complete, unsnapped record
    in a classic pcap file, in file order. Stops (without raising) on a truncated
    trailing record -- callers that need to count that should use
    `_iter_pcap_packets_counted`."""
    for ts, frame, truncated, _linktype, snapped in _iter_pcap_packets_counted(path):
        if not truncated and not snapped:
            yield ts, frame


def _iter_pcap_packets_counted(path):
    """Internal: yields (ts, frame_bytes, truncated_flag, linktype, snapped_flag)
    records. The LAST yielded item, if truncated_flag is True, carries no usable
    frame_bytes (b""). A snapped record (incl_len < orig_len) carries its truncated
    (incomplete) payload in frame_bytes -- callers must check snapped_flag before using
    it, since that payload is NOT the real packet."""
    with open(path, "rb") as f:
        head = f.read(24)
        if len(head) < 24:
            raise PcapFormatError(f"{path}: too short to be a pcap file")
        _kind, le, ns = _detect_format(head[:4])
        endian = "<" if le else ">"
        (_magic, _vmaj, _vmin, _tz, _sigfigs, _snaplen, linktype) = struct.unpack(
            endian + "IHHiIII", head)
        if linktype not in (_LINKTYPE_ETHERNET, _LINKTYPE_RAW, _LINKTYPE_LINUX_SLL):
            raise PcapFormatError(
                f"{path}: unsupported linktype {linktype} "
                f"(only Ethernet=1, raw IP=101, Linux SLL=113 are supported)")

        while True:
            rec_hdr = f.read(16)
            if len(rec_hdr) == 0:
                return
            if len(rec_hdr) < 16:
                yield (0.0, b"", True, linktype, False)
                return
            ts_sec, ts_frac, incl_len, orig_len = struct.unpack(endian + "IIII", rec_hdr)
            data = f.read(incl_len)
            if len(data) < incl_len:
                yield (0.0, b"", True, linktype, False)
                return
            ts = ts_sec + (ts_frac / 1e9 if ns else ts_frac / 1e6)
            snapped = incl_len < orig_len
            yield (ts, data, False, linktype, snapped)


def _strip_link_header(frame: bytes, linktype: int) -> bytes | None:
    """Return the IPv4 payload (starting at the IP header) for a link-layer frame, or
    None if it isn't IPv4 / is too short to tell."""
    if linktype == _LINKTYPE_ETHERNET:
        if len(frame) < 14:
            return None
        ethertype = struct.unpack(">H", frame[12:14])[0]
        if ethertype == 0x8100:  # 802.1Q VLAN tag
            if len(frame) < 18:
                return None
            ethertype = struct.unpack(">H", frame[16:18])[0]
            frame = frame[4:]
        if ethertype != 0x0800:
            return None
        return frame[14:]
    if linktype == _LINKTYPE_RAW:
        return frame
    if linktype == _LINKTYPE_LINUX_SLL:
        if len(frame) < 16:
            return None
        protocol = struct.unpack(">H", frame[14:16])[0]
        if protocol != 0x0800:
            return None
        return frame[16:]
    return None


def udp_payload_from_ip(ip_pkt: bytes):
    """Parse an IPv4 packet and return (src_port, dst_port, payload) for its UDP
    datagram, or None if it isn't a well-formed, unfragmented IPv4/UDP packet.
    Returns the string "fragment" instead of a tuple if it's a fragmented IPv4 packet
    (callers use this to count fragments separately)."""
    if len(ip_pkt) < 20:
        return None
    vihl = ip_pkt[0]
    version = vihl >> 4
    if version != 4:
        return None
    ihl = (vihl & 0x0F) * 4
    if ihl < 20 or len(ip_pkt) < ihl:
        return None
    protocol = ip_pkt[9]
    flags_frag = struct.unpack(">H", ip_pkt[6:8])[0]
    mf = (flags_frag >> 13) & 0x1
    frag_offset = flags_frag & 0x1FFF
    if mf or frag_offset:
        return "fragment"
    if protocol != 17:  # UDP
        return None
    udp = ip_pkt[ihl:]
    if len(udp) < 8:
        return None
    src_port, dst_port, udp_len, _checksum = struct.unpack(">HHHH", udp[:8])
    payload = udp[8:udp_len] if udp_len >= 8 else udp[8:]
    return src_port, dst_port, payload


class Reassembler:
    """Mirrors `roomscan.sources.UdpSource.read()`'s fragment-reassembly logic exactly,
    including its counter semantics, so pcap-recovered captures are directly comparable
    to live-capture loss stats."""

    def __init__(self):
        self._current_seq = None
        self._total_frags = 0
        self._frags: list[bytes | None] = []
        self._frag_count = 0
        self._max_frag_seen = -1

        self.frames_incomplete = 0
        self.frags_lost = 0
        self.frags_reordered = 0
        self.frags_duplicate = 0
        self.frags_invalid = 0

    def _retire_partial_frame(self) -> None:
        if self._frag_count and self._frag_count < self._total_frags:
            self.frames_incomplete += 1
            self.frags_lost += self._total_frags - self._frag_count

    def feed(self, data: bytes) -> bytes:
        """Feed one selected UDP payload (>= 6 bytes). Returns the reassembled frame
        bytes when a frame completes, else b""."""
        seq_num, frag_idx, total_frags = struct.unpack("<IBB", data[:6])
        payload = data[6:]

        if seq_num != self._current_seq:
            self._retire_partial_frame()
            self._current_seq = seq_num
            self._total_frags = total_frags
            self._frags = [None] * total_frags
            self._frag_count = 0
            self._max_frag_seen = -1

        if total_frags != self._total_frags or not (0 <= frag_idx < total_frags):
            self.frags_invalid += 1
            return b""
        if self._frags[frag_idx] is not None:
            self.frags_duplicate += 1
            return b""

        if frag_idx < self._max_frag_seen:
            self.frags_reordered += 1
        self._max_frag_seen = max(self._max_frag_seen, frag_idx)

        self._frags[frag_idx] = payload
        self._frag_count += 1
        if self._frag_count == self._total_frags:
            res = b"".join(self._frags)
            self._frags = []
            self._current_seq = None
            self._total_frags = 0
            self._frag_count = 0
            return res
        return b""

    def finish(self) -> None:
        """Retire any still-partial frame at end of input, so a trailing incomplete
        frame is counted."""
        self._retire_partial_frame()
        self._frags = []
        self._current_seq = None
        self._total_frags = 0
        self._frag_count = 0
        self._max_frag_seen = -1


def _first_last_ts(path) -> tuple[float | None, float | None]:
    first = last = None
    for ts, _frame, truncated, _linktype, _snapped in _iter_pcap_packets_counted(path):
        if truncated:
            break
        if first is None:
            first = ts
        last = ts
    return first, last


def convert(inputs, out_path=None) -> dict:
    """Convert one or more pcap files (a tee ring) into a reassembled capture .bin.

    `inputs` may be file paths (str/Path) or a single directory (all files in it are
    used). Files are processed in first-packet-timestamp order, with reassembly state
    carried across file boundaries. Pure except for the (optional) write to `out_path`.
    Returns a stats dict; writes the concatenated frame bytes to `out_path` if given.
    """
    if isinstance(inputs, (str, Path)):
        inputs = [inputs]
    paths = [Path(p) for p in inputs]
    if len(paths) == 1 and paths[0].is_dir():
        paths = sorted(p for p in paths[0].iterdir() if p.is_file())

    for p in paths:
        if not p.is_file():
            raise FileNotFoundError(str(p))

    # Sort by first packet's timestamp, not filename (tcpdump's ring reuses names).
    file_first_ts = []
    for p in paths:
        first, last = _first_last_ts(p)
        file_first_ts.append((p, first, last))
    file_first_ts.sort(key=lambda t: (t[1] is None, t[1]))

    out_of_order_files = 0
    prev_last = None
    for _p, first, last in file_first_ts:
        if prev_last is not None and first is not None and first < prev_last:
            out_of_order_files += 1
        if last is not None:
            prev_last = last

    reasm = Reassembler()
    frames_out: list[bytes] = []
    packets_total = 0
    packets_selected = 0
    packets_snapped = 0
    packets_ip_fragmented = 0
    packets_truncated_record = 0
    first_ts = None
    last_ts = None

    for p, _first, _last in file_first_ts:
        for ts, frame, truncated, linktype, snapped in _iter_pcap_packets_counted(p):
            if truncated:
                packets_truncated_record += 1
                break
            packets_total += 1
            if first_ts is None:
                first_ts = ts
            last_ts = ts

            if snapped:
                packets_snapped += 1
                continue

            ip_pkt = _strip_link_header(frame, linktype)
            if ip_pkt is None:
                continue
            parsed = udp_payload_from_ip(ip_pkt)
            if parsed is None:
                continue
            if parsed == "fragment":
                packets_ip_fragmented += 1
                continue
            src_port, dst_port, payload = parsed
            if src_port != _SCANNER_PORT:
                continue
            if len(payload) < 6:
                continue
            packets_selected += 1
            res = reasm.feed(payload)
            if res:
                frames_out.append(res)
    reasm.finish()

    data = b"".join(frames_out)
    if out_path is not None:
        out_path = Path(out_path)
        out_path.write_bytes(data)

    frames_written = len(frames_out)
    frames_incomplete = reasm.frames_incomplete
    denom = frames_written + frames_incomplete
    frame_loss_pct = (frames_incomplete / denom * 100.0) if denom else 0.0

    return {
        "files_processed": len(file_first_ts),
        "packets_total": packets_total,
        "packets_selected": packets_selected,
        "packets_snapped": packets_snapped,
        "packets_ip_fragmented": packets_ip_fragmented,
        "packets_truncated_record": packets_truncated_record,
        "frames_written": frames_written,
        "bytes_written": len(data),
        "frames_incomplete": frames_incomplete,
        "frags_lost": reasm.frags_lost,
        "frags_reordered": reasm.frags_reordered,
        "frags_duplicate": reasm.frags_duplicate,
        "frags_invalid": reasm.frags_invalid,
        "first_ts": first_ts,
        "last_ts": last_ts,
        "duration_s": (last_ts - first_ts) if (first_ts is not None and last_ts is not None) else None,
        "out_of_order_files": out_of_order_files,
        "frame_loss_pct": frame_loss_pct,
        "inputs": [str(p) for p, _f, _l in file_first_ts],
        "out": str(out_path) if out_path is not None else None,
    }


def _print(stats: dict) -> None:
    print(f"files processed:     {stats['files_processed']}")
    print(f"packets total:       {stats['packets_total']}")
    print(f"packets selected:    {stats['packets_selected']}")
    print(f"packets snapped:     {stats['packets_snapped']}")
    print(f"packets ip-frag'd:   {stats['packets_ip_fragmented']}")
    print(f"truncated records:   {stats['packets_truncated_record']}")
    print(f"frames written:      {stats['frames_written']}  ({stats['bytes_written']} bytes)")
    print(f"frames incomplete:   {stats['frames_incomplete']}  (loss {stats['frame_loss_pct']:.2f}%)")
    print(f"frags lost:          {stats['frags_lost']}")
    print(f"frags reordered:     {stats['frags_reordered']}")
    print(f"frags duplicate:     {stats['frags_duplicate']}")
    print(f"frags invalid:       {stats['frags_invalid']}")
    if stats["duration_s"] is not None:
        print(f"duration:            {stats['duration_s']:.2f} s")
    if stats["out_of_order_files"]:
        print(f"out-of-order files:  {stats['out_of_order_files']}")
    print(f"out:                 {stats['out'] or '(not written)'}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("inputs", nargs="+", help="pcap file(s) or a directory of them")
    ap.add_argument("-o", "--out", help="output .bin path")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    for raw in args.inputs:
        p = Path(raw)
        if not p.exists():
            print(f"no such file or directory: {p}", file=sys.stderr)
            return 2

    try:
        stats = convert(args.inputs, out_path=args.out)
    except (PcapFormatError, FileNotFoundError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(stats, indent=2))
    else:
        _print(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
