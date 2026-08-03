"""Byte sources and the frame pump. All I/O lives here — decoder/deproject stay pure."""
from __future__ import annotations

import socket
import struct
import sys
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Iterator, Optional

from zeroconf import Zeroconf

from .decoder import StreamDecoder
from .protocol import (
    HEADER_SIZE,
    Frame,
    FrameHeader,
    FrameType,
    ProtocolError,
    parse_tx_queue_stats_event,
)

CDC_VID, CDC_PID = 0xCAFE, 0x4001   # milestone 1b TinyUSB descriptors (docs/protocol.md)


# --- Ethernet TX pacing model (Task 6) ---------------------------------------
#
# Pure host-side mirror of the firmware's paced-fragment TX
# (firmware/scanner-stream/Src/ethernet_transport.c: eth_tx_pump()/
# eth_tx_emit_one()/ETH_SendFrame_Gather()/ETH_TxWindowMsForPeriod()). NOT a
# ctypes byte-level cross-check like host/tests/test_protocol_c_crosscheck.py
# runs for rs_protocol.c -- that file is HAL-free by design, but
# ethernet_transport.c pulls in lwIP/HAL headers this host cannot compile.
# This is a from-spec reimplementation instead, kept textually aligned with
# the C (identical constants, identical formulas, cross-referenced in both
# directions), exercised in host/tests/test_sources.py to pin the CONTRACT
# non-negotiable finding #6 describes: the drain deadline derives from the
# ACTIVE applied frame period (never a fixed 25/33 ms assumption), enough
# fragments drain each pump to clear the backlog inside that deadline, and a
# queued frame is never interleaved with another or abandoned partway.
#
# Task 7 forward-compat: `window_ms` here is a plain "how long do you have"
# input, and `eth_tx_window_ms()` takes a *variadic* set of governing periods
# (today just the one ToF frame_period_us) rather than being hardcoded to one
# stream -- a future decoupled IMU/env tick can pass its own period alongside
# without either function changing shape.

ETH_TX_FRAG_BYTES = 1400   # mirrors ETH_TX_FRAG_BYTES
ETH_TX_SLOT_BYTES = 15104  # mirrors ETH_TX_SLOT_BYTES (>= 32 hdr + 14842 payload + 4 CRC)
ETH_TX_SLOTS = 8           # mirrors ETH_TX_SLOTS


def eth_tx_window_ms(frame_period_us: int, *other_period_us: int) -> int:
    """Drain deadline for the TX pacer, mirroring
    ETH_TxWindowMsForPeriod()/ETH_SetTxWindowMs(): derived from the ACTIVE
    (applied) frame period, never a fixed assumption (non-negotiable finding
    #6 -- the old firmware constant this replaces was ETH_TX_WINDOW_MS=25,
    sized off a 30 Hz/33 ms guess). When more than one period is passed (Task
    7: a decoupled IMU/env tick queuing through the same firmware FIFO
    alongside the ToF period), the deadline is the SHORTEST of them -- the
    queue must clear before whichever cadence reloads it fastest, or that
    stream's own backlog would never drain in time. A plain ToF-only caller
    passes just `frame_period_us`, matching the firmware's own single-period
    call site today."""
    periods_us = [p for p in (frame_period_us, *other_period_us) if p and p > 0]
    if not periods_us:
        return 1
    ms = min(periods_us) // 1000
    return ms if ms > 0 else 1


def eth_tx_budget(pending_fragments: int, elapsed_ms: int, window_ms: int) -> int:
    """How many fragments to emit on THIS pump so `pending_fragments` clears
    within `window_ms`, given `elapsed_ms` since the last pump. Mirrors
    eth_tx_pump()'s adaptive-budget formula exactly: ceil((pending * elapsed)
    / window), floored at 1 whenever there is anything pending at all (the
    firmware's "always make forward progress" comment) and 0 when there is
    nothing to do or no time has passed (a 0 elapsed_ms firmware call is a
    same-millisecond re-entry, not a real budget point)."""
    if pending_fragments <= 0 or elapsed_ms <= 0:
        return 0
    window_ms = window_ms if window_ms > 0 else 1
    budget = -(-(pending_fragments * elapsed_ms) // window_ms)  # ceil division
    return budget if budget > 0 else 1


@dataclass
class _EthTxQueuedFrame:
    seq: int
    total_frags: int
    next_frag: int = 0


class EthTxPacerModel:
    """Pure simulation of the firmware's fixed-slot FIFO + paced drain
    (`eth_tx_slots`/`eth_tx_head`/`eth_tx_tail`/`eth_tx_count` in
    ethernet_transport.c), used only to test the pacing CONTRACT host-side --
    order preservation, no interleaving, no abandonment, forward progress --
    independent of hardware. Not part of the live receive path: `UdpSource`
    only ever RECEIVES fragments the firmware already paced and reassembles
    them (see `UdpSource.read()`); this model exists purely to pin the
    SENDER-side contract in a form host/tests/test_sources.py can exercise."""

    def __init__(self, slots: int = ETH_TX_SLOTS):
        self.slots = slots
        self._queue: list[_EthTxQueuedFrame] = []
        self.emitted_order: list[tuple[int, int]] = []  # (seq, frag_idx) emission order
        self.enqueue_drops = 0  # mirrors eth_tx_enqueue_drops: no slot even after a flush

    def enqueue(self, seq: int, total_bytes: int) -> bool:
        """Mirrors ETH_SendFrame_Gather(): queue one frame's fragments (the
        firmware fragments at ETH_TX_FRAG_BYTES). Returns False (and counts an
        enqueue drop) if every slot is already full -- the model does not
        reproduce eth_tx_flush_blocking()'s synchronous drain-under-pressure,
        since that path's whole point is "the caller cannot proceed until a
        slot frees", not a pacing-contract property; callers that want to
        exercise backlog-clearing call `pump()` between `enqueue()`s instead,
        same as the firmware's own per-iteration cadence."""
        if len(self._queue) >= self.slots:
            self.enqueue_drops += 1
            return False
        total_frags = -(-total_bytes // ETH_TX_FRAG_BYTES)  # ceil division
        self._queue.append(_EthTxQueuedFrame(seq=seq, total_frags=max(1, total_frags)))
        return True

    def pending_fragments(self) -> int:
        """Mirrors eth_tx_pending_frags()."""
        return sum(f.total_frags - f.next_frag for f in self._queue)

    def pump(self, elapsed_ms: int, window_ms: int) -> int:
        """Mirrors eth_tx_pump(): drain `eth_tx_budget()` fragments from the
        HEAD of the queue (oldest frame first), one frame at a time, never
        starting a later frame's fragments before the earlier one is fully
        drained -- this IS the "never interleave, never abandon a partial
        frame" contract, made structural by always emitting from index 0.
        Returns the number of fragments actually emitted this pump."""
        budget = eth_tx_budget(self.pending_fragments(), elapsed_ms, window_ms)
        emitted = 0
        while budget > 0 and self._queue:
            head = self._queue[0]
            self.emitted_order.append((head.seq, head.next_frag))
            head.next_frag += 1
            emitted += 1
            budget -= 1
            if head.next_frag >= head.total_frags:
                self._queue.pop(0)
        return emitted


class FileSource:
    def __init__(self, path, chunk: int = 4096, start: int = 0):
        self._f = open(path, "rb")
        self._chunk = chunk
        if start:
            # Seek to a frame boundary for replay scrubbing (web Phase 3). The
            # caller (SessionController.seek) computes `start` from a capture
            # index so it always lands on a frame boundary; even if it didn't,
            # StreamDecoder resyncs on MAGIC + CRC, so a mid-frame start is
            # merely lossy, never corrupt.
            self._f.seek(start)

    def read(self) -> bytes:
        return self._f.read(self._chunk)

    def write(self, data: bytes) -> None:
        raise NotImplementedError("FileSource is replay-only; there is no device to write to")

    def close(self) -> None:
        self._f.close()


class SerialSource:
    def __init__(self, port: Optional[str] = None, baud: int = 921600, timeout: float = 0.05):
        import serial  # deferred: tests must not need pyserial hardware access
        if port is None:
            port = self.find_port()
        self._ser = serial.Serial(port, baud, timeout=timeout)
        self.port = port

    @staticmethod
    def find_port() -> str:
        """Find the sensor's native USB CDC port (CAFE:4001). No ST-Link VCOM
        fallback: that port only ever carries plain-text firmware `printf`
        debug output (`_stlink_logger_thread` in panel.py already owns it for
        that), never roomscan protocol frames, and treating it as a candidate
        scanner port put it in a losing race against that same logger thread
        for the same COM handle (owner, 2026-07-15)."""
        from serial.tools import list_ports
        ports = list(list_ports.comports())
        for p in ports:
            if p.vid == CDC_VID and p.pid == CDC_PID:
                return p.device
        raise RuntimeError(f"no scanner serial port found among {[p.device for p in ports]}")

    def read(self) -> bytes:
        return self._ser.read(4096)

    def write(self, data: bytes) -> None:
        """Write bytes to the serial port (delegates to pyserial).

        CAUTION: this blocks until the OS accepts the write and, for anything
        beyond a small command frame, potentially until the device drains its
        RX buffer per its pacing policy (see docs/protocol.md and
        host/tests/bench_commands.py). NEVER call this from the thread that is
        draining reads (the loop calling `.read()` / `pump()`): starving that
        loop for >100 ms causes the device to abort an in-flight send by
        design (proven on hardware in Phase 3 Task 2). Call it from a
        different thread than the reader — see CommandClient, which is built
        around exactly this split.
        """
        self._ser.write(data)

    def close(self) -> None:
        self._ser.close()


class UdpSource:
    def __init__(self, port: int = 5000, timeout: float = 0.05, *,
                 zeroconf_factory=Zeroconf, mdns_timeout_ms: float = 1500,
                 keepalive_s: float = 1.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.settimeout(timeout)
        self.sock.bind(("", port))

        self.target_ip = None
        self.target_port = 5000

        # Fragment reassembly (see read()). One frame in flight at a time: the
        # firmware's paced TX drains frames strictly in order, one at a time, so
        # fragments of two different seqs never interleave on the wire.
        self._current_seq = None
        self._total_frags = 0
        self._frags: list[bytes | None] = []
        self._frag_count = 0
        self._max_frag_seen = -1

        # Transport health. `frames_incomplete` is the one that matters: it is
        # the count of frames abandoned with holes, i.e. actual datagram loss.
        # The others separate the causes that used to look identical.
        self.frames_incomplete = 0   # frames dropped with >=1 fragment missing
        self.frags_lost = 0          # fragments missing from those frames
        self.frags_reordered = 0     # arrived out of order but still usable
        self.frags_duplicate = 0
        self.frags_invalid = 0       # bad index / inconsistent total_frags

        # Total link rate (Task 6 step 5): bytes/s of complete reassembled frames
        # (DATA+EVENT+ACK all included -- whatever the firmware actually sends),
        # updated in a rolling ~1 s window every time read() completes a frame.
        # None until the first window closes.
        self.link_bytes_total = 0
        self.link_bytes_per_s: float | None = None
        self._link_rate_window_start = time.time()
        self._link_rate_window_bytes = 0

        # Firmware TX-pacer queue telemetry (Task 6 step 5), populated opportunistically
        # from the periodic TX_QUEUE_STATS EVENT (docs/protocol.md EVENT code 7) --
        # "when available": None until the first one arrives (older firmware, or a
        # capture predating this feature, never sends one). See
        # `_maybe_capture_firmware_stats()`.
        self.fw_tx_queue_high_water: int | None = None
        self.fw_tx_pending_fragments: int | None = None
        self.fw_tx_enqueue_drops: int | None = None
        self.fw_tx_stack_stalls: int | None = None
        self.fw_tx_emitted_bytes: int | None = None
        self.fw_active_transport: str | None = None
        self.fw_stats_updated_at: float | None = None

        # Keepalive: the board unicasts frames to whichever host last sent it a
        # datagram (`target_ip`, set in its udp_recv callback), and only clears
        # that on reboot. get_best_source's wake teaches it our address once at
        # startup -- but if the board reboots, its Ethernet link flaps, or
        # another client (a second viewer, a diagnostic) claims the stream, we
        # go silent forever with no re-wake. Re-send a tiny wake every
        # keepalive_s so the app re-claims the target and self-heals. The board
        # treats any inbound datagram as a wake (payload ignored), so this is
        # harmless when already streaming. keepalive_s <= 0 disables it.
        self.keepalive_s = keepalive_s
        self._last_wake = 0.0
        self._last_rx_time = time.time()

        # Try to resolve roomscanner.local
        self._resolve_target(zeroconf_factory, mdns_timeout_ms)

    def _resolve_target(self, zeroconf_factory=Zeroconf, mdns_timeout_ms: float = 1500):
        """Resolve the board's IP for the initial "wake" datagram (see
        `get_best_source`). `socket.gethostbyname("roomscanner.local")` can't
        do this -- Windows has no native ".local"/mDNS resolution without
        Bonjour installed, so it always raises `gaierror` there (confirmed
        on-box, owner report 2026-07-15) and this class always fell through to
        broadcast. Query mDNS properly instead, via zeroconf's
        `get_service_info` (the same call `tools/query_mdns.py` already
        proves works: `_roomscan._udp.local.` /
        `roomscanner._roomscan._udp.local.`, per ROADMAP Phase 5's lwIP mdns
        advertisement) -- a resolved unicast IP is more reliable than
        broadcast (some networks/firewall profiles drop broadcast). Falls
        back to subnet broadcast, unchanged, if mDNS finds nothing or errors."""
        try:
            zc = zeroconf_factory()
            try:
                info = zc.get_service_info(
                    "_roomscan._udp.local.", "roomscanner._roomscan._udp.local.",
                    timeout=mdns_timeout_ms)
                if info:
                    addrs = info.parsed_addresses()
                    if addrs:
                        self.target_ip = addrs[0]
                        return
            finally:
                zc.close()
        except Exception:
            pass
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.target_ip = "255.255.255.255"

    def _maybe_keepalive(self) -> None:
        """Re-send the wake datagram every keepalive_s so the board keeps
        streaming to us (see __init__). Called on every read() -- cheap, and
        the only place in the steady-state loop that runs regularly."""
        now = time.time()
        
        if self.keepalive_s <= 0:
            return
            
        if now - self._last_wake >= self.keepalive_s:
            self._last_wake = now
            
            # If we haven't received data in 2 seconds, fall back to mDNS resolution
            # rather than just broadcasting. This recovers the stream if the board's IP 
            # changed (e.g. DHCP after replug) on host networks where 255.255.255.255 
            # won't reach it.
            if now - self._last_rx_time > 2.0:
                self._resolve_target(mdns_timeout_ms=500)
                
            if not self.target_ip:
                return
                
            try:
                self.sock.sendto(b"\x00", (self.target_ip, self.target_port))
            except Exception:
                pass

    def _retire_partial_frame(self) -> None:
        """Account for a frame abandoned because a new seq started.

        The sender never interleaves frames, so the arrival of a different seq
        is proof the previous one will never complete. Counting it here is what
        makes datagram loss *visible* -- it is otherwise inferable only from a
        header seq gap downstream.
        """
        if self._frag_count and self._frag_count < self._total_frags:
            self.frames_incomplete += 1
            self.frags_lost += self._total_frags - self._frag_count

    def read(self) -> bytes:
        self._maybe_keepalive()
        try:
            data, addr = self.sock.recvfrom(2048)

            if len(data) < 6:
                # Too short to be a device fragment -- and crucially NOT a
                # reason to re-point `target_ip` at the sender. When mDNS finds
                # nothing we fall back to broadcast, and Linux loops our own
                # 1-byte wake datagram straight back into this socket: adopting
                # that sender address made the source latch onto *the host
                # itself*, so every later wake/keepalive went to 127-of-us and
                # the board was never told where to stream. Self-poisoning on
                # the first read, permanently (owner, 2026-07-28: launch
                # reported "Ethernet/UDP - <our own IP>").
                return b""

            self.target_ip = addr[0]
            self._last_rx_time = time.time()

            seq_num, frag_idx, total_frags = struct.unpack("<IBB", data[:6])
            payload = data[6:]

            # Reassemble into INDEXED SLOTS, not an append-in-order buffer.
            #
            # This used to require `frag_idx == self._expected_frag` and append,
            # so a merely REORDERED datagram -- which UDP explicitly permits and
            # never has to explain -- discarded the whole 14.8 KB frame just as
            # surely as a lost one, and silently. Nothing counted it: the frame
            # simply never reached the decoder, so the only trace was a header
            # seq gap (which BUG-040 had also left unwired).
            #
            # Slots make ordering irrelevant: fragment k always lands at index k
            # and the frame completes when the last hole fills, whatever order
            # they arrive in. Joining slots 0..n-1 reconstructs the frame exactly
            # because the sender chunks at a fixed 1400 B (only the tail is
            # short), so index alone determines position.
            if seq_num != self._current_seq:
                self._retire_partial_frame()
                self._current_seq = seq_num
                self._total_frags = total_frags
                self._frags = [None] * total_frags
                self._frag_count = 0
                self._max_frag_seen = -1

            # A fragment whose total_frags disagrees with the rest of its seq is
            # corrupt framing, not a reorder -- drop it rather than resizing and
            # silently truncating a frame.
            if total_frags != self._total_frags or not (0 <= frag_idx < total_frags):
                self.frags_invalid += 1
                return b""
            if self._frags[frag_idx] is not None:
                self.frags_duplicate += 1   # retransmit or a network echo
                return b""

            # Purely diagnostic: counts fragments arriving before one already
            # missing, i.e. genuine reordering. Distinguishes "the network
            # shuffled it" from "the network lost it" -- previously identical.
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
                self._update_link_rate(len(res))
                self._maybe_capture_firmware_stats(res)
                return res
            return b""
        except socket.timeout:
            return b""
        except BlockingIOError:
            return b""
        except OSError:
            # Interface might be temporarily down (e.g. cable unplugged).
            # Ignore and retry next poll rather than crashing the reader thread.
            return b""

    def _update_link_rate(self, nbytes: int) -> None:
        """Total link rate (Task 6 step 5): rolling ~1 s window over every
        complete reassembled frame's wire size (header + payload + CRC),
        regardless of frame_type -- this is link bandwidth, not a per-stream
        rate (`metrics.py` already reports those). Called on every completed
        `read()`, so it costs nothing extra beyond one subtraction/compare."""
        self.link_bytes_total += nbytes
        self._link_rate_window_bytes += nbytes
        elapsed = time.time() - self._link_rate_window_start
        if elapsed >= 1.0:
            self.link_bytes_per_s = self._link_rate_window_bytes / elapsed
            self._link_rate_window_bytes = 0
            self._link_rate_window_start = time.time()

    def _maybe_capture_firmware_stats(self, frame_bytes: bytes) -> None:
        """Peek at a fully-reassembled frame for the firmware's periodic
        TX_QUEUE_STATS EVENT (docs/protocol.md EVENT code 7) and cache its
        counters as plain attributes -- entirely a side read. `frame_bytes` is
        not consumed or altered: `pump()`'s caller still decodes it normally
        through `StreamDecoder`, exactly like every other frame (this EVENT
        also gets an ordinary, harmless log line there via `parse_event()`'s
        ascii/"replace" decode of its binary tail -- ugly, not wrong).

        Deliberately tolerant: this is a diagnostic side-channel, not a
        protocol boundary `read()` should ever raise or block on. Any
        length/CRC/shape mismatch (a DATA frame, a legacy ACK, a foreign
        packet that happens to reassemble to something magic-shaped) is
        silently not-a-match, same as the wire decoder's own unknown-frame
        tolerance."""
        if len(frame_bytes) < HEADER_SIZE + 4:
            return
        try:
            hdr = FrameHeader.unpack(frame_bytes[:HEADER_SIZE])
        except ProtocolError:
            return
        if hdr.frame_type != FrameType.EVENT:
            return
        total = HEADER_SIZE + hdr.payload_len + 4
        if len(frame_bytes) != total:
            return
        (crc,) = struct.unpack_from("<I", frame_bytes, total - 4)
        if zlib.crc32(frame_bytes[: total - 4]) != crc:
            return
        payload = frame_bytes[HEADER_SIZE : HEADER_SIZE + hdr.payload_len]
        try:
            stats = parse_tx_queue_stats_event(payload)
        except ProtocolError:
            return
        self.fw_tx_queue_high_water = stats.queue_high_water
        self.fw_tx_pending_fragments = stats.pending_fragments
        self.fw_tx_enqueue_drops = stats.enqueue_drops
        self.fw_tx_stack_stalls = stats.stack_stalls
        self.fw_tx_emitted_bytes = stats.emitted_bytes
        self.fw_active_transport = stats.active_transport
        self.fw_stats_updated_at = time.time()

    def write(self, data: bytes) -> None:
        if self.target_ip:
            try:
                self.sock.sendto(data, (self.target_ip, self.target_port))
            except Exception:
                pass

    def close(self) -> None:
        self.sock.close()


def get_best_source(port: Optional[str] = None, baud: int = 921600, timeout: float = 0.05,
                     probe_s: float = 5.0, resend_s: float = 0.5):
    """Prefer Ethernet (Phase 5's production transport): probe UDP for
    `probe_s`, falling back to the serial CDC/scanner port only if nothing
    arrives. The board doesn't know the host's address up front, so a "wake"
    datagram teaches it where to reply -- but UDP has no delivery guarantee,
    and this used to send that packet exactly once. Setting the *socket's*
    timeout to the full probe window before the loop meant the first
    `udp.read()` call itself blocked for the whole window (returning early
    only on data) -- so the "while" below never actually got a second
    iteration to resend on. One dropped wake packet silently killed Ethernet
    preference for the whole launch (owner, 2026-07-15: "we had comms over
    ethernet working prior to this... it's supposed to prefer ethernet").
    Fix: short per-read timeout, real polling loop, periodic resend."""
    udp = UdpSource(timeout=timeout)
    old_timeout = udp.sock.gettimeout()
    udp.sock.settimeout(0.2)
    t0 = time.time()
    next_wake = 0.0
    while time.time() - t0 < probe_s:
        now = time.time()
        if now >= next_wake:
            udp.write(b'\x00')
            next_wake = now + resend_s
        data = udp.read()
        if data:
            udp.sock.settimeout(old_timeout)
            return udp

    # No data received, fallback to Serial.
    udp.sock.settimeout(old_timeout)
    try:
        serial_source = SerialSource(port, baud, timeout)
    except Exception as exc:
        # A *missing* CDC port is not a launch blocker. Since Phase 5 the
        # production transport is Ethernet, and a headless rig can legitimately
        # have no scanner serial port at all -- ST-Link unplugged, USB_USER
        # carrying power only. Crashing the launch there is nonsense: it
        # reports a serial problem for an Ethernet deployment, and it lost a
        # launch just because the board happened to be booting (DHCP) during
        # the probe window (owner, 2026-07-28: "this is preventing me from
        # launching the application (which does not depend on stlink, just
        # ethernet)"). Hand back the UDP source instead -- its keepalive keeps
        # re-waking the board, so the stream starts by itself the moment the
        # device shows up. A *busy* port still raises: that means a real
        # scanner port exists and something else holds it, which the caller
        # (panel._open_source) offers to resolve interactively.
        from . import portguard
        if portguard.classify_open_error(exc) == "busy":
            udp.close()
            raise
        print(f"[source] no scanner serial port ({exc}); "
              f"listening for the Ethernet stream on UDP :{udp.target_port} instead",
              file=sys.stderr, flush=True)
        return udp
    udp.close()
    return serial_source


class Recorder:
    """Thread-safe mid-stream recording handle for the GUI's Record button.

    The reader thread calls `write()` on every raw chunk unconditionally; it
    is a no-op while not recording. The UI thread calls `start()`/`stop()` to
    toggle recording at any point. All state transitions are guarded by a
    single lock so a `stop()` racing a `write()` can never write to (or
    close) a half-closed file.

    Design choice: `start()` while already recording does NOT raise — it
    closes the current file and switches to the new path. This is the
    friendlier behavior for a UI Record button (e.g. double-click, or
    starting a new take without an explicit Stop first) than forcing the
    caller to stop() before every start().
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._f = None
        self._path = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._f is not None

    @property
    def path(self):
        with self._lock:
            return self._path

    def start(self, path) -> None:
        with self._lock:
            if self._f is not None:
                self._f.close()
            self._f = open(path, "wb")
            self._path = path

    def stop(self) -> None:
        with self._lock:
            if self._f is not None:
                self._f.close()
                self._f = None
                self._path = None

    def write(self, data: bytes) -> None:
        with self._lock:
            if self._f is not None:
                self._f.write(data)
                self._f.flush()   # keep on-disk bytes current while still "active" (readable mid-recording)

    def close(self) -> None:
        """Final teardown; safe to call multiple times (alias for stop())."""
        self.stop()


def pump(source, decoder: StreamDecoder, record_path=None, recorder: Optional[Recorder] = None) -> Iterator[Frame]:
    """Read raw chunks from `source`, tee them to recording sink(s), decode, yield frames.

    `record_path`, if given, opens a file at pump start and writes every raw
    chunk to it for the whole run (legacy all-or-nothing recording); pump
    owns that file's lifecycle and closes it in `finally`, exactly as before.

    `recorder`, if given, is a `Recorder` the caller starts/stops from
    another thread (e.g. a GUI Record button) to capture only part of the
    stream. Every raw chunk is teed to `recorder.write()`, which is a no-op
    while the recorder is inactive. Pump does NOT own `recorder`'s lifecycle:
    it never calls start/stop/close on it, so the recorder is left exactly
    as the caller last set it (active or not) when pump exits — the caller
    may keep using it across multiple pump() calls.

    Both may be passed at once (record_path captures everything, recorder
    captures a caller-controlled sub-range); normal panel usage passes only
    `recorder`.
    """
    rec = None
    try:
        if record_path:
            rec = open(record_path, "wb")
        while True:
            data = source.read()
            if not data:
                # A source signals "replay reached EOF, stop the pump" either by
                # being a FileSource or by exposing a truthy `eof_on_empty`
                # attribute (web Phase 3 wraps a FileSource behind a prefix
                # source for scrub-seek, and needs the same EOF semantics).
                if getattr(source, "eof_on_empty", False) or isinstance(source, FileSource):
                    return          # EOF on replay; live sources just idle
                continue
            if rec:
                rec.write(data)
            if recorder is not None:
                recorder.write(data)
            yield from decoder.feed(data)
    finally:
        if rec:
            rec.close()
        source.close()
