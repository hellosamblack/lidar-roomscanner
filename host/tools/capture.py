"""Consolidated [HW] capture ritual: reset -> connect -> timed raw capture -> report.

Every Phase 1-3 [HW] task rebuilt this same ritual from prose: find the native CDC
port by VID/PID, SWD-reset the board and wait for the stale port to vanish and a
fresh one to reappear before reopening, retry a hung boot, tee raw bytes to a file
for `--seconds`, then read the counters back out. This is the one script (seeded
in the milestone-retro backlog 2026-07-08, executed at the Phase 3 retro 2026-07-09).

    host/.venv/Scripts/python host/tools/capture.py --reset --seconds 15 --out captures/foo.bin

Firmware now SELF-HEALS boot hangs on its own (Phase 3 Task 5: a bounded 5-attempt
retry with 100/200/400/800/1600 ms backoff runs inside vl53l9_app()'s own bring-up,
10/10 boot soak in the delivered binary -- .superpowers/sdd/progress.md, P3 Task 5).
The --boot-timeout/--max-boot-retries logic here is belt-and-braces for the rare
case a physical reset genuinely wedges the board before any host-visible frame,
not a workaround for a live firmware defect.

Post-capture report separates the documented "connect transient" (a frame-1 send
aborted by the firmware's 100 ms host-starvation policy during host-startup
scheduling -- root-caused and characterized cosmetic in Phase 3 Task 6,
docs/connect-transient-forensics.md) from genuine mid-stream anomalies, prints fps
under BOTH conventions used across the ledger (the P2.5-era "24.6 vs 27.76" style
confusion, settled here by always labeling which is which), and checks the CALIB
64-frame retransmit cadence (docs/protocol.md stream registry).
"""
from __future__ import annotations

import argparse
import os
import struct
import subprocess
import sys
import time
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from roomscan.decoder import StreamDecoder  # noqa: E402
from roomscan.sources import CDC_PID, CDC_VID, SerialSource  # noqa: E402

DEFAULT_PROGRAMMER = (
    r"C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu."
    r"externaltools.cubeprogrammer.win32_2.2.500.202603051304\tools\bin\STM32_Programmer_CLI.exe"
)

# Decode constants mirror host/tools/analyze_capture.py deliberately: this report
# reads the just-written capture file with the same magic-scan/CRC/resync policy
# as roomscan.decoder.StreamDecoder, but position-aware (needed to tell "anomaly
# before the first good frame" -- the connect transient -- from "anomaly later").
MAGIC = b"RSCN"
HEADER_SIZE = 32
_HEADER = struct.Struct("<4sBBBBIQHHII")  # magic, ver, type, stream, flags, seq, t_us, w, h, plen, reserved
assert _HEADER.size == HEADER_SIZE
MAX_PAYLOAD = 1 << 20

FRAME_TYPES = {1: "DATA", 2: "EVENT", 3: "COMMAND", 4: "ACK"}
STREAMS = {0: "DEPTH_ZF32", 1: "DEPTH_ZAPC", 2: "AMBIENT", 3: "AMPLITUDE", 4: "CONFIDENCE",
           5: "REFLECTANCE", 6: "STATUS", 7: "RAW_3DMD", 8: "CALIB"}
EVENT_CODES = {1: "SENSOR_INIT_FAIL", 2: "TRIGGER_TIMEOUT", 3: "DMA_TIMEOUT",
               4: "SENSOR_ERROR_STATUS", 5: "TX_OVERFLOW"}
CALIB_CADENCE = 64


def programmer_path() -> str:
    return os.environ.get("ROOMSCAN_PROGRAMMER", DEFAULT_PROGRAMMER)


def reset_board(programmer: str) -> None:
    """One SWD reset, no reflash: ``STM32_Programmer_CLI -c port=SWD -rst`` -- the
    exact invocation used throughout Phase 3's [HW] verification passes (e.g.
    .superpowers/sdd/p3-task-5-report.md)."""
    print(f"resetting board via SWD ({programmer}) ...")
    subprocess.run([programmer, "-c", "port=SWD", "-rst"],
                   check=True, capture_output=True, text=True, timeout=30)


def _cdc_ports() -> set[str]:
    from serial.tools import list_ports
    return {p.device for p in list_ports.comports() if p.vid == CDC_VID and p.pid == CDC_PID}


def wait_for_port_cycle(prior_port: str | None, timeout: float = 10.0) -> str:
    """After a reset, the native CDC port vanishes (the MCU's power-on reset drops
    and re-enumerates USB) and reappears a moment later. Wait for the vanish (if we
    knew a port a moment ago) then the reappearance, rather than racing a reopen
    against Windows enumeration -- the stale-port PermissionError this avoids is
    the single most common friction point across every [HW] task's reports.
    """
    t0 = time.monotonic()
    if prior_port is not None:
        while time.monotonic() - t0 < timeout and prior_port in _cdc_ports():
            time.sleep(0.1)
    t1 = time.monotonic()
    while time.monotonic() - t1 < timeout:
        ports = _cdc_ports()
        if ports:
            return sorted(ports)[0]
        time.sleep(0.1)
    raise RuntimeError(f"CDC port (VID {CDC_VID:04x}:PID {CDC_PID:04x}) did not "
                        f"reappear within {timeout}s of reset")


def acquire(initial_port: str | None, seconds: float, out_path: Path, boot_timeout: float,
            max_retries: int, programmer: str, do_reset: bool) -> tuple[str, float]:
    """Reset (optional) + boot-hang retry + timed raw capture to `out_path`.

    Returns (port actually used, measured wall-clock capture duration in seconds).
    """
    current_port = initial_port
    attempt = 0
    while True:
        attempt += 1
        if attempt == 1 and do_reset:
            reset_board(programmer)
            current_port = wait_for_port_cycle(current_port, timeout=10.0)
        elif attempt > 1:
            print(f"boot hang: no decoded frames within {boot_timeout}s; "
                  f"reset+retry {attempt - 1}/{max_retries} ...")
            reset_board(programmer)
            current_port = wait_for_port_cycle(current_port, timeout=10.0)
        if current_port is None:
            current_port = SerialSource.find_port()

        print(f"opening {current_port} (attempt {attempt}) ...")
        source = SerialSource(current_port)
        decoder = StreamDecoder()
        first_frame_at: float | None = None
        boot_hang = False
        t_start = time.monotonic()
        try:
            with open(out_path, "wb") as out_f:
                while True:
                    elapsed = time.monotonic() - t_start
                    if elapsed >= seconds:
                        break
                    data = source.read()
                    if data:
                        out_f.write(data)
                        if decoder.feed(data) and first_frame_at is None:
                            first_frame_at = elapsed
                    if first_frame_at is None and elapsed > boot_timeout:
                        boot_hang = True
                        break
        finally:
            source.close()

        if not boot_hang:
            return current_port, time.monotonic() - t_start
        if attempt - 1 >= max_retries:
            raise RuntimeError(
                f"boot hang persisted after {max_retries} reset retries "
                f"(no decoded frames within {boot_timeout}s on each attempt)")


def acquire_udp(seconds: float, out_path: Path, boot_timeout: float) -> tuple[str, float]:
    """Timed raw capture over Ethernet/UDP (the headless host has no USB — the
    board streams via UDP; see the headless-host-deployment memory). No SWD
    reset / port cycling: `get_best_source()` opens the UDP source (whose
    `read()` self-heals with a keepalive wake), we dump every datagram, and
    stop after `seconds`. Returns (source label, measured wall-clock seconds).

    Raises RuntimeError on a boot hang (no decoded frames within boot_timeout) —
    for UDP that means the board isn't reachable/streaming (check
    `roomscanner.local`), not a boot flake, so there is no reset-retry here."""
    from roomscan.sources import get_best_source, UdpSource
    source = get_best_source()
    if isinstance(source, UdpSource):
        label = f"Ethernet/UDP {source.target_ip}:{source.target_port}"
    else:
        label = type(source).__name__
    print(f"opening {label} ...")
    decoder = StreamDecoder()
    first_frame_at: float | None = None
    t_start = time.monotonic()
    try:
        with open(out_path, "wb") as out_f:
            while True:
                elapsed = time.monotonic() - t_start
                if elapsed >= seconds:
                    break
                data = source.read()
                if data:
                    out_f.write(data)
                    if decoder.feed(data) and first_frame_at is None:
                        first_frame_at = elapsed
                if first_frame_at is None and elapsed > boot_timeout:
                    raise RuntimeError(
                        f"no decoded frames within {boot_timeout}s over UDP — is the board "
                        f"reachable and streaming? (check roomscanner.local / headless_doctor.py)")
    finally:
        source.close()
    return label, time.monotonic() - t_start


def decode_file(path: Path):
    """Position-aware decode pass over a recorded capture. Returns:

    ``(frames, crc_failures, bytes_skipped, first_good_offset,
      transient_skipped, transient_crc)``

    ``frames`` is a list of ``(offset, frame_type_name, stream_name, seq, flags,
    payload_len, t_us)`` in file order. ``transient_*`` counts anomalies (skipped
    bytes / CRC failures) that occurred STRICTLY BEFORE the first successfully
    decoded frame -- the documented connect-time transient. Anything after the
    first good frame is a genuine mid-stream anomaly, counted separately --
    EXCEPT the final trailing partial frame at end-of-file, which every timed
    capture produces (the cutoff lands mid-frame) and is reported on its own as
    ``eof_tail_bytes``, not folded into the "genuine anomaly" bucket.
    """
    data = path.read_bytes()
    n = len(data)
    pos = 0
    frames: list[tuple] = []
    crc_failures = 0
    bytes_skipped = 0
    first_good_offset: int | None = None
    transient_skipped = 0
    transient_crc = 0
    eof_tail_bytes = 0

    def note_skip(count: int) -> None:
        nonlocal bytes_skipped, transient_skipped
        bytes_skipped += count
        if first_good_offset is None:
            transient_skipped += count

    while pos < n:
        idx = data.find(MAGIC, pos)
        if idx < 0:
            if n - pos:
                note_skip(n - pos)
                eof_tail_bytes += n - pos
            break
        if idx > pos:
            note_skip(idx - pos)
            pos = idx
        if n - pos < HEADER_SIZE:
            note_skip(n - pos)
            eof_tail_bytes += n - pos
            break
        magic, ver, ftype, stream, flags, seq, t_us, w, h, plen, _res = _HEADER.unpack(
            data[pos:pos + HEADER_SIZE])
        if ver != 1 or plen > MAX_PAYLOAD:
            note_skip(1)
            pos += 1
            continue
        total = HEADER_SIZE + plen + 4
        if n - pos < total:
            note_skip(n - pos)
            eof_tail_bytes += n - pos
            break
        body = data[pos:pos + total]
        (crc,) = struct.unpack_from("<I", body, total - 4)
        if zlib.crc32(body[:-4]) != crc:
            crc_failures += 1
            if first_good_offset is None:
                transient_crc += 1
            note_skip(1)
            pos += 1
            continue
        if first_good_offset is None:
            first_good_offset = pos
        frames.append((pos, FRAME_TYPES.get(ftype, ftype), STREAMS.get(stream, stream),
                       seq, flags, plen, t_us))
        pos += total
    return (frames, crc_failures, bytes_skipped, first_good_offset, transient_skipped,
            transient_crc, eof_tail_bytes)


def report(path: Path, wall_elapsed: float, requested_seconds: float) -> int:
    (frames, crc_failures, bytes_skipped, first_good_offset, t_skipped, t_crc,
     eof_tail_bytes) = decode_file(path)
    n = path.stat().st_size
    print(f"\n=== decode report: {path} ===")
    print(f"file size: {n} bytes, wall-clock capture time: {wall_elapsed:.2f}s "
          f"(requested {requested_seconds}s)")

    if first_good_offset is None:
        print("no frames decoded at all -- capture is empty or entirely garbage")
        return 1

    transient_present = t_skipped > 0 or t_crc > 0
    print(f"connect transient: {'present' if transient_present else 'absent'}"
          + (f" ({t_skipped} B skipped, {t_crc} CRC failures before the first good frame; "
             "see docs/connect-transient-forensics.md)" if transient_present else ""))

    post_skipped = bytes_skipped - t_skipped
    post_crc = crc_failures - t_crc
    post_skipped_real = post_skipped - eof_tail_bytes  # exclude the expected trailing partial frame
    print(f"crc_failures: {crc_failures} total ({t_crc} in connect transient, "
          f"{post_crc} post-transient)")
    print(f"bytes_skipped: {bytes_skipped} total ({t_skipped} in connect transient, "
          f"{post_skipped} post-transient, of which {eof_tail_bytes} B is the expected trailing "
          "partial frame at the capture's end-of-file cutoff)")
    if post_skipped_real or post_crc:
        print("  ** WARNING: anomalies observed AFTER the first good frame and NOT explained by "
              "end-of-capture truncation -- NOT the known connect transient; investigate with "
              "host/tools/analyze_capture.py **")

    counts: dict[tuple[str, str], int] = {}
    for _off, ft, sid, *_rest in frames:
        counts[(ft, str(sid))] = counts.get((ft, str(sid)), 0) + 1
    print("\nframes by stream:")
    for (ft, sid), c in sorted(counts.items()):
        print(f"  {ft:8} {sid:14} {c}")

    data_streams: dict[str, list[tuple[int, int]]] = {}
    events: list[tuple[int, int, int]] = []
    calib_seqs: list[int] = []
    for off, ft, sid, seq, _flags, plen, t_us in frames:
        if ft == "DATA":
            data_streams.setdefault(str(sid), []).append((seq, t_us))
            if sid == "CALIB":
                calib_seqs.append(seq)
        elif ft == "EVENT":
            events.append((off, seq, plen))

    if not data_streams:
        print("\nno DATA frames decoded -- fps/gap analysis skipped")
    else:
        dom_sid, dom_frames = max(data_streams.items(), key=lambda kv: len(kv[1]))
        seqs = [s for s, _ in dom_frames]
        t_us_list = [t for _, t in dom_frames]
        print(f"\nfps/gap analysis on dominant DATA stream: {dom_sid} ({len(dom_frames)} frames)")
        if len(dom_frames) >= 2 and t_us_list[-1] > t_us_list[0]:
            span_s = (t_us_list[-1] - t_us_list[0]) / 1e6
            fps_interval = (len(dom_frames) - 1) / span_s
            print(f"  fps (interval convention, (N-1)/t_us-span over {span_s:.2f}s): "
                  f"{fps_interval:.2f}")
        else:
            print("  fps (interval convention): unavailable (< 2 frames or non-increasing t_us)")
        fps_wall = len(dom_frames) / wall_elapsed if wall_elapsed > 0 else 0.0
        print(f"  fps (wall-clock convention, frames / {wall_elapsed:.2f}s measured capture "
              f"time): {fps_wall:.2f}")

        gaps = [(a, b) for a, b in zip(seqs, seqs[1:]) if b != a + 1]
        print(f"  seq gaps (post connect-transient, consecutive-decoded-frame basis): {len(gaps)}")
        for a, b in gaps[:20]:
            print(f"    {a} -> {b}")
        if len(gaps) > 20:
            print(f"    ... and {len(gaps) - 20} more")

    print(f"\nCALIB frames: {len(calib_seqs)}")
    if len(calib_seqs) >= 2:
        deltas = [b - a for a, b in zip(calib_seqs, calib_seqs[1:])]
        print(f"  seq deltas between consecutive CALIB frames: {deltas}")
        off_cadence = [d for d in deltas if not (CALIB_CADENCE // 2 <= d <= CALIB_CADENCE * 2)]
        if off_cadence:
            print(f"  ** unexpected cadence (expected ~{CALIB_CADENCE}): {off_cadence} **")
        else:
            print(f"  cadence consistent with the documented ~{CALIB_CADENCE}-frame periodic "
                  "retransmit (docs/protocol.md)")

    print(f"\nEVENT frames: {len(events)}")
    if events:
        raw = path.read_bytes()
        for off, seq, plen in events:
            payload = raw[off + HEADER_SIZE: off + HEADER_SIZE + plen]
            code, detail = struct.unpack_from("<II", payload, 0)
            msg = payload[8:].decode("ascii", "replace")
            name = EVENT_CODES.get(code, f"UNKNOWN({code})")
            print(f"  seq={seq} code={code} ({name}) detail={detail} msg={msg!r}")

    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        prog="capture",
        description="Consolidated [HW] capture ritual: optional SWD reset, port discovery, "
                     "boot-hang retry, timed raw capture, decode-and-report. Serial CDC by "
                     "default; --udp captures over Ethernet (the headless host has no USB).")
    ap.add_argument("--udp", action="store_true",
                     help="capture over Ethernet/UDP via get_best_source() instead of serial CDC "
                          "(no SWD reset / port cycling); use on the headless host, which has no USB")
    ap.add_argument("--port", help="serial port override (default: auto-detect CDC CAFE:4001)")
    ap.add_argument("--reset", action="store_true",
                     help="SWD-reset the board before capturing (STM32_Programmer_CLI)")
    ap.add_argument("--programmer", default=None,
                     help="STM32_Programmer_CLI.exe path (default: $ROOMSCAN_PROGRAMMER env var, "
                          "else the STM32CubeIDE 2.2.0 bundled path)")
    ap.add_argument("--seconds", type=float, default=20.0, help="capture duration (default 20s)")
    ap.add_argument("--out", default="captures/capture.bin", help="raw output file")
    ap.add_argument("--boot-timeout", type=float, default=15.0,
                     help="seconds with zero decoded frames before treating the boot as hung "
                          "(default 15)")
    ap.add_argument("--max-boot-retries", type=int, default=3,
                     help="reset+retry this many times on a boot hang before giving up (default "
                          "3; belt-and-braces -- firmware self-heals boot hangs on its own, 10/10 "
                          "soak per .superpowers/sdd/progress.md Phase 3 Task 5)")
    args = ap.parse_args(argv)

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    if args.udp:
        if args.reset:
            print("note: --reset is ignored with --udp (no SWD over Ethernet)", file=sys.stderr)
        try:
            label, wall_elapsed = acquire_udp(args.seconds, out_path, args.boot_timeout)
        except (RuntimeError, OSError) as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        print(f"captured {out_path.stat().st_size} bytes over {wall_elapsed:.2f}s from "
              f"{label} -> {out_path}")
        return report(out_path, wall_elapsed, args.seconds)

    programmer = args.programmer or programmer_path()

    port = args.port
    if port is None:
        try:
            port = SerialSource.find_port()
        except RuntimeError:
            port = None  # not enumerated yet; acquire() waits for it post-reset
    if port is None and not args.reset:
        print("error: no scanner serial port found and --reset not requested", file=sys.stderr)
        return 1

    try:
        final_port, wall_elapsed = acquire(port, args.seconds, out_path, args.boot_timeout,
                                           args.max_boot_retries, programmer, args.reset)
    except (RuntimeError, subprocess.CalledProcessError, subprocess.TimeoutExpired,
            FileNotFoundError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    print(f"captured {out_path.stat().st_size} bytes over {wall_elapsed:.2f}s from "
          f"{final_port} -> {out_path}")
    return report(out_path, wall_elapsed, args.seconds)


if __name__ == "__main__":
    sys.exit(main())
