#!/usr/bin/env python3
"""Characterization instrument for the Pi-bridge C270 (issue #226, spec §5.1).

The spec forbids writing `roomscan-cam` before this has produced numbers, so
this tool is the gate: it runs the three arms of §5.1 -- camera-off baseline,
preview-on, preview+snapshot load -- each over ONE shared wall-clock window
with the scanner streaming live, and records what actually happened.

Two roles, one file:

    # on the PC host (the far side; records arrival times and sizes)
    python3 cam_characterize.py receiver --bind 0.0.0.0 --port 5099 \
        --seconds 90 --out host_arm.json

    # on the Pi (Raspbian trixie armv7l, system python3, no pip)
    python3 cam_characterize.py arm --name preview --seconds 60 \
        --host 172.17.2.x --port 5099 --mode 1280x720@30 --out pi_arm.json

WHY THE FAR SIDE MATTERS.  Camera cadence counted on the Pi is a producer
counter, and a producer counter reads healthy while the consumer is seconds
behind (#217's `browser_fps`).  So every frame is timestamped at capture (V4L2
MONOTONIC), at send, and again at arrival on the host, and the Pi<->host clock
offset is measured with an NTP-style four-timestamp exchange rather than
assumed from NTP.  `host_arrival_age_ms` is therefore a differenced quantity
with a stated uncertainty (the half-RTT spread), not a subtraction of two
unrelated clocks.

WHAT "PREVIEW-ON" MEANS BEFORE THE DAEMON EXISTS.  A real MJPEG capture loop
pushing real JPEG bytes over wlan0 to a real socket on the host, with the
latest-wins fanout the daemon will use -- not a synthetic load.  The snapshot
arm adds a second consumer pulling the latest frame at 2 Hz on its own
connection.
"""
from __future__ import annotations

import argparse
import json
import os
import pathlib
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import v4l2_capture as v4l2  # noqa: E402

MAGIC = b"RSCAMCHAR1"
FRAME_HDR = "<IIQQQI"          # seq, generation, capture_mono, send_mono, send_real, nbytes
FRAME_HDR_SIZE = struct.calcsize(FRAME_HDR)
PING_FMT = "<QQ"
PONG_FMT = "<QQ"


def _pct(values: list, q: float):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(q * len(ordered)))]


def _stats(values: list, scale: float = 1.0, digits: int = 1) -> dict | None:
    if not values:
        return None
    return {
        "n": len(values),
        "median": round(_pct(values, 0.5) * scale, digits),
        "p95": round(_pct(values, 0.95) * scale, digits),
        "p99": round(_pct(values, 0.99) * scale, digits),
        "max": round(max(values) * scale, digits),
        "min": round(min(values) * scale, digits),
    }


# ===========================================================================
# Pi-side system sampling
# ===========================================================================

def _read(path: str) -> str | None:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except OSError:
        return None


def _run(cmd: list[str], timeout: float = 8.0) -> str | None:
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout if out.returncode == 0 else (out.stdout or None)


def _net_counters(iface: str) -> dict:
    base = f"/sys/class/net/{iface}/statistics/"
    out = {}
    for key in ("rx_bytes", "tx_bytes", "rx_packets", "tx_packets",
                "rx_dropped", "tx_dropped", "rx_errors", "tx_errors"):
        val = _read(base + key)
        out[key] = int(val) if val and val.isdigit() else None
    return out


def _cpu_counters() -> dict:
    """System jiffies from /proc/stat and this process's own utime+stime."""
    out = {"system": None, "idle": None, "process": None, "clk_tck": os.sysconf("SC_CLK_TCK")}
    stat = _read("/proc/stat")
    if stat:
        first = stat.splitlines()[0].split()
        if first and first[0] == "cpu":
            fields = [int(x) for x in first[1:11]]
            out["system"] = sum(fields)
            out["idle"] = fields[3] + fields[4]
    self_stat = _read("/proc/self/stat")
    if self_stat:
        parts = self_stat.rsplit(") ", 1)[-1].split()
        # after the comm field: state is [0], utime is [11], stime is [12]
        try:
            out["process"] = int(parts[11]) + int(parts[12])
        except (IndexError, ValueError):
            pass
    return out


def _nft_counters() -> dict | None:
    """Per-rule packet/byte counters from the bridge's own nft table.

    This is the scanner-path observable available from the Pi itself: the
    forward chain's `roomscan-fwd-established` counter carries the scanner's
    stream, so its delta over an arm is scanner throughput measured on the
    same box and the same window as the camera load.
    """
    raw = _run(["sudo", "-n", "/usr/sbin/nft", "-j", "list", "table", "ip", "roomscan_bridge"])
    if not raw:
        return None
    try:
        doc = json.loads(raw)
    except json.JSONDecodeError:
        return None
    out = {}
    for item in doc.get("nftables", []):
        rule = item.get("rule")
        if not rule:
            continue
        comment = rule.get("comment")
        if not comment:
            continue
        for expr in rule.get("expr", []):
            counter = expr.get("counter")
            if isinstance(counter, dict):
                out[comment] = {"packets": counter.get("packets"),
                                "bytes": counter.get("bytes")}
    return out or None


def _wlan_link() -> dict:
    raw = _run(["/usr/sbin/iw", "dev", "wlan0", "link"]) or ""
    out = {"raw": raw.strip() or None, "rssi_dbm": None,
           "tx_bitrate_mbps": None, "rx_bitrate_mbps": None, "ssid": None}
    for line in raw.splitlines():
        line = line.strip()
        if line.startswith("signal:"):
            try:
                out["rssi_dbm"] = int(line.split()[1])
            except (IndexError, ValueError):
                pass
        elif line.startswith("tx bitrate:"):
            try:
                out["tx_bitrate_mbps"] = float(line.split()[2])
            except (IndexError, ValueError):
                pass
        elif line.startswith("rx bitrate:"):
            try:
                out["rx_bitrate_mbps"] = float(line.split()[2])
            except (IndexError, ValueError):
                pass
        elif line.startswith("SSID:"):
            out["ssid"] = line.split(":", 1)[1].strip()
    return out


def sample_system() -> dict:
    temp = _read("/sys/class/thermal/thermal_zone0/temp")
    throttled = _run(["/usr/bin/vcgencmd", "get_throttled"]) or ""
    loadavg = (_read("/proc/loadavg") or "").split()
    return {
        "mono_ns": time.monotonic_ns(),
        "real_ns": time.time_ns(),
        "wlan0": _net_counters("wlan0"),
        "eth0": _net_counters("eth0"),
        "cpu": _cpu_counters(),
        "nft": _nft_counters(),
        "wlan_link": _wlan_link(),
        "temp_c": round(int(temp) / 1000.0, 1) if temp and temp.isdigit() else None,
        "throttled": throttled.strip().split("=")[-1] or None,
        "loadavg": loadavg[:3] or None,
        "mem_available_kb": _meminfo("MemAvailable"),
    }


def _meminfo(key: str) -> int | None:
    raw = _read("/proc/meminfo") or ""
    for line in raw.splitlines():
        if line.startswith(key + ":"):
            parts = line.split()
            if len(parts) >= 2 and parts[1].isdigit():
                return int(parts[1])
    return None


def diff_samples(first: dict, last: dict) -> dict:
    """Rates over the window, never absolute counters (which are since-boot)."""
    span = (last["mono_ns"] - first["mono_ns"]) / 1e9
    if span <= 0:
        return {"span_s": span}
    out = {"span_s": round(span, 3)}
    for iface in ("wlan0", "eth0"):
        pair = {}
        for key, value in last[iface].items():
            before = first[iface].get(key)
            if value is None or before is None:
                pair[key + "_per_s"] = None
            else:
                pair[key + "_per_s"] = round((value - before) / span, 1)
        out[iface] = pair
    clk = last["cpu"]["clk_tck"] or 100
    if last["cpu"]["system"] is not None and first["cpu"]["system"] is not None:
        total = last["cpu"]["system"] - first["cpu"]["system"]
        idle = (last["cpu"]["idle"] or 0) - (first["cpu"]["idle"] or 0)
        out["cpu_system_busy_pct"] = round(100.0 * (total - idle) / total, 1) if total else None
    if last["cpu"]["process"] is not None and first["cpu"]["process"] is not None:
        used = (last["cpu"]["process"] - first["cpu"]["process"]) / clk
        out["cpu_process_pct_of_one_core"] = round(100.0 * used / span, 1)
    if first["nft"] and last["nft"]:
        nft = {}
        for name, counters in last["nft"].items():
            before = first["nft"].get(name)
            if not before:
                continue
            nft[name] = {
                "packets_per_s": round((counters["packets"] - before["packets"]) / span, 1),
                "bytes_per_s": round((counters["bytes"] - before["bytes"]) / span, 1),
            }
        out["nft"] = nft
    return out


# ===========================================================================
# Scanner-path observable: a short pcap of the scanner's UDP stream on eth0
# ===========================================================================

def scanner_pcap_gaps(seconds: float, port: int = 5000) -> dict:
    """Packet cadence + the MAX inter-packet gap of the live scanner stream.

    Rate alone cannot see a stall that a bus-contention event would cause:
    a 200 ms hole followed by a burst averages out.  The max gap can.
    """
    # A FIXED path here silently reported the PREVIOUS run's capture when a
    # later tcpdump failed to replace it: the arm printed `seconds: 15` beside
    # a 5-second run's packet count, byte for byte.  The filename carries the
    # pid + start instant so a stale artifact cannot be mistaken for a fresh
    # one, tcpdump's own exit status and stderr are reported rather than
    # swallowed, and the file is removed with the same privilege that made it
    # (it is owned by the unprivileged `tcpdump` user, so a plain unlink from
    # sticky /tmp fails).
    path = f"/tmp/roomscan-cam-scanner-{os.getpid()}-{time.monotonic_ns()}.pcap"
    out = {"available": False, "seconds": seconds, "port": port, "path": path}
    started_ns = time.time_ns()
    try:
        proc = subprocess.run(["sudo", "-n", "/usr/bin/timeout", "-s", "INT",
                               str(int(seconds)), "/usr/bin/tcpdump", "-i", "eth0",
                               "-n", "-s", "64", "-w", path, f"udp port {port}"],
                              capture_output=True, text=True, timeout=seconds + 30)
        out["tcpdump_returncode"] = proc.returncode
        out["tcpdump_stderr"] = (proc.stderr or "").strip()[-400:] or None
    except (OSError, subprocess.TimeoutExpired) as exc:
        out["error"] = f"tcpdump failed to run: {exc}"
        return out
    if not os.path.exists(path):
        out["error"] = "tcpdump produced no capture file"
        return out
    try:
        blob = pathlib.Path(path).read_bytes()
        out["file_bytes"] = len(blob)
        out["file_is_fresh"] = os.stat(path).st_mtime_ns >= started_ns
    except OSError as exc:
        out["error"] = f"pcap unreadable: {exc}"
        return out
    finally:
        subprocess.run(["sudo", "-n", "/bin/rm", "-f", path], capture_output=True)
    if not out.get("file_is_fresh"):
        out["error"] = "capture file predates this run; refusing to report stale packets"
        return out
    if len(blob) < 24:
        out["error"] = "pcap empty"
        return out
    magic = blob[:4]
    if magic in (b"\xd4\xc3\xb2\xa1", b"\x4d\x3c\xb2\xa1"):
        endian = "<"
    elif magic in (b"\xa1\xb2\xc3\xd4", b"\xa1\xb2\x3c\x4d"):
        endian = ">"
    else:
        out["error"] = f"unrecognised pcap magic {magic.hex()}"
        return out
    nano = magic in (b"\x4d\x3c\xb2\xa1", b"\xa1\xb2\x3c\x4d")
    times, total = [], 0
    off = 24
    while off + 16 <= len(blob):
        ts_sec, ts_frac, incl, orig = struct.unpack_from(endian + "IIII", blob, off)
        off += 16 + incl
        times.append(ts_sec + ts_frac / (1e9 if nano else 1e6))
        total += orig
    if len(times) < 2:
        out["error"] = f"only {len(times)} packets captured"
        return out
    span = times[-1] - times[0]
    gaps = [(b - a) * 1000.0 for a, b in zip(times, times[1:])]
    out.update({
        "available": True,
        "packets": len(times),
        "span_s": round(span, 3),
        "packets_per_s": round((len(times) - 1) / span, 1) if span > 0 else None,
        "bytes_per_s_on_wire": round(total / span, 1) if span > 0 else None,
        "interpacket_gap_ms": _stats(gaps, 1.0, 3),
    })
    return out


# ===========================================================================
# Pi side: capture loop + latest-slot fanout + senders
# ===========================================================================

class LatestSlot:
    """One immutable frame, condvar fanout, no per-client queue.

    This is the shape §5.2 mandates for the daemon, exercised here so the
    characterization measures the real behaviour (a stalled consumer skips
    frames) rather than a queue that would flatter the numbers.
    """

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._closed = False

    def publish(self, frame) -> None:
        with self._cond:
            self._frame = frame
            self._cond.notify_all()

    def close(self) -> None:
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def wait_newer(self, last_seq: int, timeout: float = 1.0):
        with self._cond:
            if self._frame is None or self._frame["seq"] <= last_seq:
                self._cond.wait(timeout)
            if self._closed and (self._frame is None or self._frame["seq"] <= last_seq):
                return None
            frame = self._frame
        if frame is None or frame["seq"] <= last_seq:
            return None
        return frame


def _connect(host: str, port: int, role: str, meta: dict) -> socket.socket:
    sock = socket.create_connection((host, port), timeout=15.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    header = json.dumps({"role": role, **meta}).encode() + b"\n"
    sock.sendall(MAGIC + header)
    return sock


def _sender_thread(slot: LatestSlot, host: str, port: int, role: str, meta: dict,
                   stop: threading.Event, result: dict, min_period: float = 0.0):
    try:
        sock = _connect(host, port, role, meta)
    except OSError as exc:
        result["error"] = f"connect: {exc}"
        return
    last_seq = 0
    sent = skipped = 0
    next_at = 0.0
    try:
        while not stop.is_set():
            frame = slot.wait_newer(last_seq, timeout=0.5)
            if frame is None:
                continue
            if min_period:
                now = time.monotonic()
                if now < next_at:
                    # Consume the frame even though we are not sending it.
                    # Leaving last_seq behind made wait_newer() return this
                    # same frame immediately, so the rate-limited sender spun
                    # a core flat out and starved the capture thread through
                    # the GIL -- the first run of this arm reported 98% of a
                    # core and 147 dropped camera frames that were entirely
                    # the instrument's doing.
                    skipped += frame["seq"] - last_seq
                    last_seq = frame["seq"]
                    continue
                next_at = now + min_period
            skipped += max(0, frame["seq"] - last_seq - 1)
            last_seq = frame["seq"]
            payload = frame["data"]
            head = struct.pack(FRAME_HDR, frame["seq"], frame["generation"],
                               frame["capture_mono_ns"], time.monotonic_ns(),
                               time.time_ns(), len(payload))
            sock.sendall(b"F" + head + payload)
            sent += 1
    except OSError as exc:
        result["error"] = f"send: {exc}"
    finally:
        result["sent"] = sent
        result["skipped"] = skipped
        try:
            sock.shutdown(socket.SHUT_WR)
        except OSError:
            pass
        sock.close()


def _clock_thread(host: str, port: int, stop: threading.Event, result: dict,
                  period: float = 2.0):
    """NTP-style four-timestamp exchange, Pi realtime <-> host realtime.

    Without this, "host arrival age" would be a subtraction of two clocks
    that nobody measured against each other -- a number with no uncertainty
    attached, which is the failure mode §4.5 names.
    """
    samples = []
    try:
        sock = _connect(host, port, "clock", {})
    except OSError as exc:
        result["error"] = f"connect: {exc}"
        return
    try:
        while not stop.is_set():
            t1 = time.time_ns()
            t1_mono = time.monotonic_ns()
            sock.sendall(b"P" + struct.pack(PING_FMT, t1, t1_mono))
            reply = _recv_exactly(sock, 1 + struct.calcsize(PONG_FMT))
            t4 = time.time_ns()
            if not reply or reply[0:1] != b"Q":
                break
            t2, t3 = struct.unpack(PONG_FMT, reply[1:])
            samples.append({
                "t1": t1, "t2": t2, "t3": t3, "t4": t4, "t1_mono": t1_mono,
                "offset_ns": ((t2 - t1) + (t3 - t4)) // 2,
                "rtt_ns": (t4 - t1) - (t3 - t2),
            })
            stop.wait(period)
    except OSError as exc:
        result["error"] = f"exchange: {exc}"
    finally:
        result["samples"] = samples
        sock.close()


def _recv_exactly(sock: socket.socket, n: int) -> bytes | None:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def parse_mode(text: str) -> tuple[int, int, int]:
    geom, _, fps = text.partition("@")
    width, _, height = geom.partition("x")
    return int(width), int(height), int(fps or 30)


def run_arm(args) -> dict:
    width, height, fps = parse_mode(args.mode)
    report = {
        "arm": args.name, "mode_requested": args.mode, "seconds": args.seconds,
        "host": f"{args.host}:{args.port}",
        "started_wall": time.time(), "started_mono_ns": time.monotonic_ns(),
        "camera": None, "device_selection": None, "controls": None,
        "capture": None, "preview": None, "snapshot": None, "clock": None,
        "system_before": None, "system_after": None, "system_series": [],
        "scanner_pcap": None, "errors": [],
    }
    camera_on = args.name != "baseline"

    selection = v4l2.select_device(override=args.device)
    report["device_selection"] = {"selected": selection["selected"],
                                  "reason": selection["reason"],
                                  "candidates": selection["candidates"]}

    device = None
    if camera_on:
        if not selection["selected"]:
            report["errors"].append("no camera selected: " + str(selection["reason"]))
            return report
        if args.exposure_dynamic_framerate is not None:
            _run(["/usr/bin/v4l2-ctl", "-d", selection["selected"], "-c",
                  f"exposure_dynamic_framerate={args.exposure_dynamic_framerate}"])
        device = v4l2.V4L2Device(selection["selected"])
        device.open()
        report["camera"] = {"querycap": device.query_cap()}
        started = device.start(width, height, v4l2.V4L2_PIX_FMT_MJPEG, fps)
        report["camera"]["negotiated"] = started
        report["controls"] = _run(["/usr/bin/v4l2-ctl", "-d", selection["selected"],
                                   "--list-ctrls"])

    slot = LatestSlot()
    stop = threading.Event()
    threads, results = [], {}
    if camera_on:
        results["preview"] = {}
        meta = {"arm": args.name, "mode": args.mode,
                "negotiated": report["camera"]["negotiated"]}
        threads.append(threading.Thread(
            target=_sender_thread,
            args=(slot, args.host, args.port, "preview", meta, stop, results["preview"]),
            daemon=True))
        if args.snapshot_hz:
            results["snapshot"] = {}
            threads.append(threading.Thread(
                target=_sender_thread,
                args=(slot, args.host, args.port, "snapshot", meta, stop,
                      results["snapshot"], 1.0 / args.snapshot_hz),
                daemon=True))
    results["clock"] = {}
    threads.append(threading.Thread(
        target=_clock_thread, args=(args.host, args.port, stop, results["clock"]),
        daemon=True))
    for thread in threads:
        thread.start()

    report["system_before"] = sample_system()
    deadline = time.monotonic() + args.seconds
    next_sample = time.monotonic() + args.sample_period
    pcap_result: dict = {}
    pcap_thread = None
    pcap_at = time.monotonic() + args.seconds / 3.0
    captures, sizes, invalid, buffer_errors = [], [], 0, 0
    ts_domains = {}
    seq_first = seq_last = None
    generation = 1
    try:
        while time.monotonic() < deadline:
            if camera_on:
                try:
                    frame = device.dequeue(timeout_s=2.0)
                except TimeoutError:
                    report["errors"].append("dequeue timeout")
                    continue
                if frame["buffer_error"]:
                    buffer_errors += 1
                if not v4l2.looks_like_jpeg(frame["data"]):
                    invalid += 1
                    continue
                captures.append(frame["capture_mono_ns"])
                sizes.append(frame["bytesused"])
                ts_domains[frame["timestamp_domain"]] = ts_domains.get(frame["timestamp_domain"], 0) + 1
                if seq_first is None:
                    seq_first = frame["sequence"]
                seq_last = frame["sequence"]
                slot.publish({"seq": len(captures), "generation": generation,
                              "capture_mono_ns": frame["capture_mono_ns"],
                              "data": frame["data"]})
            else:
                time.sleep(0.05)
            now = time.monotonic()
            if now >= next_sample:
                report["system_series"].append(sample_system())
                next_sample = now + args.sample_period
            if pcap_thread is None and args.pcap_seconds and now >= pcap_at:
                # A third of the way in, so the pcap window sits inside the
                # load rather than at an edge -- and on its own thread, so it
                # cannot punch a hole in the dequeue cadence it is measuring.
                pcap_thread = threading.Thread(
                    target=lambda: pcap_result.update(
                        scanner_pcap_gaps(args.pcap_seconds)), daemon=True)
                pcap_thread.start()
    finally:
        stop.set()
        slot.close()
        if pcap_thread is not None:
            pcap_thread.join(timeout=args.pcap_seconds + 25)
        report["scanner_pcap"] = pcap_result or None
        report["system_after"] = sample_system()
        for thread in threads:
            thread.join(timeout=10.0)
        if device is not None:
            try:
                device.stream_off()
            finally:
                device.close()

    if camera_on and len(captures) >= 2:
        span = (captures[-1] - captures[0]) / 1e9
        # Discard the first second: a UVC stream's first frames arrive late
        # while auto-exposure settles, and folding that into p95 would report
        # a startup transient as a steady-state cadence defect.
        settle_ns = captures[0] + 1_000_000_000
        steady = [t for t in captures if t >= settle_ns]
        gaps = [(b - a) / 1e6 for a, b in zip(captures, captures[1:])]
        steady_gaps = [(b - a) / 1e6 for a, b in zip(steady, steady[1:])]
        report["capture"] = {
            "frames": len(captures),
            "span_s": round(span, 3),
            "fps_achieved": round((len(captures) - 1) / span, 3) if span > 0 else None,
            "fps_steady": (round((len(steady) - 1) / ((steady[-1] - steady[0]) / 1e9), 3)
                           if len(steady) > 2 else None),
            "interframe_gap_ms": _stats(gaps, 1.0, 2),
            "interframe_gap_ms_steady": _stats(steady_gaps, 1.0, 2),
            "jpeg_bytes": _stats(sizes, 1.0, 0),
            "jpeg_bytes_per_s": round(sum(sizes) / span, 1) if span > 0 else None,
            "invalid_jpeg": invalid,
            "buffer_errors": buffer_errors,
            "timestamp_domains": ts_domains,
            "driver_sequence_first": seq_first,
            "driver_sequence_last": seq_last,
            "driver_sequence_gaps": (seq_last - seq_first + 1 - len(captures) - invalid
                                     if seq_first is not None else None),
        }
    report["preview"] = results.get("preview")
    report["snapshot"] = results.get("snapshot")
    report["clock"] = _summarize_clock(results.get("clock", {}))
    report["deltas"] = diff_samples(report["system_before"], report["system_after"])
    return report


def _summarize_clock(result: dict) -> dict:
    samples = result.get("samples") or []
    if not samples:
        return {"samples": 0, "error": result.get("error")}
    best = sorted(samples, key=lambda s: s["rtt_ns"])[:max(1, len(samples) // 4)]
    offsets = [s["offset_ns"] for s in best]
    rtts = [s["rtt_ns"] for s in samples]
    return {
        "samples": len(samples),
        "error": result.get("error"),
        # Offset from the lowest-RTT quartile: high-RTT samples carry an
        # unknown asymmetry, so including them widens the estimate with noise
        # rather than information.
        "pi_to_host_offset_ns": int(sum(offsets) / len(offsets)),
        "offset_spread_ns": max(offsets) - min(offsets),
        "rtt_ms": _stats(rtts, 1e-6, 3),
        "uncertainty_ms": round(min(rtts) / 2e6, 3),
        "raw": samples,
    }


# ===========================================================================
# Host side: receiver
# ===========================================================================

def run_receiver(args) -> dict:
    report = {"bind": f"{args.bind}:{args.port}", "seconds": args.seconds,
              "connections": [], "started_wall": time.time()}
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind((args.bind, args.port))
    listener.listen(8)
    listener.settimeout(1.0)
    deadline = time.monotonic() + args.seconds
    threads = []
    try:
        while time.monotonic() < deadline:
            try:
                conn, addr = listener.accept()
            except socket.timeout:
                continue
            record = {"peer": f"{addr[0]}:{addr[1]}"}
            report["connections"].append(record)
            thread = threading.Thread(target=_serve_connection,
                                      args=(conn, record, deadline), daemon=True)
            thread.start()
            threads.append(thread)
    finally:
        listener.close()
        for thread in threads:
            thread.join(timeout=15.0)
    return report


def _serve_connection(conn: socket.socket, record: dict, deadline: float) -> None:
    conn.settimeout(5.0)
    buf = b""
    try:
        while b"\n" not in buf:
            chunk = conn.recv(4096)
            if not chunk:
                record["error"] = "closed before header"
                return
            buf += chunk
        line, _, buf = buf.partition(b"\n")
        if not line.startswith(MAGIC):
            record["error"] = f"bad magic {line[:16]!r}"
            return
        header = json.loads(line[len(MAGIC):].decode())
        record["header"] = header
        role = header.get("role")
        record["role"] = role
        if role == "clock":
            _serve_clock(conn, buf, record, deadline)
        else:
            _serve_frames(conn, buf, record, deadline)
    except (OSError, ValueError) as exc:
        record["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        conn.close()


def _serve_clock(conn: socket.socket, buf: bytes, record: dict, deadline: float) -> None:
    exchanges = 0
    need = 1 + struct.calcsize(PING_FMT)
    while time.monotonic() < deadline + 5:
        while len(buf) < need:
            try:
                chunk = conn.recv(4096)
            except socket.timeout:
                chunk = b""
            if not chunk:
                record["exchanges"] = exchanges
                return
            buf += chunk
        if buf[0:1] != b"P":
            record["error"] = "bad clock record"
            return
        t2 = time.time_ns()
        buf = buf[need:]
        t3 = time.time_ns()
        conn.sendall(b"Q" + struct.pack(PONG_FMT, t2, t3))
        exchanges += 1
    record["exchanges"] = exchanges


def _serve_frames(conn: socket.socket, buf: bytes, record: dict, deadline: float) -> None:
    frames = []
    total_bytes = 0
    while time.monotonic() < deadline + 5:
        while len(buf) < 1 + FRAME_HDR_SIZE:
            try:
                chunk = conn.recv(65536)
            except socket.timeout:
                chunk = b""
            if not chunk:
                _finish_frames(record, frames, total_bytes)
                return
            buf += chunk
        if buf[0:1] != b"F":
            record["error"] = f"bad frame magic {buf[0:1]!r}"
            _finish_frames(record, frames, total_bytes)
            return
        seq, gen, capture_ns, send_mono, send_real, nbytes = struct.unpack_from(
            FRAME_HDR, buf, 1)
        want = 1 + FRAME_HDR_SIZE + nbytes
        while len(buf) < want:
            try:
                chunk = conn.recv(min(262144, want - len(buf)))
            except socket.timeout:
                chunk = b""
            if not chunk:
                record["truncated_frame"] = True
                _finish_frames(record, frames, total_bytes)
                return
            buf += chunk
        arrival_real = time.time_ns()
        payload = buf[1 + FRAME_HDR_SIZE:want]
        buf = buf[want:]
        total_bytes += nbytes
        frames.append({
            "seq": seq, "generation": gen, "bytes": nbytes,
            "capture_mono_ns": capture_ns, "send_mono_ns": send_mono,
            "send_real_ns": send_real, "arrival_real_ns": arrival_real,
            "jpeg_ok": payload[0:2] == b"\xff\xd8" and b"\xff\xd9" in payload[-16:],
        })
    _finish_frames(record, frames, total_bytes)


def _finish_frames(record: dict, frames: list, total_bytes: int) -> None:
    record["frames"] = len(frames)
    record["bytes"] = total_bytes
    record["bad_jpeg"] = sum(1 for f in frames if not f["jpeg_ok"])
    if len(frames) >= 2:
        span = (frames[-1]["arrival_real_ns"] - frames[0]["arrival_real_ns"]) / 1e9
        arrivals = [(b["arrival_real_ns"] - a["arrival_real_ns"]) / 1e6
                    for a, b in zip(frames, frames[1:])]
        record["arrival_fps"] = round((len(frames) - 1) / span, 3) if span > 0 else None
        record["arrival_bytes_per_s"] = round(total_bytes / span, 1) if span > 0 else None
        record["arrival_gap_ms"] = _stats(arrivals, 1.0, 2)
        record["seq_first"] = frames[0]["seq"]
        record["seq_last"] = frames[-1]["seq"]
        record["seq_skipped"] = frames[-1]["seq"] - frames[0]["seq"] + 1 - len(frames)
        record["pipeline_ns_samples"] = [
            {"seq": f["seq"], "capture_mono_ns": f["capture_mono_ns"],
             "send_mono_ns": f["send_mono_ns"], "send_real_ns": f["send_real_ns"],
             "arrival_real_ns": f["arrival_real_ns"], "bytes": f["bytes"]}
            for f in frames
        ]


# ===========================================================================
# Analysis: join the Pi arm report to the host receiver report
# ===========================================================================

def analyze(pi_report: dict, host_report: dict) -> dict:
    """Compute host-arrival age from the two half-reports and the clock fit.

    This is where the far-side number actually gets produced, and it is
    deliberately NOT a subtraction of two wall clocks: the Pi's send realtime
    is mapped into host realtime with the measured four-timestamp offset, and
    the residual half-RTT is carried through as `uncertainty_ms` so nobody
    reads a 40 ms transit as exact.
    """
    clock = pi_report.get("clock") or {}
    offset_ns = clock.get("pi_to_host_offset_ns")
    out = {
        "arm": pi_report.get("arm"),
        "mode_requested": pi_report.get("mode_requested"),
        "clock_offset_ns": offset_ns,
        "clock_uncertainty_ms": clock.get("uncertainty_ms"),
        "connections": [],
    }
    if offset_ns is None:
        out["error"] = "no clock exchange; host-arrival age is not computable"
        return out
    for conn in host_report.get("connections", []):
        samples = conn.get("pipeline_ns_samples")
        if not samples:
            continue
        capture_to_send = [(s["send_mono_ns"] - s["capture_mono_ns"]) / 1e6
                           for s in samples]
        transit = [(s["arrival_real_ns"] - (s["send_real_ns"] + offset_ns)) / 1e6
                   for s in samples]
        total = [a + b for a, b in zip(capture_to_send, transit)]
        out["connections"].append({
            "role": conn.get("role"),
            "frames": conn.get("frames"),
            "arrival_fps": conn.get("arrival_fps"),
            "arrival_bytes_per_s": conn.get("arrival_bytes_per_s"),
            "arrival_gap_ms": conn.get("arrival_gap_ms"),
            "seq_skipped": conn.get("seq_skipped"),
            "bad_jpeg": conn.get("bad_jpeg"),
            "capture_to_send_ms": _stats(capture_to_send, 1.0, 2),
            "pi_to_host_transit_ms": _stats(transit, 1.0, 2),
            "host_arrival_age_ms": _stats(total, 1.0, 2),
            # A growing age is queueing; a flat age with jitter is not.  The
            # first-vs-last-decile comparison is the cheap version of that
            # test, and it is the one #217 says to run.
            "age_first_decile_median_ms": round(
                _pct(total[:max(1, len(total) // 10)], 0.5), 2) if total else None,
            "age_last_decile_median_ms": round(
                _pct(total[-max(1, len(total) // 10):], 0.5), 2) if total else None,
        })
    return out


# ===========================================================================

def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="cmd", required=True)

    recv = sub.add_parser("receiver", help="run on the PC host (far side)")
    recv.add_argument("--bind", default="0.0.0.0")
    recv.add_argument("--port", type=int, default=5099)
    recv.add_argument("--seconds", type=float, default=90.0)
    recv.add_argument("--out", default="-")

    arm = sub.add_parser("arm", help="run one §5.1 arm on the Pi")
    arm.add_argument("--name", required=True,
                     choices=("baseline", "preview", "preview_snapshot"))
    arm.add_argument("--seconds", type=float, default=60.0)
    arm.add_argument("--host", required=True)
    arm.add_argument("--port", type=int, default=5099)
    arm.add_argument("--mode", default="1280x720@30")
    arm.add_argument("--device", default=None,
                     help="stable by-path override; default is USB-identity selection")
    arm.add_argument("--snapshot-hz", type=float, default=0.0)
    arm.add_argument("--sample-period", type=float, default=5.0)
    arm.add_argument("--pcap-seconds", type=float, default=10.0)
    arm.add_argument("--exposure-dynamic-framerate", type=int, default=None,
                     choices=(0, 1))
    arm.add_argument("--out", default="-")

    probe = sub.add_parser("probe", help="enumerate candidates + negotiated mode only")
    probe.add_argument("--device", default=None)
    probe.add_argument("--out", default="-")

    ana = sub.add_parser("analyze", help="join a Pi arm report to its host report")
    ana.add_argument("--pi", required=True)
    ana.add_argument("--host-report", required=True)
    ana.add_argument("--out", default="-")

    args = parser.parse_args(argv)
    if args.cmd == "receiver":
        report = run_receiver(args)
    elif args.cmd == "arm":
        report = run_arm(args)
    elif args.cmd == "analyze":
        report = analyze(json.loads(Path(args.pi).read_text()),
                         json.loads(Path(args.host_report).read_text()))
    else:
        report = {"device_selection": v4l2.select_device(override=args.device)}

    text = json.dumps(report, indent=2, sort_keys=True)
    if args.out == "-":
        print(text)
    else:
        Path(args.out).write_text(text)
        print(f"wrote {args.out} ({len(text)} bytes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
