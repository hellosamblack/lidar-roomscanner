"""Comm-status probing for the top-bar link indicator (owner ask, 2026-08-05).

The web UI shows one small cluster in the top-right that answers "can the server
actually reach the rig?" for each hop the frames traverse:

  * **FileHub**  (default 172.17.2.57) -- the RavPower Wi-Fi bridge the rig sits
    behind (see the wifi-bridge-filehub memory / docs). Reachability = ICMP echo.
  * **Scanner**  (default 172.17.2.58) -- the STM32H563. Reachability = ICMP echo,
    enriched with whether DATA frames are *actually arriving* right now (the real
    data path, from the MetricsRegistry) so a reachable-but-silent board reads
    differently from a streaming one.
  * **ST-Link**  -- the SWD/VCOM debug probe, if one is plugged into THIS host.
    Purely informational: on the normal Ethernet-tethered rig there is no
    ST-Link here at all, and its absence is a healthy state, not a fault.

Everything here is transport-only and pure enough to test: `build_comm_message`
assembles the wire dict from already-collected facts, `ping_host` is a thin
async wrapper over the system `ping` (works unprivileged on this Linux host --
verified against both rig addresses), and `detect_stlink` is a cheap best-effort
USB scan. The web layer runs `probe()` on a slow cadence and broadcasts the
result as the `/ws` `comm` message (docs/web-protocol.md); nothing here binds a
socket or touches the device stream.
"""

from __future__ import annotations

import asyncio
import os
import shutil
import subprocess
from dataclasses import dataclass

# Infrastructure addresses. The FileHub is a fixed management address on this
# network; the scanner's is normally discovered via mDNS (sources.py) and the
# live UDP peer is the authoritative value -- web.py prefers that and falls back
# to this default. Both are overridable from the environment so a different
# deployment (travel mode, a second rig) needs no code change.
DEFAULT_FILEHUB_IP = os.environ.get("ROOMSCAN_FILEHUB_IP", "172.17.2.57")
DEFAULT_SCANNER_IP = os.environ.get("ROOMSCAN_SCANNER_IP", "172.17.2.58")

# ST-Link/V2-1 and V3 present VID 0483 with a PID in the 0x374x family. Matching
# on the VID alone is enough to say "a debug probe is attached to this host".
_STLINK_USB_VID = "0483"

# A stream slower than this (in Hz) counts as "not really flowing" for the
# scanner detail line -- covers the idle/parked laser case where only the IMU
# trickles. It is a display nicety, never the reachability verdict (ping is).
_STREAMING_FLOOR_HZ = 1.0


@dataclass(frozen=True)
class PingResult:
    up: bool
    rtt_ms: float | None = None


async def ping_host(ip: str, timeout_s: float = 1.0) -> PingResult:
    """One ICMP echo to `ip`, off the event loop via an async subprocess.

    Uses the system `ping` (`-c1 -W<timeout>`) rather than a raw socket so it
    needs no CAP_NET_RAW/root. Any failure -- no `ping`, timeout, unreachable,
    non-zero exit -- is reported as `up=False`, never raised: a probe must not
    be able to kill the loop that runs it.
    """
    ping = shutil.which("ping")
    if ping is None:
        return PingResult(False)
    # -W is whole seconds on iputils ping; round up so a sub-second timeout still
    # allows at least one second of wait rather than truncating to 0 (== no wait).
    wait_s = max(1, int(timeout_s + 0.999))
    try:
        proc = await asyncio.create_subprocess_exec(
            ping, "-c", "1", "-W", str(wait_s), ip,
            stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)
    except OSError:
        return PingResult(False)
    try:
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=wait_s + 1.0)
    except (asyncio.TimeoutError, Exception):
        try:
            proc.kill()
        except ProcessLookupError:
            pass
        return PingResult(False)
    if proc.returncode != 0:
        return PingResult(False)
    return PingResult(True, _parse_rtt_ms(stdout.decode("ascii", "replace")))


def _parse_rtt_ms(text: str) -> float | None:
    """Pull the 'time=1.23 ms' round-trip out of `ping`'s one-line reply."""
    marker = "time="
    i = text.find(marker)
    if i < 0:
        return None
    tail = text[i + len(marker):]
    num = tail.split("ms", 1)[0].strip()
    try:
        return round(float(num), 1)
    except ValueError:
        return None


def detect_stlink() -> bool:
    """True iff an ST-Link debug probe is attached to THIS host (best effort).

    Cheap and privilege-free: scans `lsusb` for the ST-Link USB vendor id.
    Returns False (rather than raising) when `lsusb` is absent or errors -- the
    ST-Link line is informational, so "can't tell" degrades to "not present".
    """
    lsusb = shutil.which("lsusb")
    if lsusb is None:
        return False
    try:
        out = subprocess.run([lsusb], capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return False
    for line in out.stdout.splitlines():
        # `lsusb` lines read "... ID 0483:374b STMicroelectronics ST-LINK/V2.1".
        if f"ID {_STLINK_USB_VID}:" in line or "st-link" in line.lower():
            return True
    return False


def _fmt_rtt(rtt_ms: float | None) -> str:
    return f"{rtt_ms:g} ms" if rtt_ms is not None else "reachable"


def build_comm_message(
    *,
    filehub: PingResult,
    scanner: PingResult,
    stlink_present: bool,
    filehub_ip: str,
    scanner_ip: str,
    scanner_fps: float | None = None,
) -> dict:
    """Assemble the `/ws` `comm` message from already-gathered facts (pure).

    `state` is one of "up" | "down" | "absent" | "unknown". The scanner's detail
    reports live throughput when frames are flowing, so a reachable-but-silent
    board is visibly distinct from a streaming one.
    """
    filehub_target = {
        "id": "filehub",
        "label": "FileHub",
        "addr": filehub_ip,
        "state": "up" if filehub.up else "down",
        "detail": _fmt_rtt(filehub.rtt_ms) if filehub.up else "no reply",
    }

    if not scanner.up:
        scanner_detail = "no reply"
    elif scanner_fps is not None and scanner_fps >= _STREAMING_FLOOR_HZ:
        scanner_detail = f"streaming {scanner_fps:.1f} fps"
    else:
        scanner_detail = f"reachable, no frames ({_fmt_rtt(scanner.rtt_ms)})"
    scanner_target = {
        "id": "scanner",
        "label": "Scanner",
        "addr": scanner_ip,
        "state": "up" if scanner.up else "down",
        "detail": scanner_detail,
    }

    # Informational: absence is normal on the Ethernet-tethered rig, so it is its
    # own "absent" state (rendered neutral), never "down" (which reads as a fault).
    stlink_target = {
        "id": "stlink",
        "label": "ST-Link",
        "addr": None,
        "state": "up" if stlink_present else "absent",
        "detail": "debug probe attached" if stlink_present else "not connected (informational)",
        "informational": True,
    }

    return {"type": "comm", "targets": [filehub_target, scanner_target, stlink_target]}


async def probe(
    filehub_ip: str,
    scanner_ip: str,
    scanner_fps: float | None = None,
    timeout_s: float = 1.0,
) -> dict:
    """Probe all three hops and return the `comm` message. Never raises.

    The two pings run concurrently; the ST-Link USB scan is sync and cheap but
    still offloaded to a thread so it can't stall the loop. Any probe that dies
    is treated as its failure verdict (down / absent), so a flaky `ping` degrades
    the indicator rather than the server.
    """
    loop = asyncio.get_running_loop()
    filehub, scanner, stlink = await asyncio.gather(
        ping_host(filehub_ip, timeout_s),
        ping_host(scanner_ip, timeout_s),
        loop.run_in_executor(None, detect_stlink),
        return_exceptions=True,
    )
    if isinstance(filehub, BaseException):
        filehub = PingResult(False)
    if isinstance(scanner, BaseException):
        scanner = PingResult(False)
    if isinstance(stlink, BaseException):
        stlink = False
    return build_comm_message(
        filehub=filehub, scanner=scanner, stlink_present=bool(stlink),
        filehub_ip=filehub_ip, scanner_ip=scanner_ip, scanner_fps=scanner_fps)
