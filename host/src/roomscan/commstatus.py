"""Comm-status probing for the top-bar link indicator (owner ask, 2026-08-05;
reworked 2026-08-20 to retire the FileHub and add the Pi bridge + CrowPanel,
issues #192/#193/#209).

The web UI shows one small cluster in the top-right that answers "can the
server actually reach the rig?" for each hop the frames traverse:

  * **Bridge**    (default 172.17.2.66) -- the Raspberry Pi bridge node that
    replaced the RavPower FileHub as the scanner's wireless uplink (issue
    #191). Reachability = ICMP echo, the same cadence as every other hop here;
    a slower, separate ssh poll (`host/tools/pi_bridge.py`, driven from
    web.py -- this module stays a pure transport layer with no ssh of its
    own) enriches the chip with radio RSSI/bitrate, uptime-ish health, tee
    fill and the scanner's DHCP lease, cached and passed in as `bridge_health`.
    **Direct Ethernet is a first-class path** (no bridge in the loop at all),
    so an unreachable Bridge reads as `absent`/informational, exactly like
    ST-Link below -- never `down`, which would read as a fault on a rig that
    was never wired through the bridge in the first place. A ping failure
    cannot tell "no bridge deployed" apart from "bridge deployed but dead"
    (both look identical from here), so this module does not try -- it is the
    honest state ICMP can actually see.
  * **Scanner**   (default 172.17.2.58) -- the STM32H563. Reachability = ICMP
    echo, enriched with whether DATA frames are *actually arriving* right now
    (the real data path, from the MetricsRegistry) so a reachable-but-silent
    board reads differently from a streaming one.
  * **CrowPanel**  -- a `/ws-thin` v2 thin client (issues #202/#205). Not
    pinged (it is a local websocket, not an IP target this host reaches out
    to); web.py hands in whatever it already knows about the connected
    flow(s). No connection is a normal, informational state -- most sessions
    never have one attached.
  * **ST-Link**  -- the SWD/VCOM debug probe, if one is plugged into THIS
    host. Purely informational: on the normal Ethernet-tethered rig there is
    no ST-Link here at all, and its absence is a healthy state, not a fault.

Everything here is transport-only and pure enough to test: `build_comm_message`
assembles the wire dict from already-collected facts, `ping_host` is a thin
async wrapper over the system `ping` (works unprivileged on this Linux host --
verified against both rig addresses), and `detect_stlink` is a cheap best-effort
USB scan. The web layer runs `probe()` on a slow cadence and broadcasts the
result as the `/ws` `comm` message (docs/web-protocol.md); nothing here binds a
socket or touches the device stream. `bridge_health` / `crowpanel` are just
already-collected dicts handed through -- collecting them (an ssh hop, a
websocket flow map) is web.py's job, not this module's, so it stays free of any
transport heavier than ICMP + a USB list.
"""

from __future__ import annotations

import asyncio
import os
import shutil
import subprocess
from dataclasses import dataclass

# Infrastructure addresses. Both overridable from the environment so a
# different deployment (travel mode, a second rig, no bridge at all) needs no
# code change. The scanner's is normally discovered via mDNS (sources.py) and
# the live UDP peer is the authoritative value -- web.py prefers that and
# falls back to this default.
DEFAULT_BRIDGE_IP = os.environ.get("ROOMSCAN_BRIDGE_IP", "172.17.2.66")
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


def _bridge_detail(up: bool, rtt_ms: float | None, health: dict | None) -> str:
    if not up:
        return "not reachable (direct Ethernet, or bridge powered off)"
    wlan = (health or {}).get("wlan0") or {}
    rssi = wlan.get("rssi_dbm")
    bits = []
    if rssi is not None:
        bits.append(f"{rssi} dBm")
    rate = wlan.get("bitrate_mbps")
    if rate is not None:
        bits.append(f"{rate} Mb/s")
    if bits:
        return ", ".join(bits)
    return _fmt_rtt(rtt_ms)


def _bridge_link(bridge: PingResult, bridge_ip: str, health: dict | None) -> dict:
    wlan = (health or {}).get("wlan0") or {}
    return {
        "reachable": bridge.up,
        "rtt_ms": bridge.rtt_ms,
        "addr": bridge_ip,
        "rssi_dbm": wlan.get("rssi_dbm"),
        "bitrate_mbps": wlan.get("bitrate_mbps"),
        "ssh_latency_ms": (health or {}).get("ssh_latency_ms"),
    }


def _crowpanel_detail(connected: bool, cp: dict | None) -> str:
    if not connected or not cp:
        return "not connected (informational)"
    fmt = cp.get("format", "raw")
    fps = cp.get("fps")
    fps_txt = f"{fps:.0f} fps" if isinstance(fps, (int, float)) else "? fps"
    credits = cp.get("credits")
    credits_max = cp.get("credits_max")
    cred_txt = f", credits {credits}/{credits_max}" if credits is not None and credits_max else ""
    count = cp.get("count", 1)
    extra = f" (+{count - 1} more)" if isinstance(count, int) and count > 1 else ""
    return f"{fmt} {fps_txt}{cred_txt}{extra}"


def build_comm_message(
    *,
    bridge: PingResult,
    scanner: PingResult,
    stlink_present: bool,
    bridge_ip: str,
    scanner_ip: str,
    scanner_fps: float | None = None,
    scanner_transport: str | None = None,
    bridge_health: dict | None = None,
    crowpanel: dict | None = None,
) -> dict:
    """Assemble the `/ws` `comm` message from already-gathered facts (pure).

    `state` is one of "up" | "down" | "absent" | "unknown". `informational`
    marks a chip whose absence is a normal state, not a fault (Bridge and
    CrowPanel join ST-Link in that set as of #209 -- see the module docstring
    for why Bridge specifically can never read "down"). Every target carries
    `link` (reachability-layer facts), `device` (health, or None when there is
    none to show), and `management` (available control actions, or None) so
    the UI's per-chip modal has one schema to render regardless of which chip
    was clicked.
    """
    bridge_up = bridge.up
    bridge_target = {
        "id": "bridge",
        "label": "Bridge",
        "addr": bridge_ip,
        "state": "up" if bridge_up else "absent",
        "detail": _bridge_detail(bridge_up, bridge.rtt_ms, bridge_health),
        "informational": not bridge_up,
        "link": _bridge_link(bridge, bridge_ip, bridge_health),
        "device": bridge_health if (bridge_up and bridge_health) else None,
        "management": {"reboot": True, "logs": True} if bridge_up else None,
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
        "link": {"reachable": scanner.up, "rtt_ms": scanner.rtt_ms, "addr": scanner_ip},
        "device": {"fps": scanner_fps, "transport": scanner_transport} if scanner.up else None,
        "management": None,
    }

    cp_connected = bool(crowpanel and crowpanel.get("connected"))
    crowpanel_target = {
        "id": "crowpanel",
        "label": "CrowPanel",
        "addr": None,
        "state": "up" if cp_connected else "absent",
        "detail": _crowpanel_detail(cp_connected, crowpanel),
        "informational": not cp_connected,
        "link": {
            "connected": cp_connected,
            "count": (crowpanel or {}).get("count", 0),
            "tx_fps": (crowpanel or {}).get("tx_fps"),
            "tx_bytes_per_s": (crowpanel or {}).get("tx_bytes_per_s"),
            "credits": (crowpanel or {}).get("credits"),
            "credits_max": (crowpanel or {}).get("credits_max"),
            "proto": (crowpanel or {}).get("proto"),
        } if cp_connected else {"connected": False},
        "device": {
            "format": crowpanel.get("format"),
            "fps": crowpanel.get("fps"),
            "width": crowpanel.get("width"),
            "height": crowpanel.get("height"),
            "dropped": crowpanel.get("dropped"),
        } if cp_connected else None,
        "management": None,
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
        "link": {"present": stlink_present},
        "device": None,
        "management": None,
    }

    return {"type": "comm",
            "targets": [bridge_target, scanner_target, crowpanel_target, stlink_target]}


async def probe(
    bridge_ip: str,
    scanner_ip: str,
    scanner_fps: float | None = None,
    scanner_transport: str | None = None,
    bridge_health: dict | None = None,
    crowpanel: dict | None = None,
    timeout_s: float = 1.0,
) -> dict:
    """Probe the pingable hops (Bridge, Scanner, ST-Link) and return the `comm`
    message. Never raises.

    `bridge_health` and `crowpanel` are NOT collected here -- they are already-
    gathered facts the caller hands in (see the module docstring): an ssh poll
    is far too slow/heavy to run on this cadence, and CrowPanel state lives in
    web.py's own thin-client map, not anything this module can reach. The two
    pings run concurrently; the ST-Link USB scan is sync and cheap but still
    offloaded to a thread so it can't stall the loop. Any probe that dies is
    treated as its failure verdict (down / absent), so a flaky `ping` degrades
    the indicator rather than the server.
    """
    loop = asyncio.get_running_loop()
    bridge, scanner, stlink = await asyncio.gather(
        ping_host(bridge_ip, timeout_s),
        ping_host(scanner_ip, timeout_s),
        loop.run_in_executor(None, detect_stlink),
        return_exceptions=True,
    )
    if isinstance(bridge, BaseException):
        bridge = PingResult(False)
    if isinstance(scanner, BaseException):
        scanner = PingResult(False)
    if isinstance(stlink, BaseException):
        stlink = False
    return build_comm_message(
        bridge=bridge, scanner=scanner, stlink_present=bool(stlink),
        bridge_ip=bridge_ip, scanner_ip=scanner_ip, scanner_fps=scanner_fps,
        scanner_transport=scanner_transport, bridge_health=bridge_health,
        crowpanel=crowpanel)
