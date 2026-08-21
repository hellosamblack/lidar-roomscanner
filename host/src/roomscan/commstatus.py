"""Comm-status probing for the top-bar link indicator (owner ask, 2026-08-05;
reworked 2026-08-20 to retire the FileHub and add the Pi bridge + CrowPanel,
issues #192/#193/#209).

The web UI shows one small cluster in the top-right that answers "can the
server actually reach the rig?" for each hop the frames traverse:

  * **Bridge**    (`roomscan-bridge.local` via mDNS, falling back to the
    configured/last-resort IP -- #209 mDNS follow-up, see
    `resolve_bridge_target`) -- the Raspberry Pi bridge node that
    replaced the RavPower FileHub as the scanner's wireless uplink (issue
    #191). Its host-facing address is wlan0 via DHCP and can move, so the
    hostname is the durable target; the literal `DEFAULT_BRIDGE_IP`
    (172.17.2.66) is only reached when mDNS resolution fails. Reachability =
    ICMP echo against whichever address won, the same cadence as every other
    hop here;
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
  * **CrowPanel**  -- `comm-unit-7`, a `/ws-thin` v2 thin client (issues
    #202/#205) that is ALSO a real host on the network (mDNS `comm-unit-7.local`,
    falling back to `DEFAULT_CROWPANEL_IP`, same resolve-then-ping shape as
    Bridge). A `/ws-thin` session is the richer signal (it carries format/fps/
    credits), so `state` prefers it, but the unit reads "up" off ICMP alone
    too -- a CrowPanel that is powered on and reachable but not currently
    holding a `/ws-thin` flow (reconnect backoff, nobody has the viewer open)
    used to be indistinguishable from "never existed"; it no longer is.
    `informational` follows the Bridge's rule (`not up`): reachable (via
    either signal) is a real "up" fact, colored like the Bridge chip is when
    reachable; only true absence -- no ping reply AND no `/ws-thin` flow --
    is the neutral/informational state.
  * **CrowPanel Prog**  -- the CrowPanel's CH340 USB-serial programming
    adapter, if plugged into THIS host. Mirrors ST-Link exactly (same
    best-effort `lsusb` scan, same "absence is healthy" semantics) but keys
    off the CH340 vendor id instead.
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
import socket
import subprocess
import time
from dataclasses import dataclass

# Infrastructure addresses. Both overridable from the environment so a
# different deployment (travel mode, a second rig, no bridge at all) needs no
# code change. The scanner's is normally discovered via mDNS (sources.py) and
# the live UDP peer is the authoritative value -- web.py prefers that and
# falls back to this default.
#
# The Bridge's *literal* address is not durable (issue #209/#193 follow-up):
# eth0 sits on the scanner-side subnet, and the host-facing address is wlan0
# via DHCP -- it moves. `DEFAULT_BRIDGE_IP` (env-overridable, last-resort
# 172.17.2.66) is therefore only the FALLBACK target now; `probe()` resolves
# `DEFAULT_BRIDGE_HOSTNAME` first via `resolve_bridge_target` below and only
# falls back to this literal when that resolution fails.
DEFAULT_BRIDGE_IP = os.environ.get("ROOMSCAN_BRIDGE_IP", "172.17.2.66")
DEFAULT_BRIDGE_HOSTNAME = "roomscan-bridge.local"
DEFAULT_SCANNER_IP = os.environ.get("ROOMSCAN_SCANNER_IP", "172.17.2.58")

# The CrowPanel (`comm-unit-7`) is a real network host, not just a `/ws-thin`
# client -- resolved the same way as the Bridge (mDNS first, literal IP as the
# last-resort fallback) so its chip can read "up" off ICMP even when nothing
# has an active `/ws-thin` flow open to it.
DEFAULT_CROWPANEL_IP = os.environ.get("ROOMSCAN_CROWPANEL_IP", "172.17.2.67")
DEFAULT_CROWPANEL_HOSTNAME = "comm-unit-7.local"

# Resolving a `.local` name is a real multicast round-trip -- slow when mDNS
# is healthy, much slower (a full timeout) when it is not. `BRIDGE_RESOLVE_TTL_S`
# bounds how often that cost is paid: the ICMP probe cadence is 3s
# (`COMM_PROBE_INTERVAL_S` in web.py) and re-resolving on every tick would mean
# every single ping pays a multicast round-trip. Caching for a couple of
# minutes means a DHCP-moved wlan0 address is picked up promptly without
# paying resolution cost on the hot path. `BRIDGE_RESOLVE_TIMEOUT_S` bounds the
# resolution call itself so a dead mDNS responder degrades to "fell back to
# the configured IP" within ~1.5s, not a multi-second stall.
BRIDGE_RESOLVE_TTL_S = 120.0
BRIDGE_RESOLVE_TIMEOUT_S = 1.5

# ST-Link/V2-1 and V3 present VID 0483 with a PID in the 0x374x family. Matching
# on the VID alone is enough to say "a debug probe is attached to this host".
_STLINK_USB_VID = "0483"

# The CrowPanel's programming port is a CH340/CH34x USB-serial adapter
# (QinHeng Electronics, VID 1a86) -- distinct from ST-Link's VID above.
_CH340_USB_VID = "1a86"

# A stream slower than this (in Hz) counts as "not really flowing" for the
# scanner detail line -- covers the idle/parked laser case where only the IMU
# trickles. It is a display nicety, never the reachability verdict (ping is).
_STREAMING_FLOOR_HZ = 1.0


@dataclass(frozen=True)
class PingResult:
    up: bool
    rtt_ms: float | None = None


@dataclass
class _BridgeResolveCache:
    """Mutable, module-scoped cache for the last resolved bridge target.

    A plain dataclass (not frozen) so `resolve_bridge_target` can update it in
    place; tests construct their own instance to avoid cross-test leakage
    rather than mutating the module-level singleton.
    """
    addr: str | None = None
    resolved_from: str | None = None
    expires_at: float = 0.0


#: Shared across every `probe()`/`resolve_bridge_target()` call in this
#: process so the ICMP loop (3s cadence) and the ssh health loop (45s
#: cadence, web.py) pay for exactly one resolution per TTL window between them.
_bridge_resolve_cache = _BridgeResolveCache()

#: Same idea, separate cache -- CrowPanel and Bridge resolve independently
#: (different hostnames, different TTL windows), so they must not share one
#: `_BridgeResolveCache` instance or the second resolve would clobber the first.
_crowpanel_resolve_cache = _BridgeResolveCache()


def _resolve_hostname_sync(hostname: str) -> str | None:
    """Blocking `getaddrinfo` -- ONLY ever called inside an executor thread
    (see `resolve_bridge_target`), never on the event loop directly. Returns
    the first IPv4 address, or None on any failure (no responder, mDNS down,
    hostname unknown) -- never raises."""
    try:
        infos = socket.getaddrinfo(hostname, None, family=socket.AF_INET)
    except Exception:
        return None
    return infos[0][4][0] if infos else None


async def resolve_bridge_target(
    configured_ip: str,
    *,
    hostname: str = DEFAULT_BRIDGE_HOSTNAME,
    timeout_s: float = BRIDGE_RESOLVE_TIMEOUT_S,
    ttl_s: float = BRIDGE_RESOLVE_TTL_S,
    cache: _BridgeResolveCache | None = None,
    now: float | None = None,
) -> dict:
    """Resolve the bridge probe target: `hostname` (mDNS) first, falling back
    to `configured_ip` (env `ROOMSCAN_BRIDGE_IP` / config `bridge_ip`, itself
    defaulting to the last-resort literal `DEFAULT_BRIDGE_IP`). Returns
    `{"addr": ..., "resolved_from": "mdns"|"fallback", "hostname": hostname}`.

    Never touches the event loop with `getaddrinfo` directly -- the `.local`
    lookup is a real multicast round-trip that can block far longer than a
    ping, especially when nothing answers. The lookup runs in the default
    executor under `timeout_s`, and the result (win OR fallback) is cached for
    `ttl_s` so the 3s ICMP cadence isn't paying a resolution per ping; caching
    the fallback too (not just successes) means a live mDNS recovery is picked
    up again once the TTL lapses, rather than being pinned to fallback forever
    the first time it fires.
    """
    cache = cache if cache is not None else _bridge_resolve_cache
    t = time.monotonic() if now is None else now
    if cache.addr is not None and t < cache.expires_at:
        return {"addr": cache.addr, "resolved_from": cache.resolved_from, "hostname": hostname}

    loop = asyncio.get_running_loop()
    try:
        addr = await asyncio.wait_for(
            loop.run_in_executor(None, _resolve_hostname_sync, hostname), timeout=timeout_s)
    except Exception:
        addr = None

    if addr:
        cache.addr, cache.resolved_from, cache.expires_at = addr, "mdns", t + ttl_s
    else:
        cache.addr, cache.resolved_from, cache.expires_at = configured_ip, "fallback", t + ttl_s
    return {"addr": cache.addr, "resolved_from": cache.resolved_from, "hostname": hostname}


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


def detect_crowpanel_prog() -> bool:
    """True iff the CrowPanel's CH340 USB-serial programming adapter is
    attached to THIS host (best effort). Structurally identical to
    `detect_stlink` -- same `lsusb` scan, same "can't tell degrades to not
    present" contract -- just matching the CH340 vendor id instead.
    """
    lsusb = shutil.which("lsusb")
    if lsusb is None:
        return False
    try:
        out = subprocess.run([lsusb], capture_output=True, text=True, timeout=2.0)
    except (OSError, subprocess.SubprocessError):
        return False
    for line in out.stdout.splitlines():
        # `lsusb` lines read "... ID 1a86:7522 QinHeng Electronics CH340 serial converter".
        if f"ID {_CH340_USB_VID}:" in line or "ch340" in line.lower():
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


def _bridge_link(bridge: PingResult, bridge_ip: str, health: dict | None,
                 resolved_from: str | None = None) -> dict:
    wlan = (health or {}).get("wlan0") or {}
    d = {
        "reachable": bridge.up,
        "rtt_ms": bridge.rtt_ms,
        "addr": bridge_ip,
        "rssi_dbm": wlan.get("rssi_dbm"),
        "bitrate_mbps": wlan.get("bitrate_mbps"),
        "ssh_latency_ms": (health or {}).get("ssh_latency_ms"),
    }
    if resolved_from is not None:
        # "mdns" | "fallback" -- which target actually got probed (#209
        # mDNS follow-up). Surfaced so the modal can say e.g. "resolved via
        # mDNS" vs. "using the configured fallback address" rather than
        # silently showing whichever address won.
        d["resolved_from"] = resolved_from
    return d


def _scanner_addr_and_via(scanner_ip: str, bridge_ip: str, bridge_up: bool,
                          bridge_health: dict | None) -> tuple[str, dict | None]:
    """Work out what address to show for the Scanner chip, and whether the
    stream is actually arriving via the Bridge's NAT hop (#209 mDNS follow-up).

    When the host receives frames via the bridge, the apparent source address
    of the UDP stream (parsed in web.py's `_scanner_ip`) is the BRIDGE's own
    masquerade/NAT address on its host-facing interface -- not the scanner's.
    That is exactly `bridge_ip` once resolution (mDNS or fallback) has run, so
    `scanner_ip == bridge_ip` while the bridge is reachable is the signal.

    Decision (#209/#193): prefer the scanner's real behind-bridge lease IP
    from the already-cached `bridge_health` (`eth0.scanner_lease.ip`,
    collected by the existing ssh health poll -- this adds no new ssh
    traffic). When no health has been cached yet, or the lease is missing,
    there is no address left that is honestly "the scanner's" -- so this
    returns no addr at all rather than presenting the NAT address as one; the
    caller folds the NAT address into `detail`/`link` as "via bridge (<addr>)"
    instead.
    """
    if not (bridge_up and scanner_ip and bridge_ip and scanner_ip == bridge_ip):
        return scanner_ip, None
    lease = ((bridge_health or {}).get("eth0") or {}).get("scanner_lease")
    lease_ip = lease.get("ip") if isinstance(lease, dict) else \
        (lease if isinstance(lease, str) else None)
    if lease_ip:
        return lease_ip, {"via_bridge": True, "bridge_addr": scanner_ip, "source": "lease"}
    return None, {"via_bridge": True, "bridge_addr": scanner_ip, "source": "nat"}


def _crowpanel_detail(connected: bool, cp: dict | None, reachable: bool = False,
                      rtt_ms: float | None = None) -> str:
    if not connected or not cp:
        if reachable:
            return f"reachable, no /ws-thin session ({_fmt_rtt(rtt_ms)})"
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
    bridge_resolved_from: str | None = None,
    crowpanel_ping: PingResult | None = None,
    crowpanel_ip: str | None = None,
    crowpanel_resolved_from: str | None = None,
    crowpanel_prog_present: bool = False,
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

    `bridge_ip` is whatever address was actually probed -- `probe()` resolves
    `DEFAULT_BRIDGE_HOSTNAME` (or falls back) before calling this, so by the
    time it gets here it is a plain IP; `bridge_resolved_from` says how that
    IP was obtained ("mdns" | "fallback" | None when the caller didn't resolve
    at all, e.g. a direct unit-test call). See `_scanner_addr_and_via` for how
    that same resolved address is used to keep the Scanner chip honest when
    the stream is arriving via the Bridge's NAT hop.

    `crowpanel_ping`/`crowpanel_ip`/`crowpanel_resolved_from` mirror the
    Bridge's resolve-then-ping facts for the CrowPanel's network identity
    (`comm-unit-7`) -- all default to "not probed" so callers that only pass
    the `/ws-thin` `crowpanel` dict (older call sites, most unit tests) keep
    behaving exactly as before. `crowpanel_prog_present` is the CrowPanel's
    CH340 programming-port analog of `stlink_present`.
    """
    bridge_up = bridge.up
    bridge_target = {
        "id": "bridge",
        "label": "Bridge",
        "addr": bridge_ip,
        "state": "up" if bridge_up else "absent",
        "detail": _bridge_detail(bridge_up, bridge.rtt_ms, bridge_health),
        "informational": not bridge_up,
        "link": _bridge_link(bridge, bridge_ip, bridge_health, bridge_resolved_from),
        "device": bridge_health if (bridge_up and bridge_health) else None,
        "management": {"reboot": True, "logs": True} if bridge_up else None,
    }

    if not scanner.up:
        scanner_detail = "no reply"
    elif scanner_fps is not None and scanner_fps >= _STREAMING_FLOOR_HZ:
        scanner_detail = f"streaming {scanner_fps:.1f} fps"
    else:
        scanner_detail = f"reachable, no frames ({_fmt_rtt(scanner.rtt_ms)})"

    scanner_addr, via_bridge = _scanner_addr_and_via(scanner_ip, bridge_ip, bridge_up,
                                                      bridge_health)
    if via_bridge is not None:
        if via_bridge["source"] == "lease":
            scanner_detail = f"{scanner_detail} -- via bridge (lease {scanner_addr})"
        else:
            scanner_detail = f"{scanner_detail} -- via bridge ({via_bridge['bridge_addr']})"
    scanner_link = {"reachable": scanner.up, "rtt_ms": scanner.rtt_ms, "addr": scanner_addr}
    if via_bridge is not None:
        scanner_link.update(via_bridge)
    scanner_target = {
        "id": "scanner",
        "label": "Scanner",
        "addr": scanner_addr,
        "state": "up" if scanner.up else "down",
        "detail": scanner_detail,
        "link": scanner_link,
        "device": {"fps": scanner_fps, "transport": scanner_transport} if scanner.up else None,
        "management": None,
    }

    cp_connected = bool(crowpanel and crowpanel.get("connected"))
    cp_reachable = bool(crowpanel_ping and crowpanel_ping.up)
    cp_up = cp_connected or cp_reachable
    crowpanel_link = {
        "connected": cp_connected,
        "reachable": cp_reachable,
        "rtt_ms": crowpanel_ping.rtt_ms if crowpanel_ping else None,
        "addr": crowpanel_ip,
    }
    if crowpanel_resolved_from is not None:
        crowpanel_link["resolved_from"] = crowpanel_resolved_from
    if cp_connected:
        crowpanel_link.update({
            "count": (crowpanel or {}).get("count", 0),
            "tx_fps": (crowpanel or {}).get("tx_fps"),
            "tx_bytes_per_s": (crowpanel or {}).get("tx_bytes_per_s"),
            "credits": (crowpanel or {}).get("credits"),
            "credits_max": (crowpanel or {}).get("credits_max"),
            "proto": (crowpanel or {}).get("proto"),
        })
    crowpanel_target = {
        "id": "crowpanel",
        "label": "CrowPanel",
        "addr": crowpanel_ip if cp_reachable else None,
        "state": "up" if cp_up else "absent",
        "detail": _crowpanel_detail(cp_connected, crowpanel, cp_reachable,
                                    crowpanel_ping.rtt_ms if crowpanel_ping else None),
        "informational": not cp_up,
        "link": crowpanel_link,
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
    crowpanel_prog_target = {
        "id": "crowpanel_prog",
        "label": "CrowPanel Prog",
        "addr": None,
        "state": "up" if crowpanel_prog_present else "absent",
        "detail": "programming adapter attached" if crowpanel_prog_present
                   else "not connected (informational)",
        "informational": True,
        "link": {"present": crowpanel_prog_present},
        "device": None,
        "management": None,
    }

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
            "targets": [crowpanel_target, crowpanel_prog_target, bridge_target,
                        scanner_target, stlink_target]}


async def probe(
    bridge_ip: str,
    scanner_ip: str,
    scanner_fps: float | None = None,
    scanner_transport: str | None = None,
    bridge_health: dict | None = None,
    crowpanel: dict | None = None,
    timeout_s: float = 1.0,
    bridge_hostname: str = DEFAULT_BRIDGE_HOSTNAME,
    crowpanel_ip: str = DEFAULT_CROWPANEL_IP,
    crowpanel_hostname: str = DEFAULT_CROWPANEL_HOSTNAME,
) -> dict:
    """Probe the pingable hops (CrowPanel, Bridge, Scanner, ST-Link, CrowPanel
    Prog) and return the `comm` message. Never raises.

    `bridge_ip` here is the CONFIGURED fallback (env `ROOMSCAN_BRIDGE_IP` /
    config `bridge_ip`, itself defaulting to the last-resort literal
    `DEFAULT_BRIDGE_IP`) -- `bridge_hostname` (mDNS) is tried first via
    `resolve_bridge_target`, and only the winning address is actually pinged.
    `crowpanel_ip`/`crowpanel_hostname` go through the exact same
    resolve-then-ping shape, with their own cache (`_crowpanel_resolve_cache`)
    so the two resolutions don't clobber each other.
    `bridge_health` and `crowpanel` are NOT collected here -- they are already-
    gathered facts the caller hands in (see the module docstring): an ssh poll
    is far too slow/heavy to run on this cadence, and the `/ws-thin` flow facts
    live in web.py's own thin-client map, not anything this module can reach.
    All pings run concurrently; the USB scans (ST-Link, CrowPanel Prog) are
    sync and cheap but still offloaded to a thread so they can't stall the
    loop. Any probe that dies is treated as its failure verdict (down /
    absent), so a flaky `ping` degrades the indicator rather than the server.
    """
    bridge_resolved, crowpanel_resolved = await asyncio.gather(
        resolve_bridge_target(bridge_ip, hostname=bridge_hostname),
        resolve_bridge_target(crowpanel_ip, hostname=crowpanel_hostname,
                              cache=_crowpanel_resolve_cache),
    )
    bridge_addr = bridge_resolved["addr"]
    crowpanel_addr = crowpanel_resolved["addr"]

    loop = asyncio.get_running_loop()
    bridge, scanner, crowpanel_ping, stlink, crowpanel_prog = await asyncio.gather(
        ping_host(bridge_addr, timeout_s),
        ping_host(scanner_ip, timeout_s),
        ping_host(crowpanel_addr, timeout_s),
        loop.run_in_executor(None, detect_stlink),
        loop.run_in_executor(None, detect_crowpanel_prog),
        return_exceptions=True,
    )
    if isinstance(bridge, BaseException):
        bridge = PingResult(False)
    if isinstance(scanner, BaseException):
        scanner = PingResult(False)
    if isinstance(crowpanel_ping, BaseException):
        crowpanel_ping = PingResult(False)
    if isinstance(stlink, BaseException):
        stlink = False
    if isinstance(crowpanel_prog, BaseException):
        crowpanel_prog = False
    return build_comm_message(
        bridge=bridge, scanner=scanner, stlink_present=bool(stlink),
        bridge_ip=bridge_addr, scanner_ip=scanner_ip, scanner_fps=scanner_fps,
        scanner_transport=scanner_transport, bridge_health=bridge_health,
        crowpanel=crowpanel, bridge_resolved_from=bridge_resolved["resolved_from"],
        crowpanel_ping=crowpanel_ping, crowpanel_ip=crowpanel_addr,
        crowpanel_resolved_from=crowpanel_resolved["resolved_from"],
        crowpanel_prog_present=bool(crowpanel_prog))
