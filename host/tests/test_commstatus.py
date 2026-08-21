"""Tests for the top-bar comm-status indicator (commstatus.py + web wiring)."""

import asyncio
from types import SimpleNamespace

import pytest

from roomscan import commstatus
from roomscan.commstatus import PingResult, build_comm_message


def _by_id(msg, tid):
    return next(t for t in msg["targets"] if t["id"] == tid)


def test_message_shape_and_order():
    msg = build_comm_message(
        bridge=PingResult(True, 1.2), scanner=PingResult(True, 0.8),
        stlink_present=False, bridge_ip="172.17.2.66", scanner_ip="172.17.2.58",
        scanner_fps=27.8)
    assert msg["type"] == "comm"
    assert [t["id"] for t in msg["targets"]] == \
        ["crowpanel", "crowpanel_prog", "bridge", "scanner", "stlink"]


def test_bridge_up_carries_rtt_and_addr():
    up = _by_id(build_comm_message(
        bridge=PingResult(True, 3.0), scanner=PingResult(False), stlink_present=False,
        bridge_ip="a", scanner_ip="b"), "bridge")
    assert up["state"] == "up" and "3" in up["detail"] and up["addr"] == "a"
    assert up["informational"] is False
    assert up["link"]["reachable"] is True and up["link"]["rtt_ms"] == 3.0


def test_bridge_unreachable_is_absent_not_down():
    """Direct Ethernet is a first-class path (#193): an unreachable Bridge must
    read as informational absence, never a red 'down' fault -- ICMP cannot
    tell 'no bridge deployed' apart from 'bridge deployed but dead', so this
    is the only honest state to show. Regression: an earlier draft reported
    'down' here, which is exactly the defect this test pins."""
    down = _by_id(build_comm_message(
        bridge=PingResult(False), scanner=PingResult(False), stlink_present=False,
        bridge_ip="a", scanner_ip="b"), "bridge")
    assert down["state"] == "absent"
    assert down["state"] != "down"
    assert down["informational"] is True
    assert down["management"] is None


def test_bridge_health_enriches_link_and_device_when_reachable():
    health = {
        "wlan0": {"rssi_dbm": -52, "bitrate_mbps": 72.2},
        "ssh_latency_ms": 41.0,
        "tee": {"active": True, "ring_files": 4, "ring_bytes": 1234},
        "eth0": {"scanner_lease": "172.31.253.10"},
        "temp_c": 48.1,
    }
    t = _by_id(build_comm_message(
        bridge=PingResult(True, 2.5), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", bridge_health=health), "bridge")
    assert t["state"] == "up"
    assert t["link"]["rssi_dbm"] == -52
    assert t["link"]["bitrate_mbps"] == 72.2
    assert "-52 dBm" in t["detail"]
    assert t["device"] == health
    assert t["management"] == {"reboot": True, "logs": True}


def test_bridge_health_absent_omits_device_section():
    """No poll yet (or the bridge isn't reachable) -> device is None, so the
    UI can say 'no data yet' rather than render an empty/stale table."""
    t = _by_id(build_comm_message(
        bridge=PingResult(True, 2.5), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", bridge_health=None), "bridge")
    assert t["device"] is None


def test_scanner_streaming_vs_reachable_vs_down():
    # Reachable AND frames flowing -> streaming with the rate.
    streaming = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        bridge_ip="a", scanner_ip="b", scanner_fps=30.4), "scanner")
    assert streaming["state"] == "up" and "streaming 30.4 fps" == streaming["detail"]
    assert streaming["device"] == {"fps": 30.4, "transport": None}

    # Reachable but silent (idle laser / no frames) -> distinct detail, still up.
    silent = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        bridge_ip="a", scanner_ip="b", scanner_fps=0.0), "scanner")
    assert silent["state"] == "up" and "no frames" in silent["detail"]

    # Unreachable -> down (the scanner, unlike the bridge, is always expected
    # to be present on this rig, so a genuine fault reads as a fault).
    down = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(False), stlink_present=False,
        bridge_ip="a", scanner_ip="b", scanner_fps=30.0), "scanner")
    assert down["state"] == "down" and down["detail"] == "no reply"
    assert down["device"] is None


def test_scanner_transport_passthrough():
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        bridge_ip="a", scanner_ip="b", scanner_fps=28.0, scanner_transport="Ethernet/UDP"),
        "scanner")
    assert t["device"]["transport"] == "Ethernet/UDP"


def test_crowpanel_absent_is_informational():
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", crowpanel=None), "crowpanel")
    assert t["state"] == "absent"
    assert t["informational"] is True
    assert t["device"] is None
    assert "not connected" in t["detail"]


def test_crowpanel_reachable_but_no_ws_thin_session_is_still_up():
    """Regression: a CrowPanel that is powered on and reachable but not
    currently holding a `/ws-thin` flow (reconnect backoff, nobody has the
    viewer open) used to read identically to 'never existed' -- absent. It
    must now read up off ICMP alone -- and, matching the Bridge chip's rule
    (`informational = not up`), a reachable CrowPanel is a real 'up' fact, not
    a muted/informational one, exactly like a reachable Bridge."""
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", crowpanel=None,
        crowpanel_ping=PingResult(True, 4.0), crowpanel_ip="172.17.2.67"), "crowpanel")
    assert t["state"] == "up"
    assert t["informational"] is False
    assert t["addr"] == "172.17.2.67"
    assert t["device"] is None
    assert "reachable" in t["detail"] and "no /ws-thin session" in t["detail"]
    assert t["link"]["reachable"] is True
    assert t["link"]["connected"] is False


def test_crowpanel_truly_absent_is_neither_reachable_nor_connected():
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", crowpanel=None,
        crowpanel_ping=PingResult(False), crowpanel_ip="172.17.2.67"), "crowpanel")
    assert t["state"] == "absent"
    assert t["informational"] is True
    assert t["addr"] is None


def test_crowpanel_connected_reports_format_fps_and_credits():
    cp = {"connected": True, "count": 1, "format": "jpeg", "fps": 12.0,
          "width": 480, "height": 480, "dropped": 3,
          "tx_fps": 11.8, "tx_bytes_per_s": 40000, "credits": 2, "credits_max": 4,
          "proto": 2}
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", crowpanel=cp), "crowpanel")
    assert t["state"] == "up"
    assert t["informational"] is False
    assert t["link"]["connected"] is True
    assert t["link"]["tx_fps"] == 11.8
    assert t["link"]["credits"] == 2 and t["link"]["credits_max"] == 4
    assert t["device"]["format"] == "jpeg" and t["device"]["dropped"] == 3
    assert "jpeg" in t["detail"] and "credits 2/4" in t["detail"]


def test_stlink_absent_is_not_down():
    """Absence is normal on the Ethernet rig: its own neutral state, never a fault."""
    absent = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b"), "stlink")
    assert absent["state"] == "absent" and absent["informational"] is True
    assert absent["management"] is None
    present = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=True,
        bridge_ip="a", scanner_ip="b"), "stlink")
    assert present["state"] == "up"
    assert present["link"] == {"present": True}


def test_parse_rtt_ms():
    line = "64 bytes from 172.17.2.66: icmp_seq=1 ttl=64 time=1.23 ms"
    assert commstatus._parse_rtt_ms(line) == 1.2
    assert commstatus._parse_rtt_ms("no timing here") is None


def test_ping_host_reports_down_when_ping_missing(monkeypatch):
    monkeypatch.setattr(commstatus.shutil, "which", lambda _: None)
    res = asyncio.run(commstatus.ping_host("10.0.0.1"))
    assert res == PingResult(False)


def test_detect_stlink_false_when_lsusb_missing(monkeypatch):
    monkeypatch.setattr(commstatus.shutil, "which", lambda _: None)
    assert commstatus.detect_stlink() is False


def test_detect_crowpanel_prog_false_when_lsusb_missing(monkeypatch):
    monkeypatch.setattr(commstatus.shutil, "which", lambda _: None)
    assert commstatus.detect_crowpanel_prog() is False


def test_detect_crowpanel_prog_matches_ch340_vid(monkeypatch):
    monkeypatch.setattr(commstatus.shutil, "which", lambda name: "/usr/bin/lsusb")
    fake = SimpleNamespace(
        stdout="Bus 001 Device 025: ID 1a86:7522 QinHeng Electronics CH340 serial converter\n")
    monkeypatch.setattr(commstatus.subprocess, "run", lambda *a, **k: fake)
    assert commstatus.detect_crowpanel_prog() is True


def test_crowpanel_prog_target_shape():
    absent = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b"), "crowpanel_prog")
    assert absent["state"] == "absent" and absent["informational"] is True
    present = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="a", scanner_ip="b", crowpanel_prog_present=True), "crowpanel_prog")
    assert present["state"] == "up"
    assert present["link"] == {"present": True}


def _no_mdns(monkeypatch):
    """Simulate an unreachable/absent mDNS responder (no live network call) so
    `probe()` falls back to pinging its `bridge_ip`/`crowpanel_ip` argument
    verbatim -- the behaviour these older tests were written against, before
    the mDNS resolution step existed. Also stubs `detect_crowpanel_prog` (the
    other new USB scan `probe()` now runs) to a safe default so tests that
    don't care about it don't shell out to a real `lsusb`."""
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync", lambda hostname: None)
    monkeypatch.setattr(commstatus, "_bridge_resolve_cache", commstatus._BridgeResolveCache())
    monkeypatch.setattr(commstatus, "_crowpanel_resolve_cache", commstatus._BridgeResolveCache())
    monkeypatch.setattr(commstatus, "detect_crowpanel_prog", lambda: False)


def test_probe_orchestrates_without_raising(monkeypatch):
    async def fake_ping(ip, timeout_s=1.0):
        return PingResult(ip.endswith(".66"), 2.0)   # only the bridge answers
    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
    _no_mdns(monkeypatch)
    msg = asyncio.run(commstatus.probe("172.17.2.66", "172.17.2.58", scanner_fps=None))
    assert _by_id(msg, "bridge")["state"] == "up"
    assert _by_id(msg, "scanner")["state"] == "down"
    assert _by_id(msg, "stlink")["state"] == "absent"
    assert _by_id(msg, "crowpanel")["state"] == "absent"


def test_probe_survives_a_probe_that_raises(monkeypatch):
    async def boom(ip, timeout_s=1.0):
        raise RuntimeError("network gremlin")
    monkeypatch.setattr(commstatus, "ping_host", boom)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: (_ for _ in ()).throw(OSError()))
    _no_mdns(monkeypatch)
    msg = asyncio.run(commstatus.probe("a", "b"))          # must not raise
    assert _by_id(msg, "bridge")["state"] == "absent"      # never "down" (#193)
    assert _by_id(msg, "stlink")["state"] == "absent"


def test_probe_passes_bridge_health_and_crowpanel_through(monkeypatch):
    async def fake_ping(ip, timeout_s=1.0):
        return PingResult(True, 1.0)
    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
    _no_mdns(monkeypatch)
    health = {"wlan0": {"rssi_dbm": -40}}
    cp = {"connected": True, "format": "raw", "fps": 10.0}
    msg = asyncio.run(commstatus.probe("a", "b", bridge_health=health, crowpanel=cp))
    assert _by_id(msg, "bridge")["link"]["rssi_dbm"] == -40
    assert _by_id(msg, "crowpanel")["state"] == "up"


@pytest.mark.parametrize("label,expected", [
    ("Ethernet/UDP · 172.17.2.58:5000", "172.17.2.58"),
    ("Ethernet/UDP · 255.255.255.255:5000", commstatus.DEFAULT_SCANNER_IP),  # broadcast -> default
    ("Replay · web_123.bin", commstatus.DEFAULT_SCANNER_IP),                 # a replay isn't the scanner
    ("", commstatus.DEFAULT_SCANNER_IP),
])
def test_web_scanner_ip_from_source_label(label, expected):
    from roomscan import web
    state = SimpleNamespace(controller=SimpleNamespace(source_label=label))
    assert web._scanner_ip(state) == expected


@pytest.mark.parametrize("label,expected", [
    ("Ethernet/UDP · 172.17.2.58:5000", "Ethernet/UDP"),
    ("USB CDC · /dev/ttyACM0", "USB CDC"),
    ("Replay · web_123.bin", None),
    ("", None),
])
def test_web_scanner_transport_from_source_label(label, expected):
    from roomscan import web
    state = SimpleNamespace(controller=SimpleNamespace(source_label=label))
    assert web._scanner_transport(state) == expected


# --- bridge target resolution (#209 mDNS follow-up) -------------------------
#
# The bridge's literal IP moved off the network entirely (eth0 is on the
# scanner-side subnet; the host-facing address is wlan0 via DHCP), so the
# probe target must resolve `roomscan-bridge.local` first and only fall back
# to the configured IP when that fails. Never a live network call in these
# tests -- `_resolve_hostname_sync` is monkeypatched everywhere below.


def test_resolve_bridge_target_prefers_mdns_hostname(monkeypatch):
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: "172.17.2.78")
    cache = commstatus._BridgeResolveCache()
    result = asyncio.run(commstatus.resolve_bridge_target(
        "172.17.2.66", cache=cache, now=0.0))
    assert result["addr"] == "172.17.2.78"
    assert result["resolved_from"] == "mdns"
    assert result["hostname"] == commstatus.DEFAULT_BRIDGE_HOSTNAME


def test_resolve_bridge_target_falls_back_to_configured_ip_when_mdns_fails(monkeypatch):
    """No responder, mDNS down, or the hostname is simply unknown -- all look
    identical from here (`_resolve_hostname_sync` returns None), and every one
    of them must fall back to the configured IP, never raise or hang."""
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync", lambda hostname: None)
    cache = commstatus._BridgeResolveCache()
    result = asyncio.run(commstatus.resolve_bridge_target(
        "172.17.2.66", cache=cache, now=0.0))
    assert result["addr"] == "172.17.2.66"
    assert result["resolved_from"] == "fallback"


def test_resolve_bridge_target_falls_back_when_resolution_times_out(monkeypatch):
    """A hung resolver (mDNS multicast round-trip with nothing answering) must
    degrade to the fallback within `timeout_s`, not block forever."""
    def slow_resolve(hostname):
        import time as _time
        _time.sleep(10)
        return "172.17.2.78"

    monkeypatch.setattr(commstatus, "_resolve_hostname_sync", slow_resolve)
    cache = commstatus._BridgeResolveCache()
    result = asyncio.run(commstatus.resolve_bridge_target(
        "172.17.2.66", cache=cache, now=0.0, timeout_s=0.05))
    assert result["addr"] == "172.17.2.66"
    assert result["resolved_from"] == "fallback"


def test_resolve_bridge_target_caches_within_ttl_no_per_call_resolution(monkeypatch):
    """The whole point of the cache: repeated calls inside the TTL window must
    NOT re-invoke the resolver -- that is what keeps a 3s ICMP cadence from
    paying a multicast round-trip on every single ping."""
    calls = []
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: calls.append(hostname) or "172.17.2.78")
    cache = commstatus._BridgeResolveCache()

    async def hit_many_times():
        results = []
        for _ in range(5):
            results.append(await commstatus.resolve_bridge_target(
                "172.17.2.66", cache=cache, now=1.0, ttl_s=120.0))
        return results

    results = asyncio.run(hit_many_times())
    assert len(calls) == 1, f"resolver invoked {len(calls)} times for 5 calls inside the TTL"
    assert all(r["addr"] == "172.17.2.78" for r in results)


def test_resolve_bridge_target_re_resolves_after_ttl_expires(monkeypatch):
    """A DHCP-moved wlan0 address (or a recovering mDNS responder) must be
    picked up again once the cached entry ages out -- the cache must not pin
    a result forever."""
    calls = []
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: calls.append(hostname) or "172.17.2.78")
    cache = commstatus._BridgeResolveCache()

    asyncio.run(commstatus.resolve_bridge_target(
        "172.17.2.66", cache=cache, now=0.0, ttl_s=60.0))
    asyncio.run(commstatus.resolve_bridge_target(
        "172.17.2.66", cache=cache, now=61.0, ttl_s=60.0))
    assert len(calls) == 2, "cache entry must expire and re-resolve after the TTL"


def test_resolve_bridge_target_caches_the_fallback_too(monkeypatch):
    """A failed resolution is cached the same as a success -- otherwise a dead
    mDNS responder would pay a full resolution timeout on every single probe
    tick instead of once per TTL window."""
    calls = []
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: calls.append(hostname) or None)
    cache = commstatus._BridgeResolveCache()

    async def hit_twice():
        await commstatus.resolve_bridge_target("172.17.2.66", cache=cache, now=0.0, ttl_s=60.0)
        return await commstatus.resolve_bridge_target(
            "172.17.2.66", cache=cache, now=1.0, ttl_s=60.0)

    result = asyncio.run(hit_twice())
    assert len(calls) == 1
    assert result["addr"] == "172.17.2.66" and result["resolved_from"] == "fallback"


def test_probe_resolves_bridge_hostname_before_pinging(monkeypatch):
    """`probe()` must ping the RESOLVED address, not the raw configured
    default -- and report which one won in the bridge link facts."""
    pinged = []

    async def fake_ping(ip, timeout_s=1.0):
        pinged.append(ip)
        return PingResult(True, 1.0)

    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
    monkeypatch.setattr(commstatus, "detect_crowpanel_prog", lambda: False)
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: "172.17.2.78")
    monkeypatch.setattr(commstatus, "_bridge_resolve_cache", commstatus._BridgeResolveCache())
    monkeypatch.setattr(commstatus, "_crowpanel_resolve_cache", commstatus._BridgeResolveCache())

    msg = asyncio.run(commstatus.probe("172.17.2.66", "172.17.2.58"))

    assert "172.17.2.78" in pinged
    assert "172.17.2.66" not in pinged
    bridge = _by_id(msg, "bridge")
    assert bridge["addr"] == "172.17.2.78"
    assert bridge["link"]["resolved_from"] == "mdns"


def test_probe_regression_pinging_the_configured_default_directly(monkeypatch):
    """Regression proof (engineering-practices: prove a regression test by
    reintroducing the defect): this pins EXACTLY the pre-fix behaviour --
    `probe()` pinging `bridge_ip` verbatim, never resolving the hostname --
    the failure mode this issue exists to fix (the configured default,
    172.17.2.66, currently answers nothing at all). If `probe()` regresses to
    that behaviour, this test fails."""
    pinged = []

    async def fake_ping(ip, timeout_s=1.0):
        pinged.append(ip)
        return PingResult(False)

    def _pre_fix_probe(bridge_ip, scanner_ip):
        """The exact old code path, reintroduced here (not in commstatus.py)
        to prove the new test can see the defect."""
        return asyncio.run(fake_ping(bridge_ip))

    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    _pre_fix_probe("172.17.2.66", "172.17.2.58")
    assert pinged == ["172.17.2.66"], "sanity: the pre-fix behaviour pings the raw default"

    # Now the real, fixed `probe()`: must NOT reproduce that behaviour.
    pinged.clear()
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
    monkeypatch.setattr(commstatus, "detect_crowpanel_prog", lambda: False)
    monkeypatch.setattr(commstatus, "_resolve_hostname_sync",
                        lambda hostname: "172.17.2.78")
    monkeypatch.setattr(commstatus, "_bridge_resolve_cache", commstatus._BridgeResolveCache())
    monkeypatch.setattr(commstatus, "_crowpanel_resolve_cache", commstatus._BridgeResolveCache())
    asyncio.run(commstatus.probe("172.17.2.66", "172.17.2.58"))
    assert "172.17.2.66" not in pinged
    assert "172.17.2.78" in pinged


# --- scanner-behind-bridge honesty (#209/#193) -------------------------------


def test_scanner_addr_unaffected_when_not_behind_bridge():
    """Distinct bridge/scanner addresses (the normal Ethernet-tethered rig) ->
    no change at all."""
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="172.17.2.78", scanner_ip="172.17.2.58"), "scanner")
    assert t["addr"] == "172.17.2.58"
    assert "via_bridge" not in t["link"]


def test_scanner_via_bridge_uses_lease_ip_when_available():
    """The apparent scanner address equals the bridge's resolved address (the
    stream is arriving via the bridge's NAT hop) AND a lease is cached ->
    show the real behind-bridge address, not the NAT one."""
    health = {"eth0": {"scanner_lease": {"ip": "172.31.100.20"}}}
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        bridge_ip="172.17.2.78", scanner_ip="172.17.2.78", scanner_fps=27.8,
        bridge_health=health), "scanner")
    assert t["addr"] == "172.31.100.20"
    assert t["addr"] != "172.17.2.78"
    assert t["link"]["via_bridge"] is True
    assert t["link"]["bridge_addr"] == "172.17.2.78"
    assert "via bridge (lease 172.31.100.20)" in t["detail"]


def test_scanner_via_bridge_without_lease_never_shows_the_nat_addr_as_scanners():
    """No health cached yet (or no lease in it) -> still must not present the
    NAT address as the scanner's own; `addr` becomes None (nothing honest to
    show) and the NAT address is folded into `detail`/`link` as a note
    instead. Regression: an earlier draft left `addr` as the raw NAT address
    here, which is exactly the defect this test pins."""
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        bridge_ip="172.17.2.78", scanner_ip="172.17.2.78", scanner_fps=27.8,
        bridge_health=None), "scanner")
    assert t["addr"] is None
    assert t["addr"] != "172.17.2.78"
    assert t["link"]["via_bridge"] is True
    assert t["link"]["bridge_addr"] == "172.17.2.78"
    assert "via bridge (172.17.2.78)" in t["detail"]


def test_scanner_via_bridge_only_when_bridge_actually_reachable():
    """The equality is only meaningful evidence of NAT while the bridge chip
    itself is up -- if the bridge ping failed, an address match is
    coincidence (e.g. both defaulted the same way), not proof of NAT."""
    t = _by_id(build_comm_message(
        bridge=PingResult(False), scanner=PingResult(True), stlink_present=False,
        bridge_ip="172.17.2.78", scanner_ip="172.17.2.78"), "scanner")
    assert t["addr"] == "172.17.2.78"
    assert "via_bridge" not in t["link"]


def test_scanner_via_bridge_lease_as_plain_string_also_works():
    """Defensive: some call sites may hand `scanner_lease` as a bare IP
    string rather than `{"ip": ...}` -- both must be read the same way."""
    health = {"eth0": {"scanner_lease": "172.31.100.20"}}
    t = _by_id(build_comm_message(
        bridge=PingResult(True), scanner=PingResult(True), stlink_present=False,
        bridge_ip="172.17.2.78", scanner_ip="172.17.2.78", bridge_health=health), "scanner")
    assert t["addr"] == "172.31.100.20"
