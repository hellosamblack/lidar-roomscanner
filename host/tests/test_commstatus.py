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
    assert [t["id"] for t in msg["targets"]] == ["bridge", "scanner", "crowpanel", "stlink"]


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


def test_probe_orchestrates_without_raising(monkeypatch):
    async def fake_ping(ip, timeout_s=1.0):
        return PingResult(ip.endswith(".66"), 2.0)   # only the bridge answers
    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
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
    msg = asyncio.run(commstatus.probe("a", "b"))          # must not raise
    assert _by_id(msg, "bridge")["state"] == "absent"      # never "down" (#193)
    assert _by_id(msg, "stlink")["state"] == "absent"


def test_probe_passes_bridge_health_and_crowpanel_through(monkeypatch):
    async def fake_ping(ip, timeout_s=1.0):
        return PingResult(True, 1.0)
    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
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
