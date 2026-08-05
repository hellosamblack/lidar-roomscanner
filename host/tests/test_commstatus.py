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
        filehub=PingResult(True, 1.2), scanner=PingResult(True, 0.8),
        stlink_present=False, filehub_ip="172.17.2.57", scanner_ip="172.17.2.58",
        scanner_fps=27.8)
    assert msg["type"] == "comm"
    assert [t["id"] for t in msg["targets"]] == ["filehub", "scanner", "stlink"]


def test_filehub_up_and_down():
    up = _by_id(build_comm_message(
        filehub=PingResult(True, 3.0), scanner=PingResult(False), stlink_present=False,
        filehub_ip="a", scanner_ip="b"), "filehub")
    assert up["state"] == "up" and "3" in up["detail"] and up["addr"] == "a"
    down = _by_id(build_comm_message(
        filehub=PingResult(False), scanner=PingResult(False), stlink_present=False,
        filehub_ip="a", scanner_ip="b"), "filehub")
    assert down["state"] == "down" and down["detail"] == "no reply"


def test_scanner_streaming_vs_reachable_vs_down():
    # Reachable AND frames flowing -> streaming with the rate.
    streaming = _by_id(build_comm_message(
        filehub=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        filehub_ip="a", scanner_ip="b", scanner_fps=30.4), "scanner")
    assert streaming["state"] == "up" and "streaming 30.4 fps" == streaming["detail"]

    # Reachable but silent (idle laser / no frames) -> distinct detail, still up.
    silent = _by_id(build_comm_message(
        filehub=PingResult(True), scanner=PingResult(True, 0.5), stlink_present=False,
        filehub_ip="a", scanner_ip="b", scanner_fps=0.0), "scanner")
    assert silent["state"] == "up" and "no frames" in silent["detail"]

    # Unreachable -> down.
    down = _by_id(build_comm_message(
        filehub=PingResult(True), scanner=PingResult(False), stlink_present=False,
        filehub_ip="a", scanner_ip="b", scanner_fps=30.0), "scanner")
    assert down["state"] == "down" and down["detail"] == "no reply"


def test_stlink_absent_is_not_down():
    """Absence is normal on the Ethernet rig: its own neutral state, never a fault."""
    absent = _by_id(build_comm_message(
        filehub=PingResult(True), scanner=PingResult(True), stlink_present=False,
        filehub_ip="a", scanner_ip="b"), "stlink")
    assert absent["state"] == "absent" and absent["informational"] is True
    present = _by_id(build_comm_message(
        filehub=PingResult(True), scanner=PingResult(True), stlink_present=True,
        filehub_ip="a", scanner_ip="b"), "stlink")
    assert present["state"] == "up"


def test_parse_rtt_ms():
    line = "64 bytes from 172.17.2.57: icmp_seq=1 ttl=64 time=1.23 ms"
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
        return PingResult(ip.endswith(".57"), 2.0)   # only the filehub answers
    monkeypatch.setattr(commstatus, "ping_host", fake_ping)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: False)
    msg = asyncio.run(commstatus.probe("172.17.2.57", "172.17.2.58", scanner_fps=None))
    assert _by_id(msg, "filehub")["state"] == "up"
    assert _by_id(msg, "scanner")["state"] == "down"
    assert _by_id(msg, "stlink")["state"] == "absent"


def test_probe_survives_a_probe_that_raises(monkeypatch):
    async def boom(ip, timeout_s=1.0):
        raise RuntimeError("network gremlin")
    monkeypatch.setattr(commstatus, "ping_host", boom)
    monkeypatch.setattr(commstatus, "detect_stlink", lambda: (_ for _ in ()).throw(OSError()))
    msg = asyncio.run(commstatus.probe("a", "b"))          # must not raise
    assert _by_id(msg, "filehub")["state"] == "down"
    assert _by_id(msg, "stlink")["state"] == "absent"


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
