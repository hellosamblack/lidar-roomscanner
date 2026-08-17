"""Tests for the host side of the Pi bridge: `pi_bridge.py` and its MCP tools.

The bridge exists because the FileHub was unobservable, so the property these
tests care about most is that nothing here reports success it did not verify --
and that an unreachable Pi is an *answer* (`{"ok": False, "error": ...}`), not
an exception that kills the agent's turn.
"""
from __future__ import annotations

import asyncio

import pytest

from roomscan.mcp_server.server import build
from tools import pi_bridge as pb


# ---------------------------------------------------------------------------
# Target resolution
# ---------------------------------------------------------------------------

def test_explicit_host_beats_everything(monkeypatch):
    monkeypatch.setenv(pb.ENV_HOST, "10.0.0.9")
    got = pb.resolve_host("bridge.example")
    assert got == {"host": "bridge.example", "via": "argument"}


def test_env_beats_mdns(monkeypatch):
    monkeypatch.setenv(pb.ENV_HOST, "10.0.0.9")
    assert pb.resolve_host() == {"host": "10.0.0.9", "via": "env"}


def test_mdns_lookup_uses_the_bridge_service_name(monkeypatch):
    monkeypatch.delenv(pb.ENV_HOST, raising=False)
    seen = {}

    class FakeInfo:
        port = 22

        def parsed_addresses(self):
            return ["172.17.2.44"]

    class FakeZc:
        def get_service_info(self, type_, name, timeout=0):
            seen["type"], seen["name"] = type_, name
            return FakeInfo()

        def close(self):
            seen["closed"] = True

    got = pb.resolve_host(zeroconf_factory=FakeZc)
    assert got["host"] == "172.17.2.44" and got["via"] == "mdns"
    assert seen["type"] == "_roomscan-bridge._tcp.local."
    # Distinct from the scanner's own `roomscanner._roomscan._udp` instance:
    # the Pi publishes BOTH, and confusing them would ssh to the STM32.
    assert seen["name"] == "roomscan-bridge._roomscan-bridge._tcp.local."
    assert seen["closed"], "the zeroconf listener must be closed"


def test_no_responder_falls_back_and_says_so(monkeypatch):
    monkeypatch.delenv(pb.ENV_HOST, raising=False)

    class FakeZc:
        def get_service_info(self, *a, **k):
            return None

        def close(self):
            pass

    got = pb.resolve_host(zeroconf_factory=FakeZc)
    assert got == {"host": pb.FALLBACK_HOST, "via": "fallback"}


def test_a_broken_zeroconf_stack_does_not_raise(monkeypatch):
    monkeypatch.delenv(pb.ENV_HOST, raising=False)

    def boom():
        raise RuntimeError("no network")

    got = pb.resolve_host(zeroconf_factory=boom)
    assert got["via"] == "fallback" and "no network" in got["mdns_error"]


# ---------------------------------------------------------------------------
# Input guards -- these must not reach the network at all
# ---------------------------------------------------------------------------

def test_logs_refuses_a_shell_injecting_unit_name():
    got = pb.logs("foo; rm -rf /")
    assert got["ok"] is False and "unsafe" in got["error"]


def test_wifi_update_rejects_an_unusable_passphrase_before_applying_it():
    # Applying an 8-char-invalid PSK remotely takes away the link used to fix it.
    got = pb.wifi_update("SSID", "short")
    assert got["ok"] is False and "8-63" in got["error"]


def test_wifi_update_rejects_an_unknown_profile():
    got = pb.wifi_update("SSID", "a-valid-passphrase", profile="nope")
    assert got["ok"] is False and "profile" in got["error"]


def test_tee_fetch_refuses_a_path_traversing_name():
    got = pb.tee_fetch("../../etc/shadow")
    assert got["ok"] is False and "unsafe" in got["error"]


def test_update_reports_a_missing_secrets_file_rather_than_raising(tmp_path):
    got = pb.update(secrets=tmp_path / "nope.yaml")
    assert got["ok"] is False and "not found" in got["error"]


# ---------------------------------------------------------------------------
# Truth-telling: the warnings a human acts on
# ---------------------------------------------------------------------------

HEALTHY = {
    "wlan0": {"ssid": "Surly_Office", "rssi_dbm": -48, "power_save": "off",
              "connected": True},
    "eth0": {"carrier": True, "scanner_lease": {"ip": "172.31.100.20"}},
    "tee": {"active": True},
    "units": {"dnsmasq": "active", "roomscan-tee": "active"},
    "ntp_synced": True,
    "throttled": "0x0",
}


def test_a_healthy_bridge_warns_about_nothing():
    assert pb._status_warnings(HEALTHY) == []


def test_power_save_on_is_called_out():
    # brcmfmac power save is a known source of burst latency and loss, and it
    # is exactly the kind of setting that silently reverts.
    d = {**HEALTHY, "wlan0": {**HEALTHY["wlan0"], "power_save": "on"}}
    assert any("power_save" in w for w in pb._status_warnings(d))


def test_a_missing_scanner_lease_points_at_the_firmware_fallback():
    d = {**HEALTHY, "eth0": {"carrier": True, "scanner_lease": None}}
    warns = pb._status_warnings(d)
    assert any("172.31.253.1" in w for w in warns)


def test_no_carrier_is_reported_as_the_cable_not_the_lease():
    d = {**HEALTHY, "eth0": {"carrier": False, "scanner_lease": None}}
    warns = pb._status_warnings(d)
    assert any("carrier" in w for w in warns)
    assert not any("172.31.253.1" in w for w in warns), \
        "no cable is not a fallback-mode diagnosis"


def test_a_stopped_tee_is_reported_because_loss_becomes_unrecoverable():
    d = {**HEALTHY, "tee": {"active": False}}
    assert any("tee" in w for w in pb._status_warnings(d))


def test_an_unsynced_clock_is_reported_because_the_pi_3_has_no_rtc():
    d = {**HEALTHY, "ntp_synced": False}
    assert any("RTC" in w for w in pb._status_warnings(d))


def test_a_dead_unit_is_named():
    d = {**HEALTHY, "units": {**HEALTHY["units"], "dnsmasq": "failed"}}
    assert any("dnsmasq" in w and "failed" in w for w in pb._status_warnings(d))


def test_undervoltage_throttling_is_surfaced():
    # The Pi 3 spikes past 700 mA on Wi-Fi + CPU; an undersized rail on the rig
    # presents as flaky wireless, not as an obvious power fault.
    d = {**HEALTHY, "throttled": "0x50005"}
    assert any("throttl" in w.lower() for w in pb._status_warnings(d))


# ---------------------------------------------------------------------------
# Unreachable Pi
# ---------------------------------------------------------------------------

def _unreachable(monkeypatch):
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": False, "rc": 255, "stdout": "", "stderr": "ssh: connect: timed out",
        "error": "ssh: connect: timed out", "latency_ms": 12.0,
        "target": {"host": "192.0.2.1", "via": "env"}})


def test_status_against_an_unreachable_pi_is_an_answer_not_an_exception(monkeypatch):
    _unreachable(monkeypatch)
    got = pb.status()
    assert got["ok"] is False
    assert "timed out" in got["error"]
    assert got["target"]["host"] == "192.0.2.1"


def test_status_rejects_non_json_from_the_remote_script(monkeypatch):
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": True, "rc": 0, "stdout": "sudo: a password is required\n", "stderr": "",
        "latency_ms": 5.0, "target": {"host": "h", "via": "env"}})
    got = pb.status()
    assert got["ok"] is False and "not JSON" in got["error"]
    assert "password" in got["raw"], "the raw output is kept for diagnosis"


def test_status_annotates_a_healthy_reading_with_latency_and_warnings(monkeypatch):
    import json
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": True, "rc": 0, "stdout": json.dumps(HEALTHY), "stderr": "",
        "latency_ms": 7.5, "target": {"host": "h", "via": "mdns"}})
    got = pb.status()
    assert got["ok"] is True and got["ssh_latency_ms"] == 7.5
    assert got["warnings"] == []


def test_reboot_treats_a_dropped_channel_as_success(monkeypatch):
    # sshd dies with the machine; a clean exit is the unusual case.
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": False, "rc": 255, "error": "Connection closed by remote host",
        "target": {"host": "h", "via": "env"}})
    assert pb.reboot()["ok"] is True


def test_tee_list_parses_the_ring_newest_first(monkeypatch):
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": True, "rc": 0, "stderr": "", "latency_ms": 3.0,
        "target": {"host": "h", "via": "env"},
        "stdout": "104857600 1700000100 ring.pcap01\n"
                  "104857600 1700000300 ring.pcap02\n"
                  "  52428800 1700000500 ring.pcap\n"})
    got = pb.tee_list()
    assert got["ok"] and got["count"] == 3
    assert [f["name"] for f in got["files"]] == ["ring.pcap", "ring.pcap02", "ring.pcap01"]
    assert got["total_bytes"] == 104857600 * 2 + 52428800


def test_update_treats_the_deferred_restart_exit_code_as_benign(monkeypatch, tmp_path):
    """install.sh exits 75 when it refused to restart dnsmasq/nftables because a
    capture was streaming. That is the designed behaviour, but the new config is
    on disk and not yet in force -- so it must be reported, not hidden."""
    import yaml
    sec = tmp_path / "s.yaml"
    sec.write_text(yaml.safe_dump({
        "hostname": "h", "username": "roomscan", "password": "pw",
        "wifi_country": "US",
        "wifi": {"home": {"ssid": "S", "passphrase": "a-real-passphrase"}}}))
    sec.chmod(0o600)

    monkeypatch.setattr(pb, "_scp", lambda *a, **k: {"ok": True, "target": {}})
    monkeypatch.setattr(pb, "ssh_run", lambda *a, **k: {
        "ok": False, "rc": 75, "stdout": "skipped restart: stream is live\n",
        "stderr": "", "target": {"host": "h", "via": "env"}})
    monkeypatch.setattr(pb, "status", lambda **k: {
        "ok": True, "units": {"dnsmasq": "active"}})
    monkeypatch.setattr(pb, "ensure_ssh_key_stub", None, raising=False)

    import importlib.util
    import sys
    from pathlib import Path
    spec = importlib.util.spec_from_file_location(
        "pi_bridge_build_image_u",
        Path(__file__).resolve().parents[2] / "pi-bridge" / "build_image.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    monkeypatch.setattr(mod, "ensure_ssh_key", lambda **k: "ssh-ed25519 AAAA test")
    monkeypatch.setitem(sys.modules, "build_image", mod)

    got = pb.update(secrets=sec)
    assert got["restarts_deferred"] is True
    assert got["install_ok"] is True
    assert "not restarted" in got["restarts_deferred_note"].lower() or \
        "NOT restarted" in got["restarts_deferred_note"]


# ---------------------------------------------------------------------------
# MCP surface
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def tools():
    return {t.name: t for t in asyncio.run(build().list_tools())}


BRIDGE_TOOLS = ("bridge_status", "bridge_logs", "bridge_wifi_update", "bridge_update",
                "bridge_tee_list", "bridge_tee_fetch", "bridge_reboot",
                "bridge_image_build", "bridge_pcap_convert")


def test_every_bridge_tool_is_registered(tools):
    missing = [t for t in BRIDGE_TOOLS if t not in tools]
    assert not missing, missing


def test_bridge_tools_take_an_optional_host_override(tools):
    for name in ("bridge_status", "bridge_logs", "bridge_tee_list", "bridge_reboot"):
        props = tools[name].input_schema["properties"]
        assert "host" in props, name
        assert name not in (tools[name].input_schema.get("required") or [])


def test_bridge_tools_return_structured_failure_for_an_unreachable_host(monkeypatch):
    """An MCP tool that raises costs the agent its turn; one that returns
    `{"ok": False}` lets it read the error and move on."""
    from roomscan.mcp_server import tools_bridge  # noqa: F401
    _unreachable(monkeypatch)
    for fn in (pb.status, pb.tee_list):
        got = fn()
        assert got["ok"] is False
        assert isinstance(got.get("error"), str) and got["error"]


def test_bridge_pcap_convert_reports_a_missing_file(monkeypatch):
    from roomscan.mcp_server.tools_bridge import bridge_pcap_convert
    fn = getattr(bridge_pcap_convert, "fn", bridge_pcap_convert)
    got = fn(path="captures/definitely-not-here.pcap")
    assert got["ok"] is False and "no such pcap" in got["error"]


def test_bridge_image_build_reports_a_missing_secrets_file(monkeypatch, tmp_path):
    from roomscan.mcp_server.tools_bridge import bridge_image_build
    fn = getattr(bridge_image_build, "fn", bridge_image_build)
    got = fn(secrets=str(tmp_path / "nope.yaml"))
    assert got["ok"] is False and "not found" in got["error"]
