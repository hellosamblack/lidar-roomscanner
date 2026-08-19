#!/usr/bin/env python3
"""Administer the Raspberry Pi bridge node remotely over ssh.

The Pi 3 bridge replaces the RavPower FileHub as the scanner's wireless uplink
(issue #191). The FileHub's defining problem was not that it broke -- it was
that nothing about it was observable when it did, so a capture that lost 2.3-9.4
percent of its frames looked identical to a clean one. Every function here
therefore reports what it actually found, never what it asked for: `wifi_update`
returns the resulting association state rather than "applied", `status` returns
the radio's real `power_save` as the driver reports it, and an unreachable Pi
comes back as a structured `{"ok": False, ...}` rather than an exception.

The remote side is `roomscan-bridge-status`, a script installed on the Pi that
emits JSON; this module is a transport and a parser, not a second source of
truth about the bridge's state.

Usage:

    ./host/tools/pi_bridge.py status
    ./host/tools/pi_bridge.py status --json
    ./host/tools/pi_bridge.py logs roomscan-bridge-reconcile --lines 100
    ./host/tools/pi_bridge.py tee-fetch ring.pcap03 --convert
"""
from __future__ import annotations

import argparse
import json
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "host" / "src"))

#: How the MCP tools and the CLI find the Pi, in order:
#:   1. an explicit `host=` argument
#:   2. $ROOMSCAN_BRIDGE_HOST
#:   3. mDNS `_roomscan-bridge._tcp` (published by the Pi on wlan0 only)
#:   4. the mDNS hostname, as a last resort
ENV_HOST = "ROOMSCAN_BRIDGE_HOST"
ENV_USER = "ROOMSCAN_BRIDGE_USER"
ENV_KEY = "ROOMSCAN_BRIDGE_KEY"
ENV_SECRETS = "ROOMSCAN_BRIDGE_SECRETS"

MDNS_TYPE = "_roomscan-bridge._tcp.local."
MDNS_NAME = "roomscan-bridge._roomscan-bridge._tcp.local."
FALLBACK_HOST = "roomscan-bridge.local"
DEFAULT_USER = "roomscan"
DEFAULT_KEY = Path("~/.ssh/roomscan-bridge").expanduser()
DEFAULT_SECRETS = Path("~/.config/roomscan/pi-bridge-secrets.yaml").expanduser()

REMOTE_ROOT = "/opt/roomscan-bridge"
TEE_DIR = "/var/lib/roomscan-bridge/tee"

#: Units the bridge needs up. `bridge_status` and `bridge_update` both report
#: on exactly this list so the two never drift into disagreeing.
REQUIRED_UNITS = ("dnsmasq", "avahi-daemon", "nftables", "roomscan-tee",
                  "roomscan-bridge-reconcile.timer", "roomscan-bridge-healthlog.timer")


# ---------------------------------------------------------------------------
# Target resolution + ssh transport
# ---------------------------------------------------------------------------

def resolve_host(host: str = "", *, mdns_timeout_ms: float = 1500,
                 zeroconf_factory=None) -> dict:
    """Work out which address to ssh to, and say how we got it.

    Pure apart from the mDNS query. The `via` field matters during bring-up:
    "env" and "mdns" mean very different things when the Pi is unreachable --
    the first says nothing about whether it is on the network, the second says
    it announced itself and then refused the connection.
    """
    if host:
        return {"host": host, "via": "argument"}
    env = os.environ.get(ENV_HOST, "").strip()
    if env:
        return {"host": env, "via": "env"}

    try:
        if zeroconf_factory is None:
            from zeroconf import Zeroconf as zeroconf_factory  # noqa: N813
        zc = zeroconf_factory()
        try:
            info = zc.get_service_info(MDNS_TYPE, MDNS_NAME, timeout=mdns_timeout_ms)
            if info:
                addrs = info.parsed_addresses()
                if addrs:
                    return {"host": addrs[0], "via": "mdns", "port": info.port}
        finally:
            zc.close()
    except Exception as e:  # zeroconf missing, no network, no responder
        return {"host": FALLBACK_HOST, "via": "fallback", "mdns_error": str(e)}
    return {"host": FALLBACK_HOST, "via": "fallback"}


def ssh_run(argv: list[str], *, host: str = "", user: str = "", key: Path | None = None,
            timeout: float = 20.0, input_bytes: bytes | None = None) -> dict:
    """Run a command on the Pi. Never raises; returns a structured result.

    `BatchMode=yes` is deliberate: without it ssh will sit on a password prompt
    forever when the baked key is wrong, and an MCP tool that hangs is worse
    than one that fails.
    """
    target = resolve_host(host)
    user = user or os.environ.get(ENV_USER) or DEFAULT_USER
    key = Path(key) if key else Path(os.environ.get(ENV_KEY, DEFAULT_KEY)).expanduser()

    cmd = ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
           "-o", f"ConnectTimeout={int(max(2, min(timeout, 15)))}",
           "-o", "LogLevel=ERROR"]
    if key.is_file():
        cmd += ["-i", str(key), "-o", "IdentitiesOnly=yes"]
    # ssh joins its trailing arguments with spaces and hands the result to the
    # REMOTE LOGIN SHELL as one string -- it does not preserve argv boundaries.
    # Passing `argv` through unquoted therefore re-parses it remotely, and
    # `["bash", "-lc", "sudo mkdir -p /opt/x && ..."]` arrived as
    # `bash -lc sudo mkdir -p /opt/x && ...`: bash took `sudo` alone as its
    # command string, bare `sudo` printed its usage and exited 1, and the &&
    # chain short-circuited. bridge_update() could not extract a payload at
    # all, while reporting the sudo usage text as the install error (#191).
    # shlex.join quotes each element so the remote shell reconstructs exactly
    # the argv intended here.
    cmd += [f"{user}@{target['host']}", "--", shlex.join(argv)]

    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=timeout, input=input_bytes)
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"ssh to {target['host']} timed out after {timeout}s",
                "target": target, "latency_ms": round((time.monotonic() - t0) * 1000, 1)}
    except FileNotFoundError:
        return {"ok": False, "error": "ssh client not found on this host", "target": target}

    latency = round((time.monotonic() - t0) * 1000, 1)
    out = p.stdout.decode("utf-8", "replace")
    err = p.stderr.decode("utf-8", "replace").strip()
    res = {"ok": p.returncode == 0, "rc": p.returncode, "stdout": out, "stderr": err,
           "target": target, "latency_ms": latency}
    if p.returncode != 0 and not err:
        res["error"] = f"remote command failed with rc={p.returncode}"
    elif p.returncode != 0:
        res["error"] = err
    if key.is_file():
        res["key"] = str(key)
    else:
        res["key"] = None
        res.setdefault("hint", f"no ssh key at {key}; the builder generates it")
    return res


def _scp(remote_path: str, local_path: Path, *, host: str = "", user: str = "",
         key: Path | None = None, timeout: float = 300.0, push: bool = False) -> dict:
    target = resolve_host(host)
    user = user or os.environ.get(ENV_USER) or DEFAULT_USER
    key = Path(key) if key else Path(os.environ.get(ENV_KEY, DEFAULT_KEY)).expanduser()
    cmd = ["scp", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "LogLevel=ERROR"]
    if key.is_file():
        cmd += ["-i", str(key), "-o", "IdentitiesOnly=yes"]
    remote = f"{user}@{target['host']}:{remote_path}"
    cmd += [str(local_path), remote] if push else [remote, str(local_path)]

    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, capture_output=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"ok": False, "error": f"scp timed out after {timeout}s", "target": target}
    latency = round((time.monotonic() - t0) * 1000, 1)
    if p.returncode != 0:
        return {"ok": False, "error": p.stderr.decode("utf-8", "replace").strip(),
                "target": target, "latency_ms": latency}
    return {"ok": True, "target": target, "latency_ms": latency,
            "local": str(local_path), "remote": remote_path}


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------

def status(host: str = "", *, timeout: float = 20.0) -> dict:
    """Full bridge health, straight from the Pi's own status script.

    The remote script is the single source of truth (it is also what a human
    runs over a serial console when the network is the thing that is broken);
    this only adds the ssh hop's own latency, which is itself a Wi-Fi health
    signal worth having next to the RSSI.
    """
    r = ssh_run(["/usr/local/sbin/roomscan-bridge-status"], host=host, timeout=timeout)
    if not r["ok"]:
        return {"ok": False, "error": r.get("error", "ssh failed"),
                "target": r.get("target"), "ssh_latency_ms": r.get("latency_ms")}
    try:
        data = json.loads(r["stdout"])
    except json.JSONDecodeError as e:
        return {"ok": False, "error": f"remote status was not JSON: {e}",
                "raw": r["stdout"][:2000], "target": r["target"]}
    data["ok"] = True
    data["ssh_latency_ms"] = r["latency_ms"]
    data["target"] = r["target"]
    data["warnings"] = _status_warnings(data)
    return data


def _status_warnings(d: dict) -> list[str]:
    """Turn the raw readings into the few sentences a human actually acts on.

    Pure. Kept separate from the reading so the warnings can be tested against
    canned status JSON without a Pi.
    """
    warn: list[str] = []
    wlan = d.get("wlan0") or {}
    if wlan.get("power_save") not in (None, "off"):
        warn.append(f"wlan0 power_save is {wlan.get('power_save')!r}, not 'off' -- "
                    f"brcmfmac power save adds burst latency and drops frames")
    rssi = wlan.get("rssi_dbm")
    if isinstance(rssi, (int, float)) and rssi < -70:
        warn.append(f"weak signal ({rssi} dBm); expect loss on the Wi-Fi hop")
    eth = d.get("eth0") or {}
    if not eth.get("carrier"):
        warn.append("eth0 has no carrier -- the scanner is not plugged in")
    elif not eth.get("scanner_lease"):
        warn.append("no DHCP lease for the scanner; it may have fallen back to its "
                    "self-assigned 172.31.253.1 server mode (reconcile handles this)")
    tee = d.get("tee") or {}
    if tee.get("readable") is False:
        warn.append("could not read the pcap ring directory (it is owned by the "
                    "tcpdump user) -- its fill level is UNKNOWN here, not zero")
    if tee.get("active") is False:
        warn.append("the pcap tee is not running -- frames lost over Wi-Fi will not "
                    "be recoverable for captures taken now")
    if d.get("ntp_synced") is False:
        warn.append("clock not NTP-synced; the Pi 3 has no RTC, so tee and journal "
                    "timestamps are meaningless until it syncs")
    restarts = d.get("unit_restarts") or {}
    for unit, state in (d.get("units") or {}).items():
        n = restarts.get(unit)
        if state not in ("active", "activating", None):
            warn.append(f"unit {unit} is {state}")
        elif isinstance(n, int) and n > 0:
            # `activating` is what a crash-looping unit looks like when it is
            # sampled mid-backoff, and it is on the pass list above because it
            # is also what a healthy unit looks like while starting. The
            # restart counter separates them: roomscan-tee sat at `activating`
            # on the first real Pi while failing every 5 seconds since boot,
            # and this check is the reason that now surfaces (issue #191).
            warn.append(f"unit {unit} is {state} but has restarted {n} time(s) -- "
                        f"it is crash-looping, not starting up; "
                        f"`bridge_logs {unit}` for why")

    nft = d.get("nft") or {}
    if nft.get("readable") is False:
        warn.append("could not read the nftables ruleset (needs root on the Pi) -- "
                    "the DNAT rule and its counters are UNKNOWN here, not absent")
    if wlan.get("connected") is None:
        warn.append("could not determine wlan0 association (iw missing from PATH?) -- "
                    "the wlan0 readings below are unknown, not negative")
    thr = d.get("throttled")
    if thr and thr not in ("0x0", "throttled=0x0"):
        warn.append(f"firmware reports throttling ({thr}) -- check the 5 V supply")
    return warn


def logs(unit: str = "roomscan-bridge-reconcile", lines: int = 200,
         host: str = "", *, timeout: float = 30.0) -> dict:
    """Tail one unit's journal on the Pi.

    Also accepts the syslog tag `roomscan-bridge`, which is where the reconcile
    and install scripts log; systemd will happily match it as an identifier.
    """
    if not unit or any(c in unit for c in " ;|&$`\n"):
        return {"ok": False, "error": f"refusing unsafe unit name {unit!r}"}
    lines = max(1, min(int(lines), 5000))
    argv = ["journalctl", "--no-pager", "-n", str(lines)]
    argv += ["-t", unit] if not _looks_like_unit(unit) else ["-u", unit]
    r = ssh_run(argv, host=host, timeout=timeout)
    if not r["ok"]:
        return {"ok": False, "error": r.get("error", "ssh failed"), "unit": unit,
                "target": r.get("target")}
    text = r["stdout"]
    return {"ok": True, "unit": unit, "lines_requested": lines,
            "lines_returned": text.count("\n"), "text": text, "target": r["target"]}


def _looks_like_unit(name: str) -> bool:
    return name.endswith((".service", ".timer", ".socket", ".target", ".mount")) or \
        name in REQUIRED_UNITS


def wifi_update(ssid: str, psk: str, profile: str = "home", host: str = "",
                *, settle_s: float = 8.0, timeout: float = 60.0) -> dict:
    """Change one Wi-Fi profile's credentials and report what the radio then did.

    This is the one operation that can strand the Pi: a wrong PSK applied
    remotely takes away the very link used to fix it. Two things make that
    recoverable -- the change is applied to a *named profile* (the other profile
    still autoconnects), and the SD card's FAT partition accepts a
    `wifi-override.nmconnection` that `install.sh` re-applies on every boot, so
    the fix can be made from any laptop with a card reader.

    Returns the association state read back after `settle_s`, not the nmcli exit
    code. "It said OK" is not evidence the radio associated.
    """
    if len(psk) < 8 or len(psk) > 63:
        return {"ok": False, "error": f"WPA-PSK passphrase must be 8-63 chars, got {len(psk)}"}
    if profile not in ("home", "travel"):
        return {"ok": False, "error": f"unknown profile {profile!r}; use 'home' or 'travel'"}
    conn = f"roomscan-wifi-{profile}"

    script = (
        f"sudo nmcli connection modify {shlex.quote(conn)} "
        f"wifi.ssid {shlex.quote(ssid)} wifi-sec.psk {shlex.quote(psk)} "
        f"wifi-sec.key-mgmt wpa-psk && "
        f"sudo nmcli connection up {shlex.quote(conn)} || true"
    )
    r = ssh_run(["bash", "-lc", script], host=host, timeout=timeout)
    applied = r["ok"]
    time.sleep(max(0.0, settle_s))
    after = status(host=host)
    wlan = (after.get("wlan0") or {}) if after.get("ok") else {}
    associated = bool(wlan.get("connected")) and wlan.get("ssid") == ssid
    return {
        "ok": bool(after.get("ok")) and associated,
        "profile": profile,
        "requested_ssid": ssid,
        "nmcli_applied": applied,
        "nmcli_error": None if applied else r.get("error"),
        "associated": associated,
        "actual_ssid": wlan.get("ssid"),
        "rssi_dbm": wlan.get("rssi_dbm"),
        "ipv4": wlan.get("ipv4"),
        "note": None if associated else
                "not associated with the requested SSID; if the Pi is now "
                "unreachable, put a wifi-override.nmconnection on the card's FAT "
                "partition from any laptop -- install.sh applies it every boot",
    }


def update(host: str = "", *, secrets: Path | None = None, timeout: float = 300.0) -> dict:
    """Re-render the payload from this checkout, push it, and rerun install.sh.

    This is how a payload change ships without reflashing a card. The report is
    the unit states *afterwards*, read back from the Pi, because "install.sh
    exited 0" and "the bridge is working" are different claims.
    """
    secrets = Path(secrets) if secrets else \
        Path(os.environ.get(ENV_SECRETS, DEFAULT_SECRETS)).expanduser()
    if not secrets.is_file():
        return {"ok": False, "error": f"secrets file not found: {secrets} "
                                      f"(set ${ENV_SECRETS} or pass --secrets)"}

    sys.path.insert(0, str(REPO / "pi-bridge"))
    try:
        import build_image as bi  # noqa: E402
    except Exception as e:
        return {"ok": False, "error": f"cannot import pi-bridge/build_image.py: {e}"}

    try:
        sec = bi.load_secrets(secrets)
        pubkey = bi.ensure_ssh_key(generate=False)
        tokens = bi.secret_tokens(sec, pubkey)
        stage = bi.HERE / "build" / "payload"
        render = bi.render_payload(bi.PAYLOAD_SRC, stage, tokens)
        # Same extras the image build stages -- notably node.env, which the Pi's
        # reconcile and status scripts read for the scanner's address. Omitting
        # it here made every identity change unshippable without a reflash,
        # while this tool still reported success.
        extras = bi.stage_extras(stage, tokens, pubkey)
        tar = bi.make_payload_tar(stage, bi.HERE / "build" / bi.PAYLOAD_TAR_NAME)
    except Exception as e:
        return {"ok": False, "error": f"payload render failed: {e}"}

    push = _scp(f"/tmp/{bi.PAYLOAD_TAR_NAME}", tar, host=host, push=True, timeout=timeout)
    if not push["ok"]:
        return {"ok": False, "stage": "scp", "error": push["error"],
                "target": push.get("target")}

    script = (
        f"sudo mkdir -p {REMOTE_ROOT} && "
        f"sudo tar -xzf /tmp/{bi.PAYLOAD_TAR_NAME} -C {REMOTE_ROOT} && "
        f"sudo {REMOTE_ROOT}/install.sh"
    )
    r = ssh_run(["bash", "-lc", script], host=host, timeout=timeout)
    # install.sh exits 75 when it deliberately skipped restarting dnsmasq /
    # nftables because the NAT counters showed a live stream. That is the
    # designed behaviour mid-capture, not a failure -- but it does mean the new
    # config is on disk and not yet in force, so say so rather than reporting a
    # clean success.
    restarts_deferred = r.get("rc") == 75
    install_ok = r["ok"] or restarts_deferred

    after = status(host=host)
    units = after.get("units") if after.get("ok") else None
    bad = [u for u, s in (units or {}).items() if s not in ("active", "activating")]
    return {
        "ok": install_ok and after.get("ok", False) and not bad,
        "install_rc": r.get("rc"),
        "install_ok": install_ok,
        "restarts_deferred": restarts_deferred,
        "restarts_deferred_note": (
            "files installed, but dnsmasq/nftables were NOT restarted because a "
            "stream is live; rerun after the capture, or use the CLI with --force"
        ) if restarts_deferred else None,
        "install_error": None if install_ok else r.get("error"),
        "install_tail": "\n".join(r.get("stdout", "").splitlines()[-25:]),
        "rendered": len(render.get("rendered", [])),
        "extras": extras,
        "units": units,
        "units_not_running": bad,
        "target": r.get("target"),
    }


def tee_list(host: str = "", *, timeout: float = 30.0) -> dict:
    """List the pcap ring files on the Pi, newest first."""
    script = (f"ls -l --time-style=+%s {TEE_DIR} 2>/dev/null | "
              f"awk '$1 !~ /^total/ && NF>=7 {{print $5, $6, $7}}'")
    r = ssh_run(["bash", "-lc", script], host=host, timeout=timeout)
    if not r["ok"]:
        return {"ok": False, "error": r.get("error", "ssh failed"), "target": r.get("target")}
    files = []
    for line in r["stdout"].splitlines():
        parts = line.split()
        if len(parts) != 3:
            continue
        size, mtime, name = parts
        try:
            files.append({"name": name, "bytes": int(size), "mtime": int(mtime)})
        except ValueError:
            continue
    files.sort(key=lambda f: f["mtime"], reverse=True)
    return {"ok": True, "dir": TEE_DIR, "files": files,
            "total_bytes": sum(f["bytes"] for f in files), "count": len(files),
            "target": r["target"]}


def tee_fetch(name: str, out: str = "", convert: bool = True, host: str = "",
              *, timeout: float = 600.0) -> dict:
    """Copy one ring pcap off the Pi and (by default) convert it to a capture .bin.

    The conversion is the point: the resulting `.bin` is byte-format-identical
    to what `capture.py` writes, so every existing `capture_*` tool reads it
    unchanged -- and its loss counters are directly comparable with the ones
    the live host recorded for the same take. That comparison is the whole
    argument for the tee: frames the Wi-Fi hop lost are still in the pcap.
    """
    if not name or "/" in name or name.startswith("."):
        return {"ok": False, "error": f"refusing unsafe pcap name {name!r}"}
    captures = REPO / "captures"
    captures.mkdir(exist_ok=True)
    local = Path(out).expanduser() if out else captures / name
    if not local.is_absolute():
        local = REPO / local
    local.parent.mkdir(parents=True, exist_ok=True)

    got = _scp(f"{TEE_DIR}/{name}", local, host=host, timeout=timeout)
    if not got["ok"]:
        return {"ok": False, "stage": "scp", "error": got["error"],
                "target": got.get("target")}
    res = {"ok": True, "pcap": str(local.relative_to(REPO) if local.is_relative_to(REPO)
                                   else local),
           "bytes": local.stat().st_size, "target": got.get("target"), "converted": None}
    if not convert:
        return res

    from tools.pcap2capture import convert as pcap_convert  # noqa: E402

    bin_path = local.with_suffix(".bin") if local.suffix else Path(str(local) + ".bin")
    try:
        stats = pcap_convert([local], out_path=bin_path)
    except Exception as e:
        res["converted"] = {"ok": False, "error": str(e)}
        return res
    stats["ok"] = True
    stats["out"] = str(bin_path.relative_to(REPO) if bin_path.is_relative_to(REPO)
                       else bin_path)
    res["converted"] = stats
    return res


def reboot(host: str = "", *, timeout: float = 20.0) -> dict:
    """Reboot the Pi. Returns immediately; the Pi takes ~30 s to come back."""
    r = ssh_run(["bash", "-lc", "sudo systemctl reboot"], host=host, timeout=timeout)
    # A reboot commonly kills the ssh channel before it can exit cleanly; that
    # is a success, not a failure, so treat 255/-1 with an empty error as OK.
    ok = r["ok"] or r.get("rc") in (255, -1)
    return {"ok": ok, "rc": r.get("rc"), "target": r.get("target"),
            "note": "the Pi takes roughly 30 s to come back; bridge_status will "
                    "fail until then",
            "error": None if ok else r.get("error")}


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print_status(d: dict) -> None:
    if not d.get("ok"):
        print(f"bridge unreachable: {d.get('error')}")
        print(f"  target: {d.get('target')}")
        return
    wlan = d.get("wlan0") or {}
    eth = d.get("eth0") or {}
    tee = d.get("tee") or {}
    print(f"model:    {d.get('model')}  ({d.get('hostname')})")
    print(f"ssh:      {d.get('ssh_latency_ms')} ms via {(d.get('target') or {}).get('via')}")
    print(f"wlan0:    {wlan.get('ssid')}  {wlan.get('rssi_dbm')} dBm  "
          f"{wlan.get('bitrate_mbps')} Mb/s  power_save={wlan.get('power_save')}  "
          f"{wlan.get('ipv4')}")
    print(f"eth0:     carrier={eth.get('carrier')}  {eth.get('ipv4')}  "
          f"scanner={eth.get('scanner_lease') or eth.get('scanner_neigh') or 'absent'}")
    print(f"tee:      active={tee.get('active')}  {tee.get('ring_files')} files  "
          f"{(tee.get('ring_bytes') or 0) / 1e6:.0f} MB")
    print(f"temp:     {d.get('temp_c')} C   ntp_synced={d.get('ntp_synced')}")
    nft = d.get("nft") or {}
    print(f"dnat ->:  {nft.get('dnat_target')}")
    for rule in nft.get("rules") or []:
        print(f"nft {rule.get('name')}: {rule.get('packets')} pkts / "
              f"{rule.get('bytes')} B")
    for u, s in (d.get("units") or {}).items():
        print(f"unit:     {u} = {s}")
    for w in d.get("warnings", []):
        print(f"WARNING:  {w}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="", help="override target (else $ROOMSCAN_BRIDGE_HOST, else mDNS)")
    ap.add_argument("--json", action="store_true")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("status")
    p = sub.add_parser("logs")
    p.add_argument("unit", nargs="?", default="roomscan-bridge")
    p.add_argument("--lines", type=int, default=200)
    p = sub.add_parser("wifi-update")
    p.add_argument("ssid")
    p.add_argument("psk")
    p.add_argument("--profile", default="home", choices=["home", "travel"])
    p = sub.add_parser("update")
    p.add_argument("--secrets", type=Path, default=None)
    sub.add_parser("tee-list")
    p = sub.add_parser("tee-fetch")
    p.add_argument("name")
    p.add_argument("-o", "--out", default="")
    p.add_argument("--no-convert", action="store_true")
    sub.add_parser("reboot")

    args = ap.parse_args(argv)
    h = args.host

    if args.cmd == "status":
        rep = status(h)
        printer = _print_status
    elif args.cmd == "logs":
        rep = logs(args.unit, args.lines, h)
        printer = lambda r: print(r.get("text") or r.get("error"))  # noqa: E731
    elif args.cmd == "wifi-update":
        rep = wifi_update(args.ssid, args.psk, args.profile, h)
        printer = lambda r: print(json.dumps(r, indent=2))  # noqa: E731
    elif args.cmd == "update":
        rep = update(h, secrets=args.secrets)
        printer = lambda r: print(json.dumps(r, indent=2))  # noqa: E731
    elif args.cmd == "tee-list":
        rep = tee_list(h)
        printer = lambda r: [print(f"{f['bytes']:>12}  {f['name']}")  # noqa: E731
                             for f in r.get("files", [])] and None
    elif args.cmd == "tee-fetch":
        rep = tee_fetch(args.name, args.out, not args.no_convert, h)
        printer = lambda r: print(json.dumps(r, indent=2))  # noqa: E731
    else:  # pragma: no cover - argparse enforces
        return 2

    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        printer(rep)
    return 0 if rep.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
