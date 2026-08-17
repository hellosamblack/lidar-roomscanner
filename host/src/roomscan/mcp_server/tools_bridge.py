"""Administer the Raspberry Pi bridge node, and build its SD image.

The Pi 3 bridge (issue #191) replaces the RavPower FileHub as the scanner's
wireless uplink. The FileHub's real defect was opacity -- when it dropped
frames, nothing said so -- so every tool here is a thin wrapper over a pure
function in `host/tools/pi_bridge.py` that reports the state it read back, not
the action it requested. An unreachable Pi returns `{"ok": False, "error": ...}`
rather than raising, because "I could not reach the bridge" is an answer.

Target resolution is automatic: `$ROOMSCAN_BRIDGE_HOST`, else mDNS
`_roomscan-bridge._tcp`, else `roomscan-bridge.local`. Every tool takes an
optional `host` to override it.
"""
from __future__ import annotations

from .server import mcp


@mcp.tool()
def bridge_status(host: str = "") -> dict:
    """Full health of the Pi bridge node: radio, scanner link, NAT counters, tee.

    Runs the Pi's own `roomscan-bridge-status` script and returns its JSON, plus
    the ssh hop's latency (itself a Wi-Fi health signal). Fields worth knowing:

      wlan0.power_save   the driver's ACTUAL setting, not the one we asked for
      eth0.scanner_lease whether the scanner took its DHCP lease, or fell back
                         to its self-assigned 172.31.253.1 server mode
      nft.*              per-rule packet/byte counters -- the truth-source for
                         "is the stream actually flowing through the NAT"
      tee.*              whether the pcap ring is recording, and how full it is
      ntp_synced         the Pi 3 has no RTC; timestamps are meaningless until
                         this is true

    `warnings` is the short list of things a human should act on. Start here
    when a capture looks lossy and you suspect the wireless hop.
    """
    from tools.pi_bridge import status

    return status(host)


@mcp.tool()
def bridge_logs(unit: str = "roomscan-bridge", lines: int = 200,
                host: str = "") -> dict:
    """Tail one unit's journal on the Pi.

    `unit` accepts either a systemd unit (`roomscan-tee.service`, `dnsmasq`,
    `roomscan-bridge-reconcile.timer`) or the syslog tag `roomscan-bridge`,
    which is where the reconcile loop and the installer log. The reconcile log
    is where to look when the scanner dropped to its fallback address.
    """
    from tools.pi_bridge import logs

    return logs(unit, lines, host)


@mcp.tool()
def bridge_wifi_update(ssid: str, psk: str, profile: str = "home",
                       host: str = "") -> dict:
    """Change one Wi-Fi profile's credentials, and report whether it associated.

    Returns the association state read back after the radio settles -- not the
    nmcli exit code. Check `associated` and `actual_ssid`, not `ok` alone.

    This can strand the Pi if the new PSK is wrong: the change is applied to one
    named profile (`home` or `travel`) so the other still autoconnects, and the
    recovery path is a `wifi-override.nmconnection` dropped on the SD card's FAT
    partition from any laptop, which `install.sh` re-applies on every boot.
    """
    from tools.pi_bridge import wifi_update

    return wifi_update(ssid, psk, profile, host)


@mcp.tool()
def bridge_update(host: str = "", secrets: str = "") -> dict:
    """Re-render the bridge payload from this checkout, push it, and reinstall.

    How a payload change ships without reflashing a card. Reports the unit
    states read back from the Pi afterwards (`units`, `units_not_running`),
    because "install.sh exited 0" and "the bridge works" are different claims.

    `secrets` defaults to `$ROOMSCAN_BRIDGE_SECRETS`, else
    `~/.config/roomscan/pi-bridge-secrets.yaml`. The installer refuses to
    restart dnsmasq/nftables while the NAT counters show a live stream, so this
    is safe to run mid-session -- it will tell you what it skipped.
    """
    from pathlib import Path

    from tools.pi_bridge import update

    return update(host, secrets=Path(secrets) if secrets else None)


@mcp.tool()
def bridge_tee_list(host: str = "") -> dict:
    """List the pcap ring files the Pi has teed off the scanner link, newest first.

    The tee is a bounded 2 GB ring (20 x 100 MB, roughly 70 minutes at the
    measured ~466 KB/s), so a capture older than that is gone. Check here before
    assuming a lost frame is recoverable.
    """
    from tools.pi_bridge import tee_list

    return tee_list(host)


@mcp.tool()
def bridge_tee_fetch(name: str, out: str = "", convert: bool = True,
                     host: str = "") -> dict:
    """Copy one ring pcap off the Pi and convert it into a capture `.bin`.

    The conversion is the point of the tee: the `.bin` is byte-format-identical
    to what `capture.py` writes, so every `capture_*` tool reads it unchanged,
    and its loss counters (`frames_incomplete`, `frags_lost`, `frame_loss_pct`)
    are directly comparable with what the host recorded for the same take.
    Frames the Wi-Fi hop dropped are still in the pcap -- that comparison is the
    whole argument for the tee against the 2.3-9.4 % FileHub baseline.

    `name` comes from `bridge_tee_list`. Lands in `captures/` unless `out` says
    otherwise.
    """
    from tools.pi_bridge import tee_fetch

    return tee_fetch(name, out, convert, host)


@mcp.tool()
def bridge_reboot(host: str = "") -> dict:
    """Reboot the Pi bridge. Returns immediately; it takes ~30 s to come back.

    A dropped ssh channel during the reboot is counted as success, not failure.
    `bridge_status` will fail until the Pi is up again.
    """
    from tools.pi_bridge import reboot

    return reboot(host)


@mcp.tool()
def bridge_image_build(secrets: str = "", release: str = "trixie",
                       xz: bool = False, skip_debs: bool = False) -> dict:
    """Build the flashable Pi bridge SD image, rootlessly, from this checkout.

    Downloads and sha256-verifies the pinned Raspberry Pi OS Lite image, parses
    its MBR in pure Python, resolves and bundles the Debian package closure the
    bridge needs (so the Pi's first boot needs no internet -- the only thing on
    its eth0 side is the scanner), renders the payload with your Wi-Fi
    credentials and ssh key, and injects everything into the FAT boot partition
    with `mtools`. No root, no loop devices.

    Requires `mtools` on this host (`sudo apt install mtools`). `secrets`
    defaults to `$ROOMSCAN_BRIDGE_SECRETS`, else
    `~/.config/roomscan/pi-bridge-secrets.yaml`; the template is
    `pi-bridge/pi-bridge-secrets.example.yaml`.

    The report is read back out of the built image (the real cmdline, the files
    genuinely present in the boot partition), not echoed from the inputs.
    """
    import os
    import sys
    from pathlib import Path

    from .paths import REPO

    sys.path.insert(0, str(Path(REPO) / "pi-bridge"))
    try:
        import build_image as bi
    except Exception as e:  # pragma: no cover - import guard
        return {"ok": False, "error": f"cannot import pi-bridge/build_image.py: {e}"}

    path = Path(secrets).expanduser() if secrets else Path(
        os.environ.get("ROOMSCAN_BRIDGE_SECRETS",
                       "~/.config/roomscan/pi-bridge-secrets.yaml")).expanduser()
    try:
        return bi.build(path, release_key=release, xz=xz, skip_debs=skip_debs,
                        quiet=True)
    except bi.BuildError as e:
        return {"ok": False, "error": str(e)}
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}


@mcp.tool()
def bridge_pcap_convert(path: str, out: str = "") -> dict:
    """Convert a tee pcap already on this host into a capture `.bin` + loss stats.

    The offline half of `bridge_tee_fetch`, for a pcap you already have. Accepts
    one file, several, or a directory of ring files -- they are processed in
    timestamp order, and a frame whose fragments straddle two ring files still
    reassembles.

    Wraps `host/tools/pcap2capture.py::convert()`.
    """
    from pathlib import Path

    from tools.pcap2capture import convert

    from .paths import REPO

    p = Path(path)
    if not p.is_absolute():
        p = Path(REPO) / path
    if not p.exists():
        return {"ok": False, "error": f"no such pcap: {path}"}
    dest = None
    if out:
        dest = Path(out)
        if not dest.is_absolute():
            dest = Path(REPO) / out
    try:
        stats = convert([p], out_path=dest)
    except Exception as e:
        return {"ok": False, "error": f"{type(e).__name__}: {e}"}
    stats["ok"] = True
    return stats
