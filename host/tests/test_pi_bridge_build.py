"""Tests for the rootless Pi bridge SD-image builder (issue #191).

None of this needs a Raspberry Pi, and none of it needs root -- which is the
whole point of the builder's design. What it cannot cover is the one thing only
a real first boot proves (that Raspberry Pi OS's `systemd.run` firstrun
mechanism fires and `install.sh` completes on the target); that stays an
operator gate, backed by firstrun.log landing on the FAT partition.
"""
from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
import xml.dom.minidom
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
PI = REPO / "pi-bridge"
PAYLOAD = PI / "payload"
BOOT = PI / "boot"


def _load_build_image():
    """Import `pi-bridge/build_image.py` without putting it on sys.path.

    It is a standalone build script, not part of the `roomscan` package, and it
    is named with a hyphenated parent directory -- so a plain import cannot
    reach it and a sys.path insert would leak into every other test module.
    """
    spec = importlib.util.spec_from_file_location("pi_bridge_build_image",
                                                  PI / "build_image.py")
    mod = importlib.util.module_from_spec(spec)
    # `@dataclass` resolves annotations through sys.modules, so the module has
    # to be registered before it executes, not after.
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


bi = _load_build_image()

SAMPLE_SECRETS = {
    "hostname": "roomscan-bridge-test",
    "username": "roomscan",
    "password": "test-password",
    "wifi_country": "US",
    "wifi": {
        "home": {"ssid": "TestHomeSSID", "passphrase": "home-passphrase-1", "priority": 50},
        "travel": {"ssid": "TestTravelSSID", "passphrase": "travel-passphrase-1",
                   "priority": 10},
    },
}
SAMPLE_PUBKEY = "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAITESTKEYTESTKEYTESTKEY roomscan-bridge"

has_mtools = pytest.mark.skipif(
    shutil.which("mcopy") is None or shutil.which("mformat") is None,
    reason="mtools not installed (sudo apt install mtools)")


# ---------------------------------------------------------------------------
# MBR
# ---------------------------------------------------------------------------

def _mbr(entries, signature=True) -> bytes:
    """entries: list of (bootable, ptype, lba_start, sectors)."""
    b = bytearray(512)
    for i, (boot, ptype, lba, sectors) in enumerate(entries):
        off = 446 + 16 * i
        b[off] = 0x80 if boot else 0x00
        b[off + 4] = ptype
        b[off + 8:off + 12] = int(lba).to_bytes(4, "little")
        b[off + 12:off + 16] = int(sectors).to_bytes(4, "little")
    if signature:
        b[510:512] = b"\x55\xaa"
    return bytes(b)


#: What a real Raspberry Pi OS image looks like: a 512 MiB FAT32(LBA) boot
#: partition at 4 MiB, then the ext4 root.
RPI_LIKE = [(True, 0x0C, 8192, 1048576), (False, 0x83, 1056768, 5000000)]


def test_parse_mbr_reads_a_raspberry_pi_like_table():
    parts = bi.parse_mbr(_mbr(RPI_LIKE))
    assert [p.index for p in parts] == [1, 2]
    boot = bi.boot_partition(parts)
    root = bi.root_partition(parts)
    assert boot.offset == 8192 * 512 == 4 * 1024 * 1024
    assert boot.size == 1048576 * 512
    assert boot.is_fat and not boot.is_ext
    assert root.offset == 1056768 * 512
    assert root.is_ext


def test_parse_mbr_rejects_a_missing_signature():
    with pytest.raises(bi.BuildError, match="0x55AA"):
        bi.parse_mbr(_mbr(RPI_LIKE, signature=False))


def test_parse_mbr_rejects_a_short_sector():
    with pytest.raises(bi.BuildError, match="short read"):
        bi.parse_mbr(b"\x00" * 100)


def test_parse_mbr_refuses_extended_partitions():
    # An offset computed from the wrong table shape would be handed straight to
    # mcopy, which would corrupt the image quietly rather than failing.
    with pytest.raises(bi.BuildError, match="extended partition"):
        bi.parse_mbr(_mbr([(False, 0x05, 2048, 1000)]))


def test_parse_mbr_refuses_a_zero_length_partition():
    with pytest.raises(bi.BuildError, match="0 sectors"):
        bi.parse_mbr(_mbr([(False, 0x0C, 2048, 0)]))


def test_parse_mbr_skips_empty_slots():
    parts = bi.parse_mbr(_mbr([(True, 0x0C, 8192, 100), (False, 0x00, 0, 0),
                               (False, 0x83, 9000, 100)]))
    assert [p.ptype for p in parts] == [0x0C, 0x83]


def test_boot_and_root_lookup_report_what_they_saw():
    parts = bi.parse_mbr(_mbr([(False, 0x83, 2048, 100)]))
    with pytest.raises(bi.BuildError, match="0x83"):
        bi.boot_partition(parts)


# ---------------------------------------------------------------------------
# cmdline.txt
# ---------------------------------------------------------------------------

REAL_CMDLINE = ("console=serial0,115200 console=tty1 root=PARTUUID=2f8c1a4d-02 "
                "rootfstype=ext4 fsck.repair=yes rootwait quiet")


def test_append_firstrun_args_appends_and_keeps_one_line():
    out = bi.append_firstrun_args(REAL_CMDLINE + "\n")
    assert out.endswith("\n")
    assert out.count("\n") == 1
    assert out.startswith(REAL_CMDLINE)
    for arg in bi.FIRSTRUN_ARGS:
        assert arg in out.split()


def test_append_firstrun_args_is_idempotent():
    once = bi.append_firstrun_args(REAL_CMDLINE)
    assert bi.append_firstrun_args(once) == once


def test_append_firstrun_args_refuses_a_multiline_cmdline():
    # The Pi bootloader passes only the first line; appending to line 2 would
    # silently do nothing, and present as a card that boots normally and never
    # installs the bridge.
    with pytest.raises(bi.BuildError, match="more than one line"):
        bi.append_firstrun_args(REAL_CMDLINE + "\nextra=1\n")


def test_append_firstrun_args_refuses_an_empty_cmdline():
    with pytest.raises(bi.BuildError, match="empty"):
        bi.append_firstrun_args("   \n")


def test_append_firstrun_args_refuses_a_partially_injected_cmdline():
    half = REAL_CMDLINE + " " + bi.FIRSTRUN_ARGS[0]
    with pytest.raises(bi.BuildError, match="some but not all"):
        bi.append_firstrun_args(half)


def test_strip_firstrun_args_round_trips():
    # This is the same edit firstrun.sh makes on the Pi to clean up after
    # itself; if it did not round-trip, every subsequent boot would re-install.
    injected = bi.append_firstrun_args(REAL_CMDLINE)
    assert bi.strip_firstrun_args(injected).strip() == REAL_CMDLINE


# ---------------------------------------------------------------------------
# Secrets hygiene
# ---------------------------------------------------------------------------

def _write_secrets(tmp_path: Path, data: dict, mode: int = 0o600) -> Path:
    import yaml
    p = tmp_path / "secrets.yaml"
    p.write_text(yaml.safe_dump(data))
    p.chmod(mode)
    return p


def test_load_secrets_accepts_a_good_private_file(tmp_path):
    got = bi.load_secrets(_write_secrets(tmp_path, SAMPLE_SECRETS))
    assert got["hostname"] == "roomscan-bridge-test"


def test_load_secrets_refuses_the_committed_example():
    with pytest.raises(bi.BuildError, match="example template"):
        bi.load_secrets(bi.EXAMPLE_SECRETS)


def test_load_secrets_refuses_world_readable_perms(tmp_path):
    p = _write_secrets(tmp_path, SAMPLE_SECRETS, mode=0o644)
    with pytest.raises(bi.BuildError, match="0644"):
        bi.load_secrets(p)
    # ...and can be overridden deliberately, not by accident.
    assert bi.load_secrets(p, allow_insecure_perms=True)["username"] == "roomscan"


def test_load_secrets_reports_every_missing_key(tmp_path):
    bad = {k: v for k, v in SAMPLE_SECRETS.items() if k not in ("hostname", "wifi_country")}
    with pytest.raises(bi.BuildError, match="hostname"):
        bi.load_secrets(_write_secrets(tmp_path, bad))


@pytest.mark.parametrize("psk", ["short", "x" * 64])
def test_load_secrets_rejects_out_of_range_passphrases(tmp_path, psk):
    # wpa_supplicant would reject these on the Pi, where nobody can see the error.
    data = {**SAMPLE_SECRETS,
            "wifi": {"home": {"ssid": "S", "passphrase": psk}}}
    with pytest.raises(bi.BuildError, match="8-63"):
        bi.load_secrets(_write_secrets(tmp_path, data))


def test_load_secrets_rejects_a_placeholder_passphrase(tmp_path):
    data = {**SAMPLE_SECRETS,
            "wifi": {"home": {"ssid": "S", "passphrase": "CHANGEME-home-psk"}}}
    with pytest.raises(bi.BuildError, match="placeholder"):
        bi.load_secrets(_write_secrets(tmp_path, data))


def test_load_secrets_rejects_a_bad_country_code(tmp_path):
    with pytest.raises(bi.BuildError, match="2-letter"):
        bi.load_secrets(_write_secrets(tmp_path, {**SAMPLE_SECRETS, "wifi_country": "USA"}))


def test_the_committed_example_carries_no_real_credentials():
    text = bi.EXAMPLE_SECRETS.read_text()
    assert "CHANGEME" in text
    for line in text.splitlines():
        if "passphrase:" in line or "password:" in line:
            assert "CHANGEME" in line, f"example file may carry a real secret: {line!r}"


def test_no_real_secrets_file_can_be_tracked_in_git():
    """The example is committed; the filled-in file and the multi-GB build
    artefacts must be ignored. Asserted against `git check-ignore` rather than
    the index, so it holds before the first commit too."""
    assert bi.EXAMPLE_SECRETS.is_file()
    for path in ("pi-bridge/pi-bridge-secrets.yaml", "pi-bridge/cache/x.img.xz",
                 "pi-bridge/out/x.img", "pi-bridge/build/payload/debs/x.deb"):
        r = subprocess.run(["git", "check-ignore", "-q", path], cwd=REPO)
        assert r.returncode == 0, f"{path} is not git-ignored"
    tracked = subprocess.run(["git", "ls-files", "pi-bridge/"], cwd=REPO,
                             capture_output=True, text=True).stdout.split()
    assert not [t for t in tracked if t.endswith("pi-bridge-secrets.yaml")]
    assert not [t for t in tracked if t.endswith(".deb") or t.endswith(".img")]


def test_no_payload_file_is_accidentally_git_ignored():
    """Every payload file must actually be committable.

    The repo ignores `*.d` for C dependency files, which also matches every
    Debian drop-in *directory* -- `dnsmasq.d`, `sysctl.d`, `conf.d`. Three
    load-bearing configs (the scanner's DHCP lease, IP forwarding, Wi-Fi power
    save) were silently absent from the first commit because of it, which would
    have produced a Pi that boots fine and routes nothing. The builder cannot
    catch this: it reads the working tree, where the files are present.
    """
    paths = [str(p.relative_to(REPO)) for p in _payload_files()]
    assert paths
    r = subprocess.run(["git", "check-ignore", "--stdin"], cwd=REPO,
                       input="\n".join(paths), capture_output=True, text=True)
    ignored = [ln for ln in r.stdout.splitlines() if ln.strip()]
    assert not ignored, f"payload files excluded from git by .gitignore: {ignored}"


# ---------------------------------------------------------------------------
# Token contract
# ---------------------------------------------------------------------------

def _payload_files(include_docs: bool = True) -> list[Path]:
    return [p for p in list(PAYLOAD.rglob("*")) + list(BOOT.rglob("*"))
            if p.is_file() and (include_docs or p.name not in bi.PAYLOAD_DOCS)]


def test_every_token_the_payload_uses_can_be_resolved():
    """The builder/payload contract, enforced rather than documented.

    An unresolved `{{HOME_PSK}}` in a NetworkManager keyfile is a Pi that never
    associates and says nothing about why -- so the builder hard-errors, and
    this test proves the error cannot fire for the payload as committed.
    """
    tokens = bi.secret_tokens(SAMPLE_SECRETS, SAMPLE_PUBKEY)
    used: dict[str, list[str]] = {}
    for p in _payload_files(include_docs=False):
        raw = p.read_bytes()
        if b"\x00" in raw[:4096]:
            continue
        for m in bi.TOKEN_RE.finditer(raw.decode("utf-8", "replace")):
            used.setdefault(m.group(1), []).append(str(p.relative_to(PI)))
    unresolved = {t: f for t, f in used.items() if t not in tokens}
    assert not unresolved, f"payload references unknown token(s): {unresolved}"
    assert used, "no tokens found at all -- did the payload move?"


def test_render_text_reports_unresolved_tokens_rather_than_dropping_them():
    out, missing = bi.render_text("a={{KNOWN}} b={{NOPE}}", {"KNOWN": "1"})
    assert out == "a=1 b={{NOPE}}"
    assert missing == {"NOPE"}


def test_profile_uuids_are_stable_across_builds_but_differ_per_profile():
    # A fresh random UUID per build would leave the Pi accumulating duplicate
    # NetworkManager profiles every time bridge_update pushed the payload.
    a = bi.secret_tokens(SAMPLE_SECRETS, SAMPLE_PUBKEY)
    b = bi.secret_tokens(SAMPLE_SECRETS, SAMPLE_PUBKEY)
    assert a["UUID_ETH0"] == b["UUID_ETH0"]
    assert a["UUID_WIFI_HOME"] == b["UUID_WIFI_HOME"] != a["UUID_WIFI_TRAVEL"]
    assert a["ETH0_UUID"] == a["UUID_ETH0"]  # alias spelling the templates use
    other = bi.secret_tokens({**SAMPLE_SECRETS, "hostname": "other"}, SAMPLE_PUBKEY)
    assert other["UUID_ETH0"] != a["UUID_ETH0"]


def test_a_missing_travel_profile_still_renders():
    data = {**SAMPLE_SECRETS, "wifi": {"home": SAMPLE_SECRETS["wifi"]["home"]}}
    tokens = bi.secret_tokens(data, SAMPLE_PUBKEY)
    assert tokens["TRAVEL_SSID"] == "" and tokens["TRAVEL_PRIORITY"] == "0"


def test_scanner_identity_matches_the_firmware_constants():
    """`00:80:E1:00:00:00` and the 172.31.253.1 fallback are compile-time
    constants in `firmware/scanner-stream` -- the dnsmasq static lease and the
    reconcile probe are only correct because they match.

    This test used to compare the bridge config against the ETH_MAC_ADDR*
    defines ALONE, and passed for as long as it existed while being blind to the
    actual defect: nothing in the firmware read those defines. `ethernetif.c`
    hardcoded CubeMX's `{0x02,0,0,0,0,0}` placeholder, so the header and the
    bridge agreed with each other and both disagreed with the wire. On the real
    rig the scanner presented 02:00:00:00:00:00, missed its pinned reservation,
    and took a dynamic-range address, breaking every fixed-address assumption in
    the bridge (issue #191).

    So the header is no longer trusted on its own: the assertion is against the
    array that is actually loaded into the MAC filter."""
    hdr = (REPO / "firmware" / "scanner-stream" / "Inc" / "ethernet_transport.h").read_text()
    octets = []
    for i in range(6):
        for line in hdr.splitlines():
            if f"ETH_MAC_ADDR{i}" in line and "define" in line:
                octets.append(int(line.split()[-1], 16))
                break
    assert len(octets) == 6
    mac = ":".join(f"{o:02X}" for o in octets)
    assert mac == bi.DEFAULT_NETWORK["SCANNER_MAC"].upper()

    # The defines must actually reach the hardware. `low_level_init()` is where
    # the MAC array is handed to HAL_ETH_Init via EthHandle.Init.MACAddr; if that
    # array is a literal again, the constants above are decoration.
    eif = (REPO / "firmware" / "scanner-stream" / "Src" / "ethernetif.c").read_text()
    # Read the whole STATEMENT, not one line: the initialiser wraps, and a
    # line-scoped match happily saw ETH_MAC_ADDR0..2 and missed 3..5.
    at = eif.find("macaddress[6]")
    assert at != -1, "could not find the MAC array in ethernetif.c"
    decl_text = eif[at:eif.index(";", at)]
    for i in range(6):
        assert f"ETH_MAC_ADDR{i}" in decl_text, (
            f"ethernetif.c's MAC array does not use ETH_MAC_ADDR{i} -- the header "
            f"constants are dead code and the board will present something else: "
            f"{' '.join(decl_text.split())}")

    src = (REPO / "firmware" / "scanner-stream" / "Src" / "ethernet_transport.c").read_text()
    assert "172.31.253.1" in bi.DEFAULT_NETWORK["SCANNER_FALLBACK_IP"]
    assert "IP_ADDR3" in src  # the fallback address is built from these defines


# ---------------------------------------------------------------------------
# Payload render + lint
# ---------------------------------------------------------------------------

@pytest.fixture(scope="module")
def rendered(tmp_path_factory) -> Path:
    dest = tmp_path_factory.mktemp("payload")  / "rendered"
    bi.render_payload(PAYLOAD, dest, bi.secret_tokens(SAMPLE_SECRETS, SAMPLE_PUBKEY))
    return dest


def test_rendered_keyfiles_carry_the_credentials_and_are_not_world_readable(rendered):
    conns = rendered / "etc" / "NetworkManager" / "system-connections"
    home = conns / "roomscan-wifi-home.nmconnection"
    assert home.is_file(), sorted(p.name for p in conns.iterdir())
    text = home.read_text()
    assert "TestHomeSSID" in text and "home-passphrase-1" in text
    assert "{{" not in text
    assert oct(home.stat().st_mode & 0o777) == "0o600"
    eth = (conns / "roomscan-eth0.nmconnection").read_text()
    assert bi.DEFAULT_NETWORK["ETH_CIDR"] in eth or bi.DEFAULT_NETWORK["ETH_ADDR"] in eth


def test_no_rendered_file_still_holds_a_token(rendered):
    for p in rendered.rglob("*"):
        if not p.is_file():
            continue
        raw = p.read_bytes()
        if b"\x00" in raw[:4096]:
            continue
        assert not bi.TOKEN_RE.search(raw.decode("utf-8", "replace")), \
            f"{p.relative_to(rendered)} still holds an unrendered token"


def test_render_fails_loudly_on_an_unknown_token(tmp_path):
    src = tmp_path / "src"
    (src / "etc").mkdir(parents=True)
    (src / "etc" / "x.conf.tmpl").write_text("psk={{NOT_A_REAL_TOKEN}}\n")
    with pytest.raises(bi.BuildError, match="NOT_A_REAL_TOKEN"):
        bi.render_payload(src, tmp_path / "out", {"HOSTNAME": "h"})


def test_no_payload_file_has_crlf_endings():
    # CRLF in a shell script or a NetworkManager keyfile fails on the Pi in ways
    # that read as "the file is wrong" rather than "the newlines are wrong".
    bad = [str(p.relative_to(PI)) for p in _payload_files()
           if b"\r\n" in p.read_bytes()]
    assert not bad, f"CRLF line endings in {bad}"


def _shell_scripts() -> list[Path]:
    out = []
    for p in _payload_files():
        if p.suffix == ".sh" or "sbin" in p.parts or p.name == "firstrun.sh":
            head = p.read_bytes()[:64]
            if head.startswith(b"#!") and b"sh" in head.split(b"\n")[0]:
                out.append(p)
    return out


def test_every_shell_script_parses(rendered):
    scripts = _shell_scripts()
    assert len(scripts) >= 4, f"expected the payload's scripts, found {scripts}"
    for p in scripts:
        r = subprocess.run(["bash", "-n", str(p)], capture_output=True, text=True)
        assert r.returncode == 0, f"bash -n {p.relative_to(PI)}: {r.stderr}"


def test_every_shell_script_is_executable():
    for p in _shell_scripts():
        assert os.access(p, os.X_OK), f"{p.relative_to(PI)} is not executable"


def test_avahi_publishes_the_exact_instance_name_the_host_looks_up(rendered):
    """`UdpSource` resolves `roomscanner._roomscan._udp.local.` by literal name
    (`sources.py:287`). If avahi published `%h` or anything else, discovery
    would fail with zero host-side symptoms beyond "the board is not found"."""
    svc = rendered / "etc" / "avahi" / "services" / "roomscanner.service"
    dom = xml.dom.minidom.parse(str(svc))
    names = [n.firstChild.data.strip() for n in dom.getElementsByTagName("name")]
    assert "roomscanner" in names, names
    types = [t.firstChild.data.strip() for t in dom.getElementsByTagName("type")]
    assert "_roomscan._udp" in types, types
    ports = [p.firstChild.data.strip() for p in dom.getElementsByTagName("port")]
    assert "5000" in ports, ports

    bridge = rendered / "etc" / "avahi" / "services" / "roomscan-bridge.service"
    bdom = xml.dom.minidom.parse(str(bridge))
    bnames = [n.firstChild.data.strip() for n in bdom.getElementsByTagName("name")]
    assert "roomscan-bridge" in bnames, bnames


def test_avahi_is_confined_to_wlan0(rendered):
    """On eth0 the *scanner* announces `roomscanner`. Two responders with the
    same instance name on the same segment is a name collision, and avahi's
    resolution of it is to rename -- silently breaking discovery."""
    conf = (rendered / "etc" / "avahi" / "avahi-daemon.conf").read_text()
    assert "allow-interfaces=wlan0" in conf.replace(" ", "")


def test_dnsmasq_serves_dhcp_only_and_pins_the_scanner_lease(rendered):
    conf = (rendered / "etc" / "dnsmasq.d" / "roomscan-bridge.conf").read_text()
    flat = conf.replace(" ", "")
    # port=0 disables DNS: the Pi must not become a resolver for the LAN.
    assert "port=0" in flat
    assert "dhcp-authoritative" in flat
    assert bi.DEFAULT_NETWORK["SCANNER_MAC"].lower() in conf.lower()
    assert bi.DEFAULT_NETWORK["SCANNER_IP"] in conf


def test_dnsmasq_answers_inside_the_firmware_dhcp_window(rendered):
    """The firmware gives up on DHCP after 3000 ms and self-assigns
    (`ethernet_transport.c`). A long `dhcp-lease-max`/`dhcp-range` lease time is
    fine, but any configured *delay* would push the answer past that window."""
    conf = (rendered / "etc" / "dnsmasq.d" / "roomscan-bridge.conf").read_text()
    assert "dhcp-mac" not in conf or "delay" not in conf
    for line in conf.splitlines():
        assert not line.strip().startswith("dhcp-script"), \
            "a dhcp-script runs before the reply and can blow the 3 s window"


def test_nftables_counts_every_rule_and_dnats_the_stream(rendered):
    nft = (rendered / "etc" / "nftables" / "roomscan-bridge.nft").read_text()
    assert "table ip roomscan_bridge" in nft
    assert "dnat" in nft.lower() and bi.DEFAULT_NETWORK["SCANNER_IP"] in nft
    assert "masquerade" in nft.lower()
    rules = [ln for ln in nft.splitlines()
             if ("dnat" in ln.lower() or "masquerade" in ln.lower())
             and not ln.strip().startswith("#")]
    assert rules
    for ln in rules:
        # bridge_status reads these counters as the truth-source for "is the
        # stream flowing"; a rule without one is invisible.
        assert "counter" in ln, f"rule without a counter: {ln.strip()}"


@pytest.mark.skipif(shutil.which("nft") is None, reason="nft not installed")
def test_nftables_ruleset_is_syntactically_valid(rendered, tmp_path):
    nft = rendered / "etc" / "nftables" / "roomscan-bridge.nft"
    r = subprocess.run(["nft", "-c", "-f", str(nft)], capture_output=True, text=True)
    if r.returncode != 0 and "Operation not permitted" in r.stderr:
        pytest.skip("nft -c needs CAP_NET_ADMIN for cache init in this container")
    assert r.returncode == 0, r.stderr


def test_systemd_units_parse_and_declare_what_they_must(rendered):
    import configparser
    units = sorted((rendered / "etc" / "systemd" / "system").glob("*"))
    assert {p.name for p in units} >= {
        "roomscan-tee.service", "roomscan-bridge-reconcile.service",
        "roomscan-bridge-reconcile.timer"}
    for p in units:
        cp = configparser.ConfigParser(strict=False, allow_no_value=True)
        cp.optionxform = str
        cp.read_string(p.read_text())
        assert "Unit" in cp, f"{p.name} has no [Unit] section"

    tee = configparser.ConfigParser(strict=False, allow_no_value=True)
    tee.optionxform = str
    tee.read_string((rendered / "etc" / "systemd" / "system"
                     / "roomscan-tee.service").read_text())
    exec_start = tee["Service"]["ExecStart"]
    assert "tcpdump" in exec_start
    # A bounded ring, not an unbounded capture: the Pi has one SD card.
    assert "-C" in exec_start and "-W" in exec_start
    assert "port 5000" in exec_start or "port {{" in exec_start


def test_reconcile_timer_runs_often_enough_to_matter(rendered):
    import configparser
    cp = configparser.ConfigParser(strict=False, allow_no_value=True)
    cp.optionxform = str
    cp.read_string((rendered / "etc" / "systemd" / "system"
                    / "roomscan-bridge-reconcile.timer").read_text())
    spec = cp["Timer"].get("OnUnitActiveSec") or cp["Timer"].get("OnCalendar", "")
    assert "10" in spec or "sec" in spec.lower(), spec


def test_ip_forwarding_is_enabled_with_loose_rp_filter(rendered):
    # Loose (2), not strict: the DNAT/masquerade return path is asymmetric, and
    # strict rp_filter would drop it with no log.
    sysctl = (rendered / "etc" / "sysctl.d" / "99-roomscan-bridge.conf").read_text()
    flat = sysctl.replace(" ", "")
    assert "net.ipv4.ip_forward=1" in flat
    assert "rp_filter=2" in flat


def test_install_script_is_shared_by_firstrun_and_update():
    """One installer, two callers -- so a payload change cannot behave one way
    on a fresh card and another way over `bridge_update`."""
    firstrun = (BOOT / "firstrun.sh").read_text()
    assert "install.sh" in firstrun
    assert "--first-boot" in firstrun
    install = (PAYLOAD / "install.sh").read_text()
    assert "--first-boot" in install


def test_firstrun_logs_to_the_fat_partition():
    """The one failure mode with no other diagnostic: a first boot that dies
    before the network exists. The log has to land somewhere a laptop can read
    with a card reader, which means the FAT partition."""
    firstrun = (BOOT / "firstrun.sh").read_text()
    assert "/boot/firmware/firstrun.log" in firstrun


def test_firstrun_cleans_up_its_own_cmdline_arguments():
    firstrun = (BOOT / "firstrun.sh").read_text()
    assert "cmdline.txt" in firstrun
    assert "systemd.run" in firstrun


def test_wifi_override_recovery_path_exists():
    """The bad-credentials chicken-and-egg: fixing Wi-Fi over Wi-Fi. install.sh
    applies an override dropped on the FAT partition from any laptop."""
    install = (PAYLOAD / "install.sh").read_text()
    assert "wifi-override.nmconnection" in install
    assert "/boot/firmware" in install


def test_stage_extras_writes_the_node_identity_install_sh_reads(tmp_path):
    """`node.env` and `authorized_keys` are generated, not templated.

    They lived inline in `build()` at first, which meant `bridge_update` pushed a
    payload with no `node.env`: install.sh logged a warning nobody surfaced, kept
    the Pi's old values, and the tool still reported success -- so a changed
    scanner address or hostname could never reach a provisioned Pi without a
    reflash. Both callers now go through this one function.
    """
    tokens = bi.secret_tokens(SAMPLE_SECRETS, SAMPLE_PUBKEY)
    stage = tmp_path / "stage"
    names = bi.stage_extras(stage, tokens, SAMPLE_PUBKEY)
    assert set(names) == {"authorized_keys", "node.env"}

    env = dict(ln.split("=", 1) for ln in
               (stage / "node.env").read_text().strip().splitlines())
    assert env["SCANNER_IP"] == bi.DEFAULT_NETWORK["SCANNER_IP"]
    assert env["SCANNER_FALLBACK_IP"] == bi.DEFAULT_NETWORK["SCANNER_FALLBACK_IP"]
    assert env["USERNAME"] == SAMPLE_SECRETS["username"]
    assert env["HOSTNAME"] == SAMPLE_SECRETS["hostname"]
    assert env["WIFI_COUNTRY"] == "US"
    assert (stage / "authorized_keys").read_text().strip() == SAMPLE_PUBKEY

    # Every key the Pi-side scripts source must actually be emitted.
    for script in ("usr/local/sbin/roomscan-bridge-common.sh",
                   "usr/local/sbin/roomscan-bridge-reconcile",
                   "usr/local/sbin/roomscan-bridge-status"):
        text = (PAYLOAD / script).read_text()
        for key in ("SCANNER_IP", "SCANNER_FALLBACK_IP", "STREAM_PORT"):
            if f"${{{key}" in text or f"${key}" in text:
                assert key in env, f"{script} reads ${key} but node.env omits it"


def test_install_sh_survives_a_user_that_does_not_exist_yet():
    """The `getent` lookup must not be able to kill the installer.

    install.sh runs under `set -euo pipefail` from `kernel-command-line.target`,
    while Raspberry Pi OS creates the `userconf.txt` account during normal boot --
    so the account can legitimately not exist yet. An unguarded
    `home="$(getent passwd "$user" | cut -d: -f6)"` exits 2 through `pipefail`,
    `set -e` kills the script before `activate_units()`, no unit is ever enabled,
    and firstrun.sh never reaches `exit 0` so the Pi never reboots.
    """
    text = (PAYLOAD / "install.sh").read_text()
    getent_lines = [ln for ln in text.splitlines() if "getent passwd" in ln]
    assert getent_lines
    for ln in getent_lines:
        stripped = ln.strip()
        guarded = ("|| " in stripped or stripped.startswith("if ")
                   or stripped.startswith("local ") and "=" in stripped)
        assert guarded, f"unguarded getent under set -e: {stripped}"


# ---------------------------------------------------------------------------
# First-boot regressions -- every test below pins a defect that shipped to the
# real Pi 3 on 2026-08-18 and was found during bring-up (issue #191).
# ---------------------------------------------------------------------------

def test_the_ssh_key_does_not_depend_on_an_account_existing():
    """The one failure that cannot be repaired remotely.

    install.sh runs from `kernel-command-line.target`; the `roomscan` account is
    created LATER, by userconf.txt during normal boot. So on the boot that
    matters the home directory does not exist, and the first real Pi came up
    fully provisioned and unreachable by key:

        install.sh: ERROR: user 'roomscan' does not exist on this system

    The key must therefore land somewhere that needs no account -- and sshd must
    be told to read it.
    """
    text = (PAYLOAD / "install.sh").read_text()
    assert "/etc/roomscan-bridge/authorized_keys" in text

    # The account-independent write must NOT sit behind the getent lookup.
    body = text.split("install_authorized_key()", 1)[1].split("\n}", 1)[0]
    etc_at = body.index("/etc/roomscan-bridge/authorized_keys")
    getent_at = body.index("getent passwd")
    assert etc_at < getent_at, \
        "the account-independent key install must happen BEFORE the user lookup"

    dropin = (PAYLOAD / "etc" / "ssh" / "sshd_config.d" / "roomscan-bridge.conf").read_text()
    assert "AuthorizedKeysFile" in dropin
    assert "/etc/roomscan-bridge/authorized_keys" in dropin
    # Scoped, not global: Debian's default `PermitRootLogin prohibit-password`
    # still permits *publickey*, so a global AuthorizedKeysFile would hand the
    # bridge key a root shell.
    assert "Match User" in dropin, "AuthorizedKeysFile must be scoped to the bridge account"


def test_remote_administration_has_the_sudo_grant_every_tool_assumes():
    """`pi_bridge.py` shells out through `sudo` for nmcli, install.sh and reboot,
    over a non-interactive ssh channel with no terminal to type a password into.
    The account is in the `sudo` group, whose default rule *demands* a password,
    so every day-2 tool failed on the first real Pi with

        sudo: a terminal is required to read the password

    A grant the tools already depend on belongs in the payload."""
    sudoers = (PAYLOAD / "etc" / "sudoers.d" / "roomscan-bridge").read_text()
    rules = [ln for ln in sudoers.splitlines()
             if ln.strip() and not ln.strip().startswith("#")]
    assert len(rules) == 1, rules
    assert "NOPASSWD" in rules[0]

    # A malformed sudoers fragment locks root out of the machine entirely, so
    # the installer must validate it rather than trust the render.
    text = (PAYLOAD / "install.sh").read_text()
    assert "visudo -c" in text
    assert "sshd -t" in text or '"${sshd_bin}" -t' in text, \
        "an sshd drop-in must be validated too; a bad one is unrecoverable remotely"


def test_install_sh_checks_units_are_still_alive_not_just_that_start_returned():
    """The defect that let every other one reach the operator as 'it worked'.

    `systemctl enable --now` exits 0 when the START JOB succeeded, not when the
    service survived. install.sh believed that exit code, logged `completed
    successfully`, and exited 0 while TWO of its three REQUIRED units (dnsmasq,
    avahi-daemon) were crash-looping. REQUIRED_UNITS exists precisely to fail
    the install in that case and it silently did not."""
    text = (PAYLOAD / "install.sh").read_text()
    assert "verify_unit_active" in text
    assert "is-active" in text
    # Sampling immediately would catch a unit mid-backoff and read `activating`
    # as healthy, so there must be a settle before the verdict.
    assert "UNIT_SETTLE_SECS" in text
    verify_body = text.split("verify_unit_active()", 1)[1].split("\n}", 1)[0]
    assert '"activating"' not in verify_body.replace("'", '"').split("return 0")[0], \
        "`activating` is also what a crash-looping unit looks like mid-backoff"


def test_the_journal_survives_the_reboot_that_needs_explaining():
    """Raspberry Pi OS ships `Storage=volatile`: the journal lives only in
    /run/log/journal and is destroyed on every boot. (/var/log/journal exists on
    the stock image but is unused, which makes this easy to misread as already
    persistent.)

    The bridge's signature failure is "it stopped being reachable", and the only
    evidence is the journal of the boot that failed. On 2026-08-18 a
    `systemctl reboot` left the Pi off the network entirely; recovering it took a
    power cycle, which destroyed the sole copy of the log that could have
    explained it. That failure is still unexplained (issue #191)."""
    conf = (PAYLOAD / "etc" / "systemd" / "journald.conf.d"
            / "roomscan-bridge.conf").read_text()
    flat = conf.replace(" ", "")
    assert "Storage=persistent" in flat
    # Bounded, or the SD card becomes the next problem.
    assert "SystemMaxUse=" in flat


def test_nmcli_is_never_called_without_a_bounded_wait():
    """`nmcli con up`/`nmcli device connect` block up to 90 s by default, and
    both are reached from reconcile, which runs on a 10 s timer. systemd
    serialises the service so this does not stack processes -- but reconcile
    is what notices a scanner stuck in fallback mode (and, since issue #200,
    a wlan0 stuck in NetworkManager's "no secrets" dead end), and a 90 s
    stall is 90 s of not noticing either."""
    for path in ("usr/local/sbin/roomscan-bridge-common.sh",
                 "usr/local/sbin/roomscan-bridge-reconcile"):
        text = (PAYLOAD / path).read_text()
        for ln in text.splitlines():
            s = ln.strip()
            if s.startswith("#") or s.startswith("log ") or "nmcli" not in s:
                continue
            if " con up" in s or " connection up" in s or " device connect" in s:
                assert "--wait" in s or "-w " in s, \
                    f"unbounded nmcli activation in {path}: {s}"


def test_reconcile_self_heals_a_disconnected_wlan0():
    """Issue #200: NetworkManager can reach a permanent wlan0 dead-end (a WPA
    handshake IE mismatch makes it ask for secrets it already has, on a
    headless box with no agent to answer, so activation fails with
    "no secrets" and wlan0 sits `disconnected` forever). Confirmed live that
    `nmcli device connect wlan0` recovers it instantly on the stored
    credentials -- nothing else in this stack ever retries it, so reconcile
    must, on every 10 s pass."""
    rec = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-reconcile").read_text()
    assert "ensure_wlan0_connected()" in rec, \
        "reconcile has no wlan0 self-heal function"
    assert "nmcli" in rec and "device connect wlan0" in rec
    # And main() must actually call it, not just define it.
    main_body = rec.split("main() {", 1)[1].split("\n}", 1)[0]
    assert "ensure_wlan0_connected" in main_body, \
        "ensure_wlan0_connected is defined but never called from main()"


def test_every_path_that_pokes_eth0_reasserts_its_static_address():
    """On the real Pi, eth0 came up with NO IPv4 address at all, so dnsmasq had
    nothing to serve DHCP from and the scanner ALWAYS missed its 3000 ms window
    and self-assigned. NetworkManager had the device `connected (externally)`,
    which stops it autoconnecting our profile -- and the thing that puts it in
    that state is us: reconcile adds a probe alias with `ip addr add` and
    bounces the link with `ip link set eth0 down/up`. Poking a NM-managed device
    with iproute2 costs you the address NM was supposed to provide (issue #191).

    So every path that touches eth0 directly must re-assert it afterwards."""
    common = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-common.sh").read_text()
    assert "roomscan_ensure_eth0_address()" in common

    rec = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-reconcile").read_text()
    live = [ln for ln in rec.splitlines() if not ln.strip().startswith("#")]
    pokes = [i for i, ln in enumerate(live)
             if ("ip addr add" in ln or "ip addr del" in ln or "ip link set eth0" in ln)]
    assert pokes, "expected reconcile to manipulate eth0 directly"
    reasserts = [i for i, ln in enumerate(live) if "roomscan_ensure_eth0_address" in ln]
    assert reasserts, "reconcile pokes eth0 but never re-asserts its address"
    # Every poke must be followed by a re-assert somewhere after it.
    assert max(reasserts) > max(pokes), \
        "the last eth0 manipulation is not followed by a re-assert"

    # And a freshly provisioned box must converge without waiting for a tick.
    assert "roomscan_ensure_eth0_address" in (PAYLOAD / "install.sh").read_text()


def test_the_live_stream_guard_tests_a_rate_not_any_increase():
    """The guard that stops reconcile bouncing eth0 mid-capture latched forever
    on the real rig. It tested `after > before`, so a single byte counted as a
    live capture -- and the host's roomscan-web broadcasts a 1-byte discovery
    beacon to 255.255.255.255:5000 once a second, which the DNAT rule matches.
    The counter ticked ~25 B/s with no scanner traffic at all, reconcile read
    "capture in progress" on every pass, and so never performed the one action
    that recovers a scanner stuck in fallback mode (issue #191).

    A real stream is ~466 KB/s, four orders of magnitude above the beacon, so
    any sane threshold separates them."""
    for path in ("usr/local/sbin/roomscan-bridge-common.sh", "install.sh"):
        text = (PAYLOAD / path).read_text()
        assert "STREAM_LIVE_MIN_BYTES_PER_SEC" in text, path
        live = [ln for ln in text.splitlines() if not ln.strip().startswith("#")]
        # The any-increase comparison must be gone from both copies -- they are
        # mirrored implementations and a fix to one is not a fix to the other.
        assert not [ln for ln in live
                    if '"${after}" -gt "${before}"' in ln], \
            f"{path} still treats any counter increase as a live stream"


def test_install_sh_clears_a_latched_start_limit_before_restarting():
    """A unit that burned its StartLimitBurst answers every later start with
    `Start request repeated too quickly` and never runs the new code -- so
    pushing the very fix for its crash appears to change nothing. Observed on
    the real Pi: roomscan-tee's fix landed and the unit stayed dead, still
    rate-limited from the failures the fix addressed. `bridge_update()` exists
    to repair a broken box, and a broken box is where the latch is set."""
    # Match live code only. An earlier version of this test compared against the
    # first textual `enable --now`, which lives in a COMMENT explaining the
    # unit-liveness check, and so failed on correct code -- the same
    # scope-your-match trap the repo has been bitten by before.
    lines = [ln for ln in (PAYLOAD / "install.sh").read_text().splitlines()
             if not ln.strip().startswith("#")]
    reset = [i for i, ln in enumerate(lines) if "reset-failed" in ln]
    start = [i for i, ln in enumerate(lines)
             if "enable --now" in ln or "systemctl restart" in ln]
    assert reset, "install.sh must clear the start-rate-limit latch"
    assert start
    assert min(reset) < min(start), \
        "the latch must be cleared BEFORE the start attempt, or it changes nothing"


def test_dnsmasq_survives_an_eth0_that_has_no_address_yet():
    """eth0 is a point-to-point link to a scanner that is unplugged most of the
    time, and NM addresses it asynchronously after boot regardless.
    `bind-interfaces` resolves the interface ONCE at startup and treats an
    addressless one as fatal -- so dnsmasq crash-looped every 11 s from first
    boot with `unknown interface eth0`, and would not have recovered when the
    cable went in. `bind-dynamic` binds when the interface actually appears,
    while keeping dnsmasq off the Wi-Fi side."""
    conf = (PAYLOAD / "etc" / "dnsmasq.d" / "roomscan-bridge.conf").read_text()
    active = [ln.strip() for ln in conf.splitlines()
              if ln.strip() and not ln.strip().startswith("#")]
    assert "bind-dynamic" in active
    assert "bind-interfaces" not in active, \
        "bind-interfaces cannot survive a cable that is not plugged in at boot"
    assert "interface=eth0" in active, "dnsmasq must still never answer on wlan0"


def test_the_tee_runs_as_the_user_that_must_write_its_ring():
    """tcpdump drops privileges to the unprivileged `tcpdump` user immediately
    after opening its socket -- that is its security model, not a flag. Run as
    root, the post-drop process could not create its own ring files:

        tcpdump: /var/lib/roomscan-bridge/tee/ring.pcap00: Permission denied

    which crash-looped roomscan-tee every 5 s from first boot.

    Chowning the directory does NOT fix it, and the failure is silent: measured
    on the real Pi, `install -d -o tcpdump` succeeds and the very next
    `systemctl restart` shows the directory back at root:root 0755, because
    StateDirectory= re-asserts ownership matching User= on every start. Declaring
    the owner is the fix; fighting systemd for it is not."""
    import configparser
    cp = configparser.ConfigParser(strict=False, allow_no_value=True)
    cp.optionxform = str
    cp.read_string((PAYLOAD / "etc" / "systemd" / "system"
                    / "roomscan-tee.service").read_text())
    svc = cp["Service"]
    assert svc.get("User") == "tcpdump", "the unit must run as the writing user"
    # Running as tcpdump costs the root privileges tcpdump needed to open the
    # interface; the ambient capabilities are what replace them.
    assert "CAP_NET_RAW" in svc.get("AmbientCapabilities", "")
    # An unbounded retry of a structural failure is journal churn that never
    # escalates; bound it so a dead tee is visible as `failed`.
    assert cp["Unit"].get("StartLimitBurst"), "the restart loop must be bounded"

    # And no chown may creep back into install.sh: it reads like a safeguard
    # while being a guaranteed no-op.
    install = (PAYLOAD / "install.sh").read_text()
    live = [ln for ln in install.splitlines() if not ln.strip().startswith("#")]
    assert not [ln for ln in live if "install -d" in ln and "tee" in ln], \
        "StateDirectory= undoes this before ExecStart runs"


def test_avahi_does_not_enforce_the_rlimits_debian_ships_commented_out():
    """avahi-daemon crash-looped on the first real Pi with `Out of memory,
    aborting` ~1 s into every startup, on a box with 790 MB free. The malloc was
    failing against an RLIMIT we imposed, not against the machine.

    Debian ships the whole `[rlimits]` block **commented out** -- those values
    are upstream's example, not the running config. Copying them into a derived
    file and dropping the `#` switches on caps nothing on Debian exercises,
    while the file still reads as 'stock, plus one change'. Same family as
    VL53L9_TRANSFORM_LIGHT and `enableOptionalEffects`: read what the vendor
    *applies*, not what its file contains."""
    conf = (PAYLOAD / "etc" / "avahi" / "avahi-daemon.conf").read_text()
    active = [ln.strip() for ln in conf.splitlines()
              if ln.strip() and not ln.strip().startswith("#")]
    offenders = [ln for ln in active if ln.startswith("rlimit-")]
    assert not offenders, f"uncommented avahi rlimits: {offenders}"


def test_status_probes_report_unknown_rather_than_a_confident_negative():
    """`iw` and `nft` live in /usr/sbin, which is not on a non-root login PATH --
    and bridge_status runs over ssh as the unprivileged account. Every probe died
    with `command not found`, and the script turned that into readings: `wlan0
    connected: false, ssid: null` for an associated interface holding a lease,
    and `nft rules: []` for a loaded ruleset. An absent answer that reads as a
    real negative sends the next session chasing a fault that does not exist."""
    text = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-status").read_text()
    assert "/usr/sbin" in text.split("SELF_DIR", 1)[0], "PATH must be fixed before any probe"
    assert 'NFT_OK="no"' in text
    assert '"readable"' in text
    # nft_get sets globals; calling it in a subshell would discard the flag it
    # exists to set, leaving `readable` false forever.
    assert 'NFT_RAW="$(nft_get)"' not in text, \
        "command substitution runs nft_get in a subshell and discards NFT_OK"
    # The tri-state: "" (unknown) must be reachable, not just yes/no.
    assert 'WLAN0_CONNECTED=""' in text
    assert "NRestarts" in text, \
        "a crash-looping unit reads as `activating`; the restart count separates them"


def test_install_sh_verifies_dnsmasq_actually_reads_the_dropin():
    """Debian reads /etc/dnsmasq.d via `CONFIG_DIR` in /etc/default/dnsmasq and
    the helper's `-7` flag -- *not* via /etc/dnsmasq.conf, where every
    `conf-dir=` line ships commented out. We depend entirely on a mechanism we
    do not own, and its failure is silent: dnsmasq starts, units report active,
    and the scanner just never gets a lease. So install.sh checks."""
    text = (PAYLOAD / "install.sh").read_text()
    assert "CONFIG_DIR" in text
    assert "conf-dir" in text
    assert "ensure_dnsmasq_reads_dropins" in text


def test_payload_tar_is_deterministic_and_root_owned(rendered, tmp_path):
    import tarfile
    a = bi.make_payload_tar(rendered, tmp_path / "a.tar.gz")
    b = bi.make_payload_tar(rendered, tmp_path / "b.tar.gz")
    with tarfile.open(a) as ta:
        members = {m.name: m for m in ta.getmembers()}
    assert "install.sh" in members
    assert members["install.sh"].mode & 0o111, "install.sh must stay executable in the tar"
    for m in members.values():
        assert m.uid == 0 and m.uname == "root"
    assert a.read_bytes() == b.read_bytes(), "tar is not reproducible"


# ---------------------------------------------------------------------------
# Debian dependency closure
# ---------------------------------------------------------------------------

CANNED_STATUS = """\
Package: libc6
Status: install ok installed
Version: 2.36-9

Package: base-files
Status: install ok installed
Version: 12.4

Package: half-installed-thing
Status: install ok half-configured
Version: 1.0

Package: awk-provider
Status: install ok installed
Provides: awk
Version: 1.0
"""

CANNED_PACKAGES = """\
Package: dnsmasq
Version: 2.89-1
Depends: dnsmasq-base (>= 2.89), netbase, awk
Filename: pool/main/d/dnsmasq/dnsmasq_2.89-1_all.deb
SHA256: aaaa
Size: 1000

Package: dnsmasq-base
Version: 2.89-1
Depends: libc6 (>= 2.34), libnetfilter-conntrack3
Filename: pool/main/d/dnsmasq/dnsmasq-base_2.89-1_armhf.deb
SHA256: bbbb
Size: 2000

Package: libnetfilter-conntrack3
Version: 1.0.9-3
Depends: libc6, libmnl0 | libmnl-alt
Filename: pool/main/l/libnetfilter/libnetfilter-conntrack3_1.0.9-3_armhf.deb
SHA256: cccc
Size: 3000

Package: libmnl0
Version: 1.0.4-3
Depends: libc6
Filename: pool/main/l/libmnl/libmnl0_1.0.4-3_armhf.deb
SHA256: dddd
Size: 4000

Package: netbase
Version: 6.4
Filename: pool/main/n/netbase/netbase_6.4_all.deb
SHA256: eeee
Size: 500

Package: half-installed-thing
Version: 2.0
Filename: pool/main/h/half/half_2.0_all.deb
SHA256: ffff
Size: 100
"""


@pytest.fixture(scope="module")
def canned():
    by_name, provides = bi.index_packages([CANNED_PACKAGES])
    return by_name, provides, bi.installed_set(CANNED_STATUS)


def test_installed_set_counts_only_fully_installed_packages_and_their_provides():
    have = bi.installed_set(CANNED_STATUS)
    assert "libc6" in have and "base-files" in have
    assert "awk" in have, "a Provides: line satisfies a dependency"
    # Half-configured is NOT installed; treating it as installed would omit a
    # .deb the Pi then cannot fetch, on a first boot with no internet.
    assert "half-installed-thing" not in have


def test_resolve_closure_walks_dependencies_and_skips_what_is_installed(canned):
    by_name, provides, installed = canned
    got = bi.resolve_closure(["dnsmasq"], by_name, provides, installed)
    assert not got["unsatisfied"]
    pkgs = got["packages"]
    assert set(pkgs) == {"dnsmasq", "dnsmasq-base", "libnetfilter-conntrack3",
                         "libmnl0", "netbase"}
    assert "libc6" not in pkgs, "already installed in the base image"
    assert "awk" not in pkgs, "satisfied by a Provides"
    # Dependencies must precede their dependents, so `dpkg -i` in this order
    # succeeds without a configure pass.
    assert pkgs.index("dnsmasq-base") < pkgs.index("dnsmasq")
    assert pkgs.index("libmnl0") < pkgs.index("libnetfilter-conntrack3")


def test_resolve_closure_picks_the_first_available_alternative(canned):
    by_name, provides, installed = canned
    got = bi.resolve_closure(["libnetfilter-conntrack3"], by_name, provides, installed)
    assert "libmnl0" in got["packages"]
    assert "libmnl-alt" not in got["packages"]


def test_resolve_closure_reports_what_it_could_not_satisfy(canned):
    by_name, provides, installed = canned
    got = bi.resolve_closure(["not-in-any-archive"], by_name, provides, installed)
    assert got["unsatisfied"] == ["not-in-any-archive"]
    assert got["packages"] == []


def test_resolve_closure_reports_already_installed_requests(canned):
    by_name, provides, installed = canned
    got = bi.resolve_closure(["libc6"], by_name, provides, installed)
    assert got["already"] == ["libc6"] and got["packages"] == []


def test_relation_parsing_strips_versions_arches_and_build_profiles():
    assert bi._strip_relation("libc6 (>= 2.34) [!armel] <!nocheck>") == "libc6"
    assert bi._strip_relation("python3:any") == "python3"
    assert bi._split_relations("a (>=1) | b, c") == [["a", "b"], ["c"]]
    assert bi._split_relations("") == []


def test_control_parser_folds_continuation_lines():
    paras = bi.parse_control("Package: x\nDepends: a,\n b\n\nPackage: y\n")
    assert len(paras) == 2
    assert bi._split_relations(paras[0]["Depends"]) == [["a"], ["b"]]


# ---------------------------------------------------------------------------
# Mini-image round trip (the injection stage, end to end)
# ---------------------------------------------------------------------------

@has_mtools
def test_mini_image_round_trip(tmp_path, rendered):
    """Build a synthetic MBR + FAT image, run the real injection stage against
    it, and read every artefact back out with mtools.

    This is the closest thing to a real build that runs without a Pi, without
    root and without a 1.3 GB download: same MBR parser, same `mcopy` calls,
    same cmdline edit.
    """
    img = tmp_path / "mini.img"
    boot_lba, boot_sectors = 2048, 65536          # 1 MiB in, 32 MiB FAT
    root_lba, root_sectors = boot_lba + boot_sectors, 2048
    total = (root_lba + root_sectors) * bi.SECTOR
    with img.open("wb") as f:
        f.truncate(total)
        f.seek(0)
        f.write(_mbr([(True, 0x0C, boot_lba, boot_sectors),
                      (False, 0x83, root_lba, root_sectors)]))

    parts = bi.read_mbr(img)
    boot = bi.boot_partition(parts)
    assert boot.offset == boot_lba * bi.SECTOR

    env = dict(os.environ, MTOOLS_SKIP_CHECK="1")
    r = subprocess.run(["mformat", "-i", f"{img}@@{boot.offset}", "-T", str(boot_sectors),
                        "-v", "bootfs", "::"], capture_output=True, text=True, env=env)
    assert r.returncode == 0, r.stderr

    # Seed a realistic base cmdline, exactly as the released image ships one.
    seed = tmp_path / "cmdline.txt"
    seed.write_text(REAL_CMDLINE + "\n")
    bi.fat_write(img, boot.offset, seed, "/cmdline.txt")

    # --- the injection stage ------------------------------------------------
    firstrun = tmp_path / "firstrun.sh"
    firstrun.write_bytes(bi._lf((BOOT / "firstrun.sh").read_text()).encode())
    bi.fat_write(img, boot.offset, firstrun, "/firstrun.sh")

    tar = bi.make_payload_tar(rendered, tmp_path / bi.PAYLOAD_TAR_NAME)
    bi.fat_write(img, boot.offset, tar, f"/{bi.PAYLOAD_TAR_NAME}")

    uc = tmp_path / "userconf.txt"
    uc.write_text(bi.userconf_line("roomscan", "test-password"))
    bi.fat_write(img, boot.offset, uc, "/userconf.txt")

    flag = tmp_path / "ssh"
    flag.write_text("")
    bi.fat_write(img, boot.offset, flag, "/ssh")

    cmdline = bi.fat_read(img, boot.offset, "/cmdline.txt").decode()
    patched = tmp_path / "patched.txt"
    patched.write_text(bi.append_firstrun_args(cmdline))
    bi.fat_write(img, boot.offset, patched, "/cmdline.txt")

    # --- read everything back OUT of the image ------------------------------
    listing = {Path(n).name for n in bi.fat_list(img, boot.offset)}
    assert {"firstrun.sh", bi.PAYLOAD_TAR_NAME, "userconf.txt", "ssh",
            "cmdline.txt"} <= listing, listing

    got_cmdline = bi.fat_read(img, boot.offset, "/cmdline.txt").decode()
    assert got_cmdline.count("\n") == 1, "cmdline.txt must stay a single line"
    for arg in bi.FIRSTRUN_ARGS:
        assert arg in got_cmdline.split()
    assert "root=PARTUUID=2f8c1a4d-02" in got_cmdline, "the base cmdline was rewritten"

    got_firstrun = bi.fat_read(img, boot.offset, "/firstrun.sh")
    assert b"\r\n" not in got_firstrun
    assert got_firstrun.startswith(b"#!")

    got_userconf = bi.fat_read(img, boot.offset, "/userconf.txt").decode()
    assert got_userconf.startswith("roomscan:$6$"), got_userconf[:40]

    # The tarball must survive the FAT round trip byte for byte -- it is the
    # entire install medium.
    assert bi.fat_read(img, boot.offset, f"/{bi.PAYLOAD_TAR_NAME}") == tar.read_bytes()


@has_mtools
def test_mini_image_injection_is_idempotent(tmp_path):
    """A second build over the same card must not double the cmdline args."""
    img = tmp_path / "mini.img"
    boot_lba, boot_sectors = 2048, 65536
    with img.open("wb") as f:
        f.truncate((boot_lba + boot_sectors) * bi.SECTOR)
        f.seek(0)
        f.write(_mbr([(True, 0x0C, boot_lba, boot_sectors)]))
    off = boot_lba * bi.SECTOR
    env = dict(os.environ, MTOOLS_SKIP_CHECK="1")
    subprocess.run(["mformat", "-i", f"{img}@@{off}", "-T", str(boot_sectors), "::"],
                   capture_output=True, check=True, env=env)
    seed = tmp_path / "c.txt"
    seed.write_text(REAL_CMDLINE + "\n")
    bi.fat_write(img, off, seed, "/cmdline.txt")

    for _ in range(2):
        cur = bi.fat_read(img, off, "/cmdline.txt").decode()
        p = tmp_path / "p.txt"
        p.write_text(bi.append_firstrun_args(cur))
        bi.fat_write(img, off, p, "/cmdline.txt")

    final = bi.fat_read(img, off, "/cmdline.txt").decode()
    for arg in bi.FIRSTRUN_ARGS:
        assert final.split().count(arg) == 1, final


def test_pinned_releases_declare_a_full_sha256_and_a_matching_suite():
    for key, rel in bi.RELEASES.items():
        assert len(rel.sha256) == 64 and all(c in "0123456789abcdef" for c in rel.sha256)
        assert rel.suite in rel.filename, f"{key}: filename and suite disagree"
        assert rel.url.endswith(rel.filename)
        assert rel.url.startswith("https://"), "the pin must be fetched over TLS"
    assert bi.DEFAULT_RELEASE in bi.RELEASES
