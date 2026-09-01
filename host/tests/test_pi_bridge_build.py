"""Tests for the rootless Pi bridge SD-image builder (issue #191).

None of this needs a Raspberry Pi, and none of it needs root -- which is the
whole point of the builder's design. What it cannot cover is the one thing only
a real first boot proves (that Raspberry Pi OS's `systemd.run` firstrun
mechanism fires and `install.sh` completes on the target); that stays an
operator gate, backed by firstrun.log landing on the FAT partition.
"""
from __future__ import annotations

import importlib.util
import json
import lzma
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


# Importing a payload module writes __pycache__ INTO the payload tree, which
# install.sh would then try to ship to /usr/local/sbin. Off for this session.
sys.dont_write_bytecode = True


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


def test_update_enables_new_optional_units_for_the_next_boot():
    """Starting a newly deployed timer is not the same as installing it.

    Issue #200's health logger was added through ``bridge_update()``.  The
    update branch restarted its timer, so samples appeared immediately, but it
    never enabled the unit.  The next cold boot therefore left the timer
    disabled and inactive.  Keep an explicit enable in the update branch; a
    successful restart only proves the current boot.
    """
    text = (PAYLOAD / "install.sh").read_text()
    optional_loop = text.split(
        'for unit in "${OPTIONAL_UNITS[@]}"; do', 1
    )[1].split("\n    done", 1)[0]
    update_branch = optional_loop.split("else", 1)[1]
    assert 'systemctl enable "${unit}"' in update_branch, \
        "bridge_update starts new optional units but leaves them disabled at boot"


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


def test_compress_xz_roundtrip(tmp_path):
    img = tmp_path / "test.img"
    data = b"pi bridge disk image payload test data " * 500
    img.write_bytes(data)
    out = bi.compress_xz(img, quiet=True)
    assert Path(out).is_file()
    assert img.is_file()  # -k keeps original
    with lzma.open(out, "rb") as f:
        assert f.read() == data


def test_compress_xz_fallback_when_no_xz(tmp_path, monkeypatch):
    monkeypatch.setattr(shutil, "which", lambda cmd: None)
    img = tmp_path / "test_fallback.img"
    data = b"fallback lzma compression test data " * 500
    img.write_bytes(data)
    out = bi.compress_xz(img, quiet=True)
    assert Path(out).is_file()
    assert img.is_file()
    with lzma.open(out, "rb") as f:
        assert f.read() == data



# ---------------------------------------------------------------------------
# Camera payload (issue #226)
# ---------------------------------------------------------------------------

CAM_UNIT = "roomscan-cam.service"


def _cam_unit(rendered):
    import configparser
    cp = configparser.ConfigParser(strict=False, allow_no_value=True)
    cp.optionxform = str
    cp.read_string((rendered / "etc" / "systemd" / "system" / CAM_UNIT).read_text())
    return cp


def test_the_camera_unit_is_optional_and_never_a_required_one():
    """A bridge with no camera fitted is a healthy bridge.

    `roomscan-cam` sits in OPTIONAL_UNITS for the same structural reason
    `roomscan-tee` does: the scanner data path is kernel forwarding plus the
    nftables rules and does not consult it. Promoting it to REQUIRED_UNITS
    would make an unplugged webcam fail the whole install.
    """
    text = (PAYLOAD / "install.sh").read_text()
    optional = text.split("OPTIONAL_UNITS=(", 1)[1].split("\n)", 1)[0]
    required = text.split("REQUIRED_UNITS=(", 1)[1].split("\n)", 1)[0]
    assert CAM_UNIT in optional
    assert CAM_UNIT not in required


def test_the_camera_unit_does_not_hide_the_devices_it_exists_to_read():
    """`PrivateDevices=yes` gives a unit a /dev with no `video*` in it.

    It is the obvious-looking hardening line to add and it would break this
    service completely -- and silently, because the daemon's own failure mode
    for a missing node is `device_absent`, which reads as "no camera plugged
    in". The correct pairing is a closed device policy plus one explicit
    allow, so assert BOTH halves: the trap is absent, the allow is present.
    """
    raw = (PAYLOAD / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    directives = [line.split("#", 1)[0].strip() for line in raw.splitlines()]
    assert not [d for d in directives if d.startswith("PrivateDevices=")], \
        "PrivateDevices= would hide /dev/video* from the camera service"
    assert "DevicePolicy=closed" in directives
    assert "DeviceAllow=char-video4linux rw" in directives


def test_the_camera_unit_runs_unprivileged_with_video_access_only(rendered):
    cp = _cam_unit(rendered)
    service = cp["Service"]
    assert service["DynamicUser"] == "yes"
    assert service["SupplementaryGroups"] == "video"
    assert service["NoNewPrivileges"] == "yes"
    # An empty bounding set is the strongest statement available: this service
    # needs no capability at all -- the C270 nodes are root:video 0660 and the
    # port is above 1024.
    assert service["CapabilityBoundingSet"] == ""
    assert service["ProtectSystem"] == "strict"
    # Bounded rather than best-effort: a leak on a 920 MB box must become a
    # restart, not an OOM that takes the bridge with it.
    assert service["MemoryMax"] == "256M"
    assert int(service["TasksMax"]) <= 64
    assert int(service["LimitNOFILE"]) <= 1024


def test_the_camera_unit_bounds_its_own_restart_loop(rendered):
    cp = _cam_unit(rendered)
    assert cp["Service"]["Restart"] == "on-failure"
    assert int(cp["Unit"]["StartLimitBurst"]) <= 5
    assert int(cp["Unit"]["StartLimitIntervalSec"]) >= 60


def test_the_camera_unit_ships_the_control_the_measurement_required(rendered):
    """Measured on the real bridge 2026-09-01: with
    `exposure_dynamic_framerate=1` the C270 delivers 14.7 fps at 720p AND at
    480p -- the resolution is not the variable, the control is. Shipping the
    unit without this line would advertise a 30 fps mode nothing supports."""
    raw = (rendered / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    assert "ROOMSCAN_CAM_CONTROLS=exposure_dynamic_framerate=0" in raw
    assert "ROOMSCAN_CAM_MODE=1280x720@30" in raw
    assert "ROOMSCAN_CAM_IFACE=wlan0" in raw


def test_the_camera_port_is_one_token_shared_by_the_unit_and_the_firewall():
    """Two literals for one port drift; the nft rule would then guard a port
    nothing listens on, and read as a working firewall."""
    unit = (PAYLOAD / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    nft = (PAYLOAD / "etc" / "nftables" / "roomscan-bridge.nft").read_text()
    assert "ROOMSCAN_CAM_PORT={{CAM_PORT}}" in unit
    assert nft.count("{{CAM_PORT}}") == 2
    assert bi.DEFAULT_NETWORK["CAM_PORT"] == "5001"
    assert "CAM_PORT" in bi.NODE_ENV_KEYS


def test_the_camera_port_is_scoped_to_wlan0_and_counted(rendered):
    """The scanner-facing side must not reach the camera feed, and the rule
    that says so must COUNT -- a silent drop rule and an absent one look the
    same from outside. Policy stays `accept`: turning on input filtering for
    the whole box would be a firewall change smuggled in with a feature."""
    nft = (rendered / "etc" / "nftables" / "roomscan-bridge.nft").read_text()
    chain = nft.split("chain input {", 1)[1].split("\n\t}", 1)[0]
    assert "policy accept" in chain
    deny = [ln for ln in chain.splitlines() if "roomscan-cam-deny-eth0" in ln]
    allow = [ln for ln in chain.splitlines() if "roomscan-cam-allow-wlan0" in ln]
    assert len(deny) == 1 and len(allow) == 1
    assert 'iifname "eth0"' in deny[0] and "counter drop" in deny[0]
    assert 'iifname "wlan0"' in allow[0] and "counter accept" in allow[0]
    assert "tcp dport 5001" in deny[0] and "tcp dport 5001" in allow[0]


def test_reconcile_restarts_the_camera_only_when_a_camera_is_present():
    """Asymmetric on purpose. A bridge with no camera fitted has nothing to
    heal, and restarting on every 10 s pass would burn the unit's
    StartLimitBurst on a non-problem -- leaving it dead for the moment a
    camera IS plugged in, which is precisely when it is wanted."""
    text = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-reconcile").read_text()
    body = text.split("ensure_camera_service() {", 1)[1].split("\n}", 1)[0]
    assert "camera_present" in body
    assert "046d:0825" in text, "the camera is identified by USB id, not /dev/videoN"
    # Same latched-start-limit clear the tee needed in #191.
    assert "reset-failed roomscan-cam.service" in body
    assert "ensure_camera_service" in text.split("main() {", 1)[1]


def test_status_separates_no_camera_fitted_from_camera_service_broken():
    """Collapsing those two is how an unplugged webcam becomes a bridge alarm
    -- and how a genuinely dead daemon becomes invisible. Same class as the
    `iw`-missing bug in #191: an absent answer must not read as a real
    negative."""
    text = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-status").read_text()
    assert "CAM_USB_PRESENT" in text and "CAM_STATUS_RAW" in text
    assert '"usb_present": yesno(cam_usb_present)' in text
    assert '"daemon": cam_daemon' in text
    assert '"daemon_reachable"' in text
    assert '"warning": cam_warning' in text
    # The daemon's own status.json is passed through, not re-derived: a
    # re-derived label diverges from its source the moment either side moves.
    assert "cam_daemon = json.loads(cam_status_raw)" in text
    assert "roomscan-cam.service" in text.split("UNITS_LIST=", 1)[1].splitlines()[0]


def _payload_python() -> list[Path]:
    """Payload files that the Pi's python3 has to be able to run."""
    out = []
    for p in _payload_files():
        head = p.read_bytes()[:64]
        if p.suffix == ".py" or (head.startswith(b"#!") and b"python" in head):
            out.append(p)
    return out


def test_every_payload_python_file_compiles_and_is_executable(tmp_path):
    import py_compile
    files = _payload_python()
    assert len(files) >= 2, f"expected roomscan-cam + its v4l2 module, found {files}"
    for p in files:
        # cfile into tmp_path, never beside the source: a .pyc left in the
        # payload is a file install.sh would ship to the Pi.
        py_compile.compile(str(p), cfile=str(tmp_path / (p.name + "c")),
                           doraise=True)
        assert os.access(p, os.X_OK), f"{p.relative_to(PI)} is not executable"


def test_the_payload_carries_no_build_artifacts():
    """Everything in `usr/local/sbin/` is installed verbatim onto the Pi, so a
    stray `__pycache__` from a test run would be shipped as a system tool."""
    strays = [str(p.relative_to(PI)) for p in _payload_files()
              if "__pycache__" in p.parts or p.suffix in (".pyc", ".pyo")]
    assert not strays, strays


def test_the_daemon_and_the_characterization_tool_share_one_v4l2_layer():
    """One copy, not two. When two tools disagree about the same bytes the
    tool is usually the bug, and a second transcription of `struct
    v4l2_buffer` offsets is a guaranteed future disagreement."""
    module = PAYLOAD / "usr" / "local" / "sbin" / "roomscan_cam_v4l2.py"
    assert module.is_file()
    assert not (PI / "tools" / "v4l2_capture.py").exists(), \
        "the V4L2 layer was duplicated back into pi-bridge/tools/"
    tool = (PI / "tools" / "cam_characterize.py").read_text()
    assert "import roomscan_cam_v4l2 as v4l2" in tool
    daemon = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-cam").read_text()
    assert "import roomscan_cam_v4l2 as v4l2" in daemon


def test_the_v4l2_layer_probes_the_buffer_layout_instead_of_deriving_it():
    """Raspbian 13 armhf is a 64-bit-`time_t` port, so `struct v4l2_buffer` is
    80 bytes and `VIDIOC_DQBUF` is a different ioctl NUMBER than the 68-byte
    TIME32 one an armv7l guess would pick. The kernel accepts both; only one
    matches what this process packs. Measured on the real bridge: `time64`."""
    module = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan_cam_v4l2.py").read_text()
    assert "_probe_layout" in module
    import importlib.util
    spec = importlib.util.spec_from_file_location(
        "roomscan_cam_v4l2_under_test",
        PAYLOAD / "usr" / "local" / "sbin" / "roomscan_cam_v4l2.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    sizes = sorted(layout.size for layout in mod.BUFFER_LAYOUTS)
    assert sizes == [68, 80], sizes
    # Distinct ioctl numbers is the whole reason a probe is possible.
    numbers = {layout.DQBUF for layout in mod.BUFFER_LAYOUTS}
    assert len(numbers) == len(mod.BUFFER_LAYOUTS)


def test_the_measured_characterization_result_is_committed_beside_the_daemon():
    """The spec forbids writing the daemon before the numbers exist. Keeping
    the evidence in the tree is what makes that checkable a year from now."""
    results = PI / "tools" / "cam_characterize_results_20260901.md"
    assert results.is_file()
    text = results.read_text()
    assert "exposure_dynamic_framerate" in text
    # The measured cadence, not the requested one -- if this file ever stops
    # carrying a real number the daemon's default has lost its evidence.
    assert "29.7" in text and "14.7" in text
    raw = PI / "tools" / "cam_characterize_results_20260901"
    assert (raw / "B_preview_pi.json").is_file()
    assert (raw / "A_baseline_pi.json").is_file()


# ---------------------------------------------------------------------------
# Camera payload, review round 2 (issue #226)
# ---------------------------------------------------------------------------

#: systemd's symbolic IPAddressAllow names, expanded so the prefixes can be
#: reasoned about numerically. From systemd.resource-control(5).
_SYMBOLIC_PREFIXES = {
    "localhost": ["127.0.0.0/8", "::1/128"],
    "link-local": ["169.254.0.0/16", "fe80::/64"],
    "multicast": ["224.0.0.0/4", "ff00::/8"],
    "any": ["0.0.0.0/0", "::/0"],
}


def _ip_allow_networks(rendered):
    import ipaddress
    raw = (rendered / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    tokens = []
    for line in raw.splitlines():
        if line.startswith("IPAddressAllow="):
            tokens += line.split("=", 1)[1].split()
    assert tokens, "the unit declares no IPAddressAllow at all"
    nets = []
    for token in tokens:
        for prefix in _SYMBOLIC_PREFIXES.get(token, [token]):
            nets.append(ipaddress.ip_network(prefix))
    return nets


def test_the_camera_unit_ip_filter_excludes_the_scanner_segment(rendered):
    """The systemd layer must agree with the bind address and the nft rule.

    A plain `172.16.0.0/12` covers `172.31.100.0/24` -- so the unit would have
    permitted traffic to the scanner segment while its own comment claimed it
    was "not reachable from the scanner segment", and it would have looked
    hardened while doing it. Three layers that disagree are worse than one,
    because each is read as corroborating the others.
    """
    import ipaddress
    nets = _ip_allow_networks(rendered)
    for forbidden in (bi.DEFAULT_NETWORK["SCANNER_IP"],
                      bi.DEFAULT_NETWORK["SCANNER_FALLBACK_IP"],
                      bi.DEFAULT_NETWORK["ETH_ADDR"]):
        addr = ipaddress.ip_address(forbidden)
        covering = [str(n) for n in nets if addr in n]
        assert not covering, f"{forbidden} (scanner segment) is allowed by {covering}"
    # The whole /16 the scanner segment lives in, not just the two hosts.
    scanner_16 = ipaddress.ip_network("172.31.0.0/16")
    overlapping = [str(n) for n in nets if n.version == 4 and n.overlaps(scanner_16)]
    assert not overlapping, f"{overlapping} overlap the scanner /16"


def test_the_camera_unit_ip_filter_still_permits_an_ordinary_home_lan(rendered):
    """A filter tightened until it blocks the product is not a fix. These are
    the RFC1918 ranges a home or travel Wi-Fi actually hands out, including
    172.17.2.0/24 -- the subnet this bridge was measured on."""
    import ipaddress
    nets = _ip_allow_networks(rendered)
    for allowed in ("192.168.1.50", "10.0.0.7", "172.17.2.78", "172.30.4.4"):
        addr = ipaddress.ip_address(allowed)
        assert any(addr in n for n in nets), f"{allowed} would be blocked"


def test_the_rendered_unit_carries_the_real_port_not_only_the_token(rendered):
    """`{{CAM_PORT}}` surviving into an installed unit is a service that never
    starts, and the token test alone cannot see that."""
    raw = (rendered / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    assert "{{" not in raw, "an unrendered token reached the installed unit"
    assert "ROOMSCAN_CAM_PORT=5001" in raw
    nft = (rendered / "etc" / "nftables" / "roomscan-bridge.nft").read_text()
    assert nft.count("tcp dport 5001") == 2


def test_the_image_ships_the_tool_the_frame_rate_control_needs():
    """`v4l2-ctl` is how `exposure_dynamic_framerate` gets set at all.

    Without `v4l-utils` on the image the set is a silent no-op, the C270 stays
    at the value that halves its cadence, and the daemon advertises 30 fps
    while delivering 14.7 -- the exact "silently fall back while continuing to
    advertise 720p30" the spec forbids. The daemon now reports that as a
    control mismatch, but reporting a self-inflicted fault is not a substitute
    for shipping the package.
    """
    assert "v4l-utils" in bi.EXTRA_PACKAGES
    daemon = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-cam").read_text()
    assert "/usr/bin/v4l2-ctl" in daemon


def test_the_daemon_compares_the_control_readback_against_the_request():
    """A readback nobody compares is the same failure as a getter that returns
    what its setter just wrote (#106): it proves the call was made, nothing
    more. The comparison, the degraded flag, and the measured-vs-requested
    cadence must all be present."""
    daemon = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-cam").read_text()
    assert "def control_mismatch(" in daemon
    assert "def verify_controls(" in daemon
    assert '"degraded"' in daemon and '"degraded_reasons"' in daemon
    assert '"fps_measured"' in daemon


def test_the_daemon_handles_sigterm_so_systemd_can_stop_it():
    daemon = (PAYLOAD / "usr" / "local" / "sbin" / "roomscan-cam").read_text()
    assert "signal.signal(signal.SIGTERM" in daemon
    # Restart=on-failure only restarts on a non-zero exit, so the
    # address-change path must exit non-zero or the daemon stays stopped.
    assert "EXIT_ADDRESS_CHANGED" in daemon
    cp_raw = (PAYLOAD / "etc" / "systemd" / "system" / CAM_UNIT).read_text()
    assert "Restart=on-failure" in cp_raw


# ---------------------------------------------------------------------------
# Camera self-heal vs. operator intent (issue #226, found on the deployed Pi)
# ---------------------------------------------------------------------------

RECONCILE = PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-reconcile"


def _camera_decision(present: str, enabled: str, active_state: str,
                     result: str) -> str:
    """Actually RUN reconcile's decision function.

    Every other reconcile test in this file asserts on the script's source
    text, which cannot tell a policy that works from one that merely mentions
    the right words -- and the defect this covers shipped, deployed, and was
    caught by a human running `systemctl stop`. `ROOMSCAN_RECONCILE_LIB_ONLY`
    loads the definitions without running a pass, so the policy can be
    executed here instead of read.
    """
    proc = subprocess.run(
        ["bash", "-c",
         f'source "{RECONCILE}"; camera_restart_decision "$1" "$2" "$3" "$4"',
         "_", present, enabled, active_state, result],
        capture_output=True, text=True,
        env={**os.environ, "ROOMSCAN_RECONCILE_LIB_ONLY": "1", "PATH": os.environ["PATH"]})
    assert proc.returncode == 0, proc.stderr
    return proc.stdout.strip()


def test_reconcile_restarts_a_camera_service_that_actually_crashed():
    """The self-heal must survive the fix that stops it overriding intent.

    `failed` with a non-success Result is a crash, an exhausted
    StartLimitBurst, or a signal -- none of which any operator asked for.
    """
    for result in ("exit-code", "signal", "start-limit-hit", "timeout", "core-dump"):
        decision = _camera_decision("yes", "enabled", "failed", result)
        assert decision.startswith("restart:"), f"Result={result}: {decision}"
    # Inactive but with a failure Result is the same class of fault.
    assert _camera_decision("yes", "enabled", "inactive", "exit-code"
                            ).startswith("restart:")


def test_reconcile_leaves_a_camera_the_operator_stopped_alone():
    """The reported defect, exactly.

    `sudo systemctl stop roomscan-cam.service` was undone two seconds later by
    the next reconcile tick, so the camera could not be turned off at all and
    the spec's section 5.1 camera-off arm could not be run without masking the
    unit. A clean stop leaves ActiveState=inactive with Result=success, and
    that pair is the intent.
    """
    decision = _camera_decision("yes", "enabled", "inactive", "success")
    assert decision.startswith("skip:"), decision
    assert "operator" in decision


@pytest.mark.parametrize("enabled", ["disabled", "masked", "masked-runtime",
                                     "static", "not-found"])
def test_reconcile_leaves_a_disabled_or_masked_camera_alone(enabled):
    """`disable`, `mask` and `mask --runtime` have no cause other than an
    administrator. A self-heal that overrides them is not a self-heal."""
    decision = _camera_decision("yes", enabled, "inactive", "success")
    assert decision.startswith("skip:"), decision
    assert enabled in decision, decision


def test_reconcile_does_not_race_systemds_own_restart_backoff():
    """`activating` is a unit coming up -- very often systemd's own
    `Restart=on-failure` backoff. Restarting it there burns a start attempt
    against the StartLimitBurst for nothing."""
    for state in ("activating", "deactivating", "reloading"):
        assert _camera_decision("yes", "enabled", state, "success"
                                ).startswith("skip:")


def test_reconcile_still_does_nothing_when_no_camera_is_fitted():
    assert _camera_decision("no", "enabled", "failed", "exit-code"
                            ).startswith("skip:")
    assert _camera_decision("yes", "enabled", "active", "success"
                            ).startswith("skip:")


def test_reconcile_skips_a_camera_unit_that_is_not_installed_at_all():
    assert _camera_decision("yes", "", "", "").startswith("skip:")


def test_the_reconcile_library_guard_does_not_run_a_pass_when_sourced():
    """The guard that makes the test above possible must not be reachable by
    accident: without the environment variable, sourcing still runs `main`."""
    text = RECONCILE.read_text()
    assert 'if [ "${ROOMSCAN_RECONCILE_LIB_ONLY:-0}" != "1" ]; then' in text
    assert text.rstrip().endswith("fi")
    assert "\nmain \"$@\"\n" not in text, "main is still called unconditionally"


STATUS_SCRIPT = PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-status"

#: argv order of roomscan-bridge-status's python assembler, which unpacks
#: sys.argv[1:32] positionally.
_STATUS_ARGV = (
    "model", "hostname_val", "uptime_s",
    "wlan0_ssid", "wlan0_rssi", "wlan0_bitrate", "wlan0_powersave",
    "wlan0_ipv4", "wlan0_connected",
    "eth0_carrier", "eth0_ipv4", "eth0_neigh", "leases_raw",
    "nft_raw", "nft_ok",
    "tee_active", "ring_files_count", "ring_bytes", "ring_newest",
    "ring_oldest", "tee_readable",
    "units_raw",
    "temp_c", "throttled", "ntp_synced",
    "scanner_ip", "stream_port",
    "cam_usb_present", "cam_status_raw", "cam_port", "cam_unit_enabled",
    "cam_active_state", "cam_result",
)


def _run_status_assembler(**overrides) -> dict:
    """Execute roomscan-bridge-status's JSON assembler with a chosen reading.

    Reading the script's source cannot tell a branch that fires from one that
    merely exists -- proved the hard way here: an earlier version of this test
    passed with the operator branch replaced by `if False:`, because the
    string it asserted on was still in the file. So extract the real
    `python3 -c` body the Pi runs and run it, argv and all.
    """
    lines = STATUS_SCRIPT.read_text().splitlines()
    start = next(i for i, ln in enumerate(lines) if ln.startswith("python3 -c '"))
    end = next(i for i, ln in enumerate(lines[start + 1:], start + 1)
               if ln.strip().startswith("' \\"))
    body = "\n".join(lines[start + 1:end])
    argv = [overrides.get(name, "") for name in _STATUS_ARGV]
    proc = subprocess.run([sys.executable, "-c", body, *argv],
                          capture_output=True, text=True)
    assert proc.returncode == 0, proc.stderr
    return json.loads(proc.stdout)


def test_status_reports_a_deliberately_off_camera_as_off_not_as_broken():
    """bridge_status must stay clean while the camera is off on purpose.

    Otherwise every check of a bridge whose camera is masked for a measurement
    reports a warning that is true, useless, and indistinguishable from the
    one that means the daemon died. A masked unit is ALSO unreachable, so the
    "not answering" branch would win if the ordering were wrong -- which is
    why this asserts the text, not merely that a warning exists.
    """
    for enabled in ("masked", "masked-runtime", "disabled"):
        out = _run_status_assembler(cam_usb_present="yes", cam_unit_enabled=enabled,
                                    cam_active_state="inactive", cam_result="success",
                                    cam_status_raw="", cam_port="5001")
        cam = out["camera"]
        assert cam["unit_enabled"] == enabled
        assert cam["disabled_by_operator"] is True
        assert cam["warning"] == (
            f"camera disabled by operator (systemctl is-enabled = {enabled})"), cam


def test_status_agrees_with_reconcile_about_a_plain_systemctl_stop():
    """The state the deployed bridge was actually in when this was reported.

    is-enabled=enabled, ActiveState=inactive, Result=success. reconcile calls
    that "off on purpose" and leaves it alone; status must say the same, or
    the two mechanisms contradict each other about one reading -- which is how
    a healthy deliberate state gets chased as a fault.
    """
    out = _run_status_assembler(cam_usb_present="yes", cam_unit_enabled="enabled",
                                cam_active_state="inactive", cam_result="success",
                                cam_status_raw="", cam_port="5001")
    cam = out["camera"]
    assert cam["disabled_by_operator"] is True, cam
    assert "camera disabled by operator" in cam["warning"]
    assert "Result=success" in cam["warning"]
    # And the two really do agree, rather than merely both looking plausible.
    assert _camera_decision("yes", "enabled", "inactive", "success"
                            ).startswith("skip:")


def test_status_still_reports_a_camera_that_crashed():
    """The fault case must survive the fix that silences the deliberate one.

    A crash leaves `failed`, or a Result that is not `success` -- and that is
    exactly when reconcile restarts it, so status must not call it deliberate.
    """
    out = _run_status_assembler(cam_usb_present="yes", cam_unit_enabled="enabled",
                                cam_active_state="failed", cam_result="exit-code",
                                cam_status_raw="", cam_port="5001")
    cam = out["camera"]
    assert cam["disabled_by_operator"] is False, cam
    assert cam["warning"] == ("C270 is on the USB bus but roomscan-cam is not "
                              "answering on its port")
    assert _camera_decision("yes", "enabled", "failed", "exit-code"
                            ).startswith("restart:")


def test_status_is_silent_about_a_healthy_camera():
    out = _run_status_assembler(
        cam_usb_present="yes", cam_unit_enabled="enabled", cam_port="5001",
        cam_active_state="active", cam_result="success",
        cam_status_raw=json.dumps({"state": "streaming", "generation": 1}))
    cam = out["camera"]
    assert cam["warning"] is None, cam
    assert cam["daemon_reachable"] is True
    assert cam["daemon"]["state"] == "streaming"


def test_status_does_not_invent_a_camera_verdict_it_could_not_reach():
    """No camera fitted and no unit installed: every camera field is null or
    false, and nothing is reported as a fault."""
    out = _run_status_assembler(cam_usb_present="no", cam_port="5001")
    cam = out["camera"]
    assert cam["usb_present"] is False
    assert cam["unit_enabled"] is None
    assert cam["disabled_by_operator"] is False
    assert cam["warning"] is None


def test_install_leaves_a_masked_optional_unit_masked():
    """A deploy must not silently switch the camera back on.

    `masked` is safe to honour where `disabled` is not: a freshly copied unit
    is never masked, so this cannot be confused with "brand new and not
    enabled yet" -- the case the issue #200 fix exists for, which the same
    loop still handles.
    """
    text = (PAYLOAD / "install.sh").read_text()
    optional_loop = text.split('for unit in "${OPTIONAL_UNITS[@]}"; do', 1)[1]
    optional_loop = optional_loop.split("\n    done", 1)[0]
    assert "masked" in optional_loop
    assert "leaving it alone" in optional_loop
    # And the #200 behaviour is still there for everything else.
    assert 'systemctl enable "${unit}"' in optional_loop.split("else", 1)[1]
