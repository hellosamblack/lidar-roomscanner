#!/usr/bin/env python3
"""Build a flashable Raspberry Pi bridge SD-card image, entirely rootless.

The Pi 3 bridge node replaces the RavPower FileHub as the scanner's wireless
uplink (docs/superpowers/specs/2026-08-17-pi3-bridge-node-design.md, issue #191).
This builder produces a `.img` you can write to a card with `dd`/Imager/balena,
with Wi-Fi credentials, ssh access, the bridge payload and its Debian package
closure already baked in -- so the Pi's *first* boot needs no keyboard, no
screen, and crucially no network on the eth0 side (where the only other device
is the scanner, which is not a DHCP server we want to depend on).

Rootless is a hard requirement, not a preference: this host is an LXC container
with no loop devices and no passwordless sudo, so the usual
`losetup`/`mount`-the-image recipe is unavailable. Instead we parse the MBR in
pure Python to find the FAT boot partition's byte offset, and read/write files
inside it with `mtools` (`mcopy -i image@@offset`), which speaks FAT directly
against a plain file. The ext4 root partition is read (never written) with
`debugfs`, which accepts an `image?offset=N` device suffix -- also no mount.

Stages, in order:

  download   fetch the pinned Raspberry Pi OS Lite image, verify its sha256,
             decompress (cached under `pi-bridge/cache/`)
  mbr        parse the partition table, locate the FAT boot + ext4 root slices
  debs       read the base image's dpkg status through `debugfs`, resolve the
             dependency closure of the packages we add (dnsmasq, tcpdump, ...)
             against the archive's Packages index, download the missing .debs
  render     copy `payload/` to a staging dir, substituting `{{TOKEN}}`s from
             the secrets file, and tar it up
  inject     `mcopy` firstrun.sh + the payload tarball + userconf.txt + `ssh`
             into the FAT partition, and append the firstrun arguments to
             cmdline.txt
  report     what was actually produced (`--json` for machines)

Usage:

    ./pi-bridge/build_image.py --secrets ~/.config/roomscan/pi-bridge-secrets.yaml
    ./pi-bridge/build_image.py --secrets ... --xz --json

Everything the builder writes lands under `pi-bridge/out/` (images) and
`pi-bridge/cache/` (downloads); both are git-ignored.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import lzma
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import uuid
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
PAYLOAD_SRC = HERE / "payload"
BOOT_SRC = HERE / "boot"
CACHE_DIR = HERE / "cache"
OUT_DIR = HERE / "out"
EXAMPLE_SECRETS = HERE / "pi-bridge-secrets.example.yaml"

# ---------------------------------------------------------------------------
# Pinned base images
# ---------------------------------------------------------------------------

_BASE = "https://downloads.raspberrypi.com/raspios_lite_armhf/images"


@dataclass(frozen=True)
class Release:
    """A pinned Raspberry Pi OS Lite armhf release.

    `suite` is the Debian suite name, which selects the archive indexes the deb
    closure is resolved against -- resolving Trixie packages against Bookworm's
    index would produce .debs that dpkg refuses on first boot.
    """
    key: str
    filename: str
    url: str
    sha256: str
    suite: str


RELEASES: dict[str, Release] = {
    # Current stable. The plan (2026-08-17) said "Bookworm" because that was
    # stable when it was written; Trixie has since shipped and is what the
    # armhf Lite line is now published from -- the last Bookworm armhf Lite
    # build is 2025-05-13 and receives only oldstable updates. Everything the
    # design depends on (/boot/firmware, the systemd.run firstrun mechanism,
    # NetworkManager keyfiles, dnsmasq, nftables, avahi) is unchanged between
    # the two, so we default to Trixie and keep Bookworm selectable.
    "trixie": Release(
        key="trixie",
        filename="2026-06-18-raspios-trixie-armhf-lite.img.xz",
        url=f"{_BASE}/raspios_lite_armhf-2026-06-19/"
            "2026-06-18-raspios-trixie-armhf-lite.img.xz",
        sha256="ea4e84c501d6dd4f4b1d04eb84df133a03f90a05ee2e8ab849185c17c2b0707b",
        suite="trixie",
    ),
    "bookworm": Release(
        key="bookworm",
        filename="2025-05-13-raspios-bookworm-armhf-lite.img.xz",
        url=f"{_BASE}/raspios_lite_armhf-2025-05-13/"
            "2025-05-13-raspios-bookworm-armhf-lite.img.xz",
        sha256="a73d68b618c3ca40190c1aa04005a4dafcf32bc861c36c0d1fc6ddc48a370b6e",
        suite="bookworm",
    ),
}

DEFAULT_RELEASE = "trixie"

#: Packages the bridge payload needs that Raspberry Pi OS Lite does not ship.
EXTRA_PACKAGES = ("dnsmasq", "tcpdump", "nftables")

#: Archives to resolve the closure against, in preference order. The Raspberry
#: Pi archive wins where it overrides a Debian package (it is listed first).
ARCHIVES = (
    ("archive.raspberrypi.com", "http://archive.raspberrypi.com/debian", "main"),
    ("raspbian", "http://raspbian.raspberrypi.com/raspbian", "main"),
)

# ---------------------------------------------------------------------------
# firstrun wiring
# ---------------------------------------------------------------------------

#: Appended verbatim to cmdline.txt. This is the same mechanism Raspberry Pi
#: Imager uses: systemd's kernel-command-line generator runs the named script
#: once, then reboots on success. `systemd.unit=kernel-command-line.target`
#: keeps the first boot minimal (no getty/network races while we install).
FIRSTRUN_ARGS = (
    "systemd.run=/boot/firmware/firstrun.sh",
    "systemd.run_success_action=reboot",
    "systemd.unit=kernel-command-line.target",
)

PAYLOAD_TAR_NAME = "roomscan-bridge-payload.tar.gz"

#: uuid5 namespace so NetworkManager profile UUIDs are stable across rebuilds.
#: A fresh random UUID each build would leave the Pi accumulating duplicate
#: connection profiles every time `bridge_update` re-pushed the payload.
_NM_NS = uuid.UUID("6f3a1f2e-2f39-5a5b-9a2b-1b0f2d1c4e70")

DEFAULT_NETWORK = {
    "ETH_ADDR": "172.31.100.1",
    "ETH_PREFIX": "24",
    "ETH_CIDR": "172.31.100.1/24",
    "SCANNER_IP": "172.31.100.20",
    "SCANNER_MAC": "00:80:E1:00:00:00",  # firmware compile-time constant
    "SCANNER_FALLBACK_IP": "172.31.253.1",  # firmware self-assigned server mode
    "DHCP_RANGE_START": "172.31.100.20",
    "DHCP_RANGE_END": "172.31.100.40",
    "STREAM_PORT": "5000",
    "MDNS_INSTANCE": "roomscanner",
    "MDNS_SERVICE": "_roomscan._udp",
    "BRIDGE_MDNS_INSTANCE": "roomscan-bridge",
    "BRIDGE_MDNS_SERVICE": "_roomscan-bridge._tcp",
}

TOKEN_RE = re.compile(r"\{\{([A-Z0-9_]+)\}\}")

#: Files under `payload/` that document the payload rather than being part of
#: it. Never staged onto the Pi.
PAYLOAD_DOCS = {"README.md"}


class BuildError(RuntimeError):
    """Anything that should stop the build with a readable message."""


# ---------------------------------------------------------------------------
# Secrets
# ---------------------------------------------------------------------------

REQUIRED_SECRETS = ("hostname", "username", "wifi_country", "wifi")


def load_secrets(path: Path, *, allow_insecure_perms: bool = False) -> dict:
    """Load and validate the secrets file. Pure apart from reading `path`.

    Two hygiene rules are enforced here rather than left to discipline, because
    both failures are silent: refusing the committed example file (whose
    placeholder PSK would produce an image that never associates, with no error
    until the Pi is already in the field), and refusing a world/group-readable
    file (a Wi-Fi PSK and a login password in plain text).
    """
    path = path.expanduser()
    if not path.is_file():
        raise BuildError(f"secrets file not found: {path}")
    if path.resolve() == EXAMPLE_SECRETS.resolve():
        raise BuildError(
            f"{path} is the committed example template, not real credentials. "
            f"Copy it somewhere private, fill it in, and pass --secrets that path.")

    mode = stat.S_IMODE(path.stat().st_mode)
    if mode & 0o077 and not allow_insecure_perms:
        raise BuildError(
            f"{path} is mode {mode:04o}; it holds a Wi-Fi PSK and a login "
            f"password in plain text. `chmod 600 {path}` (or pass "
            f"--allow-insecure-perms if you really mean it).")

    import yaml  # deferred: only the build path needs it

    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise BuildError(f"{path}: expected a YAML mapping at the top level")

    missing = [k for k in REQUIRED_SECRETS if not data.get(k)]
    if missing:
        raise BuildError(f"{path}: missing required key(s): {', '.join(missing)}")

    wifi = data["wifi"]
    if not isinstance(wifi, dict) or "home" not in wifi:
        raise BuildError(f"{path}: `wifi` must be a mapping with at least a `home` profile")
    for name, prof in wifi.items():
        if not isinstance(prof, dict) or not prof.get("ssid") or not prof.get("passphrase"):
            raise BuildError(f"{path}: wifi.{name} needs both `ssid` and `passphrase`")
        psk = str(prof["passphrase"])
        if not 8 <= len(psk) <= 63:
            raise BuildError(
                f"{path}: wifi.{name}.passphrase is {len(psk)} chars; WPA-PSK "
                f"passphrases are 8-63. wpa_supplicant would reject it on the Pi, "
                f"where you cannot see the error.")
        if "CHANGEME" in psk or "example" in psk.lower():
            raise BuildError(f"{path}: wifi.{name}.passphrase still looks like a placeholder")

    country = str(data["wifi_country"])
    if len(country) != 2 or not country.isalpha():
        raise BuildError(f"{path}: wifi_country must be a 2-letter ISO code, got {country!r}")
    return data


def secret_tokens(secrets: dict, ssh_pubkey: str) -> dict[str, str]:
    """Build the full `{{TOKEN}}` substitution map. Pure."""
    host = str(secrets["hostname"])
    tokens = dict(DEFAULT_NETWORK)
    tokens.update({
        "HOSTNAME": host,
        "USERNAME": str(secrets["username"]),
        "WIFI_COUNTRY": str(secrets["wifi_country"]).upper(),
        "SSH_PUBKEY": ssh_pubkey.strip(),
        "UUID_ETH0": str(uuid.uuid5(_NM_NS, f"{host}/eth0")),
    })
    # Alias spelling used by the payload templates. Both names resolve so a
    # template can be written either way without a silent unresolved token.
    tokens["ETH0_UUID"] = tokens["UUID_ETH0"]
    for name in ("home", "travel"):
        prof = secrets.get("wifi", {}).get(name)
        up = name.upper()
        if prof:
            tokens[f"{up}_SSID"] = str(prof["ssid"])
            tokens[f"{up}_PSK"] = str(prof["passphrase"])
            tokens[f"{up}_PRIORITY"] = str(prof.get("priority", 50 if name == "home" else 10))
        else:
            # A travel profile is optional. Emit a disabled placeholder rather
            # than leaving the token unresolved, so the template still renders.
            tokens[f"{up}_SSID"] = ""
            tokens[f"{up}_PSK"] = ""
            tokens[f"{up}_PRIORITY"] = "0"
        tokens[f"UUID_WIFI_{up}"] = str(uuid.uuid5(_NM_NS, f"{host}/wifi/{name}"))
        tokens[f"{up}_UUID"] = tokens[f"UUID_WIFI_{up}"]  # alias, see ETH0_UUID
    return tokens


# ---------------------------------------------------------------------------
# MBR
# ---------------------------------------------------------------------------

SECTOR = 512
_FAT_TYPES = {0x01, 0x04, 0x06, 0x0B, 0x0C, 0x0E}
_EXT_TYPES = {0x83}
_EXTENDED_TYPES = {0x05, 0x0F, 0x85}


@dataclass(frozen=True)
class Partition:
    index: int
    bootable: bool
    ptype: int
    lba_start: int
    sectors: int

    @property
    def offset(self) -> int:
        return self.lba_start * SECTOR

    @property
    def size(self) -> int:
        return self.sectors * SECTOR

    @property
    def is_fat(self) -> bool:
        return self.ptype in _FAT_TYPES

    @property
    def is_ext(self) -> bool:
        return self.ptype in _EXT_TYPES


def parse_mbr(first_sector: bytes) -> list[Partition]:
    """Parse a classic MBR partition table from the first 512 bytes. Pure.

    Deliberately strict: an unexpected table shape means our byte offsets would
    be wrong, and a wrong offset handed to `mcopy` corrupts the image quietly
    rather than failing loudly.
    """
    if len(first_sector) < SECTOR:
        raise BuildError(f"short read: need {SECTOR} bytes of MBR, got {len(first_sector)}")
    if first_sector[510:512] != b"\x55\xaa":
        raise BuildError("no MBR boot signature (0x55AA) -- not an MBR-partitioned image")

    parts: list[Partition] = []
    for i in range(4):
        e = first_sector[446 + 16 * i: 446 + 16 * (i + 1)]
        ptype = e[4]
        if ptype == 0:
            continue
        if ptype in _EXTENDED_TYPES:
            raise BuildError(
                f"partition {i + 1} is an extended partition (type 0x{ptype:02x}); "
                f"this builder only understands the flat primary table Raspberry "
                f"Pi OS ships")
        lba_start = int.from_bytes(e[8:12], "little")
        sectors = int.from_bytes(e[12:16], "little")
        if sectors == 0:
            raise BuildError(f"partition {i + 1} declares 0 sectors")
        parts.append(Partition(i + 1, bool(e[0] & 0x80), ptype, lba_start, sectors))
    if not parts:
        raise BuildError("MBR has no partitions")
    return parts


def read_mbr(img: Path) -> list[Partition]:
    with img.open("rb") as f:
        return parse_mbr(f.read(SECTOR))


def boot_partition(parts: list[Partition]) -> Partition:
    fats = [p for p in parts if p.is_fat]
    if not fats:
        raise BuildError(
            "no FAT partition found; expected the Raspberry Pi boot partition "
            f"(saw types {[hex(p.ptype) for p in parts]})")
    return fats[0]


def root_partition(parts: list[Partition]) -> Partition:
    exts = [p for p in parts if p.is_ext]
    if not exts:
        raise BuildError(
            "no ext partition found; expected the Raspberry Pi root filesystem "
            f"(saw types {[hex(p.ptype) for p in parts]})")
    return exts[0]


# ---------------------------------------------------------------------------
# cmdline.txt
# ---------------------------------------------------------------------------

def append_firstrun_args(cmdline: str, args: tuple[str, ...] = FIRSTRUN_ARGS) -> str:
    """Append the firstrun kernel arguments to cmdline.txt. Pure, idempotent.

    cmdline.txt is a *single line* and its content is release-specific -- the
    root= UUID, the rootwait/quiet flags, the cgroup settings all belong to the
    image. So this appends and asserts; it never rewrites. A stray newline here
    silently truncates the kernel command line at the newline, which on a Pi
    presents as an unbootable card with no diagnostics.
    """
    body = cmdline.rstrip("\r\n")
    if "\n" in body or "\r" in body:
        raise BuildError(
            "cmdline.txt has more than one line; the Pi bootloader passes only "
            "the first, so appending would silently do nothing")
    if not body.strip():
        raise BuildError("cmdline.txt is empty -- refusing to invent one")

    present = [a for a in args if a in body.split()]
    if present and len(present) != len(args):
        raise BuildError(
            f"cmdline.txt already carries some but not all firstrun args "
            f"({present}); refusing to guess. Re-extract a clean base image.")
    if present:
        return body + "\n"  # already injected; idempotent re-run
    return body + " " + " ".join(args) + "\n"


def strip_firstrun_args(cmdline: str, args: tuple[str, ...] = FIRSTRUN_ARGS) -> str:
    """Inverse of `append_firstrun_args` -- the same edit firstrun.sh makes on
    the Pi. Pure; kept here so a test can prove the round trip."""
    body = cmdline.rstrip("\r\n")
    kept = [tok for tok in body.split() if tok not in args]
    return " ".join(kept) + "\n"


# ---------------------------------------------------------------------------
# Debian package closure
# ---------------------------------------------------------------------------

def parse_control(text: str) -> list[dict[str, str]]:
    """Parse RFC822-ish Debian control paragraphs (Packages / dpkg status).

    Pure. Continuation lines are folded; we only care about single-line fields.
    """
    paras: list[dict[str, str]] = []
    cur: dict[str, str] = {}
    key: str | None = None
    for line in text.splitlines():
        if not line.strip():
            if cur:
                paras.append(cur)
                cur, key = {}, None
            continue
        if line[0] in " \t":
            if key:
                cur[key] += "\n" + line.strip()
            continue
        if ":" not in line:
            continue
        key, _, val = line.partition(":")
        key = key.strip()
        cur[key] = val.strip()
    if cur:
        paras.append(cur)
    return paras


def installed_set(status_text: str) -> set[str]:
    """Names (and Provides) of packages already installed in the base image. Pure."""
    have: set[str] = set()
    for p in parse_control(status_text):
        status = p.get("Status", "")
        if "install ok installed" not in status:
            continue
        name = p.get("Package")
        if name:
            have.add(name)
        for prov in _split_relations(p.get("Provides", "")):
            for alt in prov:
                have.add(alt)
    return have


def _strip_relation(tok: str) -> str:
    """`libc6 (>= 2.34) [!armel] <!nocheck>` -> `libc6`. Pure."""
    tok = re.sub(r"\(.*?\)|\[.*?\]|<.*?>", " ", tok)
    tok = tok.strip().split()[0] if tok.strip() else ""
    return tok.split(":", 1)[0]  # drop :any / :native arch qualifiers


def _split_relations(field_text: str) -> list[list[str]]:
    """`a (>=1) | b, c` -> [['a','b'], ['c']]. Pure."""
    out: list[list[str]] = []
    for group in field_text.replace("\n", " ").split(","):
        alts = [_strip_relation(a) for a in group.split("|")]
        alts = [a for a in alts if a]
        if alts:
            out.append(alts)
    return out


def index_packages(texts: list[str]) -> tuple[dict[str, dict], dict[str, str]]:
    """Build name -> paragraph and virtual -> provider maps. Pure.

    Earlier texts win, so callers pass the Raspberry Pi archive before Raspbian.
    """
    by_name: dict[str, dict] = {}
    provides: dict[str, str] = {}
    for text in texts:
        for p in parse_control(text):
            name = p.get("Package")
            if not name or name in by_name:
                continue
            by_name[name] = p
            for group in _split_relations(p.get("Provides", "")):
                for virt in group:
                    provides.setdefault(virt, name)
    return by_name, provides


def resolve_closure(wanted: list[str], by_name: dict[str, dict],
                    provides: dict[str, str], installed: set[str]) -> dict:
    """Resolve the dependency closure of `wanted` minus what's already installed.

    Pure. Returns `{"packages": [...], "unsatisfied": [...], "already": [...]}`,
    where `packages` is in a dpkg-friendly order (dependencies first).

    Version constraints are deliberately ignored: every candidate comes from the
    *same suite* as the base image, so the archive's own consistency is the
    guarantee. `dpkg -i` on the Pi is the backstop -- and it is a loud one.
    """
    seen: set[str] = set()
    order: list[str] = []
    unsatisfied: list[str] = []
    already: list[str] = []

    def visit(name: str, chain: tuple[str, ...]) -> None:
        if name in installed:
            if name in wanted:
                already.append(name)
            return
        if name in seen:
            return
        seen.add(name)
        para = by_name.get(name)
        if para is None and name in provides:
            para = by_name.get(provides[name])
            name = provides[name]
            if name in installed or name in order:
                return
        if para is None:
            unsatisfied.append(" -> ".join(chain + (name,)))
            return
        deps = _split_relations(para.get("Pre-Depends", "")) + \
            _split_relations(para.get("Depends", ""))
        for alts in deps:
            # Prefer an alternative that is already installed; else the first
            # one the archive actually has. Matching apt's real solver is not
            # the goal -- not silently dropping a dependency is.
            chosen = next((a for a in alts if a in installed), None)
            if chosen is not None:
                continue
            chosen = next((a for a in alts if a in by_name or a in provides), None)
            if chosen is None:
                unsatisfied.append(" -> ".join(chain + (name, "|".join(alts))))
                continue
            visit(chosen, chain + (name,))
        order.append(name)

    for w in wanted:
        visit(w, ())
    return {"packages": order, "unsatisfied": unsatisfied, "already": already}


# ---------------------------------------------------------------------------
# Reading the base image's dpkg status (no mount, no root)
# ---------------------------------------------------------------------------

def _run(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, **kw)


def read_dpkg_status(img: Path, part: Partition) -> str:
    """Read `/var/lib/dpkg/status` out of the image's ext4 root, rootlessly.

    `debugfs` accepts an `image?offset=N` device suffix (e2fsprogs >= 1.43), so
    the partition can be read in place. If that fails -- an older debugfs, or a
    build host where the suffix is not honoured -- fall back to copying the
    partition out to a scratch file, which is correct but costs a couple of GB.
    """
    if shutil.which("debugfs") is None:
        raise BuildError("debugfs not found (install e2fsprogs) -- needed to read "
                         "the base image's package list")
    dev = f"{img}?offset={part.offset}"
    r = _run(["debugfs", "-R", "cat /var/lib/dpkg/status", dev])
    if r.returncode == 0 and b"Package:" in r.stdout:
        return r.stdout.decode("utf-8", "replace")

    with tempfile.TemporaryDirectory(dir=str(CACHE_DIR)) as td:
        slice_path = Path(td) / "root.img"
        _copy_range(img, slice_path, part.offset, part.size)
        r = _run(["debugfs", "-R", "cat /var/lib/dpkg/status", str(slice_path)])
        if r.returncode != 0 or b"Package:" not in r.stdout:
            raise BuildError(
                "could not read /var/lib/dpkg/status from the image root "
                f"partition: {r.stderr.decode('utf-8', 'replace').strip()}")
        return r.stdout.decode("utf-8", "replace")


def _copy_range(src: Path, dst: Path, offset: int, size: int, chunk: int = 8 << 20) -> None:
    with src.open("rb") as fi, dst.open("wb") as fo:
        fi.seek(offset)
        left = size
        while left:
            buf = fi.read(min(chunk, left))
            if not buf:
                break
            fo.write(buf)
            left -= len(buf)


# ---------------------------------------------------------------------------
# Download helpers
# ---------------------------------------------------------------------------

def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def _fetch(url: str, dest: Path, *, quiet: bool = False) -> Path:
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    if not quiet:
        print(f"  fetching {url}", file=sys.stderr)
    with urllib.request.urlopen(url, timeout=120) as r, tmp.open("wb") as f:
        shutil.copyfileobj(r, f, 1 << 20)
    tmp.replace(dest)
    return dest


def fetch_base_image(release: Release, *, offline: bool = False,
                     quiet: bool = False) -> Path:
    """Download (cached), verify and decompress the pinned base image."""
    CACHE_DIR.mkdir(parents=True, exist_ok=True)
    xz_path = CACHE_DIR / release.filename
    img_path = CACHE_DIR / release.filename.removesuffix(".xz")

    if img_path.is_file() and img_path.stat().st_size > 0:
        return img_path
    if not xz_path.is_file():
        if offline:
            raise BuildError(f"--offline but {xz_path} is not cached")
        _fetch(release.url, xz_path, quiet=quiet)

    got = sha256_file(xz_path)
    if got != release.sha256:
        xz_path.unlink(missing_ok=True)
        raise BuildError(
            f"sha256 mismatch for {release.filename}\n  expected {release.sha256}"
            f"\n  got      {got}\n(the corrupt download has been deleted)")

    if not quiet:
        print(f"  decompressing {release.filename}", file=sys.stderr)
    tmp = img_path.with_suffix(".part")
    with lzma.open(xz_path, "rb") as fi, tmp.open("wb") as fo:
        shutil.copyfileobj(fi, fo, 1 << 20)
    tmp.replace(img_path)
    return img_path


def fetch_packages_indexes(release: Release, *, offline: bool = False,
                           quiet: bool = False) -> list[str]:
    """Fetch (cached) the binary-armhf Packages index for each archive."""
    texts: list[str] = []
    for tag, base, component in ARCHIVES:
        dest = CACHE_DIR / f"Packages-{tag}-{release.suite}-{component}.gz"
        if not dest.is_file():
            if offline:
                raise BuildError(f"--offline but {dest} is not cached")
            url = f"{base}/dists/{release.suite}/{component}/binary-armhf/Packages.gz"
            _fetch(url, dest, quiet=quiet)
        texts.append(gzip.decompress(dest.read_bytes()).decode("utf-8", "replace"))
    return texts


def download_debs(names: list[str], by_name: dict[str, dict], dest_dir: Path, *,
                  offline: bool = False, quiet: bool = False) -> list[dict]:
    """Download each resolved .deb into `dest_dir`, verifying its digest.

    The Pi's first boot has no internet on the eth0 side (the only thing there
    is the scanner), so this closure IS the install medium -- an unverified or
    missing .deb here becomes a bricked-looking first boot in the field.
    """
    dest_dir.mkdir(parents=True, exist_ok=True)
    out: list[dict] = []
    for name in names:
        para = by_name[name]
        rel = para["Filename"]
        base = next(b for tag, b, _ in ARCHIVES if _archive_for(para) == tag)
        url = f"{base}/{rel}"
        fname = rel.rsplit("/", 1)[-1]
        target = dest_dir / fname
        cached = CACHE_DIR / "debs" / fname
        if not cached.is_file():
            if offline:
                raise BuildError(f"--offline but {cached} is not cached")
            _fetch(url, cached, quiet=quiet)
        want = para.get("SHA256")
        got = sha256_file(cached)
        if want and got != want:
            cached.unlink(missing_ok=True)
            raise BuildError(f"sha256 mismatch for {fname}: expected {want}, got {got}")
        shutil.copy2(cached, target)
        out.append({"package": name, "file": fname, "sha256": got,
                    "size": int(para.get("Size", 0) or 0)})
    return out


def _archive_for(para: dict) -> str:
    """Which archive a Packages paragraph came from (stamped at index time)."""
    return para.get("_archive", ARCHIVES[0][0])


# ---------------------------------------------------------------------------
# Payload rendering
# ---------------------------------------------------------------------------

def render_text(text: str, tokens: dict[str, str]) -> tuple[str, set[str]]:
    """Substitute `{{TOKEN}}`s. Pure. Returns (rendered, unresolved token names).

    Unresolved tokens are *returned*, not left in place silently: a stray
    `{{HOME_PSK}}` in a NetworkManager keyfile is a Pi that never associates and
    says nothing about why.
    """
    missing: set[str] = set()

    def sub(m: re.Match) -> str:
        name = m.group(1)
        if name not in tokens:
            missing.add(name)
            return m.group(0)
        return tokens[name]

    return TOKEN_RE.sub(sub, text), missing


def render_payload(src: Path, dest: Path, tokens: dict[str, str]) -> dict:
    """Copy `payload/` to `dest`, rendering `*.tmpl` and enforcing LF endings.

    CRLF matters here in a way it does not on the host: a shell script or a
    NetworkManager keyfile with CRLF endings fails on the Pi in ways that read
    as "the file is wrong" rather than "the newlines are wrong".
    """
    if dest.exists():
        shutil.rmtree(dest)
    dest.mkdir(parents=True)

    rendered: list[str] = []
    copied: list[str] = []
    skipped: list[str] = []
    missing: set[str] = set()
    crlf_fixed: list[str] = []

    for path in sorted(src.rglob("*")):
        rel = path.relative_to(src)
        if path.is_dir():
            (dest / rel).mkdir(parents=True, exist_ok=True)
            continue
        if rel.name in PAYLOAD_DOCS:
            # Repo documentation, not payload. It also *describes* the token
            # contract, so shipping it would trip the unresolved-token check on
            # its own prose.
            skipped.append(str(rel))
            continue
        target = dest / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        if path.suffix == ".tmpl":
            target = target.with_suffix("")
            text, miss = render_text(path.read_text(), tokens)
            missing |= miss
            target.write_text(_lf(text))
            rendered.append(str(target.relative_to(dest)))
        else:
            raw = path.read_bytes()
            if b"\r\n" in raw and _looks_textual(raw):
                raw = raw.replace(b"\r\n", b"\n")
                crlf_fixed.append(str(rel))
            # Non-template files may still carry tokens (units, confs).
            if _looks_textual(raw) and TOKEN_RE.search(raw.decode("utf-8", "replace")):
                text, miss = render_text(raw.decode(), tokens)
                missing |= miss
                raw = _lf(text).encode()
                rendered.append(str(rel))
            else:
                copied.append(str(rel))
            target.write_bytes(raw)
        shutil.copymode(path, target)
        if target.suffix in (".nmconnection",) or target.name.endswith(".nmconnection"):
            target.chmod(0o600)
        elif "sbin" in target.parts or "bin" in target.parts or target.suffix == ".sh":
            target.chmod(0o755)

    if missing:
        raise BuildError(
            "payload templates reference token(s) the secrets file does not "
            f"provide: {', '.join(sorted(missing))}")
    return {"rendered": rendered, "copied": copied, "crlf_fixed": crlf_fixed,
            "skipped": skipped}


#: Node identity the Pi reads at runtime from /etc/roomscan-bridge/node.env. The
#: reconcile and status scripts source it instead of hard-coding addresses.
NODE_ENV_KEYS = ("HOSTNAME", "USERNAME", "SCANNER_IP", "SCANNER_MAC",
                 "SCANNER_FALLBACK_IP", "ETH_ADDR", "STREAM_PORT", "WIFI_COUNTRY")


def stage_extras(stage: Path, tokens: dict[str, str], ssh_pubkey: str) -> list[str]:
    """Write the payload files that are generated rather than templated.

    Shared by the image build and by `bridge_update`, deliberately: when this
    lived only in `build()`, a `bridge_update` pushed a payload with no
    `node.env`, `install.sh` logged a warning nobody surfaced, kept the old
    values, and the tool still reported success -- so a changed `SCANNER_IP` or
    hostname could never reach an already-provisioned Pi without a reflash.
    """
    stage.mkdir(parents=True, exist_ok=True)
    (stage / "authorized_keys").write_text(ssh_pubkey.strip() + "\n")
    (stage / "node.env").write_text(
        "\n".join(f"{k}={tokens[k]}" for k in NODE_ENV_KEYS if k in tokens) + "\n")
    return ["authorized_keys", "node.env"]


def _lf(text: str) -> str:
    return text.replace("\r\n", "\n").replace("\r", "\n")


def _looks_textual(raw: bytes) -> bool:
    return b"\x00" not in raw[:4096]


def make_payload_tar(payload_dir: Path, out: Path) -> Path:
    """Tar the rendered payload, deterministically (no mtimes, no uid/gid)."""
    out.parent.mkdir(parents=True, exist_ok=True)

    def _norm(ti: tarfile.TarInfo) -> tarfile.TarInfo:
        ti.uid = ti.gid = 0
        ti.uname = ti.gname = "root"
        ti.mtime = 0
        return ti

    # Drive gzip explicitly: `w:gz` stamps the output filename and the current
    # time into the gzip header, so two builds of identical content would differ
    # -- which makes "did the payload actually change?" unanswerable by hash.
    with out.open("wb") as raw, \
            gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz, \
            tarfile.open(fileobj=gz, mode="w") as tf:
        for path in sorted(payload_dir.rglob("*")):
            tf.add(path, arcname=str(path.relative_to(payload_dir)), filter=_norm)
    return out


# ---------------------------------------------------------------------------
# mtools injection
# ---------------------------------------------------------------------------

def _mtools_env() -> dict[str, str]:
    env = dict(os.environ)
    # The Pi boot partition's geometry does not match mtools' expectations; the
    # check is advisory and refusing it would block every build.
    env["MTOOLS_SKIP_CHECK"] = "1"
    return env


def require_mtools() -> None:
    missing = [c for c in ("mcopy", "mdir") if shutil.which(c) is None]
    if missing:
        raise BuildError(
            f"mtools not installed (missing {', '.join(missing)}). This is the "
            f"only way to write the image's FAT partition without root:\n"
            f"    sudo apt install mtools")


def fat_read(img: Path, offset: int, path: str) -> bytes:
    """Read one file out of the FAT partition at `offset`."""
    with tempfile.TemporaryDirectory() as td:
        local = Path(td) / "f"
        r = _run(["mcopy", "-i", f"{img}@@{offset}", "-n", "-o",
                  f"::{path}", str(local)], env=_mtools_env())
        if r.returncode != 0:
            raise BuildError(f"mcopy read of {path} failed: "
                             f"{r.stderr.decode('utf-8', 'replace').strip()}")
        return local.read_bytes()


def fat_write(img: Path, offset: int, local: Path, path: str) -> None:
    r = _run(["mcopy", "-i", f"{img}@@{offset}", "-o", str(local), f"::{path}"],
             env=_mtools_env())
    if r.returncode != 0:
        raise BuildError(f"mcopy write of {path} failed: "
                         f"{r.stderr.decode('utf-8', 'replace').strip()}")


def fat_list(img: Path, offset: int) -> list[str]:
    r = _run(["mdir", "-i", f"{img}@@{offset}", "-b", "::/"], env=_mtools_env())
    if r.returncode != 0:
        raise BuildError(f"mdir failed: {r.stderr.decode('utf-8', 'replace').strip()}")
    return [ln.strip() for ln in r.stdout.decode().splitlines() if ln.strip()]


# ---------------------------------------------------------------------------
# ssh key + userconf
# ---------------------------------------------------------------------------

DEFAULT_KEY = Path("~/.ssh/roomscan-bridge").expanduser()


def ensure_ssh_key(path: Path = DEFAULT_KEY, *, generate: bool = True) -> str:
    """Return the public key for the bridge, generating the pair if absent.

    The MCP `bridge_*` tools ssh in with this exact key, so generating it here
    and baking the pubkey into the image is what makes remote administration
    work on the very first boot rather than after a manual copy-id.
    """
    pub = path.with_suffix(path.suffix + ".pub") if path.suffix else Path(str(path) + ".pub")
    if pub.is_file():
        return pub.read_text().strip()
    if not generate:
        raise BuildError(f"no ssh public key at {pub} and generation disabled")
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    r = _run(["ssh-keygen", "-t", "ed25519", "-N", "", "-C", "roomscan-bridge",
              "-f", str(path)])
    if r.returncode != 0:
        raise BuildError(f"ssh-keygen failed: {r.stderr.decode('utf-8', 'replace')}")
    return pub.read_text().strip()


def userconf_line(username: str, password: str) -> str:
    """`username:sha512-crypt-hash`, the format Raspberry Pi OS's userconf reads."""
    r = _run(["openssl", "passwd", "-6", password])
    if r.returncode != 0:
        raise BuildError(f"openssl passwd failed: {r.stderr.decode('utf-8', 'replace')}")
    return f"{username}:{r.stdout.decode().strip()}\n"


# ---------------------------------------------------------------------------
# The build
# ---------------------------------------------------------------------------

@dataclass
class BuildResult:
    image: Path | None = None
    release: str = ""
    partitions: list[dict] = field(default_factory=list)
    boot_offset: int = 0
    cmdline: str = ""
    payload: dict = field(default_factory=dict)
    debs: list[dict] = field(default_factory=list)
    deb_closure: dict = field(default_factory=dict)
    injected: list[str] = field(default_factory=list)
    xz: str | None = None
    warnings: list[str] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "ok": True,
            "image": str(self.image) if self.image else None,
            "release": self.release,
            "partitions": self.partitions,
            "boot_offset": self.boot_offset,
            "cmdline": self.cmdline,
            "payload": self.payload,
            "debs": self.debs,
            "deb_closure": self.deb_closure,
            "injected": self.injected,
            "xz": self.xz,
            "warnings": self.warnings,
        }


def build(secrets_path: Path, *, release_key: str = DEFAULT_RELEASE,
          out: Path | None = None, offline: bool = False, xz: bool = False,
          skip_debs: bool = False, quiet: bool = False,
          allow_insecure_perms: bool = False) -> dict:
    """Run every stage and return a structured report.

    Reports what actually happened, not what was requested (project law): the
    resolved package list, the real cmdline the image ends up with, and the
    files that are genuinely present in the FAT partition afterwards -- read
    back out of the image, not echoed from the input.
    """
    if release_key not in RELEASES:
        raise BuildError(f"unknown release {release_key!r}; have {sorted(RELEASES)}")
    release = RELEASES[release_key]
    secrets = load_secrets(secrets_path, allow_insecure_perms=allow_insecure_perms)
    require_mtools()

    res = BuildResult(release=release.key)

    # --- download / verify -------------------------------------------------
    base_img = fetch_base_image(release, offline=offline, quiet=quiet)

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = out or OUT_DIR / f"roomscan-bridge-{secrets['hostname']}-{release.key}.img"
    if not quiet:
        print(f"  copying base image -> {out}", file=sys.stderr)
    shutil.copy2(base_img, out)
    res.image = out

    # --- MBR ---------------------------------------------------------------
    parts = read_mbr(out)
    res.partitions = [{"index": p.index, "type": f"0x{p.ptype:02x}",
                       "offset": p.offset, "size": p.size} for p in parts]
    boot = boot_partition(parts)
    root = root_partition(parts)
    res.boot_offset = boot.offset

    # --- deb closure -------------------------------------------------------
    payload_stage = HERE / "build" / "payload"
    debs_dir = payload_stage / "debs"
    if skip_debs:
        res.warnings.append(
            "deb closure skipped (--skip-debs): the Pi will need working "
            "internet on first boot to install dnsmasq/tcpdump/nftables")
    else:
        status = read_dpkg_status(out, root)
        installed = installed_set(status)
        texts = fetch_packages_indexes(release, offline=offline, quiet=quiet)
        by_name, provides = _index_with_archive_tags(texts)
        closure = resolve_closure(list(EXTRA_PACKAGES), by_name, provides, installed)
        res.deb_closure = closure
        if closure["unsatisfied"]:
            raise BuildError(
                "unresolved dependencies against "
                f"{release.suite}: {closure['unsatisfied']}")

    # --- payload render ----------------------------------------------------
    ssh_pubkey = ensure_ssh_key()
    tokens = secret_tokens(secrets, ssh_pubkey)
    res.payload = render_payload(PAYLOAD_SRC, payload_stage, tokens)

    # Extras the templates cannot carry: the ssh key install.sh deploys, and
    # the node identity it reads.
    res.payload["extras"] = stage_extras(payload_stage, tokens, ssh_pubkey)

    if not skip_debs:
        res.debs = download_debs(res.deb_closure["packages"], by_name, debs_dir,
                                 offline=offline, quiet=quiet)

    tar_path = HERE / "build" / PAYLOAD_TAR_NAME
    make_payload_tar(payload_stage, tar_path)

    # --- inject ------------------------------------------------------------
    firstrun = BOOT_SRC / "firstrun.sh"
    if not firstrun.is_file():
        raise BuildError(f"missing {firstrun}")
    with tempfile.TemporaryDirectory() as td:
        td_p = Path(td)
        fr = td_p / "firstrun.sh"
        fr.write_bytes(_lf(firstrun.read_text()).encode())
        fat_write(out, boot.offset, fr, "/firstrun.sh")

        fat_write(out, boot.offset, tar_path, f"/{PAYLOAD_TAR_NAME}")

        uc = td_p / "userconf.txt"
        uc.write_text(userconf_line(secrets["username"],
                                    str(secrets.get("password", "roomscan-bridge"))))
        fat_write(out, boot.offset, uc, "/userconf.txt")

        sshflag = td_p / "ssh"
        sshflag.write_text("")
        fat_write(out, boot.offset, sshflag, "/ssh")

        cmdline = fat_read(out, boot.offset, "/cmdline.txt").decode()
        new_cmdline = append_firstrun_args(cmdline)
        cl = td_p / "cmdline.txt"
        cl.write_text(new_cmdline)
        fat_write(out, boot.offset, cl, "/cmdline.txt")

    # --- verify by reading back, not by echoing ---------------------------
    res.cmdline = fat_read(out, boot.offset, "/cmdline.txt").decode().strip()
    for arg in FIRSTRUN_ARGS:
        if arg not in res.cmdline.split():
            raise BuildError(f"cmdline.txt in the built image is missing {arg!r}")
    listing = fat_list(out, boot.offset)
    res.injected = sorted(n for n in ("firstrun.sh", PAYLOAD_TAR_NAME, "userconf.txt",
                                      "ssh", "cmdline.txt")
                          if any(n == Path(x).name for x in listing))
    for want in ("firstrun.sh", PAYLOAD_TAR_NAME, "userconf.txt", "ssh"):
        if want not in res.injected:
            raise BuildError(f"{want} is not present in the built image's boot partition")

    if xz:
        if not quiet:
            print("  compressing image", file=sys.stderr)
        xz_out = Path(str(out) + ".xz")
        with out.open("rb") as fi, lzma.open(xz_out, "wb", preset=6) as fo:
            shutil.copyfileobj(fi, fo, 1 << 20)
        res.xz = str(xz_out)

    return res.to_dict()


def _index_with_archive_tags(texts: list[str]) -> tuple[dict[str, dict], dict[str, str]]:
    """`index_packages`, but stamping each paragraph with its archive tag so
    `download_debs` knows which base URL to prepend to `Filename`."""
    tagged: list[str] = []
    stamped: list[list[dict]] = []
    for (tag, _base, _comp), text in zip(ARCHIVES, texts):
        paras = parse_control(text)
        for p in paras:
            p["_archive"] = tag
        stamped.append(paras)
        tagged.append(tag)
    by_name: dict[str, dict] = {}
    provides: dict[str, str] = {}
    for paras in stamped:
        for p in paras:
            name = p.get("Package")
            if not name or name in by_name:
                continue
            by_name[name] = p
            for group in _split_relations(p.get("Provides", "")):
                for virt in group:
                    provides.setdefault(virt, name)
    return by_name, provides


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _print(rep: dict) -> None:
    print(f"image:      {rep['image']}")
    print(f"release:    {rep['release']}")
    print(f"boot @:     {rep['boot_offset']} bytes")
    print(f"injected:   {', '.join(rep['injected'])}")
    print(f"cmdline:    {rep['cmdline']}")
    pl = rep.get("payload", {})
    print(f"payload:    {len(pl.get('rendered', []))} rendered, "
          f"{len(pl.get('copied', []))} copied")
    debs = rep.get("debs", [])
    if debs:
        print(f"debs:       {len(debs)} ({sum(d['size'] for d in debs) / 1e6:.1f} MB)")
        for d in debs:
            print(f"              {d['package']}")
    if rep.get("xz"):
        print(f"compressed: {rep['xz']}")
    for w in rep.get("warnings", []):
        print(f"WARNING:    {w}")


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--secrets", required=True, type=Path,
                    help="path to your private pi-bridge secrets YAML")
    ap.add_argument("--release", default=DEFAULT_RELEASE, choices=sorted(RELEASES))
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--offline", action="store_true",
                    help="fail instead of downloading anything not already cached")
    ap.add_argument("--xz", action="store_true", help="also write a .img.xz")
    ap.add_argument("--skip-debs", action="store_true",
                    help="do not bundle the package closure (first boot then needs internet)")
    ap.add_argument("--allow-insecure-perms", action="store_true")
    ap.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)

    try:
        rep = build(args.secrets, release_key=args.release, out=args.out,
                    offline=args.offline, xz=args.xz, skip_debs=args.skip_debs,
                    quiet=args.json, allow_insecure_perms=args.allow_insecure_perms)
    except BuildError as e:
        if args.json:
            print(json.dumps({"ok": False, "error": str(e)}, indent=2))
        else:
            print(f"build_image: {e}", file=sys.stderr)
        return 2

    if args.json:
        print(json.dumps(rep, indent=2))
    else:
        _print(rep)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
