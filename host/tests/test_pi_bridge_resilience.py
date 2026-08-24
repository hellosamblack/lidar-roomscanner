"""Tests for the Pi bridge's power-loss resilience + SD-wear hardening
(issue #210).

Static payload/builder tests only -- see the header of `build_image.py` for
why: this host is rootless and has no loop devices, so nothing here mounts
an image, flashes a card, or pulls a plug. What a real first boot proves
(the fstab edits apply cleanly, the masked units stay masked, a genuine
power cut against a mounted card actually recovers) is an operator gate.
"""
from __future__ import annotations

import configparser
import importlib.util
import os
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
PI = REPO / "pi-bridge"
PAYLOAD = PI / "payload"
FSTAB_TOOL = PAYLOAD / "usr" / "local" / "sbin" / "roomscan-bridge-harden-fstab"


def _load_build_image():
    spec = importlib.util.spec_from_file_location("pi_bridge_build_image_resilience",
                                                    PI / "build_image.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = mod
    spec.loader.exec_module(mod)
    return mod


bi = _load_build_image()

REAL_CMDLINE = ("console=serial0,115200 console=tty1 root=PARTUUID=2f8c1a4d-02 "
                "rootfstype=ext4 rootwait quiet")

#: A cmdline that ALREADY carries one of our two args, as a stock Raspberry
#: Pi OS release can (some ship `fsck.repair=yes` by default). This is the
#: exact case `append_firstrun_args`'s strict "some but not all" rule would
#: wrongly reject.
CMDLINE_WITH_PARTIAL_OVERLAP = REAL_CMDLINE + " fsck.repair=yes"


# ---------------------------------------------------------------------------
# cmdline.txt: fsck.mode=force fsck.repair=yes (issue #210, tier: cmdline)
# ---------------------------------------------------------------------------

def test_ensure_cmdline_args_appends_every_missing_arg():
    out = bi.ensure_cmdline_args(REAL_CMDLINE, bi.RESILIENCE_CMDLINE_ARGS)
    assert out.endswith("\n")
    assert out.count("\n") == 1
    for arg in bi.RESILIENCE_CMDLINE_ARGS:
        assert arg in out.split()
    # Nothing else on the line was touched.
    assert out.startswith(REAL_CMDLINE)


def test_ensure_cmdline_args_is_idempotent():
    once = bi.ensure_cmdline_args(REAL_CMDLINE, bi.RESILIENCE_CMDLINE_ARGS)
    twice = bi.ensure_cmdline_args(once, bi.RESILIENCE_CMDLINE_ARGS)
    assert once == twice
    for arg in bi.RESILIENCE_CMDLINE_ARGS:
        assert twice.split().count(arg) == 1, twice


def test_ensure_cmdline_args_tolerates_a_cmdline_that_already_carries_one():
    """Unlike `append_firstrun_args`, a partial pre-existing match must not
    raise -- a stock cmdline.txt legitimately carrying `fsck.repair=yes`
    already is not a corrupted build, and `append_firstrun_args`'s strict
    all-or-nothing rule would wrongly reject every build against such a
    release."""
    out = bi.ensure_cmdline_args(CMDLINE_WITH_PARTIAL_OVERLAP, bi.RESILIENCE_CMDLINE_ARGS)
    assert out.split().count("fsck.repair=yes") == 1, "must not duplicate the pre-existing arg"
    assert "fsck.mode=force" in out.split(), "the missing arg must still be added"


def test_ensure_cmdline_args_refuses_a_multiline_cmdline():
    with pytest.raises(bi.BuildError, match="more than one line"):
        bi.ensure_cmdline_args(REAL_CMDLINE + "\nextra=1\n", bi.RESILIENCE_CMDLINE_ARGS)


def test_ensure_cmdline_args_refuses_an_empty_cmdline():
    with pytest.raises(bi.BuildError, match="empty"):
        bi.ensure_cmdline_args("   \n", bi.RESILIENCE_CMDLINE_ARGS)


def test_resilience_args_are_force_and_repair():
    # Pinning the exact values, not just that *some* fsck tuning exists:
    # fsck.mode=force is what makes fsck run even when the ext4 journal
    # replay would otherwise mask a dirty filesystem, and fsck.repair=yes is
    # what makes it unattended -- this box has no keyboard or screen to
    # answer an interactive prompt with.
    assert bi.RESILIENCE_CMDLINE_ARGS == ("fsck.mode=force", "fsck.repair=yes")


def test_firstrun_args_and_resilience_args_compose_on_one_line():
    """The real build() call chains append_firstrun_args then
    ensure_cmdline_args over the same cmdline -- both sets must survive the
    composition, in one line, with nothing dropped."""
    step1 = bi.append_firstrun_args(REAL_CMDLINE)
    step2 = bi.ensure_cmdline_args(step1, bi.RESILIENCE_CMDLINE_ARGS)
    assert step2.count("\n") == 1
    tokens = step2.split()
    for arg in bi.FIRSTRUN_ARGS:
        assert arg in tokens
    for arg in bi.RESILIENCE_CMDLINE_ARGS:
        assert arg in tokens


# ---------------------------------------------------------------------------
# journald: bounded persistence with an explicit sync interval
# ---------------------------------------------------------------------------

def _journald_conf() -> configparser.ConfigParser:
    cp = configparser.ConfigParser(strict=False, allow_no_value=True)
    cp.optionxform = str
    cp.read_string((PAYLOAD / "etc" / "systemd" / "journald.conf.d"
                    / "roomscan-bridge.conf").read_text())
    return cp


def test_journald_sync_interval_is_bounded_not_just_present():
    """Assert an interval, not a type (AGENTS.md's BUG-050 rule): a
    `SyncIntervalSec` field existing at all does not tell you whether it
    actually bounds fsync-driven SD writes -- a 1-second value would defeat
    the whole point just as surely as an absent one."""
    cp = _journald_conf()
    raw = cp["Journal"]["SyncIntervalSec"]
    assert raw.endswith("min"), raw
    minutes = int(raw[:-3])
    assert 1 <= minutes <= 15, f"SyncIntervalSec={raw} is not a sane bound"


def test_journald_storage_is_still_persistent_and_bounded():
    """Regression guard for the existing #191/#200 fix: this issue must not
    regress it back toward volatile storage while tuning sync behaviour."""
    cp = _journald_conf()
    j = cp["Journal"]
    assert j["Storage"] == "persistent"
    assert j["SystemMaxUse"].endswith("M")
    assert int(j["SystemMaxUse"][:-1]) <= 128, \
        "the journal must stay small next to the tee's 2 GB ring"


# ---------------------------------------------------------------------------
# fstab hardening script: noatime / commit=60 / tmpfs /tmp
# ---------------------------------------------------------------------------

def _run_fstab_tool(fstab_path: Path) -> subprocess.CompletedProcess:
    env = dict(os.environ, FSTAB=str(fstab_path))
    return subprocess.run([str(FSTAB_TOOL)], env=env, capture_output=True, text=True)


STOCK_FSTAB = (
    "proc            /proc           proc    defaults          0       0\n"
    "PARTUUID=abc-01 /boot/firmware  vfat    defaults          0       2\n"
    "PARTUUID=abc-02 /               ext4    defaults,noatime  0       1\n"
)


def test_fstab_tool_is_executable_and_parses():
    assert os.access(FSTAB_TOOL, os.X_OK)
    r = subprocess.run(["bash", "-n", str(FSTAB_TOOL)], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_fstab_tool_adds_noatime_and_commit_to_root(tmp_path):
    fstab = tmp_path / "fstab"
    fstab.write_text(STOCK_FSTAB)
    r = _run_fstab_tool(fstab)
    assert r.returncode == 0, r.stderr

    lines = {ln.split()[1]: ln for ln in fstab.read_text().splitlines()
             if ln.strip() and not ln.strip().startswith("#") and len(ln.split()) >= 4}
    root_opts = lines["/"].split()[3].split(",")
    assert "noatime" in root_opts
    assert "commit=60" in root_opts
    # The pre-existing option on that line must survive the edit.
    assert "defaults" in root_opts


def test_fstab_tool_adds_noatime_to_boot_firmware(tmp_path):
    fstab = tmp_path / "fstab"
    fstab.write_text(STOCK_FSTAB)
    _run_fstab_tool(fstab)
    lines = {ln.split()[1]: ln for ln in fstab.read_text().splitlines() if ln.strip()}
    boot_opts = lines["/boot/firmware"].split()[3].split(",")
    assert "noatime" in boot_opts


def test_fstab_tool_appends_a_bounded_tmpfs_line_for_tmp(tmp_path):
    fstab = tmp_path / "fstab"
    fstab.write_text(STOCK_FSTAB)
    _run_fstab_tool(fstab)
    tmp_lines = [ln for ln in fstab.read_text().splitlines()
                 if ln.split() and len(ln.split()) >= 2 and ln.split()[1] == "/tmp"]
    assert len(tmp_lines) == 1, tmp_lines
    fields = tmp_lines[0].split()
    assert fields[0] == "tmpfs" and fields[2] == "tmpfs"
    assert "size=" in fields[3], "an unbounded tmpfs /tmp can exhaust RAM"


def test_fstab_tool_is_idempotent(tmp_path):
    fstab = tmp_path / "fstab"
    fstab.write_text(STOCK_FSTAB)
    _run_fstab_tool(fstab)
    after_first = fstab.read_text()
    _run_fstab_tool(fstab)
    after_second = fstab.read_text()
    assert after_first == after_second, "a second run must not duplicate any edit"


def test_fstab_tool_does_not_error_when_boot_firmware_is_absent(tmp_path):
    """Not every image ships a separate /boot/firmware line -- must be a
    no-op for that mount point, not a failure that aborts the root-fs edit
    too."""
    fstab = tmp_path / "fstab"
    fstab.write_text(
        "proc            /proc           proc    defaults          0       0\n"
        "PARTUUID=abc-02 /               ext4    defaults          0       1\n"
    )
    r = _run_fstab_tool(fstab)
    assert r.returncode == 0, r.stderr
    lines = {ln.split()[1]: ln for ln in fstab.read_text().splitlines() if ln.strip()}
    assert "noatime" in lines["/"].split()[3].split(",")


def test_fstab_tool_never_touches_var_log_or_the_tee_spool(tmp_path):
    """The whole point of keeping these two on real storage (see the
    script's own header): a future edit must not silently start tmpfs-ing
    them and destroy the postmortem evidence #191/#200 exist to keep."""
    fstab = tmp_path / "fstab"
    fstab.write_text(STOCK_FSTAB)
    _run_fstab_tool(fstab)
    mountpoints = {ln.split()[1] for ln in fstab.read_text().splitlines()
                   if len(ln.split()) >= 2}
    assert "/var/log" not in mountpoints
    assert "/var/lib/roomscan-bridge/tee" not in mountpoints
    assert "/var/lib/roomscan-bridge" not in mountpoints


def test_fstab_tool_leaves_a_missing_fstab_alone():
    r = subprocess.run([str(FSTAB_TOOL)],
                       env=dict(os.environ, FSTAB="/nonexistent/fstab-does-not-exist"),
                       capture_output=True, text=True)
    assert r.returncode == 0
    assert "nothing to do" in r.stderr


# ---------------------------------------------------------------------------
# install.sh wiring: harden_storage is called, and masks the right units
# ---------------------------------------------------------------------------

def _install_text() -> str:
    return (PAYLOAD / "install.sh").read_text()


def test_harden_storage_is_defined_and_called_from_main():
    text = _install_text()
    assert "harden_storage()" in text
    main_body = text.split("main() {", 1)[1].split("\n}", 1)[0]
    assert "harden_storage" in main_body, \
        "harden_storage is defined but never called from main()"
    # Must run after the tool it depends on is actually installed on disk.
    idx_sbin = main_body.index("install_sbin_tools")
    idx_harden = main_body.index("harden_storage")
    assert idx_sbin < idx_harden, \
        "harden_storage runs roomscan-bridge-harden-fstab before install_sbin_tools installs it"


def test_harden_storage_masks_swap_and_rsyslog():
    text = _install_text()
    body = text.split("harden_storage() {", 1)[1].split("\n}", 1)[0]
    assert "dphys-swapfile.service" in body
    assert "rsyslog.service" in body
    assert "swapoff" in body


def test_mask_if_present_never_assumes_the_unit_exists():
    """AGENTS.md's vendor-default rule: never assume a package is present or
    absent without checking. `mask_if_present` must gate on `unit_exists`,
    not mask unconditionally."""
    text = _install_text()
    body = text.split("mask_if_present() {", 1)[1].split("\n}", 1)[0]
    assert "unit_exists" in body
    assert "systemctl mask" in body


def test_harden_storage_runs_the_fstab_tool_non_fatally():
    text = _install_text()
    body = text.split("harden_storage() {", 1)[1].split("\n}", 1)[0]
    assert "roomscan-bridge-harden-fstab" in body
    # A failure here must not abort the whole install (the same
    # "structurally fail-open" pattern as the tee and healthlog units).
    call_lines = [ln for ln in body.splitlines() if "fstab_tool}\"" in ln and "-x" not in ln]
    assert call_lines
    assert any("||" in ln for ln in call_lines), \
        f"the fstab tool call is not guarded against failure: {call_lines}"


def test_masked_units_are_not_in_required_units():
    """A masked unit must never be something the install fails over."""
    text = _install_text()
    required_block = text.split("REQUIRED_UNITS=(", 1)[1].split(")", 1)[0]
    assert "dphys-swapfile" not in required_block
    assert "rsyslog" not in required_block


# ---------------------------------------------------------------------------
# install.sh cmdline resilience args (issue #213)
# ---------------------------------------------------------------------------

def _extract_install_resilience_args() -> tuple[str, ...]:
    text = _install_text()
    match = re.search(r"RESILIENCE_CMDLINE_ARGS=\(\s*([^)]+)\s*\)", text)
    assert match, "RESILIENCE_CMDLINE_ARGS array not found in install.sh"
    raw = match.group(1).split()
    return tuple(a.strip('"\'') for a in raw if a.strip())


def _run_install_ensure_cmdline(cmdline_path: Path | None) -> subprocess.CompletedProcess:
    env_str = f"CMDLINE_FILE='{cmdline_path}'" if cmdline_path is not None else ""
    script = (
        f"source {PAYLOAD / 'install.sh'}\n"
        f"{env_str} ensure_cmdline_args\n"
    )
    return subprocess.run(["bash", "-c", script], capture_output=True, text=True)


def test_install_sh_resilience_args_match_build_image():
    """Guard against divergence: install.sh and build_image.py must declare the
    exact same kernel cmdline resilience arguments (issue #213)."""
    from_install = _extract_install_resilience_args()
    assert from_install == bi.RESILIENCE_CMDLINE_ARGS


def test_ensure_cmdline_args_is_defined_and_called_from_main():
    text = _install_text()
    assert "ensure_cmdline_args()" in text
    main_body = text.split("main() {", 1)[1].split("\n}", 1)[0]
    assert "ensure_cmdline_args" in main_body, \
        "ensure_cmdline_args is defined but never called from main()"
    idx_harden = main_body.index("harden_storage")
    idx_cmdline = main_body.index("ensure_cmdline_args")
    assert idx_cmdline > idx_harden, \
        "ensure_cmdline_args should be called in main() next to harden_storage"


def test_install_ensure_cmdline_args_appends_missing_args(tmp_path):
    cmdline = tmp_path / "cmdline.txt"
    cmdline.write_text(REAL_CMDLINE + "\n")
    r = _run_install_ensure_cmdline(cmdline)
    assert r.returncode == 0, r.stderr
    content = cmdline.read_text()
    assert content.endswith("\n")
    assert content.count("\n") == 1
    for arg in bi.RESILIENCE_CMDLINE_ARGS:
        assert arg in content.split()
    assert content.startswith(REAL_CMDLINE)


def test_install_ensure_cmdline_args_is_idempotent(tmp_path):
    cmdline = tmp_path / "cmdline.txt"
    cmdline.write_text(REAL_CMDLINE + "\n")
    _run_install_ensure_cmdline(cmdline)
    after_first = cmdline.read_text()
    _run_install_ensure_cmdline(cmdline)
    after_second = cmdline.read_text()
    assert after_first == after_second
    for arg in bi.RESILIENCE_CMDLINE_ARGS:
        assert after_second.split().count(arg) == 1


def test_install_ensure_cmdline_args_handles_partial_overlap(tmp_path):
    cmdline = tmp_path / "cmdline.txt"
    cmdline.write_text(CMDLINE_WITH_PARTIAL_OVERLAP + "\n")
    r = _run_install_ensure_cmdline(cmdline)
    assert r.returncode == 0, r.stderr
    content = cmdline.read_text()
    assert content.split().count("fsck.repair=yes") == 1
    assert "fsck.mode=force" in content.split()


def test_install_ensure_cmdline_args_never_invents_missing_file(tmp_path):
    nonexistent = tmp_path / "nonexistent" / "cmdline.txt"
    r = _run_install_ensure_cmdline(nonexistent)
    assert r.returncode == 0, r.stderr
    assert not nonexistent.exists()


def test_install_ensure_cmdline_args_refuses_empty_and_multiline(tmp_path):
    empty = tmp_path / "empty_cmdline.txt"
    empty.write_text("")
    r = _run_install_ensure_cmdline(empty)
    assert r.returncode == 0
    assert empty.read_text() == ""

    multiline = tmp_path / "multi_cmdline.txt"
    multi_content = REAL_CMDLINE + "\nextra=1\n"
    multiline.write_text(multi_content)
    r = _run_install_ensure_cmdline(multiline)
    assert r.returncode == 0
    assert multiline.read_text() == multi_content


# ---------------------------------------------------------------------------
# The new script must not be swallowed by .gitignore (the *.d trap, #191)
# ---------------------------------------------------------------------------

def test_the_fstab_tool_is_not_git_ignored():
    r = subprocess.run(["git", "check-ignore", "-q",
                        str(FSTAB_TOOL.relative_to(REPO))], cwd=REPO)
    assert r.returncode == 1, \
        f"{FSTAB_TOOL} is git-ignored (check-ignore exit {r.returncode})"
