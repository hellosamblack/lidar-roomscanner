#!/bin/bash
# firstrun.sh -- Raspberry Pi Imager first-boot hook for the roomscan bridge Pi.
#
# Invoked because the image builder appends to cmdline.txt:
#   systemd.run=/boot/firmware/firstrun.sh systemd.run_success_action=reboot
#   systemd.unit=kernel-command-line.target
#
# Runs once, as root, very early (kernel-command-line.target), before normal
# multi-user boot. Responsibilities:
#   1. Log everything (stdout+stderr, timestamped) to /boot/firmware/firstrun.log
#      AND the console, so the log survives on the FAT partition and is
#      readable post-mortem by plugging the SD card into any laptop.
#   2. Extract the payload tarball to /opt/roomscan-bridge.
#   3. Run the installer in --first-boot mode.
#   4. Self-clean cmdline.txt (remove exactly the three systemd.* args this
#      mechanism added) so a normal reboot does not re-run this script.
#   5. Exit 0 so systemd's run_success_action=reboot fires.
#
# On failure: leave the log on FAT, do NOT strip cmdline.txt (so the
# first-boot mechanism fires again on retry after the operator fixes
# whatever broke), and print a loud banner before exiting non-zero.

set -euo pipefail

BOOT_DIR="/boot/firmware"
LOG_FILE="${BOOT_DIR}/firstrun.log"
CMDLINE_FILE="${BOOT_DIR}/cmdline.txt"
PAYLOAD_TARBALL="${BOOT_DIR}/roomscan-bridge-payload.tar.gz"
INSTALL_DIR="/opt/roomscan-bridge"

# Redirect all output (stdout+stderr) through a timestamping tee to both the
# log file on FAT and the console. This uses a process substitution so every
# line printed by this script (and anything it execs) gets a timestamp
# prefix without needing to pipe the whole script externally.
exec > >(while IFS= read -r line; do printf '%s %s\n' "$(date -u '+%Y-%m-%dT%H:%M:%SZ')" "$line"; done | tee -a "${LOG_FILE}") 2>&1

banner() {
    echo "=================================================================="
    echo "== $*"
    echo "=================================================================="
}

on_error() {
    local exit_code=$?
    banner "FIRSTRUN FAILED (exit ${exit_code}) -- log kept at ${LOG_FILE}"
    banner "cmdline.txt was NOT modified; reboot to retry after fixing the issue"
    exit "${exit_code}"
}
trap on_error ERR

banner "roomscan-bridge firstrun.sh starting"
echo "kernel: $(uname -a)"
echo "boot dir: ${BOOT_DIR}"

if [ ! -f "${PAYLOAD_TARBALL}" ]; then
    echo "ERROR: payload tarball not found at ${PAYLOAD_TARBALL}" >&2
    exit 1
fi

echo "Extracting ${PAYLOAD_TARBALL} to ${INSTALL_DIR}"
mkdir -p "${INSTALL_DIR}"
tar -xzf "${PAYLOAD_TARBALL}" -C "${INSTALL_DIR}"

if [ ! -x "${INSTALL_DIR}/install.sh" ]; then
    echo "ERROR: ${INSTALL_DIR}/install.sh missing or not executable after extraction" >&2
    exit 1
fi

banner "Running installer in --first-boot mode"
"${INSTALL_DIR}/install.sh" --first-boot

banner "Installer succeeded -- cleaning cmdline.txt"

if [ ! -f "${CMDLINE_FILE}" ]; then
    echo "ERROR: ${CMDLINE_FILE} not found, cannot self-clean" >&2
    exit 1
fi

# Remove exactly the three systemd.* args this mechanism added. cmdline.txt
# is a single line of space-separated args; do a targeted sed on that line
# rather than rewriting the whole file, so anything else on the line (and
# the trailing newline convention) is left alone.
sed -i \
    -e 's/[[:space:]]*systemd\.run=[^[:space:]]*//' \
    -e 's/[[:space:]]*systemd\.run_success_action=[^[:space:]]*//' \
    -e 's/[[:space:]]*systemd\.unit=kernel-command-line\.target//' \
    "${CMDLINE_FILE}"

echo "cmdline.txt after cleanup:"
cat "${CMDLINE_FILE}"

banner "roomscan-bridge firstrun.sh complete -- reboot will follow"

# Clear the trap so a clean exit doesn't print the failure banner.
trap - ERR
exit 0
