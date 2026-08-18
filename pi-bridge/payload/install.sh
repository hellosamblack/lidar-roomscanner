#!/bin/bash
# install.sh -- idempotent installer for the roomscan bridge payload.
#
# Shared by two callers:
#   1. firstrun.sh, immediately after extracting the payload tarball to
#      /opt/roomscan-bridge, invoked as:  install.sh --first-boot
#   2. bridge_update over ssh, re-run with no flags to push updated files to
#      an already-provisioned Pi:          install.sh
#
# --first-boot additionally:
#   - installs bundled .debs from /opt/roomscan-bridge/debs/*.deb via
#     `dpkg -i` (no network access assumed at this point in first boot)
#   - enables (rather than merely restarts) the managed systemd units
#
# --force overrides the in-flight-capture restart guard (see below).
#
# Both modes:
#   - install /opt/roomscan-bridge/node.env (if staged) to
#     /etc/roomscan-bridge/node.env and source it, EARLY, so every later
#     step sees real HOSTNAME/USERNAME/SCANNER_IP/etc rather than
#     roomscan-bridge-common.sh's built-in defaults
#   - sync this box's hostname (and /etc/hosts' 127.0.1.1 line) to
#     node.env's HOSTNAME
#   - apply node.env's WIFI_COUNTRY as the Wi-Fi regulatory domain
#   - copy the etc/ tree into /etc/, preserving modes
#   - install usr/local/sbin/* as 0755
#   - install a Wi-Fi credential override from
#     /boot/firmware/wifi-override.nmconnection if present, on every run
#   - install /opt/roomscan-bridge/authorized_keys (if staged) into the
#     node.env USERNAME's ~/.ssh/authorized_keys, merging rather than
#     clobbering
#   - reload/restart the managed units -- EXCEPT that in update mode
#     (no --first-boot), restarting dnsmasq/nftables is SKIPPED if the
#     nftables DNAT counter shows live stream traffic, so a config push
#     never interrupts an in-progress capture; --force overrides this
#
# Logging: every significant action goes to both stdout and `logger -t
# roomscan-bridge` so it lands in the journal regardless of how this script
# was invoked.
#
# Exit status:
#   0   completed, everything (including unit restarts) applied
#   1   a REQUIRED unit failed to come up -- hard failure
#   75  completed, but the dnsmasq/nftables restart was skipped because a
#       capture looked to be in progress (benign -- file changes ARE
#       installed; only the restart was deferred). Distinct from 1 so a
#       caller (bridge_update) can tell "everything's fine, try the
#       restart again later" apart from a real failure.
# Does not abort the whole script just because one OPTIONAL unit is
# missing or fails to (re)start -- it logs and continues.

set -euo pipefail

PAYLOAD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FIRST_BOOT=0
FORCE=0

for arg in "$@"; do
    case "${arg}" in
        --first-boot)
            FIRST_BOOT=1
            ;;
        --force)
            FORCE=1
            ;;
        *)
            echo "install.sh: unknown argument: ${arg}" >&2
            exit 2
            ;;
    esac
done

log() {
    echo "install.sh: $*"
    logger -t roomscan-bridge "install.sh: $*" || true
}

# Shared nft/DNAT helpers + node-identity defaults (SCANNER_IP,
# SCANNER_MAC, SCANNER_FALLBACK_IP, STREAM_PORT, HOSTNAME, ...),
# overridden below by node.env once install_node_env has installed it.
COMMON_LIB="${PAYLOAD_ROOT}/usr/local/sbin/roomscan-bridge-common.sh"
if [ -f "${COMMON_LIB}" ]; then
    # shellcheck disable=SC1090
    . "${COMMON_LIB}"
else
    log "WARNING: ${COMMON_LIB} not found in payload; using inline fallback defaults"
    : "${SCANNER_IP:=172.31.100.20}"
    : "${SCANNER_MAC:=00:80:e1:00:00:00}"
    : "${SCANNER_FALLBACK_IP:=172.31.253.1}"
    : "${STREAM_PORT:=5000}"
    NFT_TABLE="ip roomscan_bridge"
    NFT_CHAIN="prerouting"
    DNAT_COMMENT="roomscan-dnat-to-scanner"
    roomscan_dnat_counter_bytes() { nft list chain ${NFT_TABLE} ${NFT_CHAIN} 2>/dev/null | grep -F "${DNAT_COMMENT}" | grep -oP 'bytes \K[0-9]+' || true; }
    # Keep this in lockstep with roomscan-bridge-common.sh's copy: a RATE
    # test, not "did the counter move". Any-increase latches permanently on
    # the real rig, because the host's roomscan-web broadcasts a 1-byte
    # discovery beacon to 255.255.255.255:5000 every second and the DNAT rule
    # counts it (~25 B/s against a real stream's ~466 KB/s) -- see the long
    # note in roomscan-bridge-common.sh (issue #191).
    : "${STREAM_LIVE_MIN_BYTES_PER_SEC:=20480}"
    roomscan_stream_is_live() {
        local sample_secs="${1:-1}" before after
        before="$(roomscan_dnat_counter_bytes)"; [ -n "${before}" ] || before=0
        sleep "${sample_secs}"
        after="$(roomscan_dnat_counter_bytes)"; [ -n "${after}" ] || after=0
        if [ $(( after - before )) -ge $(( STREAM_LIVE_MIN_BYTES_PER_SEC * sample_secs )) ]; then
            echo "yes"
        else
            echo "no"
        fi
    }
fi

NODE_ENV_INSTALLED="/etc/roomscan-bridge/node.env"

# Units that MUST end up active; a failure here fails the whole install.
REQUIRED_UNITS=(
    "dnsmasq.service"
    "nftables.service"
    "avahi-daemon.service"
)

# Subset of REQUIRED_UNITS whose restart (in update mode) is guarded
# against interrupting an in-progress capture -- see capture_in_progress.
NET_RESTART_UNITS=(
    "dnsmasq.service"
    "nftables.service"
)

# Units that are nice-to-have; failures are logged but non-fatal.
OPTIONAL_UNITS=(
    "roomscan-tee.service"
    "roomscan-bridge-reconcile.timer"
)

# Set by activate_units(); read by main() after it returns.
ACTIVATE_FAILED_REQUIRED=0
ACTIVATE_SKIPPED_NET=0

require_root() {
    if [ "$(id -u)" -ne 0 ]; then
        log "ERROR: must run as root"
        exit 1
    fi
}

# ---------------------------------------------------------------------------
# node.env / hostname / Wi-Fi country
# ---------------------------------------------------------------------------

install_node_env() {
    local src="${PAYLOAD_ROOT}/node.env"
    if [ ! -f "${src}" ]; then
        log "WARNING: no node.env staged in payload root; USERNAME/HOSTNAME/WIFI_COUNTRY are unset and SCANNER_IP/STREAM_PORT/etc fall back to built-in defaults (SCANNER_IP=${SCANNER_IP}, STREAM_PORT=${STREAM_PORT}); hostname-sync, Wi-Fi-country, and authorized_keys steps will be skipped"
        return 0
    fi
    log "installing node.env to ${NODE_ENV_INSTALLED}"
    mkdir -p "$(dirname "${NODE_ENV_INSTALLED}")"
    install -o root -g root -m 0644 "${src}" "${NODE_ENV_INSTALLED}"
    # Re-source the now-canonical installed copy, EARLY, so every later
    # step in this run (hostname sync, Wi-Fi country, authorized_keys, the
    # restart guard) sees real values instead of roomscan-bridge-common.sh's
    # built-in defaults.
    # shellcheck disable=SC1090
    . "${NODE_ENV_INSTALLED}"
}

sync_hostname() {
    local wanted="${HOSTNAME:-}"
    if [ -z "${wanted}" ]; then
        log "no HOSTNAME available (node.env missing or lacks HOSTNAME), skipping hostname sync"
        return 0
    fi
    local current
    current="$(hostname 2>/dev/null || true)"
    if [ "${current}" != "${wanted}" ]; then
        log "hostname is '${current}', setting to '${wanted}'"
        if command -v hostnamectl >/dev/null 2>&1; then
            hostnamectl set-hostname "${wanted}" || log "WARNING: hostnamectl set-hostname failed"
        else
            echo "${wanted}" > /etc/hostname
        fi
    else
        log "hostname already '${wanted}', no change needed"
    fi

    # Keep /etc/hosts' 127.0.1.1 line in sync -- an unresolvable own
    # hostname makes sudo (and anything else doing a self name lookup)
    # hang for several seconds, which presents as a network fault when
    # it is actually just /etc/hosts drift.
    if grep -qE "^127\.0\.1\.1[[:space:]]+${wanted}([[:space:]]|\$)" /etc/hosts 2>/dev/null; then
        : # already correct, no-op
    elif grep -q '^127\.0\.1\.1[[:space:]]' /etc/hosts 2>/dev/null; then
        sed -i "s/^127\.0\.1\.1[[:space:]].*/127.0.1.1\t${wanted}/" /etc/hosts
        log "updated /etc/hosts 127.0.1.1 line to '${wanted}'"
    else
        printf '127.0.1.1\t%s\n' "${wanted}" >> /etc/hosts
        log "appended /etc/hosts 127.0.1.1 line for '${wanted}'"
    fi
}

install_wifi_country() {
    local country="${WIFI_COUNTRY:-}"
    if [ -z "${country}" ]; then
        log "no WIFI_COUNTRY available (node.env missing or lacks it), skipping regulatory-domain setup"
        return 0
    fi
    log "applying Wi-Fi regulatory domain: ${country}"

    # The Pi 3 radio stays soft-blocked / regulatory-limited until a
    # country is set on some images -- this presents as "Wi-Fi just
    # doesn't work" with no error, so every mechanism actually present on
    # this system is applied, logging which one(s) fired.
    if [ -f /etc/default/crda ]; then
        if grep -q '^REGDOMAIN=' /etc/default/crda; then
            sed -i "s/^REGDOMAIN=.*/REGDOMAIN=${country}/" /etc/default/crda
        else
            echo "REGDOMAIN=${country}" >> /etc/default/crda
        fi
        log "mechanism used: wrote REGDOMAIN=${country} to /etc/default/crda"
    else
        log "/etc/default/crda not present on this image, skipping that mechanism"
    fi

    if command -v raspi-config >/dev/null 2>&1; then
        if raspi-config nonint do_wifi_country "${country}"; then
            log "mechanism used: raspi-config nonint do_wifi_country ${country}"
        else
            log "WARNING: raspi-config nonint do_wifi_country ${country} failed"
        fi
    else
        log "raspi-config not present on this system, skipping that mechanism"
    fi

    if command -v iw >/dev/null 2>&1; then
        if iw reg set "${country}"; then
            log "mechanism used: iw reg set ${country}"
        else
            log "WARNING: iw reg set ${country} failed"
        fi
    else
        log "iw not present, skipping that mechanism"
    fi

    if command -v rfkill >/dev/null 2>&1; then
        if rfkill unblock wifi; then
            log "mechanism used: rfkill unblock wifi"
        else
            log "WARNING: rfkill unblock wifi failed"
        fi
    else
        log "rfkill not present, skipping that mechanism"
    fi
}

# ---------------------------------------------------------------------------
# debs / etc tree / nftables include / sbin tools / wifi override
# ---------------------------------------------------------------------------

install_debs() {
    local deb_dir="${PAYLOAD_ROOT}/debs"
    if [ ! -d "${deb_dir}" ]; then
        log "no debs/ directory present, skipping bundled package install"
        return 0
    fi
    shopt -s nullglob
    local debs=("${deb_dir}"/*.deb)
    shopt -u nullglob
    if [ "${#debs[@]}" -eq 0 ]; then
        log "debs/ directory present but empty, skipping"
        return 0
    fi
    log "installing ${#debs[@]} bundled .deb package(s) (no network)"
    dpkg -i "${debs[@]}" || {
        log "dpkg -i reported errors; attempting to continue (dependency ordering)"
    }
}

copy_etc_tree() {
    if [ ! -d "${PAYLOAD_ROOT}/etc" ]; then
        log "no etc/ tree in payload, skipping"
        return 0
    fi
    log "copying etc/ tree into /etc/, preserving modes"
    # cp -a preserves mode/ownership/timestamps and recurses; run per-file so
    # we can skip stray *.tmpl files defensively (the builder should already
    # have rendered these away, but never install an un-rendered template).
    while IFS= read -r -d '' src; do
        rel="${src#"${PAYLOAD_ROOT}"/etc/}"
        case "${rel}" in
            *.tmpl)
                log "WARNING: skipping unrendered template left in payload: ${rel}"
                continue
                ;;
        esac
        dest="/etc/${rel}"
        mkdir -p "$(dirname "${dest}")"
        cp -a "${src}" "${dest}"
    done < <(find "${PAYLOAD_ROOT}/etc" -type f -print0)

    # NetworkManager keyfiles are security-sensitive (contain PSKs) and NM
    # itself refuses to load a keyfile connection that is not 0600 root:root.
    if [ -d /etc/NetworkManager/system-connections ]; then
        log "tightening permissions on NetworkManager system-connections/"
        chown root:root /etc/NetworkManager/system-connections/*.nmconnection 2>/dev/null || true
        chmod 0600 /etc/NetworkManager/system-connections/*.nmconnection 2>/dev/null || true
    fi
}

ensure_nftables_include() {
    local conf="/etc/nftables.conf"
    local include_line='include "/etc/nftables/roomscan-bridge.nft";'
    if [ ! -f "${conf}" ]; then
        log "WARNING: ${conf} not found, cannot wire in roomscan-bridge.nft include"
        return 0
    fi
    if grep -qF "${include_line}" "${conf}"; then
        log "nftables.conf already includes roomscan-bridge.nft"
        return 0
    fi
    log "appending roomscan-bridge.nft include to ${conf}"
    {
        echo ""
        echo "# roomscan-bridge: load the Wi-Fi<->eth0 NAT/DNAT ruleset (see"
        echo "# /etc/nftables/roomscan-bridge.nft for the full explanation)."
        echo "${include_line}"
    } >> "${conf}"
}

ensure_dnsmasq_reads_dropins() {
    # Do not assume the vendor default in either direction (AGENTS.md, the
    # VL53L9_TRANSFORM_LIGHT lesson). Debian/Raspberry Pi OS DO read
    # /etc/dnsmasq.d, but *not* through /etc/dnsmasq.conf -- every `conf-dir=`
    # line there ships commented out. It works because /etc/default/dnsmasq
    # ships `CONFIG_DIR=/etc/dnsmasq.d,...` uncommented and the packaged
    # systemd-helper passes it as `-7 ${CONFIG_DIR}`.
    #
    # That is a mechanism we depend on completely and do not own: if CONFIG_DIR
    # is ever commented out or the helper changes, our whole dnsmasq config
    # silently does not load, dnsmasq still starts, every unit reports active --
    # and the scanner misses its 3000 ms DHCP window on every single boot,
    # falling back to self-assigned mode forever. So assert it rather than hope.
    local defaults="/etc/default/dnsmasq"
    local conf="/etc/dnsmasq.conf"
    if [ -f "${defaults}" ] && grep -qE '^[[:space:]]*CONFIG_DIR=' "${defaults}"; then
        log "dnsmasq reads /etc/dnsmasq.d via CONFIG_DIR in ${defaults}"
        return 0
    fi
    if [ -f "${conf}" ] && grep -qE '^[[:space:]]*conf-dir=/etc/dnsmasq\.d' "${conf}"; then
        log "dnsmasq reads /etc/dnsmasq.d via conf-dir in ${conf}"
        return 0
    fi
    if [ -f "${conf}" ]; then
        log "WARNING: neither ${defaults} (CONFIG_DIR) nor ${conf} (conf-dir) reads /etc/dnsmasq.d; appending conf-dir so our DHCP config is actually loaded"
        {
            echo ""
            echo "# roomscan-bridge: without this the drop-in in /etc/dnsmasq.d is never"
            echo "# read, dnsmasq starts clean, and the scanner never gets its lease."
            echo "conf-dir=/etc/dnsmasq.d,.dpkg-dist,.dpkg-old,.dpkg-new"
        } >> "${conf}"
        return 0
    fi
    log "ERROR: ${conf} not found -- cannot confirm /etc/dnsmasq.d is read"
    return 1
}

install_sbin_tools() {
    local src_dir="${PAYLOAD_ROOT}/usr/local/sbin"
    if [ ! -d "${src_dir}" ]; then
        log "no usr/local/sbin in payload, skipping"
        return 0
    fi
    mkdir -p /usr/local/sbin
    local f name
    for f in "${src_dir}"/*; do
        [ -f "${f}" ] || continue
        name="$(basename "${f}")"
        log "installing /usr/local/sbin/${name} (0755)"
        install -o root -g root -m 0755 "${f}" "/usr/local/sbin/${name}"
    done
}

install_wifi_override() {
    local override_src="/boot/firmware/wifi-override.nmconnection"
    local override_dest="/etc/NetworkManager/system-connections/roomscan-wifi-override.nmconnection"
    if [ ! -f "${override_src}" ]; then
        return 0
    fi
    log "installing Wi-Fi credential override from ${override_src}"
    install -o root -g root -m 0600 "${override_src}" "${override_dest}"
    if command -v nmcli >/dev/null 2>&1; then
        log "reloading NetworkManager connections after override install"
        nmcli connection reload || log "WARNING: nmcli connection reload failed"
    fi
}

# ---------------------------------------------------------------------------
# ssh key (merge, don't clobber)
# ---------------------------------------------------------------------------

install_authorized_key() {
    local key_src="${PAYLOAD_ROOT}/authorized_keys"
    local user="${USERNAME:-}"
    if [ ! -f "${key_src}" ]; then
        log "no authorized_keys staged in payload root, skipping ssh key install"
        return 0
    fi

    local key_line
    key_line="$(head -n 1 "${key_src}")"
    if [ -z "${key_line}" ]; then
        log "ERROR: ${key_src} is empty, skipping ssh key install"
        return 0
    fi

    # PRIMARY path: a root-owned key file that depends on no account and no
    # home directory. On a first boot install.sh runs from
    # kernel-command-line.target while the USERNAME account is not created
    # until userconf.txt is processed during normal boot -- so the
    # home-directory path below CANNOT work on the boot that matters, and on
    # the first real Pi it did not (issue #191: the box came up fully
    # provisioned and unreachable by key). /etc/ssh/sshd_config.d/
    # roomscan-bridge.conf points sshd here; see that file for why it is
    # scoped to Match User rather than set globally.
    local etc_auth="/etc/roomscan-bridge/authorized_keys"
    mkdir -p "$(dirname "${etc_auth}")"
    if [ -f "${etc_auth}" ] && grep -qF "${key_line}" "${etc_auth}"; then
        log "${etc_auth} already contains the bridge key, leaving untouched"
    else
        log "installing bridge ssh key to ${etc_auth} (account-independent path)"
        touch "${etc_auth}"
        echo "${key_line}" >> "${etc_auth}"
    fi
    # 0644 root:root: sshd refuses an AuthorizedKeysFile that is writable by
    # anyone but root, and this path is read by sshd running as root before
    # any privilege drop.
    chown root:root "${etc_auth}"
    chmod 0644 "${etc_auth}"

    # SECONDARY path, best-effort: also merge into the account's own
    # ~/.ssh/authorized_keys when the account does exist (every update-mode
    # run, and first boots on images that create the user earlier). This
    # keeps the box reachable if the sshd drop-in is ever removed, and keeps
    # a hand-run `ssh-copy-id` recovery consistent with what the payload
    # believes is installed.
    if [ -z "${user}" ]; then
        log "USERNAME is unset (node.env missing or lacks USERNAME), skipping the home-directory copy of the ssh key -- ${etc_auth} is installed and is what sshd reads"
        return 0
    fi
    local home
    # `|| home=""` is load-bearing under `set -euo pipefail`. `getent` exits 2 for an
    # unknown user, pipefail propagates that through the pipe, and because `local` is
    # declared on its own line above there is no assignment-masking -- so `set -e` would
    # kill install.sh HERE, three lines before the check below, with no log line and,
    # critically, before activate_units() ever enables dnsmasq/nftables/avahi. On a first
    # boot that also means firstrun.sh never reaches its `exit 0`, so systemd's
    # run_success_action=reboot never fires and the Pi looks bricked.
    home="$(getent passwd "${user}" | cut -d: -f6)" || home=""
    if [ -z "${home}" ]; then
        log "user '${user}' does not exist yet (expected on a first boot -- userconf.txt creates it later); skipping the home-directory copy. Remote administration still works via ${etc_auth}"
        return 0
    fi

    local ssh_dir="${home}/.ssh"
    local auth_file="${ssh_dir}/authorized_keys"

    mkdir -p "${ssh_dir}"
    chmod 0700 "${ssh_dir}"
    chown "${user}:${user}" "${ssh_dir}"

    if [ -f "${auth_file}" ] && grep -qF "${key_line}" "${auth_file}"; then
        log "authorized_keys for ${user} already contains the bridge key, leaving untouched"
    else
        log "adding bridge ssh key to ${auth_file}"
        touch "${auth_file}"
        echo "${key_line}" >> "${auth_file}"
    fi
    chmod 0600 "${auth_file}"
    chown "${user}:${user}" "${auth_file}"
}

# ---------------------------------------------------------------------------
# sudoers / sshd -- the two files that make remote administration possible
# ---------------------------------------------------------------------------

# Both of these were copied in by copy_etc_tree; this fixes up the modes and
# validates them, because a malformed sudoers file locks root out and a
# malformed sshd config kills the only way back in.
harden_admin_access() {
    local sudoers="/etc/sudoers.d/roomscan-bridge"
    if [ -f "${sudoers}" ]; then
        chown root:root "${sudoers}"
        chmod 0440 "${sudoers}"
        if command -v visudo >/dev/null 2>&1; then
            if visudo -cf "${sudoers}" >/dev/null 2>&1; then
                log "validated ${sudoers} with visudo -c"
            else
                log "ERROR: ${sudoers} FAILED visudo -c -- removing it rather than leaving a broken sudoers fragment that would break sudo for every user"
                rm -f "${sudoers}"
            fi
        else
            log "WARNING: visudo not present, installed ${sudoers} unvalidated"
        fi
    fi

    local sshd_dropin="/etc/ssh/sshd_config.d/roomscan-bridge.conf"
    if [ -f "${sshd_dropin}" ]; then
        chown root:root "${sshd_dropin}"
        chmod 0644 "${sshd_dropin}"
        # sshd -t validates the WHOLE effective config including our drop-in.
        # If it does not pass, take our file back out: an sshd that refuses to
        # start is unrecoverable without physical access to the SD card.
        if command -v sshd >/dev/null 2>&1 || [ -x /usr/sbin/sshd ]; then
            local sshd_bin
            sshd_bin="$(command -v sshd || echo /usr/sbin/sshd)"
            if "${sshd_bin}" -t >/dev/null 2>&1; then
                log "validated sshd config with sshd -t; reloading ssh"
                systemctl reload ssh 2>/dev/null || systemctl reload sshd 2>/dev/null || log "WARNING: could not reload ssh (it may not be running yet; the drop-in applies at next start)"
            else
                log "ERROR: sshd -t FAILED with ${sshd_dropin} installed -- removing it rather than risking an sshd that will not start"
                rm -f "${sshd_dropin}"
            fi
        fi
    fi
}

# ---------------------------------------------------------------------------
# tee state directory
# ---------------------------------------------------------------------------

# ---------------------------------------------------------------------------
# in-flight-capture restart guard
# ---------------------------------------------------------------------------

# True (0) if the DNAT counter shows live stream traffic right now, in
# which case dnsmasq/nftables restarts should be deferred rather than
# risk interrupting a recording. Always false in --first-boot mode
# (nothing is running yet to interrupt) and always false with --force.
capture_in_progress() {
    if [ "${FIRST_BOOT}" -eq 1 ]; then
        return 1
    fi
    if [ "${FORCE}" -eq 1 ]; then
        return 1
    fi
    if ! command -v nft >/dev/null 2>&1; then
        return 1
    fi
    if ! nft list table ip roomscan_bridge >/dev/null 2>&1; then
        # Table not loaded yet -- nothing to protect.
        return 1
    fi
    local live
    live="$(roomscan_stream_is_live 1)"
    [ "${live}" = "yes" ]
}

is_net_restart_unit() {
    local u="$1" x
    for x in "${NET_RESTART_UNITS[@]}"; do
        [ "${u}" = "${x}" ] && return 0
    done
    return 1
}

# ---------------------------------------------------------------------------
# unit activation
# ---------------------------------------------------------------------------

unit_exists() {
    systemctl list-unit-files "$1" --no-legend 2>/dev/null | grep -q "^$1"
}

# Settle time, in seconds, before believing a unit that "started fine".
UNIT_SETTLE_SECS="${UNIT_SETTLE_SECS:-6}"

# `systemctl enable --now` exiting 0 means "the start job succeeded", NOT
# "the service is still running". A Type=simple/forking unit that starts and
# then immediately dies -- or a Type=oneshot-ish daemon that fails its own
# config check a moment later -- gives systemctl a clean exit, and install.sh
# then reports `completed successfully` over a dead service.
#
# That is precisely what happened on the first real Pi (issue #191):
# install.sh logged `completed successfully` and exited 0 while TWO of its
# three REQUIRED units, dnsmasq and avahi-daemon, were crash-looping. The
# whole point of REQUIRED_UNITS is to fail the install when they do not come
# up, and it silently did not. Every other defect that boot -- no DHCP for
# the scanner, no mDNS -- reached the operator as "it worked" because of
# this one missing check.
#
# So: after starting things, wait out the restart window and ask systemd what
# is ACTUALLY active now. Anything that is not gets named, with the tail of
# its journal, which is the information a bring-up session actually needs.
verify_unit_active() {
    local unit="$1"
    local state
    state="$(systemctl is-active "${unit}" 2>/dev/null || true)"
    if [ "${state}" = "active" ]; then
        return 0
    fi
    # `activating` is a legitimate transient for a unit still coming up, but
    # it is ALSO what a crash-looping unit with Restart= looks like when
    # sampled mid-backoff, so it is not treated as success.
    log "ERROR: unit ${unit} is '${state}', not 'active', ${UNIT_SETTLE_SECS}s after being started -- recent journal follows"
    local line
    while IFS= read -r line; do
        log "  ${unit}: ${line}"
    done < <(journalctl -u "${unit}" -n 12 --no-pager -o cat 2>/dev/null || true)
    return 1
}

activate_units() {
    log "systemctl daemon-reload"
    systemctl daemon-reload || log "WARNING: daemon-reload failed"

    local skip_net=0
    if capture_in_progress; then
        skip_net=1
        ACTIVATE_SKIPPED_NET=1
        log "WARNING: nftables DNAT counter shows live stream traffic -- SKIPPING restart of ${NET_RESTART_UNITS[*]} so an in-progress capture is not interrupted. File changes ARE installed. Re-run install.sh (no flags) once the capture finishes, or re-run with --force to override now."
    fi

    local failed_required=0
    local unit

    # Clear any latched start-rate-limit BEFORE trying to (re)start anything.
    # A unit that already burned its StartLimitBurst answers every subsequent
    # start with `Start request repeated too quickly` and never runs the new
    # code -- so pushing the very fix for its crash appears to change nothing.
    # That happened on the real Pi: roomscan-tee's fix landed and the unit
    # stayed dead, because it was still rate-limited from the failures the fix
    # addressed (issue #191). "Update repairs a broken box" is the whole point
    # of this path, and a broken box is exactly where the latch is set.
    for unit in "${REQUIRED_UNITS[@]}" "${OPTIONAL_UNITS[@]}"; do
        unit_exists "${unit}" || continue
        systemctl reset-failed "${unit}" 2>/dev/null || true
    done

    for unit in "${REQUIRED_UNITS[@]}"; do
        if ! unit_exists "${unit}"; then
            log "ERROR: required unit ${unit} not found on this system"
            failed_required=1
            continue
        fi
        if [ "${FIRST_BOOT}" -eq 1 ]; then
            log "enabling + starting required unit ${unit}"
            if ! systemctl enable --now "${unit}"; then
                log "ERROR: failed to enable+start required unit ${unit}"
                failed_required=1
            fi
        else
            if [ "${skip_net}" -eq 1 ] && is_net_restart_unit "${unit}"; then
                log "skipping restart of ${unit} (capture in progress, see WARNING above)"
                continue
            fi
            log "restarting required unit ${unit}"
            if ! systemctl restart "${unit}"; then
                log "ERROR: failed to restart required unit ${unit}"
                failed_required=1
            fi
        fi
    done

    for unit in "${OPTIONAL_UNITS[@]}"; do
        if ! unit_exists "${unit}"; then
            log "optional unit ${unit} not found, skipping (non-fatal)"
            continue
        fi
        if [ "${FIRST_BOOT}" -eq 1 ]; then
            log "enabling + starting optional unit ${unit}"
            systemctl enable --now "${unit}" || log "WARNING: optional unit ${unit} failed to enable+start"
        else
            log "restarting optional unit ${unit}"
            systemctl restart "${unit}" || log "WARNING: optional unit ${unit} failed to restart"
        fi
    done

    # Second pass: what is actually still running? See verify_unit_active --
    # a clean `systemctl` exit is not evidence a service survived.
    log "waiting ${UNIT_SETTLE_SECS}s, then verifying units are still active"
    sleep "${UNIT_SETTLE_SECS}"

    for unit in "${REQUIRED_UNITS[@]}"; do
        unit_exists "${unit}" || continue
        if [ "${skip_net}" -eq 1 ] && is_net_restart_unit "${unit}"; then
            # Deliberately not restarted, so its current state is whatever the
            # in-progress capture is using -- not this run's to judge.
            continue
        fi
        verify_unit_active "${unit}" || failed_required=1
    done

    for unit in "${OPTIONAL_UNITS[@]}"; do
        unit_exists "${unit}" || continue
        # A .timer is active-or-waiting; is-active reports 'active' for both.
        verify_unit_active "${unit}" || log "WARNING: optional unit ${unit} is not active (non-fatal, but the capability it provides is absent)"
    done

    ACTIVATE_FAILED_REQUIRED="${failed_required}"
}

# ---------------------------------------------------------------------------

main() {
    require_root
    log "starting (first_boot=${FIRST_BOOT}, force=${FORCE})"

    install_node_env
    sync_hostname
    install_wifi_country

    if [ "${FIRST_BOOT}" -eq 1 ]; then
        install_debs
    fi

    copy_etc_tree
    ensure_nftables_include
    ensure_dnsmasq_reads_dropins
    install_sbin_tools
    install_wifi_override
    install_authorized_key
    harden_admin_access

    # Converge eth0's static address now rather than waiting for a reconcile
    # tick. On the real Pi eth0 came up with no IPv4 at all -- NM had it
    # "connected (externally)" and so never applied our profile -- which means
    # dnsmasq has nothing to serve DHCP from and the scanner ALWAYS misses its
    # 3000 ms window. See roomscan_ensure_eth0_address (issue #191).
    if command -v roomscan_ensure_eth0_address >/dev/null 2>&1 || \
       declare -f roomscan_ensure_eth0_address >/dev/null 2>&1; then
        log "ensuring eth0 carries its static address"
        roomscan_ensure_eth0_address || log "WARNING: could not assert eth0's static address"
    else
        log "WARNING: roomscan_ensure_eth0_address not available (old common.sh?); eth0's address is not being asserted"
    fi

    # Note for a future editor: do NOT add a step here that chowns the tee's
    # ring directory to `tcpdump`. It looks like the fix for the tee's
    # Permission denied failure and it is not -- systemd's StateDirectory=
    # re-asserts ownership matching the unit's User= on every start, silently
    # undoing it before ExecStart runs (measured on the real Pi, issue #191).
    # roomscan-tee.service declares User=tcpdump, which is what actually works.

    activate_units

    if [ "${ACTIVATE_FAILED_REQUIRED}" -ne 0 ]; then
        log "FAILED: one or more required units did not come up"
        exit 1
    fi

    if [ "${ACTIVATE_SKIPPED_NET}" -ne 0 ]; then
        log "COMPLETED WITH SKIPPED RESTART: all files installed, but ${NET_RESTART_UNITS[*]} were NOT restarted because a capture appeared to be in progress"
        echo "install.sh: SKIPPED-RESTART -- capture in progress, ${NET_RESTART_UNITS[*]} not restarted (re-run later, or use --force)"
        exit 75
    fi

    log "completed successfully"
    exit 0
}

main "$@"
