#!/bin/bash
# roomscan-bridge-common.sh -- shared helpers sourced by
# roomscan-bridge-reconcile, roomscan-bridge-status, and install.sh's
# in-flight-capture guard.
#
# This file is meant to be SOURCED, not executed. If it is run directly it
# is a harmless no-op (see the `return`/`exit` guard at the bottom) -- it
# is installed as 0755 alongside the other usr/local/sbin/* tools by
# install.sh's generic "install everything in this directory" loop, so it
# needs to survive being invoked as a script even though nothing does
# that on purpose.
#
# Defines the node-identity variables (SCANNER_IP, SCANNER_MAC,
# SCANNER_FALLBACK_IP, ETH_ADDR, STREAM_PORT, HOSTNAME) with the network
# design's fixed defaults, then overrides them from
# /etc/roomscan-bridge/node.env if that file is present -- so every
# consumer works standalone (defaults only) even before node.env exists,
# and picks up the real values once install.sh has installed it.

# --- defaults (overridden by /etc/roomscan-bridge/node.env when present) --
: "${SCANNER_IP:=172.31.100.20}"
: "${SCANNER_MAC:=00:80:e1:00:00:00}"
: "${SCANNER_FALLBACK_IP:=172.31.253.1}"
: "${ETH_ADDR:=172.31.100.1}"
: "${STREAM_PORT:=5000}"
: "${HOSTNAME:=roomscan-bridge}"

NODE_ENV_FILE="${NODE_ENV_FILE:-/etc/roomscan-bridge/node.env}"

# Loads /etc/roomscan-bridge/node.env if present, so its values override
# the built-in defaults above. Safe to call more than once (idempotent --
# re-sourcing the same KEY=VALUE lines just reassigns the same values).
roomscan_load_node_env() {
    if [ -f "${NODE_ENV_FILE}" ]; then
        # shellcheck disable=SC1090
        . "${NODE_ENV_FILE}"
    fi
}
roomscan_load_node_env

NFT_TABLE="ip roomscan_bridge"
NFT_CHAIN="prerouting"
DNAT_COMMENT="roomscan-dnat-to-scanner"

# --- nftables DNAT rule helpers -------------------------------------------

roomscan_dnat_rule_line() {
    nft list chain ${NFT_TABLE} ${NFT_CHAIN} 2>/dev/null | grep -F "${DNAT_COMMENT}" || true
}

roomscan_dnat_target_ip() {
    roomscan_dnat_rule_line | grep -oP 'dnat to \K[0-9.]+(?=:[0-9]+)' || true
}

roomscan_dnat_counter_bytes() {
    roomscan_dnat_rule_line | grep -oP 'bytes \K[0-9]+' || true
}

roomscan_dnat_rule_handle() {
    nft -a list chain ${NFT_TABLE} ${NFT_CHAIN} 2>/dev/null | grep -F "${DNAT_COMMENT}" | grep -oP 'handle \K[0-9]+' || true
}

# Retargets the DNAT rule's destination address, preserving port/comment.
# Returns non-zero (without touching anything) if the rule's handle
# cannot be found, e.g. the table isn't loaded.
roomscan_set_dnat_target() {
    local new_ip="$1"
    local handle
    handle="$(roomscan_dnat_rule_handle)"
    if [ -z "${handle}" ]; then
        return 1
    fi
    nft replace rule ${NFT_TABLE} ${NFT_CHAIN} handle "${handle}" \
        iifname "wlan0" udp dport "${STREAM_PORT}" counter dnat to "${new_ip}:${STREAM_PORT}" comment "${DNAT_COMMENT}"
}

# Samples the DNAT rule's byte counter twice, ${1:-1} seconds apart.
# Echoes "yes" if the counter grew FAST ENOUGH to be an actual scanner
# stream, "no" otherwise -- which also covers "couldn't read a counter at
# all" (e.g. table not loaded yet), so callers gating a destructive action
# on "no traffic seen" should keep in mind that "no" here means "no
# evidence of a stream", not a hardware-verified idle state.
#
# Why a RATE and not "did it increase at all" (issue #191):
#
#   The original test was `after > before`, i.e. any single byte counted as
#   a live capture. On the real rig that guard latched permanently: the
#   host's roomscan-web broadcasts a 1-byte discovery beacon to
#   255.255.255.255:5000 once a second, the DNAT rule matches it, and the
#   counter therefore ticks up ~25 B/s forever with no scanner traffic at
#   all. reconcile read that as "capture in progress" on every pass and so
#   NEVER bounced eth0 -- which is the one action that recovers a scanner
#   stuck in its self-assigned fallback mode. The bridge could see the
#   problem, knew the fix, and declined to apply it, indefinitely.
#
#   The real stream is ~466 KB/s (30 fps of ToF frames). The stray beacon is
#   ~25 B/s. Four orders of magnitude apart, so the threshold does not need
#   to be delicate: anything above a few KB/s is a stream and anything below
#   is noise. 20 KB/s is ~4% of the real rate -- low enough that even a
#   badly degraded capture still counts as live and is protected, high
#   enough that beacons, ARP, mDNS and probe traffic never trip it.
: "${STREAM_LIVE_MIN_BYTES_PER_SEC:=20480}"

roomscan_stream_is_live() {
    local sample_secs="${1:-1}"
    local before after delta threshold
    before="$(roomscan_dnat_counter_bytes)"
    [ -n "${before}" ] || before=0
    sleep "${sample_secs}"
    after="$(roomscan_dnat_counter_bytes)"
    [ -n "${after}" ] || after=0
    delta=$(( after - before ))
    threshold=$(( STREAM_LIVE_MIN_BYTES_PER_SEC * sample_secs ))
    if [ "${delta}" -ge "${threshold}" ]; then
        echo "yes"
    else
        echo "no"
    fi
}

# Make sure eth0 actually carries its static address, re-activating the
# NetworkManager profile if it does not.
#
# Why this is needed at all (issue #191). On the real Pi, eth0 came up with NO
# IPv4 address whatsoever, so dnsmasq had nothing to serve DHCP from and the
# scanner necessarily missed its 3000 ms window and self-assigned. NM reported:
#
#   eth0:ethernet:connected (externally):eth0
#
# "connected (externally)" means NM has decided the device was configured by
# something outside NM, so it will not autoconnect a profile onto it -- our
# roomscan-eth0 profile sat at autoconnect=yes, priority 100, bound to nothing.
# The something outside NM is US: the fallback probe adds and deletes a
# temporary alias with `ip addr add`, and the recovery path bounces the link
# with `ip link set eth0 down/up`. Poking a NM-managed device with iproute2 is
# what puts it into that state, and the state then blocks the address we need.
#
# So every path that touches eth0 directly calls this afterwards, and it is
# also called once at install time so a freshly provisioned box converges
# without waiting for a reconcile tick.
roomscan_ensure_eth0_address() {
    # Built from the same ETH_ADDR this library already defines, not a second
    # copy of the address -- two constants for one value drift apart.
    local want="${ETH_ADDR}/${ETH_PREFIX:-24}"
    if ip -4 -o addr show eth0 2>/dev/null | grep -qF "${want}"; then
        return 0
    fi
    if ! command -v nmcli >/dev/null 2>&1; then
        # No NM: assert the address directly rather than leave eth0 unusable.
        ip addr add "${want}" dev eth0 2>/dev/null || true
        return 0
    fi
    # `--wait 10`, never the default: `nmcli con up` blocks for up to NINETY
    # seconds, and this is reached from reconcile, which systemd fires every
    # 10 s.
    #
    # It does NOT stack concurrent nmcli processes -- systemd will not run two
    # instances of one service at once, so a slow call just makes reconcile
    # tick every ~90 s instead of every 10 s. (That pile-up was a stated
    # hypothesis for the unexplained 2026-08-18 post-reboot outage; it is
    # wrong, and the outage remains unexplained because the volatile journal
    # took the evidence with it. See the journald drop-in.)
    #
    # The real cost is the one that matters here: reconcile is the thing that
    # notices a scanner stuck in fallback mode, and a 90 s stall inside it is
    # 90 s of not noticing, on a 10 s duty cycle.
    nmcli --wait 10 con up roomscan-eth0 >/dev/null 2>&1 || true
    # Verify rather than assume -- `nmcli con up` can report success while the
    # device stays externally-managed.
    if ip -4 -o addr show eth0 2>/dev/null | grep -qF "${want}"; then
        return 0
    fi
    ip addr add "${want}" dev eth0 2>/dev/null || true
}

# Harmless no-op if this file is executed directly instead of sourced.
return 0 2>/dev/null || exit 0
