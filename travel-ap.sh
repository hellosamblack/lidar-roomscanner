#!/usr/bin/env bash
# Configure and control the roomscanner travel AP from the Proxmox host.
#
# The AP is never started while eno1 has physical carrier or while the radio
# can see the protected SSID.  It also fails closed when the SSID scan fails.
set -euo pipefail

readonly TOOL_NAME="roomscanner-travel-ap"
readonly INSTALL_PATH="/usr/local/sbin/${TOOL_NAME}"
readonly HOST_CONFIG="/etc/roomscanner/travel-ap.conf"
readonly CT_CONFIG="/etc/roomscanner/travel-ap.conf"

die() { echo "${TOOL_NAME}: $*" >&2; exit 1; }
note() { echo "${TOOL_NAME}: $*"; }

usage() {
    cat <<'EOF'
Usage (run on the Proxmox host as root):
  sudo ./travel-ap.sh install --secrets /root/travel-ap-secrets.yaml
  sudo ./travel-ap.sh install --secrets /root/travel-ap-secrets.yaml --profile test
  sudo ./travel-ap.sh {auto|reconcile|up|down|status}

install options:
  --secrets FILE   Root-only YAML file containing both profiles (required)
  --profile NAME   Select test or production non-interactively; otherwise prompt
  CT_ID            Proxmox container ID (default: 300)
  WIRED_IF         Physical Ethernet interface (default: eno1)
  WIFI_IF          Wi-Fi interface inside the container (default: wlan0)
  WIFI_HOST_IF     Matching physical Wi-Fi interface on the Proxmox host
                   (default: wlp0s20f3; used only for validation)
  AP_CIDR          AP/gateway address (default: 192.168.50.1/24)
  DHCP_RANGE       DHCP range (default: 192.168.50.10,192.168.50.150,255.255.255.0,24h)
  WIFI_CHANNEL     2.4 GHz channel (default: 6)

--secrets FILE has exactly two profiles, each with an SSID and passphrase:
  test:
    ssid: Surly_Office_Test
    passphrase: replace-with-a-test-WPA2-passphrase
  production:
    ssid: Surly_Office
    passphrase: replace-with-a-production-WPA2-passphrase

The file must be owned by root and unreadable by group/other (mode 0600 or
stricter).  YAML parsing requires the Proxmox host package python3-yaml.

The selected profile's SSID is always the protected SSID: the AP starts only
when Ethernet has no carrier and that same SSID is absent from a Wi-Fi scan.

The tool must be installed only after WIFI_HOST_IF has been passed through to
the container as WIFI_IF.  It never modifies the Proxmox LXC configuration or
restarts the container.

Home-test example (the test AP stays off while eno1 has carrier):
  sudo ./travel-ap.sh install --secrets /root/travel-ap-secrets.yaml

To test AP startup, unplug eno1 after installation, or run it on an isolated
test host.  `up` deliberately obeys the Ethernet and protected-SSID gates; it
does not provide a force-start bypass.  `down` disables automatic AP control;
run `auto` to restore it.
EOF
}

require_root_and_host() {
    [[ ${EUID} -eq 0 ]] || die "run as root on the Proxmox host"
    command -v pct >/dev/null || die "pct is required (run this on the Proxmox host)"
}

write_kv_file() {
    local file=$1; shift
    install -d -m 0700 "$(dirname "$file")"
    : >"$file"
    chmod 0600 "$file"
    local key value
    while (($#)); do
        key=$1; value=$2; shift 2
        printf '%s=%q\n' "$key" "$value" >>"$file"
    done
}

load_config() {
    [[ -r $HOST_CONFIG ]] || die "not installed; run install first"
    # Written by write_kv_file; it is root-owned and mode 0600.
    # shellcheck disable=SC1090
    source "$HOST_CONFIG"
}

yaml_python() {
    python3 - "$@" <<'PY'
import base64
import sys
try:
    import yaml
except ImportError:
    raise SystemExit("python3-yaml is required: apt-get install python3-yaml")

with open(sys.argv[1], encoding="utf-8") as stream:
    data = yaml.safe_load(stream)
if not isinstance(data, dict) or set(data) != {"test", "production"}:
    raise SystemExit("secrets YAML must contain exactly 'test' and 'production' mappings")
for name in ("test", "production"):
    profile = data[name]
    if not isinstance(profile, dict) or set(profile) != {"ssid", "passphrase"}:
        raise SystemExit(f"{name} must contain exactly string keys 'ssid' and 'passphrase'")
    if not all(isinstance(profile[key], str) for key in ("ssid", "passphrase")):
        raise SystemExit(f"{name}.ssid and {name}.passphrase must be YAML strings")

action = sys.argv[2]
if action == "list":
    print("test")
    print("production")
else:
    if action not in data:
        raise SystemExit("profile must be 'test' or 'production'")
    for key in ("ssid", "passphrase"):
        encoded = base64.b64encode(data[action][key].encode()).decode()
        print(f"{key}:{encoded}")
PY
}

choose_profile() {
    local file=$1 profiles choice
    profiles=$(yaml_python "$file" list) || die "could not read profiles from $file"
    [[ -t 0 ]] || die "--profile is required without an interactive terminal"
    echo "Choose travel AP profile:" >&2
    select choice in $profiles; do
        [[ -n ${choice:-} ]] || { echo "Enter 1 or 2." >&2; continue; }
        printf '%s\n' "$choice"
        return
    done
}

validate_secrets_file() {
    local file=$1 mode owner
    [[ -f $file ]] || die "secrets file does not exist: $file"
    owner=$(stat -c '%u' "$file")
    mode=$(stat -c '%a' "$file")
    [[ $owner == 0 ]] || die "secrets file must be owned by root: $file"
    (( (8#$mode & 077) == 0 )) || die "secrets file must not be readable by group/other: $file"
    command -v python3 >/dev/null || die "python3 is required to read YAML secrets"
}

load_secrets_yaml() {
    local file=$1 profile=$2 line key encoded value
    validate_secrets_file "$file"

    # yaml.safe_load rejects arbitrary object construction.  Values are emitted
    # base64-encoded, avoiding eval and preserving shell punctuation.
    while IFS= read -r line; do
        key=${line%%:*}
        encoded=${line#*:}
        value=$(printf '%s' "$encoded" | base64 -d)
        case $key in
            ssid) AP_SSID=$value ;;
            passphrase) AP_PASSPHRASE=$value ;;
            *) die "unsupported YAML key in secrets file: $key" ;;
        esac
    done < <(yaml_python "$file" "$profile")
    [[ -n ${AP_SSID:-} && -n ${AP_PASSPHRASE:-} ]] || die "could not read selected profile from $file"
    PROTECTED_SSID=$AP_SSID
}

ct_exec() { pct exec "$CT_ID" -- "$@"; }

carrier() {
    local path="/sys/class/net/${WIRED_IF}/carrier"
    [[ -r $path ]] || die "wired interface $WIRED_IF has no carrier file"
    cat "$path"
}

validate_install_inputs() {
    [[ -n ${AP_SSID:-} ]] || die "AP_SSID is required"
    [[ -n ${AP_PASSPHRASE:-} ]] || die "AP_PASSPHRASE is required"
    [[ ${#AP_SSID} -le 32 && $AP_SSID != *$'\n'* ]] || die "AP_SSID must be at most 32 bytes and one line"
    [[ ${#AP_PASSPHRASE} -ge 8 && ${#AP_PASSPHRASE} -le 63 && $AP_PASSPHRASE != *$'\n'* ]] ||
        die "AP_PASSPHRASE must be 8–63 characters and one line"
    [[ ${WIFI_CHANNEL:-6} =~ ^[0-9]+$ ]] || die "WIFI_CHANNEL must be numeric"
}

install_ct_files() {
    # AP_* values are passed in the environment and never logged.  The
    # container persists only its own 0600 config file and hostapd.conf
    # (also 0600).
    pct exec "$CT_ID" -- env \
        AP_SSID="$AP_SSID" AP_PASSPHRASE="$AP_PASSPHRASE" \
        PROTECTED_SSID="$PROTECTED_SSID" WIFI_IF="$WIFI_IF" \
        AP_CIDR="$AP_CIDR" DHCP_RANGE="$DHCP_RANGE" WIFI_CHANNEL="$WIFI_CHANNEL" \
        bash -s <<'CT_SCRIPT'
set -euo pipefail
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y hostapd dnsmasq iw
systemctl stop hostapd dnsmasq || true
systemctl disable hostapd dnsmasq || true
systemctl unmask hostapd

install -d -m 0700 /etc/roomscanner /etc/dnsmasq.d
umask 077
cat > /etc/roomscanner/travel-ap.conf <<EOF
AP_SSID=$(printf '%q' "$AP_SSID")
AP_PASSPHRASE=$(printf '%q' "$AP_PASSPHRASE")
PROTECTED_SSID=$(printf '%q' "$PROTECTED_SSID")
WIFI_IF=$(printf '%q' "$WIFI_IF")
AP_CIDR=$(printf '%q' "$AP_CIDR")
DHCP_RANGE=$(printf '%q' "$DHCP_RANGE")
WIFI_CHANNEL=$(printf '%q' "$WIFI_CHANNEL")
EOF
chmod 0600 /etc/roomscanner/travel-ap.conf

cat > /etc/dnsmasq.d/roomscanner-travel.conf <<EOF
interface=$WIFI_IF
bind-interfaces
dhcp-range=$DHCP_RANGE
EOF

cat > /etc/hostapd/hostapd.conf <<EOF
interface=$WIFI_IF
driver=nl80211
ssid=$AP_SSID
hw_mode=g
channel=$WIFI_CHANNEL
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$AP_PASSPHRASE
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF
chmod 0600 /etc/hostapd/hostapd.conf
sed -i 's|^#\?DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd

cat > /etc/systemd/system/travel-ap-network.service <<'EOF'
[Unit]
Description=Roomscanner travel AP network address
Before=hostapd.service dnsmasq.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/bash -c '. /etc/roomscanner/travel-ap.conf; /usr/sbin/ip link set "$WIFI_IF" up; /usr/sbin/ip addr replace "$AP_CIDR" dev "$WIFI_IF"'
ExecStop=/bin/bash -c '. /etc/roomscanner/travel-ap.conf; /usr/sbin/ip addr del "$AP_CIDR" dev "$WIFI_IF" || true'
EOF

install -d /etc/systemd/system/hostapd.service.d /etc/systemd/system/dnsmasq.service.d
cat > /etc/systemd/system/hostapd.service.d/travel-ap.conf <<'EOF'
[Unit]
Requires=travel-ap-network.service
After=travel-ap-network.service
EOF
cp /etc/systemd/system/hostapd.service.d/travel-ap.conf /etc/systemd/system/dnsmasq.service.d/travel-ap.conf

cat > /usr/local/sbin/roomscanner-travel-ap-reconcile <<'EOF'
#!/bin/bash
set -euo pipefail
source /etc/roomscanner/travel-ap.conf

# Do not briefly stop a running AP just to scan: that disconnects clients.
if systemctl is-active --quiet hostapd; then
    exit 0
fi
systemctl stop hostapd dnsmasq travel-ap-network || true
/usr/sbin/ip link set "$WIFI_IF" up

if ! scan=$(iw dev "$WIFI_IF" scan 2>/dev/null); then
    logger -t roomscanner-travel-ap 'Wi-Fi scan failed; travel AP remains off'
    exit 0
fi
if printf '%s\n' "$scan" | awk -F': ' '/^[[:space:]]*SSID: / {print $2}' | grep -Fqx -- "$PROTECTED_SSID"; then
    logger -t roomscanner-travel-ap "saw protected SSID $PROTECTED_SSID; travel AP remains off"
    exit 0
fi
systemctl start travel-ap-network hostapd dnsmasq
logger -t roomscanner-travel-ap 'no protected SSID seen; travel AP started'
EOF
chmod 0755 /usr/local/sbin/roomscanner-travel-ap-reconcile
systemctl daemon-reload
CT_SCRIPT
}

cmd_install() {
    require_root_and_host
    local secrets_file= profile=
    while (($#)); do
        case $1 in
            --secrets)
                (($# >= 2)) || die "--secrets requires a file path"
                secrets_file=$2
                shift 2
                ;;
            --profile)
                (($# >= 2)) || die "--profile requires test or production"
                profile=$2
                shift 2
                ;;
            *) die "unknown install option: $1" ;;
        esac
    done
    [[ -n $secrets_file ]] || die "install requires --secrets FILE"
    [[ -z $profile || $profile == test || $profile == production ]] ||
        die "--profile must be test or production"
    validate_secrets_file "$secrets_file"
    [[ -n $profile ]] || profile=$(choose_profile "$secrets_file")
    AP_SSID= AP_PASSPHRASE= PROTECTED_SSID=
    load_secrets_yaml "$secrets_file" "$profile"
    CT_ID=${CT_ID:-300}
    WIRED_IF=${WIRED_IF:-eno1}
    WIFI_IF=${WIFI_IF:-wlan0}
    WIFI_HOST_IF=${WIFI_HOST_IF:-wlp0s20f3}
    AP_CIDR=${AP_CIDR:-192.168.50.1/24}
    DHCP_RANGE=${DHCP_RANGE:-192.168.50.10,192.168.50.150,255.255.255.0,24h}
    WIFI_CHANNEL=${WIFI_CHANNEL:-6}
    validate_install_inputs
    [[ -r "/sys/class/net/${WIRED_IF}/carrier" ]] || die "no physical carrier file for $WIRED_IF"
    pct status "$CT_ID" | grep -Fqx 'status: running' || die "CT $CT_ID must be running"
    pct config "$CT_ID" | grep -Fq "lxc.net.1.link: ${WIFI_HOST_IF}" ||
        die "CT $CT_ID does not appear to own $WIFI_HOST_IF; configure Wi-Fi passthrough first"

    write_kv_file "$HOST_CONFIG" \
        CT_ID "$CT_ID" WIRED_IF "$WIRED_IF" WIFI_IF "$WIFI_IF" \
        AP_SSID "$AP_SSID" PROTECTED_SSID "$PROTECTED_SSID" AP_CIDR "$AP_CIDR"
    install -m 0755 "$0" "$INSTALL_PATH"
    install_ct_files

    cat > "/etc/systemd/system/${TOOL_NAME}-gate.service" <<EOF
[Unit]
Description=Gate roomscanner travel AP on Ethernet and nearby SSID

[Service]
Type=oneshot
ExecStart=${INSTALL_PATH} reconcile
EOF
    cat > "/etc/systemd/system/${TOOL_NAME}-gate.timer" <<'EOF'
[Unit]
Description=Reconcile roomscanner travel AP state

[Timer]
OnBootSec=10s
OnUnitActiveSec=5s
AccuracySec=1s

[Install]
WantedBy=timers.target
EOF
    systemctl daemon-reload
    systemctl enable --now "${TOOL_NAME}-gate.timer"
    reconcile
    note "installed; AP policy is active (wired=${WIRED_IF}, protected SSID=${PROTECTED_SSID})"
}

reconcile() {
    require_root_and_host
    load_config
    if [[ $(carrier) == 1 ]]; then
        ct_exec systemctl stop hostapd dnsmasq travel-ap-network
        note "wired carrier on ${WIRED_IF}; AP stopped"
    else
        ct_exec /usr/local/sbin/roomscanner-travel-ap-reconcile
        note "wired carrier absent; evaluated protected-SSID gate"
    fi
}

cmd_down() {
    require_root_and_host
    load_config
    systemctl disable --now "${TOOL_NAME}-gate.timer"
    ct_exec systemctl stop hostapd dnsmasq travel-ap-network
    note "AP stopped; automatic AP control disabled"
}

cmd_auto() {
    require_root_and_host
    load_config
    systemctl enable --now "${TOOL_NAME}-gate.timer"
    reconcile
    note "automatic Ethernet/SSID AP control restored"
}

cmd_status() {
    require_root_and_host
    load_config
    echo "wired_interface=${WIRED_IF}"
    echo "wired_carrier=$(carrier)"
    echo "container=${CT_ID}"
    echo "ap_ssid=${AP_SSID}"
    echo "protected_ssid=${PROTECTED_SSID}"
    printf 'automatic_gate='; systemctl is-active "${TOOL_NAME}-gate.timer" || true
    printf 'hostapd='; ct_exec systemctl is-active hostapd || true
    printf 'dnsmasq='; ct_exec systemctl is-active dnsmasq || true
    printf 'ap_address='; ct_exec bash -c "ip -4 -o addr show dev '$WIFI_IF' | awk '{print \$4}'" || true
}

case ${1:-} in
    install) shift; cmd_install "$@" ;;
    auto) cmd_auto ;;
    reconcile|up) reconcile ;;
    down) cmd_down ;;
    status) cmd_status ;;
    -h|--help|help|'') usage ;;
    *) die "unknown command: $1 (try --help)" ;;
esac
