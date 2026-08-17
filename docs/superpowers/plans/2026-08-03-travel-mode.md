# Travel-mode Wi-Fi AP — Ethernet-first

## Contract

- The scanner's Ethernet path is preferred.  **If the Proxmox host's real
  physical wired uplink has carrier, the travel AP must be down**: no SSID,
  no wireless DHCP, and no NAT.
- If that physical uplink has no carrier **and the adapter cannot see an
  existing `Surly_Office` SSID**, CT 300 exposes a 2.4 GHz AP for the FileHub
  and the control client.  The AP serves `192.168.50.0/24`; this is deliberately
  distinct from the scanner's no-DHCP direct-link fallback, `172.31.253.0/24`.
- A scan failure is treated as “do not start the AP,” not as permission to
  start it.  Avoiding interference with an already-present `Surly_Office`
  network is more important than automatic travel fallback.
- “Connected to Ethernet” means physical carrier in this plan.  A cable with
  carrier but no Internet still suppresses the AP.  If the desired policy is
  instead “no usable default route/Internet,” change the gate deliberately;
  do not infer that from CT 300's virtual `eth0`.

The scanner remains a DHCP client on a bridged network.  When travel AP mode
is active, `dnsmasq` gives it a normal `192.168.50.x` lease through the FileHub
bridge; its `172.31.253.1` fallback is not used.

## Automated control

After the Wi-Fi passthrough prerequisite below is in place, use the repository
tool from the Proxmox host instead of manually creating the service files:

```sh
cp travel-ap-secrets.example.yaml /root/travel-ap-secrets.yaml
chmod 0600 /root/travel-ap-secrets.yaml
# Edit /root/travel-ap-secrets.yaml with the real passphrase, then:
sudo ./travel-ap.sh install --secrets /root/travel-ap-secrets.yaml
sudo ./travel-ap.sh status
```

The YAML file defines separate `test` and `production` SSID/passphrase pairs.
`install` prompts for the profile; add `--profile test` or
`--profile production` for non-interactive use.  The selected SSID is also the
SSID the tool scans for before AP startup.  The tool installs the five-second
Ethernet/SSID gate and provides `reconcile` (also `up`), `down`, and `status`.
`up` is policy-preserving, not a force-start command.  `down` turns the AP
off and disables the automatic timer; run `auto` to restore automatic AP mode.

## 0. Preflight: identify the real wired uplink

Run on the **Proxmox host** and use `eno1`, the verified physical wired uplink.
On 2026-08-03, with the cable inserted, `eno1` reported `carrier = 1` and
`LOWER_UP`.  Do not use `vmbr0` or CT 300's `eth0`: they are virtual bridges/
veth interfaces and may show carrier while the host's physical cable is
unplugged.

```sh
ip -br link
cat /sys/class/net/eno1/carrier
```

With the cable inserted the latter must be `1`; unplug it and confirm it becomes
`0`.  Stop here if it does not—the AP gate would otherwise make the wrong choice.

Also confirm `wlp0s20f3` is not being used by the Proxmox host; passing it into
CT 300 makes CT 300 its exclusive owner.

## 1. Run on the Proxmox host

```sh
apt update && apt install -y iw
```

Append the Wi-Fi passthrough configuration to CT 300 only once:

```sh
cat <<'EOF' >> /etc/pve/lxc/300.conf
lxc.net.1.type: phys
lxc.net.1.link: wlp0s20f3
lxc.net.1.name: wlan0
lxc.mount.entry: /dev/rfkill dev/rfkill none bind,optional,create=file
lxc.apparmor.profile: unconfined
EOF

pct stop 300
pct start 300
```

## 2. Configure the AP inside CT 300 (but do not start it)

```sh
pct enter 300
apt update && apt install -y hostapd dnsmasq iw
systemctl stop hostapd dnsmasq
systemctl disable hostapd dnsmasq
systemctl unmask hostapd
```

Create an isolated dnsmasq snippet instead of replacing the system-wide
configuration:

```sh
cat <<'EOF' > /etc/dnsmasq.d/roomscanner-travel.conf
interface=wlan0
bind-interfaces
dhcp-range=192.168.50.10,192.168.50.150,255.255.255.0,24h
# No public DNS is advertised: in isolated travel mode there may be no uplink.
EOF
```

Configure a 2.4 GHz AP—the FileHub requires 2.4 GHz.  Set the SSID and WPA2
passphrase to the home values if automatic FileHub association is wanted.
For a home dry run, use a distinct test SSID such as `Surly_Office_Test`, while
keeping `Surly_Office` as the protected SSID that suppresses AP startup when a
real nearby network already uses it.

```sh
cat <<'EOF' > /etc/hostapd/hostapd.conf
interface=wlan0
driver=nl80211
ssid=REPLACE_WITH_HOME_SSID
hw_mode=g
channel=6
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=REPLACE_WITH_HOME_WPA2_PASSPHRASE
wpa_key_mgmt=WPA-PSK
rsn_pairwise=CCMP
EOF

sed -i 's|^#\?DAEMON_CONF=.*|DAEMON_CONF="/etc/hostapd/hostapd.conf"|' /etc/default/hostapd
```

The address is owned by a dedicated service, so it exists only while AP mode
is active and repeated starts are safe:

```sh
cat <<'EOF' > /etc/systemd/system/travel-ap-network.service
[Unit]
Description=Roomscanner travel AP network address
Before=hostapd.service dnsmasq.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set wlan0 up
ExecStart=/usr/sbin/ip addr replace 192.168.50.1/24 dev wlan0
ExecStop=/usr/sbin/ip addr del 192.168.50.1/24 dev wlan0

[Install]
WantedBy=multi-user.target
EOF

mkdir -p /etc/systemd/system/hostapd.service.d /etc/systemd/system/dnsmasq.service.d
cat <<'EOF' > /etc/systemd/system/hostapd.service.d/travel-ap.conf
[Unit]
Requires=travel-ap-network.service
After=travel-ap-network.service
EOF
cp /etc/systemd/system/hostapd.service.d/travel-ap.conf \
  /etc/systemd/system/dnsmasq.service.d/travel-ap.conf
systemctl daemon-reload
```

Do not configure forwarding, NAT, or public DNS for this scanner-only AP.  Add
them later only if travel clients explicitly need Internet sharing.

Exit CT 300 after this configuration.

## 3. Install the Ethernet- and SSID-first gate on the Proxmox host

This runs on the host precisely because it can observe `eno1`'s physical link.
When Ethernet is absent it delegates the SSID scan to CT 300, where `wlan0`
lives.

The scan matches `Surly_Office` exactly.  If “Surly_Office SSIDs” means a family
such as `Surly_Office-Guest`, change `PROTECTED_SSID` matching deliberately
before use (for example, a documented anchored prefix match); do not use a
loose substring that could suppress the AP because of an unrelated SSID.

Create the CT-side reconcile script first.  It scans only before a new AP
startup; continuously stopping a live AP to scan would itself disconnect the
FileHub and control clients.  Ethernet returning always stops it immediately.

```sh
pct enter 300
cat <<'EOF' > /usr/local/sbin/roomscanner-travel-ap-reconcile
#!/bin/sh
set -eu

PROTECTED_SSID='Surly_Office'

# A live AP cannot be scanned without disrupting its clients.  It passed the
# scan gate when started; leave it alone until Ethernet returns.
if systemctl is-active --quiet hostapd; then
    exit 0
fi

systemctl stop hostapd dnsmasq travel-ap-network || true
ip link set wlan0 up

SCAN=$(iw dev wlan0 scan 2>/dev/null) || {
    logger -t roomscanner-travel-ap 'Wi-Fi scan failed; travel AP remains off'
    exit 0
}

if printf '%s\n' "$SCAN" | awk -F': ' '/^[[:space:]]*SSID: / {print $2}' | \
        grep -Fqx -- "$PROTECTED_SSID"; then
    logger -t roomscanner-travel-ap \
        "saw protected SSID $PROTECTED_SSID; travel AP remains off"
    exit 0
fi

systemctl start travel-ap-network hostapd dnsmasq
logger -t roomscanner-travel-ap 'no protected SSID seen; travel AP started'
EOF
chmod 0755 /usr/local/sbin/roomscanner-travel-ap-reconcile
exit
```

```sh
cat <<'EOF' > /usr/local/sbin/roomscanner-travel-ap-gate
#!/bin/sh
set -eu

CT_ID=300
WIRED_IF=eno1
CARRIER=/sys/class/net/$WIRED_IF/carrier

if [ -r "$CARRIER" ] && [ "$(cat "$CARRIER")" = 1 ]; then
    # Wired link wins: remove SSID and DHCP immediately.
    pct exec "$CT_ID" -- systemctl stop hostapd dnsmasq travel-ap-network
else
    # No physical Ethernet: scan for Surly_Office before enabling an AP.
    pct exec "$CT_ID" -- /usr/local/sbin/roomscanner-travel-ap-reconcile
fi
EOF
chmod 0755 /usr/local/sbin/roomscanner-travel-ap-gate

cat <<'EOF' > /etc/systemd/system/roomscanner-travel-ap-gate.service
[Unit]
Description=Gate roomscanner travel AP on Ethernet and nearby SSID

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/roomscanner-travel-ap-gate
EOF

cat <<'EOF' > /etc/systemd/system/roomscanner-travel-ap-gate.timer
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
systemctl enable --now roomscanner-travel-ap-gate.timer
systemctl start roomscanner-travel-ap-gate.service
```

The five-second timer is a conservative first implementation.  It is
intentionally simple and observable; replace it with a link-event trigger only
after the physical interface's events have been verified on this host.  When
the AP is off, the timer re-scans and starts only if `Surly_Office` is absent;
when it is on, it leaves the radio untouched until Ethernet returns.

## 4. FileHub bridge and management

> **Superseded in the primary path (2026-08-17, #191).** The
> [Pi 3 bridge node](../../pi-bridge-runbook.md) replaces the FileHub as the rig's wireless
> uplink. It carries **both** SSIDs as baked NetworkManager profiles — `home` and `travel`,
> with autoconnect priorities — so it joins the travel AP with no bridge-mode dance and no
> management address at all; day-2 administration is `bridge_status()` and friends over ssh.
> **`travel-ap.sh` itself needs no changes**: its gating logic never cared which client
> associates. This section stays as the operational contract for the FileHub while it remains
> a cold spare, and until #191's hardware acceptance passes. Retiring
> `filehub-bridgemode.sh` outright is tracked as #192.

Matching SSID/passphrase lets the FileHub join the travel AP automatically, but
it does **not** make bridge mode permanent.  Bridge recovery still follows the
project's required order: unplug Ethernet from the FileHub, power-cycle the
FileHub, apply bridge mode, then reconnect Ethernet.

The existing `filehub-bridgemode.sh` assumes a management address of
`172.17.2.57`; that address is not routable from this travel subnet.  Before a
trip, choose and document one reliable travel-side management path:

1. Reserve a known `192.168.50.x` lease for the FileHub's Wi-Fi MAC in
   dnsmasq, and update the bridge command to use that address; or
2. Add a temporary `172.17.2.x/24` address to the host only for the bridge
   recovery operation, then remove it.

Do not rely on discovering this during a scan.  Test FileHub association,
bridge recovery, scanner DHCP lease, and `roomscanner.local` discovery once
while disconnected from the wired uplink.

## 5. Acceptance checks

1. With physical Ethernet connected, wait up to five seconds and verify no
   `Proxmox_AP`/travel SSID is visible, `hostapd` and `dnsmasq` are inactive,
   and `wlan0` has no `192.168.50.1` address.
2. With physical Ethernet unplugged and a real `Surly_Office` AP in range,
   wait more than one timer interval.  Verify the travel AP remains absent and
   `journalctl -t roomscanner-travel-ap` records the protected-SSID decision.
3. Turn that external AP off or move out of range.  Within five seconds, verify
   the travel AP appears and a client receives `192.168.50.10`–`.150` with
   gateway `192.168.50.1`.
4. With the FileHub bridged and associated, verify the scanner receives a
   `192.168.50.x` DHCP lease and the host application discovers
   `roomscanner.local` and receives UDP frames.
5. Reconnect physical Ethernet.  Within five seconds, verify the AP and DHCP
   server stop, then verify the scanner resumes DHCP on the wired network.
