# pi-bridge payload

Files installed onto the Raspberry Pi 3 (Raspberry Pi OS Lite Bookworm,
armhf) that turns it into a routed+NAT wireless bridge between the
STM32H563 scanner (on `eth0`) and the house/travel Wi-Fi (on `wlan0`).

Two entry points drive this payload:

- `pi-bridge/boot/firstrun.sh` (built into the boot FAT partition as
  `/boot/firmware/firstrun.sh`) extracts this payload to
  `/opt/roomscan-bridge` on first boot and runs `install.sh --first-boot`.
- `install.sh` itself is also re-run with no flags over ssh (`bridge_update`)
  to push updates to an already-provisioned Pi. Pass `--force` to override
  the in-flight-capture restart guard (see below).

## File inventory

| Path | Purpose |
|---|---|
| `install.sh` | Idempotent installer. `--first-boot`: installs bundled `debs/*.deb`, installs `node.env`/hostname/Wi-Fi-country, copies `etc/` into `/etc/`, wires the nftables include, installs `usr/local/sbin/*`, installs any Wi-Fi override, installs `authorized_keys`, enables+starts required/optional units. No flag (update mode): same steps, but restarts rather than enables units -- and SKIPS restarting `dnsmasq`/`nftables` if the DNAT counter shows live stream traffic (logging loudly, exit code 75), unless `--force` is given. |
| `node.env` *(staged at payload root, not under `etc/`, optional)* | `KEY=VALUE` lines: `HOSTNAME, USERNAME, SCANNER_IP, SCANNER_MAC, SCANNER_FALLBACK_IP, ETH_ADDR, STREAM_PORT, WIFI_COUNTRY`. Installed to `/etc/roomscan-bridge/node.env` (0644) and sourced early by `install.sh`, and at runtime by `roomscan-bridge-reconcile` / `roomscan-bridge-status` (via `roomscan-bridge-common.sh`). |
| `authorized_keys` *(staged at payload root, not under `etc/`, optional)* | Single ed25519 public key line. `install.sh` installs it into the `node.env` `USERNAME`'s `~/.ssh/authorized_keys` (0600, dir 0700), merging rather than clobbering -- this is what the host-side MCP `bridge_*` tools ssh in with. |
| `etc/NetworkManager/system-connections/roomscan-eth0.nmconnection.tmpl` | Static eth0 profile (`{{ETH_CIDR}}`), no gateway/DNS. |
| `etc/NetworkManager/system-connections/roomscan-wifi-home.nmconnection.tmpl` | Wi-Fi client profile, "home" network, DHCP, `powersave=2`. |
| `etc/NetworkManager/system-connections/roomscan-wifi-travel.nmconnection.tmpl` | Wi-Fi client profile, "travel" network, DHCP, `powersave=2`. |
| `etc/NetworkManager/conf.d/roomscan.conf` | Global `wifi.powersave=2` fallback for any connection not covered by the two profiles above. |
| `etc/dnsmasq.d/roomscan-bridge.conf` | DHCP-only (`port=0`), authoritative dnsmasq config for `eth0`; static lease `{{SCANNER_MAC}} -> {{SCANNER_IP}}`. |
| `etc/nftables/roomscan-bridge.nft` | `table ip roomscan_bridge`: DNAT UDP/`{{STREAM_PORT}}` wlan0->eth0, masquerade both return paths, forward-chain accept rules. Every rule has a `counter`. Self-flushing (`delete table` + redeclare) idiom, safe to reload repeatedly. Loaded via an `include` line `install.sh` appends to `/etc/nftables.conf` (see header comment in the file for why this approach was chosen over a dedicated systemd unit). Contains build-time tokens like the `.tmpl` files even though it has no `.tmpl` suffix -- the builder substitutes tokens in every textual payload file, not only `*.tmpl` ones. |
| `etc/avahi/services/roomscanner.service` | Avahi static service: literal instance `roomscanner` (never tokenized), type `{{MDNS_SERVICE}}`, port `{{STREAM_PORT}}` — what the PC host's zeroconf lookup resolves. |
| `etc/avahi/services/roomscan-bridge.service` | Avahi static service: literal instance `roomscan-bridge` (never tokenized), type `{{BRIDGE_MDNS_SERVICE}}`, port 22 (literal, no ssh-port token exists) — how MCP tooling finds the Pi over ssh. |
| `etc/avahi/avahi-daemon.conf` | Full avahi-daemon config, `allow-interfaces=wlan0` only (never announces on `eth0`). |
| `etc/sysctl.d/99-roomscan-bridge.conf` | `net.ipv4.ip_forward=1`, `net.ipv4.conf.{all,default}.rp_filter=2` (loose, for the asymmetric NAT path). |
| `etc/systemd/system/roomscan-tee.service` | Bounded pcap ring of the eth0 UDP/`{{STREAM_PORT}}` stream (`tcpdump -C 100 -W 20`, 2 GB max). Fail-open: purely observational, nothing depends on it. |
| `etc/systemd/system/roomscan-bridge-reconcile.service` | Oneshot that runs `usr/local/sbin/roomscan-bridge-reconcile`. |
| `etc/systemd/system/roomscan-bridge-reconcile.timer` | Fires the reconcile service every 10s (`AccuracySec=1s`, `Persistent=false`). |
| `usr/local/sbin/roomscan-bridge-common.sh` | Shared library (sourced, not executed): node-identity defaults + `/etc/roomscan-bridge/node.env` loader, and the nft DNAT rule helpers (`roomscan_dnat_target_ip`, `roomscan_set_dnat_target`, `roomscan_stream_is_live`, ...) used by both `roomscan-bridge-reconcile` and `install.sh`'s restart guard. |
| `usr/local/sbin/roomscan-bridge-reconcile` | Fail-closed bash: detects scanner presence/fallback-mode, retargets the nft DNAT rule, bounces `eth0` only when no stream traffic is flowing, restarts dead required units. Reads `SCANNER_IP`/`SCANNER_FALLBACK_IP`/`STREAM_PORT` via `roomscan-bridge-common.sh` (node.env-overridable), never hardcoded. |
| `usr/local/sbin/roomscan-bridge-status` | Emits one JSON object on stdout — the source of truth for the MCP `bridge_status` tool. Every documented field is always present (`null` when unknown). Also sources `roomscan-bridge-common.sh` for `SCANNER_IP`/`STREAM_PORT`. |

Not part of this payload but referenced by it:

- `/opt/roomscan-bridge/debs/*.deb` (first-boot only, optional) — bundled
  packages `install.sh --first-boot` installs offline via `dpkg -i`.
- `/boot/firmware/wifi-override.nmconnection` (optional, any time) — drop a
  rendered `.nmconnection` file here (edit the SD card from any laptop) to
  recover from bad baked-in Wi-Fi credentials without a reflash;
  `install.sh` installs it as
  `roomscan-wifi-override.nmconnection` (0600) and reloads NetworkManager
  on every run, first-boot or update.

## `install.sh` exit codes

| Code | Meaning |
|---|---|
| `0` | Completed; everything (including unit restarts) applied. |
| `1` | A REQUIRED unit failed to come up -- hard failure. |
| `2` | Bad command-line argument. |
| `75` | Completed, but the `dnsmasq`/`nftables` restart was **skipped** because a capture looked to be in progress (DNAT byte counter increased across a ~1s sample). Benign: file changes ARE installed, only the restart was deferred. Re-run later, or pass `--force`. |

## Placeholder-token contract

The image builder (`pi-bridge/build_image.py`, owned elsewhere) substitutes
`{{TOKEN}}` placeholders in **every textual payload file it copies**, not
only `*.tmpl`-suffixed ones, and hard-errors on any token it cannot
resolve. `*.nmconnection.tmpl` files are additionally renamed to
`*.nmconnection` (suffix dropped) after rendering; other files (e.g.
`roomscan-bridge.nft`, the avahi `.service` files, `dnsmasq.d/roomscan-bridge.conf`)
keep their names and are rendered in place. `install.sh` never renders
templates itself — it only installs already-rendered files, and
defensively skips (with a logged warning) any stray `*.tmpl` it finds
still in the payload.

Builder-supplied tokens actually referenced by files under `pi-bridge/payload/`
and `pi-bridge/boot/`:

| Token | File(s) | Meaning |
|---|---|---|
| `{{ETH0_UUID}}` (alias of `{{UUID_ETH0}}`) | `roomscan-eth0.nmconnection.tmpl` | Fixed, pre-generated UUID for the eth0 connection profile. Must stay stable across rebuilds. |
| `{{ETH_CIDR}}` | `roomscan-eth0.nmconnection.tmpl` | eth0 address in CIDR form, e.g. `172.31.100.1/24` (the address-only half is `{{ETH_ADDR}}`, the prefix-length half is `{{ETH_PREFIX}}` -- this file uses the combined form since that's what `address1=` wants). |
| `{{HOME_UUID}}` (alias of `{{UUID_WIFI_HOME}}`) | `roomscan-wifi-home.nmconnection.tmpl` | Fixed UUID for the "home" Wi-Fi profile. |
| `{{HOME_SSID}}` | `roomscan-wifi-home.nmconnection.tmpl` | SSID of the home Wi-Fi network. |
| `{{HOME_PSK}}` | `roomscan-wifi-home.nmconnection.tmpl` | WPA-PSK passphrase for the home network. |
| `{{HOME_PRIORITY}}` | `roomscan-wifi-home.nmconnection.tmpl` | Integer `autoconnect-priority` for the home profile. |
| `{{TRAVEL_UUID}}` (alias of `{{UUID_WIFI_TRAVEL}}`) | `roomscan-wifi-travel.nmconnection.tmpl` | Fixed UUID for the "travel" Wi-Fi profile. |
| `{{TRAVEL_SSID}}` | `roomscan-wifi-travel.nmconnection.tmpl` | SSID of the travel Wi-Fi network. |
| `{{TRAVEL_PSK}}` | `roomscan-wifi-travel.nmconnection.tmpl` | WPA-PSK passphrase for the travel network. |
| `{{TRAVEL_PRIORITY}}` | `roomscan-wifi-travel.nmconnection.tmpl` | Integer `autoconnect-priority` for the travel profile (typically lower than `{{HOME_PRIORITY}}`). |
| `{{WIFI_COUNTRY}}` | both wifi `.tmpl` files (documentary comment only); also expected as a `node.env` key | ISO 3166-1 alpha-2 regulatory domain (e.g. `US`). NetworkManager keyfile connections have no per-connection regulatory-domain key, so the *enforced* application of this value happens at install time: `install.sh` reads `WIFI_COUNTRY` from `node.env` and applies it via every mechanism actually present on the box (`/etc/default/crda`, `raspi-config nonint do_wifi_country`, `iw reg set`, `rfkill unblock wifi`), logging which fired. The token is still substituted into the `.tmpl` comments too, so no literal `{{WIFI_COUNTRY}}` is ever left in an installed file. `WIFI_COUNTRY` is not in the coordinator's required `node.env` key list but is accepted as an addition if present. |
| `{{SCANNER_IP}}` | `roomscan-bridge.nft`, `dnsmasq.d/roomscan-bridge.conf`; also expected as a `node.env` key | The scanner's static eth0 address (`172.31.100.20` in the fixed design). Runtime scripts read it via `node.env`, not a build-time substitution, in their own hardcoded fallback defaults. |
| `{{SCANNER_MAC}}` | `dnsmasq.d/roomscan-bridge.conf`; also expected as a `node.env` key | The scanner's eth0 MAC address. |
| `{{SCANNER_FALLBACK_IP}}` | `roomscan-bridge.nft` (comment only -- the live value is runtime-managed); also expected as a `node.env` key | The scanner's self-assigned fallback address (`172.31.253.1`). |
| `{{DHCP_RANGE_START}}` / `{{DHCP_RANGE_END}}` | `dnsmasq.d/roomscan-bridge.conf` | Single-host DHCP range bounds (both equal `{{SCANNER_IP}}` in this design, since there's exactly one static lease). |
| `{{STREAM_PORT}}` | `roomscan-bridge.nft`, both avahi `.service` files, `roomscan-tee.service`; also expected as a `node.env` key | The RSCN UDP stream port (`5000`). |
| `{{MDNS_SERVICE}}` | `avahi/services/roomscanner.service` | The scanner-stream mDNS service type (`_roomscan._udp`). The `<name>` instance (`roomscanner`) is deliberately left literal -- see the caveat below. |
| `{{BRIDGE_MDNS_SERVICE}}` | `avahi/services/roomscan-bridge.service` | The bridge-ssh mDNS service type (`_roomscan-bridge._tcp`). The `<name>` instance (`roomscan-bridge`) is deliberately left literal -- see the caveat below. |

Tokens provided by the builder but **not** referenced by any file here
(consumed elsewhere, e.g. by `build_image.py` itself, or intentionally
unused): `HOSTNAME`, `USERNAME`, `SSH_PUBKEY` (the builder presumably uses
this to populate the staged `authorized_keys` file itself, rather than a
token substituted into a payload file), `ETH_ADDR`, `ETH_PREFIX` (used to
compose `{{ETH_CIDR}}`, not directly referenced), `MDNS_INSTANCE`,
`BRIDGE_MDNS_INSTANCE` (see caveat below).

**Caveat -- avahi instance names are never tokenized.** `roomscanner` and
`roomscan-bridge` are left as literal strings in the `<name>` elements of
`etc/avahi/services/*.service`, even though `{{MDNS_INSTANCE}}` /
`{{BRIDGE_MDNS_INSTANCE}}` tokens exist, because a test asserts the exact
literal string. Do not substitute those two tokens into this payload.

**Caveat -- runtime values vs. build-time tokens.** `SCANNER_IP`,
`SCANNER_MAC`, `SCANNER_FALLBACK_IP`, and `STREAM_PORT` appear twice in
different roles: as `{{TOKEN}}` placeholders baked into the static config
files above at *build* time, and as `node.env` `KEY=VALUE` entries read at
*install/run* time by `install.sh`, `roomscan-bridge-reconcile`, and
`roomscan-bridge-status` (via `roomscan-bridge-common.sh`). Both must
agree -- the builder is expected to derive `node.env`'s values from the
same source of truth it substitutes into the static files.

## Design notes / constraints this payload assumes

- `eth0` never carries a gateway or DNS; the default route lives on `wlan0`
  only.
- dnsmasq on `eth0` is DHCP-only (`port=0`) and authoritative, and must
  answer inside the scanner firmware's 3000 ms DHCP window — see the
  comment header in `etc/dnsmasq.d/roomscan-bridge.conf`.
- The nftables DNAT target starts at the scanner's static address and is
  only ever swapped to its self-assigned fallback address by
  `roomscan-bridge-reconcile`, and only swapped back once the scanner is
  reachable again via DHCP.
- avahi only ever announces on `wlan0` (`allow-interfaces=wlan0`), never
  `eth0`.
- `install.sh` never restarts `dnsmasq`/`nftables` in update mode while the
  nftables DNAT counter shows live stream traffic, so a config push never
  interrupts an in-progress capture (see the exit-code table above);
  `--force` overrides this guard. First-boot mode is never guarded --
  nothing is running yet to interrupt.
- An unresolvable own hostname makes `sudo` (and anything else doing a
  self name lookup) hang for several seconds, which presents as a network
  fault -- `install.sh` keeps `/etc/hostname` and `/etc/hosts`' `127.0.1.1`
  line in sync with `node.env`'s `HOSTNAME` specifically to avoid this.
