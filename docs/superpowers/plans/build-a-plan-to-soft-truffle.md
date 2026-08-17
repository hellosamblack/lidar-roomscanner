# Pi 3 bridge node — flashable SD image + MCP remote admin

## Context

Implements `docs/superpowers/specs/2026-08-17-pi3-bridge-node-design.md`: a Raspberry
Pi 3 Model B mounted on the rig replaces the flaky RavPower FileHub as the scanner's
wireless uplink. The FileHub is an unobservable black box with a manual recovery
dance, and is the lead suspect for the measured 2.3–9.4 % silent frame loss.
Deliverables: **(1)** a flashable SD-card image with Wi-Fi credentials baked in,
**(2)** remote administration of the Pi through new `bridge_*` tools in the
roomscan MCP server, **(3)** a local pcap tee so lost-over-Wi-Fi frames are
recoverable after the fact.

**Owner decisions (2026-08-17):** network shape = **routed + NAT** (true L2 is
infeasible — brcmfmac has no 4-addr station bridging; proxy-ARP would leave scanner
DHCP hostage to the Wi-Fi hop); **tee in v1**; hardware = **Pi 3 Model B**.

## Design in one paragraph

Pi eth0 = `172.31.100.1/24` static (NM keyfile), running dnsmasq (DHCP-only,
authoritative — answers inside the scanner firmware's **3000 ms DHCP window**,
after which it flips to self-assigned server at 172.31.253.1, `ethernet_transport.c`
~183–205) with a static lease `00:80:E1:00:00:00 → 172.31.100.20` (MAC is a
compile-time constant, `ethernet_transport.h:7-12`). Pi wlan0 = NM client to
`Surly_Office` / travel AP (both keyfiles baked, autoconnect priorities,
`powersave=2`). Avahi on **wlan0 only** publishes service instance exactly
`roomscanner` / `_roomscan._udp` / port 5000 — so the host's `UdpSource`
(`sources.py:287`, zeroconf lookup of `roomscanner._roomscan._udp.local.`) resolves
the Pi with **zero host-code changes**; nftables DNATs UDP 5000 wlan0→172.31.100.20
and masquerades the return path (1 Hz keepalive keeps conntrack warm).
`allow-interfaces=wlan0` also stops avahi from colliding with the scanner's own
`roomscanner` announcement on eth0. Subnet avoids 172.31.253.0/24 (scanner
fallback), 192.168.50.0/24 (travel AP), 172.17.0.0/16 (home LAN).

**Build is fully rootless** (this LXC has no loop devices / passwordless sudo):
download pinned official Raspberry Pi OS Lite Bookworm armhf, verify sha256, parse
the MBR in pure Python for the FAT partition offset, inject via `mtools`
(`mcopy -i img@@off`): a `firstrun.sh` (Imager mechanism —
`systemd.run=/boot/firmware/firstrun.sh systemd.run_success_action=reboot
systemd.unit=kernel-command-line.target` appended to the single-line cmdline.txt),
a payload tarball, `userconf.txt`, and the `ssh` flag. First boot on the Pi (as
root) extracts the payload to `/opt/roomscan-bridge`, runs idempotent
`install.sh --first-boot` (dpkg -i **bundled .debs** — dnsmasq/tcpdump closure
computed at build time against the base image's dpkg status, read via a partition
slice + `debugfs` — so eth0 side needs no network), enables units, self-cleans
cmdline.txt, reboots. firstrun logs to `/boot/firmware/firstrun.log` (FAT —
readable post-mortem from any laptop).

## New files

```
pi-bridge/
  build_image.py                  # rootless builder; stages: download/verify,
                                  #   mbr-parse, deb-closure, payload-render,
                                  #   inject (mtools), report (--json)
  pi-bridge-secrets.example.yaml  # committed template (real file git-ignored):
                                  #   hostname, wifi.home/.travel {ssid,passphrase,
                                  #   priority}, wifi_country, ssh_pubkey
  boot/firstrun.sh                # template; LF-only enforced by builder
  payload/
    install.sh                    # idempotent; shared by firstrun AND bridge_update
    etc/NetworkManager/system-connections/*.nmconnection.tmpl  # eth0 static + 2 wifi
    etc/NetworkManager/conf.d/roomscan.conf      # wifi.powersave=2 global
    etc/dnsmasq.d/roomscan-bridge.conf           # DHCP-only (port=0), authoritative,
                                                 #   dhcp-host=00:80:E1:00:00:00,172.31.100.20
    etc/nftables: table ip roomscan_bridge       # DNAT 5000, masquerade, counter on
                                                 #   every rule (bridge_status reads them)
    etc/avahi/services/*.service                 # instance `roomscanner` _roomscan._udp:5000
                                                 #   + `roomscan-bridge` _roomscan-bridge._tcp:22
    etc/avahi: allow-interfaces=wlan0            # conflict + LAN-leak prevention
    etc/sysctl.d/: ip_forward=1, rp_filter=2
    etc/systemd/system/: roomscan-bridge-reconcile.{service,timer}, roomscan-tee.service
    usr/local/sbin/roomscan-bridge-reconcile     # bash, travel-ap.sh house style
    usr/local/sbin/roomscan-bridge-status        # bash → JSON (single source for MCP)
host/tools/pi_bridge.py           # pure functions + --json CLI (ssh BatchMode subprocess)
host/tools/pcap2capture.py        # tee pcap → capture .bin + loss stats
host/src/roomscan/mcp_server/tools_bridge.py   # thin @mcp.tool() wrappers
host/tests/test_pi_bridge_build.py
host/tests/test_pcap2capture.py
docs/pi-bridge-runbook.md
```

Plus edits: `server.py::build()` import line; `test_mcp_registry.py` EXPOSED
entries; `docs/mcp-server.md`; `.gitignore` (cache/, out/, secrets, debs/).

## Key behaviors

**Reconcile timer** (10 s, fail-closed, `logger -t roomscan-bridge`): if scanner
absent from lease/neigh, probe fallback mode via temp alias `172.31.253.2/24` +
ping `.253.1`; if found, **first** swap the DNAT target to `.253.1` (stream keeps
working immediately), **then** bounce eth0 link to re-enter the firmware's DHCP
client (verified: link-down resets to `DHCP_STATE_INIT`) — but **only when nft
counters show no active stream traffic**, never mid-capture. Restore DNAT to
`.100.20` when the lease reappears. Also restarts any dead unit (dnsmasq/avahi/
nftables/tee) and logs it.

**Tee** (`roomscan-tee.service`): `tcpdump -i eth0 udp port 5000 -w ring.pcap
-C 100 -W 20` → bounded 2 GB ring (~70 min at the measured ~466 KB/s), `Nice=5`,
`Restart=on-failure`; structurally fail-open (kernel forwards, tcpdump observes;
nothing depends on the unit).

**Wi-Fi cred recovery without reflash**: install.sh also applies any
`wifi-override.nmconnection` found on the FAT partition at every boot (tiny
bootcfg pass) — fixes the bad-creds chicken/egg by editing the card from any
laptop.

**MCP surface** (`tools_bridge.py`; target resolution `ROOMSCAN_BRIDGE_HOST` env →
zeroconf `_roomscan-bridge._tcp`; ssh key `~/.ssh/roomscan-bridge`, generated by
the builder if absent, pubkey baked into authorized_keys; every tool reports what
actually happened, e.g. wifi_update returns resulting association state):

- `bridge_status()` — runs remote `roomscan-bridge-status`, returns its JSON
  (model, wlan0 SSID/RSSI/bitrate/power_save, eth0 carrier + scanner lease/neigh,
  nft counters, tee state + ring usage, temp/throttled, NTP-synced) + ssh-hop
  latency
- `bridge_logs(unit, lines=200)` · `bridge_wifi_update(ssid, psk, profile="home")`
- `bridge_update()` — re-render payload locally, scp, rerun install.sh, report unit states
- `bridge_tee_list()` · `bridge_tee_fetch(name, out="", convert=True)` — scp pcap
  into captures/, run pcap2capture, return loss stats
- `bridge_image_build(profile="production", xz=False)` · `bridge_reboot()`

**pcap2capture.py**: stdlib pcap parser (both endiannesses + ns magic), selects
scanner→host UDP src-port-5000 payloads ≥ 6 B (skips 1-byte keepalives), reassembles
the `<IBB` seq/frag_idx/total_frags fragments with the same indexed-slot rules and
counter names as `UdpSource.read()` (`sources.py:357-399`), writes concatenated
frame bytes — byte-identical to `capture.py`'s `out_f.write(source.read())` format
so all existing `capture_*` tooling consumes it unmodified. `--json` stats are the
acceptance-gate comparison vocabulary (frames_incomplete, frags_lost, …).

## Verification on this host (no Pi needed)

- `test_pi_bridge_build.py`: MBR-parser unit tests; **mini-image round-trip**
  (synthetic MBR + `mformat` 32 MB FAT → run the builder's inject stage → read
  back with `mdir`/`mtype`: single-line cmdline with the 3 args, rendered keyfile
  has the test SSID/PSK, tarball modes correct) — skip-if mtools absent; secrets
  hygiene (refuses example file / 0644 perms; no real psk in git); payload lint
  (units parse, avahi XML instance == `roomscanner`, `bash -n`, no CRLF,
  `nft -c` skip-if); deb-closure resolver against canned Packages/status fixtures.
- `test_pcap2capture.py`: golden synthetic pcaps — clean 11-fragment frame is
  byte-exact, reordered/dropped/duplicate fragment cases hit the right counters,
  output decodes via the existing capture decoder.
- Registry: `test_mcp_registry.py` EXPOSED entries + every tool named in
  docs/mcp-server.md (test-enforced); `tools_bridge` returns structured
  `{"ok": False}` (not raise) with an unreachable host.
- `run_tests` MCP tool for the suite; one-time host prereq: `sudo apt install
  mtools` (user runs it — no passwordless sudo here).

**Hardware acceptance (spec §11) cannot run on this host** → per project law the
governing issue stays open with `needs/operator` + an operator runbook
(operator-request skill): flash + HDMI first boot; discovery parity; side-by-side
FileHub vs Pi loss on the same scene (baseline 2.3–9.4 %); tee beats the Wi-Fi
path; bounded-failure drills (Wi-Fi kill / Pi power-cycle / Pi removal → direct
Ethernet fallback intact); reconcile drill (boot Pi after scanner).

## Process & docs

- **session-start first**: create governing issue "Pi 3 bridge node v1 — image,
  network, tee, MCP admin" (records locked decisions + acceptance checklist;
  ends `needs/operator`), implement as two PR-sized tranches with disjoint files —
  (B) image builder + payload, (C) MCP surface + pcap2capture — then file
  follow-ups (retire filehub-bridgemode.sh + /api/bridge-mode; re-point
  commstatus FileHub semantics; travel-mode plan cross-ref). `travel-ap.sh`
  itself needs **zero changes** (no MAC allowlist).
- status-sync at land: `docs/system-architecture.md` network section,
  `ROADMAP.md` register row, `docs/mcp-server.md`, new `docs/pi-bridge-runbook.md`
  (build → flash → first boot → day-2 via MCP → failure playbook → FileHub
  cold-spare rule: never both plugged in at once — duplicate `roomscanner`
  announcements), spec Status → Accepted.

## Top risks (mitigations designed in)

1. Bookworm firstrun path (`/boot/firmware`, `systemd.run` generator) is the one
   thing only a real first boot proves — hence firstrun.log-on-FAT and the HDMI
   first-boot checklist; fallback mechanism is Bookworm's native `custom.toml`.
2. cmdline.txt is single-line and image-release-specific — builder appends and
   asserts, never rewrites.
3. brcmfmac power save off (per-profile + global), `bridge_status` reports the
   actual `iw get power_save` (truth-telling, not request-echo).
4. No RTC: tee/journal timestamps bogus until NTP sync — reported in
   `bridge_status`, noted in runbook.
