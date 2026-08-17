# Pi 3 bridge node — build, flash, operate

The Raspberry Pi 3 bridge (issue [#191](https://github.com/hellosamblack/lidar-roomscanner/issues/191))
replaces the RavPower FileHub as the scanner's wireless uplink. The FileHub's
defect was never that it broke — it was that nothing about it was observable
when it did, so a capture that lost 2.3–9.4% of its frames looked exactly
like a clean one ([#60](https://github.com/hellosamblack/lidar-roomscanner/issues/60)),
and the FileHub is the lead suspect. This document is the whole lifecycle:
build the SD image, flash it, bring the Pi up for the first time, run it
day-to-day through the MCP `bridge_*` tools, recover lost frames from its
pcap tee, and diagnose the ways it can fail.

Design rationale (why routed+NAT, why Ethernet not USB, the invariants and
acceptance gates) lives in
[`superpowers/specs/2026-08-17-pi3-bridge-node-design.md`](superpowers/specs/2026-08-17-pi3-bridge-node-design.md);
the implementation plan is
[`superpowers/plans/build-a-plan-to-soft-truffle.md`](superpowers/plans/build-a-plan-to-soft-truffle.md).
The MCP tool reference is [`mcp-server.md`](mcp-server.md) → "bridge"; the
runtime map that shows where this hop sits in the data path is
[`system-architecture.md`](system-architecture.md) → "Ingestion and recording".

## 1. What this is, and why routed + NAT

The Pi sits between the scanner's `eth0` and the wireless network on
`wlan0`, replacing the FileHub's opaque bridge mode with a box whose state
you can actually read: `bridge_status()` reports the radio's real
`power_save`, the DHCP lease, the nftables packet counters that prove
traffic is moving, and the pcap tee's fill level.

The network shape is **routed + NAT**, not a transparent L2 bridge, even
though L2 was the design doc's starting recommendation. The Pi 3's onboard
`brcmfmac` Wi-Fi driver has no 4-address station-mode bridging, so true
transparent Ethernet-frame bridging over the radio is not available on this
hardware at all — the alternative, proxy-ARP, would leave the scanner's DHCP
timing hostage to the Wi-Fi hop's own latency and disconnects, reintroducing
exactly the opacity this project is meant to remove. Routed+NAT instead
gives the Pi its own stable subnet on `eth0`, independent of whatever the
wireless hop is doing.

## 2. Topology and addressing

```
                          Pi eth0                    Pi wlan0
STM32H563  --Ethernet-->  172.31.100.1/24  ---NAT-->  DHCP client  --Wi-Fi-->  server
 (scanner)   00:80:E1:      dnsmasq DHCP              (Surly_Office
             00:00:00       (static lease)             or travel AP)
             172.31.100.20  avahi: nothing              avahi: roomscanner
             (or, after      here (eth0 is               (_roomscan._udp:5000)
              3000ms DHCP     never announced)            + roomscan-bridge
              timeout,                                    (_roomscan-bridge._tcp:22)
              self-assigned
              172.31.253.1)
```

| What | Value |
|---|---|
| Pi `eth0` address | `172.31.100.1/24` (static, no gateway, no DNS) |
| Scanner static DHCP lease | `00:80:E1:00:00:00` → `172.31.100.20` |
| Scanner firmware fallback | `172.31.253.1` (self-assigned once its 3000 ms DHCP window expires) |
| Stream port | UDP `5000` |
| Avahi instance on `wlan0` (scanner stream) | `roomscanner` / `_roomscan._udp` / port `5000` |
| Avahi instance on `wlan0` (bridge ssh) | `roomscan-bridge` / `_roomscan-bridge._tcp` / port `22` |
| DHCP range | `172.31.100.20`–`172.31.100.40` (one static lease; a range is still required by dnsmasq) |

The `roomscanner` avahi announcement on `wlan0` is what makes the host's
`UdpSource` resolve the Pi with **zero host-code changes** — it looks up
`roomscanner._roomscan._udp.local.` exactly as it always has, and now
resolves to the Pi's `wlan0` address instead of the scanner's own. The
`roomscan-bridge` instance is separate and is how the MCP `bridge_*` tools
find the Pi over ssh without a hardcoded IP.

The subnet `172.31.100.0/24` deliberately avoids three others already in
use: `172.31.253.0/24` (the scanner's self-assigned fallback), `192.168.50.0/24`
(the travel AP), and `172.17.0.0/16` (the home LAN / Docker's default). Avahi
only ever announces on `wlan0` — `allow-interfaces=wlan0` in
`etc/avahi/avahi-daemon.conf` — both to avoid colliding with the scanner's
own `roomscanner` announcement on `eth0`, and so nothing about the private
`eth0` link leaks onto the house or travel network.

## 3. Build

The image builder (`pi-bridge/build_image.py`) is rootless: this dev host is
an LXC container with no loop devices and no passwordless sudo, so it never
mounts the image. It parses the MBR in pure Python to find partition byte
offsets, reads the base image's installed-package list through `debugfs`
(no mount needed), and writes into the FAT boot partition with `mtools`
(`mcopy -i image@@offset`). Only one prerequisite needs root, and you run it
yourself:

```sh
sudo apt install mtools
```

Everything else in the build is unprivileged.

**Secrets file.** Copy the committed template somewhere private and lock it
down — the builder refuses to run against the committed example (its
placeholder Wi-Fi PSK would produce an image that never associates, with no
error until the Pi is already on the rig) and refuses a group- or
world-readable file, since it holds a Wi-Fi PSK and a login password in
plain text:

```sh
cp pi-bridge/pi-bridge-secrets.example.yaml ~/.config/roomscan/pi-bridge-secrets.yaml
chmod 600 ~/.config/roomscan/pi-bridge-secrets.yaml
# edit hostname, username, password, wifi_country, wifi.home (and optionally wifi.travel)
```

**Build:**

```sh
./pi-bridge/build_image.py --secrets ~/.config/roomscan/pi-bridge-secrets.yaml
```

Useful flags: `--release {trixie,bookworm}` (see the base-image note below),
`--xz` to also emit a compressed `.img.xz`, `--offline` to fail instead of
downloading anything not already cached, `--skip-debs` if you'd rather the
Pi pull `dnsmasq`/`tcpdump`/`nftables` from the internet on first boot
instead of bundling them (only viable if the Pi has network on `eth0`,
which it normally does not — the only thing on that side is the scanner),
and `--json` for a machine-readable report. The equivalent MCP tool is
`bridge_image_build(secrets?, release="trixie", xz?, skip_debs?)`.

**Base image.** The builder pins **Raspberry Pi OS Lite, Trixie, armhf**
by default. The original design plan said "Bookworm" because that was the
current stable release when it was written (2026-08-17); Trixie has since
shipped and is now what the armhf Lite line is actually published from —
the last Bookworm armhf Lite build is dated 2025-05-13 and only receives
oldstable updates. Everything the design depends on (`/boot/firmware`, the
`systemd.run` firstrun mechanism, NetworkManager keyfiles, dnsmasq,
nftables, avahi) is unchanged between the two releases, so Trixie is the
default and Bookworm stays selectable with `--release bookworm`. Check the
`RELEASES` dict at the top of `pi-bridge/build_image.py` for the exact
pinned filenames, URLs and sha256 hashes rather than trusting a date here —
those are the values the builder actually verifies against.

**Stages, in order** (from the builder's own module docstring):

1. `download` — fetch the pinned image (cached under `pi-bridge/cache/`), verify its sha256, decompress.
2. `mbr` — parse the partition table, locate the FAT boot and ext4 root slices.
3. `debs` — read the base image's installed-package list via `debugfs`, resolve the dependency closure of `dnsmasq`, `tcpdump`, `nftables` against the Raspberry Pi + Raspbian archive indexes, download the missing `.deb`s.
4. `render` — copy `payload/` to a staging directory, substitute `{{TOKEN}}`s from the secrets file, tar it up.
5. `inject` — `mcopy` `firstrun.sh`, the payload tarball, `userconf.txt`, and the `ssh` flag file into the FAT partition, and append the firstrun arguments to `cmdline.txt`.
6. `report` — read back what is genuinely present in the built image (not what was requested) and print it, or emit it as JSON.

Expect the first build to spend a few minutes downloading the base image
(a few hundred MB) plus the deb closure, then well under a minute for the
mtools injection. Cached downloads make repeat builds fast. The output
image lands under `pi-bridge/out/roomscan-bridge-<hostname>-<release>.img`
(both `pi-bridge/out/` and `pi-bridge/cache/` are git-ignored).

## 4. Flash

Identify the SD card device carefully before writing — this step overwrites
whatever is at the target path with no confirmation prompt from `dd`.

```sh
lsblk    # find the card by its SIZE column, not by guessing a device name
```

Then, with the card unmounted:

```sh
sudo dd if=pi-bridge/out/roomscan-bridge-<hostname>-trixie.img of=/dev/sdX bs=4M status=progress conv=fsync
sync
```

Double-check `/dev/sdX` against `lsblk`'s output before running this — `dd`
gives no warning and no undo if `of=` points at the wrong disk (including a
disk holding this very repo). Raspberry Pi Imager or balenaEtcher work too
if you'd rather not use `dd` directly; either way, write the plain `.img`
(or the `.img.xz` if you built with `--xz` and your tool decompresses on
the fly), not a filesystem-level copy.

## 5. First boot

Attach an HDMI display and keyboard for the *first* boot only — you are
verifying the firstrun mechanism worked, not operating the Pi this way
day-to-day.

What happens: the kernel command line the builder appended
(`systemd.run=/boot/firmware/firstrun.sh systemd.run_success_action=reboot
systemd.unit=kernel-command-line.target`) runs `firstrun.sh` once, very
early, as root, before normal multi-user boot. It extracts the payload
tarball to `/opt/roomscan-bridge`, runs `install.sh --first-boot` (installs
the bundled `.deb`s with no network needed, applies the Wi-Fi regulatory
domain, lays down the `etc/` tree, enables the required systemd units), then
self-cleans the three `systemd.*` args it added to `cmdline.txt` and lets
the success action reboot the Pi. **Expect exactly one automatic reboot** —
that is the mechanism working, not a crash loop. After the second boot the
Pi should come up on `wlan0` and be reachable over ssh.

Everything firstrun does is timestamped and logged to both the console and
`/boot/firmware/firstrun.log` — which lives on the FAT boot partition, so if
first boot fails (the reboot never happens, or the Pi comes up but nothing
works), you can pull the card, plug it into any laptop with a card reader,
and read that log directly without needing the Pi to be network-reachable
at all. On failure, `cmdline.txt` is deliberately **not** cleaned up, so a
reboot after fixing whatever broke retries the whole firstrun sequence.

Checklist before calling first boot done:

- Console shows the `roomscan-bridge firstrun.sh starting` banner, then
  `Running installer in --first-boot mode`, then `Installer succeeded`.
- The Pi reboots once on its own.
- After the reboot, `bridge_status()` (§6) reaches it and reports `ok: true`.
- `wlan0.connected` is `true` and `wlan0.ssid` matches the home network.
- `units` shows `dnsmasq.service`, `nftables.service`, and
  `avahi-daemon.service` all `active` (these are required; a failure here
  fails the whole install with exit code 1).
- `eth0.carrier` is `true` once the scanner is plugged in, and
  `eth0.scanner_lease` shows the static lease once the scanner boots.

## 6. Day-2 operations via MCP

Target resolution is automatic for every `bridge_*` tool and CLI
subcommand, in order: an explicit `host` argument, `$ROOMSCAN_BRIDGE_HOST`,
mDNS lookup of `_roomscan-bridge._tcp` (published on `wlan0` only), then the
literal fallback `roomscan-bridge.local`. ssh authentication uses the key
pair at `~/.ssh/roomscan-bridge` (`$ROOMSCAN_BRIDGE_KEY` to override),
generated by the builder the first time it runs and baked into the image's
`authorized_keys`. `ssh` runs with `BatchMode=yes`, so a wrong or missing
key fails fast instead of hanging on a password prompt.

| MCP tool | Answers | CLI equivalent |
|---|---|---|
| `bridge_status(host?)` | Full health: model/hostname, `wlan0` SSID/RSSI/bitrate/power_save/IP, `eth0` carrier + scanner lease/neighbor state, nftables per-rule packet/byte counters, tee ring state, per-unit systemd state, temperature/throttling, NTP sync — plus a `warnings` list of the handful of things worth acting on. | `host/tools/pi_bridge.py status [--json]` |
| `bridge_logs(unit?, lines?, host?)` | Tails one unit's journal. `unit` accepts a systemd unit name or the syslog tag `roomscan-bridge` (where the installer and reconcile timer log). | `host/tools/pi_bridge.py logs [unit] [--lines N]` |
| `bridge_wifi_update(ssid, psk, profile?, host?)` | Changes one named Wi-Fi profile (`home` or `travel`) and reports the *actual* association state read back after the radio settles, not the `nmcli` exit code. | `host/tools/pi_bridge.py wifi-update SSID PSK [--profile home\|travel]` |
| `bridge_update(host?, secrets?)` | Re-renders the payload from this checkout, pushes it over `scp`, reruns `install.sh`, and reports the resulting unit states — how a config/payload change ships without reflashing a card. | `host/tools/pi_bridge.py update [--secrets PATH]` |
| `bridge_tee_list(host?)` | Lists the pcap ring files on the Pi, newest first, with size and mtime. | `host/tools/pi_bridge.py tee-list` |
| `bridge_tee_fetch(name, out?, convert?, host?)` | Copies one ring pcap off the Pi and (by default) converts it to a capture `.bin`. See §7. | `host/tools/pi_bridge.py tee-fetch NAME [-o OUT] [--no-convert]` |
| `bridge_reboot(host?)` | Reboots the Pi; returns immediately (a dropped ssh channel mid-reboot counts as success, not failure). | `host/tools/pi_bridge.py reboot` |
| `bridge_image_build(secrets?, release?, xz?, skip_debs?)` | Builds the SD image (§3). | `./pi-bridge/build_image.py --secrets ...` |
| `bridge_pcap_convert(path, out?)` | Converts a pcap already on this host (no ssh) into a capture `.bin` + loss stats. | `host/.venv/bin/python host/tools/pcap2capture.py PCAP [-o OUT] [--json]` |

Every tool reports what it actually found, not what was asked for: an
unreachable Pi returns `{"ok": false, "error": ...}` rather than raising,
`bridge_wifi_update` reports the real `associated`/`actual_ssid` state
rather than trusting `nmcli`'s exit code, and `bridge_update` reads unit
states back from the Pi after the install runs. `bridge_status` in
particular is where to start whenever a capture looks lossy and you suspect
the wireless hop — read its `warnings` list first.

## 7. Recovering lost frames from the tee

The Pi tees every scanner UDP packet on `eth0` to a bounded local pcap ring
(`roomscan-tee.service`: `tcpdump -i eth0 -n -s 0 'udp port 5000' -w
ring.pcap -C 100 -W 20`) independently of whether it makes it across the
Wi-Fi hop. This is the direct fix for the FileHub's silent frame loss: a
frame the wireless link drops is still sitting in the pcap.

Recovery flow:

1. `bridge_tee_list()` — find the ring file(s) covering the take.
2. `bridge_tee_fetch(name)` — copies the pcap off the Pi and, by default,
   converts it in the same call.

The conversion is the whole point: the resulting `.bin` is
**byte-format-identical** to what `capture.py` writes directly from the
live stream, so every existing `capture_*` tool (`capture_analyze`,
`capture_meta`, `capture_motion`, …) reads it unmodified. Compare its loss
counters against the host's own recorded counters for the same take —
`frames_incomplete`, `frame_loss_pct`, `frags_lost`, `frags_reordered`,
`frags_duplicate`, `frags_invalid` (the exact keys `pcap2capture.py`'s
`convert()` returns). If the pcap's loss numbers are lower than what the
live host recorded, those frames were genuinely lost over Wi-Fi and are now
recovered; if they match, the loss happened upstream of the Pi and the tee
can't help with it.

For a pcap already sitting on this host (already fetched, or copied off the
card by hand), skip the ssh hop with `bridge_pcap_convert(path)` or
`host/tools/pcap2capture.py` directly — same conversion, same stats.

**The ring is bounded: 20 files × 100 MB = 2 GB total**, roughly **70
minutes** of coverage at the measured ~466 KB/s stream rate. Don't expect
to recover a capture from last week — fetch what you need soon after the
session, before the ring wraps around it.

## 8. Failure playbook

| Symptom | Check | Fix |
|---|---|---|
| Pi not discoverable via MCP/CLI | `avahi-browse -r _roomscan-bridge._tcp` from a laptop on the same Wi-Fi; `$ROOMSCAN_BRIDGE_HOST` set to something stale | Set `$ROOMSCAN_BRIDGE_HOST` explicitly to a known-good IP as a workaround; confirm the Pi actually associated (needs HDMI or the FileHub cold-spare path, §9, if wireless is fully down) |
| Scanner has no lease / `eth0.scanner_lease` is null | `bridge_status()` → `eth0.carrier` (is it plugged in?), `eth0.scanner_neigh` | The scanner missed dnsmasq's 3000 ms DHCP window and self-assigned `172.31.253.1`. `roomscan-bridge-reconcile` (runs every 10s) detects this via a temporary probe alias, retargets the nftables DNAT rule to the fallback address so the stream keeps working immediately, then bounces `eth0` to push the scanner's DHCP client back to `INIT` and re-request a real lease — **but only when the nftables DNAT counters show no live stream traffic**; it will not bounce the link mid-capture. Check `bridge_logs("roomscan-bridge-reconcile")` for what it decided. |
| Scanner *never* gets a lease, on every boot | `ssh` in and run `dnsmasq --test -7 /etc/dnsmasq.d`, and `grep CONFIG_DIR /etc/default/dnsmasq` | Our DHCP config lives in `/etc/dnsmasq.d/roomscan-bridge.conf`, and Debian reads that directory through **`CONFIG_DIR` in `/etc/default/dnsmasq`** plus the packaged systemd-helper's `-7` flag — *not* through `/etc/dnsmasq.conf`, where every `conf-dir=` line ships commented out. If `CONFIG_DIR` is gone, dnsmasq starts clean, every unit reports `active`, and the scanner simply never gets an answer. `install.sh::ensure_dnsmasq_reads_dropins` checks this on every run and appends a `conf-dir=` line if neither mechanism is in place, so re-running `bridge_update()` repairs it. |
| Wi-Fi associated but lossy | `bridge_status()` → `wlan0.power_save` (should read `off`; `on`/anything else means `brcmfmac` power-save is adding burst latency and dropping frames), `wlan0.rssi_dbm` (below ~-70 dBm expect loss) | If `power_save` drifted back on, `bridge_update()` reapplies the payload's `wifi.powersave=2`→off NetworkManager config; a weak RSSI is a physical/placement problem, not a config one — move the Pi or the AP |
| Bad Wi-Fi credentials (Pi unreachable over wireless) | Nothing responds on `wlan0` at all | Chicken-and-egg: you can't ssh in over Wi-Fi to fix Wi-Fi. Pull the SD card, mount its FAT partition on any laptop, and drop a rendered `.nmconnection` file at `/boot/firmware/wifi-override.nmconnection` — `install.sh` installs it (0600, `roomscan-wifi-override.nmconnection`) and reloads NetworkManager on **every** run, first-boot or update, so this recovers without a reflash |
| Tee not running | `bridge_status()` → `tee.active` | `bridge_update()` (re-enables/restarts it) or `bridge_logs("roomscan-tee")`; the tee is deliberately fail-open, so its absence never affects the live stream — only future recoverability |
| Clock looks wrong | `bridge_status()` → `ntp_synced` | The Pi 3 has no RTC, so its clock is meaningless (and tee/journal timestamps with it) until NTP sync completes after boot; this resolves itself once the Pi has had wireless connectivity for a bit — no fix needed unless it stays `false` indefinitely |
| Undervoltage / throttling | `bridge_status()` → `temp_c`, `throttled` (non-`0x0` means the firmware itself reported throttling) | A Pi 3 draws ~300–400 mA idle and spikes past 700 mA under Wi-Fi + CPU load at 5 V — it needs its own properly sized 5 V rail off the rig battery. Do not assume the FileHub's old supply is interchangeable; check the actual current budget before reusing it |

## 9. FileHub cold-spare rule

Keep the FileHub as a cold spare, but **never plug it in at the same time
as the Pi bridge.** Both would announce the mDNS instance `roomscanner` on
the same Wi-Fi segment, which is a name collision — avahi resolves that by
silently renaming one of the two instances (e.g. to `roomscanner #2`), and
the host's `UdpSource` lookup, which asks for the literal name
`roomscanner`, would then find whichever one didn't get renamed, or
neither, with no error surfaced anywhere. If you need to fall back to the
FileHub, physically disconnect the Pi first.

## 10. Invariants this must keep

From the design spec §8 — these are load-bearing, not incidental:

1. **Additive, not mandatory.** The scanner must remain directly usable
   over a plain Ethernet cable to a laptop with the Pi completely absent.
   This bridge is an optional hop, never a required one.
2. **Transparent relay on the primary path.** The Pi does not decode,
   mutate, or rate-limit RSCN frames — it forwards bytes. The pcap tee is
   copy-only and fail-open: `tcpdump` only observes what the kernel already
   forwarded, and nothing about the data path depends on the tee unit being
   installed, enabled, or even alive.

## 11. Not yet verified on hardware

Everything above is implemented and covered by unit tests that run on this
dev host (`test_pi_bridge_build.py`: MBR parsing, a mini-image mtools
inject/read-back round trip, secrets hygiene, payload lint, deb-closure
resolution; `test_pcap2capture.py`: golden synthetic pcaps exercising clean,
reordered, dropped, and duplicate fragment cases). None of that reaches a
real Pi. Per this project's rule that nothing closes on unverified work, the
governing issue [#191](https://github.com/hellosamblack/lidar-roomscanner/issues/191)
stays open with `needs/operator` until the spec's acceptance gates (§11 of
the design spec) are actually run:

1. **Parity** — discovery, FPS, CRC, and gap counters over a representative
   capture match or beat the current FileHub baseline, measured
   side-by-side, not just "it streams."
2. **Reliability** — with the tee enabled, measured frame loss on a real
   capture is lower than the FileHub-only baseline (2.3–9.4%); if it isn't,
   the tee hasn't earned its complexity.
3. **Bounded failure** — killing Wi-Fi, power-cycling the Pi, or unplugging
   it mid-capture degrades no worse than today's FileHub failure mode, and
   the direct-Ethernet-to-laptop fallback still works untouched.
4. **Reconcile drill** — booting the Pi after the scanner (so the scanner
   is already in its self-assigned fallback state) exercises the retarget +
   bounce path live, not just in the logic that unit-tests can reach.
5. **Kiosk display, if ever enabled** — measured FPS/CPU on real Pi 3
   hardware, no optimistic default (out of scope for v1; noted for
   completeness since the design spec flags it).

None of these can be exercised on this host — they need a real Pi 3, a real
capture, and a human's eyes on the rig.
