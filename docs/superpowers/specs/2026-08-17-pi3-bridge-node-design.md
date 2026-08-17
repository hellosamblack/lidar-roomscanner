# Raspberry Pi 3 bridge node

## Status

**Accepted and implemented (2026-08-17), hardware acceptance outstanding.**
The §10 open questions are decided (see below) and the design is built:
`pi-bridge/` (rootless image builder + payload), `host/tools/pi_bridge.py` +
`bridge_*` MCP tools, and `host/tools/pcap2capture.py` for the tee. Governing
issue **#191**; procedure in [`../../pi-bridge-runbook.md`](../../pi-bridge-runbook.md).
No firmware or wire-protocol change was needed, as anticipated.

**None of §11's acceptance gates have run** -- every one of them needs a real Pi
and a real capture -- so #191 stays open with `needs/operator`. Do not read
"implemented" as "verified".

Owner decisions, 2026-08-17, superseding the open questions in §10:

- **§10.1 bridge shape: routed + NAT, not L2.** True transparent bridging is not
  available here: `brcmfmac` has no 4-address station bridging, and the
  proxy-ARP workaround would make the scanner's DHCP lease hostage to the Wi-Fi
  hop -- which, given the firmware's 3000 ms DHCP window (§3), means a slow
  association silently drops the scanner into its self-assigned fallback mode.
  §4's "recommend L2 first" is therefore withdrawn, not deferred. The mDNS
  concern in §8.3 is met by publishing `roomscanner._roomscan._udp` from the
  Pi's own avahi on `wlan0`, so `UdpSource` needs no change.
- **§10.2/§7.1 tee: in v1.** Bounded 2 GB pcap ring on the Pi.
- **§10.3 hardware: Pi 3 Model B.**
- **§10.4 travel-mode:** `travel-ap.sh` needs no changes (it does not care which
  client associates); `filehub-bridgemode.sh` becomes dead weight and should be
  retired in a follow-up.
- **§10.5 FileHub: kept as a cold spare**, never plugged in alongside the Pi --
  two responders announcing `roomscanner` on one segment is a name collision.

## 1. Goal

Mount a Raspberry Pi 3 on the rig stack, wired to the existing STM32H563
Ethernet link, replacing the RavPower FileHub as the scanner's wireless
uplink to the server. Optionally: tee raw frames to local storage as a
write-ahead capture buffer, and optionally display `roomscan-web` on an
attached screen. Explicitly **no on-device SLAM or splat compute** — the Pi
is an I/O and bridging node only, matching its actual class (BCM2837, 1 GB
RAM). A future revision may add a USB global-shutter camera to this same
node (§9); this spec anticipates that but does not design it.

## 2. Scope and non-goals

**In scope:** network topology (scanner → Pi → server), transport choice for
the scanner-to-Pi hop, wireless-uplink behavior replacing FileHub bridge
mode, optional local recording tee, optional kiosk display, power/mounting
budget, and the bus-sharing constraint a future USB camera would introduce.

**Out of scope:** the camera module itself (sensor choice, capture pipeline,
timestamp alignment — a separate future spec once a module is picked), any
on-device compute (SLAM, ICP, splat training all stay on the server GPU per
the existing Phase 6/7 architecture and the `distributed-slam-spike` note),
and a full task-level implementation plan.

## 3. Current state (baseline being replaced)

- The scanner streams RSCN frames over Ethernet/UDP (transport-agnostic
  framing, `docs/protocol.md`); the host's `UdpSource` discovers the device
  via mDNS (`roomscanner.local`) and sends a periodic unicast keepalive
  (`docs/system-architecture.md`).
- Today's wireless hop is a commercial FileHub acting as a Wi-Fi bridge:
  wired to the scanner, associated to the wireless network as a client,
  forwarding raw Ethernet frames. Bridge-mode recovery is a fragile manual
  sequence (unplug Ethernet → power-cycle → apply bridge mode → reconnect
  Ethernet), and it is a black box — nothing on that hop is observable.
- The `travel-mode` plan (`docs/superpowers/plans/2026-08-03-travel-mode.md`)
  adds a Proxmox-hosted 2.4 GHz AP as a fallback network when no wired
  uplink is present at the server end; the FileHub associates to whichever
  SSID (home `Surly_Office` or the travel AP) is currently live.
- Known problem this is meant to help: captures lose 2.3–9.4% of frames
  while reporting clean (`[[capture-loss-invisible-and-dominant]]`), and the
  wireless bridge — opaque, unobservable, with a manual recovery dance — is
  the leading suspect.

## 4. Proposed topology

```
STM32H563 --[Ethernet: UdpSource discovery + RSCN/UDP]--> Pi 3 (eth0)
                                                              |
                                          Pi 3 (wlan0, 2.4 GHz client)
                                          --[Surly_Office or travel AP]--> server
                                   (optional) local storage: write-ahead frame tee
                                   (optional) HDMI/DSI: roomscan-web kiosk view
                                   (future)   USB global-shutter camera
```

**Bridge shape — decide L2 vs L3 (§10.1):** the simplest drop-in replacement
for the FileHub is an L2 bridge (`eth0`↔`wlan0`, same subnet, no DHCP/NAT on
the Pi) so the scanner keeps its existing address behavior and the host's
`UdpSource`/mDNS/keepalive path is untouched. An L3/NAT hop (Pi owns its own
subnet, forwards/NATs to the wireless network) is more robust to Wi-Fi
flakiness (the Pi can hold a stable link-local address to the scanner even
while its uplink bounces) but requires re-verifying mDNS/multicast discovery
and the keepalive path across the extra hop. Recommend starting with L2 to
minimize the blast radius of this change, and revisit L3 only if bridging
turns out to reintroduce the same opacity problem it's meant to fix.

## 5. Scanner-to-Pi link: Ethernet, not USB

Both are already scanner-supported transports (protocol.md's framing is
transport-agnostic; CDC and UDP are both live). Ethernet is the right choice
here:

- It's a transparent drop-in for the FileHub's existing role — no host-side
  change, no new driver.
- It keeps USB free. A Pi 3 has exactly one shared USB2 controller behind
  all four ports (and, on the 3B+, behind the "gigabit" Ethernet jack too) —
  reserving USB for a future camera avoids putting camera bytes on the same
  bus as anything else from day one.
- USB CDC would reintroduce the cable-length limit that Ethernet (Phase 5)
  was specifically built to remove.

## 6. Wireless uplink, replacing FileHub bridge mode

The Pi 3's onboard radio is 802.11n, **2.4 GHz only** — same constraint the
travel-mode AP already designs around, and it must land on Home Wi-Fi's
2.4 GHz band specifically (not the network's 5 GHz SSID, if dual-band).
Rather than the FileHub's fragile explicit "bridge mode" toggle, the Pi
associates as a normal `wpa_supplicant` client to whichever SSID is active
(home `Surly_Office` or the travel AP), taking its own DHCP lease. That's
arguably a reliability win on its own: it removes the manual
unplug/power-cycle/re-apply recovery sequence entirely, and — if this
extends the travel-mode plan — the Pi is the thing that should register as
the travel AP's known client, not the FileHub's MAC as recorded today.

## 7. Optional roles

**7.1 Local write-ahead recording tee (highest-value option).** The Pi
copies frame bytes to local storage (microSD or a USB stick, budget
permitting against §9) before or independent of forwarding them wirelessly,
following the same invariants as the Rerun sidecar design (fail open, off
the hot path, bounded queue, drop only its own tee on overload, never delay
or drop a frame headed to the primary link). This is the direct fix for
§3's frame-loss problem: today, if the Wi-Fi hop drops a frame, it is gone;
with a local tee, it's recoverable from the Pi after the fact. Needs a
measured write bandwidth check against card/stick throughput before this is
more than a proposal (§10.2).

**7.2 `roomscan-web` kiosk display.** Optional and off by default. Be
realistic about the hardware: headless Chrome/WebGL is known to be
expensive even on real desktop-class CPUs
(`[[headless-chrome-burns-cores]]`), and a Pi 3's VideoCore IV GPU is not in
that class. Full 3D `roomscan-web` at usable FPS on a Pi 3 is unlikely;
treat this as "maybe a stripped-down status page," not "the real UI on a
small screen," until measured.

**7.3 PTP boundary clock.** The Pi is a plausible place to run `linuxptp`
ahead of Phase 6's hardware time-sync item, since it already sits on the
scanner's wired segment. Worth a line item later; not designed here.

## 8. Required invariants

1. **No new single point of failure worse than today's.** The scanner must
   remain directly usable over a wired Ethernet cable straight to a laptop
   if the Pi is absent, dead, or being swapped — this is an additive bridge
   node, not a mandatory hop.
2. **Transparent relay on the primary path.** In its bridging role the Pi
   forwards bytes; it does not decode, mutate, or rate-limit RSCN frames.
   Any optional local tee is copy-only and non-blocking, matching the
   Rerun-sidecar fail-open pattern — a tee fault disables only the tee.
3. **Multicast/mDNS survives the extra hop.** This needs explicit
   verification, not an assumption — the STM32H5 ETH MAC already drops
   multicast by default on the firmware side
   (`[[eth-mdns-multicast-filter]]`); an L2 bridge or Wi-Fi AP hop is a
   second place multicast forwarding can silently fail. Confirm
   `roomscanner.local` discovery and the UDP keepalive both work end to end
   before calling this a working replacement.
4. **Truthful reporting.** If the Pi is a bridge, its own health (radio
   state, association status, forwarding drops) needs to be visible
   somewhere — a black-box bridge is exactly the problem being replaced, so
   this one should not become opaque in the same way.

## 9. Power, mounting, and the future-camera bus constraint

A Pi 3 draws roughly 300–400 mA idle and can spike past 700 mA under
Wi-Fi + CPU load at 5 V — it needs its own sized regulated rail off the rig
battery, not an assumption that the FileHub's existing supply is
interchangeable. Mounting, weight, and heat on a handheld rig also need a
pass; this spec doesn't design the physical mount.

On the anticipated USB global-shutter camera: flag now, don't design yet.
A Pi 3 has a single shared USB2 bus. Scanner traffic goes over Ethernet
(§5), which helps, but a future camera stream would still share that one
USB2 bus with anything else USB-attached (a local-recording USB stick, if
that's the storage choice in §7.1). Budget the camera's real bandwidth need
against that shared bus before committing to a Pi 3 as the camera host —
this is also consistent with the caution given in the earlier camera
conversation: a Pi 3 is fine as an I/O node, but its ceiling is meaningfully
lower than a Pi 4/5 or CM4/5, and camera quality/bus headroom is exactly
where that ceiling would first be felt.

## 10. Risks and open questions (owner input needed)

1. **Bridge shape (§4):** L2 transparent bridge vs. L3/NAT hop. Recommend
   L2 first; needs an explicit decision before implementation.
2. **Bandwidth check, not assumption.** Before designing the local tee
   (§7.1), measure the scanner's actual sustained frame rate/size against
   the Pi 3's real Ethernet throughput (10/100 vs. Gigabit-over-USB2 depends
   on the exact Pi 3 model — 3, 3B, or 3B+; confirm which units are on
   hand) and against microSD/USB-stick sustained write speed. Don't assume
   headroom either direction.
3. **Which Pi 3 model(s)?** 3B+'s Ethernet is faster but still shares the
   one USB2 controller; plain 3/3B is 10/100 only. This changes the
   bandwidth math in #2 and matters more once the camera is in play.
4. **Interaction with travel-mode.** Does the Pi replace the FileHub as the
   travel AP's client too, or does travel-mode need its own update to
   recognize the Pi's MAC/behavior? `travel-ap.sh`'s gating logic doesn't
   care who the wireless client is, but the FileHub-specific bridge-recovery
   steps (`filehub-bridgemode.sh`) become dead weight once the Pi takes over
   and should be retired, not left as a second, now-redundant path.
5. **Does the FileHub get removed entirely, or kept as a cold spare?** Given
   how flaky it's been, keeping one as a fallback bridge (not in the primary
   path) may be worth the shelf space during the Pi rollout.

## 11. Acceptance gates (before calling this shipped)

1. **Parity:** discovery, FPS, CRC, and gap counters over a representative
   capture match or beat the current FileHub baseline — not just "it
   streams," but a real side-by-side.
2. **Reliability:** with the local tee enabled, measured frame loss on a
   real capture is lower than the FileHub-only baseline
   (`[[capture-loss-invisible-and-dominant]]`); if it isn't, the tee isn't
   earning its complexity.
3. **Bounded failure:** killing Wi-Fi, power-cycling the Pi, or unplugging
   it mid-capture degrades no worse than today's FileHub failure mode, and
   the direct-Ethernet-to-laptop fallback (§8.1) still works untouched.
4. **Kiosk display, if enabled:** measured FPS/CPU on real Pi 3 hardware
   decides whether it's the real UI or a stripped status page — no
   optimistic default.

## 12. Documentation affected by implementation

`docs/system-architecture.md`'s network description, the travel-mode plan
(§10 above), `ROADMAP.md`'s plans/specs register, and likely a new
operational runbook parallel to `travel-ap.sh` for the Pi's own
install/reconcile/status story. No change to `docs/protocol.md` — this adds
no wire-protocol behavior.
