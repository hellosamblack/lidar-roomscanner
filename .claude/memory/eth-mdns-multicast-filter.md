---
name: eth-mdns-multicast-filter
description: STM32H5 ETH MAC drops ALL multicast by default — mDNS (and future PTP) need PassAllMulticast or an igmp_mac_filter callback; how to diagnose/test mDNS on this rig
metadata: 
  node_type: memory
  type: project
  originSessionId: c24b983e-f789-420a-86d2-d13871ed2a73
---

**Fixed & merged 2026-07-15 (PR #11).** Symptom: the board answered unicast to its DHCP IP
(172.17.2.58) and `ping <ip>` worked, but `roomscanner.local` (mDNS) never resolved.

**Root cause:** `HAL_ETH_Init` leaves the STM32H5 MAC receive filter at its reset default — only the
perfect (own-unicast) address and broadcast pass; **all multicast is dropped in hardware.** lwIP's mDNS
responder joins group 224.0.0.251 (MAC `01:00:5E:00:00:FB`) at the IP layer, but with no
`netif->igmp_mac_filter` callback wired to this driver (`Src/ethernetif.c`), the peripheral never learns
to accept that MAC, so inbound mDNS queries are discarded before lwIP ever sees them → the responder
can't answer. `LWIP_IGMP` and `NETIF_FLAG_IGMP` were already on; those govern IP-layer join, not the
hardware filter.

**Fix:** in `low_level_init`, after `HAL_ETH_Init`, `HAL_ETH_GetMACFilterConfig` → set
`filter.PassAllMulticast = ENABLE` → `HAL_ETH_SetMACFilterConfig`. Lets all multicast reach lwIP, which
filters in software. Chosen over a selective `igmp_mac_filter` (which needs CRC32 hash-table
programming) because on a direct link / small LAN the extra multicast volume is negligible.

**Forward-looking:** Phase 6's optional PTP hardware time-sync is *also* multicast (224.0.1.129 /
`01:00:5E:00:01:81`). PassAllMulticast already covers it; if that's ever tightened to a selective HW
filter, add the proper `igmp_mac_filter` instead — don't reintroduce the drop.

**Diagnosing/testing mDNS from the Windows host (this rig):**
- `ping roomscanner.local` uses the OS resolver, which queries mDNS on **all** NICs — this is the
  simplest live check (no hosts entry exists; `.local` is mDNS-only, so a resolve = the responder
  answered).
- A **raw-socket** mDNS query egresses on the *virtual* Hyper-V/WSL adapter (172.20.48.1) by default and
  never reaches the board. Pin it: bind the host IP on the board's subnet (172.17.2.x), join 224.0.0.251,
  and set `IP_MULTICAST_IF` to that host IP. A correct query then gets a direct A-record answer
  (ancount=1) from 172.17.2.58.
- lwIP replies to a non-QU query via **multicast to 224.0.0.251:5353**, so a listener must bind 5353 +
  join the group to hear it — an ephemeral-port socket won't.

See [[mapping-pipeline-plan]] (Phase 5 Ethernet). Firmware build/flash loop: [[worktree-subagent-gotchas]]
(building in a worktree) + the firmware-loop skill.
