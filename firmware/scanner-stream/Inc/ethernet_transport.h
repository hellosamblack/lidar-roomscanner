#ifndef ETHERNET_TRANSPORT_H
#define ETHERNET_TRANSPORT_H

#include <stdint.h>
#include <stdbool.h>

#define ETH_MAC_ADDR0 0x00
#define ETH_MAC_ADDR1 0x80
#define ETH_MAC_ADDR2 0xE1
#define ETH_MAC_ADDR3 0x00
#define ETH_MAC_ADDR4 0x00
#define ETH_MAC_ADDR5 0x00

void ETH_Init(void);
void ETH_Process(void);
bool ETH_IsUp(void);
bool ETH_HasTarget(void);
bool ETH_SendFrame_Gather(const uint8_t *hdr, uint32_t hdr_len, const uint8_t *payload, uint32_t payload_len, const uint8_t *tail, uint32_t tail_len);

/* Drain buffered inbound COMMAND bytes (received over UDP) into `dst`, up to `max`.
 * Returns the number copied (0 if none). Same contract as tud_cdc_read, so the main
 * loop's transport-agnostic command poll can pull from either transport. Called only
 * from the main loop (the udp_recv callback that fills the buffer runs synchronously
 * inside ETH_Process, also on the main loop -- no ISR concurrency). */
uint32_t ETH_ReadCommands(uint8_t *dst, uint32_t max);

/* Frames discarded by the paced TX queue because the link/target went away
 * before they could be sent. Monotonic; 0 on a healthy link. Distinct from
 * host-side seq gaps, which also count datagrams lost in the network. */
uint32_t ETH_TxDroppedFrames(void);

/* ===================== Task 6: applied-period-aware pacing + queue telemetry =====
 *
 * docs/superpowers/plans/2026-07-31-high-framerate-and-manual-ranging-modes.md Task 6.
 * The old ETH_TX_WINDOW_MS was a fixed 25 ms compile-time constant sized off a 30 Hz
 * (33 ms) assumption (non-negotiable finding #6) -- wrong the moment a profile runs
 * faster (HFR: 21739 us) or slower (Manual: down to 1 fps) than that. The window is
 * now runtime-settable; the caller (vl53l9_app.c) derives it from whichever stream
 * period(s) actually govern the queue right now and calls ETH_SetTxWindowMs() every
 * time the applied ranging profile changes. Deliberately generic here -- this file
 * only receives "drain within window_ms", never computes a period itself -- so a
 * future decoupled IMU/env tick (Task 7) can fold into the caller's aggregate
 * (e.g. min of the ToF and IMU/env periods) without this API changing shape. */

/* Sets the pacer's drain deadline (see eth_tx_pump()'s block comment in the .c file).
 * Clamped to >= 1 ms; a 0 or absurdly large caller value would either divide-by-zero
 * the budget formula or starve the queue for the caller's whole (mistaken) window. */
void ETH_SetTxWindowMs(uint32_t window_ms);

/* Convenience: window_ms for a single governing frame period (the common case --
 * just the ToF profile). frame_period_us == 0 (should not happen; g_active_profile is
 * always seeded before use) floors to 1 ms rather than dividing by zero. Pure integer
 * math, no HAL/lwIP dependency -- mirrored host-side (NOT a byte-level cross-check;
 * this file pulls in lwIP headers the host cannot compile) by
 * roomscan.sources.eth_tx_window_ms(), exercised in host/tests/test_sources.py. */
uint32_t ETH_TxWindowMsForPeriod(uint32_t frame_period_us);

/* Queue telemetry (Task 6 step 2), all monotonic since boot. A 60 s hardware capture
 * samples these at start and end and takes the delta -- "prove zero", not infer it. */
uint8_t  ETH_TxQueueHighWater(void);    /* max frames ever queued at once (0..ETH_TX_SLOTS) */
uint32_t ETH_TxPendingFragments(void);  /* fragments currently owed across every queued frame */
uint32_t ETH_TxEnqueueDrops(void);      /* frames that never got a queue slot at all (queue
                                          * full even after a blocking flush) -- distinct from
                                          * ETH_TxDroppedFrames(), which is queued frames later
                                          * discarded because the link/target went away */
uint32_t ETH_TxStackStalls(void);       /* lwIP/pbuf refusals (pool pressure or ERR_MEM) that
                                          * eth_tx_emit_one() retried past, across every call */
uint32_t ETH_TxEmittedBytes(void);      /* total bytes actually handed to udp_sendto() (wraps) */

#endif
