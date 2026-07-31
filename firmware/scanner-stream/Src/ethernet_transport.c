#include "ethernet_transport.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/dhcp.h"
#include "lwip/udp.h"
#include "ethernetif.h"
#include "dhcpserver.h"
#include "netif/ethernet.h"
#include "stm32h5xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "lwip/apps/mdns.h"

struct netif gnetif;
static struct udp_pcb *upcb = NULL;
static bool eth_link_up = false;
static uint32_t frame_seq_num = 0;
static ip_addr_t target_ip;

/* Inbound COMMAND accumulation. Every datagram's payload is appended here (bounded);
 * the main loop drains it via ETH_ReadCommands at the command-poll safe point and runs
 * it through the same rs_parse_command path as the USB CDC RX. Keepalive datagrams and
 * any other non-command bytes are harmless -- the parser scans for the frame magic and
 * resyncs past junk. Sized to hold several 44-byte command frames between ~36 ms polls;
 * on overflow new bytes are dropped (the host's command client retries on ACK timeout).
 * No lock: the udp_recv callback runs synchronously inside ethernetif_input() (bare-metal
 * lwIP, NO_SYS), i.e. on the same main-loop thread as ETH_ReadCommands -- never an ISR. */
#define ETH_CMD_BUF_SIZE 256
static uint8_t eth_cmd_buf[ETH_CMD_BUF_SIZE];
static uint16_t eth_cmd_len = 0;

/* Static IP config in case DHCP fails or no link */
#define IP_ADDR0 172
#define IP_ADDR1 31
#define IP_ADDR2 253
#define IP_ADDR3 1

#define NETMASK_ADDR0 255
#define NETMASK_ADDR1 255
#define NETMASK_ADDR2 255
#define NETMASK_ADDR3 0

#define GW_ADDR0 172
#define GW_ADDR1 31
#define GW_ADDR2 253
#define GW_ADDR3 1

typedef enum {
    DHCP_STATE_INIT,
    DHCP_STATE_CLIENT_WAITING,
    DHCP_STATE_CLIENT_BOUND,
    DHCP_STATE_SERVER
} dhcp_state_t;

static dhcp_state_t dhcp_state = DHCP_STATE_INIT;
static uint32_t dhcp_start_time = 0;

/* Defined with the paced-TX block further down; ETH_Process (above it) drives it. */
static void eth_tx_pump(void);

static void Netif_Config(void)
{
    ip4_addr_t ipaddr;
    ip4_addr_t netmask;
    ip4_addr_t gw;

    IP4_ADDR(&ipaddr, 0, 0, 0, 0);
    IP4_ADDR(&netmask, 0, 0, 0, 0);
    IP4_ADDR(&gw, 0, 0, 0, 0);

    /* Add the network interface */
    netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);

    /* Register the default network interface */
    netif_set_default(&gnetif);

    if (netif_is_link_up(&gnetif))
    {
        netif_set_up(&gnetif);
    }
    else
    {
        /* When the netif link is down this function must be called */
        netif_set_down(&gnetif);
    }
}

static void udp_receive_callback(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port) {
    if (p != NULL) {
        target_ip = *addr;
        /* Buffer the payload as potential COMMAND-frame bytes for the main loop to
         * parse (see eth_cmd_buf). This is also how a bare keepalive datagram claims
         * the stream target above -- its non-magic bytes are simply resynced away. */
        uint16_t space = (eth_cmd_len < ETH_CMD_BUF_SIZE) ? (uint16_t)(ETH_CMD_BUF_SIZE - eth_cmd_len) : 0;
        if (space > 0) {
            uint16_t n = (p->tot_len < space) ? (uint16_t)p->tot_len : space;
            pbuf_copy_partial(p, eth_cmd_buf + eth_cmd_len, n, 0);
            eth_cmd_len = (uint16_t)(eth_cmd_len + n);
        }
        pbuf_free(p);
    }
}

uint32_t ETH_ReadCommands(uint8_t *dst, uint32_t max) {
    uint32_t n = eth_cmd_len;
    if (n > max) {
        n = max;
    }
    if (n == 0) {
        return 0;
    }
    memcpy(dst, eth_cmd_buf, n);
    uint16_t remain = (uint16_t)(eth_cmd_len - n);
    if (remain > 0) {
        memmove(eth_cmd_buf, eth_cmd_buf + n, remain);
    }
    eth_cmd_len = remain;
    return n;
}

void ETH_Init(void)
{
    lwip_init();
    Netif_Config();
    mdns_resp_init();
    IP4_ADDR(&target_ip, 0, 0, 0, 0);
    upcb = udp_new();
    udp_bind(upcb, IP_ANY_TYPE, 5000);
    udp_recv(upcb, udp_receive_callback, NULL);
}

void ETH_Process(void)
{
    /* Before the RX/timeout work: the queued fragments are time-critical (they
     * must clear inside the frame period) and ethernetif_input can run long. */
    eth_tx_pump();

    ethernetif_input(&gnetif);
    sys_check_timeouts();
    
    static uint32_t last_link_check = 0;
    if (HAL_GetTick() - last_link_check > 500) {
        last_link_check = HAL_GetTick();
        ethernet_link_check_state(&gnetif);
        static bool last_printed_up = false;
        bool up = netif_is_link_up(&gnetif);
        if (up != last_printed_up || (HAL_GetTick() % 5000 < 500)) {
            printf("[ETH] Link State Poll: %s\n", up ? "UP" : "DOWN");
            last_printed_up = up;
        }
    }

    if (netif_is_link_up(&gnetif) && !eth_link_up)
    {
        eth_link_up = true;
        printf("[ETH] Link UP\n");
        netif_set_up(&gnetif);
        
        dhcp_state = DHCP_STATE_CLIENT_WAITING;
        dhcp_start(&gnetif);
        dhcp_start_time = HAL_GetTick();
        printf("[ETH] DHCP Client Started\n");
    }
    else if (!netif_is_link_up(&gnetif) && eth_link_up)
    {
        eth_link_up = false;
        printf("[ETH] Link DOWN\n");
        if (dhcp_state == DHCP_STATE_SERVER) {
            dhcps_deinit();
        } else {
            dhcp_stop(&gnetif);
        }
        netif_set_down(&gnetif);
        dhcp_state = DHCP_STATE_INIT;
        target_ip.addr = 0; // Clear target IP on link down
    }

    if (eth_link_up) {
        static bool mdns_added = false;

        if (dhcp_state == DHCP_STATE_CLIENT_WAITING) {
            if (gnetif.ip_addr.addr != 0) {
                dhcp_state = DHCP_STATE_CLIENT_BOUND;
                if (!mdns_added) {
                    mdns_resp_add_netif(&gnetif, "roomscanner", 3600);
                    mdns_resp_add_service(&gnetif, "roomscanner", "_roomscan", DNSSD_PROTO_UDP, 5000, 3600, NULL, NULL);
                    mdns_added = true;
                }
                mdns_resp_netif_settings_changed(&gnetif);
                printf("[ETH] DHCP Client Bound: IP %s\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)));
            } else if ((HAL_GetTick() - dhcp_start_time) > 3000) {
                // Timeout, switch to server
                printf("[ETH] DHCP Client Timeout, switching to Server (172.31.253.1)\n");
                dhcp_stop(&gnetif);
                
                ip4_addr_t ipaddr, netmask, gw;
                IP4_ADDR(&ipaddr, IP_ADDR0, IP_ADDR1, IP_ADDR2, IP_ADDR3);
                IP4_ADDR(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
                IP4_ADDR(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
                
                netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
                
                dhcps_init();
                dhcp_state = DHCP_STATE_SERVER;
                if (!mdns_added) {
                    mdns_resp_add_netif(&gnetif, "roomscanner", 3600);
                    mdns_resp_add_service(&gnetif, "roomscanner", "_roomscan", DNSSD_PROTO_UDP, 5000, 3600, NULL, NULL);
                    mdns_added = true;
                }
                mdns_resp_netif_settings_changed(&gnetif);
            }
        } else if (dhcp_state == DHCP_STATE_CLIENT_BOUND || dhcp_state == DHCP_STATE_SERVER) {
            static uint32_t last_ip_print = 0;
            if (HAL_GetTick() - last_ip_print > 5000) {
                last_ip_print = HAL_GetTick();
                printf("[ETH] Current IP: %s (%s mode)\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)), dhcp_state == DHCP_STATE_SERVER ? "Server" : "Client");
            }
        }
    }
}

bool ETH_IsUp(void)
{
    return eth_link_up;
}

/* ===================== paced fragment TX =====================
 *
 * A depth frame is RS_HEADER_SIZE + 14842 + CRC = 14878 B, which this splits
 * into 11 datagrams. Those used to go out back-to-back in a tight loop, which
 * is wrong twice over:
 *
 *   1. The TX descriptor ring is only ETH_TX_DESC_CNT (8) deep, and at 250 MHz
 *      the CPU enqueues a 1400 B memcpy far faster than the DMA drains one
 *      (~112 us each at 100 Mbit). On overrun udp_sendto returned ERR_MEM and
 *      the old code ABANDONED the frame mid-burst -- the fragments already
 *      sent are then guaranteed waste, because the host's reassembly (see
 *      sources.py) needs every fragment of a seq, in order.
 *   2. Through a Wi-Fi bridge, 11 packets arriving in ~1.3 ms is exactly the
 *      burst shape that overflows a cheap AP's queue. Loss there is not
 *      cosmetic: one lost datagram costs the WHOLE 14.8 KB frame, i.e. ~11x
 *      loss amplification, and a dropped depth frame doubles the inter-frame
 *      motion that ICP has to solve (BUG-036 territory).
 *
 * So frames are copied into a slot FIFO and their fragments are metered out
 * from ETH_Process(). Two invariants:
 *
 *   - STRICTLY IN ORDER, one frame at a time. The host keys reassembly on a
 *     single `seq` and resets its buffer the moment a different seq arrives,
 *     so interleaving fragments from two frames would destroy BOTH.
 *   - Never abandon a partially-sent frame. A send failure leaves next_frag
 *     put and retries on the next pump.
 *
 * Fixed slots rather than a byte ring: the wrap arithmetic is the only place
 * this could grow a memory-corruption bug, and at 573 KB free the ~118 KB the
 * slots cost is affordable insurance.
 */

#define ETH_TX_FRAG_BYTES   1400u
#define ETH_TX_SLOT_BYTES   15104u  /* >= 32 hdr + 14842 payload + 4 CRC */
#define ETH_TX_SLOTS        8u      /* one acquisition iteration queues <= 6 frames */
/* Drain whatever is queued within this window. Shorter than the 33 ms frame
 * period so the queue is empty again before the next frame lands. */
#define ETH_TX_WINDOW_MS    25u

typedef struct {
    uint8_t  data[ETH_TX_SLOT_BYTES];
    uint32_t len;
    uint32_t seq;
    uint8_t  total_frags;
    uint8_t  next_frag;
} eth_tx_slot_t;

static eth_tx_slot_t eth_tx_slots[ETH_TX_SLOTS];
static uint8_t  eth_tx_head = 0;    /* next slot to write */
static uint8_t  eth_tx_tail = 0;    /* oldest queued slot  */
static uint8_t  eth_tx_count = 0;
static uint32_t eth_tx_last_pump = 0;
static uint32_t eth_tx_dropped = 0; /* frames discarded (link lost) */

/* Emit ONE fragment of the tail slot. Returns false if the stack refused it,
 * leaving the slot's next_frag untouched so the same fragment is retried. */
static bool eth_tx_emit_one(void)
{
    if (eth_tx_count == 0) return false;
    eth_tx_slot_t *s = &eth_tx_slots[eth_tx_tail];

    uint32_t offset = (uint32_t)s->next_frag * ETH_TX_FRAG_BYTES;
    uint32_t chunk = s->len - offset;
    if (chunk > ETH_TX_FRAG_BYTES) chunk = ETH_TX_FRAG_BYTES;

    struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, chunk + 6, PBUF_RAM);
    if (!p) return false;            /* pool pressure: retry next pump */

    uint8_t *p_out = (uint8_t *)p->payload;
    p_out[0] = (uint8_t)(s->seq & 0xFF);
    p_out[1] = (uint8_t)((s->seq >> 8) & 0xFF);
    p_out[2] = (uint8_t)((s->seq >> 16) & 0xFF);
    p_out[3] = (uint8_t)((s->seq >> 24) & 0xFF);
    p_out[4] = s->next_frag;
    p_out[5] = s->total_frags;
    memcpy(&p_out[6], &s->data[offset], chunk);

    err_t err = udp_sendto(upcb, p, &target_ip, 5000);
    pbuf_free(p);
    if (err != ERR_OK) return false; /* ring full: retry, do NOT abandon */

    s->next_frag++;
    if (s->next_frag >= s->total_frags) {
        eth_tx_tail = (uint8_t)((eth_tx_tail + 1u) % ETH_TX_SLOTS);
        eth_tx_count--;
    }
    return true;
}

/* Fragments still owed across every queued slot -- the quantity the pacing
 * budget has to clear inside ETH_TX_WINDOW_MS. */
static uint32_t eth_tx_pending_frags(void)
{
    uint32_t n = 0;
    for (uint8_t i = 0; i < eth_tx_count; i++) {
        const eth_tx_slot_t *s = &eth_tx_slots[(eth_tx_tail + i) % ETH_TX_SLOTS];
        n += (uint32_t)(s->total_frags - s->next_frag);
    }
    return n;
}

/* Send everything queued, as fast as the stack accepts it. Used when the FIFO
 * is full (back-pressure) and when the link drops. Bounded: it gives up after
 * a fixed number of refusals rather than spinning on a wedged MAC. */
static void eth_tx_flush_blocking(void)
{
    uint32_t stalls = 0;
    while (eth_tx_count > 0 && stalls < 1000u) {
        if (!eth_tx_emit_one()) stalls++;
    }
}

/* Meter queued fragments onto the wire. Called from ETH_Process().
 *
 * The budget is adaptive because the call cadence is coarse and not ours to
 * set: platform_wait_for_event busy-waits in 5 ms slices, so ETH_Process may
 * only run every ~5 ms. A fixed inter-fragment gap would therefore under-drain
 * (2 frags/5 ms = 13 per frame period, against the ~17 a CALIB iteration
 * queues) and the backlog would grow without bound. Sizing the per-call budget
 * from the OUTSTANDING count instead guarantees the queue clears inside the
 * window whatever the cadence, while still spreading a small backlog out. */
static void eth_tx_pump(void)
{
    if (eth_tx_count == 0) { eth_tx_last_pump = HAL_GetTick(); return; }

    if (!eth_link_up || !upcb || target_ip.addr == 0) {
        /* Nowhere to send: discard rather than stall the acquisition loop
         * behind a queue that can never drain. */
        eth_tx_dropped += eth_tx_count;
        eth_tx_head = eth_tx_tail = eth_tx_count = 0;
        return;
    }

    uint32_t now = HAL_GetTick();
    uint32_t elapsed = now - eth_tx_last_pump;
    if (elapsed == 0) return;
    eth_tx_last_pump = now;

    uint32_t pending = eth_tx_pending_frags();
    uint32_t budget = (pending * elapsed + ETH_TX_WINDOW_MS - 1u) / ETH_TX_WINDOW_MS;
    if (budget == 0) budget = 1;     /* always make forward progress */

    while (budget-- > 0) {
        if (!eth_tx_emit_one()) break;
    }
}

bool ETH_SendFrame_Gather(const uint8_t *hdr, uint32_t hdr_len, const uint8_t *payload, uint32_t payload_len, const uint8_t *tail, uint32_t tail_len)
{
    if (!eth_link_up || !upcb) return false;
    if (target_ip.addr == 0) return false; // Wait for host to send a packet first!

    uint32_t total_len = hdr_len + payload_len + tail_len;
    if (total_len == 0 || total_len > ETH_TX_SLOT_BYTES) {
        printf("[ETH] frame too large to queue: %lu\n", (unsigned long)total_len);
        return false;
    }

    /* Back-pressure, not loss: the caller's payload buffer is about to be
     * reused (raw double-buffering), so the bytes must be taken now. Draining
     * the queue synchronously costs latency on this one frame; dropping would
     * cost the host a whole frame. */
    if (eth_tx_count >= ETH_TX_SLOTS) {
        eth_tx_flush_blocking();
        if (eth_tx_count >= ETH_TX_SLOTS) return false;
    }

    eth_tx_slot_t *s = &eth_tx_slots[eth_tx_head];
    memcpy(&s->data[0], hdr, hdr_len);
    memcpy(&s->data[hdr_len], payload, payload_len);
    memcpy(&s->data[hdr_len + payload_len], tail, tail_len);
    s->len = total_len;
    s->seq = frame_seq_num++;
    s->total_frags = (uint8_t)((total_len + ETH_TX_FRAG_BYTES - 1u) / ETH_TX_FRAG_BYTES);
    s->next_frag = 0;

    eth_tx_head = (uint8_t)((eth_tx_head + 1u) % ETH_TX_SLOTS);
    eth_tx_count++;

    /* Send the first fragment inline so a single-fragment frame (IMU/env, the
     * latency-sensitive ones) still leaves immediately and pays nothing for
     * the pacer. */
    eth_tx_emit_one();
    return true;
}

uint32_t ETH_TxDroppedFrames(void) { return eth_tx_dropped; }

bool ETH_HasTarget(void)
{
    return target_ip.addr != 0;
}
