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

void ETH_Init(void)
{
    lwip_init();
    Netif_Config();
    mdns_resp_init();
    upcb = udp_new();
}

void ETH_Process(void)
{
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
    }

    if (eth_link_up) {
        if (dhcp_state == DHCP_STATE_CLIENT_WAITING) {
            if (gnetif.ip_addr.addr != 0) {
                dhcp_state = DHCP_STATE_CLIENT_BOUND;
                err_t m1 = mdns_resp_add_netif(&gnetif, "roomscanner", 3600);
                s8_t m2 = mdns_resp_add_service(&gnetif, "roomscanner", "_roomscan", DNSSD_PROTO_UDP, 5000, 3600, NULL, NULL);
                mdns_resp_netif_settings_changed(&gnetif);
                printf("[ETH] DHCP Client Bound: IP %s, mdns_add=%d, srv=%d\n", ip4addr_ntoa(netif_ip4_addr(&gnetif)), m1, m2);
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
                mdns_resp_add_netif(&gnetif, "roomscanner", 3600);
                mdns_resp_add_service(&gnetif, "roomscanner", "_roomscan", DNSSD_PROTO_UDP, 5000, 3600, NULL, NULL);
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

bool ETH_SendFrame_Gather(const uint8_t *hdr, uint32_t hdr_len, const uint8_t *payload, uint32_t payload_len, const uint8_t *tail, uint32_t tail_len)
{
    if (!eth_link_up || !upcb) return false;

    ip_addr_t target_ip;
    IP4_ADDR(&target_ip, 255, 255, 255, 255);

    uint32_t total_len = hdr_len + payload_len + tail_len;
    uint32_t offset = 0;
    uint8_t frag_idx = 0;
    uint8_t total_frags = (total_len + 1400 - 1) / 1400;
    
    while (offset < total_len)
    {
        uint32_t chunk = total_len - offset;
        if (chunk > 1400) chunk = 1400;

        struct pbuf *p = pbuf_alloc(PBUF_TRANSPORT, chunk + 6, PBUF_RAM);
        if (!p) return false;

        uint8_t *p_out = (uint8_t *)p->payload;
        
        p_out[0] = (uint8_t)(frame_seq_num & 0xFF);
        p_out[1] = (uint8_t)((frame_seq_num >> 8) & 0xFF);
        p_out[2] = (uint8_t)((frame_seq_num >> 16) & 0xFF);
        p_out[3] = (uint8_t)((frame_seq_num >> 24) & 0xFF);
        p_out[4] = frag_idx;
        p_out[5] = total_frags;
        
        uint32_t out_idx = 6;
        uint32_t remain = chunk;
        
        /* Copy from hdr, payload, tail based on offset */
        if (offset < hdr_len && remain > 0) {
            uint32_t copy_len = hdr_len - offset;
            if (copy_len > remain) copy_len = remain;
            memcpy(&p_out[out_idx], &hdr[offset], copy_len);
            out_idx += copy_len;
            remain -= copy_len;
            offset += copy_len;
        }
        
        uint32_t payload_offset = offset > hdr_len ? offset - hdr_len : 0;
        if (payload_offset < payload_len && remain > 0) {
            uint32_t copy_len = payload_len - payload_offset;
            if (copy_len > remain) copy_len = remain;
            memcpy(&p_out[out_idx], &payload[payload_offset], copy_len);
            out_idx += copy_len;
            remain -= copy_len;
            offset += copy_len;
        }
        
        uint32_t tail_offset = offset > (hdr_len + payload_len) ? offset - (hdr_len + payload_len) : 0;
        if (tail_offset < tail_len && remain > 0) {
            uint32_t copy_len = tail_len - tail_offset;
            if (copy_len > remain) copy_len = remain;
            memcpy(&p_out[out_idx], &tail[tail_offset], copy_len);
            out_idx += copy_len;
            remain -= copy_len;
            offset += copy_len;
        }

        if (udp_sendto(upcb, p, &target_ip, 5000) != ERR_OK)
        {
            pbuf_free(p);
            return false;
        }

        pbuf_free(p);
        frag_idx++;
    }
    
    frame_seq_num++;
    return true;
}
