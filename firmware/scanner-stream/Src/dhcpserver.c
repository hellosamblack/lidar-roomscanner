#include "dhcpserver.h"
#include "lwip/udp.h"
#include "lwip/pbuf.h"
#include "lwip/netif.h"
#include "lwip/def.h"
#include <string.h>

#define DHCP_PORT_SERVER 67
#define DHCP_PORT_CLIENT 68

#define DHCP_MSG_DISCOVER 1
#define DHCP_MSG_OFFER    2
#define DHCP_MSG_REQUEST  3
#define DHCP_MSG_ACK      5

#define DHCP_OPTION_MSG_TYPE   53
#define DHCP_OPTION_SERVER_ID  54
#define DHCP_OPTION_SUBNET_MASK 1
#define DHCP_OPTION_ROUTER     3
#define DHCP_OPTION_LEASE_TIME 51
#define DHCP_OPTION_END        255

struct dhcp_msg {
    uint8_t op, htype, hlen, hops;
    uint8_t xid[4];
    uint16_t secs, flags;
    uint8_t ciaddr[4];
    uint8_t yiaddr[4];
    uint8_t siaddr[4];
    uint8_t giaddr[4];
    uint8_t chaddr[16];
    uint8_t sname[64];
    uint8_t file[128];
    uint32_t cookie;
    uint8_t options[308];
} __attribute__((packed));

static struct udp_pcb *dhcps_pcb;

static void dhcps_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
    if (p == NULL || p->len < sizeof(struct dhcp_msg) - 308) {
        if (p) pbuf_free(p);
        return;
    }

    struct dhcp_msg *msg = (struct dhcp_msg *)p->payload;
    if (msg->op != 1) { // BOOTREQUEST
        pbuf_free(p);
        return;
    }

    uint8_t msg_type = 0;
    uint8_t *opt = msg->options;
    uint16_t opt_len = p->tot_len - (sizeof(struct dhcp_msg) - 308);
    for (uint16_t i = 0; i < opt_len && opt[i] != DHCP_OPTION_END;) {
        if (opt[i] == DHCP_OPTION_MSG_TYPE && opt[i+1] == 1) {
            msg_type = opt[i+2];
            break;
        }
        i += 2 + opt[i+1];
    }

    if (msg_type == DHCP_MSG_DISCOVER || msg_type == DHCP_MSG_REQUEST) {
        struct pbuf *out_p = pbuf_alloc(PBUF_TRANSPORT, sizeof(struct dhcp_msg), PBUF_RAM);
        if (out_p) {
            struct dhcp_msg *out_msg = (struct dhcp_msg *)out_p->payload;
            memset(out_msg, 0, sizeof(struct dhcp_msg));
            out_msg->op = 2; // BOOTREPLY
            out_msg->htype = msg->htype;
            out_msg->hlen = msg->hlen;
            memcpy(out_msg->xid, msg->xid, 4);
            out_msg->flags = msg->flags;
            
            // Assign 172.31.253.2
            out_msg->yiaddr[0] = 172;
            out_msg->yiaddr[1] = 31;
            out_msg->yiaddr[2] = 253;
            out_msg->yiaddr[3] = 2;
            
            // Server IP 172.31.253.1
            out_msg->siaddr[0] = 172;
            out_msg->siaddr[1] = 31;
            out_msg->siaddr[2] = 253;
            out_msg->siaddr[3] = 1;
            
            memcpy(out_msg->chaddr, msg->chaddr, 16);
            out_msg->cookie = lwip_htonl(0x63825363);

            uint8_t *out_opt = out_msg->options;
            int idx = 0;
            
            out_opt[idx++] = DHCP_OPTION_MSG_TYPE;
            out_opt[idx++] = 1;
            out_opt[idx++] = (msg_type == DHCP_MSG_DISCOVER) ? DHCP_MSG_OFFER : DHCP_MSG_ACK;

            out_opt[idx++] = DHCP_OPTION_SERVER_ID;
            out_opt[idx++] = 4;
            out_opt[idx++] = 172; out_opt[idx++] = 31; out_opt[idx++] = 253; out_opt[idx++] = 1;

            out_opt[idx++] = DHCP_OPTION_SUBNET_MASK;
            out_opt[idx++] = 4;
            out_opt[idx++] = 255; out_opt[idx++] = 255; out_opt[idx++] = 255; out_opt[idx++] = 0;

            out_opt[idx++] = DHCP_OPTION_ROUTER;
            out_opt[idx++] = 4;
            out_opt[idx++] = 172; out_opt[idx++] = 31; out_opt[idx++] = 253; out_opt[idx++] = 1;

            out_opt[idx++] = DHCP_OPTION_LEASE_TIME;
            out_opt[idx++] = 4;
            out_opt[idx++] = 0; out_opt[idx++] = 1; out_opt[idx++] = 0x51; out_opt[idx++] = 0x80; // 86400

            out_opt[idx++] = DHCP_OPTION_END;

            udp_sendto(pcb, out_p, IP_ADDR_BROADCAST, DHCP_PORT_CLIENT);
            pbuf_free(out_p);
        }
    }

    pbuf_free(p);
}

void dhcps_init(void)
{
    dhcps_pcb = udp_new();
    if (dhcps_pcb) {
        udp_bind(dhcps_pcb, IP_ADDR_ANY, DHCP_PORT_SERVER);
        udp_recv(dhcps_pcb, dhcps_recv, NULL);
    }
}
