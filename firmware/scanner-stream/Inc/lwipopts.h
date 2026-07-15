#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

/* NO_SYS = 1: bare metal, no OS */
#define NO_SYS 1
#define LWIP_NETCONN 0
#define LWIP_SOCKET 0
#define SYS_LIGHTWEIGHT_PROT 0

/* Core IPv4/UDP */
#define LWIP_IPV4 1
#define LWIP_IPV6 0
#define LWIP_TCP 0
#define LWIP_UDP 1

/* DHCP client */
#define LWIP_DHCP 1
#define LWIP_DNS 0
#define LWIP_IGMP 0
#define LWIP_NETIF_HOSTNAME 1

/* Memory configuration for fast UDP streaming */
#define MEM_ALIGNMENT 4
#define MEM_SIZE (16 * 1024)

#define MEMP_NUM_PBUF 16
#define MEMP_NUM_UDP_PCB 4
#define PBUF_POOL_SIZE 16
#define PBUF_POOL_BUFSIZE 1524

/* Enable Ethernet/ARP */
#define LWIP_ARP 1
#define LWIP_ETHERNET 1

/* Callbacks */
#define LWIP_NETIF_LINK_CALLBACK 1
#define LWIP_NETIF_STATUS_CALLBACK 1

/* Checksums */
#define CHECKSUM_BY_HARDWARE 0
#define CHECKSUM_GEN_IP 1
#define CHECKSUM_GEN_UDP 1
#define CHECKSUM_GEN_TCP 0
#define CHECKSUM_GEN_ICMP 1
#define CHECKSUM_CHECK_IP 1
#define CHECKSUM_CHECK_UDP 1
#define CHECKSUM_CHECK_TCP 0
#define CHECKSUM_CHECK_ICMP 1

#endif /* __LWIPOPTS_H__ */
