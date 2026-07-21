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

#endif
