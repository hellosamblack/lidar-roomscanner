/* Wire protocol v1 — single source of truth: roomscanner/docs/protocol.md.
 * HAL-free on purpose: host-compilable for cross-checking against the Python codec. */
#ifndef RS_PROTOCOL_H
#define RS_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#define RS_PROTO_VERSION     (1u)
#define RS_HEADER_SIZE       (32u)
#define RS_FRAME_DATA        (1u)
#define RS_FRAME_EVENT       (2u)
#define RS_STREAM_DEPTH_ZF32 (0u)
#define RS_FLAG_DROPPED      (0x01u)

void rs_put_u32(uint8_t *p, uint32_t v);

/* IEEE 802.3 / zlib CRC-32. Chain calls by passing the previous return as crc (start 0). */
uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len);

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len);

#endif /* RS_PROTOCOL_H */
