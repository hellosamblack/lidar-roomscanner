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

/* Stream registry — see roomscanner/docs/protocol.md. 1-6 reserved for Phase 2. */
#define RS_STREAM_DEPTH_ZAPC  (1u)
#define RS_STREAM_AMBIENT     (2u)
#define RS_STREAM_AMPLITUDE   (3u)
#define RS_STREAM_CONFIDENCE  (4u)
#define RS_STREAM_REFLECTANCE (5u)
#define RS_STREAM_STATUS      (6u)

/* EVENT (RS_FRAME_EVENT) payload: u32 code, u32 detail, optional ASCII message. */
#define RS_EVT_SENSOR_INIT_FAIL    (1u)
#define RS_EVT_TRIGGER_TIMEOUT     (2u)
#define RS_EVT_DMA_TIMEOUT         (3u)
#define RS_EVT_SENSOR_ERROR_STATUS (4u)
#define RS_EVT_TX_OVERFLOW         (5u)

void rs_put_u32(uint8_t *p, uint32_t v);

/* IEEE 802.3 / zlib CRC-32. Chain calls by passing the previous return as crc (start 0). */
uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len);

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len);

#endif /* RS_PROTOCOL_H */
