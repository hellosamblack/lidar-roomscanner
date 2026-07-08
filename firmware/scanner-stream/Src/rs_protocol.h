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
#define RS_FRAME_COMMAND     (3u)
#define RS_FRAME_ACK         (4u)
#define RS_STREAM_DEPTH_ZF32 (0u)
#define RS_FLAG_DROPPED      (0x01u)

/* Stream registry — see roomscanner/docs/protocol.md. 1-6 reserved (Phase 2+); 7-8 live. */
#define RS_STREAM_DEPTH_ZAPC  (1u)
#define RS_STREAM_AMBIENT     (2u)
#define RS_STREAM_AMPLITUDE   (3u)
#define RS_STREAM_CONFIDENCE  (4u)
#define RS_STREAM_REFLECTANCE (5u)
#define RS_STREAM_STATUS      (6u)
#define RS_STREAM_RAW_3DMD    (7u) /* opaque vendor raw frame (transform input) */
#define RS_STREAM_CALIB       (8u) /* per-device calibration blob */
#define RS_RAW_3DMD_SIZE_BIN2 (14842u)
#define RS_CALIB_SIZE         (2332u)

/* EVENT (RS_FRAME_EVENT) payload: u32 code, u32 detail, optional ASCII message. */
#define RS_EVT_SENSOR_INIT_FAIL    (1u)
#define RS_EVT_TRIGGER_TIMEOUT     (2u)
#define RS_EVT_DMA_TIMEOUT         (3u)
#define RS_EVT_SENSOR_ERROR_STATUS (4u)
#define RS_EVT_TX_OVERFLOW         (5u)

/* COMMAND (RS_FRAME_COMMAND) payload: u32 cmd, u32 param (LE). */
#define RS_CMD_PING                (1u)
#define RS_CMD_SEND_CALIB          (2u)
#define RS_CMD_SET_USECASE         (3u)
#define RS_CMD_SET_FRAME_PERIOD_US (4u)
#define RS_CMD_SET_EXPOSURE_MS     (5u)
#define RS_CMD_REINIT              (6u)

/* ACK (RS_FRAME_ACK) payload: u32 cmd, u32 result, u32 applied (LE). */
#define RS_RESULT_OK               (0u)
#define RS_RESULT_UNKNOWN_CMD      (1u)
#define RS_RESULT_BAD_PARAM        (2u)
#define RS_RESULT_REJECTED_BINNING (3u)
#define RS_RESULT_SENSOR_ERROR     (4u)
#define RS_RESULT_BUSY             (5u)

void rs_put_u32(uint8_t *p, uint32_t v);

/* IEEE 802.3 / zlib CRC-32. Chain calls by passing the previous return as crc (start 0). */
uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len);

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len);

#endif /* RS_PROTOCOL_H */
