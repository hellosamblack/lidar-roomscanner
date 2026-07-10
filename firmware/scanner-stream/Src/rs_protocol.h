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

/* Stream registry — see roomscanner/docs/protocol.md. 1-6 reserved (Phase 2+); 7-8 + 9-10 live. */
#define RS_STREAM_DEPTH_ZAPC  (1u)
#define RS_STREAM_AMBIENT     (2u)
#define RS_STREAM_AMPLITUDE   (3u)
#define RS_STREAM_CONFIDENCE  (4u)
#define RS_STREAM_REFLECTANCE (5u)
#define RS_STREAM_STATUS      (6u)
#define RS_STREAM_RAW_3DMD    (7u) /* opaque vendor raw frame (transform input) */
#define RS_STREAM_CALIB       (8u) /* per-device calibration blob */
#define RS_STREAM_IMU_QUAT    (9u)  /* 4x float32 [w,x,y,z] quaternion, LSM body frame */
#define RS_STREAM_ENV         (10u) /* f32 pressure(Pa) + 3xf32 mag(uT) + f32 temp(C) */
#define RS_RAW_3DMD_SIZE_BIN2 (14842u)
#define RS_CALIB_SIZE         (2332u)
#define RS_IMU_QUAT_SIZE      (16u)
#define RS_ENV_SIZE           (20u)

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

/* Wire size of one inbound COMMAND frame: RS_HEADER_SIZE (32) + cmd/param (8) + CRC32 (4). */
#define RS_CMD_PAYLOAD_LEN (8u)
#define RS_CMD_FRAME_SIZE  (RS_HEADER_SIZE + RS_CMD_PAYLOAD_LEN + 4u)

void rs_put_u32(uint8_t *p, uint32_t v);

/* IEEE 802.3 / zlib CRC-32. Chain calls by passing the previous return as crc (start 0). */
uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len);

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len);

/* rs_parse_command: pull one COMMAND frame out of the front of an accumulation buffer.
 * Pure buffer parsing, no I/O -- the caller owns RX (tud_cdc_read or anything else) and
 * buffer accumulation/compaction; this function only decides how many bytes at buf[0..len)
 * can be discarded and whether a valid command was found among them.
 *
 * Scans for the 4-byte "RSCN" magic starting at buf[0]. Three outcomes:
 *
 *   - No magic candidate anywhere in buf: returns -(int32_t)len, i.e. "drop everything"
 *     (up to the last 3 bytes, which are kept in case they are the start of a magic that
 *     completes with the next RX chunk -- so the true drop count can be len-3 in that case).
 *     cmd, param, and token are left untouched.
 *
 *   - A magic candidate is found at offset k, but fewer than RS_CMD_FRAME_SIZE (44) bytes
 *     remain from k to the end of buf: returns -(int32_t)k. If k == 0 this is 0, meaning
 *     "consume nothing, just wait for more RX bytes before calling again" -- the candidate
 *     is still pending, not yet known good or bad. If k > 0, the garbage strictly before
 *     the candidate is dropped while the candidate itself is kept for next call.
 *
 *   - A magic candidate at offset k with >= RS_CMD_FRAME_SIZE bytes available: the header
 *     (version == RS_PROTO_VERSION, frame_type == RS_FRAME_COMMAND, payload_len ==
 *     RS_CMD_PAYLOAD_LEN -- version rejection mirrors the host decoder's symmetric
 *     behavior) and the CRC-32 over bytes [k, k+40) against the trailing u32 LE at
 *     [k+40, k+44) are validated.
 *       - All pass: decodes cmd/param/token (LE) and returns k + RS_CMD_FRAME_SIZE (the
 *         garbage prefix, if any, plus the consumed frame) -- a positive return.
 *       - Any fails: the candidate was a false positive (e.g. "RSCN" bytes inside a
 *         payload) or an incompatible version. Returns -(int32_t)(k + 1): drop the
 *         prefix plus one byte of the candidate, so the next call rescans starting one
 *         byte later (in case the real magic starts there).
 *
 * Caller convention: a POSITIVE return is a decoded command -- consume that many bytes
 * from the front of the accumulation buffer, dispatch the command, no counting. A NEGATIVE
 * return means bytes should still be dropped (consume -return bytes) but nothing was
 * decoded -- the caller should treat this as one resync/malformed event for its counters
 * (regardless of how many bytes were dropped). A ZERO return means: do not drop anything,
 * a candidate may still complete once more RX bytes arrive -- do not count this as
 * malformed. */
int32_t rs_parse_command(const uint8_t *buf, size_t len, uint32_t *cmd, uint32_t *param,
                         uint32_t *token);

#endif /* RS_PROTOCOL_H */
