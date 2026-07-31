#include "rs_protocol.h"

static void put_u16(uint8_t *p, uint16_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

void rs_put_u32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void put_u64(uint8_t *p, uint64_t v) {
    rs_put_u32(p, (uint32_t)v);
    rs_put_u32(p + 4, (uint32_t)(v >> 32));
}

/* CRC-32 (IEEE, reflected, poly 0xEDB88320) -- NIBBLE table, 16 entries / 64 B.
 *
 * This was a table-free bit-serial loop: 8 shift/xor iterations per byte, run
 * over header+payload for EVERY frame. At 14,874 bytes/frame that is ~119,000
 * inner iterations, an estimated 2.4-3.6 ms of the 33 ms frame period -- the
 * single largest piece of arithmetic in the acquisition loop, and pure
 * overhead. Two table lookups per byte replace the eight iterations.
 *
 * A nibble table rather than the usual 256-entry byte table because 64 bytes is
 * small enough to verify by eye, and the byte table's extra ~2x buys speed the
 * loop does not need. Entries are generated, not transcribed:
 *     t[n] = n folded 4 times through (c>>1) ^ (poly if c&1)
 *
 * BIT-EXACT with the previous implementation by construction -- same
 * polynomial, same reflection, same pre/post inversion. That is not taken on
 * faith: the host verifies this CRC on every frame it receives, so any
 * divergence shows up immediately as a 100% CRC-failure rate rather than as a
 * subtle drift.
 */
static const uint32_t rs_crc32_nibble[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
    0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
    0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        crc = (crc >> 4) ^ rs_crc32_nibble[crc & 0x0Fu];
        crc = (crc >> 4) ^ rs_crc32_nibble[crc & 0x0Fu];
    }
    return ~crc;
}

void rs_write_header(uint8_t out[RS_HEADER_SIZE], uint8_t frame_type, uint8_t stream_id,
                     uint8_t flags, uint32_t seq, uint64_t t_us, uint16_t width,
                     uint16_t height, uint32_t payload_len) {
    out[0] = 'R'; out[1] = 'S'; out[2] = 'C'; out[3] = 'N';
    out[4] = RS_PROTO_VERSION;
    out[5] = frame_type;
    out[6] = stream_id;
    out[7] = flags;
    rs_put_u32(out + 8, seq);
    put_u64(out + 12, t_us);
    put_u16(out + 20, width);
    put_u16(out + 22, height);
    rs_put_u32(out + 24, payload_len);
    rs_put_u32(out + 28, 0u); /* reserved */
}

static uint32_t get_u32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

int32_t rs_parse_command(const uint8_t *buf, size_t len, uint32_t *cmd, uint32_t *param,
                         uint32_t *token) {
    size_t k;
    int found = 0;

    /* magic can only start where 4 bytes are available */
    for (k = 0; k + 4 <= len; k++) {
        if (buf[k] == 'R' && buf[k + 1] == 'S' && buf[k + 2] == 'C' && buf[k + 3] == 'N') {
            found = 1;
            break;
        }
    }
    if (!found) {
        /* keep the last up-to-3 bytes: they could be the start of a magic that
         * completes once more RX bytes are appended */
        size_t keep = (len < 3u) ? len : 3u;
        return -(int32_t)(len - keep);
    }

    size_t remaining = len - k;
    if (remaining < RS_CMD_FRAME_SIZE) {
        /* candidate pending -- not enough bytes yet to validate it */
        return -(int32_t)k;
    }

    const uint8_t *p = buf + k;
    uint8_t version = p[4];
    uint8_t frame_type = p[5];
    uint32_t payload_len = get_u32(p + 24);
    if (version != RS_PROTO_VERSION || frame_type != RS_FRAME_COMMAND ||
        payload_len != RS_CMD_PAYLOAD_LEN) {
        return -(int32_t)(k + 1u); /* false-positive magic: resync one byte in */
    }

    uint32_t crc_calc = rs_crc32(0u, p, RS_HEADER_SIZE + RS_CMD_PAYLOAD_LEN);
    uint32_t crc_wire = get_u32(p + RS_HEADER_SIZE + RS_CMD_PAYLOAD_LEN);
    if (crc_calc != crc_wire) {
        return -(int32_t)(k + 1u);
    }

    *cmd = get_u32(p + 32);
    *param = get_u32(p + 36);
    *token = get_u32(p + 8); /* header seq field: host-chosen token */
    return (int32_t)(k + RS_CMD_FRAME_SIZE);
}
