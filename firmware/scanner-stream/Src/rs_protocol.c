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

static uint16_t get_u16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

int32_t rs_parse_command(const uint8_t *buf, size_t len, rs_parsed_command_t *out) {
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
    if (remaining < RS_HEADER_SIZE) {
        /* candidate pending -- not even a full header yet, so payload_len (which decides
         * the real total length) cannot be read */
        return -(int32_t)k;
    }

    const uint8_t *p = buf + k;
    uint8_t version = p[4];
    uint8_t frame_type = p[5];
    uint32_t payload_len = get_u32(p + 24);

    uint32_t frame_total;
    rs_parsed_cmd_kind_t kind;
    if (payload_len == RS_CMD_PAYLOAD_LEN) {
        frame_total = RS_CMD_FRAME_SIZE_LEGACY;
        kind = RS_PARSED_CMD_LEGACY;
    } else if (payload_len == RS_CMD_MANUAL_PAYLOAD_LEN) {
        frame_total = RS_CMD_FRAME_SIZE_MANUAL;
        kind = RS_PARSED_CMD_MANUAL;
    } else {
        /* neither known COMMAND payload shape: false-positive magic (or an incompatible
         * payload_len) -- resync one byte in, same as a version/frame_type mismatch */
        return -(int32_t)(k + 1u);
    }
    if (version != RS_PROTO_VERSION || frame_type != RS_FRAME_COMMAND) {
        return -(int32_t)(k + 1u); /* false-positive magic: resync one byte in */
    }

    if (remaining < frame_total) {
        /* header decoded and shape known, but the body/CRC hasn't fully arrived yet */
        return -(int32_t)k;
    }

    uint32_t crc_calc = rs_crc32(0u, p, RS_HEADER_SIZE + payload_len);
    uint32_t crc_wire = get_u32(p + RS_HEADER_SIZE + payload_len);
    if (crc_calc != crc_wire) {
        return -(int32_t)(k + 1u);
    }

    out->version = version;
    out->token = get_u32(p + 8); /* header seq field: host-chosen token */
    out->cmd = get_u32(p + RS_HEADER_SIZE);
    out->payload_len = payload_len;
    out->kind = kind;
    if (kind == RS_PARSED_CMD_MANUAL) {
        /* cmd(4) + ranging_mode(1) + frame_period_us(4) + exposure_ms(2) + power_mode(1),
         * no padding on the wire -- offsets are cumulative from RS_HEADER_SIZE. */
        out->u.manual.ranging_mode = p[RS_HEADER_SIZE + 4u];
        out->u.manual.frame_period_us = get_u32(p + RS_HEADER_SIZE + 5u);
        out->u.manual.exposure_ms = get_u16(p + RS_HEADER_SIZE + 9u);
        out->u.manual.power_mode = p[RS_HEADER_SIZE + 11u];
    } else {
        out->u.legacy.param = get_u32(p + RS_HEADER_SIZE + 4u);
    }
    return (int32_t)(k + frame_total);
}
