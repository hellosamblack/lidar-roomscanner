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

uint32_t rs_crc32(uint32_t crc, const uint8_t *data, size_t len) {
    crc = ~crc;
    while (len--) {
        crc ^= *data++;
        for (int k = 0; k < 8; k++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
        }
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
