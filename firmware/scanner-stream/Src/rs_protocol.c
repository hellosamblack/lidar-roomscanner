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
