/* LSM6DSV16X driver — see rs_lsm.h.
 *
 * Bring-up is incremental: SFLP orientation first (validated on the bench), then the
 * I2C sensor-hub environmental slaves. SHUB config is compiled in but gated by
 * RS_LSM_ENABLE_SHUB so orientation can be validated in isolation. */
#include "rs_lsm.h"

#include <math.h>
#include <string.h>

#include "lsm6dsv16x_reg.h"
#include "stm32h5xx_hal.h"

#define RS_LSM_ENABLE_SHUB (0)   /* flip to 1 once SFLP orientation is validated */

#define LSM_ADDR 0x50u           /* LSM6DSV16X dynamic I3C address (rs_assign_dynamic_addresses) */

extern I3C_HandleTypeDef hi3c1;

/* ---- native-I3C register transport (ctx read/write) --------------------------------
 * The LSM joined the bus via ENTDAA as a genuine I3C target, so register access uses
 * I3C PRIVATE transfers (not legacy-I2C), mirroring the ToF ULD's _i3c_read pattern and
 * the iks4a1_i3c_probe helper: write the 1-byte register pointer (RESTART), then the
 * data (STOP). */
static int32_t lsm_i3c_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    (void)handle;
    uint32_t cbw[1], sbw[1];
    I3C_PrivateTypeDef pd_w = { LSM_ADDR, { &reg, 1 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE };
    I3C_XferTypeDef ctx_w = { { &cbw[0], 1 }, { &sbw[0], 1 }, { &reg, 1 }, { NULL, 0 } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_w, &ctx_w, 1, I3C_PRIVATE_WITHOUT_ARB_RESTART) != HAL_OK) {
        return -1;
    }
    if (HAL_I3C_Ctrl_Transmit(&hi3c1, &ctx_w, 100) != HAL_OK) {
        return -1;
    }
    while ((HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY) && (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_LISTEN)) {
    }
    uint32_t cbr[1], sbr[1];
    I3C_PrivateTypeDef pd_r = { LSM_ADDR, { NULL, 0 }, { data, len }, HAL_I3C_DIRECTION_READ };
    I3C_XferTypeDef ctx_r = { { &cbr[0], 1 }, { &sbr[0], 1 }, { NULL, 0 }, { data, len } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_r, &ctx_r, 1, I3C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
        return -1;
    }
    if (HAL_I3C_Ctrl_Receive(&hi3c1, &ctx_r, 100) != HAL_OK) {
        return -1;
    }
    return 0;
}

static int32_t lsm_i3c_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len) {
    (void)handle;
    uint8_t buf[32];
    if (len > sizeof(buf) - 1u) {
        return -1;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    uint16_t total = (uint16_t)(len + 1u);
    uint32_t cbw[1], sbw[1];
    I3C_PrivateTypeDef pd_w = { LSM_ADDR, { buf, total }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE };
    I3C_XferTypeDef ctx_w = { { &cbw[0], 1 }, { &sbw[0], 1 }, { buf, total }, { NULL, 0 } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_w, &ctx_w, 1, I3C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
        return -1;
    }
    if (HAL_I3C_Ctrl_Transmit(&hi3c1, &ctx_w, 100) != HAL_OK) {
        return -1;
    }
    return 0;
}

static void lsm_mdelay(uint32_t ms) {
    HAL_Delay(ms);
}

static lsm6dsv16x_ctx_t g_ctx = {
    .write_reg = lsm_i3c_write,
    .read_reg = lsm_i3c_read,
    .mdelay = lsm_mdelay,
    .handle = NULL,
};

/* ---- IEEE-754 half-precision -> float (SFLP game-rotation-vector components) -------- */
static float half_to_float(uint16_t h) {
    uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
    uint32_t exp = (h >> 10) & 0x1Fu;
    uint32_t mant = h & 0x3FFu;
    uint32_t f;
    if (exp == 0u) {
        if (mant == 0u) {
            f = sign;
        } else {
            exp = 127u - 15u + 1u;
            while ((mant & 0x400u) == 0u) {
                mant <<= 1;
                exp--;
            }
            mant &= 0x3FFu;
            f = sign | (exp << 23) | (mant << 13);
        }
    } else if (exp == 0x1Fu) {
        f = sign | 0x7F800000u | (mant << 13);
    } else {
        f = sign | ((exp - 15u + 127u) << 23) | (mant << 13);
    }
    float out;
    memcpy(&out, &f, sizeof(out));
    return out;
}

/* Reconstruct the full quaternion from the SFLP game-rotation-vector's 3 stored
 * components (x, y, z as little-endian fp16). w = sqrt(max(0, 1 - x^2 - y^2 - z^2)). */
static void sflp_word_to_quat(const uint8_t data[6], float quat_wxyz[4]) {
    float x = half_to_float((uint16_t)(data[0] | (data[1] << 8)));
    float y = half_to_float((uint16_t)(data[2] | (data[3] << 8)));
    float z = half_to_float((uint16_t)(data[4] | (data[5] << 8)));
    float sumsq = x * x + y * y + z * z;
    float w = (sumsq < 1.0f) ? sqrtf(1.0f - sumsq) : 0.0f;
    quat_wxyz[0] = w;
    quat_wxyz[1] = x;
    quat_wxyz[2] = y;
    quat_wxyz[3] = z;
}

int rs_lsm_init(void) {
    uint8_t whoami = 0;
    if (lsm6dsv16x_device_id_get(&g_ctx, &whoami) != 0 || whoami != LSM6DSV16X_ID) {
        return -1;
    }

    /* NB: no GLOBAL_RST here -- a software reset drops the I3C dynamic address the
     * controller assigned via ENTDAA (the device would fall off 0x50), so we configure
     * from the power-on-reset register defaults instead. */
    lsm6dsv16x_block_data_update_set(&g_ctx, 1);

    /* Accel + gyro must run for SFLP to fuse. */
    if (lsm6dsv16x_xl_data_rate_set(&g_ctx, LSM6DSV16X_ODR_AT_120Hz) != 0 ||
        lsm6dsv16x_gy_data_rate_set(&g_ctx, LSM6DSV16X_ODR_AT_120Hz) != 0) {
        return -3;
    }

    /* SFLP game rotation vector -> FIFO. */
    if (lsm6dsv16x_sflp_data_rate_set(&g_ctx, LSM6DSV16X_SFLP_120Hz) != 0 ||
        lsm6dsv16x_sflp_game_rotation_set(&g_ctx, 1) != 0) {
        return -4;
    }
    lsm6dsv16x_fifo_sflp_raw_t sflp_batch = { .game_rotation = 1, .gravity = 0, .gbias = 0 };
    if (lsm6dsv16x_fifo_sflp_batch_set(&g_ctx, sflp_batch) != 0) {
        return -5;
    }

#if RS_LSM_ENABLE_SHUB
    /* Sensor-hub environmental slaves are configured in a later bring-up step. */
    if (rs_lsm_shub_init() != 0) {
        return -6;
    }
#endif

    /* Continuous FIFO. */
    if (lsm6dsv16x_fifo_mode_set(&g_ctx, LSM6DSV16X_STREAM_MODE) != 0) {
        return -7;
    }
    return 0;
}

int rs_lsm_read_latest(rs_lsm_sample_t *out) {
    out->have_quat = 0;
    out->have_env = 0;

    lsm6dsv16x_fifo_status_t status;
    if (lsm6dsv16x_fifo_status_get(&g_ctx, &status) != 0) {
        return -1;
    }
    uint16_t level = status.fifo_level;
    for (uint16_t i = 0; i < level; i++) {
        lsm6dsv16x_fifo_out_raw_t word;
        if (lsm6dsv16x_fifo_out_raw_get(&g_ctx, &word) != 0) {
            break;
        }
        switch (word.tag) {
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
            sflp_word_to_quat(word.data, out->quat);
            out->have_quat = 1;
            break;
#if RS_LSM_ENABLE_SHUB
        case LSM6DSV16X_SENSORHUB_SLAVE0_TAG:   /* LPS22DF pressure */
        case LSM6DSV16X_SENSORHUB_SLAVE1_TAG:   /* LIS2MDL mag */
        case LSM6DSV16X_SENSORHUB_SLAVE2_TAG:   /* STTS22H temp */
            rs_lsm_shub_demux(&word, out);
            break;
#endif
        default:
            break;
        }
    }
    return (out->have_quat || out->have_env) ? 0 : -1;
}
