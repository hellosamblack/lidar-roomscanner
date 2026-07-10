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

/* Sensor-hub baro/mag/temp slaves. Gated OFF -- diagnosed but not yet working.
 * Findings (bench, 2026-07-09/10):
 *   - MASTER_CONFIG reads back 0x46 after sh_master_set: MASTER_ON=1 (bit2), AUX_SENS_ON=2
 *     (bits1:0 -> 3 slaves), WRITE_ONCE pending (bit6). So the enable + slave config DO
 *     latch over I3C -- SHUB-bank writes work.
 *   - BUT the master never runs a single cycle: STATUS_MASTER.sens_hub_endop stays 0, no
 *     slave NACK ever, no FIFO SHUB-slave tags. The state machine never STARTS a transaction
 *     -> the XL/GY-DRDY trigger isn't reaching the sensor hub.
 *   - Ruled out: NOT an SFLP conflict (SHUB stays dead with SFLP explicitly disabled);
 *     NOT the write-once init (dead with the init writes removed); NOT enable-not-latching.
 *   - SHUB_PU_EN (MASTER_CONFIG bit3) reads 0 despite sh_master_interface_pull_up_set(1).
 * Next bench steps (owner): scope SENS_SDA/SENS_SCL for any master activity + check board
 * pull-ups; try START_CONFIG/INT2 trigger instead of XL_GY_DRDY; investigate why SHUB_PU_EN
 * won't set; confirm the accel DRDY actually pulses the sensor-hub trigger over I3C.
 * NB: LSM config persists across an MCU -rst (independently powered) -- set states
 * explicitly (see RS_LSM_SFLP_ON), don't rely on POR defaults, when bench-testing. */
#define RS_LSM_ENABLE_SHUB (0)
#define RS_LSM_SFLP_ON (1)  /* SFLP game-rotation-vector; set 0 only to bench SHUB in isolation */

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

uint8_t g_lsm_master_config = 0xFF;   /* diagnostic: MASTER_CONFIG readback after sh_master_set */

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

#if RS_LSM_ENABLE_SHUB
/* Sensor-hub slave map (7-bit addresses; the driver adds the R/W bit):
 *   slot 0 = LPS22DF baro  (0x5C): PRESS_OUT_XL 0x28, 3 bytes, hPa=raw/4096
 *   slot 1 = LIS2MDL mag   (0x1E): OUTX_L_REG   0x68, 6 bytes, gauss=raw*1.5e-3
 *   slot 2 = STTS22H temp  (0x38): TEMP_L_OUT   0x06, 2 bytes, C=raw*0.01 */
static int rs_lsm_shub_init(void) {
    /* One-time slave power-up writes via the write-once channel. Each needs its own
     * enable-cycle-disable so the single DATAWRITE channel fires per slave. */
    static const struct { uint8_t addr, reg, val; } inits[3] = {
        { 0x5C, 0x10, 0x20 },  /* LPS22DF CTRL_REG1: ODR 25 Hz continuous */
        { 0x1E, 0x60, 0x8C },  /* LIS2MDL CFG_REG_A: temp-comp, 100 Hz, continuous */
        { 0x38, 0x04, 0x3C },  /* STTS22H CTRL: free-run + auto-inc + BDU */
    };
    /* Enable the sensor-hub master's internal pull-ups on the aux SENS_I2C bus --
     * without these the master can't drive SDx/SCx and never completes a cycle. */
    lsm6dsv16x_sh_master_interface_pull_up_set(&g_ctx, 1);
    lsm6dsv16x_sh_write_mode_set(&g_ctx, LSM6DSV16X_ONLY_FIRST_CYCLE);
    lsm6dsv16x_sh_syncro_mode_set(&g_ctx, LSM6DSV16X_SH_TRG_XL_GY_DRDY);
    for (int i = 0; i < 3; i++) {
        lsm6dsv16x_sh_cfg_write_t w = { inits[i].addr, inits[i].reg, inits[i].val };
        if (lsm6dsv16x_sh_cfg_write(&g_ctx, &w) != 0) {
            return -1;
        }
        lsm6dsv16x_sh_slave_connected_set(&g_ctx, LSM6DSV16X_SLV_0);
        lsm6dsv16x_sh_master_set(&g_ctx, 1);
        HAL_Delay(20);   /* let the write-once fire on an XL/GY trigger cycle */
        lsm6dsv16x_sh_master_set(&g_ctx, 0);
        HAL_Delay(5);
    }

    /* Configure the three read slaves. */
    lsm6dsv16x_sh_cfg_read_t r0 = { 0x5C, 0x28, 3 };
    lsm6dsv16x_sh_cfg_read_t r1 = { 0x1E, 0x68, 6 };
    lsm6dsv16x_sh_cfg_read_t r2 = { 0x38, 0x06, 2 };
    if (lsm6dsv16x_sh_slv_cfg_read(&g_ctx, 0, &r0) != 0 ||
        lsm6dsv16x_sh_slv_cfg_read(&g_ctx, 1, &r1) != 0 ||
        lsm6dsv16x_sh_slv_cfg_read(&g_ctx, 2, &r2) != 0) {
        return -2;
    }
    lsm6dsv16x_sh_slave_connected_set(&g_ctx, LSM6DSV16X_SLV_0_1_2);
    lsm6dsv16x_sh_data_rate_set(&g_ctx, LSM6DSV16X_SH_60Hz);
    lsm6dsv16x_fifo_sh_batch_slave_set(&g_ctx, 0, 1);
    lsm6dsv16x_fifo_sh_batch_slave_set(&g_ctx, 1, 1);
    lsm6dsv16x_fifo_sh_batch_slave_set(&g_ctx, 2, 1);
    lsm6dsv16x_sh_master_set(&g_ctx, 1);

    /* DIAG: read MASTER_CONFIG back to see whether MASTER_ON (bit2) / AUX_SENS_ON (bits1:0)
     * / SHUB_PU_EN (bit3) actually latched over I3C. 0x00 => SHUB-bank write not landing. */
    lsm6dsv16x_mem_bank_set(&g_ctx, LSM6DSV16X_SENSOR_HUB_MEM_BANK);
    lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_MASTER_CONFIG, &g_lsm_master_config, 1);
    lsm6dsv16x_mem_bank_set(&g_ctx, LSM6DSV16X_MAIN_MEM_BANK);
    return 0;
}

static void rs_lsm_shub_demux(const lsm6dsv16x_fifo_out_raw_t *w, rs_lsm_sample_t *out) {
    switch (w->tag) {
    case LSM6DSV16X_SENSORHUB_SLAVE0_TAG: {  /* LPS22DF pressure, 24-bit LE */
        uint32_t raw = (uint32_t)w->data[0] | ((uint32_t)w->data[1] << 8) |
                       ((uint32_t)w->data[2] << 16);
        out->pressure_pa = (float)raw * (100.0f / 4096.0f);  /* hPa=raw/4096 -> Pa */
        out->have_env = 1;
        break;
    }
    case LSM6DSV16X_SENSORHUB_SLAVE1_TAG: {  /* LIS2MDL mag x,y,z int16 LE */
        for (int i = 0; i < 3; i++) {
            int16_t raw = (int16_t)(w->data[2 * i] | (w->data[2 * i + 1] << 8));
            out->mag_ut[i] = (float)raw * 0.15f;  /* 1.5 mgauss/LSB * 0.1 µT/mgauss */
        }
        out->have_env = 1;
        break;
    }
    case LSM6DSV16X_SENSORHUB_SLAVE2_TAG: {  /* STTS22H temp int16 LE */
        int16_t raw = (int16_t)(w->data[0] | (w->data[1] << 8));
        out->temp_c = (float)raw * 0.01f;
        out->have_env = 1;
        break;
    }
    default:
        break;
    }
}
#endif /* RS_LSM_ENABLE_SHUB */

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
    lsm6dsv16x_xl_mode_set(&g_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    lsm6dsv16x_gy_mode_set(&g_ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);
    if (lsm6dsv16x_xl_data_rate_set(&g_ctx, LSM6DSV16X_ODR_AT_120Hz) != 0 ||
        lsm6dsv16x_gy_data_rate_set(&g_ctx, LSM6DSV16X_ODR_AT_120Hz) != 0) {
        return -3;
    }

    /* SFLP game rotation vector -> FIFO. (LSM config persists across MCU -rst, so
     * explicitly set game_rotation to the desired state rather than skipping the call.) */
    lsm6dsv16x_sflp_data_rate_set(&g_ctx, LSM6DSV16X_SFLP_120Hz);
    lsm6dsv16x_sflp_game_rotation_set(&g_ctx, RS_LSM_SFLP_ON);
    lsm6dsv16x_fifo_sflp_raw_t sflp_batch = { .game_rotation = RS_LSM_SFLP_ON, .gravity = 0, .gbias = 0 };
    lsm6dsv16x_fifo_sflp_batch_set(&g_ctx, sflp_batch);

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

uint16_t g_lsm_tag_hist[32] = { 0 };  /* diagnostic: FIFO tag histogram (bench probe reads it) */

uint8_t rs_lsm_shub_status_raw(void) {
    lsm6dsv16x_status_master_t st = { 0 };
    lsm6dsv16x_sh_status_get(&g_ctx, &st);
    return (uint8_t)(st.sens_hub_endop | (st.slave0_nack << 3) | (st.slave1_nack << 4) |
                     (st.slave2_nack << 5) | (st.slave3_nack << 6) | (st.wr_once_done << 7));
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
        if (word.tag < 32) {
            g_lsm_tag_hist[word.tag]++;
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
