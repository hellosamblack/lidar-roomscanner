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

/* Sensor-hub baro/mag/temp slaves. WORKING as of 2026-07-10 -- verified on-target via
 * CONF_LSM_PROBE (all three slaves reading: P=982 hPa, T=26.6C, mag; shstat=0x01 ENDOP, nack=0).
 * The long "master never cycles" hunt turned out to be TWO things, both outside firmware:
 *   1. The IKS4A1 aux bus was electrically dead. Every register was correct all along
 *      (MASTER_CONFIG=0x46 master-on; IF_CFG=0x40 SHUB_PU_EN=1; SLV0_ADD latched; CTRL7=0x00;
 *      SFLP trigger alive) -- the "no NACK EVER, STATUS_MASTER=0x00" signature meant the master
 *      could never get a free bus. Root cause was the J4/J5 jumpers: the aux bus (SENS_I2C =
 *      HUB1_SDx/SCx) was first shorted to GND (pos 11-12), then shorted to the STM primary bus
 *      (pos 1-2, which loops the LSM's aux-master output back onto its own primary interface).
 *      FIX: J4/J5 = pos 5-6 ONLY (env sensors isolated on the LSM aux master). See
 *      docs/iks4a1-stacking.md "Sensor hub (Mode 2)".
 *   2. The barometer answers at 0x5D (SA0=1) on this board, not 0x5C -- 0x5C NACKed (slave0_nack).
 * NB the old diagnostic that "SHUB_PU_EN reads 0" was a red herring: it read MASTER_CONFIG bit3
 * (=not_used0); SHUB_PU_EN is IF_CFG bit6 and reads 1. The CTRL7-clear + RST_MASTER_REGS pulse
 * below are kept as cheap defensive hygiene (this device is never software-reset, so it can carry
 * stale state); they were not the fix.
 * NB: LSM config persists across an MCU -rst (independently powered) -- set states explicitly
 * (see RS_LSM_SFLP_ON), don't rely on POR defaults. */
#define RS_LSM_ENABLE_SHUB (1)  /* full stack: ToF + SFLP orientation + env hub (J4/J5=5-6, baro@0x5D) */
#define RS_LSM_SFLP_ON (1)  /* SFLP game-rotation-vector; set 0 only to isolate SHUB */

/* Orientation-path tuning. The accel+gyro feed SFLP, whose quaternion is the SLAM rotation
 * prior; this rig is NOT power-constrained, so favour rate + range over current draw.
 * Rationale in docs/iks4a1-stacking.md "LSM6DSV16X tuning". All safe for the shipping quat
 * output (unitless), independent of the SHUB bring-up above. */
#define RS_LSM_XL_GY_ODR    LSM6DSV16X_ODR_AT_480Hz  /* was 120Hz; also the SFLP trigger rate */
#define RS_LSM_SFLP_ODR     LSM6DSV16X_SFLP_480Hz     /* was 120Hz; orientation-prior rate (max) */
#define RS_LSM_XL_FS        LSM6DSV16X_4g             /* was ±2g POR; headroom vs handheld-shake clip */
#define RS_LSM_GY_FS        LSM6DSV16X_500dps         /* was ±250dps POR; wrist-flick headroom */
/* Batch SFLP gravity + gyro-bias vectors to FIFO for host observability. Staged OFF: enabling
 * adds GRAVITY(0x17)/GBIAS(0x16) FIFO tags that the host stream layer must demux first. The
 * game-rotation vector is already internally bias-corrected regardless of this flag. */
#define RS_LSM_SFLP_BATCH_AUX (0)

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

uint8_t g_lsm_master_config = 0xFF;   /* diagnostic: MASTER_CONFIG (SHUB bank) readback after sh_master_set */
uint8_t g_lsm_if_cfg = 0xFF;          /* diagnostic: IF_CFG (0x03) readback -- SHUB_PU_EN=bit6, SDA_PU_EN=bit7 */
uint8_t g_lsm_slv0_add = 0xFF;        /* diagnostic: SLV0_ADD readback -- confirms slave-cfg latched */
uint8_t g_lsm_ctrl7_pre = 0xFF;       /* diagnostic: CTRL7 as found -- AH_QVAR_EN=bit7 steals SDx/SCx pins */

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
 *   slot 0 = LPS22DF baro  (0x5D): PRESS_OUT_XL 0x28, 3 bytes, hPa=raw/4096 (SA0=1 on this IKS4A1; 0x5C NACKs)
 *   slot 1 = LIS2MDL mag   (0x1E): OUTX_L_REG   0x68, 6 bytes, gauss=raw*1.5e-3
 *   slot 2 = STTS22H temp  (0x38): TEMP_L_OUT   0x06, 2 bytes, C=raw*0.01 */
static int rs_lsm_shub_init(void) {
    /* One-time slave power-up writes via the write-once channel. Each needs its own
     * enable-cycle-disable so the single DATAWRITE channel fires per slave. */
    static const struct { uint8_t addr, reg, val; } inits[3] = {
        { 0x5D, 0x10, 0x20 },  /* LPS22DF CTRL_REG1: ODR 25 Hz continuous */
        { 0x1E, 0x60, 0x8C },  /* LIS2MDL CFG_REG_A: temp-comp, 100 Hz, continuous */
        { 0x38, 0x04, 0x3C },  /* STTS22H CTRL: free-run + auto-inc + BDU */
    };
    /* We never software-reset the LSM (would drop the I3C dynamic address), so the sensor-hub
     * master block can hold stale/wedged state from a prior config. RST_MASTER_REGS resets ONLY
     * the I2C-master interface + its config/output regs -- not the chip, not the I3C address.
     * Pulse it before (re)configuring. Must be manually asserted then de-asserted (AN5763 7.2.1). */
    lsm6dsv16x_sh_reset_set(&g_ctx, 1);
    lsm_mdelay(1);
    lsm6dsv16x_sh_reset_set(&g_ctx, 0);
    lsm_mdelay(1);

    /* The aux-master pins are muxed SDx/AH1/QVAR1 and SCx/AH2/QVAR2: if the analog-hub / Qvar
     * front-end owns them (CTRL7.AH_QVAR_EN, bit7) the I2C master can never drive them, and the
     * master state machine never issues a START -- exactly our symptom (MASTER_ON=1, config +
     * pull-up all latched, yet zero cycles / zero NACK). AH_QVAR_EN can persist from a prior
     * config because we deliberately skip the software reset (keeps the I3C dynamic address).
     * Capture it as-found, then force it off before bringing the hub up. */
    lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_CTRL7, &g_lsm_ctrl7_pre, 1);
    {
        uint8_t ctrl7 = (uint8_t)(g_lsm_ctrl7_pre & ~0x80u);  /* clear AH_QVAR_EN */
        lsm6dsv16x_write_reg(&g_ctx, LSM6DSV16X_CTRL7, &ctrl7, 1);
    }

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
    lsm6dsv16x_sh_cfg_read_t r0 = { 0x5D, 0x28, 3 };
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

    /* DIAG: IF_CFG (main bank) holds the aux-bus pull-up enable. SHUB_PU_EN is bit6 -- if it
     * reads 0 here despite sh_master_interface_pull_up_set(1), the pull-up write isn't landing
     * and the Mode-3 aux bus floats (root cause A). The OLD diag checked MASTER_CONFIG bit3,
     * which is not_used0 -- it never told us anything about the pull-up. */
    lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_IF_CFG, &g_lsm_if_cfg, 1);

    /* DIAG: MASTER_CONFIG + SLV0_ADD (SHUB bank) -- confirm enable + slave-cfg latched.
     * MASTER_CONFIG: MASTER_ON=bit2, AUX_SENS_ON=bits1:0, WRITE_ONCE=bit6, START_CONFIG=bit5.
     * SLV0_ADD: 7-bit addr in bits7:1, rw_0 in bit0 (expect 0x5C<<1 | 1 = 0xB9 for LPS22DF read). */
    lsm6dsv16x_mem_bank_set(&g_ctx, LSM6DSV16X_SENSOR_HUB_MEM_BANK);
    lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_MASTER_CONFIG, &g_lsm_master_config, 1);
    lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_SLV0_ADD, &g_lsm_slv0_add, 1);
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

    /* Accel + gyro must run for SFLP to fuse. High-performance mode (anti-alias filter on),
     * high ODR, and generous full scale -- see RS_LSM_* tuning knobs at the top of this file. */
    lsm6dsv16x_xl_mode_set(&g_ctx, LSM6DSV16X_XL_HIGH_PERFORMANCE_MD);
    lsm6dsv16x_gy_mode_set(&g_ctx, LSM6DSV16X_GY_HIGH_PERFORMANCE_MD);
    lsm6dsv16x_xl_full_scale_set(&g_ctx, RS_LSM_XL_FS);
    lsm6dsv16x_gy_full_scale_set(&g_ctx, RS_LSM_GY_FS);
    if (lsm6dsv16x_xl_data_rate_set(&g_ctx, RS_LSM_XL_GY_ODR) != 0 ||
        lsm6dsv16x_gy_data_rate_set(&g_ctx, RS_LSM_XL_GY_ODR) != 0) {
        return -3;
    }

    /* SFLP game rotation vector -> FIFO. (LSM config persists across MCU -rst, so
     * explicitly set game_rotation to the desired state rather than skipping the call.) */
    lsm6dsv16x_sflp_data_rate_set(&g_ctx, RS_LSM_SFLP_ODR);
    lsm6dsv16x_sflp_game_rotation_set(&g_ctx, RS_LSM_SFLP_ON);
    lsm6dsv16x_fifo_sflp_raw_t sflp_batch = {
        .game_rotation = RS_LSM_SFLP_ON,
        .gravity = RS_LSM_SFLP_BATCH_AUX,
        .gbias = RS_LSM_SFLP_BATCH_AUX,
    };
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
