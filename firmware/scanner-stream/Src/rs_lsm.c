/* LSM6DSV16X driver — see rs_lsm.h.
 *
 * Bring-up is incremental: SFLP orientation first (validated on the bench), then the
 * I2C sensor-hub environmental slaves. SHUB config is compiled in but gated by
 * RS_LSM_ENABLE_SHUB so orientation can be validated in isolation. */
#include "rs_lsm.h"

#include <math.h>
#include <string.h>

#include "lsm6dsv16x_reg.h"
#include "rs_protocol.h"
#include "stm32h5xx_hal.h"

/* stream 11 puts an array of these straight on the wire — no padding allowed. */
_Static_assert(sizeof(rs_lsm_raw_word_t) == RS_IMU_RAW_REC_SIZE, "IMU_RAW record must be 8 B");

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
/* Batch SFLP gravity + gyro-bias vectors to FIFO for host observability. ENABLED 2026-07-28:
 * the host stream layer now demuxes GRAVITY(0x17)/GBIAS(0x16) — they ride stream 11
 * (RS_STREAM_IMU_RAW) verbatim. The game-rotation vector is already internally bias-corrected
 * regardless of this flag. */
#define RS_LSM_SFLP_BATCH_AUX (1)

/* ---- Auto-idle wake-on-motion (2026-08-03) -------------------------------------------------
 * The LSM's embedded Wake-Up function (WAKE_UP_SRC/THS/DUR, FUNCTIONS_ENABLE) is a hardware
 * block SEPARATE from the SFLP engine above (SFLP lives under EMB_FUNC_EN_A; Wake-Up is a
 * "basic interrupt" under FUNCTIONS_ENABLE) -- confirmed against the datasheet, no shared state,
 * no coexistence restriction. It also does not need HAODR mode (which we don't use -- both XL and
 * GY run LSM6DSV16X_*_HIGH_PERFORMANCE_MD above), so the one documented HAODR/activity caveat
 * doesn't apply either way. Configured with act_mode = XL_AND_GY_NOT_AFFECTED (INACT_EN=00):
 * "stationary/motion-only interrupts generated, accelerometer/gyroscope configuration does not
 * change" -- this is a pure motion DETECTOR, it must never itself change the XL/GY power state
 * SFLP depends on. Polled (rs_lsm_check_wake_up), not wired to the INTR pin: the IKS4A1 routes
 * the LSM's own INT1/INT2 through a jumper-selectable selector that isn't populated on this rig
 * (docs/iks4a1-stacking.md), and polling this one status byte over I3C is cheap enough that
 * wiring a physical interrupt buys nothing here.
 * Threshold: 500 mg (WU_INACT_THS_W=4 -> 125 mg/LSB, WK_THS=4) -- a deliberate "picked it up"
 * level, well above handheld tremor/ambient vibration and the accelerometer's own noise floor,
 * not a fine motion-jitter detector (that job belongs to the display-only SLAM stationarity gate,
 * not this one). Duration = 3 (max, ~6.25 ms at 480 Hz) for a touch of debounce against a single
 * transient sample; both are the firmware's own constants for now, not host-configurable -- see
 * the auto-idle plan doc if on-rig testing shows they need tuning. */
#define RS_LSM_WAKE_THS_WEIGHT (4u)   /* WU_INACT_THS_W: 125 mg/LSB */
#define RS_LSM_WAKE_THS        (4u)   /* WK_THS: 4 * 125 mg = 500 mg */
#define RS_LSM_WAKE_DUR        (3u)   /* WAKE_DUR: 3 / ODR_XL */

/* ---- Raw FIFO pass-through (stream 11, 2026-07-28) ----------------------------------------
 * The SFLP game-rotation quaternion is encoded fp16 in the FIFO (~0.056 deg/step), which is now
 * the orientation noise FLOOR — averaging the batch (RS_LSM_SFLP_AVERAGE, below) already took us
 * as far as decimation can. Escaping it means fusing on the host from the 16-bit fixed-point
 * FIFO words, so batch the raw gyro + accel at the full XL/GY ODR and pass them through
 * untouched. BDR_GY and BDR_XL are independent fields (FIFO_CTRL3), no shared decimation.
 *
 * TIMESTAMP is batched with them because the frame header's t_us is HAL_GetTick()*1000 — 1 ms
 * granularity, far too coarse to integrate a 480 Hz gyro. The timestamp word carries the LSM's
 * own 32-bit tick (~21.7 us/LSB) so the host can put every sample on the sensor's clock.
 *
 * Budget: 5 tags x 480 Hz = 2400 words/s + 480 timestamp words/s, drained at the ToF rate
 * (~28 Hz) = ~90-105 words per drain against a 256-word FIFO: 2.3x headroom, but ONE missed
 * drain overruns. g_lsm_fifo_ovr counts FIFO_STATUS.FIFO_OVR_IA so an overrun is visible
 * on-target rather than silent. */
#define RS_LSM_RAW_BATCH (1)
#define RS_LSM_XL_BDR    LSM6DSV16X_XL_BATCHED_AT_480Hz
#define RS_LSM_GY_BDR    LSM6DSV16X_GY_BATCHED_AT_480Hz

/* ---- Orientation-noise reduction (2026-07-28) --------------------------------------------
 * Measured on a stationary rig: the host saw ~0.14 deg mean / 0.25 deg p95 of zero-mean
 * orientation noise per frame (net 0.14 deg over 15 s against 22.9 deg summed absolute), which
 * the point-cloud renderer swings on a 3 m lever arm into visible edge shimmer.
 *
 * Root cause is a DECIMATION defect, not sensor grade: SFLP runs at 480 Hz and the host consumes
 * one sample per ToF frame (~28 Hz), and rs_lsm_read_latest used to keep only the LAST quaternion
 * of each ~17-sample FIFO batch. Point-sampling a 480 Hz signal at 28 Hz folds the entire
 * 0-240 Hz noise band into 0-14 Hz -- textbook aliasing, no anti-alias filter anywhere in the
 * chain. Averaging the batch instead is the correct decimation and cuts white noise by sqrt(N)
 * (~4.1x at 17 samples) for a handful of flops.
 *
 * RS_LSM_GY_LPF1_BW additionally band-limits the gyro feeding the fusion. AN5763 Table 20, the
 * ODR = 480 Hz block: LPF1 bypassed (the POR default we currently run) leaves the gyro chain
 * 187 Hz wide -- absurd for a handheld scanner, where real motion lives below ~20 Hz. Selecting
 * 110 gives 28.4 Hz (-50.7 deg @ 20 Hz, i.e. ~7 ms group delay -- a fifth of a 36 ms frame, so
 * the rotation prior stays timely). White noise scales as sqrt(BW), so 187 -> 28.4 Hz is worth
 * ~2.6x on its own. NB AN5763 Figure 6 does not draw the SFLP tap point, so whether LPF1 actually
 * reaches the fusion is MEASURED on-target, not assumed; set RS_LSM_GY_LPF1_ON to 0 to A/B it.
 * (The accelerometer has no equivalent knob: its LPF1 is fixed at ODR/2 in high-performance mode,
 * and Figure 3 shows the embedded-function tap sitting ahead of the configurable LPF2.) */
#define RS_LSM_SFLP_AVERAGE (1)   /* average each FIFO batch instead of keeping the last sample */
#define RS_LSM_GY_LPF1_ON   (1)
#define RS_LSM_GY_LPF1_BW   LSM6DSV16X_GY_AGGRESSIVE  /* 110b -> 28.4 Hz @ ODR 480 Hz */

/* ---- Sensor-hub averaging (2026-07-28, same class of defect as RS_LSM_SFLP_AVERAGE) -------
 * The sensor-hub master reads ALL connected slaves (baro/mag/temp, see rs_lsm_shub_init) once
 * per hub cycle at LSM6DSV16X_SH_60Hz -- one shared rate, not per-slave -- and every enabled
 * slave is FIFO-batched at that same 60 Hz. The FIFO is drained once per ToF frame (~28-30 Hz),
 * so each drain sees ~2 FIFO words per slave (60 / 30), and rs_lsm_shub_demux used to just
 * overwrite out->{pressure_pa,mag_ut,temp_c} on every word, keeping only the last of that ~2 and
 * throwing the other away -- a milder version of the same point-sampling defect BUG-027 fixed
 * for SFLP (17:1 there vs ~2:1 here). Accumulate-and-divide is the correct decimation; averaging
 * white noise over 2 samples is worth ~sqrt(2) = 1.4x.
 *
 * All three slaves share one hub cycle, so the same ~2-words-per-drain arithmetic applies to
 * all of them mechanically -- there's no structural reason to average mag but not baro/temp.
 * (Whether each of the ~2 hub reads is a genuinely new physical conversion depends on the
 * individual slave's own ODR vs. the 60 Hz hub rate -- LPS22DF is configured well below 60 Hz,
 * so some of its pairs may be duplicate reads of the same conversion; STTS22H free-run AVG=4 is
 * close to 60 Hz. Either way averaging is safe: the mean of two identical duplicate reads is
 * that same value, so a slave with no new sample yet just costs a few flops for zero effect,
 * while a slave that did convert twice gets the same 1.4x. Averaging all three keeps the demux
 * uniform instead of special-casing by (unmeasured, config-dependent) per-slave update rate. */
#define RS_LSM_SHUB_AVERAGE (1)   /* average each slave's FIFO batch instead of keeping the last sample */

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
    /* LIS2MDL noise (AN5069 Table 9): we already run LP=0 (high-resolution) at 100 Hz with
     * temperature compensation, but CFG_REG_B was left at its 0x00 POR default -- filter OFF and
     * offset cancellation OFF -- which is the 4.5 mG RMS / ODR/2 (50 Hz) corner of that table.
     * Setting LPF|OFF_CANC moves us to the 3.0 mG RMS / ODR/4 (25 Hz) corner; the AN notes the
     * low-pass costs no extra current, and OFF_CANC additionally runs alternating set/reset
     * pulses so the AMR bridge's own offset is cancelled sample-to-sample (AN5069 §8) instead of
     * drifting under the host's static mag_cal.json hard-iron fit. 25 Hz of bandwidth is still
     * ~10x what a 20 s yaw-fusion time constant can use. */
    /* LIS2MDL BDU (CFG_REG_C bit4), added 2026-07-29 -- we were the exact case AN5069 6.4 warns
     * about: "If reading the magnetometer data is not synchronized with either the Zyxda event bit
     * ... or with the data-ready signal ... it is strongly recommended to set the BDU bit to 1."
     * The sensor hub polls the mag on ITS OWN 60 Hz cadence against the mag's 100 Hz ODR, so reads
     * are unsynchronised by construction and an output-register refresh can land mid-burst. Without
     * BDU that yields a TORN sample -- MSB from one measurement, LSB from the next -- and at
     * 1.5 mG/LSB a torn low byte is worth up to 255 LSB = 382 mG = 38 uT, i.e. comparable to Earth's
     * whole field. Those are exactly the outliers a least-squares ellipsoid fit (fit_ellipsoid) has
     * no defence against, so they can skew a hard/soft-iron calibration -- a candidate contributor
     * to BUG-030. BDU blocks the refresh of an axis pair only until both its bytes are read, and the
     * hub reads all 6 bytes in one burst, so nothing stalls.
     * AN5069's own recommended init (section 13 flow) is "COMP_TEMP_EN, BDU, Continuous mode, enable
     * offset cancellation, ODR = 100 Hz" -- we had every item except BDU.
     * Caveat per the same section: BDU guarantees LSB/MSB coherence per axis, NOT that X, Y and Z
     * come from one sample; that residual needs a read fast relative to the ODR, which a single
     * 6-byte hub burst is. */
    static const struct { uint8_t addr, reg, val; } inits[5] = {
        { 0x5D, 0x10, 0x20 },  /* LPS22DF CTRL_REG1: ODR 25 Hz continuous */
        { 0x1E, 0x60, 0x8C },  /* LIS2MDL CFG_REG_A: temp-comp, 100 Hz, continuous */
        { 0x1E, 0x61, 0x03 },  /* LIS2MDL CFG_REG_B: OFF_CANC | LPF -> 25 Hz BW, 3.0 mG RMS */
        { 0x1E, 0x62, 0x10 },  /* LIS2MDL CFG_REG_C: BDU (only) -- I2C_DIS/BLE/4WSPI/self-test off */
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
    for (int i = 0; i < (int)(sizeof inits / sizeof inits[0]); i++) {
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

#if RS_LSM_SHUB_AVERAGE
/* Accumulator for one drain's worth of sensor-hub samples -- see RS_LSM_SHUB_AVERAGE. Mirrors
 * quat_acc/quat_n in rs_lsm_read_latest_raw, just three independent scalar/vector channels
 * instead of one quaternion (no hemisphere alignment needed, these aren't rotations). */
typedef struct {
    float press_acc;     uint16_t press_n;
    float mag_acc[3];    uint16_t mag_n;
    float temp_acc;      uint16_t temp_n;
} rs_lsm_shub_acc_t;
#endif

static void rs_lsm_shub_demux(const lsm6dsv16x_fifo_out_raw_t *w, rs_lsm_sample_t *out
#if RS_LSM_SHUB_AVERAGE
                              , rs_lsm_shub_acc_t *acc
#endif
                              ) {
    switch (w->tag) {
    case LSM6DSV16X_SENSORHUB_SLAVE0_TAG: {  /* LPS22DF pressure, 24-bit LE */
        uint32_t raw = (uint32_t)w->data[0] | ((uint32_t)w->data[1] << 8) |
                       ((uint32_t)w->data[2] << 16);
        float pa = (float)raw * (100.0f / 4096.0f);  /* hPa=raw/4096 -> Pa */
#if RS_LSM_SHUB_AVERAGE
        acc->press_acc += pa;
        acc->press_n++;
#else
        out->pressure_pa = pa;
#endif
        out->have_env = 1;
        break;
    }
    case LSM6DSV16X_SENSORHUB_SLAVE1_TAG: {  /* LIS2MDL mag x,y,z int16 LE */
        for (int i = 0; i < 3; i++) {
            int16_t raw = (int16_t)(w->data[2 * i] | (w->data[2 * i + 1] << 8));
            float ut = (float)raw * 0.15f;  /* 1.5 mgauss/LSB * 0.1 µT/mgauss */
#if RS_LSM_SHUB_AVERAGE
            acc->mag_acc[i] += ut;
#else
            out->mag_ut[i] = ut;
#endif
        }
#if RS_LSM_SHUB_AVERAGE
        acc->mag_n++;
#endif
        out->have_env = 1;
        break;
    }
    case LSM6DSV16X_SENSORHUB_SLAVE2_TAG: {  /* STTS22H temp int16 LE */
        int16_t raw = (int16_t)(w->data[0] | (w->data[1] << 8));
        float c = (float)raw * 0.01f;
#if RS_LSM_SHUB_AVERAGE
        acc->temp_acc += c;
        acc->temp_n++;
#else
        out->temp_c = c;
#endif
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

    /* Band-limit the gyro (see RS_LSM_GY_LPF1_* above). Order matters: set the bandwidth before
     * enabling, so the filter never runs a cycle at the POR bandwidth. LPF1 is only available in
     * high-performance mode -- which we selected above -- and must not be used if the OIS/EIS
     * chains are ever enabled (AN5763 3.9); this build enables neither. */
    lsm6dsv16x_filt_gy_lp1_bandwidth_set(&g_ctx, RS_LSM_GY_LPF1_BW);
    lsm6dsv16x_filt_gy_lp1_set(&g_ctx, RS_LSM_GY_LPF1_ON);

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

#if RS_LSM_RAW_BATCH
    /* Raw gyro/accel + the LSM timestamp into the same FIFO (see RS_LSM_RAW_BATCH above).
     * TIMESTAMP_EN must be on for the counter to run at all; DEC_TS_BATCH = DEC_1 stamps
     * every batch-counter tick, i.e. one timestamp word per XL/GY sample time. */
    lsm6dsv16x_fifo_xl_batch_set(&g_ctx, RS_LSM_XL_BDR);
    lsm6dsv16x_fifo_gy_batch_set(&g_ctx, RS_LSM_GY_BDR);
    lsm6dsv16x_timestamp_set(&g_ctx, 1);
    lsm6dsv16x_fifo_timestamp_batch_set(&g_ctx, LSM6DSV16X_TMSTMP_DEC_1);
#endif

#if RS_LSM_ENABLE_SHUB
    /* Sensor-hub environmental slaves are configured in a later bring-up step. */
    if (rs_lsm_shub_init() != 0) {
        return -6;
    }
#endif

    /* Clock calibration. INTERNAL_FREQ_FINE (0x4F) is the per-part trim of the internal
     * oscillator that clocks BOTH the ODRs and the FIFO timestamp counter. AN5763 6.4:
     *
     *     t_tick = 1 / (46080 * (1 + 0.0013 * FREQ_FINE))   [seconds]
     *
     * i.e. the nominal 21.7 us tick is only right for FREQ_FINE = 0. Parts are trimmed
     * several percent away from nominal, and the host was integrating gyro against the
     * nominal tick -- a pure scale error on every angle it derived. Read once here (the
     * register is a factory trim, it does not change at runtime) and ship it to the host
     * as stream 12 so the host can apply the formula itself.
     *
     * A read failure is NOT fatal: g_lsm_freq_fine_valid stays 0, no stream 12 goes out,
     * and the host keeps its nominal-tick fallback. */
    {
        int8_t ff = 0;
        if (lsm6dsv16x_odr_cal_reg_get(&g_ctx, &ff) == 0) {
            g_lsm_freq_fine = ff;
            g_lsm_freq_fine_valid = 1u;
        }
    }

    /* Continuous FIFO. */
    if (lsm6dsv16x_fifo_mode_set(&g_ctx, LSM6DSV16X_STREAM_MODE) != 0) {
        return -7;
    }

    /* Wake-on-motion (see RS_LSM_WAKE_* above). Best-effort like the freq-fine read above: a
     * failure here only means the device can never self-wake from an idled standby -- the
     * existing host-commanded wake/idle path (SET_STANDBY) is completely unaffected, so it
     * does not fail rs_lsm_init(). act_mode is set explicitly (not left at its POR default)
     * for the same reason SFLP is above: this device's config persists across an MCU -rst. */
    {
        lsm6dsv16x_act_thresholds_t wk = { 0 };
        wk.inactivity_cfg.wu_inact_ths_w = RS_LSM_WAKE_THS_WEIGHT;
        wk.threshold = RS_LSM_WAKE_THS;
        wk.duration = RS_LSM_WAKE_DUR;
        lsm6dsv16x_act_thresholds_set(&g_ctx, &wk);
        lsm6dsv16x_act_mode_set(&g_ctx, LSM6DSV16X_XL_AND_GY_NOT_AFFECTED);
        lsm6dsv16x_interrupt_mode_t irq = { .enable = 1, .lir = 1 };
        lsm6dsv16x_interrupt_enable_set(&g_ctx, irq);
    }
    return 0;
}

int rs_lsm_check_wake_up(uint8_t *wake_up_src_out) {
    uint8_t raw = 0;
    if (lsm6dsv16x_read_reg(&g_ctx, LSM6DSV16X_WAKE_UP_SRC, &raw, 1) != 0) {
        return -1;
    }
    if (wake_up_src_out != NULL) {
        *wake_up_src_out = raw;
    }
    return (raw & 0x08u) ? 1 : 0;   /* bit3 = WU_IA, see docs/protocol.md AUTO_WAKE_MOTION */
}

int8_t  g_lsm_freq_fine = 0;          /* INTERNAL_FREQ_FINE (0x4F), latched in rs_lsm_init */
uint8_t g_lsm_freq_fine_valid = 0;    /* 1 once the register above has actually been read */

uint16_t g_lsm_tag_hist[32] = { 0 };  /* diagnostic: FIFO tag histogram (bench probe reads it) */
uint32_t g_lsm_fifo_ovr = 0;          /* diagnostic: drains that found FIFO_STATUS.FIFO_OVR_IA set */
uint16_t g_lsm_raw_dropped = 0;       /* diagnostic: stream-11 words dropped for want of buffer */

uint8_t rs_lsm_shub_status_raw(void) {
    lsm6dsv16x_status_master_t st = { 0 };
    lsm6dsv16x_sh_status_get(&g_ctx, &st);
    return (uint8_t)(st.sens_hub_endop | (st.slave0_nack << 3) | (st.slave1_nack << 4) |
                     (st.slave2_nack << 5) | (st.slave3_nack << 6) | (st.wr_once_done << 7));
}

int rs_lsm_read_latest(rs_lsm_sample_t *out) {
    return rs_lsm_read_latest_raw(out, NULL, 0u, NULL);
}

/* Latch the LSM's own timestamp counter (TIMESTAMP0..3, 0x40-0x43) in ONE 4-byte register
 * read -- see rs_lsm.h. Deliberately NOT the FIFO: the FIFO tells you when a *sample* was
 * taken, which is only ever "some time before the drain", whereas this register answers
 * "what does the LSM's clock read right now", which is the question you have to ask at the
 * ToF's FRAME_READY edge (BUG-031). TIMESTAMP_EN is already on (rs_lsm_init, RS_LSM_RAW_BATCH),
 * so the counter is the same one stream 11's TIMESTAMP words come from and shares stream 12's
 * tick period. */
int rs_lsm_read_timestamp(uint32_t *ticks) {
    if (ticks == NULL) {
        return -1;
    }
    return (lsm6dsv16x_timestamp_raw_get(&g_ctx, ticks) == 0) ? 0 : -1;
}

int rs_lsm_read_latest_raw(rs_lsm_sample_t *out, rs_lsm_raw_word_t *raw, uint16_t raw_max,
                           uint16_t *raw_count) {
    uint16_t raw_n = 0;
    out->have_quat = 0;
    out->have_env = 0;
    out->quat_mid_ticks = 0u;
    out->quat_n = 0u;
    /* Span of this drain's TIMESTAMP words, for out->quat_mid_ticks. The midpoint of the
     * span is used rather than a per-sample tally deliberately: the SFLP game-rotation word
     * carries no timestamp of its own, and pairing it with "the last TIMESTAMP word seen"
     * would inherit whatever order the FIFO happens to emit a sample-time's words in (a
     * silent one-sample, 2.08 ms bias). SFLP and XL/GY share one ODR and one drain, so the
     * quat samples tile the same span the TIMESTAMP words do, and the span's midpoint is the
     * mean of their times without depending on intra-group ordering at all. */
    uint32_t ts_first = 0u, ts_last = 0u;
    uint8_t ts_seen = 0u;
    if (raw_count != NULL) {
        *raw_count = 0;
    }
#if RS_LSM_SFLP_AVERAGE
    /* Accumulate the batch rather than keeping its last sample -- see RS_LSM_SFLP_AVERAGE. */
    float quat_acc[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    uint16_t quat_n = 0;
#endif
#if RS_LSM_ENABLE_SHUB && RS_LSM_SHUB_AVERAGE
    /* Accumulate each shub slave's batch rather than keeping its last sample -- see
     * RS_LSM_SHUB_AVERAGE. */
    rs_lsm_shub_acc_t shub_acc = { 0 };
#endif

    lsm6dsv16x_fifo_status_t status;
    if (lsm6dsv16x_fifo_status_get(&g_ctx, &status) != 0) {
        return -1;
    }
    if (status.fifo_ovr) {
        g_lsm_fifo_ovr++;   /* a drain was missed: samples were lost before we got here */
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
        if (word.tag == LSM6DSV16X_TIMESTAMP_TAG) {
            uint32_t t = (uint32_t)word.data[0] | ((uint32_t)word.data[1] << 8) |
                         ((uint32_t)word.data[2] << 16) | ((uint32_t)word.data[3] << 24);
            if (!ts_seen) {
                ts_first = t;
                ts_seen = 1u;
            }
            ts_last = t;
        }
        if (raw != NULL) {
            /* stream-11 pass-through: 16-bit fixed-point words only. 0x13 (game rotation)
             * belongs to stream 9 and the sensor-hub tags to stream 10. */
            switch (word.tag) {
            case LSM6DSV16X_GY_NC_TAG:
            case LSM6DSV16X_XL_NC_TAG:
            case LSM6DSV16X_TIMESTAMP_TAG:
            case LSM6DSV16X_SFLP_GYROSCOPE_BIAS_TAG:
            case LSM6DSV16X_SFLP_GRAVITY_VECTOR_TAG:
                if (raw_n < raw_max) {
                    /* rebuild the FIFO_DATA_OUT_TAG register byte the driver took apart */
                    raw[raw_n].tag = (uint8_t)(((uint8_t)word.tag << 3) | ((word.cnt & 0x3u) << 1));
                    memcpy(raw[raw_n].data, word.data, 6);
                    raw[raw_n].reserved = 0u;
                    raw_n++;
                } else {
                    g_lsm_raw_dropped++;
                }
                break;
            default:
                break;
            }
        }
        switch (word.tag) {
        case LSM6DSV16X_SFLP_GAME_ROTATION_VECTOR_TAG:
#if RS_LSM_SFLP_AVERAGE
        {
            float q[4];
            sflp_word_to_quat(word.data, q);
            /* q and -q are the same rotation, so align each sample to the accumulator before
             * adding or opposite-hemisphere samples would cancel. Within one ~35 ms batch the
             * spread is milliradians, so a component-wise mean + renormalize is indistinguishable
             * from a proper quaternion barycentre and costs no iteration. */
            if (quat_n != 0) {
                float dot = quat_acc[0] * q[0] + quat_acc[1] * q[1] +
                            quat_acc[2] * q[2] + quat_acc[3] * q[3];
                if (dot < 0.0f) {
                    q[0] = -q[0]; q[1] = -q[1]; q[2] = -q[2]; q[3] = -q[3];
                }
            }
            for (int k = 0; k < 4; k++) {
                quat_acc[k] += q[k];
            }
            quat_n++;
            break;
        }
#else
            sflp_word_to_quat(word.data, out->quat);
            out->have_quat = 1;
            break;
#endif
#if RS_LSM_ENABLE_SHUB
        case LSM6DSV16X_SENSORHUB_SLAVE0_TAG:   /* LPS22DF pressure */
        case LSM6DSV16X_SENSORHUB_SLAVE1_TAG:   /* LIS2MDL mag */
        case LSM6DSV16X_SENSORHUB_SLAVE2_TAG:   /* STTS22H temp */
            rs_lsm_shub_demux(&word, out
#if RS_LSM_SHUB_AVERAGE
                              , &shub_acc
#endif
                              );
            break;
#endif
        default:
            break;
        }
    }
#if RS_LSM_SFLP_AVERAGE
    if (quat_n != 0) {
        float n = sqrtf(quat_acc[0] * quat_acc[0] + quat_acc[1] * quat_acc[1] +
                        quat_acc[2] * quat_acc[2] + quat_acc[3] * quat_acc[3]);
        if (n > 1e-6f) {
            for (int k = 0; k < 4; k++) {
                out->quat[k] = quat_acc[k] / n;   /* normalize == divide by the mean's magnitude */
            }
            out->have_quat = 1;
            out->quat_n = quat_n;
            if (ts_seen) {
                /* Unsigned midpoint written so a counter wrap between first and last (once
                 * per ~26 h at 22.28 µs/tick) still lands on the right value. */
                out->quat_mid_ticks = ts_first + (uint32_t)((ts_last - ts_first) / 2u);
            }
        }
    }
#endif
#if RS_LSM_ENABLE_SHUB && RS_LSM_SHUB_AVERAGE
    if (shub_acc.press_n != 0) {
        out->pressure_pa = shub_acc.press_acc / (float)shub_acc.press_n;
    }
    if (shub_acc.mag_n != 0) {
        for (int k = 0; k < 3; k++) {
            out->mag_ut[k] = shub_acc.mag_acc[k] / (float)shub_acc.mag_n;
        }
    }
    if (shub_acc.temp_n != 0) {
        out->temp_c = shub_acc.temp_acc / (float)shub_acc.temp_n;
    }
#endif
    if (raw_count != NULL) {
        *raw_count = raw_n;
    }
    return (out->have_quat || out->have_env || raw_n != 0) ? 0 : -1;
}
