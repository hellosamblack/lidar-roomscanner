/**
 ******************************************************************************
 * @file    vl53l9_app.c
 * @author  IMD Software Team
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "ethernet_transport.h"
#include <stdlib.h>
#include <string.h>

#include "vl53l9.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include "vl53l9_transform.h"
#include "vl53l9_utils.h"

/* application customization */
#define CONF_DEVICE_ID   (0) /**< select device entry in platform descriptor array (see vl53l9_device.c) */
#define CONF_PRINT_FRAME   (0) /**< ASCII art disabled in streaming builds */
#define CONF_STREAM_BINARY (1) /**< emit rs_protocol frames over native USB CDC (see rs_send_frame_cdc) */
#define CONF_STREAM_RAW (1) /**< also stream RAW_3DMD + periodic CALIB (dual-stream validation / PC-transform mode) */
/* 1 = run vl53l9_transform on-MCU and stream DEPTH (Phase 1 behavior, also the golden-pair
 * regeneration path with CONF_STREAM_RAW=1); 0 = raw-only, transform runs on the PC (Phase 2 --
 * equivalence gate passed, on-MCU transform removed from the hot path).
 *
 * Selected at build time: CMakeLists.txt always defines this on the command line from the
 * CONF_TRANSFORM_ONBOARD option (OFF by default -> 0), and the DebugOnboardTransform preset sets
 * it ON. The #ifndef fallback keeps non-CMake/direct compiles building the production raw-only
 * config; do not hand-edit the value here, pass -DCONF_TRANSFORM_ONBOARD=ON to cmake instead. */
#ifndef CONF_TRANSFORM_ONBOARD
#define CONF_TRANSFORM_ONBOARD (0)
#endif
#define CONF_USECASE     (VL53L9_USECASE_AR_PRECISION) /**< select ranging profile to be applied (see vl53l9_utils.h) */

/* Every output path is knob-gated: DEPTH send needs BINARY && TRANSFORM, RAW/CALIB send needs
 * BINARY && RAW, the legacy ASCII print needs !BINARY && TRANSFORM. With the transform off-board
 * the only possible output is the binary RAW stream -- reject silent no-output combos loudly. */
#if !CONF_TRANSFORM_ONBOARD && !(CONF_STREAM_BINARY && CONF_STREAM_RAW)
#error "No output stream: transform off-board requires CONF_STREAM_BINARY=1 and CONF_STREAM_RAW=1"
#endif

#include "rs_protocol.h"
#include "stm32h5xx_nucleo.h"
#include "tusb.h"
#include "rs_lsm.h"
#include "rs_ranging.h"

extern UART_HandleTypeDef hcom_uart[];
extern void rs_boot_heartbeat(void);   /* yellow LD2 liveness -- see main.c */
extern I3C_HandleTypeDef hi3c1;

static void handle_error(void);

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

/* ---- IKS4A1 bus probe (bench diagnostic) -------------------------------------------
 *
 * Standalone WHO_AM_I probe over I3C1's legacy-I2C private-transfer mode, bypassing
 * vl53l9's ENTDAA/dynamic-address assignment entirely. Rationale: the IKS4A1's sensors
 * are legacy-I2C only and can never answer ENTDAA (an I3C-only CCC), so the normal boot
 * path (rs_boot_bringup -> platform_assign_dynamic_address) exhausts its 5-attempt retry
 * and hangs in tud_disconnect()+while(1) before ever touching the IKS4A1's addresses --
 * see docs/iks4a1-stacking.md "Known conflict". This probe never calls that path at all.
 *
 * Uses the exact I2C-legacy private-transfer pattern vl53l9_platform.c's _i3c_read()
 * already uses for PLATFORM_BUS_PROPERTY_I3C_LEGACY devices (HAL_I3C_AddDescToFrame +
 * Ctrl_Transmit/Ctrl_Receive with I2C_PRIVATE_WITHOUT_ARB_STOP), just with an 8-bit
 * MEMS register address instead of the ToF's 16-bit one, and a bare 7-bit TargetAddr
 * (no >>1 normalization needed since we pass it in already-7-bit here).
 *
 * Output goes over ST-Link VCOM (printf, COM1, set up by BSP_COM_Init in main() before
 * vl53l9_app() runs) rather than native CDC -- VCOM needs no host enumeration/tud_connect,
 * so it works even when the normal boot path never gets that far.
 *
 * Disabled by default. Flip to 1 for a bus bring-up bench session only; never ship 1. */
#define CONF_IKS4A1_BUS_PROBE (0)

#if CONF_IKS4A1_BUS_PROBE
static int iks4a1_read_reg(uint8_t addr7, uint8_t reg, uint8_t *out) {
    uint8_t reg_byte = reg;
    uint32_t cb1[1], sb1[1];
    I3C_PrivateTypeDef pd_w = { addr7, { &reg_byte, 1 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE };
    I3C_XferTypeDef ctx_w = { { &cb1[0], 1 }, { &sb1[0], 1 }, { &reg_byte, 1 }, { NULL, 0 } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_w, &ctx_w, 1, I2C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
        return -1;
    }
    if (HAL_I3C_Ctrl_Transmit(&hi3c1, &ctx_w, 100) != HAL_OK) {
        return -2;
    }
    while ((HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY) && (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_LISTEN)) {
    }

    uint32_t cb2[1], sb2[1];
    I3C_PrivateTypeDef pd_r = { addr7, { NULL, 0 }, { out, 1 }, HAL_I3C_DIRECTION_READ };
    I3C_XferTypeDef ctx_r = { { &cb2[0], 1 }, { &sb2[0], 1 }, { NULL, 0 }, { out, 1 } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_r, &ctx_r, 1, I2C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
        return -3;
    }
    if (HAL_I3C_Ctrl_Receive(&hi3c1, &ctx_r, 100) != HAL_OK) {
        return -4;
    }
    return 0;
}

static void iks4a1_bus_probe(void) {
    static const struct {
        const char *name;
        uint8_t addr7;
        uint8_t reg;
        uint8_t expect;
    } targets[] = {
        { "LSM6DSV16X SA0=0", 0x6A, 0x0F, 0x70 },
        { "LSM6DSV16X SA0=1", 0x6B, 0x0F, 0x70 },
        { "LIS2MDL", 0x1E, 0x4F, 0x40 },
        { "LPS22DF SA0=0", 0x5C, 0x0F, 0xB4 },
        { "LPS22DF SA0=1", 0x5D, 0x0F, 0xB4 },
    };

    HAL_Delay(200); /* let the bus settle post-boot before the first transfer */
    printf("\n[IKS4A1 PROBE] starting -- I3C1 legacy-I2C WHO_AM_I probe (ToF path never touched)\n");

    for (;;) {
        for (size_t i = 0; i < sizeof(targets) / sizeof(targets[0]); i++) {
            uint8_t val = 0xFF;
            int ret = iks4a1_read_reg(targets[i].addr7, targets[i].reg, &val);
            if (ret == 0) {
                printf("[IKS4A1 PROBE] %-10s @0x%02X reg 0x%02X = 0x%02X (expect 0x%02X) %s\n", targets[i].name,
                       targets[i].addr7, targets[i].reg, val, targets[i].expect,
                       (val == targets[i].expect) ? "PASS" : "MISMATCH");
            } else {
                printf("[IKS4A1 PROBE] %-10s @0x%02X: I3C transfer FAILED (ret=%d)\n", targets[i].name,
                       targets[i].addr7, ret);
            }
        }
        printf("[IKS4A1 PROBE] --- pass complete, repeating in 2s ---\n");
        HAL_Delay(2000);
    }
}
#endif /* CONF_IKS4A1_BUS_PROBE */

/* ---- IKS4A1 native-I3C ENTDAA probe (follow-up to iks4a1_bus_probe) -----------------
 *
 * The LSM6DSV16X (HUB1) datasheet confirms a genuine MIPI I3C v1.1 SDR slave interface
 * (ENTDAA/SETDASA/RSTDAA CCCs, private read/write, IBI -- DS13510 sec 5.2) with
 * WHO_AM_I (0Fh) fixed at 0x70 (sec 9.13). Unlike the legacy-I2C-only environmental
 * sensors, it can legitimately answer the SAME ENTDAA broadcast the ToF normally uses --
 * meaning it might already join the shared bus as a real I3C citizen (full 12.5 MHz PP
 * speed, no legacy-I2C loading) with zero jumper/solder-bridge rework, before reaching
 * for the Mode-3-sensor-hub rewire (docs/iks4a1-stacking.md candidate workaround list).
 *
 * Reuses platform_assign_dynamic_address() (platform_utils.c) verbatim -- the exact same
 * ENTDAA call the ToF boot path already uses -- then attempts a genuine I3C PRIVATE
 * (not I2C-legacy) WHO_AM_I read at the address that function hands out (hardcoded 0x52
 * in the non-retry path; that's the assumption worth testing here). */
#define CONF_IKS4A1_I3C_PROBE (0)
#define CONF_LSM_PROBE (0) /* bench: assign addrs, init LSM SFLP, print quaternion over VCOM */

#if CONF_IKS4A1_I3C_PROBE
extern I3C_HandleTypeDef hi3c1; /* redundant extern kept local to this probe for clarity */

/* Native-I3C PRIVATE register read (reg-pointer write then read), mirroring the reference
 * ULD's _i3c_read() option flags (vl53l9_platform.c). reg_len selects the register-address
 * width: 2 for the ToF's 16-bit map (e.g. MODEL_ID @0x0000), 1 for the LSM6DSV16X's 8-bit
 * map (e.g. WHO_AM_I @0x0F). Returns 0 on success, negative on the failing HAL step. */
static int i3c_priv_read(uint8_t addr7, const uint8_t *reg, uint8_t reg_len, uint8_t *out, uint8_t out_len) {
    uint32_t cbw[1], sbw[1];
    I3C_PrivateTypeDef pd_w = { addr7, { (uint8_t *)reg, reg_len }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE };
    I3C_XferTypeDef ctx_w = { { &cbw[0], 1 }, { &sbw[0], 1 }, { (uint8_t *)reg, reg_len }, { NULL, 0 } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_w, &ctx_w, 1, I3C_PRIVATE_WITHOUT_ARB_RESTART) != HAL_OK) {
        return -1;
    }
    if (HAL_I3C_Ctrl_Transmit(&hi3c1, &ctx_w, 100) != HAL_OK) {
        return -2;
    }
    while ((HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_READY) && (HAL_I3C_GetState(&hi3c1) != HAL_I3C_STATE_LISTEN)) {
    }
    uint32_t cbr[1], sbr[1];
    I3C_PrivateTypeDef pd_r = { addr7, { NULL, 0 }, { out, out_len }, HAL_I3C_DIRECTION_READ };
    I3C_XferTypeDef ctx_r = { { &cbr[0], 1 }, { &sbr[0], 1 }, { NULL, 0 }, { out, out_len } };
    if (HAL_I3C_AddDescToFrame(&hi3c1, NULL, &pd_r, &ctx_r, 1, I3C_PRIVATE_WITHOUT_ARB_STOP) != HAL_OK) {
        return -3;
    }
    if (HAL_I3C_Ctrl_Receive(&hi3c1, &ctx_r, 100) != HAL_OK) {
        return -4;
    }
    return 0;
}

static void iks4a1_i3c_probe(void) {
    HAL_Delay(200);
    printf("\n[IKS4A1 I3C PROBE] attempting ENTDAA against the shared bus (LSM6DSV16X is I3C-capable)\n");

    platform_power_reset(CONF_DEVICE_ID);

    /* Call the raw HAL directly (not platform_assign_dynamic_address()'s wrapper) so we can
     * inspect the actual ENTDAA payload -- the wrapper never validates whether a real device
     * answered before unconditionally registering address 0x52, so its "return 0" doesn't by
     * itself prove real discovery happened. A nonzero payload with recognizable PID bytes does. */
    /* debug ref 7.4 -- reduce push-pull reliance: slow the PP clock to the floor (0xff) while
     * keeping OD at the reference ~1 MHz (0x7c). Tests whether the NXS0108 auto-direction
     * translator tolerates I3C push-pull when it's slow enough, vs. can't do PP at any speed. */
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0xff;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0xff;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    HAL_I3C_Init(&hi3c1);

    /* Enumerate EVERY ENTDAA responder, giving each a DISTINCT dynamic address, then
     * PRIVATE-read WHO_AM_I from each so we can positively identify which PartID belongs
     * to which physical device. The plan's MIPIID discriminator turned out degenerate on
     * this hardware -- the ToF and the LSM6DSV16X both report MIPIID=0x09 (and identical
     * BCR/DCR/MIPIMID); PartID is the only field that differs, and this read maps each
     * PartID to a real device (0x70 == LSM6DSV16X WHO_AM_I). */
    uint64_t payload = 0;
    HAL_StatusTypeDef daa_status;
    uint16_t resp_part_id[2] = { 0 };
    uint8_t resp_addr[2] = { 0 };
    uint32_t resp_bcr[2] = { 0 };
    int n = 0;
    do {
        payload = 0;
        daa_status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
        if (daa_status == HAL_BUSY) {
            I3C_ENTDAAPayloadTypeDef pinfo = { 0 };
            HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &pinfo);
            uint32_t attempt_bcr = __HAL_I3C_GET_BCR(payload);
            uint8_t assign = (n < 2) ? (uint8_t)(0x50 + 2 * n) : 0x50; /* 0x50, 0x52 -- distinct, clear of IKS4A1 static addrs */
            printf("[IKS4A1 I3C PROBE] responder %d: PartID=0x%04X MIPIID=0x%02X BCR=0x%02lX DCR=0x%02lX -> assign 0x%02X\n",
                   n, pinfo.PID.PartID, pinfo.PID.MIPIID, (unsigned long)attempt_bcr,
                   (unsigned long)pinfo.DCR, assign);
            if (n < 2) {
                resp_part_id[n] = pinfo.PID.PartID;
                resp_addr[n] = assign;
                resp_bcr[n] = attempt_bcr;
                n++;
            }
            HAL_I3C_Ctrl_SetDynAddr(&hi3c1, assign & 0x7F);
        }
    } while (daa_status == HAL_BUSY);

    printf("[IKS4A1 I3C PROBE] ENTDAA complete: %d responder(s), final status=%d\n", n, (int)daa_status);

    /* keep the slow-PP timing (do NOT restore 12.5 MHz) so the continuous ENTDAA loop below
     * runs at the same slow-PP settings we're diagnosing. */
    HAL_I3C_Init(&hi3c1);

    if (n == 0) {
        printf("[IKS4A1 I3C PROBE] no device answered ENTDAA -- halting probe\n");
        for (;;) {
            HAL_Delay(2000);
        }
    }

    I3C_DeviceConfTypeDef dev_conf[2] = { 0 };
    for (int i = 0; i < n; i++) {
        dev_conf[i].DeviceIndex = (uint8_t)(i + 1);
        dev_conf[i].TargetDynamicAddr = resp_addr[i] & 0x7F;
        dev_conf[i].IBIAck = __HAL_I3C_GET_IBI_CAPABLE(resp_bcr[i]);
        dev_conf[i].IBIPayload = __HAL_I3C_GET_IBI_PAYLOAD(resp_bcr[i]);
        dev_conf[i].CtrlRoleReqAck = __HAL_I3C_GET_CR_CAPABLE(resp_bcr[i]);
        dev_conf[i].CtrlStopTransfer = DISABLE;
    }
    if (HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, dev_conf, (uint8_t)n) != HAL_OK) {
        printf("[IKS4A1 I3C PROBE] ConfigBusDevices FAILED\n");
        for (;;) {
            HAL_Delay(2000);
        }
    }

    /* Continuous ENTDAA at the slow-PP timing -- steady scope-observable traffic + a running
     * ToF-appearance tally. RSTDAA_THEN_ENTDAA resets every dynamic address each pass, so each
     * iteration is a fresh full enumeration. Scope SCL/SDA at 53L9A1 TP5/TP4 (ToF side of the
     * PI4ULS3V204) and compare to host PB8/PB9 to see whether the translator passes the PP bits. */
    uint32_t pass = 0, tof_hits = 0, lsm_hits = 0;
    for (;;) {
        uint64_t p = 0;
        HAL_StatusTypeDef st;
        int m = 0, tof = 0, lsm = 0;
        do {
            p = 0;
            st = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &p, I3C_RSTDAA_THEN_ENTDAA, 5000);
            if (st == HAL_BUSY) {
                I3C_ENTDAAPayloadTypeDef pi = { 0 };
                HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, p, &pi);
                if (pi.PID.PartID == 0x0102) tof = 1;       /* TOF_PART_ID (defined later in file) */
                else if (pi.PID.PartID == 0x0070) lsm = 1;   /* IKS4A1_LSM6DSV16X_PART_ID */
                HAL_I3C_Ctrl_SetDynAddr(&hi3c1, (uint8_t)(0x50 + 2 * m) & 0x7F);
                m++;
            }
        } while (st == HAL_BUSY);
        pass++;
        if (tof) tof_hits++;
        if (lsm) lsm_hits++;
        printf("[I3C PP-DIAG] pass %lu: %d resp ToF=%d LSM=%d | ToF seen %lu/%lu, LSM %lu/%lu\n",
               (unsigned long)pass, m, tof, lsm,
               (unsigned long)tof_hits, (unsigned long)pass, (unsigned long)lsm_hits, (unsigned long)pass);
        HAL_Delay(100);
    }
}
#endif /* CONF_IKS4A1_I3C_PROBE */

/* ---- microsecond wall clock (TIM2) ----------------------------------------------------
 *
 * v1 was `HAL_GetTick() * 1000`: 1 ms granular, which put a ~0.3 ms RMS quantisation floor
 * under every frame timestamp and, with the send-time stamping below, showed up on the bench
 * as 1.9 ms RMS / 6.2 ms max skew between the ToF frame stamp and the IMU's own clock. At a
 * handheld 100 deg/s that is 0.19 deg RMS / 0.62 deg worst case of angular misalignment
 * between a depth frame and the rotation used to orient it -- an order of magnitude above the
 * stationary orientation noise floor. So: a real hardware clock.
 *
 * TIM2 is the only 32-bit general-purpose timer this firmware does not already use (main.c's
 * CubeMX init brings up TIM3 for PWM and nothing else; grep for `TIM` in Src/ -- TIM2 has no
 * init, no MSP entry, no IRQ handler and no clock enable anywhere). Free-running, no
 * interrupt, no channel: prescaled to exactly 1 MHz and left to roll over its full 32-bit
 * range (~71.6 min), extended to the wire's u64 microseconds in software. The extension is
 * safe as long as rs_time_us() is called at least once per wrap period, which the ~30 Hz
 * acquisition loop guarantees by five orders of magnitude.
 *
 * Reading costs one register load plus a compare; the critical section only exists so the
 * carry bookkeeping stays consistent if this is ever called from an ISR. */
static uint64_t g_time_wraps_us = 0;  /* accumulated 2^32 µs carries */
static uint32_t g_time_last_cnt = 0;  /* previous TIM2->CNT, for wrap detection */
static uint8_t  g_time_started = 0;

static void rs_time_start(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* Timer kernel clock = PCLK1, doubled by hardware when the APB1 prescaler is not 1
     * (RM0481 timer clock section). Derived at runtime so a clock-tree change cannot
     * silently detune the µs tick. */
    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR2 & RCC_CFGR2_PPRE1) != 0u) {
        tim_clk *= 2u;
    }
    TIM2->CR1 = 0u;
    TIM2->PSC = (tim_clk / 1000000u) - 1u;   /* 250 MHz / 250 = 1 MHz */
    TIM2->ARR = 0xFFFFFFFFu;
    TIM2->EGR = TIM_EGR_UG;                  /* latch PSC/ARR, zero CNT */
    TIM2->SR  = 0u;                          /* UG set UIF; nothing consumes it, clear it */
    g_time_last_cnt = 0u;
    g_time_wraps_us = 0u;
    g_time_started = 1u;
    TIM2->CR1 = TIM_CR1_CEN;
}

static uint64_t rs_time_us(void) {
    if (!g_time_started) {
        rs_time_start();
    }
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t cnt = TIM2->CNT;
    if (cnt < g_time_last_cnt) {
        g_time_wraps_us += 0x100000000ull;
    }
    g_time_last_cnt = cnt;
    uint64_t now = g_time_wraps_us + cnt;
    __set_PRIMASK(primask);
    return now;
}

/* Stamp of the most recent platform event seen by rs_wait_event_usb(), taken before the
 * USB/Ethernet pumping that follows the wait -- see the comment there. */
static uint64_t g_evt_stamp_us = 0;

/* When non-zero, rs_send_generic_cdc() stamps frames with this instead of reading the clock
 * at send time. Armed around a ToF frame's sends with that frame's FRAME_READY instant so
 * the variable command-poll/trigger/transmit latency between "data ready" and "bytes on the
 * wire" stops folding into t_us. Disarmed immediately after, so EVENT/ACK frames -- which
 * genuinely describe the moment they are sent -- keep the live clock. */
static uint64_t g_frame_stamp_us = 0;

static uint64_t rs_stamp_us(void) {
    return g_frame_stamp_us ? g_frame_stamp_us : rs_time_us();
}

/* ---- CDC host attach (DTR rising) -------------------------------------------------------
 *
 * BUG-005: a host that opens the CDC port lands wherever the device happens to be in a frame,
 * so its first bytes are the middle of a frame it never saw the header of -- one CRC failure
 * and one resync -- and it then waits up to 63 frames for the periodic CALIB it needs before
 * it can transform anything.
 *
 * The synchronisation this needs is smaller than it looks, and the reason is worth writing
 * down because it is the whole objection that deferred this fix. TinyUSB's class callbacks do
 * NOT run in interrupt context here: `USB_DRD_FS_IRQHandler` calls `tud_int_handler`, which
 * only enqueues a `dcd_event_t` (vendor/tinyusb/src/device/usbd.c: `osal_queue_send`), and
 * `tud_task()` dequeues and dispatches. So `tud_cdc_line_state_cb` runs on the acquisition
 * loop's own thread -- but REENTRANTLY, because `rs_cdc_send()` below calls `tud_task()`
 * between chunks. That makes this a reentrancy problem, not a concurrency one: a single
 * volatile flag, set in the callback and consumed at the loop's per-frame safe point, is
 * sufficient, and no state the loop owns (raw_mem_index, the CALIB countdown, the in-flight
 * byte cursor) is ever touched from the callback.
 *
 * ETHERNET IS UNAFFECTED BY CONSTRUCTION. This callback only ever fires for a USB host, and
 * the abort below only shortens the CDC copy of a frame -- `rs_send_generic_cdc` hands the
 * whole frame to `ETH_SendFrame_Gather` before the CDC loop starts, so an Ethernet host cannot
 * see a truncated frame no matter what a USB host does. */
static volatile uint8_t g_cdc_connect_evt = 0;  /* set on DTR rise, consumed at the safe point */
static uint8_t g_cdc_dtr = 0;                   /* previous DTR, to see the EDGE not the level */
static uint32_t rs_calib_countdown = 0;         /* frames until the next periodic CALIB */

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    uint8_t now = dtr ? 1u : 0u;
    if (now && !g_cdc_dtr) {
        /* Drop whatever is still queued for the host that just went away. Safe from here:
         * see the context note above -- this is the main loop's thread, so the FIFO is not
         * being mutated underneath us by an ISR. */
        tud_cdc_write_clear();
        g_cdc_connect_evt = 1u;
    }
    g_cdc_dtr = now;
}

/* Pump the CDC FIFO out. Bounded by a NO-PROGRESS deadline, not a total-send budget: the
 * CDC TX FIFO is far smaller than a 14.8 KB frame payload, so a healthy full-speed link
 * legitimately spends several milliseconds doing write-drain-write to push one frame --
 * a total-time cap would abort that streaming, not just a dead reader. Instead this tracks
 * bytes actually accepted by tud_cdc_write() and resets the deadline every time progress is
 * made; it aborts only once NO bytes have been accepted for > RS_CDC_STALL_MS, i.e. "FIFO
 * full and host not draining". HAL_GetTick() has 1 ms resolution, so 1 ms is not a reliably
 * distinguishable interval from this call's own jitter (a tick can roll over immediately
 * after the deadline is (re)armed); RS_CDC_STALL_MS=2 is the smallest bound that stays robust
 * to that -- a live, draining reader never approaches it (drains within microseconds of FIFO
 * space opening up), while a dead one now aborts in ~2 ms per call instead of the old 100 ms.
 * Returns false on either that stall or a mid-frame host attach (frame aborted: the host
 * decoder counts one CRC failure/resync and we set DROPPED on the next frame). */
#define RS_CDC_STALL_MS 2u

static bool rs_cdc_send(const uint8_t *p, uint32_t n) {
    uint32_t t_last_progress = HAL_GetTick();
    while (n) {
        uint32_t avail = tud_cdc_write_available();
        if (avail) {
            uint32_t k = MIN(avail, n);
            tud_cdc_write(p, k);
            p += k;
            n -= k;
            t_last_progress = HAL_GetTick();
        }
        tud_task(); ETH_Process();
        if (g_cdc_connect_evt) {
            /* A host attached partway through this frame (tud_task above can dispatch the
             * line-state callback). It can only ever see this frame's tail, so stop feeding
             * it: abandoning here costs the same single resync as sending the rest would,
             * minus the wasted bytes, and the DROPPED flag on the next frame says so. */
            return false;
        }
        if ((HAL_GetTick() - t_last_progress) > RS_CDC_STALL_MS) {
            return false;
        }
    }
    tud_cdc_write_flush();
    return true;
}

/* Task 6 transport-truth routing (plan step 3): which physical transport(s) a
 * frame is allowed to go out on. RS_ROUTE_BOTH is the historical behavior
 * (send to whichever of ETH/CDC is currently connected/up) and is what every
 * call site used before this task; RS_ROUTE_ETH_ONLY/CDC_ONLY pin a frame to
 * exactly one. Two independent uses:
 *   - DATA (and the periodic diagnostic EVENT below) isolate to Ethernet-only
 *     when Ethernet has a claimed target AND the applied FPS is above 60 --
 *     see rs_active_data_route() -- so an open-but-not-draining CDC endpoint
 *     can never inject rs_cdc_send()'s no-progress stall (RS_CDC_STALL_MS per
 *     call, was ~100 ms pre-#198) into the Ethernet acquisition path at high
 *     rate.
 *   - ACKs (rs_send_ack/rs_send_ack_ranging_config) always pin to the
 *     COMMAND's own originating transport (rs_pending_cmd_t.transport,
 *     Task 4), independent of the isolation rule above -- a CDC-issued
 *     command must always get its reply on CDC even while DATA is isolated
 *     to Ethernet, because a live CDC command means a host is actively using
 *     that link right now. */
typedef enum {
    RS_ROUTE_BOTH = 0,
    RS_ROUTE_ETH_ONLY,
    RS_ROUTE_CDC_ONLY,
} rs_send_route_t;

/* Which physical transport a COMMAND arrived on -- CDC (native USB) or ETH
 * (UDP). Defined here (rather than down near rs_pending_cmd_t, Task 4's
 * original location) so the routing helpers immediately below, and
 * rs_send_frame_cdc/rs_send_event further down, can all reference it; the
 * struct that actually STORES a transport (rs_pending_cmd_t) still lives
 * near its other pending-command fields. */
typedef enum {
    RS_CMD_TRANSPORT_CDC = 0,
    RS_CMD_TRANSPORT_ETH = 1,
} rs_cmd_transport_t;

static rs_send_route_t rs_route_for_transport(rs_cmd_transport_t transport) {
    return (transport == RS_CMD_TRANSPORT_ETH) ? RS_ROUTE_ETH_ONLY : RS_ROUTE_CDC_ONLY;
}

/* Forward (tentative) declaration: g_active_profile's real definition, with its
 * full block comment, stays further down near rs_pending_cmd_t (Task 4's
 * original location) -- repeated here, identically and without an initializer,
 * so rs_active_data_route() below (used by rs_send_frame_cdc, which appears
 * before that point in the file) can read its frame_period_us. Both are
 * `static` file-scope tentative definitions of the same object; the C standard
 * merges them into one. */
static rs_ranging_profile_t g_active_profile;

/* Isolation decision for DATA/diagnostic-EVENT frames (plan step 3): Ethernet
 * has a claimed target (a host has sent it at least one datagram -- the same
 * gate ETH_SendFrame_Gather itself uses) AND the applied ranging profile's
 * fps is above the CDC-supportable ceiling (docs/protocol.md, global
 * constraint: "Ethernet is the only 90 Hz acceptance path"). Below that
 * threshold, or with no Ethernet target at all, behavior is unchanged from
 * pre-Task-6: send to whichever transport(s) are actually connected
 * (automatic CDC fallback when Ethernet has no target -- plan step 4). */
static rs_send_route_t rs_active_data_route(void) {
    if (ETH_HasTarget() &&
        rs_ranging_fps_from_period(g_active_profile.vendor.frame_period_us) > RS_RANGING_DSS_FPS_CEILING) {
        return RS_ROUTE_ETH_ONLY;
    }
    return RS_ROUTE_BOTH;
}

/* Shared low-level sender: builds header + CRC and pushes header/payload/tail over CDC.
 * frame_type-agnostic so DATA (via rs_send_frame_cdc) and ACK (via rs_send_ack) share one
 * wire-framing implementation -- the only thing that differs between them is what goes in
 * the payload and whether the DROPPED-flag bookkeeping below applies. Stays OUTSIDE the
 * !CONF_TRANSFORM_ONBOARD guard (unlike rs_send_ack) because rs_send_frame_cdc, used by
 * both loop variants, is built on it. `route` (Task 6) restricts which transport(s) this
 * one send is allowed to use -- RS_ROUTE_BOTH reproduces every pre-Task-6 call site's
 * exact prior behavior. */

static bool rs_send_generic_cdc(uint8_t frame_type, uint8_t stream_id, uint32_t seq, uint8_t flags,
                                const uint8_t *payload, uint32_t len, uint16_t w, uint16_t h,
                                rs_send_route_t route) {
    bool eth_allowed = (route != RS_ROUTE_CDC_ONLY);
    bool cdc_allowed = (route != RS_ROUTE_ETH_ONLY);
    if ((!cdc_allowed || !tud_cdc_connected()) && (!eth_allowed || !ETH_IsUp())) {
        return false;
    }
    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    rs_write_header(hdr, frame_type, stream_id, flags, seq, rs_stamp_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);

    bool eth_sent = false;
    if (eth_allowed && ETH_IsUp()) {
        eth_sent = ETH_SendFrame_Gather(hdr, RS_HEADER_SIZE, payload, len, tail, 4);
    }

    bool usb_sent = false;
    if (cdc_allowed && tud_cdc_connected()) {
        usb_sent = rs_cdc_send(hdr, RS_HEADER_SIZE) && rs_cdc_send(payload, len) && rs_cdc_send(tail, 4);
    }

    return eth_sent || usb_sent;
}

static void rs_send_frame_cdc(uint8_t stream_id, uint32_t seq, uint8_t flags, const uint8_t *payload,
                              uint32_t len, uint16_t w, uint16_t h) {
    static uint8_t pending_dropped = 0;
    rs_send_route_t route = rs_active_data_route();
    bool eth_allowed = (route != RS_ROUTE_CDC_ONLY);
    bool cdc_allowed = (route != RS_ROUTE_ETH_ONLY);

    if ((!cdc_allowed || !tud_cdc_connected()) && (!eth_allowed || !ETH_IsUp())) {
        /* no reachable host on an allowed transport: don't burn rs_cdc_send's stall
         * budget (RS_CDC_STALL_MS x 3 calls, was up to 100 ms pre-#198) per frame */
        pending_dropped = 1;
        return;
    }
    flags |= pending_dropped ? RS_FLAG_DROPPED : 0u;

    bool ok = rs_send_generic_cdc(RS_FRAME_DATA, stream_id, seq, flags, payload, len, w, h, route);
    pending_dropped = ok ? 0u : 1u;
}

/* Last successfully captured frame's counter. EVENT frames carry this as their header
 * `seq` (docs/protocol.md: "an EVENT does not increment it -- it carries the seq of the
 * last captured frame"). Stays 0 before any frame is ever captured (boot bring-up, or an
 * early boot-retry attempt) -- correct per the same spec sentence, there IS no captured
 * frame yet. Updated by the raw-only loop right after each successful
 * vl53l9_utils_parse_frame() (see its call site below); the on-board-transform loop does
 * not update it because that loop never calls rs_send_event (Task 5 scope is raw-only). */
static uint32_t g_last_seq = 0;
static uint8_t g_lsm_ok = 0; /* 1 once rs_lsm_init() succeeds; IMU/env streams are optional */

/* ---- ToF<->IMU clock correspondence, latched per frame (stream 13, BUG-031) -------------
 *
 * The frame's t_us has been the FRAME_READY instant on the MCU's TIM2 clock since 2026-07-28,
 * but a host that wants to orient that frame needs the same instant on the *LSM's* clock, and
 * the only LSM timestamps on the wire used to be stream 11's FIFO words -- samples the LSM took
 * at some point before a drain that happens far later in this loop, after the DMA readout and
 * the RAW send. "Far" is now measured rather than guessed, by drain_delay_us below: **24.3 ms**
 * past the edge, std 848 us. Inferring the frame's position from those words carries all of
 * that variable gap (host-side, 5331 static frames: 1070 us RMS against a windowed clock fit,
 * and a frame that also sends CALIB drains 655 us later still -- Welch t = -5.8).
 *
 * So ask the LSM directly, at the edge: one 4-byte TIMESTAMP register read issued immediately
 * after the FRAME_READY event, before the ToF's DMA readout is kicked (the two devices share
 * I3C1, so this is the last moment the bus is idle for the next several ms). The read is
 * bracketed by rs_time_us() so the wire carries both the delay from the edge and the read's
 * own duration -- i.e. the residual uncertainty is reported, not assumed. */
typedef struct {
    uint32_t lsm_ticks;       /* LSM TIMESTAMP counter, latched near FRAME_READY */
    uint32_t latch_delay_us;  /* FRAME_READY -> midpoint of that read, MCU us */
    uint32_t drain_delay_us;  /* FRAME_READY -> the FIFO drain, MCU us (the old inference's lag) */
    uint16_t read_us;         /* duration of the latch read: the uncertainty on lsm_ticks */
    uint8_t  valid;           /* 0 = the read failed; no stream-13 frame is emitted */
} rs_frame_sync_t;
static rs_frame_sync_t g_frame_sync = { 0 };

/* ---- Task 7: decoupled IMU/env poll rate (SET_IMU_ENV_RATE / GET_IMU_ENV_RATE) --------
 *
 * RS_IMU_ENV_RATE_COUPLED (0, the default): unchanged pre-Task-7 behavior -- the LSM is
 * drained exactly once per ToF frame, at the existing per-frame point below, stream 13
 * always paired with it. A non-zero rate decouples the LSM drain onto its own TIM2-paced
 * schedule (docs/protocol.md cmd 11/12): the drain fires whenever `g_lsm_next_due_us` is
 * reached, independent of the FRAME_READY wait, whether that happens to land at the
 * per-ToF-frame point (a genuinely coincident FRAME_READY edge -- stream 13 IS sent,
 * seq = the just-captured frame counter, same as coupled mode) or during the wait for the
 * NEXT frame (no coincident edge -- stream 13 is skipped and seq is g_last_seq, frozen,
 * exactly the idle loop's own convention). `rs_lsm_decoupled_due()` is the single place
 * that consumes a due tick (advances the schedule), so whichever call site notices it
 * first services it exactly once -- there is no double-draining and no missed tick. */
static uint32_t g_imu_env_rate_hz = RS_IMU_ENV_RATE_COUPLED;
static uint64_t g_lsm_next_due_us = 0;

/* True iff a decoupled drain is due right now, and advances the schedule to the next
 * period if so. Always false in coupled mode (rate 0): coupled mode's LSM drain lives
 * entirely at the per-ToF-frame point below, unconditional, byte-identical to before this
 * feature existed. Locks to whole periods (loops while behind) rather than a single
 * `+= period_us`, so a stretch of lateness (e.g. a slow BUSY command dispatch) re-syncs
 * to the schedule instead of firing a catch-up burst the next time the loop gets a
 * chance to run. */
static bool rs_lsm_decoupled_due(void) {
    if (g_imu_env_rate_hz == RS_IMU_ENV_RATE_COUPLED) {
        return false;
    }
    uint64_t now = rs_time_us();
    if (now < g_lsm_next_due_us) {
        return false;
    }
    uint32_t period_us = 1000000u / g_imu_env_rate_hz;
    do {
        g_lsm_next_due_us += (uint64_t)period_us;
    } while (g_lsm_next_due_us <= now);
    return true;
}

/* Shared LSM6DSV16X FIFO drain + stream 9/10/11 emit -- the logic the idle loop
 * (rs_idle_lsm_tick, ~18.2 Hz) and the active loop's per-ToF-frame point used to
 * duplicate verbatim (Task 7 plan step 1). `seq` is whatever the caller wants stamped on
 * the emitted frames (the just-captured frame counter when this drain is coincident with
 * a ToF frame, the frozen g_last_seq otherwise -- same convention EVENT frames already
 * use). `out_quat_mid_ticks`/`out_quat_n` (NULL-able) receive the drained sample's fields
 * for a caller that also needs to build stream 13 -- ALWAYS written when non-NULL, zeroed
 * on a failed/empty drain, matching rs_lsm_read_latest_raw()'s own contract for those two
 * fields. Returns 0 if anything was sent, <0 if the drain yielded nothing or the LSM
 * never came up (g_lsm_ok == 0).
 *
 * `abortable` (BUG-077): when true, the underlying drain bails out the instant the
 * ToF's FRAME_READY event becomes pending instead of draining the whole FIFO
 * regardless (rs_lsm_read_latest_raw_abortable() -- see its own comment). ONLY
 * rs_wait_frame_ready_svc()'s off-cycle call site below passes true: that is the one
 * call that can otherwise hold the shared I3C bus busy long enough to delay the
 * time-critical DMA-readout kickoff. The idle loop and the coupled/coincident
 * per-ToF-frame call both pass false, unchanged. */
static int rs_lsm_service_tick(uint32_t seq, uint32_t *out_quat_mid_ticks, uint16_t *out_quat_n,
                                bool abortable) {
    if (out_quat_mid_ticks) { *out_quat_mid_ticks = 0u; }
    if (out_quat_n) { *out_quat_n = 0u; }
    if (!g_lsm_ok) {
        return -1;
    }
    rs_lsm_sample_t lsm = { 0 };
    /* One full FIFO's worth (256 words x 8 B = 2 KB); a drain can never yield more.
     * Static, not stack: shared by every call site (idle/off-cycle/per-frame), all on
     * the single main-loop thread -- never concurrent. */
    static rs_lsm_raw_word_t lsm_service_raw[RS_LSM_RAW_FIFO_MAX];
    uint16_t lsm_raw_n = 0;
    int ret = abortable
                  ? rs_lsm_read_latest_raw_abortable(&lsm, lsm_service_raw, (uint16_t)RS_LSM_RAW_FIFO_MAX,
                                                      &lsm_raw_n)
                  : rs_lsm_read_latest_raw(&lsm, lsm_service_raw, (uint16_t)RS_LSM_RAW_FIFO_MAX, &lsm_raw_n);
    if (lsm.have_quat) {
        rs_send_frame_cdc(RS_STREAM_IMU_QUAT, seq, 0u, (const uint8_t *)lsm.quat, RS_IMU_QUAT_SIZE, 0u, 0u);
    }
    if (lsm.have_env) {
        uint8_t env[RS_ENV_SIZE];
        memcpy(env + 0, &lsm.pressure_pa, 4);
        memcpy(env + 4, lsm.mag_ut, 12);
        memcpy(env + 16, &lsm.temp_c, 4);
        rs_send_frame_cdc(RS_STREAM_ENV, seq, 0u, env, RS_ENV_SIZE, 0u, 0u);
    }
    if (lsm_raw_n != 0) {
        rs_send_frame_cdc(RS_STREAM_IMU_RAW, seq, 0u, (const uint8_t *)lsm_service_raw,
                          (uint32_t)lsm_raw_n * RS_IMU_RAW_REC_SIZE, lsm_raw_n, 0u);
    }
    if (out_quat_mid_ticks) { *out_quat_mid_ticks = lsm.quat_mid_ticks; }
    if (out_quat_n) { *out_quat_n = lsm.quat_n; }
    return ret;
}

/* Calibration blob buffer, per-device (VL53L9_CALIB_DATA_SIZE bytes). File-scope rather
 * than a vl53l9_app() stack local (as it was before Task 5) so handle_error()'s bounded
 * recovery (raw-only builds only, see handle_error()'s definition) can hand it straight
 * to rs_sensor_reinit() without threading a pointer down from vl53l9_app()'s stack,
 * across a function (handle_error) that many call sites invoke with no arguments at all.
 * Declared unconditionally (not inside the !CONF_TRANSFORM_ONBOARD guard) so both build
 * modes share one definition; every existing consumer already takes/passes `calib_data`
 * as an explicit parameter and is unaffected by it now living at file scope instead of on
 * vl53l9_app()'s stack. */
static uint8_t calib_data[VL53L9_CALIB_DATA_SIZE];

/* rs_send_event(): builds an EVENT frame (frame_type RS_FRAME_EVENT; payload = u32 code +
 * u32 detail + optional ASCII message, docs/protocol.md) and sends it via the shared
 * generic CDC sender. stream_id/width/height are always 0 for EVENT (ignored fields per
 * the spec). Not-connected sends drop silently through rs_send_generic_cdc's own
 * tud_cdc_connected() gate -- existing, deliberate policy: boot-time events emitted
 * before a host has attached are lost (there is no one to read them), which is fine
 * because the bounded recovery itself -- not the diagnostic -- is what fixes the boot
 * hang; events exist to make an already-recovering device observable, not to guarantee
 * delivery. msg may be NULL for a 0-length message tail (every call site in this task
 * passes NULL: the numeric code+detail carry what a recovery/fault needs, and adding
 * human-readable text is a cheap future addition, not required for this pass). */
static void rs_send_event(uint32_t code, uint32_t detail, const char *msg) {
    uint8_t payload[8 + 64];
    size_t msg_len = msg ? strlen(msg) : 0u;
    if (msg_len > 64u) {
        msg_len = 64u; /* defensive cap matching the fixed local buffer; no call site
                         * in this task passes a message at all */
    }
    rs_put_u32(payload + 0, code);
    rs_put_u32(payload + 4, detail);
    if (msg_len) {
        memcpy(payload + 8, msg, msg_len);
    }
    /* RS_ROUTE_BOTH: unchanged pre-Task-6 behavior. These are rare fault/boot
     * diagnostics (SENSOR_INIT_FAIL, TRIGGER_TIMEOUT, ...), not part of the
     * steady-state DATA cadence the isolation rule exists to protect -- see
     * rs_send_tx_queue_stats_event() below for the one EVENT code that IS
     * isolated. */
    (void)rs_send_generic_cdc(RS_FRAME_EVENT, 0u, g_last_seq, 0u, payload, (uint32_t)(8u + msg_len), 0u, 0u,
                              RS_ROUTE_BOTH);
}

/* Task 6 step 2: periodic Ethernet TX-pacer queue telemetry, sent as EVENT code
 * RS_EVT_TX_QUEUE_STATS (7) on the same 64-frame cadence as the periodic CALIB
 * retransmit (see its call site in the acquisition loop below) -- so a 60 s
 * hardware capture at even the slowest Manual rate (1 fps) still samples it many
 * times, and the hardware gate can read start/end counters off the wire and
 * take the delta ("prove zero, don't infer it") instead of needing VCOM/printf,
 * which is not usable unprivileged on this host.
 *
 * UNLIKE rs_send_event(), this does NOT build an ASCII message tail -- its
 * payload past code+detail is three packed binary u32 counters (see
 * docs/protocol.md's "TX_QUEUE_STATS (EVENT code 7) payload layout"), so it
 * writes the whole 20-byte payload directly rather than routing through
 * rs_send_event()'s strlen()-based message path. Routed through
 * rs_active_data_route() (not RS_ROUTE_BOTH) because it shares the DATA
 * cadence's isolation contract: at >60 fps with an Ethernet target, this must
 * never fall back to a stalling CDC send any more than a RAW frame may. */
static void rs_send_tx_queue_stats_event(uint32_t seq) {
    uint8_t payload[RS_EVT_TX_QUEUE_STATS_LEN];
    uint32_t high_water = ETH_TxQueueHighWater();
    uint32_t pending = ETH_TxPendingFragments();
    /* Coarse hint only (0=none,1=cdc,2=udp) -- the host's authoritative transport
     * truth is computed independently (roomscan.web._transport_kind(), from which
     * physical source class it is actually reading), not from this field. */
    uint32_t active_transport = ETH_HasTarget() ? 2u : (tud_cdc_connected() ? 1u : 0u);
    uint32_t detail = (high_water & 0xFFu) | ((active_transport & 0xFFu) << 8) | ((pending & 0xFFFFu) << 16);
    uint32_t enqueue_drops = ETH_TxEnqueueDrops() + ETH_TxDroppedFrames();

    rs_put_u32(payload + 0, RS_EVT_TX_QUEUE_STATS);
    rs_put_u32(payload + 4, detail);
    rs_put_u32(payload + 8, enqueue_drops);
    rs_put_u32(payload + 12, ETH_TxStackStalls());
    rs_put_u32(payload + 16, ETH_TxEmittedBytes());
    (void)rs_send_generic_cdc(RS_FRAME_EVENT, 0u, seq, 0u, payload, sizeof(payload), 0u, 0u,
                              rs_active_data_route());
}

/* Wait for a platform event in short slices, pumping TinyUSB between slices so
 * USB control transfers are serviced within host timeouts. Safe with the
 * platform event semantics: the ISR-set flag in g_platform_evt persists until
 * platform_acknowledge_event, so an event landing between slices is returned
 * by the next slice immediately. Return convention matches
 * platform_wait_for_event: 0 = event received, non-zero = timeout. */
static int rs_wait_event_usb(uint32_t evt, uint32_t timeout_ms) {
    uint32_t waited = 0;
    for (;;) {
        int ret = platform_wait_for_event(evt, 5);
        /* Stamp BEFORE pumping. platform_wait_for_event busy-waits on the ISR-set flag, so
         * this lands within microseconds of the interrupt; tud_task()/ETH_Process() below
         * can run for hundreds of microseconds and must not be inside the measurement. */
        if (ret == 0) {
            g_evt_stamp_us = rs_time_us();
        }
        tud_task(); ETH_Process();
        if (ret == 0) {
            return 0;
        }
        waited += 5;
        if (waited >= timeout_ms) {
            return ret;
        }
    }
}

/* As rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, timeout_ms), but also services a decoupled
 * IMU/env rate (Task 7) on each 5 ms slice -- this wait IS most of the gap between ToF
 * frames, which a decoupled rate faster than the ToF's own cadence needs to drain into
 * (the per-frame point below only fires once FRAME_READY actually lands). Coupled mode
 * (the default, rate 0) takes an IDENTICAL path to rs_wait_event_usb: rs_lsm_decoupled_due()
 * always returns false immediately, so this function's behavior -- including timing -- is
 * unchanged from before this feature existed. Every off-cycle drain here uses
 * seq = g_last_seq (frozen, no new frame exists yet) and never sends stream 13 -- same
 * convention the idle loop already uses; this drain has no coincident FRAME_READY edge by
 * construction (it only ever runs BETWEEN edges, never AT one). Deliberately a separate
 * function rather than a parameter on rs_wait_event_usb: this decoupled service is specific
 * to the raw-only autonomous loop's own FRAME_READY wait (its call site below) -- the
 * DMA-readout wait and the on-board-transform build's own waits keep calling plain
 * rs_wait_event_usb(), unaffected.
 *
 * BUG-077 fix, two parts, both load-bearing:
 * 1. `if (ret == 0) return 0;` now runs BEFORE the due-drain check, not after. The
 *    previous ordering violated this very comment's "no coincident edge by construction"
 *    claim: if a drain became due in the SAME 5 ms slice FRAME_READY was detected, the
 *    old code ran the (multi-ms, blocking) drain anyway before returning to the caller --
 *    directly delaying the caller's time-critical vl53l9_get_frame_async() kickoff by the
 *    drain's full duration, on top of every other per-frame cost. Checking `ret == 0`
 *    first means a coincident tick is never drained here at all -- it is left due, and
 *    the per-ToF-frame point below (which runs AFTER this frame's own readout is fully
 *    acked, i.e. genuinely idle bus, the same safe timing coupled mode already relies on)
 *    picks it up instead, WITH correct stream-13 pairing since it is now genuinely
 *    coincident rather than misclassified as off-cycle.
 * 2. The remaining off-cycle drain call is now the ABORTABLE variant
 *    (rs_lsm_service_tick(..., abortable=true)): if FRAME_READY becomes pending WHILE an
 *    off-cycle drain that legitimately started earlier (this loop's `ret` was genuinely
 *    non-zero at drain-start) is still running, the drain bails out word-by-word instead
 *    of running to completion regardless -- bounding how much of the sensor's tight
 *    Precision/HFR margin one drain can consume once the next frame is actually ready.
 * See BUGS.md BUG-077 for the measured effect of both parts. */
static int rs_wait_frame_ready_svc(uint32_t timeout_ms) {
    uint32_t waited = 0;
    for (;;) {
        int ret = platform_wait_for_event(PLATFORM_GPIO_IT_EVT, 5);
        if (ret == 0) {
            g_evt_stamp_us = rs_time_us();
        }
        tud_task(); ETH_Process();
        if (ret == 0) {
            return 0;
        }
        if (rs_lsm_decoupled_due()) {
            (void)rs_lsm_service_tick(g_last_seq, NULL, NULL, true);
        }
        waited += 5;
        if (waited >= timeout_ms) {
            return ret;
        }
    }
}

#if !CONF_TRANSFORM_ONBOARD
/* Autonomous acquisition (Task 5): the manual-trigger settle-then-trigger helper that
 * used to live here (rs_trigger_next(), ~2 ms HAL_Delay + vl53l9_trigger_frame() per
 * frame) is GONE from this build. rs_ranging's presets and manual candidates now
 * resolve to VL53L9_SYNC_AUTONOMOUS (rs_ranging.c) -- vl53l9_trigger_frame() itself
 * rejects any call outside VL53L9_SYNC_MANUAL (vl53l9.c:609,
 * VL53L9_ERROR_INVALID_OPERATION), so calling it here would simply fail. Once
 * vl53l9_start() runs (rs_boot_bringup / rs_sensor_reinit / the reconfig-apply path
 * below), the sensor free-runs FRAME_READY interrupts on its own configured
 * frame_period_us with no host action per frame -- realizing a real target FPS at
 * all, per non-negotiable finding #2 (frame_period_us was inert under
 * VL53L9_SYNC_MANUAL). Every former rs_trigger_next() call site below is either
 * deleted outright (nothing to trigger) or, where a genuine FRAME_READY timeout needs
 * bounding, replaced by rs_ranging_frame_timeout_ms()-sized waits with no re-trigger
 * step. Legacy on-board-transform builds (CONF_TRANSFORM_ONBOARD=1) are untouched:
 * that loop still calls vl53l9_trigger_frame() inline, unconditionally, and remains
 * VL53L9_SYNC_MANUAL (plan step 1: "MANUAL_TRIGGER only for the legacy
 * onboard-transform path"). */

/* ---- Host->device command channel (Phase 3 Task 2) --------------------------------
 *
 * Raw-only path only (not the on-MCU-transform golden loop above): the poll point is
 * called once per acquisition-loop iteration, after frame N's readout is fully acked
 * and before anything else touches the sensor for frame N+1 (moved from its original
 * after-the-RAW-send position by Task 4 -- the reconfig safe point needs the sensor
 * genuinely idle, see the call site's ORDERING IS LOAD-BEARING comment; Task 5 removed
 * the manual-trigger step this used to precede, but the safe point itself -- readout
 * acked, nothing else pending -- is unchanged), never from inside rs_wait_event_usb
 * (that primitive stays single-purpose: pump tud_task while waiting on a platform
 * event, nothing else).
 *
 * Backpressure honesty: the RX side never blocks (tud_cdc_available()/tud_cdc_read()
 * are non-blocking), but the TX responses ride the same best-effort rs_cdc_send policy
 * as every DATA frame -- each of its calls can stall up to RS_CDC_STALL_MS (2 ms, #198;
 * was up to 100 ms pre-#198) of NO PROGRESS waiting on a non-draining host before
 * aborting -- a healthy but slow drain is not capped, only a dead one is. Worst case
 * per dispatched command against a genuinely wedged (FIFO-full, non-draining) host: an
 * ACK is 3 rs_cdc_send calls (header/payload/tail, up to ~6 ms of stall); SEND_CALIB
 * adds a CALIB frame first (up to ~12 ms total). With the dispatch cap below (2 per
 * poll), one poll's command handling is bounded at roughly ~24 ms of stall against a
 * wedged host -- a bounded acquisition hiccup, never a deadlock, and identical in kind
 * to what a wedged host already costs the RAW send path. A healthy host drains fast
 * enough that none of these limits are approached (measured: no fps change at ~28 fps).
 *
 * RX accumulation: a small flat buffer (commands are 44 or 48 B depending on shape -- see
 * rs_protocol.h's RS_CMD_FRAME_SIZE_LEGACY/_MANUAL; a handful fit comfortably) with
 * memmove-compaction after each parse step -- simpler than a true ring buffer at this size
 * and call rate (one poll per ~36 ms frame period), and rs_parse_command's contract (see
 * rs_protocol.h) already does the "how much of the front can I discard" reasoning, so the
 * buffer code here only needs to shuffle bytes, not interpret them. Draining and parsing
 * are interleaved (parse after every read chunk) so a burst of back-to-back commands
 * larger than the buffer -- e.g. 3+ commands in one host write, TinyUSB's 256 B RX FIFO
 * holds them fine -- is consumed command-by-command instead of overflowing and losing a
 * valid frame already at the buffer front. */
#define RS_CMD_RX_BUFSIZE (128u)

/* Bounds one poll's worth of command handling (and thus its worst-case TX stall, see
 * block comment above). Anything beyond the cap stays buffered -- in rx_buf and, past
 * that, TinyUSB's RX FIFO -- and is handled on subsequent polls (~36 ms apart). */
#define RS_CMD_MAX_DISPATCH_PER_POLL (2u)

static uint32_t rs_malformed_cmd_count = 0;

/* ---- Runtime reconfig (Phase 3 Task 4) ---------------------------------------------
 *
 * Pending-config pattern: rs_handle_command VALIDATES a reconfig command (bounds /
 * binning checks that never touch the sensor) and, if valid, stores it here instead of
 * acking immediately -- only ONE slot, so a second reconfig command arriving before this
 * one is applied acks BUSY (see rs_handle_command's SET_* / REINIT cases). The actual
 * sensor-touching apply (stop -> reprofile -> restart) runs from
 * rs_apply_pending_config(), called once per main-loop iteration from
 * rs_poll_commands()'s call site (#else / raw-only branch of vl53l9_app()). Task 5:
 * "restart" is now the whole story -- VL53L9_SYNC_AUTONOMOUS means the sensor resumes
 * free-running FRAME_READY on its own the instant vl53l9_start() returns, so there is
 * no longer a trigger-for-N+1 step this apply needs to precede or replace; it simply
 * runs at the loop's one safe point before the next iteration's FRAME_READY wait.
 *
 * Safe-point requirement (empirical, hardware finding from the original manual-trigger
 * design -- kept because the underlying vl53l9_stop() constraint is unchanged by Task 5):
 * vl53l9_stop() must never be called while a trigger/ranging cycle is genuinely in
 * flight. An earlier version of this code applied pending config AFTER the iteration's
 * trigger-for-N+1 call, on the assumption that vl53l9_stop() would cleanly cancel it; on
 * hardware this instead corrupted the sensor's internal ranging state (one good frame
 * post-restart, then FSM_STATE_STREAMING silently dropped back to FSM_STATE_STANDBY with
 * sof_outside_blanking + internal_fw error bits set) -- reproduced even with a
 * same-profile no-op reapply, so it was the stop-while-in-flight itself, not any
 * particular profile field. The apply runs at the one point in the iteration where frame
 * N's own ranging is fully read out (DMA ack complete) and the sensor is genuinely idle
 * -- Task 5 re-validated this same safe point is still correct for
 * VL53L9_SYNC_AUTONOMOUS: nothing is "in flight" to race because there is no separate
 * trigger step at all, only the free-running FSM itself, which vl53l9_stop() is defined
 * to gate on FSM_STATE_STREAMING (vl53l9.c:591-597) regardless of sync mode.
 *
 * The ACK for a pending command is sent from rs_apply_pending_config(), not from
 * rs_handle_command() -- "ACK only after the sensor accepted" per the brief.
 *
 * Task 4 additions: `manual` carries a decoded SET_MANUAL_PARAMS candidate (valid only
 * when cmd == RS_CMD_SET_MANUAL_PARAMS -- every other command ignores it, left
 * zero-initialized by the compound literals that stash them). `transport` records which
 * transport the command arrived on -- Task 6 now uses it (via rs_route_for_transport(),
 * defined near rs_send_generic_cdc above) to route every ACK back to the command's own
 * originating transport instead of broadcasting it over both, independent of the DATA
 * isolation rule rs_active_data_route() applies. `rs_cmd_transport_t` itself is defined
 * up near rs_send_generic_cdc (not here) so that routing code can reference it. */
typedef struct {
    bool pending;
    uint32_t cmd;
    uint32_t param;
    uint32_t token;
    rs_cmd_transport_t transport;
    rs_ranging_manual_params_t manual;
} rs_pending_cmd_t;

static rs_pending_cmd_t rs_pending = { 0 };

/* Standby shadow state (RS_CMD_SET_STANDBY): RS_STANDBY_ACTIVE while streaming,
 * RS_STANDBY_SOFT once vl53l9_stop() has parked the sensor in FSM STANDBY (VCSEL idle),
 * RS_STANDBY_HARD once platform_power_disable() has additionally cut XSHUT. Written and
 * read only from the single main-loop thread (rs_apply_pending_config + the loop-top idle
 * check below), never from an ISR, so no volatile needed -- same discipline as rs_pending.
 * The default of ACTIVE means a device that is never commanded to idle behaves exactly as
 * before this feature: it streams continuously, no behavior change. */
static uint8_t rs_standby_level = RS_STANDBY_ACTIVE;

/* Active profile, persists across reconfig commands so SET_FRAME_PERIOD_US /
 * SET_EXPOSURE_MS compose (each edits a copy of the currently-active profile, never the
 * shared g_ranging_profiles[] table -- vl53l9_utils.h:152) and so REINIT/hard-standby-wake/
 * recovery can re-apply the exact same configuration, DSS included (rs_ranging.h's
 * rs_ranging_profile_t wraps the vendor vl53l9_profile_t with the scanner-owned profile_id
 * and dss_enabled fields -- Task 4). Seeded via rs_ranging_boot_default() (Room Mapping,
 * plan step 6) right before the raw-only loop starts (see vl53l9_app() below); the legacy
 * SET_USECASE/SET_FRAME_PERIOD_US/SET_EXPOSURE_MS commands mutate .vendor fields only (see
 * their handling in rs_apply_pending_config -- they predate DSS-awareness and always force
 * .dss_enabled = 1, matching their exact pre-Task-4 behavior); SET_RANGING_PROFILE/
 * SET_MANUAL_PARAMS replace the whole struct. */
static rs_ranging_profile_t g_active_profile;

/* Last known-good live readback (rs_ranging_read_config()) of the config actually in
 * force -- updated on every successful cmd 8/9/10 apply/read, and seeded at boot. Used as
 * the ACK payload for cmd 9/10 requests that are rejected or BUSY before ever touching
 * hardware (BAD_PARAM/BUSY): docs/protocol.md's ranging-config ACK shape is sent
 * "regardless of result", with "config fields zeroed or holding the prior known-good
 * config" -- this IS that prior known-good config, never the rejected request itself
 * (plan step 3: "never from the requested candidate"). Zero-initialized (matches "config
 * fields zeroed") until the first successful read. */
static rs_ranging_readback_t g_ranging_last_readback = { 0 };

/* Last SET_MANUAL_PARAMS candidate the sensor actually accepted, and whether one exists
 * yet. SET_RANGING_PROFILE's MANUAL (3) reapplies this; rs_handle_command() rejects MANUAL
 * with BAD_PARAM until g_ranging_have_manual is true (docs/protocol.md cmd 8). */
static rs_ranging_profile_t g_ranging_last_manual;
static bool g_ranging_have_manual = false;

/* Pack a vl53l9_status_t (vl53l9.h:124) into one u32 for the ACK's `applied` field on a
 * SENSOR_ERROR result (docs/protocol.md: "applied = status word"): fsm state in the top
 * byte, last command in the next byte, firmware version in the low 16 bits. Not a 1:1
 * encoding of every field (the per-bit error flags and laser_driver[] are dropped) --
 * enough to see "what state was the sensor in" on the host/log side without growing the
 * ACK payload beyond its fixed 12 bytes. */
static uint32_t rs_pack_status(const vl53l9_status_t *s) {
    return ((uint32_t)s->fsm << 24) | ((uint32_t)s->command << 16) | (uint32_t)s->firmware;
}

/* Task 7 plan step 6: the Ethernet TX pacer's drain deadline used to be derived solely
 * from the applied ToF frame period (Task 6). A decoupled IMU/env rate (cmd 11) queues
 * frames through the SAME firmware FIFO off its own cadence, so the deadline must be
 * whichever period reloads the queue fastest -- mirrors
 * roomscan.sources.eth_tx_window_ms()'s variadic "shortest of the governing periods"
 * contract (host/tests/test_sources.py) exactly, just with a fixed arity of two instead
 * of Python's *args. Call this instead of ETH_SetTxWindowMs(ETH_TxWindowMsForPeriod(...))
 * directly at every call site that used to compute it from g_active_profile alone: the
 * two existing ones (boot bring-up and every successful profile-apply) plus the new
 * SET_IMU_ENV_RATE apply below -- whichever of the two periods just changed. */
static void rs_update_eth_tx_window(void) {
    uint32_t window_ms = ETH_TxWindowMsForPeriod(g_active_profile.vendor.frame_period_us);
    if (g_imu_env_rate_hz != RS_IMU_ENV_RATE_COUPLED) {
        uint32_t imu_period_us = 1000000u / g_imu_env_rate_hz;
        uint32_t imu_window_ms = ETH_TxWindowMsForPeriod(imu_period_us);
        if (imu_window_ms < window_ms) {
            window_ms = imu_window_ms;
        }
    }
    ETH_SetTxWindowMs(window_ms);
}

/* FSM_STATE_STANDBY's numeric value (vl53l9.c's private _fsm_state_t enum: NONE=0,
 * READY_TO_BOOT=1, STANDBY=2, STREAMING=3) -- that enum has no public header, but
 * vl53l9_status_t.fsm (vl53l9_get_status(), a PUBLIC call) is a bare uint8_t carrying
 * the exact same register value, confirmed on-target (Task 5: a stop-then-status
 * probe read back 3 immediately after a successful vl53l9_stop(), i.e. still
 * FSM_STATE_STREAMING -- see rs_wait_standby()'s own comment). Mirrored here as a
 * local constant rather than re-declaring the vendor's private enum. */
#define RS_FSM_STATE_STANDBY_VALUE (2u)

/* Bound for rs_wait_standby()'s poll, used at the profile-apply safe point (this is a
 * one-time cost per reconfigure command, not a per-frame cost, so a generous margin
 * over the observed settle time -- typically 1-2 ms on-target -- costs nothing in
 * steady-state cadence). */
#define RS_RANGING_STOP_SETTLE_TIMEOUT_MS (50u)

/* Bounded settle wait for vl53l9_stop() (Task 5 hardware finding, plan step 3's
 * vl53l9_stop()-safety investigation). vl53l9_stop() returning success only means
 * COMMAND_STOP_STREAM was accepted -- its own _write_cmd() implementation
 * (vl53l9.c:891-915) polls the COMMAND register until it clears, NOT the FSM state
 * register, and those are not the same instant. Reproduced on hardware (2026-08-03):
 * immediately calling rs_ranging_write_profile() after a successful vl53l9_stop()
 * failed VL53L9_ERROR_INVALID_STATE on vl53l9_set_sync_mode()'s own
 * FSM_STATE_STANDBY gate (vl53l9_utils_set_profile()'s first setter call) -- a status
 * probe taken at that exact point read fsm=3 (STREAMING), not yet settled. The
 * restore-to-old-profile attempt a few instructions later hit the identical race and
 * ALSO failed, so profile-apply commands got NO ack at all (neither write ever
 * reached an ack-sending line) and silently funneled into handle_error()'s bounded
 * recovery on every single attempt -- 6/6 reproduced with a manual SET_MANUAL_PARAMS
 * switch and separately with a same-value SET_RANGING_PROFILE reapply, so this is not
 * specific to any one candidate profile.
 *
 * Specific to VL53L9_SYNC_AUTONOMOUS (Task 5): under the old VL53L9_SYNC_MANUAL
 * design this race was apparently never observed (Task 4's hardware gate exercised
 * the identical stop-then-reprofile sequence repeatedly) -- plausible mechanism,
 * though not confirmed at the register level (out of this task's reach without
 * vendor-internal documentation): a manual-sync sensor only ranges between explicit
 * triggers, so at any moment stop() is issued it is very likely already idle between
 * cycles; an autonomous-sync sensor is continuously scheduling its next frame
 * on-chip, so interrupting that scheduler mid-cycle plausibly takes measurably
 * longer to fully settle to FSM_STATE_STANDBY. Whatever the exact mechanism, the fix
 * is the same either way: prove STANDBY via the PUBLIC vl53l9_get_status() readback
 * before writing profile fields, rather than trusting vl53l9_stop()'s return alone.
 * A fixed guessed delay was deliberately rejected -- polling is both faster in the
 * common case (most settles observed within 1-2 ms) and correct in the worst case
 * (bounded by timeout_ms rather than hoping a guessed constant is always enough).
 *
 * Lives in vl53l9_app.c, not rs_ranging.c/.h, because rs_ranging.h documents itself
 * as HAL-free (register I/O + wire constants only) and this loop's pacing needs
 * HAL_Delay(), which every other settle wait in this file already uses. Returns 0
 * once FSM_STATE_STANDBY is observed, the first vl53l9_get_status() error
 * encountered, or VL53L9_ERROR_TIMEOUT if it never settles within timeout_ms. */
static int rs_wait_standby(vl53l9_device_t *p_dev, uint32_t timeout_ms) {
    for (uint32_t waited = 0;; waited++) {
        vl53l9_status_t status = { 0 };
        int ret = vl53l9_get_status(p_dev, &status);
        if (ret) {
            return ret;
        }
        if (status.fsm == RS_FSM_STATE_STANDBY_VALUE) {
            return 0;
        }
        if (waited >= timeout_ms) {
            return VL53L9_ERROR_TIMEOUT;
        }
        HAL_Delay(1);
    }
}

/* ---- Multi-device I3C dynamic address assignment (IKS4A1 HUB1 native-I3C bus) ------
 *
 * Replaces platform_assign_dynamic_address() (platform_utils.c, read-only reference --
 * never edited in place per CLAUDE.md) for boards where the IKS4A1's LSM6DSV16X (HUB1)
 * shares I3C1 with the ToF as a genuine I3C target -- see docs/iks4a1-stacking.md. The
 * reference's single-device function hardcodes "whoever answers ENTDAA first is the ToF,
 * address 0x52" and registers only one device; with two real I3C arbiters that either
 * assigns the wrong device to 0x52 or leaves the second unmanaged, which is why the boot
 * sequence hung with both stacked (2026-07-09 bench session).
 *
 * Discriminates by PID.PartID, a stable 16-bit per-device value MEASURED and confirmed on
 * hardware via the iks4a1_i3c_probe() diagnostic above (commit 43f42b9 / this plan's
 * Task 1) with both devices stacked in HUB1-only jumper config:
 *   ToF (VL53L9CX):  PartID 0x0102, MODEL_ID 0x394C3353 -> keeps 0x52 (VL53L9_DEFAULT_ADDRESS)
 *   LSM6DSV16X:      PartID 0x0070, WHO_AM_I 0x70       -> assigned 0x50
 * NB: the plan's original PID.MIPIID discriminator is degenerate on this hardware (identical
 * BCR=0x07, near-identical MIPIID); PartID is the reliable key. A PartID that matches neither
 * makes this bail with -2 rather than misconfigure the bus. */
#define TOF_PART_ID                (0x0102) /* VL53L9CX, MODEL_ID 0x394C3353 -- measured Task 1 */
#define IKS4A1_LSM6DSV16X_PART_ID  (0x0070) /* LSM6DSV16X, WHO_AM_I 0x70 -- measured Task 1 */
#define IKS4A1_LSM6DSV16X_I3C_ADDR (0x50)   /* dynamic address for the LSM6DSV16X; avoids 0x52
                                             * (ToF) and every IKS4A1 static address (0x1E/0x38/
                                             * 0x5C/0x5D/0x6A/0x6B) per docs/iks4a1-stacking.md */

static int rs_assign_dynamic_addresses(void) {
    HAL_StatusTypeDef status;
    uint64_t payload;
    I3C_DeviceConfTypeDef dev_conf[2];
    uint8_t nb_configured = 0;

    /* Slow-PP for ENTDAA: the IKS4A1 NXS0108 auto-direction translator can't pass 12.5 MHz I3C
     * push-pull, so the ToF (behind the 53L9A1 shifter) drops from ENTDAA when stacked. At slow
     * PP it enumerates 100% (diagnosed: 105/105 passes). OD kept at the reference ~1 MHz. */
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0xff;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0xff;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    /* Multi-device ENTDAA is a race: with the IKS4A1 stacked, the always-on LSM6DSV16X can win
     * arbitration and the HAL reports enumeration "complete" (HAL_OK) after the LSM alone --
     * before the ToF (just released from its XSHUT reset) joins -- leaving the ToF unaddressed
     * and boot dead (observed: "1 responder(s)", LSM only). It worked at session start only
     * because the ToF happened to win the race. RSTDAA_THEN_ENTDAA resets EVERY dynamic address
     * each pass, so we can safely re-run the whole enumeration until the ToF appears. The ToF is
     * mandatory; the IKS4A1 is optional, so we gate on the ToF, not on a device count. */
    uint8_t tof_seen = 0;
    for (int attempt = 0; attempt < 6 && !tof_seen; attempt++) {
        nb_configured = 0;
        do {
            payload = 0;
            status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
            if (status == HAL_BUSY) {
                I3C_ENTDAAPayloadTypeDef pinfo = { 0 };
                HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &pinfo);
                uint32_t bcr = __HAL_I3C_GET_BCR(payload);

                uint8_t address;
                if (pinfo.PID.PartID == TOF_PART_ID) {
                    address = VL53L9_DEFAULT_ADDRESS;
                    tof_seen = 1;
                } else if (pinfo.PID.PartID == IKS4A1_LSM6DSV16X_PART_ID) {
                    address = IKS4A1_LSM6DSV16X_I3C_ADDR;
                } else {
                    return -2; /* unrecognized device answered ENTDAA -- bail rather than guess */
                }

                HAL_I3C_Ctrl_SetDynAddr(&hi3c1, address & 0x7F);

                if (nb_configured < 2) {
                    dev_conf[nb_configured].DeviceIndex = (uint8_t)(nb_configured + 1);
                    dev_conf[nb_configured].TargetDynamicAddr = address & 0x7F;
                    dev_conf[nb_configured].IBIAck = __HAL_I3C_GET_IBI_CAPABLE(bcr);
                    dev_conf[nb_configured].IBIPayload = __HAL_I3C_GET_IBI_PAYLOAD(bcr);
                    dev_conf[nb_configured].CtrlRoleReqAck = __HAL_I3C_GET_CR_CAPABLE(bcr);
                    dev_conf[nb_configured].CtrlStopTransfer = DISABLE;
                    nb_configured++;
                }
            }
        } while (status == HAL_BUSY);

        if (status != HAL_OK) {
            return -3;
        }
        if (!tof_seen) {
            HAL_Delay(20); /* give the ToF a beat to (re)join, then RSTDAA + re-enumerate */
        }
    }

    /* If the ToF never answered (genuinely absent, e.g. board removed for LSM-only bring-up), we
     * still proceed: whatever DID enumerate is addressed below, and the shipping boot path fails
     * loudly downstream at vl53l9_init() if the ToF is required. This keeps LSM-only diagnostics
     * (CONF_LSM_PROBE) working while the retry above still maximises the ToF's chances when stacked. */

    /* Steady-state (ranging) timing. Ranging reads are also I3C push-pull, so they must also run
     * slow enough for the NXS0108 -- start at the ENTDAA-proven floor (0xff) to confirm streaming,
     * then tune PP up toward max sustainable fps. OD (0x59) for any legacy-I2C traffic. */
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x0a;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x59;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    if (nb_configured > 0 && HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, dev_conf, nb_configured) != HAL_OK) {
        return -4;
    }

    return 0;
}

/* Full sensor re-init cycle, SELF-CONTAINED through to a running stream: reset -> I3C
 * address -> init -> calib re-read -> apply the CURRENT g_active_profile (autonomous
 * sync included) -> start -> settle -> stale-event clear. Mirrors vl53l9_app()'s own
 * pre-loop setup sequence (reset/platform_assign_dynamic_address/vl53l9_init/
 * vl53l9_get_calib_data/vl53l9_utils_set_profile/vl53l9_start, above) so REINIT is a
 * faithful "do the boot sequence again" rather than a partial reset -- and this is
 * exactly the sequence Task 5's bounded-retry recovery needs, hence factored out as a
 * standalone callable. The post-start tail (settle + event-ack) lives INSIDE this
 * function deliberately: it is the safety envelope for the stale-event hardware bug
 * documented below, and any future caller (Task 5's recovery path) must inherit it
 * structurally rather than having to know to replicate it. On success the sensor is
 * already streaming (VL53L9_SYNC_AUTONOMOUS: vl53l9_start() alone puts the FSM into a
 * free-running FRAME_READY cadence at g_active_profile's frame_period_us -- no trigger
 * of any kind is needed or possible here, see the block comment above this file's
 * "Autonomous acquisition" section) -- the caller resumes the normal
 * wait-for-GPIO-event loop directly.
 *
 * Stale-event hazard (empirical, Task 4 hardware finding): platform_power_reset()
 * toggles XSHUT (platform_utils.c:75-81) and platform_assign_dynamic_address() re-inits
 * the I3C peripheral -- either can put a spurious edge on the sensor's interrupt line
 * that the EXTI ISR latches into g_platform_evt with no real frame behind it. Left
 * uncleared, the main loop's next rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, ...) consumes
 * that stale flag immediately and vl53l9_get_frame_async() correctly reports
 * VL53L9_ERROR_INVALID_STATE (vl53l9.c:706-711: FRAME_READY register reads 0) --
 * reproduced on hardware (under the original manual-trigger design): the re-init and
 * seeded trigger both succeeded, then the very next frame read failed this way and the
 * loop's retry budget exhausted into handle_error(). Acknowledging both events right
 * before returning ensures the loop's next wait can only be satisfied by a genuinely
 * new edge -- still correct under autonomous sync, where the "next edge" is the
 * sensor's own free-running cadence rather than anything this function triggers.
 *
 * calib_data is written in place, and the CALLER RETRANSMITS it over CDC after a
 * successful return (calibration may have changed across a physical reset) -- every
 * caller honors this: rs_apply_pending_config()'s two direct call sites send CALIB
 * explicitly, and rs_recover() retransmits on its success path so all
 * handle_error()-driven recoveries inherit it (see its comment).
 * Returns 0 on success, the first non-zero vl53l9_error on failure (VL53L9_ERROR_* per
 * vl53l9.h:47-53). This is deliberate (recursion guard, see rs_recover()'s comment):
 * this function must never call handle_error(), because handle_error()'s own recovery
 * loop is what calls this function. */
static int rs_sensor_reinit(vl53l9_device_t *p_dev, uint8_t *calib_data) {
    int ret;

    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        int daa_ret = rs_assign_dynamic_addresses();
        if (daa_ret) {
            return daa_ret;
        }
    }

    ret = vl53l9_init(p_dev);
    if (ret) {
        return ret;
    }

    ret = vl53l9_get_calib_data(p_dev, calib_data);
    if (ret) {
        return ret;
    }

    /* Task 4: rs_ranging_write_profile() applies g_active_profile's vendor fields via
     * vl53l9_utils_set_profile() -- INCLUDING .sync (VL53L9_SYNC_AUTONOMOUS, Task 5) --
     * AND its DSS state via rs_ranging_apply_dss() -- both are needed to reproduce the
     * pre-reset config exactly (vl53l9_set_binning(), called from
     * vl53l9_utils_set_profile(), ALWAYS re-enables DSS, so without the second step a
     * REINIT/hard-standby-wake while a >60fps DSS-off profile was active would silently
     * turn DSS back on -- plan step 6). No separate sync-mode override follows this call
     * (Task 5 removed it): the profile's own .sync field is authoritative. */
    ret = rs_ranging_write_profile(p_dev, &g_active_profile);
    if (ret) {
        return ret;
    }

    ret = vl53l9_start(p_dev);
    if (ret) {
        return ret;
    }

    /* Post-start tail (the safety envelope -- see the function comment): settle margin
     * matching the pre-loop boot sequence's HAL_Delay(50), then clear any stale latched
     * events from the reset. Nothing to trigger (autonomous free-run) -- success. */
    HAL_Delay(50);
    platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);
    platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);
    return 0;
}

/* ---- Bounded sensor recovery (Phase 3 Task 5) --------------------------------------
 *
 * Recursion guard, spelled out (the residual flagged in Task 4's review; Task 5 removed
 * the seed-trigger step this guard was originally written around, but the guard itself
 * still matters -- see below): the naive version of this feature has handle_error() call
 * rs_sensor_reinit() to recover, and (under the pre-Task-5 manual-trigger design)
 * rs_sensor_reinit()'s own tail called rs_trigger_next() to seed frame 1; had
 * rs_trigger_next() dead-ended into handle_error() on failure, a sensor that keeps coming
 * up trigger-broken would recurse handle_error -> rs_sensor_reinit -> rs_trigger_next ->
 * handle_error -> rs_sensor_reinit -> ... without bound (each level consuming stack,
 * never unwinding). Fixed structurally, not by convention, and the fix generalizes to
 * Task 5's autonomous design unchanged: rs_sensor_reinit() returns an ordinary error code
 * and never calls handle_error() itself. rs_recover() below is the ONLY function that
 * calls rs_sensor_reinit() in a retry loop, and rs_recover() itself never calls
 * handle_error() or itself -- it is the bottom of this call chain, not a link in it.
 *
 * Resume-the-loop design: handle_error() (below) either (a) recovers via rs_recover()
 * and returns normally, or (b) exhausts recovery, disconnects, and spins forever -- it
 * never "returns false" or signals failure some other way. Every call site in the
 * raw-only loop that used to read `if (ret) handle_error();` and fall through becomes
 * `if (ret) { handle_error(); continue; }` (or an equivalent flag-then-continue for
 * sites nested inside an inner retry loop) -- the `continue` restarts the OUTERMOST
 * while(1) iteration from its top, deliberately abandoning whatever local state (a
 * partially retried wait, a parsed frame, a pending command) belonged to the pre-fault
 * sensor generation. This is safe because rs_sensor_reinit() (which the recovery calls)
 * leaves the sensor ALREADY STREAMING (VL53L9_SYNC_AUTONOMOUS: vl53l9_start() alone is
 * enough, no seed trigger exists to fail) and both stale platform events acknowledged
 * (its own safety envelope) -- exactly the state the top of the raw-only loop expects
 * when it begins by waiting on PLATFORM_GPIO_IT_EVT. A command that was
 * mid-apply when the fault hit gets no ACK; the host's CommandClient times out and the
 * user can retry -- simpler and safer than trying to reconstruct a coherent ACK for a
 * config change that may not have taken effect on the now-fully-reset sensor.
 *
 * Event semantics (a judgment call, documented here because it reads as a deviation from
 * docs/protocol.md's event-code table): the table lists SENSOR_INIT_FAIL's detail as "vl53l9
 * status word", written before this task pinned actual emission. This task's own brief is
 * more specific for the recovery loop ("EVENT per attempt, code SENSOR_INIT_FAIL, detail =
 * attempt#") and that is what's implemented below -- detail is the 1-based attempt number,
 * not a packed status word. docs/protocol.md is updated alongside this change to match. */
static int rs_recover(void) {
    for (int attempt = 1; attempt <= 5; attempt++) {
        uint32_t backoff_ms = 100u << (attempt - 1); /* 100, 200, 400, 800, 1600 ms */
        HAL_Delay(backoff_ms);
        int ret = rs_sensor_reinit(&device[CONF_DEVICE_ID], calib_data);
        if (ret == 0) {
            /* Streaming again (autonomous free-run, no trigger involved) -- rs_sensor_reinit
             * also re-read calib_data across the physical reset, so RETRANSMIT it here
             * (rs_sensor_reinit's contract: the caller owning the recovery retransmits).
             * Doing it INSIDE rs_recover, not at handle_error()'s or any other caller's
             * level, means every recovery path inherits it structurally (same
             * self-contained-envelope principle as rs_sensor_reinit's post-start tail) --
             * without this, a handle_error()-driven recovery would leave the host
             * deprojecting with possibly-stale calibration for up to 63 frames until the
             * periodic CALIB cadence fires. Width/height are derived from the active
             * profile's binning (fixed at 2 -> 54x42) so this function stays
             * parameter-free for its no-context caller. seq = g_last_seq: no new frame
             * exists yet post-reinit, and the next RAW's counter is unknowable here (the
             * sensor's counter restarts across the reset); the last captured seq is the
             * spec-consistent stand-in, same as EVENT frames use. */
            uint8_t w = 0, h = 0;
            vl53l9_utils_get_resolution(g_active_profile.vendor.binning, &w, &h);
            rs_send_frame_cdc(RS_STREAM_CALIB, g_last_seq, 0u, calib_data,
                              VL53L9_CALIB_DATA_SIZE, w, h);
            return 0;
        }
        rs_send_event(RS_EVT_SENSOR_INIT_FAIL, (uint32_t)attempt, NULL);
    }
    return -1; /* 5 attempts exhausted */
}

/* Boot bring-up (Task 5): reset -> I3C address -> init -> calib -> profile-apply
 * (autonomous sync included) -> start -- the full sequence vl53l9_app() used to run
 * inline exactly once, with no error recovery on any step. Its call site in
 * vl53l9_app() wraps this in the SAME bounded-retry shape as rs_recover() (5 attempts,
 * 100/200/400/800/1600 ms backoff), which is what converts the historical ~1-in-5
 * first-power-up failure into a self-healing delay instead of an immediate
 * handle_error() hang.
 *
 * Deliberately NOT rs_sensor_reinit(): that function's post-start tail also clears
 * stale platform events -- correct for REINIT/recovery, where the caller is about to
 * resume the acquisition loop immediately, but wrong here. Boot bring-up runs BEFORE
 * vl53l9_app()'s own buffer allocation, tud_connect(), and DTR-gate sequence (further
 * down, unchanged). This function leaves the sensor already STREAMING under
 * VL53L9_SYNC_AUTONOMOUS (Task 5: vl53l9_start() alone is sufficient -- there is no
 * "trigger frame 1" step to race against those later steps, unlike the pre-Task-5
 * manual-trigger design this comment used to describe) (out_calib_data is written in
 * place, as before).
 *
 * Task 4: `profile` is now an rs_ranging_profile_t (vendor fields + DSS state), applied
 * via rs_ranging_write_profile() so this function's callers -- the raw-only build's
 * initial cold boot AND the hard-standby wake path inside rs_apply_pending_config() --
 * both reproduce DSS correctly, not just the vendor fields (plan step 6). */
static int rs_boot_bringup(vl53l9_device_t *p_dev, uint8_t *out_calib_data,
                           const rs_ranging_profile_t *profile) {
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        int daa_ret = rs_assign_dynamic_addresses();
        if (daa_ret) {
            return daa_ret;
        }
    }

    int ret = vl53l9_init(p_dev);
    if (ret) {
        return ret;
    }

    ret = vl53l9_get_calib_data(p_dev, out_calib_data);
    if (ret) {
        return ret;
    }

    ret = rs_ranging_write_profile(p_dev, profile);
    if (ret) {
        return ret;
    }

    return vl53l9_start(p_dev);
}

/* ACK sender: builds the 12-byte (cmd, result, applied) payload and sends an RS_FRAME_ACK
 * with header seq = the echoed command token (per docs/protocol.md, NOT a frame counter).
 * Best-effort like every other CDC send on this link -- no retry/queue if the host is gone
 * or stalls (bounded at ~6 ms worst case -- 3 rs_cdc_send calls x RS_CDC_STALL_MS -- by
 * rs_cdc_send's per-call no-progress deadline, see the channel block comment above), and
 * RS_FLAG_DROPPED does not apply to control frames (always flags=0).
 * Lives inside the !CONF_TRANSFORM_ONBOARD guard because only the raw-only loop has a
 * command channel; the dual-stream golden loop would leave it unused.
 *
 * `transport` (Task 6 step 3): routes this ACK to the COMMAND's own originating
 * transport only (rs_route_for_transport()), never broadcasting it over both --
 * every call site passes the `transport` it received from rs_handle_command() or
 * rs_pending.transport. */
static void rs_send_ack(uint32_t token, uint32_t cmd, uint32_t result, uint32_t applied,
                        rs_cmd_transport_t transport) {
    uint8_t payload[RS_ACK_PAYLOAD_LEN];
    rs_put_u32(payload + 0, cmd);
    rs_put_u32(payload + 4, result);
    rs_put_u32(payload + 8, applied);
    (void)rs_send_generic_cdc(RS_FRAME_ACK, 0u, token, 0u, payload, sizeof(payload), 0u, 0u,
                              rs_route_for_transport(transport));
}

/* Extended ACK for commands 9 (SET_MANUAL_PARAMS) and 10 (GET_RANGING_CONFIG): the complete
 * applied/readback ranging configuration after cmd+result (docs/protocol.md #ACK), sent
 * regardless of `result` so the ACK shape a host decodes never depends on whether the
 * command succeeded -- only the values do. Task 2 (this commit) is codec/registry only, so
 * every call site today passes zeros with a non-OK result; Task 4 wires real readback.
 * `transport` (Task 6 step 3): same originating-transport routing as rs_send_ack() above. */
static void rs_send_ack_ranging_config(uint32_t token, uint32_t cmd, uint32_t result,
                                       uint8_t ranging_mode, uint32_t frame_period_us,
                                       uint16_t exposure_ms, uint8_t power_mode,
                                       rs_cmd_transport_t transport) {
    uint8_t payload[RS_ACK_RANGING_CONFIG_LEN];
    rs_put_u32(payload + 0, cmd);
    rs_put_u32(payload + 4, result);
    payload[8] = ranging_mode;
    rs_put_u32(payload + 9, frame_period_us);
    payload[13] = (uint8_t)exposure_ms;
    payload[14] = (uint8_t)(exposure_ms >> 8);
    payload[15] = power_mode;
    (void)rs_send_generic_cdc(RS_FRAME_ACK, 0u, token, 0u, payload, sizeof(payload), 0u, 0u,
                              rs_route_for_transport(transport));
}

/* `manual` is non-NULL only when the wire frame that produced this dispatch decoded as
 * RS_PARSED_CMD_MANUAL (rs_protocol.h) -- i.e. only ever for a genuine 12-byte
 * SET_MANUAL_PARAMS payload; every other cmd value ignores it. `transport` records which
 * transport this command arrived on (Task 4: stored on rs_pending for Task 6; not yet used
 * to route the ACK -- see rs_pending_cmd_t's comment). */
static void rs_handle_command(uint32_t cmd, uint32_t param, uint32_t token, const uint8_t *calib_data,
                              uint16_t out_width, uint16_t out_height, uint32_t seq_for_calib,
                              rs_cmd_transport_t transport, const rs_ranging_manual_params_t *manual) {
    switch (cmd) {
    case RS_CMD_PING:
        rs_send_ack(token, cmd, RS_RESULT_OK, RS_PROTO_VERSION, transport);
        break;
    case RS_CMD_SEND_CALIB:
        /* Send a CALIB frame immediately, independent of the periodic 64-frame cadence
         * below (that countdown is left untouched -- it is local `static` state scoped
         * to the send block and simply keeps counting down; this handler doesn't reset
         * it). Rationale: decoupling avoids adding shared mutable state between the
         * command channel and the per-frame send path for a command that is rare and
         * whose only requirement (docs/protocol.md #98) is "device transmits a CALIB
         * frame immediately" -- resetting the countdown as well would be a harmless
         * alternative but buys nothing here and would require lifting that static out
         * of its current block scope. */
        rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data, VL53L9_CALIB_DATA_SIZE,
                          out_width, out_height);
        rs_send_ack(token, cmd, RS_RESULT_OK, 0u, transport);
        break;
    case RS_CMD_SET_USECASE:
        /* Validate WITHOUT touching the sensor: out-of-range id, or an in-range id whose
         * profile doesn't preserve binning 2 (the plan's global constraint -- binning
         * stays fixed at full 54x42 resolution, see docs/superpowers/plans/
         * 2026-07-08-phase3-runtime-config-robustness.md's Global Constraints). Per
         * g_ranging_profiles[] (vl53l9_utils.c:29-66): AR_RANGE/AR_PRECISION are binning
         * 2, AF_RANGE/AF are binning 4 -- so exactly half the usecase table is rejected
         * by design, not a defensive check that never fires.
         *
         * Precedence (all three SET_* cases + REINIT): validation BEFORE the pending
         * check, so an invalid param acks BAD_PARAM/REJECTED_BINNING even while another
         * command is pending -- by design (neither outcome touches the sensor, and the
         * more specific diagnosis wins over the transient BUSY). */
        if (param >= VL53L9_NB_USECASES) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (g_ranging_profiles[param].binning != 2u) {
            rs_send_ack(token, cmd, RS_RESULT_REJECTED_BINNING, g_ranging_profiles[param].binning, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_SET_FRAME_PERIOD_US:
        /* Same bounds vl53l9_set_frame_period() itself enforces (vl53l9.c:402): 10 ms -
         * 1 s. Reject out of range here rather than let the driver call fail later, so a
         * bad param never touches the sensor or consumes the one pending slot. */
        if (param < 10000u || param > 1000000u) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_SET_EXPOSURE_MS:
        /* Same bounds vl53l9_set_exposure() itself enforces (vl53l9.c:550): 1-30 ms
         * (the brief's own guess of "1-100ms" doesn't match the driver -- the profile
         * table's exposure_ms values, 4/5/8/10, all sit comfortably inside 1-30). */
        if (param < 1u || param > 30u) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_REINIT:
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = 0u, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_SET_STANDBY:
        /* Validate the level without touching the sensor (same precedence as the SET_*
         * cases: a bad param acks BAD_PARAM even while another command is pending). The
         * actual stop()/power-down/wake runs from rs_apply_pending_config at the safe
         * point -- vl53l9_stop() must never race an in-flight trigger (see rs_pending's
         * safe-point block comment). */
        if (param > RS_STANDBY_HARD) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    /* Protocol v2 registry (commands 8-12, docs/protocol.md). Task 4 wires real
     * application for 8 (SET_RANGING_PROFILE), 9 (SET_MANUAL_PARAMS), and
     * 10 (GET_RANGING_CONFIG) -- see rs_ranging.h/.c and rs_apply_pending_config() below.
     * 11/12 (SET_IMU_ENV_RATE/GET_IMU_ENV_RATE) are wired in Task 7, just below. */
    case RS_CMD_SET_RANGING_PROFILE:
        /* Same precedence as every other SET_* case: validate without touching the
         * sensor, BEFORE the pending/BUSY check. */
        if (param > RS_PROFILE_MANUAL) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (param == RS_PROFILE_MANUAL && !g_ranging_have_manual) {
            /* docs/protocol.md cmd 8: "MANUAL (3) reapplies the last accepted
             * SET_MANUAL_PARAMS candidate and is rejected until one exists." */
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_GET_RANGING_CONFIG:
        /* Read-only, but still routed through the one-pending-slot/safe-point discipline
         * (plan step 4) so its I3C reads never race the acquisition loop's own bus use --
         * rs_apply_pending_config() special-cases it (no stop/start, see there). */
        if (rs_pending.pending) {
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_BUSY, g_ranging_last_readback.ranging_mode,
                                       g_ranging_last_readback.frame_period_us,
                                       g_ranging_last_readback.exposure_ms,
                                       g_ranging_last_readback.power_mode, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = 0u, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_SET_MANUAL_PARAMS: {
        /* `manual` is NULL only if a malformed/adversarial frame claimed cmd=9 in the
         * LEGACY 8-byte shape (rs_parse_command's `kind` is derived from payload_len, not
         * from `cmd`) -- treat that the same as any other invalid candidate. */
        uint32_t vr = manual ? rs_ranging_validate_manual(manual) : RS_RESULT_BAD_PARAM;
        if (vr != RS_RESULT_OK) {
            rs_send_ack_ranging_config(token, cmd, vr, g_ranging_last_readback.ranging_mode,
                                       g_ranging_last_readback.frame_period_us,
                                       g_ranging_last_readback.exposure_ms,
                                       g_ranging_last_readback.power_mode, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_BUSY, g_ranging_last_readback.ranging_mode,
                                       g_ranging_last_readback.frame_period_us,
                                       g_ranging_last_readback.exposure_ms,
                                       g_ranging_last_readback.power_mode, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .token = token,
                                         .transport = transport, .manual = *manual };
        break;
    }
    case RS_CMD_SET_IMU_ENV_RATE:
        /* Legacy cmd+param ACK shape (RS_ACK_PAYLOAD_LEN), `applied` = the accepted rate_hz
         * -- same precedence as every other SET_* case: validate without touching anything
         * (this command never touches the sensor at all, see rs_apply_pending_config()
         * below), BEFORE the pending/BUSY check. 0 (coupled) is always valid; an explicit
         * rate above RS_IMU_ENV_RATE_MAX_HZ (480, the XL/GY/SFLP ODR ceiling) is rejected --
         * the >60 Hz sensor-hub-cycle mismatch (stream 10 sub-sampling) is reported, not
         * rejected, by the host's own profiles.validate_imu_env_rate() (plan step 4: quat/
         * raw can still hit the requested rate, only env sub-samples). */
        if (param > RS_IMU_ENV_RATE_MAX_HZ) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, g_imu_env_rate_hz, transport);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, g_imu_env_rate_hz, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token,
                                         .transport = transport };
        break;
    case RS_CMD_GET_IMU_ENV_RATE:
        /* Read-only, but still routed through the one-pending-slot/safe-point discipline
         * (plan step 5), same reasoning as GET_RANGING_CONFIG above -- independent of any
         * ranging-profile command sharing the same slot, never requiring one. */
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, g_imu_env_rate_hz, transport);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = 0u, .token = token,
                                         .transport = transport };
        break;
    default:
        rs_send_ack(token, cmd, RS_RESULT_UNKNOWN_CMD, 0u, transport);
        break;
    }
}

/* Applies a pending reconfig command, if any, at THIS iteration's safe point (see the
 * block comment on rs_pending above for why here is safe). Sends the deferred ACK for
 * whatever command was pending, then clears the slot. seq_for_calib carries the current
 * frame's counter, reused as the seq on a REINIT's re-sent CALIB frame (no "next frame"
 * counter exists yet at this point in the iteration).
 *
 * Return value (Task 5): true means a fault occurred mid-apply and handle_error()'s
 * bounded recovery already got the sensor back to a known-good streaming state (frame 1
 * already triggered inside rs_sensor_reinit) -- the caller MUST `continue` its own
 * while(1) loop immediately rather than fall through to code that assumes the command
 * completed (see the resume-the-loop design comment above rs_recover()). false means
 * normal completion (success or a cleanly-handled/acked failure) -- the caller proceeds
 * as usual. handle_error() itself never "returns false" (it recovers and returns, or
 * spins forever) -- every `handle_error(); return true;` pair below is this function's
 * half of that contract. A command abandoned via `return true` gets no ACK: the host's
 * CommandClient times out and the user can retry, simpler and safer than reconstructing
 * a coherent ACK for a config change that may not have taken effect on the now-fully-
 * reset sensor. */
static bool rs_apply_pending_config(vl53l9_device_t *p_dev, uint8_t *calib_data, uint16_t out_width,
                                    uint16_t out_height, uint32_t seq_for_calib) {
    if (!rs_pending.pending) {
        return false;
    }

    uint32_t cmd = rs_pending.cmd;
    uint32_t param = rs_pending.param;
    uint32_t token = rs_pending.token;
    rs_cmd_transport_t transport = rs_pending.transport; /* Task 6: route the deferred ACK
                                                             * back to whichever transport
                                                             * this command actually arrived
                                                             * on (see rs_send_ack()'s doc) */
    rs_ranging_manual_params_t manual = rs_pending.manual; /* only meaningful when
                                                              * cmd == RS_CMD_SET_MANUAL_PARAMS;
                                                              * copied before freeing the slot */
    rs_pending.pending = false; /* single in-flight slot: free it before any hardware call below */

    if (cmd == RS_CMD_REINIT) {
        int ret = rs_sensor_reinit(p_dev, calib_data);
        if (ret) {
            /* Full re-init failed outright (not a "restore the old profile" situation --
             * there is no known-good state to fall back to short of trying again).
             * handle_error() runs its own bounded recovery (a FRESH rs_recover() cycle,
             * independent of this failed attempt) and either resumes or never returns. */
            handle_error();
            return true;
        }
        /* rs_sensor_reinit() returned with the sensor genuinely STREAMING (autonomous
         * free-run resumed inside it -- its safety envelope, see its comment),
         * regardless of what rs_standby_level said before this REINIT (a client may
         * legitimately REINIT while the sensor is soft/hard-parked -- rs_handle_command's
         * REINIT case never checks rs_standby_level). Same fix as the profile-apply
         * vl53l9_stop()-failure branch below, and the same bug class: without this, a
         * REINIT issued while parked leaves rs_standby_level stuck at its stale
         * SOFT/HARD value and the loop-top idle check silently starves RAW/CALIB/
         * stream-13 from here on even though the hardware is genuinely active again.
         * All that's left here: re-send CALIB (it may have changed across the physical
         * reset; unconditional, independent of the periodic 64-frame cadence in the
         * caller, same rationale as RS_CMD_SEND_CALIB above) and ack. */
        rs_standby_level = RS_STANDBY_ACTIVE;
        rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data, VL53L9_CALIB_DATA_SIZE, out_width,
                          out_height);
        rs_send_ack(token, cmd, RS_RESULT_OK, 0u, transport);
        return false;
    }

    if (cmd == RS_CMD_SET_STANDBY) {
        /* Laser-wear idle. Applied here at the safe point (frame N read out, nothing
         * triggered) so every vl53l9_stop() below is race-free -- the same requirement
         * the profile-reconfig path relies on. rs_standby_level is the shadow of the FSM
         * power state; keep it and the hardware in lockstep on every arm.
         *
         * A wake sets rs_standby_level = ACTIVE *before* the hardware calls, on purpose:
         * the post-condition of any wake attempt that RETURNS (rather than spinning) is
         * "sensor streaming" (autonomous free-run resumes on its own the instant
         * vl53l9_start() returns -- no trigger step to wait on) -- either the clean path
         * here or, on fault, handle_error()'s recovery via rs_sensor_reinit(). Flipping
         * the shadow first keeps it truthful even when a fault forces `return true`, so
         * the loop-top idle check does not strand a now-streaming sensor in the idle
         * branch. */
        uint8_t from = rs_standby_level;
        uint8_t to = (uint8_t)param; /* handler validated 0..RS_STANDBY_HARD */

        if (to == from) {
            rs_send_ack(token, cmd, RS_RESULT_OK, to, transport); /* idempotent no-op, no HW touched */
            return false;
        }

        if (to == RS_STANDBY_ACTIVE) {
            rs_standby_level = RS_STANDBY_ACTIVE; /* optimistic -- see block comment */
            if (from == RS_STANDBY_SOFT) {
                /* Sensor stayed configured, just parked in STANDBY: restart resumes the
                 * autonomous free-run directly (no trigger step). Post-restart settle +
                 * stale-event clear mirror the reconfig path's tail (cheap insurance even
                 * though a clean stop() left no reset edges). */
                if (vl53l9_start(p_dev)) {
                    /* BUG-074 investigation (2026-08-04): rs_standby_level is ALREADY
                     * correct here (flipped above, before this call) -- this is NOT the
                     * shadow-desync BUG-072 fixed (confirmed by reading the b10f44d diff:
                     * this optimistic-set predates BUG-072 and its own writeup credits it
                     * with incidentally repairing ANOTHER path's desync). The bug this
                     * branch DID have: falling straight into handle_error()'s bounded
                     * recovery with NO ack sent, unlike every sibling stop/start-failure
                     * branch in this same function (the profile-apply path above, and the
                     * to==SOFT/HARD arm below), which ack a SENSOR_ERROR BEFORE attempting
                     * recovery. A silent failure here reproduces the observed symptom
                     * exactly -- SET_STANDBY(wake=0) timing out with no ACK at all, up to
                     * the host's full timeout, while REINIT (which also acks OK on success
                     * but at least always tries the same recovery) "fixed" it because it
                     * was issued as a fresh, separate command that got its OWN chance to
                     * ack. Ack SENSOR_ERROR here too, matching the sibling idiom, then
                     * best-effort recover via the heavier rs_sensor_reinit() (vl53l9_start()
                     * alone already failed once, so escalate straight to the full
                     * re-bring-up rather than retrying the same call). */
                    vl53l9_status_t status = { 0 };
                    vl53l9_get_status(p_dev, &status);
                    rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status), transport);
                    if (rs_sensor_reinit(p_dev, calib_data)) {
                        handle_error();
                        return true;
                    }
                    rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data,
                                      VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                    return false; /* already acked above */
                }
                HAL_Delay(50);
                platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);
                platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);
            } else {
                /* from HARD: XSHUT was low, so a full re-bring-up is required (reset ->
                 * re-address -> init -> calib -> start, all inside rs_sensor_reinit's
                 * safety envelope; autonomous free-run resumes with no trigger step).
                 * calib may have changed across the physical reset -- retransmit it, same
                 * as the REINIT path above. */
                if (rs_sensor_reinit(p_dev, calib_data)) {
                    /* Same BUG-074 fix as the SOFT arm above: ack before handle_error()'s
                     * recovery instead of leaving the host to time out silently. This IS
                     * already the heaviest recovery step, so there is no lighter fallback
                     * to try first -- ack, then let handle_error() take it from here. */
                    vl53l9_status_t status = { 0 };
                    vl53l9_get_status(p_dev, &status);
                    rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status), transport);
                    handle_error();
                    return true;
                }
                rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data,
                                  VL53L9_CALIB_DATA_SIZE, out_width, out_height);
            }
            rs_send_ack(token, cmd, RS_RESULT_OK, RS_STANDBY_ACTIVE, transport);
            return false;
        }

        /* to == SOFT or HARD: reach a clean, configured FSM-STANDBY baseline (VCSEL
         * already idle) first, then apply whatever power state the target wants. */
        if (from == RS_STANDBY_ACTIVE) {
            /* Streaming -> STANDBY. Safe point guarantees nothing is triggered. */
            if (vl53l9_stop(p_dev)) {
                /* stop() only fails if the sensor already left STREAMING (or the stop
                 * timed out) -- it is not healthily streaming, so a plain return would
                 * dead-end at the next trigger. Best-effort reinit back to a known-good
                 * STREAMING state and REPORT the standby request as failed: we cannot
                 * then stop() (rs_sensor_reinit left frame 1 triggered -- stopping with a
                 * trigger in flight is the exact corruption we forbid), so the device
                 * stays ACTIVE. Mirrors the profile path's identical stop-failure arm. */
                vl53l9_status_t status = { 0 };
                vl53l9_get_status(p_dev, &status);
                rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status), transport);
                if (rs_sensor_reinit(p_dev, calib_data)) {
                    handle_error();
                    return true;
                }
                rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data,
                                  VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                rs_standby_level = RS_STANDBY_ACTIVE;
                return false;
            }
        } else if (from == RS_STANDBY_HARD) {
            /* HARD -> SOFT: power back up and reconfigure to a *parked* STANDBY.
             * rs_boot_bringup (NOT rs_sensor_reinit) is used deliberately: it ends
             * STREAMING with NO trigger in flight, so the vl53l9_stop() immediately below
             * is safe. calib re-read across the reset -> retransmit. */
            if (rs_boot_bringup(p_dev, calib_data, &g_active_profile)) {
                handle_error();
                return true;
            }
            if (vl53l9_stop(p_dev)) {
                handle_error();
                return true;
            }
            rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data,
                              VL53L9_CALIB_DATA_SIZE, out_width, out_height);
        }
        /* from == SOFT falls straight through: already parked in STANDBY. */

        if (to == RS_STANDBY_HARD) {
            platform_power_disable(CONF_DEVICE_ID); /* XSHUT low: fully unpower the VCSEL */
        }
        rs_standby_level = to;
        rs_send_ack(token, cmd, RS_RESULT_OK, to, transport);
        return false;
    }

    if (cmd == RS_CMD_GET_RANGING_CONFIG) {
        /* Read-only: never stops the sensor. Under the pre-Task-5 manual-trigger design
         * this branch had to issue its own trigger-for-N+1 (every other pending command
         * triggered as part of its own stop/restart sequence, and this one never stops
         * the sensor). Task 5: nothing to trigger at all -- the autonomous free-run
         * continues regardless of whether a GET_RANGING_CONFIG landed this iteration. */
        rs_ranging_readback_t rb = { 0 };
        if (rs_ranging_read_config(p_dev, &rb) == 0) {
            g_ranging_last_readback = rb;
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_OK, rb.ranging_mode, rb.frame_period_us,
                                       rb.exposure_ms, rb.power_mode, transport);
        } else {
            /* Transient I3C read failure: nothing was written, so there is nothing to
             * restore -- report SENSOR_ERROR with the last known-good readback rather
             * than invoking bounded recovery for a read-only command. */
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_SENSOR_ERROR,
                                       g_ranging_last_readback.ranging_mode,
                                       g_ranging_last_readback.frame_period_us,
                                       g_ranging_last_readback.exposure_ms,
                                       g_ranging_last_readback.power_mode, transport);
        }
        return false;
    }

    if (cmd == RS_CMD_SET_IMU_ENV_RATE) {
        /* Task 7 plan step 5: never touches the sensor at all -- the LSM's own ODRs
         * (XL/GY/SFLP, rs_lsm.c's RS_LSM_* constants) are unaffected by this command;
         * only the SOFTWARE drain cadence (rs_lsm_decoupled_due()'s schedule) changes.
         * So, unlike every ranging-profile command above, there is no vl53l9_stop()/
         * start() here and no way for this to fail -- it just adopts the
         * already-validated param (rs_handle_command bounds-checked it 0..480) and
         * re-arms the schedule to fire immediately rather than waiting out whatever
         * was left of the OLD period. Still routed through the same pending-slot/
         * safe-point discipline as everything else (plan step 5's "independent of
         * ranging-profile changes" means neither requires the other, not that this
         * skips the discipline). */
        g_imu_env_rate_hz = param;
        g_lsm_next_due_us = rs_time_us(); /* due immediately on the new schedule */
        rs_update_eth_tx_window(); /* plan step 6: budget also depends on this rate now */
        rs_send_ack(token, cmd, RS_RESULT_OK, g_imu_env_rate_hz, transport);
        return false;
    }

    if (cmd == RS_CMD_GET_IMU_ENV_RATE) {
        /* Read-only, no hardware touched -- g_imu_env_rate_hz IS the applied value,
         * there is nothing to read back from the sensor (contrast GET_RANGING_CONFIG,
         * which re-reads the ToF's own registers). */
        rs_send_ack(token, cmd, RS_RESULT_OK, g_imu_env_rate_hz, transport);
        return false;
    }

    /* SET_USECASE / SET_FRAME_PERIOD_US / SET_EXPOSURE_MS (legacy, deprecated
     * diagnostics -- control.py keeps them for one release) / SET_RANGING_PROFILE /
     * SET_MANUAL_PARAMS: build a whole candidate rs_ranging_profile_t (never mutating
     * g_active_profile, g_ranging_profiles[], or rs_ranging's own preset table until
     * the sensor has actually accepted it), stop -> apply -> restart. */
    rs_ranging_profile_t candidate = g_active_profile;
    if (cmd == RS_CMD_SET_USECASE) {
        /* param already bounds- and binning-checked in rs_handle_command; re-reading
         * g_ranging_profiles[param] here (rather than caching it at validation time)
         * costs nothing and keeps the two checks visibly in sync. Legacy path: it
         * predates DSS-awareness, and vl53l9_set_binning() (inside
         * vl53l9_utils_set_profile()) always re-enabled DSS regardless -- forcing it on
         * here keeps this deprecated action byte-identical to its pre-Task-4 behavior
         * (profile_id is left whatever it was: neither this ACK nor GET_RANGING_CONFIG's
         * wire shape carries profile_id, so a stale value here is inert bookkeeping, not
         * a protocol-visible regression). */
        candidate.vendor = g_ranging_profiles[param];
        candidate.dss_enabled = 1u;
    } else if (cmd == RS_CMD_SET_FRAME_PERIOD_US) {
        candidate.vendor.frame_period_us = param;
        candidate.dss_enabled = 1u;
    } else if (cmd == RS_CMD_SET_EXPOSURE_MS) {
        candidate.vendor.exposure_ms = (uint16_t)param;
        candidate.dss_enabled = 1u;
    } else if (cmd == RS_CMD_SET_RANGING_PROFILE) {
        if (param == RS_PROFILE_MANUAL) {
            /* rs_handle_command already rejected this with BAD_PARAM if no manual
             * candidate had ever been accepted -- reaching here means one exists. */
            candidate = g_ranging_last_manual;
        } else {
            /* param already bounds-checked (0..RS_PROFILE_HIGH_FRAMERATE) in
             * rs_handle_command. */
            (void)rs_ranging_preset(param, &candidate);
        }
    } else if (cmd == RS_CMD_SET_MANUAL_PARAMS) {
        /* rs_handle_command already ran rs_ranging_validate_manual() before stashing
         * this candidate as pending -- not re-validated here. */
        rs_ranging_manual_candidate(&manual, &candidate);
    }

    /* vl53l9_utils_set_profile()'s setters all reject anything but FSM_STATE_STANDBY
     * (vl53l9.c: vl53l9_set_sync_mode:385, vl53l9_set_frame_period:399,
     * vl53l9_set_context:424, vl53l9_set_binning:462, vl53l9_set_exposure has no such
     * gate but is meaningless while streaming) -- vl53l9_stop() (vl53l9.c:591) is the
     * STREAMING -> STANDBY transition and is what the vl53l9_utils_set_profile header
     * note (vl53l9_utils.h:127) means by "device must be in standby mode".
     *
     * vl53l9_stop()'s own return alone is NOT sufficient proof of that, though (Task 5
     * hardware finding -- see rs_wait_standby()'s own comment for the full trace): its
     * success only means the STOP command was accepted, not that the FSM register has
     * actually settled to STANDBY yet. Chaining rs_wait_standby() onto a successful
     * stop() folds that settle wait into this same `ret`/`if (ret)` failure handling --
     * a timeout here is handled completely identically to vl53l9_stop() itself
     * failing, which is correct: either way, the device is not provably in a state
     * rs_ranging_write_profile() can safely touch. */
    int ret = vl53l9_stop(p_dev);
    if (ret == 0) {
        ret = rs_wait_standby(p_dev, RS_RANGING_STOP_SETTLE_TIMEOUT_MS);
    }
    if (ret) {
        /* vl53l9_stop() only fails when the sensor has ALREADY left FSM_STATE_STREAMING
         * (its sole gate, vl53l9.c:593 -- INVALID_STATE) or the stop command itself
         * timed out; either way the device is NOT healthily streaming, so returning to
         * the acquisition loop as-is would just dead-end in handle_error() at the next
         * trigger (vl53l9_trigger_frame's own STREAMING gate, vl53l9.c:604). The
         * settle-wait above can also land here (VL53L9_ERROR_TIMEOUT) if the FSM never
         * reaches STANDBY within its bound -- same non-streaming, not-safe-to-write
         * conclusion. Ack the failure, then attempt a full best-effort re-init back
         * onto the previous known-good profile (g_active_profile is still the old
         * profile -- candidate was never applied); only if THAT also fails, fall to
         * the terminal spin (Task 5 upgrades it). */
        vl53l9_status_t status = { 0 };
        vl53l9_get_status(p_dev, &status);
        if (cmd == RS_CMD_SET_MANUAL_PARAMS) {
            /* ranging-config ACK shape has no room for a packed status word -- report
             * the last known-good (pre-request) readback instead, same convention as
             * every other never-touched-hardware SET_MANUAL_PARAMS rejection. */
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_SENSOR_ERROR,
                                       g_ranging_last_readback.ranging_mode,
                                       g_ranging_last_readback.frame_period_us,
                                       g_ranging_last_readback.exposure_ms,
                                       g_ranging_last_readback.power_mode, transport);
        } else {
            rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status), transport);
        }
        if (rs_sensor_reinit(p_dev, calib_data)) {
            /* the direct best-effort reinit above also failed: hand off to
             * handle_error()'s own (fresh) bounded recovery loop */
            handle_error();
            return true;
        }
        /* recovered: sensor streaming again on the old profile (DSS included --
         * rs_sensor_reinit() now applies it via rs_ranging_write_profile()); calib may
         * have changed across the reset.
         *
         * BUG found on hardware during Task 5's vl53l9_stop()-safety testing (plan step
         * 3), fixed here: vl53l9_stop() above can fail for a reason this branch's own
         * comment already names -- "the sensor has ALREADY left FSM_STATE_STREAMING" --
         * and the concrete way that happens in practice is the sensor genuinely being
         * parked by RS_CMD_SET_STANDBY (soft/hard), e.g. the web server's own
         * activity-based auto-idle firing between commands. rs_sensor_reinit() above
         * unconditionally leaves the sensor STREAMING regardless of why it wasn't
         * streaming a moment ago, but this branch never updated rs_standby_level to
         * match -- so the shadow stayed at its stale SOFT/HARD value while the hardware
         * was genuinely ACTIVE again. The loop-top idle check
         * (`if (rs_standby_level != RS_STANDBY_ACTIVE)`) then kept routing every
         * subsequent iteration into the idle branch: RAW_3DMD/CALIB/stream-13 silently
         * stopped being serviced entirely (the FRAME_READY edges the free-running sensor
         * kept producing were never read out) while IMU/env kept flowing at the idle
         * tick's ~18 Hz, and profile-apply commands kept ACKing OK with correct readback
         * throughout -- a state where every command-level probe says healthy while the
         * data plane is silently dead. Reproduced on hardware (2026-08-03): a
         * SET_MANUAL_PARAMS landing while server-auto-idled hit exactly this path, then
         * every RAW_3DMD/CALIB/IMU_SYNC frame went missing for the rest of that session
         * until an explicit SET_STANDBY(ACTIVE) command's OWN optimistic
         * shadow-set-before-hardware-call (see the block comment above rs_pending's
         * `from`/`to` handling) incidentally repaired it. Fix: this reinit path is just
         * as much a "the sensor is now definitely active" event as that SET_STANDBY
         * ACTIVE arm is, so it gets the same shadow update. */
        rs_standby_level = RS_STANDBY_ACTIVE;
        rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data, VL53L9_CALIB_DATA_SIZE, out_width,
                          out_height);
        return false;
    }

    ret = rs_ranging_write_profile(p_dev, &candidate);
    bool applied_ok = (ret == 0);
    rs_ranging_readback_t rb = { 0 };
    int rb_ret;
    if (applied_ok) {
        /* Still in standby (vl53l9_start() below hasn't run yet) -- the plan's "read
         * back... while the device is in standby" point. */
        rb_ret = rs_ranging_read_config(p_dev, &rb);
    } else {
        /* restore the previous (known-good) profile, DSS included, before leaving standby.
         * rs_wait_standby() above already proved the FSM was in STANDBY before this
         * candidate write was attempted, so a failure here is a genuine write error
         * (e.g. an out-of-range register value), not the same settle race. */
        int restore_ret = rs_ranging_write_profile(p_dev, &g_active_profile);
        if (restore_ret) {
            /* double failure: no known-good profile could be re-applied.
             * handle_error()'s bounded recovery is the only way back. */
            handle_error();
            return true;
        }
        rb_ret = rs_ranging_read_config(p_dev, &rb);
    }

    /* .sync (VL53L9_SYNC_AUTONOMOUS, Task 5) was already written by
     * rs_ranging_write_profile() above -- candidate's on success, g_active_profile's on
     * restore -- so no separate sync-mode call is needed here (Task 5 removed the
     * unconditional VL53L9_SYNC_MANUAL override this used to be). */
    int start_ret = vl53l9_start(p_dev);
    if (start_ret) {
        /* Could not get back to streaming at all (neither candidate nor restored
         * profile). handle_error()'s bounded recovery is the only way back. */
        handle_error();
        return true;
    }

    /* Post-restart settle + stale-event clear -- see the identical comments on the
     * REINIT path above; same margin, same reasoning, applies here too since both paths
     * call vl53l9_start() (vl53l9_stop()/vl53l9_start() are a less violent transition
     * than a physical reset, but the defensive clear is cheap and this path was where
     * the stop-while-triggered fault originally reproduced under the old manual-trigger
     * design, so it gets the same care). Nothing to trigger afterward: the autonomous
     * free-run resumes on its own the instant vl53l9_start() returns, under whichever
     * profile is now active. */
    HAL_Delay(50);
    platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);
    platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);

    if (rb_ret == 0) {
        /* a genuine readback succeeded -- best-effort: a readback failure alone never
         * demotes a successful write, it just leaves the ACK's config fields (and the
         * shadow) at their prior/zeroed values, matching the documented "config fields
         * zeroed... per the firmware's implementation" ACK convention. */
        g_ranging_last_readback = rb;
    }

    if (!applied_ok) {
        if (cmd == RS_CMD_SET_MANUAL_PARAMS) {
            rs_send_ack_ranging_config(token, cmd, RS_RESULT_SENSOR_ERROR, rb.ranging_mode,
                                       rb.frame_period_us, rb.exposure_ms, rb.power_mode, transport);
        } else {
            /* legacy shape (usecase/period/exposure/profile): applied = packed status
             * word on SENSOR_ERROR, matching docs/protocol.md's general convention --
             * NOT the rejected candidate's id/value. */
            vl53l9_status_t status = { 0 };
            vl53l9_get_status(p_dev, &status);
            rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status), transport);
        }
        return false;
    }

    g_active_profile = candidate; /* adopt only now that the sensor has accepted it */
    /* Task 6: the TX pacer's drain deadline follows whichever period is now active --
     * never the fixed 25/33 ms assumption this replaces (non-negotiable finding #6).
     * Legacy SET_USECASE/SET_FRAME_PERIOD_US/SET_EXPOSURE_MS reach here too (they edit
     * a copy of g_active_profile, same as every other candidate path), so the window
     * tracks the period under every reconfig command, not only cmd 8/9. Task 7: folds in
     * whatever decoupled IMU/env rate is ALSO in force (rs_update_eth_tx_window()) --
     * see its own comment. */
    rs_update_eth_tx_window();

    if (cmd == RS_CMD_SET_RANGING_PROFILE) {
        rs_send_ack(token, cmd, RS_RESULT_OK, candidate.profile_id, transport);
    } else if (cmd == RS_CMD_SET_MANUAL_PARAMS) {
        g_ranging_last_manual = candidate;
        g_ranging_have_manual = true;
        rs_send_ack_ranging_config(token, cmd, RS_RESULT_OK, rb.ranging_mode, rb.frame_period_us,
                                   rb.exposure_ms, rb.power_mode, transport);
    } else {
        /* legacy usecase/period/exposure: applied = the value actually in effect
         * (docs/protocol.md) -- usecase has no driver-side clamping to observe, so echo
         * the id; period/exposure come from the readback just taken above (equivalent to
         * the pre-Task-4 direct vl53l9_get_frame_period/vl53l9_get_exposure calls, now
         * folded into rs_ranging_read_config()). */
        uint32_t applied = param;
        if (cmd == RS_CMD_SET_FRAME_PERIOD_US) {
            applied = rb.frame_period_us;
        } else if (cmd == RS_CMD_SET_EXPOSURE_MS) {
            applied = rb.exposure_ms;
        }
        rs_send_ack(token, cmd, RS_RESULT_OK, applied, transport);
    }
    return false;
}

/* Transport-agnostic byte source for the command poll: fill up to `max` bytes at `dst`,
 * return the count (0 if none) -- the tud_cdc_read contract, so either transport plugs in. */
typedef uint32_t (*rs_cmd_read_fn)(uint8_t *dst, uint32_t max);

static uint32_t rs_cdc_read_cmd(uint8_t *dst, uint32_t max) {
    return tud_cdc_read(dst, max);
}
static uint32_t rs_eth_read_cmd(uint8_t *dst, uint32_t max) {
    return ETH_ReadCommands(dst, max);
}

/* The command parse-drain loop, factored out so USB CDC and Ethernet UDP run the EXACT
 * same wire parser (one copy of rs_parse_command handling -- a second, divergent copy of
 * the command decoder is exactly the bug the protocol-change discipline guards against).
 * Each transport owns its persistent accumulation buffer + length (passed in), so a
 * partial frame straddling two polls survives; `read_fn` is the only per-transport part. */
static void rs_poll_commands_from(rs_cmd_read_fn read_fn, uint8_t *rx_buf, uint32_t *p_rx_len,
                                  const uint8_t *calib_data, uint16_t out_width,
                                  uint16_t out_height, uint32_t seq_for_calib,
                                  rs_cmd_transport_t transport) {
    uint32_t dispatched = 0;

    /* Parse-while-draining: after every chunk read from the source, the parse loop runs
     * to consume completed commands out of the buffer front BEFORE reading more, so a
     * burst larger than the buffer flows through it command-by-command instead of
     * overflowing (the Task 2 review's critical fix -- the old drain-everything-first
     * version wiped a valid buffered command when a 3+-command burst arrived). Outer
     * loop terminates when an iteration makes no progress (nothing read AND nothing
     * consumed) or the dispatch cap is reached. */
    for (;;) {
        bool progressed = false;

        uint32_t space = RS_CMD_RX_BUFSIZE - *p_rx_len;
        if (space > 0) {
            uint32_t got = read_fn(rx_buf + *p_rx_len, space); /* 0 if source empty */
            if (got > 0) {
                *p_rx_len += got;
                progressed = true;
            }
        }

        /* Consume everything parseable right now; rs_parse_command reports exactly how
         * many front bytes to drop each step (full contract in rs_protocol.h). */
        while (*p_rx_len > 0 && dispatched < RS_CMD_MAX_DISPATCH_PER_POLL) {
            rs_parsed_command_t pc;
            int32_t r = rs_parse_command(rx_buf, *p_rx_len, &pc);
            if (r == 0) {
                break; /* candidate pending: wait for more RX bytes */
            }
            uint32_t consume = (uint32_t)((r > 0) ? r : -r);
            if (consume > *p_rx_len) {
                consume = *p_rx_len; /* defensive; rs_parse_command never over-reports */
            }
            if (consume > 0) {
                memmove(rx_buf, rx_buf + consume, *p_rx_len - consume);
                *p_rx_len -= consume;
                progressed = true;
            }
            if (r > 0) {
                /* Legacy shape: pass the scalar param through as before. Manual shape
                 * (Task 4): build the decoded rs_ranging_manual_params_t and pass its
                 * address instead -- rs_handle_command's SET_MANUAL_PARAMS case is the
                 * only one that reads it (every other cmd value ignores the pointer). */
                if (pc.kind == RS_PARSED_CMD_MANUAL) {
                    rs_ranging_manual_params_t m = {
                        .ranging_mode = pc.u.manual.ranging_mode,
                        .frame_period_us = pc.u.manual.frame_period_us,
                        .exposure_ms = pc.u.manual.exposure_ms,
                        .power_mode = pc.u.manual.power_mode,
                    };
                    rs_handle_command(pc.cmd, 0u, pc.token, calib_data, out_width, out_height,
                                      seq_for_calib, transport, &m);
                } else {
                    rs_handle_command(pc.cmd, pc.u.legacy.param, pc.token, calib_data, out_width,
                                      out_height, seq_for_calib, transport, NULL);
                }
                dispatched++;
            } else {
                rs_malformed_cmd_count++;
                if (consume == 0) {
                    break; /* defensive: no forward motion, avoid spinning */
                }
            }
        }

        if (dispatched >= RS_CMD_MAX_DISPATCH_PER_POLL) {
            return; /* cap reached: the rest stays buffered for the next poll */
        }
        if (!progressed) {
            if (*p_rx_len == RS_CMD_RX_BUFSIZE) {
                /* Full buffer the parser cannot advance. Theoretically unreachable: a
                 * full 128 B buffer always yields parser progress (any complete-frame,
                 * false-magic, or no-magic outcome consumes bytes; the only 0-consume
                 * outcome needs len < RS_CMD_FRAME_SIZE_MANUAL (48, the larger of the two
                 * shapes) at a front magic). Kept as a
                 * defensive escape: drop ONE byte past the front (preserving any later
                 * magic candidate, unlike a whole-buffer wipe) and count it. */
                memmove(rx_buf, rx_buf + 1, *p_rx_len - 1u);
                *p_rx_len -= 1u;
                rs_malformed_cmd_count++;
                continue;
            }
            return; /* source drained, nothing parseable left pending */
        }
    }
}

/* USB CDC command poll: drains the TinyUSB RX FIFO. */
static void rs_poll_commands(const uint8_t *calib_data, uint16_t out_width, uint16_t out_height,
                             uint32_t seq_for_calib) {
    static uint8_t rx_buf[RS_CMD_RX_BUFSIZE];
    static uint32_t rx_len = 0;
    rs_poll_commands_from(rs_cdc_read_cmd, rx_buf, &rx_len, calib_data, out_width, out_height,
                          seq_for_calib, RS_CMD_TRANSPORT_CDC);
}

/* Ethernet UDP command poll: drains the eth_cmd_buf the udp_recv callback fills. A
 * separate persistent buffer from the CDC path so the two transports never interleave a
 * half-received frame. Called at the same safe points as rs_poll_commands. */
static void rs_poll_eth_commands(const uint8_t *calib_data, uint16_t out_width, uint16_t out_height,
                                 uint32_t seq_for_calib) {
    static uint8_t rx_buf[RS_CMD_RX_BUFSIZE];
    static uint32_t rx_len = 0;
    rs_poll_commands_from(rs_eth_read_cmd, rx_buf, &rx_len, calib_data, out_width, out_height,
                          seq_for_calib, RS_CMD_TRANSPORT_ETH);
}
#endif /* !CONF_TRANSFORM_ONBOARD */

static void print_frame(float *p_frame, size_t height, size_t width);
static memory_t allocate_memory(uint16_t size);

void vl53l9_app() {

#if CONF_IKS4A1_BUS_PROBE
    iks4a1_bus_probe(); /* never returns -- diagnostic-only entry point */
#endif
#if CONF_IKS4A1_I3C_PROBE
    iks4a1_i3c_probe(); /* never returns -- diagnostic-only entry point */
#endif
#if CONF_LSM_PROBE
    {
        platform_power_reset(CONF_DEVICE_ID);
        int daa = rs_assign_dynamic_addresses(); /* ToF -> 0x52, LSM -> 0x50 */
        HAL_Delay(50);
        int ir = rs_lsm_init();
        printf("\n[LSM PROBE] daa=%d init=%d (0=ok)\n", daa, ir);
        extern uint16_t g_lsm_tag_hist[32];
        extern uint8_t g_lsm_master_config;
        extern uint8_t g_lsm_if_cfg;
        extern uint8_t g_lsm_slv0_add;
        extern uint8_t g_lsm_ctrl7_pre;
        extern uint8_t rs_lsm_shub_status_raw(void);
        printf("[LSM PROBE] CTRL7 as-found=0x%02X (AH_QVAR_EN=bit7 0x80 -> steals SDx/SCx from I2C master)\n",
               g_lsm_ctrl7_pre);
        printf("[LSM PROBE] MASTER_CONFIG=0x%02X (MASTER_ON=bit2 0x04, AUX_SENS_ON=bits1:0, START_CFG=bit5 0x20, WR_ONCE=bit6 0x40)\n",
               g_lsm_master_config);
        printf("[LSM PROBE] IF_CFG=0x%02X (SHUB_PU_EN=bit6 0x40 -> aux-bus pull-up; 0 => Mode-3 aux bus floats) | SLV0_ADD=0x%02X (expect 0xB9)\n",
               g_lsm_if_cfg, g_lsm_slv0_add);
        for (;;) {
            rs_lsm_sample_t s;
            (void)rs_lsm_read_latest(&s);
            printf("[LSM PROBE] tags quat=%u sh0=%u sh1=%u sh2=%u nack=%u | shstat=0x%02X mcfg=0x%02X | P=%d(Pa) T=%d(Cx100) env=%u\n",
                   g_lsm_tag_hist[0x13], g_lsm_tag_hist[0x0E], g_lsm_tag_hist[0x0F],
                   g_lsm_tag_hist[0x10], g_lsm_tag_hist[0x19], rs_lsm_shub_status_raw(),
                   g_lsm_master_config, (int)s.pressure_pa, (int)(s.temp_c * 100.0f), s.have_env);
            HAL_Delay(300);
        }
    }
#endif

    int ret;
#if CONF_TRANSFORM_ONBOARD
    transform_t *p_transform = vl53l9_transform_create();
#endif
    vl53l9_device_t *p_dev = &device[CONF_DEVICE_ID];
    vl53l9_profile_t *p_profile = &g_ranging_profiles[CONF_USECASE];

    /* NOTE: g_ranging_profiles[] (vl53l9_utils.c, read-only reference) already sets
     * frame_period_us = FPS_TO_FRAME_PERIOD(30) for every usecase, AR_PRECISION included --
     * the sensor has been on a 30 fps profile all along. No override needed here.
     * `p_profile` (CONF_USECASE) remains the on-board-transform golden path's own profile
     * source below (#else branch, unchanged) and feeds the buffer-sizing calls just below
     * (binning is 2 for every rs_ranging preset too, so this is never a size mismatch). */

    /* Task 4: Room Mapping is the boot default (plan step 6), independent of CONF_USECASE.
     * Seeded here (file-scope g_active_profile, before the raw-only build's boot sequence)
     * so the raw-only build's cold-boot rs_boot_bringup() call below actually configures the
     * sensor for Room Mapping from power-up, not AR_PRECISION-then-reseeded -- a shadow
     * update alone would leave g_active_profile disagreeing with the hardware until the
     * first reconfig command. g_active_profile/rs_ranging exist only in the raw-only build
     * (the on-board-transform build has no command channel at all -- plan step 8, "by
     * construction" -- so it has nothing to reject high-rate/DSS-off commands FROM). */
#if !CONF_TRANSFORM_ONBOARD
    rs_ranging_boot_default(&g_active_profile);
    /* Task 6: seed the TX pacer's window before the acquisition loop (and hence
     * ETH_Process()/eth_tx_pump()) ever runs, so it is never left at
     * ETH_TX_WINDOW_MS_DEFAULT's boot placeholder once real streaming starts. g_imu_env_rate_hz
     * is still RS_IMU_ENV_RATE_COUPLED here (its static initializer, Task 7), so this is
     * byte-identical to the plain ETH_TxWindowMsForPeriod() call it replaces. */
    rs_update_eth_tx_window();
#endif
    uint16_t raw_buffer_size = 0; /* bytes */
    uint8_t out_width = 0, out_height = 0; /* pixels */
#if CONF_TRANSFORM_ONBOARD
    uint32_t in_width = 0, in_height = 0; /* pixels */
    uint16_t frame_buffer_size = 0;       /* bytes */
#endif
    vl53l9_get_raw_buffer_size(p_profile->binning, &raw_buffer_size);
    vl53l9_utils_get_resolution(p_profile->binning, &out_width, &out_height);
#if CONF_TRANSFORM_ONBOARD
    frame_buffer_size = out_width * out_height * sizeof(float);
#endif

    if (p_profile->binning == 2) {
#if CONF_TRANSFORM_ONBOARD
        in_width = 14842;
        in_height = 1;
#endif
    } else if (p_profile->binning == 4) {
#if CONF_TRANSFORM_ONBOARD
        in_width = 3844;
        in_height = 1;
#endif
    } else {
        /* Unsupported binning: effectively unreachable (CONF_USECASE is compile-time and
         * every table profile is binning 2 or 4). NOTE (raw-only builds): handle_error()'s
         * recovery here runs before the sensor has ever been brought up (g_active_profile
         * is already seeded, above, but nothing has been written to hardware yet) and is
         * guaranteed to fail into the terminal spin ~3 s later -- acceptable for a
         * can't-happen site, documented so it isn't mistaken for a real recovery path. */
        handle_error();
    }

    /* Boot bring-up: reset -> I3C address -> init -> calib -> profile-apply. Raw-only
     * builds also fold sync-mode + start in here (via rs_boot_bringup(), see its
     * comment) and wrap the whole sequence in the same bounded retry as mid-stream
     * recovery (rs_recover()) -- this is what converts the historical ~1-in-5 boot hang
     * (Task 8/prior reports) into a self-healing delay. On-board-transform builds keep
     * the original unretried inline sequence verbatim (sync-mode + start stay at their
     * original position, further down) -- golden-path stability, per the brief. */
#if !CONF_TRANSFORM_ONBOARD
    {
        int boot_ret = -1;
        for (int attempt = 1; attempt <= 5; attempt++) {
            boot_ret = rs_boot_bringup(p_dev, calib_data, &g_active_profile);
            if (boot_ret == 0) {
                break;
            }
            /* Dropped silently: tud_connect() has not run yet at this point in boot (by
             * design -- see its call site further down), so no host is attached to
             * receive this. The retry is the actual fix; the event is a diagnostic for
             * the rare case a debug probe is already watching the CDC port this early. */
            rs_send_event(RS_EVT_SENSOR_INIT_FAIL, (uint32_t)attempt, NULL);
            HAL_Delay(100u << (attempt - 1)); /* 100,200,400,800,1600 ms -- same ladder as
                                                * rs_recover(), placed AFTER the failed
                                                * attempt here (attempt 1 runs immediately
                                                * on a cold boot) vs rs_recover()'s
                                                * delay-BEFORE-each-attempt (a mid-stream
                                                * fault wants settle time before touching
                                                * the sensor again) */
        }
        if (boot_ret) {
            /* 5 attempts exhausted: the sensor will not come up at all. Last resort,
             * matching the legacy immediate-hang contract -- there is no acquisition
             * loop yet to resume. */
            tud_disconnect();
            while (1)
                ;
        }
        /* LSM6DSV16X (IKS4A1 HUB1) is at 0x50 now -- bring up SFLP/sensor-hub. Optional:
         * a failure just means no IMU/env streams; the ToF stream is never blocked. */
        g_lsm_ok = (rs_lsm_init() == 0) ? 1u : 0u;

        /* Seed the ranging-config shadow (best-effort) so a GET_RANGING_CONFIG or a
         * BAD_PARAM/BUSY rejection arriving before any command has ever succeeded reports
         * the sensor's real boot config instead of an all-zero readback. I3C bus is idle
         * here (nothing else runs between boot bring-up and tusb_init()/tud_connect()
         * below); a failure just leaves the shadow zeroed, same as before this call. */
        (void)rs_ranging_read_config(p_dev, &g_ranging_last_readback);
    }
#else
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        platform_assign_dynamic_address();
    }

    ret = vl53l9_init(p_dev);
    if (ret) {
        handle_error();
    }

    ret = vl53l9_get_calib_data(p_dev, calib_data);
    if (ret) {
        handle_error();
    }

    vl53l9_utils_set_profile(p_dev, p_profile);
#endif

#if CONF_TRANSFORM_ONBOARD
    /* initialize processing pipeline */
    ret = transform_initialize(p_transform);
    if (ret) {
        handle_error();
    }

    /* inspect available streams and controls */
    const streams_t *stream_list;
    transform_get_streams(p_transform, &stream_list);
    streams_inspect(stream_list, printf);

    const controls_t *control_list;
    transform_get_controls(p_transform, &control_list);
    controls_inspect(control_list, printf);

    /* set capabilities */

    /**
     * NOTE:
     * setting capabilities is a mandatory step:
     *  - at least one input and one output stream must be set
     *  - input stream must be configured before output ones
     *  - there are no default capabilities, they must be explicitly set
     */

    /* build raw stream capabilities */
    property_t raw_format = { "format", { .val.v_string = "3DMD", .tid = VTID_STRING } };
    property_t raw_width = { "width", { .val.v_uint32 = in_width, .tid = VTID_UINT32 } };
    property_t raw_height = { "height", { .val.v_uint32 = in_height, .tid = VTID_UINT32 } };

    properties_t *raw_props = properties_new(3); /* format, width, height */
    properties_add(raw_props, &raw_format);
    properties_add(raw_props, &raw_width);
    properties_add(raw_props, &raw_height);
    capabilities_t *raw_caps = capabilities_new_simple(&raw_props);

    /* build depth stream capabilities */
    property_t depth_format = { "format", { .val.v_string = "ZF32", .tid = VTID_STRING } };
    property_t depth_width = { "width", { .val.v_uint32 = out_width, .tid = VTID_UINT32 } };
    property_t depth_height = { "height", { .val.v_uint32 = out_height, .tid = VTID_UINT32 } };

    properties_t *depth_props = properties_new(3); /* format, width, height */
    properties_add(depth_props, &depth_format);
    properties_add(depth_props, &depth_width);
    properties_add(depth_props, &depth_height);
    capabilities_t *depth_caps = capabilities_new_simple(&depth_props);

    /* set stream capabilities */
    ret = transform_set_stream_capabilities(p_transform, "raw", raw_caps);
    if (ret) {
        handle_error();
    }

    ret = transform_set_stream_capabilities(p_transform, "depth", depth_caps);
    if (ret) {
        handle_error();
    }

    /* free properties and capabilities (TODO: improve using free functions) */
    properties_free(raw_props, NULL);
    properties_free(depth_props, NULL);
    capabilities_free(raw_caps, NULL);
    capabilities_free(depth_caps, NULL);

    /* set controls */

    /* NOTE: the following control is mandatory and must be set before calling prepare() */
    ret = transform_set_control(p_transform, "calib-buffer", (value_t){ .val.v_ptr = calib_data, .tid = VTID_POINTER });
    if (ret) {
        handle_error();
    }

    /* check pipeline configuration and compute internal parameters required for processing */
    ret = transform_prepare(p_transform);
    if (ret) {
        handle_error();
    }
#endif /* CONF_TRANSFORM_ONBOARD */

    /* allocate memory and initialize buffers (raw data is double buffered) */
    uint8_t raw_mem_index = 0;
    memory_t in_raw_mem[2] = { allocate_memory(raw_buffer_size), allocate_memory(raw_buffer_size) };
#if CONF_TRANSFORM_ONBOARD
    memory_t out_depth_mem = allocate_memory(frame_buffer_size);

    memories_t in_raw_mems = { .items = &in_raw_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };
    memories_t out_depth_mems = { .items = &out_depth_mem, .size = 1, .capacity = 1, .item_size = sizeof(memory_t) };

    stream_buffer_t in_raw_stream_buffer = { .name = "raw", .buffer = { .memories = &in_raw_mems, .nb = 1 } };
    stream_buffer_t out_depth_stream_buffer = { .name = "depth", .buffer = { .memories = &out_depth_mems, .nb = 1 } };

    /* build stream buffers container */
    stream_buffers_t stream_buffers = { .items =
                                            (stream_buffer_t[]){
                                                in_raw_stream_buffer,
                                                out_depth_stream_buffer,
                                            },
                                        .size = 2,
                                        .capacity = 2,
                                        .item_size = sizeof(stream_buffer_t) };
#endif /* CONF_TRANSFORM_ONBOARD */

#if CONF_TRANSFORM_ONBOARD
    /* raw-only builds already did this inside rs_boot_bringup() above (folded in
     * there so the whole boot sequence shares one bounded-retry wrapper) */
    ret = vl53l9_set_sync_mode(p_dev, VL53L9_SYNC_MANUAL);
    if (ret) {
        handle_error();
    }

    ret = vl53l9_start(p_dev);
    if (ret) {
        handle_error();
    }
#endif

    platform_profiler_enable();
    uint32_t start_time = platform_profiler_get_timestamp();
    uint32_t stop_time;
    float frame_rate;

#if CONF_TRANSFORM_ONBOARD
    bool is_first_frame = true;

    uint32_t rs_prev_counter = 0;
    bool rs_have_prev = false;
#endif

    /* Sensor is up and the loop below pumps tud_task(): present the USB
     * device only now (D+ pull-up was held off after tud_init in main.c so
     * the host never saw a device we couldn't answer). */
    tusb_init();
    tud_connect();

#if CONF_STREAM_RAW
    /* Golden-pair captures need frame 1: TNR state is per-pixel and cumulative, so the
     * host must witness the stream from the first processed frame. Hold acquisition
     * until a host opens the CDC port (DTR) OR sends a UDP packet to Ethernet. This gate is also what makes raw-only mode
     * (CONF_TRANSFORM_ONBOARD=0) golden-capture-compatible, so it stays on by default here
     * too; a headless/production build (no PC waiting on the far end) may want to revisit
     * blocking acquisition start on a host connection. */
    while (!tud_cdc_connected() && !(ETH_IsUp() && ETH_HasTarget())) {
        tud_task(); ETH_Process();
    }
    HAL_Delay(50); /* let the host's reader thread settle after opening the port */
#endif

#if CONF_TRANSFORM_ONBOARD
    /* Dual-stream / on-MCU-transform loop: UNCHANGED (golden-pair regeneration path).
     * The raw-only loop with trigger-early overlap lives in the #else branch below. */
    while (1) {

        /* Keep USB serviced every iteration, including frames that skip the
         * send call below (first frame, or a stalled host). */
        tud_task(); ETH_Process();

        /* Trigger the next frame, wait for data-ready, and start the raw readout.
         *
         * NOTE (deviation from the reference app): the reference one-shot
         * handshake (trigger -> wait 1000 ms -> read) is racy on real hardware
         * once nothing throttles the loop. Measured on this board:
         *  - a trigger issued immediately after the previous readout-ack is
         *    intermittently ignored by the sensor (GPIO event never fires);
         *  - the INT falling edge can lead the FRAME_READY register, so an
         *    immediate vl53l9_get_frame_async fails VL53L9_ERROR_INVALID_STATE.
         * The ASCII print in the reference build (~300 ms/frame) masked both.
         * Bounded retries cover them; repeated failure still dies in
         * handle_error(). */
        int rs_attempts = 0;
        for (;;) {
            HAL_Delay(5); /* sensor settle after previous readout-ack; a trigger
                           * issued back-to-back with the ack is ignored */
            ret = vl53l9_trigger_frame(p_dev);
            if (ret) {
                handle_error();
            }

            ret = rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, 1000);
            if (ret) {
                /* no edge seen: either the trigger was lost or the edge landed
                 * after the timeout -- poll FRAME_READY to disambiguate */
                uint8_t rs_is_ready = 0;
                (void)vl53l9_poll_frame(p_dev, &rs_is_ready);
                if (!rs_is_ready) {
                    if (++rs_attempts > 3) {
                        handle_error();
                    }
                    continue; /* trigger lost: re-trigger (no event to ack) */
                }
                /* frame is ready: fall through and ack, clearing any edge that
                 * arrived between the timeout and the poll so it cannot leak
                 * into the next iteration as a spurious event */
            }
            platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);

            /* grab raw data from sensor and fill input buffer */
            ret = vl53l9_get_frame_async(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
            if (ret == VL53L9_ERROR_INVALID_STATE) {
                /* early edge: FRAME_READY not visible yet, give it a moment */
                if (++rs_attempts > 8) {
                    handle_error();
                }
                HAL_Delay(1);
                ret = vl53l9_get_frame_async(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
            }
            if (ret) {
                handle_error();
            }
            break;
        }

        /* process the previous frame while the sensor is acquiring the next one */
        if (is_first_frame) {
            is_first_frame = false;
        } else {
#if CONF_TRANSFORM_ONBOARD
            /* TODO: find a better way to handle this, maybe leveraging mems list */
            in_raw_mems.items = &in_raw_mem[(raw_mem_index + 1) % 2];
            ret = transform_process_stream(p_transform, &stream_buffers);
            if (ret) {
                handle_error();
            }
#endif
#if CONF_STREAM_BINARY
            if (rs_have_prev) {
#if CONF_STREAM_RAW
                /* Ordering constraint: this block runs (after transform_process_stream, when the
                 * transform is on-MCU, so depth for rs_prev_counter is valid) and before
                 * raw_mem_index toggles at the bottom of the loop. The raw buffer being read here
                 * is in_raw_mem[(raw_mem_index + 1) % 2] -- when the transform runs, the same
                 * buffer transform_process_stream just consumed above via in_raw_mems.items,
                 * holding rs_prev_counter's raw frame; when raw-only, it is simply the buffer the
                 * previous loop iteration's DMA filled, which parse_frame below still hasn't
                 * touched this iteration. The sensor DMA in progress this iteration targets
                 * in_raw_mem[raw_mem_index] (the *other* buffer, kicked off earlier this iteration
                 * by vl53l9_get_frame_async). The buffer read here IS the next iteration's DMA
                 * target (raw_mem_index toggles at loop-bottom), but that next DMA kick cannot
                 * start until this iteration finishes -- and this send is synchronous, completing
                 * before loop-bottom. So reading it here is race-free. */
                {
                    static uint32_t rs_calib_countdown = 0;
                    if (rs_calib_countdown == 0) {
                        rs_send_frame_cdc(RS_STREAM_CALIB, rs_prev_counter, 0u, calib_data,
                                          VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                        rs_calib_countdown = 64;
                    }
                    rs_calib_countdown--;
                    /* raw buffer of the frame being processed = the PREVIOUS index (the pipeline
                     * input, or -- raw-only -- simply the previously captured frame); send it with
                     * the same seq as the depth it produces (or would have, on-MCU) */
                    rs_send_frame_cdc(RS_STREAM_RAW_3DMD, rs_prev_counter, 0u,
                                      (const uint8_t *)in_raw_mem[(raw_mem_index + 1) % 2].data,
                                      raw_buffer_size, out_width, out_height);
                }
#endif
#if CONF_TRANSFORM_ONBOARD
                rs_send_frame_cdc(RS_STREAM_DEPTH_ZF32, rs_prev_counter, 0u, (const uint8_t *)out_depth_mem.data,
                                  frame_buffer_size, out_width, out_height);
#endif
            }
#endif
        }

        ret = rs_wait_event_usb(PLATFORM_I3C_DMA_RX_EVT, 1000);
        if (ret) {
            handle_error();
        }
        platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);

        ret = vl53l9_get_frame_async_ack(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
        if (ret) {
            handle_error();
        }

        /* TODO: to be moved below but avoid printing for first frame */
        vl53l9_frame_t frame = { 0 };
        ret = vl53l9_utils_parse_frame(in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size, &frame);
        if (ret) {
            handle_error();
        }

        rs_prev_counter = (uint32_t)frame.p_metadata->frame_counter;
        rs_have_prev = true;

        /* measure frame rate */
        stop_time = platform_profiler_get_timestamp();
        frame_rate = (1.0f / (float)(platform_profiler_convert_to_us(stop_time - start_time))) * 1000000;
        start_time = stop_time;
#if !CONF_STREAM_BINARY && CONF_TRANSFORM_ONBOARD
        /* legacy ASCII debug path: only meaningful with an on-board transform
         * (it renders out_depth_mem, which does not exist off-board) */
        print_frame((float *)out_depth_mem.data, out_height, out_width);
        printf("Processed frame n. %lu @ %u fps\n", (unsigned long)frame.p_metadata->frame_counter,
               (unsigned int)frame_rate);
#endif

        /* swap raw buffer index for next frame acquisition */
        raw_mem_index = (raw_mem_index + 1) % 2;
    }

#else /* !CONF_TRANSFORM_ONBOARD */

    /* Raw-only autonomous acquisition loop (Task 5; supersedes the Phase 2.5 Task 4
     * "trigger-early overlap" design this comment used to describe).
     *
     * Every rs_ranging preset/manual candidate now resolves to VL53L9_SYNC_AUTONOMOUS
     * (rs_ranging.c) and boot bring-up (above) has already called vl53l9_start() --
     * the sensor is ALREADY free-running FRAME_READY at g_active_profile's
     * frame_period_us before this loop's first iteration, with no host trigger of any
     * kind, ever (vl53l9_trigger_frame() itself rejects a call outside
     * VL53L9_SYNC_MANUAL -- vl53l9.c:609). This is what makes frame_period_us -- and
     * therefore a real target FPS -- effective at all; non-negotiable finding #2 of the
     * plan was that it stayed inert under the old manual-trigger design no matter what
     * value was written. The steady-state shape per iteration:
     *
     *   GPIO wait (pumped, timeout sized from the applied period) -> ack -> LSM-clock
     *   latch -> DMA kick(N) -> DMA wait (pumped) -> readout ack(N) -> parse metadata(N)
     *   -> command poll / safe-point reconfig (may stop/reprofile/restart the sensor,
     *   still autonomous afterward) -> send CALIB-cadence + RAW(N) + IMU/env/sync(N) ->
     *   loop.
     *
     * There is no separate "trigger N+1" step to position early or late anymore: the
     * sensor's own cadence is independent of how long this loop takes to process and
     * send frame N. If this loop's per-iteration work (dominated by the RAW send, ~15 ms
     * observed at 30 fps) exceeds the applied frame period, iterations simply fall
     * behind the sensor's free-running cadence -- there is no software knob here that
     * changes that; Task 5's hardware gate is precisely a test of whether that happens
     * at 60/90/100 fps, see the report.
     *
     * parse BEFORE send (unchanged from the prior design): vl53l9_utils_parse_frame is
     * pure pointer arithmetic over the raw buffer -- no bus traffic (vl53l9_utils.c:
     * 149-179) -- so it can run any time after the buffer is complete. It must run after
     * the readout ack (the metadata lives at buffer_size - sizeof(vl53l9_meta_t), i.e.
     * in the tail segment vl53l9_get_frame_async_ack retrieves), and running it before
     * the send lets the wire seq be frame N's OWN frame_counter.
     *
     * Buffer safety (unchanged from the prior design): the send reads
     * in_raw_mem[raw_mem_index] -- the SAME buffer this iteration's DMA filled --
     * strictly after the DMA-done wait and readout ack for it completed. No DMA is in
     * flight during the send at all: the next DMA is kicked only in the next iteration
     * (after the GPIO wait), and it targets in_raw_mem[raw_mem_index ^ 1] because
     * raw_mem_index toggles at loop bottom. Single-buffer semantics would therefore
     * suffice in this mode, but the double buffer is KEPT: the allocation is shared with
     * the dual-stream loop above, which genuinely needs it (its DMA of N overlaps its
     * processing/send of N-1).
     *
     * First-frame edge: no seed trigger exists to run before this loop (Task 5 removed
     * it) -- the sensor is already producing frames from boot bring-up's vl53l9_start().
     * Iteration 1 captures frame 1 (whatever the sensor produced first) completely
     * before anything is sent, so every frame -- including frame 1, which golden
     * captures need for TNR alignment -- is sent, and the CALIB countdown (initial value
     * 0) fires before the first RAW send exactly as before.
     *
     * No CONF_STREAM_BINARY/CONF_STREAM_RAW guards inside this loop: the #error at the
     * top of the file guarantees both are 1 whenever CONF_TRANSFORM_ONBOARD is 0. */

    /* g_active_profile is already Room Mapping, live and free-running on the sensor
     * (rs_ranging_boot_default() + rs_boot_bringup(), above) -- no re-seed needed here.
     * Later SET_USECASE/PERIOD/EXPOSURE/SET_RANGING_PROFILE/SET_MANUAL_PARAMS commands
     * mutate this copy, never g_ranging_profiles[]/rs_ranging's own preset table. */

    while (1) {

        rs_boot_heartbeat();    /* yellow LD2 blinking == this loop is turning over */

        /* Keep USB serviced every iteration, even when waits below return fast. */
        tud_task(); ETH_Process();

        /* Laser-wear idle (RS_CMD_SET_STANDBY). While parked in standby the sensor is not
         * ranging, so there is no frame-ready edge coming -- entering the wait cycle below
         * would just block on rs_wait_event_usb's (period-derived) timeout every iteration
         * and emit spurious TRIGGER_TIMEOUT events. Instead keep the transport + command
         * channel alive and re-loop; the wake command arrives here, and
         * rs_apply_pending_config's wake arm restarts ranging (start/reinit; autonomous
         * free-run resumes with no trigger) and flips rs_standby_level
         * back to ACTIVE, at which point the next iteration falls through to the normal
         * cycle. seq_for_calib = g_last_seq (last captured counter) per the EVENT-frame /
         * recovery convention -- no new frame exists while idled. */
        if (rs_standby_level != RS_STANDBY_ACTIVE) {
            rs_poll_commands(calib_data, out_width, out_height, g_last_seq);
            rs_poll_eth_commands(calib_data, out_width, out_height, g_last_seq); /* UDP wake path */

            /* Idle-loop IMU/env service (2026-08-03). "Idle" only means the TOF LASER is
             * parked -- the LSM6DSV16X is a separate chip on the shared I3C bus, unaffected by
             * vl53l9_stop()/platform_power_disable(), and keeps sampling at its own ODR the
             * whole time regardless of whether anything drains it. Two things ride this one
             * tick, nominally every 17 idle-loop iterations (a ~34 ms guess at the ~30 Hz
             * active-path rate off the 2 ms HAL_Delay below) -- ON-RIG MEASURED at 18.2 Hz
             * (~55 ms/tick) instead, since the per-tick command-poll + I3C drain + 3 sends
             * cost more than the naive tick-count math assumed. Still a clean, stable,
             * genuinely useful cadence for "orientation/env while parked" -- not re-tuned to
             * hit exactly 30 Hz, since nothing here requires that specific number:
             *
             *   1. Drain + stream IMU_QUAT/ENV/IMU_RAW exactly as the active path does, so a
             *      host still gets orientation/env data while the sensor is parked (only the
             *      laser is meant to idle, per owner: "we only want to idle the laser, not the
             *      IMU/env"). seq = g_last_seq (frozen -- no new ToF frame exists, same
             *      convention EVENT frames already use) and t_us = the live clock
             *      (rs_stamp_us() falls back to it automatically since g_frame_stamp_us stays
             *      disarmed here) -- both correct: device_hz/host_hz are computed from t_us
             *      deltas and arrival timing, never from seq (see docs/protocol.md).
             *      IMU_SYNC (stream 13) is skipped: its whole meaning is "where THIS ToF
             *      frame's FRAME_READY edge sits on the LSM clock", which doesn't exist here.
             *
             *   2. Poll WAKE_UP_SRC (rs_lsm_check_wake_up) on the SAME tick -- it's one more
             *      register in a transaction already happening, and the wake latency benefit
             *      (up to ~55 ms instead of a separate slower poll) is free. Only arms the wake
             *      if nothing else already claimed the single rs_pending slot this iteration (a
             *      real host command already means the sensor is about to wake anyway).
             *      token=0 is a fire-and-forget synthetic command: no host issued it, so no
             *      host is awaiting its ACK -- an unmatched token is a silent no-op on the
             *      receiving end (CommandClient.offer()), same as any other unsolicited frame. */
            static uint16_t rs_idle_lsm_tick = 0;
            if (g_lsm_ok && ++rs_idle_lsm_tick >= 17u) { /* ~34 ms at the 2 ms idle cadence below */
                rs_idle_lsm_tick = 0;

                /* Task 7: shared with the active loop's per-frame/decoupled-off-cycle
                 * service (rs_lsm_service_tick()) -- identical drain + stream 9/10/11
                 * emit this used to duplicate inline. No coincident FRAME_READY edge
                 * exists while idled, so no stream 13, exactly as before. abortable=false
                 * (BUG-077): the ToF is stopped while idled, so there is no FRAME_READY
                 * deadline to protect here -- unchanged, full drain every time. */
                (void)rs_lsm_service_tick(g_last_seq, NULL, NULL, false);

                if (!rs_pending.pending) {
                    uint8_t rs_wake_src = 0;
                    if (rs_lsm_check_wake_up(&rs_wake_src) > 0) {
                        rs_send_event(RS_EVT_AUTO_WAKE_MOTION, (uint32_t)rs_wake_src, NULL);
                        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = RS_CMD_SET_STANDBY,
                                                         .param = RS_STANDBY_ACTIVE, .token = 0u };
                    }
                }
            }

            if (rs_pending.pending) {
                (void)rs_apply_pending_config(p_dev, calib_data, out_width, out_height, g_last_seq);
                /* Return value ignored on purpose: a fault mid-wake already ran
                 * handle_error()'s recovery (sensor streaming again) and the wake arm
                 * set rs_standby_level = ACTIVE up front, so the next iteration simply
                 * takes the normal path -- no fault-resume bookkeeping needed here. */
            }
            HAL_Delay(2); /* gentle idle cadence: keep CPU/USB calm while the laser rests */
            continue;
        }

        /* Wait for data-ready. Task 5: the sensor is free-running (VL53L9_SYNC_AUTONOMOUS)
         * -- there is no "trigger lost" case anymore, only "no FRAME_READY within budget"
         * (a genuine fault: a wedged sensor, a dropped I3C transaction) or "the edge landed
         * between the timeout firing and this check" (a race, not a fault -- poll
         * FRAME_READY directly and, if it's actually ready, fall through and ack, clearing
         * any late edge so it cannot leak into the next iteration). The wait timeout is
         * sized from the CURRENTLY APPLIED frame period (plan step 5) rather than a fixed
         * 1000 ms: at 90/100 fps a fixed 1000 ms window would silently swallow ~90-100
         * missed frames before ever declaring a fault, and at manual's 1 fps floor it would
         * be too tight for genuinely healthy hardware. See rs_ranging_frame_timeout_ms(). */
        int rs_attempts = 0;
        bool rs_fault_recovered = false; /* set when handle_error() ran and recovered
                                           * (never left true across a handle_error()
                                           * that exhausts -- that path never returns) --
                                           * checked right after the loop below to
                                           * `continue` the OUTER while(1) from a clean
                                           * iteration instead of trusting any state
                                           * computed in this one (see the design
                                           * comment above rs_recover()). A plain
                                           * `continue` inside this inner for(;;) cannot
                                           * reach the outer loop directly in C, hence
                                           * the flag. */
        /* The instant frame N's data became ready -- i.e. the sensor's FRAME_READY edge,
         * which is the end of its integration window and therefore the physical time the
         * depth samples describe. This, not the send-time clock, is what goes in the
         * frame's t_us (armed below, just before the sends). Everything between here and
         * the sends -- DMA readout, metadata parse, the command poll, the transmit itself
         * -- is variable-latency work that used to fold into the stamp and show up as
         * skew against the IMU clock. */
        uint64_t rs_ready_us = 0;
        uint32_t rs_wait_timeout_ms = rs_ranging_frame_timeout_ms(g_active_profile.vendor.frame_period_us);
        for (;;) {
            /* rs_wait_frame_ready_svc(), not plain rs_wait_event_usb(): this wait is the
             * gap a decoupled IMU/env rate (Task 7) needs to drain into off-cycle -- see
             * that function's own comment. Coupled mode (the default) is unaffected. */
            ret = rs_wait_frame_ready_svc(rs_wait_timeout_ms);
            rs_ready_us = g_evt_stamp_us; /* stamped inside the wait, at the interrupt */
            if (ret) {
                uint8_t rs_is_ready = 0;
                (void)vl53l9_poll_frame(p_dev, &rs_is_ready);
                rs_ready_us = rs_time_us(); /* recovery path: no edge was seen, so the best
                                             * available stamp is "when the poll found it" */
                if (!rs_is_ready) {
                    if (++rs_attempts > 3) {
                        rs_send_event(RS_EVT_TRIGGER_TIMEOUT, (uint32_t)rs_attempts, NULL);
                        handle_error();
                        rs_fault_recovered = true;
                        break;
                    }
                    /* AUTONOMOUS: nothing to re-trigger -- the sensor free-runs on its
                     * own schedule regardless of what this loop does. Just keep waiting,
                     * bounded by rs_attempts above; a real fault still terminates into
                     * handle_error() within a small, period-scaled number of attempts. */
                    continue;
                }
            }
            platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);

            /* Put frame N's FRAME_READY edge on the LSM's clock while the shared I3C bus is
             * still idle -- see the rs_frame_sync_t comment. Costs one 4-byte register read
             * before the DMA kick below; a failure only suppresses this frame's stream 13
             * (absence means "unknown", exactly like stream 12), it never touches the ToF
             * path. Deliberately NOT retried: a late latch is worse than no latch. */
            g_frame_sync.valid = 0u;
            if (g_lsm_ok) {
                uint32_t lsm_ticks = 0u;
                uint64_t t_pre = rs_time_us();
                if (rs_lsm_read_timestamp(&lsm_ticks) == 0) {
                    uint64_t t_post = rs_time_us();
                    uint64_t t_mid = t_pre + ((t_post - t_pre) / 2u);
                    g_frame_sync.lsm_ticks = lsm_ticks;
                    g_frame_sync.latch_delay_us =
                        (uint32_t)((t_mid > rs_ready_us) ? (t_mid - rs_ready_us) : 0u);
                    g_frame_sync.read_us = (uint16_t)((t_post - t_pre) > 0xFFFFu
                                                          ? 0xFFFFu : (t_post - t_pre));
                    g_frame_sync.drain_delay_us = 0u; /* filled in at the drain, below */
                    g_frame_sync.valid = 1u;
                }
            }

            /* kick the DMA readout of frame N into this iteration's buffer */
            ret = vl53l9_get_frame_async(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
            if (ret == VL53L9_ERROR_INVALID_STATE) {
                /* early edge: FRAME_READY not visible yet, give it a moment (Task 8) */
                if (++rs_attempts > 8) {
                    handle_error();
                    rs_fault_recovered = true;
                    break;
                }
                HAL_Delay(1);
                ret = vl53l9_get_frame_async(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
            }
            if (ret) {
                handle_error();
                rs_fault_recovered = true;
                break;
            }
            break;
        }
        if (rs_fault_recovered) {
            continue; /* resume the outer while(1) from a clean iteration */
        }

        ret = rs_wait_event_usb(PLATFORM_I3C_DMA_RX_EVT, rs_wait_timeout_ms);
        if (ret) {
            rs_send_event(RS_EVT_DMA_TIMEOUT, 1u, NULL); /* single period-scaled wait, no
                                                            * internal retry at this
                                                            * point -- detail is a
                                                            * constant attempt count */
            handle_error();
            continue;
        }
        platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);

        ret = vl53l9_get_frame_async_ack(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
        if (ret) {
            handle_error();
            continue;
        }

        /* parse frame N's metadata (pure in-memory reads; buffer complete after the
         * readout ack above) so the send below carries frame N's own counter */
        vl53l9_frame_t frame = { 0 };
        ret = vl53l9_utils_parse_frame(in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size, &frame);
        if (ret) {
            handle_error();
            continue;
        }
        uint32_t rs_counter = (uint32_t)frame.p_metadata->frame_counter;
        g_last_seq = rs_counter; /* EVENT frames from here on carry this as their seq */

        /* Command-channel poll point: after frame N's DMA readout is fully acked (no I3C
         * transaction in flight) so RX draining and any reconfig it decides on run with
         * the bus idle. RX never blocks; response TX is best-effort with bounded
         * worst-case stalls against a wedged host (capped at RS_CMD_MAX_DISPATCH_PER_POLL
         * dispatches, ~24 ms ceiling -- was ~1.2 s pre-#198, see the channel block
         * comment). PING and
         * SEND_CALIB ack immediately inside; SET_USECASE/SET_FRAME_PERIOD_US/
         * SET_EXPOSURE_MS/REINIT only validate and stash a pending request (rs_pending)
         * here -- applied below.
         *
         * ORDERING IS LOAD-BEARING (empirical, Task 4 hardware finding, re-validated for
         * Task 5's autonomous design): under the ORIGINAL manual-trigger design,
         * rs_poll_commands()/rs_apply_pending_config() ran AFTER that iteration's
         * trigger-for-N+1 call, on the theory that vl53l9_stop() would simply cancel
         * whatever trigger was already in flight. On hardware this corrupted the
         * sensor's internal ranging state instead: EVERY reconfig (including a same-
         * profile no-op re-apply -- isolated by testing SET_USECASE 1 while usecase 1
         * was already active) captured exactly one good frame post-restart, then the
         * NEXT trigger failed with VL53L9_ERROR_INVALID_STATE (-3) because the sensor
         * had autonomously dropped itself from FSM_STATE_STREAMING back to
         * FSM_STATE_STANDBY, with vl53l9_status_t.error.sof_outside_blanking = 1 and
         * .error.internal_fw = 1 (a firmware-detected internal fault, not a bad register
         * write -- every driver call in the apply sequence itself returned 0/success).
         * Root cause: vl53l9_stop() while ranging is genuinely in flight is not a clean
         * cancel. The poll/apply point stays HERE under Task 5's autonomous design --
         * after frame N's own ranging is fully read out, with nothing else touching the
         * sensor -- for the same reason: vl53l9_stop() is only ever called with the
         * sensor genuinely idle (there is no "N+1 trigger" to be ahead of anymore, only
         * the free-running FSM itself, which vl53l9_stop() gates on
         * FSM_STATE_STREAMING regardless of sync mode). */
        rs_poll_commands(calib_data, out_width, out_height, rs_counter);
        rs_poll_eth_commands(calib_data, out_width, out_height, rs_counter); /* UDP command channel */

        /* BUG-005 safe point: a CDC host attached since the last frame. Anything it saw of the
         * frame in flight was already abandoned in rs_cdc_send(); what it needs now is to start
         * at a frame boundary WITH calibration, so lead this frame's group with CALIB instead of
         * making it wait out the rest of the 64-frame cadence. Consumed here, in the loop, and
         * nowhere else -- the callback only ever sets the flag. */
        if (g_cdc_connect_evt) {
            g_cdc_connect_evt = 0u;
            rs_calib_countdown = 0u;
        }

        if (rs_pending.pending) {
            /* rs_apply_pending_config() leaves the sensor free-running (autonomous)
             * under whichever profile ends up active before returning -- there is no
             * separate "trigger N+1" step to replace anymore (Task 5). A `true` return
             * means a fault hit mid-apply and handle_error() already recovered the
             * sensor via its OWN reinit -- this iteration's frame N send below would be
             * reading stale/irrelevant buffers, so abandon it and resume the loop fresh
             * (see the design comment above rs_recover()). */
            if (rs_apply_pending_config(p_dev, calib_data, out_width, out_height, rs_counter)) {
                continue;
            }
        }
        /* AUTONOMOUS (Task 5): nothing to trigger here either way -- the sensor free-runs
         * at g_active_profile's frame_period_us on its own, whether or not a command was
         * pending this iteration. The CDC/Ethernet sends below simply run concurrently
         * with whatever ranging the sensor is already doing for frame N+1. */

        /* Everything from here to the disarm below describes frame N, so it is stamped with
         * frame N's FRAME_READY instant rather than the moment each send happens. That
         * includes the paired IMU/env/raw frames: they are the samples that belong to this
         * ToF frame, and giving the whole group one common, physically meaningful t_us is
         * exactly what the host needs to align depth against rotation. */
        g_frame_stamp_us = rs_ready_us;

        /* send frame N (and the periodic CALIB before it, so a host joining at frame 1
         * always has calib before its first RAW) while the sensor works on N+1 */
        {
            static uint32_t last_print = 0;
            uint32_t now = HAL_GetTick();
            if (now - last_print >= 1000) {
                printf("[STREAM] Processed frame %lu (ToF only via ST-LINK)\n", (unsigned long)rs_counter);
                last_print = now;
            }
            /* rs_calib_countdown is at file scope (not a function static as it used to be)
             * so the DTR-attach handler above the loop can zero it -- see BUG-005. */
            if (rs_calib_countdown == 0) {
                rs_send_frame_cdc(RS_STREAM_CALIB, rs_counter, 0u, calib_data,
                                  VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                /* IMU clock calibration rides the same cadence as CALIB, and for the same
                 * reason: it is static per-device metadata that a host joining late (or
                 * seeking into the middle of a recording) still has to have. Skipped
                 * entirely when the register read failed -- absence means "use the
                 * nominal tick", which is exactly the pre-2026-07-28 host behaviour. */
                if (g_lsm_ok && g_lsm_freq_fine_valid) {
                    uint8_t imu_cal[RS_IMU_CAL_SIZE];
                    imu_cal[0] = (uint8_t)g_lsm_freq_fine; /* int8, two's complement */
                    imu_cal[1] = 1u;                       /* valid */
                    imu_cal[2] = 0u;
                    imu_cal[3] = 0u;
                    rs_send_frame_cdc(RS_STREAM_IMU_CAL, rs_counter, 0u, imu_cal,
                                      RS_IMU_CAL_SIZE, 0u, 0u);
                }
                /* Task 6 step 2: Ethernet TX-pacer queue telemetry, same cadence as CALIB
                 * (see rs_send_tx_queue_stats_event()'s own comment for why). */
                rs_send_tx_queue_stats_event(rs_counter);
                rs_calib_countdown = 64;
            }
            rs_calib_countdown--;
            rs_send_frame_cdc(RS_STREAM_RAW_3DMD, rs_counter, 0u,
                              (const uint8_t *)in_raw_mem[raw_mem_index].data,
                              raw_buffer_size, out_width, out_height);
        }

        /* IMU orientation + env (LSM6DSV16X), via the shared rs_lsm_service_tick() (Task 7
         * plan step 1). Coupled mode (the default, rate 0): drains here unconditionally,
         * once per ToF frame, exactly as before this feature existed. Decoupled mode:
         * drains HERE only when rs_lsm_decoupled_due() says the TIM2-paced schedule is due
         * right now -- most decoupled drains instead happen off-cycle, inside the
         * FRAME_READY wait (rs_wait_frame_ready_svc()), where there is no coincident edge
         * and stream 13 is skipped (idle loop's own convention). When a decoupled drain
         * DOES land here, it coincides with THIS frame's FRAME_READY edge exactly like
         * coupled mode, so it gets stream 13 too. A read failure only skips this frame's
         * IMU/env -- the ToF stream above is already sent. */
        bool rs_lsm_service_now =
            (g_imu_env_rate_hz == RS_IMU_ENV_RATE_COUPLED) || rs_lsm_decoupled_due();
        if (g_lsm_ok && rs_lsm_service_now) {
            /* How far after FRAME_READY this drain actually happens -- the gap the host used
             * to have to guess at (BUG-031). Measured, shipped on stream 13, so the guess is
             * both unnecessary and checkable. */
            if (g_frame_sync.valid) {
                uint64_t t_drain = rs_time_us();
                g_frame_sync.drain_delay_us =
                    (uint32_t)((t_drain > rs_ready_us) ? (t_drain - rs_ready_us) : 0u);
            }
            uint32_t quat_mid_ticks = 0;
            uint16_t quat_n = 0;
            /* abortable=false (BUG-077): this point runs AFTER frame N's own DMA readout
             * is fully acked -- the bus is genuinely idle and nothing time-critical is
             * waiting behind this drain, so it keeps the full-drain guarantee unchanged
             * (coupled mode's byte-identical contract depends on this). */
            (void)rs_lsm_service_tick(rs_counter, &quat_mid_ticks, &quat_n, false);
            /* Stream 13: where this frame's FRAME_READY edge sits on the LSM clock. Sent last
             * in the group because drain_delay_us is only known once the drain above has run,
             * and still inside the armed g_frame_stamp_us window so it carries the SAME t_us
             * as the RAW frame it describes -- the pairing is the header, not a payload field. */
            if (g_frame_sync.valid) {
                uint8_t sync[RS_IMU_SYNC_SIZE];
                rs_put_u32(sync + 0, g_frame_sync.lsm_ticks);
                rs_put_u32(sync + 4, g_frame_sync.latch_delay_us);
                rs_put_u32(sync + 8, g_frame_sync.drain_delay_us);
                rs_put_u32(sync + 12, quat_mid_ticks);
                sync[16] = (uint8_t)(g_frame_sync.read_us & 0xFFu);
                sync[17] = (uint8_t)(g_frame_sync.read_us >> 8);
                sync[18] = (uint8_t)(quat_n & 0xFFu);
                sync[19] = (uint8_t)(quat_n >> 8);
                sync[20] = 1u;   /* valid */
                sync[21] = 0u;   /* reserved */
                rs_send_frame_cdc(RS_STREAM_IMU_SYNC, rs_counter, 0u, sync,
                                  RS_IMU_SYNC_SIZE, 0u, 0u);
            }
        }

        g_frame_stamp_us = 0; /* disarm: EVENT/ACK frames go back to the live clock */

        /* measure frame rate */
        stop_time = platform_profiler_get_timestamp();
        frame_rate = (1.0f / (float)(platform_profiler_convert_to_us(stop_time - start_time))) * 1000000;
        start_time = stop_time;

        /* swap raw buffer index: purely cosmetic in this mode (see buffer-safety note
         * above), kept so both loops use the double buffer identically */
        raw_mem_index = (raw_mem_index + 1) % 2;
    }

#endif /* CONF_TRANSFORM_ONBOARD */

    /* NOTE: free memory and pipeline resources to avoid leaks */
    /* free(in_raw_mem[0].data); */
    /* free(in_raw_mem[1].data); */
    /* free(out_depth_mem.data); */
    /* transform_finalize(p_transform); */
    /* transform_release(p_transform); */
    /* vl53l9_transform_destroy(p_transform); */
}

static void print_frame(float *p_frame, size_t height, size_t width) {
#if CONF_PRINT_FRAME
    static const char ASCII_CHARS[] = "@%#*+=-:. ";

    printf("\033[%d;%dH", 0, 0); /* set cursor to the top of the screen */
    int pixel_step = 1;
    uint32_t min = UINT32_MAX;
    uint32_t max = 0;

    for (uint32_t i = 0; i < (height * width); i++) {
        uint32_t value = (uint32_t)p_frame[i];
        min = MIN(value, min);
        max = MAX(value, max);
    }

    uint32_t average = (uint32_t)((max - min) * 0.05f);
    min = MAX(min - average, 0);
    max = MIN(max + average, UINT32_MAX);

    for (uint32_t y = 0; y < height; y += pixel_step) {
        for (uint32_t x = 0; x < width; x += pixel_step) {
            uint32_t pixel_index = (y * width + x);
            uint32_t value = (uint32_t)p_frame[pixel_index];

            uint32_t ascii_index = (value - min) * (sizeof(ASCII_CHARS) - 1) / (max - min);
            ascii_index = MIN(ascii_index, sizeof(ASCII_CHARS) - 1);

            printf("%c", ASCII_CHARS[ascii_index]);
        }
        printf("\n");
    }
#endif /* CONF_PRINT_FRAME */
    return;
}

static memory_t allocate_memory(uint16_t size) {
    memory_t memory;
    memory.size = size;
    memory.data = malloc(size);
    if (memory.data == NULL) {
        /* Effectively unreachable (two fixed ~15 KB buffers against 640 KB SRAM). NOTE
         * (raw-only builds): handle_error()'s sensor recovery is irrelevant to a malloc
         * failure; in the pre-loop call context g_active_profile is also still
         * uninitialized, so recovery fails into the terminal spin ~3 s later -- the
         * de-facto behavior is "spin on OOM", same as before Task 5, just delayed. */
        handle_error();
    }
    return memory;
}

/* Fault entry point for every driver-call failure in the app. Raw-only builds (Task 5):
 * emit an EVENT carrying the sensor's status word, then run rs_recover()'s bounded
 * retry (5 attempts, 100/200/400/800/1600 ms backoff, its own SENSOR_INIT_FAIL EVENT per
 * failed attempt) -- on success this function RETURNS NORMALLY (every call site above
 * treats that as "resume via `continue`", see the design comment above rs_recover()).
 * On-board-transform builds (CONF_TRANSFORM_ONBOARD=1, the golden-pair regeneration
 * path) keep the original, unmodified terminal spin -- no EVENT, no recovery -- per the
 * brief's "golden-path stability" call. rs_recover() lives inside the
 * !CONF_TRANSFORM_ONBOARD guard so that build genuinely cannot call it;
 * rs_send_event() is file-scope (it sits with the other generic senders, above the
 * guard) but is deliberately not called from the dual-stream build either -- hence
 * that build's expected unused-function warning for it. This function is the only
 * place the two builds' fault policies have to be made explicit.
 *
 * Exhaustion (either build): drop off the USB bus (this spin never services tud_task, so
 * leaving the D+ pull-up asserted would present a dead device to the host, Code 43;
 * harmless if called before tud_connect, pull-up already off) and spin forever -- the
 * unchanged last-resort contract this function has always had. */
static void handle_error(void) {
#if !CONF_TRANSFORM_ONBOARD
    vl53l9_status_t status = { 0 };
    vl53l9_get_status(&device[CONF_DEVICE_ID], &status);
    rs_send_event(RS_EVT_SENSOR_ERROR_STATUS, rs_pack_status(&status), NULL);

    if (rs_recover() == 0) {
        return; /* recovered: caller resumes via `continue` */
    }
    /* 5 attempts exhausted: fall through to the same terminal spin as the
     * on-board-transform build below -- last resort, unchanged. */
#endif
    tud_disconnect();
    vl53l9_status_t final_status = { 0 };
    vl53l9_get_status(&device[CONF_DEVICE_ID], &final_status);
    BSP_LED_On(LED3);   /* red LD3: this is the other spin-forever path (see main.c) */
    while (1)
        ;
}
