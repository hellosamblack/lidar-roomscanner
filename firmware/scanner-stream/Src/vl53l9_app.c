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
#define CONF_TRANSFORM_ONBOARD (0) /**< 1 = run vl53l9_transform on-MCU and stream DEPTH (Phase 1 behavior, also the
                                     * golden-pair regeneration path with CONF_STREAM_RAW=1); 0 = raw-only, transform
                                     * runs on the PC (Phase 2 -- equivalence gate passed, on-MCU transform removed
                                     * from the hot path) */
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

extern UART_HandleTypeDef hcom_uart[];
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

static uint64_t rs_time_us(void) {
    /* v1: HAL tick, 1 ms resolution widened to the u64 µs wire field.
     * Upgrade to a TIM-based µs clock when IMU fusion needs it (Phase 5). */
    return (uint64_t)HAL_GetTick() * 1000u;
}

/* Pump the CDC FIFO out. Returns false if the host stalled >100 ms (frame aborted:
 * the host decoder counts one CRC failure/resync and we set DROPPED on the next frame). */
static bool rs_cdc_send(const uint8_t *p, uint32_t n) {
    uint32_t t0 = HAL_GetTick();
    while (n) {
        uint32_t avail = tud_cdc_write_available();
        if (avail) {
            uint32_t k = MIN(avail, n);
            tud_cdc_write(p, k);
            p += k;
            n -= k;
        }
        tud_task(); ETH_Process();
        if ((HAL_GetTick() - t0) > 100u) {
            return false;
        }
    }
    tud_cdc_write_flush();
    return true;
}

/* Shared low-level sender: builds header + CRC and pushes header/payload/tail over CDC.
 * frame_type-agnostic so DATA (via rs_send_frame_cdc) and ACK (via rs_send_ack) share one
 * wire-framing implementation -- the only thing that differs between them is what goes in
 * the payload and whether the DROPPED-flag bookkeeping below applies. Stays OUTSIDE the
 * !CONF_TRANSFORM_ONBOARD guard (unlike rs_send_ack) because rs_send_frame_cdc, used by
 * both loop variants, is built on it. */

static bool rs_send_generic_cdc(uint8_t frame_type, uint8_t stream_id, uint32_t seq, uint8_t flags,
                                const uint8_t *payload, uint32_t len, uint16_t w, uint16_t h) {
    if (!tud_cdc_connected() && !ETH_IsUp()) {
        return false;
    }
    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    rs_write_header(hdr, frame_type, stream_id, flags, seq, rs_time_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);

    bool eth_sent = false;
    if (ETH_IsUp()) {
        eth_sent = ETH_SendFrame_Gather(hdr, RS_HEADER_SIZE, payload, len, tail, 4);
    }
    
    bool usb_sent = false;
    if (tud_cdc_connected()) {
        usb_sent = rs_cdc_send(hdr, RS_HEADER_SIZE) && rs_cdc_send(payload, len) && rs_cdc_send(tail, 4);
    }
    
    return eth_sent || usb_sent;
}

static void rs_send_frame_cdc(uint8_t stream_id, uint32_t seq, uint8_t flags, const uint8_t *payload,
                              uint32_t len, uint16_t w, uint16_t h) {
    static uint8_t pending_dropped = 0;

    if (!tud_cdc_connected() && !ETH_IsUp()) {   /* no host: don't burn 100 ms per frame */
        pending_dropped = 1;
        return;
    }
    flags |= pending_dropped ? RS_FLAG_DROPPED : 0u;

    bool ok = rs_send_generic_cdc(RS_FRAME_DATA, stream_id, seq, flags, payload, len, w, h);
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
    (void)rs_send_generic_cdc(RS_FRAME_EVENT, 0u, g_last_seq, 0u, payload, (uint32_t)(8u + msg_len), 0u, 0u);
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

#if !CONF_TRANSFORM_ONBOARD
/* Raw-only trigger-early overlap (this task): settle then trigger, sharing one helper
 * for every trigger call in this mode -- the pre-loop seed for frame 1, the
 * "trigger(N+1)" issued right after frame N's readout ack, and any retry after a lost
 * trigger inside the acquire loop below. Every one of those needs the same settle
 * (a hardware requirement: a trigger issued back-to-back with the previous readout ack
 * -- COMMAND_ACK_FRAME_READ inside vl53l9_get_frame_async_ack -- is intermittently
 * ignored by the sensor, see Task 8's race writeup); folding settle+trigger into one
 * function makes that impossible to accidentally skip at a new call site.
 *
 * Settle-time experiment (the one bounded experiment allowed by the P2.5 Task 4 brief):
 * 5 ms (the Task 8 value) measured 25.9 fps (med 39 ms/frame); 2 ms measured 27.7 fps
 * (med 36-37 ms/frame) across two 30 s captures, both with crc 0, gaps 0, and max
 * inter-frame delta <= 39 ms -- i.e. no lost-trigger stalls (a lost trigger appears as a
 * ~1 s delta via the retry path below, and none occurred). 2 ms is kept; if the
 * lost-trigger race ever resurfaces at this value, the bounded-retry net below degrades
 * it to a visible stall rather than a lost frame, and this is the knob to raise back to
 * 5. (HAL_Delay(n) actually waits n+1 ticks, so 2 here means ~3 ms wall time.) */
#define RS_TRIGGER_SETTLE_MS (2u)
/* Returns the vl53l9 error code (0 on success) instead of dead-ending into
 * handle_error() itself (Task 5 recursion guard -- see the block comment above
 * rs_recover() below for the full reasoning): rs_sensor_reinit()'s own post-start tail
 * calls this to seed frame 1, and rs_sensor_reinit() is itself called BY handle_error()'s
 * recovery loop -- if this function called handle_error() on failure, a sensor that keeps
 * coming up trigger-broken would recurse handle_error -> rs_sensor_reinit ->
 * rs_trigger_next -> handle_error without bound. Every ordinary (non-recovery) call site
 * below still routes a failure through handle_error() explicitly, exactly as before. */
static int rs_trigger_next(vl53l9_device_t *p_dev) {
    HAL_Delay(RS_TRIGGER_SETTLE_MS);
    return vl53l9_trigger_frame(p_dev);
}

/* ---- Host->device command channel (Phase 3 Task 2) --------------------------------
 *
 * Raw-only path only (not the on-MCU-transform golden loop above): the poll point is
 * called once per acquisition-loop iteration, after frame N's readout is fully acked
 * and BEFORE frame N+1's trigger (moved from its original after-the-RAW-send position
 * by Task 4 -- the reconfig safe point needs the sensor genuinely idle, see the
 * call site's ORDERING IS LOAD-BEARING comment), never from inside rs_wait_event_usb
 * (that primitive stays single-purpose: pump tud_task while waiting on a platform
 * event, nothing else).
 *
 * Backpressure honesty: the RX side never blocks (tud_cdc_available()/tud_cdc_read()
 * are non-blocking), but the TX responses ride the same best-effort rs_cdc_send policy
 * as every DATA frame -- each of its calls can stall up to 100 ms waiting on a
 * non-draining host before aborting. Worst case per dispatched command: an ACK is
 * 3 rs_cdc_send calls (header/payload/tail, up to ~300 ms); SEND_CALIB adds a CALIB
 * frame first (up to ~600 ms total). With the dispatch cap below (2 per poll), one
 * poll's command handling is bounded at roughly ~1.2 s of stall against a wedged host
 * -- a bounded acquisition hiccup, never a deadlock, and identical in kind to what a
 * wedged host already costs the RAW send path. A healthy host drains fast enough that
 * none of these limits are approached (measured: no fps change at ~28 fps).
 *
 * RX accumulation: a small flat buffer (commands are 44 B; a handful fit comfortably)
 * with memmove-compaction after each parse step -- simpler than a true ring buffer at
 * this size and call rate (one poll per ~36 ms frame period), and rs_parse_command's
 * contract (see rs_protocol.h) already does the "how much of the front can I discard"
 * reasoning, so the buffer code here only needs to shuffle bytes, not interpret them.
 * Draining and parsing are interleaved (parse after every read chunk) so a burst of
 * back-to-back commands larger than the buffer -- e.g. 3+ x 44 B in one host write,
 * TinyUSB's 256 B RX FIFO holds them fine -- is consumed command-by-command instead of
 * overflowing and losing a valid frame already at the buffer front. */
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
 * sensor-touching apply (stop -> reprofile -> restart -> re-trigger) runs from
 * rs_apply_pending_config(), called once per main-loop iteration from
 * rs_poll_commands()'s call site (#else / raw-only branch of vl53l9_app()) -- BEFORE
 * that iteration's own rs_trigger_next(N+1) call, which is skipped in favor of
 * rs_apply_pending_config()'s own post-restart trigger whenever a command is pending.
 *
 * Safe-point requirement (empirical, hardware finding -- see the call site's comment
 * for the full trace): vl53l9_stop() must never be called while a trigger is genuinely
 * in flight. An earlier version of this code applied pending config AFTER the
 * iteration's trigger-for-N+1 call, on the assumption that vl53l9_stop() would cleanly
 * cancel it; on hardware this instead corrupted the sensor's internal ranging state
 * (one good frame post-restart, then FSM_STATE_STREAMING silently dropped back to
 * FSM_STATE_STANDBY with sof_outside_blanking + internal_fw error bits set) --
 * reproduced even with a same-profile no-op reapply, so it was the stop-while-in-flight
 * itself, not any particular profile field. The apply now runs at the one point in the
 * iteration where frame N's own ranging is fully read out (DMA ack complete) and NOTHING
 * has been triggered yet for N+1 -- genuinely idle, matching the brief's "safe point
 * apply, before [re-]trigger" literally rather than just in spirit.
 *
 * The ACK for a pending command is sent from rs_apply_pending_config(), not from
 * rs_handle_command() -- "ACK only after the sensor accepted" per the brief. */
typedef struct {
    bool pending;
    uint32_t cmd;
    uint32_t param;
    uint32_t token;
} rs_pending_cmd_t;

static rs_pending_cmd_t rs_pending = { 0 };

/* Active profile, persists across reconfig commands so SET_FRAME_PERIOD_US /
 * SET_EXPOSURE_MS compose (each edits a copy of the currently-active profile, never the
 * shared g_ranging_profiles[] table -- vl53l9_utils.h:152). Seeded from
 * g_ranging_profiles[CONF_USECASE] once, right before the raw-only loop starts (see
 * vl53l9_app() below); SET_USECASE replaces it wholesale with a copy of the requested
 * table entry, SET_FRAME_PERIOD_US/SET_EXPOSURE_MS edit one field of the existing copy. */
static vl53l9_profile_t g_active_profile;

/* Pack a vl53l9_status_t (vl53l9.h:124) into one u32 for the ACK's `applied` field on a
 * SENSOR_ERROR result (docs/protocol.md: "applied = status word"): fsm state in the top
 * byte, last command in the next byte, firmware version in the low 16 bits. Not a 1:1
 * encoding of every field (the per-bit error flags and laser_driver[] are dropped) --
 * enough to see "what state was the sensor in" on the host/log side without growing the
 * ACK payload beyond its fixed 12 bytes. */
static uint32_t rs_pack_status(const vl53l9_status_t *s) {
    return ((uint32_t)s->fsm << 24) | ((uint32_t)s->command << 16) | (uint32_t)s->firmware;
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
 * address -> init -> calib re-read -> apply the CURRENT g_active_profile -> re-assert
 * manual sync -> start -> settle -> stale-event clear -> first trigger. Mirrors
 * vl53l9_app()'s own pre-loop setup sequence (reset/platform_assign_dynamic_address/
 * vl53l9_init/vl53l9_get_calib_data/vl53l9_utils_set_profile/vl53l9_set_sync_mode/
 * vl53l9_start, above) so REINIT is a faithful "do the boot sequence again" rather than
 * a partial reset -- and this is exactly the sequence Task 5's bounded-retry recovery
 * needs, hence factored out as a standalone callable. The post-start tail (settle +
 * event-ack + trigger) lives INSIDE this function deliberately: it is the safety
 * envelope for the stale-event hardware bug documented below, and any future caller
 * (Task 5's recovery path) must inherit it structurally rather than having to know to
 * replicate it. On success the sensor is streaming with frame 1 already triggered --
 * the caller resumes the normal wait-for-GPIO-event loop directly.
 *
 * Stale-event hazard (empirical, Task 4 hardware finding): platform_power_reset()
 * toggles XSHUT (platform_utils.c:75-81) and platform_assign_dynamic_address() re-inits
 * the I3C peripheral -- either can put a spurious edge on the sensor's interrupt line
 * that the EXTI ISR latches into g_platform_evt with no real frame behind it. Left
 * uncleared, the main loop's next rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, ...) consumes
 * that stale flag immediately and vl53l9_get_frame_async() correctly reports
 * VL53L9_ERROR_INVALID_STATE (vl53l9.c:706-711: FRAME_READY register reads 0) --
 * reproduced on hardware: the re-init and seeded trigger both succeeded, then the very
 * next frame read failed this way and the loop's retry budget (Task 8's 1 ms/8-attempt
 * window, sized for the sub-millisecond real race, not a fully stale flag) exhausted
 * into handle_error(). Acknowledging both events right before the fresh trigger ensures
 * the next wait can only be satisfied by a genuinely new edge.
 *
 * calib_data is written in place, and the CALLER RETRANSMITS it over CDC after a
 * successful return (calibration may have changed across a physical reset) -- every
 * caller honors this: rs_apply_pending_config()'s two direct call sites send CALIB
 * explicitly, and rs_recover() retransmits on its success path so all
 * handle_error()-driven recoveries inherit it (see its comment).
 * Returns 0 on success, the first non-zero vl53l9_error on failure (VL53L9_ERROR_* per
 * vl53l9.h:47-53) -- INCLUDING a failed seed trigger: rs_trigger_next() (Task 5) now
 * returns its error code instead of calling handle_error() itself, and this function
 * propagates it like any other stage failure. This is deliberate (recursion guard, see
 * rs_recover()'s comment): this function must never call handle_error(), because
 * handle_error()'s own recovery loop is what calls this function. */
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

    ret = vl53l9_utils_set_profile(p_dev, &g_active_profile);
    if (ret) {
        return ret;
    }

    /* g_ranging_profiles[] entries all set .sync = VL53L9_SYNC_AUTONOMOUS
     * (vl53l9_utils.c:32/41/51/59); this app is manual-trigger only (see
     * vl53l9_app()'s own override of the same shape below), so re-assert it after every
     * profile application, exactly like the pre-loop setup does. */
    ret = vl53l9_set_sync_mode(p_dev, VL53L9_SYNC_MANUAL);
    if (ret) {
        return ret;
    }

    ret = vl53l9_start(p_dev);
    if (ret) {
        return ret;
    }

    /* Post-start tail (the safety envelope -- see the function comment): settle margin
     * matching the pre-loop boot sequence's HAL_Delay(50), clear any stale latched
     * events from the reset, then seed the first frame. A failed seed trigger is
     * propagated to our own caller (recursion guard -- see the function comment and
     * rs_trigger_next's own comment); it is NOT retried here. */
    HAL_Delay(50);
    platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);
    platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);
    return rs_trigger_next(p_dev);
}

/* ---- Bounded sensor recovery (Phase 3 Task 5) --------------------------------------
 *
 * Recursion guard, spelled out (the residual flagged in Task 4's review): the naive
 * version of this feature has handle_error() call rs_sensor_reinit() to recover, and
 * rs_sensor_reinit()'s own tail calls rs_trigger_next() to seed frame 1. If
 * rs_trigger_next() dead-ended into handle_error() on failure (as it used to), a sensor
 * that keeps coming up trigger-broken would recurse handle_error -> rs_sensor_reinit ->
 * rs_trigger_next -> handle_error -> rs_sensor_reinit -> ... without bound (each level
 * consuming stack, never unwinding). Fixed structurally, not by convention:
 * rs_trigger_next() and rs_sensor_reinit() now both return ordinary error codes and
 * NEITHER of them ever calls handle_error(). rs_recover() below is the ONLY function
 * that calls rs_sensor_reinit() in a retry loop, and rs_recover() itself never calls
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
 * leaves the sensor with frame 1 ALREADY TRIGGERED and both stale platform events
 * acknowledged (its own safety envelope) -- exactly the state the top of the raw-only
 * loop expects when it begins by waiting on PLATFORM_GPIO_IT_EVT. A command that was
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
            /* Streaming again, frame 1 already triggered inside rs_sensor_reinit -- which
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
            vl53l9_utils_get_resolution(g_active_profile.binning, &w, &h);
            rs_send_frame_cdc(RS_STREAM_CALIB, g_last_seq, 0u, calib_data,
                              VL53L9_CALIB_DATA_SIZE, w, h);
            return 0;
        }
        rs_send_event(RS_EVT_SENSOR_INIT_FAIL, (uint32_t)attempt, NULL);
    }
    return -1; /* 5 attempts exhausted */
}

/* Boot bring-up (Task 5): reset -> I3C address -> init -> calib -> profile-apply ->
 * sync-mode -> start -- the full sequence vl53l9_app() used to run inline exactly once,
 * with no error recovery on any step. Its call site in vl53l9_app() wraps this in the
 * SAME bounded-retry shape as rs_recover() (5 attempts, 100/200/400/800/1600 ms
 * backoff), which is what converts the historical ~1-in-5 first-power-up failure into a
 * self-healing delay instead of an immediate handle_error() hang.
 *
 * Deliberately NOT rs_sensor_reinit(): that function's post-start tail also seeds frame
 * 1's trigger and clears stale platform events -- correct for REINIT/recovery, where the
 * caller is about to resume the acquisition loop immediately, but wrong here. Boot
 * bring-up runs BEFORE vl53l9_app()'s own buffer allocation, tud_connect(), and
 * DTR-gate-then-trigger-frame-1 sequence (further down, unchanged) -- triggering frame 1
 * this early would race those steps and risk an unwanted second trigger once the main
 * loop seeds frame 1 itself. This function leaves the sensor in STANDBY, matching
 * exactly what the original inline boot sequence did (out_calib_data is written in
 * place, as before). */
static int rs_boot_bringup(vl53l9_device_t *p_dev, uint8_t *out_calib_data, vl53l9_profile_t *p_profile) {
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

    ret = vl53l9_utils_set_profile(p_dev, p_profile);
    if (ret) {
        return ret;
    }

    ret = vl53l9_set_sync_mode(p_dev, VL53L9_SYNC_MANUAL);
    if (ret) {
        return ret;
    }

    return vl53l9_start(p_dev);
}

/* ACK sender: builds the 12-byte (cmd, result, applied) payload and sends an RS_FRAME_ACK
 * with header seq = the echoed command token (per docs/protocol.md, NOT a frame counter).
 * Best-effort like every other CDC send on this link -- no retry/queue if the host is gone
 * or stalls (bounded at ~300 ms by rs_cdc_send's per-call timeout, see the channel block
 * comment above), and RS_FLAG_DROPPED does not apply to control frames (always flags=0).
 * Lives inside the !CONF_TRANSFORM_ONBOARD guard because only the raw-only loop has a
 * command channel; the dual-stream golden loop would leave it unused. */
static void rs_send_ack(uint32_t token, uint32_t cmd, uint32_t result, uint32_t applied) {
    uint8_t payload[12];
    rs_put_u32(payload + 0, cmd);
    rs_put_u32(payload + 4, result);
    rs_put_u32(payload + 8, applied);
    (void)rs_send_generic_cdc(RS_FRAME_ACK, 0u, token, 0u, payload, sizeof(payload), 0u, 0u);
}

static void rs_handle_command(uint32_t cmd, uint32_t param, uint32_t token, const uint8_t *calib_data,
                              uint16_t out_width, uint16_t out_height, uint32_t seq_for_calib) {
    switch (cmd) {
    case RS_CMD_PING:
        rs_send_ack(token, cmd, RS_RESULT_OK, RS_PROTO_VERSION);
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
        rs_send_ack(token, cmd, RS_RESULT_OK, 0u);
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
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param);
            break;
        }
        if (g_ranging_profiles[param].binning != 2u) {
            rs_send_ack(token, cmd, RS_RESULT_REJECTED_BINNING, g_ranging_profiles[param].binning);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token };
        break;
    case RS_CMD_SET_FRAME_PERIOD_US:
        /* Same bounds vl53l9_set_frame_period() itself enforces (vl53l9.c:402): 10 ms -
         * 1 s. Reject out of range here rather than let the driver call fail later, so a
         * bad param never touches the sensor or consumes the one pending slot. */
        if (param < 10000u || param > 1000000u) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token };
        break;
    case RS_CMD_SET_EXPOSURE_MS:
        /* Same bounds vl53l9_set_exposure() itself enforces (vl53l9.c:550): 1-30 ms
         * (the brief's own guess of "1-100ms" doesn't match the driver -- the profile
         * table's exposure_ms values, 4/5/8/10, all sit comfortably inside 1-30). */
        if (param < 1u || param > 30u) {
            rs_send_ack(token, cmd, RS_RESULT_BAD_PARAM, param);
            break;
        }
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = param, .token = token };
        break;
    case RS_CMD_REINIT:
        if (rs_pending.pending) {
            rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u);
            break;
        }
        rs_pending = (rs_pending_cmd_t){ .pending = true, .cmd = cmd, .param = 0u, .token = token };
        break;
    default:
        rs_send_ack(token, cmd, RS_RESULT_UNKNOWN_CMD, 0u);
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
        /* rs_sensor_reinit() returned with the sensor streaming and frame 1 already
         * triggered (settle + stale-event clear + trigger are inside it -- its safety
         * envelope, see its comment). All that's left here: re-send CALIB (it may have
         * changed across the physical reset; unconditional, independent of the periodic
         * 64-frame cadence in the caller, same rationale as RS_CMD_SEND_CALIB above)
         * and ack. */
        rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data, VL53L9_CALIB_DATA_SIZE, out_width,
                          out_height);
        rs_send_ack(token, cmd, RS_RESULT_OK, 0u);
        return false;
    }

    /* SET_USECASE / SET_FRAME_PERIOD_US / SET_EXPOSURE_MS: build a candidate profile
     * (never mutating g_active_profile or the shared g_ranging_profiles[] table until
     * the sensor has actually accepted it), stop -> apply -> restart. */
    vl53l9_profile_t candidate = g_active_profile;
    if (cmd == RS_CMD_SET_USECASE) {
        /* param already bounds- and binning-checked in rs_handle_command; re-reading
         * g_ranging_profiles[param] here (rather than caching it at validation time)
         * costs nothing and keeps the two checks visibly in sync. */
        candidate = g_ranging_profiles[param];
    } else if (cmd == RS_CMD_SET_FRAME_PERIOD_US) {
        candidate.frame_period_us = param;
    } else if (cmd == RS_CMD_SET_EXPOSURE_MS) {
        candidate.exposure_ms = (uint16_t)param;
    }

    /* vl53l9_utils_set_profile()'s setters all reject anything but FSM_STATE_STANDBY
     * (vl53l9.c: vl53l9_set_sync_mode:385, vl53l9_set_frame_period:399,
     * vl53l9_set_context:424, vl53l9_set_binning:462, vl53l9_set_exposure has no such
     * gate but is meaningless while streaming) -- vl53l9_stop() (vl53l9.c:591) is the
     * STREAMING -> STANDBY transition and is what the vl53l9_utils_set_profile header
     * note (vl53l9_utils.h:127) means by "device must be in standby mode". */
    int ret = vl53l9_stop(p_dev);
    if (ret) {
        /* vl53l9_stop() only fails when the sensor has ALREADY left FSM_STATE_STREAMING
         * (its sole gate, vl53l9.c:593 -- INVALID_STATE) or the stop command itself
         * timed out; either way the device is NOT healthily streaming, so returning to
         * the acquisition loop as-is would just dead-end in handle_error() at the next
         * trigger (vl53l9_trigger_frame's own STREAMING gate, vl53l9.c:604). Ack the
         * failure, then attempt a full best-effort re-init back onto the previous
         * known-good profile (g_active_profile is still the old profile -- candidate
         * was never applied); only if THAT also fails, fall to the terminal spin
         * (Task 5 upgrades it). */
        vl53l9_status_t status = { 0 };
        vl53l9_get_status(p_dev, &status);
        rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status));
        if (rs_sensor_reinit(p_dev, calib_data)) {
            /* the direct best-effort reinit above also failed: hand off to
             * handle_error()'s own (fresh) bounded recovery loop */
            handle_error();
            return true;
        }
        /* recovered: sensor streaming again on the old profile, frame 1 triggered
         * inside rs_sensor_reinit; calib may have changed across the reset */
        rs_send_frame_cdc(RS_STREAM_CALIB, seq_for_calib, 0u, calib_data, VL53L9_CALIB_DATA_SIZE, out_width,
                          out_height);
        return false;
    }

    ret = vl53l9_utils_set_profile(p_dev, &candidate);
    bool applied_ok = (ret == 0);
    if (!applied_ok) {
        /* restore the previous (known-good) profile before leaving standby */
        int restore_ret = vl53l9_utils_set_profile(p_dev, &g_active_profile);
        if (restore_ret) {
            /* double failure: no known-good profile could be re-applied.
             * handle_error()'s bounded recovery is the only way back. */
            handle_error();
            return true;
        }
    }

    /* Re-assert manual sync after EVERY profile application, success or restore (both
     * paths just wrote .sync = VL53L9_SYNC_AUTONOMOUS via vl53l9_utils_set_profile) --
     * same reasoning as rs_sensor_reinit() above. */
    int sync_ret = vl53l9_set_sync_mode(p_dev, VL53L9_SYNC_MANUAL);
    int start_ret = vl53l9_start(p_dev);
    if (sync_ret || start_ret) {
        /* Could not get back to streaming at all (neither candidate nor restored
         * profile). handle_error()'s bounded recovery is the only way back. */
        handle_error();
        return true;
    }

    /* Post-restart settle + stale-event clear -- see the identical comments on the
     * REINIT path above; same margin, same reasoning, applies here too since both paths
     * call vl53l9_start() then trigger cold (vl53l9_stop()/vl53l9_start() are a less
     * violent transition than a physical reset, but the defensive clear is cheap and
     * this path was where the stop-while-triggered fault originally reproduced, so it
     * gets the same care). */
    HAL_Delay(50);
    platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);
    platform_acknowledge_event(PLATFORM_I3C_DMA_RX_EVT);
    if (rs_trigger_next(p_dev)) { /* seed the first frame under whichever profile is now active */
        handle_error();
        return true;
    }

    if (!applied_ok) {
        vl53l9_status_t status = { 0 };
        vl53l9_get_status(p_dev, &status);
        rs_send_ack(token, cmd, RS_RESULT_SENSOR_ERROR, rs_pack_status(&status));
        return false;
    }

    g_active_profile = candidate; /* adopt only now that the sensor has accepted it */

    /* applied = the value actually in effect (docs/protocol.md): usecase has no
     * driver-side clamping to observe, so echo the id; period/exposure are read back in
     * case the driver clamped (vl53l9_set_frame_period/vl53l9_set_exposure do bounds
     * validation and reject out-of-range instead of clamping -- vl53l9.c:402,550 -- so
     * in practice these will equal param, but reading back reports reality either way,
     * per the brief). */
    uint32_t applied = param;
    if (cmd == RS_CMD_SET_FRAME_PERIOD_US) {
        uint32_t readback = param;
        (void)vl53l9_get_frame_period(p_dev, &readback);
        applied = readback;
    } else if (cmd == RS_CMD_SET_EXPOSURE_MS) {
        uint16_t readback = (uint16_t)param;
        (void)vl53l9_get_exposure(p_dev, candidate.context, &readback);
        applied = readback;
    }
    rs_send_ack(token, cmd, RS_RESULT_OK, applied);
    return false;
}

static void rs_poll_commands(const uint8_t *calib_data, uint16_t out_width, uint16_t out_height,
                             uint32_t seq_for_calib) {
    static uint8_t rx_buf[RS_CMD_RX_BUFSIZE];
    static uint32_t rx_len = 0;

    uint32_t dispatched = 0;

    /* Parse-while-draining: after every chunk read from TinyUSB, the parse loop runs
     * to consume completed commands out of the buffer front BEFORE reading more, so a
     * burst larger than the buffer flows through it command-by-command instead of
     * overflowing (the Task 2 review's critical fix -- the old drain-everything-first
     * version wiped a valid buffered command when a 3+-command burst arrived). Outer
     * loop terminates when an iteration makes no progress (nothing read AND nothing
     * consumed) or the dispatch cap is reached. */
    for (;;) {
        bool progressed = false;

        uint32_t space = RS_CMD_RX_BUFSIZE - rx_len;
        if (space > 0) {
            uint32_t got = tud_cdc_read(rx_buf + rx_len, space); /* 0 if FIFO empty */
            if (got > 0) {
                rx_len += got;
                progressed = true;
            }
        }

        /* Consume everything parseable right now; rs_parse_command reports exactly how
         * many front bytes to drop each step (full contract in rs_protocol.h). */
        while (rx_len > 0 && dispatched < RS_CMD_MAX_DISPATCH_PER_POLL) {
            uint32_t cmd, param, token;
            int32_t r = rs_parse_command(rx_buf, rx_len, &cmd, &param, &token);
            if (r == 0) {
                break; /* candidate pending: wait for more RX bytes */
            }
            uint32_t consume = (uint32_t)((r > 0) ? r : -r);
            if (consume > rx_len) {
                consume = rx_len; /* defensive; rs_parse_command never over-reports */
            }
            if (consume > 0) {
                memmove(rx_buf, rx_buf + consume, rx_len - consume);
                rx_len -= consume;
                progressed = true;
            }
            if (r > 0) {
                rs_handle_command(cmd, param, token, calib_data, out_width, out_height, seq_for_calib);
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
            if (rx_len == RS_CMD_RX_BUFSIZE) {
                /* Full buffer the parser cannot advance. Theoretically unreachable: a
                 * full 128 B buffer always yields parser progress (any complete-frame,
                 * false-magic, or no-magic outcome consumes bytes; the only 0-consume
                 * outcome needs len < RS_CMD_FRAME_SIZE at a front magic). Kept as a
                 * defensive escape: drop ONE byte past the front (preserving any later
                 * magic candidate, unlike a whole-buffer wipe) and count it. */
                memmove(rx_buf, rx_buf + 1, rx_len - 1u);
                rx_len -= 1u;
                rs_malformed_cmd_count++;
                continue;
            }
            return; /* FIFO drained, nothing parseable left pending */
        }
    }
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
     * the sensor has been on a 30 fps profile all along. No override needed here. */
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
         * recovery here runs with g_active_profile still uninitialized (it is seeded just
         * before the raw-only loop) and is guaranteed to fail into the terminal spin
         * ~3 s later -- acceptable for a can't-happen site, documented so it isn't
         * mistaken for a real recovery path. */
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
            boot_ret = rs_boot_bringup(p_dev, calib_data, p_profile);
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
     * until a host opens the CDC port (DTR). This gate is also what makes raw-only mode
     * (CONF_TRANSFORM_ONBOARD=0) golden-capture-compatible, so it stays on by default here
     * too; a headless/production build (no PC waiting on the far end) may want to revisit
     * blocking acquisition start on a host connection. */
    while (!tud_cdc_connected()) {
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

    /* Raw-only loop with trigger-early overlap (Phase 2.5 Task 4).
     *
     * The Phase 2 raw-only loop serialized the ~15 ms CDC send of frame N-1 into the
     * frame period: trigger(N) sat at the TOP of the loop, so the sensor idled while the
     * MCU pushed bytes to the host (~41 ms/frame = 5 ms settle + ~15 ms un-hidden send +
     * ~20 ms ranging/DMA/parse; see the P2 Task 5 report). Here the trigger for frame
     * N+1 is issued BEFORE frame N's send, so the sensor's integration/ranging of N+1
     * runs concurrently with the send of N:
     *
     *   GPIO wait (pumped) -> ack -> DMA kick(N) -> DMA wait (pumped) -> readout ack(N)
     *   -> parse metadata(N) -> settle + trigger(N+1) -> send CALIB-cadence + RAW(N)
     *   while the sensor integrates N+1 -> loop.
     *
     * Ordering decisions, in the order they appear:
     *
     *  - parse BEFORE send (deviation from the plan sketch, which listed parse last):
     *    vl53l9_utils_parse_frame is pure pointer arithmetic over the raw buffer -- no
     *    bus traffic (vl53l9_utils.c:149-179) -- so it can run any time after the buffer
     *    is complete. It must run after the readout ack (the metadata lives at
     *    buffer_size - sizeof(vl53l9_meta_t), i.e. in the tail segment that
     *    vl53l9_get_frame_async_ack retrieves), and running it before the send lets the
     *    wire seq be frame N's OWN frame_counter -- the send in this loop carries the
     *    CURRENT frame, so the prev-counter tracking of the dual-stream loop
     *    (rs_prev_counter/rs_have_prev) is gone in this mode. Seq on the wire always
     *    matches the payload by construction.
     *
     *  - trigger only after readout ack + settle (Task 8 races): the settle+trigger
     *    helper rs_trigger_next enforces the RS_TRIGGER_SETTLE_MS gap after the readout
     *    ack; the ack happened just above (parse in between is microseconds of pointer
     *    reads).
     *
     *  - trigger BEFORE send: the INT edge for N+1 may fire while the send is still in
     *    flight (ranging ~20 ms vs send ~15 ms, and a slow host can stall the send up to
     *    100 ms). That is safe: the ISR-set event flag in g_platform_evt persists until
     *    platform_acknowledge_event, so an edge landing during the send is latched and
     *    the next iteration's GPIO wait returns immediately (same contract
     *    rs_wait_event_usb already relies on between its 5 ms slices).
     *
     * Buffer safety (truth for THIS ordering): the send reads in_raw_mem[raw_mem_index]
     * -- the SAME buffer this iteration's DMA filled -- strictly after the DMA-done wait
     * and readout ack for it completed. No DMA is in flight during the send at all: the
     * next DMA is kicked only in the next iteration (after the GPIO wait), and it
     * targets in_raw_mem[raw_mem_index ^ 1] because raw_mem_index toggles at loop
     * bottom. Single-buffer semantics would therefore suffice in this mode, but the
     * double buffer is KEPT: the allocation is shared with the dual-stream loop above,
     * which genuinely needs it (its DMA of N overlaps its processing/send of N-1).
     *
     * First-frame edge: the trigger for frame 1 is seeded once before the loop (below,
     * after the DTR gate so acquisition still starts on host connect). Iteration 1 then
     * captures frame 1 completely before anything is sent, so every frame -- including
     * frame 1, which golden captures need for TNR alignment -- is sent, and the CALIB
     * countdown (initial value 0) fires before the first RAW send exactly as before.
     *
     * No CONF_STREAM_BINARY/CONF_STREAM_RAW guards inside this loop: the #error at the
     * top of the file guarantees both are 1 whenever CONF_TRANSFORM_ONBOARD is 0. */

    /* Seed the runtime-reconfig baseline from the profile this build actually started
     * with (CONF_USECASE) -- a plain struct copy, so later SET_USECASE/PERIOD/EXPOSURE
     * commands only ever mutate this local copy, never g_ranging_profiles[] itself. */
    g_active_profile = *p_profile;

    if (rs_trigger_next(p_dev)) { /* seed trigger for frame 1 */
        handle_error(); /* recovers (re-triggers frame 1 itself) or never returns */
    }

    while (1) {

        /* Keep USB serviced every iteration, even when waits below return fast. */
        tud_task(); ETH_Process();

        /* Wait for data-ready. Same bounded-retry disambiguation as the dual-stream
         * loop (Task 8): a timeout means either the trigger was lost (re-trigger, with
         * settle, via rs_trigger_next) or the edge landed after the timeout (poll
         * FRAME_READY, then fall through and ack, clearing any late edge so it cannot
         * leak into the next iteration). */
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
        for (;;) {
            ret = rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, 1000);
            if (ret) {
                uint8_t rs_is_ready = 0;
                (void)vl53l9_poll_frame(p_dev, &rs_is_ready);
                if (!rs_is_ready) {
                    if (++rs_attempts > 3) {
                        rs_send_event(RS_EVT_TRIGGER_TIMEOUT, (uint32_t)rs_attempts, NULL);
                        handle_error();
                        rs_fault_recovered = true;
                        break;
                    }
                    if (rs_trigger_next(p_dev)) { /* trigger lost: re-trigger (no event to ack) */
                        rs_send_event(RS_EVT_TRIGGER_TIMEOUT, (uint32_t)rs_attempts, NULL);
                        handle_error();
                        rs_fault_recovered = true;
                        break;
                    }
                    continue;
                }
            }
            platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);

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

        ret = rs_wait_event_usb(PLATFORM_I3C_DMA_RX_EVT, 1000);
        if (ret) {
            rs_send_event(RS_EVT_DMA_TIMEOUT, 1u, NULL); /* single 1000 ms wait, no
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

        /* Command-channel poll point: BEFORE this iteration's trigger-for-N+1 (moved
         * here from after it -- see the empirical finding below), after frame N's DMA
         * readout is fully acked (no I3C transaction in flight) so RX draining and any
         * reconfig it decides on run with the bus idle. RX never blocks; response TX is
         * best-effort with bounded worst-case stalls against a wedged host (capped at
         * RS_CMD_MAX_DISPATCH_PER_POLL dispatches, ~1.2 s ceiling -- see the channel
         * block comment). PING and SEND_CALIB ack immediately inside; SET_USECASE/
         * SET_FRAME_PERIOD_US/SET_EXPOSURE_MS/REINIT only validate and stash a pending
         * request (rs_pending) here -- applied below.
         *
         * ORDERING IS LOAD-BEARING (empirical, Task 4 hardware finding): the original
         * design called rs_poll_commands()/rs_apply_pending_config() AFTER this
         * iteration's rs_trigger_next(N+1) (the "trigger-early overlap" position used
         * every other iteration), on the theory that vl53l9_stop() would simply cancel
         * whatever trigger was already in flight. On hardware this corrupted the
         * sensor's internal ranging state instead: EVERY reconfig (including a same-
         * profile no-op re-apply -- isolated by testing SET_USECASE 1 while usecase 1
         * was already active) captured exactly one good frame post-restart, then the
         * NEXT trigger failed with VL53L9_ERROR_INVALID_STATE (-3) because the sensor
         * had autonomously dropped itself from FSM_STATE_STREAMING back to
         * FSM_STATE_STANDBY, with vl53l9_status_t.error.sof_outside_blanking = 1 and
         * .error.internal_fw = 1 (a firmware-detected internal fault, not a bad register
         * write -- every driver call in the apply sequence itself returned 0/success).
         * Root cause: vl53l9_stop() while a trigger is genuinely in flight is not a
         * clean cancel. Moving the poll/apply point to HERE -- after frame N's own
         * ranging is fully read out and before N+1 is ever triggered -- means
         * vl53l9_stop() is only ever called with nothing in flight; the fault did not
         * reproduce after this change (see the task report for the before/after
         * hardware traces). */
        rs_poll_commands(calib_data, out_width, out_height, rs_counter);

        if (rs_pending.pending) {
            /* rs_apply_pending_config() triggers its own first frame under whichever
             * profile ends up active before returning -- this REPLACES the normal
             * rs_trigger_next(N+1) call below for this iteration. A `true` return means
             * a fault hit mid-apply and handle_error() already recovered the sensor via
             * its OWN reinit -- this iteration's frame N send below would be reading
             * stale/irrelevant buffers, so abandon it and resume the loop fresh (see
             * the design comment above rs_recover()). */
            if (rs_apply_pending_config(p_dev, calib_data, out_width, out_height, rs_counter)) {
                continue;
            }
        } else {
            /* trigger frame N+1 now (settle enforced inside): the sensor integrates
             * while the CDC sends below are in flight */
            if (rs_trigger_next(p_dev)) {
                handle_error();
                continue;
            }
        }

        /* send frame N (and the periodic CALIB before it, so a host joining at frame 1
         * always has calib before its first RAW) while the sensor works on N+1 */
        {
            static uint32_t rs_calib_countdown = 0;
            if (rs_calib_countdown == 0) {
                rs_send_frame_cdc(RS_STREAM_CALIB, rs_counter, 0u, calib_data,
                                  VL53L9_CALIB_DATA_SIZE, out_width, out_height);
                rs_calib_countdown = 64;
            }
            rs_calib_countdown--;
            rs_send_frame_cdc(RS_STREAM_RAW_3DMD, rs_counter, 0u,
                              (const uint8_t *)in_raw_mem[raw_mem_index].data,
                              raw_buffer_size, out_width, out_height);
        }

        /* IMU orientation + env, paired with this ToF frame (LSM6DSV16X). Read failures
         * skip this frame's IMU/env only -- the ToF stream above is already sent. */
        if (g_lsm_ok) {
            rs_lsm_sample_t lsm;
            if (rs_lsm_read_latest(&lsm) == 0) {
                if (lsm.have_quat) {
                    rs_send_frame_cdc(RS_STREAM_IMU_QUAT, rs_counter, 0u,
                                      (const uint8_t *)lsm.quat, RS_IMU_QUAT_SIZE, 0u, 0u);
                }
                if (lsm.have_env) {
                    uint8_t env[RS_ENV_SIZE];
                    memcpy(env + 0, &lsm.pressure_pa, 4);
                    memcpy(env + 4, lsm.mag_ut, 12);
                    memcpy(env + 16, &lsm.temp_c, 4);
                    rs_send_frame_cdc(RS_STREAM_ENV, rs_counter, 0u, env, RS_ENV_SIZE, 0u, 0u);
                }
            }
        }

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
    while (1)
        ;
}
