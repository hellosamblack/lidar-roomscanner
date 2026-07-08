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

extern UART_HandleTypeDef hcom_uart[];

static void handle_error(void);

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

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
        tud_task();
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
 * the payload and whether the DROPPED-flag bookkeeping below applies. */
static bool rs_send_generic_cdc(uint8_t frame_type, uint8_t stream_id, uint32_t seq, uint8_t flags,
                                const uint8_t *payload, uint32_t len, uint16_t w, uint16_t h) {
    if (!tud_cdc_connected()) {
        return false;
    }
    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    rs_write_header(hdr, frame_type, stream_id, flags, seq, rs_time_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);
    return rs_cdc_send(hdr, RS_HEADER_SIZE) && rs_cdc_send(payload, len) && rs_cdc_send(tail, 4);
}

static void rs_send_frame_cdc(uint8_t stream_id, uint32_t seq, uint8_t flags, const uint8_t *payload,
                              uint32_t len, uint16_t w, uint16_t h) {
    static uint8_t pending_dropped = 0;

    if (!tud_cdc_connected()) {   /* no host: don't burn 100 ms per frame */
        pending_dropped = 1;
        return;
    }
    flags |= pending_dropped ? RS_FLAG_DROPPED : 0u;

    bool ok = rs_send_generic_cdc(RS_FRAME_DATA, stream_id, seq, flags, payload, len, w, h);
    pending_dropped = ok ? 0u : 1u;
}

/* ACK sender: builds the 12-byte (cmd, result, applied) payload and sends an RS_FRAME_ACK
 * with header seq = the echoed command token (per docs/protocol.md, NOT a frame counter).
 * Best-effort like every other CDC send on this link -- no retry/queue if the host is gone
 * or stalls, and RS_FLAG_DROPPED does not apply to control frames (always flags=0). */
static void rs_send_ack(uint32_t token, uint32_t cmd, uint32_t result, uint32_t applied) {
    uint8_t payload[12];
    rs_put_u32(payload + 0, cmd);
    rs_put_u32(payload + 4, result);
    rs_put_u32(payload + 8, applied);
    (void)rs_send_generic_cdc(RS_FRAME_ACK, 0u, token, 0u, payload, sizeof(payload), 0u, 0u);
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
        tud_task();
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
static void rs_trigger_next(vl53l9_device_t *p_dev) {
    int ret;
    HAL_Delay(RS_TRIGGER_SETTLE_MS);
    ret = vl53l9_trigger_frame(p_dev);
    if (ret) {
        handle_error();
    }
}

/* ---- Host->device command channel (Phase 3 Task 2) --------------------------------
 *
 * Raw-only path only (not the on-MCU-transform golden loop above): the poll point is
 * called once per acquisition-loop iteration, after that iteration's RAW send, never
 * from inside rs_wait_event_usb (that primitive stays single-purpose: pump tud_task
 * while waiting on a platform event, nothing else). Polling never blocks acquisition --
 * tud_cdc_available()/tud_cdc_read() are non-blocking, and command handling itself only
 * ever does bounded, best-effort CDC sends (rs_send_ack / the CALIB send below), same as
 * every other frame this firmware emits.
 *
 * RX accumulation: a small flat buffer (commands are 44 B; a handful fit comfortably)
 * with memmove-compaction after each parse step -- simpler than a true ring buffer at
 * this size and call rate (one poll per ~36 ms frame period), and rs_parse_command's
 * contract (see rs_protocol.h) already does the "how much of the front can I discard"
 * reasoning, so the buffer code here only needs to shuffle bytes, not interpret them. */
#define RS_CMD_RX_BUFSIZE (128u)

static uint32_t rs_malformed_cmd_count = 0;

static void rs_handle_command(uint32_t cmd, uint32_t token, const uint8_t *calib_data,
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
    case RS_CMD_SET_FRAME_PERIOD_US:
    case RS_CMD_SET_EXPOSURE_MS:
    case RS_CMD_REINIT:
        /* Registered (rs_protocol.h) but not implemented until Task 4: explicit
         * not-yet-implemented placeholder rather than silently matching the default. */
        rs_send_ack(token, cmd, RS_RESULT_BUSY, 0u);
        break;
    default:
        rs_send_ack(token, cmd, RS_RESULT_UNKNOWN_CMD, 0u);
        break;
    }
}

static void rs_poll_commands(const uint8_t *calib_data, uint16_t out_width, uint16_t out_height,
                             uint32_t seq_for_calib) {
    static uint8_t rx_buf[RS_CMD_RX_BUFSIZE];
    static uint32_t rx_len = 0;

    /* Drain whatever TinyUSB is holding into the accumulation buffer. */
    while (tud_cdc_available()) {
        uint32_t avail = tud_cdc_available();
        uint32_t space = RS_CMD_RX_BUFSIZE - rx_len;
        if (space == 0) {
            /* A full buffer with no valid command found in it: the host is either
             * out of sync or sending garbage. Resync by dropping everything -- never
             * grow the buffer or block acquisition on the host. */
            rx_len = 0;
            rs_malformed_cmd_count++;
            space = RS_CMD_RX_BUFSIZE;
        }
        uint32_t want = MIN(avail, space);
        uint32_t got = tud_cdc_read(rx_buf + rx_len, want);
        if (got == 0) {
            break;
        }
        rx_len += got;
    }

    /* Parse everything currently available; rs_parse_command tells us exactly how many
     * bytes to drop from the front each step (see rs_protocol.h for the full contract). */
    for (;;) {
        uint32_t cmd, param, token;
        int32_t r = rs_parse_command(rx_buf, rx_len, &cmd, &param, &token);
        if (r == 0) {
            break; /* candidate pending: wait for more RX bytes */
        }
        uint32_t consume = (uint32_t)((r > 0) ? r : -r);
        if (consume > rx_len) {
            consume = rx_len; /* defensive; rs_parse_command never over-reports */
        }
        memmove(rx_buf, rx_buf + consume, rx_len - consume);
        rx_len -= consume;
        if (r > 0) {
            (void)param; /* no Task-2 command consumes param yet (Task 4 will) */
            rs_handle_command(cmd, token, calib_data, out_width, out_height, seq_for_calib);
        } else {
            rs_malformed_cmd_count++;
        }
    }
}
#endif /* !CONF_TRANSFORM_ONBOARD */

static void print_frame(float *p_frame, size_t height, size_t width);
static memory_t allocate_memory(uint16_t size);

void vl53l9_app() {

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
        handle_error(); /* unsupported binning */
    }

    /* sensor reset */
    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        platform_assign_dynamic_address();
    }

    /* initialize sensor and retrieve calibration data */
    ret = vl53l9_init(p_dev);
    if (ret) {
        handle_error();
    }

    uint8_t calib_data[VL53L9_CALIB_DATA_SIZE];
    ret = vl53l9_get_calib_data(p_dev, calib_data);
    if (ret) {
        handle_error();
    }

    vl53l9_utils_set_profile(p_dev, p_profile);

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

    ret = vl53l9_set_sync_mode(p_dev, VL53L9_SYNC_MANUAL);
    if (ret) {
        handle_error();
    }

    ret = vl53l9_start(p_dev);
    if (ret) {
        handle_error();
    }

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
    tud_connect();

#if CONF_STREAM_RAW
    /* Golden-pair captures need frame 1: TNR state is per-pixel and cumulative, so the
     * host must witness the stream from the first processed frame. Hold acquisition
     * until a host opens the CDC port (DTR). This gate is also what makes raw-only mode
     * (CONF_TRANSFORM_ONBOARD=0) golden-capture-compatible, so it stays on by default here
     * too; a headless/production build (no PC waiting on the far end) may want to revisit
     * blocking acquisition start on a host connection. */
    while (!tud_cdc_connected()) {
        tud_task();
    }
    HAL_Delay(50); /* let the host's reader thread settle after opening the port */
#endif

#if CONF_TRANSFORM_ONBOARD
    /* Dual-stream / on-MCU-transform loop: UNCHANGED (golden-pair regeneration path).
     * The raw-only loop with trigger-early overlap lives in the #else branch below. */
    while (1) {

        /* Keep USB serviced every iteration, including frames that skip the
         * send call below (first frame, or a stalled host). */
        tud_task();

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

    rs_trigger_next(p_dev); /* seed trigger for frame 1 */

    while (1) {

        /* Keep USB serviced every iteration, even when waits below return fast. */
        tud_task();

        /* Wait for data-ready. Same bounded-retry disambiguation as the dual-stream
         * loop (Task 8): a timeout means either the trigger was lost (re-trigger, with
         * settle, via rs_trigger_next) or the edge landed after the timeout (poll
         * FRAME_READY, then fall through and ack, clearing any late edge so it cannot
         * leak into the next iteration). */
        int rs_attempts = 0;
        for (;;) {
            ret = rs_wait_event_usb(PLATFORM_GPIO_IT_EVT, 1000);
            if (ret) {
                uint8_t rs_is_ready = 0;
                (void)vl53l9_poll_frame(p_dev, &rs_is_ready);
                if (!rs_is_ready) {
                    if (++rs_attempts > 3) {
                        handle_error();
                    }
                    rs_trigger_next(p_dev); /* trigger lost: re-trigger (no event to ack) */
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
                }
                HAL_Delay(1);
                ret = vl53l9_get_frame_async(p_dev, in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size);
            }
            if (ret) {
                handle_error();
            }
            break;
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

        /* parse frame N's metadata (pure in-memory reads; buffer complete after the
         * readout ack above) so the send below carries frame N's own counter */
        vl53l9_frame_t frame = { 0 };
        ret = vl53l9_utils_parse_frame(in_raw_mem[raw_mem_index].data, in_raw_mem[raw_mem_index].size, &frame);
        if (ret) {
            handle_error();
        }
        uint32_t rs_counter = (uint32_t)frame.p_metadata->frame_counter;

        /* trigger frame N+1 now (settle enforced inside): the sensor integrates while
         * the CDC sends below are in flight */
        rs_trigger_next(p_dev);

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

        /* Command-channel poll point: once per iteration, after this iteration's RAW
         * send (never inside rs_wait_event_usb -- see the channel's block comment
         * above). Non-blocking; PING and SEND_CALIB are handled here, the reconfig
         * commands (SET_USECASE, SET_FRAME_PERIOD_US, SET_EXPOSURE_MS, REINIT) ack
         * BUSY as placeholders (Task 4 implements them). */
        rs_poll_commands(calib_data, out_width, out_height, rs_counter);

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
        handle_error();
    }
    return memory;
}

static void handle_error(void) {
    /* Drop off the USB bus: this spin never services tud_task, so leaving the
     * D+ pull-up asserted would present a dead device to the host (Code 43).
     * Harmless if called before tud_connect (pull-up already off). */
    tud_disconnect();
    vl53l9_status_t status = { 0 };
    vl53l9_get_status(&device[CONF_DEVICE_ID], &status);
    while (1)
        ;
}
