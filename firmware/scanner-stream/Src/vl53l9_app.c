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

#include "vl53l9.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include "vl53l9_transform.h"
#include "vl53l9_utils.h"

/* application customization */
#define CONF_DEVICE_ID   (0) /**< select device entry in platform descriptor array (see vl53l9_device.c) */
#define CONF_PRINT_FRAME   (0) /**< ASCII art disabled in streaming builds */
#define CONF_STREAM_BINARY (1) /**< emit rs_protocol frames on the COM1 UART */
#define CONF_USECASE     (VL53L9_USECASE_AR_PRECISION) /**< select ranging profile to be applied (see vl53l9_utils.h) */

#include "rs_protocol.h"
#include "stm32h5xx_nucleo.h"
#include "tusb.h"

extern UART_HandleTypeDef hcom_uart[];

static void handle_error(void);

static uint64_t rs_time_us(void) {
    /* v1: HAL tick, 1 ms resolution widened to the u64 µs wire field.
     * Upgrade to a TIM-based µs clock when IMU fusion needs it (Phase 5). */
    return (uint64_t)HAL_GetTick() * 1000u;
}

static void rs_send_depth_uart(uint32_t seq, uint8_t flags, const uint8_t *payload,
                               uint32_t len, uint16_t w, uint16_t h) {
    uint8_t hdr[RS_HEADER_SIZE];
    uint8_t tail[4];
    /* HAL_UART_Transmit's u16 length bounds a single transfer; if a future
     * profile produces >65535 B payloads, Phase 3 must chunk the send. */
    if (len > 0xFFFFu) {
        handle_error();
    }
    rs_write_header(hdr, RS_FRAME_DATA, RS_STREAM_DEPTH_ZF32, flags, seq, rs_time_us(), w, h, len);
    uint32_t crc = rs_crc32(0u, hdr, RS_HEADER_SIZE);
    crc = rs_crc32(crc, payload, len);
    rs_put_u32(tail, crc);
    HAL_UART_Transmit(&hcom_uart[COM1], hdr, RS_HEADER_SIZE, 1000);
    HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t *)payload, (uint16_t)len, 1000);
    HAL_UART_Transmit(&hcom_uart[COM1], tail, 4, 1000);
}

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static void print_frame(float *p_frame, size_t height, size_t width);
static memory_t allocate_memory(uint16_t size);
static void handle_error(void);

void vl53l9_app() {

    int ret;
    transform_t *p_transform = vl53l9_transform_create();
    vl53l9_device_t *p_dev = &device[CONF_DEVICE_ID];
    vl53l9_profile_t *p_profile = &g_ranging_profiles[CONF_USECASE];

    uint16_t raw_buffer_size = 0, frame_buffer_size = 0; /* bytes */
    uint32_t in_width = 0, in_height = 0;                /* pixels */
    uint8_t out_width = 0, out_height = 0;               /* pixels */
    vl53l9_get_raw_buffer_size(p_profile->binning, &raw_buffer_size);
    vl53l9_utils_get_resolution(p_profile->binning, &out_width, &out_height);
    frame_buffer_size = out_width * out_height * sizeof(float);

    if (p_profile->binning == 2) {
        in_width = 14842;
        in_height = 1;
    } else if (p_profile->binning == 4) {
        in_width = 3844;
        in_height = 1;
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

    /* allocate memory and initialize buffers (raw data is double buffered) */
    uint8_t raw_mem_index = 0;
    memory_t in_raw_mem[2] = { allocate_memory(raw_buffer_size), allocate_memory(raw_buffer_size) };
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

    bool is_first_frame = true;

    uint32_t rs_prev_counter = 0;
    bool rs_have_prev = false;

    while (1) {

        /* TASK10 TEMP: CDC heartbeat, removed in Task 11 */
        tud_task();
        if (tud_cdc_connected()) {
            tud_cdc_write_str("hb\r\n");
            tud_cdc_write_flush();
        }

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

            ret = platform_wait_for_event(PLATFORM_GPIO_IT_EVT, 1000);
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
            /* TODO: find a better way to handle this, maybe leveraging mems list */
            in_raw_mems.items = &in_raw_mem[(raw_mem_index + 1) % 2];
            ret = transform_process_stream(p_transform, &stream_buffers);
            if (ret) {
                handle_error();
            }
#if CONF_STREAM_BINARY
            if (rs_have_prev) {
                rs_send_depth_uart(rs_prev_counter, 0u, (const uint8_t *)out_depth_mem.data,
                                   frame_buffer_size, out_width, out_height);
            }
#endif
        }

        ret = platform_wait_for_event(PLATFORM_I3C_DMA_RX_EVT, 1000);
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
#if !CONF_STREAM_BINARY
        print_frame((float *)out_depth_mem.data, out_height, out_width);
        printf("Processed frame n. %lu @ %u fps\n", (unsigned long)frame.p_metadata->frame_counter,
               (unsigned int)frame_rate);
#endif

        /* swap raw buffer index for next frame acquisition */
        raw_mem_index = (raw_mem_index + 1) % 2;
    }

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
    vl53l9_status_t status = { 0 };
    vl53l9_get_status(&device[CONF_DEVICE_ID], &status);
    while (1)
        ;
}
