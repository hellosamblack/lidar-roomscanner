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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vl53l9.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include "vl53l9_transform.h"
#include "vl53l9_utils.h"

#include "main.h"
#include "stm32n6xx.h" // SCB_InvalidateDCache_by_Addr

#define LAYER_SCALER     (10)
#define LAYER_X_MAX_SIZE (54 * LAYER_SCALER)
#define LAYER_Y_MAX_SIZE (42 * LAYER_SCALER)
#define LAYER_X_MARGIN   (120)
#define LAYER_Y_MARGIN   (25)

#define CONF_DEVICE_ID (0) /**< select device entry in platform descriptor array (see vl53l9_device.c) */
#define CONF_USECASE   (VL53L9_USECASE_AR_PRECISION) /**< select ranging profile to be applied (see vl53l9_utils.h) */

#ifdef USE_STM32N6XX_NUCLEO /* display not available on discovery kit but not on nucleo boards */
#define CONF_DISPLAY (0)    /**< enable/disable display output */
#else
#define CONF_DISPLAY (1) /**< enable/disable display output */
static void update_display_depth_layer(platform_display_layer_config_t *config, float *depth);
#endif

static memory_t allocate_memory(uint16_t size);
static void handle_error(void);

__attribute__((aligned(32))) volatile uint8_t g_csi_output_buffer[14900]; /* 100 * 149 (csi_width * csi_height) */
volatile uint8_t g_lcd_layer_depth[LAYER_Y_MAX_SIZE * LAYER_X_MAX_SIZE];

void vl53l9_app() {

    int ret;
    transform_t *p_transform = vl53l9_transform_create();
    vl53l9_device_t *p_dev = &device[CONF_DEVICE_ID];
    vl53l9_profile_t *p_profile = &g_ranging_profiles[CONF_USECASE];
    vl53l9_hw_config_t hw_config;

    uint16_t raw_buffer_size, frame_buffer_size; /* bytes */
    uint16_t in_width, in_height;                /* pixels */
    uint16_t out_width, out_height;              /* pixels */

    vl53l9_get_raw_buffer_size(p_profile->binning, &raw_buffer_size);
    vl53l9_utils_get_csi_resolution(p_profile->binning, &in_width, &in_height);
    vl53l9_utils_get_frame_resolution(p_profile->binning, &out_width, &out_height);
    frame_buffer_size = out_width * out_height * sizeof(float);

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

    /* retrieve and override output interface parameters */
    ret = vl53l9_get_hw_config(p_dev, &hw_config);
    if (ret) {
        handle_error();
    }

    hw_config.output_interface = VL53L9_OUTPUT_CSI2;
    hw_config.signaling_mode = true;
    hw_config.csi_data_rate = 1e9;
    hw_config.csi_virtual_channel = 0;
    hw_config.csi_status_line_force_width = false;
    hw_config.csi_status_line_datatype = 0x2A;
    hw_config.csi_frame_datatype = 0x2A;
    hw_config.csi_frame_height = in_height - 1; /* no need to consider last row containing status line */
    hw_config.csi_frame_width = in_width;

    ret = vl53l9_set_hw_config(p_dev, hw_config);
    if (ret) {
        handle_error();
    }
    ret = platform_start_csi_pipe((uint8_t *)g_csi_output_buffer);
    if (ret) {
        handle_error();
    }

    /* initialize processing pipeline */
    ret = transform_initialize(p_transform);
    if (ret) {
        handle_error();
    }

    /* inspect available streams, capabilities and controls */
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

    /* free properties and capabilities */
    /* TODO: improve using free functions */
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

    ret = transform_set_control(p_transform, "bypass-tnr-algo", (value_t){ .val.v_bool = false, .tid = VTID_BOOL });
    if (ret) {
        handle_error();
    }

    /* check pipeline configuration and compute internal parameters required for processing */
    ret = transform_prepare(p_transform);
    if (ret) {
        handle_error();
    }

    /* allocate memory and initialize buffers */
    memory_t in_raw_mem = allocate_memory(raw_buffer_size);
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

    ret = vl53l9_start(p_dev);
    if (ret) {
        handle_error();
    }

    platform_profiler_enable();
    uint32_t start_time = platform_profiler_get_timestamp();
    uint32_t stop_time;
    float frame_rate;
    uint32_t previous_frame_counter = 0;

#if CONF_DISPLAY
    /* configure display */
    platform_display_layer_config_t layer_config = { .x_size = out_width,
                                                     .y_size = out_height,
                                                     .x_margin = LAYER_X_MARGIN,
                                                     .y_margin = LAYER_Y_MARGIN,
                                                     .scaler = LAYER_SCALER * (p_profile->binning / 2),
                                                     .frame_buffer = (uint32_t *)g_lcd_layer_depth };

    platform_display_enable();
    platform_display_configure_layer(&layer_config, LTDC_LAYER_1);
    platform_display_set_color_lut(LTDC_LAYER_1);
#endif /* CONF_DISPLAY */

    while (1) {

        ret = platform_wait_for_event(PLATFORM_CAM_PIPE_FRAME_EVT, 1000);
        if (ret) {
            handle_error();
        }

        platform_acknowledge_event(PLATFORM_CAM_PIPE_FRAME_EVT);

        /* invalidate cache to ensure data coherency */
        SCB_InvalidateDCache_by_Addr((uint32_t *)g_csi_output_buffer,
                                     sizeof(g_csi_output_buffer)); /* TODO: abstract this call */

        vl53l9_frame_t frame = { 0 };
        uint32_t csi_buffer_size = in_width * in_height;
        ret = vl53l9_utils_parse_frame((uint8_t *)g_csi_output_buffer, csi_buffer_size, &frame);
        if (ret) {
            handle_error();
        }
        /* copy frame and skip csi padding */
        ret = vl53l9_utils_dump_csi_frame(&frame, (uint8_t *)in_raw_mem.data, in_raw_mem.size);
        if (ret) {
            handle_error();
        }
        /* process the previous frame while the sensor is acquiring the next one */
        ret = transform_process_stream(p_transform, &stream_buffers);
        if (ret) {
            handle_error();
        }

#if CONF_DISPLAY
        update_display_depth_layer(&layer_config, (float *)out_depth_mem.data);
#endif /* CONF_DISPLAY */

        /* measure frame rate */
        stop_time = platform_profiler_get_timestamp();
        frame_rate = (1.0f / (float)(platform_profiler_convert_to_us(stop_time - start_time))) * 1000000;
        start_time = stop_time;

        printf("Processed frame n. %lu @ %u fps  (missed frames = %d) \n", frame.p_metadata->frame_counter,
               (unsigned int)frame_rate, (int)(frame.p_metadata->frame_counter - 1 - previous_frame_counter));

        previous_frame_counter = frame.p_metadata->frame_counter;
    }

    /* NOTE: free memory and pipeline resources */
    /* free(in_raw_mem.data); */
    /* free(out_depth_mem.data); */
    /* transform_finalize(p_transform); */
    /* transform_release(p_transform); */
    /* vl53l9_transform_destroy(p_transform); */
}

#if CONF_DISPLAY
static void update_display_depth_layer(platform_display_layer_config_t *config, float *depth) {

    const float filtered_pixel_distance = 12000.0f;
    const uint32_t scaling_factor = 38; /* scale depth values to 0-255 range accordingly to max distance */
    const uint32_t stride = config->x_size * config->scaler;
    uint8_t *frame_buffer = (uint8_t *)config->frame_buffer;

    for (uint32_t r = 0; r < config->y_size; r++) {
        for (uint32_t c = 0; c < config->x_size; c++) {
            for (uint32_t lr = 0; lr < config->scaler; lr++) {
                for (uint32_t lc = 0; lc < config->scaler; lc++) {
                    uint32_t upscaled_x = c * config->scaler + lc;
                    uint32_t upscaled_y = r * config->scaler + lr;
                    uint32_t lcd_idx = upscaled_y * stride + upscaled_x;
                    uint32_t depth_idx = r * config->x_size + c;

                    if (depth[depth_idx] >= filtered_pixel_distance) {
                        /* Keep the last LUT entry reserved for filtered pixels. */
                        frame_buffer[lcd_idx] = 255;
                    } else {
                        uint32_t depth_value = (uint32_t)(depth[depth_idx] / scaling_factor);
                        frame_buffer[lcd_idx] = (uint8_t)(depth_value > 254 ? 254 : depth_value);
                    }
                }
            }
        }
    }
}
#endif /* CONF_DISPLAY */

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
    while (1)
        ;
}
