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

#include <stdio.h>
#include <stdlib.h>

#include "vl53l9.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include "vl53l9_utils.h"

#define CONF_DEVICE_ID   (0) /**< select device entry in platform descriptor array (see vl53l9_device.c) */
#define CONF_PRINT_FRAME (0) /**< enable printing depth frames as ascii art (slows performance) */
#define CONF_USECASE     (VL53L9_USECASE_AR_PRECISION) /**< select ranging profile to be applied (see vl53l9_utils.h) */

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

static void print_frame(const vl53l9_frame_t frame);
static void handle_error(void);

void vl53l9_app() {

    int ret;
    vl53l9_device_t *p_dev = &device[CONF_DEVICE_ID];
    vl53l9_profile_t *p_profile = &g_ranging_profiles[CONF_USECASE];
    uint16_t buffer_size;

    vl53l9_get_raw_buffer_size(p_profile->binning, &buffer_size);
    uint8_t *p_buffer = malloc(buffer_size);

    platform_power_reset(CONF_DEVICE_ID);
    if (p_dev->bus_type & PLATFORM_BUS_I3C) {
        platform_assign_dynamic_address();
    }

    ret = vl53l9_init(p_dev);
    if (ret) {
        handle_error();
    }

    vl53l9_utils_set_profile(p_dev, p_profile);

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

    while (1) {

        ret = vl53l9_trigger_frame(p_dev);
        if (ret) {
            handle_error();
        }

        ret = platform_wait_for_event(PLATFORM_GPIO_IT_EVT, 1000);
        if (ret) {
            handle_error();
        }

        platform_acknowledge_event(PLATFORM_GPIO_IT_EVT);

        ret = vl53l9_get_frame(p_dev, p_buffer, buffer_size);
        if (ret) {
            handle_error();
        }

        vl53l9_frame_t frame = { 0 };
        ret = vl53l9_utils_parse_frame(p_buffer, buffer_size, &frame);
        if (ret) {
            handle_error();
        }

        /* measure frame rate */
        stop_time = platform_profiler_get_timestamp();
        frame_rate = (1.0f / (float)(platform_profiler_convert_to_us(stop_time - start_time))) * 1000000;
        start_time = stop_time;

#if CONF_PRINT_FRAME
        print_frame(frame);
#endif /* CONF_PRINT_FRAME */

        printf("Frame n. %lu @ %u fps\n", frame.p_metadata->frame_counter, (unsigned int)frame_rate);
    }
}

static void print_frame(const vl53l9_frame_t frame) {

    static const char ASCII_CHARS[] = "@%#*+=-:. ";

    printf("\033[%d;%dH", 0, 0); /* set cursor to the top of the screen */

    int pixel_step = 1;
    uint16_t min = UINT16_MAX;
    uint16_t max = 0;

    for (int i = 0; i < (frame.p_metadata->frame_height * frame.p_metadata->frame_width); i++) {
        uint16_t value = frame.p_distance[i].value;
        min = MIN(value, min);
        max = MAX(value, max);
    }

    uint16_t average = (max - min) * 0.05;
    min = MAX(min - average, 0);
    max = MIN(max + average, UINT16_MAX);

    for (int y = 0; y < frame.p_metadata->frame_height; y += pixel_step) {
        for (int x = 0; x < frame.p_metadata->frame_width; x += pixel_step) {
            int pixel_index = (y * frame.p_metadata->frame_width + x);
            uint16_t value = frame.p_distance[pixel_index].value;

            int ascii_index = (value - min) * (sizeof(ASCII_CHARS) - 1) / (max - min);
            ascii_index = MAX(0, MIN(ascii_index, sizeof(ASCII_CHARS) - 1));

            printf("%c", ASCII_CHARS[ascii_index]);
        }
        printf("\n");
    }
}

static void handle_error(void) {
    while (1)
        ;
}
