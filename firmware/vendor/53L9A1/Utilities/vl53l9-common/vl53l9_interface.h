/**
 ******************************************************************************
 * @file    vl53l9_interface.h
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

#ifndef VL53L9_INTERFACE_H
#define VL53L9_INTERFACE_H

#include "vl53l9.h"
#include <stdint.h>

// exported symbols

// when updating use semantic versioning (https://semver.org/)
#define INTERFACE_MAJOR (1U)
#define INTERFACE_MINOR (0U)
#define INTERFACE_PATCH (0U)

#define BOARD_NAME_STR_SIZE (20U)

// exported types
typedef struct {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
} _version_t;

typedef struct {
    _version_t interface;
    _version_t firmware;
    _version_t driver;
    char board_name[BOARD_NAME_STR_SIZE];
} platform_version_t;

typedef enum {
    PLATFORM_GPIO_STATE_SET = 0,
    PLATFORM_GPIO_STATE_RESET = 1,
    PLATFORM_GPIO_STATE_TOGGLE = 2
} platform_gpio_state_t;

typedef struct {
    uint16_t pin;
    void *port;
} platform_gpio_t;

typedef enum {
    PLATFORM_NONE_EVT = 0,
    PLATFORM_GPIO_IT_EVT = 1,
    PLATFORM_I3C_IBI_EVT = 2,
    PLATFORM_I3C_DMA_RX_EVT = 4,
    PLATFORM_I3C_DAA_EVT = 8,
    PLATFORM_CAM_PIPE_FRAME_EVT = 16,
} platform_event_t;

typedef enum {
    PLATFORM_BUS_I2C = 1,
    PLATFORM_BUS_I3C = 2,
    PLATFORM_BUS_CSI = 4,
} platform_bus_type_t;

typedef enum {
    PLATFORM_BUS_PROPERTY_NONE = 0,
    PLATFORM_BUS_PROPERTY_I3C_LEGACY = 1,
    PLATFORM_BUS_PROPERTY_I3C_IBI = 2,
} platform_bus_property_t;

typedef struct {
    void *bus;
    platform_bus_type_t bus_type;
    platform_bus_property_t bus_property;
    uint8_t address;
    vl53l9_vdda_t vdda;
    vl53l9_vddio_t vddio;
    uint32_t ext_clock;
    uint8_t instance_id;
    platform_gpio_t xshut;
    platform_gpio_t intr;
} vl53l9_device_t;

// definition of external variables

extern platform_gpio_t g_debug_gpio_1;
extern platform_gpio_t g_debug_gpio_2;

// exported functions
int platform_get_version(platform_version_t *version);
int platform_power_reset(uint8_t id);
int platform_power_enable(uint8_t id);
int platform_power_disable(uint8_t id);
int platform_set_device_address(uint8_t id, uint8_t address);
int platform_assign_dynamic_address(void);
int platform_assign_dynamic_address_multisensor(void);
int platform_profiler_enable(void);
int platform_profiler_disable(void);
uint32_t platform_profiler_get_timestamp(void);
uint32_t platform_profiler_convert_to_us(uint32_t timestamp);
int platform_start_csi_pipe(uint8_t *buff_csi);
int platform_stop_csi_pipe(void);
int platform_enable_event(platform_event_t event);
int platform_disable_event(platform_event_t event);
int platform_acknowledge_event(platform_event_t event);
int platform_wait_for_event(platform_event_t event, uint32_t timeout_ms);
int platform_get_event_status(platform_event_t event, bool *active);
int platform_ctrl_gpio(platform_gpio_t gpio, platform_gpio_state_t state);

#endif // VL53L9_INTERFACE_H
