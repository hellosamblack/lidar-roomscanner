/**
 ******************************************************************************
 * @file    vl53l9_device.c
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

#include "vl53l9_device.h"
#include "main.h"

extern I3C_HandleTypeDef hi3c1;

// NOTE: add more entries to this array in case of multiple sensors on the board

vl53l9_device_t device[NB_DEVICES] = { { .bus = &hi3c1,
                                         .bus_type = PLATFORM_BUS_I3C,
                                         .bus_property = PLATFORM_BUS_PROPERTY_NONE,
                                         .address = VL53L9_DEFAULT_ADDRESS,
                                         .vdda = VDDA_2V8,
                                         .vddio = VDDIO_1V8,
                                         .ext_clock = 12.0e6, // NOTE: make sure SW1 is set to INT
                                         .intr = { .pin = INTR_Pin, .port = INTR_GPIO_Port },
                                         .xshut = { .pin = XSHUT_Pin, .port = XSHUT_GPIO_Port } } };
