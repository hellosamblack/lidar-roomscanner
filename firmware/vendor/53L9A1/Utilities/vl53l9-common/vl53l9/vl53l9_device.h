/**
 ******************************************************************************
 * @file    vl53l9_device.h
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

#ifndef VL53L9_DEVICE_H
#define VL53L9_DEVICE_H

#include "vl53l9_interface.h"

#define NB_DEVICES (1) // NOTE: customize this value according to the number of sensors on the board

extern vl53l9_device_t device[NB_DEVICES];

#endif // VL53L9_DEVICE_H
