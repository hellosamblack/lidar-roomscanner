/**
 ******************************************************************************
 * @file    platform_utils.c
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

#include "main.h"
#include "stm32h5xx_hal.h"
#include "stm32h5xx_hal_i3c.h"
#include "vl53l9_device.h"
#include "vl53l9_interface.h"
#include <stdint.h>
#include <string.h>

// when updating use semantic versioning (https://semver.org/)
#define FW_MAJOR (1)
#define FW_MINOR (0)
#define FW_PATCH (0)

/* global variables */

extern I3C_HandleTypeDef hi3c1;

platform_gpio_t g_debug_gpio_1 = { DEBUG_GPIO_1_Pin, DEBUG_GPIO_1_GPIO_Port };
platform_gpio_t g_debug_gpio_2 = { DEBUG_GPIO_2_Pin, DEBUG_GPIO_2_GPIO_Port };

static volatile platform_event_t g_platform_evt;

/* private functions */

static int _timeout_expire(uint32_t to_start, uint32_t to_value) {
    uint64_t currentlong = HAL_GetTick();
    uint32_t current = (uint32_t)currentlong;

    return current >= to_start ? (current - to_start) >= to_value : (current + (0xffffffff - to_start) + 1) >= to_value;
}

/* exported functions */

/**
 * @brief get firmware version
 * @param version structure filled with version details
 * @return 0 if success
 */
int platform_get_version(platform_version_t *version) {

    version->interface = (_version_t){ .major = INTERFACE_MAJOR, .minor = INTERFACE_MINOR, .patch = INTERFACE_PATCH };
    version->firmware = (_version_t){ .major = FW_MAJOR, .minor = FW_MINOR, .patch = FW_PATCH };
    version->driver =
        (_version_t){ .major = VL53L9_CORE_MAJOR, .minor = VL53L9_CORE_MINOR, .patch = VL53L9_CORE_PATCH };

    strncpy(version->board_name, "nucleo-h563", BOARD_NAME_STR_SIZE);

    return 0;
}

/* device power management */

/**
 * @brief Reset a device
 * @param[in] id Device identifier
 * @return 0 in case of success, negative value otherwise
 */
int platform_power_reset(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_SET);
    HAL_Delay(50);
    return 0;
}

/**
 * @brief Power-up a device
 * @param[in] id Device identifier
 * @return 0 in case of success, negative value otherwise
 */
int platform_power_enable(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_SET);
    HAL_Delay(50);
    return 0;
}

/**
 * @brief Power-down a device
 * @param[in] id Device identifier
 * @return 0 in case of success, negative value otherwise
 */
int platform_power_disable(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    return 0;
}

/* i2c/i3c interfaces */

/**
 * @brief Update the I2C static address stored in the device descriptor
 *
 * This method is meant to be called after requesting the device to change its I2C static address.
 * In order to finalize the change on the platform and ensure coherency, the address must be updated in the device
 * descriptor as well.
 *
 * @param[in] id Instance identifier of the device
 * @param[in] address New address to be used (7-bit format)
 * @return 0 in case of success, negative value otherwise
 */
int platform_set_device_address(uint8_t id, uint8_t address) {
    if (device[id].bus_type == PLATFORM_BUS_I2C) {
        device[id].address = address & 0x7F;
        return 0;
    } else {
        return -1;
    }
}

int platform_assign_dynamic_address() {

    HAL_StatusTypeDef status;
    uint64_t payload;

    // set i3c bus frequency to 1 MHz before dynamic address assignment
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    // NOTE: for the moment apply static address as dynamic address
    do {
        status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
        if (status == HAL_BUSY) {
            HAL_I3C_Ctrl_SetDynAddr(&hi3c1, 0x52 & 0x7F);
        }
    } while (status == HAL_BUSY);

    // set i3c bus frequency to 12 MHz after dynamic address assignement
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x0a;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x59;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    // required to register ibi notifications
    I3C_DeviceConfTypeDef DeviceConf;
    DeviceConf.DeviceIndex = 1;
    DeviceConf.TargetDynamicAddr = 0x52 & 0x7F;
    DeviceConf.IBIAck = __HAL_I3C_GET_IBI_CAPABLE(__HAL_I3C_GET_BCR(payload));
    DeviceConf.IBIPayload = __HAL_I3C_GET_IBI_PAYLOAD(__HAL_I3C_GET_BCR(payload));
    DeviceConf.CtrlRoleReqAck = __HAL_I3C_GET_CR_CAPABLE(__HAL_I3C_GET_BCR(payload));
    DeviceConf.CtrlStopTransfer = DISABLE;

    if (HAL_I3C_Ctrl_ConfigBusDevices(&hi3c1, &DeviceConf, 1U) != HAL_OK) {
        Error_Handler();
    }

    return 0;
}

int platform_assign_dynamic_address_multisensor() {

    HAL_StatusTypeDef status;
    uint64_t payload;
    I3C_ENTDAAPayloadTypeDef payload_info;
    uint8_t address = VL53L9_DEFAULT_ADDRESS;

    // set i3c bus frequency to 1 MHz before dynamic address assignment
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x7c;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x7c;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    // NOTE: for the moment apply static address as dynamic address
    do {
        payload = 0;
        status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
        if (status == HAL_BUSY) {
            HAL_I3C_Get_ENTDAA_Payload_Info(&hi3c1, payload, &payload_info);

            for (int i = 0; i < NB_DEVICES; i++) {
                if (device[i].instance_id == payload_info.PID.MIPIID) {
                    address = device[i].address;
                    break;
                }
            }
            HAL_I3C_Ctrl_SetDynAddr(&hi3c1, address & 0x7F);
        }
    } while (status == HAL_BUSY);

    // set i3c bus frequency to 12 MHz after dynamic address assignement
    hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x0a;
    hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
    hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x59;
    if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
        return -1;
    }

    // TODO: add HAL_I3C_Ctrl_ConfigBusDevices call to enable ibi notifications

    return 0;
}

int platform_ctrl_gpio(platform_gpio_t gpio, platform_gpio_state_t state) {
    switch (state) {
    case PLATFORM_GPIO_STATE_RESET:
        HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_RESET);
        break;
    case PLATFORM_GPIO_STATE_SET:
        HAL_GPIO_WritePin(gpio.port, gpio.pin, GPIO_PIN_SET);
        break;
    case PLATFORM_GPIO_STATE_TOGGLE:
        HAL_GPIO_TogglePin(gpio.port, gpio.pin);
        break;
    default:
        return -1;
        break;
    }
    return 0;
}

/* profiling */

int platform_profiler_enable() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable counter
    return 0;
}

int platform_profiler_disable() {
    // TODO: clear CoreDebug_DEMCR_TRCENA_Msk in DEMCR register
    // TODO: disable counter
    return 0;
}

uint32_t platform_profiler_get_timestamp() {
    return DWT->CYCCNT;
}

uint32_t platform_profiler_convert_to_us(uint32_t timestamp) {
    uint32_t tick_per_1us = SystemCoreClock / 1000000;
    return timestamp / tick_per_1us;
}

/* event handling */

int platform_enable_event(platform_event_t event) {
    int res = 0;
    switch (event) {
    case PLATFORM_I3C_IBI_EVT:
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
        HAL_I3C_ActivateNotification(&hi3c1, NULL, LL_I3C_IER_IBIIE);
        HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
        break;
    case PLATFORM_GPIO_IT_EVT:
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        HAL_NVIC_EnableIRQ(EXTI7_IRQn);
        break;
    default:
        res = -1;
        break;
    }
    return res;
}

int platform_disable_event(platform_event_t event) {
    int res = 0;
    switch (event) {
    case PLATFORM_I3C_IBI_EVT:
        HAL_I3C_DeactivateNotification(&hi3c1, LL_I3C_IER_IBIIE);
        HAL_NVIC_DisableIRQ(I3C1_EV_IRQn);
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
        break;
    case PLATFORM_GPIO_IT_EVT:
        HAL_NVIC_DisableIRQ(EXTI7_IRQn);
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        break;
    default:
        res = -1; // not supported
        break;
    }
    return res;
}

int platform_wait_for_event(platform_event_t event, uint32_t timeout_ms) {
    int res = 0;
    uint64_t to_startlong = HAL_GetTick();
    uint32_t to_start = (uint32_t)to_startlong;
    while (!(event & g_platform_evt) && !_timeout_expire(to_start, timeout_ms))
        ;

    if (!(event & g_platform_evt)) {
        res = -1;
    }
    return res;
}

int platform_acknowledge_event(platform_event_t event) {
    int res = 0;
    switch (event) {
    case PLATFORM_GPIO_IT_EVT:
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        break;
    case PLATFORM_I3C_DMA_RX_EVT:
        g_platform_evt &= ~PLATFORM_I3C_DMA_RX_EVT;
        break;
    case PLATFORM_I3C_IBI_EVT:
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
    default:
        res = -1;
        break;
    }

    return res;
}

int platform_get_event_status(platform_event_t event, bool *active) {
    *active = (g_platform_evt & event) ? true : false;
    return 0;
}

/* HAL callbacks */

void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c) {
    g_platform_evt |= PLATFORM_I3C_DMA_RX_EVT;
}

void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *hi3c, uint32_t eventId) {
    if ((eventId & EVENT_ID_IBI) == EVENT_ID_IBI) {
        g_platform_evt |= PLATFORM_I3C_IBI_EVT;
    }
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    for (uint8_t i = 0; i < NB_DEVICES; i++) {
        if (GPIO_Pin == device[i].intr.pin) {
            g_platform_evt |= PLATFORM_GPIO_IT_EVT;
        }
    }
}

/* csi interface */

int platform_start_csi_pipe(uint8_t *buff_csi) {
    return -1; // not supported
}

int platform_stop_csi_pipe() {
    return -1; // not supported
}
