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

// private includes
#include "stm32n6xx_hal.h"

#include "vl53l9_device.h"
#include "vl53l9_interface.h"

#include <stdio.h>
#include <string.h>

// when updating use semantic versioning (https://semver.org/)
#define FW_MAJOR (1)
#define FW_MINOR (1)
#define FW_PATCH (0)

extern DCMIPP_HandleTypeDef hdcmipp;
extern I3C_HandleTypeDef hi3c1;
extern LTDC_HandleTypeDef hltdc;

static volatile platform_event_t g_platform_evt;

// the following are used during the I3C DAA procedure
static unsigned int dev_count = 0;
static uint64_t dev_payload[NB_DEVICES];

/* private functions */

static int _timeout_expire(uint32_t to_start, uint32_t to_value) {
    uint64_t currentlong = HAL_GetTick();
    uint32_t current = (uint32_t)currentlong;

    return current >= to_start ? (current - to_start) >= to_value : (current + (0xffffffff - to_start) + 1) >= to_value;
}

/* exported functions */

int platform_delay(uint32_t delay_ms) {
    HAL_Delay(delay_ms);
    return 0;
}

int platform_get_version(platform_version_t *version) {

    version->interface = (_version_t){ .major = INTERFACE_MAJOR, .minor = INTERFACE_MINOR, .patch = INTERFACE_PATCH };
    version->firmware = (_version_t){ .major = FW_MAJOR, .minor = FW_MINOR, .patch = FW_PATCH };
    version->driver =
        (_version_t){ .major = VL53L9_CORE_MAJOR, .minor = VL53L9_CORE_MINOR, .patch = VL53L9_CORE_PATCH };

    strncpy(version->board_name, "stm32n6570-dk", BOARD_NAME_STR_SIZE);

    return 0;
}

/* device power management */

int platform_power_reset(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_SET);
    HAL_Delay(50);
    return 0;
}

int platform_power_enable(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_SET);
    HAL_Delay(50);
    return 0;
}

int platform_power_disable(uint8_t id) {
    HAL_GPIO_WritePin((GPIO_TypeDef *)device[id].xshut.port, device[id].xshut.pin, GPIO_PIN_RESET);
    HAL_Delay(50);
    return 0;
}

/* i2c/i3c interfaces */

int platform_set_device_address(uint8_t id, uint8_t address) {
    if (device[id].bus_type == PLATFORM_BUS_I2C) { // TODO: replace with I3C_LEGACY
        device[id].address = address & 0x7F;
        return 0;
    } else {
        return -1;
    }
}

int platform_assign_dynamic_address() {
    // TODO: how to handle dynamic address assignment for multiple devices?

    uint8_t address = 0x52;

    if (device[0].bus_type & PLATFORM_BUS_I3C) {
        HAL_StatusTypeDef status;
        uint64_t payload;

        if (HAL_I3C_DeInit(&hi3c1) != HAL_OK) {
            return -1;
        }

        // set i3c bus frequency to 1 MHz before dynamic address assignment
        hi3c1.Instance = I3C1;
        hi3c1.Mode = HAL_I3C_MODE_CONTROLLER;
        hi3c1.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
        hi3c1.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
        hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x63;
        hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x63;
        hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x63;
        hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
        hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x27;
        hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xc6;
        if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
            return -1;
        }

        // NOTE: for the moment apply static address as dynamic address
        do {
            status = HAL_I3C_Ctrl_DynAddrAssign(&hi3c1, &payload, I3C_RSTDAA_THEN_ENTDAA, 5000);
            if (status == HAL_BUSY) {
                HAL_I3C_Ctrl_SetDynAddr(&hi3c1, address & 0x7F);
                dev_count++;
                dev_payload[dev_count - 1] = payload;
            }
        } while (status == HAL_BUSY);

        // set i3c bus frequency to 10 MHz after dynamic address assignement
        if (HAL_I3C_DeInit(&hi3c1) != HAL_OK) {
            return -1;
        }
        hi3c1.Instance = I3C1;
        hi3c1.Mode = HAL_I3C_MODE_CONTROLLER;
        hi3c1.Init.CtrlBusCharacteristic.SDAHoldTime = HAL_I3C_SDA_HOLD_TIME_1_5;
        hi3c1.Init.CtrlBusCharacteristic.WaitTime = HAL_I3C_OWN_ACTIVITY_STATE_0;
        hi3c1.Init.CtrlBusCharacteristic.SCLPPLowDuration = 0x09;
        hi3c1.Init.CtrlBusCharacteristic.SCLI3CHighDuration = 0x09;
        hi3c1.Init.CtrlBusCharacteristic.SCLODLowDuration = 0x47;
        hi3c1.Init.CtrlBusCharacteristic.SCLI2CHighDuration = 0x00;
        hi3c1.Init.CtrlBusCharacteristic.BusFreeDuration = 0x27;
        hi3c1.Init.CtrlBusCharacteristic.BusIdleDuration = 0xc6;
        if (HAL_I3C_Init(&hi3c1) != HAL_OK) {
            return -1;
        }

        // TODO: enable ibi notifications (up to 4 devices only)
    }
    return 0;
}

int platform_assign_dynamic_address_multisensor() {
    // TODO: how to handle dynamic address assignment for multiple devices?

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
    uint8_t res = 0;
    switch (event) {
    case PLATFORM_I3C_IBI_EVT:
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
        HAL_I3C_ActivateNotification(&hi3c1, NULL, LL_I3C_IER_IBIIE);
        HAL_NVIC_EnableIRQ(I3C1_EV_IRQn);
        break;
    case PLATFORM_GPIO_IT_EVT:
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
        HAL_NVIC_EnableIRQ(EXTI6_IRQn);
        break;
    case PLATFORM_CAM_PIPE_FRAME_EVT:
        g_platform_evt &= ~PLATFORM_CAM_PIPE_FRAME_EVT;
        HAL_NVIC_EnableIRQ(DCMIPP_IRQn);
        HAL_NVIC_EnableIRQ(CSI_IRQn);
        break;
    default:
        res = -1;
        break;
    }
    return res;
}

int platform_disable_event(platform_event_t event) {
    uint8_t res = 0;
    switch (event) {
    case PLATFORM_I3C_IBI_EVT:
        HAL_I3C_DeactivateNotification(&hi3c1, LL_I3C_IER_IBIIE);
        HAL_NVIC_DisableIRQ(I3C1_EV_IRQn);
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
        break;
    case PLATFORM_GPIO_IT_EVT:
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
        HAL_NVIC_DisableIRQ(EXTI6_IRQn);
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        break;
    case PLATFORM_CAM_PIPE_FRAME_EVT:
        HAL_NVIC_DisableIRQ(DCMIPP_IRQn);
        HAL_NVIC_DisableIRQ(CSI_IRQn);
        g_platform_evt &= ~PLATFORM_CAM_PIPE_FRAME_EVT;
        break;
    default:
        res = -1;
        break;
    }
    return res;
}

int platform_wait_for_event(platform_event_t event, uint32_t timeout_ms) {
    uint8_t res = 0;
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
    uint8_t res = 0;
    switch (event) {
    case PLATFORM_GPIO_IT_EVT:
        g_platform_evt &= ~PLATFORM_GPIO_IT_EVT;
        break;
    case PLATFORM_CAM_PIPE_FRAME_EVT:
        g_platform_evt &= ~PLATFORM_CAM_PIPE_FRAME_EVT;
        break;
    case PLATFORM_I3C_DMA_RX_EVT:
        g_platform_evt &= ~PLATFORM_I3C_DMA_RX_EVT;
        break;
    case PLATFORM_I3C_IBI_EVT:
        g_platform_evt &= ~PLATFORM_I3C_IBI_EVT;
        break;
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

void HAL_I3C_TgtReqDynamicAddrCallback(I3C_HandleTypeDef *hi3c, uint64_t targetPayload) {
    dev_payload[dev_count] = targetPayload;
    HAL_I3C_Ctrl_SetDynAddr(hi3c, device[dev_count++].address);
}

void HAL_I3C_CtrlDAACpltCallback(I3C_HandleTypeDef *hi3c) {
    g_platform_evt |= PLATFORM_I3C_DAA_EVT;
}

void HAL_I3C_NotifyCallback(I3C_HandleTypeDef *hi3c, uint32_t eventId) {
    if ((eventId & EVENT_ID_IBI) == EVENT_ID_IBI) {
        I3C_CCCInfoTypeDef CCCInfo;
        if (HAL_I3C_GetCCCInfo(hi3c, EVENT_ID_IBI, &CCCInfo) != HAL_OK) {
            return; // error in handling event
        }
        if (CCCInfo.IBITgtPayload == 0) {
            g_platform_evt |= PLATFORM_I3C_IBI_EVT;
        }
    }
}

void HAL_I3C_CtrlRxCpltCallback(I3C_HandleTypeDef *hi3c) {
    g_platform_evt |= PLATFORM_I3C_DMA_RX_EVT;
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
    for (int i = 0; i < NB_DEVICES; i++) {
        if (GPIO_Pin == device[i].intr.pin) {
            g_platform_evt |= PLATFORM_GPIO_IT_EVT;
        }
    }
}

void HAL_DCMIPP_PIPE_FrameEventCallback(DCMIPP_HandleTypeDef *hdcmipp, uint32_t Pipe) {
    if (Pipe == DCMIPP_PIPE0) {
        g_platform_evt |= PLATFORM_CAM_PIPE_FRAME_EVT;
    }
}

/* csi interface */

int platform_start_csi_pipe(uint8_t *buff_csi) {
    if (HAL_OK != HAL_DCMIPP_CSI_PIPE_Start(&hdcmipp, DCMIPP_PIPE0, DCMIPP_VIRTUAL_CHANNEL0, (uint32_t)buff_csi,
                                            DCMIPP_MODE_CONTINUOUS)) {
        return -1;
    }
    return 0;
}

int platform_stop_csi_pipe(void) {
    if (HAL_OK != HAL_DCMIPP_CSI_PIPE_Stop(&hdcmipp, DCMIPP_PIPE0, DCMIPP_VIRTUAL_CHANNEL0)) {
        return -1;
    }
    return 0;
}

#include "depth_color_scale.h"
#include "main.h"

/* display */
int platform_display_enable() {
    HAL_GPIO_WritePin(LCD_DE_GPIO_Port, LCD_DE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_ONOFF_GPIO_Port, LCD_ONOFF_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_SET);
    return 0;
}

int platform_display_disable() {
    HAL_GPIO_WritePin(LCD_DE_GPIO_Port, LCD_DE_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_ONOFF_GPIO_Port, LCD_ONOFF_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);
    return 0;
}

int platform_display_set_color_lut(uint8_t layer_id) {

    HAL_StatusTypeDef ret;

    if (layer_id == LTDC_LAYER_1) {
        ret = HAL_LTDC_ConfigCLUT(&hltdc, (uint32_t *)depth_to_rgb, 256, layer_id);
        if (ret != HAL_OK) {
            return -1;
        }
        ret = HAL_LTDC_EnableCLUT(&hltdc, layer_id);
        if (ret != HAL_OK) {
            return -1;
        }
    }

    return 0;
}

int platform_display_configure_layer(platform_display_layer_config_t *config, uint8_t layer_id) {

    HAL_StatusTypeDef ret;
    LTDC_LayerCfgTypeDef pLayerCfg = { 0 };

    pLayerCfg.WindowX0 = config->x_margin;
    pLayerCfg.WindowX1 = config->x_size * config->scaler + config->x_margin;
    pLayerCfg.WindowY0 = config->y_margin;
    pLayerCfg.WindowY1 = config->y_size * config->scaler + config->y_margin;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_L8;
    pLayerCfg.Alpha = 255;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    pLayerCfg.FBStartAdress = (uint32_t)config->frame_buffer;
    pLayerCfg.ImageWidth = config->x_size * config->scaler;
    pLayerCfg.ImageHeight = config->y_size * config->scaler;
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;

    ret = HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, layer_id);
    if (ret != HAL_OK) {
        return -1;
    }

    return 0;
}
