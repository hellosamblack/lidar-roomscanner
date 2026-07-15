/**
 ******************************************************************************
 * @file    vl53l9_platform.c
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
#include "vl53l9_platform.h"
#include "stm32h5xx_hal.h"
#include "vl53l9.h"
#include "vl53l9_interface.h"
#include <stdio.h>
#include <string.h>

/* Chunk size used by vl53l9_write() to avoid VLAs/heap.
 * Each I3C private write message is: 2-byte register address + up to (CHUNK_SIZE - 2) payload bytes.
 * Tune this based on stack constraints vs. throughput.
 */
#ifndef VL53L9_PLATFORM_I3C_WRITE_CHUNK_SIZE
#define VL53L9_PLATFORM_I3C_WRITE_CHUNK_SIZE (64U)
#endif

#if (VL53L9_PLATFORM_I3C_WRITE_CHUNK_SIZE < 3U)
#error "VL53L9_PLATFORM_I3C_WRITE_CHUNK_SIZE must be >= 3 (2-byte address + >=1 payload byte)"
#endif

// private function prototypes
static int _i3c_read(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor, I3C_XferTypeDef *aContextBuffers);
static int _i3c_read_async(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor, I3C_XferTypeDef *aContextBuffers);
static int _i3c_write(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor, I3C_XferTypeDef *aContextBuffers);

int vl53l9_read(void *const p_dev, uint16_t address, uint8_t *p_values, uint32_t size) {

    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[2];
    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;

    uint32_t cb[2];
    uint32_t sb[2];
    I3C_PrivateTypeDef pd[2] = { { p_device->address, { data_write, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
                                 { p_device->address, { NULL, 0 }, { p_values, size }, HAL_I3C_DIRECTION_READ } };
    I3C_XferTypeDef ctxtb[2] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 2 }, { NULL, 0 } },
                                 { { &cb[1], 1 }, { &sb[1], 1 }, { NULL, 0 }, { p_values, size } } };

    return _i3c_read(p_device, pd, ctxtb);
}

int vl53l9_read_async(void *const p_dev, uint16_t address, volatile uint8_t *p_values, uint32_t size) {

    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[2];
    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;

    uint32_t cb[2];
    uint32_t sb[2];
    I3C_PrivateTypeDef pd[2] = {
        { p_device->address, { data_write, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
        { p_device->address, { NULL, 0 }, { (uint8_t *)p_values, size }, HAL_I3C_DIRECTION_READ }
    };
    I3C_XferTypeDef ctxtb[2] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 2 }, { NULL, 0 } },
                                 { { &cb[1], 1 }, { &sb[1], 1 }, { NULL, 0 }, { (uint8_t *)p_values, size } } };

    return _i3c_read_async(p_device, pd, ctxtb);
}

int vl53l9_read8(void *const p_dev, uint16_t address, uint8_t *p_value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[2];
    uint8_t data_read[1];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;
    uint32_t cb[2];
    uint32_t sb[2];

    I3C_PrivateTypeDef pd[2] = { { p_device->address, { data_write, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
                                 { p_device->address, { NULL, 0 }, { data_read, 1 }, HAL_I3C_DIRECTION_READ } };
    I3C_XferTypeDef ctxtb[2] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 2 }, { NULL, 0 } },
                                 { { &cb[1], 1 }, { &sb[1], 1 }, { NULL, 0 }, { data_read, 1 } } };

    ret = _i3c_read(p_device, pd, ctxtb);
    // TODO: return in case of error

    *p_value = data_read[0];

    return ret;
}

int vl53l9_read16(void *const p_dev, uint16_t address, uint16_t *p_value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[2];
    uint8_t data_read[2];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;

    uint32_t cb[2];
    uint32_t sb[2];

    I3C_PrivateTypeDef pd[2] = { { p_device->address, { data_write, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
                                 { p_device->address, { NULL, 0 }, { data_read, 2 }, HAL_I3C_DIRECTION_READ } };
    I3C_XferTypeDef ctxtb[2] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 2 }, { NULL, 0 } },
                                 { { &cb[1], 1 }, { &sb[1], 1 }, { NULL, 0 }, { data_read, 2 } } };

    ret = _i3c_read(p_device, pd, ctxtb);
    // TODO: return here in case of error

    *p_value = ((data_read[0] << 0) | (data_read[1] << 8));

    return ret;
}

int vl53l9_read32(void *const p_dev, uint16_t address, uint32_t *p_value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[2];
    uint8_t data_read[4];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;

    uint32_t cb[2];
    uint32_t sb[2];

    I3C_PrivateTypeDef pd[2] = { { p_device->address, { data_write, 2 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
                                 { p_device->address, { NULL, 0 }, { data_read, 4 }, HAL_I3C_DIRECTION_READ } };
    I3C_XferTypeDef ctxtb[2] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 2 }, { NULL, 0 } },
                                 { { &cb[1], 1 }, { &sb[1], 1 }, { NULL, 0 }, { data_read, 4 } } };
    ret = _i3c_read(p_device, pd, ctxtb);

    *p_value = ((data_read[0] << 0) | (data_read[1] << 8) | (data_read[2] << 16) | (data_read[3] << 24));

    return ret;
}

int vl53l9_write(void *const p_dev, uint16_t address, uint8_t *p_values, uint32_t size) {

    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    // NOTE: each chunk is sent as a separate private write message

    uint8_t data_write[VL53L9_PLATFORM_I3C_WRITE_CHUNK_SIZE];
    const uint32_t max_payload_per_chunk = (uint32_t)(sizeof(data_write) - 2U);
    uint32_t offset = 0U;

    /* validate input */
    if ((p_dev == NULL) || (p_values == NULL) || (size == 0U)) {
        return VL53L9_ERROR_PLATFORM;
    }

    while (offset < size) {
        uint32_t cb[1];
        uint32_t sb[1];
        I3C_PrivateTypeDef pd[1];
        I3C_XferTypeDef ctxtb[1];

        const uint32_t remaining = size - offset;
        const uint32_t payload_this_chunk = (remaining < max_payload_per_chunk) ? remaining : max_payload_per_chunk;
        const uint32_t addr32 = (uint32_t)address + offset;
        const uint16_t addr_this_chunk = (uint16_t)addr32;
        const uint32_t total_this_chunk = 2U + payload_this_chunk;

        if (addr32 > 0xFFFFU) {
            return VL53L9_ERROR_PLATFORM;
        }
        if (payload_this_chunk == 0U) {
            return VL53L9_ERROR_PLATFORM;
        }

        data_write[0] = (uint8_t)((addr_this_chunk >> 8) & 0xFFU);
        data_write[1] = (uint8_t)(addr_this_chunk & 0xFFU);
        memcpy(&data_write[2], &p_values[offset], payload_this_chunk);

        pd[0].TargetAddr = p_device->address;
        pd[0].TxBuf.pBuffer = data_write;
        pd[0].TxBuf.Size = total_this_chunk;
        pd[0].RxBuf.pBuffer = NULL;
        pd[0].RxBuf.Size = 0U;
        pd[0].Direction = HAL_I3C_DIRECTION_WRITE;

        ctxtb[0].CtrlBuf.pBuffer = &cb[0];
        ctxtb[0].CtrlBuf.Size = 1U;
        ctxtb[0].StatusBuf.pBuffer = &sb[0];
        ctxtb[0].StatusBuf.Size = 1U;
        ctxtb[0].TxBuf.pBuffer = data_write;
        ctxtb[0].TxBuf.Size = total_this_chunk;
        ctxtb[0].RxBuf.pBuffer = NULL;
        ctxtb[0].RxBuf.Size = 0U;

        if (_i3c_write(p_device, pd, ctxtb) != VL53L9_ERROR_NONE) {
            return VL53L9_ERROR_PLATFORM;
        }

        offset += payload_this_chunk;
    }

    return VL53L9_ERROR_NONE;
}

int vl53l9_write8(void *const p_dev, uint16_t address, uint8_t value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[3];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;
    data_write[2] = value & 0xFF;

    uint32_t cb[1];
    uint32_t sb[1];
    I3C_PrivateTypeDef pd[1] = {
        { p_device->address, { data_write, 3 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
    };
    I3C_XferTypeDef ctxtb[1] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 3 }, { NULL, 0 } } };
    ret = _i3c_write(p_device, pd, ctxtb);

    return ret;
}

int vl53l9_write32(void *const p_dev, uint16_t address, uint32_t value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[6];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;
    data_write[2] = (value >> 0) & 0xFF;
    data_write[3] = (value >> 8) & 0xFF;
    data_write[4] = (value >> 16) & 0xFF;
    data_write[5] = (value >> 24) & 0xFF;

    uint32_t cb[1];
    uint32_t sb[1];
    I3C_PrivateTypeDef pd[1] = {
        { p_device->address, { data_write, 6 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
    };
    I3C_XferTypeDef ctxtb[1] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 6 }, { NULL, 0 } } };
    ret = _i3c_write(p_device, pd, ctxtb);

    return ret;
}

int vl53l9_write16(void *const p_dev, uint16_t address, uint16_t value) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    uint8_t data_write[4];

    data_write[0] = (address >> 8) & 0xFF;
    data_write[1] = address & 0xFF;
    data_write[2] = (value >> 0) & 0xFF;
    data_write[3] = (value >> 8) & 0xFF;

    uint32_t cb[1];
    uint32_t sb[1];
    I3C_PrivateTypeDef pd[1] = {
        { p_device->address, { data_write, 4 }, { NULL, 0 }, HAL_I3C_DIRECTION_WRITE },
    };
    I3C_XferTypeDef ctxtb[1] = { { { &cb[0], 1 }, { &sb[0], 1 }, { data_write, 4 }, { NULL, 0 } } };
    ret = _i3c_write(p_device, pd, ctxtb);

    return ret;
}

int vl53l9_wait_ms(void *const p_dev, uint32_t delay_ms) {
    (void)p_dev;
    HAL_Delay(delay_ms);
    return VL53L9_ERROR_NONE;
}

int vl53l9_get_config_vddio(void *const p_dev, vl53l9_vddio_t *voltage) {
    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;
    *voltage = p_device->vddio;
    return ret;
}
int vl53l9_get_config_vdda(void *const p_dev, vl53l9_vdda_t *voltage) {
    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;
    *voltage = p_device->vdda;
    return ret;
}

int vl53l9_get_config_ext_clock(void *const p_dev, uint32_t *ext_clock) {
    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;
    *ext_clock = p_device->ext_clock;
    return ret;
}

static int _i3c_read(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor, I3C_XferTypeDef *aContextBuffers) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;
    I3C_HandleTypeDef *p_hi3c = (I3C_HandleTypeDef *)p_device->bus;
    uint32_t option;

    if (p_device->bus_property & PLATFORM_BUS_PROPERTY_I3C_LEGACY) {
        aPrivateDescriptor[0].TargetAddr = aPrivateDescriptor[0].TargetAddr >> 1;
        aPrivateDescriptor[1].TargetAddr = aPrivateDescriptor[1].TargetAddr >> 1;
        option = I2C_PRIVATE_WITHOUT_ARB_STOP;
    } else {
        option = I3C_PRIVATE_WITHOUT_ARB_RESTART;
    }

    if (HAL_I3C_AddDescToFrame(p_hi3c, NULL, &aPrivateDescriptor[0], &aContextBuffers[0],
                               aContextBuffers[0].CtrlBuf.Size, option) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    if (HAL_I3C_Ctrl_Transmit(p_hi3c, &aContextBuffers[0], 100) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }

    while ((HAL_I3C_GetState(p_hi3c) != HAL_I3C_STATE_READY) && (HAL_I3C_GetState(p_hi3c) != HAL_I3C_STATE_LISTEN)) {
    }

    if (p_device->bus_property & PLATFORM_BUS_PROPERTY_I3C_LEGACY) {
        option = I2C_PRIVATE_WITHOUT_ARB_STOP;
    } else {
        option = I3C_PRIVATE_WITHOUT_ARB_STOP;
    }

    if (HAL_I3C_AddDescToFrame(p_hi3c, NULL, &aPrivateDescriptor[1], &aContextBuffers[1],
                               aContextBuffers[1].CtrlBuf.Size, option) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    if ((ret = HAL_I3C_Ctrl_Receive(p_hi3c, &aContextBuffers[1], 100)) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    return ret;
}

static int _i3c_read_async(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor,
                           I3C_XferTypeDef *aContextBuffers) {
    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;
    I3C_HandleTypeDef *p_hi3c = (I3C_HandleTypeDef *)p_device->bus;
    uint32_t option;

    if (p_device->bus_property & PLATFORM_BUS_PROPERTY_I3C_LEGACY) {
        aPrivateDescriptor[0].TargetAddr = aPrivateDescriptor[0].TargetAddr >> 1;
        aPrivateDescriptor[1].TargetAddr = aPrivateDescriptor[1].TargetAddr >> 1;
        option = I2C_PRIVATE_WITHOUT_ARB_STOP;
    } else {
        option = I3C_PRIVATE_WITHOUT_ARB_RESTART;
    }

    if (HAL_I3C_AddDescToFrame(p_hi3c, NULL, &aPrivateDescriptor[0], &aContextBuffers[0],
                               aContextBuffers[0].CtrlBuf.Size, option) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    if (HAL_I3C_Ctrl_Transmit(p_hi3c, &aContextBuffers[0], 100) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }

    // NOTE: when ibi are enabled the bus state is set to listening
    while ((HAL_I3C_GetState(p_hi3c) != HAL_I3C_STATE_READY) && (HAL_I3C_GetState(p_hi3c) != HAL_I3C_STATE_LISTEN)) {
    }

    if (p_device->bus_property & PLATFORM_BUS_PROPERTY_I3C_LEGACY) {
        option = I2C_PRIVATE_WITH_ARB_RESTART;
    } else {
        option = I3C_PRIVATE_WITHOUT_ARB_STOP;
    }

    if (HAL_I3C_AddDescToFrame(p_hi3c, NULL, &aPrivateDescriptor[1], &aContextBuffers[1],
                               aContextBuffers[1].CtrlBuf.Size, option) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    ret = HAL_I3C_Ctrl_Receive_DMA(p_hi3c, &aContextBuffers[1]);
    if (ret != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    return ret;
}

static int _i3c_write(void *const p_dev, I3C_PrivateTypeDef *aPrivateDescriptor, I3C_XferTypeDef *aContextBuffers) {

    int ret = VL53L9_ERROR_NONE;
    vl53l9_device_t *p_device = (vl53l9_device_t *)p_dev;

    I3C_HandleTypeDef *p_hi3c = (I3C_HandleTypeDef *)p_device->bus;
    uint32_t option;
    if (p_device->bus_property & PLATFORM_BUS_PROPERTY_I3C_LEGACY) {
        aPrivateDescriptor[0].TargetAddr = aPrivateDescriptor[0].TargetAddr >> 1;
        option = I2C_PRIVATE_WITHOUT_ARB_STOP;
    } else {
        option = I3C_PRIVATE_WITHOUT_ARB_RESTART;
    }

    if (HAL_I3C_AddDescToFrame(p_hi3c, NULL, &aPrivateDescriptor[0], &aContextBuffers[0],
                               aContextBuffers[0].CtrlBuf.Size, option) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }
    if (HAL_I3C_Ctrl_Transmit(p_hi3c, &aContextBuffers[0], 100) != HAL_OK) {
        return VL53L9_ERROR_PLATFORM;
    }

    return ret;
}
