/**
  ******************************************************************************
  * @file    LSM6DSO16ISSensor.cpp
  * @author  STMicroelectornics
  * @brief   LSM6DSO16IS driver file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "LSM6DSO16ISSensor.h"
/* Class Implementation ------------------------------------------------------*/
/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
LSM6DSO16ISSensor::LSM6DSO16ISSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  dev_spi = NULL;
  reg_ctx.write_reg = LSM6DSO16IS_io_write;
  reg_ctx.read_reg = LSM6DSO16IS_io_read;
  reg_ctx.handle = (void *)this;
  isInitialized = 0;
  X_isEnabled = 0;
  G_isEnabled = 0;

}

/** Constructor
 * @param spi object of an helper class which handles the SPI peripheral
 * @param cs_pin the chip select pin
 * @param spi_speed the SPI speed
 */
LSM6DSO16ISSensor::LSM6DSO16ISSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed) : dev_spi(spi), cs_pin(cs_pin), spi_speed(spi_speed)
{
  reg_ctx.write_reg = LSM6DSO16IS_io_write;
  reg_ctx.read_reg = LSM6DSO16IS_io_read;
  reg_ctx.handle = (void *)this;
  dev_i2c = NULL;
  address = 0;
  isInitialized = 0;
  X_isEnabled = 0;
  G_isEnabled = 0;
}


/**
  * @brief  Initialize the LSM6DSO16IS sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::begin(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  if (dev_spi) {
    // Configure CS pin
    pinMode(cs_pin, OUTPUT);
    digitalWrite(cs_pin, HIGH);
  }
  /* Enable register address automatically incremented during a multiple byte
  access with a serial interface. */
  if (lsm6dso16is_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Enable BDU */
  if (lsm6dso16is_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Select default output data rate. */
  X_Last_ODR = 104;

  /* Output data rate selection - power down. */
  if (lsm6dso16is_xl_data_rate_set(&reg_ctx, LSM6DSO16IS_XL_ODR_OFF) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dso16is_xl_full_scale_set(&reg_ctx, LSM6DSO16IS_2g) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Select default output data rate. */
  G_Last_ODR = 104;

  /* Output data rate selection - power down. */
  if (lsm6dso16is_gy_data_rate_set(&reg_ctx, LSM6DSO16IS_GY_ODR_OFF) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Full scale selection. */
  if (lsm6dso16is_gy_full_scale_set(&reg_ctx, LSM6DSO16IS_2000dps) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (ret == LSM6DSO16IS_STATUS_OK) {
    isInitialized = 1;
  }

  return ret;
}

/**
  * @brief  Deinitialize the LSM6DSO16IS sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::end(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  /* Disable the component */
  if (Disable_X() != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (Disable_G() != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (ret == LSM6DSO16IS_STATUS_OK) {
    /* Reset output data rate. */
    X_Last_ODR = LSM6DSO16IS_XL_ODR_OFF;
    G_Last_ODR = LSM6DSO16IS_GY_ODR_OFF;

    isInitialized = 0;
  }

  return ret;
}

/**
  * @brief  Read component ID
  * @param  Id the WHO_AM_I value
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::ReadID(uint8_t *Id)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  if (lsm6dso16is_device_id_get(&reg_ctx, Id) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Enable the LSM6DSO16IS accelerometer sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Enable_X(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_data_rate_t new_odr;

  /* Check if the component is already enabled */
  if (X_isEnabled == 1U) {
    ret = LSM6DSO16IS_STATUS_OK;
  } else {
    new_odr = (X_Last_ODR <=   12.5f) ? LSM6DSO16IS_XL_ODR_AT_12Hz5_HP
              : (X_Last_ODR <=   26.0f) ? LSM6DSO16IS_XL_ODR_AT_26H_HP
              : (X_Last_ODR <=   52.0f) ? LSM6DSO16IS_XL_ODR_AT_52Hz_HP
              : (X_Last_ODR <=  104.0f) ? LSM6DSO16IS_XL_ODR_AT_104Hz_HP
              : (X_Last_ODR <=  208.0f) ? LSM6DSO16IS_XL_ODR_AT_208Hz_HP
              : (X_Last_ODR <=  416.0f) ? LSM6DSO16IS_XL_ODR_AT_416Hz_HP
              : (X_Last_ODR <=  833.0f) ? LSM6DSO16IS_XL_ODR_AT_833Hz_HP
              : (X_Last_ODR <= 1667.0f) ? LSM6DSO16IS_XL_ODR_AT_1667Hz_HP
              : (X_Last_ODR <= 3333.0f) ? LSM6DSO16IS_XL_ODR_AT_3333Hz_HP
              :                    LSM6DSO16IS_XL_ODR_AT_6667Hz_HP;

    /* Output data rate selection. */
    if (lsm6dso16is_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }

    X_isEnabled = 1;
  }

  return ret;
}

/**
  * @brief  Disable the LSM6DSO16IS accelerometer sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Disable_X(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  /* Check if the component is already disabled */
  if (X_isEnabled == 0U) {
    ret = LSM6DSO16IS_STATUS_OK;
  } else {
    /* Get current output data rate. */
    if (Get_X_ODR(&X_Last_ODR) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }
    /* Output data rate selection - power down. */
    if (lsm6dso16is_xl_data_rate_set(&reg_ctx, LSM6DSO16IS_XL_ODR_OFF) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }

    X_isEnabled = 0;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS accelerometer sensor sensitivity
  * @param  Sensitivity pointer
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_Sensitivity(float_t *Sensitivity)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso16is_xl_full_scale_get(&reg_ctx, &full_scale) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Store the Sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSO16IS_2g:
      *Sensitivity = LSM6DSO16IS_ACC_SENSITIVITY_FS_2G;
      break;

    case LSM6DSO16IS_4g:
      *Sensitivity = LSM6DSO16IS_ACC_SENSITIVITY_FS_4G;
      break;

    case LSM6DSO16IS_8g:
      *Sensitivity = LSM6DSO16IS_ACC_SENSITIVITY_FS_8G;
      break;

    case LSM6DSO16IS_16g:
      *Sensitivity = LSM6DSO16IS_ACC_SENSITIVITY_FS_16G;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS accelerometer sensor output data rate
  * @param  Odr pointer where the output data rate is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_ODR(float_t *Odr)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dso16is_xl_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSO16IS_XL_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_12Hz5_HP:
      *Odr = 12.5f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_26H_HP:
      *Odr = 26.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_52Hz_HP:
      *Odr = 52.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_104Hz_HP:
      *Odr = 104.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_208Hz_HP:
      *Odr = 208.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_416Hz_HP:
      *Odr = 416.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_833Hz_HP:
      *Odr = 833.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_1667Hz_HP:
      *Odr = 1667.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_3333Hz_HP:
      *Odr = 3333.0f;
      break;

    case LSM6DSO16IS_XL_ODR_AT_6667Hz_HP:
      *Odr = 6667.0f;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS accelerometer sensor output data rate
  * @param  Odr the output data rate value to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_ODR(float_t Odr)
{
  LSM6DSO16ISStatusTypeDef ret;

  /* Check if the component is enabled */
  if (X_isEnabled == 1U) {
    ret = Set_X_ODR_When_Enabled(Odr);
  } else {
    ret = Set_X_ODR_When_Disabled(Odr);
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS accelerometer sensor full scale
  * @param  FullScale pointer where the full scale is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_FS(int32_t *FullScale)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso16is_xl_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSO16IS_2g:
      *FullScale =  2;
      break;

    case LSM6DSO16IS_4g:
      *FullScale =  4;
      break;

    case LSM6DSO16IS_8g:
      *FullScale =  8;
      break;

    case LSM6DSO16IS_16g:
      *FullScale = 16;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS accelerometer sensor full scale
  * @param  FullScale the functional full scale to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_FS(int32_t FullScale)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_full_scale_t new_fs;

  new_fs = (FullScale <= 2) ? LSM6DSO16IS_2g
           : (FullScale <= 4) ? LSM6DSO16IS_4g
           : (FullScale <= 8) ? LSM6DSO16IS_8g
           :                    LSM6DSO16IS_16g;

  if (lsm6dso16is_xl_full_scale_set(&reg_ctx, new_fs) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS accelerometer sensor raw axes
  * @param  Value pointer where the raw values of the axes are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_AxesRaw(int32_t *Value)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  int16_t data_raw[3];

  /* Read raw data values. */
  if (lsm6dso16is_acceleration_raw_get(&reg_ctx, data_raw) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Format the data. */
  Value[0] = (int32_t) data_raw[0];
  Value[1] = (int32_t) data_raw[1];
  Value[2] = (int32_t) data_raw[2];

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS accelerometer sensor axes
  * @param  Acceleration pointer where the values of the axes are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_Axes(int32_t *Acceleration)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  int16_t data_raw[3];
  float_t sensitivity = 0.0f;

  /* Read raw data values. */
  if (lsm6dso16is_acceleration_raw_get(&reg_ctx, data_raw) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Get LSM6DSO16IS actual sensitivity. */
  if (Get_X_Sensitivity(&sensitivity) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }


  /* Calculate the data. */
  Acceleration[0] = (int32_t)((float_t)((float_t)data_raw[0] * sensitivity));
  Acceleration[1] = (int32_t)((float_t)((float_t)data_raw[1] * sensitivity));
  Acceleration[2] = (int32_t)((float_t)((float_t)data_raw[2] * sensitivity));

  return ret;
}

/**
  * @brief  Enable the LSM6DSO16IS gyroscope sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Enable_G(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_data_rate_t new_odr;

  /* Check if the component is already enabled */
  if (G_isEnabled == 1U) {
    ret = LSM6DSO16IS_STATUS_OK;
  } else {
    new_odr = (G_Last_ODR <=   12.5f) ? LSM6DSO16IS_GY_ODR_AT_12Hz5_HP
              : (G_Last_ODR <=   26.0f) ? LSM6DSO16IS_GY_ODR_AT_26H_HP
              : (G_Last_ODR <=   52.0f) ? LSM6DSO16IS_GY_ODR_AT_52Hz_HP
              : (G_Last_ODR <=  104.0f) ? LSM6DSO16IS_GY_ODR_AT_104Hz_HP
              : (G_Last_ODR <=  208.0f) ? LSM6DSO16IS_GY_ODR_AT_208Hz_HP
              : (G_Last_ODR <=  416.0f) ? LSM6DSO16IS_GY_ODR_AT_416Hz_HP
              : (G_Last_ODR <=  833.0f) ? LSM6DSO16IS_GY_ODR_AT_833Hz_HP
              : (G_Last_ODR <= 1667.0f) ? LSM6DSO16IS_GY_ODR_AT_1667Hz_HP
              : (G_Last_ODR <= 3333.0f) ? LSM6DSO16IS_GY_ODR_AT_3333Hz_HP
              :                    LSM6DSO16IS_GY_ODR_AT_6667Hz_HP;
    /* Output data rate selection. */
    if (lsm6dso16is_gy_data_rate_set(&reg_ctx, new_odr) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }

    G_isEnabled = 1;
  }

  return ret;
}

/**
  * @brief  Disable the LSM6DSO16IS gyroscope sensor
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Disable_G(void)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  /* Check if the component is already disabled */
  if (G_isEnabled == 0U) {
    ret = LSM6DSO16IS_STATUS_OK;
  } else {
    /* Get current output data rate. */
    if (Get_G_ODR(&G_Last_ODR) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }
    /* Output data rate selection - power down. */
    if (lsm6dso16is_gy_data_rate_set(&reg_ctx, LSM6DSO16IS_GY_ODR_OFF) != LSM6DSO16IS_STATUS_OK) {
      ret = LSM6DSO16IS_STATUS_ERROR;
    }

    G_isEnabled = 0;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS gyroscope sensor sensitivity
  * @param  Sensitivity pointer
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_Sensitivity(float_t *Sensitivity)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_full_scale_t full_scale;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso16is_gy_full_scale_get(&reg_ctx, &full_scale) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Store the sensitivity based on actual full scale. */
  switch (full_scale) {
    case LSM6DSO16IS_125dps:
      *Sensitivity = LSM6DSO16IS_GYRO_SENSITIVITY_FS_125DPS;
      break;

    case LSM6DSO16IS_250dps:
      *Sensitivity = LSM6DSO16IS_GYRO_SENSITIVITY_FS_250DPS;
      break;

    case LSM6DSO16IS_500dps:
      *Sensitivity = LSM6DSO16IS_GYRO_SENSITIVITY_FS_500DPS;
      break;

    case LSM6DSO16IS_1000dps:
      *Sensitivity = LSM6DSO16IS_GYRO_SENSITIVITY_FS_1000DPS;
      break;

    case LSM6DSO16IS_2000dps:
      *Sensitivity = LSM6DSO16IS_GYRO_SENSITIVITY_FS_2000DPS;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS gyroscope sensor output data rate
  * @param  Odr pointer where the output data rate is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_ODR(float_t *Odr)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_data_rate_t odr_low_level;

  /* Get current output data rate. */
  if (lsm6dso16is_gy_data_rate_get(&reg_ctx, &odr_low_level) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  switch (odr_low_level) {
    case LSM6DSO16IS_GY_ODR_OFF:
      *Odr = 0.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_12Hz5_HP:
      *Odr = 12.5f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_26H_HP:
      *Odr = 26.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_52Hz_HP:
      *Odr = 52.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_104Hz_HP:
      *Odr = 104.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_208Hz_HP:
      *Odr = 208.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_416Hz_HP:
      *Odr = 416.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_833Hz_HP:
      *Odr = 833.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_1667Hz_HP:
      *Odr =  1667.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_3333Hz_HP:
      *Odr =  3333.0f;
      break;

    case LSM6DSO16IS_GY_ODR_AT_6667Hz_HP:
      *Odr =  6667.0f;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS gyroscope sensor output data rate
  * @param  Odr the output data rate value to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_ODR(float_t Odr)
{
  LSM6DSO16ISStatusTypeDef ret;

  /* Check if the component is enabled */
  if (G_isEnabled == 1U) {
    ret = Set_G_ODR_When_Enabled(Odr);
  } else {
    ret = Set_G_ODR_When_Disabled(Odr);
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS gyroscope sensor full scale
  * @param  FullScale pointer where the full scale is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_FS(int32_t  *FullScale)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_full_scale_t fs_low_level;

  /* Read actual full scale selection from sensor. */
  if (lsm6dso16is_gy_full_scale_get(&reg_ctx, &fs_low_level) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  switch (fs_low_level) {
    case LSM6DSO16IS_125dps:
      *FullScale =  125;
      break;

    case LSM6DSO16IS_250dps:
      *FullScale =  250;
      break;

    case LSM6DSO16IS_500dps:
      *FullScale =  500;
      break;

    case LSM6DSO16IS_1000dps:
      *FullScale = 1000;
      break;

    case LSM6DSO16IS_2000dps:
      *FullScale = 2000;
      break;

    default:
      ret = LSM6DSO16IS_STATUS_ERROR;
      break;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS gyroscope sensor full scale
  * @param  FullScale the functional full scale to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_FS(int32_t FullScale)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_full_scale_t new_fs;

  new_fs = (FullScale <= 125)  ? LSM6DSO16IS_125dps
           : (FullScale <= 250)  ? LSM6DSO16IS_250dps
           : (FullScale <= 500)  ? LSM6DSO16IS_500dps
           : (FullScale <= 1000) ? LSM6DSO16IS_1000dps
           :                       LSM6DSO16IS_2000dps;

  if (lsm6dso16is_gy_full_scale_set(&reg_ctx, new_fs) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS gyroscope sensor raw axes
  * @param  Value pointer where the raw values of the axes are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_AxesRaw(int32_t *Value)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  int16_t data_raw[3];

  /* Read raw data values. */
  if (lsm6dso16is_angular_rate_raw_get(&reg_ctx, data_raw) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Format the data. */
  Value[0] = (int32_t) data_raw[0];
  Value[1] = (int32_t) data_raw[1];
  Value[2] = (int32_t) data_raw[2];

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS gyroscope sensor axes
  * @param  AngularRate pointer where the values of the axes are written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_Axes(int32_t *AngularRate)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  int16_t data_raw[3];
  float_t sensitivity;

  /* Read raw data values. */
  if (lsm6dso16is_angular_rate_raw_get(&reg_ctx, data_raw) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Get LSM6DSO16IS actual sensitivity. */
  if (Get_G_Sensitivity(&sensitivity) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  /* Calculate the data. */
  AngularRate[0] = (int32_t)((float_t)((float_t)data_raw[0] * sensitivity));
  AngularRate[1] = (int32_t)((float_t)((float_t)data_raw[1] * sensitivity));
  AngularRate[2] = (int32_t)((float_t)((float_t)data_raw[2] * sensitivity));

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS register value
  * @param  Reg address to be read
  * @param  Data pointer where the value is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  if (lsm6dso16is_read_reg(&reg_ctx, Reg, Data, 1) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
 * @brief  Get the status of all ISPU events
 * @param  Status the status of all ISPU events
 * @retval 0 in case of success, an error code otherwise
 */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_ISPU_Status(LSM6DSO16IS_ISPU_Status_t *Status)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  union {
    uint32_t val;
    LSM6DSO16IS_ISPU_Status_t status;
  } ispu_status;

  if (lsm6dso16is_ia_ispu_get(&reg_ctx, &ispu_status.val) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  *Status = ispu_status.status;
  return ret;
}


/**
  * @brief  Get the LSM6DSO16IS ISPU Output
  * @param  Reg address where to start reading
  * @param  len number of registers to read
  * @param  Data pointer where the value is written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Read_ISPU_Output(uint8_t Reg, uint8_t *Data, uint8_t len)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  //Check that register to read is an ISPU Output register
  if (Reg < LSM6DSO16IS_ISPU_DOUT_00_L || Reg > LSM6DSO16IS_ISPU_DOUT_31_H) {
    return LSM6DSO16IS_STATUS_ERROR;
  }
  //Enable the access to the ISPU interaction registers
  if (lsm6dso16is_mem_bank_set(&reg_ctx, LSM6DSO16IS_ISPU_MEM_BANK) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }
  if (lsm6dso16is_read_reg(&reg_ctx, Reg, Data, len) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }
  //Disable the access to the ISPU interaction registers
  if (lsm6dso16is_mem_bank_set(&reg_ctx, LSM6DSO16IS_MAIN_MEM_BANK) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }
  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS register value
  * @param  Reg address to be written
  * @param  Data value to be written
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  if (lsm6dso16is_write_reg(&reg_ctx, Reg, &Data, 1) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Set self test
  * @param  Val the value of st_xl in reg CTRL5_C
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_SelfTest(uint8_t Val)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_self_test_t reg;

  reg = (Val == 1U)  ? LSM6DSO16IS_XL_ST_POSITIVE
        : (Val == 2U)  ? LSM6DSO16IS_XL_ST_NEGATIVE
        :                LSM6DSO16IS_XL_ST_DISABLE;

  if (lsm6dso16is_xl_self_test_set(&reg_ctx, reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS ACC data ready bit value
  * @param  Status the status of data ready bit
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_DRDY_Status(uint8_t *Status)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  if (lsm6dso16is_xl_flag_data_ready_get(&reg_ctx, Status) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS ACC initialization status
  * @param  Status 1 if initialized, 0 otherwise
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_X_Init_Status(uint8_t *Status)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  *Status = isInitialized;

  return ret;
}

/**
  * @brief  Set DRDY on INT1
  * @param  Val the value of int1_drdy_xl in reg INT1_CTRL
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_INT1_DRDY(uint8_t Val)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_pin_int1_route_t reg;

  if (lsm6dso16is_pin_int1_route_get(&reg_ctx, &reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (Val <= 1U) {
    reg.drdy_xl = Val;
  } else {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (lsm6dso16is_pin_int1_route_set(&reg_ctx, reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Set self test
  * @param  Val the value of st_xl in reg CTRL5_C
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_SelfTest(uint8_t Val)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_self_test_t reg;

  reg = (Val == 1U)  ? LSM6DSO16IS_GY_ST_POSITIVE
        : (Val == 2U)  ? LSM6DSO16IS_GY_ST_NEGATIVE
        :                LSM6DSO16IS_GY_ST_DISABLE;


  if (lsm6dso16is_gy_self_test_set(&reg_ctx, reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS GYRO data ready bit value
  * @param  Status the status of data ready bit
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_DRDY_Status(uint8_t *Status)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  if (lsm6dso16is_gy_flag_data_ready_get(&reg_ctx, Status) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Get the LSM6DSO16IS GYRO initialization status
  * @param  Status 1 if initialized, 0 otherwise
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Get_G_Init_Status(uint8_t *Status)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;

  *Status = isInitialized;

  return ret;
}

/**
  * @brief  Set DRDY on INT1
  * @param  Val the value of int1_drdy_g in reg INT1_CTRL
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_INT1_DRDY(uint8_t Val)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_pin_int1_route_t reg;

  if (lsm6dso16is_pin_int1_route_get(&reg_ctx, &reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (Val <= 1U) {
    reg.drdy_gy = Val;
  } else {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  if (lsm6dso16is_pin_int1_route_set(&reg_ctx, reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Set DRDY mode
  * @param  Val the value of drdy_pulsed in reg LSM6DSO16IS_DRDY_PULSE_CFG_G
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_DRDY_Mode(uint8_t Val)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_data_ready_mode_t reg;

  reg = (Val == 0U)  ? LSM6DSO16IS_DRDY_LATCHED
        :                LSM6DSO16IS_DRDY_PULSED;

  if (lsm6dso16is_data_ready_mode_set(&reg_ctx, reg) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}


/**
  * @brief  Set the LSM6DSO16IS accelerometer sensor output data rate when enabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_ODR_When_Enabled(float_t Odr)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_xl_data_rate_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSO16IS_XL_ODR_AT_12Hz5_HP
            : (Odr <=   26.0f) ? LSM6DSO16IS_XL_ODR_AT_26H_HP
            : (Odr <=   52.0f) ? LSM6DSO16IS_XL_ODR_AT_52Hz_HP
            : (Odr <=  104.0f) ? LSM6DSO16IS_XL_ODR_AT_104Hz_HP
            : (Odr <=  208.0f) ? LSM6DSO16IS_XL_ODR_AT_208Hz_HP
            : (Odr <=  416.0f) ? LSM6DSO16IS_XL_ODR_AT_416Hz_HP
            : (Odr <=  833.0f) ? LSM6DSO16IS_XL_ODR_AT_833Hz_HP
            : (Odr <= 1667.0f) ? LSM6DSO16IS_XL_ODR_AT_1667Hz_HP
            : (Odr <= 3333.0f) ? LSM6DSO16IS_XL_ODR_AT_3333Hz_HP
            :                    LSM6DSO16IS_XL_ODR_AT_6667Hz_HP;

  /* Output data rate selection. */
  if (lsm6dso16is_xl_data_rate_set(&reg_ctx, new_odr) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS accelerometer sensor output data rate when disabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_X_ODR_When_Disabled(float_t Odr)
{
  X_Last_ODR = (Odr <=   12.5f) ? LSM6DSO16IS_XL_ODR_AT_12Hz5_HP
               : (Odr <=   26.0f) ? LSM6DSO16IS_XL_ODR_AT_26H_HP
               : (Odr <=   52.0f) ? LSM6DSO16IS_XL_ODR_AT_52Hz_HP
               : (Odr <=  104.0f) ? LSM6DSO16IS_XL_ODR_AT_104Hz_HP
               : (Odr <=  208.0f) ? LSM6DSO16IS_XL_ODR_AT_208Hz_HP
               : (Odr <=  416.0f) ? LSM6DSO16IS_XL_ODR_AT_416Hz_HP
               : (Odr <=  833.0f) ? LSM6DSO16IS_XL_ODR_AT_833Hz_HP
               : (Odr <= 1667.0f) ? LSM6DSO16IS_XL_ODR_AT_1667Hz_HP
               : (Odr <= 3333.0f) ? LSM6DSO16IS_XL_ODR_AT_3333Hz_HP
               :                    LSM6DSO16IS_XL_ODR_AT_6667Hz_HP;

  return LSM6DSO16IS_STATUS_OK;
}

/**
  * @brief  Set the LSM6DSO16IS gyroscope sensor output data rate when enabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_ODR_When_Enabled(float_t Odr)
{
  LSM6DSO16ISStatusTypeDef ret = LSM6DSO16IS_STATUS_OK;
  lsm6dso16is_gy_data_rate_t new_odr;

  new_odr = (Odr <=   12.5f) ? LSM6DSO16IS_GY_ODR_AT_12Hz5_HP
            : (Odr <=   26.0f) ? LSM6DSO16IS_GY_ODR_AT_26H_HP
            : (Odr <=   52.0f) ? LSM6DSO16IS_GY_ODR_AT_52Hz_HP
            : (Odr <=  104.0f) ? LSM6DSO16IS_GY_ODR_AT_104Hz_HP
            : (Odr <=  208.0f) ? LSM6DSO16IS_GY_ODR_AT_208Hz_HP
            : (Odr <=  416.0f) ? LSM6DSO16IS_GY_ODR_AT_416Hz_HP
            : (Odr <=  833.0f) ? LSM6DSO16IS_GY_ODR_AT_833Hz_HP
            : (Odr <= 1667.0f) ? LSM6DSO16IS_GY_ODR_AT_1667Hz_HP
            : (Odr <= 3333.0f) ? LSM6DSO16IS_GY_ODR_AT_3333Hz_HP
            :                    LSM6DSO16IS_GY_ODR_AT_6667Hz_HP;

  /* Output data rate selection. */
  if (lsm6dso16is_gy_data_rate_set(&reg_ctx, new_odr) != LSM6DSO16IS_STATUS_OK) {
    ret = LSM6DSO16IS_STATUS_ERROR;
  }

  return ret;
}

/**
  * @brief  Set the LSM6DSO16IS gyroscope sensor output data rate when disabled
  * @param  Odr the functional output data rate to be set
  * @retval 0 in case of success, an error code otherwise
  */
LSM6DSO16ISStatusTypeDef LSM6DSO16ISSensor::Set_G_ODR_When_Disabled(float_t Odr)
{
  G_Last_ODR = (Odr <=   12.5f) ? LSM6DSO16IS_GY_ODR_AT_12Hz5_HP
               : (Odr <=   26.0f) ? LSM6DSO16IS_GY_ODR_AT_26H_HP
               : (Odr <=   52.0f) ? LSM6DSO16IS_GY_ODR_AT_52Hz_HP
               : (Odr <=  104.0f) ? LSM6DSO16IS_GY_ODR_AT_104Hz_HP
               : (Odr <=  208.0f) ? LSM6DSO16IS_GY_ODR_AT_208Hz_HP
               : (Odr <=  416.0f) ? LSM6DSO16IS_GY_ODR_AT_416Hz_HP
               : (Odr <=  833.0f) ? LSM6DSO16IS_GY_ODR_AT_833Hz_HP
               : (Odr <= 1667.0f) ? LSM6DSO16IS_GY_ODR_AT_1667Hz_HP
               : (Odr <= 3333.0f) ? LSM6DSO16IS_GY_ODR_AT_3333Hz_HP
               :                    LSM6DSO16IS_GY_ODR_AT_6667Hz_HP;

  return LSM6DSO16IS_STATUS_OK;
}



int32_t LSM6DSO16IS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((LSM6DSO16ISSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t LSM6DSO16IS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((LSM6DSO16ISSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
