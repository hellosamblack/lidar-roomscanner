/**
 ******************************************************************************
 * @file    STTS22HSensor.cpp
 * @author  SRA
 * @version V1.0.0
 * @date    January 2020
 * @brief   Implementation of a STTS22H temperature sensor. 
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */


/* Includes ------------------------------------------------------------------*/

#include "STTS22HSensor.h"


/* Class Implementation ------------------------------------------------------*/

/** Constructor
 * @param i2c object of an helper class which handles the I2C peripheral
 * @param address the address of the component's instance
 */
STTS22HSensor::STTS22HSensor(TwoWire *i2c, uint8_t address) : dev_i2c(i2c), address(address)
{
  reg_ctx.write_reg = STTS22H_io_write;
  reg_ctx.read_reg = STTS22H_io_read;
  reg_ctx.handle = (void *)this;
  temp_is_enabled = 0U;
}

/**
 * @brief  Configure the sensor in order to be used
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::begin()
{
  /* Set default ODR */
  temp_odr = 1.0f;

  /* Enable BDU */
  if(stts22h_block_data_update_set(&reg_ctx, PROPERTY_ENABLE) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  /* Enable Automatic Address Increment */
  if(stts22h_auto_increment_set(&reg_ctx, PROPERTY_ENABLE) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  /* Put the component in standby mode. */
  if (stts22h_temp_data_rate_set(&reg_ctx, STTS22H_POWER_DOWN) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }
  
  temp_is_enabled = 0U;

  return STTS22H_OK;
}

/**
 * @brief  Disable the sensor and relative resources
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::end()
{
  /* Disable temperature sensor */
  if (Disable() != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get WHO_AM_I value
 * @param  Id the WHO_AM_I value
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::ReadID(uint8_t *Id)
{
  uint8_t buf;

  if (stts22h_dev_id_get(&reg_ctx, &buf) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  *Id = buf;

  return STTS22H_OK;
}

/**
 * @brief  Enable the STTS22H temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Enable()
{
  /* Check if the component is already enabled */
  if (temp_is_enabled == 1U)
  {
    return STTS22H_OK;
  }

  /* Power on the component and set the odr. */
  if (SetOutputDataRate(temp_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  temp_is_enabled = 1U;

  return STTS22H_OK;
}

/**
 * @brief  Disable the STTS22H temperature sensor
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Disable()
{
  /* Check if the component is already disabled */
  if (temp_is_enabled == 0U)
  {
    return STTS22H_OK;
  }

  /* Save the current odr. */
  if (GetOutputDataRate(&temp_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }
  
  /* Put the component in standby mode. */
  if (stts22h_temp_data_rate_set(&reg_ctx, STTS22H_POWER_DOWN) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  temp_is_enabled = 0U;

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature sensor output data rate
 * @param  Odr pointer where the output data rate is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::GetOutputDataRate(float *Odr)
{
  STTS22HStatusTypeDef ret = STTS22H_OK;
  stts22h_odr_temp_t odr_low_level;

  if (stts22h_temp_data_rate_get(&reg_ctx, &odr_low_level) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  switch (odr_low_level)
  {
    case STTS22H_POWER_DOWN:
    case STTS22H_ONE_SHOT:
      *Odr = 0.0f;
      break;

    case STTS22H_1Hz:
      *Odr = 1.0f;
      break;

    case STTS22H_25Hz:
      *Odr = 25.0f;
      break;

    case STTS22H_50Hz:
      *Odr = 50.0f;
      break;

    case STTS22H_100Hz:
      *Odr = 100.0f;
      break;

    case STTS22H_200Hz:
      *Odr = 200.0f;
      break;

    default:
      ret = STTS22H_ERROR;
      break;
  }

  return ret;
}

/**
 * @brief  Set the STTS22H temperature sensor output data rate
 * @param  Odr the output data rate value to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::SetOutputDataRate(float Odr)
{
  stts22h_odr_temp_t new_odr;

  new_odr = (Odr <= 1.0f   ) ? STTS22H_1Hz
          : (Odr <= 25.0f  ) ? STTS22H_25Hz
          : (Odr <= 50.0f  ) ? STTS22H_50Hz
          : (Odr <= 100.0f ) ? STTS22H_100Hz
          :                    STTS22H_200Hz;

  if (stts22h_temp_data_rate_set(&reg_ctx, new_odr) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature value
 * @param  Value pointer where the temperature value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::GetTemperature(float *Value)
{
  int16_t raw_value = 0;

  /* Get the temperature */
  if (stts22h_temperature_raw_get(&reg_ctx, &raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  *Value = stts22h_from_lsb_to_celsius(raw_value);

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature data ready bit value
 * @param  Status the status of data ready bit
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Get_DRDY_Status(uint8_t *Status)
{
  uint8_t val;

  if (stts22h_temp_flag_data_ready_get(&reg_ctx, &val) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(val)
  {
    *Status = 1;
  } else
  {
    *Status = 0;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H low temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::SetLowTemperatureThreshold(float Value)
{
  uint8_t raw_value;

  raw_value = (uint8_t)((Value / 0.64f) + 63.0f);

  /* Set the temperature threshold */
  if (stts22h_temp_trshld_low_set(&reg_ctx, raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H high temperature threshold value
 * @param  Value the high temperature threshold to be set
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::SetHighTemperatureThreshold(float Value)
{
  uint8_t raw_value;

  raw_value = (uint8_t)((Value / 0.64f) + 63.0f);

  /* Set the temperature threshold */
  if (stts22h_temp_trshld_high_set(&reg_ctx, raw_value) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H temperature limits status
 * @param  HighLimit indicates that high temperature limit has been exceeded
 * @param  LowhLimit indicates that low temperature limit has been exceeded
 * @param  ThermLimit indicates that therm temperature limit has been exceeded
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::GetTemperatureLimitStatus(uint8_t *HighLimit, uint8_t *LowLimit)
{
  stts22h_temp_trlhd_src_t status;

  /* Read status register */
  if (stts22h_temp_trshld_src_get(&reg_ctx, &status) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(HighLimit)
  {
    *HighLimit = status.over_thh;
  }

  if(LowLimit)
  {
    *LowLimit = status.under_thl;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H register value
 * @param  Reg address to be read
 * @param  Data pointer where the value is written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Read_Reg(uint8_t Reg, uint8_t *Data)
{
  if (stts22h_read_reg(&reg_ctx, Reg, Data, 1) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H register value
 * @param  Reg address to be written
 * @param  Data value to be written
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Write_Reg(uint8_t Reg, uint8_t Data)
{
  if (stts22h_write_reg(&reg_ctx, Reg, &Data, 1) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Set the STTS22H One Shot Mode
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Set_One_Shot()
{
  /* Start One Shot Measurement */
  if(stts22h_temp_data_rate_set(&reg_ctx, STTS22H_ONE_SHOT) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  return STTS22H_OK;
}

/**
 * @brief  Get the STTS22H One Shot Status
 * @param  Status pointer to the one shot status (1 means measurements available, 0 means measurements not available yet)
 * @retval 0 in case of success, an error code otherwise
 */
STTS22HStatusTypeDef STTS22HSensor::Get_One_Shot_Status(uint8_t *Status)
{
  stts22h_dev_status_t status;

  /* Get Busy flag */
  if(stts22h_dev_status_get(&reg_ctx, &status) != STTS22H_OK)
  {
    return STTS22H_ERROR;
  }

  if(status.busy)
  {
    *Status = 0;
  }
  else
  {
    *Status = 1;
  }

  return STTS22H_OK;
}


int32_t STTS22H_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite)
{
  return ((STTS22HSensor *)handle)->IO_Write(pBuffer, WriteAddr, nBytesToWrite);
}

int32_t STTS22H_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead)
{
  return ((STTS22HSensor *)handle)->IO_Read(pBuffer, ReadAddr, nBytesToRead);
}
