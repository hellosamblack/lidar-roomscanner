/**
  ******************************************************************************
  * @file    LSM6DSO16ISSensor.h
  * @author  MEMS Software Solutions Team
  * @brief   LSM6DSO16IS header driver file
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LSM6DSO16ISSensor_H__
#define __LSM6DSO16ISSensor_H__


/* Includes ------------------------------------------------------------------*/
#include "Wire.h"
#include "SPI.h"
#include "lsm6dso16is_reg.h"
#include <string.h>


/* Typedefs ------------------------------------------------------------------*/


typedef enum {
  LSM6DSO16IS_STATUS_OK = 0,
  LSM6DSO16IS_STATUS_ERROR
} LSM6DSO16ISStatusTypeDef;

typedef enum {
  LSM6DSO16IS_INT1_PIN,
  LSM6DSO16IS_INT2_PIN,
} LSM6DSO16IS_SensorIntPin_t;

typedef struct {
  unsigned int ia_ispu_0 : 1;
  unsigned int ia_ispu_1 : 1;
  unsigned int ia_ispu_2 : 1;
  unsigned int ia_ispu_3 : 1;
  unsigned int ia_ispu_4 : 1;
  unsigned int ia_ispu_5 : 1;
  unsigned int ia_ispu_6 : 1;
  unsigned int ia_ispu_7 : 1;
  unsigned int ia_ispu_8 : 1;
  unsigned int ia_ispu_9 : 1;
  unsigned int ia_ispu_10 : 1;
  unsigned int ia_ispu_11 : 1;
  unsigned int ia_ispu_12 : 1;
  unsigned int ia_ispu_13 : 1;
  unsigned int ia_ispu_14 : 1;
  unsigned int ia_ispu_15 : 1;
  unsigned int ia_ispu_16 : 1;
  unsigned int ia_ispu_17 : 1;
  unsigned int ia_ispu_18 : 1;
  unsigned int ia_ispu_19 : 1;
  unsigned int ia_ispu_20 : 1;
  unsigned int ia_ispu_21 : 1;
  unsigned int ia_ispu_22 : 1;
  unsigned int ia_ispu_23 : 1;
  unsigned int ia_ispu_24 : 1;
  unsigned int ia_ispu_25 : 1;
  unsigned int ia_ispu_26 : 1;
  unsigned int ia_ispu_27 : 1;
  unsigned int ia_ispu_28 : 1;
  unsigned int ia_ispu_29 : 1;
} LSM6DSO16IS_ISPU_Status_t;

/* Defines -------------------------------------------------------------------*/
/* For compatibility with ESP32 platforms */
#ifdef ESP32
  #ifndef MSBFIRST
    #define MSBFIRST SPI_MSBFIRST
  #endif
#endif

#define LSM6DSO16IS_I2C_BUS                 0U
#define LSM6DSO16IS_SPI_4WIRES_BUS          1U
#define LSM6DSO16IS_SPI_3WIRES_BUS          2U

#define LSM6DSO16IS_ACC_SENSITIVITY_FS_2G   0.061f
#define LSM6DSO16IS_ACC_SENSITIVITY_FS_4G   0.122f
#define LSM6DSO16IS_ACC_SENSITIVITY_FS_8G   0.244f
#define LSM6DSO16IS_ACC_SENSITIVITY_FS_16G  0.488f

#define LSM6DSO16IS_GYRO_SENSITIVITY_FS_125DPS    4.375f
#define LSM6DSO16IS_GYRO_SENSITIVITY_FS_250DPS    8.750f
#define LSM6DSO16IS_GYRO_SENSITIVITY_FS_500DPS   17.500f
#define LSM6DSO16IS_GYRO_SENSITIVITY_FS_1000DPS  35.000f
#define LSM6DSO16IS_GYRO_SENSITIVITY_FS_2000DPS  70.000f

/* Class Declaration ---------------------------------------------------------*/

/**
 * Abstract class of an LSM6DSO16IS Inertial Measurement Unit (IMU) 3 axes
 * sensor.
 */
class LSM6DSO16ISSensor {
  public:
    LSM6DSO16ISSensor(TwoWire *i2c, uint8_t address = LSM6DSO16IS_I2C_ADD_L);
    LSM6DSO16ISSensor(SPIClass *spi, int cs_pin, uint32_t spi_speed = 2000000);

    LSM6DSO16ISStatusTypeDef begin(void);
    LSM6DSO16ISStatusTypeDef end(void);
    LSM6DSO16ISStatusTypeDef ReadID(uint8_t *Id);
    LSM6DSO16ISStatusTypeDef Read_Reg(uint8_t reg, uint8_t *Data);
    LSM6DSO16ISStatusTypeDef Read_ISPU_Output(uint8_t reg, uint8_t *Data, uint8_t len);
    LSM6DSO16ISStatusTypeDef Write_Reg(uint8_t reg, uint8_t Data);
    LSM6DSO16ISStatusTypeDef Set_Interrupt_Latch(uint8_t Status);
    LSM6DSO16ISStatusTypeDef Get_ISPU_Status(LSM6DSO16IS_ISPU_Status_t *Status);
    LSM6DSO16ISStatusTypeDef Enable_X(void);
    LSM6DSO16ISStatusTypeDef Disable_X(void);
    LSM6DSO16ISStatusTypeDef Get_X_Sensitivity(float *Sensitivity);
    LSM6DSO16ISStatusTypeDef Get_X_ODR(float *Odr);
    LSM6DSO16ISStatusTypeDef Set_X_ODR(float Odr);
    LSM6DSO16ISStatusTypeDef Get_X_FS(int32_t *FullScale);
    LSM6DSO16ISStatusTypeDef Set_X_FS(int32_t FullScale);
    LSM6DSO16ISStatusTypeDef Get_X_AxesRaw(int32_t *Value);
    LSM6DSO16ISStatusTypeDef Get_X_Axes(int32_t *Acceleration);
    LSM6DSO16ISStatusTypeDef Set_X_SelfTest(uint8_t Val);
    LSM6DSO16ISStatusTypeDef Set_X_INT1_DRDY(uint8_t Val);
    LSM6DSO16ISStatusTypeDef Get_X_Init_Status(uint8_t *Status);
    LSM6DSO16ISStatusTypeDef Get_X_DRDY_Status(uint8_t *Status);
    LSM6DSO16ISStatusTypeDef Enable_G(void);
    LSM6DSO16ISStatusTypeDef Disable_G(void);
    LSM6DSO16ISStatusTypeDef Get_G_Sensitivity(float *Sensitivity);
    LSM6DSO16ISStatusTypeDef Get_G_ODR(float *Odr);
    LSM6DSO16ISStatusTypeDef Set_G_ODR(float Odr);
    LSM6DSO16ISStatusTypeDef Get_G_FS(int32_t *FullScale);
    LSM6DSO16ISStatusTypeDef Set_G_FS(int32_t FullScale);
    LSM6DSO16ISStatusTypeDef Get_G_AxesRaw(int32_t *Value);
    LSM6DSO16ISStatusTypeDef Get_G_Axes(int32_t *AngularRate);
    LSM6DSO16ISStatusTypeDef Set_G_SelfTest(uint8_t Val);
    LSM6DSO16ISStatusTypeDef Set_G_INT1_DRDY(uint8_t Val);
    LSM6DSO16ISStatusTypeDef Get_G_Init_Status(uint8_t *Status);
    LSM6DSO16ISStatusTypeDef Get_G_DRDY_Status(uint8_t *Status);
    LSM6DSO16ISStatusTypeDef Set_DRDY_Mode(uint8_t Val);
    /**
     * @brief Utility function to read data.
     * @param  pBuffer: pointer to data to be read.
     * @param  RegisterAddr: specifies internal address register to be read.
     * @param  NumByteToRead: number of bytes to be read.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Read(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToRead)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr | 0x80);
        /* Read the data */
        for (uint16_t i = 0; i < NumByteToRead; i++) {
          *(pBuffer + i) = dev_spi->transfer(0x00);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));
        dev_i2c->write(RegisterAddr);
        dev_i2c->endTransmission(false);

        dev_i2c->requestFrom(((uint8_t)(((address) >> 1) & 0x7F)), (uint8_t) NumByteToRead);

        int i = 0;
        while (dev_i2c->available()) {
          pBuffer[i] = dev_i2c->read();
          i++;
        }

        return 0;
      }

      return 1;
    }

    /**
     * @brief Utility function to write data.
     * @param  pBuffer: pointer to data to be written.
     * @param  RegisterAddr: specifies internal address register to be written.
     * @param  NumByteToWrite: number of bytes to write.
     * @retval 0 if ok, an error code otherwise.
     */
    uint8_t IO_Write(uint8_t *pBuffer, uint8_t RegisterAddr, uint16_t NumByteToWrite)
    {
      if (dev_spi) {
        dev_spi->beginTransaction(SPISettings(spi_speed, MSBFIRST, SPI_MODE3));

        digitalWrite(cs_pin, LOW);

        /* Write Reg Address */
        dev_spi->transfer(RegisterAddr);
        /* Write the data */
        for (uint16_t i = 0; i < NumByteToWrite; i++) {
          dev_spi->transfer(pBuffer[i]);
        }

        digitalWrite(cs_pin, HIGH);

        dev_spi->endTransaction();

        return 0;
      }

      if (dev_i2c) {
        dev_i2c->beginTransmission(((uint8_t)(((address) >> 1) & 0x7F)));

        dev_i2c->write(RegisterAddr);
        for (uint16_t i = 0 ; i < NumByteToWrite ; i++) {
          dev_i2c->write(pBuffer[i]);
        }

        dev_i2c->endTransmission(true);

        return 0;
      }

      return 1;
    }

  private:
    LSM6DSO16ISStatusTypeDef Set_X_ODR_When_Enabled(float Odr);
    LSM6DSO16ISStatusTypeDef Set_X_ODR_When_Disabled(float Odr);
    LSM6DSO16ISStatusTypeDef Set_G_ODR_When_Enabled(float Odr);
    LSM6DSO16ISStatusTypeDef Set_G_ODR_When_Disabled(float Odr);

    /* Helper classes. */
    TwoWire *dev_i2c;
    SPIClass *dev_spi;

    /* Configuration */
    uint8_t address;
    int cs_pin;
    uint32_t spi_speed;

    float X_Last_ODR;
    float G_Last_ODR;
    uint8_t X_isEnabled;
    uint8_t G_isEnabled;
    uint8_t isInitialized;

    lsm6dso16is_ctx_t reg_ctx;
};

#ifdef __cplusplus
extern "C" {
#endif
int32_t LSM6DSO16IS_io_write(void *handle, uint8_t WriteAddr, uint8_t *pBuffer, uint16_t nBytesToWrite);
int32_t LSM6DSO16IS_io_read(void *handle, uint8_t ReadAddr, uint8_t *pBuffer, uint16_t nBytesToRead);
#ifdef __cplusplus
}
#endif

#endif

