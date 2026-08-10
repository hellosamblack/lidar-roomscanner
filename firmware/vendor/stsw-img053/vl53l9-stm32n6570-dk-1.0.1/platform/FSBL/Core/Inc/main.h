/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define MIC_D1_Pin GPIO_PIN_8
#define MIC_D1_GPIO_Port GPIOE
#define LCD_B4_Pin GPIO_PIN_3
#define LCD_B4_GPIO_Port GPIOH
#define LCD_B5_Pin GPIO_PIN_6
#define LCD_B5_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define LCD_R2_Pin GPIO_PIN_15
#define LCD_R2_GPIO_Port GPIOD
#define LCD_HSYNC_Pin GPIO_PIN_14
#define LCD_HSYNC_GPIO_Port GPIOB
#define VCP_TX_Pin GPIO_PIN_5
#define VCP_TX_GPIO_Port GPIOE
#define LCD_B2_Pin GPIO_PIN_2
#define LCD_B2_GPIO_Port GPIOB
#define NRST_CAM_Pin GPIO_PIN_8
#define NRST_CAM_GPIO_Port GPIOC
#define SD_CK_Pin GPIO_PIN_2
#define SD_CK_GPIO_Port GPIOC
#define VCP_RX_Pin GPIO_PIN_6
#define VCP_RX_GPIO_Port GPIOE
#define XSHUT_ARD_Pin GPIO_PIN_5
#define XSHUT_ARD_GPIO_Port GPIOD
#define LCD_G4_Pin GPIO_PIN_15
#define LCD_G4_GPIO_Port GPIOB
#define TOF_INT_Pin GPIO_PIN_0
#define TOF_INT_GPIO_Port GPIOQ
#define TOF_INT_EXTI_IRQn EXTI0_IRQn
#define LCD_VSYNC_Pin GPIO_PIN_11
#define LCD_VSYNC_GPIO_Port GPIOE
#define LCD_R7_Pin GPIO_PIN_8
#define LCD_R7_GPIO_Port GPIOD
#define SAI1_FS_A_Pin GPIO_PIN_0
#define SAI1_FS_A_GPIO_Port GPIOB
#define SAI1_SD_A_Pin GPIO_PIN_7
#define SAI1_SD_A_GPIO_Port GPIOB
#define SAI1_CLK_A_Pin GPIO_PIN_6
#define SAI1_CLK_A_GPIO_Port GPIOB
#define User_Pin GPIO_PIN_13
#define User_GPIO_Port GPIOC
#define LCD_R4_Pin GPIO_PIN_4
#define LCD_R4_GPIO_Port GPIOH
#define SAI1_SD_B_Pin GPIO_PIN_3
#define SAI1_SD_B_GPIO_Port GPIOE
#define MIC_CK_Pin GPIO_PIN_2
#define MIC_CK_GPIO_Port GPIOE
#define TAMP_Pin GPIO_PIN_0
#define TAMP_GPIO_Port GPIOE
#define HEXASPI_IO_7_Pin GPIO_PIN_7
#define HEXASPI_IO_7_GPIO_Port GPIOP
#define HEXASPI_IO_6_Pin GPIO_PIN_6
#define HEXASPI_IO_6_GPIO_Port GPIOP
#define HEXASPI_IO_0_Pin GPIO_PIN_0
#define HEXASPI_IO_0_GPIO_Port GPIOP
#define LCD_R1_Pin GPIO_PIN_9
#define LCD_R1_GPIO_Port GPIOD
#define HEXASPI_IO_4_Pin GPIO_PIN_4
#define HEXASPI_IO_4_GPIO_Port GPIOP
#define HEXASPI_IO_1_Pin GPIO_PIN_1
#define HEXASPI_IO_1_GPIO_Port GPIOP
#define HEXASPI_IO_15_Pin GPIO_PIN_15
#define HEXASPI_IO_15_GPIO_Port GPIOP
#define HEXASPI_IO_5_Pin GPIO_PIN_5
#define HEXASPI_IO_5_GPIO_Port GPIOP
#define HEXASPI_IO_12_Pin GPIO_PIN_12
#define HEXASPI_IO_12_GPIO_Port GPIOP
#define HEXASPI_IO_3_Pin GPIO_PIN_3
#define HEXASPI_IO_3_GPIO_Port GPIOP
#define HEXASPI_IO_2_Pin GPIO_PIN_2
#define HEXASPI_IO_2_GPIO_Port GPIOP
#define HEXASPI_IO_13_Pin GPIO_PIN_13
#define HEXASPI_IO_13_GPIO_Port GPIOP
#define HEXASPI_DQS0_Pin GPIO_PIN_2
#define HEXASPI_DQS0_GPIO_Port GPIOO
#define SAI1_MCLK_A_Pin GPIO_PIN_7
#define SAI1_MCLK_A_GPIO_Port GPIOG
#define LCD_B3_Pin GPIO_PIN_6
#define LCD_B3_GPIO_Port GPIOG
#define INTR_ARD_Pin GPIO_PIN_6
#define INTR_ARD_GPIO_Port GPIOF
#define INTR_ARD_EXTI_IRQn EXTI6_IRQn
#define HEXASPI_IO_11_Pin GPIO_PIN_11
#define HEXASPI_IO_11_GPIO_Port GPIOP
#define HEXASPI_IO_8_Pin GPIO_PIN_8
#define HEXASPI_IO_8_GPIO_Port GPIOP
#define HEXASPI_IO_14_Pin GPIO_PIN_14
#define HEXASPI_IO_14_GPIO_Port GPIOP
#define HEXASPI_DQS1_Pin GPIO_PIN_3
#define HEXASPI_DQS1_GPIO_Port GPIOO
#define HEXASPI_NCS_Pin GPIO_PIN_0
#define HEXASPI_NCS_GPIO_Port GPIOO
#define HEXASPI_IO_9_Pin GPIO_PIN_9
#define HEXASPI_IO_9_GPIO_Port GPIOP
#define HEXASPI_IO_10_Pin GPIO_PIN_10
#define HEXASPI_IO_10_GPIO_Port GPIOP
#define HEXASPI_CLK_Pin GPIO_PIN_4
#define HEXASPI_CLK_GPIO_Port GPIOO
#define OCTOSPI_IO2_Pin GPIO_PIN_4
#define OCTOSPI_IO2_GPIO_Port GPION
#define LCD_G2_Pin GPIO_PIN_1
#define LCD_G2_GPIO_Port GPIOA
#define LCD_G6_Pin GPIO_PIN_11
#define LCD_G6_GPIO_Port GPIOB
#define LCD_R5_Pin GPIO_PIN_15
#define LCD_R5_GPIO_Port GPIOA
#define OCTOSPI_CLK_Pin GPIO_PIN_6
#define OCTOSPI_CLK_GPIO_Port GPION
#define OCTOSPI_IO4_Pin GPIO_PIN_8
#define OCTOSPI_IO4_GPIO_Port GPION
#define OCTOSPI_DQS_Pin GPIO_PIN_0
#define OCTOSPI_DQS_GPIO_Port GPION
#define LCD_B0_Pin GPIO_PIN_15
#define LCD_B0_GPIO_Port GPIOG
#define LCD_G1_Pin GPIO_PIN_1
#define LCD_G1_GPIO_Port GPIOG
#define LCD_G5_Pin GPIO_PIN_12
#define LCD_G5_GPIO_Port GPIOB
#define LCD_B1_Pin GPIO_PIN_7
#define LCD_B1_GPIO_Port GPIOA
#define LCD_R0_Pin GPIO_PIN_0
#define LCD_R0_GPIO_Port GPIOG
#define OCTOSPI_IO1_Pin GPIO_PIN_3
#define OCTOSPI_IO1_GPIO_Port GPION
#define OCTOSPI_IO3_Pin GPIO_PIN_5
#define OCTOSPI_IO3_GPIO_Port GPION
#define OCTOSPI_NCS_Pin GPIO_PIN_1
#define OCTOSPI_NCS_GPIO_Port GPION
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define UCPD1_VSENSE_Pin GPIO_PIN_11
#define UCPD1_VSENSE_GPIO_Port GPIOA
#define LCD_B7_Pin GPIO_PIN_2
#define LCD_B7_GPIO_Port GPIOA
#define LCD_G0_Pin GPIO_PIN_12
#define LCD_G0_GPIO_Port GPIOG
#define LCD_R3_Pin GPIO_PIN_4
#define LCD_R3_GPIO_Port GPIOB
#define LCd_G7_Pin GPIO_PIN_8
#define LCd_G7_GPIO_Port GPIOG
#define OCTOSPI_IO5_Pin GPIO_PIN_9
#define OCTOSPI_IO5_GPIO_Port GPION
#define OCTOSPI_IO0_Pin GPIO_PIN_2
#define OCTOSPI_IO0_GPIO_Port GPION
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define LCD_B6_Pin GPIO_PIN_8
#define LCD_B6_GPIO_Port GPIOA
#define LCD_DE_Pin GPIO_PIN_13
#define LCD_DE_GPIO_Port GPIOG
#define LCD_G3_Pin GPIO_PIN_0
#define LCD_G3_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_5
#define SWO_GPIO_Port GPIOB
#define LCD_R6_Pin GPIO_PIN_11
#define LCD_R6_GPIO_Port GPIOG
#define OCTOSPI_IO6_Pin GPIO_PIN_10
#define OCTOSPI_IO6_GPIO_Port GPION
#define OCTOSPI_IO7_Pin GPIO_PIN_11
#define OCTOSPI_IO7_GPIO_Port GPION

/* USER CODE BEGIN Private defines */
#define LCD_BL_CTRL_GPIO_Port GPIOQ
#define LCD_BL_CTRL_Pin       GPIO_PIN_6
#define LCD_ONOFF_GPIO_Port   GPIOQ
#define LCD_ONOFF_Pin         GPIO_PIN_3

// active high
#define LED1_GREEN_GPIO_Port GPIOO
#define LED1_GREEN_Pin GPIO_PIN_1

// active low
#define LED2_RED_GPIO_Port GPIOG
#define LED2_RED_Pin GPIO_PIN_10
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
