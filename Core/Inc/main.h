/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GAIN1_Pin GPIO_PIN_0
#define GAIN1_GPIO_Port GPIOC
#define GAIN0_Pin GPIO_PIN_1
#define GAIN0_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_2
#define A0_GPIO_Port GPIOC
#define TEMP_Pin GPIO_PIN_3
#define TEMP_GPIO_Port GPIOC
#define DRDY_DROUT_Pin GPIO_PIN_0
#define DRDY_DROUT_GPIO_Port GPIOA
#define SCLK_Pin GPIO_PIN_1
#define SCLK_GPIO_Port GPIOA
#define PDWN_Pin GPIO_PIN_2
#define PDWN_GPIO_Port GPIOA
#define SPEED_Pin GPIO_PIN_3
#define SPEED_GPIO_Port GPIOA
#define X_Z_SELECT_Pin GPIO_PIN_0
#define X_Z_SELECT_GPIO_Port GPIOB
#define ACCEL_MISO_Pin GPIO_PIN_2
#define ACCEL_MISO_GPIO_Port GPIOB
#define ACCEL_SCK_Pin GPIO_PIN_10
#define ACCEL_SCK_GPIO_Port GPIOB
#define ACCEL_MOSI_Pin GPIO_PIN_11
#define ACCEL_MOSI_GPIO_Port GPIOB
#define ACCEL_CS_Pin GPIO_PIN_12
#define ACCEL_CS_GPIO_Port GPIOB
#define SENSOR_I2C2_SCL_Pin GPIO_PIN_13
#define SENSOR_I2C2_SCL_GPIO_Port GPIOB
#define SENSOR_I2C2_SDA_Pin GPIO_PIN_14
#define SENSOR_I2C2_SDA_GPIO_Port GPIOB
#define FLASH_NSS_Pin GPIO_PIN_7
#define FLASH_NSS_GPIO_Port GPIOC
#define FLASH_SCK_Pin GPIO_PIN_8
#define FLASH_SCK_GPIO_Port GPIOD
#define FLASH_WP_Pin GPIO_PIN_9
#define FLASH_WP_GPIO_Port GPIOD
#define FLASH_HOLD_Pin GPIO_PIN_10
#define FLASH_HOLD_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_11
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_12
#define FLASH_MOSI_GPIO_Port GPIOA
#define DBG_GPIO1_Pin GPIO_PIN_9
#define DBG_GPIO1_GPIO_Port GPIOC
#define Test_pin_3_Pin GPIO_PIN_0
#define Test_pin_3_GPIO_Port GPIOD
#define DBG_GPIO2_Pin GPIO_PIN_1
#define DBG_GPIO2_GPIO_Port GPIOD
#define LED_ERROR_Pin GPIO_PIN_2
#define LED_ERROR_GPIO_Port GPIOD
#define LED_STATUS_Pin GPIO_PIN_3
#define LED_STATUS_GPIO_Port GPIOD
#define USART2DE_Pin GPIO_PIN_4
#define USART2DE_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
