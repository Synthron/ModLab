/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define LED_SPI_Pin GPIO_PIN_2
#define LED_SPI_GPIO_Port GPIOE
#define LED_I2C_Pin GPIO_PIN_3
#define LED_I2C_GPIO_Port GPIOE
#define LED_SD_Pin GPIO_PIN_4
#define LED_SD_GPIO_Port GPIOE
#define LED_CAN_Pin GPIO_PIN_5
#define LED_CAN_GPIO_Port GPIOE
#define LED_TFT_Pin GPIO_PIN_6
#define LED_TFT_GPIO_Port GPIOE
#define ESP_RDY_Pin GPIO_PIN_0
#define ESP_RDY_GPIO_Port GPIOC
#define ESP_CS_Pin GPIO_PIN_1
#define ESP_CS_GPIO_Port GPIOC
#define RS485_DIR_Pin GPIO_PIN_2
#define RS485_DIR_GPIO_Port GPIOA
#define ESP_CSA3_Pin GPIO_PIN_3
#define ESP_CSA3_GPIO_Port GPIOA
#define SD_DETECT_Pin GPIO_PIN_0
#define SD_DETECT_GPIO_Port GPIOB
#define SD_PROTECT_Pin GPIO_PIN_1
#define SD_PROTECT_GPIO_Port GPIOB
#define LED_485_Pin GPIO_PIN_7
#define LED_485_GPIO_Port GPIOE
#define CS0_Pin GPIO_PIN_10
#define CS0_GPIO_Port GPIOE
#define CS1_Pin GPIO_PIN_11
#define CS1_GPIO_Port GPIOE
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOE
#define CS3_Pin GPIO_PIN_13
#define CS3_GPIO_Port GPIOE
#define CS4_Pin GPIO_PIN_14
#define CS4_GPIO_Port GPIOE
#define CS5_Pin GPIO_PIN_15
#define CS5_GPIO_Port GPIOE
#define IO_0_Pin GPIO_PIN_10
#define IO_0_GPIO_Port GPIOD
#define IO_1_Pin GPIO_PIN_11
#define IO_1_GPIO_Port GPIOD
#define IO_2_Pin GPIO_PIN_12
#define IO_2_GPIO_Port GPIOD
#define IO_3_Pin GPIO_PIN_13
#define IO_3_GPIO_Port GPIOD
#define IO_4_Pin GPIO_PIN_14
#define IO_4_GPIO_Port GPIOD
#define IO_5_Pin GPIO_PIN_15
#define IO_5_GPIO_Port GPIOD
#define USB_Disconnect_Pin GPIO_PIN_10
#define USB_Disconnect_GPIO_Port GPIOA
#define LED_USB_Pin GPIO_PIN_0
#define LED_USB_GPIO_Port GPIOE
#define LED_ESP_Pin GPIO_PIN_1
#define LED_ESP_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
