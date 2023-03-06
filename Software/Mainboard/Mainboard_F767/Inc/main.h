/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include "stm32f7xx_hal.h"

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
#define GPIO_1_Pin GPIO_PIN_0
#define GPIO_1_GPIO_Port GPIOC
#define GPIO_2_Pin GPIO_PIN_3
#define GPIO_2_GPIO_Port GPIOC
#define SD_Detect_Pin GPIO_PIN_0
#define SD_Detect_GPIO_Port GPIOA
#define SD_Protect_Pin GPIO_PIN_1
#define SD_Protect_GPIO_Port GPIOA
#define ESP_RDY_Pin GPIO_PIN_2
#define ESP_RDY_GPIO_Port GPIOA
#define ESP_CS_Pin GPIO_PIN_3
#define ESP_CS_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_4
#define GPIO_3_GPIO_Port GPIOC
#define GPIO_4_Pin GPIO_PIN_5
#define GPIO_4_GPIO_Port GPIOC
#define CS_3_Pin GPIO_PIN_0
#define CS_3_GPIO_Port GPIOB
#define CS_4_Pin GPIO_PIN_1
#define CS_4_GPIO_Port GPIOB
#define GPIO_0_Pin GPIO_PIN_12
#define GPIO_0_GPIO_Port GPIOB
#define LED_CAN_Pin GPIO_PIN_6
#define LED_CAN_GPIO_Port GPIOG
#define LED_485_Pin GPIO_PIN_7
#define LED_485_GPIO_Port GPIOG
#define LED_I2C1_Pin GPIO_PIN_8
#define LED_I2C1_GPIO_Port GPIOG
#define GPIO_5_Pin GPIO_PIN_6
#define GPIO_5_GPIO_Port GPIOC
#define CS_0_Pin GPIO_PIN_8
#define CS_0_GPIO_Port GPIOA
#define CS_1_Pin GPIO_PIN_9
#define CS_1_GPIO_Port GPIOA
#define USB_Disconnect_Pin GPIO_PIN_10
#define USB_Disconnect_GPIO_Port GPIOA
#define CS_2_Pin GPIO_PIN_15
#define CS_2_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_3
#define LCD_CS_GPIO_Port GPIOD
#define LED_I2C2_Pin GPIO_PIN_10
#define LED_I2C2_GPIO_Port GPIOG
#define LED_ESP_Pin GPIO_PIN_12
#define LED_ESP_GPIO_Port GPIOG
#define LED_SD_Pin GPIO_PIN_13
#define LED_SD_GPIO_Port GPIOG
#define LED_SPI3_Pin GPIO_PIN_14
#define LED_SPI3_GPIO_Port GPIOG
#define LED_SPI4_Pin GPIO_PIN_15
#define LED_SPI4_GPIO_Port GPIOG
#define CS_5_Pin GPIO_PIN_5
#define CS_5_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define HBC

void LEDs_Off(void);
void CAN_Filter(void);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
