/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dac.h"
#include "fatfs.h"
#include "i2c.h"
#include "rtc.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "tft.h"
#include "modlab.h"
#include <stdio.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t buffer[64];
char out_buf[64];
uint16_t counter;
uint32_t tick_loop;

uint8_t address = 0;

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t send_ok = 0;

uint8_t can_monitor = 1;
uint8_t can_ack = 0;
uint8_t rec_ack, rec_nack, rec_8;
uint16_t rec_16;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_UART7_Init();
  MX_DAC_Init();
  MX_FMC_Init();
  MX_SPI3_Init();
  MX_SPI4_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

  timer_init();

  LEDs_Off();
  HAL_GPIO_WritePin(USB_Disconnect_GPIO_Port, USB_Disconnect_Pin, 1);
  HAL_Delay(200);
  //HAL_GPIO_WritePin(USB_Disconnect_GPIO_Port, USB_Disconnect_Pin, 0);

  CAN_Filter();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  __enable_irq();

  LCD_Init();
  LCD_Rect_Fill(0, 0, 799, 479, BLUE);
	LCD_Rect_Fill(10, 10, 788, 468, BLACK);
	

//https://controllerstech.com/can-protocol-in-stm32/




  HAL_Delay(2000);


  tick_loop = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    send_set16(ADDR_SymPSU, V_POS_SET, counter*100);

    HAL_Delay(500);

    send_set16(ADDR_SymPSU, I_POS_SET, 500);

    HAL_Delay(500);

    send_cmd(ADDR_SymPSU, CMD_ENOUT);

    HAL_Delay(500);

    counter++;
    if(counter > 10)
      counter = 0;
    
    HAL_GPIO_TogglePin(LED_CAN_GPIO_Port, LED_CAN_Pin);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_UART7
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_SDMMC1|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInitStruct.Uart7ClockSelection = RCC_UART7CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  PeriphClkInitStruct.Sdmmc1ClockSelection = RCC_SDMMC1CLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void transmit(void)
{
  send_ok = 0;
  #ifdef HBC
  if(can_monitor)
  {
    int buflen = sprintf(out_buf,"[s] 0x%03x ", (uint16_t)TxHeader.StdId);
    for(uint8_t i = 0; i < TxHeader.DLC; i++)
    {
      buflen += sprintf(out_buf+buflen, "%02x ", TxData[i]);
    }
    sprintf(out_buf+buflen, "\n");
    CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
  }
  #endif
  
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
  uint32_t temp = HAL_GetTick();
  while(!send_ok && (HAL_GetTick() - temp < 1000));

}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }
  switch (RxData[0])
  {
  case 0x01: //Reset Module
    rec_ack = RxData[1];
    break;
  case 0x10: //Set 8bit
    rec_ack = RxData[1];
    break;
  case 0x11: //Get 8bit
    rec_ack = RxData[1];
    rec_8 = RxData[2];
    break;
  case 0x14: //Set 16bit
    rec_ack = RxData[1];
    break;
  case 0x15: //get 16bit
    rec_ack = RxData[1];
    rec_8 = RxData[2] << 8 | RxData[3];
    break;
  case 0x20: //Get Status
    rec_ack = RxData[1];
    rec_8 = RxData[2];
    break;
  case 0x40: //Enable Output
    rec_ack = RxData[1];
    break;
  case 0x41: //Disable Output
    rec_ack = RxData[1];
    break;
  case 0x60: //Set Operation mode
    rec_ack = RxData[1];
    break;
  case 0x61: //Get Operation Mode
    rec_ack = RxData[1];
    rec_8 = RxData[2];
    break;

  default:
    rec_ack = 0xFF;
    break;
  }


  if(can_monitor)
  {
    int buflen = sprintf(out_buf,"[r] 0x%03x ", (uint16_t)RxHeader.StdId);
    for(uint8_t i = 0; i < RxHeader.DLC; i++)
    {
      buflen += sprintf(out_buf+buflen, "%02x ", RxData[i]);
    }
    sprintf(out_buf+buflen, "\n");
    CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
  }
  
  send_ok = 1;
}

void CAN_Filter (void)
{
  //no filtering, Message types decoded in interrupt
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0; 
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x0000;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0; 
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 1; 

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
}

void USB_Mon(void)
{

  sprintf(out_buf,"USB-Receive Test\r\n");
  CDC_Transmit_FS((uint8_t *)buffer, strlen((const char*)buffer));

  buffer[0] = 0xff;

}

void LEDs_Off(void)
{
  HAL_GPIO_WritePin(LED_CAN_GPIO_Port,  LED_CAN_Pin,  0);   //CAN
  HAL_GPIO_WritePin(LED_485_GPIO_Port,  LED_485_Pin,  0);   //485
  HAL_GPIO_WritePin(LED_I2C1_GPIO_Port, LED_I2C1_Pin, 0);   //I2C1
  HAL_GPIO_WritePin(LED_I2C2_GPIO_Port, LED_I2C2_Pin, 0);   //I2C1
  HAL_GPIO_WritePin(LED_ESP_GPIO_Port,  LED_ESP_Pin,  0);   //ESP
  HAL_GPIO_WritePin(LED_SD_GPIO_Port,   LED_SD_Pin,   0);   //SD
  HAL_GPIO_WritePin(LED_SPI3_GPIO_Port, LED_SPI3_Pin, 0);   //SPI3
  HAL_GPIO_WritePin(LED_SPI4_GPIO_Port, LED_SPI4_Pin, 0);   //SPI4
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/