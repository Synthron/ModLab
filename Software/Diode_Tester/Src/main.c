/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modlab.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADC_OFFSET 20
#define ADC_VOLT 420

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t address;

uint8_t start_curr, end_curr, mode;
uint8_t start = 0;
uint16_t currents[15];

uint32_t raw;

// https://controllerstech.com/can-protocol-in-stm32/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
uint8_t RxData[8];
uint8_t stat = 0;

uint32_t average;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint32_t measure_Diode (void);
void CAN_Filter(uint8_t addr);
void measure_sequence (void);
void set_output (uint8_t val);

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  //initialize us-timer
  timer_init();


  //Set outputs to 0
  HAL_GPIO_WritePin(GPIOA, MA2_Pin, 0);
  HAL_GPIO_WritePin(GPIOA, MA5_Pin, 0);
  HAL_GPIO_WritePin(GPIOA, MA10_Pin, 0);
  HAL_GPIO_WritePin(GPIOA, MA20_Pin, 0);

  //initialize address
  address = ADDR_DiodeTester | (HAL_GPIO_ReadPin(GPIOB, ADDR0_Pin)) | (HAL_GPIO_ReadPin(GPIOB, ADDR1_Pin) << 1);

  //calibrate ADC
  while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);

  //initialize CAN and enable Interrupt
  HAL_CAN_Start(&hcan);
  CAN_Filter(address);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(start)
    {
      for(int i = 0; i < 15; i++)
        currents[i] = 0;
      start = 0;
      measure_sequence();
      send_cmd(ADDR_HBC & 0x0ff | ADDR_AUTO, AUTO_DONE);
    }    

    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
    Error_Handler();
  }

  if (RxData[0] == CMD_PING)
  {
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_SET8) // set 8bit
  {
    switch (RxData[1])
    {
    case START_CURR:
      start_curr = RxData[2];
      stat = 0;
      break;
    case END_CURR:
      end_curr = RxData[2];
      stat = 0;
      break;
    default: // Index Mismatch
      stat = 1;
      break;
    }
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_GET16) // get 16bit
  {
    if(RxData[1] >= 0x10 && RxData[1] <= 0x1E)
    {
      stat = 0;
      send_rec16(address, RxData[0], stat, currents[RxData[1] & 0x0F]);
    }
    else
    {
      stat = 1; //Index Mismatch
      send_rec(address, RxData[0], stat);
    }
  }
  else if (RxData[0] == CMD_SETMODE) // set operating mode
  {
    mode = RxData[1]-1;
    stat = 0;
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_ENOUT) // start conversion
  {
    start = 1;
    stat = 0;
    send_rec(address, RxData[0], stat);
  }
  else // Unknown Command
  {
    stat = 0x40;
    send_rec(address, RxData[0], stat);
  }
}

void CAN_Filter(uint8_t addr)
{
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 0; // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = addr << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x1FFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFF;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 1; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

void measure_sequence (void)
{
  uint8_t i = 0;
  switch (mode)
  {
    case MODE_SINGLE:
      set_output(start_curr+1);
      currents[start_curr] = (uint16_t)measure_Diode();
      break;
    case MODE_COARSE:
      if(start_curr == 0)
        i = 0x1;
      else if(start_curr <= 2)
        i = 0x2;
      else if(start_curr <= 4)
        i = 4;
      else 
        i = 8;
      while(i <= end_curr)
      {
        set_output(i);
        currents[i-1] = (uint16_t)measure_Diode();
        i = i << 1;
      }
      break;
    case MODE_MEDIUM:
      i = start_curr;
      while(i <= end_curr)
      {
        set_output(i+1);
        currents[i] = (uint16_t)measure_Diode();
        i += 2;
      }
      break;
    case MODE_FINE:
      i = start_curr;
      while(i <= end_curr)
      {
        set_output(i+1);
        currents[i] = (uint16_t)measure_Diode();
        i++;
      }
      break;
  }
  set_output(0);
}

void set_output (uint8_t val)
{
  HAL_GPIO_WritePin(GPIOA, MA2_Pin, val & 0x01);
  HAL_GPIO_WritePin(GPIOA, MA5_Pin, (val & 0x02) >> 1);
  HAL_GPIO_WritePin(GPIOA, MA10_Pin, (val & 0x04) >> 2);
  HAL_GPIO_WritePin(GPIOA, MA20_Pin, (val & 0x08) >> 3);
  HAL_Delay(100);
}

uint32_t measure_Diode (void)
{
  raw = 0;
  //measuring
  for(uint8_t i = 0; i < 10; i++)
  {
    delay_us(100);
    HAL_ADC_Start(&hadc1);                                      // start analog to digital conversion
    while(HAL_ADC_PollForConversion(&hadc1, 1000000) != HAL_OK);// wait for completing the conversion
    raw += HAL_ADC_GetValue(&hadc1);                        // read sensor's digital value
  }
  //average and return
  return ((((raw / 10)+ ADC_OFFSET)*100) / (ADC_VOLT));
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
