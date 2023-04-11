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
#define ADC_A 251.3662556
#define PWM_FACTOR 2037.311873

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t address, samples = 10, getTemp;

uint16_t setValues[2], getValues[2];

uint32_t raw;

// https://controllerstech.com/can-protocol-in-stm32/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
uint8_t RxData[8];
uint8_t stat = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t ADC_read (uint8_t channel);
void CAN_Filter(uint8_t addr);
void set_channel(uint8_t channel, uint16_t val);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  timer_init();

  HAL_CAN_Start(&hcan);

  // get address
  address = ADDR_Load | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)) | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) << 1);

  // calibrate AD convertor
  while (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Current Channel 1
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Current Channel 2

  TIM2->CCR1 = 0;
  TIM2->CCR2 = 0;

  HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, 0);


  CAN_Filter(address);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __enable_irq();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //read temp and set fans
    getTemp = ADC_read(2);
    if(getTemp >= 35)
      HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, 1);
    if(getTemp < 30)
      HAL_GPIO_WritePin(FAN_EN_GPIO_Port, FAN_EN_Pin, 0);

    //set and get values
    set_channel(0, setValues[0]);
    set_channel(1, setValues[1]);
    HAL_Delay(10);
    getValues[0] = ADC_read(0);
    getValues[1] = ADC_read(1);

    //safety cutoff
    if(getTemp >= 60)
    {
      set_channel(0, 0);
      set_channel(1, 0);
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

  //Send Ping
  if (RxData[0] == CMD_PING)
  {
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_SET16) // set 16bit
  {
    switch (RxData[1])
    {
    case LOAD_CH0_Set:
      setValues[0] = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    case LOAD_CH1_Set:
      setValues[1] = (RxData[2] << 8) | RxData[3];
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
    switch (RxData[1])
    {
    case LOAD_CH0_Get:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[0]);
      break;
    case LOAD_CH1_Get:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[1]);
      break;
    default: // Index Mismatch
      stat = 1;
      send_rec(address, RxData[0], stat);
      break;
    }
  }
  else if (RxData[0] == CMD_GET8) // get 8bit
  {
    if(RxData[1] == LOAD_TEMP_Get)
    {
      stat = 0;
      send_rec8(address, RxData[0], stat, getTemp);
    }
    else // Index Mismatch
    {
      stat = 1;
      send_rec(address, RxData[0], stat);
    }
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
  canfilterconfig.FilterIdHigh = (0x500 | addr) << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x1FFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFF;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 1; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

uint16_t ADC_read (uint8_t channel)
{
  if(channel == 0)
    ADC_Select_CH0();
  else if (channel == 1)
    ADC_Select_CH1();
  else
    ADC_Select_CH2();

  HAL_Delay(1);

  raw = 0;

  for (int i = 0; i < samples; i++)
  {
    delay_us(100);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    raw += HAL_ADC_GetValue(&hadc1);
  }
  if(channel == 0 || channel == 1)
    return ((raw / samples)*100)/ADC_A;
    //returns currents in 10mA increments
  else
    return ((raw / samples)*3.3)/40;
    //returns temperature in °C increments
}

//set current in 10mA
void set_channel(uint8_t channel, uint16_t val)
{
  if(channel == 0)
    TIM2->CCR1 = (val*PWM_FACTOR)/100;
  else 
    TIM2->CCR2 = (val*PWM_FACTOR)/100;
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
