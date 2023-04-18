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
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "modlab.h"
#include "ad9833.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_V_SQUARE (float)779.8319328
#define SQUARE_OFFSET (0x0100 * 27 + 0x00FF)
#define PWM_V_SINE  (float)2645.180602
#define SINE_OFFSET (0x0100 * 30 + 0x00FF)
#define PWM_OFFSET_NULL 0x8210
#define PWM_OFFSET_STEP100 22*0x0010

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t address;

uint8_t outputState = 0;
uint8_t setfreq = 0;

uint8_t waveform = wave_square;
uint16_t gain  = 0;
int16_t offset = 0;
uint32_t freq = 0;

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
void setgain(uint8_t voltage);
void setoffset(int16_t os);
void CAN_Filter(uint8_t addr);
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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  timer_init();

  HAL_CAN_Start(&hcan);

  // get address
  address = ADDR_WaveGen | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) << 1);


  HAL_GPIO_WritePin(OUT_Relais_GPIO_Port, OUT_Relais_Pin, 0);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // GAIN
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // OFFSET

  CAN_Filter(address);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __enable_irq();

  
  AD9833_Init(waveform, 0, 0);
  setgain(0);
  setoffset(0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(OUT_Relais_GPIO_Port, OUT_Relais_Pin, outputState);
    if(setfreq)
    {
      setfreq = 0;
      AD9833_Init(waveform, freq, 0);
    }

    setgain(gain);
    setoffset(offset);


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
    case GAINOUT:
      gain = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    case OFFSETOUT:
      offset = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    default: // Index Mismatch
      stat = 1;
      break;
    }
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_SET32) // set 32bit
  {
    switch (RxData[1])
    {
    case FREQUENCY:
      freq = (RxData[2] << 24) | (RxData[3] << 16) | (RxData[4] << 8) | RxData[5];
      stat = 0;
      setfreq = 1;
      break;
    default: // Index Mismatch
      stat = 1;
      break;
    }
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_SETMODE) // set opmode waveform
  {
    switch (RxData[1])
    {
    case SQUAREWAVE:
      stat = 0;
      waveform = wave_square;
      setfreq = 1;
      break;
    case SINEWAVE:
      stat = 0;
      waveform = wave_sine;
      setfreq = 1;
      break;
    case TRIANGLEWAVE:
      stat = 0;
      waveform = wave_triangle;
      setfreq = 1;
      break;
    default: // Index Mismatch
      stat = 1;
      break;
    }
    send_rec(address, RxData[0], stat);
  }
  else if (RxData[0] == CMD_ENOUT || RxData[0] == CMD_DISOUT) // set Output
  {
    outputState = !(RxData[0] & 0x01);
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
  canfilterconfig.FilterIdHigh = (0x500 | addr) << 5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x1FFF;
  canfilterconfig.FilterMaskIdLow = 0xFFFF;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 1; // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
}

void setgain(uint8_t voltage)
{
  switch (waveform)
  {
  case wave_square:
    TIM2->CCR1 = ((uint16_t)(((float)(voltage-4)/10.0) * PWM_V_SQUARE)) + SQUARE_OFFSET;
    break;
  case wave_sine:
  case wave_triangle:
    TIM2->CCR1 = ((uint16_t)(((float)(voltage)/10.0) * PWM_V_SINE)) + SINE_OFFSET;
    break;  
  default:
    break;
  }
}

void setoffset(int16_t os)
{
  TIM2->CCR2 = PWM_OFFSET_NULL - (os * PWM_OFFSET_STEP100);
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
