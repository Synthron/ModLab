/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * 10mV Resolution
 * 1mA  Resolution
 *
 * https://hackaday.io/project/4154-bench-power-supply
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

#define V_POS 0
#define I_POS 1
#define V_NEG 2
#define I_NEG 3

#define PWM_MAX 0xFFFF

#define VOLTP_ZERO 0x0900
#define VOLTP_MAX 1173

#define AMPP_ZERO 0x0B50
#define AMPP_REF 0x90E0
#define AMPP_MAX 1270

#define VOLTN_ZERO 0x0A70
#define VOLTN_MAX 1174

#define AMPN_ZERO 0x0B50
#define AMPN_REF 0x90E0
#define AMPN_MAX 1270

#define ADCpv_factor 0.52
#define ADCpv_offset 0
#define ADCpc_factor 0.52
#define ADCpc_offset 0
#define ADCnv_factor 0.52
#define ADCnv_offset 0
#define ADCnc_factor 0.52
#define ADCnc_offset 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t address;

uint16_t setValues[4], getValues[4];
uint8_t outputState = 0;

uint32_t raw[4];

// https://controllerstech.com/can-protocol-in-stm32/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;
uint8_t RxData[8];
uint8_t stat = 0;

uint8_t data1[1], data2[2];

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
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  timer_init();

  HAL_CAN_Start(&hcan);

  // get address
  address = ADDR_SymPSU | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)) | (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) << 1);

  // calibrate AD convertor
  while (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    ;

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Volt+
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Amp+
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // Volt-
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // Amp-

  CAN_Filter(address);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
  __enable_irq();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    HAL_GPIO_WritePin(Out_EN_GPIO_Port, Out_EN_Pin, outputState);
    Set_All();
    delay_us(10);
    for (uint8_t i = 0; i < 4; i++)
    {
      Read_Single(i);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  else if (RxData[0] == CMD_SET16) // set 16bit
  {
    switch (RxData[1])
    {
    case V_POS_SET:
      setValues[0] = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    case I_POS_SET:
      setValues[1] = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    case V_NEG_SET:
      setValues[2] = (RxData[2] << 8) | RxData[3];
      stat = 0;
      break;
    case I_NEG_SET:
      setValues[3] = (RxData[2] << 8) | RxData[3];
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
    case V_POS_GET:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[0]);
      break;
    case I_POS_GET:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[1]);
      break;
    case V_NEG_GET:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[2]);
      break;
    case I_NEG_GET:
      stat = 0;
      send_rec16(address, RxData[0], stat, getValues[3]);
      break;
    default: // Index Mismatch
      stat = 1;
      send_rec(address, RxData[0], stat);
      break;
    }
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

void Read_Single(uint8_t ch)
{
  switch (ch)
  {
  case 0:
    ADC_Select_CH0();
    break;
  case 1:
    ADC_Select_CH1();
    break;
  case 2:
    ADC_Select_CH2();
    break;
  case 3:
    ADC_Select_CH3();
    break;
  }

  uint8_t samples = 5;

  for (int i = 0; i < samples; i++)
  {
    delay_us(100);
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    raw[ch] += HAL_ADC_GetValue(&hadc1);
  }
  switch (ch)
  {
  case 0:
    getValues[ch] = ADCpv_factor * (uint16_t)(raw[ch] / samples) + ADCpv_offset;
    break;
  case 1:
    getValues[ch] = ADCpc_factor * (uint16_t)(raw[ch] / samples) + ADCpc_offset;
    break;
  case 2:
    getValues[ch] = ADCnv_factor * (uint16_t)(raw[ch] / samples) + ADCnv_offset;
    break;
  case 3:
    getValues[ch] = ADCnc_factor * (uint16_t)(raw[ch] / samples) + ADCnc_offset;
    break;
  }

  raw[ch] = 0;
}

void Set_All(void)
{
  Set_Output(V_POS, setValues[0]);
  Set_Output(I_POS, setValues[1]);
  Set_Output(V_NEG, setValues[2]);
  Set_Output(I_NEG, setValues[3]);
}

void Set_Output(uint8_t channel, uint16_t value)
{
  uint16_t temp1;
  double multiplicator;
  switch (channel)
  {
  case V_POS:
    if (value > VOLTP_MAX)
      value = VOLTP_MAX;
    multiplicator = (double)((PWM_MAX - VOLTP_ZERO) / (double)VOLTP_MAX) + 1.0;
    temp1 = (uint16_t)((double)value * multiplicator) + VOLTP_ZERO;
    TIM2->CCR1 = temp1;
    break;
  case I_POS:
    if (value > AMPP_MAX)
      value = AMPP_MAX;
    multiplicator = (double)((AMPP_REF - AMPP_ZERO) / (double)AMPP_MAX) + 1.8;
    temp1 = (uint16_t)((double)value * multiplicator) + AMPP_ZERO;
    TIM2->CCR2 = temp1;
    break;
  case V_NEG:
    if (value > VOLTN_MAX)
      value = VOLTN_MAX;
    multiplicator = (double)((PWM_MAX - VOLTN_ZERO) / (double)VOLTN_MAX) + 1.0;
    temp1 = (uint16_t)((double)value * multiplicator) + VOLTN_ZERO;
    TIM2->CCR3 = temp1;
    break;
  case I_NEG:
    // if(value > AMPN_MAX)
    //   value = AMPN_MAX;
    // multiplicator = (double)((AMPN_REF - AMPN_ZERO) / (double)AMPN_MAX) + 1.8;
    // temp1 = (uint16_t)((double)value * multiplicator) + AMPN_ZERO;
    // TIM2->CCR4 = temp1;
    TIM2->CCR4 = value;
    break;
  default:
    break;
  }
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

#ifdef USE_FULL_ASSERT
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
