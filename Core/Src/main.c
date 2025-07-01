/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// Macros For 6 Timer Channels to be more readable
#define PHASE_A TIM_CHANNEL_1
#define PHASE_B TIM_CHANNEL_2
#define PHASE_C TIM_CHANNEL_3
// Macros for Throttle Range
#define MIN_THROTTLE 0
#define MAX_THROTTLE 3560
// Macros For PWM Signal Range
#define MAX_PWM 3599
#define MIN_PWM 0
/*
 * Macro For indicating the value at which the Low turns zero
 * 0 -> duty ( O/P = 0), duty -> LOW ( O/P = 1 )
 */
#define LOW 3599
// Macro For Dead Time Period
#define DEADTIME_DELAY 20 // Real Deadtime in microsecond
// Macro For indicating that Hall in invalid state to avoid any damage
#define INVALID_STATE 255
// Macro for Filter Size of ADC
#define ADC_FILTER_SIZE   254  // Size of the moving average window
// Static Global Variables for ADC Filter
static uint16_t adcSamples[ADC_FILTER_SIZE];
static uint8_t  adcIndex = 0;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Global Variables For Debugging
uint8_t  HALL_GLOBAL = 0;
uint16_t ADC_BEFORE_SAMPLING = 0;
uint16_t ADC_AFTER_SAMPLING = 0;
uint16_t ADC_MAPPING = 0;
// Global Variable For Motor Position
uint8_t  Recent_state = 0;
uint8_t  Prev_state = INVALID_STATE;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t Read_Hall_Sensors(void)
{
	// Read Hall Sensors FeedBack Signal
	uint8_t h3= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11);
	uint8_t h2= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12);
	uint8_t h1= HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
	// Shift Values to be in one variable for Checking
	uint8_t hall_value = h3<<2 | h2<<1 | h1<<0;
	// Check Values of Hall Sensor Reading ( For Debugging )
	HALL_GLOBAL = hall_value;
	return hall_value;
}

//void CheckState(uint16_t duty)
//{
//	if (duty > 3599) {
//		duty = 3599; // Clamp near 100% duty to avoid bootstrap capacitor issues
//	}
//	Recent_state = hallState;
//
//	if (Recent_state != Prev_state || Recent_state == INVALID_STATE) {
//		safeDisableOutputs(); // Unified handling for state change or invalid state
//		if (new_state != INVALID_STATE) {
//			old_state = hallState; // Only update old_state if valid
//		}
//	}
//}

void Disable_Switches()
{
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;

	HAL_TIM_PWM_Stop(&htim1, PHASE_A);  // A_HIGH A8
	HAL_TIM_PWM_Stop(&htim1, PHASE_B);  // B_HIGH A9
	HAL_TIM_PWM_Stop(&htim1, PHASE_C);  // C_HIGH A10

	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_A); // A_LOW A7
	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_B); // B_LOW B0
	HAL_TIMEx_PWMN_Stop(&htim1, PHASE_C); // C_LOW B1
}

uint16_t Throttle_mapping(uint16_t Value, uint16_t ADC_Min, uint16_t ADC_Max, uint16_t Min_PWM, uint16_t Max_PWM)
{
	if(Value > ADC_Max)
	{
		Value = ADC_Max;
	}
	else if (Value < ADC_Min)
	{
		Value = ADC_Min;
	}
	return (uint16_t)((Value - ADC_Min) * (Max_PWM - Min_PWM) / (ADC_Max - ADC_Min) + Min_PWM);
}

void Step1(uint16_t duty)
{
	TIM1->CCR1 = duty; // A
	TIM1->CCR2 = LOW;  // B
	TIM1->CCR3 = 0;    // C

	HAL_TIM_PWM_Start(&htim1, PHASE_A); 	  // A_HIGH A8

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_B);    // B_LOW B0
}
void Step2(uint16_t duty)
{
	TIM1->CCR1 = duty;   // A
	TIM1->CCR2 = 0;      // B
	TIM1->CCR3 = LOW;    // C

	HAL_TIM_PWM_Start(&htim1, PHASE_A);       // A_HIGH A0

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_C);    // C_LOW B1
}
void Step3(uint16_t duty)
{
	TIM1->CCR1 = 0;      // A
	TIM1->CCR2 = duty;   // B
	TIM1->CCR3 = LOW;    // C

	HAL_TIM_PWM_Start(&htim1, PHASE_B);       // B_HIGH A1

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_C);    // C_LOW B1
}
void Step4(uint16_t duty)
{
	TIM1->CCR1 = LOW;      // A
	TIM1->CCR2 = duty;     // B
	TIM1->CCR3 = 0;        // C

	HAL_TIM_PWM_Start(&htim1, PHASE_B);  // B_HIGH A1

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_A);   // A_LOW A7

}
void Step5(uint16_t duty)
{
	TIM1->CCR1 = LOW;      // A
	TIM1->CCR2 = 0;        // B
	TIM1->CCR3 = duty;     // C

	HAL_TIM_PWM_Start(&htim1, PHASE_C);  // C_HIGH A2

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_A);     // A_LOW A7

}
void Step6(uint16_t duty)
{
	TIM1->CCR1 = 0;        // A
	TIM1->CCR2 = LOW;      // B
	TIM1->CCR3 = duty;     // C

	HAL_TIM_PWM_Start(&htim1, PHASE_C);  // C_HIGH A2

	HAL_TIMEx_PWMN_Start(&htim1, PHASE_B);     // B_LOW A6
}

// Assumes 72 MHz system clock
void deadTimeDelay(uint32_t us) {
	__HAL_TIM_SET_COUNTER(&htim2, 0); // Reset the timer counter
	while (__HAL_TIM_GET_COUNTER(&htim2) < us); // Wait until the counter reaches the delay value
}

void safeDisableOutputs(uint16_t DeadTime)
{
	Disable_Switches();
//	HAL_Delay(500);
	deadTimeDelay(DeadTime);
}

void Control_BLDC(uint8_t hall, uint16_t ADC_TO_PWM)
{
	  switch (hall)
	  {
	    case 0b101:  // Hall = 5 → Step1
	      Step1(ADC_TO_PWM);
	      break;
	    case 0b100:  // Step2
	      Step2(ADC_TO_PWM);
	      break;
	    case 0b110:  // Step3
	      Step3(ADC_TO_PWM);
	      break;
	    case 0b010:  // Step4
	      Step4(ADC_TO_PWM);
	      break;
	    case 0b011:  // Step5
	      Step5(ADC_TO_PWM);
	      break;
	    case 0b001:  // Step6
	      Step6(ADC_TO_PWM);
	      break;
	    default:
	    	 Disable_Switches();
	      break;
	  }
	  // Disable Switches for Dead Time to make sure that High & Low of each phase won't open together ( For Safety )
//	  safeDisableOutputs(DEADTIME_DELAY);
}

uint16_t ADC_Sampling(uint16_t ADC_Before)
{
	// Insert new sample into moving-average buffer
	adcSamples[adcIndex] = ADC_Before;
	adcIndex = (adcIndex + 1) % ADC_FILTER_SIZE;

	// Compute average of the buffer
	uint32_t sum = 0;
	for(uint16_t i = 0; i < ADC_FILTER_SIZE; i++) {
		sum += adcSamples[i];
	}
	return (uint16_t)(sum / ADC_FILTER_SIZE);
}

uint16_t StartADC(uint32_t TimeOut)
{
	// Start and Convert Analog Throttle Signal To Digital To be handled
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, TimeOut);
	return HAL_ADC_GetValue(&hadc1);
}

void CheckState(uint8_t HALL_State)
{
	  if( Recent_state != Prev_state || Recent_state == INVALID_STATE )
	  {
		  safeDisableOutputs(DEADTIME_DELAY); // Unified handling for state change or invalid state
		  if(Recent_state != INVALID_STATE)
		  {
			  Prev_state = HALL_State; // Only update old_state if valid
		  }
	  }
}

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
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  uint16_t ADCValue;
  uint16_t AdcMapp;
  uint16_t ADC_Sampled;
  uint8_t hall;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Start and read ADC (once)
	  ADCValue = StartADC(100);
	  // Check Values of ADC Before Sampling ( For Debugging )
	  ADC_BEFORE_SAMPLING = ADCValue;
	  //Sample / Filter The ADC Values to check the best one and reduce the noise
	  ADC_Sampled = ADC_Sampling(ADCValue);
	  // Check That Max & Min Values of sampled within Range of Throttle
	  if(ADC_Sampled < MIN_THROTTLE)
	  {
		  ADC_Sampled = MIN_THROTTLE;
	  }
	  if (ADC_Sampled > MAX_THROTTLE)
	  {
		  ADC_Sampled = MAX_THROTTLE;
	  }
	  // Check Values of ADC After Sampling ( For Debugging )
	  ADC_AFTER_SAMPLING = ADC_Sampled;
	  // Mapping the Throttle Value from Throttle Rang to Used PWM Range To Control Duty Cycle of PWM
	  AdcMapp = Throttle_mapping(ADC_Sampled, MIN_THROTTLE, MAX_THROTTLE, MIN_PWM, MAX_PWM);
	  // Check Values of Throttle Mapping ( For Debugging )
	  ADC_MAPPING = AdcMapp;
	  // Read Hall Sensor State ( Hall Sensor Feedback )
	  hall = Read_Hall_Sensors();
	  Recent_state = hall;
	  // Check Hall State
	  CheckState(hall);
	  /*
	   * ControlLing BLDC Motor By using two parameters ( Arguments )
	   * AdcMapp -> Control the speed of Motor and the switching of each MOSFETS
	   * hall -> FeedBack of Hall indicate the rotor position to take the right action ( Step ) to control motor
	   *
	   */
	  Control_BLDC(hall, AdcMapp);

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3599;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  HAL_TIM_Base_Start(&htim2); // Start Timer 2

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
