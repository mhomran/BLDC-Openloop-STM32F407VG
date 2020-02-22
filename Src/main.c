/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "hall_sensor.h"
#include "main_open_loop.h"

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim5;

GPIO_TypeDef * hgpioa = GPIOA;

GPIO_TypeDef * hgpioe = GPIOE;

TIM_OC_InitTypeDef sConfigOC = {0};
;

unsigned char Hall_DIR_sequence[] = { 0x00,            
                                      HS_C|LS_B,       // Hall position 001
                                      HS_B|LS_A,       // Hall position 010
                                      HS_C|LS_A,       // Hall position 011
                                      HS_A|LS_C,       // Hall position 100
                                      HS_A|LS_B,       // Hall position 101
                                      HS_B|LS_C,       // Hall position 110
                                        0x00   };

// Motor and Commutation Variables
unsigned int Desired_PWM_DutyCycle, Current_PWM_DutyCycle, PWM_BucketStep, PWM_Update_Counter,ADC_Sample_Counter , Temp_DutyCycle;
unsigned char PreDriver_Sequence, Hall_IN, Motor_Status;
unsigned char Motor_status;

// ADC Variables
volatile int curADC, prevADC = 0;
volatile unsigned char SampleADC = false;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM5_Init(void);

void PWM_update (unsigned char Next_Hall_Sequence);
void Start_Motor(void);
void Stop_Motor(void);
void Start_ADC_Conversation(void);

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
  //Variable Initializations
  PWM_Update_Counter = 0x0;
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  //start the motor
  Start_Motor();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
			Start_ADC_Conversation();
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 200;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);
	
	TIM5->CR1 |= 1 << 2;
	TIM5->DIER |= 1;
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LSA_Pin|HSA_Pin|LSB_Pin|HSB_Pin 
                          |LSC_Pin|HSC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : HA_Pin HB_Pin HC_Pin */
  GPIO_InitStruct.Pin = HA_Pin|HB_Pin|HC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LSA_Pin HSA_Pin LSB_Pin HSB_Pin 
                           LSC_Pin HSC_Pin */
  GPIO_InitStruct.Pin = LSA_Pin|HSA_Pin|LSB_Pin|HSB_Pin 
                          |LSC_Pin|HSC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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

  /* USER CODE END Error_Handler_Debug */
}

void Start_Motor(void)
{
  Start_ADC_Conversation(); 
  
  Hall_IN = hgpioa->IDR;
	Hall_IN = (Hall_IN & 0xE0) >> 5;
  PreDriver_Sequence = Hall_DIR_sequence[Hall_IN];
  PWM_update(PreDriver_Sequence); 
  
	//start the timer
	Motor_Status = Running;
  
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
}

void PWM_update (unsigned char Next_Hall_Sequence)
{
	hgpioe->ODR &= ~(HSC_Pin | LSC_Pin | HSB_Pin | LSB_Pin | HSA_Pin | LSA_Pin);
  switch(Next_Hall_Sequence)
  {
  case HS_C|LS_B:            // Hall_IN DIR1_001 DIR0_110
  hgpioe->ODR |= HSC_Pin | LSB_Pin;
  break;
    
  case HS_B|LS_A:           // Hall_IN DIR1_010 DIR0_101
  hgpioe->ODR |= HSB_Pin | LSA_Pin;
  break;
    
  case HS_C|LS_A:            // Hall_IN DIR1_011 DIR0_100
  hgpioe->ODR |= HSC_Pin | LSA_Pin;
  break;
    
  case HS_A|LS_C:            // Hall_IN CCW_100 CW_011
  hgpioe->ODR |= HSA_Pin | LSC_Pin;
	break;
    
  case HS_A|LS_B:            // Hall_IN CCW_101 CW_010
  hgpioe->ODR |= HSA_Pin | LSB_Pin;
  break;
    
  case HS_B|LS_C:            // Hall_IN CCW_110 CW_001
  hgpioe->ODR |= HSB_Pin | LSC_Pin;
  break;
    
  default:
  break;
}
}

void Start_ADC_Conversation(void){
  //Trigger ADC Sampling
  HAL_ADC_Start(&hadc1);
  curADC =HAL_ADC_GetValue (&hadc1);
  if ((curADC > (prevADC + 10)) || (curADC < (prevADC - 10))){
    prevADC = curADC;
   
    Temp_DutyCycle = ((prevADC-200)/655.0) * TIMER_PWM_PERIOD; //to not get to the top value


    //TCCR1A |= 1 << COM1B1;
    
    if (prevADC < 200)
    {
      //TCCR1A &= ~(1 << COM1B1);
      //PORTB &= ~(1 << PINB2);
      Desired_PWM_DutyCycle = MIN_PWM_DUTYCYCLE;
    } 
    else if (prevADC >= 855){  //to prevent closing the switches fast
      //TCCR1A &= ~(1 << COM1B1);
      //PORTB |= 1 << PINB2;
    }
    else
    {
      Desired_PWM_DutyCycle = Temp_DutyCycle;
    }
  }
  
	// Update PWM duty cycle values

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
