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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <algorithm>
#include <xprintf.h>
#include <UARTOut.hpp>
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
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static constexpr uint32_t sk_BattChargeFinishTime = 500;       // バッテリ充電終了判定時間[10ms]

// ローパスフィルタの定数
// Cutoff   = 0.01s  = 100[Hz]
// Sampling = 0.1ms  = 0.0001[s]
// Const = (Cutoff / (Cutoff + Sampling)) = 0.909091 ≒ 0.909
static constexpr uint32_t sk_LPF_Precision          = 1000;
static constexpr uint32_t sk_SolarPanelVoltLPFConst = 909;
static constexpr uint32_t sk_BattVoltLPFConst       = 909;
static constexpr uint32_t sk_ADCRefVoltLPFConst     = 909;

static constexpr uint32_t sk_SolarPanelOpenVoltDiff = 10;               // 開放電圧測定時差分電圧[mV]
static constexpr uint32_t sk_SolarPanelOpenVoltSensStableTime = 5;      // 開放電圧測定安定回数

static constexpr uint32_t sk_ADC_Covnert_DataSize = 4;
static uint16_t s_ADC_ConvertData[sk_ADC_Covnert_DataSize];

static int s_10msTickCount = 0;
static bool s_10msHandlerInvoke = false;
static int s_100msTickCount = 0;
static bool s_100msHandlerInvoke = false;
static int s_1sTickCount = 0;
static bool s_1sHandlerInvoke = false;

static uint32_t s_LED_BlinkTime = 0;                        // LED 点滅時間[10ms]
static uint32_t s_LED_BlinkCnt = 0;
static int s_LED_Light = 0;

static uint32_t s_MPPT_PWMWidth = 0;

static uint32_t s_BattMillVolt = 0;
static uint32_t s_BattChargeFinishMillVolt = 1450;
static uint32_t s_BattChargeFinishCnt = 0;
static bool s_IsBattChargeFinish = false;

static uint32_t s_SolarPanelMillVolt = 0;
static uint32_t s_SolarPanel_MPPT_TargetMillVolt = 0;
static bool s_IsSolarPanelOpenVoltSensing = false;
static bool s_IsSolarPanelVoltDetect = false;
static uint32_t s_SolarPanelVoltSensCnt = 0;
static uint16_t s_SolarPanelVoltBuffBefore;
static uint16_t s_SolarPanelVoltBuff;

static uint32_t s_ADCRefMillVolt = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
static void PWM_SetValue( uint32_t value );
static void MPPT_SetPWMWidth();
static void MPPT_CheckSolarPanelMPPTVolt();
static void CheckBattChargeFinished();
static void Set_LEDBlinkTime( uint32_t time_ms );
static void Update_LED();
static void Show_LED();
static void Show_Voltage();
static uint32_t SenseSolarPanelOpenVolt();
static uint32_t SolarPanel_ADCValToMillVolt( uint32_t adcval );
static uint32_t Batt_ADCValToMillVolt( uint32_t adcval );
static uint32_t Get_SolarPanelPWMScale();
static uint32_t Get_SolarPanelVolt();

static void MsTimerHandler();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  // PWM Timer Start
  HAL_TIM_PWM_Start( &htim3, TIM_CHANNEL_4 );
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim14);

  HAL_ADCEx_Calibration_Start(&hadc);

  HAL_ADC_Start_DMA( &hadc,
                     (uint32_t *)s_ADC_ConvertData,
                     sk_ADC_Covnert_DataSize
  );

  // xprintf initialize (set callback func)
  xfunc_out = OutputFunc;

  xprintf( "Initialize complete!\n" );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if( s_10msHandlerInvoke ){
      //xprintf( "10ms handler invoke.\n" );
      s_10msHandlerInvoke = false;
      
      if( s_IsBattChargeFinish ){
        // PWMを停止
        s_MPPT_PWMWidth = 0;
        PWM_SetValue( s_MPPT_PWMWidth );
        Set_LEDBlinkTime( 100 );
      }
      else {
        MPPT_SetPWMWidth();
        CheckBattChargeFinished();
        Set_LEDBlinkTime( 0 );
      }

      Update_LED();
    }
    if( s_100msHandlerInvoke ){
      xprintf( "100ms handler invoke.\n" );
      s_100msHandlerInvoke = false;
      MPPT_CheckSolarPanelMPPTVolt();
    }
    if( s_1sHandlerInvoke ){
      xprintf( "1s handler invoke.\n" );
      s_1sHandlerInvoke = false;
      Show_Voltage();
    }

    Show_LED();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }

  asm( "wfi ");
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_TRGO;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 24-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (24*50)-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 24-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart1.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart1.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void PWM_SetValue( uint32_t duty )
{
    TIM_OC_InitTypeDef sConfigOC;
  
    if( htim3.Init.Period < duty ){
      duty = htim3.Init.Period;
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = duty;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

static void MPPT_SetPWMWidth()
{
  uint32_t current_millvolt = Get_SolarPanelVolt();

  if( current_millvolt > s_SolarPanel_MPPT_TargetMillVolt ){
    //uint32_t diff = current_millvolt - s_SolarPanel_MPPT_TargetMillVolt;
    //uint32_t pwm  = diff * 100 / 4167;
    s_MPPT_PWMWidth = s_MPPT_PWMWidth + 1 < Get_SolarPanelPWMScale() ? s_MPPT_PWMWidth + 1 : Get_SolarPanelPWMScale();
  }
  else {
    //uint32_t diff = s_SolarPanel_MPPT_TargetMillVolt - current_millvolt;
    //uint32_t pwm  = diff * 100 / 4167;
    s_MPPT_PWMWidth = s_MPPT_PWMWidth > 0 ? s_MPPT_PWMWidth - 1 : 0;
  }

  PWM_SetValue( s_MPPT_PWMWidth );
}

static void MPPT_CheckSolarPanelMPPTVolt()
{
  uint32_t open_millvolt = 0;
  uint32_t target_volt = 0;

  open_millvolt = SenseSolarPanelOpenVolt();
  target_volt = (open_millvolt * 83 / 100);     // 解放電圧の0.83倍の電圧を目標にする

  // 2.4V 以下だとマイコンが動作できないので、2.4V以上を目標にする
  if( target_volt < 2400 ){
    target_volt = 2400;
  }

  s_SolarPanel_MPPT_TargetMillVolt = target_volt;
}

static void CheckBattChargeFinished()
{
  if( s_BattMillVolt >= s_BattChargeFinishMillVolt ){
    ++s_BattChargeFinishCnt;
    if( s_BattChargeFinishCnt >= sk_BattChargeFinishTime ){
      s_IsBattChargeFinish = true;
    }
  }
  else {
    s_IsBattChargeFinish  = false;
    s_BattChargeFinishCnt = 0;
  }
}

static void Set_LEDBlinkTime( uint32_t time_ms )
{
  if( s_LED_BlinkTime != time_ms ){
    s_LED_BlinkTime = time_ms;
    s_LED_BlinkCnt  = 0;
  }
}

static void Update_LED()
{
  if( s_LED_BlinkTime > 0 ){
    ++s_LED_BlinkCnt;
    if( s_LED_BlinkCnt >= s_LED_BlinkTime ){
      s_LED_Light = s_LED_Light ? 0 : 1;
      s_LED_BlinkCnt = 0;
    }
  }
  else {
    s_LED_BlinkCnt = 0;
  }
}

static void Show_LED()
{
  if( s_LED_BlinkTime > 0 ){
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, static_cast<GPIO_PinState>(s_LED_Light));
  }
  else {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
}

static void Show_Voltage()
{
  if( s_IsBattChargeFinish ){
    xprintf( "Battery Charge Finished\n" );
  }
  else {
    xprintf( "Battery Charging [%d]\n", s_BattChargeFinishCnt );
  }
  xprintf( "SolarPanel Voltage  : %u\n", s_SolarPanelMillVolt );
  xprintf( "Batt Voltage        : %u\n", s_BattMillVolt );
  xprintf( "MPPT Target Voltage : %u\n", s_SolarPanel_MPPT_TargetMillVolt );
  xprintf( "ADC RefMillVolt     : %u\n", s_ADCRefMillVolt );

  for( int i = 0; i < 4; ++i ){
    xprintf( "ADC RawValue[%d]: %u\n", i, s_ADC_ConvertData[i] );
  }
}

static uint32_t SenseSolarPanelOpenVolt()
{
  s_IsSolarPanelOpenVoltSensing = true;
  s_SolarPanelVoltSensCnt = 0;
  PWM_SetValue( 0 );

  while( 1 ) {
    if( s_IsSolarPanelVoltDetect ){
      // 電圧の差分を取得
      uint32_t diff = std::max( s_SolarPanelVoltBuff, s_SolarPanelVoltBuffBefore ) - std::min( s_SolarPanelVoltBuff, s_SolarPanelVoltBuffBefore );

      // 開放電圧が安定するまで測定を繰り返す
      if( diff < sk_SolarPanelOpenVoltDiff ){
        s_SolarPanelVoltSensCnt++;
        if( s_SolarPanelVoltSensCnt > sk_SolarPanelOpenVoltSensStableTime ){
          break;
        }
      }
      else {
        s_SolarPanelVoltSensCnt = 0;
      }

      s_SolarPanelVoltBuffBefore = s_SolarPanelVoltBuff;
      s_IsSolarPanelVoltDetect = false;
    }
    asm( "wfi" );
  }

  PWM_SetValue( s_MPPT_PWMWidth );
  s_IsSolarPanelOpenVoltSensing = false;

  return SolarPanel_ADCValToMillVolt( s_SolarPanelVoltBuff );
}

static uint32_t SolarPanel_ADCValToMillVolt( uint32_t adcval )
{
  // 5V -> 1k / 1k の分圧で 降圧
  // adcRefVoltの2倍の電圧がフルスケール
  //static constexpr float sk_Scale = ((6.6f / 4096.0f) * 1000.0f);
  // return (sk_Scale * adcval) + 0.5f;

  return (adcval * (s_ADCRefMillVolt * 2)) / 4096;
}

static uint32_t Batt_ADCValToMillVolt( uint32_t adcval )
{
  // adcRefVolt フルスケール
  //static constexpr float sk_Scale = ((3.3f / 4096.0f) * 1000.0f);
  //return (sk_Scale * (float)adcval) + 0.5f;

  return (adcval * (s_ADCRefMillVolt)) / 4096;
}

static uint32_t ADCRef_ADCValToMillVolt( uint32_t adcval )
{
  static __IO uint16_t* vrefcal = (__IO uint16_t*)0x1FFFF7BA;
  return 3300 * (*vrefcal) / adcval;
}

static uint32_t Get_SolarPanelPWMScale()
{
  return htim3.Init.Period;
}

static uint32_t Get_SolarPanelVolt()
{
  return s_SolarPanelMillVolt;
}

static void MsTimerHandler()
{
  if( !s_IsBattChargeFinish && !s_IsSolarPanelOpenVoltSensing ){
    MPPT_SetPWMWidth();
  }

  ++s_10msTickCount;
  if( s_10msTickCount >= 10 ){
    s_10msTickCount = 0;
    s_10msHandlerInvoke = true;
  }

  ++s_100msTickCount;
  if( s_100msTickCount >= 100 ){
    s_100msTickCount = 0;
    s_100msHandlerInvoke = true;
  }

  ++s_1sTickCount;
  if( s_1sTickCount >= 1000 ){
    s_1sTickCount = 0;
    s_1sHandlerInvoke = true;
  }
}

// interrupt handlers
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if( htim->Instance == TIM14 ){
    MsTimerHandler();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  // user defined interrupt handler
  uint32_t millvolt = 0;

  // SolarPanel voltage sensing
  if( s_IsSolarPanelOpenVoltSensing && !s_IsSolarPanelVoltDetect ){
    s_SolarPanelVoltBuff = s_ADC_ConvertData[0];
    s_IsSolarPanelVoltDetect = true;
  }
  else {
    millvolt = SolarPanel_ADCValToMillVolt( s_ADC_ConvertData[0] );
    s_SolarPanelMillVolt = millvolt; //(((sk_LPF_Precision - sk_SolarPanelVoltLPFConst) * millvolt) + (sk_SolarPanelVoltLPFConst * s_SolarPanelMillVolt)) / sk_LPF_Precision;
    s_SolarPanelVoltSensCnt = 0;
  }

  // Batt voltage sensing
  millvolt = Batt_ADCValToMillVolt( s_ADC_ConvertData[1] );
  s_BattMillVolt = millvolt; //(((sk_LPF_Precision - sk_BattVoltLPFConst) * millvolt) + (sk_BattVoltLPFConst * s_BattMillVolt)) / sk_LPF_Precision;

  // ADCRef voltage sensing
  millvolt = ADCRef_ADCValToMillVolt( s_ADC_ConvertData[3] );
  s_ADCRefMillVolt = millvolt; //(((sk_LPF_Precision - sk_ADCRefVoltLPFConst) * millvolt) + (sk_ADCRefVoltLPFConst * s_ADCRefMillVolt)) / sk_LPF_Precision;
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
