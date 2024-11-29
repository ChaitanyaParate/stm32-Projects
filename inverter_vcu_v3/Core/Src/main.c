/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdbool.h>

#include "string.h"
#include "stdio.h"
#include "math.h"
#include "stdlib.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

enum ERROR_CODE{
	CAN_ERROR,
	TEMP_ERROR,
	PRESSURE_1_ERROR,
	IMD_ERROR,
	MVCU_ERROR
};

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

CAN_RxHeaderTypeDef RxHeader;
CAN_TxHeaderTypeDef TxHeader;
uint32_t TxMailbox;
uint8_t CAN_DATA_BUFF[8];
uint8_t CAN_DATA_Tx[8];
volatile uint8_t temp1;
volatile uint8_t temp1_arr[2];

int START_FLAG = 0;
uint8_t SYSTEM_STATE = 0;
int ERROR_CODE = '0';

HAL_StatusTypeDef ret;

uint16_t samples;
uint16_t * lookUp;
uint16_t num1 = 0, num2, num3;
uint16_t count = 0;
bool directionReversed = false;
uint8_t sw, frequency, amplitude;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
void startSPWM(float switchingFreq, float sinFreq, float amp, bool reversed);
void invCommand(uint8_t sw1, uint8_t amplitude1, uint8_t frequency1);
void stopSPWM();
void ERROR_FUNCTION(int error_code);
uint8_t getTemp(uint8_t a);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CAN_DATA_BUFF) != HAL_OK){
		ERROR_FUNCTION(CAN_ERROR);
	}
	invCommand(CAN_DATA_BUFF[0],CAN_DATA_BUFF[2] , CAN_DATA_BUFF[1]);
	/*switch(CAN_DATA_BUFF[0]){
		case 0: SYSTEM_STATE = 0;
				invCommand(0, 0, 0);
				break;

		case 1: SYSTEM_STATE = 1;
				uint8_t freq;
				switch(CAN_DATA_BUFF[1]){
					case 1: freq  = 10;
					case 2: freq = 15;
					case 3: freq = 20;
					case 4: freq = 25;
					case 5: freq = 30;
					case 6: freq = 35;
					case 7: freq = 40;
					case 8: freq = 45;
					case 9: freq = 50;
					default: freq = 0;
				}
				uint8_t amp = CAN_DATA_BUFF[2] / 10;
				invCommand(1, amp, freq);
				break;

		case 2: SYSTEM_STATE = 2;
				invCommand(0, 0, 0);
				break;

		default:invCommand(0, 0, 0);
				break;
	}*/

}


/*void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan){

	uint8_t imd_can[8];

	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &RxHeader, imd_can) != HAL_OK){
		ERROR_FUNCTION(CAN_ERROR);
	}
	if(RxHeader.StdId == 0x37){
		if(imd_can[2] != 0){
			ERROR_FUNCTION(IMD_ERROR);
		}
		else if(imd_can[3] != 0){
			ERROR_FUNCTION(IMD_ERROR);
		}
	}
	CAN_DATA_Tx[5] = '0';
	CAN_DATA_Tx[6] = '0';

}*/

uint8_t getTemp(uint8_t a){
	float adc_val;
 	float beta_val=4304.05;
	float Vin= 5.00;
	float Rs=20000.00;
	float R25=32762.00;

	ret = HAL_ADC_Start(&hadc1);
		if(ret != HAL_OK)
		{
			return -1;
		}
	  ret = HAL_ADC_PollForConversion(&hadc1, 50);
		if(ret != HAL_OK)
		{
			return -1;
		}
	  adc_val = HAL_ADC_GetValue(&hadc1); // get the adc value
	  float Vout = adc_val * (Vin/4095.00); //4095 = 12 bit resolution
	  float Rt = (Vout * Rs) / (Vin - Vout);
	  float temperature = (1/((1/298.15)+(log(Rt/R25)/beta_val))-272.5);

	//if(a == 1){
		//uint8_t temp1 = 28;
	    uint8_t temp1 = temperature;
		if(temp1>40){
			CAN_DATA_Tx[1] = 2;					//using 2 for temp1 error
		}
		temp1_arr[1] = temp1%10; 				//first digit(tens place)
 		temp1_arr[0] = temp1 - temp1_arr[1];	//second digit (one's place)
		temp1_arr[0] = temp1_arr[0]/10;

		temp1_arr[1] = temp1_arr[1] + '0';		//ascii value of resp dec value taken
		temp1_arr[0] = temp1_arr[0] + '0';

		return temp1;
	//}



}

void ERROR_FUNCTION(int error_code){
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
	invCommand(0, 0, 0);
	SYSTEM_STATE = 0;
	CAN_DATA_Tx[0] = SYSTEM_STATE;
	CAN_DATA_Tx[1] = error_code;
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_DATA_Tx, &TxMailbox);
	__disable_irq();
	while(1){
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_15);
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
		HAL_Delay(50);

		/*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);*/
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

    HAL_CAN_Start(&hcan);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);


    CAN_DATA_Tx[4] = ',';
    CAN_DATA_Tx[7] = ',';
    TxHeader.DLC = 8;
  	TxHeader.StdId = 0x302;
  	TxHeader.IDE = CAN_ID_STD;
  	TxHeader.RTR = CAN_RTR_DATA;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  /*if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)!= GPIO_PIN_SET){
	 		  ERROR_FUNCTION(MVCU_ERROR);
	 	  }*/
	  while(SYSTEM_STATE == 1){
		  /*if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)!= GPIO_PIN_SET){
		  	 		  ERROR_FUNCTION(MVCU_ERROR);
		  	 	  }*/
	  		  temp1 = getTemp(1);

	  		  CAN_DATA_Tx[0] = SYSTEM_STATE;
	  		  CAN_DATA_Tx[1] = ERROR_CODE;
	  		  CAN_DATA_Tx[2] = temp1_arr[0];
	  		  CAN_DATA_Tx[3] = temp1_arr[1];

	  		  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_DATA_Tx, &TxMailbox) != HAL_OK){
	  			  ERROR_FUNCTION(CAN_ERROR);
	  		  }
	  		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
	  		  HAL_Delay(150);
	  	  }
	  	  while(SYSTEM_STATE == 2){
	  		/*if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)!= GPIO_PIN_SET){
	  			 		  ERROR_FUNCTION(MVCU_ERROR);
	  			 	  }*/
	  		  temp1 = getTemp(1);

	  		  CAN_DATA_Tx[0] = SYSTEM_STATE;
	  		  CAN_DATA_Tx[1] = ERROR_CODE;
	  		  CAN_DATA_Tx[2] = temp1_arr[0];
	  		  CAN_DATA_Tx[3] = temp1_arr[1];

	  		  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, CAN_DATA_Tx, &TxMailbox) != HAL_OK){
	  			  ERROR_FUNCTION(CAN_ERROR);
	  		  }
	  		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	  		  HAL_Delay(750);
	  	  }
	  	  if(SYSTEM_STATE == 0){
	  		/*if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14)!= GPIO_PIN_SET){
	  			 		  ERROR_FUNCTION(MVCU_ERROR);
	  			 	  }*/

	  			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
	  	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	 /* for (float f = 10; f < 300; f+=10) {
		  startSPWM(f*100, f, (f / 450.0) + 0.25);
		  HAL_Delay(200);
	  }
	  for (float f = 300; f > 10; f-=10) {
	  		  startSPWM(f * 100, f, (f / 450.0) + 0.25);
	  		  HAL_Delay(200);
	  	  }*/

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  	  CAN_FilterTypeDef CAN_Filter;

    	CAN_Filter.FilterActivation = CAN_FILTER_ENABLE;
    	CAN_Filter.FilterBank  = 10;
    	CAN_Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  	//CAN_Filter.FilterIdHigh = 0x0000;
    	CAN_Filter.FilterIdHigh = 0x103<<5;
    	CAN_Filter.FilterIdLow = 0x0000;
  	//CAN_Filter.FilterMaskIdHigh = 0X0000;
    	CAN_Filter.FilterMaskIdHigh = 0x103<<5;
    	CAN_Filter.FilterMaskIdLow = 0x0000;
    	CAN_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
    	CAN_Filter.FilterScale = CAN_FILTERSCALE_32BIT;

  	if( HAL_CAN_ConfigFilter(&hcan,&CAN_Filter) != HAL_OK)
  	{
  		Error_Handler();
  	}




  	/*CAN_FilterTypeDef CAN_Filter_IMD;

  	CAN_Filter_IMD.FilterActivation = CAN_FILTER_ENABLE;
  	CAN_Filter_IMD.FilterBank  = 10;
  	CAN_Filter_IMD.FilterFIFOAssignment = CAN_RX_FIFO1;
	//CAN_Filter_IMD.FilterIdHigh = 0x0000;
  	CAN_Filter_IMD.FilterIdHigh = 0x037<<5;
  	CAN_Filter_IMD.FilterIdLow = 0x0000;
	//CAN_Filter_IMD.FilterMaskIdHigh = 0X0000;
  	CAN_Filter_IMD.FilterMaskIdHigh = 0x037<<5;
  	CAN_Filter_IMD.FilterMaskIdLow = 0x0000;
  	CAN_Filter_IMD.FilterMode = CAN_FILTERMODE_IDMASK;
  	CAN_Filter_IMD.FilterScale = CAN_FILTERSCALE_32BIT;

	if( HAL_CAN_ConfigFilter(&hcan,&CAN_Filter_IMD) != HAL_OK)
	{
		Error_Handler();
	}*/

  /* USER CODE END CAN_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sBreakDeadTimeConfig.DeadTime = 150;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  //TIM1->ARR = 14400;
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void invCommand(uint8_t sw1, uint8_t amplitude1, uint8_t frequency1) {

	sw = sw1;
	amplitude = amplitude1;
	frequency = (frequency1 * 5) + 5;

}
void startSPWM(float switchingFreq, float sinFreq, float amp, bool reversed) { //amp 0 to 1 and sinfrq > 0.01 to 10000 and switchingfreq from 100+
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);//max switching freq to sinfreq ratio is 25000
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
	directionReversed = reversed;

	if (sinFreq < 0.1) {
		TIM1-> PSC = (0.4 / sinFreq) - 1.0;
		sinFreq = 0.4;
		switchingFreq *= TIM1->PSC;

	}
	else {
		TIM1-> PSC = 0;
	}
	samples = switchingFreq/sinFreq;
	while (samples > 25000) {
		switchingFreq-=1;
		samples = switchingFreq/sinFreq;
	}
	TIM1->ARR = HAL_RCC_GetPCLK2Freq() / (samples * sinFreq);
	samples /= 3;
	amp *= 1;
	count = 0;
	num1 = 0;
	free(lookUp);
	lookUp = (uint16_t *)malloc(samples * sizeof(samples));
	for (uint16_t i = 0; i < samples; i++) {
		lookUp[i] = (sin(2 * 3.14 * ((double)i/(double)samples)) * (TIM1->ARR / 2.05)) * amp  + (TIM1->ARR / 2);
	}
	  __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE );
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
}
void stopSPWM() {
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
	__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_UPDATE);
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
