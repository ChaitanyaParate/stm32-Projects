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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_START_ADDRESS 0x08008000
#define MAX_BIN_SIZE      39528

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
#define CHUNK_SIZE 256
uint8_t rx_buf[MAX_BIN_SIZE];
uint32_t rx_size = 0;
uint32_t offset = 0;
uint8_t block = 0;
uint32_t remaining;
uint8_t trigger[2];
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void jump_to_app(void) {
    uint32_t app_stack = *(volatile uint32_t*)APP_START_ADDRESS;
    uint32_t app_reset = *(volatile uint32_t*)0x08008004;
    void (*app_entry)(void) = (void*)app_reset;

    __disable_irq();
    HAL_RCC_DeInit();
    SCB->VTOR = APP_START_ADDRESS;
    __set_MSP(app_stack);
    if ((app_stack & 0x2FFE0000) == 0x20000000) {
    	__DSB(); __ISB();
        app_entry();
    } else {
        HAL_UART_Transmit(&huart1, (uint8_t*)"Invalid App\n", 12, HAL_MAX_DELAY);
        while(1);
    }
}

void write_to_flash(uint32_t addr, uint8_t* data, uint32_t len) {
    HAL_FLASH_Unlock();
    for (uint32_t i = 0; i < len; i += 8) {
        uint64_t dq = 0;
        uint32_t chunk = (len - i >= 8) ? 8 : len - i;
        memcpy(&dq, &data[i], chunk);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr + i, dq) != HAL_OK) {
            HAL_FLASH_Lock();
            Error_Handler();
        }
    }
    HAL_FLASH_Lock();
}



//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//	offset += CHUNK_SIZE;
//
//    if (offset < rx_size) {
//        remaining = rx_size - offset;
//        uint32_t next_chunk = (remaining >= CHUNK_SIZE) ? CHUNK_SIZE : remaining;
//
//        HAL_UART_Receive_DMA(&huart1, &rx_buf[offset], next_chunk);
//        block++;
//        HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, HAL_MAX_DELAY);
//
//    } else {
//        // Done receiving
//        HAL_UART_Transmit(&huart1, (uint8_t*)"\nFlashing...\n", 13, HAL_MAX_DELAY);
//
//        FLASH_EraseInitTypeDef eraseInit;
//        uint32_t sectorErr;
//        uint32_t pageStart = (APP_START_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
//        uint32_t numPages = (rx_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
//
//        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
//        eraseInit.Page = pageStart;
//        eraseInit.NbPages = numPages;
//        eraseInit.Banks = FLASH_BANK_1;
//
//        HAL_FLASH_Unlock();
//        if (HAL_FLASHEx_Erase(&eraseInit, &sectorErr) != HAL_OK)
//            Error_Handler();
//        HAL_FLASH_Lock();
//
//        write_to_flash(APP_START_ADDRESS, rx_buf, rx_size);
//
//        HAL_UART_Transmit(&huart1, (uint8_t*)"\nDone. Jumping\n", 16, HAL_MAX_DELAY);
//        //HAL_Delay(500);
//
//        block++;
//        jump_to_app();
//    }
//}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	HAL_UART_Transmit(&huart1, (uint8_t*)"\nFlashing...\n", 13, HAL_MAX_DELAY);

	FLASH_EraseInitTypeDef eraseInit;
	uint32_t sectorErr;
	eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
	eraseInit.Page = (APP_START_ADDRESS - FLASH_BASE) / FLASH_PAGE_SIZE;
	eraseInit.NbPages = (rx_size + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
	eraseInit.Banks = FLASH_BANK_1;

	HAL_FLASH_Unlock();
	if (HAL_FLASHEx_Erase(&eraseInit, &sectorErr) != HAL_OK)
	    Error_Handler();
	HAL_FLASH_Lock();

	write_to_flash(APP_START_ADDRESS, rx_buf, rx_size);

	HAL_UART_Transmit(&huart1, (uint8_t*)"\nDone. Jumping\n", 16, HAL_MAX_DELAY);
	jump_to_app();

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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart1, (uint8_t*)"Waiting ^U\n", 11, HAL_MAX_DELAY);
     while (1) {
         HAL_UART_Receive(&huart1, trigger, 2, HAL_MAX_DELAY);
         if (trigger[0] == '^' && trigger[1] == 'U') break;
     }

     HAL_UART_Transmit(&huart1, (uint8_t*)"Send size\n", 10, HAL_MAX_DELAY);

     // Receive exact binary size
     rx_size = 39528;
     //HAL_UART_Receive(&huart1, (uint8_t*)&rx_size, 4, HAL_MAX_DELAY);
     if (rx_size > MAX_BIN_SIZE) Error_Handler();

     HAL_UART_Transmit(&huart1, (uint8_t*)"Send BIN\n", 9, HAL_MAX_DELAY);

     offset = 0;
     while (offset < rx_size) {
         uint32_t remaining = rx_size - offset;
         uint32_t chunk = (remaining > CHUNK_SIZE) ? CHUNK_SIZE : remaining;

         if (HAL_UART_Receive(&huart1, &rx_buf[offset], chunk, HAL_MAX_DELAY) != HAL_OK) {
             Error_Handler();  // handle error
         }

         offset += chunk;

         // Optional progress indicator
         HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, HAL_MAX_DELAY);
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.BaudRate = 2400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
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
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
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
