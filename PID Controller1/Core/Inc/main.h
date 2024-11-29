/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_c13_Pin GPIO_PIN_13
#define led_c13_GPIO_Port GPIOC
#define led_c15_Pin GPIO_PIN_15
#define led_c15_GPIO_Port GPIOC
#define OCD_Pin GPIO_PIN_1
#define OCD_GPIO_Port GPIOC
#define ADC2_curr1_Pin GPIO_PIN_2
#define ADC2_curr1_GPIO_Port GPIOC
#define ADC1_airgap_Pin GPIO_PIN_0
#define ADC1_airgap_GPIO_Port GPIOA
#define ADC2_temp4_Pin GPIO_PIN_4
#define ADC2_temp4_GPIO_Port GPIOA
#define ADC2_temp3_Pin GPIO_PIN_5
#define ADC2_temp3_GPIO_Port GPIOA
#define ADC2_temp1_Pin GPIO_PIN_6
#define ADC2_temp1_GPIO_Port GPIOA
#define ADC2_temp2_Pin GPIO_PIN_7
#define ADC2_temp2_GPIO_Port GPIOA
#define ADC3_VD_Pin GPIO_PIN_1
#define ADC3_VD_GPIO_Port GPIOB
#define ADC2_curr2_Pin GPIO_PIN_11
#define ADC2_curr2_GPIO_Port GPIOB
#define OCD2_Pin GPIO_PIN_12
#define OCD2_GPIO_Port GPIOB
#define ADC3_AMS_Pin GPIO_PIN_13
#define ADC3_AMS_GPIO_Port GPIOB
#define ADC4_airgap2_Pin GPIO_PIN_14
#define ADC4_airgap2_GPIO_Port GPIOB
#define led_b15_Pin GPIO_PIN_15
#define led_b15_GPIO_Port GPIOB
#define led_c6_Pin GPIO_PIN_6
#define led_c6_GPIO_Port GPIOC
#define led_a8_Pin GPIO_PIN_8
#define led_a8_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define MVCU_HBT_Pin GPIO_PIN_10
#define MVCU_HBT_GPIO_Port GPIOC
#define VCU_HBT_Pin GPIO_PIN_11
#define VCU_HBT_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
