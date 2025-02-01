/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
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
#define CURRENT_Pin GPIO_PIN_0
#define CURRENT_GPIO_Port GPIOA
#define NTC_Pin GPIO_PIN_1
#define NTC_GPIO_Port GPIOA
#define ZEROCROSS_Pin GPIO_PIN_4
#define ZEROCROSS_GPIO_Port GPIOA
#define ZEROCROSS_EXTI_IRQn EXTI4_15_IRQn
#define GATE_Pin GPIO_PIN_5
#define GATE_GPIO_Port GPIOA
#define A4_Pin GPIO_PIN_6
#define A4_GPIO_Port GPIOA
#define A3_Pin GPIO_PIN_7
#define A3_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_1
#define A2_GPIO_Port GPIOB
#define A1_Pin GPIO_PIN_9
#define A1_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_10
#define A0_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
