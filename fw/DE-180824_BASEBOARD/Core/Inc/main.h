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
#include <stdint.h>
#include <stdbool.h>
#include "TinyFrame.h"
#include "common.h"
#include "LuxNET.h"
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
#define BUTTON_UP_Pin GPIO_PIN_0
#define BUTTON_UP_GPIO_Port GPIOB
#define BUTTON_OK_Pin GPIO_PIN_1
#define BUTTON_OK_GPIO_Port GPIOB
#define BUTTON_DOWN_Pin GPIO_PIN_2
#define BUTTON_DOWN_GPIO_Port GPIOB
#define ADDRESS_2_Pin GPIO_PIN_10
#define ADDRESS_2_GPIO_Port GPIOB
#define ADDRESS_3_Pin GPIO_PIN_11
#define ADDRESS_3_GPIO_Port GPIOB
#define ADDRESS_4_Pin GPIO_PIN_12
#define ADDRESS_4_GPIO_Port GPIOB
#define ADDRESS_5_Pin GPIO_PIN_13
#define ADDRESS_5_GPIO_Port GPIOB
#define LED_CH4_Pin GPIO_PIN_14
#define LED_CH4_GPIO_Port GPIOB
#define LED_LEARN_Pin GPIO_PIN_15
#define LED_LEARN_GPIO_Port GPIOB
#define LED_CH1_Pin GPIO_PIN_3
#define LED_CH1_GPIO_Port GPIOB
#define LED_CH2_Pin GPIO_PIN_4
#define LED_CH2_GPIO_Port GPIOB
#define LED_CH3_Pin GPIO_PIN_5
#define LED_CH3_GPIO_Port GPIOB
#define ADDRESS_0_Pin GPIO_PIN_8
#define ADDRESS_0_GPIO_Port GPIOB
#define ADDRESS_1_Pin GPIO_PIN_9
#define ADDRESS_1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
