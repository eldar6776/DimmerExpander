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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "LuxNET.h"
#include "TinyFrame.h"
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
extern volatile bool ch1_enable, ch2_enable;
extern volatile uint16_t sync0, sync1; 
extern volatile uint8_t val0, val1, val2, val3, val4, val5, val6, val7;
extern volatile uint8_t val8, val9, val10, val11, val12, val13, val14, val15;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SYNC0_Pin GPIO_PIN_13
#define SYNC0_GPIO_Port GPIOC
#define SYNC0_EXTI_IRQn EXTI15_10_IRQn
#define SYNC1_Pin GPIO_PIN_14
#define SYNC1_GPIO_Port GPIOC
#define SYNC1_EXTI_IRQn EXTI15_10_IRQn
#define SELECT2_Pin GPIO_PIN_15
#define SELECT2_GPIO_Port GPIOC
#define SELECT3_Pin GPIO_PIN_0
#define SELECT3_GPIO_Port GPIOD
#define SELECT4_Pin GPIO_PIN_1
#define SELECT4_GPIO_Port GPIOD
#define DIN0_Pin GPIO_PIN_0
#define DIN0_GPIO_Port GPIOA
#define DIN1_Pin GPIO_PIN_1
#define DIN1_GPIO_Port GPIOA
#define DIN2_Pin GPIO_PIN_2
#define DIN2_GPIO_Port GPIOA
#define DIN3_Pin GPIO_PIN_3
#define DIN3_GPIO_Port GPIOA
#define DIN4_Pin GPIO_PIN_4
#define DIN4_GPIO_Port GPIOA
#define DIN5_Pin GPIO_PIN_5
#define DIN5_GPIO_Port GPIOA
#define DIN6_Pin GPIO_PIN_6
#define DIN6_GPIO_Port GPIOA
#define DIN7_Pin GPIO_PIN_7
#define DIN7_GPIO_Port GPIOA
#define DIMM0_Pin GPIO_PIN_0
#define DIMM0_GPIO_Port GPIOB
#define DIMM1_Pin GPIO_PIN_1
#define DIMM1_GPIO_Port GPIOB
#define DIMM2_Pin GPIO_PIN_2
#define DIMM2_GPIO_Port GPIOB
#define DIMM10_Pin GPIO_PIN_10
#define DIMM10_GPIO_Port GPIOB
#define DIMM11_Pin GPIO_PIN_11
#define DIMM11_GPIO_Port GPIOB
#define DIMM12_Pin GPIO_PIN_12
#define DIMM12_GPIO_Port GPIOB
#define DIMM13_Pin GPIO_PIN_13
#define DIMM13_GPIO_Port GPIOB
#define DIMM14_Pin GPIO_PIN_14
#define DIMM14_GPIO_Port GPIOB
#define DIMM15_Pin GPIO_PIN_15
#define DIMM15_GPIO_Port GPIOB
#define SELECT0_Pin GPIO_PIN_8
#define SELECT0_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_11
#define RS485_DE_GPIO_Port GPIOA
#define SELECT1_Pin GPIO_PIN_12
#define SELECT1_GPIO_Port GPIOA
#define STATUS_Pin GPIO_PIN_15
#define STATUS_GPIO_Port GPIOA
#define DIMM3_Pin GPIO_PIN_3
#define DIMM3_GPIO_Port GPIOB
#define DIMM4_Pin GPIO_PIN_4
#define DIMM4_GPIO_Port GPIOB
#define DIMM5_Pin GPIO_PIN_5
#define DIMM5_GPIO_Port GPIOB
#define DIMM6_Pin GPIO_PIN_6
#define DIMM6_GPIO_Port GPIOB
#define DIMM7_Pin GPIO_PIN_7
#define DIMM7_GPIO_Port GPIOB
#define DIMM8_Pin GPIO_PIN_8
#define DIMM8_GPIO_Port GPIOB
#define DIMM9_Pin GPIO_PIN_9
#define DIMM9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
