/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
typedef enum{
    READY = 0,
    RECEIVED,
}led_status_td;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
TinyFrame tfapp;
uint8_t sel = 0, rec;
uint16_t dimm_start, dimm_end;
bool init_tf = false;
led_status_td led_status = READY;
volatile uint16_t sync0, sync1;
volatile bool ch1_enable = false, ch2_enable = false;
volatile uint8_t dimm_new_val[16] = {0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff}; 
volatile uint8_t val0=0xff, val1=0xff, val2=0xff, val3=0xff, val4=0xff, val5=0xff, val6=0xff, val7=0xff;
volatile uint8_t val8=0xff, val9=0xff, val10=0xff, val11=0xff, val12=0xff, val13=0xff, val14=0xff, val15=0xff;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void RS485_Tick(void);
void RS485_Init(void);
void sys_init(void);
TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result TYPE_Listener(TinyFrame *tf, TF_Msg *msg);
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg);
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
uint32_t rcnt,rsta = 0,cnt = 0;
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
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    RS485_Init();
    sys_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    /* USER CODE BEGIN 3 */
    switch(led_status)
    {
        case READY:
            if(!rsta)
            {
                rcnt = HAL_GetTick();
                ++rsta;
            }
            else if((HAL_GetTick() - rcnt) >= 500U)
            {
                rsta = 0;
                if(HAL_GPIO_ReadPin(STATUS_GPIO_Port, STATUS_Pin) == GPIO_PIN_SET){
                    HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);
                } 
                else 
                {
                    HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_SET);
                }
            }
            break;

        case RECEIVED:
        default:    
            if(!rsta)
            {
                rcnt = HAL_GetTick();
                ++rsta;
            }
            else if((HAL_GetTick() - rcnt) >= 50U)
            {
                rsta = 0;
                if(HAL_GPIO_ReadPin(STATUS_GPIO_Port, STATUS_Pin) == GPIO_PIN_SET)
                {
                    HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_RESET);
                } 
                else
                {
                    HAL_GPIO_WritePin(STATUS_GPIO_Port, STATUS_Pin, GPIO_PIN_SET);
                }
                
                if(++cnt > 10)
                {
                    cnt = 0;
                    led_status = READY;
                }
            }
            break;
    }    
  }
#ifdef USE_WATCHDOG
  HAL_IWDG_Refresh(&hiwdg);
#endif
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */
#ifdef USE_WATCHDOG
  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_4;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */
#endif
  /* USER CODE END IWDG_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
    HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 6399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
    HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIMM0_Pin|DIMM1_Pin|DIMM2_Pin|DIMM10_Pin
                          |DIMM11_Pin|DIMM12_Pin|DIMM13_Pin|DIMM14_Pin
                          |DIMM15_Pin|DIMM3_Pin|DIMM4_Pin|DIMM5_Pin
                          |DIMM6_Pin|DIMM7_Pin|DIMM8_Pin|DIMM9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RS485_DE_Pin|STATUS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SYNC0_Pin SYNC1_Pin */
  GPIO_InitStruct.Pin = SYNC0_Pin|SYNC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SELECT2_Pin */
  GPIO_InitStruct.Pin = SELECT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SELECT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT3_Pin SELECT4_Pin */
  GPIO_InitStruct.Pin = SELECT3_Pin|SELECT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN0_Pin DIN1_Pin DIN2_Pin DIN3_Pin
                           DIN4_Pin DIN5_Pin DIN6_Pin DIN7_Pin */
  GPIO_InitStruct.Pin = DIN0_Pin|DIN1_Pin|DIN2_Pin|DIN3_Pin
                          |DIN4_Pin|DIN5_Pin|DIN6_Pin|DIN7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIMM0_Pin DIMM1_Pin DIMM2_Pin DIMM10_Pin
                           DIMM11_Pin DIMM12_Pin DIMM13_Pin DIMM14_Pin
                           DIMM15_Pin DIMM3_Pin DIMM4_Pin DIMM5_Pin
                           DIMM6_Pin DIMM7_Pin DIMM8_Pin DIMM9_Pin */
  GPIO_InitStruct.Pin = DIMM0_Pin|DIMM1_Pin|DIMM2_Pin|DIMM10_Pin
                          |DIMM11_Pin|DIMM12_Pin|DIMM13_Pin|DIMM14_Pin
                          |DIMM15_Pin|DIMM3_Pin|DIMM4_Pin|DIMM5_Pin
                          |DIMM6_Pin|DIMM7_Pin|DIMM8_Pin|DIMM9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT0_Pin SELECT1_Pin */
  GPIO_InitStruct.Pin = SELECT0_Pin|SELECT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RS485_DE_Pin */
  GPIO_InitStruct.Pin = RS485_DE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RS485_DE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : STATUS_Pin */
  GPIO_InitStruct.Pin = STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure peripheral I/O remapping */
  __HAL_AFIO_REMAP_PD01_ENABLE();

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint16_t tmp;
    if(GPIO_Pin == SYNC0_Pin)
    {
        __HAL_TIM_SET_COUNTER(&htim3, 0U);
        sync0 = 0;
        tmp = 0xFF00;
        GPIOB->ODR &= tmp;
        ch1_enable = true;
    }
    else if(GPIO_Pin == SYNC1_Pin)
    {
        __HAL_TIM_SET_COUNTER(&htim4, 0U);
        sync1 = 0;
        tmp = 0x00FF;
        GPIOB->ODR &= tmp;
        ch2_enable = true;
    }
}

void sys_init(void){
    if(HAL_GPIO_ReadPin(SELECT0_GPIO_Port, SELECT0_Pin) == GPIO_PIN_RESET) sel |= 0x01U;
    if(HAL_GPIO_ReadPin(SELECT1_GPIO_Port, SELECT1_Pin) == GPIO_PIN_RESET) sel |= 0x02U;
    if(HAL_GPIO_ReadPin(SELECT2_GPIO_Port, SELECT2_Pin) == GPIO_PIN_RESET) sel |= 0x04U;
    if(HAL_GPIO_ReadPin(SELECT3_GPIO_Port, SELECT3_Pin) == GPIO_PIN_RESET) sel |= 0x08U;
    if(HAL_GPIO_ReadPin(SELECT4_GPIO_Port, SELECT4_Pin) == GPIO_PIN_RESET) sel |= 0x10U;
    dimm_start = (16 * sel) + 1;
    dimm_end = dimm_start + 15;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result ID_Listener(TinyFrame *tf, TF_Msg *msg){
    return TF_CLOSE;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result GEN_Listener(TinyFrame *tf, TF_Msg *msg){    
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result STATUS_Listener(TinyFrame *tf, TF_Msg *msg){
    uint16_t adr = ((msg->data[0]<<8)|msg->data[1]);
    uint8_t ret[20];
    if ((adr >= dimm_start)&&(adr <= dimm_end)){
        ret[0] = dimm_start << 8;
        ret[1] = dimm_start & 0xFF;
        ret[2] = dimm_end << 8;
        ret[3] = dimm_end & 0xFF;
        ret[4] = ~val0;
        ret[5] = ~val1;
        ret[6] = ~val2;
        ret[7] = ~val3;
        ret[8] = ~val4;
        ret[9] = ~val5;
        ret[10] = ~val6;
        ret[11] = ~val7;
        ret[12] = ~val8;
        ret[13] = ~val9;
        ret[14] = ~val10;
        ret[15] = ~val11;
        ret[16] = ~val12;
        ret[17] = ~val13;
        ret[18] = ~val14;
        ret[19] = ~val15;
        msg->data = ret;
        msg->len = 20;
        led_status = RECEIVED;
        TF_Respond(tf, msg);   
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
TF_Result DIMMER_Listener(TinyFrame *tf, TF_Msg *msg){
    uint8_t res = 0, vpos = 2, apos = 3;   
    uint16_t adr = ((msg->data[0]<<8)|msg->data[1]);
    uint8_t ret[20];
    while((vpos < msg->len) && adr){
        if ((adr >= dimm_start) && (adr <= dimm_end)){
            while(adr > 16) adr -= 16;
            dimm_new_val[adr-1] = ~msg->data[vpos];
            res = 1;
        }
        adr = ((msg->data[apos]<<8)|msg->data[apos+1]);
        apos += 3;
        vpos += 3;
    }
    if(res){
        val0 = dimm_new_val[0];
        val1 = dimm_new_val[1];
        val2 = dimm_new_val[2];
        val3 = dimm_new_val[3];
        val4 = dimm_new_val[4];
        val5 = dimm_new_val[5];
        val6 = dimm_new_val[6];
        val7 = dimm_new_val[7];
        val8 = dimm_new_val[8];
        val9 = dimm_new_val[9];
        val10 = dimm_new_val[10];
        val11 = dimm_new_val[11];
        val12 = dimm_new_val[12];
        val13 = dimm_new_val[13];
        val14 = dimm_new_val[14];
        val15 = dimm_new_val[15];
        ret[0] = dimm_start << 8;
        ret[1] = dimm_start & 0xFF;
        ret[2] = dimm_end << 8;
        ret[3] = dimm_end & 0xFF;
        ret[4] = ~val0;
        ret[5] = ~val1;
        ret[6] = ~val2;
        ret[7] = ~val3;
        ret[8] = ~val4;
        ret[9] = ~val5;
        ret[10] = ~val6;
        ret[11] = ~val7;
        ret[12] = ~val8;
        ret[13] = ~val9;
        ret[14] = ~val10;
        ret[15] = ~val11;
        ret[16] = ~val12;
        ret[17] = ~val13;
        ret[18] = ~val14;
        ret[19] = ~val15;
        msg->data = ret;
        msg->len = 20;
        led_status = RECEIVED;
        TF_Respond(tf, msg);
    }
    return TF_STAY;
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void){
    if(!init_tf){
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, V_LEVEL, STATUS_Listener);
        TF_AddTypeListener(&tfapp, S_DIMMER, DIMMER_Listener);
    }
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Tick(void){
    if (init_tf == true) {
        TF_Tick(&tfapp);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len){
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart1,(uint8_t*)buff, len, RESP_TOUT);
    while(huart1.gState != HAL_UART_STATE_READY) continue;
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    TF_AcceptChar(&tfapp, rec);
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
    HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET);
	HAL_UART_Receive_IT(&huart1, &rec, 1);
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
    HAL_NVIC_SystemReset(); 
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
