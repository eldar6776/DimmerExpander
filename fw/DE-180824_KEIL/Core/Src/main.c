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
#define UART_BUFFER_SIZE 8
#define DIMMER_COUNT 16
#define ACK 0x06
#define RETRY_COUNT 3
#define TIMEOUT 100 // ms

// Definicije komandi
enum {
    DIMM_SET_VALUE = 0x01,
    DIMM_SET_RAMP = 0x02,
    DIMM_SET_CURRENT = 0x03,
    DIMM_SET_TEMPERATURE = 0x04,
    DIMM_RESET = 0x05,
    DIMM_GET_STATE = 0x06
};

// Struktura za čuvanje stanja dimera
typedef struct {
    uint8_t value;
    uint8_t current_limit;
    uint8_t temp_limit;
    uint8_t state;
} Dimmer_t;

typedef enum {
    INIT = 0,
    STOP,
    RUN,
    ERR1,
    ERR2,
    ERR3,
    ERR4,
    ERR5,
    ERR6,
    ERR7,
    ERR8
} eSYSstate;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
void process_uart_command();
void ramp_update(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
volatile eSYSstate dimmer_state = INIT;
volatile uint8_t dimmer_address = 0, i;
volatile uint8_t dimmer_current_limit;
volatile uint8_t dimmer_temp_limit;
volatile uint8_t dimmer_ramp;
uint8_t uart_rx_buffer[UART_BUFFER_SIZE];
uint8_t rx_byte;
uint8_t rxcnt = 0;

volatile uint16_t temperature = 0;
volatile uint16_t current = 0;
bool error = false;
volatile uint16_t dimmer_new_value = 0;   // Ciljna vrednost
volatile uint32_t last_ramp_tick = 0;  // Pamti zadnje a�uriranje
volatile uint32_t ramp_step = 5;  // Brzina rampe (ms izmedu koraka)
volatile uint16_t dimmer_value = 500;  // Vrijednost (0-1000) odreduje ugao iskljucenja
volatile uint8_t mosfet_on = 0; // Oznaka da je MOSFET ukljucen
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ramp_update(void) {
    if ((HAL_GetTick() - last_ramp_tick) >= ramp_step) {  // Koristi podesivi inkrement
        last_ramp_tick = HAL_GetTick();  // A�uriraj vreme poslednje promene

        if (dimmer_value < dimmer_new_value) {
            dimmer_value++;  // Povecaj vrednost za 1
        } else if (dimmer_value > dimmer_new_value) {
            dimmer_value--;  // Smanji vrednost za 1
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
    MX_USART1_UART_Init();
    MX_ADC_Init();
    MX_IWDG_Init();
    MX_TIM14_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {

        switch(dimmer_state)
        {
        case INIT:
            dimmer_address = 0;
            HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_SET);
            if(HAL_GPIO_ReadPin(A0_GPIO_Port, A0_Pin) == GPIO_PIN_RESET) dimmer_address |= 0x01;
            if(HAL_GPIO_ReadPin(A1_GPIO_Port, A1_Pin) == GPIO_PIN_RESET) dimmer_address |= 0x02;
            if(HAL_GPIO_ReadPin(A2_GPIO_Port, A2_Pin) == GPIO_PIN_RESET) dimmer_address |= 0x04;
            if(HAL_GPIO_ReadPin(A3_GPIO_Port, A3_Pin) == GPIO_PIN_RESET) dimmer_address |= 0x08;
            dimmer_new_value = 0;
            dimmer_value = 0;
            temperature = 0;
            current = 0;
            dimmer_state = STOP;
            break;

        case STOP:
            dimmer_state = RUN;
            break;

        case RUN:
            ramp_update();  // Neprekidno a�urira value ka new_value
            break;

        default:
        case ERR1:
        case ERR2:
        case ERR3:
        case ERR4:
        case ERR5:
        case ERR6:
        case ERR7:
        case ERR8:
            dimmer_state = INIT;
            break;
        }

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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                                       |RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.HSI14CalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
    RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

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
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
    */
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
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
    /* USER CODE BEGIN ADC_Init 2 */

    /* USER CODE END ADC_Init 2 */

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
    hiwdg.Init.Window = 4095;
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
    htim14.Init.Prescaler = 47;
    htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim14.Init.Period = 99;
    htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM14_Init 2 */
    HAL_TIM_Base_Start_IT(&htim14);
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
    huart1.Init.BaudRate = 9600;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_SET);

    /*Configure GPIO pin : ZEROCROSS_Pin */
    GPIO_InitStruct.Pin = ZEROCROSS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(ZEROCROSS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : GATE_Pin */
    GPIO_InitStruct.Pin = GATE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GATE_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : A4_Pin A3_Pin A1_Pin A0_Pin */
    GPIO_InitStruct.Pin = A4_Pin|A3_Pin|A1_Pin|A0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : A2_Pin */
    GPIO_InitStruct.Pin = A2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(A2_GPIO_Port, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  stop timer interrupt, fire triac, and compare current angle in 100us increments
  * @param
  * @retval
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM14 && mosfet_on) {
        // Iskljuci MOSFET nakon izračunatog vremena
        HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_SET);  // GATE_PIN_SET isključuje MOSFET
        HAL_TIM_Base_Stop_IT(&htim14);
        mosfet_on = 0;
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ZEROCROSS_Pin) {
        if (dimmer_value > 0) {
            // Ukljuci MOSFET odmah nakon detekcije nule (invertovana logika za trailing edge)
            HAL_GPIO_WritePin(GATE_GPIO_Port, GATE_Pin, GPIO_PIN_RESET);  // GATE_PIN_RESET ukljucuje MOSFET
            mosfet_on = 1;

            // Izračunaj koliko treba čekati do isključenja (dimmer_value * 10 µs)
            uint16_t delay_ticks = dimmer_value * 10;

            // Postavi tajmer za isključenje MOSFET-a
            __HAL_TIM_SET_COUNTER(&htim14, 0);
            __HAL_TIM_SET_AUTORELOAD(&htim14, delay_ticks);
            HAL_TIM_Base_Start_IT(&htim14);
        }
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void process_uart_command() {
    uint8_t id = uart_rx_buffer[0];
    uint8_t command = uart_rx_buffer[1];
    uint8_t response[UART_BUFFER_SIZE] = {0};

    if (id >= DIMMER_COUNT && id != 0x55) return;

    switch (command) {
    case DIMM_SET_VALUE:
        if (id == dimmer_address) {
            dimmer_value = (uint16_t) uart_rx_buffer[2] * 10;
            response[0] = ACK;
        }
        break;
    case DIMM_SET_CURRENT:
        if (id == dimmer_address) {
            dimmer_current_limit = uart_rx_buffer[2];
            response[0] = ACK;
        }
        break;
    case DIMM_SET_TEMPERATURE:
        if (id == dimmer_address) {
            dimmer_temp_limit = uart_rx_buffer[2];
            response[0] = ACK;
        }
        break;
    case DIMM_RESET:
        if (id == 0x55) {
            dimmer_value = 0;
            dimmer_state = INIT;
        } else {
            dimmer_value = 0;
            dimmer_ramp = uart_rx_buffer[2];
            dimmer_current_limit = uart_rx_buffer[3];
            dimmer_temp_limit = uart_rx_buffer[4];
            dimmer_state = INIT;
        }
        response[0] = ACK;
        break;
    case DIMM_GET_STATE:
        if (id < DIMMER_COUNT) {
            response[0] = id;
            response[1] = DIMM_GET_STATE;
            response[2] = dimmer_value / 10;
            response[4] = dimmer_ramp;
            response[5] = dimmer_current_limit;
            response[6] = dimmer_temp_limit;
            response[7] = dimmer_state;
        }
        break;
    default:
        return;
    }
    HAL_UART_Transmit(&huart1, response, sizeof(response), HAL_MAX_DELAY);
}
/**
  * @brief packet 1DIMMCH,2CMD,3VALHI,4VALLO,5CRC
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) {
        uart_rx_buffer[rxcnt++] = rx_byte;
        
        // Provera dužine komande na osnovu drugog bajta (komande)
        uint8_t expected_length = 3;
        if (rxcnt > 1) {
            switch (uart_rx_buffer[1]) {
                case DIMM_RESET:
                    expected_length = 4;
                    break;
                case DIMM_GET_STATE:
                    expected_length = 2;
                    break;
                default:
                    expected_length = 3;
                    break;
            }
        }

        if (rxcnt >= expected_length) {
            process_uart_command();
            rxcnt = 0; // Resetujemo brojač nakon obrade
        }
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_PEFLAG(&huart1);
    __HAL_UART_CLEAR_FEFLAG(&huart1);
    __HAL_UART_CLEAR_NEFLAG(&huart1);
    __HAL_UART_CLEAR_IDLEFLAG(&huart1);
    __HAL_UART_CLEAR_OREFLAG(&huart1);
    HAL_UART_AbortReceive(&huart1);
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
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
    HAL_ADC_MspDeInit(&hadc);
    HAL_TIM_Base_MspDeInit(&htim14);
    HAL_UART_MspDeInit(&huart1);
    HAL_DeInit();
    while (1)
    {
        HAL_NVIC_SystemReset();
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