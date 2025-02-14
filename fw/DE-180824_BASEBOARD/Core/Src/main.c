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
enum {
    _SET_DIMMER_VALUE       = 0x01,
    _SET_DIMMER_RAMP        = 0x02,
    _SET_DIMMER_TEMPERATURE = 0x03,
    _DIMMER_RESET           = 0x04,  // softverski reset jednog kanala dimera 0x55 specijalna adresa za sve kanale na busu
    _GET_DIMMER_STATE       = 0x05,
    _DIMMER_RESTART         = 0x06   // softverski restart modula dimera i svih kanala na njemu
};

typedef enum {
    INIT = 0,
    RUN,
    ERR

} eSysState;

typedef struct {
    uint16_t value;
    uint16_t scale_hi;
    uint16_t scale_lo;
    uint8_t ramp;
    uint8_t temp;
    uint8_t temp_limit;
    uint8_t state;
    bool update;
    bool reset;
} sDimmerChanel;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIMRXBUF_SIZE 16
#define NUMBER_OF_CHANEL 4
#define MENU_TIMEOUT 10000 // 10 sekundi timeout
#define SHORT_PRESS_MIN 10  // 10 ms najmanje vrijeme kratkog pritisnutog tastera "klika"
#define SHORT_PRESS_MAX 200 // 200 ms najduže vrijeme jednog klika tastera
#define LONG_PRESS_MIN 1000 // 1 sekunda vrijeme nakon kojeg krece dim funkcija tastera
#define RESET_PRESS_MIN 5000 // 5 sekunda
#define LONG_PRESS_INTERVAL 0 // Brzina promjene kod dugog pritiska
#define INCREMENT_STEP 5   // Korak inkrementacije za dugi pritisak
#define TOGGLE_INTERVAL 500 // Period treptanja LED
#define ERROR_TOGGLE_INTERVAL 50 // Period treptanja LED kod greï¿½ke
#define BUTTON_PAUSE    200 // dodatni debouncing 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t dimrxbuf[DIMRXBUF_SIZE] = {0}, dimrxcnt = 0, dimrxbyte;
uint8_t rec, my_address;
TinyFrame tfapp;
bool init_tf = false;
static uint8_t ee_sta = 0;
bool restart = false;
eSysState sys_state = INIT;

static uint32_t delay_t = 20000000;
GPIO_TypeDef *LED_PORT[4] = {GPIOB, GPIOB, GPIOB, GPIOB};
uint16_t LED_PIN[4] = {GPIO_PIN_3, GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_14};
sDimmerChanel dimm_ch[4] = {0};
volatile uint8_t selected_channel = 3;
volatile uint8_t menu_activ = 0;
volatile uint32_t menu_timer = 0;
volatile uint8_t toggle_state = 0;
volatile uint8_t error_state = 0;
volatile uint32_t last_press_time = 0;
volatile uint32_t last_toggle_time = 0;
volatile uint32_t last_check_time = 0;
volatile uint32_t last_err_toggle_time = 0;
volatile uint32_t last_increment_time = 0;
uint8_t button_ok_prev = 0, button_up_prev = 0, button_down_prev = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_I2C1_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
TF_Result TYPE_Listener(TinyFrame *tf, TF_Msg *msg);
void save_value(uint8_t ch, uint16_t value);
uint8_t send(uint8_t *buf, uint8_t len);
uint8_t set_value(uint8_t ch);
uint8_t set_ramp(uint8_t ch);
uint8_t reset_ch(uint8_t ch);
uint8_t set_temp(uint8_t ch);
uint8_t get_state(uint8_t ch);
void refresh(void);
void handle_buttons(void) ;
void menu_logic(void);
void RS485_Init(void);
uint8_t received;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  staticka inline funkcija pauze 1~2ms za kašnjenje odgovora za stabilne repeatere
  *         u interruptu se ne može koristiti HAL_GetTick ni HAL_Delay tako da ova funkcija
  *         obezbjeduje kašnjenje oko 1 ms bez hal drivera gubljenjem vremena izvršavanjem 
  *         ciklusa koji ne rade ništa
  * @param
  * @retval
  */
static inline void delay_us(uint32_t us) {
    volatile uint32_t cycles = (HAL_RCC_GetHCLKFreq() / 20000000) * us;  // Smanjen faktor
    while (cycles--) {                                  
        __asm volatile("NOP");  // Izbegava optimizaciju
    }
}
/**
  * @brief  sacuvaj u eeprom skaliranje jednog dimera i njegovu rampu promjene 
  * @param
  * @retval 0 = spašeno / 1 = neuspjeh eeprom ne radi
  */
static uint8_t save_ch(uint8_t ch)
{
    uint8_t data[6] = {0};
    uint8_t addr = ch * 8;
    data[0] = dimm_ch[ch].ramp;
    data[1] = dimm_ch[ch].scale_hi >> 8;
    data[2] = dimm_ch[ch].scale_hi & 0xff;
    data[3] = dimm_ch[ch].scale_lo >> 8;
    data[4] = dimm_ch[ch].scale_lo & 0xff;
    data[5] = dimm_ch[ch].temp_limit;
    if (HAL_I2C_Mem_Write(&hi2c1, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, data, 6, 1000) != HAL_OK) ee_sta = 1; // upiši i ako nevalja upis oznaci ee_sta 
    else ee_sta = 0;
    HAL_Delay(5);
    return ee_sta;
}
/**
  * @brief  kopiraj iz eeproma rampu jednog dimera i njegovo skaliranje izlaza 
  * @param
  * @retval 0 = ucitano / 1 = neuspjeh eeprom ne radi
  */
static uint8_t load_ch(uint8_t ch)
{
    uint8_t data[6] = {0};
    uint8_t addr = ch * 8;

    if (HAL_I2C_Mem_Read(&hi2c1, 0xA0, addr, I2C_MEMADD_SIZE_8BIT, data, 6, 1000) != HAL_OK)
    {
        ++ee_sta;   // ako nevalja eeprom oznaci ee_sta i postavi defaultne vrijednosti
        dimm_ch[ch].ramp = 5; // 5 ms rampa
        dimm_ch[ch].scale_hi = 1000; // 1000 za najveci moguci raspon
        dimm_ch[ch].scale_lo = 0;   // 0 za najveci moguci raspon
        dimm_ch[ch].temp_limit = 90; // izbacuj grešku na 90°C mosfeta
    }
    else // samo ucitaj podatke ako valja eeprom
    {
        dimm_ch[ch].ramp = data[0];
        dimm_ch[ch].scale_hi = (uint16_t) (data[1] << 8) | data[2];
        dimm_ch[ch].scale_lo = (uint16_t) (data[3] << 8) | data[4];
        dimm_ch[ch].temp_limit = data[5];
        dimm_ch[ch].update = true; // šalji i na dimer izlaz 
        ee_sta = 0;
    }
    
    return ee_sta;
}
/**
  * @brief  Mapiranje vrednosti izmedu 1 i 99 u raspon 0~1000, ulazni 0 i 100 se odmah vracaju kao 0 i 1000
  * @param  uint8_t dimmer_value je ulazna vrijednost 0~100, 
            uint16_t lower_limit je minimalna izlazna vrijednost za ulaznu 1
            uint16_t upper_limit je maksimalna izlazna vrijednost za ulaznu 99
  * @retval 0~1000
  */
uint16_t map_dimmer_value(uint8_t dimmer_value, uint16_t lower_limit, uint16_t upper_limit)
{
    //dimmer_value = 0 ? izlaz = 0
    //dimmer_value = 100 ? izlaz = 1000
    //dimmer_value = 1 ? izlaz = lower_limit (npr. 200)
    //dimmer_value = 99 ? izlaz ï¿½ upper_limit (npr. 700)
    //Sve ostali vrednosti (2-98) se proporcionalno mapiraju izmedu lower_limit i upper_limit.
    if (dimmer_value == 0) return 0;  // Ako je podešavanje 0, izlaz je 0
    if (dimmer_value == 100) return 1000; // Ako je podešavanje 100, izlaz je 1000
    // Mapiranje vrednosti izmedu 1 i 99 u raspon lower_limit - upper_limit
    return (uint16_t)((((uint32_t)(dimmer_value - 1) * (upper_limit - lower_limit)) / 98) + lower_limit);
}
/**
  * @brief  Genericka funkcija za mapiranje ulazne vrednosti iz vece u manju sa limiterima za ulazne i izlazne vrijednosti
            ovdje se koristi da se vrijednost dimera koja je u rasponu 0~1000 skalira na vrijednost podešavanja 0~100%
  * @param  uint16_t in_value je ulazna vrijednost 
  * @retval uint8_t
  */
uint8_t map_hi2lo (uint16_t in_value, uint16_t in_min, uint16_t in_max, uint8_t out_min, uint8_t out_max)
{
    if (in_value <= in_min) return out_min;
    if (in_value >= in_max) return out_max;
    return (uint8_t)((((uint32_t)(in_value - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
}
/**
  * @brief  Genericka funkcija za mapiranje ulazne vrednosti iz manjih u vece sa limiterima za ulazne i izlazne vrijednosti
  * @param  uint8_t in_value je ulazna vrijednost 
  * @retval uint16_t
  */
uint16_t map_lo2hi(uint8_t in_value, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max)
{
    if (in_value <= in_min) return out_min;
    if (in_value >= in_max) return out_max;
    return (uint16_t)((((uint32_t)(in_value - in_min) * (out_max - out_min)) / (in_max - in_min)) + out_min);
}
/**
  * @brief  resetuj jedan dimera tako da se izlaz postavlja na 0 a rampa i temp.limit reinicijalizujui briše status greške
  * @param  rampa dimera i temperaturni limit kanala dimera
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t reset_ch(uint8_t ch)
{
    uint8_t res = 1; // odgovor
    uint8_t buf[4];

    buf[0] = ch + 1;
    buf[1] = _DIMMER_RESET;
    buf[2] = dimm_ch[ch].ramp;
    buf[3] = dimm_ch[ch].temp_limit;
    res = send(buf, 4);
    return res;
}
/**
  * @brief  restartuj jedan dimera bez inicijalizacije t.j. svi parametri su 0
  * @param  rampa dimera i temperaturni limit kanala dimera
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t restart_ch(uint8_t ch)
{
    uint8_t res = 1; // odgovor
    uint8_t buf[4];

    buf[0] = ch + 1;
    buf[1] = _DIMMER_RESTART;
    res = send(buf, 2);
    return res;
}
/**
  * @brief  podesi izlaz jednog dimera
  * @param  ch = kanal dimera
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t set_value(uint8_t ch)
{
    uint8_t res = 1; // odgovor
    uint8_t buf[4];

    buf[0] = ch + 1; // dimer moduli su adresirani od 1 pa dodaj offset
    buf[1] = _SET_DIMMER_VALUE;
    buf[2] = dimm_ch[ch].value >> 8;
    buf[3] = dimm_ch[ch].value & 0xff;
    res = send(buf, 4); // šalji sa odgovorom i 
    return res; //  informiši pozivaoca o izvršenju
}
/**
  * @brief  podesi temperaturni limit jednog dimera
  * @param  ch = kanal dimera, temp_limit
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t set_temp(uint8_t ch)
{
    uint8_t res = 1; // odgovor
    uint8_t buf[3];

    buf[0] = ch + 1;
    buf[1] = _SET_DIMMER_TEMPERATURE;
    buf[2] = dimm_ch[ch].temp_limit;

    res = send(buf, 3);
    return res;
}
/**
  * @brief  podesi rampu promjene izlaza jednog  dimera
  * @param  ch = kanal dimera, ramp
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t set_ramp(uint8_t ch)
{
    uint8_t res = 1; // odgovor
    uint8_t buf[3];

    buf[0] = ch + 1;
    buf[1] = _SET_DIMMER_RAMP;
    buf[2] = dimm_ch[ch].ramp;
    res = send(buf, 3);
    return res;
}
/**
  * @brief  trazi stanje jednog dimera
  * @param  ucitani su u strukturu iz dimera value, ramp, temp, temp_limit, state
  * @retval 0 = odgovor potvrdan / 1 = nije potvrdan
  */
uint8_t get_state(uint8_t ch)
{
    uint8_t buf[3];
    uint8_t ret = 5; // broj pokušaja
    uint8_t res = 1; // odgovor negativan
    uint8_t wait = 10; // vrijeme odgovora * 2ms

    buf[0] = ch + 1;
    buf[1] = _GET_DIMMER_STATE;

    do {
        dimrxcnt = 0;
        memset(dimrxbuf, 0, DIMRXBUF_SIZE);
        HAL_UART_Transmit(&huart2, buf, 2, 100);

        while(wait)
        {
            HAL_Delay(2);
            if((dimrxcnt == 8) && (dimrxbuf[0] == (ch + 1)) && (dimrxbuf[1] == _GET_DIMMER_STATE))
            { 
                // uzmi samo read only varijable iz modula
                // pokazalo se kao bolja opcija
                ret = res = wait = 0;
//                dimm_ch[ch].value        = ((uint16_t)dimrxbuf[2] << 8) | dimrxbuf[3];
//                dimm_ch[ch].ramp         = dimrxbuf[4];
                dimm_ch[ch].temp         = dimrxbuf[5];
//                dimm_ch[ch].temp_limit   = dimrxbuf[6];
                dimm_ch[ch].state        = dimrxbuf[7];
            } else --wait;
        }
    } while(ret--);
    return res;
}
/**
  * @brief  podesi adresu modula dimera na busu prema poziciji DIP switcha
  * @param  
  * @retval 
  */
void get_address(void)
{
    my_address = 0;
    if(HAL_GPIO_ReadPin(ADDRESS_0_GPIO_Port, ADDRESS_0_Pin) == GPIO_PIN_RESET) my_address |= 0x01;
    if(HAL_GPIO_ReadPin(ADDRESS_1_GPIO_Port, ADDRESS_1_Pin) == GPIO_PIN_RESET) my_address |= 0x02;
    if(HAL_GPIO_ReadPin(ADDRESS_2_GPIO_Port, ADDRESS_2_Pin) == GPIO_PIN_RESET) my_address |= 0x04;
    if(HAL_GPIO_ReadPin(ADDRESS_3_GPIO_Port, ADDRESS_3_Pin) == GPIO_PIN_RESET) my_address |= 0x08;
    if(HAL_GPIO_ReadPin(ADDRESS_4_GPIO_Port, ADDRESS_4_Pin) == GPIO_PIN_RESET) my_address |= 0x10;
    if(HAL_GPIO_ReadPin(ADDRESS_5_GPIO_Port, ADDRESS_5_Pin) == GPIO_PIN_RESET) my_address |= 0x20;
}
/**
  * @brief  restartuj sve dimere i ponovo reinicijalizuj iz eeprom-a 
            rampe, skaliranje i tem. limit  bez odgovora
  * @param  
  * @retval 
  */
void restart_all(void)
{
    uint8_t buf[2];
    buf[0] = 0x55;
    buf[1] = _DIMMER_RESTART;
    send(buf, 2);
    load_ch(0);
    load_ch(1);
    load_ch(2);
    load_ch(3);
}
/**
  * @brief  povremena provjera stanja dimera i podešavanje novih vrijednosti
  * @param  
  * @retval 
  */
void refresh_ch(void)
{
    uint8_t response = 0;
    static uint8_t chanel = 0;

    if ((HAL_GetTick() - last_check_time) >= 50) {
        last_check_time = HAL_GetTick();  // Azuriraj vreme poslednje promene
        if(dimm_ch[chanel].update == true)
        {
            response  = set_value(chanel);
            response += set_temp(chanel);
            response += set_ramp(chanel);
            if(response == 0) dimm_ch[chanel].update = false;
            else dimm_ch[chanel].state = ERR;
            save_ch(chanel);
        }
        if(get_state(chanel) != 0) dimm_ch[chanel].state = ERR;
        if(++chanel >= NUMBER_OF_CHANEL) chanel = 0;
    }
}
/**
  * @brief  pošalji paket dimeru i cekaj odgovor sa više pokušaja
  * @param  
  * @retval 0 = uspješno / 1 = nije uspješno
  */
uint8_t send(uint8_t *buf, uint8_t len)
{
    uint8_t ret = 5; // broj pokušaja
    uint8_t res = 1; // odgovor negativan
    uint8_t wait = 10; // vrijeme odgovora * 2ms

    do {
        dimrxcnt = 0;
        memset(dimrxbuf, 0, DIMRXBUF_SIZE);
        HAL_UART_Transmit(&huart2, buf, len, 100);

        while(wait)
        {
            HAL_Delay(2);
            if(dimrxbuf[0] == ACK)
            {
                ret = res = wait = 0;
            } else --wait;
        }
    } while(ret--);
    return res;
}
/**
  * @brief  podesi stanje led dioda na panelu prema vrijednosti izlaza
  * @param   
  * @retval 
  */
void refresh_led(void)
{
    for (uint8_t i = 0; i < 4; i++) {
        if(menu_activ || dimm_ch[i].state < ERR)
        {
            HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], (dimm_ch[i].value > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }
    }
}
/**
  * @brief  reaguj na pritisak tastera na prednjem panelu modula
  * @param   
  * @retval 
  */
void handle_buttons(void)
{
    uint32_t current_time = HAL_GetTick();
    uint8_t button_ok = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    uint8_t button_up = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
    uint8_t button_down = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);

    if (button_ok == GPIO_PIN_RESET) {
        if (!button_ok_prev) {
            button_ok_prev = 1; // lock press time load
            menu_activ = 1;
            toggle_state = 1;
            if(++selected_channel > 3) selected_channel = 0;
            menu_timer = current_time + MENU_TIMEOUT;
            last_press_time = current_time; // load first press time
            refresh_led();
            get_address();
            HAL_Delay(BUTTON_PAUSE);
        }
        else if (error_state && (current_time - last_press_time > RESET_PRESS_MIN)) {
            error_state = 0;
            restart_all(); // restartuj sve izlaze i ponovo ucitaj varijable iz eeproma
            HAL_Delay(200);
            for (uint8_t i = 0; i < 4; i++) {
                reset_ch(i);
                get_state(i);
                if(dimm_ch[i].state >= ERR) error_state = 1;
            }
        }

    }
    else if ((button_ok == GPIO_PIN_SET) && button_ok_prev)
    {
        button_ok_prev = 0;
        HAL_Delay(BUTTON_PAUSE);
        refresh_led();
    }

    if (menu_activ && (button_up == GPIO_PIN_RESET)) {
        if (!button_up_prev) {
            button_up_prev = 1; // lock press time load
            last_press_time = current_time; // load first press time
            HAL_Delay(BUTTON_PAUSE);
        }
        else if (current_time - last_press_time > LONG_PRESS_MIN && current_time - last_increment_time > LONG_PRESS_INTERVAL) {
            if (dimm_ch[selected_channel].value < 1000) {
                ++dimm_ch[selected_channel].value;
                set_value(selected_channel);
            }
            last_increment_time = current_time;
        }
        menu_timer = current_time + MENU_TIMEOUT;
    }
    else if ((button_up == GPIO_PIN_SET) && button_up_prev)
    {
        button_up_prev = 0; // release button press
        if((current_time - last_press_time) < LONG_PRESS_MIN)// check  for short press time window
        {
            dimm_ch[selected_channel].value = 1000;
            set_value(selected_channel);
        }
        HAL_Delay(BUTTON_PAUSE);
    }

    if (menu_activ && (button_down == GPIO_PIN_RESET)) {
        if (!button_down_prev) {
            button_down_prev = 1; // lock press time load
            last_press_time = current_time; // load first press time
            HAL_Delay(BUTTON_PAUSE);
        } else if (current_time - last_press_time > LONG_PRESS_MIN && current_time - last_increment_time > LONG_PRESS_INTERVAL) {
            if (dimm_ch[selected_channel].value > 0) {
                --dimm_ch[selected_channel].value;
                set_value(selected_channel);
            }
            last_increment_time = current_time;
        }
        menu_timer = current_time + MENU_TIMEOUT;
    }
    else if ((button_down == GPIO_PIN_SET) && button_down_prev)
    {
        button_down_prev = 0; // release button press
        if((current_time - last_press_time) < LONG_PRESS_MIN)// check  for short press time windows
        {
            dimm_ch[selected_channel].value = 0;
            set_value(selected_channel);
        }
        HAL_Delay(BUTTON_PAUSE);
    }
}
/**
  * @brief  logika korisnickog menija sa led diodama i tasterima na prednjem panelu modula
  * @param   
  * @retval 
  */
void menu_logic(void)
{
    static bool error_led_state = true;  // true = LED ukljucene, false = LED iskljucene
    uint32_t current_time = HAL_GetTick();
    if (current_time > menu_timer) {
        selected_channel = 3;
        menu_activ = 0;
        toggle_state = 0;
        refresh_led();
    }
    if (toggle_state && (current_time - last_toggle_time >= TOGGLE_INTERVAL)) {
        HAL_GPIO_TogglePin(LED_PORT[selected_channel], LED_PIN[selected_channel]);
        last_toggle_time = current_time;
    }
    if (!menu_activ && error_state && (current_time - last_err_toggle_time >= ERROR_TOGGLE_INTERVAL)) {
        // Ako je uredaj u error stanju, postavi sve LED u istom stanju
        for (uint8_t i = 0; i < 4; i++) {
            if (dimm_ch[i].state >= ERR) {
                // Ako je error_led_state true, ukljucuju se LED, inace iskljucuju
                HAL_GPIO_WritePin(LED_PORT[i], LED_PIN[i], error_led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
            }
        }
        last_err_toggle_time = current_time;
        error_led_state = !error_led_state;  // Prebacivanje stanja (ako se neko stanje promeni)
    }
}
/**
  * @brief  mali efekt sa LED diodama prednjeg panela kod ukljucenja
  * @param   
  * @retval 
  */
void start_effect(void)
{
    // 1. Punjenje iz sredine (LED_CH2 i LED_CH3 prvo)
    HAL_GPIO_WritePin(LED_CH2_GPIO_Port, LED_CH2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_CH3_GPIO_Port, LED_CH3_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(LED_CH1_GPIO_Port, LED_CH1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_CH4_GPIO_Port, LED_CH4_Pin, GPIO_PIN_SET);
    HAL_Delay(200);

    // Gašenje svih LED
    HAL_GPIO_WritePin(LED_CH1_GPIO_Port, LED_CH1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CH2_GPIO_Port, LED_CH2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CH3_GPIO_Port, LED_CH3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_CH4_GPIO_Port, LED_CH4_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

#ifdef USE_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif

    // 2. Brzi svjetlosni odjek (napred i nazad)
    GPIO_TypeDef *ports[] = {LED_CH1_GPIO_Port, LED_CH2_GPIO_Port, LED_CH3_GPIO_Port, LED_CH4_GPIO_Port};
    const uint16_t pins[] = {LED_CH1_Pin, LED_CH2_Pin, LED_CH3_Pin, LED_CH4_Pin};
    for(int i = 0; i < 4; i++)
    {
        // Naprijed (CH1 -> CH2 -> CH3 -> CH4)
        for (int i = 0; i < 4; i++)
        {
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        }

        // Nazad (CH4 -> CH3 -> CH2 -> CH1)
        for (int i = 3; i >= 0; i--)
        {
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_SET);
            HAL_Delay(50);
            HAL_GPIO_WritePin(ports[i], pins[i], GPIO_PIN_RESET);
        }        
    }


#ifdef USE_WATCHDOG
    HAL_IWDG_Refresh(&hiwdg);
#endif

    // 3. Završni treptaji (3 puta)
    for (int i = 0; i < 3; i++)
    {
        HAL_GPIO_WritePin(LED_CH1_GPIO_Port, LED_CH1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_CH2_GPIO_Port, LED_CH2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_CH3_GPIO_Port, LED_CH3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED_CH4_GPIO_Port, LED_CH4_Pin, GPIO_PIN_SET);
        HAL_Delay(100);

        HAL_GPIO_WritePin(LED_CH1_GPIO_Port, LED_CH1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_CH2_GPIO_Port, LED_CH2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_CH3_GPIO_Port, LED_CH3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_CH4_GPIO_Port, LED_CH4_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
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
    MX_CRC_Init();
    MX_I2C1_Init();
    MX_IWDG_Init();
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    RS485_Init();
    start_effect();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        if(restart == true) restart_all(), Error_Handler(); // neko je iskao restart
        handle_buttons();
        menu_logic();

        switch(sys_state)
        {
        case INIT:
            get_address();
            restart_all();
            error_state = 1; 
            sys_state = RUN;
            break;
        case RUN:
            refresh_ch();
            break;
        case ERR:
        default:
            break;
        }
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
#ifdef USE_WATCHDOG
        HAL_IWDG_Refresh(&hiwdg);
#endif
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
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

    /* USER CODE BEGIN CRC_Init 0 */

    /* USER CODE END CRC_Init 0 */

    /* USER CODE BEGIN CRC_Init 1 */

    /* USER CODE END CRC_Init 1 */
    hcrc.Instance = CRC;
    hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
    hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
    hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
    hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
    if (HAL_CRC_Init(&hcrc) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN CRC_Init 2 */

    /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

    /* USER CODE BEGIN I2C1_Init 0 */

    /* USER CODE END I2C1_Init 0 */

    /* USER CODE BEGIN I2C1_Init 1 */

    /* USER CODE END I2C1_Init 1 */
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x0010020A;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Analogue filter
    */
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Digital filter
    */
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN I2C1_Init 2 */

    /* USER CODE END I2C1_Init 2 */

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
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
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
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 9600;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */
    HAL_UART_Receive_IT(&huart2, &dimrxbyte, 1);
    /* USER CODE END USART2_Init 2 */

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
    HAL_GPIO_WritePin(GPIOB, LED_CH4_Pin|LED_LEARN_Pin|LED_CH1_Pin|LED_CH2_Pin
                      |LED_CH3_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pins : BUTTON_UP_Pin BUTTON_OK_Pin BUTTON_DOWN_Pin ADDRESS_2_Pin
                             ADDRESS_3_Pin ADDRESS_4_Pin ADDRESS_5_Pin ADDRESS_0_Pin
                             ADDRESS_1_Pin */
    GPIO_InitStruct.Pin = BUTTON_UP_Pin|BUTTON_OK_Pin|BUTTON_DOWN_Pin|ADDRESS_2_Pin
                          |ADDRESS_3_Pin|ADDRESS_4_Pin|ADDRESS_5_Pin|ADDRESS_0_Pin
                          |ADDRESS_1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_CH4_Pin LED_LEARN_Pin LED_CH1_Pin LED_CH2_Pin
                             LED_CH3_Pin */
    GPIO_InitStruct.Pin = LED_CH4_Pin|LED_LEARN_Pin|LED_CH1_Pin|LED_CH2_Pin
                          |LED_CH3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  Prati komunikaciju za DIMMER_SET tipu kanala za podešavanje nove vrijednosti izlaza dimera
  * @param
  * @retval
  */
TF_Result DIMMER_SET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t resp[4] = {0};
    uint16_t addr = (msg->data[0] << 8) + msg->data[1];
    uint8_t diml = (my_address * 4) - 3;
    uint8_t dimh =  my_address * 4;
    uint8_t ch = addr - diml; // selected dimmer

    if(addr && my_address && (addr >= diml) && (addr <= dimh))
    {
        dimm_ch[ch].value = map_dimmer_value(msg->data[2], dimm_ch[ch].scale_lo, dimm_ch[ch].scale_hi); // Skaliraj u 0~1000 okvir
        dimm_ch[ch].update = true; // Nova vrijednost na tržištu
        memcpy(resp, msg->data, 3); // Kopiraj ulazni bafer u odgovor
        resp[3] = ACK; // Dodaj potvrdni bajt
        msg->data = resp; // Naštimaj odgovor za pointer
        msg->len = 4; // Odgovor je 4 bajta
        TF_Respond(tf, msg); // Šalji nazad
    }
    return TF_STAY;;
}
/**
  * @brief  Prati komunikaciju za DIMMER_GET tipu kanala za zahtjev o statusu kanala dimera
  * @param
  * @retval
  */
TF_Result DIMMER_GET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t resp[11] = {0};
    uint16_t addr = (msg->data[0] << 8) + msg->data[1];
    uint8_t diml = (my_address * 4) - 3;
    uint8_t dimh =  my_address * 4;

    if(addr && my_address && (addr >= diml) && (addr <= dimh))
    {
        resp[0] = msg->data[0]; // Kopiraj adresu dimera u odgovoru zbog
        resp[1] = msg->data[1]; // provjere na uredaju koji je poslao zahtjev
        resp[2] = map_hi2lo(dimm_ch[addr - diml].value, dimm_ch[addr - diml].scale_hi, dimm_ch[addr - diml].scale_lo, 0, 100); // mapiraj 0~1000 na 0~100
        resp[3] = dimm_ch[addr - diml].ramp; // Trenutna rampa prirasta kanala dimera
        resp[4] = dimm_ch[addr - diml].temp; // Trenutna temperature kanala dimera
        resp[5] = dimm_ch[addr - diml].temp_limit; // Podešeni limit temperature
        resp[6] = dimm_ch[addr - diml].scale_hi >> 8; // Skalirna vrijednost maksimuma gornji bajt
        resp[7] = dimm_ch[addr - diml].scale_hi & 0xff; // Skalirna vrijednost maksimuma donji bajt
        resp[8] = dimm_ch[addr - diml].scale_lo >> 8; // Skalirna vrijednost minimuma gornji bajt
        resp[9] = dimm_ch[addr - diml].scale_lo & 0xff; // Skalirna vrijednost maksimuma donji bajt
        resp[10] = dimm_ch[addr - diml].state; // Trenutno stanje kanala  dimera 0 = init, 1 = u radu, 2 = greška temperaturna zaštita
        msg->data = resp; // Podesi pointer na bafer odgovora
        msg->len = 11; // Odgovor je 11 bajta
        TF_Respond(tf, msg); // Šalji odgovor
    }
    return TF_STAY;
}
/**
  * @brief  Prati komunikaciju za DIMMER_RESET tipu kanala za softverski restart jednog kanala dimera
  * @param
  * @retval
  */
TF_Result DIMMER_RESET_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t resp[3] = {0};
    uint16_t addr = (uint16_t)(msg->data[0] << 8) | msg->data[1];
    uint8_t diml = (my_address * 4) - 3;
    uint8_t dimh =  my_address * 4;
    uint8_t ch = addr - diml; // selected dimmer

    if(addr && my_address && (addr >= diml) && (addr <= dimh))
    {
        resp[0] = msg->data[0];
        resp[1] = msg->data[1];
        resp[2] = (reset_ch(ch) == 0) ? ACK:NAK;
        msg->data = resp; // Naštimaj odgovor za pointer
        msg->len = 3; // Odgovor je 4 bajta
        TF_Respond(tf, msg); // Šalji nazad
    }
    else if(my_address && (addr == 0x55AA))
    {
        for(int i = 0; i < NUMBER_OF_CHANEL; i++)
        {
            reset_ch(i);
        }
    }
    return TF_STAY;;
}
/**
  * @brief  Prati komunikaciju na DIMMER_SETUP tipu kanala za podešavanje novih postavki dimera
  * @param
  * @retval
  */
TF_Result DIMMER_SETUP_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t resp[9] = {0};
    uint16_t addr = (msg->data[0] << 8) + msg->data[1];
    uint8_t diml = (my_address * 4) - 3;
    uint8_t dimh =  my_address * 4;
    uint8_t ch = addr - diml; // selected dimmer

    if(addr && my_address && (addr >= diml) && (addr <= dimh))
    {
        dimm_ch[ch].ramp = msg->data[2]; // Podei novu rampu prirasta avrijednosti dimera
        dimm_ch[ch].temp_limit = msg->data[3]; // Podesi novi temperaturni limit
        dimm_ch[ch].scale_hi = ((uint16_t) msg->data[4] << 8)|msg->data[5]; // Nova vrijednost skaliranja maksimum
        dimm_ch[ch].scale_lo = ((uint16_t) msg->data[6] << 8)|msg->data[7]; // Nova vrijednost skaliranja minimum
        dimm_ch[ch].update = true; // Nova vrijednost za dimmer kanal
        memcpy(resp, msg->data, 8);  // Kopiraj ulazni bafer u odgovor
        resp[8] = ACK; // Dodaj potvrdni bajt
        msg->data = resp; // Naštimaj odgovor za pointer
        msg->len = 9; // Odgovor je 9 bajta
        TF_Respond(tf, msg); // Šalji nazad
    }
    return TF_STAY;;
}
/**
  * @brief  Prati komunikaciju za DIMMER_RESTART tipu kanala za softverski restart modula dimera ciji je adresirani kanal
  * @param
  * @retval
  */
TF_Result DIMMER_RESTART_Listener(TinyFrame *tf, TF_Msg *msg)
{
    uint8_t resp[3] = {0};
    uint16_t addr = (uint16_t)(msg->data[0] << 8) | msg->data[1];
    uint8_t diml = (my_address * 4) - 3;
    uint8_t dimh =  my_address * 4;

    if(addr && my_address && (addr >= diml) && (addr <= dimh))
    {
        resp[0] = msg->data[0];
        resp[1] = msg->data[1];
        resp[2] = ACK;
        msg->data = resp; // Naštimaj odgovor za pointer
        msg->len = 3; // Odgovor je 4 bajta
        TF_Respond(tf, msg); // Šalji nazad
        restart = true; // podesi flag da bi vratio odgovor prvo pa tek onda pokrenuo restart
    }
    else if(my_address && (addr == 0x55AA))
    {
        restart = true; // podesi flag da bi restartovo i module
    }
    return TF_STAY;;
}
/**
  * @brief
  * @param
  * @retval
  */
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
    delay_us(2000);
    HAL_UART_Transmit(&huart1,(uint8_t*)buff, len, RESP_TOUT);
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void RS485_Init(void)
{
    if(!init_tf)
    {
        init_tf = TF_InitStatic(&tfapp, TF_SLAVE);
        TF_AddTypeListener(&tfapp, DIMMER_SET, DIMMER_SET_Listener);
        TF_AddTypeListener(&tfapp, DIMMER_GET, DIMMER_GET_Listener);
        TF_AddTypeListener(&tfapp, DIMMER_SETUP, DIMMER_SETUP_Listener);
        TF_AddTypeListener(&tfapp, DIMMER_RESET, DIMMER_RESET_Listener);
        TF_AddTypeListener(&tfapp, DIMMER_RESTART, DIMMER_RESTART_Listener);
    }
    HAL_UART_Receive_IT(&huart1, &rec, 1);
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        TF_AcceptChar(&tfapp, rec);
        HAL_UART_Receive_IT(huart, &rec, 1);
    }
    else if (huart->Instance == USART2)
    {
        dimrxbuf[dimrxcnt++] = dimrxbyte;
        if(dimrxcnt >= sizeof(dimrxbuf))
        {
            dimrxcnt = 0;
            memset(dimrxbuf, 0, sizeof(dimrxbuf));
        }
        HAL_UART_Receive_IT(huart, &dimrxbyte, 1);
    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
    }
    else if (huart->Instance == USART2)
    {

    }
}
/**
  * @brief
  * @param
  * @retval
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_AbortReceive(huart);

    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(huart, &rec, 1);
    }
    else if (huart->Instance == USART2)
    {
        dimrxcnt = 0;
        memset(dimrxbuf, 0, sizeof(dimrxbuf));
        HAL_UART_Receive_IT(huart, &dimrxbyte, 1);
    }
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
