/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    uint16_t amplitude;
    uint32_t frequency;
    uint32_t phase_acc;
    uint32_t phase_inc;
    uint16_t current_value;
    uint8_t waveform;
    uint8_t sync_mode;
    uint32_t clock_period;
    uint8_t subdivision;
} Signal_Generator_t;

typedef enum {
    WAVE_SINE = 0, WAVE_SQUARE, WAVE_TRIANGLE, WAVE_SAWTOOTH, WAVE_RAMP_DOWN, WAVE_RANDOM, WAVE_COUNT
} WaveformType_t;

typedef enum { MODE_FREE = 0, MODE_SYNC } OperationMode_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MCP4922_CH_A            0x0000
#define MCP4922_CH_B            0x8000
#define MCP4922_GAINx1          0x2000
#define MCP4922_SHUTDOWN_OFF    0x1000

#define SAMPLE_RATE         10000
#define MUX_CHANNELS        8
#define PHASE_RESOLUTION    24
#define FREQ_SCALE          ((1UL << PHASE_RESOLUTION) / SAMPLE_RATE)

#define DEBOUNCE_TIME_MS     200
#define LONG_PRESS_TIME_MS   3000
#define SYNC_DEBOUNCE_MS    10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Signal_Generator_t signal_gen_A;
Signal_Generator_t signal_gen_B;
uint16_t adc_values[MUX_CHANNELS];
volatile uint8_t update_flag = 0;
uint32_t debug_counter = 0;
uint32_t spi_errors = 0;

const uint8_t subdivisions[6] = {1, 2, 4, 8, 16, 32};
const char* waveform_names[WAVE_COUNT] = {"Sine", "Square", "Triangle", "Sawtooth", "RampDn", "Random"};
const char* mode_names[2] = {"FREE", "SYNC"};

const uint16_t sine_table[256] = {
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
    3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3976, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3976, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1998, 1948, 1898, 1848, 1798, 1748, 1698, 1649, 1600, 1551, 1502, 1454, 1406, 1359, 1312,
    1265, 1219, 1173, 1128, 1083, 1039,  996,  953,  911,  870,  829,  789,  750,  711,  673,  637,
     601,  566,  531,  498,  466,  434,  404,  374,  346,  319,  292,  267,  243,  220,  198,  177,
     157,  138,  120,  104,   89,   75,   62,   51,   40,   31,   23,   16,   11,    7,    3,    2,
       1,    2,    3,    7,   11,   16,   23,   31,   40,   51,   62,   75,   89,  104,  120,  138,
     157,  177,  198,  220,  243,  267,  292,  319,  346,  374,  404,  434,  466,  498,  531,  566,
     601,  637,  673,  711,  750,  789,  829,  870,  911,  953,  996, 1039, 1083, 1128, 1173, 1219,
    1265, 1312, 1359, 1406, 1454, 1502, 1551, 1600, 1649, 1698, 1748, 1798, 1848, 1898, 1948, 1998
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int _write(int file, char *ptr, int len);
void MCP4922_Init(void);
HAL_StatusTypeDef MCP4922_WriteValue(uint8_t channel, uint16_t value);
uint16_t GenerateWaveform(Signal_Generator_t* gen);
uint16_t GenerateSineWave(Signal_Generator_t* gen);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}

void MCP4922_Init(void) {
    // Configurar CS como inactivo (alto)
    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_SET);

    // Inicializar los generadores de señal
    signal_gen_A.amplitude = 1023;
    signal_gen_A.frequency = 1000000;
    signal_gen_A.phase_acc = 0;
    signal_gen_A.phase_inc = (signal_gen_A.frequency * FREQ_SCALE) / 1000;
    signal_gen_A.current_value = 2048;
    signal_gen_A.waveform = WAVE_SINE;
    signal_gen_A.sync_mode = MODE_FREE;

    signal_gen_B.amplitude = 512;
    signal_gen_B.frequency = 2000000;
    signal_gen_B.phase_acc = 0;
    signal_gen_B.phase_inc = (signal_gen_B.frequency * FREQ_SCALE) / 1000;
    signal_gen_B.current_value = 2048;
    signal_gen_B.waveform = WAVE_SINE;
    signal_gen_B.sync_mode = MODE_FREE;

    // Escribir valores iniciales en ambos canales
    MCP4922_WriteValue(0, 2048);
    MCP4922_WriteValue(1, 2048);
}

HAL_StatusTypeDef MCP4922_WriteValue(uint8_t channel, uint16_t value) {
    uint16_t spi_data;
    uint8_t data_bytes[2];
    HAL_StatusTypeDef result;

    if(value > 4095) value = 4095;

    spi_data = (channel == 0) ? MCP4922_CH_A : MCP4922_CH_B;
    spi_data |= MCP4922_GAINx1;
    spi_data |= MCP4922_SHUTDOWN_OFF;
    spi_data |= (value & 0x0FFF);

    data_bytes[0] = (spi_data >> 8) & 0xFF;
    data_bytes[1] = spi_data & 0xFF;

    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi1, data_bytes, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_SET);

    if(result != HAL_OK) spi_errors++;
    return result;
}

uint16_t GenerateSineWave(Signal_Generator_t* gen) {
    gen->phase_acc += gen->phase_inc;
    uint8_t table_index = (gen->phase_acc >> (PHASE_RESOLUTION - 8)) & 0xFF;
    uint16_t sine_value = sine_table[table_index];
    int32_t centered_sine = sine_value - 2048;
    int32_t scaled_sine = (centered_sine * gen->amplitude) >> 11;
    int32_t final_value = 2048 + scaled_sine;
    if(final_value < 0) final_value = 0;
    if(final_value > 4095) final_value = 4095;
    return (uint16_t)final_value;
}

uint16_t GenerateWaveform(Signal_Generator_t* gen) {
    return GenerateSineWave(gen);  // Por ahora solo seno
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == TIM2) {
        update_flag = 1;
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
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  printf("\r\n=== MCP4922 Dual Channel LFO Generator ===\r\n");
  printf("Sistema iniciado correctamente\r\n");

  MCP4922_Init();
  printf("MCP4922 inicializado\r\n");

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if(update_flag) {
        update_flag = 0;
        signal_gen_A.current_value = GenerateWaveform(&signal_gen_A);
        signal_gen_B.current_value = GenerateWaveform(&signal_gen_B);
        MCP4922_WriteValue(0, signal_gen_A.current_value);
        MCP4922_WriteValue(1, signal_gen_B.current_value);
    }

    static uint32_t last_debug = 0;
    if(HAL_GetTick() - last_debug > 2000) {
        last_debug = HAL_GetTick();
        debug_counter++;
        printf("=== Debug #%lu ===\r\n", debug_counter);
        printf("Ch A: DAC=%d, Ch B: DAC=%d\r\n", signal_gen_A.current_value, signal_gen_B.current_value);
        HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    }

    HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
