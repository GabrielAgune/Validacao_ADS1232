/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (v9.1 - Refatorado para Evitar Bug do Compilador)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"
#include "ads1232_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef enum {
    STATE_TRANSITION = 0,
    STATE_STABILIZING,
    STATE_LOCKED
} ScaleState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILT_WIN_SIZE 64

const float STABILITY_SLOPE_THRESHOLD = 0.003f;     
const float STABILITY_SIGMA_THRESHOLD = 0.020f;    
const uint32_t TIME_TO_LOCK_MS = 1200;         

const float HYSTERESIS_GRAMS = 0.05f; 
/* USER CODE END PD */

const uint32_t AUTO_ZERO_COOLDOWN_MS  = 15000;  // período refratário após qualquer tara

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
const float    STEP_DETECT_THRESHOLD_G = 0.30f;   // detecta passo > 0,3 g
const uint32_t STEP_GUARD_MS           = 400;  
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
int32_t readings_buffer[FILT_WIN_SIZE] = {0};
int buffer_index = 0;
int buffer_filled = 0;

long long sum_y = 0;
long long sum_y_sq = 0;
long long sum_xy = 0;
long long sum_x = 0;
long long sum_x_sq = 0;
float regression_denominator = 0.0f;

ScaleState_t scale_state = STATE_TRANSITION;
float locked_weight_grams = 0.0f;
uint32_t stabilizing_start_time = 0;
uint32_t step_guard_until = 0;

int32_t absolute_zero_offset = 0;

const int32_t AUTO_ZERO_THRESHOLD_COUNTS = 1500;
const float   AUTO_ZERO_THRESHOLD_GRAMS  = 0.020f;

const uint32_t auto_zero_time_ms = 3000;
uint32_t near_zero_start_time = 0;
uint32_t last_tare_time = 0;
uint8_t is_near_zero = 0;

/* USER CODE BEGIN PV */
float sigma_g = 0.0f;
float slope_g = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DWIN_SendData(uint16_t vp_address, int32_t value);
// --- Nova função auxiliar para os cálculos ---
static void Calculate_Statistics_Counts(float* p_avg_y, float* p_sigma_counts, float* p_slope_counts);

static float CountsToGramsGain(int32_t raw_counts);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100); 
  return ch;
}

void DWIN_SendData(uint16_t vp_address, int32_t value)
{
    uint8_t dwin_frame[10];
    dwin_frame[0] = 0x5A; dwin_frame[1] = 0xA5; dwin_frame[2] = 0x07;
    dwin_frame[3] = 0x82; dwin_frame[4] = (uint8_t)(vp_address >> 8);
    dwin_frame[5] = (uint8_t)(vp_address); dwin_frame[6] = (uint8_t)(value >> 24);
    dwin_frame[7] = (uint8_t)(value >> 16); dwin_frame[8] = (uint8_t)(value >> 8);
    dwin_frame[9] = (uint8_t)(value);
    HAL_UART_Transmit(&huart1, dwin_frame, 10, 100);
}

static void Calculate_Statistics_Counts(float* p_avg_y, float* p_sigma_counts, float* p_slope_counts)
{
    float average_y = 0.0f;
    *p_avg_y = 0.0f; 
    *p_sigma_counts = 0.0f; 
    *p_slope_counts = 0.0f; 

    if (buffer_filled) {
        long long sum = 0;
        for (int i = 0; i < FILT_WIN_SIZE; i++) {
            sum += readings_buffer[i];
        }
        average_y = (float)((double)sum / (double)FILT_WIN_SIZE);

        long long sum_sq_diff = 0;
        for (int i = 0; i < FILT_WIN_SIZE; i++) {
            long long diff = readings_buffer[i] - (long long)average_y;
            sum_sq_diff += diff * diff;
        }
        float sigma_counts = sqrtf((float)((double)sum_sq_diff / (double)FILT_WIN_SIZE));

        long long sum_xy_local = 0;
        long long sum_x_local = 0;
        long long sum_y_local = 0;
        long long sum_x_sq_local = 0;

        for (int i = 0; i < FILT_WIN_SIZE; i++) {
            sum_xy_local += i * readings_buffer[i];
            sum_x_local  += i;
            sum_y_local  += readings_buffer[i];
            sum_x_sq_local += i * i;
        }

        long long N = FILT_WIN_SIZE;
        long long numerator = (N * sum_xy_local - sum_x_local * sum_y_local);
        long long denominator = (N * sum_x_sq_local - sum_x_local * sum_x_local);

        float slope_counts = 0.0f;
        if (denominator != 0) {
            slope_counts = (float)((double)numerator / (double)denominator);
        }

        *p_avg_y = average_y;
        *p_sigma_counts = sigma_counts;
        *p_slope_counts = slope_counts;
    }
}

static float CountsToGramsGain(int32_t raw_counts)
{
    /* Usa sua função de conversão, sem acessar a tabela diretamente */
    float g1 = ADS1232_ConvertToGrams(raw_counts);
    float g2 = ADS1232_ConvertToGrams(raw_counts + 64); // passo pequeno
    float dg = g2 - g1;
    if (dg <= 0.0f) {
        /* fallback razoável próximo da sua calibração (~6200 counts/g) */
        return 1.0f / 6200.0f;
    }
    return dg / 64.0f;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  printf("\r\n--- Balanca de Precisao com ADS1232 e Display DWIN ---\r\n");

  ADS1232_Init();
  HAL_Delay(100);

  absolute_zero_offset = ADS1232_Tare();
  for (int i = 0; i < FILT_WIN_SIZE; i++) {
      readings_buffer[i] = absolute_zero_offset;
  }
	
  buffer_index = 0;
  buffer_filled = 1;
  is_near_zero = 0;
  near_zero_start_time = HAL_GetTick();
  last_tare_time = HAL_GetTick();

  printf("\r\nSistema pronto. Balanca vazia e tarada.\r\n");
  printf("Driver DWIN Inicializado na USART1.\r\n");

  GPIO_PinState button_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
  GPIO_PinState last_button_state = button_state;
  uint32_t last_debounce_time = HAL_GetTick();

  float weight_in_grams = 0.0f;
	float prev_avg_for_step = 0.0f;
  uint8_t first_avg = 1;

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- LÓGICA DE LEITURA DO BOTÃO ---
      GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
      if (current_state != last_button_state) {
          last_debounce_time = HAL_GetTick();
      }
      if ((HAL_GetTick() - last_debounce_time) > 50) {
          if (current_state != button_state) {
              button_state = current_state;
              if (button_state == GPIO_PIN_RESET) {
                  absolute_zero_offset = ADS1232_Tare();
                  scale_state = STATE_TRANSITION;
                  for (int i = 0; i < FILT_WIN_SIZE; i++) {
                      readings_buffer[i] = absolute_zero_offset;
                  }
                  buffer_index  = 0;
                  buffer_filled = 1;
                  weight_in_grams = 0.0f;
                  is_near_zero = 0;
                  near_zero_start_time = HAL_GetTick();
                  last_tare_time = HAL_GetTick();
                  step_guard_until = HAL_GetTick(); // reseta proteção
              }
          }
      }
      last_button_state = current_state;

      // --- LEITURA E PREENCHIMENTO DO BUFFER ---
      int32_t current_reading = ADS1232_Read_Median_of_3();
      readings_buffer[buffer_index] = current_reading;
      buffer_index = (buffer_index + 1) % FILT_WIN_SIZE;
      if (!buffer_filled && buffer_index == 0) {
          buffer_filled = 1;
      }
      
      float average_y_counts = 0.0f;
      float sigma_counts = 0.0f;
      float slope_counts = 0.0f;

      float current_avg_grams = 0.0f;
      uint8_t is_stable = 0;

      if (buffer_filled) {
          Calculate_Statistics_Counts(&average_y_counts, &sigma_counts, &slope_counts);

          /* Converte média/sigma/slope para GRAMAS usando ganho local */
          float gpc = CountsToGramsGain((int32_t)average_y_counts); // g per count
          current_avg_grams = ADS1232_ConvertToGrams((int32_t)average_y_counts);
          sigma_g = sigma_counts * gpc;
          slope_g = slope_counts * gpc;
					
          if (sigma_g < STABILITY_SIGMA_THRESHOLD && fabsf(slope_g) < STABILITY_SLOPE_THRESHOLD) {
              is_stable = 1;
          }
      } else {
          current_avg_grams = 0.0f;
      }

			/* --- DETECÇÃO DE DEGRAU (inserção/retirada de peso) --- */
      if (buffer_filled) {
          if (first_avg) {
              prev_avg_for_step = current_avg_grams;
              first_avg = 0;
          }
          float step_delta = fabsf(current_avg_grams - prev_avg_for_step);
          if (step_delta > STEP_DETECT_THRESHOLD_G) {
              /* houve degrau: volta pra TRANSITION e abre guarda p/ anelamento */
              scale_state = STATE_TRANSITION;
              step_guard_until = HAL_GetTick() + STEP_GUARD_MS;
          }
          prev_avg_for_step = current_avg_grams;
      }


      // --- MÁQUINA DE ESTADOS COM HISTERESE ---
      switch(scale_state)
      {
          case STATE_TRANSITION:
              weight_in_grams = current_avg_grams;
              if (is_stable && (HAL_GetTick() > step_guard_until)) {
                  scale_state = STATE_STABILIZING;
                  stabilizing_start_time = HAL_GetTick();
              }
              break;

          case STATE_STABILIZING:
              weight_in_grams = current_avg_grams;
              if (is_stable && (HAL_GetTick() > step_guard_until)) {
                  if (HAL_GetTick() - stabilizing_start_time > TIME_TO_LOCK_MS) {
                      locked_weight_grams = current_avg_grams;
                      scale_state = STATE_LOCKED;
                  }
              } else {
                  scale_state = STATE_TRANSITION;
              }
              break;

          case STATE_LOCKED:
              weight_in_grams = locked_weight_grams;
              if (fabsf(current_avg_grams - locked_weight_grams) > HYSTERESIS_GRAMS) {
                  scale_state = STATE_TRANSITION;
              }
              break;
      }
      
      /* --- AUTO-ZERO: usa limiar em gramas + estabilidade + cooldown --- */
      uint8_t cooldown_ok = (HAL_GetTick() - last_tare_time) > AUTO_ZERO_COOLDOWN_MS;
      if (cooldown_ok && buffer_filled && fabsf(current_avg_grams) < AUTO_ZERO_THRESHOLD_GRAMS) {
          if (is_near_zero == 0) {
              is_near_zero = 1;
              near_zero_start_time = HAL_GetTick();
          } else if (is_stable && (HAL_GetTick() - near_zero_start_time > auto_zero_time_ms)) {
              absolute_zero_offset = ADS1232_Tare();
              for (int i = 0; i < FILT_WIN_SIZE; i++) {
                  readings_buffer[i] = absolute_zero_offset;
              }
              buffer_index  = 0;
              buffer_filled = 1;
              weight_in_grams = 0.0f;
              scale_state = STATE_TRANSITION;
              is_near_zero = 0;
              near_zero_start_time = HAL_GetTick();
              last_tare_time = HAL_GetTick();
              step_guard_until = HAL_GetTick();
          }
      } else {
          is_near_zero = 0;
      }
      if (is_near_zero && scale_state != STATE_LOCKED) {
          weight_in_grams = 0.0f;
      }
      
      // --- CONTROLE DE VELOCIDADE DO PRINTF ---
      static uint32_t last_print_time = 0;
      if (HAL_GetTick() - last_print_time > 100) { 
          printf("Peso: %.3f | Sigma: %.2f | Slope: %.2f\r\n", weight_in_grams, sigma_g, slope_g);
          DWIN_SendData(0x2000, (int32_t)(weight_in_grams * 1000.0f));
          last_print_time = HAL_GetTick();
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
  __HAL_FLASH_SET_LATENCY(FLASH_LATENCY_0);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
/* USER CODE END 4 */

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
