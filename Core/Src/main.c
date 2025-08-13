/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (v11 - Lógica de Auto-Zero Corrigida)
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
/* USER CODE BEGIN PTD */
typedef enum {
    STATE_TRANSITION,
    STATE_STABILIZING,
    STATE_LOCKED
} ScaleState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FILT_WIN_SIZE 32

const float STABILITY_SLOPE_THRESHOLD = 8.0f;     
const float STABILITY_SIGMA_THRESHOLD = 40.0f; 
const uint32_t TIME_TO_LOCK_MS = 1000;         

const float HYSTERESIS_GRAMS = 0.05f; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

int32_t readings_buffer[FILT_WIN_SIZE] = {0};
int buffer_index = 0;
uint8_t buffer_filled = 0;

long long sum_x = 0;
long long sum_x_sq = 0;
float regression_denominator = 0.0f;

ScaleState_t scale_state = STATE_TRANSITION;
float locked_weight_grams = 0.0f;
uint32_t stabilizing_start_time = 0;

int32_t absolute_zero_offset = 0;
const int32_t AUTO_ZERO_THRESHOLD_COUNTS = 1500;
const uint32_t auto_zero_time_ms = 3000;
uint32_t near_zero_start_time = 0;
uint8_t is_near_zero = 0;

/* USER CODE BEGIN PV */
float sigma_g = 0.0f;
float slope_g = 0.0f;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DWIN_SendData(uint16_t vp_address, int32_t value);
void Calculate_Statistics(float* p_avg_y, float* p_sigma, float* p_slope);
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  GPIO_PinState button_state = GPIO_PIN_SET;
  GPIO_PinState last_button_state = GPIO_PIN_SET;
  uint32_t last_debounce_time = 0;
	
  printf("\r\n--- Balanca de Precisao com ADS1232 e Display DWIN ---\r\n");

  ADS1232_Init();
  
  absolute_zero_offset = ADS1232_Tare();
  
  for(int i = 0; i < FILT_WIN_SIZE; i++) {
      readings_buffer[i] = absolute_zero_offset;
  }
	
  for(int i = 0; i < FILT_WIN_SIZE; i++) {
      sum_x += i;
      sum_x_sq += (long long)i * i;
  }
  regression_denominator = (float)((long long)FILT_WIN_SIZE * sum_x_sq - (sum_x * sum_x));
	
  printf("\r\nSistema pronto. Balanca vazia e tarada.\r\n");
  
  printf("Driver DWIN Inicializado na USART1.\r\n");
	
  float weight_in_grams;
  /* USER CODE END 2 */

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
                  for(int i = 0; i < FILT_WIN_SIZE; i++) {
                      readings_buffer[i] = absolute_zero_offset;
                  }
                  buffer_index = 0;
                  buffer_filled = 1; // O buffer está "cheio" com o novo valor de tara
                  scale_state = STATE_TRANSITION;
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
      
      float current_avg_grams = 0;
      uint8_t is_stable = 0;
      float average_y = 0.0f;

      // --- CÁLCULO ESTATÍSTICO ---
      if (buffer_filled) {
          Calculate_Statistics(&average_y, &sigma_g, &slope_g);
          current_avg_grams = ADS1232_ConvertToGrams((int32_t)average_y);
          if (sigma_g < STABILITY_SIGMA_THRESHOLD && fabsf(slope_g) < STABILITY_SLOPE_THRESHOLD) {
              is_stable = 1;
          }
      } else {
          current_avg_grams = 0.0f;
      }

      // --- MÁQUINA DE ESTADOS COM HISTERESE ---
      switch(scale_state)
      {
          case STATE_TRANSITION:
              weight_in_grams = current_avg_grams;
              if (is_stable) {
                  scale_state = STATE_STABILIZING;
                  stabilizing_start_time = HAL_GetTick();
              }
              break;
          case STATE_STABILIZING:
              weight_in_grams = current_avg_grams;
              if (is_stable) {
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
      
      // --- LÓGICA DE AUTO-ZERO SIMPLIFICADA ---
      if (buffer_filled && abs((int32_t)average_y - absolute_zero_offset) < AUTO_ZERO_THRESHOLD_COUNTS) {
          if (is_near_zero == 0) {
              is_near_zero = 1;
              near_zero_start_time = HAL_GetTick();
          } else if (HAL_GetTick() - near_zero_start_time > auto_zero_time_ms) {
              // A condição 'is_stable' foi removida para evitar o ciclo vicioso
              absolute_zero_offset = ADS1232_Tare();
              weight_in_grams = 0.0f;
              scale_state = STATE_TRANSITION;
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
      HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    /* USER CODE END 3 */
  }
}

// ... (Resto do ficheiro main.c, incluindo Calculate_Statistics, permanece igual) ...

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
void Calculate_Statistics(float* p_avg_y, float* p_sigma, float* p_slope)
{
    long long sum_y = 0;
    long long sum_xy = 0;
    float sum_sq_diff = 0;
    
    for (int i = 0; i < FILT_WIN_SIZE; i++) {
        sum_y += readings_buffer[i];
        sum_xy += (long long)i * readings_buffer[(buffer_index + i) % FILT_WIN_SIZE];
    }
    *p_avg_y = (float)sum_y / FILT_WIN_SIZE;

    for (int i = 0; i < FILT_WIN_SIZE; i++) {
        float diff = (float)readings_buffer[i] - (*p_avg_y);
        sum_sq_diff += diff * diff;
    }
    
    *p_sigma = sqrtf(sum_sq_diff / FILT_WIN_SIZE);
    
    if (regression_denominator != 0.0f) {
        *p_slope = ((float)((long long)FILT_WIN_SIZE * sum_xy - sum_x * sum_y)) / regression_denominator;
    } else {
        *p_slope = 0.0f;
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
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
      HAL_Delay(200);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
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