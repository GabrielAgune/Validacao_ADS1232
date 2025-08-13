/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (v5 - Filtro Reajustado, Guarda Removida)
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
const float stability_slope_threshold = 20.0f; // Limiar de velocidade para estabilização
const uint32_t time_to_lock_ms = 1000;      // Tempo de espera para travar
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

// --- Variáveis do Filtro Preditivo Alpha-Beta (Valores Equilibrados) ---
float kalman_x = 0.0f;
float kalman_v = 0.0f;
const float alpha = 0.95f;  // Rápido, mas estável
const float beta  = 0.08f; // Suaviza a velocidade, mas permite reação

// --- Variáveis da máquina de estados ---
ScaleState_t scale_state = STATE_TRANSITION;
float locked_weight_grams = 0.0f;
uint32_t stabilizing_start_time = 0;

// --- Variáveis para o Auto-Zero Inteligente ---
int32_t absolute_zero_offset = 0;
const int32_t AUTO_ZERO_THRESHOLD_COUNTS = 1500;
const uint32_t auto_zero_time_ms = 3000;
uint32_t near_zero_start_time = 0;
uint8_t is_near_zero = 0;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DWIN_SendData(uint16_t vp_address, int32_t value);
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
  kalman_x = (float)absolute_zero_offset;
  kalman_v = 0.0f;
	
  printf("\r\nSistema pronto. Balanca vazia e tarada.\r\n");
  
  ADS1232_SetCalibrationFactor(6208.87f); 
	
  printf("Driver DWIN Inicializado na USART1.\r\n");
	
  int32_t raw_value;
  float weight_in_grams;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  
	  // --- LÓGICA DE LEITURA DO BOTÃO ---
      GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
      if (current_state != last_button_state) {
          last_debounce_time = HAL_GetTick();
      }
      if ((HAL_GetTick() - last_debounce_time) > 50) {
          if (current_state != button_state) {
              button_state = current_state;
              if (button_state == GPIO_PIN_RESET) {
                  int32_t new_offset = ADS1232_Tare();
                  kalman_x = (float)new_offset;
                  kalman_v = 0.0f;
                  scale_state = STATE_TRANSITION;
              }
          }
      }
      last_button_state = current_state;

      // --- LÓGICA PRINCIPAL COM FILTRO ALPHA-BETA ---
      if (HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_RESET)
      {
          raw_value = ADS1232_Read_Filtered();
          
          kalman_x += kalman_v;
          float residual = (float)raw_value - kalman_x;
          kalman_x += alpha * residual;
          kalman_v += beta * residual;

          // --- ATUALIZAÇÃO DA MÁQUINA DE ESTADOS ---
          if (fabsf(kalman_v) > stability_slope_threshold) {
              scale_state = STATE_TRANSITION;
          }

          switch(scale_state)
          {
              case STATE_TRANSITION:
                  weight_in_grams = ADS1232_ConvertToGrams((int32_t)kalman_x);
                  if (fabsf(kalman_v) <= stability_slope_threshold) {
                      scale_state = STATE_STABILIZING;
                      stabilizing_start_time = HAL_GetTick();
                  }
                  break;
              case STATE_STABILIZING:
                  weight_in_grams = ADS1232_ConvertToGrams((int32_t)kalman_x);
                  if (HAL_GetTick() - stabilizing_start_time > time_to_lock_ms) {
                      locked_weight_grams = weight_in_grams;
                      if (fabsf(locked_weight_grams) > 0.01f){
                         scale_state = STATE_LOCKED;
                      }
                  }
                  break;
              case STATE_LOCKED:
                  weight_in_grams = locked_weight_grams;
                  break;
          }
          
          // --- LÓGICA DE AUTO-ZERO E AJUSTE FINAL DO DISPLAY ---
          int32_t filtered_raw_value = (int32_t)kalman_x;
          if (abs(filtered_raw_value - absolute_zero_offset) < AUTO_ZERO_THRESHOLD_COUNTS) {
              if (is_near_zero == 0) {
                  is_near_zero = 1;
                  near_zero_start_time = HAL_GetTick();
              } else if (HAL_GetTick() - near_zero_start_time > auto_zero_time_ms) {
                  ADS1232_SetOffset(filtered_raw_value); 
                  absolute_zero_offset = filtered_raw_value; 
                  scale_state = STATE_TRANSITION; 
                  near_zero_start_time = HAL_GetTick();
              }
              
              if(scale_state != STATE_LOCKED) {
                  weight_in_grams = 0.0f;
              }
          } else {
              is_near_zero = 0;
          }
          
          printf("Peso: %.3f | Velocidade: %.2f\r\n", weight_in_grams, kalman_v);
          
          int32_t weight_for_dwin = (int32_t)(weight_in_grams * 1000.0f);
          DWIN_SendData(0x2000, weight_for_dwin);
          
          HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
      }
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