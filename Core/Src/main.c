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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

// --- VARIÁVEIS PARA O FILTRO EMA ADAPTATIVO ---
const float alpha_slow = 0.12f; // Alpha para alta estabilidade
const float alpha_fast = 0.5f;  // Alpha para resposta rápida
const int32_t change_threshold = 400; // Limiar para detectar mudança de peso

float smoothed_value_ema = 0.0f;
int first_reading = 1;

// --- VARIÁVEIS PARA O RASTREAMENTO AUTOMÁTICO DE ZERO ---
const float auto_zero_threshold_grams = 0.2f;
const uint32_t auto_zero_time_ms = 3000; // 3 segundos
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
    dwin_frame[0] = 0x5A;
    dwin_frame[1] = 0xA5;
    dwin_frame[2] = 0x07;
    dwin_frame[3] = 0x82;
    dwin_frame[4] = (uint8_t)(vp_address >> 8);
    dwin_frame[5] = (uint8_t)(vp_address);
    dwin_frame[6] = (uint8_t)(value >> 24);
    dwin_frame[7] = (uint8_t)(value >> 16);
    dwin_frame[8] = (uint8_t)(value >> 8);
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
  
  printf("Aguardando estabilizacao do ADC...\r\n");
  HAL_Delay(120); 

  ADS1232_Read();

  // Força o filtro a inicializar com o primeiro valor de tara
  smoothed_value_ema = (float)ADS1232_Tare(); 
  first_reading = 0;
	
  printf("\r\nSistema pronto. Balanca vazia e tarada.\r\n");
  
  ADS1232_SetCalibrationFactor(6208.51f); 
	
  printf("Driver DWIN Inicializado na USART1.\r\n");
	
  int32_t raw_value;
  float weight_in_grams;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- LÓGICA DE LEITURA DO BOTÃO (POLLING & DEBOUNCE) - CORRIGIDA ---
      GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

      // Esta linha é crucial para o debounce funcionar
      if (current_state != last_button_state) {
          last_debounce_time = HAL_GetTick();
      }

      if ((HAL_GetTick() - last_debounce_time) > 50) { 
          if (current_state != button_state) {
              button_state = current_state;
              if (button_state == GPIO_PIN_RESET) { // Botão foi pressionado
                  int32_t new_offset = ADS1232_Tare(); 
                  smoothed_value_ema = (float)new_offset;
                  first_reading = 0;
              }
          }
      }
      last_button_state = current_state;
      // --- FIM DA LÓGICA DO BOTÃO ---


      // --- LÓGICA DE LEITURA, FILTRO E ENVIO DIRETO ---
      if (HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_RESET)
      {
          raw_value = ADS1232_Read();

          if (first_reading) {
              smoothed_value_ema = (float)raw_value;
              first_reading = 0;
          } else {
              int32_t diff = abs((int32_t)raw_value - (int32_t)smoothed_value_ema);
              if (diff > change_threshold) {
                  smoothed_value_ema = (alpha_fast * (float)raw_value) + ((1.0f - alpha_fast) * smoothed_value_ema);
              } else {
                  smoothed_value_ema = (alpha_slow * (float)raw_value) + ((1.0f - alpha_slow) * smoothed_value_ema);
              }
          }
          
          weight_in_grams = ADS1232_ConvertToGrams((int32_t)smoothed_value_ema);
          
          if (fabsf(weight_in_grams) < 0.015f || weight_in_grams < 0.0f) {
              weight_in_grams = 0.0f;
          }
          
          if (fabsf(weight_in_grams) < auto_zero_threshold_grams) {
              if (is_near_zero == 0) {
                  is_near_zero = 1;
                  near_zero_start_time = HAL_GetTick();
              } else if (HAL_GetTick() - near_zero_start_time > auto_zero_time_ms) {
                  int32_t new_offset = (int32_t)smoothed_value_ema;
                  ADS1232_SetOffset(new_offset);
                  near_zero_start_time = HAL_GetTick();
              }
          } else {
              is_near_zero = 0;
          }
					
					printf("Peso: %.3f\n\r", weight_in_grams);
					
          int32_t weight_for_dwin = (int32_t)(weight_in_grams * 1000.0f);
          
          DWIN_SendData(0x2000, weight_for_dwin); // VP 0x2000 conforme seu código

          HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV4;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
  /* User can add his own implementation to report the HAL error return state */
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */