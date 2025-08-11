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
#include <stdlib.h> // Necessário para a função abs()

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
const float alpha_slow = 0.02f; // Alpha para alta estabilidade (quando o peso está parado)
const float alpha_fast = 0.4f;  // Alpha para resposta rápida (quando o peso muda)
const int32_t change_threshold = 500; // Limiar (em unidades do ADC) para detectar uma mudança de peso.

float smoothed_value_ema = 0.0f;
int first_reading = 1; // Flag para inicializar o filtro na primeira leitura

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DWIN_SendData(uint16_t vp_address, int32_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Redireciona o printf para a UART2 (Debug).
  */
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100); 
  return ch;
}

/**
  * @brief  Envia um valor inteiro de 32 bits para um endereço VP específico no display DWIN.
  * @param  vp_address: O endereço do Variable Pointer (VP) no display.
  * @param  value: O valor de 32 bits a ser enviado.
  */
void DWIN_SendData(uint16_t vp_address, int32_t value)
{
    // Frame DWIN: 5A A5 | Len | 82 | VP_H | VP_L | VAL_B3 | VAL_B2 | VAL_B1 | VAL_B0
    uint8_t dwin_frame[10];
    
    dwin_frame[0] = 0x5A;
    dwin_frame[1] = 0xA5;
    dwin_frame[2] = 0x07; // Comprimento: 1(cmd) + 2(vp) + 4(valor) = 7 bytes
    dwin_frame[3] = 0x82; // Comando de escrita no VP
    
    // Endereço VP (Big Endian)
    dwin_frame[4] = (uint8_t)(vp_address >> 8);
    dwin_frame[5] = (uint8_t)(vp_address);
    
    // Valor 32-bit (Big Endian)
    dwin_frame[6] = (uint8_t)(value >> 24);
    dwin_frame[7] = (uint8_t)(value >> 16);
    dwin_frame[8] = (uint8_t)(value >> 8);
    dwin_frame[9] = (uint8_t)(value);

    // Envia o frame pela USART1, que deve estar conectada ao display
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
  MX_USART1_UART_Init(); // USART1 para o Display DWIN
  /* USER CODE BEGIN 2 */

  // Variáveis para controle do botão (polling e debounce)
  GPIO_PinState button_state = GPIO_PIN_SET;
  GPIO_PinState last_button_state = GPIO_PIN_SET;
  uint32_t last_debounce_time = 0;
	
  printf("\r\n--- Balanca de Precisao com ADS1232 e Display DWIN ---\r\n");

  ADS1232_Init();
  
  printf("Aguardando estabilizacao do ADC...\r\n");
  HAL_Delay(120); 

  ADS1232_Read(); // Leitura inicial para limpar buffer

  ADS1232_Tare(); 
	
  printf("\r\nSistema pronto. Balanca vazia e tarada.\r\n");
  
  // Fator de calibração previamente calculado
  ADS1232_SetCalibrationFactor(6210); 
	
  printf("Driver DWIN Inicializado na USART1.\r\n");
	
  int32_t raw_value;
  float weight_in_grams;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // --- LÓGICA DE LEITURA DO BOTÃO (POLLING & DEBOUNCE) ---
      GPIO_PinState current_state = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

      if (current_state != last_button_state) {
          last_debounce_time = HAL_GetTick();
      }

      if ((HAL_GetTick() - last_debounce_time) > 50) { // Debounce de 50ms
          if (current_state != button_state) {
              button_state = current_state;
              if (button_state == GPIO_PIN_RESET) { // Botão foi pressionado
                  ADS1232_Tare(); // Executa a tara imediatamente
                  first_reading = 1; // Reinicia o filtro EMA após a tara
              }
          }
      }
      last_button_state = current_state;
      // --- FIM DA LÓGICA DO BOTÃO ---


      // --- LÓGICA DE LEITURA, FILTRO E ENVIO DIRETO ---
      // Roda sempre que o ADC tem um novo dado pronto (DRDY/DOUT em nível baixo). [cite: 1127]
      if (HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_RESET)
      {
          // 1. LÊ o valor bruto do ADC
          raw_value = ADS1232_Read();

          // 2. APLICA o filtro adaptativo
          if (first_reading) {
              smoothed_value_ema = (float)raw_value;
              first_reading = 0;
          } else {
              int32_t diff = abs((int32_t)raw_value - (int32_t)smoothed_value_ema);

              if (diff > change_threshold) {
                  // Mudança grande detectada: usa o filtro RÁPIDO para convergir rapidamente
                  smoothed_value_ema = (alpha_fast * (float)raw_value) + ((1.0f - alpha_fast) * smoothed_value_ema);
              } else {
                  // Leitura estável: usa o filtro LENTO para máxima estabilidade
                  smoothed_value_ema = (alpha_slow * (float)raw_value) + ((1.0f - alpha_slow) * smoothed_value_ema);
              }
          }

          // 3. CONVERTE o valor filtrado para gramas
          weight_in_grams = ADS1232_ConvertToGrams((int32_t)smoothed_value_ema);
          
          // 4. PREPARA o valor para o display
          int32_t weight_for_dwin = (int32_t)(weight_in_grams * 100.0f);
          
          // 5. ENVIA o novo valor para o display IMEDIATAMENTE
          DWIN_SendData(0x2000, weight_for_dwin);

          // 6. Pisca o LED para indicar atividade
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