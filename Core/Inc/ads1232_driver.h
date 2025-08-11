#ifndef INC_ADS1232_DRIVER_H_
#define INC_ADS1232_DRIVER_H_

#include "main.h" // Inclui para ter acesso aos tipos HAL

// --- Fun��es P�blicas do Driver ---

/**
  * @brief Inicializa o ADC ADS1232.
  */
void ADS1232_Init(void);

/**
  * @brief Executa o procedimento de tara, zerando a medi��o atual.
  */
int32_t ADS1232_Tare(void);

/**
  * @brief L� o valor raw de 24 bits do ADC (leitura �nica e direta).
  * @retval O valor de 32 bits com sinal lido do ADC.
  */
int32_t ADS1232_Read(void);

/**
  * @brief Define o fator de calibra��o para a convers�o para gramas.
  * @param factor O fator calculado (valor_raw / peso_conhecido_g).
  */
void ADS1232_SetCalibrationFactor(float factor);

/**
  * @brief Converte o valor raw atual em gramas.
  * @retval O peso calculado em gramas.
  */
float ADS1232_ConvertToGrams(int32_t raw_value);

/**
  * @brief Obt�m o valor de offset atual (tara).
  * @retval O valor do offset.
  */
int32_t ADS1232_GetOffset(void);

void ADS1232_SetOffset(int32_t new_offset);

#endif /* INC_ADS1232_DRIVER_H_ */
