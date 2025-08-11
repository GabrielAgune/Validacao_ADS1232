#ifndef __CLI_DRIVER_H
#define __CLI_DRIVER_H

#include "stm32c0xx_hal.h"
#include <stdbool.h>

/**
 * @brief Inicializa a Command Line Interface (CLI).
 * @param debug_huart Ponteiro para o handle da UART de debug (normalmente USART2).
 */
void CLI_Init(UART_HandleTypeDef* debug_huart);

/**
 * @brief Processa um comando pendente na CLI.
 * Deve ser chamada repetidamente no laço principal do programa.
 */
void CLI_Process(void);

/**
 * @brief Entrega um caractere recebido da UART para o buffer da CLI.
 * Deve ser chamada pela interrupção de recepção da UART.
 * @param received_char O caractere recebido.
 */
void CLI_Receive_Char(uint8_t received_char);

#endif // __CLI_DRIVER_H