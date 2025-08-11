#ifndef __DWIN_DRIVER_H
#define __DWIN_DRIVER_H

#include "stm32c0xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

// Define o tipo da fun��o de callback que tratar� os dados recebidos do DWIN
typedef void (*dwin_rx_callback_t)(const uint8_t* buffer, uint16_t len);

// Fun��es P�blicas
void DWIN_Driver_Init(UART_HandleTypeDef *huart, dwin_rx_callback_t callback);
void DWIN_Driver_Process(void);
void DWIN_Driver_SetScreen(uint16_t screen_id);
void DWIN_Driver_WriteInt(uint16_t vp_address, int16_t value);
void DWIN_Driver_WriteInt32(uint16_t vp_address, int32_t value);

// --- PROT�TIPO ADICIONADO AQUI ---
void DWIN_Driver_WriteRawBytes(const uint8_t* data, uint16_t size);

// Fun��es internas, chamadas pelas interrup��es da HAL
void DWIN_Driver_HandleRxEvent(uint16_t size);
void DWIN_Driver_HandleError(UART_HandleTypeDef *huart);

#endif // __DWIN_DRIVER_H