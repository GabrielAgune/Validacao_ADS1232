#ifndef __ADS1232_DRIVER_H
#define __ADS1232_DRIVER_H

#include "main.h"
#include <stdint.h>

// --- DEFINI��ES PARTILHADAS PARA CALIBRA��O ---
#define NUM_CAL_POINTS 4

typedef struct {
    float grams;
    int32_t adc_value;
} CalPoint_t;

// Declara��o 'extern' para tornar a tabela vis�vel para outros ficheiros
extern CalPoint_t cal_points[NUM_CAL_POINTS];


// --- Fun��es P�blicas ---
void ADS1232_Init(void);
int32_t ADS1232_Read(void);
int32_t ADS1232_Read_Median_of_3(void);
int32_t ADS1232_Tare(void);
void ADS1232_SetCalibrationFactor(float factor);
float ADS1232_ConvertToGrams(int32_t raw_value);
int32_t ADS1232_GetOffset(void);
void ADS1232_SetOffset(int32_t new_offset);
float ADS1232_GetCalibrationFactor(void);


#endif // __ADS1232_DRIVER_H