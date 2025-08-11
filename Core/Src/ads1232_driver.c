#include "ads1232_driver.h"
#include <stdio.h>
#include <stdlib.h>

// --- Vari�veis Est�ticas (Privadas ao M�dulo) ---
static int32_t adc_offset = 0;
static float calibration_factor = 1.0f;

// --- Fun��es Privadas ---
static void delay_us(uint32_t us) {
    for(volatile uint32_t i = 0; i < us * 8; i++);
}

// --- Implementa��o das Fun��es P�blicas ---

void ADS1232_Init(void) {
    // Garante que PDWN esteja baixo enquanto a alimenta��o estabiliza
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // Espera 1ms para garantir

    // Sequ�ncia de toggle recomendada pelo datasheet (t16 e t17)
    // t16: Pulso ALTO em PDWN (m�nimo 26�s) [cite: 1311]
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_SET);
    delay_us(50);

    // t17: Pulso BAIXO em PDWN (m�nimo 26�s) [cite: 1311]
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_RESET);
    delay_us(50);

    // Deixa PDWN em ALTO para o modo de opera��o normal
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_SET);
    //printf("Driver ADS1232 Inicializado.\r\n");
}

// Em ads1232_driver.c

int32_t ADS1232_Read(void) {
    uint32_t data = 0;
    uint32_t timeout = HAL_GetTick() + 200;

    // Espera at� que DRDY/DOUT fique em n�vel baixo
    while(HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() > timeout) {
            //printf("TIMEOUT! DRDY nao ficou baixo.\r\n");
            return 0; // Retorna 0 em caso de timeout
        }
    }

    // Pequeno delay para garantir que o sinal esteja est�vel
    delay_us(1);

    // Desativa interrup��es brevemente para garantir a integridade da leitura bit-a-bit
    __disable_irq();

    // L� os 24 bits de dados
    for(int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);
        delay_us(2);
        data = data << 1;
        if(HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_SET) {
            data |= 1;
        }
        HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_RESET);
        delay_us(2);
    }

    // --- CORRE��O CR�TICA ABAIXO ---
    // Envia um 25� pulso de clock para for�ar DRDY/DOUT para n�vel ALTO.
    // Isso evita que o loop principal tente ler os mesmos dados duas vezes.
    HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);
    delay_us(2);
    HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_RESET);
    delay_us(2);

    // Reativa as interrup��es
    __enable_irq();

    // Faz a extens�o de sinal para 32 bits (n�mero negativo)
    if (data & 0x800000) {
        data |= 0xFF000000;
    }

    return (int32_t)data;
}

int32_t ADS1232_Tare(void) { 
    printf("Tarando... Aguarde estabilidade.\r\n");

    const int num_samples_for_stability = 20; // N�mero de amostras para checar estabilidade
    const int32_t stability_threshold = 500;   // Toler�ncia m�xima entre leituras (ajuste conforme necess�rio)
    int32_t samples[num_samples_for_stability];
    int stable_count = 0;
    int max_retries = 10; // Tenta estabilizar por no m�ximo 10 ciclos
    
    for (int retry = 0; retry < max_retries; retry++) {
        int32_t min_val = 0x7FFFFFFF;
        int32_t max_val = 0x80000000;
        int64_t sum = 0;

        for (int i = 0; i < num_samples_for_stability; i++) {
            samples[i] = ADS1232_Read();
            sum += samples[i];
            if (samples[i] < min_val) min_val = samples[i];
            if (samples[i] > max_val) max_val = samples[i];
            HAL_Delay(10); // Pequeno delay entre as leituras
        }

        // Verifica se a diferen�a entre a maior e a menor leitura est� dentro da toler�ncia
        if ((max_val - min_val) < stability_threshold) {
            stable_count++;
        } else {
            stable_count = 0; // Reseta se n�o estiver est�vel
        }

        // Se as leituras permanecerem est�veis por 2 ciclos consecutivos, consideramos a tara bem-sucedida
        if (stable_count >= 2) {
            adc_offset = sum / num_samples_for_stability;
            printf("Tara estavel concluida! Offset = %ld\r\n", (long)adc_offset);
            return adc_offset; 
        }
        
        printf("Leituras instaveis (diff: %ld). Tentando novamente...\r\n", (long)(max_val - min_val));
    }

    // Se saiu do loop, a balan�a n�o estabilizou. Usa a m�dia da �ltima tentativa.
    printf("AVISO: Balanca nao estabilizou. Usando ultima media como tara.\r\n");
    int64_t last_sum = 0;
    for(int i = 0; i < num_samples_for_stability; i++) {
        last_sum += samples[i];
    }
    adc_offset = last_sum / num_samples_for_stability;
    printf("Tara concluida! Offset = %ld\r\n", (long)adc_offset);
		return adc_offset;
}

void ADS1232_SetCalibrationFactor(float factor) {
    calibration_factor = factor;
}

float ADS1232_ConvertToGrams(int32_t raw_value) {
    if (calibration_factor == 0.0f) {
        return 0.0f;
    }
    return (float)(raw_value - adc_offset) / calibration_factor;
}

int32_t ADS1232_GetOffset(void) {
    return adc_offset;
}

void ADS1232_SetOffset(int32_t new_offset) {
    adc_offset = new_offset;
}
