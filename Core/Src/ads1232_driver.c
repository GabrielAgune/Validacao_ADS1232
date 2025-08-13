#include "ads1232_driver.h"
#include <stdio.h>
#include <stdlib.h>

// --- DEFINIÇÃO DA TABELA DE CALIBRAÇÃO ---
// Os valores de adc_value devem ser preenchidos por você com a rotina de calibração
CalPoint_t cal_points[NUM_CAL_POINTS] = {
    {0.0f, 235469},
    {50.0f, 546061},
    {100.0f, 856428},
    {200.0f, 1477409}
};

// --- Variáveis Estáticas ---
static int32_t adc_offset = 0;

// --- Funções Privadas ---
static void delay_us(uint32_t us) {
    for(volatile uint32_t i = 0; i < us * 8; i++);
}

static void sort_three(int32_t *a, int32_t *b, int32_t *c) {
    int32_t temp;
    if (*a > *b) { temp = *a; *a = *b; *b = temp; }
    if (*b > *c) { temp = *b; *b = *c; *c = temp; }
    if (*a > *b) { temp = *a; *a = *b; *b = temp; }
}

// --- Implementação das Funções Públicas ---
void ADS1232_Init(void) {
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); 
    HAL_GPIO_WritePin(ADC_PDWN_GPIO_Port, ADC_PDWN_Pin, GPIO_PIN_SET);
}

int32_t ADS1232_Read(void) {
    uint32_t data = 0;
    uint32_t timeout = HAL_GetTick() + 200;

    while(HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_SET) {
        if (HAL_GetTick() > timeout) return 0;
    }

    __disable_irq();
    for(int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);
        delay_us(1);
        data = data << 1;
        if(HAL_GPIO_ReadPin(ADC_DOUT_GPIO_Port, ADC_DOUT_Pin) == GPIO_PIN_SET) data |= 1;
        HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_RESET);
        delay_us(1);
    }
    HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_SET);
    delay_us(1);
    HAL_GPIO_WritePin(ADC_SCLK_GPIO_Port, ADC_SCLK_Pin, GPIO_PIN_RESET);
    __enable_irq();

    if (data & 0x800000) data |= 0xFF000000;
    return (int32_t)data;
}

int32_t ADS1232_Read_Median_of_3(void) {
    int32_t s1 = ADS1232_Read();
    int32_t s2 = ADS1232_Read();
    int32_t s3 = ADS1232_Read();
    sort_three(&s1, &s2, &s3);
    return s2;
}

int32_t ADS1232_Tare(void) { 
    printf("Tarando... Aguarde estabilidade.\r\n");
    const int num_samples = 20;
    const int32_t stability_threshold = 500;
    int max_retries = 10;
    
    for (int retry = 0; retry < max_retries; retry++) {
        int64_t sum = 0;
        int32_t min_val = 0x7FFFFFFF, max_val = 0x80000000;
        for (int i = 0; i < num_samples; i++) {
            int32_t sample = ADS1232_Read_Median_of_3();
            sum += sample;
            if (sample < min_val) min_val = sample;
            if (sample > max_val) max_val = sample;
            HAL_Delay(10);
        }
        if ((max_val - min_val) < stability_threshold) {
            adc_offset = sum / num_samples;
            // Atualiza o ponto zero na tabela de calibração
            cal_points[0].adc_value = adc_offset; 
            printf("Tara estavel concluida! Offset = %d\r\n", (int)adc_offset);
            return adc_offset; 
        }
        printf("Leituras instaveis (diff: %d). Tentando novamente...\r\n", (int)(max_val - min_val));
    }
    printf("AVISO: Balanca nao estabilizou.\r\n");
    return adc_offset; // Retorna o offset antigo se falhar
}

float ADS1232_ConvertToGrams(int32_t raw_value) {
    // A extrapolação para valores abaixo de 0g é feita com o primeiro segmento (0-50g)
    if (raw_value < cal_points[0].adc_value) {
        float factor = (cal_points[1].grams - cal_points[0].grams) / 
                       (float)(cal_points[1].adc_value - cal_points[0].adc_value);
        return (float)(raw_value - cal_points[0].adc_value) * factor;
    }

    // Procura o segmento correto para interpolação
    for (int i = 0; i < NUM_CAL_POINTS - 1; i++) {
        if (raw_value <= cal_points[i+1].adc_value) {
            float adc_range = (float)(cal_points[i+1].adc_value - cal_points[i].adc_value);
            if (adc_range == 0) return cal_points[i].grams;

            float gram_range = cal_points[i+1].grams - cal_points[i].grams;
            float ratio = (float)(raw_value - cal_points[i].adc_value) / adc_range;
            return cal_points[i].grams + (ratio * gram_range);
        }
    }

    // A extrapolação para valores acima do último ponto é feita com o último segmento
    int last = NUM_CAL_POINTS - 1;
    int second_last = NUM_CAL_POINTS - 2;
    float factor = (cal_points[last].grams - cal_points[second_last].grams) / 
                   (float)(cal_points[last].adc_value - cal_points[second_last].adc_value);
    return cal_points[last].grams + ((float)(raw_value - cal_points[last].adc_value) * factor);
}

int32_t ADS1232_GetOffset(void) {
    return adc_offset;
}

void ADS1232_SetOffset(int32_t new_offset) {
    adc_offset = new_offset;
    cal_points[0].adc_value = new_offset; // Garante que o ponto zero está sempre atualizado
}