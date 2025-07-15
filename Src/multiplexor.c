#include "multiplexor.h"
#include "signal_generator.h"

/* Variables externas */
extern ADC_HandleTypeDef hadc1;

/* Variables globales */
uint16_t adc_values[MUX_CHANNELS];

void MUX_SelectChannel(uint8_t channel) {
    // Seleccionar canal del multiplexor usando los bits A, B, C
    HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, (channel & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, (channel & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MUX_C_GPIO_Port, MUX_C_Pin, (channel & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Pequeña pausa para estabilización
    HAL_Delay(1);
}

void MUX_ReadAllChannels(void) {
    for(uint8_t i = 0; i < MUX_CHANNELS; i++) {
        // Seleccionar canal del multiplexor
        MUX_SelectChannel(i);

        // Leer valor del ADC
        HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
            adc_values[i] = HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
    }

    // Procesar los valores leídos
    ProcessADCValues();
}
