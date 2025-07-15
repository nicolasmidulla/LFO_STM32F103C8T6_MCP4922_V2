#include "user_interface.h"
#include "signal_generator.h"
#include "multiplexor.h"
#include "mcp4922.h"
#include "project_config.h"
#include <stdio.h>

/* Variables globales para botones */
volatile uint32_t last_btn_A_press = 0;
volatile uint32_t last_btn_B_press = 0;
volatile uint32_t btn_A_press_start = 0;
volatile uint32_t btn_B_press_start = 0;
volatile uint8_t btn_A_long_press = 0;
volatile uint8_t btn_B_long_press = 0;

/* Variables globales para sincronización */
volatile uint32_t last_sync_A_time = 0;
volatile uint32_t last_sync_B_time = 0;
volatile uint32_t sync_A_prev_time = 0;
volatile uint32_t sync_B_prev_time = 0;
volatile uint8_t sync_A_triggered = 0;
volatile uint8_t sync_B_triggered = 0;

/* Variables de debug */
uint32_t debug_counter = 0;

void CheckButtons(void) {
    uint32_t current_time = HAL_GetTick();

    // Revisar botón del canal A
    if(HAL_GPIO_ReadPin(BTN_WAVE_A_GPIO_Port, BTN_WAVE_A_Pin) == GPIO_PIN_RESET) {
        if(btn_A_press_start == 0) {
            btn_A_press_start = current_time;
        } else if((current_time - btn_A_press_start) >= LONG_PRESS_TIME_MS && !btn_A_long_press) {
            btn_A_long_press = 1;
            // Cambiar modo de operación (pulsación larga)
            signal_gen_A.sync_mode = (signal_gen_A.sync_mode == MODE_FREE) ? MODE_SYNC : MODE_FREE;
            printf("Canal A: Modo cambiado a %s\r\n", mode_names[signal_gen_A.sync_mode]);
        }
    } else {
        if(btn_A_press_start != 0) {
            uint32_t press_duration = current_time - btn_A_press_start;
            if(press_duration < LONG_PRESS_TIME_MS && (current_time - last_btn_A_press) > DEBOUNCE_TIME_MS) {
                last_btn_A_press = current_time;
                // Cambiar forma de onda (pulsación corta)
                signal_gen_A.waveform = (signal_gen_A.waveform + 1) % WAVE_COUNT;
                printf("Canal A: Forma de onda cambiada a %s\r\n", waveform_names[signal_gen_A.waveform]);
            }
            btn_A_press_start = 0;
            btn_A_long_press = 0;
        }
    }

    // Revisar botón del canal B
    if(HAL_GPIO_ReadPin(BTN_WAVE_B_GPIO_Port, BTN_WAVE_B_Pin) == GPIO_PIN_RESET) {
        if(btn_B_press_start == 0) {
            btn_B_press_start = current_time;
        } else if((current_time - btn_B_press_start) >= LONG_PRESS_TIME_MS && !btn_B_long_press) {
            btn_B_long_press = 1;
            // Cambiar modo de operación (pulsación larga)
            signal_gen_B.sync_mode = (signal_gen_B.sync_mode == MODE_FREE) ? MODE_SYNC : MODE_FREE;
            printf("Canal B: Modo cambiado a %s\r\n", mode_names[signal_gen_B.sync_mode]);
        }
    } else {
        if(btn_B_press_start != 0) {
            uint32_t press_duration = current_time - btn_B_press_start;
            if(press_duration < LONG_PRESS_TIME_MS && (current_time - last_btn_B_press) > DEBOUNCE_TIME_MS) {
                last_btn_B_press = current_time;
                // Cambiar forma de onda (pulsación corta)
                signal_gen_B.waveform = (signal_gen_B.waveform + 1) % WAVE_COUNT;
                printf("Canal B: Forma de onda cambiada a %s\r\n", waveform_names[signal_gen_B.waveform]);
            }
            btn_B_press_start = 0;
            btn_B_long_press = 0;
        }
    }
}

void CheckSyncInputs(void) {
    uint32_t current_time = HAL_GetTick();

    // Revisar entrada de sincronización del canal A
    if(HAL_GPIO_ReadPin(SYNC_A_GPIO_Port, SYNC_A_Pin) == GPIO_PIN_RESET) {
        if(current_time - last_sync_A_time > SYNC_DEBOUNCE_MS && !sync_A_triggered) {
            sync_A_triggered = 1;

            // Calcular período del reloj externo
            if(sync_A_prev_time != 0) {
                signal_gen_A.clock_period = current_time - sync_A_prev_time;

                if(signal_gen_A.sync_mode == MODE_SYNC && signal_gen_A.clock_period > 0) {
                    uint32_t effective_period = signal_gen_A.clock_period * signal_gen_A.subdivision;
                    uint32_t effective_freq = 1000000 / effective_period;
                    signal_gen_A.phase_inc = (effective_freq * FREQ_SCALE) / 1000;
                }
            }

            sync_A_prev_time = current_time;
            last_sync_A_time = current_time;

            // Reset de fase para sincronización
            signal_gen_A.phase_acc = 0;
        }
    } else {
        sync_A_triggered = 0;
    }

    // Revisar entrada de sincronización del canal B
    if(HAL_GPIO_ReadPin(SYNC_B_GPIO_Port, SYNC_B_Pin) == GPIO_PIN_RESET) {
        if(current_time - last_sync_B_time > SYNC_DEBOUNCE_MS && !sync_B_triggered) {
            sync_B_triggered = 1;

            // Calcular período del reloj externo
            if(sync_B_prev_time != 0) {
                signal_gen_B.clock_period = current_time - sync_B_prev_time;

                if(signal_gen_B.sync_mode == MODE_SYNC && signal_gen_B.clock_period > 0) {
                    uint32_t effective_period = signal_gen_B.clock_period * signal_gen_B.subdivision;
                    uint32_t effective_freq = 1000000 / effective_period;
                    signal_gen_B.phase_inc = (effective_freq * FREQ_SCALE) / 1000;
                }
            }

            sync_B_prev_time = current_time;
            last_sync_B_time = current_time;

            // Reset de fase para sincronización
            signal_gen_B.phase_acc = 0;
        }
    } else {
        sync_B_triggered = 0;
    }
}

void PrintDebugInfo(void) {
    debug_counter++;

    printf("=== Debug #%lu ===\r\n", debug_counter);
    printf("SPI Errors: %lu\r\n", spi_errors);
    printf("ADC: [%d, %d, %d, %d]\r\n", adc_values[0], adc_values[1], adc_values[2], adc_values[3]);

    // Información del canal A
    if(signal_gen_A.sync_mode == MODE_FREE) {
        printf("Ch A: %s [%s], Amp=%d, Freq=%lu Hz, DAC=%d\r\n",
               waveform_names[signal_gen_A.waveform], mode_names[signal_gen_A.sync_mode],
               signal_gen_A.amplitude, signal_gen_A.frequency/1000, signal_gen_A.current_value);
    } else {
        printf("Ch A: %s [%s], Amp=%d, Period=%lu ms, Subdiv=/%d, DAC=%d\r\n",
               waveform_names[signal_gen_A.waveform], mode_names[signal_gen_A.sync_mode],
               signal_gen_A.amplitude, signal_gen_A.clock_period, signal_gen_A.subdivision, signal_gen_A.current_value);
    }

    // Información del canal B
    if(signal_gen_B.sync_mode == MODE_FREE) {
        printf("Ch B: %s [%s], Amp=%d, Freq=%lu Hz, DAC=%d\r\n",
               waveform_names[signal_gen_B.waveform], mode_names[signal_gen_B.sync_mode],
               signal_gen_B.amplitude, signal_gen_B.frequency/1000, signal_gen_B.current_value);
    } else {
        printf("Ch B: %s [%s], Amp=%d, Period=%lu ms, Subdiv=/%d, DAC=%d\r\n",
               waveform_names[signal_gen_B.waveform], mode_names[signal_gen_B.sync_mode],
               signal_gen_B.amplitude, signal_gen_B.clock_period, signal_gen_B.subdivision, signal_gen_B.current_value);
    }

    // Toggle del LED de heartbeat
    HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);
    printf("Heartbeat LED\r\n\r\n");
}], mode_names[signal_gen_B.sync_mode],
               signal_gen_B.amplitude, signal_gen_B.clock_period, signal_gen_B.subdivision, signal_gen_B.current_value);
    }

    // Toggle del LED de heartbeat
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    printf("Heartbeat LED\r\n\r\n");
}
