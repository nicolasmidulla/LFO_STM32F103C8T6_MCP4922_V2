#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include "main.h"
#include <stdint.h>

/* Configuraciones */
#define SAMPLE_RATE         10000
#define PHASE_RESOLUTION    24
#define FREQ_SCALE          ((1UL << PHASE_RESOLUTION) / SAMPLE_RATE)

/* Tipos de forma de onda */
typedef enum {
    WAVE_SINE = 0,
    WAVE_SQUARE,
    WAVE_TRIANGLE,
    WAVE_SAWTOOTH,
    WAVE_RAMP_DOWN,
    WAVE_RANDOM,
    WAVE_COUNT
} WaveformType_t;

/* Modos de operación */
typedef enum {
    MODE_FREE = 0,
    MODE_SYNC
} OperationMode_t;

/* Estructura del generador */
typedef struct {
    uint16_t amplitude;
    uint32_t frequency;
    uint32_t phase_acc;
    uint32_t phase_inc;
    uint16_t current_value;
    uint8_t waveform;
    uint8_t sync_mode;
    uint32_t clock_period;
    uint8_t subdivision;
} Signal_Generator_t;

/* Variables globales */
extern Signal_Generator_t signal_gen_A;
extern Signal_Generator_t signal_gen_B;
extern const char* waveform_names[WAVE_COUNT];

/* Funciones */
void SignalGenerator_Init(void);
void UpdateWaveform(void);
uint16_t GenerateWaveform(Signal_Generator_t* gen);

#endif
