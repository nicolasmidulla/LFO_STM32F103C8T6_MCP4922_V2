#ifndef SIGNAL_GENERATOR_H
#define SIGNAL_GENERATOR_H

#include "main.h"
#include "project_config.h"
#include <stdint.h>

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

/* Estructura del generador de señales */
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

/* Variables globales exportadas */
extern Signal_Generator_t signal_gen_A;
extern Signal_Generator_t signal_gen_B;
extern const uint8_t subdivisions[6];
extern const char* waveform_names[WAVE_COUNT];
extern const char* mode_names[2];
extern const uint16_t sine_table[256];

/* Prototipos de funciones */
void SignalGenerator_Init(void);
void UpdateWaveform(void);
uint16_t GenerateWaveform(Signal_Generator_t* gen);
uint16_t GenerateSineWave(Signal_Generator_t* gen);
uint16_t GenerateSquareWave(Signal_Generator_t* gen);
uint16_t GenerateTriangleWave(Signal_Generator_t* gen);
uint16_t GenerateSawtoothWave(Signal_Generator_t* gen);
uint16_t GenerateRampDownWave(Signal_Generator_t* gen);
uint16_t GenerateRandomWave(Signal_Generator_t* gen);

#endif /* SIGNAL_GENERATOR_H */
