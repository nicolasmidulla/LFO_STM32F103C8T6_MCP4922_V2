#include "signal_generator.h"

/* Variables globales */
Signal_Generator_t signal_gen_A;
Signal_Generator_t signal_gen_B;

const char* waveform_names[WAVE_COUNT] = {"Sine", "Square", "Triangle", "Sawtooth", "RampDn", "Random"};

const uint16_t sine_table[256] = {
    2048, 2098, 2148, 2198, 2248, 2298, 2348, 2398, 2447, 2496, 2545, 2594, 2642, 2690, 2737, 2784,
    2831, 2877, 2923, 2968, 3013, 3057, 3100, 3143, 3185, 3226, 3267, 3307, 3346, 3385, 3423, 3459,
    3495, 3530, 3565, 3598, 3630, 3662, 3692, 3722, 3750, 3777, 3804, 3829, 3853, 3876, 3898, 3919,
    3939, 3958, 3976, 3992, 4007, 4021, 4034, 4045, 4056, 4065, 4073, 4080, 4085, 4089, 4093, 4094,
    4095, 4094, 4093, 4089, 4085, 4080, 4073, 4065, 4056, 4045, 4034, 4021, 4007, 3992, 3976, 3958,
    3939, 3919, 3898, 3876, 3853, 3829, 3804, 3777, 3750, 3722, 3692, 3662, 3630, 3598, 3565, 3530,
    3495, 3459, 3423, 3385, 3346, 3307, 3267, 3226, 3185, 3143, 3100, 3057, 3013, 2968, 2923, 2877,
    2831, 2784, 2737, 2690, 2642, 2594, 2545, 2496, 2447, 2398, 2348, 2298, 2248, 2198, 2148, 2098,
    2048, 1998, 1948, 1898, 1848, 1798, 1748, 1698, 1649, 1600, 1551, 1502, 1454, 1406, 1359, 1312,
    1265, 1219, 1173, 1128, 1083, 1039,  996,  953,  911,  870,  829,  789,  750,  711,  673,  637,
     601,  566,  531,  498,  466,  434,  404,  374,  346,  319,  292,  267,  243,  220,  198,  177,
     157,  138,  120,  104,   89,   75,   62,   51,   40,   31,   23,   16,   11,    7,    3,    2,
       1,    2,    3,    7,   11,   16,   23,   31,   40,   51,   62,   75,   89,  104,  120,  138,
     157,  177,  198,  220,  243,  267,  292,  319,  346,  374,  404,  434,  466,  498,  531,  566,
     601,  637,  673,  711,  750,  789,  829,  870,  911,  953,  996, 1039, 1083, 1128, 1173, 1219,
    1265, 1312, 1359, 1406, 1454, 1502, 1551, 1600, 1649, 1698, 1748, 1798, 1848, 1898, 1948, 1998
};

void SignalGenerator_Init(void) {
    signal_gen_A.amplitude = 1023;
    signal_gen_A.frequency = 1000000;
    signal_gen_A.phase_acc = 0;
    signal_gen_A.phase_inc = (signal_gen_A.frequency * FREQ_SCALE) / 1000;
    signal_gen_A.current_value = 2048;
    signal_gen_A.waveform = WAVE_SINE;
    signal_gen_A.sync_mode = MODE_FREE;

    signal_gen_B.amplitude = 512;
    signal_gen_B.frequency = 2000000;
    signal_gen_B.phase_acc = 0;
    signal_gen_B.phase_inc = (signal_gen_B.frequency * FREQ_SCALE) / 1000;
    signal_gen_B.current_value = 2048;
    signal_gen_B.waveform = WAVE_SQUARE;
    signal_gen_B.sync_mode = MODE_FREE;
}

void UpdateWaveform(void) {
    signal_gen_A.current_value = GenerateWaveform(&signal_gen_A);
    signal_gen_B.current_value = GenerateWaveform(&signal_gen_B);
}

uint16_t GenerateWaveform(Signal_Generator_t* gen) {
    gen->phase_acc += gen->phase_inc;

    switch(gen->waveform) {
        case WAVE_SINE: {
            uint8_t table_index = (gen->phase_acc >> (PHASE_RESOLUTION - 8)) & 0xFF;
            uint16_t sine_value = sine_table[table_index];
            int32_t centered_sine = sine_value - 2048;
            int32_t scaled_sine = (centered_sine * gen->amplitude) >> 11;
            int32_t final_value = 2048 + scaled_sine;
            if(final_value < 0) final_value = 0;
            if(final_value > 4095) final_value = 4095;
            return (uint16_t)final_value;
        }

        case WAVE_SQUARE: {
            uint32_t phase_norm = gen->phase_acc >> (PHASE_RESOLUTION - 16);
            if(phase_norm < 32768) {
                return 2048 + gen->amplitude;
            } else {
                return 2048 - gen->amplitude;
            }
        }

        default:
            return 2048; // Centro
    }
}
