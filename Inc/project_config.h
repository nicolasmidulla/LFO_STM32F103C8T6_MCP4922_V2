#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#include "main.h"
#include <stdint.h>

/* Configuraciones del sistema */
#define SAMPLE_RATE         10000
#define TIMER_FREQUENCY     72000000
#define TIMER_PRESCALER     720
#define TIMER_PERIOD        (TIMER_FREQUENCY / TIMER_PRESCALER / SAMPLE_RATE)

/* Configuraciones del MUX y MCP4922 */
#define MUX_CHANNELS        8
#define PHASE_RESOLUTION    24
#define FREQ_SCALE          ((1UL << PHASE_RESOLUTION) / SAMPLE_RATE)

/* Configuraciones del MCP4922 */
#define MCP4922_CH_A            0x0000
#define MCP4922_CH_B            0x8000
#define MCP4922_GAINx1          0x2000
#define MCP4922_SHUTDOWN_OFF    0x1000

/* Configuraciones de timing */
#define DEBOUNCE_TIME_MS     200
#define LONG_PRESS_TIME_MS   3000
#define SYNC_DEBOUNCE_MS     10

/* Los pines ya están definidos en main.h generado por CubeMX */

#endif /* PROJECT_CONFIG_H */