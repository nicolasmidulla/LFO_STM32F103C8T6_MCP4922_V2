#ifndef MULTIPLEXOR_H
#define MULTIPLEXOR_H

#include "main.h"
#include "project_config.h"
#include <stdint.h>

/* Variables exportadas */
extern uint16_t adc_values[MUX_CHANNELS];

/* Prototipos de funciones */
void MUX_SelectChannel(uint8_t channel);
void MUX_ReadAllChannels(void);
void ProcessADCValues(void);

#endif /* MULTIPLEXOR_H */
