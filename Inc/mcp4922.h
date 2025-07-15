#ifndef MCP4922_H
#define MCP4922_H

#include "main.h"
#include "project_config.h"
#include <stdint.h>

/* Prototipos de funciones del MCP4922 */
void MCP4922_Init(void);
HAL_StatusTypeDef MCP4922_WriteValue(uint8_t channel, uint16_t value);

/* Variables exportadas */
extern uint32_t spi_errors;

#endif /* MCP4922_H */
