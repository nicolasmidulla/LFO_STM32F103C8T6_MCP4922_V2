#include "mcp4922.h"
#include "signal_generator.h"
#include "spi.h"

/* Variables locales */
uint32_t spi_errors = 0;

/* Configuraciones del MCP4922 */
#define MCP4922_CH_A            0x0000
#define MCP4922_CH_B            0x8000
#define MCP4922_GAINx1          0x2000
#define MCP4922_SHUTDOWN_OFF    0x1000

void MCP4922_Init(void) {
    // Configurar CS como inactivo (alto)
    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_SET);

    // Inicializar los generadores de señal
    SignalGenerator_Init();

    // Escribir valores iniciales en ambos canales
    MCP4922_WriteValue(0, 2048);  // Canal A en el centro
    MCP4922_WriteValue(1, 2048);  // Canal B en el centro
}

HAL_StatusTypeDef MCP4922_WriteValue(uint8_t channel, uint16_t value) {
    uint16_t spi_data;
    uint8_t data_bytes[2];
    HAL_StatusTypeDef result;

    // Limitar el valor a 12 bits
    if(value > 4095) value = 4095;

    // Formar el comando para el MCP4922
    spi_data = (channel == 0) ? MCP4922_CH_A : MCP4922_CH_B;
    spi_data |= MCP4922_GAINx1;           // Ganancia x1
    spi_data |= MCP4922_SHUTDOWN_OFF;     // DAC activo
    spi_data |= (value & 0x0FFF);         // Valor de 12 bits

    // Convertir a bytes para transmisión
    data_bytes[0] = (spi_data >> 8) & 0xFF;
    data_bytes[1] = spi_data & 0xFF;

    // Transmitir por SPI
    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_RESET);
    result = HAL_SPI_Transmit(&hspi1, data_bytes, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(MCP4922_CS_GPIO_Port, MCP4922_CS_Pin, GPIO_PIN_SET);

    // Contar errores si ocurren
    if(result != HAL_OK) {
        spi_errors++;
    }

    return result;
}
