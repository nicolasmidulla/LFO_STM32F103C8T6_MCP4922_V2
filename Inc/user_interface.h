#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "project_config.h"
#include <stdint.h>

/* Prototipos de funciones */
void CheckButtons(void);
void CheckSyncInputs(void);
void PrintDebugInfo(void);

/* Variables exportadas */
extern volatile uint32_t last_btn_A_press;
extern volatile uint32_t last_btn_B_press;
extern volatile uint32_t btn_A_press_start;
extern volatile uint32_t btn_B_press_start;
extern volatile uint8_t btn_A_long_press;
extern volatile uint8_t btn_B_long_press;
extern volatile uint32_t last_sync_A_time;
extern volatile uint32_t last_sync_B_time;
extern volatile uint32_t sync_A_prev_time;
extern volatile uint32_t sync_B_prev_time;
extern volatile uint8_t sync_A_triggered;
extern volatile uint8_t sync_B_triggered;
extern uint32_t debug_counter;

#endif /* USER_INTERFACE_H */
