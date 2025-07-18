/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MUX_A_Pin GPIO_PIN_1
#define MUX_A_GPIO_Port GPIOA
#define MUX_B_Pin GPIO_PIN_2
#define MUX_B_GPIO_Port GPIOA
#define MUX_C_Pin GPIO_PIN_3
#define MUX_C_GPIO_Port GPIOA
#define MCP4922_CS_Pin GPIO_PIN_4
#define MCP4922_CS_GPIO_Port GPIOA
#define BOTON_A_Pin GPIO_PIN_0
#define BOTON_A_GPIO_Port GPIOB
#define BOTON_B_Pin GPIO_PIN_1
#define BOTON_B_GPIO_Port GPIOB
#define SYNCIN_A_Pin GPIO_PIN_10
#define SYNCIN_A_GPIO_Port GPIOB
#define SYNCIN_B_Pin GPIO_PIN_11
#define SYNCIN_B_GPIO_Port GPIOB
#define LED_OUT_Pin GPIO_PIN_13
#define LED_OUT_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
