/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cmsis_os.h"

#include "arm_math.h"

#include "stdio.h"
#include "stdbool.h"
#include "malloc.h"
#include "stdarg.h"
#include "string.h"

#include "MicroSeconds.h"

#include "as5048a.h"
#include "eeprom.hpp"
#include "ina226.hpp"
#include "foc.h"

#define DEBUG_MAIN 0
#define DEBUG_EEPROM 0
#define DEBUG_INA226 0
#define DEBUG_ENCODER 0
#define DEBUG_MOTOR 1

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOT_DRV_FAULT_Pin GPIO_PIN_13
#define MOT_DRV_FAULT_GPIO_Port GPIOC
#define MOT_DRV_RST_Pin GPIO_PIN_14
#define MOT_DRV_RST_GPIO_Port GPIOC
#define MPU_INIT_Pin GPIO_PIN_2
#define MPU_INIT_GPIO_Port GPIOA
#define MPU_CS_Pin GPIO_PIN_3
#define MPU_CS_GPIO_Port GPIOA
#define ENCODER_CS_Pin GPIO_PIN_4
#define ENCODER_CS_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_1
#define RED_LED_GPIO_Port GPIOB
#define GREEN_LED_Pin GPIO_PIN_2
#define GREEN_LED_GPIO_Port GPIOB
#define MOT_EN1_Pin GPIO_PIN_10
#define MOT_EN1_GPIO_Port GPIOB
#define MOT_EN2_Pin GPIO_PIN_11
#define MOT_EN2_GPIO_Port GPIOB
#define MOT_EN3_Pin GPIO_PIN_12
#define MOT_EN3_GPIO_Port GPIOB
#define MOT_DRV_SLEEP_Pin GPIO_PIN_13
#define MOT_DRV_SLEEP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
