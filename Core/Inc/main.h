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
#include "stm32g4xx_hal.h"

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
#define IMU2_INT_Pin GPIO_PIN_1
#define IMU2_INT_GPIO_Port GPIOA
#define IMU1_INT_Pin GPIO_PIN_4
#define IMU1_INT_GPIO_Port GPIOA
#define IMU1_INT_EXTI_IRQn EXTI4_IRQn
#define IMU1_CS_Pin GPIO_PIN_0
#define IMU1_CS_GPIO_Port GPIOB
#define MAG_INT_Pin GPIO_PIN_10
#define MAG_INT_GPIO_Port GPIOA
#define core_LED_Pin GPIO_PIN_11
#define core_LED_GPIO_Port GPIOA
#define IMU2_CS_Pin GPIO_PIN_12
#define IMU2_CS_GPIO_Port GPIOA
#define DRDY_Pin GPIO_PIN_15
#define DRDY_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#define LED_TOGGLE HAL_GPIO_TogglePin(core_LED_GPIO_Port,core_LED_Pin)
#define LED(x) HAL_GPIO_WritePin(core_LED_GPIO_Port,core_LED_Pin,(GPIO_PinState)x)
#define DRDY_TOGGLE HAL_GPIO_TogglePin(DRDY_GPIO_Port,DRDY_Pin)
#define DRDY(x) HAL_GPIO_WritePin(DRDY_GPIO_Port,DRDY_Pin,(GPIO_PinState)x)
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
