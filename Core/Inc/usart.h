/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"
#include "math.h"

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */

#define	AT_Print_ASCII  0x0001  //数据输出字符格式
#define	AT_Print_HEX  0x0002		//数据输出16进制格式
#define AT_USE_MMU 0x0003				//使用磁力计
#define AT_NOT_MMU 0x0004				//关闭磁力计
#define AT_USE_INT 0x0005				//使用中断引脚
#define AT_NOT_INT 0x0006				//关闭中断引脚

typedef struct
{
	uint32_t uart_print_mode; //数据打印模式
	uint32_t mmu_mode; //磁力计模式
	uint32_t int_pin_mode;	//中断引脚模式
}AT_CMD_MODE_TypeDef;


void uart_printf(UART_HandleTypeDef *huart,const char *format, ...);
void uart_send_IMU_data(uint16_t at_cmd);
extern AT_CMD_MODE_TypeDef my_at_cmd;
extern uint8_t USART1_RX_BUF[512];
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

