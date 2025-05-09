/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
 
  return ch;
}

#define APP_TX_DATA_SIZE  512
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

//多串口DMA数据格式化打印
void uart_printf(UART_HandleTypeDef *huart,const char *format, ...)
{
    va_list args;
    uint32_t length;
 
    va_start(args, format);
    length = vsnprintf((char *)UserTxBufferFS, APP_TX_DATA_SIZE, (char *)format, args);
    va_end(args);
		if(length>APP_TX_DATA_SIZE)
			length = APP_TX_DATA_SIZE;
    HAL_UART_Transmit_DMA(huart,UserTxBufferFS, length);   //只需要更改这儿就能一直到其他平台

}

#include "angle.h"
#include "icm45686_task.h"
#include "bmm350_task.h"
#include "stmflash.h"
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel1;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel2;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 1);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

uint8_t USART1_RX_BUF[512];


uint8_t data_buf[50];
void uart_send_IMU_data(uint16_t at_cmd)
{

	if(at_cmd==AT_Print_ASCII)
	{
		//模式1 串口ASCII码打印--满足VOFA+ FireWater格式
		if(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_READY)
			uart_printf(&huart1,"Data:%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\r\n",my_ahrs.Angle_Data.roll,my_ahrs.Angle_Data.pitch,my_ahrs.Angle_Data.yaw,my_ahrs.IMU_Data.gyro.x,my_ahrs.IMU_Data.gyro.y,my_ahrs.IMU_Data.gyro.z,my_ahrs.IMU_Data.acc.x,my_ahrs.IMU_Data.acc.y,my_ahrs.IMU_Data.acc.z,my_ahrs.IMU_Data.temperature,my_MMU.angle.yaw);
	}
	else if(at_cmd==AT_Print_HEX)
	{
		//模式2 串口HEX打印
		memset(data_buf,0xFF,50);
		/*帧头*/
		data_buf[0] = 0x5A;
		data_buf[1] = 0xA5;
		
		/*数据*/
		memcpy(&data_buf[2],(uint8_t *)&my_ahrs.Angle_Data,4*3);
		memcpy(&data_buf[14],(uint8_t *)&my_ahrs.IMU_Data,4*7);
		
		/*帧尾*/
		data_buf[42] = 0x0D;
		data_buf[43] = 0x0A;		
		
		if(HAL_DMA_GetState(&hdma_usart1_tx) == HAL_DMA_STATE_READY)
			HAL_UART_Transmit_DMA(&huart1,data_buf,44);
	}
	else
	{
		printf("change AT:%d\r\n",at_cmd);
	}
}


AT_CMD_MODE_TypeDef my_at_cmd;
void uart_AT_CMD_Init()
{
	my_at_cmd.uart_print_mode = AT_Print_HEX;//初始默认16进制打印
	my_at_cmd.mmu_mode = AT_NOT_MMU;//初始默认不使用磁力计
	my_at_cmd.int_pin_mode = AT_NOT_INT;//初始默认不使用中断
}
void uart_AT_cmd_decoder(uint8_t * data,uint16_t size)
{

	if(data[0] == 'A'&&data[1] == 'T'&&data[size-1]==0x0A)
	{
		/***确认为AT指令***/
		if(data[2]=='+')
		{
			switch(data[3])
			{
				case '1':
					my_at_cmd.uart_print_mode = AT_Print_ASCII;

          stmflash_write(FLASH_ADDR_BASE,(uint64_t*)&my_at_cmd.uart_print_mode,1);//写入FLASH
					break;
				case '2':
					my_at_cmd.uart_print_mode = AT_Print_HEX;

          stmflash_write(FLASH_ADDR_BASE,(uint64_t*)&my_at_cmd.uart_print_mode,1);//写入FLASH
					break;
			}
		}
		else
		{
			printf("AT_OK\r\n");
		}

		
	}
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if (huart->Instance == USART1)
    {
			uint8_t cnt = sizeof(USART1_RX_BUF) - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

			 /*解析协议*/
			uart_AT_cmd_decoder(USART1_RX_BUF,cnt);
			memset(USART1_RX_BUF, 0, cnt);
    }


}
/* USER CODE END 1 */
