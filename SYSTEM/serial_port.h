/*******************************************************************************
* 版权所有 :  
* 版本号   :  1.0
* 文件名   :  serial_port.h
* 生成日期 : 
* 作者     :  
* 功能说明 :  命令处理任务

*******************************************************************************/
#ifndef  __SERIAL_PORT_H__
#define  __SERIAL_PORT_H__
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "stm32f0xx.h"
#include "device_type.h"
/**********************************
*宏定义
***********************************/
//#define STM32F030F4P6

#ifdef STM32F030F4P6
#define UART    USART1
#define UART_CLKSRC   RCC_APB2Periph_USART1
#define UART_DMA_RX_CHANNEL   DMA1_Channel3
#define UART_DMA_TX_CHANNEL   DMA1_Channel2
#else
#define UART    USART2
#define UART_CLKSRC RCC_APB1Periph_USART2
#define UART_DMA_RX_CHANNEL   DMA1_Channel5
#define UART_DMA_TX_CHANNEL   DMA1_Channel4
#endif


#define UART_IO_PORT			GPIOA
#define	UART_IO_CLKSRC		RCC_AHBPeriph_GPIOA
#define UART_RX_PIN				GPIO_Pin_3
#define UART_TX_PIN				GPIO_Pin_2
#define UART_RX_AF_PIN_SOURCE GPIO_PinSource3
#define UART_TX_AF_PIN_SOURCE GPIO_PinSource2

#define UART_BAUDRATE   115200 

#define DMA_CLKSRC RCC_AHBPeriph_DMA1
//#define UART_DMA_RX_CHANNEL   DMA1_Channel3
//#define UART_DMA_TX_CHANNEL   DMA1_Channel2
////#define UART_DMA_RX_CHANNEL   DMA1_Channel5
////#define UART_DMA_TX_CHANNEL   DMA1_Channel4

/***********************************
* 全局变量
***********************************/

/***********************************
* 類型定義
***********************************/

/***********************************
* 外部函数
***********************************/
void UARTInit(uint8_t* p_rec_buf, uint32_t rec_num);
void UartSendNBytes (uint8_t *p_buf, uint32_t num);
int32_t GetUartReceiverResidualCnt(void);
#endif
