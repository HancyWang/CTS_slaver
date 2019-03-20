/*******************************************************************************
* 项目编号 :  
* 版本号   :  1.0
* 文件名   :  serial_port.c
* 生成日期 :  
* 作者     :  
* 功能说明 :  命令处理任务
*******************************************************************************/
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "serial_port.h"
#include "hardware.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_dma.h"
#include "datatype.h"

/*******************************************************************************
*                                 局部變量
*******************************************************************************/

/*******************************************************************************
*                                 全局變量
*******************************************************************************/

/*******************************************************************************
*                                 函數定義
*******************************************************************************/
/*******************************************************************************
* 函数名称 : UARTInit
* 功能描述 : 初始化串口
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void UARTInit(uint8_t* p_rec_buf, uint32_t rec_num)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHBPeriphClockCmd(UART_IO_CLKSRC, ENABLE);  //使能GPIOA的时钟
	RCC_AHBPeriphClockCmd(DMA_CLKSRC, ENABLE);
	
	#ifdef STM32F030F4P6
	RCC_APB2PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	#else
	RCC_APB1PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	#endif
	
//	RCC_APB2PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
//	//RCC_APB1PeriphClockCmd(UART_CLKSRC, ENABLE);//使能USART的时钟
	
	/* USART1的端口配置 */
	GPIO_PinAFConfig(UART_IO_PORT, UART_RX_AF_PIN_SOURCE, GPIO_AF_1);//配置PA9成第二功能引脚	TX
	GPIO_PinAFConfig(UART_IO_PORT, UART_TX_AF_PIN_SOURCE, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

	GPIO_InitStructure.GPIO_Pin = UART_RX_PIN | UART_TX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(UART_IO_PORT, &GPIO_InitStructure);

	/* USART1的基本配置 */
	USART_InitStructure.USART_BaudRate = UART_BAUDRATE;              //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART, &USART_InitStructure);	
	#if 1	
	//DMA 發送配置
	DMA_DeInit(UART_DMA_TX_CHANNEL);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART->TDR);//外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)0;//内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//外设作为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = (uint32_t)0;//
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器不变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority设定DMA通道x的软件优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(UART_DMA_TX_CHANNEL, &DMA_InitStructure);

		//DMA 接收配置
	DMA_DeInit(UART_DMA_RX_CHANNEL);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&UART->RDR);//外设地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)p_rec_buf;//内存地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//外设作为数据传输的来源
  DMA_InitStructure.DMA_BufferSize = rec_num;//
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器不变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_Priority设定DMA通道x的软件优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(UART_DMA_RX_CHANNEL, &DMA_InitStructure);
	
	USART_DMACmd(UART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	
	DMA_Cmd(UART_DMA_RX_CHANNEL, ENABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, ENABLE);
	#endif	
	USART_Cmd(UART, ENABLE);

}

/*******************************************************************************
* 函数名称 : DMA_ReWorkCfg
* 功能描述 : 对于一个已经初始化的DMA通道, 重新配置其工作
* 输入参数 : dma_Channel_x  指向DMA通道的指针
*            mem_address    重新配置的内存地址
*            num            重新传输的数目
*******************************************************************************/
static void DMA_ReWorkCfg (DMA_Channel_TypeDef* dma_Channel_x, uint32_t mem_address, uint32_t num)
{
	dma_Channel_x->CCR &= 0xFFFFFFFE;                /* 先禁止DMAx通道 */

	dma_Channel_x->CMAR = mem_address;               /* 重新配置内存地址 */

	dma_Channel_x->CNDTR = num;                      /* 重新传输数目 */

	dma_Channel_x->CCR |= 0x00000001;                /* 使能DMAx通道 */
}

/*******************************************************************************
* 函数名称 : UartSendNBytes
* 功能描述 : 通过DMA机制从串口发送数据
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 : 无
*******************************************************************************/
void UartSendNBytes (uint8_t *p_buf, uint32_t num)
{
	DMA_ReWorkCfg(UART_DMA_TX_CHANNEL, (uint32_t)p_buf, num);
}

/*******************************************************************************
* 函数名称 : GetUartReceiverResidualCnt
* 功能描述 : 取串口pdc缓冲区剩余计数
* 输入参数 : 无
* 输出参数 : 无
* 函数返回 :串口pdc缓冲区剩余计数 
*******************************************************************************/
int32_t GetUartReceiverResidualCnt(void)
{
	return(DMA_GetCurrDataCounter(UART_DMA_RX_CHANNEL));
}



