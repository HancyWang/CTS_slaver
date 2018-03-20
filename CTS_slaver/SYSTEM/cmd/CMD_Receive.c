/*******************************************************************************
* 项目编号 :  
* 版本号   :  1.0
* 文件名   :  CMD_receive.c
* 生成日期 :  
* 作者     :  
* 功能说明 :  命令处理任务
*******************************************************************************/
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "CMD_receive.h"
#include "serial_port.h"
/*******************************************************************************
** 函数名称: Init_Receive
** 功能描述: 初始化Receive参数
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void Init_Receive(CMD_Receive* p_RX)
{
    p_RX->m_Buf2Rptr = 0;//二级缓冲读指针
    p_RX->m_Buf2Wptr = 0;//二级缓冲写指针
    p_RX->m_Cnt = BUF1_LENGTH;
    p_RX->m_Position = 0;
}      
/*******************************************************************************
** 函数名称: Uart0ReceiveBufferTwoWrite
** 功能描述: 往二级缓冲区写入一个数据
** 输　入:  data: 要写入的数据
** 输　出: 无
** 全局变量: recevieBufferTwoWritePoint 写指针
** 调用模块: 无
*******************************************************************************/
void Buf2Write(CMD_Receive* p_RX, UINT8 data)
{
	p_RX->m_Buf2[p_RX->m_Buf2Wptr] = data;
	//写指针加一
	p_RX->m_Buf2Wptr++;
	if(BUF2_LENGTH == p_RX->m_Buf2Wptr)
	{
     p_RX->m_Buf2Wptr = 0;
	}
}

/*******************************************************************************
** 函数名称: Buf2Read
** 功能描述: 从二级缓冲区读出一个数据
** 输　  入: 无
** 输　  出: 从缓冲区读出数据
** 全局变量: recevieBufferTwoReadPoint 写指针
** 调用模块: 无
*******************************************************************************/
UINT8 Buf2Read(CMD_Receive* p_RX)
{
	UINT8 data;
        data = p_RX->m_Buf2[p_RX->m_Buf2Rptr];
	//读指针加一
        p_RX->m_Buf2Rptr++;
        if(BUF2_LENGTH == p_RX->m_Buf2Rptr)
	{
                p_RX->m_Buf2Rptr = 0;
	}
	return data;
}

/*******************************************************************************
** 函数名称: GetBuf2Length
** 功能描述: 取二级缓冲区包含数据的长度
** 输　  入: 无
** 输　  出: 缓冲区数据长度
** 全局变量: recevieBufferTwoWritePoint 写指针
**           receiveBufferTwoReadPoint  读指针
** 调用模块: 无
*******************************************************************************/
INT32 GetBuf2Length(CMD_Receive* p_RX)
{
	INT32 length;
        if(p_RX->m_Buf2Wptr >= p_RX->m_Buf2Rptr)  //写指针 > 读指针
	{
                length = p_RX->m_Buf2Wptr - p_RX->m_Buf2Rptr;
	}
	else
	{
                length = BUF2_LENGTH + p_RX->m_Buf2Wptr - p_RX->m_Buf2Rptr;
	}  
	return length;
}

/*******************************************************************************
** 函数名称: ReceiveData
** 功能描述: 串口接收数据
** 输　  入: 无
** 输  　出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void ReceiveData(CMD_Receive* p_RX)
{
	INT32 i;
	INT32 cnt;                     //串口接收的数据个数
	INT32 cnt_tmp;
	//取串口pdc缓冲区剩余计数，计算接收到的数据个数
	cnt_tmp = GetUartReceiverResidualCnt();	
  if(p_RX->m_Cnt >= cnt_tmp)
	{
			cnt = p_RX->m_Cnt - cnt_tmp;
	}
	else
	{
			cnt = p_RX->m_Cnt + BUF1_LENGTH - cnt_tmp;
	}
	p_RX->m_Cnt = cnt_tmp;

	//将数据移入处理的数组
	for(i = 0; i < cnt; i++)
	{
		if(p_RX->m_Position + i >= BUF1_LENGTH)						
		{           
			//写入二级缓冲区
			Buf2Write(p_RX, p_RX->m_Buf1[p_RX->m_Position + i-BUF1_LENGTH]);
		}
		else
		{           
			//写入二级缓冲区
			Buf2Write(p_RX, p_RX->m_Buf1[p_RX->m_Position + i]);
		}
	}

	//串口pdc接收数据，存放的位置
	p_RX->m_Position += cnt;
	if(p_RX->m_Position >= BUF1_LENGTH)
	{
			p_RX->m_Position -= BUF1_LENGTH;
	}   
}
/*************************************结束*************************************/


