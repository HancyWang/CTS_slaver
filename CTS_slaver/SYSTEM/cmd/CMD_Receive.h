/*******************************************************************************
* 版权所有 :  
* 版本号   :  1.0
* 文件名   :  CMD_receive.h
* 生成日期 : 
* 作者     :  
* 功能说明 :  命令处理任务

*******************************************************************************/
#ifndef  __CMD_RECEIVE_H__
#define  __CMD_RECEIVE_H__
/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include <DataType.h>

typedef enum
{
	BUF1_LENGTH = 64,
	BUF2_LENGTH = 64
}BUF_LENGTH;

typedef struct
{
	UINT8  m_Buf2[BUF2_LENGTH];                //定义接收字符串二级缓冲区
	INT32  m_Buf2Wptr;                         //二级缓冲写指针
	INT32  m_Buf2Rptr;                         //二级缓冲读指针
	INT32  m_Cnt;
	INT32  m_Position;

	UINT8  m_Buf1[BUF1_LENGTH];                //定义接收字符串缓冲区  	  
}CMD_Receive;

void Init_Receive(CMD_Receive *p_RX);
void Buf2Write(CMD_Receive* p_RX, UINT8 data);
UINT8 Buf2Read(CMD_Receive* p_RX);
INT32 GetBuf2Length(CMD_Receive* p_RX);
void ReceiveData(CMD_Receive* p_RX);
#endif
/*************************************结束*************************************/
