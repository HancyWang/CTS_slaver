/*******************************************************************************
* 文件名   :  fifo.h
* 生成日期 :  
* 作者     :  
* 功能说明 :  数据发送缓冲区
*******************************************************************************/
#ifndef  __FIFO_H__
#define  __FIFO_H__

/*******************************************************************************
*                                 头文件包含
*******************************************************************************/
#include "datatype.h"

typedef struct {
	UINT16 gFifoWrPtr;
	UINT16 gFifoRdPtr;
	UINT16 gFifoCnt;
	UINT16 gFifoLen;
	UINT8* gFifoBuffer;
}FIFO_TYPE;

void fifoInit(FIFO_TYPE* ptr, UINT8* buffer, UINT16 lenght);
void fifoReset(FIFO_TYPE* ptr);
UINT16 fifoCnt(FIFO_TYPE* ptr);
BOOL fifoIsFull(FIFO_TYPE* ptr);
BOOL fifoWriteData(FIFO_TYPE* ptr, UINT8 *array, UINT16 length); 
UINT16 fifoReadData(FIFO_TYPE* ptr, UINT8 *array, UINT16 length);
#endif
/*************************************结束*************************************/
