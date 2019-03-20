#include "stm32f0xx_conf.h"
#include "common.h"
#include "delay.h"

INT16U g_uwDelayTime_ms = 0;
//static INT8U  g_ubMultiple_us = 0;
//static INT16U g_uwMultiple_ms = 0;


void Delay(INT16U mTime)
{
	g_uwDelayTime_ms = mTime;
	while(g_uwDelayTime_ms != 0);
}




// 微秒定时
void Delay_us(INT16U time)
{
	delay_us(time);

}


// 毫秒定时
void Delay_ms(INT16U time)
{
	delay_ms(time);

}




