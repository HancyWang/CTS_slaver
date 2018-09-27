#ifndef _RTC_H
#define _RTC_H
#include "stm32f0xx_rtc.h"

_Bool Set_RTC(uint8_t* pdata);
void Get_DataTime(RTC_DateTypeDef* RTC_DateStructure,RTC_TimeTypeDef* RTC_TimeStructure);
#endif //_RTC_H
