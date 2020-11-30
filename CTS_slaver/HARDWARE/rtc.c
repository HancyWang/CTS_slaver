#include "rtc.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
#include "delay.h"
//#include "hardware.h"

volatile unsigned int AsynchPrediv = 0, SynchPrediv = 0;



void Get_DataTime(RTC_DateTypeDef* RTC_DateStructure,RTC_TimeTypeDef* RTC_TimeStructure)
{ 
	//先读time,在读date，如果反过来会导致跨零点是，time ok而date还跟以前一样
	RTC_GetTime(RTC_Format_BIN,RTC_TimeStructure);
	RTC_GetDate(RTC_Format_BIN,RTC_DateStructure);
}

char chage_to_RCT_month(char month)
{
	if(month==10)
	{
		return 0x10;
	}
	else if(month==11)
	{
		return 0x11;
	}
	else if(month==12)
	{
		return 0x12;
	}
	else
	{
		return month;
	}
}

void RTC_Config(void)
{
	/* Enable the PWR clock */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

  /* Allow access to RTC */
  PWR_BackupAccessCmd(ENABLE);
	
	 /* Enable the LSE OSC */
  RCC_LSEConfig(RCC_LSE_ON);

  /* Wait till LSE is ready */  
  while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {
  }

  /* Select the RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
  
  SynchPrediv = 0xFF;
  AsynchPrediv = 0x7F;
	
	/* Enable the RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC APB registers synchronisation */
  RTC_WaitForSynchro();
}

_Bool Set_RTC(uint8_t* pdata)
//void Init_RTC()
{
	
	if(pdata[0]!=0xFF)
	{
		return 0;
	}
	if(pdata[1]!=11)
	{
		return 0;
	}
	
	RTC_InitTypeDef RTC_InitStructure;
	RTC_DateTypeDef RTC_DateStructure;
	RTC_TimeTypeDef RTC_TimeStructure; 
	
	PWR_BackupAccessCmd(ENABLE);
	//if(RTC_ReadBackupRegister(RTC_BKP_DR1)!=0x1234)
	{
		RTC_Config();
		
		RTC_InitStructure.RTC_HourFormat=RTC_HourFormat_24;
		RTC_InitStructure.RTC_SynchPrediv=SynchPrediv;
		RTC_InitStructure.RTC_AsynchPrediv=AsynchPrediv;
		
//		RTC_DateStructure.RTC_Year=18;
//		RTC_DateStructure.RTC_Month=RTC_Month_September;
//		RTC_DateStructure.RTC_Date=25;
//		RTC_DateStructure.RTC_WeekDay=RTC_Weekday_Tuesday;
//		RTC_TimeStructure.RTC_Hours=9;
//		RTC_TimeStructure.RTC_Minutes=58;
//		RTC_TimeStructure.RTC_Seconds=25;
		
		RTC_DateStructure.RTC_Year=pdata[5];
//		RTC_DateStructure.RTC_Month=chage_to_RCT_month(pdata[6]);
		RTC_DateStructure.RTC_Month=pdata[6];
		RTC_DateStructure.RTC_Date=pdata[7];
	//	RTC_DateStructure.RTC_WeekDay=RTC_Weekday_Tuesday;
		RTC_TimeStructure.RTC_Hours=pdata[8];
		RTC_TimeStructure.RTC_Minutes=pdata[9];
		RTC_TimeStructure.RTC_Seconds=pdata[10];
		
		
		if(RTC_Init(&RTC_InitStructure)== ERROR)
		{
			while(1);   //初始化失败
		}
		
		RTC_SetDate(RTC_Format_BIN,&RTC_DateStructure);
		RTC_SetTime(RTC_Format_BIN,&RTC_TimeStructure);
	
//		RTC_WriteBackupRegister(RTC_BKP_DR1,0x1234);  //这个不需要
#if 0
		//这里发现两个问题:
		//1.读取时间不能马上就读取，要延时几秒才能读取
		//2. if语句是个bug：如果碰到 跨年，月，日，时分秒等，这个语句就不成立
//		while(1)   
//		{
//			delay_ms(2000);
//			delay_ms(2000);
//			delay_ms(2000);
//			Get_DataTime(&RTC_DateStructure,&RTC_TimeStructure);
//			
////			if(RTC_DateStructure.RTC_Year==pdata[5]&&RTC_DateStructure.RTC_Month==chage_to_RCT_month(pdata[6])
////				&&RTC_DateStructure.RTC_Date==pdata[7]&&RTC_TimeStructure.RTC_Hours==pdata[8]
////			&&RTC_TimeStructure.RTC_Minutes==pdata[9]&&RTC_TimeStructure.RTC_Seconds==pdata[10])
////			{
////				break;
////			}
//		}
#endif

	}
//	else
//	{
//		RTC_WaitForSynchro();
//	}
	return 1;
}





