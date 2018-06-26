/**
********************************************************************************
* ∞Êô‡£∫
* ƒ£øÈ√˚≥∆£∫hardware.c
* ƒ£øÈπ¶ƒ‹£∫
* ¥¥Ω®»’∆⁄£∫
* ¥¥ Ω® ’ﬂ£∫
* …˘    √˜£∫
********************************************************************************
**/

/***********************************
* Õ∑Œƒº˛
***********************************/

#include "hardware.h"
#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_pwr.h"
//#include "stm32f0xx_rtc.h"
#include "stm32f0xx_dma.h"

#include "delay.h"
#include "os_cfg.h"
#include "datatype.h"

#include "time.h"
#include "i2c.h"
#include "comm_task.h"
#include "protocol_module.h"
#include "key_power_on_task.h"
#include "app.h"
/**********************************
*∫Í∂®“Â
***********************************/
#define  SAMPLING_CNT 8
/***********************************
* »´æ÷±‰¡ø
***********************************/
unsigned short inner_adc_result[SAMPLING_CNT];
 uint16_t RegularConvData_Tab[2]; //µÁ—π÷µ(2.2VŒ™¡ŸΩÁ÷µ),∞¥º¸ƒ£ Ω

int16_t zero_point_of_pressure_sensor;
extern uint8_t parameter_buf[PARAMETER_BUF_LEN];


extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];
extern BOOL b_Is_PCB_PowerOn;

//extern uint8_t mode;
/***********************************
* æ÷≤ø±‰¡ø
***********************************/
//static //∆ΩƒÍµƒ‘¬∑›»’∆⁄±Ì
//const uint8_t mon_table[12]={31,28,31,30,31,30,31,31,30,31,30,31};
/***********************************
* æ÷≤ø∫Ø ˝
***********************************/


////Õ‚≤ø∫Ø ˝
//extern void I2C_SendByte(INT8U dat);
//extern void I2C_Stop(void); 
/**************************************************************
* ≥ı ºªØOSµŒ¥ ±÷”
// π”√TIM3
**************************************************************/
/*
void init_tim(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

  NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC;           // ◊‘∂Ø÷ÿ◊∞‘ÿºƒ¥Ê∆˜÷‹∆⁄µƒ÷µ(º∆ ˝÷µ) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	// ±÷”‘§∑÷∆µ ˝ 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//œÚ…œº∆ ˝ƒ£ Ω
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_ClearFlag(TIM3, TIM_FLAG_Update);			        // «Â≥˝“Á≥ˆ÷–∂œ±Í÷æ 
  TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM3, ENABLE);
}*/

/**************************************************************
* ≥ı ºªØOSµŒ¥ ±÷”
// π”√TIM16
**************************************************************/
void init_tim(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
	
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);//Õ‚…Ë ±÷”TIM16

  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
 
  TIM_TimeBaseStructure.TIM_Period = SystemCoreClock/OS_TICKS_PER_SEC;           // ◊‘∂Ø÷ÿ◊∞‘ÿºƒ¥Ê∆˜÷‹∆⁄µƒ÷µ(º∆ ˝÷µ) 
  TIM_TimeBaseStructure.TIM_Prescaler = 0;	// ±÷”‘§∑÷∆µ ˝ 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;			//œÚ…œº∆ ˝ƒ£ Ω
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);

//	TIM_BDTRInitTypeDef TIM_BDTRInitStruct;//–¬‘ˆ

////	TIM_BDTRInitStruct.TIM_OSSRState=TIM_OSSRState_Enable; 
////	TIM_BDTRInitStruct.TIM_OSSIState=TIM_OSSIState_Enable; 
//	TIM_BDTRInitStruct.TIM_LOCKLevel=TIM_LOCKLevel_1; 
//	TIM_BDTRInitStruct.TIM_DeadTime=0x00; 
//	TIM_BDTRInitStruct.TIM_Break=TIM_Break_Disable; 
//	TIM_BDTRInitStruct.TIM_BreakPolarity=TIM_BreakPolarity_Low; 
//	TIM_BDTRInitStruct.TIM_AutomaticOutput=ENABLE; 
//	TIM_BDTRConfig(TIM16,&TIM_BDTRInitStruct);
	
	
  TIM_ClearFlag(TIM16, TIM_FLAG_Update);			        // «Â≥˝“Á≥ˆ÷–∂œ±Í÷æ 
  TIM_ITConfig(TIM16,TIM_IT_Update,ENABLE);
  TIM_Cmd(TIM16, ENABLE);
}


/*******************************************************************************
** ∫Ø ˝√˚≥∆: Set_Led
** π¶ƒ‹√Ë ˆ: ªÒ»°∞¥º¸À˘∂‘”¶µƒƒ£ Ω
**  ‰°°  »Î: num-LED±‡∫≈£¨ON_OFF-¥Úø™ªÚπÿ±’
**  ‰°°  ≥ˆ: Œﬁ
** »´æ÷±‰¡ø: Œﬁ
** µ˜”√ƒ£øÈ: Œﬁ
*******************************************************************************/
void set_led(LED_ID id,BOOL ON_OFF)
{
	switch(id)
	{
		case LED_ID_GREEN:       //power green
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_GREEN_PWR_PIN);
			}
			else 
			{
				GPIO_SetBits(LED_PORT,LED_GREEN_PWR_PIN);
			}
			break;
		case LED_ID_YELLOW:     //power yellow
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_YELLOW_PWR_PIN);
			}
			else
			{
				GPIO_SetBits(LED_PORT,LED_YELLOW_PWR_PIN);
			}
			break;
		case LED_ID_MODE1:         //MODE1
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_MODE1_PIN);
			}
			else
			{
				GPIO_SetBits(LED_PORT,LED_MODE1_PIN);
			}
			break;
		case LED_ID_MODE2:   //MODE2
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_MODE2_PIN);
			}
			else
			{
				GPIO_SetBits(LED_PORT,LED_MODE2_PIN);
			}
			break;
		case LED_ID_MODE3:  //MODE3
			if(ON_OFF==TRUE)
			{
				GPIO_ResetBits(LED_PORT,LED_MODE3_PIN);
			}
			else
			{
				GPIO_SetBits(LED_PORT,LED_MODE3_PIN);
			}
			break;
		default:
			break;
	}
}

/**************************************************************
* ≥ı ºLEDπ‹Ω≈
**************************************************************/
void Init_LED(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//Õ∆ÕÏ ‰≥ˆ
	GPIO_InitStructure.GPIO_Pin = LED_GREEN_PWR_PIN|LED_YELLOW_PWR_PIN|LED_MODE1_PIN|LED_MODE2_PIN|LED_MODE2_PIN|LED_MODE3_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(LED_PORT, &GPIO_InitStructure);
	
	GPIO_SetBits(LED_PORT,LED_GREEN_PWR_PIN);
	GPIO_SetBits(LED_PORT,LED_YELLOW_PWR_PIN);
	GPIO_SetBits(LED_PORT,LED_MODE1_PIN);
	GPIO_SetBits(LED_PORT,LED_MODE2_PIN);
	GPIO_SetBits(LED_PORT,LED_MODE3_PIN);
	
//	if(mode==1)
//	{	
//		set_led(LED_ID_MODE1,TRUE); 
//	}
//	else if(mode==2)
//	{
//		set_led(LED_ID_MODE2,TRUE);   
//	}
//	else if(mode==3)
//	{
//		set_led(LED_ID_MODE3,TRUE);  
//	}
//	else
//	{
//		//do nothing
//	}
	
	
	//set_led(LED_ID_MODE1,TRUE);
}

void Init_PWRSAVE(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	//µÁ‘¥PWR_SAVE
	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
}

void Init_Valve()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}

void Init_Switch_Mode()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_Bat_Charge_Stby()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_PWR_EN()      //PA15
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_15);
}

void Init_PWR_ON_OFF()      //PA8
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Init_SWITCH_ON_OFF()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_USB_OE()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


//–£—Èpressure sensor
void Calibrate_pressure_sensor(int16_t* p_zeroPoint)
{
	//»Áπ˚ «∏∫—π‘ı√¥∞Ï£ø”¶∏√∂®“Â≥…int16?÷µª·≤ªª·±‰≥…∏∫ ˝£ø
	uint16_t arr[10]={0};
	uint16_t sum=0;
	delay_ms(50);
//	GPIO_SetBits(GPIOB,GPIO_Pin_10);
//	GPIO_SetBits(GPIOB,GPIO_Pin_11);
//	
//	delay_ms(2000); 
//	GPIO_SetBits(GPIOB,GPIO_Pin_11);
//	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	
	for(uint8_t i=0;i<10;i++)
	{
		arr[i]=ADS115_readByte(0x90);
		sum+=arr[i];
		delay_us(5);
	}
	*p_zeroPoint=sum/10;
}

/**************************************************************
* ≥ı ºªØ”≤º˛π‹Ω≈
**************************************************************/
void init_hardware()
{
	//GPIO_InitTypeDef  GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
#if 0
//	// ‰»ÎºÏ≤‚
//	GPIO_InitStructure.GPIO_Pin = EXP_DETECT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_Init(EXP_DETECT_PORT, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = KEY_DETECT_PIN;
//	GPIO_Init(KEY_DETECT_PORT, &GPIO_InitStructure);
	
//	//Õ∆ÕÏ ‰≥ˆ
//	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GREEN_LED_PORT, &GPIO_InitStructure);
	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
	
//	//µÁ‘¥PWR_SAVE
//	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
//	set_led(LED_CLOSE);
	#endif
	
	
	
	//≥ı ºªØUSB_OE
	Init_USB_OE();  //PA0
	
	//≥ı ºªØ pwr_on_off
	Init_PWR_ON_OFF();   //PA8
	
	//≥ı ºªØ switch_on_off 
	Init_SWITCH_ON_OFF(); //PB13
	
	//≥ı ºªØbat charge and standby
	Init_Bat_Charge_Stby();   //PA4,PA5
	
	//≥ı ºªØPWR_EN   
	Init_PWR_EN();      //PA15
	
	//≥ı ºªØpower saveπ‹Ω≈
	Init_PWRSAVE();      //PA12
	//≥ı ºªØ5∏ˆLEDµ∆
	Init_LED();         //PB2,PB3,PB4,PB5,PB6
	
	//≥ı ºªØµÁ¥≈∑ßøÿ÷∆π‹Ω≈
	Init_Valve();      //PB10,PB11
	
	//≥ı ºªØ∞¥º¸PB15,MODE—°‘Ò£¨÷Æ«∞µƒEARœÓƒø”√µƒ «PA4,PA4ø…“‘≤ª–Ë“™¡À
	Init_Switch_Mode();   //PB15
	
	//≥ı ºªØADC
	Init_ADC1();         //PB0£¨PA1(“™∫ÕPA15≈‰∫œ π”√)
//	//≥ı ºªØADS115,I2C
	Init_ADS115();       //PA9,PA10

	//Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
//	ADS115_enter_power_down_mode();
//	//≈‰÷√÷–∂œ
//	CfgWFI();
	//Ω¯»Îstopƒ£ Ω
	//EnterStopMode();
}




/**************************************************************
* ∞Âº∂”≤º˛◊ ‘¥øÿ÷∆
**************************************************************/
//∫ÙŒ¸ôzúyΩ≈†ÓëB
//BOOL get_exp_status(void)
//{
//	return GPIO_ReadInputDataBit(EXP_DETECT_PORT, EXP_DETECT_PIN);
//}

//∞¥º¸ôzúyΩ≈†ÓëB
//TRUE£∫∞¥œ¬
//FALSE:µØ∆
BOOL get_key_status(void)
{
	return GPIO_ReadInputDataBit(KEY_DETECT_PORT, KEY_DETECT_PIN);
}






////…Ë÷√LED÷∏ æµ∆
//void set_led(LED_COLOR color)
//{	
//	switch(color)
//	{
//		case LED_CLOSE:
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_BLUE:
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_ResetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_RED:
//			GPIO_ResetBits(RED_LED_PORT, RED_LED_PIN);
//			GPIO_SetBits(GREEN_LED_PORT, GREEN_LED_PIN);			
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		case LED_GREEN:
//			GPIO_ResetBits(GREEN_LED_PORT, GREEN_LED_PIN);
//			GPIO_SetBits(RED_LED_PORT, RED_LED_PIN);
////			GPIO_SetBits(BLUE_LED_PORT, BLUE_LED_PIN);
//			break;
//		default:
//			break;
//	}
//}
/**************************************************************
* µÁ≥ÿµÁ—πºÏ≤‚
**************************************************************/
//uint8_t get_bat_vol_per(void)
//{
//	return 0;

//}


/**************************************************************
* RTC…Ë÷√
**************************************************************/
//RTC RCC
#if 0
void init_rtc_rcc(void)
{
	//≈‰÷√ ±÷”
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
}

//rTC≈‰÷√
void config_rtc(void)
{
	RCC_LSICmd(ENABLE);//
	
	while(RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET);//µ»¥˝æÕ–˜
	
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
	
	RCC_RTCCLKCmd(ENABLE);
	RTC_WaitForSynchro();
}
#endif
#if 0
////RTC≥ı ºªØ
//void init_rtc(void)
//{
//	RTC_InitTypeDef RTC_InitStructure;
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);
//	
//	PWR_BackupAccessCmd(ENABLE);
//	
//	uint32_t dr = RTC_ReadBackupRegister(RTC_BKP_DR1);
//	
//	if(dr != 0x5051)//RTC_BKP_DR1
//	{
//		config_rtc();
//		
//		RTC_InitStructure.RTC_AsynchPrediv = 0x63;
//		RTC_InitStructure.RTC_SynchPrediv = 0x18f;
//		RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
//		
//		if(RTC_Init(&RTC_InitStructure) == ERROR)
//		{
//			//while(1);//≥ı ºªØ ß∞‹£¨–Ì–ﬁ∏ƒ
//			//»Ìº˛∏¥Œª
//			
//		}
//		
//		RTC_DateStrcuture.RTC_Year = 16;
//		RTC_DateStrcuture.RTC_Month = 12;
//		RTC_DateStrcuture.RTC_Date = 23;
//		RTC_TimeStrcuture.RTC_Hours = 12;
//		RTC_TimeStrcuture.RTC_Minutes = 34;
//		RTC_TimeStrcuture.RTC_Seconds = 56;
//		RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//		RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//		
//		RTC_WriteBackupRegister(RTC_BKP_DR1, 0x5051);
//	}
//	else
//	{
//		RCC_LSICmd(ENABLE);
//		RTC_WaitForSynchro();
//	}
//}

////≈–∂œ «∑Ò «»ÚƒÍ∫Ø ˝
////‘¬∑›   1  2  3  4  5  6  7  8  9  10 11 12
////»ÚƒÍ   31 29 31 30 31 30 31 31 30 31 30 31
////∑«»ÚƒÍ 31 28 31 30 31 30 31 31 30 31 30 31
////year:ƒÍ∑›
////∑µªÿ÷µ:∏√ƒÍ∑› «≤ª «»ÚƒÍ.1, «.0,≤ª «
//uint8_t Is_Leap_Year(uint16_t year)
//{			  
//	if(year%4==0) //±ÿ–Îƒ‹±ª4’˚≥˝
//	{ 
//		if(year%100==0) 
//		{ 
//			if(year%400==0)return 1;//»Áπ˚“‘00Ω·Œ≤,ªπ“™ƒ‹±ª400’˚≥˝ 	   
//			else return 0;   
//		}else return 1;   
//	}else return 0;	
//}	

////ÕÚ¿˙ƒÍ◊™ªØ
//void convert_rtc(_calendar_obj* calendar, uint32_t rtc)
//{
//	uint32_t temp = 0;
//	uint16_t temp1 = 0;
//	static uint16_t daycnt=0;
//	
//	temp = rtc/86400;   //µ√µΩÃÏ ˝(√Î÷” ˝∂‘”¶µƒ)
//	if(daycnt!=temp)//≥¨π˝“ªÃÏ¡À
//	{	  
//		daycnt=temp;
//		temp1 = 1970;	//¥”1970ƒÍø™ º
//		while(temp>=365)
//		{				 
//			if(Is_Leap_Year(temp1))// «»ÚƒÍ
//			{
//				if(temp>=366)temp-=366;//»ÚƒÍµƒ√Î÷” ˝
//				else break;  
//			}
//			else temp-=365;	  //∆ΩƒÍ 
//			temp1++;  
//		}   
//		calendar->w_year=temp1;//µ√µΩƒÍ∑›
//		temp1=0;
//		while(temp>=28)//≥¨π˝¡À“ª∏ˆ‘¬
//		{
//			if(Is_Leap_Year(calendar->w_year)&&temp1==1)//µ±ƒÍ «≤ª «»ÚƒÍ/2‘¬∑›
//			{
//				if(temp>=29)temp-=29;//»ÚƒÍµƒ√Î÷” ˝
//				else break; 
//			}
//			else 
//			{
//				if(temp>=mon_table[temp1])temp-=mon_table[temp1];//∆ΩƒÍ
//				else break;
//			}
//			temp1++;  
//		}
//		calendar->w_month=temp1+1;	//µ√µΩ‘¬∑›
//		calendar->w_date=temp+1;  	//µ√µΩ»’∆⁄ 
//	}
//	temp=rtc%86400;     		//µ√µΩ√Î÷” ˝   	   
//	calendar->hour=temp/3600;     	//–° ±
//	calendar->min=(temp%3600)/60; 	//∑÷÷”	
//	calendar->sec=(temp%3600)%60; 	//√Î÷”
//}

////…Ë÷√RTC
//void set_rtc(uint32_t rtc)
//{
//	RTC_DateTypeDef RTC_DateStrcuture;
//	RTC_TimeTypeDef RTC_TimeStrcuture;

//	_calendar_obj calendar;
//	
//	convert_rtc(&calendar, rtc);
//	
//	RTC_DateStrcuture.RTC_Year = calendar.w_year;
//	RTC_DateStrcuture.RTC_Month = calendar.w_month;
//	RTC_DateStrcuture.RTC_Date = calendar.w_date;
//	RTC_TimeStrcuture.RTC_Hours = calendar.hour;
//	RTC_TimeStrcuture.RTC_Minutes = calendar.min;
//	RTC_TimeStrcuture.RTC_Seconds = calendar.sec;
//	
//	RTC_SetDate(RTC_Format_BIN, &RTC_DateStrcuture);//RTC_Format_BIN
//	RTC_SetTime(RTC_Format_BIN, &RTC_TimeStrcuture);
//}

//uint32_t get_rtc(void)
//{
//#ifndef RTC_TR_RESERVED_MASK
//#define RTC_TR_RESERVED_MASK    ((uint32_t)0x007F7F7F)	
//#endif
//	
//	return (uint32_t)(RTC->TR & RTC_TR_RESERVED_MASK); 
//}
#endif
/**************************************************************
* FLASH◊xåë…Ë÷√
**************************************************************/
//±£¥Ê“ªÌìîµì˛
BOOL save_one_page_to_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
	return TRUE;
}
//◊x“ªÌìîµì˛
void read_one_page_from_flash(uint32_t Address, uint8_t* buf, uint16_t len)
{
}
//±£¥Ê“ªÇÄ∞Î◊÷£¨É…◊÷πù
BOOL save_half_word_to_flash(uint32_t Address, uint16_t data)
{
	return TRUE;
}
//◊x“ªÇÄ∞Î◊÷£¨É…◊÷πù
void read_half_word_from_flash(uint32_t Address, uint16_t* pdata)
{
}
/**************************************************************
* /EEPROM◊xåë…Ë÷√
**************************************************************/
//±£¥Ê“ªÇÄ∞Î◊÷îµΩM
BOOL save_half_word_buf_to_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
	return TRUE;
}
//◊x“ªÇÄ∞Î◊÷îµΩM
void read_half_word_buf_from_eeprom(uint32_t Address, uint16_t* buf, uint16_t len)
{
}

void Key_WakeUp_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  //GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void Init_ADC1(void)
{
	GPIO_InitTypeDef    GPIO_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;
	ADC_InitTypeDef     ADC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);  //–¬‘ˆ
	
	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;    //PA1
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure);	
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;    //PB0
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , ENABLE);		
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	
	ADC_DeInit(ADC1);//ADC???'??Ë?ÈË??		

  DMA_DeInit(DMA1_Channel1);	/* DMA1 Channel1 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_Address;//ÌaÈËµ??∑
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RegularConvData_Tab;//?˙'?µ??∑
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//ÌaÈË◊˜?aÍy?Y'?Í?µ?‡'?'
  DMA_InitStructure.DMA_BufferSize = 2;//
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ÌaÈËµ??∑??'??˜2?±?
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//?˙'?µ??∑??'??˜2?±?
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//Íy?Y?Ì?Ë?a16??
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//Íy?Y?Ì?Ë?a16??
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA_PriorityÈË?®DMAÌ®µ‡xµ?ËÌ?tÛ??Ë??
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMAÌ®µ‡x??ÛDÈË???a?˙'?µ??˙'?'?Í?
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
		  
	DMA_Cmd(DMA1_Channel1, ENABLE);/* DMA1 Channel1 enable */			
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular); /* Enable ADC_DMA */	
  ADC_DMACmd(ADC1, ENABLE);  
	
		
	ADC_StructInit(&ADC_InitStructure);//3?Í??ØADC?·11
	
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12?????Ë
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; //1??®??Í?◊∞??1§◊˜?˙·?D???Í?
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None; 
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//Íy?Y?????aÛÚ????
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Backward; //ADCµ?È®?Ë∑??Ú
  ADC_Init(ADC1, &ADC_InitStructure); 
	
  ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_239_5Cycles); /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */  
//  ADC_ChannelConfig(ADC1, ADC_Channel_4 , ADC_SampleTime_239_5Cycles); /* Convert the ADC1 Channel 11 with 239.5 Cycles as sampling time */  
	ADC_ChannelConfig(ADC1, ADC_Channel_8 , ADC_SampleTime_239_5Cycles);

//  ADC_ChannelConfig(ADC1, ADC_Channel_Vrefint ,ADC_SampleTime_239_5Cycles); 
//  ADC_VrefintCmd(ENABLE);
//	
//	ADC_ChannelConfig(ADC1, ADC_Channel_TempSensor ,ADC_SampleTime_239_5Cycles);
//	ADC_TempSensorCmd(ENABLE);
//	
//	ADC_ChannelConfig(ADC1, ADC_Channel_Vbat ,ADC_SampleTime_239_5Cycles);
//	ADC_VbatCmd(ENABLE);
	
	ADC_GetCalibrationFactor(ADC1); /* ADC Calibration */  
  ADC_Cmd(ADC1, ENABLE);  /* Enable ADCperipheral[PerIdx] */	  
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN)); /* Wait the ADCEN falg */
  ADC_StartOfConversion(ADC1); /* ADC1 regular Software Start Conv */ 
}



//uint16_t Adc_Switch(uint32_t ADC_Channel)
//{
//	
//	//while(ADC_GetFlagStatus(ADC1,ADC_FLAG_ADEN)==RESET); 
//	//≈‰÷√ADC≤…”√µƒÕ®µ¿∫Õ≤…—˘÷‹∆⁄
//	ADC_ChannelConfig(ADC1,ADC_Channel,ADC_SampleTime_239_5Cycles);  
//	
//	//Delay_ms(10);
//	//»Ìº˛∆Ù∂ØADC◊™ªª
//  ADC_StartOfConversion(ADC1);  

//	//µ»¥˝ADC◊™ªªÕÍ≥…
//	while(!ADC_GetFlagStatus(ADC1,ADC_FLAG_EOC))
//	{
//		//do nothing
//	}
//  return ADC_GetConversionValue(ADC1) ; 
//}

