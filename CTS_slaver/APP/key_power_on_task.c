

/**
********************************************************************************
* 版權：
* 模块名称：key_power_on_task.c
* 模块功能：
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/

#include "app.h"
#include "datatype.h"
#include "hardware.h"
#include "fifo.h"
#include "key_power_on_task.h"
#include "protocol_module.h"

#include "i2c.h"
#include "Motor_pwm.h"
#include "stm32f0xx_pwr.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"
#include "stm32f0xx_dma.h"
#include "serial_port.h"
#include "CMD_receive.h"
#include "app.h"
#include "delay.h"
#include "comm_task.h"
#include "iwtdg.h"
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/

/***********************************
* 局部变量
***********************************/
extern uint32_t os_ticks;
extern CMD_Receive g_CmdReceive;  // 命令接收控制对象

extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];
extern CHCKMODE_OUTPUT_PWM state;

extern PWM_STATE pwm1_state;
extern PWM_STATE pwm2_state;
extern PWM_STATE pwm3_state;

extern uint8_t mode;
extern BOOL b_switch_mode_changed;
extern BOOL b_palm_checked;
extern uint32_t detectPalm_cnt;
extern uint32_t noPalm_cnt;
extern BOOL b_bat_detected_ok;

extern LED_STATE led_state;
extern BEEP_STATE beep_state;

extern BOOL b_Motor_Ready2Shake;
extern BOOL	 b_Motor_shake;
extern BOOL b_Palm_check_complited;

//extern BOOL led_bink_timing_flag;
//extern BOOL beep_timing_flag;
//extern uint32_t prev_ledBlink_os_tick;
//extern uint32_t prev_beep_os_tick;
extern uint16_t led_bink_cnt;
extern uint16_t beep_cnt;
extern uint16_t delay_cnt;

extern BOOL b_stop_current_works;
extern BOOL b_no_hand_in_place;
extern BOOL b_end_of_treatment;

extern USB_CHARGING_STATE usb_charging_state;
extern BOOL b_release_gas;
extern BOOL led_bink_timing_flag;
extern BOOL	beep_timing_flag;
extern BOOL	usb_charge_timing_flag;
extern BOOL	key_Press_or_Release_timing_flag;
extern BOOL	b_releaseGas_timing_flag;
extern BOOL	b_timing_flag;
//extern BOOL b_detect_palm;
extern uint32_t prev_releaseGas_os_tick;
extern uint32_t prev_ledBlink_os_tick;
extern uint32_t prev_keyPressOrRelease_os_tick;
extern uint32_t prev_usbCharge_os_tick;
extern uint32_t prev_beep_os_tick;
extern uint32_t prev_WaitBeforeStart_os_tick;
extern uint32_t prev_PWM1_os_tick;
extern uint32_t prev_PWM2_os_tick;
extern uint32_t prev_PWM3_os_tick;
extern uint32_t prev_PWM4_os_tick;
extern uint32_t prev_PWM5_os_tick;
extern uint16_t checkPressAgain_cnt;
extern uint8_t wait_cnt;
extern BOOL waitBeforeStart_timing_flag;

extern BOOL	b_self_test;
extern BOOL key_self_test_timing_flag;
extern uint32_t	prev_selfTest_os_tick;
	
extern BOOL	b_usb_push_in;
extern BOOL	b_usb_pull_up;
extern BOOL	b_stop_current_works;
extern uint8_t led_beep_ID;

extern SELF_TEST_STATE self_tet_state;
extern LED_IN_TURN_STATE led_In_Turn_state;
extern BOOL b_LED_ON_in_turn;

extern BOOL b_check_bnt_release;
extern uint8_t selfTest_delay_Cnt;
extern uint8_t nLED_ON_in_turn;
extern uint8_t inflate_cnt;
extern uint8_t hold_cnt;
extern uint8_t deflate_cnt;

extern uint8_t sample_cnt;
extern uint32_t sample_sum;

extern uint8_t deflate_cnt;
extern uint16_t selfTest_inflate_record_1;
extern uint16_t selfTest_inflate_record_2;

extern uint16_t selfTest_hold_record_1;
extern uint16_t selfTest_hold_record_2;
extern uint16_t selfTest_deflate_record_1;
extern uint16_t selfTest_deflate_record_2;
extern uint8_t selfTest_fail_Cnt;
extern uint8_t selfTest_fail_period_H;
extern uint8_t selfTest_fail_period_L;
extern uint8_t selfTest_end_Cnt;

extern BOOL b_detect_hand_before_system_running;
extern uint16_t wait_between_total_cnt;
extern uint8_t value;
//extern BOOL b_start_powerOn_check;
// BOOL b_KeyWkUP_InterrupHappened=FALSE;
// BOOL b_usb_intterruptHappened=FALSE;
//KEY值，这里点按为确认蓝牙连接
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;


USB_DETECT_STATE usb_detect_state=USB_NOT_DETECT;

MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;


//SWITCH_MODE prev_switch_mode=SWITCH_MODE1;

//extern uint8_t OUTPUT_FINISH;
BOOL b_Is_PCB_PowerOn=FALSE;
//BOOL b_usb_charge_bat=FALSE;

//volatile KEY_STATE key_state=KEY_STOP_MODE;
KEY_STATE key_state=KEY_UPING;

extern uint16_t RegularConvData_Tab[2];
/***********************************
* 局部函数
***********************************/

/*******************************************************************************
** 函数名称: EnterStopMode
** 功能描述: 进入低功耗模式
** 输　  入: 无
** 输　  出: 无
** 全局变量: 无
** 调用模块: 无
*******************************************************************************/
void CfgWFI()
{
	//时钟使能
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF,ENABLE);  
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	
//	//USB充电，USB_0E,PA0  
	//外部按键GPIOA初始化,PA8 
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);

//	//将EXTI0指向PA0  
	//将EXTI8指向PA8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8);  
//	//EXTI0中断线配置
		//EXTI8中断线配置
	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);  
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;   //配置成上升沿和下降沿都可以触发中断
	EXTI_Init(&EXTI_InitStructure);

//	//EXTI0中断向量配置  
	//EXTI8中断向量配置  
	NVIC_InitTypeDef NVIC_InitStructure;  
//	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
	
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x02;
	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}


//////检测PA0(WAKE_UP)是否被按下
////检测PA8(power_on_off)是否被按下
//BOOL Check_wakeUpKey_pressed(void)
//{
//	while(TRUE)
//	{
//		//读取PA8的电平
//		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
//		{
//			//b_Interrupt_key_wakeUp=TRUE;
//			delay_ms(30);

//			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
//			{
//				while(TRUE)
//				{
//					delay_ms(10);
//					if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==1)
//					{
//						return TRUE;
//					}
//				}
//			}
//		}
//		else
//		{
//			return FALSE;
//		}
//	}
//}

////PA0,判断USB是插入还是拔出
USB_DETECT_STATE Check_USB_pull_or_push()
{
	while(TRUE)
	{
		//读取PA0的电平
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
		{
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			Motor_PWM_Freq_Dudy_Set(4,100,0);
			Motor_PWM_Freq_Dudy_Set(5,100,0);
			set_led(LED_ID_MODE1,FALSE);
			set_led(LED_ID_MODE2,FALSE);
			set_led(LED_ID_MODE3,FALSE);
			set_led(LED_ID_GREEN,FALSE);
			//delay_ms(500);  //给500ms的稳定时间
			//set_led(LED_ID_YELLOW,TRUE);  //debug
			
			uint8_t cnt=0;
			//循环5次，如果5次都是高电平，说明已经稳定的插入USB了
			for(uint8_t i=0;i<5;i++)
			{
				delay_ms(5);
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
				{
					cnt++;
				}
			}
			
			if(cnt==5) 
			{
				cnt=0;
				//set_led(LED_ID_YELLOW,TRUE);  //debug
				return USB_PUSH_IN;
			}
			else
			{
				return USB_NOT_DETECT;
			}
		}
		else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
		{
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			Motor_PWM_Freq_Dudy_Set(4,100,0);
			Motor_PWM_Freq_Dudy_Set(5,100,0);
			set_led(LED_ID_MODE1,FALSE);
			set_led(LED_ID_MODE2,FALSE);
			set_led(LED_ID_MODE3,FALSE);
			set_led(LED_ID_GREEN,FALSE);
			//delay_ms(500);  //给500ms的稳定时间
			
			uint8_t cnt=0;
			for(uint8_t i=0;i<5;i++)
			{
				delay_ms(5);
				if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)
				{
					cnt++;
				}
			}
			
			if(cnt==5) 
			{
				cnt=0;
				return USB_PULL_UP;
			}
			else
			{
				return USB_NOT_DETECT;
			}
		}
		else
		{
			return USB_NOT_DETECT;
		}
	}
}

 

void EXTI0_1_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0)!=RESET)  
	{ 
//		b_usb_intterruptHappened=TRUE;
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1) //高电平表示插入了USB,上拉
		{
			//处理USB插入
//			b_usb_push_in=TRUE;
			usb_detect_state=USB_PUSH_IN;
		}
		else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)  //低电平表示拔出了USB，下拉
		{
			//处理USB拔出
			//b_usb_pull_up=TRUE;
			usb_detect_state=USB_PULL_UP;
		}
		else
		{
			//do nothing
		}
		
		//新增，明天验证一下, //意思是判断了上拉下拉之后就不再进入这个函数，而是去
		//usb_charge_battery验证USB插拔是否有效
//		b_usb_intterruptHappened=FALSE;
	}  
	EXTI_ClearFlag(EXTI_Line0);
}

void EXTI4_15_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)  
	{ 
		//1.USB没插入的时候才响应
		//2.没有自检的时候才响应中断
		if(usb_detect_state==USB_NOT_DETECT&&!b_self_test)  
		{
			key_state=KEY_DOWNING;
			//Motor_PWM_Init();
//			Reset_Timing_Parameter();
		}
		//b_KeyWkUP_InterrupHappened=TRUE;
	//	set_led(LED_ID_YELLOW,TRUE); //debug
	} 
	EXTI_ClearFlag(EXTI_Line8);
} 

//初始化所有全局变量
void Init_gloab_viriable()
{
	value=0;
	wait_between_total_cnt=0;
	b_detect_hand_before_system_running=TRUE;
	Reset_Timing_Parameter();
	init_PWMState();
	state=LOAD_PARA;
	waitBeforeStart_timing_flag=FALSE;
//	state=WAIT_BEFORE_START;
	b_Is_PCB_PowerOn=FALSE;
	mcu_state=POWER_OFF;
	key_state=KEY_UPING;

	usb_charging_state=USB_CHARGE_NONE;

	usb_detect_state=USB_NOT_DETECT;
//	b_usb_intterruptHappened=FALSE;
//	b_KeyWkUP_InterrupHappened=FALSE;

	b_release_gas=FALSE;
	b_palm_checked=FALSE;
	b_bat_detected_ok=FALSE;

	b_no_hand_in_place=FALSE;
	b_end_of_treatment=FALSE;

	detectPalm_cnt=0;
	noPalm_cnt=0;


	led_bink_timing_flag=TRUE;
	beep_timing_flag=TRUE;
	usb_charge_timing_flag=TRUE;
	key_Press_or_Release_timing_flag=TRUE;
	//static BOOL switch_bnt_timing_flag=TRUE;
	b_releaseGas_timing_flag=TRUE;

	led_bink_cnt=0;
	beep_cnt=0;
	delay_cnt=0;

	b_Motor_Ready2Shake=TRUE;
	b_Motor_shake=FALSE;

	prev_releaseGas_os_tick=0;
	prev_ledBlink_os_tick=0;
	prev_keyPressOrRelease_os_tick=0;
	prev_usbCharge_os_tick=0;
	prev_beep_os_tick=0;
	prev_WaitBeforeStart_os_tick=0;
	prev_PWM1_os_tick=0;
	prev_PWM2_os_tick=0;
	prev_PWM3_os_tick=0;
	prev_PWM4_os_tick=0;
	prev_PWM5_os_tick=0;
	//p_prev_os_tick=NULL;

	key_self_test_timing_flag=TRUE;
	prev_selfTest_os_tick=0;
	
	checkPressAgain_cnt=0;
	wait_cnt=0;


	b_Palm_check_complited=FALSE;
	
	b_self_test=FALSE;

	b_usb_push_in=FALSE;
	b_usb_pull_up=FALSE;
	b_stop_current_works=FALSE;

	led_beep_ID=0;
	led_state=LED_INIT;
	beep_state=BEEP_INIT;
	
	self_tet_state=SELF_TEST_NONE;
  led_In_Turn_state=LED_IN_TURN_NONE;
	
	b_LED_ON_in_turn=FALSE;
	b_check_bnt_release=FALSE;
	
	
	selfTest_delay_Cnt=0;
  nLED_ON_in_turn=0;
  inflate_cnt=0;
  hold_cnt=0;
  deflate_cnt=0;
	
	sample_cnt=0;
	sample_sum=0;
	
	//自检
	deflate_cnt=0;
	selfTest_inflate_record_1=0;
	selfTest_inflate_record_2=0;

	selfTest_hold_record_1=0;
	selfTest_hold_record_2=0;
	selfTest_deflate_record_1=0;
	selfTest_deflate_record_2=0;
	selfTest_fail_Cnt=0;
	selfTest_fail_period_H=0;
	selfTest_fail_period_L=0;
	selfTest_end_Cnt=0;
}

//进入低功耗之前放气4s
void release_gas_before_sleep()
{
//	//放气4s？在进低功耗
//	Motor_PWM_Freq_Dudy_Set(1,100,0);
//	Motor_PWM_Freq_Dudy_Set(2,100,0);
//	Motor_PWM_Freq_Dudy_Set(3,100,0);
//	Motor_PWM_Freq_Dudy_Set(4,100,0);
//	Motor_PWM_Freq_Dudy_Set(5,100,0);
	
	set_led(LED_ID_GREEN,FALSE);
	set_led(LED_ID_YELLOW,FALSE);
	set_led(LED_ID_MODE1,FALSE);
	set_led(LED_ID_MODE2,FALSE);
	set_led(LED_ID_MODE3,FALSE);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	for(uint8_t i=0;i<2;i++)
	{
		delay_ms(1000);
	}
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
}

void init_system_afterWakeUp()
{
//	//init_system_afterWakeUp函数是接Enterstopmode的，在Enterstopmode函数中有if(b_usb_charge_bat) return;的容错处理，
//	//如果触发了该容错，表示有问题，此时不应该进入低功耗，那么在接下来的init_system_afterWakeUp中也不应该全部初始化系统
//	//可以使用如下方式，也可以在Enterstopmode和init_system_afterWakeUp中写if(b_usb_charge_bat)
////	if(!b_usb_charge_bat)  
////	{
////		Enterstopmode();
////		init_system_afterWakeUp();
////	}
//	
//	if(b_usb_charge_bat)  //增加容错，如果USB插上了，不允许进入低功耗
//	{
//		return;
//	}
	//Init_gloab_viriable();
	#if 0
	init_PWMState();
	state=LOAD_PARA;
	waitBeforeStart_timing_flag=FALSE;
//	state=WAIT_BEFORE_START;
	b_Is_PCB_PowerOn=FALSE;
	mcu_state=POWER_OFF;
	//key_state=KEY_UPING;

	//usb_charging_state=USB_CHARGE_NONE;

	//usb_detect_state=USB_NOT_DETECT;

	b_release_gas=FALSE;
	b_palm_checked=FALSE;
	b_bat_detected_ok=FALSE;

	b_no_hand_in_place=FALSE;
	b_end_of_treatment=FALSE;

	detectPalm_cnt=0;
	noPalm_cnt=0;


	led_bink_timing_flag=TRUE;
	beep_timing_flag=TRUE;
	usb_charge_timing_flag=TRUE;
	key_Press_or_Release_timing_flag=TRUE;
	//static BOOL switch_bnt_timing_flag=TRUE;
	b_releaseGas_timing_flag=TRUE;

	led_bink_cnt=0;
	beep_cnt=0;
	delay_cnt=0;

	b_Motor_Ready2Shake=TRUE;
	b_Motor_shake=FALSE;

	prev_releaseGas_os_tick=0;
	prev_ledBlink_os_tick=0;
	prev_keyPressOrRelease_os_tick=0;
	prev_usbCharge_os_tick=0;
	prev_beep_os_tick=0;
	prev_WaitBeforeStart_os_tick=0;
	prev_PWM1_os_tick=0;
	prev_PWM2_os_tick=0;
	prev_PWM3_os_tick=0;
	prev_PWM4_os_tick=0;
	prev_PWM5_os_tick=0;
	//p_prev_os_tick=NULL;

	key_self_test_timing_flag=TRUE;
	prev_selfTest_os_tick=0;
	
	checkPressAgain_cnt=0;
	wait_cnt=0;


	b_Palm_check_complited=FALSE;
	
	b_self_test=FALSE;

	b_usb_push_in=FALSE;
	b_usb_pull_up=FALSE;
	b_stop_current_works=FALSE;

	led_beep_ID=0;
	led_state=LED_INIT;
	beep_state=BEEP_INIT;
	
	self_tet_state=SELF_TEST_NONE;
  led_In_Turn_state=LED_IN_TURN_NONE;
	
	b_LED_ON_in_turn=FALSE;
	b_check_bnt_release=FALSE;
	
	
	selfTest_delay_Cnt=0;
  nLED_ON_in_turn=0;
  inflate_cnt=0;
  hold_cnt=0;
  deflate_cnt=0;
	
	sample_cnt=0;
	sample_sum=0;
	#endif
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	os_ticks = 0;
	//os_ticks = 4294967290;
	
	delay_init();
	os_init();
	
	SystemInit();
	
	init_task();
	//os_start();	
	#if 0
//	delay_init();
//	os_init();
//	SystemInit();
//	
//	
//	//初始化通信相关
//	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
//	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
//	Init_Receive(&g_CmdReceive);
//	
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

//	//输入检测
//	GPIO_InitStructure.GPIO_Pin = EXP_DETECT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_Init(EXP_DETECT_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = KEY_DETECT_PIN;
//	GPIO_Init(KEY_DETECT_PORT, &GPIO_InitStructure);
//	
//	//推挽输出
//	GPIO_InitStructure.GPIO_Pin = GREEN_LED_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GREEN_LED_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
//	//GPIO_ResetBits(GPIOF, GPIO_Pin_0|GPIO_Pin_1);
//	
//	//电源PWR_SAVE
//	GPIO_InitStructure.GPIO_Pin = KEY_PWR_SAVE_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(KEY_PWR_SAVE_PORT, &GPIO_InitStructure);
//	GPIO_ResetBits(KEY_PWR_SAVE_PORT,KEY_PWR_SAVE_PIN);
//	
//	GPIO_InitStructure.GPIO_Pin = RED_LED_PIN;
//	GPIO_Init(RED_LED_PORT, &GPIO_InitStructure);
//	set_led(LED_CLOSE);
//	
//	//初始化ADC
//	Init_ADC1();
//	//初始化ADS115,I2C
//	Init_ADS115();
//	
//	Motor_PWM_Init();
	
#endif
}


void CfgALLPins4StopMode()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
	
	
	//led端口配置为输出,PB2,PB3,PB4,PB5,PB6
	GPIO_InitTypeDef GPIO_InitStructure_LED;
	GPIO_InitStructure_LED.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;                       
	GPIO_InitStructure_LED.GPIO_Speed = GPIO_Speed_50MHz;     
	GPIO_InitStructure_LED.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_LED.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_LED.GPIO_PuPd=GPIO_PuPd_UP;
	//GPIO_InitStructure_PF.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure_LED);
	GPIO_SetBits(GPIOB, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
//	GPIO_SetBits(GPIOB, GPIO_Pin_2);
//	GPIO_SetBits(GPIOB, GPIO_Pin_3);
//	GPIO_SetBits(GPIOB, GPIO_Pin_4);
//	GPIO_SetBits(GPIOB, GPIO_Pin_5);
//	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	
	
//	//配置ADC1和ADC4
//	GPIO_InitTypeDef GPIO_InitStructure_PA_1_4;
//	GPIO_InitStructure_PA_1_4.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
//  GPIO_InitStructure_PA_1_4.GPIO_Mode = GPIO_Mode_AN;
//  GPIO_InitStructure_PA_1_4.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1_4);	
	
	
	//配置ADC，PA1,PB0
	GPIO_InitTypeDef GPIO_InitStructure_PA_1;
	GPIO_InitStructure_PA_1.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_AN;
	//GPIO_InitStructure_PA_1.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure_PA_1.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1);	
	
	GPIO_InitTypeDef GPIO_InitStructure_PB_0;
	GPIO_InitStructure_PB_0.GPIO_Pin = GPIO_Pin_0;
  //GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure_PB_0.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure_PB_0);
	
//	GPIO_InitTypeDef GPIO_InitStructure_PB_0;
//	GPIO_InitStructure_PB_0.GPIO_Pin = GPIO_Pin_0;
//  //GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_AN;
//	GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PB_0.GPIO_OType=GPIO_OType_PP;
//  GPIO_InitStructure_PB_0.GPIO_PuPd = GPIO_PuPd_UP ;
//  GPIO_Init(GPIOB, &GPIO_InitStructure_PB_0);
//	GPIO_SetBits(GPIOB,GPIO_Pin_0);
	
	//关闭ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , DISABLE);		
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
	
	//关闭串口
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
//	//串口IO,PA2,PA3
	GPIO_InitTypeDef GPIO_InitStructure_PA_2_3;
	GPIO_InitStructure_PA_2_3.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                       
	GPIO_InitStructure_PA_2_3.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_2_3.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PA_2_3.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_2_3);
	
//	GPIO_InitTypeDef GPIO_InitStructure_PA_2_3;
//	GPIO_InitStructure_PA_2_3.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                       
//	GPIO_InitStructure_PA_2_3.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_2_3.GPIO_Mode = GPIO_Mode_AN;
////	GPIO_InitStructure_PA_2_3.GPIO_OType=GPIO_OType_PP;
////	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_UP;
////	//GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_2_3);
	
	
	//I2C端口，PA9,PA10
//	GPIO_InitTypeDef GPIO_InitStructure_UART;
//	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                       
//	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_IN;
//	//GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	
	
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                       
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	GPIO_SetBits(GPIOA,GPIO_Pin_9|GPIO_Pin_10); 
//	
	
	//PWR save，PA12
	GPIO_InitTypeDef GPIO_InitStructure_PA12;
	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA12.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);   //输出高电平，关管子，省电

//	GPIO_InitTypeDef GPIO_InitStructure_PA12;
//	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
//	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);

	
	//PWR EN，PA15
	GPIO_InitTypeDef GPIO_InitStructure_PA15;
	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);   //输出低电平，关管子，省电

//	GPIO_InitTypeDef GPIO_InitStructure_PA15;
//	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
//	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);

	
	//PWM1(PA6),PWM2(PA7)  inflate_pwm2(PA11)
	GPIO_InitTypeDef GPIO_InitStructure_PA_6_7_11;
	GPIO_InitStructure_PA_6_7_11.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11;                       
	GPIO_InitStructure_PA_6_7_11.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_6_7_11.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure_PA_6_7_11.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PA_6_7_11.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_6_7_11);
	GPIO_ResetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11);
	
	//PWM3(PB1) BEEP_PWM(PB14)
	GPIO_InitTypeDef GPIO_InitStructure_PB_1_14;
	GPIO_InitStructure_PB_1_14.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_14;                       
	GPIO_InitStructure_PB_1_14.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_1_14.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PB_1_14.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_1_14.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_1_14.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1_14);
	GPIO_ResetBits(GPIOB,GPIO_Pin_1|GPIO_Pin_14);
	
//	GPIO_InitTypeDef GPIO_InitStructure_PB_1;
//	GPIO_InitStructure_PB_1.GPIO_Pin = GPIO_Pin_1;                       
//	GPIO_InitStructure_PB_1.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_OUT;
//	//GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_1.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_1);

//	GPIO_InitTypeDef GPIO_InitStructure_PB_14;
//	GPIO_InitStructure_PB_14.GPIO_Pin = GPIO_Pin_14;                       
//	GPIO_InitStructure_PB_14.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PB_14.GPIO_Mode = GPIO_Mode_OUT;
//	//GPIO_InitStructure_PB_1.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_14.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PB_1.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_InitStructure_PB_14.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_14);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_14);

	
	//BAT_CHARGE,BAT_STDBY   PA4,PA5
	GPIO_InitTypeDef GPIO_InitStructure_PA_4_5;
	GPIO_InitStructure_PA_4_5.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;                       
	GPIO_InitStructure_PA_4_5.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_4_5.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_4_5);
	
//	GPIO_InitTypeDef GPIO_InitStructure_PA_4_5;
//	GPIO_InitStructure_PA_4_5.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;                       
//	GPIO_InitStructure_PA_4_5.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_4_5.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_4_5);
//	GPIO_SetBits(GPIOB,GPIO_Pin_4|GPIO_Pin_5);
	
	//USB_OE PA0
	GPIO_InitTypeDef GPIO_InitStructure_PA0;
	GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0;                       
	GPIO_InitStructure_PA0.GPIO_Speed = GPIO_Speed_50MHz;       
	//GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_DOWN;
	//GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA0);
	
	//VAVLE PB10,PB11
	GPIO_InitTypeDef GPIO_InitStructure_PB_10_11;
	GPIO_InitStructure_PB_10_11.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                       
	GPIO_InitStructure_PB_10_11.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_10_11.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_10_11.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_10_11.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_10_11);
	
	//POWER ON/OFF (PA8)
//	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
//	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
//	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_IN;
//	//GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
//	//GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);
//	//GPIO_SetBits(GPIOA,GPIO_Pin_8);

	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);
	GPIO_SetBits(GPIOA,GPIO_Pin_8);

	//SWITCH ON/OFF (PB13)   ,SWITCH MODE(PB15)
	GPIO_InitTypeDef GPIO_InitStructure_PB_13_15;
	GPIO_InitStructure_PB_13_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;                       
	GPIO_InitStructure_PB_13_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_13_15.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PB_13_15.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PB_13_15.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure_PB_13_15.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_13_15);
	
//		GPIO_InitTypeDef GPIO_InitStructure_PB_15;
//		GPIO_InitStructure_PB_15.GPIO_Pin = GPIO_Pin_15;                       
//		GPIO_InitStructure_PB_15.GPIO_Speed = GPIO_Speed_50MHz;       
//		GPIO_InitStructure_PB_15.GPIO_Mode = GPIO_Mode_OUT;
//		GPIO_InitStructure_PB_15.GPIO_OType=GPIO_OType_PP;
//		GPIO_InitStructure_PB_15.GPIO_PuPd=GPIO_PuPd_UP;
//		GPIO_Init(GPIOB, &GPIO_InitStructure_PB_15);
//		GPIO_SetBits(GPIOB,GPIO_Pin_15);
	
	//PB7,PB8,PB9,PB12
	GPIO_InitTypeDef GPIO_InitStructure_PB_7_8_9_12;
	GPIO_InitStructure_PB_7_8_9_12.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_12;                       
	GPIO_InitStructure_PB_7_8_9_12.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_7_8_9_12.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PB_7_8_9_12.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_7_8_9_12);
	
	//PC13,PC14,PC15
	GPIO_InitTypeDef GPIO_InitStructure_PC_13_14_15;
	GPIO_InitStructure_PC_13_14_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;                       
	GPIO_InitStructure_PC_13_14_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PC_13_14_15.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PC_13_14_15.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure_PC_13_14_15);
	
	//PF0,PF1
	GPIO_InitTypeDef GPIO_InitStructure_PF_0_1;
	GPIO_InitStructure_PF_0_1.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;                       
	GPIO_InitStructure_PF_0_1.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PF_0_1.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure_PF_0_1.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOF, &GPIO_InitStructure_PF_0_1);
}

//进入stop模式，采用中断唤醒
void EnterStopMode()
{
//	if(b_usb_charge_bat)  //增加容错，如果USB插上了，不允许进入低功耗
//	{
//		return;
//	}
//	
	//b_KeyWkUP_InterrupHappened=FALSE;

//	led_bink_timing_flag=FALSE;
//	prev_ledBlink_os_tick=0;
//	beep_timing_flag=FALSE;
//	prev_beep_os_tick=0;
	Init_gloab_viriable();
	
	//配置中断
	CfgWFI();
	//I2C芯片ADS115进入power-down模式
	ADS115_enter_power_down_mode();

	CfgALLPins4StopMode();
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);  
	PWR_EnterSTOPMode(PWR_Regulator_LowPower, PWR_STOPEntry_WFI);
}


void key_power_on_task(void)
{
	static uint8_t wakeup_Cnt;
	//static uint8_t sleep_Cnt;
	if(key_state==KEY_DOWNING)
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
		{
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
			if(wakeup_Cnt==50)
			{
				wakeup_Cnt=0;
//				b_KeyWkUP_InterrupHappened=FALSE;  //清除中断发生标志
				b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;
				
				if(b_Is_PCB_PowerOn)
				{
					mcu_state=POWER_ON;	
					key_state=KEY_WAKE_UP;		
					state=LOAD_PARA;
					init_PWMState();
				}
				else
				{
					mcu_state=POWER_OFF;	
					key_state=KEY_STOP_MODE;
					state=LOAD_PARA;
					init_PWMState();
				}
			}
			else
			{
				wakeup_Cnt++;
			}
		}
		else
		{
			wakeup_Cnt=0;
			if(!b_Is_PCB_PowerOn)  //b_Is_PCB_PowerOn为FALSE是才进行判断，按键时间过短，不允许启动
			{
				//sleep_Cnt++;
				key_state=KEY_FAIL_WAKEUP;
				//b_KeyWkUP_InterrupHappened=FALSE;
				//key_state=KEY_STOP_MODE;
			}
		}
	}
	
	if(key_state==KEY_FAIL_WAKEUP)
	{
		//if(sleep_Cnt>0)
		{
			//sleep_Cnt=0;
			EnterStopMode();
			init_system_afterWakeUp();
		}
	}
	
	if(key_state==KEY_WAKE_UP)
	{
		if(b_bat_detected_ok)
		{
			//开机
			set_led(LED_ID_GREEN,TRUE);
			
			if(mode==1)
			{	
				set_led(LED_ID_MODE1,TRUE); 
			}
			else if(mode==2)
			{
				set_led(LED_ID_MODE2,TRUE);   
			}
			else if(mode==3)
			{
				set_led(LED_ID_MODE3,TRUE);  
			}
			else
			{
				//do nothing
			}
			//wakeup_Cnt=0;
			key_state=KEY_UPING;
		}
	}
	
	if(key_state==KEY_STOP_MODE)
	{
		//sleep_Cnt=0;
		//放气4s？在进低功耗
		Motor_PWM_Freq_Dudy_Set(1,100,0);
		Motor_PWM_Freq_Dudy_Set(2,100,0);
		Motor_PWM_Freq_Dudy_Set(3,100,0);
		Motor_PWM_Freq_Dudy_Set(1,100,0);
		Motor_PWM_Freq_Dudy_Set(2,100,0);
		Motor_PWM_Freq_Dudy_Set(3,100,0);
//		Motor_PWM_Freq_Dudy_Set(4,100,0);
//		Motor_PWM_Freq_Dudy_Set(5,100,0);
		
		release_gas_before_sleep();
		EnterStopMode();
		init_system_afterWakeUp();
		
//		static uint8_t stop_cnt;
//		if(stop_cnt==1)
//		{
//			stop_cnt=0;
//			release_gas_before_sleep();
//			EnterStopMode();
//			init_system_afterWakeUp();
//		}
//		else
//		{
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
//			stop_cnt++;
//			
//			pwm1_state=PWM_NONE;
//			pwm2_state=PWM_NONE;
//			pwm3_state=PWM_NONE;
//		}
	}
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
