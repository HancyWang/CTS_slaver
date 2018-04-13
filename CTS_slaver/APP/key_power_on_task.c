

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
//KEY值，这里点按为确认蓝牙连接
typedef enum {
	NO_KEY,
	BLUE_CHECK
}KEY_VAL;



MCU_STATE mcu_state=POWER_OFF;
//mcu_state=POWER_OFF;


//SWITCH_MODE prev_switch_mode=SWITCH_MODE1;

//extern uint8_t OUTPUT_FINISH;
BOOL b_Is_PCB_PowerOn=FALSE;

volatile KEY_STATE key_state=KEY_STOP_MODE;

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
	
//	//外部按键GPIOA初始化,PA0  
	//外部按键GPIOA初始化,PA8 
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
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

//	//EXTI0中断向量配置  
	//EXTI8中断向量配置  
	NVIC_InitTypeDef NVIC_InitStructure;  
//	NVIC_InitStructure.NVIC_IRQChannel=EXTI0_1_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_15_IRQn;  
	NVIC_InitStructure.NVIC_IRQChannelPriority=0x01;  
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;  
	NVIC_Init(&NVIC_InitStructure);  
}


////检测PA0(WAKE_UP)是否被按下
//检测PA8(power_on_off)是否被按下
BOOL Check_wakeUpKey_pressed(void)
{
	uint32_t cnt=0;
	while(TRUE)
	{
		//读取PA0的电平
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
		{
			cnt++;
			delay_ms(30);
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
			{
				while(TRUE)
				{
					delay_ms(10);
					if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==1)
					{
						return TRUE;
					}
				}
			}
			//delay_ms(5);
//			if(cnt>20)
//			{
//				return TRUE;
//			}
		}
		else
		{
			return FALSE;
		}
	}
}

//void EXTI0_1_IRQHandler(void)
void EXTI4_15_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)  
	{ 
		if(Check_wakeUpKey_pressed())
		{
			b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;		//每按一次，b_Is_PCB_PowerOn翻转一次状态
			if(b_Is_PCB_PowerOn==TRUE)
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
	}  
	//EXTI_ClearITPendingBit(EXTI_Line0);
	EXTI_ClearFlag(EXTI_Line8);
} 


void init_system_afterWakeUp()
{
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
	//GPIO_SetBits(GPIOB, GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6);
	GPIO_SetBits(GPIOB, GPIO_Pin_2);
	GPIO_SetBits(GPIOB, GPIO_Pin_3);
	GPIO_SetBits(GPIOB, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOB, GPIO_Pin_6);
	
	
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
  GPIO_InitStructure_PA_1.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1);	
	
	GPIO_InitTypeDef GPIO_InitStructure_PB_0;
	GPIO_InitStructure_PB_0.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure_PB_0.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure_PB_0.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  GPIO_Init(GPIOB, &GPIO_InitStructure_PB_0);
	
	
	//关闭ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , DISABLE);		
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
	
	
	//串口IO,PA2,PA3
	GPIO_InitTypeDef GPIO_InitStructure_PA_2_3;
	GPIO_InitStructure_PA_2_3.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;                       
	GPIO_InitStructure_PA_2_3.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_2_3.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PA_2_3.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_2_3.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_2_3);
	
	//关闭串口
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
	//I2C端口，PA9,PA10
	GPIO_InitTypeDef GPIO_InitStructure_UART;
	GPIO_InitStructure_UART.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;                       
	GPIO_InitStructure_UART.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_UART.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_UART.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_UART.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_UART);
	
	//PWR save，PA12
	GPIO_InitTypeDef GPIO_InitStructure_PA12;
	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure_PA12.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);   //输出高电平，关管子，省电
	
	
	//PWR EN，PA15
	GPIO_InitTypeDef GPIO_InitStructure_PA15;
	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_OUT;
	//GPIO_InitStructure_PA5.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_DOWN;
	//GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);   //输出低电平，关管子，省电
	
	//PWM1(PA6),PWM2(PA7)  inflate_pwm2(PA11)
	GPIO_InitTypeDef GPIO_InitStructure_PA_6_7_11;
	GPIO_InitStructure_PA_6_7_11.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_11;                       
	GPIO_InitStructure_PA_6_7_11.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_6_7_11.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA_6_7_11.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure_PA_6_7_11.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_6_7_11);
	//GPIO_SetBits(GPIOA,GPIO_Pin_6|GPIO_Pin_7);
	
	//PWM3(PB1) BEEP_PWM(PB14)
	GPIO_InitTypeDef GPIO_InitStructure_PB_1_14;
	GPIO_InitStructure_PB_1_14.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_14;                       
	GPIO_InitStructure_PB_1_14.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_1_14.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PB_1_14.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PB_1_14.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_1_14);

	
	//BAT_CHARGE,BAT_STDBY   PA4,PA5
	GPIO_InitTypeDef GPIO_InitStructure_PA_4_5;
	GPIO_InitStructure_PA_4_5.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;                       
	GPIO_InitStructure_PA_4_5.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA_4_5.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_4_5);
	
	//USB_OE PA0
	GPIO_InitTypeDef GPIO_InitStructure_PA0;
	GPIO_InitStructure_PA0.GPIO_Pin = GPIO_Pin_0;                       
	GPIO_InitStructure_PA0.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA0.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA_4_5.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA_4_5.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA0);
	
	//VAVLE PB10,PB11
	GPIO_InitTypeDef GPIO_InitStructure_PB_10_11;
	GPIO_InitStructure_PB_10_11.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;                       
	GPIO_InitStructure_PB_10_11.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_10_11.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PB_10_11.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PB_10_11.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_10_11);
	
//	//POWER ON/OFF (PA8)
//	GPIO_InitTypeDef GPIO_InitStructure_PA_8;
//	GPIO_InitStructure_PA_8.GPIO_Pin = GPIO_Pin_8;                       
//	GPIO_InitStructure_PA_8.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA_8.GPIO_Mode = GPIO_Mode_IN;
//	//GPIO_InitStructure_PA_8.GPIO_OType=GPIO_OType_PP;
//	GPIO_InitStructure_PA_8.GPIO_PuPd=GPIO_PuPd_DOWN;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA_8);
//	//GPIO_SetBits(GPIOA,GPIO_Pin_8);

	//SWITCH ON/OFF (PB13)   ,SWITCH MODE(PB15)
	GPIO_InitTypeDef GPIO_InitStructure_PB_13_15;
	GPIO_InitStructure_PB_13_15.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_15;                       
	GPIO_InitStructure_PB_13_15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PB_13_15.GPIO_Mode = GPIO_Mode_IN;
	//GPIO_InitStructure_PB_13_15.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PB_13_15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOB, &GPIO_InitStructure_PB_13_15);

}

//进入stop模式，采用中断唤醒
void EnterStopMode()
{
	//清除手掌记录状态
	detectPalm_cnt=0;
	noPalm_cnt=0;
	b_palm_checked=FALSE;
	
	b_bat_detected_ok=FALSE;
	
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
	if(key_state==KEY_STOP_MODE)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
//	
	//按键被按下，检查电池电压
	if(key_state==KEY_WAKE_UP)
	{
//		//经过1/2分压之后，电压在1.5v-2.1v之间(2048-2867)，偏差300
		//GPIO_SetBits(GPIOA,GPIO_Pin_15);
		//if(RegularConvData_Tab[0]>=2048-300&&RegularConvData_Tab[0]<=2867+300)  //以3v作为参考电压 (2730是以3.3v为参考电压的) 
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
			
			Motor_PWM_Freq_Dudy_Set(1,100,80);
			Motor_PWM_Freq_Dudy_Set(2,100,80);
//			Motor_PWM_Freq_Dudy_Set(2,100,80);
			Motor_PWM_Freq_Dudy_Set(3,100,80);
			Delay_ms(500);
			Motor_PWM_Freq_Dudy_Set(1,100,0);
			Motor_PWM_Freq_Dudy_Set(2,100,0);
			Motor_PWM_Freq_Dudy_Set(3,100,0);
			
			key_state=KEY_UPING;
		}
		
		//没电闪灯，放到了detetct battery中了
//		else
//		{
//			//橙色LED闪3s，关机
//			Red_LED_Blink(3);
//			//key_state=KEY_UPING;
//			//key_state=KEY_STOP_MODE;
//			mcu_state=POWER_OFF;
//			//进入stop模式
//			EnterStopMode();
//			//唤醒之后重新初始化
//			init_system_afterWakeUp();
//		}
  }
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
