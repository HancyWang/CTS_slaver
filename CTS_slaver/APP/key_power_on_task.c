

/**
********************************************************************************
* ��ࣺ
* ģ�����ƣ�key_power_on_task.c
* ģ�鹦�ܣ�
* �������ڣ�
* �� �� �ߣ�
* ��    ����
********************************************************************************
**/

/***********************************
* ͷ�ļ�
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
*�궨��
***********************************/

/***********************************
* ȫ�ֱ���
***********************************/

/***********************************
* �ֲ�����
***********************************/
extern uint32_t os_ticks;
extern CMD_Receive g_CmdReceive;  // ������տ��ƶ���

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

//extern BOOL b_start_powerOn_check;
// BOOL b_KeyWkUP_InterrupHappened=FALSE;
// BOOL b_usb_intterruptHappened=FALSE;
//KEYֵ������㰴Ϊȷ����������
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
* �ֲ�����
***********************************/

/*******************************************************************************
** ��������: EnterStopMode
** ��������: ����͹���ģʽ
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CfgWFI()
{
	//ʱ��ʹ��
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA|RCC_AHBPeriph_GPIOB|RCC_AHBPeriph_GPIOF,ENABLE);  
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);  
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); 
	
//	//USB��磬USB_0E,PA0  
	//�ⲿ����GPIOA��ʼ��,PA8 
	GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;  
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;  
	//GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_2MHz;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);  
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;  
	GPIO_Init(GPIOA,&GPIO_InitStructure);

//	//��EXTI0ָ��PA0  
	//��EXTI8ָ��PA8
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource8);  
//	//EXTI0�ж�������
		//EXTI8�ж�������
	EXTI_InitTypeDef EXTI_InitStructure;  
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;  
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;  
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;  
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;  
	EXTI_Init(&EXTI_InitStructure);  

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource0);  
	EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;   //���ó������غ��½��ض����Դ����ж�
	EXTI_Init(&EXTI_InitStructure);

//	//EXTI0�ж���������  
	//EXTI8�ж���������  
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


//////���PA0(WAKE_UP)�Ƿ񱻰���
////���PA8(power_on_off)�Ƿ񱻰���
//BOOL Check_wakeUpKey_pressed(void)
//{
//	while(TRUE)
//	{
//		//��ȡPA8�ĵ�ƽ
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

////PA0,�ж�USB�ǲ��뻹�ǰγ�
USB_DETECT_STATE Check_USB_pull_or_push()
{
	while(TRUE)
	{
		//��ȡPA0�ĵ�ƽ
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
			//delay_ms(500);  //��500ms���ȶ�ʱ��
			//set_led(LED_ID_YELLOW,TRUE);  //debug
			
			uint8_t cnt=0;
			//ѭ��5�Σ����5�ζ��Ǹߵ�ƽ��˵���Ѿ��ȶ��Ĳ���USB��
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
			//delay_ms(500);  //��500ms���ȶ�ʱ��
			
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
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1) //�ߵ�ƽ��ʾ������USB,����
		{
			//����USB����
//			b_usb_push_in=TRUE;
			usb_detect_state=USB_PUSH_IN;
		}
		else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==0)  //�͵�ƽ��ʾ�γ���USB������
		{
			//����USB�γ�
			//b_usb_pull_up=TRUE;
			usb_detect_state=USB_PULL_UP;
		}
		else
		{
			//do nothing
		}
		
		//������������֤һ��, //��˼���ж�����������֮��Ͳ��ٽ����������������ȥ
		//usb_charge_battery��֤USB����Ƿ���Ч
//		b_usb_intterruptHappened=FALSE;
	}  
	EXTI_ClearFlag(EXTI_Line0);
}

void EXTI4_15_IRQHandler(void)
{  
	if(EXTI_GetITStatus(EXTI_Line8)!=RESET)  
	{ 
		//1.USBû�����ʱ�����Ӧ
		//2.û���Լ��ʱ�����Ӧ�ж�
		if(usb_detect_state==USB_NOT_DETECT&&!b_self_test)  
		{
			key_state=KEY_DOWNING;
		}
		//b_KeyWkUP_InterrupHappened=TRUE;
	//	set_led(LED_ID_YELLOW,TRUE); //debug
	} 
	EXTI_ClearFlag(EXTI_Line8);
} 


void init_system_afterWakeUp()
{
//	//init_system_afterWakeUp�����ǽ�Enterstopmode�ģ���Enterstopmode��������if(b_usb_charge_bat) return;���ݴ���
//	//��������˸��ݴ���ʾ�����⣬��ʱ��Ӧ�ý���͹��ģ���ô�ڽ�������init_system_afterWakeUp��Ҳ��Ӧ��ȫ����ʼ��ϵͳ
//	//����ʹ�����·�ʽ��Ҳ������Enterstopmode��init_system_afterWakeUp��дif(b_usb_charge_bat)
////	if(!b_usb_charge_bat)  
////	{
////		Enterstopmode();
////		init_system_afterWakeUp();
////	}
//	
//	if(b_usb_charge_bat)  //�����ݴ����USB�����ˣ����������͹���
//	{
//		return;
//	}
	
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
//	//��ʼ��ͨ�����
//	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
//	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
//	Init_Receive(&g_CmdReceive);
//	
//	
//	GPIO_InitTypeDef  GPIO_InitStructure;

//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);

//	//������
//	GPIO_InitStructure.GPIO_Pin = EXP_DETECT_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_Init(EXP_DETECT_PORT, &GPIO_InitStructure);
//	
//	GPIO_InitStructure.GPIO_Pin = KEY_DETECT_PIN;
//	GPIO_Init(KEY_DETECT_PORT, &GPIO_InitStructure);
//	
//	//�������
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
//	//��ԴPWR_SAVE
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
//	//��ʼ��ADC
//	Init_ADC1();
//	//��ʼ��ADS115,I2C
//	Init_ADS115();
//	
//	Motor_PWM_Init();
	
#endif
}


void CfgALLPins4StopMode()
{
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOF, ENABLE);
	
	
	//led�˿�����Ϊ���,PB2,PB3,PB4,PB5,PB6
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
	
	
//	//����ADC1��ADC4
//	GPIO_InitTypeDef GPIO_InitStructure_PA_1_4;
//	GPIO_InitStructure_PA_1_4.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_4;
//  GPIO_InitStructure_PA_1_4.GPIO_Mode = GPIO_Mode_AN;
//  GPIO_InitStructure_PA_1_4.GPIO_PuPd = GPIO_PuPd_NOPULL ;
//  GPIO_Init(GPIOA, &GPIO_InitStructure_PA_1_4);	
	
	
	//����ADC��PA1,PB0
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
	
	//�ر�ADC
	DMA_Cmd(DMA1_Channel1, DISABLE);/* DMA1 Channel1 enable */			
  ADC_DMACmd(ADC1, DISABLE);
	ADC_Cmd(ADC1, DISABLE);  
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, DISABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 , DISABLE);		
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , DISABLE);
	
	//�رմ���
	DMA_Cmd(UART_DMA_RX_CHANNEL, DISABLE);
	DMA_Cmd(UART_DMA_TX_CHANNEL, DISABLE);
	USART_Cmd(UART, DISABLE);
	
//	//����IO,PA2,PA3
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
	
	
	//I2C�˿ڣ�PA9,PA10
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
	
	//PWR save��PA12
	GPIO_InitTypeDef GPIO_InitStructure_PA12;
	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA12.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_UP;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);
	GPIO_SetBits(GPIOA,GPIO_Pin_12);   //����ߵ�ƽ���ع��ӣ�ʡ��

//	GPIO_InitTypeDef GPIO_InitStructure_PA12;
//	GPIO_InitStructure_PA12.GPIO_Pin = GPIO_Pin_12;                       
//	GPIO_InitStructure_PA12.GPIO_Speed = GPIO_Speed_50MHz;       
//	GPIO_InitStructure_PA12.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_InitStructure_PA12.GPIO_PuPd=GPIO_PuPd_NOPULL;
//	GPIO_Init(GPIOA, &GPIO_InitStructure_PA12);

	
	//PWR EN��PA15
	GPIO_InitTypeDef GPIO_InitStructure_PA15;
	GPIO_InitStructure_PA15.GPIO_Pin = GPIO_Pin_15;                       
	GPIO_InitStructure_PA15.GPIO_Speed = GPIO_Speed_50MHz;       
	GPIO_InitStructure_PA15.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_PP;
	//GPIO_InitStructure_PA15.GPIO_OType=GPIO_OType_OD;
	GPIO_InitStructure_PA15.GPIO_PuPd=GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStructure_PA15);
	GPIO_ResetBits(GPIOA,GPIO_Pin_15);   //����͵�ƽ���ع��ӣ�ʡ��

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

void Init_gloab_viriable()
{
	init_PWMState();
	state=LOAD_PARA;

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
}


//����stopģʽ�������жϻ���
void EnterStopMode()
{
//	if(b_usb_charge_bat)  //�����ݴ����USB�����ˣ����������͹���
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
	
	//�����ж�
	CfgWFI();
	//I2CоƬADS115����power-downģʽ
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
			if(wakeup_Cnt==50)
			{
				wakeup_Cnt=0;
//				b_KeyWkUP_InterrupHappened=FALSE;  //����жϷ�����־
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
			if(!b_Is_PCB_PowerOn)  //b_Is_PCB_PowerOnΪFALSE�ǲŽ����жϣ�����ʱ����̣�����������
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
			//����
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
		EnterStopMode();
		init_system_afterWakeUp();
	}

	/*
	if(b_KeyWkUP_InterrupHappened)  //�жϻ��Ѱ����Ƿ��£�PA8
	{
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8)==0)
		{
			if(Is_timing_Xmillisec(1000,13))
			{
				InitKeyWakeUpTiming();
				//b_Interrupt_KeyWakeUp_Pressed=TRUE;
				b_KeyWkUP_InterrupHappened=FALSE;
				
				b_Is_PCB_PowerOn=!b_Is_PCB_PowerOn;		//ÿ��һ�Σ�b_Is_PCB_PowerOn��תһ��״̬
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
					
					set_led(LED_ID_GREEN,FALSE);
				}
			}
			else
			{
				set_led(LED_ID_YELLOW,FALSE);
				//InitKeyWakeUpTiming();
			}
		}
		else
		{
			  //����������������ʱ���ۼ�
			
			//if(!b_Is_PCB_PowerOn)�Ƿ�ֹ�ػ���ʱ��һ���Ӿ͹ػ���
			if(!b_Is_PCB_PowerOn)  //������ʱ����Ҫ�жϣ��ػ���ʱ��b_Is_PCB_PowerOn��TRUE����ִ�У������ð�����2s�Źػ�
			{
				EnterStopMode();
				init_system_afterWakeUp();
			}
		}
	}
	else
	{
		
		//if(b_usb_charge_bat==TRUE)  //������ڳ��,���ػ�������Ч
		if(usb_detect_state==USB_PUSH_IN)
		{
			key_state=KEY_UPING; 
		}
		else 
		{
			if(key_state==KEY_STOP_MODE)
			{
				EnterStopMode();
				init_system_afterWakeUp();
			}
		//	
			//���������£�����ص�ѹ
			if(key_state==KEY_WAKE_UP)
			{
		//		//����1/2��ѹ֮�󣬵�ѹ��1.5v-2.1v֮��(2048-2867)��ƫ��300
				//GPIO_SetBits(GPIOA,GPIO_Pin_15);
				//if(RegularConvData_Tab[0]>=2048-300&&RegularConvData_Tab[0]<=2867+300)  //��3v��Ϊ�ο���ѹ (2730����3.3vΪ�ο���ѹ��) 
				if(b_bat_detected_ok)
				{
					//����
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
					key_state=KEY_UPING;
				}
			}
		}
	}
*/
	
	os_delay_ms(KEY_LED_TASK_ID, KEY_LED_PERIOD);
}
