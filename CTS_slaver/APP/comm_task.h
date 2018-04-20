#ifndef __COMM_TASK_H
#define __COMM_TASK_H	    
//////////////////////////////////////////////////////////////////////////////////	 							  
//////////////////////////////////////////////////////////////////////////////////

#include "datatype.h"
//#include "stdint.h"
typedef unsigned char uint8_t;
typedef unsigned int uint32_t;
	

//#define SEND_BUF_LEN  255
//#define SEND_BUF_LEN  248
#define SEND_BUF_LEN  260

////434是数据，2是两个校验位
//#define PARAMETER_BUF_LEN 436

//CTS，452(434+18)数据长度+2个校验位+2字节(为了补齐)=456 
#define PARAMETER_BUF_LEN 456

//局部变量
typedef enum
{
	LOAD_PARA,  //加载参数
	GET_CYCLE_CNT,
	IDLE,
	GET_MODE,
	CHECK_PRESSURE,
	CHECK_PRESSURE_AGAIN,
	PREV_OUTPUT_PWM,
	CPY_PARA_TO_BUFFER,
	OUTPUT_PWM,
	CHECK_BAT_VOL,
	LED_RED_BLINK
}CHCKMODE_OUTPUT_PWM;

typedef enum
{
	PWM_START,
	PWM_PERIOD,
	PWM_WAIT_BETWEEN,
	PWM_WAIT_AFTER,
	PWM_OUTPUT_FINISH
}PWM_STATE;


typedef enum
{
	USB_CHARGE_NONE,
	USB_CHARGING,
	USB_CHARGED_FULL,
	USB_CHARGE_FAULT,
	USB_CHARGE_NO_BATTERY
}USB_CHARGING_STATE;


typedef enum
{
	LED_INIT,
	LED_ON,
	LED_OFF,
	LED_END
}LED_STATE;

typedef enum
{
	BEEP_INIT,
	BEEP_ON,
	BEEP_OFF,
	BEEP_END
}BEEP_STATE;

void init_PWMState(void);

void TaskDataSend (void);
void CMD_ProcessTask (void);
void CalcCheckSum(UINT8* pPacket);
void check_selectedMode_ouputPWM(void);
void PaintPWM(unsigned char num,unsigned char* pwm_buffer);
//void PaintPWM(unsigned char num );
void CheckFlashData(unsigned char* buffer);
void ResetParameter(unsigned char* buffer);
void get_switch_mode(void);
void ReleaseGas(void);
void Red_LED_Blink(unsigned char seconds);
BOOL Is_timing_Xmillisec(uint32_t n_ms,uint8_t ID);
void DetectPalm(void);
void Detect_battery_and_tmp(void);
void led_blink_beep(void);
void usb_charge_battery(void);
void InitKeyWakeUpTiming(void);
#endif
