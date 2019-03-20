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

////CTS，452(434+18)数据长度+2个校验位+2字节(为了补齐)=456 
//#define PARAMETER_BUF_LEN 456

//CTS，470(434+18+18)数据长度+2个校验位=472 
#define PARAMETER_BUF_LEN 472

//#define _DEBUG   //这个方便调试，正式发布代码的时候不需要这个
//#define _DEBUG_TEST_CYCLES  //这个是用来loop test的，测试cycle
//#define _DEBUG_FLASH_RECORD_DATETIME
//#define _DEBUG_SELF_TEST_FOR_5_LEVES_PRESSURE

//局部变量
typedef enum
{
	LOAD_PARA,  //加载参数
	GET_CYCLE_CNT,
	IDLE,
	GET_MODE,
	CHECK_PRESSURE,
	CHECK_PRESSURE_AGAIN,
	//PREV_OUTPUT_PWM,
	WAIT_BEFORE_START,
	CPY_PARA_TO_BUFFER,
	OUTPUT_PWM,
	CHECK_BAT_VOL,
	LED_RED_BLINK
}CHCKMODE_OUTPUT_PWM;

typedef enum
{
	PWM_NONE,
	PWM_START,
	PWM_PERIOD,
	PWM_DWELL,
	PWM_OVER_SAFTY_THRESHOLD,
	PWM_WAIT_BETWEEN,
	PWM_WAIT_AFTER,
	PWM_OUTPUT_FINISH
}PWM_STATE;


typedef enum
{
	USB_CHARGE_NONE,
	USB_CHECK_CHARG,
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

typedef enum
{
	SELF_TEST_NONE,
	SELF_TEST_DELAY_BEFORE_START,
	SELF_TEST_DEFLATE_BEFORE_START,
	SELF_TEST_START,
	SELF_TEST_INFLATE,
	SELF_TEST_HOLD,
	SELF_TEST_DEFLATE,
	SELF_TEST_FAIL,
	SELF_TEST_END
}SELF_TEST_STATE;


typedef enum
{
	LED_IN_TURN_NONE,
	LED_IN_TURN_MODE1,
	LED_IN_TURN_MODE2,
	LED_IN_TURN_MODE3,
}LED_IN_TURN_STATE;


typedef enum
{
	MAGNETIC_OPEN,
	MAGNETIC_CLOSE
}MAGNETIC_OPERATE;

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
void self_test(void);
void process_interrupt(void);
void Reset_Timing_Parameter(void);
void Operate_magnetic_valve(MAGNETIC_OPERATE operate);
#endif
