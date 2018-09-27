#ifndef __KEY_TASK_H_
#define __KEY_TASK_H_
/***********************************
* 头文件
***********************************/

/**********************************
*宏定义
***********************************/
#define KEY_LED_PERIOD  20
#define CHECK_MODE_OUTPUT_PWM 10
#define ONE_SEC_KEY_TIME   1000/KEY_LED_PERIOD
/***********************************
* 全局变量
***********************************/
typedef enum
{
	POWER_ON,
	POWER_OFF
}MCU_STATE;

typedef enum
{
	KEY_STOP_MODE,  //运行状态进入低功耗模式
	KEY_UPING,
	KEY_DOWNING,
	KEY_FAIL_WAKEUP,  //按开机键没启动起来，进入低功耗(也就是按的时候不够)
//	KEY_DOWN_UP,
	KEY_WAKE_UP
	//KEY_UP_DOWN
}KEY_STATE;

typedef enum
{
	USB_PUSH_IN,
	USB_FAIL_INSERT,
	USB_PULL_UP,
	USB_NOT_DETECT,
	USB_INSERTED
}USB_DETECT_STATE;


typedef enum
{
	CODE_SYSTEM_POWER_ON				=	0x11,
	CODE_TREATMMENT_FINISH			=	0x12,
	CODE_MANUAL_POWER_OFF				=	0x13,
	CODE_NOT_DETECT_HAND_IN_20s	=	0x14,
	CODE_LOW_POWER							=	0x15,
	CODE_OVER_PRESSURE					=	0x16,
	CODE_SELFTEST_SUCCESS				=	0x17,
	CODE_SELFTEST_FAIL					=	0x18,
	CODE_OVER_HEAT							= 0x19,
	CODE_PC_SYN_RTC							= 0x20
}SYSTEM_CODE;

/***********************************
* 類型定義
***********************************/

/***********************************
* 外部函数
***********************************/
void key_power_on_task(void);
void EnterStopMode(void);
void CfgWFI(void);
void CfgALLPins4StopMode(void);
void init_system_afterWakeUp(void);
void record_dateTime(SYSTEM_CODE code);
void Init_RecordPage(void);
void reset_dateTime(void);
//extern INT8U I2C_RecByte(void);



#endif
