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
//extern INT8U I2C_RecByte(void);



#endif
