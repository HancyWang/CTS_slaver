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
	KEY_STOP_MODE,
	KEY_UPING,
//	KEY_DOWNING,
//	KEY_DOWN_UP,
	KEY_WAKE_UP
	//KEY_UP_DOWN
}KEY_STATE;

/***********************************
* 類型定義
***********************************/

/***********************************
* 外部函数
***********************************/
void key_led_task(void);
void EnterStopMode(void);
void CfgPA0ASWFI(void);
void CfgALLPins4StopMode(void);
void init_system_afterWakeUp(void);
//extern INT8U I2C_RecByte(void);



#endif
