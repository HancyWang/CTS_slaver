#ifndef __APP_H
#define __APP_H

#include "os_cfg.h"

/***********************************
* 头文件
***********************************/

/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/

/***********************************
* 類型定義
***********************************/
#define BOOL unsigned char

typedef enum{
	INIT_TASK_ID = 0,
	KEY_LED_TASK_ID,
	TASK_OUTPUT_PWM,
	SEND_TASK_ID,
	//TASK_OUTPUT_PWM,
	RECEIVE_TASK_ID,
	//KEY_LED_TASK_ID,
	//TASK_OUTPUT_PWM,
	EXP_DETECT_SAVE_TASK_ID,
	EXP_READ_SEND_TASK_ID,
	TEST_TASK_ID,
	TASK_MAX_ID
}TASK_ID;


typedef struct{
	uint8_t run_status;//
	
}CONFIG_TYPE;
/***********************************
* 外部函数
***********************************/
void init_task(void);
//void init_system(void);
void init_system(BOOL bWakeUp);
#endif
