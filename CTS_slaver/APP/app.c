

/**
********************************************************************************
* 版權：
* 模块名称：app.c
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
#include "comm_task.h"
#include "CMD_Receive.h"
#include "serial_port.h"
#include "hardware.h"
#include "Motor_pwm.h"
#include "fifo.h"
#include "protocol_module.h"
#include "stm32f0xx_rtc.h"
#include "key_power_on_task.h"
//#include "exp_task.h" 
#include "stm32f0xx_usart.h"
//#include "store_fifo.h"
#include "rtc.h"
/**********************************
*宏定义
***********************************/

/***********************************
* 全局变量
***********************************/
 // 命令接收控制对象
extern CMD_Receive g_CmdReceive; 

//發送數據FIFO
extern FIFO_TYPE send_fifo;
extern uint8_t send_buf[SEND_BUF_LEN];

extern BOOL b_Is_PCB_PowerOn;

extern int16_t zero_point_of_pressure_sensor;
//保存數據
//extern FIFO_TYPE train_store_fifo;
//extern STORE_HEAD exp_store_head;
//extern TRAIN_STORE_DATA_PAGE train_store_data;
/***********************************
* 局部变量
***********************************/

/***********************************
* 函数声明
***********************************/
void test_task(void);
/***********************************
* 函数定义
***********************************/


//初始化任务
void init_task(void)
{
	//初始化硬件
	init_hardware();	
	Motor_PWM_Init();
	
#ifdef _DEBUG
	Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
#endif
	
	//初始化通信相关
	fifoInit(&send_fifo,send_buf,SEND_BUF_LEN);
	UARTInit(g_CmdReceive.m_Buf1, BUF1_LENGTH);	
	Init_Receive(&g_CmdReceive);
	//init_system(b_Is_PCB_PowerOn);

	os_create_task(TaskDataSend, OS_TRUE, SEND_TASK_ID);
	os_create_task(CMD_ProcessTask, OS_TRUE, RECEIVE_TASK_ID);
#ifdef _DEBUG
	os_create_task(Detect_battery_and_tmp,OS_TRUE,TASK_DETECT_BATTERY_ID);
	os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	os_create_task(DetectPalm, OS_TRUE, TASK_DETECT_PALM_ID);
	os_create_task(test_task, OS_TRUE, TEST_TASK_ID);
#else
	os_create_task(key_power_on_task, OS_TRUE, KEY_LED_TASK_ID);
	os_create_task(get_switch_mode,OS_TRUE,TASK_GET_SWITCH_MODE);
	os_create_task(check_selectedMode_ouputPWM,OS_TRUE,TASK_OUTPUT_PWM);
	os_create_task(ReleaseGas, OS_TRUE, TASK_RELEASE_GAS_ID);
	os_create_task(DetectPalm, OS_TRUE, TASK_DETECT_PALM_ID);
	os_create_task(Detect_battery_and_tmp,OS_TRUE,TASK_DETECT_BATTERY_ID);
	os_create_task(led_blink_beep,OS_TRUE,TASK_LED_BINK_BEEP);
	os_create_task(usb_charge_battery,OS_TRUE,TASK_USB_CHARGE_BAT);
	os_create_task(self_test,OS_TRUE,TASK_SELF_TEST);
#endif
	
	os_pend_task(INIT_TASK_ID);
}

extern uint32_t system_loop_time;
static BOOL b_testFlag=1;
static BOOL b_flag1=1;
static int flash_write_cnt=0;
void test_task(void)
{
	static RTC_DateTypeDef data_struct;
	static RTC_TimeTypeDef time_struct;
	Get_DataTime(&data_struct,&time_struct);
	
	if(b_testFlag)
	{
		if(b_flag1)
		{
			reset_dateTime(); //全部复位,变成FF
			Init_RecordPage();
			b_flag1=0;
			set_led(LED_ID_MODE1,TRUE); 
		}
		
//		for(int i=0;i<500;i++)   //模拟测试
//		{
//			record_dateTime(CODE_SYSTEM_POWER_ON);
//			record_dateTime(CODE_TREATMMENT_FINISH);
//		}
//		
		#ifdef _DEBUG
		if(flash_write_cnt==298)
		{
			flash_write_cnt=0;
			b_testFlag=0;
			set_led(LED_ID_MODE1,FALSE); 
		}
		else
		{
			record_dateTime(CODE_SYSTEM_POWER_ON);
			record_dateTime(CODE_TREATMMENT_FINISH);
			flash_write_cnt++;
		}
		#else
		#endif
			
		
//		b_testFlag=0;
	}
	
	
	
	os_delay_ms(TEST_TASK_ID, 50);
}
