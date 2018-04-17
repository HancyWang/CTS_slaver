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

////434�����ݣ�2������У��λ
//#define PARAMETER_BUF_LEN 436

//CTS��452(434+18)���ݳ���+2��У��λ+2�ֽ�(Ϊ�˲���)=456 
#define PARAMETER_BUF_LEN 456

//�ֲ�����
typedef enum
{
	LOAD_PARA,  //���ز���
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
#endif
