//////////////////////////////////////////////////////////////////////////////////	 
			  
//////////////////////////////////////////////////////////////////////////////////
#include "comm_task.h"
#include "fifo.h"
#include "CMD_Receive.h"
#include "os_cfg.h"
#include "stdio.h"
#include "delay.h"
#include "string.h"
#include "app.h"
#include "serial_port.h"
#include "protocol_module.h"
#include "key_power_on_task.h"
#include "Motor_pwm.h"
#include "i2c.h"
#include "hardware.h"
#include "iwtdg.h"

//ÿ��pressure sensor��rate����һ���������Ļ����Զ���һ���ӿڣ���flash�ж�ȡrate
//#define PRESSURE_RATE get_pressure_rate
//#define PRESSURE_RATE 20   //����3PCS,����20
//#define PRESSURE_RATE (FlashReadWord(FLASH_PRESSURE_RATE_ADDR))
//uint32_t pressure_rate;
uint32_t PRESSURE_RATE;
#define PRESSURE_SAFETY_THRESHOLD 180
#define PRESSURE_EMPTY_AIR 5
//y=ax+b
//uint32_t PRESSURE_SENSOR_VALUE;
#define PRESSURE_SENSOR_VALUE(x) (((PRESSURE_RATE)*(x))+zero_point_of_pressure_sensor)

extern int16_t zero_point_of_pressure_sensor;

//ȫ�ֱ���
CMD_Receive g_CmdReceive;  // ������տ��ƶ���
FIFO_TYPE send_fifo;//�l�͔���FIFO
UINT8 send_buf[SEND_BUF_LEN];

//����������λ���������Ĳ���
UINT8 parameter_buf[PARAMETER_BUF_LEN]; 

UINT8 buffer[PARAMETER_BUF_LEN];

UINT16 check_sum;
extern BOOL b_Is_PCB_PowerOn;
extern MCU_STATE mcu_state;
extern BOOL rcvParameters_from_PC;
extern KEY_STATE key_state;
extern const uint8_t default_parameter_buf[PARAMETER_BUF_LEN];

extern uint16_t RegularConvData_Tab[2];

extern uint32_t os_ticks;

//extern BOOL b_usb_charge_bat;
extern USB_CHARGING_STATE usb_charging_state;
extern uint8_t led_beep_ID;

extern USB_DETECT_STATE usb_detect_state;

extern BOOL b_LED_ON_in_turn;
//extern BOOL b_usb_intterruptHappened;
//extern BOOL b_KeyWkUP_InterrupHappened;
//BOOL b_start_powerOn_check=FALSE;


BOOL b_release_gas=FALSE;
BOOL b_palm_checked=FALSE;
BOOL b_bat_detected_ok=FALSE;

BOOL b_no_hand_in_place=FALSE;
BOOL b_end_of_treatment=FALSE;

 uint32_t detectPalm_cnt=0;
 uint32_t noPalm_cnt=0;

 BOOL PWM1_timing_flag=TRUE;
 BOOL PWM2_timing_flag=TRUE; 
 BOOL PWM3_timing_flag=TRUE;
static BOOL PWM4_timing_flag=TRUE;
static BOOL PWM5_timing_flag=TRUE;
 BOOL waitBeforeStart_timing_flag=TRUE;
 BOOL led_bink_timing_flag=TRUE;
 BOOL beep_timing_flag=TRUE;
 BOOL usb_charge_timing_flag=TRUE;
 BOOL key_Press_or_Release_timing_flag=TRUE;
 BOOL key_self_test_timing_flag=TRUE;
//static BOOL switch_bnt_timing_flag=TRUE;
 BOOL b_releaseGas_timing_flag=TRUE;
//static BOOL b_detect_palm=TRUE;
static BOOL* b_timing_flag;

static uint16_t pressure_result;

BOOL b_check_bnt_release=FALSE;

 uint16_t led_bink_cnt;
	uint16_t beep_cnt;
	uint16_t delay_cnt;
	
 BOOL b_Motor_Ready2Shake=TRUE;
 BOOL b_Motor_shake=FALSE;

//uint32_t prev_switchBtn_os_tick;
//uint32_t prev_detect_palm_flag;
uint32_t prev_releaseGas_os_tick;
uint32_t prev_ledBlink_os_tick;
uint32_t prev_selfTest_os_tick;
uint32_t prev_keyPressOrRelease_os_tick;
uint32_t prev_usbCharge_os_tick;
uint32_t prev_beep_os_tick;
uint32_t prev_WaitBeforeStart_os_tick;
uint32_t prev_PWM1_os_tick;
uint32_t prev_PWM2_os_tick;
uint32_t prev_PWM3_os_tick;
uint32_t prev_PWM4_os_tick;
uint32_t prev_PWM5_os_tick;
uint32_t* p_prev_os_tick;

//BOOL b_switch_mode_changed=FALSE;
//typedef enum
//{
//	SWITCH_MODE1=1,
//	SWITCH_MODE2,
//	SWITCH_MODE3
//}SWITCH_MODE;
//SWITCH_MODE switch_mode=SWITCH_MODE1;
uint8_t mode=1;
//uint16_t prev_cnt;
//uint16_t cnt;

//�ֲ�����
//typedef enum
//{
//	LOAD_PARA,  //���ز���
//	GET_MODE,
//	CHECK_PRESSURE,
//	CHECK_PRESSURE_AGAIN,
//	PREV_OUTPUT_PWM,
//	CPY_PARA_TO_BUFFER,
//	OUTPUT_PWM,
//	CHECK_BAT_VOL,
//	LED_RED_BLINK
//}CHCKMODE_OUTPUT_PWM;


//typedef enum
//{
//	PWM_START,
//	PWM_PERIOD,
//	PWM_WAIT_BETWEEN,
//	PWM_WAIT_AFTER,
//	PWM_OUTPUT_FINISH
//}PWM_STATE;

PWM_STATE pwm1_state=PWM_NONE;
PWM_STATE pwm2_state=PWM_NONE;
PWM_STATE pwm3_state=PWM_NONE;

//static uint16_t* p_PWM3_threshold; //ֻ��PWM3����threshold
static PWM_STATE* p_pwm_state;
//static uint16_t* p_PWM_period_cnt;
//static uint16_t* p_PWM_waitBetween_cnt;
//static uint16_t* p_PWM_waitAfter_cnt;
static uint8_t* p_PWM_numOfCycle;
static uint8_t* p_PWM_serial_cnt;

uint16_t PWM_waitBeforeStart_cnt=0;

//uint16_t PWM1_period_cnt=0;
//uint16_t PWM2_period_cnt=0;
//uint16_t PWM3_period_cnt=0;

//uint16_t PWM1_waitBetween_cnt=0;
//uint16_t PWM2_waitBetween_cnt=0;
//uint16_t PWM3_waitBetween_cnt=0;

//uint16_t PWM1_waitAfter_cnt=0;
//uint16_t PWM2_waitAfter_cnt=0;
//uint16_t PWM3_waitAfter_cnt=0;

uint8_t PWM1_numOfCycle=0;
uint8_t PWM2_numOfCycle=0;
uint8_t PWM3_numOfCycle=0;

uint8_t PWM1_serial_cnt=0;
uint8_t PWM2_serial_cnt=0;
uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=WAIT_BEFORE_START;
CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
//uint16_t mode;                      

//uint8_t pwm_buffer[144]={0};
//uint16_t	mode;
static uint8_t pwm1_buffer[1+6*10];
static uint8_t pwm2_buffer[1+6*10];
static uint8_t pwm3_buffer[1+6*10];
//uint8_t pressure;
uint16_t checkPressAgain_cnt=0;
uint8_t wait_cnt=0;


 BOOL b_self_test=FALSE;

BOOL b_Palm_check_complited=FALSE;


USB_CHARGING_STATE usb_charging_state=USB_CHARGE_NONE;
 BOOL b_usb_push_in;
 BOOL b_usb_pull_up;
BOOL b_stop_current_works=FALSE;
BOOL b_LED_ON_in_turn=FALSE;

SELF_TEST_STATE self_tet_state=SELF_TEST_NONE;
//SELF_TEST_STATE self_tet_state=SELF_TEST_DELAY_BEFORE_START; //debug
LED_IN_TURN_STATE led_In_Turn_state=LED_IN_TURN_NONE;


 uint8_t selfTest_delay_Cnt;
 uint8_t nLED_ON_in_turn;
 uint16_t inflate_cnt;
 uint16_t hold_cnt;
 uint8_t deflate_cnt;
//static uint8_t bat_detect_cnt=0;

//typedef enum
//{
//	INIT,
//	BLINK,
//	BEEP
//}LED_BEEP_STATE;

//LED_BEEP_STATE led_beep_state=INIT;

uint8_t sample_cnt;
uint32_t sample_sum;

LED_STATE led_state=LED_INIT;
BEEP_STATE beep_state=BEEP_INIT;
 uint8_t led_beep_ID=0;
 
 //�Լ�
  uint8_t deflate_cnt;
 uint16_t selfTest_inflate_record_1;
 uint16_t selfTest_inflate_record_2;

 uint16_t selfTest_hold_record_1;
 uint16_t selfTest_hold_record_2;
 uint16_t selfTest_deflate_record_1;
 uint16_t selfTest_deflate_record_2;
 uint8_t selfTest_fail_Cnt;
 uint8_t selfTest_fail_period_H;
 uint8_t selfTest_fail_period_L;
 uint8_t selfTest_end_Cnt;
 
 BOOL b_detect_hand_before_system_running=TRUE;
// BOOL b_stop_motors=FALSE;
uint16_t wait_between_total_cnt=0;
 uint8_t value=0;
 
//*********************debug*******************
//cycles_record���������������Եģ���ʽ�İ汾����Ҫ���
#ifdef _DEBUG_TEST_CYCLES
//�ֱ��¼���ܵ�Ȧ��,os_ticks(ÿ����һȦ��¼һ�ε�ǰʱ��),os_ticks(�ܵ�û���ʱ���¼ʱ��)
uint32_t debug_cycles_record[3]={0}; 
//uint32_t debug_cycle_cnt=0; 
#else
#endif
//*********************debug*******************

/*******************************************************************************
*                                �ڲ���������
*******************************************************************************/
static BOOL ModuleUnPackFrame(void);
static BOOL ModuleProcessPacket(UINT8 *pData);
static UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen);


void Set_ReleaseGas_flag()
{
	b_release_gas=TRUE;
}

void Reset_ReleaseGas_flag()
{
	b_release_gas=FALSE;
}

void init_PWMState(void)
{
	PWM1_timing_flag=TRUE;
	PWM2_timing_flag=TRUE; 
	PWM3_timing_flag=TRUE;
	PWM4_timing_flag=TRUE;
	PWM5_timing_flag=TRUE;
	waitBeforeStart_timing_flag=TRUE;
	//switch_bnt_timing_flag=TRUE;
	
	pwm1_state=PWM_NONE;
	pwm2_state=PWM_NONE;
	pwm3_state=PWM_NONE;

	PWM_waitBeforeStart_cnt=0;

//	PWM1_period_cnt=0;
//	PWM2_period_cnt=0;
//	PWM3_period_cnt=0;

//	PWM1_waitBetween_cnt=0;
//	PWM2_waitBetween_cnt=0;
//	PWM3_waitBetween_cnt=0;

//	PWM1_waitAfter_cnt=0;
//	PWM2_waitAfter_cnt=0;
//	PWM3_waitAfter_cnt=0;

	PWM1_numOfCycle=0;
	PWM2_numOfCycle=0;
	PWM3_numOfCycle=0;

	PWM1_serial_cnt=0;
	PWM2_serial_cnt=0;
	PWM3_serial_cnt=0;
}

//���ƣ��������б������Pump,ֹͣ���
void stop_motors_base_on_Pump(uint16_t num)
{
//	b_stop_motors=TRUE;	
	
	pwm1_state=PWM_NONE;  //��PWM1��PWM2��״̬��ΪNONE,����������
	pwm2_state=PWM_NONE;
}

void start_motors_base_on_pump(uint16_t num)
{
//	b_stop_motors=FALSE;
	//��ʱʱ��ҲҪ��ʼ��
	prev_PWM1_os_tick=0;
	prev_PWM2_os_tick=0;
	
	PWM1_timing_flag=TRUE;
	PWM2_timing_flag=TRUE; 
	
	pwm1_state=PWM_START;
	pwm2_state=PWM_START;

//	PWM1_period_cnt=0;
//	PWM2_period_cnt=0;

//	PWM1_waitBetween_cnt=0;
//	PWM2_waitBetween_cnt=0;

//	PWM1_waitAfter_cnt=0;
//	PWM2_waitAfter_cnt=0;

	PWM1_numOfCycle=0;
	PWM2_numOfCycle=0;

//	PWM1_serial_cnt=0;
//	PWM2_serial_cnt=0;
	switch(num)
	{
		case 3:
			PWM1_serial_cnt=1;
			PWM2_serial_cnt=1;
			break;
		case 6:
			PWM1_serial_cnt=2;
			PWM2_serial_cnt=2;
			break;
		case 9:
			PWM1_serial_cnt=3;
			PWM2_serial_cnt=3;
			break;
//		case 12:
//			PWM1_serial_cnt=1;
//			PWM2_serial_cnt=1;
//			break;
		default:
			break;	
	}
}

void LED_Blink_for_alert(uint8_t seconds)
{
	//ע�⣺Motor1,2,3�Ĺرվ��ǲ���д�������档������Ȼ�ͻ����
	//����ֻ��Motor1��ת�����⣬��֪��Ϊʲô
	//�������Ҫ�ر�PWM,Ҫ��Ȼ��Delay_ms��ʱ��PWM�������
//	Motor_PWM_Freq_Dudy_Set(1,100,0);
//	Motor_PWM_Freq_Dudy_Set(2,100,0);
//	Motor_PWM_Freq_Dudy_Set(3,100,0);	
//	Motor_PWM_Freq_Dudy_Set(4,100,0);  
//	Motor_PWM_Freq_Dudy_Set(5,4000,0);
	
	
	//Reset_Timing_Parameter();
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	set_led(LED_ID_GREEN,FALSE);   //�ص���Դ����ɫLED��
	set_led(LED_ID_YELLOW,FALSE);
	//�ر�ģʽָʾ��
	if(mode==1)
	{	
		set_led(LED_ID_MODE1,FALSE); 
	}
	else if(mode==2)
	{
		set_led(LED_ID_MODE2,FALSE);   
	}
	else if(mode==3)
	{
		set_led(LED_ID_MODE3,FALSE);  
	}
	else
	{
		//do nothing
	}
	
	Delay_ms(500);

	//��˸
	for(uint8_t i=0;i<seconds;i++)
	{
		set_led(LED_ID_MODE1,TRUE);
		set_led(LED_ID_MODE2,TRUE);
		set_led(LED_ID_MODE3,TRUE);
		Delay_ms(500);
		set_led(LED_ID_MODE1,FALSE);
		set_led(LED_ID_MODE2,FALSE);
		set_led(LED_ID_MODE3,FALSE);
		Delay_ms(500);
//		IWDG_Feed();   //ι��
	}
}


void No_Hand_IN_PLACE()
{
	Motor_PWM_Freq_Dudy_Set(1,100,0);
	Motor_PWM_Freq_Dudy_Set(2,100,0);
	Motor_PWM_Freq_Dudy_Set(3,100,0);	
	Motor_PWM_Freq_Dudy_Set(4,100,0);  
	Motor_PWM_Freq_Dudy_Set(5,4000,0);
	
	set_led(LED_ID_GREEN,FALSE);   //�ص���Դ����ɫLED��
	
	//������һ��ʹ��while() {is_timing_Xseconds()}
	for(uint8_t i=0;i<3;i++)
	{
		set_led(LED_ID_MODE1,TRUE); 
		set_led(LED_ID_MODE2,TRUE);
		set_led(LED_ID_MODE3,TRUE);
		Motor_PWM_Freq_Dudy_Set(5,4000,50);
		delay_ms(500);
		set_led(LED_ID_MODE1,FALSE); 
		set_led(LED_ID_MODE2,FALSE);
		set_led(LED_ID_MODE3,FALSE);
		Motor_PWM_Freq_Dudy_Set(5,4000,0);
		delay_ms(500);
	}
}


//��ɫLED��˸
void Red_LED_Blink(unsigned char seconds)
{
//	//�������Ҫ�ر�PWM,Ҫ��Ȼ��Delay_ms��ʱ��PWM�������
//	Motor_PWM_Freq_Dudy_Set(1,100,0);
//	Motor_PWM_Freq_Dudy_Set(2,100,0);
//	Motor_PWM_Freq_Dudy_Set(3,100,0);	
//	Motor_PWM_Freq_Dudy_Set(4,100,0);  
//	Motor_PWM_Freq_Dudy_Set(5,4000,0);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_10);
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	set_led(LED_ID_GREEN,FALSE);   //�ص���Դ����ɫLED��
	
	//�ر�ģʽָʾ��
	if(mode==1)
	{	
		set_led(LED_ID_MODE1,FALSE); 
	}
	else if(mode==2)
	{
		set_led(LED_ID_MODE2,FALSE);   
	}
	else if(mode==3)
	{
		set_led(LED_ID_MODE3,FALSE);  
	}
	else
	{
		//do nothing
	}
	
	Delay_ms(500);
	//��˸
	for(uint8_t i=0;i<seconds;i++)
	{
		set_led(LED_ID_YELLOW,TRUE);
		Delay_ms(500);
		set_led(LED_ID_YELLOW,FALSE);
		Delay_ms(500);
//		IWDG_Feed();   //ι��
	}
}

void CalcCheckSum(UINT8* pPacket)
{
	UINT16 dataLen = pPacket[1];
	UINT16 checkSum = 0;
	UINT16 i;

	for (i = 1; i < dataLen; i++)
	{
		checkSum += pPacket[i];
	}

	pPacket[dataLen] = checkSum >> 8;
	pPacket[dataLen+1] = checkSum&0xFF;
}

/*******************************************************************************
** ��������: ModuleUnPackFrame
** ��������: ������մ���
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
 UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
 UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};
BOOL ModuleUnPackFrame(void)
{
	static BOOL sPacketHeadFlag = FALSE;
	static BOOL sPacketLenFlag = FALSE;
	static UINT8 sCurPacketLen = 0;
	static UINT8 sResetByte = 0;
	//static UINT8 sDataBuff[CMD_BUFFER_LENGTH] = {0};	
	//static UINT8 sBackBuff[CMD_BUFFER_LENGTH] = {0};

	UINT8 *pBuff = (UINT8 *)sDataBuff;
	UINT16 dwLen = 0;
	UINT8 byCurChar;

	// �Ӵ��ڻ������ж�ȡ���յ�������
	dwLen = GetBuf2Length(&g_CmdReceive);

	// �����ݽ��н���
	while(0 < dwLen)
	{
		byCurChar = Buf2Read(&g_CmdReceive);

		if (sPacketHeadFlag)
		{
			// ��������ͷ
			if(sPacketLenFlag)
			{
				// ������������
				pBuff[sCurPacketLen] = byCurChar;
				sCurPacketLen ++;
				sResetByte --;

				if (0 >= sResetByte)
				{
					// �������
					// ����У��ͱȽ�
					if (CheckCheckSum(pBuff, pBuff[1]))
					{
						// ������һ����Ч���ݰ�
						memcpy(sBackBuff, sDataBuff, CMD_BUFFER_LENGTH);
						ModuleProcessPacket(sBackBuff);//������*********************************
						//��ֹ�������ն�������ʱ������Ӧ���źų�����
						delay_ms(2);
					}

					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;
					memset(&sDataBuff, 0x00, CMD_BUFFER_LENGTH);
				}													
			}
			else
			{
				if((CMD_BUFFER_LENGTH-1 > byCurChar) && (0 < byCurChar ))// �ݴ�����ֹ���ݰ���Ϊ49��0ʱ��� ����X5����������Ͱ�Ϊ15
				{
					// ������ģ��ĳ���
					sDataBuff[sCurPacketLen] = byCurChar;
					sResetByte = byCurChar;			
					sPacketLenFlag = TRUE;
					sCurPacketLen ++;
				}
				else
				{
					//û�н�����ģ��ĳ���, ���½���
					sPacketHeadFlag = FALSE;
					sPacketLenFlag = FALSE;					
				}
			}
		}
		
		else if (PACK_HEAD_BYTE == byCurChar)		
		{
			// ��������ͷ
			sDataBuff[0] = byCurChar;
			sPacketHeadFlag = TRUE;
			sPacketLenFlag = FALSE;			
			sCurPacketLen = 1;
			sResetByte = 0;
		}

		//pData ++;
		dwLen --;
	}
	return TRUE;
}


/*******************************************************************************
** ��������: CheckCheckSum
** ��������: ��У��
** �䡡  ��: pData ���� nLen����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen)
{
	UINT16 bySum = 0;
	int i;
	// �������ݵ�У���	
	for(i = 1; i < nLen; i++)
	{
		bySum += pData[i];
	}		

	if (bySum == (pData[nLen] << 8)+ pData[nLen + 1])
	{
		return TRUE;
	}
	else
	{
		return FALSE;	
	}
}

/*******************************************************************************
** ��������: ModuleProcessPacket
** ��������: ������
** �䡡  ��: pData ����
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
BOOL ModuleProcessPacket(UINT8 *pData)
{	
	protocol_module_process(pData);

	return TRUE;     	
}


/*******************************************************************************
* �������� : TaskDataSend
* �������� : ���ݷ�������5msִ��һ��
* ������� : arg  ��������ʱ���ݵĲ���
* ������� : ��
* ���ز��� : ��
*******************************************************************************/
void TaskDataSend (void)
{
    UINT8 send_data_buf[SEND_DATA_BUF_LENGTH] = {0};
    UINT16  len;
		
		//protocol_module_send_exp_flag(1);
#ifdef _DEBUG
#else
		if(mcu_state==POWER_ON)
#endif
		
		{
			//ѭ�h
			len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
			if(len)
			{
					UartSendNBytes(send_data_buf, len);
			}
		}
		
		
		os_delay_ms(SEND_TASK_ID, 28);  //markһ��
}

////��ŷ�����
//void ReleaseGas(uint8_t second)
//{
//	GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
//	GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
//	
//	for(uint8_t i=0;i<second;i++)
//	{
//		delay_ms(1000);
//	}
//	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
//	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
//}

void InitKeyWakeUpTiming()
{
	key_Press_or_Release_timing_flag=TRUE;
	prev_keyPressOrRelease_os_tick=0;
}

void Reset_Timing_Parameter()
{
	PWM1_timing_flag=TRUE;
	prev_PWM1_os_tick=0;;
	
	PWM2_timing_flag=TRUE;
	prev_PWM2_os_tick=0;;
	
	PWM3_timing_flag=TRUE;
	prev_PWM3_os_tick=0;;
	
	PWM4_timing_flag=TRUE;
	prev_PWM4_os_tick=0;;

	PWM5_timing_flag=TRUE;
	prev_PWM5_os_tick=0;

	waitBeforeStart_timing_flag=TRUE;
	prev_WaitBeforeStart_os_tick=0;

	b_releaseGas_timing_flag=TRUE;
	prev_releaseGas_os_tick=0;

	led_bink_timing_flag=TRUE;
	prev_ledBlink_os_tick=0;;
										 
	beep_timing_flag=TRUE;
	prev_beep_os_tick=0;
	
	usb_charge_timing_flag=TRUE;
	prev_usbCharge_os_tick=0;

	key_Press_or_Release_timing_flag=TRUE;
	prev_keyPressOrRelease_os_tick=0;
	
	key_self_test_timing_flag=TRUE;
	prev_selfTest_os_tick=0;
}

//typedef enum
//{
//	TIMING_PWM1,
//	TIMING_PWM2,
//	TIMING_PWM3,
//	TIMING_PWM4,
//	TIMING_PWM5,
//	TIMING_WAIT_BEFORE_START,
//	TIMING_RELEASE_GAS,
//	TIMING_NO_HAND_DETECT_LED_BLINK,
//	TIMING_NO_HAND_DETECT_BEEP,
//	TIMING_USB_CHARGE,
//	TIMING_POWER_KEY_PRESS_OR_RELEASE,
//	TIMING_SELF_TEST
//}TIMING_EVENT;

//��ʱx����,n_ms����255s��255000
BOOL Is_timing_Xmillisec(uint32_t n_ms,uint8_t ID)
{
	switch(ID)
	{
		case 1:      //PWM1
			b_timing_flag=&PWM1_timing_flag;
			p_prev_os_tick=&prev_PWM1_os_tick;
			break;
		case 2:     //PWM2
			b_timing_flag=&PWM2_timing_flag;
			p_prev_os_tick=&prev_PWM2_os_tick;
			break;
		case 3:    //PWM3
			b_timing_flag=&PWM3_timing_flag;
			p_prev_os_tick=&prev_PWM3_os_tick;
			break;
		case 4:    //PWM4
			b_timing_flag=&PWM4_timing_flag;
			p_prev_os_tick=&prev_PWM4_os_tick;
			break;
		case 5:   //PWM5
			b_timing_flag=&PWM5_timing_flag;
			p_prev_os_tick=&prev_PWM5_os_tick;
			break;
		case 6:   //wait before start
			b_timing_flag=&waitBeforeStart_timing_flag;
			p_prev_os_tick=&prev_WaitBeforeStart_os_tick;
			break;
//		case 7:   //ģʽ���صİ���ʱ�� ,���ʺ����������
//			b_timing_flag=&switch_bnt_timing_flag;
//			p_prev_os_tick=&prev_switchBtn_os_tick;
//			break;
		case 8:   //release gas
			b_timing_flag=&b_releaseGas_timing_flag;
			p_prev_os_tick=&prev_releaseGas_os_tick;
			break;
//		case 9:
//			b_timing_flag=&b_detect_palm;
//			p_prev_os_tick=&prev_detect_palm_flag;
//			break;
		case 10:                   //û��⵽��ʱ��LED��˸
			b_timing_flag=&led_bink_timing_flag;
			p_prev_os_tick=&prev_ledBlink_os_tick;
			break;
		case 11:                   //û��⵽��ʱ������������
			b_timing_flag=&beep_timing_flag;
			p_prev_os_tick=&prev_beep_os_tick;
		case 12:                   //USB���
			b_timing_flag=&usb_charge_timing_flag;
			p_prev_os_tick=&prev_usbCharge_os_tick;
		case 13:                 //���ػ���
			b_timing_flag=&key_Press_or_Release_timing_flag;
			p_prev_os_tick=&prev_keyPressOrRelease_os_tick;
		case 14:                 //ģʽ�����������Լ�
			b_timing_flag=&key_self_test_timing_flag;
			p_prev_os_tick=&prev_selfTest_os_tick;
			break;
		default:
			break;
	}
	
	if(*b_timing_flag==TRUE)
	{
		*p_prev_os_tick=os_ticks;
		*b_timing_flag=FALSE;
	}
	else
	{
		if(os_ticks+n_ms<os_ticks) //���os_ticks+n_ms����ˣ���ôos_ticks+n_ms��ȻС��os_ticks
		{
			//*p_prev_os_tick=os_ticks;
			if(os_ticks==os_ticks+n_ms)
			{
				*b_timing_flag=TRUE;
				*p_prev_os_tick=0;
				return TRUE;
			}
		}
		else
		{
			if(os_ticks-*p_prev_os_tick>=n_ms)
			{
				*b_timing_flag=TRUE;
				*p_prev_os_tick=0;
				return TRUE;
			}
		}
	}
	return FALSE;
}

void judge_total_PWM3_wait_between_cnt(uint8_t value)
{
	if(value==3&&PWM3_serial_cnt==0)
	{
		wait_between_total_cnt=3;
	}
	else if(value==3&&PWM3_serial_cnt==1)
	{
		wait_between_total_cnt=6;
	}
	else if(value==3&&PWM3_serial_cnt==2)
	{
		wait_between_total_cnt=9;
	}
//	else if(value==2&&PWM3_serial_cnt==3)
//	{
//		wait_between_total_cnt=3;
//	}
	else if(value==1&&PWM3_serial_cnt==4)
	{
		wait_between_total_cnt=12;
	}
//	else if(value==1&&PWM3_serial_cnt==5)
//	{
//		wait_between_total_cnt=3;
//	}
	else
	{
		//do nothing
	}
}

/*******************************************************************************
* �������� : PaintPWM
* �������� : ��PWM
* ������� : 
* ������� : ��
* ���ز��� : ��
*******************************************************************************/
void PaintPWM(unsigned char num,unsigned char* buffer)
{
	uint8_t ELEMENTS_CNT=8; //PWM1��PWM2�е�Ԫ�ظ�����8
	uint8_t THRESHOLD=0;   //�ں����case3�лὫ���¸�ֵ
	
	uint8_t FREQ=2;		//PWM1��PWM2��FREQ��[2]��λ��
	uint8_t DUTY_CYCLE=3;
	uint8_t PERIOD=4;
	uint8_t DWELL=0;    //�ں����case3�лὫ���¸�ֵ
	uint8_t NUM_OF_CYCLES=5;
	uint8_t WAIT_BETWEEN=6;
	uint8_t WAIT_AFTER=7;
	
	switch(num)
	{
		case 1:
			p_pwm_state=&pwm1_state;
//			p_PWM_period_cnt=&PWM1_period_cnt;
//			p_PWM_waitBetween_cnt=&PWM1_waitBetween_cnt;
//			p_PWM_waitAfter_cnt=&PWM1_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM1_numOfCycle;
			p_PWM_serial_cnt=&PWM1_serial_cnt;
			break;
		case 2:
			p_pwm_state=&pwm2_state;
//			p_PWM_period_cnt=&PWM2_period_cnt;
//			p_PWM_waitBetween_cnt=&PWM2_waitBetween_cnt;
//			p_PWM_waitAfter_cnt=&PWM2_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM2_numOfCycle;
			p_PWM_serial_cnt=&PWM2_serial_cnt;
			break;
		case 3:
			ELEMENTS_CNT=10;
			THRESHOLD=2;
			FREQ=3;
			DUTY_CYCLE=4;
			PERIOD=5;  
			DWELL=6;		//����DWELL
			NUM_OF_CYCLES=7;
			WAIT_BETWEEN=8;
			WAIT_AFTER=9;
			p_pwm_state=&pwm3_state;
//			p_PWM_period_cnt=&PWM3_period_cnt;
//			p_PWM_waitBetween_cnt=&PWM3_waitBetween_cnt;
//			p_PWM_waitAfter_cnt=&PWM3_waitAfter_cnt;
			p_PWM_numOfCycle=&PWM3_numOfCycle;
			p_PWM_serial_cnt=&PWM3_serial_cnt;
			break;
		default:
			break;
	}
//	if(b_Is_PCB_PowerOn==FALSE)
//	{
////		ResetAllState();
//		mcu_state=POWER_OFF;
//		state=LOAD_PARA;
//		*p_pwm_state=PWM_START;
////		*p_PWM_period_cnt=0;
////		*p_PWM_waitBetween_cnt=0;
////		*p_PWM_waitAfter_cnt=0;
//		*p_PWM_numOfCycle=0;
//		*p_PWM_serial_cnt=0;
//		//PWM_waitBeforeStart_cnt=0;
//		Motor_PWM_Freq_Dudy_Set(num,100,0);
//	}
//	else
	{
		if(*p_pwm_state==PWM_START)
		{
			//if(*p_PWM_serial_cnt>buffer[0]-1)  //�ж�serial�Ƿ�ȫ��������
			if(PWM3_serial_cnt>buffer[0]-1)  //PWM3�����Ͼ������ˣ����ù�PWM1��PWM2
			{
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Motor_PWM_Freq_Dudy_Set(3,100,0);
				
				pwm1_state=PWM_OUTPUT_FINISH;
				pwm2_state=PWM_OUTPUT_FINISH;
				pwm3_state=PWM_OUTPUT_FINISH;
				
				PWM1_serial_cnt=0;
				PWM2_serial_cnt=0;
				PWM3_serial_cnt=0;
//				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
//				*p_pwm_state=PWM_OUTPUT_FINISH;
//				*p_PWM_serial_cnt=0;
			}
			else
			{
				{
					if(b_release_gas==FALSE)
					{
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]);
						*p_pwm_state=PWM_PERIOD;
					}	
					else
					{
						//������������ʱ����ﲻ��ֹͣ�����������������д���
//						Motor_PWM_Freq_Dudy_Set(1,100,0);
//						Motor_PWM_Freq_Dudy_Set(2,100,0);

						Motor_PWM_Freq_Dudy_Set(3,100,0);
						//������ʼ����������
						state=LOAD_PARA;
//						b_Motor_Ready2Shake=TRUE;
//						b_Palm_check_complited=FALSE;
//						b_Motor_shake=FALSE;
//						nMotorShake_Cnt=0;
						init_PWMState();
						//Set_ReleaseGas_flag();
					}
				}
			}
		}
		
		if(*p_pwm_state==PWM_PERIOD)   //period
		{
			if(num==3)  //PWM3
			{
				uint16_t ret=ADS115_readByte(0x90);
				//PRESSURE_SENSOR_VALUE=buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+THRESHOLD]*pressure_rate+zero_point_of_pressure_sensor;
				//if(ret>=((pressure_rate*buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+THRESHOLD])+zero_point_of_pressure_sensor))
				if(ret>=PRESSURE_SENSOR_VALUE(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+THRESHOLD]))
				{
//					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
//					*p_pwm_state=PWM_PERIOD;
					//if(ret<(pressure_rate*PRESSURE_SAFETY_THRESHOLD+zero_point_of_pressure_sensor))
					if(ret<PRESSURE_SENSOR_VALUE(PRESSURE_SAFETY_THRESHOLD))
					{
						#if 0
//						Motor_PWM_Freq_Dudy_Set(1,100,0);
//						Motor_PWM_Freq_Dudy_Set(2,100,0);
//						Motor_PWM_Freq_Dudy_Set(3,100,0);
//						//������ʼ����������
//						state=LOAD_PARA;
////						b_Motor_Ready2Shake=TRUE;
////						b_Palm_check_complited=FALSE;
////						b_Motor_shake=FALSE;
////						nMotorShake_Cnt=0;
//						init_PWMState();
//						
////						GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
////						GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
////					//	delay_ms(4000);   //���ﲻ�У�Ҫι��
						#endif
						  
						*p_pwm_state=PWM_DWELL;
						//�ر�pump����Ҫ�򿪵�ŷ���������hold������
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);  
						PWM3_timing_flag=TRUE;
						prev_PWM3_os_tick=0;
						//Set_ReleaseGas_flag();
					}
					else   //���������ȫֵ
					{
						*p_pwm_state=PWM_OVER_SAFTY_THRESHOLD;
						
						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						Motor_PWM_Freq_Dudy_Set(3,100,0);
						
						GPIO_SetBits(GPIOB,GPIO_Pin_10);
						GPIO_SetBits(GPIOB,GPIO_Pin_11);  //�������
					} 
				}
				else
				{
					if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+PERIOD]*1000,num))
					{
						++(*p_PWM_numOfCycle);
						*p_pwm_state=PWM_WAIT_BETWEEN;
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
					}
					
				}
			}
			else  //PWM1��PWM2
			{
				if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+PERIOD]*1000,num))
				{
					++(*p_PWM_numOfCycle);
					*p_pwm_state=PWM_WAIT_BETWEEN;
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
				}
			}		
		}
		
		if(*p_pwm_state==PWM_OVER_SAFTY_THRESHOLD)
		{
			//������ʼ����������
			state=LOAD_PARA;
			init_PWMState();
			
			LED_Blink_for_alert(5);
			
			EnterStopMode();
			init_system_afterWakeUp();
		}
		
		if(*p_pwm_state==PWM_DWELL) //ֻ��PWM3����DWELL
		{
			//���period��״̬
			//��ʼ��ʱDWELL��ô��ʱ��
			
			if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DWELL]*1000,num))
			{
				wait_between_total_cnt++; //��dwell->wait between�л���ʱ���������ж��ٸ�
				++(*p_PWM_numOfCycle);
				*p_pwm_state=PWM_WAIT_BETWEEN;
				//Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
				GPIO_SetBits(GPIOB,GPIO_Pin_10); //��valve����
				GPIO_SetBits(GPIOB,GPIO_Pin_11);
			}
			else 
			{
				//if(ADS115_readByte(0x90)>=(pressure_rate*PRESSURE_SAFETY_THRESHOLD+zero_point_of_pressure_sensor))
				if(ADS115_readByte(0x90)>=PRESSURE_SENSOR_VALUE(PRESSURE_SAFETY_THRESHOLD))
				{
					*p_pwm_state=PWM_OVER_SAFTY_THRESHOLD;

					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					Motor_PWM_Freq_Dudy_Set(3,100,0);

					GPIO_SetBits(GPIOB,GPIO_Pin_10);
					GPIO_SetBits(GPIOB,GPIO_Pin_11);  //�������
				}
			}
		}
		
		
		if(*p_pwm_state==PWM_WAIT_BETWEEN)   //wait between
		{
			if(*p_PWM_numOfCycle==buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+NUM_OF_CYCLES])
			{
				//ע�⣺�ⲿ�ֵ��߼��Ƕ��Ƶģ�û��ͨ����
				//��������Ilan��Ҫ�󣬿��ܻ���仯
				
				if(num==3)
				{
					
					value=buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+NUM_OF_CYCLES];
				
					judge_total_PWM3_wait_between_cnt(value);
					//wait_between_total_cnt++;
					if(wait_between_total_cnt==3||wait_between_total_cnt==6||wait_between_total_cnt==9||wait_between_total_cnt==12)
					{
						//ֹͣ���
						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						//Motor_PWM_Freq_Dudy_Set(2,100,0);
						//ǿ�н�PWM1��PWM2������һ��serial,û������Ĳ���ֱ�����꣬������ʼ��
						stop_motors_base_on_Pump(wait_between_total_cnt);
					}
					else
					{
						//do nothing
					}
				}

				*p_pwm_state=PWM_WAIT_AFTER;
				*p_PWM_numOfCycle=0;
				
			}
			else
			{
				if(num==3)
				{
					if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_BETWEEN]*1000,num))
					{
						if(wait_between_total_cnt==4||wait_between_total_cnt==7||wait_between_total_cnt==10)
						{
						start_motors_base_on_pump(wait_between_total_cnt);
						}
						
						b_release_gas=FALSE;
						GPIO_ResetBits(GPIOB,GPIO_Pin_10);
						GPIO_ResetBits(GPIOB,GPIO_Pin_11);
						
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]); 
						*p_pwm_state=PWM_PERIOD;
						
					}
					else
					{
						GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
						GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
					}
				}
				else
				{
					if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_BETWEEN]*1000,num))
					{
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]); 
						//*p_PWM_waitBetween_cnt=0;
						*p_pwm_state=PWM_PERIOD;
					}
				}
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_AFTER)  //wait after
		{	
			if(num==3)
			{
				if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_AFTER]*1000,num))
				{
					b_release_gas=FALSE;
					GPIO_ResetBits(GPIOB,GPIO_Pin_10);
					GPIO_ResetBits(GPIOB,GPIO_Pin_11);
					
					state=OUTPUT_PWM;
					*p_PWM_numOfCycle=0;
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
					++(*p_PWM_serial_cnt);
					*p_pwm_state=PWM_START;
				}
				else
				{
//					GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
//					GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
				}
			}
			else
			{
				if(wait_between_total_cnt==3||wait_between_total_cnt==6||wait_between_total_cnt==9||wait_between_total_cnt==12)
				{
					state=OUTPUT_PWM;
					*p_PWM_numOfCycle=0;
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
					//++(*p_PWM_serial_cnt);
					*p_pwm_state=PWM_START;
				}
				else
				{
					if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_AFTER]*1000,num))
					{
						state=OUTPUT_PWM;
						*p_PWM_numOfCycle=0;
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
						++(*p_PWM_serial_cnt);
						*p_pwm_state=PWM_START;
						//*p_PWM_waitAfter_cnt=0;
					}
				}
			}
		}
	}

}

void ResetParameter(unsigned char* buffer)
{
	//���������flash���ݿ�����buffer��
	for(int i=0;i<PARAMETER_BUF_LEN;i++)
	{
		buffer[i]=default_parameter_buf[i];
	}
	//����flash
	FlashWrite(FLASH_WRITE_START_ADDR,buffer,PARAMETER_BUF_LEN/4);
}

void CheckFlashData(unsigned char* buffer)
{
	 uint16_t j=0;
	//������ݳ������Ĭ�ϵ�����
	if(buffer[0]<1||buffer[0]>255) //cycles
	{
		ResetParameter(buffer);
		return;
	}
	//buffer[1]����Ҫ��⣬���ķ�ΧΪ0-255
	for(int i=0;i<54;i++)  //һ��54��serial
	{
		j++;                 //1.������һ�� ����0x11,0x12֮��
		if(buffer[2+j++]>1) //2.enable
		{
			ResetParameter(buffer);
			return;
		}
//		//MODE1_PWM3(98,152),MODE2_PWM3(248,302),MODE3_PWM3(398,452)
//		if((j>=96&&j<=150)||(j>=246&&j<=300)||(j>=396&&j<=450))  //3.threshold
//		{
//			j++;
//		}
		//MODE1_PWM3(98,158),MODE2_PWM3(254,314),MODE3_PWM3(410,470)
		if((j>=98-2&&j<=158-2)||(j>=254-2&&j<=314-2)||(j>=410-2&&j<=470-2))  //3.threshold
		{
			j++;
		}
		
		if(buffer[2+j++]==0)  //3.freq
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<5||buffer[2+j]>100) //4.duty cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		if(buffer[2+j++]==0)            //5.period
		{
			ResetParameter(buffer);
			return;
		}
		//MODE1_PWM3(98,158),MODE2_PWM3(254,314),MODE3_PWM3(410,470)
		if((j>=98-2&&j<=158-2)||(j>=254-2&&j<=314-2)||(j>=410-2&&j<=470-2))  //6.dwell
		{
			j++;
		}
		if(buffer[2+j]<1||buffer[2+j]>250)            //6.number of cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		
		j++;                                  //7.wait between
		j++;																	//8.wait after
	}
	j=0;
}



/*******************************************************************************
** ��������: FillUpPWMbuffer
** ��������: ����serial���к��������pwm1_buffer,pwm2_buffer,pwm3_buffer
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void FillUpPWMbuffer(uint8_t* dest,uint8_t* src,uint8_t PWMX)
{
	uint8_t serial_cnt=0;
	uint8_t j=1;
	uint8_t num=0;
	for(int i=0;i<6;i++)
	{
//		//PWM1��PWM2��ÿ��serial��8��Ԫ�أ���PWM3��9��Ԫ��
		//PWM1��PWM2��ÿ��serial��8��Ԫ�أ���PWM3��10��Ԫ��
		if(PWMX==1||PWMX==2)
		{
			num=8;
		}
		else if(PWMX==3)
		{
//			num=9;
			num=10;
		}
		else
		{
			//do nothing
		}
		
		if(src[num*i+1]==0x01)
		{
			uint8_t k;
			//for(k=0;k<8;k++)
			for(k=0;k<num;k++)
			{
				dest[j++]=src[num*i+k];
			}
			serial_cnt++;
		}
	}
	//��������ж��ٸ���enable�ģ�Ȼ����䵽dest[0]��
	dest[0]=serial_cnt;  
}

 
/*******************************************************************************
** ��������: get_switch_mode
** ��������: ��ȡ��������Ӧ��ģʽ��ͨ���������º��ͷŵļ�����ж�
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void get_switch_mode()
{
//	//TEST
//	static uint16_t test;
//	test=ADS115_readByte(0x90);
	
	static uint8_t switch_mode_cnt=0;
	static uint8_t release_btn_cnt=0;
	
	//���е�ʱ��(��⵽�־ͱ�ʾ��ʼ��PWM��)
	if(b_Palm_check_complited)  
	{
		//1.����ʵ��ģʽ�����л�
		//2.�����������Լ�ģʽ(��д������ܾ͵���)
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
		{
			if(switch_mode_cnt==5)
			{
				b_check_bnt_release=TRUE;
				switch_mode_cnt=0;
			}
			else
			{
				switch_mode_cnt++;
			}
		}
		else
		{
			switch_mode_cnt=0;
			if(b_check_bnt_release==TRUE)
			{
				if(release_btn_cnt==5&&!b_self_test&&!b_LED_ON_in_turn)
				//if(release_btn_cnt==5&&!b_self_test)
				//if(release_btn_cnt==5)
				{
					//���Is_timing_Xmillisec(5000,14)�еļ�������ֹ�ۼӶ������Լ칦��
					key_self_test_timing_flag=TRUE;
					prev_selfTest_os_tick=0;

					release_btn_cnt=0;
					//b_check_bnt_pressed=FALSE;
					b_check_bnt_release=FALSE;
					//�л�����ģʽ
					if(mode==1)
					{ 
						mode=2;		
						set_led(LED_ID_MODE1,FALSE); //�ص�LED1����LED2
						set_led(LED_ID_MODE2,TRUE);
					}
					else if(mode==2)
					{
						mode=3;
						set_led(LED_ID_MODE2,FALSE);   //�ص�LED2����LED3
						set_led(LED_ID_MODE3,TRUE);
					}
					else if(mode==3)
					{
						mode=1;
						set_led(LED_ID_MODE3,FALSE);  //�ص�LED3����LED1
						set_led(LED_ID_MODE1,TRUE);
					}
					else
					{
						//do nothing
					}
					
					init_PWMState();
					state=LOAD_PARA;

					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);  //����д���Σ�����ͳ�����
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					Motor_PWM_Freq_Dudy_Set(3,100,0);
					Motor_PWM_Freq_Dudy_Set(4,100,0);
					Motor_PWM_Freq_Dudy_Set(5,100,0);
				}
				else
				{
					release_btn_cnt++;
				}
			}
		}
	}
	else
	{
		//1.����ʵ��ģʽ�����л�
		//2.���������Լ�ģʽ,�����Լ������ػ�,�������л�ģʽ
		if(!b_stop_current_works&&mcu_state==POWER_ON&&!b_self_test)  
	//if(!b_stop_current_works&&mcu_state==POWER_ON) 
		{
			if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
			{
				//if(!b_self_test&&!b_Palm_check_complited)
				//if(!b_Palm_check_complited)
				{
					if(Is_timing_Xmillisec(5000,14))  //�������
					{
						b_self_test=TRUE;
						self_tet_state=SELF_TEST_DELAY_BEFORE_START;
						init_PWMState();
						state=LOAD_PARA;

						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						Motor_PWM_Freq_Dudy_Set(3,100,0);
						Motor_PWM_Freq_Dudy_Set(4,100,0);
						Motor_PWM_Freq_Dudy_Set(5,100,0);
					}
					else
					{
						if(switch_mode_cnt==5)
						{
							b_check_bnt_release=TRUE;
							switch_mode_cnt=0;
						}
						else
						{
							switch_mode_cnt++;
						}
					}
				}
			}
			else
			{
				switch_mode_cnt=0;
				if(b_check_bnt_release==TRUE)
				{
					if(release_btn_cnt==5&&!b_self_test&&!b_LED_ON_in_turn)
					//if(release_btn_cnt==5&&!b_self_test)
					//if(release_btn_cnt==5)
					{
						//���Is_timing_Xmillisec(5000,14)�еļ�������ֹ�ۼӶ������Լ칦��
						key_self_test_timing_flag=TRUE;
						prev_selfTest_os_tick=0;
						
						release_btn_cnt=0;
						//b_check_bnt_pressed=FALSE;
						b_check_bnt_release=FALSE;
						//�л�����ģʽ
						if(mode==1)
						{
							mode=2;		
							set_led(LED_ID_MODE1,FALSE); //�ص�LED1����LED2
							set_led(LED_ID_MODE2,TRUE);
						}
						else if(mode==2)
						{
							mode=3;
							set_led(LED_ID_MODE2,FALSE);   //�ص�LED2����LED3
							set_led(LED_ID_MODE3,TRUE);
						}
						else if(mode==3)
						{
							mode=1;
							set_led(LED_ID_MODE3,FALSE);  //�ص�LED3����LED1
							set_led(LED_ID_MODE1,TRUE);
						}
						else
						{
							//do nothing
						}
						init_PWMState();
						state=LOAD_PARA;

						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);  //����д���Σ�����ͳ�����
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						Motor_PWM_Freq_Dudy_Set(3,100,0);
						Motor_PWM_Freq_Dudy_Set(4,100,0);
						Motor_PWM_Freq_Dudy_Set(5,100,0);
					}
					else
					{
						release_btn_cnt++;
					}
				}
			}
		}
	}
	
	#if 0
//	if(!b_stop_current_works&&mcu_state==POWER_ON&&!b_self_test)  
//	//if(!b_stop_current_works&&mcu_state==POWER_ON) 
//	{
//		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
//		{
//			//if(!b_self_test&&!b_Palm_check_complited)
//			//if(!b_Palm_check_complited)
//			{
//				if(Is_timing_Xmillisec(5000,14))  //�������
//				{
//					b_self_test=TRUE;
//					self_tet_state=SELF_TEST_DELAY_BEFORE_START;
//					init_PWMState();
//					state=LOAD_PARA;

//					Motor_PWM_Freq_Dudy_Set(1,100,0);
//					Motor_PWM_Freq_Dudy_Set(2,100,0);
//					Motor_PWM_Freq_Dudy_Set(3,100,0);
//					Motor_PWM_Freq_Dudy_Set(4,100,0);
//					Motor_PWM_Freq_Dudy_Set(5,100,0);
//				}
//				else
//				{
//					if(switch_mode_cnt==5)
//					{
//						b_check_bnt_release=TRUE;
//						switch_mode_cnt=0;
//					}
//					else
//					{
//						switch_mode_cnt++;
//					}
//				}
//			}
//		}
//		else
//		{
//			//if(b_Palm_check_complited)
//			{
//				switch_mode_cnt=0;
//				if(b_check_bnt_release==TRUE)
//				{
//					if(release_btn_cnt==5&&!b_self_test&&!b_LED_ON_in_turn)
//					//if(release_btn_cnt==5&&!b_self_test)
//					//if(release_btn_cnt==5)
//					{
//						//���Is_timing_Xmillisec(5000,14)�еļ�������ֹ�ۼӶ������Լ칦��
//						key_self_test_timing_flag=TRUE;
//						prev_selfTest_os_tick=0;
//						
//						
//						release_btn_cnt=0;
//						//b_check_bnt_pressed=FALSE;
//						b_check_bnt_release=FALSE;
//						//�л�����ģʽ
//						if(mode==1)
//						{
//		//					while(1)
//		//					{//��֤��
//		//					}   
//							mode=2;
//							//�����и����⣬Ϊʲô���ܵ���API��һ�þ�Ӱ��PWM2?
//							//Motor_PWM_Freq_Dudy_Set(2,100,0);����д���Σ�����ͻ�����������
//		//					GPIO_SetBits(LED_PORT,LED_ID_MODE1);
//		//					GPIO_ResetBits(LED_PORT,LED_ID_MODE2);		
//							set_led(LED_ID_MODE1,FALSE); //�ص�LED1����LED2
//							set_led(LED_ID_MODE2,TRUE);
//						}
//						else if(mode==2)
//						{
//							mode=3;
//		//					GPIO_SetBits(LED_PORT,LED_ID_MODE2);
//		//					GPIO_ResetBits(LED_PORT,LED_ID_MODE3);
//							set_led(LED_ID_MODE2,FALSE);   //�ص�LED2����LED3
//							set_led(LED_ID_MODE3,TRUE);
//						}
//						else if(mode==3)
//						{
//							mode=1;
//		//					GPIO_SetBits(LED_PORT,LED_ID_MODE3);
//		//					GPIO_ResetBits(LED_PORT,LED_ID_MODE1);
//							set_led(LED_ID_MODE3,FALSE);  //�ص�LED3����LED1
//							set_led(LED_ID_MODE1,TRUE);
//						}
//						else
//						{
//							//do nothing
//						}
//						//b_switch_mode_changed=TRUE;
//						init_PWMState();
//						state=LOAD_PARA;

//						Motor_PWM_Freq_Dudy_Set(1,100,0);
//						Motor_PWM_Freq_Dudy_Set(2,100,0);  //����д���Σ�����ͳ�����
//						Motor_PWM_Freq_Dudy_Set(2,100,0);
//						Motor_PWM_Freq_Dudy_Set(3,100,0);
//						Motor_PWM_Freq_Dudy_Set(4,100,0);
//						Motor_PWM_Freq_Dudy_Set(5,100,0);
//					}
//					else
//					{
//						release_btn_cnt++;
//					}
//				}
//			}
//			
//		}
//	}
#endif
	
	os_delay_ms(TASK_GET_SWITCH_MODE, 20);
}

void ReleaseGas()
{
	if(!b_Is_PCB_PowerOn)
	{
		return;
	}
	if(b_release_gas==TRUE)
	{
//		state=LOAD_PARA;
//		init_PWMState();
//		mcu_state=POWER_OFF;
		
		GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
		GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
		if(Is_timing_Xmillisec(4000,8))
		{
			GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			GPIO_ResetBits(GPIOB,GPIO_Pin_11);
			Reset_ReleaseGas_flag();
			//mcu_state=POWER_ON;
		}
	}
	os_delay_ms(TASK_RELEASE_GAS_ID, 50);
}
	 

//�����ж�PA0��device�п���stopģʽ��Ҳ�п�����running

//typedef enum
//{
//	USB_CHARGE_NONE,
//	USB_CHARGING,
//	USB_CHARGED_FULL,
//	USB_CHARGE_FAULT,
//	USB_CHARGE_NO_BATTERY
//}USB_CHARGING_STATE;


void usb_charge_battery()
{
	//������һ���������
	if(usb_detect_state==USB_PUSH_IN)  //�ող���USB,
	{
		Motor_PWM_Freq_Dudy_Set(1,100,0);
		Motor_PWM_Freq_Dudy_Set(2,100,0);
		Motor_PWM_Freq_Dudy_Set(3,100,0);
		Motor_PWM_Freq_Dudy_Set(4,100,0);
		Motor_PWM_Freq_Dudy_Set(5,4000,0);
		delay_ms(50);
		
		if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)==1)
		{
			usb_detect_state=USB_INSERTED;
			usb_charging_state=USB_CHECK_CHARG;
			b_stop_current_works=TRUE; //����ϵͳ���ڹ���״̬�����Ǵӻ��ѵ�����������Ҫֹͣ����,�����κ����
			
			if(!b_Is_PCB_PowerOn)
			{
				b_Is_PCB_PowerOn=TRUE;
				mcu_state=POWER_ON;	
				//key_state=KEY_WAKE_UP;  		
				state=LOAD_PARA;

				init_PWMState();
			}
		}
		else
		{
			usb_detect_state=USB_FAIL_INSERT;
		}
	}
	
	if(usb_detect_state==USB_FAIL_INSERT)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
		
	if(usb_detect_state==USB_PULL_UP)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
	
	if(usb_charging_state==USB_CHECK_CHARG)
	{
		//set_led(LED_ID_GREEN,TRUE);
		//PCB��charge��stdby�����ź�layout���ˣ���layout�������ˣ�����������ϵ���λ��
//		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)  //�����
		if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==0&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)  //�����
		{
			usb_charging_state=USB_CHARGING;
			//led_beep_ID=3;
			
			//usb_charging_state=USB_CHARGE_FAULT;
		}
//		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==0&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)  //������
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)  //������
		{
			usb_charging_state=USB_CHARGED_FULL;
		}
		else if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)  //����
		{
			usb_charging_state=USB_CHARGE_FAULT;
			
//			usb_charging_state=USB_CHARGING;
//			led_beep_ID=3;
		}
		else
		{
			usb_charging_state=USB_CHARGE_FAULT;
			//usb_charging_state=USB_CHARGE_NONE;
		}
	}
	
	if(usb_charging_state==USB_CHARGE_FAULT)
	{
		EnterStopMode();
		init_system_afterWakeUp();
	}
	
	if(usb_charging_state==USB_CHARGING)
	{
		static BOOL b_charge_flag=TRUE;
		
		set_led(LED_ID_YELLOW,FALSE); 
//		set_led(LED_ID_GREEN,FALSE);
		set_led(LED_ID_MODE1,FALSE); 
		set_led(LED_ID_MODE2,FALSE); 
		set_led(LED_ID_MODE3,FALSE); 
		
		if(b_charge_flag)
		{
			if(Is_timing_Xmillisec(1000,12)) 
			{
				set_led(LED_ID_GREEN,TRUE);
				b_charge_flag=FALSE;
				//�ж�һ�£���س�����û��
//				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==0&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==1)  //������
				if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_5)==1&&GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==0)  //������
				{		
					usb_charging_state=USB_CHARGED_FULL;
				}
			}
		}
		else
		{
			if(Is_timing_Xmillisec(1000,12))
			{
				set_led(LED_ID_GREEN,FALSE);
				b_charge_flag=TRUE;
			}
		}
	}
	
	if(usb_charging_state==USB_CHARGED_FULL)
	{
		set_led(LED_ID_GREEN,TRUE);
	}

	os_delay_ms(TASK_USB_CHARGE_BAT, 20);
}


uint16_t diff_of_two_values(uint16_t value1,uint16_t value2)
{
	if(value1>value2)
	{
		return value1-value2;
	}
	else
	{
		return value2-value1;
	}
}



//�Լ�
void self_test()
{
	//�رղ��Σ��ƣ��������ݵ���ʱ
	if(self_tet_state==SELF_TEST_DELAY_BEFORE_START)
	{
////		//�����������(��⵽�֣��ʹ���ʼ������)������������Լ�
//		if(b_Palm_check_complited)
//		{
//			b_self_test=FALSE;
//			self_tet_state=SELF_TEST_NONE;
//		}
//		else
		{
			if(!b_no_hand_in_place&&!b_end_of_treatment) //������ 1.��60sû��⵽������ �� 2.���ƽ�������,�����Լ�
			{
				//�ر�ģʽ��
				set_led(LED_ID_MODE1,FALSE);
				set_led(LED_ID_MODE2,FALSE);
				set_led(LED_ID_MODE3,FALSE);
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Motor_PWM_Freq_Dudy_Set(3,100,0);
				Motor_PWM_Freq_Dudy_Set(4,100,0);
				Motor_PWM_Freq_Dudy_Set(5,4000,0);
				
		//		state=LOAD_PARA;
		//		init_PWMState();
				
				if(selfTest_delay_Cnt==10)
				{
					selfTest_delay_Cnt=0;
		//			self_tet_state=SELF_TEST_START;
					self_tet_state=SELF_TEST_DEFLATE_BEFORE_START;
					
					led_In_Turn_state=LED_IN_TURN_MODE1;
					GPIO_SetBits(GPIOB,GPIO_Pin_10);
					GPIO_SetBits(GPIOB,GPIO_Pin_11);
				}
				else
				{
					selfTest_delay_Cnt++;
				}
			}
		}

	}
	
	//��ʼ֮ǰҪ����
	if(self_tet_state==SELF_TEST_DEFLATE_BEFORE_START)
	{
		
		b_LED_ON_in_turn=TRUE;

		if(deflate_cnt*20==4000)  //4s�ķ���ʱ��
		{
			deflate_cnt=0;
			GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			GPIO_ResetBits(GPIOB,GPIO_Pin_11);
			
			self_tet_state=SELF_TEST_START;
		}
		else
		{
			deflate_cnt++;
		}
	}
	
	if(self_tet_state==SELF_TEST_START)
	{
		//b_LED_ON_in_turn=TRUE;
		self_tet_state=SELF_TEST_INFLATE;
//		led_In_Turn_state=LED_IN_TURN_MODE1;
		
		Motor_PWM_Freq_Dudy_Set(3,100,70);  //��PWM3����ʼ������ballon��
				//��ˮ��
		//1.�����(pwm3)������5s
		//2.�رյ�ŷ�����5s��ȡ���Ƚ�
		//3.�رյ�ŷ�
	}
	
	//��ˮ��
	if(b_LED_ON_in_turn==TRUE)  
	{
		if(led_In_Turn_state==LED_IN_TURN_MODE1)
		{
			if(nLED_ON_in_turn==40)
			{
				nLED_ON_in_turn=0;
				led_In_Turn_state=LED_IN_TURN_MODE2;
				set_led(LED_ID_MODE1,FALSE);
			}
			else
			{
				nLED_ON_in_turn++;
				set_led(LED_ID_MODE1,TRUE);
			}
		}
		
		if(led_In_Turn_state==LED_IN_TURN_MODE2)
		{
			if(nLED_ON_in_turn==40)
			{
				nLED_ON_in_turn=0;
				led_In_Turn_state=LED_IN_TURN_MODE3;
				set_led(LED_ID_MODE2,FALSE);
			}
			else
			{
				nLED_ON_in_turn++;
				set_led(LED_ID_MODE2,TRUE);
			}
		}
		
		if(led_In_Turn_state==LED_IN_TURN_MODE3)
		{
			if(nLED_ON_in_turn==40)
			{
				nLED_ON_in_turn=0;
				led_In_Turn_state=LED_IN_TURN_MODE1;
				set_led(LED_ID_MODE3,FALSE);
			}
			else
			{
				nLED_ON_in_turn++;
				set_led(LED_ID_MODE3,TRUE);
			}
		}
	}
	
	//����
	if(self_tet_state==SELF_TEST_INFLATE)
	{
		if(inflate_cnt*20==8000) //����8s
		{
			Motor_PWM_Freq_Dudy_Set(3,100,0);  //������ϣ�����hold�׶Σ�����Ƿ�©��
			self_tet_state=SELF_TEST_HOLD;
			inflate_cnt=0;
		}
		else
		{
			inflate_cnt++;
			//�������Ƿ�������
			//��������⣬�����ǵ�����ˣ�Ҳ������©��
			if(inflate_cnt==5)
			{
				//��¼����1
				selfTest_inflate_record_1=ADS115_readByte(0x90);
			}
			if(inflate_cnt==399)
			{
				//��¼����2
				selfTest_inflate_record_2=ADS115_readByte(0x90);
				//����2-����1
				if(diff_of_two_values(selfTest_inflate_record_2,selfTest_inflate_record_1)<150)
				//if(selfTest_inflate_record_2-selfTest_inflate_record_1<150)  //�����ֵС��150����Ϊ������
				{
					Motor_PWM_Freq_Dudy_Set(3,100,0);
					self_tet_state=SELF_TEST_FAIL;
					inflate_cnt=0;
				}
			}
		}
	}
	
	if(self_tet_state==SELF_TEST_HOLD)
	{
		if(hold_cnt*20==30000) //hold 30��
		{
			hold_cnt=0;
			//����PB10,PB11,����,��������׶�
			GPIO_SetBits(GPIOB,GPIO_Pin_10);
			GPIO_SetBits(GPIOB,GPIO_Pin_11);
			
			self_tet_state=SELF_TEST_DEFLATE;
		}
		else
		{
			hold_cnt++;
			//����Ƿ�©����ʱ�価����
			if(hold_cnt==1)
			{
				//��¼����1
				selfTest_hold_record_1=ADS115_readByte(0x90);
			}
			if(hold_cnt==1490)
			{
				//��¼����2
				selfTest_hold_record_2=ADS115_readByte(0x90);
				//����2-����1
				if(diff_of_two_values(selfTest_hold_record_1,selfTest_hold_record_2)>300)
				//if(selfTest_hold_record_1-selfTest_hold_record_2>100)  //hold�׶Σ������ֵ����60����Ϊ©��
				{
					self_tet_state=SELF_TEST_FAIL;
					hold_cnt=0;
					//�Բ�fail��ҲҪ�������ŵ�
					GPIO_SetBits(GPIOB,GPIO_Pin_10);
					GPIO_SetBits(GPIOB,GPIO_Pin_11);
				}		
			}
		}
	}
	
	if(self_tet_state==SELF_TEST_DEFLATE)
	{
		if(deflate_cnt*20==5000) //����5s
		{
			//�رյ�ŷ�PB10,PB11,����end�׶�
			GPIO_ResetBits(GPIOB,GPIO_Pin_10);
			GPIO_ResetBits(GPIOB,GPIO_Pin_11);
			
			deflate_cnt=0;
		}
		else
		{
			deflate_cnt++;
			//����ŷ�������ʱ�価����
			if(deflate_cnt==1)
			{
				//��¼����1
				selfTest_deflate_record_1=ADS115_readByte(0x90);
			}
			if(deflate_cnt==200)
			{
				//��¼����2
				selfTest_deflate_record_2=ADS115_readByte(0x90);
				//����2-����1
				if(diff_of_two_values(selfTest_deflate_record_1,selfTest_deflate_record_2)<100)
				//if(selfTest_deflate_record_1-selfTest_deflate_record_2<100)  //�����ֵС��100,˵����û��������ŷ�����
				{
					self_tet_state=SELF_TEST_FAIL;
					deflate_cnt=0;
				}
				else
				{
					self_tet_state=SELF_TEST_END;
				}
			}
		}
	}
	
	if(self_tet_state==SELF_TEST_FAIL)
	{
		//���ƣ�ֱ�ӽ���͹���
		
		//������д������Ƚ������������Σ��Լ��в���Ӧ�������� ,�Ѿ����
		
		b_LED_ON_in_turn=FALSE;
		
		set_led(LED_ID_GREEN,FALSE);
		set_led(LED_ID_YELLOW,FALSE);
		
		if(selfTest_fail_Cnt==5)  //��ʮ��
		{
			selfTest_fail_Cnt=0;
			b_self_test=FALSE;  
			
			EnterStopMode();
			init_system_afterWakeUp();
		}
		else
		{
			//���������һ������
			if(selfTest_fail_period_H*20==300)
			{
				if(selfTest_fail_period_L*20==300)
				{
					selfTest_fail_Cnt++;
					selfTest_fail_period_H=0;
					selfTest_fail_period_L=0;
				}
				else
				{
					selfTest_fail_period_L++;
					set_led(LED_ID_MODE1,TRUE);
					set_led(LED_ID_MODE2,TRUE);
					set_led(LED_ID_MODE3,TRUE);
					//set_led(LED_ID_YELLOW,TRUE);
					Motor_PWM_Freq_Dudy_Set(5,4000,80);
				}
			}
			else
			{
				selfTest_fail_period_H++;
				set_led(LED_ID_MODE1,FALSE);
				set_led(LED_ID_MODE2,FALSE);
				set_led(LED_ID_MODE3,FALSE);
				//set_led(LED_ID_YELLOW,FALSE);
				Motor_PWM_Freq_Dudy_Set(5,4000,0);
			}
		}

	}
	
	if(self_tet_state==SELF_TEST_END)
	{
		b_LED_ON_in_turn=FALSE;  //�ر���ˮ��
		
		//�Ƴ���5s����ʾ�Լ�ok,Ȼ�����͹���
		
		if(selfTest_end_Cnt*20==5000)  //����5s����ʾ�Լ�ok
		{
			b_self_test=FALSE;

			selfTest_end_Cnt=0;
			set_led(LED_ID_MODE1,FALSE);
			set_led(LED_ID_MODE2,FALSE);
			set_led(LED_ID_MODE3,FALSE);
			EnterStopMode();
			init_system_afterWakeUp();
		}
		else
		{
			selfTest_end_Cnt++;
			set_led(LED_ID_MODE1,TRUE);
			set_led(LED_ID_MODE2,TRUE);
			set_led(LED_ID_MODE3,TRUE);
		}
	}
	
	os_delay_ms(TASK_SELF_TEST, 20);
}

void led_blink_beep()
{
//	//û��⵽��
	static uint16_t nohand_ledCnt=5;
	static uint16_t nohand_beepCnt=5;
	static uint16_t nohand_led_tm=500;  //��ʱ200ms
	static uint16_t nohand_beep_tm=500;
	
//	//���ƽ���
	static uint16_t endTreatment_ledCnt=5;
	static uint16_t endTreatment_beepCnt=5;
	static uint16_t endTreatment_led_tm=500;  //��ʱ500ms
	static uint16_t endTreatment_beep_tm=500; 
	
	//���,     ֻ�е�����û��beep
//	static uint16_t charge_ledCnt=255; //�����Ҫһֱ��
//	static uint16_t charge_led_tm=1000;  //��ʱ1000ms
	
	static uint16_t* p_ledCnt;
	static uint16_t* p_beepCnt;
	static uint16_t* p_led_tm;
	static uint16_t* p_beep_tm;

	//if(!b_usb_intterruptHappened)
	{
		if(led_state==LED_INIT)
		{
			if(b_no_hand_in_place||b_end_of_treatment)
			//	if(b_no_hand_in_place||b_end_of_treatment||b_usb_charge_bat)
			{
				//�ӳ�һ���������Ч����һЩ
				if(delay_cnt==4)  //�ӳ�4*50=200ms
				{
					delay_cnt=0;
					
					if(led_beep_ID==2)
					{
						set_led(LED_ID_GREEN,TRUE);
					}
					else
					{
						set_led(LED_ID_MODE1,TRUE); 
						set_led(LED_ID_MODE2,TRUE);
						set_led(LED_ID_MODE3,TRUE);
					}
//					set_led(LED_ID_MODE1,TRUE); 
//					set_led(LED_ID_MODE2,TRUE);
//					set_led(LED_ID_MODE3,TRUE);
					led_state=LED_ON;
					
					Motor_PWM_Freq_Dudy_Set(5,4000,50);
					beep_state=BEEP_ON;
				}
				else
				{
					delay_cnt++;
					
					//�رղ������
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					Motor_PWM_Freq_Dudy_Set(3,100,0);	
					Motor_PWM_Freq_Dudy_Set(4,100,0);  
					Motor_PWM_Freq_Dudy_Set(5,4000,0);
					Motor_PWM_Freq_Dudy_Set(5,4000,0);
					
					set_led(LED_ID_GREEN,FALSE);   //�ص���Դ����ɫLED��
					
					//�ص�ģʽ��
					set_led(LED_ID_MODE1,FALSE);
					set_led(LED_ID_MODE2,FALSE);
					set_led(LED_ID_MODE3,FALSE);
				}
			}
		}
		
	//		if(beep_state==BEEP_INIT)
	//		{
	////			Motor_PWM_Freq_Dudy_Set(5,4000,50);
	//			beep_state=BEEP_ON;
	//		}

		if(led_state==LED_ON)
		{
			//uint16_t tm=0;
			switch(led_beep_ID)
			{
				case 1:   //û��⵽��
					//tm=300;
					p_led_tm=&nohand_led_tm;
					p_ledCnt=&nohand_ledCnt;
					break;
				case 2:   //���ƽ���
		///			tm=500;
					p_led_tm=&endTreatment_led_tm;
					p_ledCnt=&endTreatment_ledCnt;
	//				case 3:   //usb���
	//					p_led_tm=&charge_led_tm;
	//					p_ledCnt=&charge_ledCnt;
					break;
				default:
					break;
			}
			if(Is_timing_Xmillisec((uint32_t)(*p_led_tm),10))  //ON
			//if(Is_timing_Xmillisec(tm,10))
			{
				if(led_beep_ID==2)
				{
					set_led(LED_ID_GREEN,FALSE);
				}
				else
				{
					set_led(LED_ID_MODE1,FALSE); 
					set_led(LED_ID_MODE2,FALSE);
					set_led(LED_ID_MODE3,FALSE);
				}
		
				led_state=LED_OFF;
				led_bink_cnt++;
			}
		}

		if(led_state==LED_OFF)
		{
			if(led_bink_cnt==*p_ledCnt)
			//if(led_bink_cnt==5)
			{
				led_bink_cnt=0;
				led_state=LED_END;
			}
			else
			{
				if(Is_timing_Xmillisec((uint32_t)(*p_led_tm),10))  //500ms,OFF
				//if(Is_timing_Xmillisec((tm),10)) 
				{
//					set_led(LED_ID_MODE1,TRUE); 
//					set_led(LED_ID_MODE2,TRUE);
//					set_led(LED_ID_MODE3,TRUE);
					if(led_beep_ID==2)
					{
						set_led(LED_ID_GREEN,TRUE);
					}
					else
					{
						set_led(LED_ID_MODE1,TRUE); 
						set_led(LED_ID_MODE2,TRUE);
						set_led(LED_ID_MODE3,TRUE);
					}
					
					led_state=LED_ON;
				}
			}
		}

		if(beep_state==BEEP_ON)
		{
			//uint16_t tm=0;
			switch(led_beep_ID)
			{
				case 1:   //û��⵽��
					//tm=300;
					p_beep_tm=&nohand_beep_tm;
					p_beepCnt=&nohand_beepCnt;
					break;
				case 2:   //���ƽ���
					//tm=500;
					p_beep_tm=&endTreatment_beep_tm;
					p_beepCnt=&endTreatment_beepCnt;
					break;
				default:
					break;
			}
	//			if(Is_timing_Xmillisec((uint32_t)(*p_tm),11))
			if(Is_timing_Xmillisec((uint32_t)(*p_beep_tm),11))
			{
				Motor_PWM_Freq_Dudy_Set(5,4000,0);
				//Motor_PWM_Freq_Dudy_Set(5,4000,0);
				beep_state=BEEP_OFF;
				beep_cnt++;
			}
		}
		
		if(beep_state==BEEP_OFF)
		{
			if(beep_cnt==*p_beepCnt)
			{
				beep_cnt=0;
				beep_state=BEEP_END;
			}
			else
			{
				if(Is_timing_Xmillisec((uint32_t)(*p_beep_tm),11))
				//if(Is_timing_Xmillisec((tm),11))
				{
					Motor_PWM_Freq_Dudy_Set(5,4000,50);
					beep_state=BEEP_ON;
				}
			}
		}
		
		if(led_state==LED_END&&beep_state==BEEP_END)
		{
//			b_no_hand_in_place=FALSE;
//			b_end_of_treatment=FALSE;
//			led_state=LED_INIT;
//			beep_state=BEEP_INIT;
//			led_bink_cnt=0;
//			beep_cnt=0;
			
			//����stopģʽ
			EnterStopMode();
			//����֮�����³�ʼ��
			init_system_afterWakeUp();
		}
	}
	
	os_delay_ms(TASK_LED_BINK_BEEP, 50);
}


void Detect_battery_and_tmp()
{
	uint16_t result_0;
	uint16_t result_1;
	result_0=RegularConvData_Tab[0];  //��Ӧ���ǵ�ص�ѹ���
	result_1=RegularConvData_Tab[1];  //��Ӧ�����¶ȼ��

	//����ص�ѹ�Լ������¶�
	//�¶�����Ϊ60�ȣ�ϵ��Ϊ0.302,  0.302*10K=3020ohm ,��Ӧ��ѹΪ3*(3020/(10k+3020))=0.695v
	//0.695/3*4096=950,���С��950����ʾ�¶ȳ�����60��
	//�¶����޲���Ҫ
	if(!b_stop_current_works)  //������ʱ�򣬲ż���غ��¶�
	{
		//��������֮�󣬽���һ�μ��
		if(b_Is_PCB_PowerOn==FALSE) //���������ִ�У�b_Is_PCB_PowerOn����FALSE
		{
			if((result_0>=2389))
			{
				b_bat_detected_ok=TRUE;
			}
			else
			{
				b_bat_detected_ok=FALSE;
				
				//�������Ҫ�ر�PWM,Ҫ��Ȼ��Delay_ms��ʱ��PWM�������
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Motor_PWM_Freq_Dudy_Set(3,100,0);	
				Motor_PWM_Freq_Dudy_Set(4,100,0);  
				Motor_PWM_Freq_Dudy_Set(5,4000,0);
				//��ɫLED��3s���ػ�
				Red_LED_Blink(5);
				//LED_Blink_for_alert(5);
				
				mcu_state=POWER_OFF;
				//����stopģʽ
				EnterStopMode();
				//����֮�����³�ʼ��
				init_system_afterWakeUp();
			}
		}
		else
		{
			//���������󣬽��в������
			if(sample_cnt==20)
			{
				uint16_t sample_avg;
				sample_avg=(sample_sum/=20); 
				sample_cnt=0;
				if((sample_avg>=2252))
				{
					b_bat_detected_ok=TRUE;
				}
				else
				{
					b_bat_detected_ok=FALSE;
					
					Motor_PWM_Freq_Dudy_Set(1,100,0);
					Motor_PWM_Freq_Dudy_Set(2,100,0);
					Motor_PWM_Freq_Dudy_Set(3,100,0);	
					Motor_PWM_Freq_Dudy_Set(4,100,0);  
					Motor_PWM_Freq_Dudy_Set(5,4000,0);
					//��ɫLED��3s���ػ�
					Red_LED_Blink(5);
					//LED_Blink_for_alert(5);
#ifdef _DEBUG_TEST_CYCLES
					debug_cycles_record[2]=os_ticks;  //��¼�ػ�ǰ��ʱ��
					FlashWrite(FLASH_ADDR_RECORD_CYCLES,(uint8_t*)debug_cycles_record,3);
#else
#endif					
					
					mcu_state=POWER_OFF;
					//����stopģʽ
					EnterStopMode();
					//����֮�����³�ʼ��
					init_system_afterWakeUp();
				}
			}
			else
			{
				sample_cnt++;
				sample_sum+=result_0;
			}
		}
		
		if(result_1<950)
		{
			static uint8_t n_over_tmperature;
			//�����¶ȹ��ߣ�3��mode����+yellow led��������Ҫ��
			if(n_over_tmperature==1)
			{
				n_over_tmperature=0;
				
				LED_Blink_for_alert(5);
			
				mcu_state=POWER_OFF;
				//����stopģʽ
				EnterStopMode();
				//����֮�����³�ʼ��
				init_system_afterWakeUp();
			}
			else
			{
				Motor_PWM_Freq_Dudy_Set(1,100,0);
				Motor_PWM_Freq_Dudy_Set(2,100,0);
				Motor_PWM_Freq_Dudy_Set(3,100,0);	
//				init_PWMState();
//				mcu_state=POWER_OFF;
				pwm1_state=PWM_NONE;
				pwm2_state=PWM_NONE;
				pwm3_state=PWM_NONE;
				n_over_tmperature++;
			}
		}
	}

	os_delay_ms(TASK_DETECT_BATTERY_ID, 50);
}

void reset_hand_detect_state()
{
	detectPalm_cnt=0;
	noPalm_cnt=0;
	b_Motor_Ready2Shake=TRUE;
	b_Motor_shake=FALSE;
	b_palm_checked=FALSE;
	//����ô��
}	

//�����������
void DetectPalm()
{
	if(!b_Is_PCB_PowerOn)
	{
		return;
	}
	//�������й����У����������USB�������������״̬
	//�����ڽ���60s�������ʱ���Ѿ�����30s,����������ϵͳ���ѵ�ʱ�򻹼ǵ���30s
	//USB_PUSH_IN
	//if(usb_detect_state==USB_PUSH_IN)
	//if(b_usb_charge_bat)
	if(b_stop_current_works)
	{
		detectPalm_cnt=0;
		noPalm_cnt=0;
	}
	else
	{
		if(mcu_state==POWER_ON)
		{		
			//1.����Ǹտ����������  
			if(b_detect_hand_before_system_running)
			//if(TRUE)
			{
				//�����һ�£���ʾ��⵽����
				if(b_Motor_Ready2Shake)
				{
					if(b_Motor_shake)
					{
						static uint8_t nMotorShake_Cnt=0;
						if(nMotorShake_Cnt==25)
						{
							nMotorShake_Cnt=0;
							b_Motor_Ready2Shake=FALSE;
							Motor_PWM_Freq_Dudy_Set(1,100,0);
							Motor_PWM_Freq_Dudy_Set(2,100,0);
							Motor_PWM_Freq_Dudy_Set(3,100,0);
							b_Palm_check_complited=TRUE;   //������
						}
						else
						{
							//Motor_PWM_Freq_Dudy_Set(1,100,80);
//							Motor_PWM_Init();
							Motor_PWM_Freq_Dudy_Set(1,100,80);
							Motor_PWM_Freq_Dudy_Set(2,100,80);
							Motor_PWM_Freq_Dudy_Set(2,100,80);
							
							//Motor_PWM_Freq_Dudy_Set(3,100,80);
							//GPIO_ResetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
							nMotorShake_Cnt++;
						}
					}
				}
				
				if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)==0)
				{
					if(detectPalm_cnt*KEY_LED_PERIOD==2*1000)  //����2s�������λ�����ʾ����⵽��
					{
						detectPalm_cnt=0;
						noPalm_cnt=0;
						
						b_palm_checked=TRUE;
						mcu_state=POWER_ON;
						b_Motor_shake=TRUE;
					}
					else
					{
						detectPalm_cnt++;
					}	
				}
				else
				{
					detectPalm_cnt=0;
					if(noPalm_cnt*KEY_LED_PERIOD==20*1000)  //�տ�����ʱ��û����⵽�֣�����ʱ������20s
					{
						noPalm_cnt=0;
						if(!b_self_test)
						{
							b_palm_checked=FALSE;

							b_no_hand_in_place=TRUE;
							led_beep_ID=1;
							
							mcu_state=POWER_OFF;
						}

					}
					else
					{
						noPalm_cnt++;
					}
				}
			}
			//2.������Ѿ�����������PWM�Ѿ�����ˣ����ʱ����ͻȻ��ʧ(����1s��Ϊ��ʧ)
			else
			{
				if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13)==1) //���û����⵽����
				{
					if(noPalm_cnt*KEY_LED_PERIOD==200)  //200ms��û��⵽����Ϊ����ʧ��
					{
						noPalm_cnt=0;
						//�رղ������
						Motor_PWM_Freq_Dudy_Set(1,100,0);
						Motor_PWM_Freq_Dudy_Set(2,100,0);
						Motor_PWM_Freq_Dudy_Set(3,100,0);
						
						GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 
						GPIO_SetBits(GPIOB,GPIO_Pin_11);
						
						b_Palm_check_complited=FALSE;
						state=LOAD_PARA;
						init_PWMState();
						b_detect_hand_before_system_running=TRUE;
						
					}
					else
					{
						noPalm_cnt++;
					}
				}
				else
				{
					noPalm_cnt=0;
				}
			}
		}
		
	}
	
	
	os_delay_ms(TASK_DETECT_PALM_ID, 20);
}

//�ݴ����������sensorб��ֵ��[10,100]֮�䣬Ϊok�������10��֮��͸�sensorĬ��ֵ20
void get_pressure_sensor_rate()
{
	uint8_t readCnt=0;
	do
	{
		if(readCnt==10) //�����10�ζ�����[10,100]֮�䣬��Ϊ������
		{
			readCnt=0;
			PRESSURE_RATE=20;
			return;
		}
		PRESSURE_RATE=FlashReadWord(FLASH_PRESSURE_RATE_ADDR);
		readCnt++;
	}while(PRESSURE_RATE<10||PRESSURE_RATE>100);
}

/*******************************************************************************
** ��������: check_selectedMode_ouputPWM
** ��������: ���ģʽ������Ӧ�����PWM����
** �䡡  ��: ��
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void check_selectedMode_ouputPWM()
{
//	pressure_result=ADS115_readByte(0x90);
	//�������һ��״̬������ȡcycle_cnt;
	static uint8_t cycle_cnt=0;
#ifdef _DEBUG
#else
	if(b_Palm_check_complited==TRUE&&!b_stop_current_works&&!b_self_test)
#endif
	
	{
		if(!b_release_gas)
		{
			//1.��flash�м��ز������ڴ�
			if(state==LOAD_PARA)      
			{
				
				uint8_t len=PARAMETER_BUF_LEN/4;  
				uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
				//FlashWrite(FLASH_WRITE_START_ADDR,(uint8_t*)tmp,PARAMETER_BUF_LEN/4);//debug
				//��ȡflash���ݵ�buffer��
				FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
				memcpy(buffer,tmp,PARAMETER_BUF_LEN);
				
				CheckFlashData(buffer);
				state=WAIT_BEFORE_START;

				get_pressure_sensor_rate();
				
				//PRESSURE_RATE=20;
#ifdef _DEBUG_TEST_CYCLES
#else
				//�ȴ򿪵�ŷ���wait_before_start��ʱ������������������ɺ�У��sensor
				GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
				GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
#endif
				b_detect_hand_before_system_running=FALSE;  //ϵͳ��������֮����Ҫ��������һ�����Ƽ��ķ���
				reset_hand_detect_state();
			}
			
			//2.ִ��wait before start
			if(state==WAIT_BEFORE_START)  //��ʼԤ�����PWM����
			{
#ifdef _DEBUG_TEST_CYCLES
				if(Is_timing_Xmillisec(5*buffer[1]*1000,6))  //����cyclesʱ������5����5*240(4min)=20min
#else
				if(Is_timing_Xmillisec(buffer[1]*1000,6))
#endif
				{
					cycle_cnt=buffer[0];  //��ȡcycle����ֵ
					--cycle_cnt; // ��Ϊstate=GET_MODE������Ĵ����ȫ��ִ��һ�Σ��൱���Ѿ�ִ�й�һ��cycle�ˣ���������Ҫ��1
					
					state=CPY_PARA_TO_BUFFER;
					
					GPIO_ResetBits(GPIOB,GPIO_Pin_10); 
					GPIO_ResetBits(GPIOB,GPIO_Pin_11); 
					Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
				} 
			}

			if(state!=IDLE)
			{
				if(state==GET_CYCLE_CNT)
				{
					if(cycle_cnt==0)
					{
#ifdef _DEBUG_TEST_CYCLES
						state=LOAD_PARA;
					
						debug_cycles_record[0]++;
						debug_cycles_record[1]=os_ticks;
						FlashWrite(FLASH_ADDR_RECORD_CYCLES,(uint8_t*)debug_cycles_record,3);
#else
						state=IDLE;
						cycle_cnt=buffer[0];
#endif
					}
					else
					{
						state=CPY_PARA_TO_BUFFER;
						cycle_cnt--;
					}
				}
				
				//3.����ѡ���ģʽ�����ݿ�����pwm_buffer
				if(state==CPY_PARA_TO_BUFFER)  //����ѡ���ģʽ����para��䵽pwm_buffer��
				{
//					uint8_t pwm_buffer[144+6]; //144=6*8*3
					uint8_t pwm_buffer[156];
					
					memset(pwm1_buffer,0,1+6*10); //49=1(��Ч��serial����,�����ж��ٸ�enable)+6*8���Ͱ�������6*10
					memset(pwm2_buffer,0,1+6*10);
					memset(pwm3_buffer,0,1+6*10);
					//mode=1;
					switch(mode)
					{
						case 1:
//							memcpy(pwm_buffer,buffer+2,144+6);  //MODE1��PWM1,PWM2,PWM3 
							memcpy(pwm_buffer,buffer+2,156);						
							break;
						case 2:
							memcpy(pwm_buffer,buffer+158,156); //MODE2��PWM1,PWM2,PWM3
							break;
						case 3:
							memcpy(pwm_buffer,buffer+314,156); //MODE3��PWM1,PWM2,PWM3
							break;
						default:
							break;
					}
					FillUpPWMbuffer(pwm1_buffer,pwm_buffer,1);
					FillUpPWMbuffer(pwm2_buffer,pwm_buffer+48,2);
					FillUpPWMbuffer(pwm3_buffer,pwm_buffer+96,3);
					
					//�����һ��У�飬����number 0f cycles�ǲ���3-3-3-2-1-1
					
					state=CHECK_PRESSURE;
				}
				
				//4.���ѹ��
				if(state==CHECK_PRESSURE) //���ѹ��
				{
					pressure_result=ADS115_readByte(0x90);
					//if(pressure_result>=buffer[0]*70)  //ѹ���ﵽthreshold���������PWMģʽ,����75Ϊб�ʣ�5mmgH��Ӧ5*70+700
					//pressure_result=900;
					//if(pressure_result<=70*5)  
					//����Ӧ����<=5mmgH���������У�5mmgH�ǹ̶�ֵ��Ŀ���Ǽ��ballom�е����壬û������������PWM
					//if(pressure_result<=(pressure_rate*PRESSURE_EMPTY_AIR+zero_point_of_pressure_sensor)) 
					if(pressure_result<=PRESSURE_SENSOR_VALUE(PRESSURE_EMPTY_AIR))
					{
						//state=PREV_OUTPUT_PWM;
						state=OUTPUT_PWM;
						//Calibrate_pressure_sensor(&zero_point_of_pressure_sensor);
						pwm1_state=PWM_START;
						pwm2_state=PWM_START;
						pwm3_state=PWM_START;
					}
					else //�������5mmgH,ֱ�ӷ���
					{
						//state=CHECK_PRESSURE_AGAIN;
						
						//�����ŷ���4s��֮��������
						//ReleaseGas(4);
						Set_ReleaseGas_flag();
						state=CHECK_PRESSURE;
					}
				}
				
				//5.��ʼ�������
				if(state==OUTPUT_PWM) //�����趨�Ĳ��������PWM1,PWM2,PWM3
				{			
					if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH&&pwm3_state==PWM_OUTPUT_FINISH)
					{
						state=GET_CYCLE_CNT;  //�����һ�ֺ��ȥcycle��һ
						init_PWMState();
						
						
						//����������
						if(cycle_cnt==0)  
						{
#ifdef _DEBUG_TEST_CYCLES     //����cycle�Ĺ��̲������������
#else
							b_end_of_treatment=TRUE;
							led_beep_ID=2;
#endif
						}
					}		
					else
					{
//						PaintPWM(3,pwm3_buffer);
						PaintPWM(1,pwm1_buffer); 
						PaintPWM(2,pwm2_buffer);
						PaintPWM(3,pwm3_buffer);
						
					}
				}
			}
			else
			{
				//���cycle_cnt=0,������κ�PWM
				//do nothing
			}
		}
	}
//	else
//	{
//		//����͹���ģʽ
////		EnterStopMode();
////		init_system_afterWakeUp();
////		Motor_PWM_Init();
//	}
//	IWDG_Feed();   //ι��
	os_delay_ms(TASK_OUTPUT_PWM, CHECK_MODE_OUTPUT_PWM);
}
/*******************************************************************************
** ��������: CMD_ProcessTask
** ��������: �����������
** �䡡  ��: arg  ��������ʱ���ݵĲ���
** �䡡  ��: ��
** ȫ�ֱ���: ��
** ����ģ��: ��
*******************************************************************************/
void CMD_ProcessTask (void)
{
#ifdef _DEBUG
#else
	if(mcu_state==POWER_ON)
#endif
	
	{
		//ѭ�h
		ReceiveData(&g_CmdReceive);//�������ݵ�������
		ModuleUnPackFrame();//�����
	}

	os_delay_ms(RECEIVE_TASK_ID, 100);
}
