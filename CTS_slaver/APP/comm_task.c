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
#include "key_led_task.h"
#include "Motor_pwm.h"
#include "i2c.h"
#include "key_led_task.h"
#include "hardware.h"


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

static BOOL PWM1_timing_flag=TRUE;
static BOOL PWM2_timing_flag=TRUE; 
static BOOL PWM3_timing_flag=TRUE;
static BOOL PWM4_timing_flag=TRUE;
static BOOL PWM5_timing_flag=TRUE;
static BOOL waitBeforeStart_timing_flag=TRUE;
//static BOOL switch_bnt_timing_flag=TRUE;
static BOOL* b_timing_flag;

//uint32_t prev_switchBtn_os_tick;
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

PWM_STATE pwm1_state=PWM_START;
PWM_STATE pwm2_state=PWM_START;
PWM_STATE pwm3_state=PWM_START;

//static uint16_t* p_PWM3_threshold; //ֻ��PWM3����threshold
static PWM_STATE* p_pwm_state;
//static uint16_t* p_PWM_period_cnt;
//static uint16_t* p_PWM_waitBetween_cnt;
//static uint16_t* p_PWM_waitAfter_cnt;
static uint8_t* p_PWM_numOfCycle;
static uint8_t* p_PWM_serial_cnt;

uint16_t PWM_waitBeforeStart_cnt=0;

uint16_t PWM1_period_cnt=0;
uint16_t PWM2_period_cnt=0;
uint16_t PWM3_period_cnt=0;

uint16_t PWM1_waitBetween_cnt=0;
uint16_t PWM2_waitBetween_cnt=0;
uint16_t PWM3_waitBetween_cnt=0;

uint16_t PWM1_waitAfter_cnt=0;
uint16_t PWM2_waitAfter_cnt=0;
uint16_t PWM3_waitAfter_cnt=0;

uint8_t PWM1_numOfCycle=0;
uint8_t PWM2_numOfCycle=0;
uint8_t PWM3_numOfCycle=0;

uint8_t PWM1_serial_cnt=0;
uint8_t PWM2_serial_cnt=0;
uint8_t PWM3_serial_cnt=0;

//volatile CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
CHCKMODE_OUTPUT_PWM state=LOAD_PARA;
//uint16_t mode;                      

//uint8_t pwm_buffer[144]={0};
//uint16_t	mode;
static uint8_t pwm1_buffer[49+6];
static uint8_t pwm2_buffer[49+6];
static uint8_t pwm3_buffer[49+6];
uint8_t pressure;
uint16_t checkPressAgain_cnt=0;
uint8_t wait_cnt=0;
/*******************************************************************************
*                                �ڲ���������
*******************************************************************************/
static BOOL ModuleUnPackFrame(void);
static BOOL ModuleProcessPacket(UINT8 *pData);
static UINT8 CheckCheckSum(UINT8* pData, UINT8 nLen);


void init_PWMState(void)
{
	PWM1_timing_flag=TRUE;
	PWM2_timing_flag=TRUE; 
	PWM3_timing_flag=TRUE;
	PWM4_timing_flag=TRUE;
	PWM5_timing_flag=TRUE;
	waitBeforeStart_timing_flag=TRUE;
	//switch_bnt_timing_flag=TRUE;
	
	pwm1_state=PWM_START;
	pwm2_state=PWM_START;
	pwm3_state=PWM_START;

	PWM_waitBeforeStart_cnt=0;

	PWM1_period_cnt=0;
	PWM2_period_cnt=0;
	PWM3_period_cnt=0;

	PWM1_waitBetween_cnt=0;
	PWM2_waitBetween_cnt=0;
	PWM3_waitBetween_cnt=0;

	PWM1_waitAfter_cnt=0;
	PWM2_waitAfter_cnt=0;
	PWM3_waitAfter_cnt=0;

	PWM1_numOfCycle=0;
	PWM2_numOfCycle=0;
	PWM3_numOfCycle=0;

	PWM1_serial_cnt=0;
	PWM2_serial_cnt=0;
	PWM3_serial_cnt=0;
}

//��ɫLED��˸
void Red_LED_Blink(uint8_t seconds)
{
	for(uint8_t i=0;i<seconds;i++)
	{
		set_led(LED_ID_YELLOW,TRUE);
		Delay_ms(500);
		set_led(LED_ID_YELLOW,FALSE);
		Delay_ms(500);
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

		//ѭ�h
		len = fifoReadData(&send_fifo, send_data_buf, SEND_DATA_BUF_LENGTH);
		if(len)
		{
				UartSendNBytes(send_data_buf, len);
		}
		
		os_delay_ms(SEND_TASK_ID, 30);  //markһ��
}

//��ŷ�����
void ReleaseGas(uint8_t second)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_10);  	 //�򿪵�ŷ�1
	GPIO_SetBits(GPIOB,GPIO_Pin_11);			//�򿪵�ŷ�2
	for(uint8_t i=0;i<second;i++)
	{
		delay_ms(1000);
	}
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
}

//��ʱx����,n_ms����255s��255000
BOOL Is_timing_Xmillisec(uint32_t n_ms,uint8_t PWM_ID)
{
	switch(PWM_ID)
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
				return TRUE;
			}
		}
		else
		{
			if(os_ticks-*p_prev_os_tick>=n_ms)
			{
				*b_timing_flag=TRUE;
				return TRUE;
			}
		}
	}
	return FALSE;
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
	uint8_t ELEMENTS_CNT=8;
	uint8_t THRESHOLD=2;
	
	uint8_t FREQ=2;
	uint8_t DUTY_CYCLE=3;
	uint8_t PERIOD=4;
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
			ELEMENTS_CNT=9;
			THRESHOLD=2;
			FREQ=3;
			DUTY_CYCLE=4;
			PERIOD=5;
			NUM_OF_CYCLES=6;
			WAIT_BETWEEN=7;
			WAIT_AFTER=8;
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
//		*p_PWM_period_cnt=0;
//		*p_PWM_waitBetween_cnt=0;
//		*p_PWM_waitAfter_cnt=0;
//		*p_PWM_numOfCycle=0;
//		*p_PWM_serial_cnt=0;
//		//PWM_waitBeforeStart_cnt=0;
//		Motor_PWM_Freq_Dudy_Set(num,100,0);
//	}
//	else
	{
		if(*p_pwm_state==PWM_START)
		{
			if(*p_PWM_serial_cnt>buffer[0]-1)  //�ж�serial�Ƿ�ȫ��������
			{
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
				*p_pwm_state=PWM_OUTPUT_FINISH;
				*p_PWM_serial_cnt=0;
			}
			else
			{
				//�ù�����δ��֤����PCBA��������֤
				if(num==3)  //���ΪPWM3,��Ҫ�ж�threshold
				{
					//�����ȡ������ѹֵС���趨��ֵ������Ϊballom�е�û�����壬���Կ�ʼ���PWM
					if(ADS115_readByte(0x90)<buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+THRESHOLD])
					{
						Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]);
						*p_pwm_state=PWM_PERIOD;
					}
					else
					{
						//if pressure <160 �򿪷��ţ�����4s
						if(ADS115_readByte(0x90)<160*70)
						{
							ReleaseGas(4);
							//������ʼ����������
							state=LOAD_PARA;
							//state=GET_MODE;
							init_PWMState();
						}
						else
						{
							 //���ƣ�����͹���
							//��ɫLED��3s
							Red_LED_Blink(3);
							state=LOAD_PARA;
							//state=GET_MODE;
							init_PWMState();
							
							EnterStopMode();
						}   
					}
				}
				else
				{
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]);
					*p_pwm_state=PWM_PERIOD;
				}
			}
		}
		
		if(*p_pwm_state==PWM_PERIOD)
		{
//			if((*p_PWM_period_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+PERIOD]*1000)
//			{
//				++(*p_PWM_numOfCycle);
//				*p_PWM_period_cnt=0;
//				*p_pwm_state=PWM_WAIT_BETWEEN;
//				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
//			}
//			else
//			{
//				++(*p_PWM_period_cnt);
//			}
			
			if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+PERIOD]*1000,num))
			{
				++(*p_PWM_numOfCycle);
				//*p_PWM_period_cnt=0;
				*p_pwm_state=PWM_WAIT_BETWEEN;
				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
			}
//			else
//			{
//				++(*p_PWM_period_cnt);
//			}
		}
		
		if(*p_pwm_state==PWM_WAIT_BETWEEN)
		{
			if(*p_PWM_numOfCycle==buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+NUM_OF_CYCLES])
			{
				*p_pwm_state=PWM_WAIT_AFTER;
				*p_PWM_numOfCycle=0;
			}
			else
			{
//				if((*p_PWM_waitBetween_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_BETWEEN]*1000)
//				{ 
//					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]); 
//					*p_PWM_waitBetween_cnt=0;
//					*p_pwm_state=PWM_PERIOD;
//				}
//				else
//				{
//					++(*p_PWM_waitBetween_cnt);
//				}
				
				if(Is_timing_Xmillisec(buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_BETWEEN]*1000,num))
				{
					Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+DUTY_CYCLE]); 
					//*p_PWM_waitBetween_cnt=0;
					*p_pwm_state=PWM_PERIOD;
				}
			}
		}
		
		if(*p_pwm_state==PWM_WAIT_AFTER)
		{
//			if((*p_PWM_waitAfter_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+WAIT_AFTER]*1000)
//			{
//				*p_PWM_numOfCycle=0;
//				Motor_PWM_Freq_Dudy_Set(num,buffer[1+ELEMENTS_CNT*(*p_PWM_serial_cnt)+FREQ],0);
//				++(*p_PWM_serial_cnt);
//				*p_pwm_state=PWM_START;
//				*p_PWM_waitAfter_cnt=0;
//			}
//			else	
//			{
//				++(*p_PWM_waitAfter_cnt);
//			}
			
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
	for(int i=0;i<54;i++)
	{
		j++;                 //1.������һ��
		if(buffer[2+j++]>1) //2.enable
		{
			ResetParameter(buffer);
			return;
		}
		//MODE1_PWM3(98,152),MODE2_PWM3(248,302),MODE3_PWM3(398,452)
		if((j>=96&&j<=150)||(j>=246&&j<=300)||(j>=396&&j<=450))  //3.threshold
		{
			j++;
		}
		
		if(buffer[2+j++]==0)  //3.freq
		{
			ResetParameter(buffer);
			return;
		}
		if(buffer[2+j]<5||buffer[2+j]>99) //4.duty cycle
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
		if(buffer[2+j]<1||buffer[2+j]>250)            //6.number of cycle
		{
			ResetParameter(buffer);
			return;
		}
		j++;
		
		j++;                                  //7.wait between
		j++;																	//8.wait after
	}
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
		//PWM1��PWM2��ÿ��serial��8��Ԫ�أ���PWM3��9��Ԫ��
		if(PWMX==1||PWMX==2)
		{
			num=8;
		}
		else if(PWMX==3)
		{
			num=9;
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
	static uint8_t switch_mode_cnt=0;
	static uint8_t release_btn_cnt=0;
	static BOOL b_check_bnt_release=FALSE;
	//static BOOL b_check_bnt_pressed=FALSE;
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
	{
		if(switch_mode_cnt==5)
		{
			//b_check_bnt_pressed=TRUE;
			b_check_bnt_release=TRUE;
			switch_mode_cnt=0;
			
//			init_PWMState();
//			state=LOAD_PARA;
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
//			Motor_PWM_Freq_Dudy_Set(4,100,0);
//			Motor_PWM_Freq_Dudy_Set(5,100,0);
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
			if(release_btn_cnt==5)
			{
				release_btn_cnt=0;
				//b_check_bnt_pressed=FALSE;
				b_check_bnt_release=FALSE;
				//�л�����ģʽ
				if(mode==1)
				{
					mode=2;
					//�����и����⣬Ϊʲô���ܵ���API��һ�þ�Ӱ��PWM2?
					//Motor_PWM_Freq_Dudy_Set(2,100,0);����д���Σ�����ͻ�����������
//					GPIO_SetBits(LED_PORT,LED_ID_MODE1);
//					GPIO_ResetBits(LED_PORT,LED_ID_MODE2);		
					set_led(LED_ID_MODE1,FALSE); //�ص�LED1����LED2
					set_led(LED_ID_MODE2,TRUE);
				}
				else if(mode==2)
				{
					mode=3;
//					GPIO_SetBits(LED_PORT,LED_ID_MODE2);
//					GPIO_ResetBits(LED_PORT,LED_ID_MODE3);
					set_led(LED_ID_MODE2,FALSE);   //�ص�LED2����LED3
					set_led(LED_ID_MODE3,TRUE);
				}
				else if(mode==3)
				{
					mode=1;
//					GPIO_SetBits(LED_PORT,LED_ID_MODE3);
//					GPIO_ResetBits(LED_PORT,LED_ID_MODE1);
					set_led(LED_ID_MODE3,FALSE);  //�ص�LED3����LED1
					set_led(LED_ID_MODE1,TRUE);
				}
				else
				{
					//do nothing
				}
				//b_switch_mode_changed=TRUE;
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
	#if 0
//  static uint8_t switch_mode_cnt=0;
//	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15)==0)
//	{
//		if(switch_mode_cnt==5)
//		{
//			switch_mode_cnt=0;
//			//�л�����ģʽ
//			if(mode==1)
//			{
//				mode=2;
////				set_led(LED_ID_MODE1,FALSE); //�ص�LED1����LED2
////				set_led(LED_ID_MODE2,TRUE);
//			}
//			else if(mode==2)
//			{
//				mode=3;
////				set_led(LED_ID_MODE2,FALSE);   //�ص�LED2����LED3
////				set_led(LED_ID_MODE3,TRUE);
//			}
//			else if(mode==3)
//			{
//				mode=1;
////				set_led(LED_ID_MODE3,FALSE);  //�ص�LED3����LED1
////				set_led(LED_ID_MODE1,TRUE);
//			}
//			else
//			{
//				//do nothing
//			}
//			init_PWMState();
//			state=LOAD_PARA;
//			Motor_PWM_Freq_Dudy_Set(1,100,0);
//			Motor_PWM_Freq_Dudy_Set(2,100,0);
//			Motor_PWM_Freq_Dudy_Set(3,100,0);
//			//Motor_PWM_Freq_Dudy_Set(4,100,0);
//		}
//		else
//		{
//			switch_mode_cnt++;
//		}
//	}
//	else
//	{
//		switch_mode_cnt=0;
//	}
#endif
	os_delay_ms(TASK_GET_SWITCH_MODE, 20);
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
	static uint16_t pressure_result; 
	//�������һ��״̬������ȡcycle_cnt;
	static uint8_t cycle_cnt=0;
//	cycle_cnt=3;//buffer[0],cycle time
	
	//if(mcu_state==POWER_ON)
	{
		//1.��flash�м��ز������ڴ�
		if(state==LOAD_PARA)      
		{
			uint8_t len=PARAMETER_BUF_LEN/4;  
			uint32_t tmp[PARAMETER_BUF_LEN/4]={0};   		
			
			//��ȡflash���ݵ�buffer��
			FlashRead(FLASH_WRITE_START_ADDR,tmp,len);
			memcpy(buffer,tmp,PARAMETER_BUF_LEN);
			CheckFlashData(buffer);
			state=GET_MODE;
			cycle_cnt=buffer[0];  //��ȡcycle����ֵ
			--cycle_cnt; // ��Ϊstate=GET_MODE������Ĵ����ȫ��ִ��һ�Σ��൱���Ѿ�ִ�й�һ��cycle�ˣ���������Ҫ��1
		}
		
		if(state!=IDLE)
		{
			if(state==GET_CYCLE_CNT)
			{
				if(cycle_cnt==0)
				{
					//state=IDLE;
					state=CHECK_BAT_VOL;
					cycle_cnt=buffer[0];
				}
				else
				{
					state=GET_MODE;  //�����ٴ�ѭ��
					cycle_cnt--;
				}
				
			}
			
			//2.��ÿ��ض�Ӧ��ģʽ
			if(state==GET_MODE)    //flash���������ڴ�֮�󣬻�ȡ���ض�Ӧ��ģʽ
			{
				//mode=GetModeSelected();  //�õ�ģʽ
				state=CPY_PARA_TO_BUFFER;
			}
			//3.����ѡ���ģʽ�����ݿ�����pwm_buffer
			if(state==CPY_PARA_TO_BUFFER)  //����ѡ���ģʽ����para��䵽pwm_buffer��
			{
				uint8_t pwm_buffer[144+6]; //144=6*8*3
				
				memset(pwm1_buffer,0,49+6); //49=1(��Ч��serial����)+6*8
				memset(pwm2_buffer,0,49+6);
				memset(pwm3_buffer,0,49+6);
				
				switch(mode)
				{
					case 1:
						memcpy(pwm_buffer,buffer+2,144+6);  //MODE1��PWM1,PWM2,PWM3          
						break;
					case 2:
						memcpy(pwm_buffer,buffer+146+6,144+6); //MODE2��PWM1,PWM2,PWM3
						break;
					case 3:
						memcpy(pwm_buffer,buffer+290+12,144+6); //MODE3��PWM1,PWM2,PWM3
						break;
					default:
						break;
				}
				FillUpPWMbuffer(pwm1_buffer,pwm_buffer,1);
				FillUpPWMbuffer(pwm2_buffer,pwm_buffer+48,2);
				FillUpPWMbuffer(pwm3_buffer,pwm_buffer+96,3);
				state=CHECK_PRESSURE;
			}
			
			//4.���ѹ��
			if(state==CHECK_PRESSURE) //���ѹ��
			{
				//pressure_result=ADS115_readByte(0x90);
				//if(pressure_result>=buffer[0]*70)  //ѹ���ﵽthreshold���������PWMģʽ,����75Ϊб�ʣ�5mmgH��Ӧ5*70+700
				//pressure_result=900;
				if(pressure_result<=70*5)  //����Ӧ����<=5mmgH���������У�5mmgH�ǹ̶�ֵ��Ŀ���Ǽ��ballom�е����壬û������������PWM
				{
					state=PREV_OUTPUT_PWM;
				}
				else
				{
					//state=CHECK_PRESSURE_AGAIN;
					
					//�����ŷ���4s��֮��������
					ReleaseGas(4);
					state=CHECK_PRESSURE;
				}
			}
			
			//5.���ѹ��Ok,��Ԥ��������Σ��ȶ�ʱwaitBeforeStart��ô��ʱ��
			if(state==PREV_OUTPUT_PWM)  //��ʼԤ�����PWM����
			{
	//			//�������if(b_Is_PCB_PowerOn==FALSE)�ᵼ�¿������¿���waitbeforestart��ʱ������Ҫ������
	//			if(b_Is_PCB_PowerOn==FALSE)
	//			{
	//				PWM_waitBeforeStart_cnt=0;
	//			}
	//			else
				{
//					if((PWM_waitBeforeStart_cnt)*CHECK_MODE_OUTPUT_PWM==buffer[1]*1000)
//					{
//						PWM_waitBeforeStart_cnt=0;
//						state=OUTPUT_PWM;
//					}
//					else
//					{
//						PWM_waitBeforeStart_cnt++;					
//					}
					
					
					if(Is_timing_Xmillisec(buffer[1]*1000,6))
					{
						state=OUTPUT_PWM;
					}
				}  
			}
			
			//6.��ʼ�������
			if(state==OUTPUT_PWM) //�����趨�Ĳ��������PWM1,PWM2,PWM3
			{			
				if(pwm1_state==PWM_OUTPUT_FINISH&&pwm2_state==PWM_OUTPUT_FINISH&&pwm3_state==PWM_OUTPUT_FINISH)
				{
					PWM1_serial_cnt=0;
					PWM2_serial_cnt=0;
					PWM3_serial_cnt=0;
					//state=CHECK_BAT_VOL;
					state=GET_CYCLE_CNT;
					init_PWMState();
				}		
				else
				{
					PaintPWM(1,pwm1_buffer); 
					PaintPWM(2,pwm2_buffer);
					PaintPWM(3,pwm3_buffer);
				}
			}
			
			//7.���������ϣ�����ص�ѹ
			if(state==CHECK_BAT_VOL) 
			{
				//����û��PCBA,��ص�ѹ���Ĺ���������
				
//					uint16_t result;
//					result=RegularConvData_Tab[0]; //��ѹֵ
//					if(result<2730) //�����ѹС��2.2v,����׼3.3v��
//					{
//						//���ƣ�����POWER_OFF
//						state=LED_RED_BLINK;
//						cycle_cnt=buffer[0]; //���˵͹���֮��������ȥ��
//					}
//					else
				{
					//state=LOAD_PARA;
					//state=GET_CYCLE_CNT;
					//state=GET_MODE;
					state=IDLE;
					pwm1_state=PWM_START;
					pwm2_state=PWM_START;
					pwm3_state=PWM_START;
				}
			}
				
				//CTS����Ҫ���¼��60s
				#if 0
//				//��Ӧ4��ѹ����⣬������ѹ����ok�����ٴμ��ѹ��
//				if(state==CHECK_PRESSURE_AGAIN) //�ٴμ��ѹ��
//				{
//					if(CHECK_MODE_OUTPUT_PWM*checkPressAgain_cnt==60*1000)   //����60s��ⲻ��������POWER_OFF
//					{
//						checkPressAgain_cnt=0;
//						mcu_state=POWER_OFF;
//						//state=LOAD_PARA;
//						state=GET_MODE;
//						set_led(LED_CLOSE);
//						
//						EnterStopMode();
//						init_system_afterWakeUp();
//					}
//					else
//					{
//						pressure_result=ADS115_readByte(0x90);
//						//�ر�ע�⣬���ﲻ����ȫ�ֱ���buffer,��Ӧ����parameter_buf
//						//���ɣ��������60s����ʱ״̬����ʱ��buffer��ֵ��CHECK_PRESSURE_AGAIN״̬�Ѿ��̶���
//						//�����ʱ��λ�������˲�����parameter_buf[0]��ı䣬Ӧ��������仯�˵�ֵ���ж�
//						if(pressure_result<parameter_buf[0]*70) 
//						{
//							checkPressAgain_cnt++;
//						}
//						else	
//						{
//							checkPressAgain_cnt=0;
//							state=LOAD_PARA;
//						}
//					}
//				}
#endif
				
				
				//��Ӧ7���������ص�ѹС��2.2V��������
			if(state==LED_RED_BLINK)
			{
				//��ɫLED��3s
				Red_LED_Blink(3);
				
				state=LOAD_PARA;
				pwm1_state=PWM_START;
				pwm2_state=PWM_START;
				pwm3_state=PWM_START;
				mcu_state=POWER_OFF;
				
				EnterStopMode();
				init_system_afterWakeUp();
			}
		}
		else
		{
			//���cycle_cnt=0,������κ�PWM
			//do nothing
		}
		

	}
//	else
//	{
//		//����͹���ģʽ
////		EnterStopMode();
////		init_system_afterWakeUp();
////		Motor_PWM_Init();
//	}
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
	//ѭ�h
	ReceiveData(&g_CmdReceive);//�������ݵ�������
	ModuleUnPackFrame();//�����
	os_delay_ms(RECEIVE_TASK_ID, 100);
}
