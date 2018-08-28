#include "stm32f0xx.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_rcc.h"

#include "delay.h"
#include "os_cfg.h"
#include "hardware.h"
#include "app.h"

#include "comm_task.h"
#include "key_power_on_task.h"
#include "hardware.h"
#include "i2c.h"
#include "Motor_pwm.h"
//#include "device_type.h"
#include "stm32f0xx_dma.h"
#include "iwtdg.h"

//#ifdef _DEBUG_TEST_CYCLES
//#include "protocol_module.h"
//extern CHCKMODE_OUTPUT_PWM state;
//extern uint16_t FlashWrite(uint32_t addr, uint8_t *p_data, uint16_t len);
//extern uint32_t debug_cycles_record[3];
//#else
//#endif

 const uint8_t default_parameter_buf[PARAMETER_BUF_LEN] = {
#if 1
2,2,

//MODE1
0x11,1,100,50,40,1,1,16,
0x12,1,100,30,40,1,1,4,
0x13,1,100,50,40,1,1,12,
0x14,1,100,30,40,1,1,0,
0x15,1,100,50,40,1,1,4,
0x16,1,100,30,40,1,1,0,

0x21,1,100,50,40,1,1,16,
0x22,1,100,30,40,1,1,4,
0x23,1,100,50,40,1,1,12,
0x24,1,100,30,40,1,1,0,
0x25,1,100,50,40,1,1,4,
0x26,1,100,30,40,1,1,0,

0x31,1,80,100,99,30,4,3,1,1,
0x32,1,100,100,99,30,4,3,1,1,
0x33,1,110,100,99,30,4,3,1,1,
0x34,1,140,100,99,30,4,2,1,1,
0x35,1,170,100,99,30,4,1,1,1,
0x36,1,177,100,99,30,4,1,1,1,


//MODE2
0x11,1,100,50,40,1,1,16,
0x12,1,100,30,40,1,1,4,
0x13,1,100,50,40,1,1,12,
0x14,1,100,30,40,1,1,0,
0x15,1,100,50,40,1,1,4,
0x16,1,100,30,40,1,1,0,

0x21,1,100,50,40,1,1,0,
0x22,1,100,30,40,1,1,0,
0x23,1,100,50,40,1,1,0,
0x24,1,100,30,40,1,1,0,
0x25,1,100,50,40,1,1,4,
0x26,1,100,30,40,1,1,0,

0x31,1,70,100,99,30,4,3,1,1,
0x32,1,90,100,99,30,4,3,1,1,
0x33,1,110,100,99,30,4,3,1,1,
0x34,1,130,100,99,30,4,2,1,1,
0x35,1,140,100,99,30,4,1,1,1,
0x36,1,160,100,99,30,4,1,1,1,


//MODE3
0x11,1,100,50,40,1,1,16,
0x12,1,100,30,40,1,1,4,
0x13,1,100,50,40,1,1,12,
0x14,1,100,30,40,1,1,0,
0x15,1,100,50,40,1,1,4,
0x16,1,100,30,40,1,1,0,

0x21,1,100,50,40,1,1,16,
0x22,1,100,30,40,1,1,4,
0x23,1,100,50,40,1,1,12,
0x24,1,100,30,40,1,1,0,
0x25,1,100,50,40,1,1,4,
0x26,1,100,30,40,1,1,0,

0x31,1,80,100,99,30,5,3,1,1,
0x32,1,100,100,99,30,5,3,1,1,
0x33,1,110,100,99,30,5,3,1,1,
0x34,1,140,100,99,30,5,2,1,1,
0x35,1,170,100,99,30,5,1,1,1,
0x36,1,177,100,99,30,5,1,1,1,


//Checksum
0x3B,0x88
		#endif
	};

//extern SELF_TEST_STATE	self_tet_state;
	
int main(void)
{
	//self_tet_state=SELF_TEST_INFLATE;
	//Init_iWtdg(4,1250);  //4*2^4=64分频，1250(大概是1250*1.6ms=2s)
  delay_init();
	
	os_init();
	//Motor_PWM_Init();
	
//#ifdef _DEBUG_TEST_CYCLES
//	debug_cycles_record[0]=0;
//	debug_cycles_record[1]=0;
//	debug_cycles_record[2]=0;
//	FlashWrite(FLASH_ADDR_RECORD_CYCLES,(uint8_t*)debug_cycles_record,3);
//	state=LOAD_PARA;
//#else
//#endif
	
	
#ifdef _DEBUG
#else
	//进入stop模式
	EnterStopMode();
	//唤醒之后先初始化系统
	init_system_afterWakeUp();
#endif	
		
	os_create_task(init_task, OS_TRUE, INIT_TASK_ID);
	os_start();

	return 0;
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t* file, uint32_t line)
{
  while (1)
  {
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
