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
200,2,

//MODE1
0x11,1,100,50,6,1,1,4,
0x12,1,100,50,6,1,1,4,
0x13,1,100,50,6,1,1,4,
0x14,1,100,60,6,1,1,4,
0x15,1,100,60,6,1,1,4,
0x16,1,100,60,6,1,1,4,

0x21,1,100,50,6,1,1,4,
0x22,1,100,50,6,1,1,4,
0x23,1,100,50,6,1,1,4,
0x24,1,100,60,6,1,1,4,
0x25,1,100,60,6,1,1,4,
0x26,1,100,60,6,1,1,4,

0x31,40,1,100,40,10,10,1,0,2,
0x32,60,1,100,45,10,10,1,0,2,
0x33,80,1,100,50,10,10,1,0,2,
0x34,100,1,100,55,10,10,1,0,2,
0x35,140,1,100,60,10,10,1,0,2,
0x36,160,1,100,65,10,10,1,0,2,


//MODE2
0x11,1,100,70,10,1,1,2,
0x12,1,100,70,10,1,1,2,
0x13,1,100,70,10,1,1,2,
0x14,1,100,80,10,1,1,2,
0x15,1,100,80,10,1,1,2,
0x16,1,100,80,10,1,1,2,

0x21,1,100,70,10,1,1,2,
0x22,1,100,70,10,1,1,2,
0x23,1,100,70,10,1,1,2,
0x24,1,100,80,10,1,1,2,
0x25,1,100,80,10,1,1,2,
0x26,1,100,80,10,1,1,2,

0x31,50,1,100,45,10,5,1,1,2,
0x32,65,1,100,50,10,5,1,1,2,
0x33,85,1,100,55,10,5,1,1,2,
0x34,105,1,100,60,10,5,1,1,2,
0x35,145,1,100,65,10,5,1,1,2,
0x36,165,1,100,65,10,5,1,1,2,


//MODE3
0x11,1,100,80,10,1,1,3,
0x12,1,100,80,10,1,1,3,
0x13,1,100,80,10,1,1,3,
0x14,1,100,90,10,1,1,3,
0x15,1,100,90,10,1,1,3,
0x16,1,100,90,10,1,1,3,

0x21,1,100,80,10,1,1,3,
0x22,1,100,80,10,1,1,3,
0x23,1,100,80,10,1,1,3,
0x24,1,100,90,10,1,1,3,
0x25,1,100,90,10,1,1,3,
0x26,1,100,90,10,1,1,3,

0x31,50,1,100,80,10,4,1,1,3,
0x32,68,1,100,80,10,4,1,1,3,
0x33,88,1,100,80,10,4,1,1,3,
0x34,110,1,100,90,10,4,1,1,3,
0x35,150,1,100,90,10,4,1,1,3,
0x36,170,1,100,90,10,4,1,1,3,

//Checksum
0x36,0xB7
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
