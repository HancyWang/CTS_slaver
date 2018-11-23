#include "iwtdg.h"
#include "stm32f0xx_iwdg.h"

void Init_iWtdg(unsigned char prer,unsigned short int rlr)
{
//	IWDG->KR=0X5555;       //解除寄存器保护        
//	IWDG->PR=prer;         // 写入初始化分频值
//	IWDG->RLR=rlr;            // 写入自动装载值
//	IWDG->KR=0XAAAA;     //开启寄存器保护
//	IWDG->KR=0XCCCC;       //启动看门狗
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);  //??????? I ???  
	IWDG_SetPrescaler(prer);  //??? IWDG ????:?? IWDG ????  
	IWDG_SetReload(rlr);  //??? IWDG ????  
	IWDG_ReloadCounter(); //??? IWDG ??????????? IWDG ???  
	IWDG_Enable();  //??? IWDG 
}

void IWDG_Feed(void)
{
   IWDG_ReloadCounter();//reload                                 
}

