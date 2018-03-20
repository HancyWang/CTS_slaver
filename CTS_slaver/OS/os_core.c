#include "os_core.h"
#include "os_cfg.h"
#include "stm32f0xx.h"
#include OS_INIT_HEAD_FILE


/******************************
* OS的全局变量定义
*******************************/
/*任務控製塊*/
TCB_DATA task_array[OS_MAX_TASK];
/*定义时钟计数*/
uint32_t os_ticks = 0;

 uint16_t prev_pwm_tick_cnt;
 uint16_t pwm_tick_cnt;

/*
* 任务调度函数
*/
void os_start(void)
{
  while(1)
  {
    static uint32_t index;
    for(index = 0; index < OS_MAX_TASK; index++)
    {
      if(task_array[index].run_en == OS_TRUE)//运行标志是否有效
      {
          if(task_array[index].delay_start + task_array[index].delay_period \
                  < os_ticks)//延時時間到
          {
              task_array[index].delay_start = 0;

              task_array[index].delay_period = 0;

              (*(task_array[index].ptask))();//运行任务
          }
      }
    }
  }
}

/*
* 创建任务
*参数如下
* void (*ptask)(void);
  uint8_t id;
  uint8_t run_en;
  uint32_t run_cnt;
*/
void os_create_task(void(*c_ptask)(void), uint8_t c_run_en, uint8_t c_id)
{
  uint8_t index = 0;
  index = c_id;
  task_array[index].run_en  = c_run_en;
  task_array[index].id      = c_id;
  task_array[index].ptask   = c_ptask;
  task_array[index].delay_period = 0;
  task_array[index].delay_start = 0;
}

/*
* 使能任务
*/
void os_resume_task(uint8_t r_id)
{
  if(r_id > OS_MAX_TASK-1)
    return;
  else
  {
    uint8_t index = 0;
    index = r_id;
    task_array[index].run_en = OS_TRUE;
  }
}

/*
* 挂起任务
*/
void os_pend_task(uint8_t r_id)
{
  if(r_id > OS_MAX_TASK-1)
    return;
  else
  {
    uint8_t index = 0;
    index = r_id;
    task_array[index].run_en = OS_FALSE;
  }
}

/*
* 延時任務，任務切換
*/
void os_delay_ms(uint8_t r_id, uint32_t ms)
{
	if(r_id > OS_MAX_TASK-1)
    return;
	else
	{
		uint8_t index = 0;
    index = r_id;
		
		task_array[index].delay_start = os_ticks;
    task_array[index].delay_period = ms*OS_TICKS_PER_SEC/1000;
	}   
}

/*
* 系统滴答时钟中断
*/
void os_isr_ticks(void)
{
  os_ticks ++;//时钟节拍计数加一
}

/*
* 系统初始化
*/
void os_init(void)
{
	OS_INIT();
}
