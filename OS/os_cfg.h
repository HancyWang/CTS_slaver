#ifndef __OS_CFG_H
#define __OS_CFG_H

#include "os_core.h"

#define OS_TRUE 1
#define OS_FALSE 0

#define OS_MAX_TASK  10

#define OS_TICKS_PER_SEC    1000u

#define OS_INIT    init_tim //初始化函數指針——初始化滴答時鐘
#define OS_INIT_HEAD_FILE  "hardware.h" 

#endif
