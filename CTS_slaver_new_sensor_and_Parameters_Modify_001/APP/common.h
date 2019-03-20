#ifndef _COMMON_H
#define _COMMON_H

typedef    unsigned char          BOOLEAN;
typedef    unsigned char          INT8U;
typedef    signed char            INT8S;
typedef    unsigned short int     INT16U;
typedef    signed short int       INT16S;
typedef    unsigned long          INT32U;
typedef    signed long            INT32S;
typedef    float                  FP32;
typedef    long double            FP64;

//typedef    enum {FALSE = 0, TRUE = !FALSE} bool;


extern INT16U g_uwDelayTime_ms;
extern void Delay(INT16U mTime);
extern void Delay_us(INT16U time);
extern void Delay_ms(INT16U time);
//extern void Delay_Init(INT8U sysclk);


#endif
