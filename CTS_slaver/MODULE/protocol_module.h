/**
********************************************************************************
* 版權：
* 模块名称：protocol.h
* 模块功能：跟上位機進行通信
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/
#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_
/***********************************
* 头文件
***********************************/
#include "stm32f0xx.h"

/**********************************
*宏定义
***********************************/
#define SEND_DATA_BUF_LENGTH        300  // 波特率 115200
#define CMD_BUFFER_LENGTH         350  //宏定义命令数据包的长度

#define		PACK_HEAD_BYTE				 0xFF //头文件标志
#define   MODULE_CMD_TYPE					0

//模块命令识别号
#define SEND_VOILD_EXP_FLAG_ID     0x01  //有效呼吸次
#define SEND_EXP_TRAIN_DATA_ID     0x02  //存储数据
#define SEND_EXP_TRAIN_DATA_CHECK_SUM_ID     0x03  //存储数据校验和
#define SEND_BAT_PER_ID     0x04  //电池电量

//主机命令识别号
#define GET_EXP_TRAIN_DATA_ID     0x01  //有效呼吸次
//#define GET_BAT_PER_ID     				0x02  //电池电量
//#define PWM_VALUE_SET_ID   				0x03  //设置PWM参数
#define COMM_PARAMETER_ID					0x04  //上位机下发的公共信息
#define GET_FLASH_DATA_1_ID				0x05  //上位机下发的获得flash中参数
#define GET_FLASH_DATA_2_ID				0x06  //上位机下发的获得flash中参数
#define GET_FLASH_DATA_3_ID				0x0C

#define IS_RCV_PARA_FINISHED			0x07  //上位机下发的询问数据是否接收完成
#define SEND_PARA_RCV_RESULT			0x08  //下位机发送接收参数的处理结果
#define SEND_FLASH_DATA_1_ID      0x09  //由于原始数据太长(434Bytes),分成两帧发
#define SEND_FLASH_DATA_2_ID      0x0A  
#define SEND_FLASH_DATA_3_ID      0x0B
#define MODE1_PWM1_ID							0x11  //上位机下发的MODE1-PWM1
#define MODE1_PWM2_ID							0x12  //上位机下发的MODE1-PWM1
#define MODE1_PWM3_ID							0x13  //上位机下发的MODE1-PWM1
#define MODE2_PWM1_ID							0x21  //上位机下发的MODE1-PWM1
#define MODE2_PWM2_ID							0x22  //上位机下发的MODE1-PWM1
#define MODE2_PWM3_ID							0x23  //上位机下发的MODE1-PWM1
#define MODE3_PWM1_ID							0x31  //上位机下发的MODE1-PWM1
#define MODE3_PWM2_ID							0x32  //上位机下发的MODE1-PWM1
#define MODE3_PWM3_ID							0x33  //上位机下发的MODE1-PWM1

#define CAL_SENSOR_MMGH_1         0x50  //上位机下发的mmgh的值1
#define CAL_SENSOR_MMGH_2         0x51	//上位机下发的mmgh的值2
#define CAL_SENSOR_MMGH_3         0x52	//上位机下发的mmgh的值3
#define CAL_SENSOR_SEND_TO_PC     0x60  //下位机回传值给上位机

//定义上位机写入flash的起始地址
#define FLASH_PAGE_SIZE      			((uint16_t)0x400)  //flash一页的大小为1K
#define FLASH_START_ADDR   				((uint32_t)0x08000000) //flash开始地址
#define FLASH_END_ADDR						((uint32_t)0x08004000) //flash结束地址
#define FLASH_WRITE_START_ADDR		((uint32_t)0x08000000+1024*30) //开始写入的开始地址
//注意070cb的page size是2K
#define FLASH_ADDR_RECORD_CYCLES	((uint32_t)0x08000000+1024*32) //记录测试的圈数，这个仅仅是为了测试电池电量能支撑多少圈
#define FLASH_WRITE_END_ADDR      ((uint32_t)0x08004000)  //flash写入的结束地址

#define FLASH_PRESSURE_RATE_ADDR  ((uint32_t)0x08000000+1024*26) //开始写入的开始地址

void FlashRead(uint32_t addr, uint32_t *p_data, uint16_t len);
uint32_t FlashReadWord(uint32_t addr);
//uint32_t FlashReadByte(char* addr);
uint8_t FlashReadByte(uint32_t addr);
//uint16_t FlashWrite(uint32_t addr, uint32_t *p_data, uint16_t len);
uint16_t FlashWrite(uint32_t addr, uint8_t *p_data, uint16_t len);
//uint16_t GetModeSelected(void);
/***********************************
* 变量類型聲明
***********************************/


/***********************************
* 函数
***********************************/
void protocol_module_process(uint8_t* pdata);

void protocol_module_send_exp_flag(uint8_t flag);
void protocol_module_send_train_data_one_page(uint8_t* buf, uint8_t len);
void protocol_module_send_train_data_check_sum(uint32_t check_sum);
void protocol_module_send_bat_per(uint8_t bat_per);
void get_comm_para_to_buf(uint8_t* pdata);
void get_parameter_to_buf_by_frameId(uint8_t* pdata,char frameId);
void send_prameter_fram1_to_PC(void);
void send_prameter_fram2_to_PC(void);
#endif
