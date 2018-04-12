/**
********************************************************************************
* 版權：深圳永勝宏基
* 模块名称：store_fifo.c
* 模块功能：管理數據存儲fifo，將數據按頁存儲與flash中，客戶端使用如下
1. 首先，定義一下參數：
	UINT32 head_offset;//頭存儲區域EEPROM——起始地址
	UINT8 data_page_num;//數據存儲區域FLASH——存储页数
	UINT16 data_page_size;//數據存儲區域域FLASH——存储的页的大小
	UINT32 data_offset;//數據存儲區域域FLASH——起始地址
2. 定義頭信息實體結構STORE_HEAD
3. 定義數據信息結構實體STORE_DATA_PAGE，次需重新定義結構
4. 初始化頭信息：init_store_head
5. 讀寫操作
注意：
1)data_page_num：0~255
2)data_page_size:為FLASH擦出頁大小，一般為128B,1KB,2KB
3)head_offset：EEPROM中
4)data_offset:FLASH中
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

#ifndef __STORE_FIFO_H
#define __STORE_FIFO_H
 
#include "datatype.h" 
#include "stm32f0xx.h"

//頭存儲區域——用判斷頭是否初始化的標志值
#define STORE_HEAD_HAVE_INIT			0x56

//FLASH存储信息头数据结构体 
typedef struct
{
	UINT8 init_flag;//頭初始化標志
	UINT8 data_page_num;//數據存儲區域——存储页数
	UINT16 data_page_size;//數據存儲區域——存储的页的大小
	UINT32 head_offset;//頭存儲區域——起始地址
	UINT32 data_offset;//數據存儲區域——起始地址
	UINT32 front;	//队头地址,0~STORE_DATA_PAGE_NUM
	UINT32 rear; 	//队尾地址,0~STORE_DATA_PAGE_NUM
}STORE_HEAD;

//每页数据结构
typedef struct
{
	UINT8* data_page_buf;
}STORE_DATA_PAGE;

//供外部调用的函数
BOOL init_store_head(STORE_HEAD* p_store_head, \
	UINT32 head_offset, UINT32 data_offset,UINT8 data_page_num,UINT16 data_page_size);
void read_store_head(STORE_HEAD* p_store_head);
BOOL save_store_head(STORE_HEAD* p_store_head);
UINT16 get_store_data_page_count(STORE_HEAD* p_store_head);
BOOL save_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page);
BOOL save_store_data_multi_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE p_store_data_page[], UINT16 page_num);
BOOL read_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page, UINT32 logic_addr);
#endif
