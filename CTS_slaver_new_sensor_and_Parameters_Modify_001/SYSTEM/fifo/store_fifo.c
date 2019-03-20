/**
********************************************************************************
* 版權：深圳永勝宏基
* 模块名称：store_fifo.c
* 模块功能：管理數據存儲fifo
* 创建日期：
* 创 建 者：
* 声    明：
********************************************************************************
**/

/***********************************
* 头文件
***********************************/
#include "store_fifo.h"
#include "datatype.h"
#include "hardware.h"

/***********************************
* 局部变量
***********************************/

/***********************************
* 函数定義
***********************************/

//获取存储头信息-EEPROM
//head_len: sizeof(store_head)
void read_store_head(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return;
	
	read_half_word_buf_from_eeprom(p_store_head->head_offset,(UINT16*)p_store_head, sizeof(STORE_HEAD)>>1);
}

//保存存储头信息-EEPROM
//head_len: sizeof(store_head)
BOOL save_store_head(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return FALSE;
	
	return save_half_word_buf_to_eeprom(p_store_head->head_offset,(UINT16*)p_store_head, sizeof(STORE_HEAD)>>1);
}

//初始化存储头
BOOL init_store_head(STORE_HEAD* p_store_head, \
	UINT32 head_offset, UINT32 data_offset,UINT8 data_page_num,UINT16 data_page_size)
{	
	if(NULL == p_store_head)
		return FALSE;
	
	p_store_head->head_offset = head_offset;
	p_store_head->data_offset = data_offset;
	p_store_head->data_page_num = data_page_num;
	p_store_head->data_page_size = data_page_size;
	
	//读取初始化标志 
	read_store_head(p_store_head);
	if(STORE_HEAD_HAVE_INIT == p_store_head->init_flag)
	{
		//已经初始化,則返回
		return TRUE;
	}
	else
	{
		//未初始化，则初始化
		p_store_head->init_flag = STORE_HEAD_HAVE_INIT;
		p_store_head->front = p_store_head->data_offset;
		p_store_head->rear = p_store_head->data_offset;
		
		//保存到flash
		return save_store_head(p_store_head);
	}
}

//获得flash存储數據的頁數大小
//返回值：len 表示存储的页数
UINT16 get_store_data_page_count(STORE_HEAD* p_store_head)
{
	if(NULL == p_store_head)
		return 0;
	
	if(p_store_head->front <= p_store_head->rear)
	{
		return (p_store_head->rear - p_store_head->front) / p_store_head->data_page_size;
	}
	else 
	{
		//flash存储满
		return p_store_head->data_page_num;
	}
}

//保存一页数据
BOOL save_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page)
{	
	BOOL err;
	if(NULL == p_store_data_page || NULL == p_store_head)
		return FALSE;
	
	//保存一頁写页
	err |= save_one_page_to_flash(p_store_head->rear, (UINT8*)p_store_data_page, p_store_head->data_page_size);
	
	//处理队尾地址
	p_store_head->rear += 1 * p_store_head->data_page_size;
	if(p_store_head->rear == p_store_head->data_page_num * p_store_head->data_page_size)//
	{
		p_store_head->rear = 0 + p_store_head->data_offset;//归零
	}
	
	//处理队首地址
	if(p_store_head->front == p_store_head->rear)
	{
		p_store_head->front += 1 * p_store_head->data_page_size;
	}	
	
	err |= save_store_head(p_store_head);
	
	return err;
}
//保存多页数据
BOOL save_store_data_multi_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE p_store_data_page[], UINT16 page_num)
{
	UINT16 cnt;
	BOOL err;
	
	if(NULL == p_store_data_page)
		return FALSE;
	
	//保存每一页
	for(cnt = 0; cnt < page_num; cnt ++)
	{
		err |= save_store_data_one_page(p_store_head, &(p_store_data_page[cnt]));
	}
		
	//保存头信息
	err |= save_store_head(p_store_head);
	
	return err;
}

//讀一頁數據
//無數據，則返回 FALSE
//p_store_data_page：保存讀到的數據
BOOL read_store_data_one_page(STORE_HEAD* p_store_head, STORE_DATA_PAGE* p_store_data_page, UINT32 logic_addr)
{
	UINT16 page_num;
	UINT32 read_addr;
	
	if(NULL == p_store_data_page || NULL == p_store_head)
		return FALSE;
	
	//判斷是否有數據
	page_num = get_store_data_page_count(p_store_head);
	if(page_num == 0 || page_num > p_store_head->data_page_num)
		return FALSE;
	
	//判斷有效地址
	if(logic_addr > p_store_head->data_page_num)
		return FALSE;
	
	//獲得讀地址
	read_addr = p_store_head->front + logic_addr * p_store_head->data_page_size;
	if(read_addr >= p_store_head->data_page_num)
	{
		read_addr -= p_store_head->data_page_num * p_store_head->data_page_size;
	}
	
	//讀一頁數據
	read_one_page_from_flash(read_addr, (uint8_t*)p_store_data_page, sizeof(STORE_DATA_PAGE));
	
	return TRUE;
}



























