/*******************************************************************************
* 版权所有 :  深圳市艾顿科技有限公司
* 项目名称 :  监护产品线参数产品室
* 项目编号 :  TBD
* 版本号   :  1.0
* 文件名   :  Datatype.h
* 生成日期 :  2008.7.26
* 作者     :  张德伟
* 功能说明 :  统一各种编译环境下的数据类型数据(主要参照WINCE下的数据类型定义)
* 其它说明 :  无
* 修改记录 :  1.0   创建文件
*******************************************************************************/

#ifndef __DATATYPE_H_
#define __DATATYPE_H_


#ifndef TRUE
#define TRUE 1
#endif

#ifndef  NULL
#define  NULL 0u
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef BOOL
typedef		unsigned char	  BOOL;
#endif

#ifndef INT8
typedef		signed char		  INT8;
#endif

// CHAR字符型
#ifndef CHAR
typedef		signed char		  CHAR;
#endif

#ifndef UCHAR
typedef		unsigned char			  UCHAR;
#endif

#ifndef UINT8
typedef		unsigned char	  UINT8; 
#endif

#ifndef INT16
typedef		signed short	  INT16;
#endif

#ifndef UINT16
typedef		unsigned short	  UINT16;
#endif

#ifndef INT32
typedef		signed int		  INT32;
#endif

#ifndef UINT32
typedef		unsigned int	  UINT32;
#endif

#ifndef INT64
typedef  long long            INT64;
#endif

#ifndef UINT64
typedef  unsigned long long   UINT64;
#endif

#ifndef FLOAT
typedef float FLOAT;    //浮点数
#endif

#ifndef DOUBLE
typedef double DOUBLE;    //双精度浮点数
#endif

#ifndef FLOAT32
typedef		float			  FLOAT32;
#endif

#ifndef FLOAT64
typedef		double 			  FLOAT64;
#endif

#ifndef FLOAT128
typedef		long double		  FLOAT128;
#endif

#endif 
/*************************************结束*************************************/


