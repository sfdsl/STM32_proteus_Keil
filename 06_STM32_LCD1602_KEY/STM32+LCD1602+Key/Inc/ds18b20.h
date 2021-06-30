#ifndef __DS18B20_H
#define __DS18B20_H 
#include "sys.h"   
#include "main.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//DS18B20驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/7
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved										  
//////////////////////////////////////////////////////////////////////////////////

//IO方向设置
#define DS18B20_IO_IN()  {IO_TEMP_GPIO_Port->CRL &= 0xFFF0FFFF;	IO_TEMP_GPIO_Port->CRL |= 8 << 16;}	// PA4 IN  MODE
#define DS18B20_IO_OUT() {IO_TEMP_GPIO_Port->CRL &= 0xFFF0FFFF;	IO_TEMP_GPIO_Port->CRL |= 3 << 16;}	// PA4 OUT MODE
////IO操作函数											   
#define	DS18B20_DQ_OUT PAout(4) //数据端口	PA4
#define	DS18B20_DQ_IN  PAin(4)  //数据端口	PA4 
   	
u8 DS18B20_Init(void);			//初始化DS18B20
short DS18B20_Get_Temp(void);	//获取温度
void DS18B20_Start(void);		//开始温度转换
void DS18B20_Write_Byte(u8 dat);//写入一个字节
u8 DS18B20_Read_Byte(void);		//读出一个字节
u8 DS18B20_Read_Bit(void);		//读出一个位
u8 DS18B20_Check(void);			//检测是否存在DS18B20
void DS18B20_Rst(void);			//复位DS18B20    
#endif















