#include "bsp-lcd1602.h"
#include "delay.h"

void LCD1602_WaitReady(void) //检测忙状态
{
	uint8_t sta;

	GPIOB->ODR |=0x00FF;
	RSO(0);
	RWO(1);
	EO(1);
	//delay_us(100);
	//
	do{
	  EO(1);
		sta=HAL_GPIO_ReadPin(DB7_GPIO_Port,DB7_Pin);
		HAL_Delay(1);
		EO(0);
	}while(sta);
}

void LCD1602_WriteCmd(uint8_t cmd) //写指令
{
	LCD1602_WaitReady();
	RSO(0);
	RWO(0);
	EO(0);
	delay_us(1);
	EO(1);
	DB0_GPIO_Port->ODR &= (cmd|0xFF00);
	EO(0);
	delay_us(400);
}

void LCD1602_WriteDat(uint8_t dat) //写数据
{
	LCD1602_WaitReady();
	RSO(1);
	RWO(0);
	delay_us(30);
	EO(1);
	DB0_GPIO_Port->ODR &=(dat|0xFF00);
	EO(0);
	delay_us(400);
}

void LCD1602_SetCursor(uint8_t x, uint8_t y)
{
    uint8_t addr;
    
    if (y == 0)  //由输入的屏幕坐标计算显示RAM的地址
        addr = 0x00 + x;  //第一行字符地址从0x00起始
    else
        addr = 0x40 + x;  //第二行字符地址从0x40起始
    LCD1602_WriteCmd(addr|0x80);  //设置RAM地址
}

void LCD1602_ShowStr(uint8_t x, uint8_t y, uint8_t *str, uint8_t len)
{
    LCD1602_SetCursor(x, y);	//设置起始地址
    while (len--)         //连续写入len个字符数据
    {
        LCD1602_WriteDat(*str++);
    }
}

void LCD1602_ShowChar(uint8_t x, uint8_t y, uint8_t str)
{
	LCD1602_SetCursor(x, y);	//设置起始地址
	LCD1602_WriteDat(str);
}

//显示单个数字字符       
//-----------------------------*/          
void LCD1602_ShowNum(uint8_t x, uint8_t y,uint8_t num)
{     

	    LCD1602_SetCursor(x, y);	//设置起始地址
      LCD1602_ShowChar(x,y,num+'0');
} 

void LCD1602_Init(void)
{
	  //LCD1602_GPIO_Config();   //开启GPIO口
    LCD1602_WriteCmd(0X38);  //16*2显示，5*7点阵，8位数据接口
    LCD1602_WriteCmd(0x0C);  //显示器开，光标关闭
    LCD1602_WriteCmd(0x06);  //文字不动，地址自动+1
    LCD1602_WriteCmd(0x01);  //清屏
}
	
	



