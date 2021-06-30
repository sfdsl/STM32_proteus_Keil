#include "bsp-lcd1602.h"
#include "delay.h"

void LCD1602_WaitReady(void) //���æ״̬
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

void LCD1602_WriteCmd(uint8_t cmd) //дָ��
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

void LCD1602_WriteDat(uint8_t dat) //д����
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
    
    if (y == 0)  //���������Ļ���������ʾRAM�ĵ�ַ
        addr = 0x00 + x;  //��һ���ַ���ַ��0x00��ʼ
    else
        addr = 0x40 + x;  //�ڶ����ַ���ַ��0x40��ʼ
    LCD1602_WriteCmd(addr|0x80);  //����RAM��ַ
}

void LCD1602_ShowStr(uint8_t x, uint8_t y, uint8_t *str, uint8_t len)
{
    LCD1602_SetCursor(x, y);	//������ʼ��ַ
    while (len--)         //����д��len���ַ�����
    {
        LCD1602_WriteDat(*str++);
    }
}

void LCD1602_ShowChar(uint8_t x, uint8_t y, uint8_t str)
{
	LCD1602_SetCursor(x, y);	//������ʼ��ַ
	LCD1602_WriteDat(str);
}

//��ʾ���������ַ�       
//-----------------------------*/          
void LCD1602_ShowNum(uint8_t x, uint8_t y,uint8_t num)
{     

	    LCD1602_SetCursor(x, y);	//������ʼ��ַ
      LCD1602_ShowChar(x,y,num+'0');
} 

void LCD1602_Init(void)
{
	  //LCD1602_GPIO_Config();   //����GPIO��
    LCD1602_WriteCmd(0X38);  //16*2��ʾ��5*7����8λ���ݽӿ�
    LCD1602_WriteCmd(0x0C);  //��ʾ���������ر�
    LCD1602_WriteCmd(0x06);  //���ֲ�������ַ�Զ�+1
    LCD1602_WriteCmd(0x01);  //����
}
	
	



