#ifndef _BSP_LCD1602_H
#define _BSP_LCD1602_H

#include "stm32f1xx_hal.h"
#include "main.h"





#define EO(X)         X? (HAL_GPIO_WritePin(LCD1602_E_GPIO_Port,LCD1602_E_Pin,GPIO_PIN_SET)):(HAL_GPIO_WritePin(LCD1602_E_GPIO_Port,LCD1602_E_Pin,GPIO_PIN_RESET))
#define RWO(X)        X? (HAL_GPIO_WritePin(LCD1602_RW_GPIO_Port,LCD1602_RW_Pin,GPIO_PIN_SET)):(HAL_GPIO_WritePin(LCD1602_RW_GPIO_Port,LCD1602_RW_Pin,GPIO_PIN_RESET))
#define RSO(X)        X? (HAL_GPIO_WritePin(LCD1602_RS_GPIO_Port,LCD1602_RS_Pin,GPIO_PIN_SET)):(HAL_GPIO_WritePin(LCD1602_RS_GPIO_Port,LCD1602_RS_Pin,GPIO_PIN_RESET))


//只能是某个GPIO口的低八位
#define DB0					GPIO_Pin_0
#define DB1					GPIO_Pin_1
#define DB2					GPIO_Pin_2
#define DB3					GPIO_Pin_3
#define DB4					GPIO_Pin_4
#define DB5					GPIO_Pin_5
#define DB6					GPIO_Pin_6
#define DB7					GPIO_Pin_7

void LCD1602_Init(void);  //初始化LCD602；
void LCD1602_ShowStr(uint8_t x, uint8_t y, uint8_t *str,uint8_t len);
void LCD1602_ShowNum(uint8_t x, uint8_t y,uint8_t num);
void LCD1602_WriteCmd(uint8_t cmd); //写指令
void LCD1602_ShowChar(uint8_t x, uint8_t y, uint8_t str);




#endif //_BSP_LCD1602_H
