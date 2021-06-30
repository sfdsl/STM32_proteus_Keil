/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f1xx_hal.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define Temp_High_Pin GPIO_PIN_0
#define Temp_High_GPIO_Port GPIOC
#define close_sw_Pin GPIO_PIN_1
#define close_sw_GPIO_Port GPIOC
#define Trip_Pin GPIO_PIN_2
#define Trip_GPIO_Port GPIOC
#define Open_sw_Pin GPIO_PIN_3
#define Open_sw_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOA
#define IO_TEMP_Pin GPIO_PIN_4
#define IO_TEMP_GPIO_Port GPIOA
#define DB0_Pin GPIO_PIN_0
#define DB0_GPIO_Port GPIOB
#define DB1_Pin GPIO_PIN_1
#define DB1_GPIO_Port GPIOB
#define DB2_Pin GPIO_PIN_2
#define DB2_GPIO_Port GPIOB
#define LCD1602_E_Pin GPIO_PIN_10
#define LCD1602_E_GPIO_Port GPIOB
#define LCD1602_RW_Pin GPIO_PIN_11
#define LCD1602_RW_GPIO_Port GPIOB
#define LCD1602_RS_Pin GPIO_PIN_12
#define LCD1602_RS_GPIO_Port GPIOB
#define Cur_High_Pin GPIO_PIN_13
#define Cur_High_GPIO_Port GPIOB
#define Vol_Low_Pin GPIO_PIN_14
#define Vol_Low_GPIO_Port GPIOB
#define Vol_High_Pin GPIO_PIN_15
#define Vol_High_GPIO_Port GPIOB
#define DB3_Pin GPIO_PIN_3
#define DB3_GPIO_Port GPIOB
#define DB4_Pin GPIO_PIN_4
#define DB4_GPIO_Port GPIOB
#define DB5_Pin GPIO_PIN_5
#define DB5_GPIO_Port GPIOB
#define DB6_Pin GPIO_PIN_6
#define DB6_GPIO_Port GPIOB
#define DB7_Pin GPIO_PIN_7
#define DB7_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */



/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
