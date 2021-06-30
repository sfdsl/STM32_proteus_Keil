#ifndef __DELAY_H
#define __DELAY_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

#include "stm32f1xx_hal.h"
#include "sys.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

void delay_init(u32 SYSCLK);
void delay_us(u32 nus);
void delay_ms(u16 nms);
/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __DELAY_H */

