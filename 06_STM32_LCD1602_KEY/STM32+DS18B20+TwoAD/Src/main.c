/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "bsp-lcd1602.h"
#include "ds18b20.h"
#include "delay.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//上下限值修改
#define Temp_High 700  //温度上限，已经乘10，实际70度
#define Temp_Low 500  //温度下限，已经乘10，实际50度

#define Vol_High 200  //电压上限值，2.0V
#define Vol_Low  180  //电压下限值，1.8V

#define Current_High 200 //电流上限值，2.0A
#define Current_Low  180  //电流上限值，1.8A

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

#define KEY1      HAL_GPIO_ReadPin(KEY1_GPIO_Port,  KEY1_Pin)
#define KEY2      HAL_GPIO_ReadPin(KEY2_GPIO_Port,  KEY2_Pin)

#define KEY1_PRESS   1
#define KEY2_PRESS   2

//按键扫描函数
u8 key_scan(u8 mode)
{
	  static u8 key_up=0;
			if(key_up) mode=1;
		
		if(mode && (KEY1==0|| KEY2==0))
		{
			mode=0;
			HAL_Delay(10);
			if(KEY1==0)  return KEY1_PRESS;
			else if(KEY2==0)  return KEY2_PRESS;		 	
		}else if(KEY1==1 && KEY2==1) return key_up=1;	
		return 0;// 无按键按下	
}

uint16_t ADC_Value[3]={0};
//单路ad转换的函数
uint16_t dong_get_adc()
{
    //开启ADC1
    HAL_ADC_Start(&hadc1);
    //等待ADC转换完成，超时为100ms
    HAL_ADC_PollForConversion(&hadc1,100);
    //判断ADC是否转换成功
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC)){
         //读取值
       return HAL_ADC_GetValue(&hadc1);
    }
    return 0;
}
u16 current_temp=0;
//此函数在LCD1602上显示温度数据  x_start（0-15），y_start（0-1） LCD1602上的坐标
void display_temperature(u8 x_start,u8 y_start)
{
		static u8 flag=1;
	  
	  LCD1602_ShowStr(x_start,y_start,"Temp:",5);
	  current_temp=DS18B20_Get_Temp();//获取温度值
    if(current_temp!=850 && current_temp!=0 )
		{
			 if(flag)
			 {
				 flag=0;
				LCD1602_WriteCmd(0x01);
			 }
			 LCD1602_ShowNum(x_start+5,y_start, current_temp/100);
			 LCD1602_ShowNum(x_start+6,y_start, current_temp%100/10);
			 LCD1602_ShowChar(x_start+7,y_start,'.');
			 LCD1602_ShowNum(x_start+8,y_start, current_temp%100%10);
			 LCD1602_ShowChar(x_start+9,y_start,0xdf);
			 LCD1602_ShowChar(x_start+10,y_start,'C');
			 
			 printf("Temp:%d\r\n",current_temp/10);//串口输出温度
			 
			  if(current_temp>=Temp_High)//温度高温
				{
					HAL_GPIO_WritePin(Temp_High_GPIO_Port, Temp_High_Pin, GPIO_PIN_RESET);//高温指示灯亮、跳闸灯亮
					LCD1602_ShowStr(x_start+5,y_start,"High   ", 5);
				}
				if(current_temp<Temp_High  && current_temp>=Temp_Low)//温度高、跳闸信号灯灭
		    { 
				  HAL_GPIO_WritePin(Temp_High_GPIO_Port, Temp_High_Pin, GPIO_PIN_RESET);//高温指示灯亮、跳闸灯灭
					LCD1602_ShowStr(x_start+5,y_start,"Low   ", 5);
				}
				 if(current_temp<Temp_Low ) 
				 {
					HAL_GPIO_WritePin(Temp_High_GPIO_Port, Temp_High_Pin, GPIO_PIN_SET);//高温指示灯灭
				 }
		}
		else 
		{
			flag=1;
			LCD1602_ShowStr(0,0,"Reading DS18B20", 15);
			LCD1602_ShowStr(0,1,"Please Wait", 11);
		}	
}
//在LCD1602上显示AD采样值
void display_sensor_data(u8 x_start,u16 vol1,u16 vol2)
{
     	 LCD1602_ShowStr(x_start,0,"Vol:", 4);
	     LCD1602_ShowNum(x_start+4,0, vol1/100);
	     LCD1602_ShowChar(x_start+5,0,'.');
			 LCD1602_ShowNum(x_start+6,0, vol1%100/10);
			 LCD1602_ShowNum(x_start+7,0, vol1%100%10);
		   LCD1602_ShowChar(x_start+8,0,'V');
	
			 LCD1602_ShowStr(x_start,1,"Cur:", 4);
	     LCD1602_ShowNum(x_start+4,1, vol2/100);
			 LCD1602_ShowChar(x_start+5,1,'.');
			 LCD1602_ShowNum(x_start+6,1, vol2%100/10);
			 LCD1602_ShowNum(x_start+7,1, vol2%100%10);
			 LCD1602_ShowChar(x_start+8,1,'A');
}
	
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

	u8 error_flag=0;//故障标志位，有故障时此标志位为1
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	  u8 i=0;
	float voltage=0.0,current=0.0;
	  unsigned int value[2]={0};
		u16 cnt=0;
	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
    LCD1602_Init();
		DS18B20_Init();//DS18B20初始化
    HAL_Delay(10);  //延时一段时间	
    
    LCD1602_ShowStr(0,0,"AC contactor",12);
    LCD1602_ShowStr(0,1,"Control system",16);
    
    HAL_Delay(1000);//延时1S
		
    LCD1602_WriteCmd( 0x01);//清屏
		

			 
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
					for(i=0;i<2;i++){
							//分别存放通道1~2的ADC值
							value[i]=dong_get_adc();
					}		
					voltage=0.08*value[0];//电压
					current=0.08*value[1];//电流	
				  display_sensor_data(0,voltage,current);//在LCD1602上显示，获取的两个AD采样的值电压，电流
					
					if(voltage>=Vol_High)
				  { 
            //过压						
						HAL_GPIO_WritePin(Vol_High_GPIO_Port, Vol_High_Pin, GPIO_PIN_RESET);//电压过压指示灯亮
						HAL_GPIO_WritePin(Vol_Low_GPIO_Port, Vol_Low_Pin, GPIO_PIN_SET);//电压欠压指示灯灭
						
						 LCD1602_ShowStr(10,0,"VHigh", 5);
					}
					if(voltage<=Vol_Low)
					{
						//欠压
						HAL_GPIO_WritePin(Vol_Low_GPIO_Port, Vol_Low_Pin, GPIO_PIN_RESET);//电压欠压指示灯亮
						HAL_GPIO_WritePin(Vol_High_GPIO_Port, Vol_High_Pin, GPIO_PIN_SET);//电压过压指示灯灭
						
					 LCD1602_ShowStr(10,0,"VLow ", 5);
					}
					if(voltage<Vol_High &&voltage>Vol_Low)
					{
						//电压正常
						HAL_GPIO_WritePin(Vol_Low_GPIO_Port, Vol_Low_Pin, GPIO_PIN_SET);//电压欠压指示灯灭    正常
						HAL_GPIO_WritePin(Vol_High_GPIO_Port, Vol_High_Pin, GPIO_PIN_SET);//电压过压指示灯灭  正常
						LCD1602_ShowStr(10,0,"     ", 5);
					}
					
			    if(current<Current_High )//电流正常
					{					
						HAL_GPIO_WritePin(Cur_High_GPIO_Port, Cur_High_Pin, GPIO_PIN_SET);//电流过流指示灯灭 正常
					}
					
		   		if(current>=Current_High)
				  {   
						//过流			
						HAL_GPIO_WritePin(Cur_High_GPIO_Port, Cur_High_Pin, GPIO_PIN_RESET);//电流过流指示灯亮 
						
						LCD1602_ShowStr(10,0,"CHigh", 5);
					}
			    //电路异常
					if(voltage>=Vol_High || voltage<=Vol_Low || current>=Current_High || current_temp>=Temp_High  )
					{
							cnt++;
						error_flag=1;//故障标志位为1
						printf("CNT=%d\r\n",cnt);
					if(cnt>=10)//故障出现之后，等待2秒
						{
							//HAL_Delay(10);//200*10=2000ms
							cnt=0;
						 	HAL_GPIO_WritePin(Trip_GPIO_Port, Trip_Pin, GPIO_PIN_RESET);//跳闸指示灯亮
						  HAL_GPIO_WritePin(close_sw_GPIO_Port, close_sw_Pin, GPIO_PIN_SET);//合闸指示灯灭
					 	  HAL_GPIO_WritePin(Open_sw_GPIO_Port, Open_sw_Pin, GPIO_PIN_RESET);//分闸指示灯亮	
							HAL_TIM_PWM_Stop(&htim1,  TIM_CHANNEL_1);//停止PWM波形
						}
					}else
					{
							error_flag=0;//故障标志位为0
				  	HAL_GPIO_WritePin(Trip_GPIO_Port, Trip_Pin, GPIO_PIN_SET);//跳闸指示灯灭
					}
					printf("vol:%d,cur:%d\r\n",(u16)voltage,(u16)current);
					display_temperature(9,1);//获取温度值并在LCD1602上显示，当前温度值与阈值进行比较并报警
					printf("============\r\n");
    }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = ENABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Temp_High_Pin|close_sw_Pin|Trip_Pin|Open_sw_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IO_TEMP_GPIO_Port, IO_TEMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DB0_Pin|DB1_Pin|DB2_Pin|LCD1602_E_Pin 
                          |LCD1602_RW_Pin|LCD1602_RS_Pin|DB3_Pin|DB4_Pin 
                          |DB5_Pin|DB6_Pin|DB7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Cur_High_Pin|Vol_Low_Pin|Vol_High_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : Temp_High_Pin close_sw_Pin Trip_Pin Open_sw_Pin */
  GPIO_InitStruct.Pin = Temp_High_Pin|close_sw_Pin|Trip_Pin|Open_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : KEY1_Pin KEY2_Pin */
  GPIO_InitStruct.Pin = KEY1_Pin|KEY2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IO_TEMP_Pin */
  GPIO_InitStruct.Pin = IO_TEMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IO_TEMP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DB0_Pin DB1_Pin DB2_Pin DB3_Pin 
                           DB4_Pin DB5_Pin DB6_Pin DB7_Pin */
  GPIO_InitStruct.Pin = DB0_Pin|DB1_Pin|DB2_Pin|DB3_Pin 
                          |DB4_Pin|DB5_Pin|DB6_Pin|DB7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD1602_E_Pin LCD1602_RW_Pin LCD1602_RS_Pin */
  GPIO_InitStruct.Pin = LCD1602_E_Pin|LCD1602_RW_Pin|LCD1602_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Cur_High_Pin Vol_Low_Pin Vol_High_Pin */
  GPIO_InitStruct.Pin = Cur_High_Pin|Vol_Low_Pin|Vol_High_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	   
     if(GPIO_Pin==GPIO_PIN_2)
		 {
			 if(error_flag==0)//合闸
			 {
				 TIM1->CCR1=500  ;//把Value 写入捕获/比较寄存器  改变此数值  即可改变PWM占空比
				HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//重新启动PWM

				HAL_GPIO_WritePin(close_sw_GPIO_Port, close_sw_Pin, GPIO_PIN_RESET);//合闸指示灯亮
				HAL_GPIO_WritePin(Open_sw_GPIO_Port, Open_sw_Pin, GPIO_PIN_SET);//分闸指示灯灭
			 }
		  }
			 
		 else if(GPIO_Pin==GPIO_PIN_3)
			{
					printf("B");
					HAL_TIM_PWM_Stop(&htim1,  TIM_CHANNEL_1);//停止PWM波形
					HAL_GPIO_WritePin(close_sw_GPIO_Port, close_sw_Pin, GPIO_PIN_SET);//合闸指示灯灭
					HAL_GPIO_WritePin(Open_sw_GPIO_Port, Open_sw_Pin, GPIO_PIN_RESET);//分闸指示灯亮
			}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
