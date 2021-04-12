/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//第十届蓝桥杯决赛题
//制作人QQ:1317379456

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static uint16_t N_Num = 0; //变化次数
static double IIC_huancun = 0;

static uint8_t Tmm = 30; 
static uint8_t Xmm = 1; 
static uint8_t Setting = 1; //设置模式
static uint8_t SetOb = 1; //设置的对象
static char LCDTEMP[20] ; //LCD暂存

static uint8_t PWMON =0; //PWM捕捉次数
static double PWMDW1 =0;	//第一次下降沿
static double PWMUP2 =0;	//第二次上升沿
static uint32_t	PWM_YICHU1 =0;	//溢出次数
static uint32_t	PWM_YICHU2 =0;
static uint32_t PWM_Zhankongbi = 0;	//占空比

//static uint8_t time4_PWM = 0;  //PWM测量刷新信号

static uint8_t time4_1S =0;		//1s定时
static uint8_t time4_2S = 0; //2S 数码管刷新定时
static uint8_t seg_num = 0; //数码管显示页面

static char BUFF_SIZE = 50; //接收长度
static uint8_t RXbuf[50];	//接收缓存

static uint16_t UART_RX_STA = 0;     // 第15bit表示一帧数据接收完成，第14~0位表示接收到的数据量
static char UARTTEMP[30];
static uint8_t RX_ST[]="ST\\r\\n";		//串口接受指令
static uint8_t RX_PARA[]="PARA\\r\\n";	
static double DS18B20_TEMP = 0;		//温度传感器

static uint8_t tim1_LD8 = 0;		//LD8闪灯标志
static uint8_t led1 = 0 ;			//LD1点亮标志

static double ADC_Value[4]={0};   //ADC电压储存     
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void yemian_JM()
{
	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET )
	{
		
		Setting++;
		HAL_TIM_IC_Stop_IT(&htim3, TIM_CHANNEL_1);
		if (Setting == 3){Setting = 1;HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);}
		LCD_Clear(White);
		while(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET);
	}
	if(Setting == 2)
	{
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET  )
		{
			SetOb++;
			HAL_Delay(200);
			if (SetOb == 3){SetOb =1;}
		}
		if(SetOb == 1)
		{
			if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET )
			{
				Tmm++;
				HAL_Delay(500);
				if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)	{Tmm+=9;}
			}
			else if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==GPIO_PIN_RESET )
			{
				Tmm--;
				HAL_Delay(500);
				if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==GPIO_PIN_RESET)	{Tmm-=9;}
				
			}
		}
		else if(SetOb == 2)
		{
			if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==RESET  )
			{
				Xmm++;
				HAL_Delay(200);
			}
			else if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==RESET )
			{
				Xmm++;
				HAL_Delay(200);
			}
			if (Xmm > 2){ Xmm = 1;}
			else if (Xmm == 0){ Xmm = 1;}
			if (Tmm >40){ Tmm = 20;}
			else if (Tmm == 0	){ Tmm = 40;}
		}

	}

}

static double ADCRead(ADC_HandleTypeDef *hadcx)
{
	    //开启ADC1
    HAL_ADC_Start(&hadc2);
    //等待ADC转换完成，超时为100ms
    HAL_ADC_PollForConversion(&hadc2,100);
    //判断ADC是否转换成功
    HAL_ADC_GetValue(&hadc2);
	//读取值
    
    
}
static double ADCRead_Num(uint8_t num)
{
	
	ADC_Value[1]= ADCRead(&hadc2);
	HAL_Delay(5);
	ADC_Value[2]= ADCRead(&hadc2);
	return ADC_Value[num];
}
static void LCDLive()
{
	if(Setting == 1)															//主页面
	{
		LCD_DisplayStringLine(Line0,(uint8_t *)"        Main       ");	
		sprintf(LCDTEMP,"    AO1:%.2lfV     ",ADCRead_Num(1));
		LCD_DisplayStringLine(Line1,(uint8_t *)LCDTEMP);	
		sprintf(LCDTEMP,"    AO2:%.2lfV     ",ADCRead_Num(2));
		LCD_DisplayStringLine(Line3,(uint8_t *)LCDTEMP);

		sprintf(LCDTEMP,"    TEMP:%.2lf C    ",DS18B20_TEMP);
		LCD_DisplayStringLine(Line7,(uint8_t *)LCDTEMP);
		sprintf(LCDTEMP,"    N:%d           ",N_Num);
		LCD_DisplayStringLine(Line9,(uint8_t *)LCDTEMP);
			if (PWMON == 3	 )									//重新计算占空比
			{
					PWMDW1 =1000000/ (PWMDW1+50000*PWM_YICHU1);
					//PWMUP2 =1000000/ (PWMUP2+50000*PWM_YICHU2);
					PWM_Zhankongbi =(100* (PWMDW1 / (PWMDW1+1000000/ (PWMUP2+50000*PWM_YICHU2))))+0.5 ;		
					
					sprintf(LCDTEMP,"    PWM2:%d%%    ",PWM_Zhankongbi);
					LCD_DisplayStringLine(Line5,(uint8_t *)LCDTEMP);
				
					PWMON = 0;
					PWM_YICHU1 = 0;
					PWM_YICHU2 = 0;
			}
	}

	else if(Setting == 2)														//设置模式
	{

		if(SetOb == 1)
		{
			LCD_DisplayStringLine(Line0,(uint8_t *)"       Para      ");
			sprintf(LCDTEMP,"    T: %d    ",Tmm);
			LCD_SetTextColor(Red);
			LCD_DisplayStringLine(Line2,(uint8_t *)LCDTEMP);
			LCD_SetTextColor(Blue);
			sprintf(LCDTEMP,"    X: AO%d    ",Xmm);
			LCD_DisplayStringLine(Line5,(uint8_t *)LCDTEMP);		
		}
		else if(SetOb == 2)
		{
			LCD_DisplayStringLine(Line0,(uint8_t *)"       Para      ");
			sprintf(LCDTEMP,"    T: %d    ",Tmm);
			LCD_DisplayStringLine(Line2,(uint8_t *)LCDTEMP);			
			sprintf(LCDTEMP,"    X: AO%d    ",Xmm);
			LCD_SetTextColor(Red);
			LCD_DisplayStringLine(Line5,(uint8_t *)LCDTEMP);
			LCD_SetTextColor(Blue);			
		}
	}
}
static void LEDClear()
{
	
	HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET); //LED全部熄灭
	HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关

}

static void UART_TempSend()
{
	double linshi =0;

	linshi = PWM_Zhankongbi*3.3;
	if(ADCRead_Num(1)>linshi || ADCRead_Num(2)> linshi)
	{
		if(led1!=2 ){led1 =1;}

		if(time4_1S > 4)
		{
			time4_1S = 0;
			memset(UARTTEMP,0,30); 
 			sprintf(UARTTEMP,"$%.2f\r\n",DS18B20_TEMP);
 			HAL_UART_Transmit(&huart1,(uint8_t *)UARTTEMP,sizeof(UARTTEMP),100);

		}
	}
	else if(led1!=0	){led1=5;}
}

static void UART_TN()
{
	if( DS18B20_TEMP> Tmm )
	{
		HAL_TIM_Base_Start_IT(&htim1);
	}
	else if	(DS18B20_TEMP < Tmm )
	{
		HAL_TIM_Base_Stop_IT(&htim1);
		if (tim1_LD8 != 0)tim1_LD8 = 3;
	}

}
static void Led_bulle()
{
	if(tim1_LD8 == 1)
	{
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
		if(led1 ==2 )	
		{	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15|GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_9,GPIO_PIN_SET);
		}
		else 
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_15|GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
		tim1_LD8 = 2;
	}
	else if (tim1_LD8  == 4)
	{
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
 		if(led1 ==2)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9,GPIO_PIN_SET);
		}
		else{	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET);	}
		
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
		tim1_LD8 = 0;
	}
	else if (tim1_LD8 == 3)
	{
		tim1_LD8 = 0;
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
 		if(led1 ==2)
		{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9,GPIO_PIN_SET);
		}
		else{	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET);	}
		
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
	}
	if(led1 ==1	)
	{
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET); 
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
		led1 = 2;
	}
	else if (led1 == 5) 
	{	
		led1 = 0;
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
	}
}

uint16_t x24c02_read(uint8_t address)		//I2C读
{
	uint16_t val;
	
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(address);
	I2CWaitAck();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	val = I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	return(val);
}

void x24c02_write(uint8_t address,uint16_t info)	//I2C写
{
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(address);
	I2CWaitAck();
	I2CSendByte(info);
	I2CWaitAck();

	I2CStop();

}
void USART1_IRQHandler(void)
{
	if(__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);
		HAL_UART_DMAStop(&huart1);
		UART_RX_STA = BUFF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
		RXbuf[UART_RX_STA] = 0;
//		UART_RX_STA |= 0X8000;
//		if(UART_RX_STA & 0X8000)
//		{
			if(strcmp((char *)RXbuf,(char *)RX_ST) == 0)
			{
				memset(UARTTEMP,0,30); 
				sprintf(UARTTEMP,"$%.2f\r\n",DS18B20_TEMP);
				HAL_UART_Transmit(&huart1,(uint8_t *)UARTTEMP,sizeof(UARTTEMP),100 );

			}
			else if(strcmp((char *)RXbuf,(char *)RX_PARA) == 0)
			{
				memset(UARTTEMP,0,30); 
				sprintf(UARTTEMP,"#%d,AO%d\r\n",Tmm,Xmm);
				HAL_UART_Transmit(&huart1,(uint8_t *)UARTTEMP,sizeof(UARTTEMP),100);

			}
//			UART_RX_STA = 0;  // 清除标记
//		}
	HAL_UART_Receive_DMA(&huart1, RXbuf, BUFF_SIZE );  // 重新启动DMA接收
	}
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	

	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC2_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	LCD_Init();
  	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue);
	
	ds18b20_init_x();

	LEDClear();
	__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);
	__HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	
	HAL_TIM_Base_Start_IT(&htim4);
	
	
	HAL_UART_Receive_DMA(&huart1,RXbuf,BUFF_SIZE);
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	N_Num = x24c02_read(0x00);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	DS18B20_TEMP = ((int)((ds18b20_read()/16.0)*100)) ;
	DS18B20_TEMP = DS18B20_TEMP/100;
	if(IIC_huancun != DS18B20_TEMP)
	{
		N_Num++;
		IIC_huancun = DS18B20_TEMP;
		x24c02_write(0x00,N_Num);
	}
	yemian_JM();
	LCDLive();
	UART_TempSend();
	UART_TN();
	Led_bulle();
	

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	//sprintf(LCDbuf, "   TEXT");		LCD_DisplayStringLine(Line8, (uint8_t *)LCDbuf);
	if(htim->Instance == TIM1)
	{
		if	   (tim1_LD8 == 0)		{	tim1_LD8 = 1;}
		else if(tim1_LD8 == 2)		{	tim1_LD8 = 4;}
	}
	if(htim->Instance == TIM4	&& Setting == 1)
	{
		time4_2S++;
		time4_1S++;
		if(time4_2S == 1 )
		{
			if(seg_num == 0)	{SEG_DisplayValue(10, 0, Xmm); seg_num = 1;	}
			else			    {SEG_DisplayValue(Tmm/10,  Tmm%10, 17); seg_num = 0;	}
			time4_2S = 0;
		}
		
	}
	if(htim->Instance == TIM3)
	{
		if(PWMON == 1)
		{
			PWM_YICHU1++;

		}
		else if (PWMON == 2)
		{
			PWM_YICHU2++;
		}
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		if(PWMON == 1 && Setting == 1 )
		{
			PWMDW1 = __HAL_TIM_GetCounter(&htim3);
			__HAL_TIM_SetCounter(&htim3,0);
			PWMON = 2;
			
		}

	}
	else if(htim ->Channel == HAL_TIM_ACTIVE_CHANNEL_2	)
	{
		if(PWMON == 0)
		{
			__HAL_TIM_SetCounter(&htim3,0);
			PWMON = 1;

		}
		else	if(PWMON == 2)
		{
			PWMUP2 = __HAL_TIM_GetCounter(&htim3);
			__HAL_TIM_SetCounter(&htim3,0);
			PWMON = 3;


		}
	}

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
