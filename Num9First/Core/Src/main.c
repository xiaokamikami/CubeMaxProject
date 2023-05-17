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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//第九届蓝桥杯省赛题
//制作人QQ:1317379456

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static uint8_t B1_Num = 0;
static uint8_t B2_Setting = 0;	//设置中
static uint8_t B4_Star = 0 ;	//0 停止 1暂停 2运行

static uint8_t Hour = 0;	//时
static uint8_t Min  = 0;	//分
static uint8_t Sek  = 0;	//秒

static uint8_t TIME_MODE_HC = 0; //模式切换缓存
static uint8_t LCD_TEMP[20];

static uint8_t LED_1S = 2;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void I2C_Wt(uint8_t num,uint8_t info);
uint8_t I2C_Read(uint8_t num);
static void LCD_Live(void);
static void TIME_Star(void);
static void LCD_Char(uint8_t num,uint16_t k);
static void LED_BLK(void);
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
void Key_Set(void)	//按键
{
	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_RESET)	
	{	
		HAL_Delay(100); 
		B1_Num ++;	
		if(B1_Num>4){B1_Num=0;}
		Hour = I2C_Read(B1_Num*3);
		Min  = I2C_Read((B1_Num*3)+1);
		Sek  = I2C_Read((B1_Num*3)+2);
		if(Hour>24)Hour=0;if(Min>60)Min=0;if(Sek>60)Sek=0;
	}
	if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET)
	{
		HAL_Delay(800);
		if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin)==GPIO_PIN_RESET)
		{	
			I2C_Wt(B1_Num*3,Hour);
			I2C_Wt((B1_Num*3)+1,Min);
			I2C_Wt((B1_Num*3)+2,Sek);
			B2_Setting = 0;
			LCD_Clear(White);
			HAL_Delay(200);
		}
		else{B2_Setting ++; if(B2_Setting >3){B2_Setting =0;}}
		LCD_Clear(White);
	}
	if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)
	{
		HAL_Delay(800);
		if(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)
		{	
			while(HAL_GPIO_ReadPin(KEY3_GPIO_Port,KEY3_Pin)==GPIO_PIN_RESET)
			{		
					if(B2_Setting == 1){Hour++;}
					if(B2_Setting == 2){Min++;}
					if(B2_Setting == 3){Sek++;}
					if(Hour >24){Hour=0;}else if(Min>60){Min=0;}else if(Sek >60){Sek=0;}
					LCD_Live();
					HAL_Delay(100);
					
			}
		}
		else
		{
			if(B2_Setting == 1){Hour++;}
			if(B2_Setting == 2){Min++;}
			if(B2_Setting == 3){Sek++;}
			if(Hour >24){Hour=0;}if(Min>60){Min=0;}if(Sek >60){Sek=0;}
		}
	}
	if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==GPIO_PIN_RESET)
	{
		HAL_Delay(800);
		if(HAL_GPIO_ReadPin(KEY4_GPIO_Port,KEY4_Pin)==GPIO_PIN_RESET)
		{
			B4_Star=0;
			LCD_Live();
			HAL_Delay(200);
		}
		else 
		{
			if(B4_Star !=2){B4_Star=2;}else if(B4_Star == 2){B4_Star=1;}
			if(B2_Setting !=0){	B2_Setting=0;}
		}
		TIME_Star();
	}
}
static void LEDClear()	//清空LED
{
	
	HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET); //LED全部熄灭
	HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关

}
uint8_t I2C_Read(uint8_t num)	//I2C读
{
	uint8_t val = 0;
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(num);
	I2CWaitAck();
	
	I2CStart();
	I2CSendByte(0xa1);
	I2CWaitAck();
	val = I2CReceiveByte();
	I2CWaitAck();
	I2CStop();
	HAL_Delay(50);
	return(val);
	
}

void I2C_Wt(uint8_t num,uint8_t info)		//I2C写
{	
	I2CStart();
	I2CSendByte(0xa0);
	I2CWaitAck();
	
	I2CSendByte(num);
	I2CWaitAck();
	
	I2CSendByte(info);
	I2CWaitAck();
	
	I2CStop();
	HAL_Delay(50);
}

static void LCD_Live(void)	//LCD显示
{
	uint8_t B1_YE = B1_Num +1;
	if(B2_Setting == 0)
	{
		sprintf((char *)LCD_TEMP,"     NO.%d      ",B1_YE);
		LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
		sprintf((char *)LCD_TEMP,"     %d:%d:%d   ",Hour,Min,Sek);
		LCD_DisplayStringLine(Line5,(uint8_t *)LCD_TEMP);

	}
	else	if(B2_Setting == 1)
	{
		sprintf((char *)LCD_TEMP,"     NO.%d      ",B1_YE);
		LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Red);
		LCD_Char(Hour,1);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Blue);
		LCD_Char(Min,2);
		LCD_Char(Sek,3);
	}
	else	if(B2_Setting == 2)
	{
		sprintf((char *)LCD_TEMP,"     NO.%d      ",B1_YE);
		LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
		LCD_Char(Hour,1);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Red);
		LCD_Char(Min,2);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Blue);
		LCD_Char(Sek,3);
	}
	else	if(B2_Setting == 3)
	{
		sprintf((char *)LCD_TEMP,"     NO.%d      ",B1_YE);
		LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
		LCD_Char(Hour,1);
		LCD_Char(Min,2);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Red);
		LCD_Char(Sek,3);
			LCD_SetBackColor(White);
			LCD_SetTextColor(Blue);
	}
	if  (B2_Setting != 0)	{	LCD_DisplayStringLine(Line7,(uint8_t *)"     Setting    ");}
	else if	(B4_Star == 0)		{	LCD_DisplayStringLine(Line7,(uint8_t *)"     Standby    ");}
	else if	(B4_Star == 1)		{	LCD_DisplayStringLine(Line7,(uint8_t *)"     Pause      ");}
	else if	(B4_Star == 2)		{	LCD_DisplayStringLine(Line7,(uint8_t *)"     Running    ");}
}
static void TIME_Star()	//定时器运行
{
	if(TIME_MODE_HC != B4_Star)
	{
		TIME_MODE_HC = B4_Star;
		if(TIME_MODE_HC == 2)
		{		
			HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
			HAL_TIM_Base_Start_IT(&htim4);
			LED_1S = 0;
		}
		else
		{
			HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
			HAL_TIM_Base_Stop_IT(&htim4);
			LED_1S = 2;
			LED_BLK();
			LED_1S = 10;
		}
	}	
}
static void LCD_Char(uint8_t num,uint16_t k)
{
	uint8_t num1=0,num2=0;
	
	if(num>=10){num1 = num/10%10+48;LCD_DisplayChar(Line5,319-(16*(2+k*3)),(uint8_t )num1);}
	else if(num<10)	{LCD_DisplayChar(Line5,319-(16*(2+k*3)),(uint8_t )32);}
	if(num>0)num2 = (num%10)+48;
	else if(num==0)num2 = 48;
	LCD_DisplayChar(Line5,319-(16*(3+k*3)),(uint8_t )num2);
	if(k!=3){LCD_DisplayChar(Line5,319-(16*(4+k*3)),58);}
}
static void time_star(void)
{
	if(Sek==0 && Min==0 && Hour==0){B4_Star = 0;}
	else if(Sek==0	&& Min!=0)
	  {	
			Sek=60;
			Min--;
	  }
	if(Min==0 && Hour!=0)	 
	  {	
		  	Min=60;
		    Hour--;
	  }
}
static void LED_BLK(void)
{
		if(LED_1S == 1)
		{
			HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9,GPIO_PIN_SET); 
			HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
			LED_1S = 5;
		}
		else if(LED_1S == 2)
		{
			HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_SET);	//锁存器开
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET); 
			HAL_GPIO_WritePin(LEDLOCK_GPIO_Port, LEDLOCK_Pin,GPIO_PIN_RESET);//锁存器关
			LED_1S = 0;
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
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	LEDClear();
	LCD_Init();
	LCD_Clear(White);
	LCD_SetBackColor(White);
	LCD_SetTextColor(Blue);
	Hour = I2C_Read(0x00);
	Min  = I2C_Read(0x01);
	Sek  = I2C_Read(0x02);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Key_Set();
	  LCD_Live();
	  LED_BLK();

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM4)
	{
		Sek--;
		time_star();
		if(LED_1S==0)LED_1S = 1;
		else if (LED_1S == 5)LED_1S=2;
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
