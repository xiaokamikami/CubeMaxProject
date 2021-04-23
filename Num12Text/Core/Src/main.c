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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static char LCD_TEMP[20];
uint8_t KEY_MODE =1;

double Vmax = 3.0;
double Vmin = 1.0;
double SETOK_Vmax =3.0;
double SETOK_Vmin =1.0;
uint8_t VT_MODE = 0; //电压状态

static double  Vlote = 0; //读取电压

static uint8_t VT_MODE_R = 0;

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
static double ADC_READ()	//采十次样平均滤波
{
	double adc = 0;
	uint8_t i = 0;
	while(i!=10)			
	{
		HAL_ADC_Start(&hadc2);
		adc += (HAL_ADC_GetValue(&hadc2)*3.3)/4096;	
		i+=1;
	}
	adc /=10;
	return adc;
}
static void LED_CLEAR()			//LED全关
{
	HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_SET);	//锁存器开
  	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9|GPIO_PIN_8,GPIO_PIN_SET); //LED全部熄灭
	HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_RESET);//锁存器关
}

static void LCD_LIVE()
{
	if(KEY_MODE == 1)
	{
		LCD_DisplayStringLine(Line1,(uint8_t *)"      Data");
		HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		while(KEY_MODE == 1)
		{
			Vlote = ADC_READ();				
			if		(	Vlote < SETOK_Vmin)				{	VT_MODE = 2; }
			else if (	Vlote > SETOK_Vmax)				{	VT_MODE = 1; }
			else	{	VT_MODE = 3; }
			if		(VT_MODE_R!=VT_MODE)				{	VT_MODE_R=VT_MODE;}
			if(VT_MODE == 1)	
			{
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 5000);
				__HAL_TIM_SetAutoreload(&htim2, 9999);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_SET);	//锁存器开
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_9,GPIO_PIN_SET); //LED全部熄灭
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_RESET);//锁存器关
			}
			else if(VT_MODE == 2)	
			{
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 800);
				__HAL_TIM_SetAutoreload(&htim2, 999);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_SET);	//锁存器开
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_10|GPIO_PIN_15|GPIO_PIN_8,GPIO_PIN_SET); //LED全部熄灭
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_9,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_RESET);//锁存器关
			}
			else if(VT_MODE == 3)	
			{	
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 20);
				__HAL_TIM_SetAutoreload(&htim2,99);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_SET);	//锁存器开
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_9|GPIO_PIN_15|GPIO_PIN_8,GPIO_PIN_SET); //LED全部熄灭
				HAL_GPIO_WritePin(GPIOC,GPIO_PIN_10,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED_LOCK_GPIO_Port, LED_LOCK_Pin,GPIO_PIN_RESET);//锁存器关
				
			}
			sprintf(LCD_TEMP," V:%.2lfV    ",Vlote);
			LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
			sprintf(LCD_TEMP," A:%d        ",VT_MODE);
			LCD_DisplayStringLine(Line4,(uint8_t *)LCD_TEMP);
		}
	}
	else
	{
		LCD_DisplayStringLine(Line1,(uint8_t *)"      Para");
		HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2); 
		LED_CLEAR();
		while(KEY_MODE == 2)
		{
			sprintf(LCD_TEMP," Vmax:%.1lfV   ",Vmax);
			LCD_DisplayStringLine(Line3,(uint8_t *)LCD_TEMP);
			sprintf(LCD_TEMP," Vmin:%.1lfV   ",Vmin);
			LCD_DisplayStringLine(Line4,(uint8_t *)LCD_TEMP);
		}
		 
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
  MX_ADC2_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  LED_CLEAR();
  LCD_Init();
  LCD_Clear(Black);
  LCD_SetBackColor(Black);
  LCD_SetTextColor(White);
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  LCD_LIVE();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
