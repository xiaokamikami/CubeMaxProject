#include "stm32g4xx_hal.h"
#include "main.h"
#include "dht11.h"
#define delay_us(X)  delayd(X*72/5)

void delayd(unsigned int n)
{
  while (n--);
}

void dht11_init (void )
{
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Enable  clock */
  
  
  /* Configure Ports */
  GPIO_InitStructure.Pin = GPIO_PIN_7;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET);

}

void mode_input(void )
{
  GPIO_InitTypeDef GPIO_InitStruct;

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode =GPIO_MODE_INPUT;
  GPIO_InitStruct.Speed =GPIO_SPEED_LOW ;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
void mode_output(void )
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.Pin = GPIO_PIN_7;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed =  GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

unsigned int dht11_read(void)
{
  int i;
  long long val;
  int timeout;

  GPIO_ResetBits(GPIOA, GPIO_Pin_7);
  delay_us(18000);  //pulldown  for 18ms
  GPIO_SetBits(GPIOA, GPIO_Pin_7);
  delay_us( 20 );	//pullup for 30us

  mode_input();

  //等待DHT11拉高，80us
  timeout = 5000;
  while( (! GPIO_ReadInputDataBit  (GPIOA, GPIO_Pin_7)) && (timeout > 0) ) timeout--;	 //wait HIGH

  //等待DHT11拉低，80us
  timeout = 5000;
  while( GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_7) && (timeout > 0) ) timeout-- ;	 //wait LOW

#define CHECK_TIME 28

  for(i=0;i<40;i++)
  {
	timeout = 5000;
	while( (! GPIO_ReadInputDataBit  (GPIOA, GPIO_Pin_7)) && (timeout > 0) ) timeout--;	 //wait HIGH

	delay_us(CHECK_TIME);
	if ( GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_7) )
	{
	  val=(val<<1)+1;
	} else {
	  val<<=1;
	}

	timeout = 5000;
	while( GPIO_ReadInputDataBit (GPIOA, GPIO_Pin_7) && (timeout > 0) ) timeout-- ;	 //wait LOW
  }

  mode_output();
  GPIO_SetBits(GPIOA, GPIO_Pin_7);

  if (((val>>32)+(val>>24)+(val>>16)+(val>>8) -val ) & 0xff  ) return 0;
    else return val>>8; 

}

