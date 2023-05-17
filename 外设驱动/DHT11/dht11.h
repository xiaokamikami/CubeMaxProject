#ifndef __DHT11_H
#define __DHT11_H

void dht11_init (void );
void delay(unsigned int n);

unsigned int dht11_read(void);
void DisplayDht11(void);
#define GPIO_ResetBits(GPIOA, GPIO_Pin_7)	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_RESET)
#define GPIO_SetBits(GPIOA, GPIO_Pin_7)		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7,GPIO_PIN_SET)
#define GPIO_Pin_7 GPIO_PIN_7
#define GPIO_ReadInputDataBit	HAL_GPIO_ReadPin
#endif
