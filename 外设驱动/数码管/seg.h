#include "stm32g4xx_hal.h"

#ifndef __SEG_H
#define __SEG_H

#define RCLK_H			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_SET))
#define RCLK_L			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_2,GPIO_PIN_RESET))

#define SER_H			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET))
#define SER_L			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET))

#define SCK_H			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_SET))
#define SCK_L			(HAL_GPIO_WritePin(GPIOA,GPIO_PIN_3,GPIO_PIN_RESET))

void STM3210B_SEG_Init(void);
void SEG_DisplayValue(uint8_t Bit1,uint8_t Bit2,uint8_t Bit3);

#endif
