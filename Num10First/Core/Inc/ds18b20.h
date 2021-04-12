#ifndef __DS18B20_H
#define __DS18B20_H
#include "stdint.h"
typedef int16_t s16;

#include "main.h"

#define OW_DIR_OUT() 	mode_output1()
#define OW_DIR_IN() 	mode_input1()
#define OW_OUT_LOW() 	(HAL_GPIO_WritePin(GPIOA,(uint16_t)GPIO_PIN_6,GPIO_PIN_RESET))
#define OW_GET_IN()  	(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6))

#define OW_SKIP_ROM 		0xCC
#define DS18B20_CONVERT 	0x44
#define DS18B20_READ 		0xBE


void ds18b20_init_x(void);
s16 ds18b20_read(void);
void DisplayDs18b20(void);
uint32_t ds18b20(void ); 
#endif

