#include "seg.h"

//
uint8_t Seg7[17] = { 0x3f,0x06,0x5b,0x4f, 0x66,0x6d,0x7d,0x07,0x7f,0x6f,0x77,0x7c, 0x39,0x4f,0x79,0x78,0x00}; 

void SEG_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_GPIOA_CLK_ENABLE();
		
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

//
void SEG_DisplayValue(uint8_t Bit1,  uint8_t Bit2, uint8_t Bit3)
{
	uint8_t i = 0;	//
	uint32_t code_tmp = (Seg7[Bit3]<<16) + (Seg7[Bit2]<<8) + Seg7[Bit1];
	for(i=0;i<24;i++){

		if(code_tmp & 0x800000){
			SER_H;
		}else{
			SER_L;
		}
		SCK_H;
		code_tmp = code_tmp << 1;   
		SCK_L;
	}
/*	
	code_tmp = Seg7[Bit2];
	for(i=0;i<8;i++){

		if(code_tmp & 0x80){
			SER_H;
		}else{
			SER_L;
		}
		SCK_H;
		code_tmp = code_tmp << 1;   
		SCK_L;
	}	
	
	code_tmp = Seg7[Bit1];
	for(i=0;i<8;i++){

		if(code_tmp & 0x80){
			SER_H;
		}else{
			SER_L;
		}
		SCK_H;
		code_tmp = code_tmp << 1;   
		SCK_L;
	}		*/	
	RCLK_H;
	RCLK_L;
}
