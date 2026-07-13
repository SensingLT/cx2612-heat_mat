#include "protection.h"
#include "PT32Y003x.h"
#include "heat.h"
#include "adc.h"
#include "uart.h"
/*
 这部分主要用来检测加热芯片的状态，当处在异常状态下，进行调节和安全保护
*/
#define SEL0_PORT GPIOA
#define SEL0_PIN  GPIO_Pin_3

#define SEN_PORT GPIOB
#define SEN_PIN  GPIO_Pin_4


typedef struct{
	uint16_t adc;//电流采样的ADC值
	uint16_t current;//电流
	uint8_t  status;//工作状态
}status_t;


void Protection_init(void){
	GPIO_InitTypeDef GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP; 
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull; 
    GPIO_InitStructure.GPIO_Pin = SEL0_PIN;       
    GPIO_Init(SEL0_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SEN_PIN;   //该引脚为高电平时，才能开启电流采样功能
    GPIO_Init(SEN_PORT, &GPIO_InitStructure);	
	
	GPIO_ResetBits(SEN_PORT,SEN_PIN);
	GPIO_ResetBits(SEL0_PORT,SEL0_PIN);
}


//切换电流检测通道
void Protection_swCurrentCH(uint8_t ch){
	if(ch == HEAT_CHANNEL_0){
		GPIO_SetBits(SEN_PORT,SEN_PIN);
		GPIO_ResetBits(SEL0_PORT,SEL0_PIN);
	}
	else if(ch == HEAT_CHANNEL_1){
		GPIO_SetBits(SEN_PORT,SEN_PIN);
		GPIO_SetBits(SEL0_PORT,SEL0_PIN);
	}
	
}

//void Protection_Task(void){
//	uint16_t adc = 0;
//	Adc_csVoltageGet(&adc);
//	DBG_LN("voltage  ad : %d ",adc);
//}
