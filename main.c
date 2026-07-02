#include "PT32Y003x.h"
#include <stdbool.h>
#include <string.h>
#include "tick.h"
#include "wdg.h"
#include "adc.h"
#include "uart.h"
#include "crc.h"
#include "config.h"
#include "ntc.h"
#include "heat.h"
#include "sif.h"

// 复位控制代码，必须使用 
void NRST_PBO_SetZero(void) 
{ 
    GPIO_InitTypeDef GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OutPP; 
    GPIO_InitStructure.GPIO_Pull = GPIO_Pull_NoPull; 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;            //nRST -- PBO 
    GPIO_Init(GPIOB, &GPIO_InitStructure); 
    GPIO_DigitalRemapConfig(AFIOB, GPIO_Pin_0, AFIO_AF_1, ENABLE); 
    GPIOB->MASKL[0x01] = 0; 
    PWR->PVDR = 0x07;                                 //LVD-2.5V复位 
}

//query_seat_req_t test;

int main (void) {
	NRST_PBO_SetZero();
	Tick_Init();
	Uart_Init(115200);
	Wdg_Init(WDG_COUNTER_PER_SECOND); //timeout=1s 设置看门狗重装载值
	Adc_Init();
	Heat_GPIOInit();
	SIF_Init();
	Heat_PIDInit(0, 2.5f, 0.10f, 0.5f);   // 通道0
   // Heat_PIDInit(1, 5.0f, 0.10f, 0.5f);   // 通道1
	
   // Heat_SetTargetTemp(0, 450);
  //  Heat_SetTargetTemp(1, 600);
	
	uint8_t revMsg[UART_MAX_REV_LEN];
	while (1) {
		//SIF_Task();
//		static uint32_t tempTick = 0;
//		if(Tick_Passed(&tempTick,50)){
//			//Heat_ControlTask();
//		}
		Wdg_Feed();
	}
}




#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(u8* file, u32 line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    Uart_SendStr("Wrong parameters value: file %s on line %lu\r\n", file, line);
    /* Infinite loop */
    while (1) {
    }
}
#endif


