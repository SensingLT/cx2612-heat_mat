#ifndef __ADC_H__
#define __ADC_H__

#include <stdbool.h>
#include <stdint.h>

void Adc_Init(void);

//返回最后更新的Tick值
void Adc_Get(uint16_t *ad1, uint16_t *ad2);

#endif //__ADC_H__
