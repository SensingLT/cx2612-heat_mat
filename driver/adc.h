#ifndef __ADC_H__
#define __ADC_H__

#include <stdbool.h>
#include <stdint.h>

void Adc_Init(void);

//返回最后更新的ntc的AD值
void Adc_NtcGet(uint16_t *ad1, uint16_t *ad2);
//返回最后更新的cs的AD值
void Adc_csVoltageGet(uint16_t *ad);

#endif //__ADC_H__
