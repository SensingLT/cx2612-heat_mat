#ifndef __ADC_H__
#define __ADC_H__

#include <stdbool.h>
#include <stdint.h>

void Adc_Init(void);

//返回最后更新的ntc的AD值
uint32_t Adc_NtcGet(uint16_t *ad1, uint16_t *ad2);
//返回最后更新的cs的电流值
uint32_t Adc_csCurrentGet(uint32_t *current);
//返回最后更新的ph的阻值
uint32_t Adc_phGet(uint16_t *ad1, uint16_t *ad2);
#endif //__ADC_H__
