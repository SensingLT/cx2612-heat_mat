#include "adc.h"
#include "PT32Y003x.h"
#include "tick.h"
#include "uart.h"

#define AD_TO_RES(ad) (10 * (ad) / (4096 - ad))

//PA1-ADC_IN1
#define NTC0_PORT AFIOA
#define NTC0_PIN GPIO_Pin_1
#define NTC0_ADC_CHANNEL ADC_Channel_1
#define NTC0_ADC_SCAN_CHANNEL ADC_ScanChannel_0

//PA2-ADC_IN0
#define NTC1_PORT AFIOA
#define NTC1_PIN GPIO_Pin_2
#define NTC1_ADC_CHANNEL ADC_Channel_0
#define NTC1_ADC_SCAN_CHANNEL ADC_ScanChannel_1

//PC3-ADC_IN10
#define CS_PORT AFIOC
#define CS_PIN  GPIO_Pin_3
#define CS_ADC_CHANNEL ADC_Channel_10
#define CS_ADC_SCAN_CHANNEL ADC_ScanChannel_2

typedef struct {
    uint16_t adc1_ntc0;
    uint16_t adc2_ntc1; 
	uint16_t adc3_csVol;
    uint32_t update_tick;
} adc_ctrl_t;

adc_ctrl_t gAdcCtrl;

void Adc_Init(void)
{
    //config gpio
    GPIO_AnalogRemapConfig(NTC0_PORT, NTC0_PIN, ENABLE); //模拟复用功能PA1复用为ADC1通道
    GPIO_AnalogRemapConfig(NTC1_PORT, NTC1_PIN, ENABLE); //模拟复用功能PA2复用为ADC0通道
    GPIO_AnalogRemapConfig(CS_PORT, CS_PIN, ENABLE);     //模拟复用功能PC3复用为ADC10通道
	
    //config mode
    ADC_InitTypeDef adcCfg;
    ADC_StructInit(&adcCfg);
    adcCfg.ADC_Prescaler = 255;
    adcCfg.ADC_Mode = ADC_Mode_Continuous;
    adcCfg.ADC_TriggerSource = ADC_TriggerSource_Software;
    adcCfg.ADC_TimerTriggerSource = ADC_TimerTriggerSource_TIM3ADC; //定时源触发选择TIM1事件
    adcCfg.ADC_Align = ADC_Align_Left; //左对齐
    adcCfg.ADC_Channel = NTC0_ADC_CHANNEL;
    adcCfg.ADC_ReferencePositive = ADC_ReferencePositive_VDD; //选择VDDA作为正端参考电平
    adcCfg.ADC_BGVoltage = ADC_BGVoltage_BG1v2;
    ADC_Init(ADC, &adcCfg);
    
    ADC_AverageCmd(ADC, ENABLE);
    ADC_AverageTimesConfig(ADC, ADC_AverageTimes_128);
    
    ADC_SampleTimeConfig(ADC, 254); //配置采样时间
    
    //order
    ADC_ScanChannelConfig(ADC, NTC0_ADC_CHANNEL, 0);
    ADC_ScanChannelConfig(ADC, NTC1_ADC_CHANNEL, 1);
	ADC_ScanChannelConfig(ADC, CS_ADC_CHANNEL, 2);
    ADC_ScanChannelNumberConfig(ADC, 3);
    ADC_ScanCmd(ADC, ENABLE);
    
    NVIC_InitTypeDef nvicCfg;
    nvicCfg.NVIC_IRQChannel = ADC_IRQn; //设置中断向量号
    nvicCfg.NVIC_IRQChannelCmd = ENABLE; //设置是否使能中断
    nvicCfg.NVIC_IRQChannelPriority = 2; //设置中断优先级
    NVIC_Init(&nvicCfg);
    ADC_ITConfig(ADC, ADC_IT_EOS, ENABLE);
    
    ADC_Cmd(ADC, ENABLE); //启动ADC外设功能
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY)); //等待ADC启动完成
    
    ADC_StartOfConversion(ADC); //启动转换
}

//中断处理函数
void ADC_Handler(void) {
    if (ADC_GetFlagStatus(ADC, ADC_FLAG_EOS)) {
        gAdcCtrl.adc1_ntc0 = ADC_GetScanData(ADC, NTC0_ADC_SCAN_CHANNEL) >> 3;
        gAdcCtrl.adc2_ntc1 = ADC_GetScanData(ADC, NTC1_ADC_SCAN_CHANNEL) >> 3;
		gAdcCtrl.adc3_csVol = ADC_GetScanData(ADC, CS_ADC_SCAN_CHANNEL) >> 3;
        gAdcCtrl.update_tick = Tick_Get();
       // DBG_LN("tick: %d, adc1: %d, adc2: %d", gAdcCtrl.update_tick, gAdcCtrl.adc1_ntc0, gAdcCtrl.adc2_ntc1);
    }
} 

uint32_t Adc_NtcGet(uint16_t *ad1, uint16_t *ad2) {
    *ad1 = gAdcCtrl.adc1_ntc0;
    *ad2 = gAdcCtrl.adc2_ntc1;
//	uint16_t r1 = AD_TO_RES(gAdcCtrl.adc1);
//	uint16_t r2 = AD_TO_RES(gAdcCtrl.adc2);
	//DBG_LN("ad1 = %d,ad2 = %d",gAdcCtrl.adc1_ntc0,gAdcCtrl.adc2_ntc1);
    return gAdcCtrl.update_tick;
}

uint32_t Adc_csCurrentGet(uint32_t *current) {
    // 电压：mV
    uint32_t voltage_mV = (uint32_t)gAdcCtrl.adc3_csVol * 5000 / 4095;
    // 电流：mA，系数 3.7 = 37/10
    uint32_t current_mA = voltage_mV * 37 / 10; 

    *current = current_mA;

    return gAdcCtrl.update_tick;
}
