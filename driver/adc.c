#include "adc.h"
#include "PT32Y003x.h"
#include "tick.h"
#include "uart.h"

#define AD_TO_RES(ad) (10 * (ad) / (4096 - ad))

#define NTC0_PORT AFIOD
#define NTC0_PIN GPIO_Pin_2
#define NTC0_ADC_CHANNEL ADC_Channel_6
#define NTC0_ADC_SCAN_CHANNEL ADC_ScanChannel_0

#define NTC1_PORT AFIOD
#define NTC1_PIN GPIO_Pin_3
#define NTC1_ADC_CHANNEL ADC_Channel_5
#define NTC1_ADC_SCAN_CHANNEL ADC_ScanChannel_1

typedef struct {
    uint16_t adc1;
    uint16_t adc2; 
    uint32_t update_tick;
} adc_ctrl_t;

adc_ctrl_t gAdcCtrl;

void Adc_Init(void)
{
    //config gpio
    GPIO_AnalogRemapConfig(NTC0_PORT, NTC0_PIN, ENABLE); //PD2复用为ADC6通道
    GPIO_AnalogRemapConfig(NTC1_PORT, NTC1_PIN, ENABLE); //PD3复用为ADC5通道
    
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
    ADC_ScanChannelNumberConfig(ADC, 2);
    ADC_ScanCmd(ADC, ENABLE);
    
    NVIC_InitTypeDef nvicCfg;
    nvicCfg.NVIC_IRQChannel = ADC_IRQn; //设置中断向量号
    nvicCfg.NVIC_IRQChannelCmd = ENABLE; //设置是否使能中断
    nvicCfg.NVIC_IRQChannelPriority = 3; //设置中断优先级
    NVIC_Init(&nvicCfg);
    ADC_ITConfig(ADC, ADC_IT_EOS, ENABLE);
    
    ADC_Cmd(ADC, ENABLE); //启动ADC外设功能
    while(!ADC_GetFlagStatus(ADC, ADC_FLAG_RDY)); //等待ADC启动完成
    
    ADC_StartOfConversion(ADC); //启动转换
}

//中断处理函数
void ADC_Handler(void) {
    if (ADC_GetFlagStatus(ADC, ADC_FLAG_EOS)) {
        gAdcCtrl.adc1 = ADC_GetScanData(ADC, NTC0_ADC_SCAN_CHANNEL) >> 3;
        gAdcCtrl.adc2 = ADC_GetScanData(ADC, NTC1_ADC_SCAN_CHANNEL) >> 3;
        gAdcCtrl.update_tick = Tick_Get();
       // DBG_LN("tick: %d, adc1: %d, adc2: %d", gAdcCtrl.update_tick, gAdcCtrl.adc1, gAdcCtrl.adc2);
        //Uart_SendStrLn(">: %d K -  %d mV, %d K - %d mV", 100 * adc1 / (4096-adc1), 5080 *adc1/4096, 100 * adc2 / (4096-adc2), 5080 *adc2/4096);
    }
} 

uint32_t Adc_Get(uint16_t *ad1, uint16_t *ad2) {
    *ad1 = gAdcCtrl.adc1;
    *ad2 = gAdcCtrl.adc2;
	uint16_t r1 = AD_TO_RES(gAdcCtrl.adc1);
	uint16_t r2 = AD_TO_RES(gAdcCtrl.adc2);
	//DBG_LN("r1 = %d,r2 = %d",r1,r2);
    return gAdcCtrl.update_tick;
}
