#include "adc.h"
#include "PT32Y003x.h"
#include "tick.h"
#include "uart.h"

typedef struct {
    uint16_t adc1;
    uint16_t adc2;
    uint32_t update_tick;
} adc_ctrl_t;

adc_ctrl_t gAdcCtrl;

void Adc_Init(void)
{
    //config gpio
    GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_5, AFIO_AF_0, DISABLE);
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_5, ENABLE); //PC5复用为ADC9通道
    
    GPIO_DigitalRemapConfig(AFIOC, GPIO_Pin_3, AFIO_AF_0, DISABLE);
    GPIO_AnalogRemapConfig(AFIOC, GPIO_Pin_3, ENABLE); //PC3复用为ADC10通道
    
    //config mode
    ADC_InitTypeDef adcCfg;
    ADC_StructInit(&adcCfg);
    adcCfg.ADC_Prescaler = 255;
    adcCfg.ADC_Mode = ADC_Mode_Continuous;
    adcCfg.ADC_TriggerSource = ADC_TriggerSource_Software;
    adcCfg.ADC_TimerTriggerSource = ADC_TimerTriggerSource_TIM1ADC; //定时源触发选择TIM0事件
    adcCfg.ADC_Align = ADC_Align_Left; //左对齐
    adcCfg.ADC_Channel = ADC_Channel_10; //通道10, PC3
    adcCfg.ADC_ReferencePositive = ADC_ReferencePositive_VDD; //选择VDDA作为正端参考电平
    adcCfg.ADC_BGVoltage = ADC_BGVoltage_BG1v2;
    ADC_Init(ADC, &adcCfg);
    
    ADC_AverageCmd(ADC, ENABLE);
    ADC_AverageTimesConfig(ADC, ADC_AverageTimes_128);
    
    ADC_SampleTimeConfig(ADC, 254); //配置采样时间
    
    //order
    ADC_ScanChannelConfig(ADC, ADC_Channel_10, 0);
    ADC_ScanChannelConfig(ADC, ADC_Channel_9, 1);
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
        gAdcCtrl.adc1 = ADC_GetScanData(ADC, ADC_ScanChannel_0) >> 3;
        gAdcCtrl.adc2 = ADC_GetScanData(ADC, ADC_ScanChannel_1) >> 3;
        gAdcCtrl.update_tick = Tick_Get();
        //Uart_SendStrLn("tick: %d, adc1: %d, adc2: %d", gAdcCtrl.update_tick, gAdcCtrl.adc1, gAdcCtrl.adc2);
        //Uart_SendStrLn(">: %d K -  %d mV, %d K - %d mV", 100 * adc1 / (4096-adc1), 5080 *adc1/4096, 100 * adc2 / (4096-adc2), 5080 *adc2/4096);
    }
}

uint32_t Adc_Get(uint16_t *ad1, uint16_t *ad2) {
    *ad1 = gAdcCtrl.adc1;
    *ad2 = gAdcCtrl.adc2;
    return gAdcCtrl.update_tick;
}
