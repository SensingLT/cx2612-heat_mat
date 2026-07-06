#include "exti.h"
#include "PT32Y003x.h"
#include "uart.h"

// 硬件定义
#define SIF_EXTI_PORT GPIOC
#define SIF_EXTI_PIN  GPIO_Pin_6
#define SIF_TIM       TIM3

// 配置：48M分频4800 → 1tick=0.1ms
#define TIM_PSC        4800 - 1
// 毛刺过滤阈值：小于50(5ms)直接忽略
#define MIN_PULSE_TICK 50

// 全局计时
static volatile uint32_t g_tick = 0;

// 电平状态
static uint8_t  last_level = 0;
static uint32_t start_tick = 0;
static uint32_t low_time  = 0;
static uint32_t high_time = 0;
static uint8_t  wave_ready = 0; // 一组高低电平完成标志

// 定时器初始化：提供0.1ms计时基准
static void TIM3_Init(void)
{
    TIM_TimeBaseInitTypeDef tim_base;
    NVIC_InitTypeDef nvic_cfg;

    TIM_TimeBaseStructInit(&tim_base);
    tim_base.TIM_Prescaler = TIM_PSC;
    tim_base.TIM_AutoReload = 1; //0.1ms进一次中断
    tim_base.TIM_Direction = TIM_Direction_Up;
    TIM_TimeBaseInit(SIF_TIM, &tim_base);

    TIM_ITConfig(SIF_TIM, TIM_IT_ARI, ENABLE);

    nvic_cfg.NVIC_IRQChannel = TIM3_IRQn;
    nvic_cfg.NVIC_IRQChannelCmd = ENABLE;
    nvic_cfg.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&nvic_cfg);

    TIM_Cmd(SIF_TIM, ENABLE);
    g_tick = 0;
}

// EXTI双边沿中断初始化
void SIF_exti_Init(void)
{

    // GPIO上拉输入
    GPIO_InitTypeDef gpio_cfg;
    GPIO_StructInit(&gpio_cfg);
    gpio_cfg.GPIO_Pin = SIF_EXTI_PIN;
    gpio_cfg.GPIO_Mode = GPIO_Mode_In;
    gpio_cfg.GPIO_Pull = GPIO_Pull_Up;
    GPIO_Init(SIF_EXTI_PORT, &gpio_cfg);

    // EXTI双边沿触发
    NVIC_InitTypeDef nvicCfg;
    nvicCfg.NVIC_IRQChannel = EXTIC_IRQn;
    nvicCfg.NVIC_IRQChannelCmd = ENABLE;
    nvicCfg.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&nvicCfg);

    EXTI_TriggerTypeConfig(EXTIC, EXTI_Pin_6, EXTI_Trigger_RisingFalling);
    EXTI_ITConfig(EXTIC, EXTI_IT_6, ENABLE);

    // 启动计时定时器
    TIM3_Init();

    // 初始状态
    last_level = GPIO_ReadDataBit(SIF_EXTI_PORT, SIF_EXTI_PIN);
    start_tick = g_tick;
    DBG_LN("[EXTI] Init Finish");
}

// 计算tick差值，自动处理溢出
static uint32_t Tick_Diff(uint32_t now, uint32_t base)
{
    if(now >= base)
        return now - base;
    else
        return (0xFFFFFFFFU - base) + now;
}

// EXTI中断：只计算电平时长，不解析协议
void EXTIC_Handler(void)
{
    if(EXTI_GetITStatus(EXTIC, EXTI_IT_6) != RESET)
    {
		DBG_LN("exti gtick : %d",g_tick);
        EXTI_ClearFlag(EXTIC, EXTI_IT_6);

        uint8_t curr_level = GPIO_ReadDataBit(SIF_EXTI_PORT, SIF_EXTI_PIN);
        uint32_t now = g_tick;
        uint32_t dur = Tick_Diff(now, start_tick);
		//DBG_LN("dur : %d",dur);
        // 过滤毛刺
        if(dur < MIN_PULSE_TICK)
        {
//            start_tick = now;
            last_level = curr_level;
            return;
        }

        if(last_level == 0)
        {
            // 上升沿：上一段是低电平
            low_time = dur;
        }
        else
        {
            // 下降沿：上一段是高电平，一组波形采集完成
            high_time = dur;
            wave_ready = 1;
        }

        last_level = curr_level;
        start_tick = now;
    }
}

// TIM3计时中断，1ms进一次，每次+10（单位0.1ms）
void TIM3_Handler(void)
{
    if(TIM_GetFlagStatus(SIF_TIM, TIM_FLAG_ARF) != RESET)
    {
        TIM_ClearFlag(SIF_TIM, TIM_FLAG_ARF);
        g_tick += 10;
    }
}

// 主循环调用：仅打印高低电平时间
void SIF_EXTI_Task(void)
{
    if(wave_ready)
    {
        // low_time / high_time 单位：0.1ms，除以100得到ms
        DBG_LN("low_len:%d, high_len:%d", low_time / 100, high_time / 100);
        wave_ready = 0;
    }
}