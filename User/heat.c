#include "heat.h"
#include "PT32Y003x.h"
#include "uart.h"
#include "adc.h"
#include "ntc.h"

#define HEATER_CHANNEL_COUNT 2

#define HEAT0_PORT AFIOC
#define HEAT0_PIN GPIO_Pin_5
#define HEAT0_AF AFIO_AF_2
#define HEAT0_PWM_CHANNEL PWM_Channel_2

#define HEAT1_PORT AFIOC
#define HEAT1_PIN GPIO_Pin_4
#define HEAT1_AF AFIO_AF_2
#define HEAT1_PWM_CHANNEL PWM_Channel_4

void Heat_GPIOInit(void)
{
	GPIO_DigitalRemapConfig(HEAT0_PORT, HEAT0_PIN, HEAT0_AF,ENABLE);//CH2
	GPIO_AnalogRemapConfig(HEAT0_PORT,HEAT0_PIN,DISABLE);
	GPIO_DigitalRemapConfig(HEAT1_PORT, HEAT1_PIN, HEAT1_AF,ENABLE);//CH4
	GPIO_AnalogRemapConfig(HEAT1_PORT,HEAT1_PIN,DISABLE);

	PWM_TimeBaseInitTypeDef PWM_TimeBaseInitType;
	PWM_OCInitTypeDef OutInit;

	PWM_TimeBaseInitType.PWM_ClockSource = PWM_ClockSource_PCLK;
	PWM_TimeBaseInitType.PWM_CenterAlignedMode = PWM_CenterAlignedMode_Disable;
	PWM_TimeBaseInitType.PWM_Direction = PWM_Direction_Up;
	PWM_TimeBaseInitType.PWM_AutoReloadValue = 10000;//分频480，此时PWM一个周期为100ms
	/* 驱动CNT计数器的时钟 = Fcksys/(psc+1)*/
	PWM_TimeBaseInitType.PWM_Prescaler = 479;
	PWM_TimeBaseInit(TIM1,&PWM_TimeBaseInitType);

	OutInit.PWM_Channel =HEAT0_PWM_CHANNEL;
	/* 配置为PWM输出模式 */	
	OutInit.PWM_OCMode = TIM_OCMode_PWM1;
	OutInit.PWM_OCNOutput = PWM_OCNOutput_Disable;		
	OutInit.PWM_OCOutput = PWM_OCOutput_Enable;  
	/* 设置PWM空闲时候的输出电平状态 */	
	OutInit.PWM_OCIdleState = PWM_OCIdleState_Low;
	OutInit.PWM_OCNIdleState = PWM_OCNIdleState_Low;
	/* 配置PWM输出的占空比 P = (PWM_OCValue+1) / (PWM_AutoReloadValue+1)*/	
	OutInit.PWM_OCValue = 0 ;
    /* 配置PWM比较输出极性*/	
	OutInit.PWM_OCPolarity = PWM_OCPolarity_High;
    OutInit.PWM_OCNPolarity = PWM_OCNPolarity_High;
	PWM_OCInit(TIM1, &OutInit);
	
	OutInit.PWM_Channel =HEAT1_PWM_CHANNEL;
	PWM_OCInit(TIM1, &OutInit);
}


/* PID 参数结构体 */
typedef struct {
    float kp;           // 比例系数
    float ki;           // 积分系数
    float kd;           // 微分系数
    float integral;     // 积分累积值
    float prev_error;   // 上一次误差
    float output_max;   // 输出上限（对应最大占空比 100）
    float output_min;   // 输出下限（通常 0）
} pid_t;

/* 两个通道的 PID */
static pid_t g_pid[HEATER_CHANNEL_COUNT];

typedef struct {
	int16_t current_temp; // 当前温度
	int16_t target_temp;  //目标温度
} heat_t;

static heat_t g_heat_status[HEATER_CHANNEL_COUNT] = {0};
/**
 * @brief  初始化 PID 参数
 * @param  channel  通道号（0 或 1）
 * @param  kp       比例系数
 * @param  ki       积分系数
 * @param  kd       微分系数
 */
void Heat_PIDInit(uint8_t channel, float kp, float ki, float kd)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_pid[channel].kp = kp;
    g_pid[channel].ki = ki;
    g_pid[channel].kd = kd;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
    g_pid[channel].output_max = 100.0f;   // 占空比最大 100%
    g_pid[channel].output_min = 0.0f;
}

/**
 * @brief  设置目标温度（×10）
 * @param  channel     通道号
 * @param  temp_x10    目标温度 ×10，例如 250 表示 25.0°C
 */
void Heat_SetTargetTemp(uint8_t channel, int16_t temp_x10)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_heat_status[channel].target_temp = temp_x10;

    /* 重置 PID 积分，防止突变 */
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
}


/**
 * @brief  停止加热
 * @param  channel     通道号
 */
void Heat_Stop(uint8_t channel)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_heat_status[channel].target_temp = 0;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;

    /* 直接关闭 PWM 输出 */
    if (channel == 0)
        TIM1->OCR2 = 0;
    else
        TIM1->OCR4 = 0; 
}

static uint8_t cnt1 = 0;
/**
 * @brief  执行一次 PID 计算，更新 PWM 占空比
 * @param  channel  通道号
 */
static void Heat_PIDProcess(uint8_t channel)
{
    float error, p_term, i_term, d_term, output;
    uint16_t duty;

    if (channel >= HEATER_CHANNEL_COUNT) return;

    /* 计算误差（目标 - 当前） */
    error = (float)(g_heat_status[channel].target_temp - g_heat_status[channel].current_temp);

    /* 比例项 */
    p_term = g_pid[channel].kp * error;

    /* 积分项（防积分饱和） */
    g_pid[channel].integral += error;
    if (g_pid[channel].integral > 1000.0f)  g_pid[channel].integral = 1000.0f;
    if (g_pid[channel].integral < -1000.0f) g_pid[channel].integral = -1000.0f;
    i_term = g_pid[channel].ki * g_pid[channel].integral;

    /* 微分项 */
    d_term = g_pid[channel].kd * (error - g_pid[channel].prev_error);
    g_pid[channel].prev_error = error;

    /* 总输出 */
    output = p_term + i_term + d_term;

    /* 输出限幅 */
    if (output > g_pid[channel].output_max) output = g_pid[channel].output_max;
    if (output < g_pid[channel].output_min) output = g_pid[channel].output_min;

    /* 转换为 PWM 占空比（0 ~ ARR-1） */
    duty = (uint16_t)(output * (10000 - 1) / 100.0f);

    /* 设置 PWM 占空比 */
    if (channel == 0)
        TIM1->OCR2 = duty;
    else
        TIM1->OCR4 = duty;
}

//static uint8_t cnt2 = 0;
/**
 * @brief  温控主处理函数
 */
void Heat_ControlTask(void)
{
    uint8_t ch;
	uint16_t adc0,adc1 = 0;
	Adc_Get(&adc0,&adc1);
	//DBG_LN("adc0 =  %d,adc1 = %d",adc0,adc1);
    /* 读取当前温度（×10） */
	g_heat_status[0].current_temp = NTC_AdcToTemp(adc0);
	g_heat_status[1].current_temp = NTC_AdcToTemp(adc1);
	//DBG_LN("current temp %d = %d , target temp %d = %d",ch,g_heat_status[ch].current_temp,ch,g_heat_status[ch].target_temp);

    for (ch = 0; ch < HEATER_CHANNEL_COUNT; ch++)
    {
	
        if (g_heat_status[ch].target_temp > 0)
        {
			DBG_LN("current t%d = %d , target t%d = %d",ch,g_heat_status[ch].current_temp,ch,g_heat_status[ch].target_temp);
            Heat_PIDProcess(ch);
        }
    }
}