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
	float integral_max; 
	float integral_min; 
    float err_filter;   // 误差低通滤波，抑制ADC抖动
} pid_t;

/* 两个通道的 PID */
static pid_t g_pid[HEATER_CHANNEL_COUNT];

typedef struct {
	int16_t current_temp; // 当前温度 ×0.1℃
	int16_t target_temp;  //目标温度 ×0.1℃
} heat_t;

static heat_t g_heat_status[HEATER_CHANNEL_COUNT] = {0};

/**
 * @brief  初始化 PID 参数
 */
void Heat_PIDInit(uint8_t channel, float kp, float ki, float kd)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_pid[channel].kp = kp;
    g_pid[channel].ki = ki;
    g_pid[channel].kd = kd;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
    g_pid[channel].output_max = 100.0f;
    g_pid[channel].output_min = 0.0f;
    // 放大积分限幅，防止轻易饱和
    g_pid[channel].integral_max = 200.0f;
    g_pid[channel].integral_min = -200.0f;
    g_pid[channel].err_filter = 0.0f;
}

/**
 * @brief  设置目标温度（×10）
 */
void Heat_SetTargetTemp(uint8_t channel, int16_t temp_x10)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_heat_status[channel].target_temp = temp_x10;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
    g_pid[channel].err_filter = 0.0f;
}

/**
 * @brief  停止加热
 */
void Heat_Stop(uint8_t channel)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;

    g_heat_status[channel].target_temp = 0;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
    g_pid[channel].err_filter = 0.0f;

    if (channel == 0)
        TIM1->OCR2 = 0;
    else
        TIM1->OCR4 = 0; 
}

static void Heat_PIDProcess(uint8_t channel)
{
    float error, p_term, i_term, d_term, output;
    uint16_t duty;
    pid_t *pid = &g_pid[channel];
    heat_t *heat = &g_heat_status[channel];

    // 原始温差：目标 - 当前
    float raw_err = (float)(heat->target_temp - heat->current_temp);
    // 一阶低通滤波
    pid->err_filter = pid->err_filter * 0.7f + raw_err * 0.3f;
    error = pid->err_filter;

    // ==================== 过温保护 ====================
    // 当前温度超过目标+1.5℃（15×0.1℃）立即关闭
    if (heat->current_temp > heat->target_temp + 15)
    {
        output = 0.0f;
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
    }
	
	else if(error <= 0){
		output = 5.0f;
	}
    // ==================== 正常控温 ====================
    else
    { 
        if (error < 20.0f)
		{
			float scale = error / 30.0f;
			output = 100.0f * scale;
		}
		// 温差 > 5.0℃：全速PID
		else
		{
			p_term = pid->kp * error;
			float temp_i = pid->ki * pid->integral;
			float temp_d = pid->kd * (error - pid->prev_error);
			float temp_total = p_term + temp_i + temp_d;

			if (temp_total < pid->output_max && temp_total > pid->output_min)
			{
				pid->integral += error;
			}
			// 积分限幅
			if (pid->integral > pid->integral_max)
				pid->integral = pid->integral_max;
			if (pid->integral < pid->integral_min)
				pid->integral = pid->integral_min;

			i_term = pid->ki * pid->integral;
			d_term = pid->kd * (error - pid->prev_error);
			pid->prev_error = error;

			output = p_term + i_term + d_term;
		}
    }

    // 输出限幅
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    // PWM占空比换算
    duty = (uint16_t)(output * 10000 / 100.0f);
    if (channel == 0)
        TIM1->OCR2 = duty;
    else
        TIM1->OCR4 = duty;
}

static uint8_t heating_flag = 0; //加热标志位

uint8_t Heat_IsHeating(void)
{
    return heating_flag;
}

/**
 * @brief  温控主处理函数
 */
void Heat_ControlTask(void)
{
    uint8_t ch;
	uint16_t adc0,adc1 = 0;
	Adc_NtcGet(&adc0,&adc1);
	g_heat_status[0].current_temp = NTC_AdcToTemp(adc0);
	g_heat_status[1].current_temp = NTC_AdcToTemp(adc1);

    for (ch = 0; ch < HEATER_CHANNEL_COUNT; ch++)
    {
        if (g_heat_status[ch].target_temp > 0)
        {
			DBG_LN("current t%d = %d , target t%d = %d",ch,g_heat_status[ch].current_temp,ch,g_heat_status[ch].target_temp);
            Heat_PIDProcess(ch);
        }
    }
	heating_flag = (TIM1->OCR2 > 0) || (TIM1->OCR4 > 0) ? 1 : 0;
}