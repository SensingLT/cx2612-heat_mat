#include "heat.h"
#include "PT32Y003x.h"
#include "uart.h"
#include "adc.h"
#include "ntc.h"
#include "protection.h"

#define HEATER_CHANNEL_COUNT 2

#define HEAT0_PORT AFIOC
#define HEAT0_PIN GPIO_Pin_7
#define HEAT0_AF AFIO_AF_2
#define HEAT0_PWM_CHANNEL PWM_Channel_2

#define HEAT1_PORT AFIOB
#define HEAT1_PIN GPIO_Pin_4
#define HEAT1_AF AFIO_AF_2
#define HEAT1_PWM_CHANNEL PWM_Channel_3

void Heat_GPIOInit(void)
{
	GPIO_DigitalRemapConfig(HEAT0_PORT, HEAT0_PIN, HEAT0_AF,ENABLE);//CH2
	GPIO_AnalogRemapConfig(HEAT0_PORT,HEAT0_PIN,DISABLE);
	GPIO_DigitalRemapConfig(HEAT1_PORT, HEAT1_PIN, HEAT1_AF,ENABLE);//CH3N
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
//	//CH3N输出
	OutInit.PWM_Channel =HEAT1_PWM_CHANNEL;
	OutInit.PWM_OCNOutput = PWM_OCNOutput_Enable;		
	OutInit.PWM_OCOutput = PWM_OCOutput_Disable; 
	OutInit.PWM_OCIdleState = PWM_OCIdleState_High;
	OutInit.PWM_OCNIdleState = PWM_OCNIdleState_High;
	/* 配置PWM输出的占空比 P = (PWM_OCValue+1) / (PWM_AutoReloadValue+1)*/	
	OutInit.PWM_OCValue = 0 ;
    /* 配置PWM比较输出极性*/	
	OutInit.PWM_OCPolarity = PWM_OCPolarity_Low;
    OutInit.PWM_OCNPolarity = PWM_OCNPolarity_Low;
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
static pid_t g_pid[HEATER_CHANNEL_COUNT];

typedef struct {
	int16_t current_temp; // 当前温度 ×0.1℃
	int16_t target_temp;  //目标温度 ×0.1℃
} heat_t;
static heat_t g_heat_status[HEATER_CHANNEL_COUNT] = {0};

#define NTC_FAULT_TIMEOUT_TICK     24000       // 单位 1 tick = 5ms，1min超时
#define ADC_FAULT_VALUE      4095       // 满量程视为开路
#define CURRENT_OPEN_THRESHOLD_MA     50    
#define CURRENT_SHORT_THRESHOLD_MA   3500   

/* 故障检测状态 */
typedef struct {
    uint16_t last_adc;          
    uint32_t last_change_tick;  
    uint8_t  ntc_fault;         
    uint8_t  current_fault;     
    uint8_t  current_fail_cnt;
	uint8_t  temp_fault;//过温异常
} fault_t;
static fault_t g_fault[HEATER_CHANNEL_COUNT] = {0};

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
 * @brief  设置目标温度
 */
void Heat_SetTargetTemp(uint8_t channel, int16_t temp_x10)
{
    if (channel >= HEATER_CHANNEL_COUNT) return;
	
  //  Protection_swCurrentCH(channel);

    g_heat_status[channel].target_temp = temp_x10;
    g_pid[channel].integral = 0.0f;
    g_pid[channel].prev_error = 0.0f;
    g_pid[channel].err_filter = 0.0f;
	
    // 清除所有故障标志和计数器
    g_fault[channel].ntc_fault = 0;
    g_fault[channel].current_fault = 0;
    g_fault[channel].current_fail_cnt = 0;
    g_fault[channel].temp_fault = 0;
    g_fault[channel].last_adc = 0;
    g_fault[channel].last_change_tick = 0;
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
        TIM1->OCR3 = 0; 
}

static void Heat_PIDProcess(uint8_t channel)
{
    float error, p_term, i_term, d_term, output;
    uint16_t duty;
    pid_t *pid = &g_pid[channel];
    heat_t *heat = &g_heat_status[channel];

    // 过温保护（使用原始当前温度判断）
    if (heat->current_temp > heat->target_temp + 15)
    {
        output = 0.0f;
        pid->integral = 0.0f;
        pid->prev_error = 0.0f;
        goto set_output;
    }

    // 通道1补偿温度5度
    int16_t pid_current_temp = heat->current_temp;
    if (channel == 1 && heat->current_temp >= 430) // 40.0℃
    {
        pid_current_temp += 50; // 加5.0℃
    }

    // 原始温差
    error = (float)(heat->target_temp - pid_current_temp);

    if (error <= 0) {
        output = 5.0f;
    }
    else if (error < 50.0f) {
        float scale = error / 60.0f;
        output = 100.0f * scale;
    }
    else {
        p_term = pid->kp * error;
        float temp_i = pid->ki * pid->integral;
        float temp_d = pid->kd * (error - pid->prev_error);
        float temp_total = p_term + temp_i + temp_d;

        if (temp_total < pid->output_max && temp_total > pid->output_min)
        {
            pid->integral += error;
        }
        if (pid->integral > pid->integral_max)
            pid->integral = pid->integral_max;
        if (pid->integral < pid->integral_min)
            pid->integral = pid->integral_min;

        i_term = pid->ki * pid->integral;
        d_term = pid->kd * (error - pid->prev_error);
        pid->prev_error = error;

        output = p_term + i_term + d_term;
    }

set_output:
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    duty = (uint16_t)(output * 10000 / 100.0f);
    if (channel == 0)
        TIM1->OCR2 = duty;
    else
        TIM1->OCR3 = duty;
}

static inline void Heater_SetPwm(uint8_t ch, uint16_t duty) {
    if (ch == 0) TIM1->OCR2 = duty;
    else         TIM1->OCR3 = duty;
}

void Heat_ControlTask(void) {
    uint16_t adc0, adc1;
    uint32_t now = Tick_Get(), curr_ma;

    Adc_NtcGet(&adc0, &adc1);
    g_heat_status[0].current_temp = NTC_AdcToTemp(adc0);
    g_heat_status[1].current_temp = NTC_AdcToTemp(adc1);
    for (uint8_t ch = 0; ch < HEATER_CHANNEL_COUNT; ch++) {
        fault_t *f = &g_fault[ch];
        heat_t *h = &g_heat_status[ch];
        uint16_t ntc_adc = (ch == 0) ? adc0 : adc1;

        // ========== NTC 故障检测与恢复 ==========
		if (ntc_adc == ADC_FAULT_VALUE) {
			f->ntc_fault = 1;
		} else if (h->target_temp > 0) {
			int16_t diff = (int16_t)ntc_adc - (int16_t)f->last_adc;
			if (diff < 0) diff = -diff;
			if (diff > 5) {
				// 有效变化
				f->last_adc = ntc_adc;
				f->last_change_tick = now;
				if (f->ntc_fault) {
					f->ntc_fault = 0;
					DBG_LN("NTC%d recovered", ch);
				}
			} else {
				// 无效变化
				if ((now - f->last_change_tick) >= NTC_FAULT_TIMEOUT_TICK) {
					if (!f->ntc_fault) {
						DBG_LN("NTC%d stuck fault", ch);
					}
					f->ntc_fault = 1;
				}
			}
		} else {
			f->last_adc = ntc_adc;
			f->last_change_tick = now;
			f->ntc_fault = 0;
		}

        // ========== 过温保护与恢复 ==========
        if (!f->ntc_fault && h->current_temp >= 650) {
            f->temp_fault = 1;
        } else if (f->temp_fault && h->current_temp < 550) {
            f->temp_fault = 0;
            DBG_LN("Over-temp recovered CH%d", ch);
        }

		// ========== 电流检测与恢复 ==========
		if (h->target_temp > 0) {
			Protection_swCurrentCH(ch);
			Adc_csCurrentGet(&curr_ma);
			// 特殊值跳过异常判断，ADC在读取负压或者接近于0的电压时，数值会跳到8100多
			if (curr_ma > 18500) {
				if (f->current_fail_cnt > 0) f->current_fail_cnt--;
				if (f->current_fault && f->current_fail_cnt == 0) {
					f->current_fault = 0;
					DBG_LN("Current fault recovered CH%d", ch);
				}
			}
			else if (curr_ma < CURRENT_OPEN_THRESHOLD_MA || curr_ma > CURRENT_SHORT_THRESHOLD_MA) {
				// 异常：累计计数
				if (f->current_fail_cnt < 50) f->current_fail_cnt++;
				if (f->current_fail_cnt >= 10 && !f->current_fault) {
					DBG_LN("CH%d FAULT: current abnormal %lu mA", ch, curr_ma);
					f->current_fault = 1;
				}
			} else {
				// 正常：递减计数
				if (f->current_fail_cnt > 0) f->current_fail_cnt--;
				if (f->current_fault && f->current_fail_cnt == 0) {
					f->current_fault = 0;
					DBG_LN("Current fault recovered CH%d", ch);
				}
			}
		} else {
			f->current_fail_cnt = 0;
			f->current_fault = 0;
		}
        // ========== PID 控制 ==========
        if (h->target_temp > 0 && !f->ntc_fault && !f->current_fault && !f->temp_fault) {
            DBG_LN("current t%d = %d , target t%d = %d", ch, h->current_temp, ch, h->target_temp);
            Heat_PIDProcess(ch);
        } else {
		//	DBG_LN("ERROR");
            Heater_SetPwm(ch, 0);
        }
    }
}