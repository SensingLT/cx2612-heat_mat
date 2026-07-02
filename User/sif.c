#include "sif.h"
#include "uart.h"
#include "PT32Y003x.h"

#define SIF_PORT AFIOC
#define SIF_PIN GPIO_Pin_5
#define SIF_AF AFIO_AF_2
#define SIF_PWM_CHANNEL PWM_Channel_2

// ===================== 时序定义（你给的标准） =====================
#define START_LOW        4000    // 起始低电平：40ms
#define START_HIGH       200     // 起始高电平：2ms
#define DATA1_LOW        200     // 数据1：低2ms
#define DATA1_HIGH       400     // 数据1：高4ms
#define DATA0_LOW        400     // 数据0：低4ms
#define DATA0_HIGH       200     // 数据0：高2ms
#define STOP_LOW         600     // 结束低电平：6ms
#define STOP_HIGH        200     // 结束高电平：2ms
#define BIT_NUM          8       // 8位/字节
#define ERR_RANGE        150     // 误差范围
// ==================================================================

static uint16_t last_cap = 0;
static uint8_t  level = 0;       // 0=低电平 1=高电平
static uint32_t cap_low = 0;
static uint32_t cap_high = 0;

static uint8_t  sif_rx_buf[8] = {0};
static uint8_t  sif_data = 0;
static uint8_t  bit_cnt = 0;
static uint8_t  byte_cnt = 0;
static uint8_t  frame_ready = 0;
static uint8_t  start_flag = 0;

// ===================== 初始化：双边沿捕获 =====================
void SIF_Init(void){
	GPIO_DigitalRemapConfig(SIF_PORT, SIF_PIN, SIF_AF,ENABLE);
	GPIO_AnalogRemapConfig(SIF_PORT,SIF_PIN,DISABLE);
	
    PWM_ICInitTypeDef icInit;
    PWM_ICStructInit(&icInit);

    icInit.PWM_Channel        = SIF_PWM_CHANNEL;
    icInit.PWM_ICSource       = PWM_ICSource_ICS2;
    icInit.PWM_ICRiseCapture  = PWM_ICRiseCapture_Enable;
    icInit.PWM_ICFallCapture  = PWM_ICFallCapture_Enable;
    icInit.PWM_ICResetCounter = PWM_ICResetCounter_Disable;
    PWM_ICInit(TIM1, &icInit);
	
	NVIC_InitTypeDef nvicCfg;
    nvicCfg.NVIC_IRQChannel = TIM1_IRQn;
    nvicCfg.NVIC_IRQChannelCmd = ENABLE;
    nvicCfg.NVIC_IRQChannelPriority = 3;
    NVIC_Init(&nvicCfg);
	
	PWM_ITConfig(TIM1, PWM_IT_IC2I, ENABLE);
	PWM_ITConfig(TIM1, PWM_IT_UI, ENABLE);
}

// 复位状态
static void SIF_ResetState(void)
{
    start_flag = 0;
    bit_cnt = 0;
    byte_cnt = 0;
    sif_data = 0;
}

// ===================== 中断解析 =====================
void TIM1_Handler(void)
{
    uint16_t curr_cap;
    uint32_t cap_diff;

    // 清除双边沿标志
    if(PWM_GetFlagStatus(TIM1, PWM_FLAG_IC2R)) PWM_ClearFlag(TIM1, PWM_FLAG_IC2R);
    if(PWM_GetFlagStatus(TIM1, PWM_FLAG_IC2F)) PWM_ClearFlag(TIM1, PWM_FLAG_IC2F);

    curr_cap = PWM_GetICxValue(TIM1, SIF_PWM_CHANNEL);
    // 计算电平时长，处理计数器溢出
    cap_diff = (curr_cap >= last_cap) ? (curr_cap - last_cap) : (10000 - last_cap + curr_cap);
    last_cap = curr_cap;

    // 电平状态机
    if(level == 0)  // 上升沿：低电平结束
    {
        cap_low = cap_diff;
		DBG_LN("cap_low : %d",cap_low / 100);
        level = 1;
    }
    else  // 下降沿：高电平结束，开始解析
    {
        cap_high = cap_diff;
		DBG_LN("cap_high : %d",cap_high /100);
        level = 0;

        // 1. 检测起始码
        if(!start_flag)
        {
            if((cap_low >= START_LOW-ERR_RANGE && cap_low <= START_LOW+ERR_RANGE) &&
               (cap_high >= START_HIGH-ERR_RANGE && cap_high <= START_HIGH+ERR_RANGE))
            {
                DBG_LN("[SIF] Start OK");
                SIF_ResetState();
                start_flag = 1;
            }
        }
        else
        {
            // 2. 检测结束码
            if((cap_low >= STOP_LOW-ERR_RANGE && cap_low <= STOP_LOW+ERR_RANGE) &&
               (cap_high >= STOP_HIGH-ERR_RANGE && cap_high <= STOP_HIGH+ERR_RANGE))
            {
                if(bit_cnt > 0) sif_rx_buf[byte_cnt++] = sif_data;
                frame_ready = 1;
                start_flag = 0;
                DBG_LN("[SIF] Stop OK");
                return;
            }

            // 3. 解析数据位（低 + 高 双校验）
            if(bit_cnt < BIT_NUM)
            {
                uint8_t bit_valid = 0;
                sif_data >>= 1;  // 低位先发

                // ===================== 数据 1 =====================
                if((cap_low >= DATA1_LOW-ERR_RANGE && cap_low <= DATA1_LOW+ERR_RANGE) &&
                   (cap_high >= DATA1_HIGH-ERR_RANGE && cap_high <= DATA1_HIGH+ERR_RANGE))
                {
                    sif_data |= 0x80;
                    DBG_LN("[SIF] Bit:1  low=%d  high=%d", cap_low/100, cap_high/100);
                    bit_valid = 1;
                }
                // ===================== 数据 0 =====================
                else if((cap_low >= DATA0_LOW-ERR_RANGE && cap_low <= DATA0_LOW+ERR_RANGE) &&
                        (cap_high >= DATA0_HIGH-ERR_RANGE && cap_high <= DATA0_HIGH+ERR_RANGE))
                {
                    sif_data &= 0x7F;
                    DBG_LN("[SIF] Bit:0  low=%d  high=%d", cap_low/100, cap_high/100);
                    bit_valid = 1;
                }
                // ===================== 无效时序 =====================
                else
                {
                    DBG_LN("[SIF] Invalid bit! low=%d high=%d", cap_low/100, cap_high/100);
                    // 干扰直接丢弃当前帧，增强稳定性
                    SIF_ResetState();
                    start_flag = 0;
                    return;
                }

                // 有效bit才计数
                if(bit_valid)
                {
                    bit_cnt++;
                    // 单字节接收完成
                    if(bit_cnt == BIT_NUM)
                    {
                        sif_rx_buf[byte_cnt++] = sif_data;
                        DBG_LN("[SIF] Byte:0x%02X", sif_data);
                        sif_data = 0;
                        bit_cnt = 0;
                    }
                }
            }
        }
    }
}

// ===================== 外部调用接口 =====================
uint8_t SIF_GetFrameReady(void)  { return frame_ready; }
uint8_t SIF_GetDataLen(void)     { return byte_cnt; }
uint8_t* SIF_GetDataBuf(void)    { return sif_rx_buf; }

void SIF_ClearFrameFlag(void)
{
    frame_ready = 0;
    byte_cnt = 0;
}

void SIF_Task(void)
{
    if(SIF_GetFrameReady())
    {
        uint8_t len = SIF_GetDataLen();
        uint8_t *buf = SIF_GetDataBuf();
        
        DBG_LN("[SIF] Frame Complete! Len:%d", len);
        for(uint8_t i=0; i<len; i++){
            DBG_LN("[SIF] Recv:0x%02X", buf[i]);
        }
        SIF_ClearFrameFlag();
    }
}