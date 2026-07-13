#include "sif.h"
#include "uart.h"
#include "PT32Y003x.h"

#define SIF_PORT AFIOC
#define SIF_PIN GPIO_Pin_6
#define SIF_AF AFIO_AF_2
#define SIF_PWM_CHANNEL PWM_Channel_1

// ===================== 初始化：双边沿捕获 =====================
void SIF_Init(void){
	GPIO_InitTypeDef gpio;

    /* 配置 TIM1_CH1 引脚为浮空输入 */
    gpio.GPIO_Pin  = SIF_PIN;
    gpio.GPIO_Mode = GPIO_Mode_In;      /* 输入模式 */
    gpio.GPIO_Pull = GPIO_Pull_NoPull;  /* 浮空 */
    GPIO_Init(GPIOC, &gpio);
	
	GPIO_DigitalRemapConfig(SIF_PORT, SIF_PIN, SIF_AF,ENABLE);
	GPIO_AnalogRemapConfig(SIF_PORT,SIF_PIN,DISABLE);
	
    PWM_ICInitTypeDef icInit;
    icInit.PWM_Channel        = SIF_PWM_CHANNEL;
    icInit.PWM_ICSource       = PWM_ICSource_ICS1;
    icInit.PWM_ICRiseCapture  = PWM_ICRiseCapture_Enable;
    icInit.PWM_ICFallCapture  = PWM_ICFallCapture_Enable;
    icInit.PWM_ICResetCounter = PWM_ICResetCounter_Enable;
    PWM_ICInit(TIM1, &icInit);
	

	PWM_ITConfig(TIM1,PWM_IT_IC1I,ENABLE);
	
	NVIC_InitTypeDef nvicCfg;
    nvicCfg.NVIC_IRQChannel = TIM1_IRQn;
    nvicCfg.NVIC_IRQChannelCmd = ENABLE;
    nvicCfg.NVIC_IRQChannelPriority = 0;
    NVIC_Init(&nvicCfg);
	
	/* 使能PWM */
	PWM_Cmd(TIM1,ENABLE);
}

typedef struct {
    uint16_t timestamp;   // 捕获的脉冲宽度
    uint8_t  edge;        // 0=下降沿, 1=上升沿
} CaptureEvent_t;

#define CAP_BUF_SIZE  128   
static volatile CaptureEvent_t cap_buf[CAP_BUF_SIZE];
static volatile uint8_t cap_head = 0;   
static volatile uint8_t cap_tail = 0;   

// ===================== 中断解析 =====================
void TIM1_Handler(void)
{
    uint16_t curr_cap;
    uint8_t edge_type = 0;
	
	uint32_t flags;
	
	flags = TIM1->SR;
    TIM1->SR = flags; /* 清除所有标志位 (写1清除) */
	
    if(flags & PWM_FLAG_IC1R) {
        edge_type = 1;   // 上升沿
    } 
	else if(flags & PWM_FLAG_IC1F) {
        edge_type = 0;   // 下降沿
    } 

    curr_cap  = (uint16_t)TIM1->ICR1;
	//DBG_LN("%d , %d\r\n",curr_cap,edge_type);
    uint8_t next_head = (cap_head + 1) % CAP_BUF_SIZE;
    if(next_head != cap_tail) {   
        cap_buf[cap_head].timestamp = curr_cap;
        cap_buf[cap_head].edge = edge_type;
        cap_head = next_head;
    }
}

#define SHORT_MAX      300    // 短脉冲：≤300
#define LONG_MIN       350    // 长脉冲：≥350

#define START_LOW_MIN  4000   // 起始低电平 ≥4000
#define START_HIGH_MIN 150    // 起始高电平 150~250
#define START_HIGH_MAX 250

#define END_LOW_MIN    450    // 结束低电平 450~550
#define END_LOW_MAX    550
#define END_HIGH_MIN   150    // 结束高电平 150~250
#define END_HIGH_MAX   250

/**
 * @brief 解析函数，适配发送端低位先行(bit0先发)
 */
uint8_t SIF_ParseFrame(uint8_t *out_buf, uint8_t *out_len)
{
    __disable_irq();
    uint8_t tail = cap_tail;
    uint8_t head = cap_head;
    __enable_irq();

    if(tail == head) return 0;

    // 状态机
    enum {
        WAIT_START_LOW,    // 等待起始上升沿
		WAIT_START_HIGH,// 等待起始下降沿
        RECV_BITS,     // 接收32位数据
        WAIT_STOP      // 等待结束信号
    } state = WAIT_START_LOW;

    uint8_t idx = tail;
    uint16_t pulse_width;
    uint8_t  edge;
    uint8_t  bit_cnt = 0;
    uint8_t  byte_cnt = 0;
    uint8_t  data_buf[4] = {0};

    while(idx != head)
    {
        pulse_width = cap_buf[idx].timestamp;
        edge = cap_buf[idx].edge;
        idx = (idx + 1) % CAP_BUF_SIZE;

        switch(state)
        {
            // ===================== 1. 匹配起始信号 =====================
            case WAIT_START_LOW:
                if(edge == 1 && pulse_width >= START_LOW_MIN) {
                    // 收到起始长低电平，等待起始高电平
                    state = WAIT_START_HIGH;
                }
				//printf("WAIT_START_HIGH\r\n");
                break;
            case WAIT_START_HIGH:
                if(edge == 0 && pulse_width >= START_HIGH_MIN && pulse_width <= START_HIGH_MAX) {
                    // 收到起始长低电平，等待起始高电平
                    state = RECV_BITS;
                    bit_cnt = 0;
                    byte_cnt = 0;
                    data_buf[0] = data_buf[1] = data_buf[2] = data_buf[3] = 0;
                }
				//printf("RECV_BITS\r\n");
                break;

            // ===================== 2. 接收数据位 =====================
            case RECV_BITS:
                if(edge == 0)  // 下降沿 = 一个完整位结束
                {
                    // ============== 先判断是不是结束信号 ==============
                    if(pulse_width >= END_LOW_MIN && pulse_width <= END_LOW_MAX)
                    {
                        if(byte_cnt == 4) {
                            state = WAIT_STOP;
						//	printf("STOP\r\n");
                            goto parse_done;
                        }
                        break;
                    }

                    // ============== 解析数据位 0/1 ==============
                    uint8_t bit;
                    if(pulse_width > LONG_MIN) {
                        bit = 1;  // 长脉冲=1
                    }
                    else if(pulse_width < SHORT_MAX) {
                        bit = 0;  // 短脉冲=0
                    }
                    else {
                        // 无效位，复位
                        state = WAIT_START_LOW;
					//	printf("RESET\r\n");
                        break;
                    }
                  data_buf[byte_cnt] = (data_buf[byte_cnt] >> 1) | (bit << 7);

                    bit_cnt++;

                    // 1字节=8位
                    if(bit_cnt == 8) {
                        byte_cnt++;
                        bit_cnt = 0;
                        if(byte_cnt >= 4) {
                            // 4字节收完，等待结束
                            state = WAIT_STOP;
                        }
                    }
                }
                break;

            case WAIT_STOP:
				//printf("STOP\r\n");
                goto parse_done;
        }
    }

parse_done:
    if(state == WAIT_STOP && byte_cnt == 4)
    {
        __disable_irq();
        cap_tail = idx;
        __enable_irq();

        out_buf[0] = data_buf[0];
        out_buf[1] = data_buf[1];
        out_buf[2] = data_buf[2];
        out_buf[3] = data_buf[3];
        *out_len = 4;
        return 1;
    }
    else {
        return 0;
    }
}

void SIF_Task(void)
{
    uint8_t data[4];
    uint8_t len;

     //原来的解析与打印（可保留或注释）
     if(SIF_ParseFrame(data, &len))
     {
         DBG_LN("[SIF] : %02X %02X %02X %02X", data[0], data[1], data[2], data[3]);
     }

//    // ---- 新增：打印 cap_buf 全部未消费事件 ----
//    __disable_irq();                     // 关中断保护
//    uint8_t tail = cap_tail;
//    uint8_t head = cap_head;
//    __enable_irq();

//    if (tail != head) {
//        DBG_LN("=== CapBuf Events (%d pending) ===", 
//               (head >= tail) ? (head - tail) : (CAP_BUF_SIZE - tail + head));
//        uint8_t idx = tail;
//        while (idx != head) {
//            DBG_LN("  [%u] ts=%u edge=%s", 
//                   idx,
//                   cap_buf[idx].timestamp,
//                   cap_buf[idx].edge ? "RISE" : "FALL");
//            idx = (idx + 1) % CAP_BUF_SIZE;
//        }
//    }
}