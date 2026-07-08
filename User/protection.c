#include "protection.h"
#include "PT32Y003x.h"
/*
 这部分主要用来检测加热芯片的状态，当处在异常状态下，进行调节和安全保护
*/
#define HEAT0_PORT AFIOC
#define HEAT0_PIN GPIO_Pin_5
#define HEAT0_AF AFIO_AF_2
#define HEAT0_PWM_CHANNEL PWM_Channel_2

#define HEAT1_PORT AFIOC
#define HEAT1_PIN GPIO_Pin_4
#define HEAT1_AF AFIO_AF_2
#define HEAT1_PWM_CHANNEL PWM_Channel_4