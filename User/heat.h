#ifndef __HEAT_H__
#define __HEAT_H__

#include <stdbool.h>
#include <stdint.h>

#define HEAT_CHANNEL_0 0
#define HEAT_CHANNEL_1 1

void Heat_GPIOInit(void);
void Heat_PIDInit(uint8_t channel, float kp, float ki, float kd);
void Heat_SetTargetTemp(uint8_t channel, int16_t temp_x10);
void Heat_Stop(uint8_t channel);
void Heat_ControlTask(void);

#endif //__HEAT_H__
