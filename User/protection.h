#ifndef __PROTECTION_H__
#define __PROTECTION_H__

#include <stdbool.h>
#include <stdint.h>
void Protection_init(void);
void Protection_swCurrentCH(uint8_t ch);
void Protection_Task(void);

#endif //__PROTECTION_H__