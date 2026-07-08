#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>
#include <stdbool.h>

bool Protocol_HandleMsg(const uint8_t* pMsg, uint16_t length);

#endif //__PROTOCOL_H__
