#ifndef __RTT_DEFINES_H__
#define __RTT_DEFINES_H__

/**
 * 说明，不要直接使用此头文件中的定义，具体使用方法参考inner_defines.h
 * 在自己的工程中创建inner_defines.h，然后按模块控制打印
 */
#include <stdint.h>
#include <stdbool.h>


//如果不支持RTT，将USE_RTT定义为0
#define USE_RTT 1

#if (USE_RTT > 0)
#include "SEGGER_RTT.h"
#endif

#if (__ARMCC_VERSION >= 6000000)
#define WEEK __attribute__((weak))
#else
#define WEEK __weak
#endif

//调试开关, 0: release 1: 部分调试（关闭中断调试） 2：完全调试
#define DEBUG_LEVEL 1

//定义打印内存最大字节数
#define PRINT_BUF_MAX_BYTE 200

#if (USE_RTT > 0)
#define BR "\n "
extern void rttPrintBuf(uint8_t channel, const void* pBuffer, uint16_t len);
int rttPrintf(unsigned modIndex, const char* fmt, ...);
#else
#define MAX_PRINT_LEVEL 0x0F
//#define BR "\n                    "
#define BR "\n\t"
extern void printBuf(const void* pBuffer, uint16_t len);
#endif

//这些宏每个模块重定义使用，尽量避免直接使用
#if (DEBUG_LEVEL >= 1)
#if (USE_RTT > 0)
#ifdef DBG_PRINT_OS_TICK
	extern uint32_t Tick_Get(); //from tick.c
	#define DBG(mod, fmt, ...) rttPrintf(mod, BR RTT_CTRL_TEXT_BRIGHT_BLACK "T%u.%s[L%d]." RTT_CTRL_TEXT_WHITE fmt, Tick_Get(), __func__, __LINE__, ##__VA_ARGS__)
	
	#define PRINT_BUF(mod, pBuf, len) ( \
		rttPrintf(mod, RTT_CTRL_TEXT_BRIGHT_BLACK "\n>T%d.%s[L%d].", Tick_Get(), __func__, __LINE__), \
		rttPrintBuf(mod, pBuf, len))
#else
	#define DBG(mod, fmt, ...) rttPrintf(mod, BR RTT_CTRL_TEXT_BRIGHT_BLACK "%s[L%d]." RTT_CTRL_TEXT_WHITE fmt, __func__, __LINE__, ##__VA_ARGS__)
	//#define DBG(mod, fmt, ...) rttPrintf(mod, BR "[L%d]." fmt, __LINE__, ##__VA_ARGS__)
	//#define DBG(mod, fmt, ...) rttPrintf(mod, BR fmt, ##__VA_ARGS__)
	
	#define PRINT_BUF(mod, pBuf, len) ( \
		rttPrintf(mod, RTT_CTRL_TEXT_BRIGHT_BLACK "\n>%s[L%d].", __func__, __LINE__), \
		rttPrintBuf(mod, pBuf, len))
#endif

	//纯换行打印
	#define PRT(mod, fmt, ...) rttPrintf(mod, BR fmt, ##__VA_ARGS__)
	
#else //USE_RTT
	#define DBG(lv, fmt, ...) do { \
		if (lv <= MAX_PRINT_LEVEL) { \
			printf(BR"%s[L%d]."fmt, __func__, __LINE__, ##__VA_ARGS__); \
		} \
	} while(0)
	
	#define PRT(lv, fmt, ...) do { \
		if (lv <= MAX_PRINT_LEVEL) { \
			printf(BR fmt, ##__VA_ARGS__); \
		} \
	} while(0)
	
	#define PRINT_BUF(lv, pBuf, len) do { \
		if (lv <= MAX_PRINT_LEVEL) { \
			printf("\n>%s[L%d].", __func__, __LINE__), printBuf(pBuf, len); \
		} \
	} while(0)
#endif //USE_RTT > 0
#endif //(DEBUG_LEVEL >= 1)

#if (DEBUG_LEVEL >= 2)
	//中断中使用，可分开开关
	#ifdef USE_RTT
		#define INTDBG(fmt, ...) DBG(0, "[INT]"fmt, ##__VA_ARGS__)
	#else
		#define INTDBG(fmt, ...) DBG(0, "[INT]"fmt, ##__VA_ARGS__)
	#endif
#endif

#ifndef DBG
#define DBG(...)
#endif

#ifndef PRT
#define PRT(...)
#endif

#ifndef PRINT_BUF
#define PRINT_BUF(...)
#endif

#ifndef INTDBG
#define INTDBG(...)
#endif


//计算数组元素个数
#define ARR_LEN(arr) (sizeof(arr)/sizeof(arr[0]))

#endif //__RTT_DEFINES_H__
