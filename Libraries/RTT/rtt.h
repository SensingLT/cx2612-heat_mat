#ifndef __INNER_DEFINES_H__
#define __INNER_DEFINES_H__

#ifndef __RTT_DEFINES_H__
#include "rtt_defines.h"
#endif

//大于0x0F不显示
typedef enum {
	MOD_CONSOLE = 0,
	MOD_APP = 1,
} module_t;

#ifndef DBG_WIN_ID
#define DBG_WIN_ID 0xFF
#endif

#if (USE_RTT <= 0)
	#define RTT_CTRL_TEXT_BRIGHT_GREEN ""
	#define RTT_CTRL_TEXT_BRIGHT_YELLOW ""
	#define RTT_CTRL_TEXT_BRIGHT_BLACK ""
	#define RTT_CTRL_TEXT_BRIGHT_CYAN ""
	#define RTT_CTRL_CLEAR ""
	#define rttPrintf(mod, ...)  printf(##__VA_ARGS__)
#endif //USE_RTT <= 0

//#undef DBG
//#define DBG(...)

#define FAILDBG(fmt, ...) DBG(MOD_FAIL, fmt, ##__VA_ARGS__)

//self define output window id
#define WINDBG(fmt, ...) DBG(DBG_WIN_ID, fmt, ##__VA_ARGS__)
#define WINDBG_RED(fmt, ...) DBG(DBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_RED fmt, ##__VA_ARGS__)
#define WINDBG_GREEN(fmt, ...) DBG(DBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_GREEN fmt, ##__VA_ARGS__)
#define WINDBG_YELLOW(fmt, ...) DBG(DBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_YELLOW fmt, ##__VA_ARGS__)
#define WINDBG_CYAN(fmt, ...) DBG(DBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_CYAN fmt, ##__VA_ARGS__)
#define WIN_CLEAR() PRT(DBG_WIN_ID, RTT_CTRL_CLEAR)
#define WIN_INT_VAR(var) WINDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", (var))
#define WIN_INT_FIELD(var, field) WINDBG(#var "." #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", var.field)
#define WIN_VAR(var, fmt) WINDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%" #fmt, (var))
#define WIN_PRINT_BUF(...) PRINT_BUF(DBG_WIN_ID, ##__VA_ARGS__)

#define EWINDBG(fmt, ...) DBG(EDBG_WIN_ID, fmt, ##__VA_ARGS__)
#define EWINDBG_RED(fmt, ...) DBG(EDBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_RED fmt, ##__VA_ARGS__)
#define EWINDBG_GREEN(fmt, ...) DBG(EDBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_GREEN fmt, ##__VA_ARGS__)
#define EWINDBG_YELLOW(fmt, ...) DBG(EDBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_YELLOW fmt, ##__VA_ARGS__)
#define EWINDBG_CYAN(fmt, ...) DBG(EDBG_WIN_ID, RTT_CTRL_TEXT_BRIGHT_CYAN fmt, ##__VA_ARGS__)
#define EWIN_CLEAR() PRT(EDBG_WIN_ID, RTT_CTRL_CLEAR)
#define EWIN_INT_VAR(var) EWINDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", (var))
#define EWIN_INT_FIELD(var, field) EWINDBG(#var "." #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", var.field)
#define EWIN_VAR(var, fmt) EWINDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%" #fmt, (var))
#define EWIN_PRINT_BUF(...) PRINT_BUF(EDBG_WIN_ID, ##__VA_ARGS__)

//console
#define CONDBG(fmt, ...) DBG(MOD_CONSOLE, fmt, ##__VA_ARGS__)
#define CONDBG_RED(fmt, ...) DBG(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_RED fmt, ##__VA_ARGS__)
#define CONDBG_GREEN(fmt, ...) DBG(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_GREEN fmt, ##__VA_ARGS__)
#define CONDBG_YELLOW(fmt, ...) DBG(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_YELLOW fmt, ##__VA_ARGS__)
#define CONDBG_CYAN(fmt, ...) DBG(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_CYAN fmt, ##__VA_ARGS__)
#define CON_INT_VAR(var) CONDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", (var))
#define CON_INT_FIELD(var, field) CONDBG(#var "." #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", var.field)
#define CON_INT_FIELD_P(var, field) CONDBG(#var "->" #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", var->field)
//#define CON_INT_FIELD_A(var, index) CONDBG(#var "[%d] = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", index, (var)[index])
#define CON_INT_FIELD_A(var, index) CONDBG(#var "[" #index "] = " RTT_CTRL_TEXT_BRIGHT_GREEN "%d", (var)[index])
#define CON_VAR(var, fmt) CONDBG(#var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%" #fmt, (var))
#define CON_FIELD(var, field, fmt) CONDBG(#var "." #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%" #fmt, var.field)
#define CON_FIELD_P(var, field, fmt) CONDBG(#var "->" #field " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%" #fmt, var->field)
#define CON_ADDR(var) CONDBG(RTT_CTRL_TEXT_BRIGHT_YELLOW "&" #var " = " RTT_CTRL_TEXT_BRIGHT_GREEN "%p", &(var))
#define CON_ARR_ITEM_ADDR(arr, index) CONDBG(RTT_CTRL_TEXT_BRIGHT_YELLOW "&" #arr "[%d] = " RTT_CTRL_TEXT_BRIGHT_GREEN "%p", index, &arr[index])
#define CON_LINE(str) DBG(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_BLACK "\n%s", str)
#define CON_PROMPT(ch) PRT(MOD_CONSOLE, RTT_CTRL_TEXT_BRIGHT_CYAN ">>%c ", ch)
#define CON_CLEAR() PRT(MOD_CONSOLE, RTT_CTRL_CLEAR)
#define CON_PRINT_BUF(...) PRINT_BUF(MOD_CONSOLE, ##__VA_ARGS__)


#endif //__INNER_DEFINES_H__