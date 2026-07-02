#include "rtt_defines.h"
#include <stdio.h>
#include "SEGGER_RTT.h"

int rttPrintf(unsigned modIndex, const char* fmt, ...) {
	//bufferIndex当成TerminalIndex来用
	if (modIndex > 0x0F) {
	  return 0;
	}
  
	static char aBuffer[128]; //根据应用需求调整大小
	va_list args;

	va_start(args, fmt);
	int n = vsnprintf(aBuffer, sizeof(aBuffer), fmt, args);
	SEGGER_RTT_TerminalOut(modIndex, aBuffer);
//	SEGGER_RTT_SetTerminal(modIndex);
//	if (n > (int)sizeof(aBuffer)) {
//		SEGGER_RTT_Write(0, aBuffer, sizeof(aBuffer));
//	} else if (n > 0) {
//		SEGGER_RTT_Write(0, aBuffer, n);
//	}
	va_end(args);
	return n;
}

void rttPrintBuf(uint8_t modIndex, const void* pBuffer, uint16_t len) {
	if (modIndex > 0x0F) {
		return;
	}
	//SEGGER_RTT_SetTerminal(modIndex);
	static char buf[256];
	int offset = 0;
	uint8_t* pBuf = (uint8_t*)pBuffer;
	int line = 1;
	
	offset += snprintf(buf + offset, sizeof(buf) - offset, RTT_CTRL_TEXT_BRIGHT_YELLOW "[buf addr: 0x%p, len: %d]" RTT_CTRL_TEXT_BRIGHT_WHITE, pBuf, len);
	if (len > PRINT_BUF_MAX_BYTE) {
		len = PRINT_BUF_MAX_BYTE;
	}
	for (int i = 0; i < len; i++) {
		int col = i % 8;
		if (col == 0) {
			offset += snprintf(buf + offset, sizeof(buf) - offset, "\n%2d>", line);
			line++;
		} else if (col == 4) {
			buf[offset++] = ',';
		} else if (col == 7) {
			if (offset > 200) {
				buf[offset] = 0;
				if (SEGGER_RTT_TerminalOut(modIndex, buf) <= 0) {
					return;
				}
				//printf("%s", buf);
				offset = 0;
			}
		}
		offset += snprintf(buf + offset, sizeof(buf) - offset, " %02X.%c", pBuf[i], pBuf[i] >= ' ' && pBuf[i] <= '~' ? pBuf[i] : ' ');
	}
	if (offset > 0) {
		buf[offset] = 0;
		if (SEGGER_RTT_TerminalOut(modIndex, buf) <= 0) {
			return;
		}
		//printf("%s", buf);
	}
}