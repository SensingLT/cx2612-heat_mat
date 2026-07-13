#include <string.h>
#include "config.h"
#include "protocol.h"
#include "uart.h"
#include "crc.h"
#include "buildInfo.h"
#include "heat.h"


//获取命令参数，从1号开始
static int parseIntPara(const uint8_t* pMsg, uint16_t length, int paraId) {
	if (paraId < 1) {
		return 0;
	}
	for (int i = 0; i < length && pMsg[i] >= ' '; i++) {
		if (pMsg[i] == ',') {
			if (--paraId == 0) {
				return atoi((const char*)&pMsg[i + 1]);
			}
		}
	}
	return 0;
}
	


bool Protocol_HandleMsg(const uint8_t* pMsg, uint16_t length) {
//判断命令名
#define IS_CMD(cmd) (0 == strncmp((const char*)pMsg, cmd, sizeof(cmd) - 1))
	
	if (pMsg[0] <= 'z') {
		//首先处理文本命令
		if (IS_CMD("reboot")) {
			Uart_SendStrForCmd("\n\n ...To Reboot...\n\n");
			while(1); //重启
		} 
		else if (IS_CMD("set T0=")) {
			const char* pValue = (const char*)pMsg + strlen("set T0=");
			int t1_value = atoi(pValue);   // 提取整数
			
			if(t1_value < 0 || t1_value > 100){
				return false;
			}
			else if(t1_value == 0){
				Heat_Stop(0);
				return true;
			}
			//Uart_SendStrForCmd("T0 set to %d\n", t1_value);
			Heat_SetTargetTemp(0,t1_value * 10);
			return true;
		}
		else if (IS_CMD("set T1=")) {
			const char* pValue = (const char*)pMsg + strlen("set T1=");
			int t1_value = atoi(pValue);   // 提取整数
			
			if(t1_value < 0 || t1_value > 100){
				return false;
			}
			else if(t1_value == 0){
				Heat_Stop(1);
				return true;
			}
			//Uart_SendStrForCmd("T1 set to %d\n", t1_value);
			Heat_SetTargetTemp(1,t1_value * 10);
			return true;
		}
	}

//	//二进制消息处理
//	if (length < sizeof(msg_header_t)) {
//		DBG_LN("msg len %d is too short", length);
//		return false;
//	}

//	const msg_header_t* pHeader = (const msg_header_t*)pMsg;
//	if (length < pHeader->size) {
//		DBG_LN("length %d < size %d", length, pHeader->size); //debug mode才会自动生效
//		return false;
//	}
//	
//	if (!CHECK_CRC(pHeader)) {
//		DBG_LN("crc mismatch");
//		return false;
//	}
//	
//	if (pHeader->cmd == CMD_QUERY_SEAT_STATUS_REQ) {
//		reportOccupyStatus();
//		return true;
//	}
//	if (pHeader->cmd == CMD_CONFIG_SEAT_REQ) {
//		config_seat_req_t* configMsg = (config_seat_req_t*)pMsg;
//		if(configMsg->min_threshold == 0 &&  configMsg->max_threshold == 0){
//			configMsg->min_threshold = DEF_MIN_THRESHOLD;
//			configMsg->max_threshold = DEF_MAX_THRESHOLD;
//		}
//		judge_threshold_t newthreshold;
//		newthreshold.min_fuse_r = configMsg->min_threshold;
//		newthreshold.max_fuse_r = configMsg->max_threshold;
//		//写入新阈值
//		Threshold_Write(&newthreshold);
//		//pSeatStatus->pJudgeThreshold = Threshold_Read();
//		reportThreshold();
//		GPIO_ResetBits(GPIOD, GPIO_Pin_2);
//		return true;
//	}

//	if (pHeader->cmd == CMD_QUERY_VERSION_REQ) {
//		reportVersion();
//		return true;
//	}
   return false;
}
