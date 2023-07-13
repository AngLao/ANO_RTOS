#include "uwb_task.h"


uint32_t totalFrameCount = 0;
uint32_t errorFrameCount = 0;
/* uwb数据更新 */
void uwb_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  static uint8_t pData[128];
	static uint16_t RingBufferDataLen;
  //uwb串口初始化
  Drv_Uart1Init(3000000);
  debugOutput("uwb use uart1，rate:3000000");

  while (1) { 
		
		RingBufferDataLen = RingBuffer_GetCount(&U1rxring) ;
		
		for(uint16_t i=0; ;i++){
			//缓冲区中数据不够一帧的长度
			if(RingBufferDataLen-i<128)
				break;
			 
			//查找帧头
			RingBuffer_Pop(&U1rxring, &pData[0]);
			if(pData[0] == 0x55) {
				//找到帧头
				totalFrameCount++;
				//弹出剩余数据
				for(uint16_t cnt = 1; cnt <128 ; cnt++){ 
					RingBuffer_Pop(&U1rxring, &pData[cnt]);
				} 
				//解析数据
				if(!g_nlt_tagframe0.UnpackData(pData, RingBufferDataLen))  
					//传输有错
					errorFrameCount++; 
			}
		} 
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 200);
  }
} 