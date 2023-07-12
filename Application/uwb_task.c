#include "uwb_task.h"

 
/* uwb数据更新 */
void uwb_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  static unsigned char pFrame[128];

	//uwb串口初始化
  Drv_Uart1Init(1500000); 
	debugOutput("uwb串口1初始化，波特率1500000");
	
  while (1) {
    //读环形缓冲区
    if(RingBuffer_GetCount(&U1rxring) > 128) {

      memset(pFrame,0,128);
      RingBuffer_PopMult(&U1rxring, pFrame, 128);

      //解析uwb数据
      uint8_t res = g_nlt_tagframe0.UnpackData(pFrame, 128); 
      //接收有误刷新缓冲区
      if (res == 0) {
        RingBuffer_Flush(&U1rxring);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 50);
  }
}
