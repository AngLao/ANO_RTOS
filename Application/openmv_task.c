#include "openmv_task.h"

static uint8_t unpack_data(void);

#define ANGLE_ID (0x10)

uint32_t value ;
uint8_t  id  ;

/* openmv数据更新 */
void openmv_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //openmv串口初始化
  Drv_Uart5Init(921600);
  debugOutput("openmv use uart5，rate:921600");

  while (1) { 
		uint8_t temp = unpack_data();
		if(temp != 0)
			id = temp;
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100);
  }
}


//解析缓冲区中的数据,返回id则解析成功
static uint8_t unpack_data(void)
{ 
	static uint8_t analyticCnt = 0;
	static uint8_t numCount = 0;
	static uint8_t numBuf[5] = {0};
	 
  uint8_t RingBufferDataLen = RingBuffer_GetCount(&openmvRing) ;

  uint8_t id = 0;

  for(uint8_t cnt = 0; cnt <RingBufferDataLen ; cnt++) {
    uint8_t data = 0;
    RingBuffer_Pop(&openmvRing, &data);
    //查找帧头
    switch(analyticCnt) {
    case 0:
      if(data == 0xAA )
        analyticCnt++;
      break;
    case 1:
      id = data;
      analyticCnt++;
      break;
    case 2:
      if(data == 0xFF)
        analyticCnt = 0;
      else {
        numBuf[numCount] = data;
        numCount++;
      }
			//出错复位
			if(numCount>5){
				analyticCnt=0;
				numCount = 0;
				id = 0; 
			}
      break;
    }
  }
	
	//组合成数字 fifo 大端模式
  if(id != 0 && analyticCnt == 0) {
    uint32_t numTemp = 0;
    for(uint8_t i=0; i<numCount; i++) {
			if(i != 0)
				numTemp *= 10;
      numTemp += numBuf[i]; 
    }
		numCount = 0;
    value = numTemp; 
  }
	return id;
}