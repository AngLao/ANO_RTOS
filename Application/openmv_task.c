#include "openmv_task.h"

float openmvSpeedOut[2]; 

openmv_t mvValue; 
static uint32_t value;

static uint8_t unpack_data(void);
static void assign_value(uint8_t mId) ;
static void position_control(uint32_t measureValue);

/* openmv数据更新 */
void openmv_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //openmv串口初始化
  Drv_Uart5Init(921600);
  debugOutput("openmv use uart5，rate:921600");

  while (1) {
    uint8_t recId = unpack_data(); 
		assign_value(recId);
		
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
        analyticCnt++;
      else {
        numBuf[numCount] = data;
        numCount++;
      }
      //出错复位
      if(numCount>5) {
        analyticCnt=0;
        numCount = 0;
        id = 0;
      }
      break;
    }

    if(analyticCnt == 3)
      break;
  }

  //组合成数字 fifo 大端模式
  if(id != 0 && analyticCnt == 3) {
    uint32_t numTemp = 0;
    for(uint8_t i=0; i<numCount; i++) {
      if(i != 0)
        numTemp *= 10;
      numTemp += numBuf[i];
    }
    numCount = 0;
    analyticCnt = 0;
    value = numTemp;
  }
  return id;
}

//根据id赋值
static void assign_value(uint8_t mId)
{
	switch (mId) {
	case ANGLE_ID :
		mvValue.angle = value;
		break;
	case AREA_ID :
		mvValue.area = value;
		break;
	case WIDTH_ID :
		mvValue.width = value;
		break;
	case POS_ID :
		mvValue.pos = value;
		break;
	case RES_ID :
		mvValue.res = value;
		break;
	default:
		break;
	} 
}

//位置控制(单位:cm)
static void position_control(uint32_t measureValue)
{
  const float kp = 0.08f;
  const float ki = 0.0f;

  float out  =  0 ;
  uint32_t exp  = 160;

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
		//I
    if(abs(error) < 10)
      errorIntegral += error;
    else
      errorIntegral = 0;

    if(errorIntegral > 100)
      errorIntegral = 100;
    if(errorIntegral < -100)
      errorIntegral = -100;


  out = error*kp + errorIntegral*ki;

}

