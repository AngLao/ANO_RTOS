#include "openmv_task.h"

static uint8_t unpack_data(void);

//修正偏航
#define ANGLE_ID (0x01)
//面积
#define AREA_ID (0x02)
//宽度
#define WIDTH_ID (0x03)
//图像位置
#define POS_ID (0x04)
//二维码识别结果
#define RES_ID (0x05)

uint32_t angleValue, areaValue, widthValue, posValue, resValue;
static uint32_t value;

static uint8_t unpack_data(void);
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
    switch (recId) {
    case 0:
      break;
    case ANGLE_ID :
      angleValue = value;
      break;
    case AREA_ID :
      areaValue = value;
      break;
    case WIDTH_ID :
      widthValue = value;
      break;
    case POS_ID :
      posValue = value;
      static uint32_t lastPos = 0; 
			//位置数据在变化且未识别到二维码
      if(posValue!=lastPos && resValue == 0)
				//控制杆的位置在图像中心保持低速前进
        position_control(posValue);
      else
        Program_Ctrl_User_Set_HXYcmps(0, 0);
      break;
    case RES_ID :
      resValue = value;
      break;
    }

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


//位置控制(单位:cm)
static void position_control(uint32_t measureValue)
{
  const float kp = 0.08f;
  const float ki = 0.0f;
  const float kd = 0.0f;


  float out  =  0 ;
  uint32_t exp  = 160;

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
//		//I
//    if(abs(error) < 10)
//      errorIntegral += error;
//    else
//      errorIntegral = 0;

//    if(errorIntegral > 100)
//      errorIntegral = 100;
//    if(errorIntegral < -100)
//      errorIntegral = -100;

  //D
  static int lastError = 0;
  int differential = lastError - error;
  lastError = error;

  out = error*kp + errorIntegral*ki + differential*kd;


  Program_Ctrl_User_Set_HXYcmps(0, out);
}

