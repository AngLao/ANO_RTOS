#include "openmv_task.h"

//只控制X,Y和YAW的速度
float openmvSpeedOut[3]; 

uint8_t useOpenmv = 0;
openmv_t mvValue; 
static uint32_t value;

static uint8_t unpack_data(void);
static void assign_value(uint8_t mId) ;
static float position_control_x(uint32_t exp ,uint32_t measureValue);
static float position_control_y(uint32_t exp ,uint32_t measureValue);

/* openmv数据更新 */
void openmv_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //openmv串口初始化
  Drv_Uart5Init(921600);
  debugOutput("openmv use uart5，rate:921600");

	static TickType_t waitTime = 0;
  while (1) {
    uint8_t recId = unpack_data(); 
		
		if(recId){ 
			assign_value(recId);
			waitTime = xTaskGetTickCount();
		}
		if(useOpenmv){
			openmvSpeedOut[X] = position_control_x(160 ,mvValue.posY);
			openmvSpeedOut[Y] = position_control_y(160 ,mvValue.posX);
		}
		//一定时间内无有效数据或者不使用openmv则控制速度清零
		if((xTaskGetTickCount() - waitTime) > pdMS_TO_TICKS(500) || useOpenmv == 0) 
			memset(openmvSpeedOut,0,sizeof(openmvSpeedOut));
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 50);
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

//根据id赋值,使用数据
static void assign_value(uint8_t mId)
{
	switch (mId) {
	case X_ID :
		mvValue.posX = value;
		break;
	case Y_ID :
		mvValue.posY = value;
		break;
	default:
		break;
	} 
}

//控制目标在图像中的位置（返回的速度控制yaw可以绕杆）
static float position_control_x(uint32_t exp ,uint32_t measureValue)
{
  const float kp = 0.15f;
  const float ki = 0.0f; 

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
	//I
	if(abs(error) < 5)
		errorIntegral += error;
	else
		errorIntegral = 0;
 
	errorIntegral = LIMIT(errorIntegral, -50,50); 
 
  float out = error*kp + errorIntegral*ki;
	return out ;
}

//控制目标在图像中像素的大小(近似控制openmv与目标的相对距离)
static float position_control_y(uint32_t exp ,uint32_t measureValue)
{
  const float kp = 0.15f;
  const float ki = 0.0f; 

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
	//I
	if(abs(error) < 5)
		errorIntegral += error;
	else
		errorIntegral = 0;
 
	errorIntegral = LIMIT(errorIntegral, -50,50); 
 
  float out = error*kp + errorIntegral*ki;
	return out ;
}