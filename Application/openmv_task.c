#include "openmv_task.h"
#include "Ano_OPMV_Ctrl.h"
#include "Ano_OPMV_CBTracking_Ctrl.h"

#define OPENMV_CBTRACK_CTR (1)

//只控制X,Y和YAW的速度
float openmvSpeedOut[3]; 

_openmv_data_st opmv;

uint8_t useOpenmv = 0;
openmv_t mvValue; 
static uint32_t value;

static uint8_t unpack_data(void);
static void assign_value(uint8_t mId) ;
static float position_control(uint32_t exp ,uint32_t measureValue);
static float area_control(uint32_t exp ,uint32_t measureValue);


/* openmv数据更新 */
void openmv_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //openmv串口初始化
  Drv_Uart5Init(921600);
  debugOutput("openmv use uart5，rate:921600");

	static TickType_t waitTime = 0;
  while (1) {
    
		
#if (OPENMV_CBTRACK_CTR == 1)
    uint8_t recId = unpack_data(); /* 解帧 */
		assign_value(recId);					 /* 解值 */
		
		if((recId == POS_X_ERROR) || (recId == POS_Y_ERROR))
		{
			if(recId == POS_X_ERROR)
			{
				opmv.cb.pos_x = value;/* 原始数据 */
				opmv.cb.pos_x = opmv.cb.pos_x - 160;/* 左边偏移至图像中心 */
				opmv.cb.pos_x = -opmv.cb.pos_x;
			}
			else if(recId == POS_Y_ERROR)
			{
				opmv.cb.pos_y = value;/* 原始数据 */
				opmv.cb.pos_y = opmv.cb.pos_y - 120;/* 左边偏移至图像中心 */
				opmv.cb.pos_y = -opmv.cb.pos_y;
			}
			
			opmv.cb.sta = 1;					/* 识别有效 */
			opmv.mode_sta = 1;				/* openmv追寻色块模式 */
			
			OpenMV_Check_Reset();	
		}

		/*OPMV检测是否掉线*/
		OpenMV_Offline_Check(20);
		/*OPMV色块追踪数据处理任务*/
		ANO_CBTracking_Task(20);
		/*OPMV控制任务*/
		ANO_OPMV_Ctrl_Task(20);
#else
		uint8_t recId = unpack_data(); 
		if(recId){ 
			assign_value(recId);
			waitTime = xTaskGetTickCount();
		}
		
		//一定时间内无有效数据或者不使用openmv则控制速度清零
		if((xTaskGetTickCount() - waitTime) > pdMS_TO_TICKS(500) || useOpenmv == 0) 
		memset(openmvSpeedOut,0,sizeof(openmvSpeedOut));
#endif
		
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
		openmvSpeedOut[YAW] = position_control(160 ,mvValue.pos);
		openmvSpeedOut[Y] = 20;
		break;
	case RES_ID :
		mvValue.res = value;
		break;
	default:
		break;
	} 
}

//控制目标在图像中的位置（返回的速度控制yaw可以绕杆）
static float position_control(uint32_t exp ,uint32_t measureValue)
{
  const float kp = -0.38f;
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
static float area_control(uint32_t exp ,uint32_t measureValue)
{
  const float kp = 0.2f;
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

