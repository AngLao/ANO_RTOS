#include "uwb_task.h"


uint8_t useUwb = 0;
uint32_t totalFrameCount = 0, errorFrameCount = 0;

#define X 0
#define Y 1

int32_t satrtPos[2];
static int32_t pos[2];

static uint8_t unpack_data(void);
static uint8_t validate_data(void);
static uint8_t set_start_point(void); 
static void position_control(const int tarX,const int tarY);
static void drawing_circle(void);

void uwb_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //uwb串口初始化
  Drv_Uart2Init(921600);
  debugOutput("uwb use uart2，rate:921600");

  while (1) {

    //解析成功,校验成功可以使用数据
    uint8_t dataValidity = unpack_data();

    //记录起飞点
		static uint8_t setPointOk;
    if(useUwb == 0)
      Program_Ctrl_User_Set_HXYcmps(0, 0);
    else if(dataValidity==1 && setPointOk==0)
      setPointOk = set_start_point();

		if(!flag.taking_off)
			setPointOk = 0;
		
    //按需执行任务
    if(dataValidity==1 && setPointOk==1) {
      switch(useUwb) {
      case 1:
				//返回起飞点 
        position_control(satrtPos[X],satrtPos[Y]);
        break;
      case 2:
        position_control(2000,2000);
        break;
      }
    }
		

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 120);
  }
}

//解析缓冲区中的数据,返回1则解析成功
static uint8_t unpack_data(void)
{
  static uint8_t pData[128];
  static uint16_t RingBufferDataLen;

  uint8_t analyticResult = 0;
  while(1) {
    RingBufferDataLen = RingBuffer_GetCount(&uwbRingBuff) ;
    //缓冲区中数据不够一帧的长度
    if(RingBufferDataLen<128)
      break;

    //查找帧头
    RingBuffer_Pop(&uwbRingBuff, &pData[0]);

    if(pData[0] == 0x55) {
      totalFrameCount++;

      //弹出剩余数据
      for(uint16_t cnt = 1; cnt <128 ; cnt++)
        RingBuffer_Pop(&uwbRingBuff, &pData[cnt]);

      //解析数据
      analyticResult = g_nlt_tagframe0.UnpackData(pData, 128);

      //传输有错
      if(analyticResult == 0)
        errorFrameCount++;
      else {
        pos[X] = g_nlt_tagframe0.result.pos_3d[X];
        pos[Y] = g_nlt_tagframe0.result.pos_3d[Y];

				//数据检验
				analyticResult = validate_data();
      }

    }
  }
  return analyticResult;
}

//判断数据是否有效
static uint8_t validate_data(void)
{
  static float lastPos[2];
  //结果为1数据有效
  uint8_t res = 0;
  //uwb数据在变化说明数据有效
  if(lastPos[X] != pos[X])
    res=1;
  //保存上次数据
  lastPos[X] = pos[X];

  //围栏外的数据无效(以四个锚点围成的矩形做围栏)
  const int16_t maxX = 5000, maxY = 4000;  ;
  if(pos[X] < 0 || pos[Y] < 0)
    res = 0;
  if(pos[X] > maxX || pos[Y] > maxY)
    res = 0;

  return res;
}

//起飞完成后根据可信度设置起飞点
static uint8_t set_start_point(void)
{
  if(g_nlt_tagframe0.result.eop_3d[X]<10 &&
     g_nlt_tagframe0.result.eop_3d[Y]<10 ){

    satrtPos[X] = pos[X];
    satrtPos[Y] = pos[Y];
			 
		return 1;
  }
 
  return 0;
}

//位置控制(单位:cm)
static void position_control(const int tarX,const int tarY)
{
  const float kp = 0.04f;
  const float ki = 0.0f;
  const float kd = 0.0f;

	//误差不大就不进行控制
  const uint8_t permitError = 8;
	if(abs(pos[X]-tarX)<permitError &&
		 abs(pos[Y]-tarY)<permitError ){
		Program_Ctrl_User_Set_HXYcmps(0, 0);  
		return;	 
  }

	
  float out[2] = {0};
  int exp[2] = {tarX, tarY};

  for(uint8_t i=0; i<2; i++) {
		//P
    int error = exp[i] - pos[i] ;

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

    out[i] = error*kp + errorIntegral*ki + differential*kd;

  }

	Program_Ctrl_User_Set_HXYcmps(out[0], out[1]);
}


static void drawing_circle(void)
{
  //圆的中心坐标(单位:cm)
  const int16_t centerCoordinates[2] = {2500,2500};
  //圆的半径(单位:cm)
  const int16_t radius = 500;
  //轨迹细分成多少个点
  const uint16_t trackDivNumber = 40;
  //步长
  const uint16_t step = 2*radius/trackDivNumber;

  static int8_t currentPoint = 0, direction= 1;

  //x2+y2=r2
  int16_t targetX = (centerCoordinates[0] - radius)+ (currentPoint * step);
  int16_t targetY = direction*(int16_t)sqrt((double)abs(radius*radius - targetX*targetX));


  position_control(targetX,targetY);


  int errorX = targetX - pos[X] ;
  int errorY = targetY - pos[Y] ;

  if(abs(errorX)<15 && abs(errorY)<15)
    currentPoint += direction;

  if(currentPoint == trackDivNumber)
    direction *= -1;
}

