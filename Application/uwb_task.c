#include "uwb_task.h"


uint8_t useUwb = 0;
uint32_t totalFrameCount = 0, errorFrameCount = 0;

int32_t satrtPos[2];
static int32_t pos[2];

static uint8_t unpack_data(void);
static uint8_t validate_data(void); 
static void position_control(const int tarX,const int tarY);
static void fusion_parameter_init(void);
static void imu_fus_update(u8 dT_ms);

void uwb_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //uwb串口初始化
  Drv_Uart2Init(921600);
  debugOutput("uwb use uart2，rate:921600");

	fusion_parameter_init();
  while (1) {

    //解析成功,校验成功可以使用数据
    uint8_t dataValidity = unpack_data();
    imu_fus_update(10);

		
		if(flag.taking_off)
			imu_fus_update(10);
		else
			fusion_parameter_init();
		 
    if(useUwb == 0)
      Program_Ctrl_User_Set_HXYcmps(0, 0); 

		if(flag.taking_off)
      switch(useUwb) {
      case 1:
        position_control(300,300);
        break;
      case 2:
        position_control(200,200);
        break;
      }

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100);
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

//位置控制(单位:cm)
static void position_control(const int tarX,const int tarY)
{
  const float kp = 0.5f;
  const float ki = 0.0f;
  const float kd = 0.0f;

//  //误差不大就不进行控制
//  const uint8_t permitError = 3;
//  if(abs(posFus[X].out-tarX)<permitError &&
//     abs(posFus[Y].out-tarY)<permitError ) {
//    Program_Ctrl_User_Set_HXYcmps(0, 0);
//    return;
//  }


  float out[2] = {0};
  int exp[2] = {tarX, tarY};

  for(uint8_t i=0; i<2; i++) {
    //P
    int error = exp[i] - posFus[i].out ;

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




static _inte_fix_filter_st accFus[2];
static _fix_inte_filter_st speedFus[2];
_fix_inte_filter_st posFus[2];

#define N_TIMES 5

//uwb数据融合加速度计
static void imu_fus_update(u8 dT_ms)
{
  s32 rawPos[2];
  rawPos[X] = pos[X];
  rawPos[Y] = pos[Y];

  static s32 lastRawPos[2],lastRawSpeed[2];
  static s32 rawSpeed[2],rawAcc[2];

  static u8 cyc_xn; 
  float hz = safe_div(1000,dT_ms,0);
  float ntimes_hz = hz/N_TIMES;

  cyc_xn ++;
  cyc_xn %= N_TIMES;

  for(uint8_t i=X; i<Z ; i++) {
    if(cyc_xn == 0) {
      rawSpeed[i] = (rawPos[i] - lastRawPos[i]) *ntimes_hz;
      rawAcc[i] = (rawSpeed[i] - lastRawSpeed[i]) *ntimes_hz;

      lastRawPos[i] = rawPos[i];
      lastRawSpeed[i] = rawSpeed[i];
    }

    accFus[i].in_est = wcx_acc_use;
    accFus[i].in_obs = rawAcc[i];
    inte_fix_filter(dT_ms*1e-3f,&accFus[i]);
 
    speedFus[i].in_est_d = accFus[i].out;
    speedFus[i].in_obs = rawSpeed[i];
    fix_inte_filter(dT_ms*1e-3f,&speedFus[i]);
 
    posFus[i].in_est_d = speedFus[i].out;
    posFus[i].in_obs = rawPos[i];
    fix_inte_filter(dT_ms*1e-3f,&posFus[i]); 
  }

}

static void fusion_parameter_init(void)
{ 
  for(uint8_t i=X; i<Z ; i++) { 
		accFus[i].fix_ki = 0.1f;
		accFus[i].ei_limit = 100;

		speedFus[i].fix_kp = 0.6f;
		speedFus[i].e_limit = 100;

		posFus[i].fix_kp = 0.3f;
		//posFus[i].e_limit = 200;
		
		
		accFus[i].out = 0;
		accFus[i].ei = -wcz_acc_use;

		speedFus[i].out = 0;
		speedFus[i].e = 0;

		posFus[i].out = 0;
		posFus[i].e = 0;
  } 
}
