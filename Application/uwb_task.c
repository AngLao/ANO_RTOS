#include "uwb_task.h"
 
uint32_t totalFrameCount = 0 , errorFrameCount = 0; 

static uint8_t validate_data(void);
static void position_control(const int tarX ,const int tarY);
static void drawing_circle(void);

uint8_t useUwb = 0; 
/* uwb数据更新 */
void uwb_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  static uint8_t pData[128];
	static uint16_t RingBufferDataLen;
	
  //uwb串口初始化
  Drv_Uart2Init(921600);
  debugOutput("uwb use uart2，rate:921600");

  while (1) { 
		uint8_t analyticResult = 0;
		for(; ;){ 
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
			}
		} 
		
		if(useUwb == 0)
			Program_Ctrl_User_Set_HXYcmps(0, 0);
		//解析成功校验成功可以使用数据
		if(analyticResult == 1 && validate_data() == 1){
			 
			if(useUwb == 1)
				drawing_circle();
			else if(useUwb == 2)
				position_control(2000 ,2000);
		} 
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100);
  }
} 

//判断数据是否有效
static uint8_t validate_data(void)
{ 
  static float lastPos[3]; 
	//结果为1数据有效
	uint8_t res = 0;
	//uwb数据在变化说明数据有效 
	if(lastPos[0] != g_nlt_tagframe0.result.pos_3d[0])
		res=1; 
	//保存上次数据
	lastPos[0] = g_nlt_tagframe0.result.pos_3d[0]; 
	
	return res; 
}

//位置控制(单位:cm)
static void position_control(const int tarX ,const int tarY)
{
	const float kp = 0.1f;
	const float ki = 0.0f;
	const float kd = 0.0f;
	
	static int errorIntegral = 0;
	static int lastError = 0;
	 
	float out[2] = {0};
	int exp[2] = {tarX , tarY};
	 
	for(uint8_t i=0;i<2;i++){
		int error = exp[i] - g_nlt_tagframe0.result.pos_3d[i] ; 
		
		if(abs(error) < 10) 
			errorIntegral += error;  
		else
			errorIntegral = 0;
		 
		if(errorIntegral > 100)
			errorIntegral = 100;
		if(errorIntegral < -100)
			errorIntegral = -100;
		
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
	
	 
	position_control(targetX ,targetY);
	
	
  int errorX = targetX - g_nlt_tagframe0.result.pos_3d[0] ; 
  int errorY = targetY - g_nlt_tagframe0.result.pos_3d[1] ; 
	
	if(abs(errorX)<5 && abs(errorY)<5)
		currentPoint += direction;
	 
	if(currentPoint == trackDivNumber) 
		direction *= -1; 
}
