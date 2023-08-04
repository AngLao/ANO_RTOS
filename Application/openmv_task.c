#include "openmv_task.h"

//只控制X,Y和YAW的速度
float openmvSpeedOut[3]; 

static int16_t firePos[2];

openmv_t mvValue; 
static uint32_t value;

static uint8_t unpack_data(void);
static void assign_value(uint8_t mId) ;
static float position_control_x(uint32_t exp ,uint32_t measureValue);
static float position_control_y(uint32_t exp ,uint32_t measureValue);
static uint8_t position_stability_judgment(uint16_t setTime);
static uint8_t throw_task(void);
static void send_fire_pos(void);

static uint8_t in_roi(void);

#define led_on() (ROM_GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, 0))
#define led_off() (ROM_GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, 1))

const uint16_t expPosX = 120 , expPosY = 160;
/* openmv数据更新 */
void openmv_update_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  //openmv串口初始化
  Drv_Uart5Init(921600);
  debugOutput("openmv use uart5，rate:921600");

	static TickType_t waitTime = 0;
	static uint8_t  onlyOnce = 1;
  while (1) {
    uint8_t recId = unpack_data(); 
		
		//找到色块
		if(recId && onlyOnce){ 
			//超时等待计时更新
			waitTime = xTaskGetTickCount();
			//更新数据
			assign_value(recId);
			//根据任务状态执行动作
			switch(taskStatus_2023){
				//巡航中找到色块
				case cruise:
					//中止巡航,开始定位火源
						if( in_roi() )
							taskStatus_2023 = fixPoint;
					break;
				case fixPoint:
					//位置稳定,记录火源位置,打开指示灯
					if(position_stability_judgment(300) == 1){
						firePos[X] = getPosX();
						firePos[Y] = getPosY();
						
						led_on();
            
						taskStatus_2023 = throwObject; 
					}
					break;
				case throwObject:
					//投掷任务执行完成且回到巡航高度,继续巡航
					if(throw_task() == 1){
						taskStatus_2023 = cruise; 
						onlyOnce = 0;
					}
					break;
			}
			
			//定位色块
			openmvSpeedOut[X] = position_control_x(expPosX ,mvValue.posX);
			openmvSpeedOut[Y] = position_control_y(expPosY ,mvValue.posY);
		}
		
		//一定时间内无有效数据或者巡航状态mv控制速度清零
		if((xTaskGetTickCount() - waitTime) > pdMS_TO_TICKS(500) || taskStatus_2023 == cruise) {
			taskStatus_2023 = cruise;
			memset(openmvSpeedOut,0,sizeof(openmvSpeedOut));
		}
		
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
		mvValue.posY = value;
		break;
	case Y_ID :
		mvValue.posX = value;
		break;
	default:
		break;
	} 
}

//控制目标在图像中的位置（返回的速度控制yaw可以绕杆）
static float position_control_x(uint32_t exp ,uint32_t measureValue)
{
  const float kp = 0.12f;
  const float ki = -0.06f; 

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
	//I
	if(abs(error) < 10)
		errorIntegral += error;
	else
		errorIntegral = 0;
  
	errorIntegral = LIMIT(errorIntegral, -300,300); 
 
  float out = error*kp + errorIntegral*ki;
	return out ;
}

static float position_control_y(uint32_t exp ,uint32_t measureValue)
{
  const float kp = 0.12f;
  const float ki = -0.06f; 

  //P
  int error = exp - measureValue  ;

  static int errorIntegral = 0;
	//I
	if(abs(error) < 10)
		errorIntegral += error;
	else
		errorIntegral = 0;
 
	errorIntegral = LIMIT(errorIntegral, -300,300); 
 
  float out = error*kp + errorIntegral*ki;
	return out ;
}


//位置稳定判断
static uint8_t position_stability_judgment(uint16_t setTime)
{  	
	static TickType_t lastTime = 0;
	TickType_t thisTime = xTaskGetTickCount();
	
	uint8_t res = 0;
	res += (ABS(mvValue.posX-expPosX) < 40) ? 1 : 0 ;
	res += (ABS(mvValue.posY-expPosY) < 40) ? 1 : 0 ;
	
	if(lastTime == 0 || res != 2)
		lastTime = thisTime;
	
	if( (thisTime - lastTime) > pdMS_TO_TICKS(setTime)){
		lastTime = 0;
	  return 1;
	}
	
	return 0;
}

//投掷任务(setTime为悬停时间)
static uint8_t throw_task(void)
{  	
	const uint16_t downWaitTime = 3200;
	const uint16_t upWaitTime = 1000;
	static uint16_t	setTime = downWaitTime;
	
	static uint16_t tarHight = 100;
	static TickType_t lastTime = 0;
	TickType_t thisTime = xTaskGetTickCount();
	
	float	hightError = tarHight - wcz_hei_fus.out;
	//计算起飞速度
	int16_t speedOut = 2.0 * hightError;
	Program_Ctrl_User_Set_Zcmps(speedOut);
	
	if( (lastTime == 0) || ( ABS((int)hightError) > 5) )
		lastTime = thisTime;
	
	if( (thisTime - lastTime) > pdMS_TO_TICKS(setTime)){
		lastTime = 0;
		//下降保持了3s,投掷物品,且发送坐标信息后上升
		if(tarHight != Ano_Parame.set.auto_take_off_height){
			setTime = upWaitTime;
			tarHight = Ano_Parame.set.auto_take_off_height;
			//控制舵机投弹
			gear_protocol_set(6 , 0 );// perDegree_90(90.0f) );
			gear_protocol_set(7 , 0 );// perDegree_90(90.0f) );
			
			//发送坐标
			send_fire_pos();
		}
		//上升完成结束任务继续巡航
		else
			return 1;
	}
	
	return 0;
}

//发送当前位置信息到小车端
static void send_fire_pos(void)
{  	
		char coordinateStr[50];
		
		memset(coordinateStr,0,sizeof(coordinateStr));
		sprintf((char*)coordinateStr,"firePos,%d,%d\r\n",firePos[X],firePos[Y]);
		
	//发送多遍防止丢失
	  Drv_Uart3SendBuf((uint8_t*)coordinateStr, sizeof(coordinateStr));
	  Drv_Uart3SendBuf((uint8_t*)coordinateStr, sizeof(coordinateStr));
	  Drv_Uart3SendBuf((uint8_t*)coordinateStr, sizeof(coordinateStr));
}

//是否在火源可能出现的区域
static uint8_t in_roi(void)
{  	
	uint8_t res ;
	uint8_t x = dotPath[dotfIndex].x , y = dotPath[dotfIndex].y;
	if( x > 3 || y > 0)
		return 1;
	else
		return 0;

}