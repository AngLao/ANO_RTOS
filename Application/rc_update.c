#include "rc_update.h"

int16_t CH_N[CH_NUM] = {0};

//遥控掉线保护
static void offLineProtection()
{
  //摇杆控制量归零
  CH_N[CH_THR] = 0;
  CH_N[CH_ROL] = 0;
  CH_N[CH_PIT] = 0;
  CH_N[CH_YAW] = 0;

  //如果飞控处于解锁状态，自动降落标记置位
  if(flag.unlock_sta &&  flag.auto_take_off_land != AUTO_LAND)
    flag.auto_take_off_land = AUTO_LAND;

}

//电调校准模式
uint8_t escCalibrationMode = 0;
//低速状态变化检测回调函数
static void vSlowDetection( void *pvParameters )
{
  //遥控信号检测
  //在定时器设定范围内有新数据到来,遥控器在线
  if(haveNewData) {
    //认为信号正常
    haveNewData = 0;
    if(flag.rc_loss) {
      flag.rc_loss = 0;
      LED_STA.noRc = 0;

      if(flag.taking_off)
        flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH; //解除下降

      debugOutput("remote connected");
    }
  } else {
    if(flag.rc_loss == 0) {
      //丢失遥控信号
      flag.rc_loss = 1;

      //打开丢失遥控信号指示灯
      LED_STA.noRc = 1;

      //执行遥控掉线保护
      offLineProtection();

      debugOutput("remote disconnection");
    }
  }

  //任务变化检测
  //通道2检测
  static uint8_t channelTwoState = 0;
  //开关回到零点
  if(CH_N[AUX2]<-100)
    channelTwoState = 0;

  if(channelTwoState == 0) {
    //开关打到高值
    if(CH_N[AUX2] > 300) {
      channelTwoState = 2;
//      debugOutput("CH_N[AUX2]  = 2"); 
			one_key_take_off(); 
      debugOutput("one_key_take_off"); 
    }
    //开关打到中值
    else if(CH_N[AUX2] > -100) {
      channelTwoState = 1;
//      debugOutput("CH_N[AUX2]  = 1"); 
			one_key_land();
      debugOutput("one_key_land"); 
    }
  }

  //通道3检测
  static uint8_t channelThreeState = 0;
  //开关回到零点
  if(CH_N[AUX3]<-100){ 
			channelThreeState = 0;  
			useUwb = 0;
	}
	
  if(channelThreeState == 0) {
    //开关打到高值
    if(CH_N[AUX3] > 300) {
      channelThreeState = 2;
//      debugOutput("CH_N[AUX3]  = 2");
			debugOutput("drawing_circle");
			useUwb = 1;
    }
    //开关打到中值
    else if(CH_N[AUX3] > -100) {
      channelThreeState = 1;
//      debugOutput("CH_N[AUX3]  = 1");
			debugOutput("x=2000,y=2000");
			useUwb = 2;
    }
  }

  //通道4做电调校准模式检测 
	//USB供电状态下打下通道四摇杆进入校准模式,PWM输出即为油门量 
  static uint8_t channelFourState = 0; 
  if(channelFourState == 1) {
    //开关回到零点
    if(CH_N[AUX4]<-100) {
      channelFourState = 0;  

			if(escCalibrationMode){ 
				escCalibrationMode = 0;
        debugOutput("Exit electrical alignment mode"); 
			}
    }
  }

  if(channelFourState == 0) {
    //开关打到高值
    if(CH_N[AUX4] > 300) {
      channelFourState = 1; 
 
      //用USB供电状态才进入校准模式
      if(flag.power_state == 4) {
        escCalibrationMode = 1; 
        debugOutput("Enter electrical alignment mode");
      }else{ 
        debugOutput("Please disconnect the battery");
			}
    }
  }



}

//高速状态变化检测回调函数
static void vFastDetection(void *pvParameters )
{
  //通道1检测
  static uint8_t channelOneState = 0;
  //开关回到零点
  if(CH_N[AUX1]<-100)
    channelOneState = 0;

  //开关打到高值
  else if(CH_N[AUX1] > 300) {
    //急停键按下后持续上锁
    if(flag.unlock_cmd == 1) {
      flag.unlock_cmd = 0;
      debugOutput("The emergency stop button is pressed");
    }

    if(channelOneState == 0) {
      channelOneState = 2;

      //通道1一键急停
      flag.unlock_sta = 0;
      flag.unlock_cmd = 0;
      flag.taking_off = 0; 
      debugOutput("Emergency stop!");
    }
  }

}

//解锁监测
static void unlockDetection(void)
{
  //系统状态判断
  flag.unlock_err = 0;	//允许解锁标志位

  //imu传感器异常
  if(!flag.sensor_imu_ok) 
    flag.unlock_err = 1; 

  //气压计异常
  if(!sens_hd_check.baro_ok) {
    LED_STA.errBaro = 1;
    flag.unlock_err = 2;
  }

  //惯性传感器异常
  if(!sens_hd_check.acc_ok && !sens_hd_check.gyro_ok) {
    LED_STA.errMpu = 1;
    flag.unlock_err = 3;
  }

  //电池电压异常
  if(flag.power_state > 2) 
    flag.unlock_err = 4; 

  //正在操作flash
  if( para_sta.save_trig != 0) 
    flag.unlock_err = 5; 

  //检测油门状态
  if(CH_N[CH_THR] > -UN_THR_VALUE)  
    flag.thr_low = 0;
	else 
		//油门拉低
    flag.thr_low = 1; 

  //飞控上锁、解锁检测
  static TickType_t xFirstWakeTime ;
  //摇杆内八状态
  if(CH_N[CH_THR] < -UN_THR_VALUE && CH_N[CH_PIT] < -UN_PIT_VALUE && CH_N[CH_ROL] < -UN_ROL_VALUE && CH_N[CH_YAW] > UN_YAW_VALUE) {

    //记录初次时间
    if(xFirstWakeTime == 0) {
      xFirstWakeTime = xTaskGetTickCount();
    }
    //本次摇杆内八状态未执行解锁操作
    if(xFirstWakeTime != 1) {
      TickType_t xThisWakeTime = xTaskGetTickCount(); //获取当前时间
      //到达指定时间差
      if(xThisWakeTime - xFirstWakeTime > pdMS_TO_TICKS(1000)) {
        //根据飞行状态决定上锁还是解锁(上锁状态就解锁,解锁状态就上锁)
        flag.unlock_cmd = !flag.unlock_sta ;
        //xFirstWakeTime为1说明当前摇杆处于内八状态且执行过一次解锁检测，直到松开内八才会归零检测下一次是否需要解锁
        xFirstWakeTime  = 1;
        debugOutput("rc change lock status");
      }
    }
  } else {

    //退出内八状态计数清零
    xFirstWakeTime = 0;
  }

  //飞控处于上锁状态 收到解锁命令
  if(flag.unlock_sta == 0 && flag.unlock_cmd == 1 ) {
    //系统存在错误,禁止解锁
    if(flag.unlock_err != 0) 
      flag.unlock_cmd = 0; 


    //打印解锁情况信息
    switch(flag.unlock_err) {
    case 0:
      debugOutput("unlocking succeeded");
      break;
    case 1:
      debugOutput("imu error");
      break;
    case 2:
      debugOutput("barometer  error");
      break;
    case 3:
      debugOutput("inertial sensor error");
      break;
    case 4:
      debugOutput("undervoltage error");
      break;
    case 5:
      debugOutput("flash error");
      break;
    }
  } else if (flag.unlock_sta == 1 && flag.unlock_cmd == 0 ) {
    //飞控处于解锁状态 收到上锁命令
    debugOutput("flight control is locked");

  }

  //飞控上锁状态切换
  flag.unlock_sta = flag.unlock_cmd;

}


//接收到的遥控数据标准化(-500 -- +500) 上位机显示值为标准化值+1500
static void dataStandardization(void)
{
  //sbus通信模式
  if(Ano_Parame.set.pwmInMode == SBUS ) {
    for(u8 i = 0; i < CH_NUM; i++) {
      CH_N[i] = 0.65f * ((s16)Rc_Sbus_In[i] - 1024); //248 --1024 --1800
      CH_N[i] = LIMIT(CH_N[i], -500, 500); //限制到+—500
    }
  } else {
    //PPM通信模式
    for(u8 i = 0; i < CH_NUM; i++) {
      //通道值+1500为上位机显示值
      CH_N[i] = ((s16)RC_PPM.Captures[i] - 1500); //1000 -- 2000us
      CH_N[i] = LIMIT(CH_N[i], -500, 500); //限制到+—500
    }
  }
}


//接收机模式初始化
void receivingModeInit()
{
  //根据参数配置接收机解析模式
  if(Ano_Parame.set.pwmInMode == SBUS){
    Drv_SbusInit();
		debugOutput("rc use sbus"); 
	}		
	else  {
    Drv_PpmInit();  
		debugOutput("rc use ppm");
	}

  //创建缓慢检测状态变化定时器（如检测遥控失联，任务启动等功能）
  TimerHandle_t xSlowDetectionTimer = xTimerCreate(
                                        "low-speed callback Detection",
                                        1000,
                                        pdTRUE,
                                        0,
                                        vSlowDetection
                                      );

  if( xSlowDetectionTimer != NULL ) {
    //开启检测遥控器连接状态定时器
    xTimerStart( xSlowDetectionTimer, 0 );
  } else {
    //内存不够处理
  }

  //创建快速检测状态变化定时器（用于实时检测，如急停等功能）
  TimerHandle_t xFastDetectionTimer = xTimerCreate(
                                        "high-speed callback Detection",
                                        25,
                                        pdTRUE,
                                        0,
                                        vFastDetection
                                      );

  if( xFastDetectionTimer != NULL ) {
    //开启遥控器按键状态变化检测定时器
    xTimerStart( xFastDetectionTimer, 0 );
  } else {
    //内存不够处理
  }
}



void receiving_task(void *pvParameters)
{
  //遥控接收模式初始化
  receivingModeInit();
	
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {
    //解锁监测(不可阻塞,否则失控无法上锁)
    unlockDetection();

    //遥控器在线
    if(flag.rc_loss != 1) 
      //遥控数据标准化
      dataStandardization(); 


    //高速sbus信号7ms1帧 约150hz
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 150);
  }
}
