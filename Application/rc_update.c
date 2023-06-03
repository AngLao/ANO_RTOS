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
  if(flag.unlock_sta) {
    if(switchs.gps_on == 0) {
      flag.auto_take_off_land = AUTO_LAND;
    } else {
      flag.rc_loss_back_home = 1;
    } 
  }
}

//遥控器掉线检测回调函数
static void vDetectionRcConnection( void *pvParameters )
{
  //在定时器设定范围内有新数据到来,遥控器在线
  if(haveNewData) {
    //认为信号正常
    haveNewData = 0;
    if(flag.rc_loss) {
      flag.rc_loss = 0;
      LED_STA.noRc = 0;

      if(flag.taking_off)
        flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH; //解除下降
    }
  } else {
    //丢失遥控信号
    flag.rc_loss = 1;

    //打开丢失遥控信号指示灯
    LED_STA.noRc = 1;

    //执行遥控掉线保护
    offLineProtection();

  }
}


//解锁监测
static void unlockDetection(void)
{
  //系统状态判断
  flag.unlock_err = 0;	//允许解锁标志位

  //imu传感器异常
  if(!flag.sensor_imu_ok) {
    flag.unlock_err = 1;//imu异常，不允许解锁
  }

  //气压计异常
  if(!sens_hd_check.baro_ok) {
    LED_STA.errBaro = 1;
    flag.unlock_err = 2;//气压计异常，不允许解锁。
  }

  //惯性传感器异常
  if(!sens_hd_check.acc_ok && !sens_hd_check.gyro_ok) {
    LED_STA.errMpu = 1;
    flag.unlock_err = 3;//惯性传感器异常，不允许解锁。
  }

  //只有电池电压最低
  if(flag.power_state > 2) {
    flag.unlock_err = 4;//电池电压异常，不允许解锁
  }

  //正在操作flash
  if( para_sta.save_trig != 0) {
    flag.unlock_err = 4;//操作flash ，不允许解锁
  }

	//飞控上锁、解锁检测要油门在拉低时才进行
  if(CH_N[CH_THR] > -UN_THR_VALUE  ) {
    flag.thr_low = 0;//油门非低
  } else {
    flag.thr_low = 1;//油门拉低
  }

  //飞控上锁、解锁检测
  static TickType_t xFirstWakeTime ;
  //摇杆内八状态
  if(CH_N[CH_THR] < -UN_THR_VALUE && CH_N[CH_PIT] < -UN_PIT_VALUE && CH_N[CH_ROL] < -UN_ROL_VALUE && CH_N[CH_YAW] > UN_YAW_VALUE) {

    //记录初次时间
    if(xFirstWakeTime == 0) {
      xFirstWakeTime = xTaskGetTickCount();
    }
		
    TickType_t xThisWakeTime = xTaskGetTickCount(); //获取当前时间
    //到达指定时间差
    if(xThisWakeTime - xFirstWakeTime > pdMS_TO_TICKS(1500)) {
      //根据飞行状态决定上锁还是解锁(上锁状态就解锁,解锁状态就上锁)
      flag.unlock_cmd = !flag.unlock_sta ;
			xFirstWakeTime  = 0;
    } 
  } else {
    xFirstWakeTime = 0;
  }
	
  //飞控处于上锁状态 收到解锁命令
  if(flag.unlock_sta == 0 && flag.unlock_cmd == 1 ) {
    //系统存在错误,禁止解锁
    if(flag.unlock_err != 0) { 
      flag.unlock_cmd = 0; 
    }

    //打印解锁情况信息
    switch(flag.unlock_err) {
    case 0:
      ANO_DT_SendString("unlocking succeeded");
      break;
    case 1:
      ANO_DT_SendString("imu error");
      break;
    case 2:
      ANO_DT_SendString("barometer  error");
      break;
    case 3:
      ANO_DT_SendString("inertial sensor error");
      break;
    case 4:
      ANO_DT_SendString("undervoltage error");
      break;
    }
  } else if (flag.unlock_sta == 1 && flag.unlock_cmd == 0 ) {
    //飞控处于解锁状态 收到上锁命令
    ANO_DT_SendString("flight control is locked");

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

 
//遥控器除摇杆外开关变化检测
static void taskSwitchDetection(void)
{  
	flag.flight_mode = LOC_HOLD; 

  //通道1一键急停
  if(CH_N[AUX1]>300) {
    if(flag.unlock_sta) {
      ANO_DT_SendString("Emergency stop OK!");
			flag.unlock_sta = 0;
      flag.unlock_cmd = 0;
      flag.taking_off = 0;
      flag.flying = 0; 
    }
  }
 
	//通道2做一键起飞
	if(CH_N[AUX2]<300) {
		onekey.val = 0;
	} else {
		onekey.val = 1;
	} 
	 
}



void receivingTask(void) 
{
  //遥控器在线是接下来操作的前提
  if(flag.rc_loss == 1)  {
    return;
  }

  //遥控数据标准化
  dataStandardization();
  //解锁监测
  unlockDetection();
	
	/*飞行模式设置任务*/
	taskSwitchDetection();
}

//接收机模式初始化
void receivingModeInit()
{
  //根据参数配置接收机解析模式
  if(Ano_Parame.set.pwmInMode == SBUS) {
    Drv_SbusInit();
  } else {
    Drv_PpmInit();
  }

  //创建检测遥控器连接状态定时器
  TimerHandle_t xDetectionRcConnectionTimer = xTimerCreate(
        "Detection remote control connection",
        800,
        pdTRUE,
        0,
        vDetectionRcConnection
      );
  if( xDetectionRcConnectionTimer != NULL ) {
    //开启检测遥控器连接状态定时器
    xTimerStart( xDetectionRcConnectionTimer, 0 );
  } else {
    //内存不够处理
  } 
}
