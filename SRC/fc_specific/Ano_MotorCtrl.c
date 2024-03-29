#include "Ano_MotorCtrl.h"
#include "Drv_PwmOut.h"

#include "rc_update.h"

/*
四轴：
      机头
   m2     m1
     \   /
      \ /
      / \
     /   \
   m3     m4
      屁股
*/
int16_t motor[MOTORSNUM];

_mc_st mc;


void power_distribution(uint8_t dT_ms)
{
  //电调校准模式,四路PWM输出即为油门值
  if(escCalibrationMode) {
    //遥控器油门响应
    uint16_t out = CH_N[CH_THR] + 500 ;
    out = LIMIT(out,0,999);

    //分配动力
    for(uint8_t i =0; i<4; i++)
      Drv_MotorPWMSet(i,out);

    return;
  }

  //上锁状态一定停转电机
  if(!flag.unlock_sta) {
    //电机停转
    for(uint8_t i =0; i<4; i++)
      Drv_MotorPWMSet(i,0);

    flag.motor_preparation = 0;

    return;
  }

  uint16_t idleOut = 10*LIMIT(Ano_Parame.set.idle_speed_pwm,0,30);

  //按顺序启动电机后才能起飞
  if(!flag.motor_preparation) {
    static uint16_t timerCount = 0;
    timerCount += dT_ms; 

    //按顺序低速启动电机
    if(timerCount<200) {
      motor[m1] = idleOut;
      motor[m2] = 0;
      motor[m3] = 0;
      motor[m4] = 0;
    } else if(timerCount<400) {
      motor[m2] = idleOut;
    } else if(timerCount<600) {
      motor[m3] = idleOut;
    } else if(timerCount<800) {
      motor[m4] = idleOut;
    }else if(timerCount>1800){
			 flag.motor_preparation = 1;
			 timerCount = 0;
		}
 
  }

  //飞行状态
  if(flag.taking_off && flag.motor_preparation) {
    motor[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
    motor[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
    motor[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
    motor[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;

    //限幅
    for(uint8_t i=0; i<MOTORSNUM; i++)
      motor[i] = LIMIT(motor[i],0,999);
  }

  //分配动力
  for(uint8_t i=0; i<MOTORSNUM; i++)
    Drv_MotorPWMSet(i,motor[i]);
}



