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
s16 motor[MOTORSNUM];
s16 motor_step[MOTORSNUM]; 

static u16 motor_prepara_cnt;
_mc_st mc;
u16 IDLING;//10*Ano_Parame.set.idle_speed_pwm  //200
void Motor_Ctrl_Task(u8 dT_ms)
{
  u8 i; 
	
  if(flag.unlock_sta) {
    IDLING = 10*LIMIT(Ano_Parame.set.idle_speed_pwm,0,30);

    if(flag.motor_preparation == 0) {
      motor_prepara_cnt += dT_ms;

      if(flag.motor_preparation == 0) {
        if(motor_prepara_cnt<300) {
          motor[m1] = IDLING;
        } else if(motor_prepara_cnt<600) {
          motor[m2] = IDLING;
        } else if(motor_prepara_cnt<900) {
          motor[m3] = IDLING;
        } else if(motor_prepara_cnt<1200) {
          motor[m4] = IDLING;
        } else {
          flag.motor_preparation = 1;
          motor_prepara_cnt = 0;
        }
      }

    }
  } else {
    flag.motor_preparation = 0;
  } 
	
  if(flag.motor_preparation == 1) {
    motor_step[m1] = mc.ct_val_thr  +mc.ct_val_yaw -mc.ct_val_rol +mc.ct_val_pit;
    motor_step[m2] = mc.ct_val_thr  -mc.ct_val_yaw +mc.ct_val_rol +mc.ct_val_pit;
    motor_step[m3] = mc.ct_val_thr  +mc.ct_val_yaw +mc.ct_val_rol -mc.ct_val_pit;
    motor_step[m4] = mc.ct_val_thr  -mc.ct_val_yaw -mc.ct_val_rol -mc.ct_val_pit;


    for(i=0; i<MOTORSNUM; i++) {
      motor_step[i] = LIMIT(motor_step[i],IDLING,1000); 
    } 
  }

  for(i=0; i<MOTORSNUM; i++) {
    if(flag.unlock_sta) {
      if(flag.motor_preparation == 1) {
        motor[i] = LIMIT(motor_step[i],IDLING,999);
      }

    } else {
      motor[i] = 0;
    }

  }

  //配置输出
  for(u8 i =0; i<4; i++) {
    Drv_MotorPWMSet(i,motor[i]);
  } 

}



