#include "Ano_LocCtrl.h"
#include "Ano_Imu.h"
#include "Ano_FlightCtrl.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_Parameter.h"


//位置速度环控制参数
_PID_arg_st loc_arg_1[2] ;

//位置速度环控制数据
_PID_val_st loc_val_1[2] ;

//位置速度环修正控制参数
_PID_arg_st loc_arg_1_fix[2] ;

//位置速度环修正控制数据
_PID_val_st loc_val_1_fix[2] ;

/*角度环PID参数初始化*/
void Loc_1level_PID_Init()
{
  //normal
  loc_arg_1[X].kp = Ano_Parame.set.pid_loc_1level[KP];
  loc_arg_1[X].ki = 0.0f  ;
  loc_arg_1[X].kd_ex = 0.00f ;
  loc_arg_1[X].kd_fb = Ano_Parame.set.pid_loc_1level[KD];
  loc_arg_1[X].k_ff =  0.05f;

  loc_arg_1[Y] = loc_arg_1[X];
  //fix
  loc_arg_1_fix[X].kp = 0.0f  ;
  loc_arg_1_fix[X].ki = Ano_Parame.set.pid_loc_1level[KI] ;
  loc_arg_1_fix[X].kd_ex = 0.00f;
  loc_arg_1_fix[X].kd_fb = 0.00f;
  loc_arg_1_fix[X].k_ff = 0.0f;

  loc_arg_1_fix[Y] = loc_arg_1_fix[X];
}

//水平位置
_loc_ctrl_st loc_ctrl_1;

/*位置速度环*/
void Loc_1level_Ctrl(u16 dT_ms)
{
  //积分修正速度反馈值
  static float fb_speed_fix[2];

  //加速度计速度反馈
  static float vel_fb_d_lpf[2];

  //期望速度赋值
  loc_ctrl_1.exp[X] = fs.speed_set_h[X];
  loc_ctrl_1.exp[Y] = fs.speed_set_h[Y];

  //加速度计速度反馈低通滤波
  LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[X],vel_fb_d_lpf[X]);
  LPF_1_(5.0f,dT_ms*1e-3f,imu_data.h_acc[Y],vel_fb_d_lpf[Y]);

  //光流正常
  if(switchs.of_flow_on) {

    if(sens_hd_check.of_ok) {
      loc_ctrl_1.fb[X] = OF_DX2 + 0.03f *vel_fb_d_lpf[X];
      loc_ctrl_1.fb[Y] = OF_DY2 + 0.03f *vel_fb_d_lpf[Y];

      fb_speed_fix[X] = OF_DX2FIX;
      fb_speed_fix[Y] = OF_DY2FIX;
    } else {
      loc_ctrl_1.fb[X] = of_rdf.gnd_vel_est_h[X] + 0.03f *vel_fb_d_lpf[X];
      loc_ctrl_1.fb[Y] = of_rdf.gnd_vel_est_h[Y] + 0.03f *vel_fb_d_lpf[Y];

      fb_speed_fix[X] = of_rdf.gnd_vel_est_h[X];
      fb_speed_fix[Y] = of_rdf.gnd_vel_est_h[Y];
    }

    for(u8 i =0; i<2; i++) {
      PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
                     loc_ctrl_1.exp[i],				//前馈值
                     loc_ctrl_1.exp[i],				//期望值（设定值）
                     loc_ctrl_1.fb[i],			//反馈值（）
                     &loc_arg_1[i], //PID参数结构体
                     &loc_val_1[i],	//PID数据结构体
                     50,//积分误差限幅
                     10 *flag.taking_off			//integration limit，积分限幅
                   )	;

      //fix
      PID_calculate( dT_ms*1e-3f,            //周期（单位：秒）
                     loc_ctrl_1.exp[i],				//前馈值
                     loc_ctrl_1.exp[i],				//期望值（设定值）
                     fb_speed_fix[i],			//反馈值（）
                     &loc_arg_1_fix[i], //PID参数结构体
                     &loc_val_1_fix[i],	//PID数据结构体
                     50,//积分误差限幅
                     10 *flag.taking_off			//integration limit，积分限幅
                   )	;

      loc_ctrl_1.out[i] = loc_val_1[i].out + loc_val_1_fix[i].out;	//(PD)+(I)
    }
  } else {
    loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[X] ;
    loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs.speed_set_h[Y] ;
  }
}

_loc_ctrl_st loc_ctrl_2;

