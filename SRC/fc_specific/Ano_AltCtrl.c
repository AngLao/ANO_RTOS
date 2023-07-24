#include "Ano_AltCtrl.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_FlightDataCal.h"
#include "Ano_FlightCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_FlightDataCal.h"
#include "Ano_FlightDataCal.h"
 

#define AUTO_TAKE_OFF_KP 2.0f

static int16_t auto_fly(u8 dT_ms)
{
  static uint16_t take_off_ok_cnt = 0; 

	//解锁判断
  if(!flag.unlock_sta) { 
    take_off_ok_cnt = 0; 
    flag.auto_take_off_land = AUTO_TAKE_OFF_NULL;
		return 0;
  }
	
	//等待电机就绪
  if(!flag.motor_preparation)  
		return 0;
	
	int16_t speedOut = 0;
	//设置最大起飞速度
	int16_t	max_take_off_vel = LIMIT(Ano_Parame.set.auto_take_off_speed,20,200); 
	switch (flag.auto_take_off_land) {
		//自动起飞
		case (AUTO_TAKE_OFF): 
			flag.taking_off = 1;
		
			//计算起飞速度
			speedOut = AUTO_TAKE_OFF_KP *(Ano_Parame.set.auto_take_off_height - wcz_hei_fus.out);
			speedOut = LIMIT(speedOut,0,max_take_off_vel);

			//到达指定高度时间大于设定值自动起飞完成
			if(Ano_Parame.set.auto_take_off_height - loc_ctrl_2.exp[Z] <2)
				take_off_ok_cnt += dT_ms;
			else 
				take_off_ok_cnt = 0;
			
			//退出起飞流程条件2，2000毫秒后判断用户正在控制油门。
			if(take_off_ok_cnt >2000 || ABS(fs.speed_set_h_norm[Z])>0.1f) 
				flag.auto_take_off_land = AUTO_TAKE_OFF_FINISH;
			break;
			
		//自动降落
		case (AUTO_LAND): 
			//设置自动下降速度
			speedOut = -(s16)LIMIT(Ano_Parame.set.auto_landing_speed,20,200);
			break; 
		
		//自动起飞完成
		case (AUTO_TAKE_OFF_FINISH):
			//自动控制速度清零
      speedOut = 0;
			break; 
		
		default:
			//默认控制速度为零
      speedOut = 0; 
			break; 
	}
	
	return speedOut; 
}


_PID_arg_st alt_arg_2;
_PID_val_st alt_val_2;

/*高度环PID参数初始化*/
void Alt_2level_PID_Init()
{
  alt_arg_2.kp = Ano_Parame.set.pid_alt_2level[KP];
  alt_arg_2.ki = Ano_Parame.set.pid_alt_2level[KI];
  alt_arg_2.kd_ex = 0.00f;
  alt_arg_2.kd_fb = Ano_Parame.set.pid_alt_2level[KD];
  alt_arg_2.k_ff = 0.0f;

}




void Alt_2level_Ctrl(float dT_s)
{
  int16_t autoFlySpeed = auto_fly(1000*dT_s);

  fs.alt_ctrl_speed_set = fs.speed_set_h[Z] + autoFlySpeed ;
  //
  loc_ctrl_2.exp[Z] += fs.alt_ctrl_speed_set *dT_s;
  loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],loc_ctrl_2.fb[Z]-200,loc_ctrl_2.fb[Z]+200);
  //
  loc_ctrl_2.fb[Z] = (s32)wcz_hei_fus.out;

  if(fs.alt_ctrl_speed_set != 0) {
    flag.ct_alt_hold = 0;
  } else {
    if(ABS(loc_ctrl_1.exp[Z] - loc_ctrl_1.fb[Z])<20) {
      flag.ct_alt_hold = 1;
    }
  }

  if(flag.taking_off == 1) {

    PID_calculate( dT_s,            //周期（单位：秒）
                   0,				//前馈值
                   loc_ctrl_2.exp[Z],				//期望值（设定值）
                   loc_ctrl_2.fb[Z],			//反馈值（）
                   &alt_arg_2, //PID参数结构体
                   &alt_val_2,	//PID数据结构体
                   100,//积分误差限幅
                   0			//integration limit，积分限幅
                 );

  } else {
    loc_ctrl_2.exp[Z] = loc_ctrl_2.fb[Z];
    alt_val_2.out = 0;

  }

  alt_val_2.out  = LIMIT(alt_val_2.out,-150,150);
}



_PID_arg_st alt_arg_1;
_PID_val_st alt_val_1;

/*垂直速度环PID参数初始化*/
void Alt_1level_PID_Init()
{
  alt_arg_1.kp = Ano_Parame.set.pid_alt_1level[KP];
  alt_arg_1.ki = Ano_Parame.set.pid_alt_1level[KI];
  alt_arg_1.kd_ex = 0.00f;
  alt_arg_1.kd_fb = 0;//Ano_Parame.set.pid_alt_1level[KD];
  alt_arg_1.k_ff = 0.0f;

}

static float err_i_comp;
static float w_acc_z_lpf;
void Alt_1level_Ctrl(float dT_s)
{
  u8 out_en = (flag.taking_off != 0) ? 1 : 0;


  loc_ctrl_1.exp[Z] = 0.6f *fs.alt_ctrl_speed_set + alt_val_2.out;//速度前馈0.6f，直接给速度

  w_acc_z_lpf += 0.2f *(imu_data.w_acc[Z] - w_acc_z_lpf); //低通滤波

  loc_ctrl_1.fb[Z] = wcz_spe_fus.out + Ano_Parame.set.pid_alt_1level[KD] *w_acc_z_lpf;//微分先行，下边PID函数微分系数为0


  PID_calculate( dT_s,            //周期（单位：秒）
                 0,				//前馈值
                 loc_ctrl_1.exp[Z],				//期望值（设定值）
                 loc_ctrl_1.fb[Z],			//反馈值（）
                 &alt_arg_1, //PID参数结构体
                 &alt_val_1,	//PID数据结构体
                 100,//积分误差限幅
                 (THR_INTE_LIM *10 - err_i_comp )*out_en			//integration limit，积分限幅
               );

  if(flag.taking_off == 1)
    LPF_1_(1.0f,dT_s,THR_START *10,err_i_comp);//err_i_comp = THR_START *10;
  else
    err_i_comp = 0;



  loc_ctrl_1.out[Z] = out_en *(alt_val_1.out + err_i_comp);

  loc_ctrl_1.out[Z] = LIMIT(loc_ctrl_1.out[Z],0,MAX_THR_SET *10);

  mc.ct_val_thr = loc_ctrl_1.out[Z];
}

