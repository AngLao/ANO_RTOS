#include "Ano_MotionCal.h"
#include "Ano_Math.h"
#include "Drv_icm20602.h"
#include "Ano_Imu.h"
#include "Ano_MotorCtrl.h"
 

_inte_fix_filter_st wcz_acc_fus;
_fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

static s32 ref_height_old,ref_speed_old;

static s32 wcz_ref_speed,wcz_ref_acc;


static s32 wcz_acc;
#define N_TIMES 5

void WCZ_Data_Calc(u8 dT_ms,s32 wcz_acc_get,s32 ref_height)
{
  static u8 cyc_xn;
  float hz,ntimes_hz;
  hz = safe_div(1000,dT_ms,0);
  ntimes_hz = hz/N_TIMES;
 
  wcz_acc = wcz_acc_get;
	
  cyc_xn ++;
  cyc_xn %= N_TIMES;

  if(cyc_xn == 0) {
    wcz_ref_speed = (ref_height - ref_height_old) *ntimes_hz;

    wcz_ref_acc = (wcz_ref_speed - ref_speed_old) *ntimes_hz;

    ref_height_old = ref_height;
    ref_speed_old = wcz_ref_speed; 
  }
	
  wcz_acc_fus.fix_ki = 0.15f;
  wcz_acc_fus.in_est = wcz_acc;
  wcz_acc_fus.in_obs = wcz_ref_acc;
  wcz_acc_fus.ei_limit = 100;
  inte_fix_filter(dT_ms*1e-3f,&wcz_acc_fus);


  wcz_spe_fus.fix_kp = 0.3f;
  wcz_spe_fus.in_est_d = wcz_acc_fus.out;
  wcz_spe_fus.in_obs = wcz_ref_speed;
  wcz_spe_fus.e_limit = 100;
  fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);


	//融合高度滤波修正系数
  wcz_hei_fus.fix_kp = 0.3f;
	//融合高度数据滤波结果
  wcz_hei_fus.in_est_d = wcz_spe_fus.out;
	//融合高度数据滤波原始值
  wcz_hei_fus.in_obs = ref_height; 
	
  fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus); 

}


void WCZ_Data_Reset()
{
  wcz_acc_fus.out = 0;
  wcz_acc_fus.ei = -wcz_acc;

  wcz_spe_fus.out = 0;
  wcz_spe_fus.e = 0;

  wcz_hei_fus.out = 0;
  wcz_hei_fus.e = 0;

}
