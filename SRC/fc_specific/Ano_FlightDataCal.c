#include "Ano_FlightDataCal.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Drv_ak8975.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Ano_OF.h"
#include "Drv_Laser.h"

#include "FreeRTOS.h"
#include "task.h"

static s16 mag_val[3];

void Mag_Update_Task(u8 dT_ms)
{

  Mag_Get(mag_val);

  Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);

}

extern s32 sensor_val_ref[];

void imu_update(u8 dT_ms)
{
  static u8 reset_imu_f;
  //如果准备飞行，复位重力复位标记和磁力计复位标记
  if(flag.unlock_sta ) {
    imu_state.G_reset = imu_state.M_reset = 0;
    reset_imu_f = 0;
  } else {
    if(reset_imu_f==0 ) {
      imu_state.G_reset = 1;//自动复位
      sensor.gyr_CALIBRATE = 2;//校准陀螺仪，不保存
      reset_imu_f = 1;     //已经置位复位标记
    }
  }

  /*设置重力互补融合修正kp系数*/
  imu_state.gkp = 0.2f;//0.4f;
  /*设置重力互补融合修正ki系数*/
  imu_state.gki = 0.01f;
  /*设置罗盘互补融合修正ki系数*/
  //imu_state.mkp = 0.1f;
  imu_state.mkp = 0;

  /*磁力计修正使能选择*/
  imu_state.M_fix_en = sens_hd_check.mag_ok;

  /*姿态计算，更新，融合*/
  IMU_update(dT_ms *1e-3f, &imu_state,sensor.Gyro_rad, sensor.Acc_cmss, mag.val,&imu_data);

}

float wcz_acc_use;

void wcz_acc_update(void)//最小周期
{
  wcz_acc_use += 0.03f *(imu_data.w_acc[Z] - wcz_acc_use);
}


static _inte_fix_filter_st wcz_acc_fus;
_fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

#define N_TIMES 5

void wcz_fus_update(u8 dT_ms)
{
  static s32 ref_height_old,ref_speed_old;
  static s32 wcz_ref_speed,wcz_ref_acc;
 
  static u8 cyc_xn;
  float hz,ntimes_hz;
  hz = safe_div(1000,dT_ms,0);
  ntimes_hz = hz/N_TIMES;

  s32 ref_height = jsdata.of_alt; 

  cyc_xn ++;
  cyc_xn %= N_TIMES;

  if(cyc_xn == 0) {
    wcz_ref_speed = (ref_height - ref_height_old) *ntimes_hz;

    wcz_ref_acc = (wcz_ref_speed - ref_speed_old) *ntimes_hz;

    ref_height_old = ref_height;
    ref_speed_old = wcz_ref_speed;

  } 
	
  wcz_acc_fus.fix_ki = 0.1f;
  wcz_acc_fus.in_est = wcz_acc_use;
  wcz_acc_fus.in_obs = wcz_ref_acc;
  wcz_acc_fus.ei_limit = 100;
  inte_fix_filter(dT_ms*1e-3f,&wcz_acc_fus);


  wcz_spe_fus.fix_kp = 0.6f;
  wcz_spe_fus.in_est_d = wcz_acc_fus.out;
  wcz_spe_fus.in_obs = wcz_ref_speed;
  wcz_spe_fus.e_limit = 100;
  fix_inte_filter(dT_ms*1e-3f,&wcz_spe_fus);


  wcz_hei_fus.fix_kp = 0.3f;
  wcz_hei_fus.in_est_d = wcz_spe_fus.out;
  wcz_hei_fus.in_obs = ref_height;
  //wcz_hei_fus.e_limit = 200;
  fix_inte_filter(dT_ms*1e-3f,&wcz_hei_fus);
 
}


void wcz_fus_reset()
{
  wcz_acc_fus.out = 0;
  wcz_acc_fus.ei = -wcz_acc_use;

  wcz_spe_fus.out = 0;
  wcz_spe_fus.e = 0;

  wcz_hei_fus.out = 0;
  wcz_hei_fus.e = 0;

}

