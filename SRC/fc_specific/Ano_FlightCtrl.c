#include "Ano_FlightCtrl.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Drv_led.h"
#include "rc_update.h"
#include "Drv_laser.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_DT.h"
#include "Ano_LED.h"
#include "Ano_ProgramCtrl_User.h"
#include "Ano_FlightDataCal.h"


/*============================================================================
更新：
201908012059-Jyoun：修正因高度失效判定光流失效的条件bug，以更好兼容超声波。



===========================================================================*/

/*PID参数初始化*/
void All_PID_Init(void)
{
  /*姿态控制，角速度PID初始化*/
  Att_1level_PID_Init();

  /*姿态控制，角度PID初始化*/
  Att_2level_PID_Init();

  /*高度控制，高度速度PID初始化*/
  Alt_1level_PID_Init();

  /*高度控制，高度PID初始化*/
  Alt_2level_PID_Init();

  /*位置速度控制PID初始化*/
  Loc_1level_PID_Init();
}

static u16 one_key_taof_start;
/*一键起飞任务（主要功能为延迟）*/
void one_key_take_off_task(u16 dt_ms)
{
  if(one_key_taof_start != 0) {
    one_key_taof_start += dt_ms;


    if(one_key_taof_start > 1400 && flag.motor_preparation == 1) {
      one_key_taof_start = 0;
      if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL) {
        flag.auto_take_off_land = AUTO_TAKE_OFF;
        //解锁、起飞

        flag.taking_off = 1;
      }

    }
  }
  //reset
  if(flag.unlock_sta == 0) {
    one_key_taof_start = 0;
  }

}
/*一键起飞*/
void one_key_take_off()
{
  if(flag.unlock_err == 0) {
    if(flag.auto_take_off_land == AUTO_TAKE_OFF_NULL && one_key_taof_start == 0) {
      one_key_taof_start = 1;
      flag.unlock_cmd = 1;
    }
  }
}
/*一键降落*/
void one_key_land()
{
  flag.auto_take_off_land = AUTO_LAND;
}


_flight_state_st fs;

static s16 landing_cnt;


/*降落检测*/

void land_discriminat(s16 dT_ms)
{
  static s16 ld_delay_cnt ;

  /*油门归一值小于0.1  或者启动自动降落*/
  if((fs.speed_set_h_norm[Z] < 0.1f) || flag.auto_take_off_land == AUTO_LAND) {
    if(ld_delay_cnt>0) {
      ld_delay_cnt -= dT_ms;
    }
  } else {
    ld_delay_cnt = 200;
  }

  /*意义是：如果向上推了油门，就需要等垂直方向加速度小于200cm/s2 保持200ms才开始检测*/
  if(ld_delay_cnt <= 0 && (flag.thr_low || flag.auto_take_off_land == AUTO_LAND) ) {
    /*油门最终输出量小于250持续1秒，认为着陆，然后上锁*/
    if(mc.ct_val_thr<250 && flag.unlock_sta == 1) { //还应当 与上速度条件，速度小于正20厘米每秒。
      if(landing_cnt<1500) {
        landing_cnt += dT_ms;
      } else {
        flag.taking_off = 0;
        landing_cnt =0;
        flag.unlock_cmd =0;

        debugOutput("Landing lock");
      }
    } else {
      landing_cnt = 0;
    }
  } else {
    landing_cnt  = 0;
  }

}


/*飞行状态任务*/
void Flight_State_Task(u8 dT_ms,const s16 *CH_N)
{
  static float max_speed_lim,vel_z_tmp[2];

  //设置油门摇杆量
  fs.speed_set_h_norm[Z] = CH_N[CH_THR] * 0.0023f;
  fs.speed_set_h_norm_lpf[Z] += 0.5f *(fs.speed_set_h_norm[Z] - fs.speed_set_h_norm_lpf[Z]);

  //解锁状态下,推油门起飞
  if(flag.unlock_sta && fs.speed_set_h_norm[Z]>0.01f && flag.motor_preparation == 1)
    flag.taking_off = 1;

  fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
  fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;

  if( flag.taking_off ) {
    //遥控设置的垂直方向速度
    if(fs.speed_set_h_norm[Z]>0)
      vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_UP);
    else
      vel_z_tmp[0] = (fs.speed_set_h_norm_lpf[Z] *MAX_Z_SPEED_DW);


    //飞控系统Z速度目标量综合设定
    vel_z_tmp[1] = vel_z_tmp[0] + pc_user.vel_cmps_set_z;
    //Z速度目标量限幅
    vel_z_tmp[1] = LIMIT(vel_z_tmp[1],fc_stv.vel_limit_z_n,fc_stv.vel_limit_z_p);
    //限制增量幅度
    fs.speed_set_h[Z] += LIMIT((vel_z_tmp[1] - fs.speed_set_h[Z]),-0.8f,0.8f);
  } else {
    fs.speed_set_h[Z] = 0 ;
  }

  //X Y轴速度设置
  float speed_set_tmp[2];
  //速度设定量，正负参考ANO坐标参考方向
  fs.speed_set_h_norm[X] = (my_deadzone(+CH_N[CH_PIT],0,50) *0.0022f);
  fs.speed_set_h_norm[Y] = (my_deadzone(-CH_N[CH_ROL],0,50) *0.0022f);

  LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[X],fs.speed_set_h_norm_lpf[X]);
  LPF_1_(3.0f,dT_ms*1e-3f,fs.speed_set_h_norm[Y],fs.speed_set_h_norm_lpf[Y]);

  max_speed_lim = MAX_SPEED;

  if(switchs.of_flow_on) {
    max_speed_lim = 1.5f *wcz_hei_fus.out;
    max_speed_lim = LIMIT(max_speed_lim,50,150);
  }

  fc_stv.vel_limit_xy = max_speed_lim;

  //飞控系统XY速度目标量综合设定
  speed_set_tmp[X] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[X] + pc_user.vel_cmps_set_h[X];
  speed_set_tmp[Y] = fc_stv.vel_limit_xy *fs.speed_set_h_norm_lpf[Y] + pc_user.vel_cmps_set_h[Y];

  length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],fc_stv.vel_limit_xy,fs.speed_set_h_cms);

  fs.speed_set_h[X] = fs.speed_set_h_cms[X];
  fs.speed_set_h[Y] = fs.speed_set_h_cms[Y];

  //倾斜过大上锁
  if(imu_data.z_vec[Z]<0.25f && flag.unlock_cmd != 0) {
    //
    if(mag.mag_CALIBRATE==0) {
      imu_state.G_reset = 1;
    }
    flag.unlock_cmd = 0;

    debugOutput("Rollover locks");
  }

  //校准中，复位重力方向*/
  if(sensor.gyr_CALIBRATE != 0 || sensor.acc_CALIBRATE != 0 ||sensor.acc_z_auto_CALIBRATE) {
    imu_state.G_reset = 1;
  }

  /*复位重力方向时，认为传感器失效*/
  if(imu_state.G_reset == 1) {
    flag.sensor_imu_ok = 0;
    LED_STA.rst_imu = 1;
    //复位高度数据融合
    wcz_fus_reset();
  } else if(imu_state.G_reset == 0 && flag.sensor_imu_ok == 0) {
    flag.sensor_imu_ok = 1;
    LED_STA.rst_imu = 0;
    debugOutput("IMU OK");
  }

  //飞行状态复位
  if(flag.unlock_sta == 0) {
    landing_cnt = 0;
    flag.taking_off = 0;

    flag.rc_loss_back_home = 0;
  }
}

_judge_sync_data_st jsdata;

//光流,激光测距传感器检测
void sensor_detection(u8 dT_ms)
{
  switchs.of_flow_on = 0;
  switchs.of_tof_on = 0;

  //匿名光流
  if(sens_hd_check.of_ok) {
    jsdata.of_qua = OF_QUALITY;
    jsdata.of_alt = (s32)OF_ALT;

    //匿名光流上的激光测距传感器状态
    if(jsdata.of_alt != -1)
      //高度数据有效
      switchs.of_tof_on = 1;
    else
      switchs.of_tof_on = 0;

    //其他光流
  } else if(sens_hd_check.of_df_ok) {
    jsdata.of_qua = of_rdf.quality;
    jsdata.of_alt = Laser_height_cm;

    //其他测距传感器状态
    if(jsdata.of_alt < 400)
      //高度数据有效
      switchs.of_tof_on = 1;
    else
      switchs.of_tof_on = 0;
  }

  static u8 of_quality_ok, of_quality_delay;
  //光流质量大于50，认为光流可用
  if(jsdata.of_qua>50 ) {
    if(of_quality_delay>200)
      of_quality_ok = 1;
    else
      of_quality_delay += dT_ms;
  } else {
    of_quality_delay =0;
    of_quality_ok = 0;
  }

  //根据光流质量以及飞行模式判定光流传感器状态
  if(flag.flight_mode == LOC_HOLD && of_quality_ok)
    switchs.of_flow_on = 1;
  else
    switchs.of_flow_on = 0;


}

