#include "Ano_FlightDataCal.h"
#include "Ano_Imu.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Drv_spl06.h"
#include "Drv_ak8975.h"
#include "Ano_MotionCal.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_FlightCtrl.h"
#include "Drv_led.h"
#include "Ano_OF.h"
#include "Drv_Laser.h"

#include "FreeRTOS.h"
#include "task.h"


void Aux_read(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*读取电子罗盘磁力计数据*/
    Drv_AK8975_Read();
    /*读取气压计数据*/
//    baro_height = (s32)Drv_Spl0601_Read();

    vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/50);
  }
}



extern s32 sensor_val_ref[];

static u8 reset_imu_f;
void imu_update(u8 dT_ms)
{
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

static s16 mag_val[3];
void Mag_Update_Task(u8 dT_ms)
{

  Mag_Get(mag_val);

  Mag_Data_Deal_Task(dT_ms,mag_val,imu_data.z_vec[Z],sensor.Gyro_deg[X],sensor.Gyro_deg[Z]);

}


 
float wcz_acc_use;

void wcz_acc_update(void)//最小周期
{
  wcz_acc_use += 0.03f *(imu_data.w_acc[Z] - wcz_acc_use);
}



s32 baro_height;  
s16 ref_tof_height;  

void WCZ_Fus_Task(u8 dT_ms)
{   
	//TOF或者OF硬件正常
  if((sens_hd_check.of_df_ok || sens_hd_check.of_ok)) { 
		//使用光流的激光测距 
		if(switchs.of_tof_on)  
			ref_tof_height = jsdata.of_alt ; 
		//使用激光测距模块 	
    else if(switchs.tof_on)  
      ref_tof_height = -1;
		else
			ref_tof_height = -1;
   
		
		//世界z方向高度信息融合
		WCZ_Data_Calc(dT_ms,(s32)wcz_acc_use,(s32)(ref_tof_height));
  }  


}


