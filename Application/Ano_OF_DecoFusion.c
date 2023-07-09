
/*==========================================================================
 * 描述    ：对优像光流(UP_OF)传回的数据进行处理，并解除因机体俯仰、横滚旋转
             而造成光流输出的耦合，也称作“旋转解耦”或“旋转补偿”。然后与高度
						 进行换算得到地面速度，再与加速度计测量数据进行融合，得到更稳定
						 的地面速度输出。

 * 更新时间：2019-07-13
 * 作者		 ：匿名科创-Jyoun
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。
============================================================================
更新：

201908012354-Jyoun：增加优像光流朝上的光流解算程序。
本版备注：优像光流安装方向还是接线方朝左（俯视飞机），需要先设定参照物的高度。
===========================================================================*/

//默认引用
#include "Ano_OF_DecoFusion.h"
#include "Ano_IMU.h"
#include "Ano_Math.h"
#include "Ano_Filter.h"

//数据接口定义：
//=========mapping===============
//需要引用的文件：
#include "Ano_Sensor_Basic.h"
#include "Drv_UP_Flow.h"
#include "Drv_laser.h"
#include "Ano_OF.h"
 

//需要调用引用的外部变量： 
#define BUF_UPDATE_CNT            (of_buf_update_cnt)
#define OF_DATA_BUF               (OF_DATA)

//使用机体惯性数据
//#define RADPS_X                   (sensor.Gyro_rad[X])
//#define RADPS_Y                   (sensor.Gyro_rad[Y])
//#define ACC_X                   	(imu_data.w_acc[X])
//#define ACC_Y                   	(imu_data.w_acc[Y])

//使用光流模块惯性数据
#define RADPS_X                   (OF_GYR_X)
#define RADPS_Y                   (OF_GYR_Y)
#define ACC_X                   	(OF_ACC_X)
#define ACC_Y                   	(OF_GYR_Y)

#define RELATIVE_HEIGHT_CM        (Laser_height_cm)  //相对高度
//需要操作赋值的外部变量：


//===============================
//全局变量：
u8 of_buf_update_flag;
_of_data_st of_data;
_of_rdf_st of_rdf;
float of_rot_d_degs[2];

float of_fus_err[2], of_fus_err_i[2];
//参数设定：
#define UPOF_PIXELPDEG_X    160.0f       //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。//
#define UPOF_PIXELPDEG_Y    160.0f       //每1角度对应的像素个数，与分辨率和焦距有关，需要调试标定。
#define UPOF_CMPPIXEL_X     0.00012f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。//目前粗略标定
#define UPOF_CMPPIXEL_Y     0.00012f     //每像素对应的地面距离，与焦距和高度有关，需要调试标定。
#define FUS_KP              5.0f
#define FUS_KI              1.0f
//
#define UPOF_UP_DW          0             //0:朝下；1：朝上
#define OBJREF_HEIGHT_CM    280           //参照物高度，厘米;光流朝上才有用。

/**********************************************************************************************************
*函 数 名: ANO_OFDF_Task
*功能说明: 匿名科创光流解耦合与融合任务
*参    数: 周期时间(ms)
*返 回 值: 无
**********************************************************************************************************/
void ANO_OFDF_Task(u8 dT_ms)
{ 
  OF_State(); 
	
  ANO_OF_Decouple();
	
  ANO_OF_Fusion(&dT_ms, (s32)RELATIVE_HEIGHT_CM);


}




/*根据飞机陀螺仪数据解算出相应姿态以解耦光流数据*/
void OF_INS_Get(float dT_s)
{ 
  static float rad_ps_lpf[2];
	
  //低通滤波
  //故意滞后为了对齐相位，这里换成零阶保持+FIFO效果更好。
  LPF_1_(5.0f, dT_s, RADPS_X, rad_ps_lpf[0]);
  LPF_1_(5.0f, dT_s, RADPS_Y, rad_ps_lpf[1]);
//	rad_ps_lpf[0] += 0.2f *(rad_ps_x - rad_ps_lpf[0]);
//	rad_ps_lpf[1] += 0.2f *(rad_ps_y - rad_ps_lpf[1]);
	
	//匿名光流直接输出角度
  of_rot_d_degs[0] = rad_ps_lpf[0] ;
  of_rot_d_degs[1] = rad_ps_lpf[1];

	
//  of_rot_d_degs[0] = rad_ps_lpf[0] * DEG_PER_RAD ;
//  of_rot_d_degs[1] = rad_ps_lpf[1] * DEG_PER_RAD ;
	
  //低通滤波
  LPF_1_(5.0f, dT_s, ACC_X , of_rdf.gnd_acc_est_w[X]);
  LPF_1_(5.0f, dT_s, ACC_Y , of_rdf.gnd_acc_est_w[Y]);

 
	//融合估计部分（此处对加速度直接积分获取地速）
	of_rdf.gnd_vel_est_w[X] += of_rdf.gnd_acc_est_w[X] * (dT_s);
	of_rdf.gnd_vel_est_w[Y] += of_rdf.gnd_acc_est_w[Y] * (dT_s);
  
}

/**********************************************************************************************************
*函 数 名: ANO_OF_Data_Get
*功能说明: 匿名科创光流数据获取
*参    数: 周期时间(s，形参),光流数据缓存（形参）
*返 回 值: 无
**********************************************************************************************************/
void ANO_OF_Data_Get(u8 dT_ms, u8 *of_data_buf)
{
  static float offline_delay_time_s;
  u8 XOR = 0;

  if(of_buf_update_flag != BUF_UPDATE_CNT) {
    //
    of_buf_update_flag = BUF_UPDATE_CNT;
    //
    XOR = of_data_buf[2];

    for(u8 i = 3; i < 12; i++) {
      XOR ^= of_data_buf[i];
    }

    //
    if(XOR == of_data_buf[12]) {
      //
      of_data.updata ++;
      //
      of_data.valid = of_data_buf[10];

      //
      if(of_data.valid != 0xf5) {
        //
        of_data.flow_x_integral = of_data.flow_y_integral = 0;
      } else {
        //
        of_data.flow_x_integral = (s16)(of_data_buf[4] | (of_data_buf[5] << 8)); //org_y
        //
        of_data.flow_y_integral = (s16)(of_data_buf[2] | (of_data_buf[3] << 8)); //org_x
      }

      //
      of_data.it_ms = ((u16)(of_data_buf[6] | (of_data_buf[7] << 8))) / 1000;

    }

    //
    offline_delay_time_s = 0;
    of_data.online = 1;
  } else {
    //null
    if(offline_delay_time_s < 1000) {
      offline_delay_time_s += dT_ms;
    } else { //掉线
      of_data.online = 0;
    }
  }
}


/**********************************************************************************************************
*函 数 名: ANO_OF_Decouple
*功能说明: 匿名科创光流解耦合
*参    数: 周期时间(形参ms)
*返 回 值: 无
*备    注: 建议20ms调用一次
**********************************************************************************************************/
static void ANO_OF_Decouple(void)
{

  if(of_data.valid != 0xf5) {
    //
    of_rdf.of_vel[X] = of_rdf.of_vel[Y] = 0;

    //quality
    if(of_rdf.quality >= 5) {
      of_rdf.quality -= 5;
    }
  } else {
    //
    if(UPOF_UP_DW == 0) {
      of_rdf.of_vel[X] = (1000 / of_data.it_ms * of_data.flow_x_integral + UPOF_PIXELPDEG_X * of_rot_d_degs[Y] );
      of_rdf.of_vel[Y] = (1000 / of_data.it_ms * of_data.flow_y_integral - UPOF_PIXELPDEG_Y * of_rot_d_degs[X] );
    } else {
      of_rdf.of_vel[X] = -(1000 / of_data.it_ms * of_data.flow_x_integral + UPOF_PIXELPDEG_X * of_rot_d_degs[Y] );
      of_rdf.of_vel[Y] =  (1000 / of_data.it_ms * of_data.flow_y_integral + UPOF_PIXELPDEG_Y * of_rot_d_degs[X] );
    }

    //quality
    if(of_rdf.quality <= 250) {
      of_rdf.quality += 5;
    }
  }

}

/**********************************************************************************************************
*函 数 名: ANO_OF_Decoupling
*功能说明: 匿名科创光流解耦合
*参    数: 周期时间(形参ms),参考高度(cm)
*返 回 值: 无
**********************************************************************************************************/
static void ANO_OF_Fusion(u8 *dT_ms, s32 ref_height_cm)
{
  float dT_s = (*dT_ms) * 1e-3f;

  if(UPOF_UP_DW == 0) {
    of_rdf.of_ref_height = LIMIT(ref_height_cm, 20, 500); //限制到20cm-500cm
  } else {
    of_rdf.of_ref_height = LIMIT((OBJREF_HEIGHT_CM - ref_height_cm), 20, 500); //限制到20cm-500cm
  }

  of_rdf.gnd_vel_obs_h[X] = UPOF_CMPPIXEL_X * of_rdf.of_vel[X] * of_rdf.of_ref_height;

  of_rdf.gnd_vel_obs_h[Y] = UPOF_CMPPIXEL_Y * of_rdf.of_vel[Y] * of_rdf.of_ref_height;

  h2w_2d_trans(of_rdf.gnd_vel_obs_h, imu_data.hx_vec, of_rdf.gnd_vel_obs_w);
 
  switch(of_rdf.state) {
  case 0:
    of_rdf.state = 1; 
    OF_INS_Reset(); 
    break;

  case 1: 
    //(这里开源最简单并且好用的PI互补融合，注意这里修正即取低频，取高频的估计的部分不在此处)

    //融合修正部分 原始值无效时不修正
    if(of_data.valid == 0xf5) {
      //
      for(u8 i = 0; i < 2; i++) {
        //
        of_fus_err[i] = of_rdf.gnd_vel_obs_w[i] - of_rdf.gnd_vel_est_w[i];
        //
        of_fus_err_i[i] += FUS_KI * of_fus_err[i] * dT_s;
        of_fus_err_i[i] = LIMIT(of_fus_err_i[i], -100, 100);
        //
        of_rdf.gnd_vel_est_w[i] += (of_fus_err[i] * FUS_KP + of_fus_err_i[i]) * dT_s;

      }
    }

    w2h_2d_trans(of_rdf.gnd_vel_est_w, imu_data.hx_vec, of_rdf.gnd_vel_est_h);
 
    break;

  default: 
    OF_INS_Reset(); 
    break;
  }
}

/**********************************************************************************************************
*函 数 名: OF_INS_Reset
*功能说明: 光流融合复位
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void OF_INS_Reset()
{
  for(u8 i = 0; i < 2; i++) {
    //
    of_rdf.gnd_vel_est_w[i] = 0;
    //
    of_fus_err_i[i] = 0;

  }
}

/**********************************************************************************************************
*函 数 名: OF_State
*功能说明: 光流状态处理
*参    数: 无
*返 回 值: 无
**********************************************************************************************************/
static void OF_State()
{
  if(imu_state.G_reset && of_rdf.state == 1 )
      of_rdf.state = 0; 
	else { 
		if(of_rdf.quality > 200)  
      of_rdf.state = 1; 
		else  
      of_rdf.state = 0; 
  }

  if(of_data.online && ultra.measure_ok ) {  
    sens_hd_check.of_df_ok = 1;
  } else {
    sens_hd_check.of_df_ok = 0;
  }
}

