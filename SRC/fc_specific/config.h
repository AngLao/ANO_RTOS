#ifndef __CONFIG_H
#define __CONFIG_H
 
#include "sysconfig.h" 

/***************换算******************/
#define ANGLE_TO_RADIAN 0.01745329f //*0.01745 = /57.3	角度转弧度  

//GYR_ACC_FILTER 参数大致范围参考
//500KV以下 0.15f~0.2f
//500~2000KV 0.2f~0.3f
//2000kv以上 0.3f-0.5f 

#define GYR_ACC_FILTER 0.22f //陀螺仪加速度计滤波系数

//FINAL_P 参数大致范围参考
//500KV以下 0.4f以上
//500~2000KV 0.4f~0.3f
//2000kv以上 0.3f-0.2f

#define FINAL_P 			 0.33f  //电机输出量比例系数

#define MOTOR_ESC_TYPE 1  //2：无刷电机带刹车的电调，1：无刷电机不带刹车的电调，
#define MOTORSNUM 4



#define MAX_ANGLE     25.0f 
 
#define MAX_ROLLING_SPEED 1600  //角度每秒

#define MAX_SPEED 500 //最大水平速度，厘米每秒 cm/s

#define MAX_Z_SPEED_UP 350 //厘米每秒 cm/s
#define MAX_Z_SPEED_DW 250 //厘米每秒 cm/s
 

#define CTRL_1_INTE_LIM 250 //角速度环积分限幅 ：输出
   

#define MAX_THR_SET    90  //最大油门百分比 %
#define THR_INTE_LIM_SET   70  //油门积分百分比 % 
 
#define THR_INTE_LIM   THR_INTE_LIM_SET/FINAL_P

#define THR_START      35  //油门起调量百分比 % 


#define BARO_FIX -0                          //气压速度积分修正起调值/CM厘米  


#endif


