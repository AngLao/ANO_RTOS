#ifndef __ANO_OF_H_
#define __ANO_OF_H_

#include "sysconfig.h"

//光流信息质量：QUA
//光照强度：LIGHT
extern uint8_t 	OF_STATE, OF_QUALITY;
//原始光流信息，具体意义见光流模块手册
extern int8_t	OF_DX, OF_DY;
//融合后的光流信息，具体意义见光流模块手册
extern int16_t	OF_DX2, OF_DY2, OF_DX2FIX, OF_DY2FIX, OF_INTEG_X, OF_INTEG_Y;
//原始高度信息和融合后高度信息
extern uint32_t	OF_ALT, OF_ALT2;
//原始陀螺仪数据
extern int16_t	OF_GYR_X, OF_GYR_Y, OF_GYR_Z; 
//原始加速度数据
extern int16_t	OF_ACC_X, OF_ACC_Y, OF_ACC_Z; 
 
extern u8 of_init_type;

 
void AnoOF_GetOneByte(uint8_t data);
void AnoOF_DataAnl_Task(u8 dT_ms);
void AnoOF_Check(u8 dT_ms);
#endif
