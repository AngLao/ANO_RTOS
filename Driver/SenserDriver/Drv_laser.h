#ifndef _LASER_H_
#define _LASER_H_
#include "sysconfig.h"

//选择定高模块，三选一，其余注释

#define USE_KS103
//#define USE_US100
//#define USE_LASER

typedef struct {
  //s32 height;//cm

  //float relative_height;
  //float h_delta;

  float h_dt;

  u8 measure_ok;
  u8 measure_ot_cnt;
} _height_st;

extern _height_st ultra;


extern u8 LASER_LINKOK;
extern u16 Laser_height_cm;

u8 		Drv_Laser_Init(void);
void 	Drv_Laser_GetOneByte(u8 data);
void Ultra_Get(u8 com_data);
void Ultra_Duty(void);
#endif
