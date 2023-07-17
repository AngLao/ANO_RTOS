#ifndef __WXY_CTRL_H
#define __WXY_CTRL_H

#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "Ano_Pid.h"

typedef struct {
  float exp[VEC_XYZ];

  float fb[VEC_XYZ];

  float out[VEC_XYZ];
} _loc_ctrl_st;

extern _loc_ctrl_st loc_ctrl_1;
extern _loc_ctrl_st loc_ctrl_2;

extern _PID_arg_st loc_arg_1[] ;
extern _PID_val_st loc_val_1[] ;
//位置速度环修正控制数据
extern _PID_val_st loc_val_1_fix[2] ;

void Loc_1level_PID_Init(void);
void Loc_1level_Ctrl(u16 dT_ms);

#endif
