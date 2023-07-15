#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H


#include "Ano_FcData.h"
#include "Ano_Pid.h"


typedef struct {
  int32_t ct_val_rol;
  int32_t ct_val_pit;
  int32_t ct_val_yaw;
  int32_t ct_val_thr;
} _mc_st;

extern _mc_st mc;

extern int16_t motor[MOTORSNUM];
 
void power_distribution(uint8_t dT_ms);

#endif

