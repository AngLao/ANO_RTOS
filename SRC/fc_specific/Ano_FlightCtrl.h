#ifndef __FLIGHT_CTRL_H
#define __FLIGHT_CTRL_H

#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
 

typedef struct {
  s16 alt_ctrl_speed_set;
  float speed_set_h[VEC_XYZ];
  float speed_set_h_cms[VEC_XYZ];

  float speed_set_h_norm[VEC_XYZ];
  float speed_set_h_norm_lpf[VEC_XYZ];

} _flight_state_st;
extern _flight_state_st fs;

typedef struct {
  u8 of_qua;
  s32 of_alt;
  u16 valid_of_alt_cm;

} _judge_sync_data_st;
extern _judge_sync_data_st jsdata;
 

void All_PID_Init(void); 
void land_discriminat(s16 dT_ms);
void Flight_State_Task(u8,const s16 *CH_N); 
void sensor_detection(u8 dT_ms);

#endif

