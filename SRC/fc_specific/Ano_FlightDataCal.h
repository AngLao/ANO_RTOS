#ifndef __FLIGHT_DATA_COMP_H
#define __FLIGHT_DATA_COMP_H

#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"

extern s32 baro_height,ref_height_get;
extern s16 ref_tof_height; 
 

void imu_update(u8 dT_ms);

void Mag_Update_Task(u8 dT_ms);

void wcz_acc_update(void);

void WCZ_Fus_Task(u8 dT_ms);
 
#endif

