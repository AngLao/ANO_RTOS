#ifndef __FLIGHT_DATA_COMP_H
#define __FLIGHT_DATA_COMP_H

#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
 
 
extern _fix_inte_filter_st wcz_spe_fus,wcz_hei_fus;

extern float wcz_acc_use;

void imu_update(u8 dT_ms);

void Mag_Update_Task(u8 dT_ms);

void wcz_acc_update(void);

void wcz_fus_update(u8 dT_ms);
  
void wcz_fus_reset(void);

#endif

