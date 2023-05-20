#ifndef _ANO_UWB_H_
#define _ANO_UWB_H_

typedef struct {
  unsigned char init_ok;
  unsigned char online;

  float ref_dir[2];
  float raw_data_loc[3];
  float raw_data_vel[3];
  float w_dis_cm[3];
  float w_vel_cmps[3];

} _uwb_data_st;

void UWB_Get_Data_Task(void);


#endif





