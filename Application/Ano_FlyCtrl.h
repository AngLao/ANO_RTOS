#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "sysconfig.h"
typedef struct {
  //sta
  u8 state_ok;
  u8 val;
  u8 update_f;
  u8 en;

} _onekey_ct_st;

extern _onekey_ct_st onekey;




typedef struct {
  //sta
  u8 state_ok;
  u8 cmd_state[2];
  //time
  u32 fb_process_t_ms[4];
  u32 exp_process_t_ms[4];

  //ctrl
  float ref_dir[2];
  float vel_cmps_ref[3];
  float vel_cmps_w[3];
  float vel_cmps_h[3];
  s16 yaw_pal_dps;

} _fly_ct_st;
extern _fly_ct_st program_ctrl;


//static
void FlyCtrlReset(void);

//public
void FlyCtrlDataAnl(u8 *data);
void FlyCtrl_Task(u8 dT_ms);
void PcCtrl(u8 dT_ms);

//user
unsigned char broadcasting_Task(unsigned char dT_ms);
unsigned char UWBTest_Task(unsigned char dT_ms);


extern u8 use_line_opmv;

#endif

