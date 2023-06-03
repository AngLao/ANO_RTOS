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
 
//public
void FlyCtrlDataAnl(u8 *data);
void FlyCtrl_Task(u8 dT_ms);
void PcCtrl(u8 dT_ms);

//user
unsigned char broadcasting_Task(unsigned char dT_ms);
unsigned char UWBTest_Task(unsigned char dT_ms);


extern u8 use_line_opmv;

#endif

