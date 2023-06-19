#ifndef _FLY_CTRL_H
#define	_FLY_CTRL_H

#include "sysconfig.h" 
 
//public
void FlyCtrlDataAnl(u8 *data);
void FlyCtrl_Task(u8 dT_ms);
void PcCtrl(u8 dT_ms);

//user
unsigned char broadcasting_Task(unsigned char dT_ms);
unsigned char UWBTest_Task(unsigned char dT_ms);


extern u8 use_line_opmv;

#endif

