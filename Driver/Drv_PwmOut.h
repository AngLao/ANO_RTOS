#ifndef _DRV_PWMOUT_H_
#define _DRV_PWMOUT_H_

#include "sysconfig.h"

void Drv_PwmOutInit(void);
void Drv_MotorPWMSet(uint8_t Motor, uint16_t PwmValue);
void Drv_HeatSet(u16 val);

//每0.1度的控制量
#define perDegree_90(val) (val*10*2.22f)
#define perDegree_180(val) (val*10*1.11f)
#define perDegree_270(val) (val*10*0.74f)
void gear_protocol_set(u16 num ,u16 val);


#endif
