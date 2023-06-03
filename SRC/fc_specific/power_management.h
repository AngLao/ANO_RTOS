#ifndef _POWER_MANAGEMENT_H_
#define _POWER_MANAGEMENT_H_

#include "Drv_adc.h"
#include "Ano_Parameter.h"
#include "Ano_Filter.h"
#include "Drv_led.h"
#include "Ano_Math.h"
#include "Ano_LED.h"
 

void batteryUpdate(void); 

float getBatteryVoltage(void);


#endif

