#ifndef _UWB_TASK_H_
#define	_UWB_TASK_H_


#include "FreeRTOS.h"
#include "task.h"

#include "Ano_DT.h"
#include "Ano_ProgramCtrl_User.h"

#include <math.h>
#include <stdlib.h>
#include "string.h"
#include "Drv_Uart.h" 
#include "nlink_linktrack_tagframe0.h"

#include "Ano_OF.h"
#include "Ano_FcData.h"
#include "Ano_Filter.h"
#include "Ano_Math.h"
#include "Ano_FlightDataCal.h"

#include "rc_update.h"

typedef struct {
	uint16_t x;
	uint16_t y;
}dot_t;

typedef enum {
	cruise = 2,//巡航状态
	fixPoint	,//定位火源状态
	throwObject	,//投掷物体
	complete , //巡航结束
}taskStatus_enum;

extern	uint8_t	taskStatus_2023;
 
extern	uint8_t	dotfIndex;
extern	dot_t		dotPath[];

extern  uint32_t	totalFrameCount , errorFrameCount  ;

extern float uwbSpeedOut[2];

extern _fix_inte_filter_st posFus[2], speedFus[2];

 
int16_t getPosX(void);
int16_t getPosY(void);
void uwb_update_task(void *pvParameters);

#endif


