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
  
 
extern int32_t satrtPos[2];
extern  uint32_t totalFrameCount , errorFrameCount ;

extern uint8_t useUwb ; 
void uwb_update_task(void *pvParameters);
 

#endif


