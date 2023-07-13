#ifndef _UWB_TASK_H_
#define	_UWB_TASK_H_


#include "FreeRTOS.h"
#include "task.h"

#include "Ano_DT.h"

#include "string.h"
#include "Drv_Uart.h" 
#include "nlink_linktrack_tagframe0.h"
  
 
extern  uint32_t totalFrameCount  ;
extern  uint32_t errorFrameCount  ;
void uwb_update_task(void *pvParameters);
 

#endif


