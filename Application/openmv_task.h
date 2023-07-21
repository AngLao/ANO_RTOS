#ifndef _OPENMV_TASK_H_
#define	_OPENMV_TASK_H_


#include "FreeRTOS.h"
#include "task.h"

#include "Ano_DT.h"
#include "Ano_ProgramCtrl_User.h"
 
#include "Drv_Uart.h"  
   
	
extern uint32_t angleValue , areaValue , widthValue , posValue , resValue; 

void openmv_update_task(void *pvParameters);
 

#endif


