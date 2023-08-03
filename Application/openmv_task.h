#ifndef _OPENMV_TASK_H_
#define	_OPENMV_TASK_H_


#include "FreeRTOS.h"
#include "task.h"

#include "Ano_DT.h"
#include "Ano_ProgramCtrl_User.h"
 
#include "Drv_Uart.h"  
#include "Drv_PwmOut.h"

#define X_ID (0x01)
#define Y_ID (0x02)

typedef struct {
	uint32_t posX ;
	uint32_t posY ;
}openmv_t ;
 
extern uint8_t useOpenmv;
extern openmv_t mvValue;
extern float openmvSpeedOut[];

void openmv_update_task(void *pvParameters);
 

#endif


