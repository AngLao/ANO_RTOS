#ifndef _OPENMV_TASK_H_
#define	_OPENMV_TASK_H_


#include "FreeRTOS.h"
#include "task.h"

#include "Ano_DT.h"
#include "Ano_ProgramCtrl_User.h"
 
#include "Drv_Uart.h"  

//修正偏航
#define ANGLE_ID (0x01)
//面积
#define AREA_ID (0x02)
//宽度
#define WIDTH_ID (0x03)
//图像位置
#define POS_ID (0x04)
//二维码识别结果
#define RES_ID (0x05)

typedef struct {
	uint32_t angle ;
	uint32_t area ;
	uint32_t width ;
	uint32_t pos ;
	uint32_t res ;
}openmv_t ;
 
extern uint8_t useOpenmv;
extern openmv_t mvValue;
extern float openmvSpeedOut[];

void openmv_update_task(void *pvParameters);
 

#endif


