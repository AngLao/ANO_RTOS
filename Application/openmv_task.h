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
//寻找色块的x轴偏差
#define POS_X_ERROR (0x06)
//寻找色块的y轴偏差
#define POS_Y_ERROR (0x07)

typedef struct {
	uint32_t angle ;
	uint32_t area ;
	uint32_t width ;
	uint32_t pos ;
	uint32_t res ;
	uint32_t pos_x_error;
	uint32_t pos_y_error;
}openmv_t ;
 
typedef struct
{
	//
	u8 color_flag;
	u8 sta;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_color_block_st;

typedef struct
{
	//
	u8 sta;	
	s16 angle;
	s16 deviation;
	u8 p_flag;
	s16 pos_x;
	s16 pos_y;
	u8 dT_ms;

}_openmv_line_tracking_st;

typedef struct
{
	u8 offline;
	u8 mode_cmd;
	u8 mode_sta;
	//
	_openmv_color_block_st cb;
	_openmv_line_tracking_st lt;
}_openmv_data_st;
//==数据声明
extern _openmv_data_st opmv;

extern uint8_t useOpenmv;
extern openmv_t mvValue;
extern float openmvSpeedOut[];

void openmv_update_task(void *pvParameters);
 

#endif


