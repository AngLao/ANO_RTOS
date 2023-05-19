#ifndef __DRV_OPENMV_H
#define __DRV_OPENMV_H

//==引用
#include "sysconfig.h"
#include "Ano_FcData.h"

//==定义
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
extern _openmv_data_st opmv2g;
//==函数声明

//static
static void OpenMV_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV_Check_Reset(void);

static void OpenMV2GND_Data_Analysis(u8 *buf_data,u8 len);
static void OpenMV2G_Check_Reset(void);

//public
void OpenMV_Offline_Check(u8 dT_ms);
void OpenMV_Byte_Get(u8 bytedata);


void OpenMV2G_Offline_Check(u8 dT_ms);
void OpenMV2GND_Byte_Get(u8 bytedata);

/////////////////////////////////////
extern u8 is_t;
#endif

