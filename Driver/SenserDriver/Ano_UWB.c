//==引用
#include "Ano_UWB.h"
#include "Drv_Uart.h"
#include "ring_buffer.h"
#include "nlink_utils.h"
#include "nlink_linktrack_tagframe0.h"

#include "Ano_Imu.h"
#include "Ano_FcData.h"

#define fc_sta flag

//==数据声明
_uwb_data_st uwb_data;
  

/**********************************************************************************************************
*函 数 名: UWB_Get_Data_Task                         
*功能说明: UWB数据获取任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
static u16 uwb_check_time;
void UWB_Get_Data_Task(u8 dT_ms)
{ 
	//读环形缓冲区 
	static  int RingBufferDataLen = 0;
	static unsigned char pData[128*5]; 
	RingBufferDataLen = RingBuffer_GetCount(&U1rxring) ;
	//解析uwb数据
	if(RingBufferDataLen){
		RingBuffer_PopMult(&U1rxring, pData,RingBufferDataLen);
		if (g_nlt_tagframe0.UnpackData(pData, RingBufferDataLen))
		{
//			printf("x:%d  y:%d\r\n",(int)g_nlt_tagframe0.result.pos_3d[0]
//														 ,(int)g_nlt_tagframe0.result.pos_3d[1] );

//			uwb_data.raw_data_loc[1] = -(float)(s16)((*(UWB_RxBuffer+6)<<8)|*(UWB_RxBuffer+7)) / 100;
//			uwb_data.raw_data_loc[0] =  (float)(s16)((*(UWB_RxBuffer+8)<<8)|*(UWB_RxBuffer+9)) / 100;
//			uwb_data.raw_data_loc[2] =  (float)(s16)((*(UWB_RxBuffer+10)<<8)|*(UWB_RxBuffer+11)) / 100;
//			uwb_data.raw_data_vel[1] = -(float)(s16)((*(UWB_RxBuffer+12)<<8)|*(UWB_RxBuffer+13)) / 100;
//			uwb_data.raw_data_vel[0] =  (float)(s16)((*(UWB_RxBuffer+14)<<8)|*(UWB_RxBuffer+15)) / 100;
//			uwb_data.raw_data_vel[2] =  (float)(s16)((*(UWB_RxBuffer+16)<<8)|*(UWB_RxBuffer+17)) / 100;
			//UWB在线
//			uwb_check_time = 0;	
//		  uwb_data.online = 1;		
			return;
		}  
	}
	
//	//UWB在线计时,若长时间未收到UWB数据着定为离线
//	if(uwb_check_time <1000){
//		uwb_check_time += dT_ms;
//	}else{
//		uwb_data.online = 0;
//	}
	
}

/**********************************************************************************************************
*函 数 名: Ano_UWB_Data_Calcu_Task
*功能说明: UWB数据计算任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
void Ano_UWB_Data_Calcu_Task(u8 dT_ms)
{
	//解锁前，记录当前机体X轴正方向在水平面投影为UWB坐标X轴正方向
	//要求解锁前，飞机放正，机体X轴正方向对准UWB坐标轴X轴正方向
	//记录参考方向
	if(!fc_sta.unlock_sta)
	{
		//
		uwb_data.init_ok = 0;
		//
		uwb_data.ref_dir[X] = imu_data.hx_vec[X];
		uwb_data.ref_dir[Y] = imu_data.hx_vec[Y];

	}
	//
	else
	{
		//
		uwb_data.init_ok = 1;
	}
	//参考方向转世界坐标（这里等同于地理坐标）
	h2w_2d_trans(uwb_data.raw_data_loc,uwb_data.ref_dir,uwb_data.w_dis_cm);
	//计算速度
	h2w_2d_trans(uwb_data.raw_data_vel,uwb_data.ref_dir,uwb_data.w_vel_cmps);
	
}
