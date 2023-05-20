//==引用
#include "Ano_UWB.h"
#include "Drv_Uart.h"
#include "ring_buffer.h"
#include "nlink_utils.h"
#include "nlink_linktrack_tagframe0.h"




/**********************************************************************************************************
*函 数 名: UWB_Get_Data_Task
*功能说明: UWB数据获取任务
*参    数: 周期（毫秒）
*返 回 值: 无
**********************************************************************************************************/
static u16 uwb_check_time;
void UWB_Get_Data_Task(void)
{
  //读环形缓冲区
  static  int RingBufferDataLen = 0;
  static unsigned char pData[128 * 5];
  RingBufferDataLen = RingBuffer_GetCount(&U1rxring) ;

  //解析uwb数据
  if(RingBufferDataLen) {
    RingBuffer_PopMult(&U1rxring, pData, RingBufferDataLen);

    if (g_nlt_tagframe0.UnpackData(pData, RingBufferDataLen)) {
      return;
    }
  }

}
