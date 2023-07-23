#include "light_flow_task.h"
 
 
static void unpack_data(void);
	
/* 光流数据更新及高度融合 */
void light_flow_task(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

	//初始化光流
	light_flow_init();
	
  while (1) { 
		
		unpack_data();
		
    /*位置传感器状态检测*/
    sensor_detection(5);
		
		/* 光流串口缓存有数据更新则组合数据，带掉线检测 */
		ANO_OF_Data_Get(5, OF_DATA);
		
    /* 光流掉线检测 */
    AnoOF_Check(5); 
 
    /* 匿名科创光流解耦合与融合任务 */
    ANO_OFDF_Task(5);
		
    /*高度数据融合任务*/
    wcz_fus_update(5);
		 
		
#if defined(USE_KS103)
    //超声波任务
    Ultra_Duty();
#endif
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 200);
  }
}
 

static void unpack_data(void)
{ 
  uint8_t RingBufferDataLen = RingBuffer_GetCount(&lightFlowRing) ;
	 
  for(uint8_t cnt = 0; cnt <RingBufferDataLen ; cnt++) {
    uint8_t data = 0;
    RingBuffer_Pop(&lightFlowRing, &data); 
 	
    //匿名光流解析
    if(of_init_type != 2)  
      AnoOF_GetOneByte(data);
		
		//优像光流解析
		else if(of_init_type!=1) 
			OFGetByte(data); 
		
  }
} 
		