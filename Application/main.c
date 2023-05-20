#include "FreeRTOS.h"
#include "task.h" 

#include "sysconfig.h"
#include "Drv_Bsp.h"
#include "Ano_Scheduler.h"
#include "Ano_FcData.h"

#include "Drv_Bsp.h"
#include "Drv_icm20602.h"
#include "Ano_LED.h"
#include "Ano_FlightDataCal.h"
#include "Ano_Sensor_Basic.h"

#include "Ano_DT.h"
#include "Ano_RC.h"
#include "Ano_Parameter.h"
#include "Drv_led.h"
#include "Drv_ak8975.h"
#include "Drv_spl06.h"
#include "Ano_FlightCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_MagProcess.h"
#include "Ano_Power.h"
#include "Ano_OF.h"
#include "Drv_heating.h"
#include "Ano_FlyCtrl.h"
#include "Ano_UWB.h" 
#include "Ano_OF_DecoFusion.h"
#include "Drv_laser.h"
#include "Drv_Uart.h"

#include "Ano_Sensor_Basic.h"
  
 
/* 基本传感器数据准备进程 该任务为精准进行的任务 执行频率精准1000Hz 优先级全局最高*/
void basic_data_read(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 
			
				if(flag.start_ok)
				{
					/*读取陀螺仪加速度计数据*/
					Drv_Icm20602_Read(); 
					
					/*惯性传感器数据准备*/
					Sensor_Data_Prepare(1);
					
					/*姿态解算更新*/
					IMU_Update_Task(1); 
					
					/*获取WC_Z加速度*/
					WCZ_Acc_Get_Task(); 
					
					/*飞行状态任务*/
					Flight_State_Task(1,CH_N);
					
					/*开关状态任务*/
					Swtich_State_Task(1);
					
					/*光流融合数据准备任务*/
					ANO_OF_Data_Prepare_Task(0.001f);
				}	
			 
				
				
				//灯光驱动
				LED_1ms_DRV();
				
				
        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/1000);
    }
} 


/* 姿态角速度环控制进程 该任务为精准进行的任务 执行频率精准500Hz 优先级第二*/
void inner_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 

				/*姿态角速度环控制*/
				Att_1level_Ctrl(2*1e-3f);
				
				/*电机输出控制*/
				Motor_Ctrl_Task(2);	
				 
			
        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/500);
    }
} 

/* 姿态角度环控制进程 该任务为精准进行的任务 执行频率精准200Hz 优先级第二*/
void outer_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 

				/*获取姿态角（ROLL PITCH YAW）*/
				calculate_RPY();
				
				/*姿态角度环控制*/
				Att_2level_Ctrl(5e-3f,CH_N);
				 

        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/200);
    }
} 

/* 高度环控制进程 该任务为精准进行的任务 执行频率精准100Hz 优先级第三 */
void height_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 
 
				/*遥控器数据处理*/
				RC_duty_task(10);
				
				/*飞行模式设置任务*/
				Flight_Mode_Set(10);
				
				/*高度数据融合任务*/
				WCZ_Fus_Task(10);
				//GPS_Data_Processing_Task(10);
				
				/*高度速度环控制*/
				Alt_1level_Ctrl(10e-3f);
				
				/*高度环控制*/
				Alt_2level_Ctrl(10e-3f);
				
				/*--*/	
				AnoOF_Check(10);

				/*灯光控制*/	
				LED_Task2(10);
				
				
				//数传响应
				int len = RingBuffer_GetCount(&U3rxring);
				u8 data =0;
				for(; len!= 0 ; len--){
					RingBuffer_Pop(&U3rxring, &data);
					AnoDTRxOneByte( data);
				}
				
        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/100);
    }
} 

/* 位置环控制进程 该任务为精准进行的任务 执行频率精准50Hz 优先级第四*/
void position_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 
			
				/*罗盘数据处理任务*/
				Mag_Update_Task(20);
				
				//通道二一键任务（swb杆往下拨）
				if(onekey.val){
					 onekey.val = UWBTest_Task(20);
				}
				
				AnoOF_Check(20);
				
				/*位置速度环控制*/
				Loc_1level_Ctrl(20,CH_N); 
				
				//解析UWB数据
				UWB_Get_Data_Task();
				
				
				
				/*数传数据交换*/
				ANO_DT_Task1Ms(); 
				 
        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/50);
    }
} 


/* 恒温控制进程 该任务为精准进行的任务 执行频率精准20Hz 优先级第四*/
void temperature_loop(void *pvParameters)
{
    TickType_t xLastWakeTime;         //用于精准定时的变量
	
    while (1)
    {
        xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值 

				/*电压相关任务*/
				Power_UpdateTask(50);
			
				//恒温控制（不能直接注释掉，否则开机过不了校准）
				Thermostatic_Ctrl_Task(50);
			
				/*延时存储任务*/
				Ano_Parame_Write_task(50); 
				   
        vTaskDelayUntil(&xLastWakeTime,configTICK_RATE_HZ/20);
    } 
} 

 
int main(void)
{

	Drv_BspInit();
	flag.start_ok = 1;  
	
	/* 基本传感器数据准备进程 1000Hz*/
	xTaskCreate(basic_data_read,"basic_data_read",152,NULL,4,NULL);
	
	/* 姿态角速度环控制进程 500Hz*/
	xTaskCreate(inner_loop,"inner_loop",104,NULL,3,NULL);
	
	/* 姿态角度环控制进程 200Hz*/
	xTaskCreate(outer_loop,"outer_loop",104,NULL,3,NULL);
	
	/* 高度环控制进程 100Hz*/
	xTaskCreate(height_loop,"height_loop",248,NULL,3,NULL);
	
	/* 位置环控制进程 50Hz*/
	xTaskCreate(position_loop,"position_loop",184,NULL,2,NULL);
	
	/* 恒温控制进程 20Hz*/
	xTaskCreate(temperature_loop,"temperature_loop",128,NULL,2,NULL);
	
	
	printf("Free_Heap_Size:%d\r\n",xPortGetFreeHeapSize());
	
	//启用任务调度器
	vTaskStartScheduler();
	
	//溢出处理
	//fun();
}


