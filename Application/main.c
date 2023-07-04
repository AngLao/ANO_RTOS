#include "FreeRTOS.h"
#include "task.h"
  
#include "Drv_Bsp.h"	
#include "Drv_Uart.h"
#include "Drv_Timer.h" 
#include "Drv_heating.h"

#include "Ano_Imu.h"
#include "Ano_Sensor_Basic.h" 
#include "Ano_FlightCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h" 
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_FlyCtrl.h" 
#include "Ano_FlightDataCal.h" 
#include "Ano_ProgramCtrl_User.h"
#include "nlink_linktrack_tagframe0.h"

#include "rc_update.h" 
#include "power_management.h" 
  
/* 基本传感器数据准备进程 该任务为精准进行的任务 执行频率精准1000Hz 优先级全局最高*/
void basic_data_read(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*读取陀螺仪加速度计数据*/
    Drv_Icm20602_Read();

    /*惯性传感器数据准备*/
    Sensor_Data_Prepare(1);

    /*姿态解算更新*/
    IMU_Update_Task(1);

    /*获取WC_Z加速度*/
    WCZ_Acc_Get_Task();

    /*飞行状态任务*/
    Flight_State_Task(1, CH_N);

    /*开关状态任务*/
    Swtich_State_Task(1);

    /*光流融合数据准备任务*/
    ANO_OF_Data_Prepare_Task(0.001f);
 
    //灯光驱动
    LED_1ms_DRV();

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 1000);
  }
}


/* 姿态角速度环控制进程 该任务为精准进行的任务 执行频率精准500Hz 优先级第二*/
void inner_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*姿态角速度环控制*/
    Att_1level_Ctrl(2 * 1e-3f);

    /*电机输出控制*/
    Motor_Ctrl_Task(2);


    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 500);
  }
}

/* 姿态角度环控制进程 该任务为精准进行的任务 执行频率精准200Hz 优先级第二*/
void outer_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*获取姿态角（ROLL PITCH YAW）*/
    calculate_RPY();

    /*姿态角度环控制*/
    Att_2level_Ctrl(5e-3f, CH_N);


    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 200);
  }
}

/* 高度环控制进程 该任务为精准进行的任务 执行频率精准100Hz 优先级第三 */
void height_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*遥控器数据处理任务*/
    receivingTask();

    /*高度数据融合任务*/
    WCZ_Fus_Task(10); 

    /*高度速度环控制*/
    Alt_1level_Ctrl(10e-3f);

    /*高度环控制*/
    Alt_2level_Ctrl(10e-3f);

    /*光流掉线检测*/
    AnoOF_Check(10);

    /*灯光控制*/
    LED_Task2(10);

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 100);
  }
}

/* 位置环控制进程 该任务为精准进行的任务 执行频率精准50Hz 优先级第四*/
void position_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*罗盘数据处理任务*/
    Mag_Update_Task(20); 

    /*位置速度环控制*/
    Loc_1level_Ctrl(20);
 
    /*数传数据交换*/
    dtTask();
 
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 50);
  }
}


/* 辅助任务进程 该任务为精准进行的任务 执行频率精准20Hz */
void auxiliary_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*更新电压值*/
    batteryUpdate(); 

    /*延时存储任务*/
    Ano_Parame_Write_task(50);
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 20);
  }
}


/* 自定义进程 */
void user_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

	static unsigned char pFrame[128];
	
  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

		//读环形缓冲区  
		if(RingBuffer_GetCount(&U1rxring) > 128) {
			RingBuffer_PopMult(&U1rxring, pFrame, 128);

			//解析uwb数据
			uint8_t res = g_nlt_tagframe0.UnpackData(pFrame, 128);
			//接收有误刷新缓冲区
			if (res == 0) { 
				RingBuffer_Flush(&U1rxring);
			}
		}
		   
		
		vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 10);
	}
}
 
int main(void)
{
	//寄存器值非默认值就进行软件复位
	if(ROM_SysCtlClockGet() != 16000000 ) 
		ROM_SysCtlReset(); 
	
  Drv_BspInit(); 

  /* 基本传感器数据准备进程 1000Hz*/
  xTaskCreate(basic_data_read, "basic_data_read", 152, NULL, 4, NULL);

  /* 姿态角速度环控制进程 500Hz*/
  xTaskCreate(inner_loop, "inner_loop", 104, NULL, 3, NULL);

  /* 姿态角度环控制进程 200Hz*/
  xTaskCreate(outer_loop, "outer_loop", 104, NULL, 3, NULL);

  /* 高度环控制进程 100Hz*/
  xTaskCreate(height_loop, "height_loop", 248, NULL, 3, NULL);

  /* 位置环控制进程 50Hz*/
  xTaskCreate(position_loop, "position_loop", 184, NULL, 2, NULL);

  /* 辅助任务进程 20Hz*/
  xTaskCreate(auxiliary_loop, "auxiliary_loop", 128, NULL, 2, NULL);
	
	
  /* 自定义进程 50Hz*/
  xTaskCreate(user_loop, "user_loop", 128, NULL, 3, NULL); 
 
  //启用任务调度器
  vTaskStartScheduler(); 
	
	//printf("Free_Heap_Size = %d \n",xPortGetFreeHeapSize());   
	//printf("MinimumEverFreeHeapSize = %d \n",xPortGetMinimumEverFreeHeapSize());   
	
  //溢出处理
  //fun();
	
}
