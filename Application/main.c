#include "FreeRTOS.h"
#include "task.h"

#include "Drv_Bsp.h"
#include "Drv_Uart.h"
#include "Drv_laser.h"
#include "Drv_Timer.h"
#include "Drv_UP_flow.h"
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
#include "Ano_FlightDataCal.h"
#include "Ano_ProgramCtrl_User.h"
#include "nlink_linktrack_tagframe0.h"

#include "watch_dog.h"
#include "rc_update.h"
#include "power_management.h"

#include "uwb_task.h"
#include "light_flow_task.h"


/* 基本传感器数据准备进程 */
void basic_data_read(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {
    /*读取陀螺仪加速度计数据*/
    Drv_Icm20602_Read();

    /*惯性传感器数据准备*/
    Sensor_Data_Prepare(1);

    /*姿态解算更新*/
    imu_update(1);

    /*对世界坐标下z轴加速度进行低通滤波*/
    wcz_acc_update();

    /*飞行状态任务*/
    Flight_State_Task(1, CH_N);
		
		/*根据飞机陀螺仪数据解算出相应姿态以解耦光流数据*/
		OF_INS_Get(0.001f);
		
    //灯光驱动
    LED_1ms_DRV();

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 1000);
  }
}


/* 姿态角速度环控制进程 */
void inner_loop(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {
    /*姿态角速度环控制*/
    Att_1level_Ctrl(2 * 1e-3f);

    /*动力分配*/
    power_distribution(2);

    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 500);
  }
}

/* 姿态角度环控制进程 */
void outer_loop(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {
    /* 四元数转换成欧拉角 */
    calculate_RPY();

    /* 姿态角度环控制 */
    Att_2level_Ctrl(5e-3f, CH_N);
 
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 200);
  }
}

/* 位置控制进程 */
void position_loop(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {  
    /*高度环控制*/
    Alt_2level_Ctrl(0.005f);  
		
    /*垂直速度环控制*/
    Alt_1level_Ctrl(0.005f);
 
    /*位置速度环控制*/
    Loc_1level_Ctrl(5);
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 200);
  }
}
 

/* 辅助任务进程 */
void auxiliary_loop(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

  while (1) {
    /*更新电压值*/
    battery_update();

    /*延时存储任务*/
    Ano_Parame_Write_task(20); //耗时较长 注意看门狗复位
   
    /*数传数据交换调度*/
    dt_scheduler(); 
		 
    /*灯光控制*/
    LED_Task2(20);
		
		/*着陆检测*/
		land_discriminat(20); 
	 
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 50);
  }
}

 
int main(void)
{ 
  /*--------------------------------------------------------
  									板载驱动初始化
  --------------------------------------------------------*/ 
  /* 驱动初始化 */
	Drv_BspInit();
	
  /*--------------------------------------------------------
  									飞控板基础任务
  --------------------------------------------------------*/
  /* 基本传感器数据准备进程 */
  xTaskCreate(basic_data_read, "basic_data_read", 136, NULL, 4, NULL);

  /* 姿态角速度环控制进程 */
  xTaskCreate(inner_loop, "inner_loop", 136, NULL, 3, NULL);

  /* 姿态角度环控制进程 */
  xTaskCreate(outer_loop, "outer_loop", 136, NULL, 3, NULL); 

  /* 位置控制进程 */
  xTaskCreate(position_loop, "position_loop", 136, NULL, 2, NULL);

  /* 辅助任务进程 */
  xTaskCreate(auxiliary_loop, "auxiliary_loop", 156, NULL, 1, NULL);

  /* 启动硬件看门狗 */
  xTaskCreate(wdt0_loop, "wdt0_loop", 136, NULL, 1, NULL);

  /*--------------------------------------------------------
  									外设扩展基础任务
  --------------------------------------------------------*/
  /* 启动遥控器数据处理任务 */
  xTaskCreate(receiving_task, "receiving_task", 276, NULL, 4, NULL);
 
  /* 光流解算任务 */
  xTaskCreate(light_flow_task, "light_flow_task", 136, NULL, 3, NULL);
	
  /* uwb数据更新 */
  xTaskCreate(uwb_update_task, "uwb_update_task", 136, NULL, 3, NULL);

  /*--------------------------------------------------------
  									上层扩展任务
  --------------------------------------------------------*/ 
	
  /*--------------------------------------------------------
  									启用任务调度器
  --------------------------------------------------------*/
  vTaskStartScheduler();



  /*--------------------------------------------------------
  									溢出处理
  --------------------------------------------------------*/
  //printf("Free_Heap_Size = %d \n",xPortGetFreeHeapSize());
  //printf("MinimumEverFreeHeapSize = %d \n",xPortGetMinimumEverFreeHeapSize());
}
