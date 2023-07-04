#include "FreeRTOS.h"
#include "task.h"

#include "sysconfig.h"
#include "Drv_Bsp.h"
#include "Ano_FcData.h"

#include "Drv_Bsp.h"
#include "Drv_icm20602.h"
#include "Ano_LED.h"
#include "Ano_Sensor_Basic.h"

#include "Ano_DT.h"
#include "rc_update.h"
#include "Drv_led.h"
#include "Ano_FlightCtrl.h"
#include "Ano_AttCtrl.h"
#include "Ano_LocCtrl.h"
#include "Ano_AltCtrl.h"
#include "Ano_MotorCtrl.h"
#include "Ano_Parameter.h"
#include "Ano_MagProcess.h"
#include "Ano_OF.h"
#include "Drv_heating.h"
#include "Ano_FlyCtrl.h" 
#include "Ano_OF_DecoFusion.h"
#include "Drv_Uart.h"
#include "Ano_Imu.h"
#include "Ano_FlightDataCal.h"
#include "Ano_Sensor_Basic.h"
#include "ano_usb.h"
#include "Ano_ProgramCtrl_User.h"
#include "Drv_Timer.h"
#include "ano_usb.h"
#include "power_management.h"
#include "Drv_Uart.h"
#include "ring_buffer.h"
#include "nlink_utils.h"
#include "nlink_linktrack_tagframe0.h"
#include "hw_ints.h"

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


/* 恒温控制进程 该任务为精准进行的任务 执行频率精准20Hz */
void temperature_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    /*更新电压值*/
    batteryUpdate();

    //恒温控制（不能直接注释掉，否则开机过不了校准）
    Thermostatic_Ctrl_Task(50);

    /*延时存储任务*/
    Ano_Parame_Write_task(50);
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 20);
  }
}

/* 看门狗进程 该任务为精准进行的任务 执行频率精准2Hz */
void wdt0_loop(void *pvParameters)
{
	// Enable the peripherals used by this example.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	
	// Enable the watchdog interrupt.
	MAP_IntEnable(INT_WATCHDOG);

	// Set the period of the watchdog timer.
	MAP_WatchdogReloadSet(WATCHDOG0_BASE, MAP_SysCtlClockGet()); /* 1s触发中断 */

	// Enable reset generation from the watchdog timer.
	MAP_WatchdogResetEnable(WATCHDOG0_BASE);

	// Enable the watchdog timer.
	MAP_WatchdogEnable(WATCHDOG0_BASE);
	
	static TickType_t xLastWakeTime;         //用于精准定时的变量
	
	xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值
	
	while(1)
	{
		MAP_WatchdogIntClear(WATCHDOG0_BASE);	/* 喂狗 */
		printf("Feed wdt0\r\n");
		vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 2);
	}
}
void WDT0_Handler(void)
{
	printf("WDT0_Handler\r\n");
	printf("MCU Fuck Game Over\r\n");
}

/* 自定义进程 */
void user_loop(void *pvParameters)
{
  TickType_t xLastWakeTime;         //用于精准定时的变量

  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

    //解析UWB数据 
		//读环形缓冲区
		static int RingBufferDataLen = 0;
		static unsigned char pData[128 * 5];
		RingBufferDataLen = RingBuffer_GetCount(&U1rxring) ;

		//解析uwb数据
		if(RingBufferDataLen) {
			RingBuffer_PopMult(&U1rxring, pData, RingBufferDataLen);

			if (g_nlt_tagframe0.UnpackData(pData, RingBufferDataLen)) {
				return;
			}
		}
 
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 50);
  }
}
 
int main(void)
{
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

  /* 恒温控制进程 20Hz*/
  xTaskCreate(temperature_loop, "temperature_loop", 128, NULL, 2, NULL);
	
	
//  /* 自定义进程 50Hz*/
//  xTaskCreate(user_loop, "user_loop", 128, NULL, 3, NULL); 
xTaskCreate(wdt0_loop, "wdt0_loop", 96 + 32, NULL, 1, NULL); 
  //启用任务调度器
  vTaskStartScheduler();

  //溢出处理
  //fun();
}


