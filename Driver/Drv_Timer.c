
#include "Drv_Timer.h"

//为1时启用运行时间统计功能

#if(configGENERATE_RUN_TIME_STATS == 1)

#include "timer.h"
#include "hw_memmap.h"
#include "hw_ints.h"
#include "gpio.h"
#include "sysctl.h"
#include "interrupt.h"
#include "rom.h"

#include "FreeRTOS.h"
#include "task.h"

void TIMER_IRQHandler(void);
void TIMER_WID_IRQHandler(void);


/* 用于统计运行时间 */
volatile uint32_t CPU_RunTime = 0UL;
//16/32bit定时器拆分
void Timer_Config(void)
{
  //使能定时器TIMER0，16/32bit
  SysCtlPeripheralEnable( SYSCTL_PERIPH_TIMER0);
  //配置定时器，将定时器拆分，并配置拆分后的定时器A为周期性计数
  TimerConfigure( TIMER0_BASE,  TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC_UP);
  //设置定时器装载值，
  TimerLoadSet( TIMER0_BASE,  TIMER_A, ROM_SysCtlClockGet()/20000-1);
  //为定时器A注册中断函数
  TimerIntRegister( TIMER0_BASE,  TIMER_A, TIMER_IRQHandler);
  //使能time0的定时器A为超时中断
  TimerIntEnable( TIMER0_BASE,  TIMER_TIMA_TIMEOUT);
  //设置中断优先级
  IntPrioritySet( INT_TIMER0A,  0);
  //使能中断
  IntEnable( INT_TIMER0A);
  IntMasterEnable();
  //使能定时器
  TimerEnable( TIMER0_BASE,  TIMER_A);
}

void TIMER_IRQHandler(void)
{ 
  //读取定时器中断状态
  uint32_t status=TimerIntStatus( TIMER0_BASE,  true);
	
  //进一次中断
  CPU_RunTime++; 
	
  //清除中断标志位
  TimerIntClear( TIMER0_BASE,  status);
} 



void task_census(void *pvParameters)
{
	#include "string.h"
  TickType_t xLastWakeTime;         //用于精准定时的变量
	uint8_t CPU_RunInfo[400];		//保存任务运行时间信息
  while (1) {
    xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值

//		memset(CPU_RunInfo,0,400);				//信息缓冲区清零

//		vTaskList((char *)&CPU_RunInfo);  //获取任务运行时间信息

//		printf("---------------------------------------------\r\n");
//		printf("任务名      任务状态 优先级   剩余栈 任务序号\r\n");
//		printf("%s", CPU_RunInfo);
//		printf("---------------------------------------------\r\n");

		memset(CPU_RunInfo,0,400);				//信息缓冲区清零

		vTaskGetRunTimeStats((char *)&CPU_RunInfo);

		printf("任务名       运行计数         利用率\r\n");
		printf("%s", CPU_RunInfo);
		printf("---------------------------------------------\r\n\n");
		
    vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 1);
  }
}
#endif
