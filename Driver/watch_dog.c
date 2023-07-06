#include "watch_dog.h" 
 
#include "FreeRTOS.h"
#include "task.h"

 
#include "TM4C123G.h"
#include "rom_map.h"
#include "rom.h" 
#include "sysctl.h" 
#include "pin_map.h" 
#include "hw_ints.h"
  
#include "Drv_Uart.h"
#include "ano_usb.h"

/* 看门狗进程 该任务为精准进行的任务 执行频率精准2Hz */
void wdt0_loop(void *pvParameters)
{
	// Enable the peripherals used by this example.
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	
	// Enable the watchdog interrupt.
	MAP_IntEnable(INT_WATCHDOG);

	// Set the period of the watchdog timer.
	MAP_WatchdogReloadSet(WATCHDOG0_BASE, MAP_SysCtlClockGet()/10); /* 100ms触发中断 */

	// Enable reset generation from the watchdog timer.
	MAP_WatchdogResetEnable(WATCHDOG0_BASE);

	// Enable the watchdog timer.
	MAP_WatchdogEnable(WATCHDOG0_BASE);
	
	static TickType_t xLastWakeTime;         //用于精准定时的变量
	
	xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值
	
	while(1)
	{
		MAP_WatchdogIntClear(WATCHDOG0_BASE);	/* 喂狗 */ 
		vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 20);
	}
}

void WDT0_Handler(void)
{
	//进入中断程序已跑飞 
	MAP_WatchdogIntClear(WATCHDOG0_BASE);	 
	MAP_IntDisable(INT_WATCHDOG);
	MAP_WatchdogResetDisable(WATCHDOG0_BASE);
	
	//遗言
	unsigned char theDogWantsToSay[] = "hello world"; 
	Drv_Uart3SendBuf(theDogWantsToSay,sizeof(theDogWantsToSay));
  
  AnoUsbCdcSend(theDogWantsToSay,sizeof(theDogWantsToSay));
	int i=0;
	for( ;i!=100000;i++);
	//复位
	if(i == 100000)
		ROM_SysCtlReset();   
}