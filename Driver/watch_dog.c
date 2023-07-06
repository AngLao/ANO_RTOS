#include "watch_dog.h" 
 
#include "FreeRTOS.h"
#include "task.h"

 
#include "TM4C123G.h"
#include "rom_map.h"
#include "rom.h" 
#include "sysctl.h" 
#include "pin_map.h" 
#include "hw_ints.h"
 
/* 看门狗进程 该任务为精准进行的任务 执行频率精准2Hz */
void wdt0_loop(void *pvParameters)
{
	// Enable the peripherals used by this example.
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);
	
	// Enable the watchdog interrupt.
	ROM_IntEnable(INT_WATCHDOG);

	// Set the period of the watchdog timer.
	ROM_WatchdogReloadSet(WATCHDOG0_BASE, MAP_SysCtlClockGet()); /* 1s触发中断 */

	// Enable reset generation from the watchdog timer.
	ROM_WatchdogResetEnable(WATCHDOG0_BASE);

	// Enable the watchdog timer.
	ROM_WatchdogEnable(WATCHDOG0_BASE);
	
	static TickType_t xLastWakeTime;         //用于精准定时的变量
	
	xLastWakeTime = xTaskGetTickCount(); //获取当前Tick次数,以赋给延时函数初值
	
	while(1)
	{
		ROM_WatchdogIntClear(WATCHDOG0_BASE);	/* 喂狗 */ 
		vTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / 2);
	}
}

void WDT0_Handler(void)
{
	//进入中断程序已跑飞 
	ROM_WatchdogIntClear(WATCHDOG0_BASE);	 
//	MAP_IntDisable(INT_WATCHDOG);
//	MAP_WatchdogResetDisable(WATCHDOG0_BASE);
	
	//遗言
	#include "ano_usb.h"
	unsigned char theDogWantsToSay[] = "Hello World";
	AnoUsbCdcSend(theDogWantsToSay, sizeof(theDogWantsToSay));
	   
	for(unsigned int i = 0; i<10000; i++){ 
			__nop(); 
	}
	//复位
	ROM_SysCtlReset();   
}


