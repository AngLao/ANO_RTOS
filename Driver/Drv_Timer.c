#include "Drv_Timer.h" 
 

#include "Timer.h" 

#include "tm4c123gh6pm.h"
#include "TM4C123G.h"
#include "rom.h"
#include "rom_map.h"
#include "sysctl.h" 
#include "hw_ints.h"

#include "Ano_DT.h"

unsigned long long int timer_count = 0;
void IntHandle_TIMER3A(void);
void vConfigureTimerForRunTimeStats(void)
{
		// 使能 Timer 1 的时钟
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);

    // 等待 Timer 1 时钟使能完成
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3))
    {
        // 等待时钟使能
    }

    // 关闭 Timer 1
    ROM_TimerDisable(TIMER3_BASE, TIMER_BOTH);

    // 配置 Timer 1 为周期计时器模式
    ROM_TimerConfigure(TIMER3_BASE, TIMER_CFG_PERIODIC);

    // 设置 Timer 1 的计时器周期（根据系统时钟频率和所需的时间分辨率进行计算）
    uint32_t timerPeriod = ROM_SysCtlClockGet() / 1000;
    ROM_TimerLoadSet(TIMER3_BASE, TIMER_BOTH, timerPeriod - 1);
	
		//注册中断服务函数，最后那个是中断服务函数句柄，其实后面看库函数手册这一步也可以不用啦，那时候中断入口函数名字就是官方默认的（具体是什么忘了...）  
		TimerIntRegister(TIMER3_BASE,TIMER_A,IntHandle_TIMER3A);
       	
		//使能定时器模块Timer0的定时器A的中断。
		ROM_IntEnable(INT_TIMER3A);
		//使能单独的定时器中断源，第一个TIMER0_BASE为Timer0的基地址，第二个是中断源启用的中断事件的标识码，TIMER_TIMA_TIMEOUT的意思是定时器A(TimerA)溢出(重装载)，以此为标志位，当TimerA重装载时就会触发中断。
		TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
		
		//使能处理器中断，使处理器能够响应中断。
		ROM_IntMasterEnable();
    // 启用 Timer 1
    ROM_TimerEnable(TIMER3_BASE, TIMER_A);
}

void IntHandle_TIMER3A(void)//和你注册中断句柄时的名称保持一致 
{
	//清除标志位，第二个是中断类型，我这里是定时器A溢出中断
	TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
      
	timer_count++;
	ANO_DT_SendString("TIMER_TIMA_TIMEOUT"); 
 }