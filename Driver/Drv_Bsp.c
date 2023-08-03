#include "Drv_Bsp.h"
 
#include "FreeRTOS.h"
#include "task.h"

#include "sysconfig.h" 
#include "Drv_RcIn.h"
#include "Drv_Spi.h"
#include "Drv_Led.h"
#include "Drv_Paramter.h"
#include "Drv_PwmOut.h"
#include "Drv_Adc.h"
#include "Drv_Uart.h"
#include "Drv_laser.h"
#include "Drv_icm20602.h"
#include "drv_ak8975.h"
#include "drv_spl06.h"

#include "Ano_FlightCtrl.h"
#include "Ano_DT.h"
#include "Ano_Parameter.h"
#include "Ano_FcData.h"
#include "Ano_Sensor_Basic.h"
#include "ano_usb.h"

#include "rc_update.h"


static void SysTick_Init(void )
{
  ROM_SysTickPeriodSet(ROM_SysCtlClockGet() / 1000);
  ROM_SysTickIntEnable();
  ROM_SysTickEnable();
}

//声明freeRTOS滴答回调
void xPortSysTickHandler( void );

void SysTick_Handler(void)
{
  if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
    xPortSysTickHandler();
  } 
}
   

void Drv_BspInit(void)
{ 
	//寄存器值非默认值就进行软件复位
  if(ROM_SysCtlClockGet() != 16000000 )
    ROM_SysCtlReset();
	
  /*设置系统主频为80M*/
  ROM_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

  /*中断优先级组别设置*/
  NVIC_SetPriorityGrouping(0x03);
	
  /*开启浮点运算单元*/
  ROM_FPULazyStackingEnable();
  ROM_FPUEnable();
	
  //数据初始化
  Dvr_ParamterInit();
	
  //读取初始数据 
  Ano_Parame_Read();
	
  //灯光初始化
  Dvr_LedInit(); 

  //spi通信初始化
  Drv_Spi0Init(); 
	
  //初始化ICM
  sens_hd_check.acc_ok = sens_hd_check.gyro_ok = Drv_Icm20602Init();
	
  //初始化气压计
  sens_hd_check.baro_ok = Drv_Spl0601Init();
	
  //标记罗盘OK，否则罗盘不参与解算（注：此处没有做罗盘是否正常的检测程序）
  sens_hd_check.mag_ok = 0;
 
	//不使用恒温功能
	flag.mems_temperature_ok = 1;
 
	//定点模式
  flag.flight_mode = LOC_HOLD;
 
  //上位机通讯设置初始化
  ANO_DT_Init();

  //ADC初始化
  Drv_AdcInit(); 
	
  //飞控传感器计算初始化
  Sensor_Basic_Init();
	
  //飞控PID初始化
  All_PID_Init();
	
  //电机输出初始化
  Drv_PwmOutInit();
	
  //滴答时钟初始化
  SysTick_Init(); 
	
			
  //发现火源指示灯引脚初始化
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  ROM_GPIOPinTypeGPIOOutput(GPIOF_BASE, GPIO_PIN_0);
  ROM_GPIOPinWrite(GPIOF_BASE, GPIO_PIN_0, 0);
	
}




