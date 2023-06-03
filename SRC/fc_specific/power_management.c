#include "power_management.h"


float singleBatteryVoltage = 0;

void batteryUpdate()
{
	static float voltage_f = 0;  
	
  //触发ADC采样
  Drv_Adc0Trigger();
	 
  //低通滤波 
  S_LPF_1(0.05f,Voltage,voltage_f); 
	
	//求出单节电池电压
  singleBatteryVoltage = voltage_f / Ano_Parame.set.bat_cell;

 

  if(singleBatteryVoltage<Ano_Parame.set.lowest_power_voltage ) {
    flag.power_state = 3;//将禁止解锁
    LED_STA.lowVt = 1;
  }else if(singleBatteryVoltage<Ano_Parame.set.warn_power_voltage) {
    LED_STA.lowVt = 1;
  }else if(singleBatteryVoltage<Ano_Parame.set.return_home_power_voltage) { 
		
  }else{
		LED_STA.lowVt = 0;
		flag.power_state = 1;
	}
}

//返回电池总电压
float getBatteryVoltage(void){
	
	return singleBatteryVoltage*Ano_Parame.set.bat_cell ;
}



