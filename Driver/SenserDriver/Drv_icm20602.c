/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：ICM20602驱动
**********************************************************************************/
#include "Drv_icm20602.h"
#include "Drv_spi.h"
#include "hw_ints.h" 


static void icm_delay(void){
	for(unsigned int i = 0; i<12 ; i++){
		for(unsigned int a = 0; a<1000 ; a++){
			__nop();
		}
	}
}

void Drv_Icm20602CSPinInit(void)
{
  ROM_SysCtlPeripheralEnable(ICM_CSPIN_SYSCTL);
  ROM_GPIOPinTypeGPIOOutput(ICM20602_CS_PORT, ICM20602_CS_PIN);
  ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN, ICM20602_CS_PIN);
}
static void icm20602_enable(u8 ena)
{
  if(ena)
    ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN, 0);
  else
    ROM_GPIOPinWrite(ICM20602_CS_PORT, ICM20602_CS_PIN, ICM20602_CS_PIN);
}

static void icm20602_readbuf(u8 reg, u8 length, u8 *data)
{
  icm20602_enable(1);
  Drv_Spi0SingleWirteAndRead(reg | 0x80);
  Drv_Spi0Receive(data, length);
  icm20602_enable(0);
}

static u8 icm20602_writebyte(u8 reg, u8 data)
{
  u8 status;

  icm20602_enable(1);
  status = Drv_Spi0SingleWirteAndRead(reg);
  Drv_Spi0SingleWirteAndRead(data);
  icm20602_enable(0);
  return status;
} 


/**************************实现函数********************************************
*功　　能:	    初始化icm进入可用状态。
*******************************************************************************/
static u8 ICM_ID;
u8 Drv_Icm20602Init(void)
{
  icm20602_writebyte(MPU_RA_PWR_MGMT_1, 0x80);
  icm_delay();
  icm20602_writebyte(MPU_RA_PWR_MGMT_1, 0x01);
  icm_delay();

  u8 tmp;
  icm20602_readbuf(MPUREG_WHOAMI, 1, &tmp);

  if(tmp != MPU_WHOAMI_20602)
    return 0;

  /*复位reg*/
  icm20602_writebyte(MPU_RA_SIGNAL_PATH_RESET, 0x03);
  icm_delay();
  /*复位reg*/
  icm20602_writebyte(MPU_RA_USER_CTRL, 0x01);
  icm_delay();

  icm20602_writebyte(0x70, 0x40); //dmp
  icm_delay();
  icm20602_writebyte(MPU_RA_PWR_MGMT_2, 0x00);
  icm_delay();
  //不分频，配置内部lpf以后，最高1000hz采样，同时对应产生1ms中断
  icm20602_writebyte(MPU_RA_SMPLRT_DIV, 0);
  icm_delay();

  /*陀螺仪LPF 20HZ*/
  icm20602_writebyte(MPU_RA_CONFIG, ICM20602_LPF_20HZ);
  icm_delay();
  /*陀螺仪量程 +-2000dps*/
  icm20602_writebyte(MPU_RA_GYRO_CONFIG, (3 << 3));
  icm_delay();
  /*加速度计量程 +-16G*/
  icm20602_writebyte(MPU_RA_ACCEL_CONFIG, (3 << 3));
  icm_delay();
  /*加速度计LPF 20HZ*/
  icm20602_writebyte(0X1D, 0x04);
  icm_delay();
  /*关闭低功耗*/
  icm20602_writebyte(0X1E, 0x00);
  icm_delay();
  /*关闭FIFO*/
  icm20602_writebyte(0X23, 0x00);
  icm_delay();

  //读取ID
  icm20602_readbuf(MPU_RA_WHO_AM_I, 1, &ICM_ID);

  //
  if(ICM_ID == 0X12) {
    return 1;
  } else {
    return 0;
  }
}



static u8 mpu_buffer[14];

#include "Ano_Sensor_Basic.h" 
void Drv_Icm20602_Read( void )
{
  //读取传感器寄存器，连续读14个字节
  icm20602_readbuf(MPUREG_ACCEL_XOUT_H, 14, mpu_buffer);
  //数据赋值
  s16 temp[2][3];
  //	/*读取buffer原始数据*/
  temp[0][X] = (s16)((((u16)mpu_buffer[0]) << 8) | mpu_buffer[1]); 
  temp[0][Y] = (s16)((((u16)mpu_buffer[2]) << 8) | mpu_buffer[3]); 
  temp[0][Z] = (s16)((((u16)mpu_buffer[4]) << 8) | mpu_buffer[5]); 

  temp[1][X] = (s16)((((u16)mpu_buffer[ 8]) << 8) | mpu_buffer[ 9]) ;
  temp[1][Y] = (s16)((((u16)mpu_buffer[10]) << 8) | mpu_buffer[11]) ;
  temp[1][Z] = (s16)((((u16)mpu_buffer[12]) << 8) | mpu_buffer[13]) ;

  sensor.Tempreature = ((((int16_t)mpu_buffer[6]) << 8) | mpu_buffer[7]); //tempreature
  /*icm20602温度*/
  sensor.Tempreature_C = sensor.Tempreature / 326.8f + 25 ; //sensor.Tempreature/340.0f + 36.5f;

  //调整物理坐标轴与软件坐标轴方向定义一致
  sensor.Acc_Original[X] = temp[0][X];
  sensor.Acc_Original[Y] = temp[0][Y];
  sensor.Acc_Original[Z] = temp[0][Z];

  sensor.Gyro_Original[X] = temp[1][X];
  sensor.Gyro_Original[Y] = temp[1][Y];
  sensor.Gyro_Original[Z] = temp[1][Z];
}




