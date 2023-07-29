/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：数据传输
**********************************************************************************/
/*============================================================================
更新：
201907272159-Jyoun：增加飞控状态界面传感器状态数据发送。
201908031126-茶不思：增加mv相关数据发送，可在上位机查看mv寻色块数据。


===========================================================================*/

#include "Ano_DT.h"
#include "drv_spl06.h"
#include "Drv_Uart.h"
#include "ano_usb.h"
#include "rc_update.h"
#include "Ano_Sensor_Basic.h"
#include "Ano_Parameter.h"
#include "ANO_IMU.h"
#include "Drv_icm20602.h"
#include "Ano_MagProcess.h"
#include "Ano_MotorCtrl.h"
#include "Ano_FlightCtrl.h"
#include "Ano_FlightDataCal.h"
#include "Ano_OF_DecoFusion.h"
#include "Ano_LocCtrl.h"
#include "Ano_OF.h"
#include "Ano_OF_DecoFusion.h"
#include "power_management.h"
#include "nlink_linktrack_tagframe0.h"
#include "uwb_task.h"


#define BYTE0(dwTemp)       ( *( (char *)(&dwTemp)		) )
#define BYTE1(dwTemp)       ( *( (char *)(&dwTemp) + 1) )
#define BYTE2(dwTemp)       ( *( (char *)(&dwTemp) + 2) )
#define BYTE3(dwTemp)       ( *( (char *)(&dwTemp) + 3) )

#define DT_RX_BUFNUM  64
#define DT_ODNUM	5	//非循环发送数据缓冲大小 

//越往前发送优先级越高，如果需要修改，这里和h文件里的枚举需要同时改
const u8  _cs_idlist[CSID_NUM]	 	= {0x01, 0x02, 0x03, 0x05, 0x06, 0x07, 0x0B, 0x0D, 0x0E, 0x20, 0x32,0x40,0x51};

//循环发送数据结构体
typedef struct {
  u8 WTS;		 //wait to send等待发送标记
  u16 fre_ms; //发送周期
  u16 time_cnt_ms; //计时变量
} _dt_frame_st;

//循环发送数据临时缓冲
u8 CycleSendData[100];

//非循环发送数据结构体
typedef struct {
  u8 WTS;
  u8 type;
  u8 len;
  u8 data[64];
} _dt_otherdata_st;
_dt_otherdata_st OtherSendData[DT_ODNUM];

u8 otherDataTmp[64];	//非循环发送数据临时缓冲

typedef struct {
  _dt_frame_st txSet_u2[CSID_NUM];
} _dt_st;
_dt_st dt;

//0x01
#define ACC_RAW_X      (sensor.Acc[X])
#define ACC_RAW_Y      (sensor.Acc[Y])
#define ACC_RAW_Z      (sensor.Acc[Z])
#define GYR_RAW_X      (sensor.Gyro[X])
#define GYR_RAW_Y      (sensor.Gyro[Y])
#define GYR_RAW_Z      (sensor.Gyro[Z])
#define SHOCK_STA      (flag.unlock_err)

//#include "openmv_task.h"
//#define ACC_RAW_X      (0)
//#define ACC_RAW_Y      (0)
//#define ACC_RAW_Z      (0)
//#define GYR_RAW_X      (0)
//#define GYR_RAW_Y      (mvValue.pos)
//#define GYR_RAW_Z      (openmvSpeedOut[YAW]*100)
//#define SHOCK_STA      (0)

//0x02
#define ECP_RAW_X      (mag.val[X])
#define ECP_RAW_Y      (mag.val[Y])
#define ECP_RAW_Z      (mag.val[Z])
#define BARO_ALT       (baroHeight*100)
#define TEMPERATURE    (sensor.Tempreature_C)
#define BARO_STA       (sens_hd_check.baro_ok)
#define ECP_STA        (sens_hd_check.mag_ok)
//0x03
#define ANGLE_ROL      (imu_data.rol)
#define ANGLE_PIT      (imu_data.pit)
#define ANGLE_YAW      (imu_data.yaw)
#define ATT_FUSION_STA (1)

//0x04
#define QUA0_10K       (imu_att.qua.w *1e4f)
#define QUA1_10K       (imu_att.qua.x *1e4f)
#define QUA2_10K       (imu_att.qua.y *1e4f)
#define QUA3_10K       (imu_att.qua.z *1e4f)

//0X05
#define ALT_FU	       (wcz_hei_fus.out*100)
#define ALT_ADD	       (jsdata.of_alt*100)
#define ALT_STA        (loc_ctrl_2.exp[Z])

//0X06
#define FC_MODE	       (flag.flight_mode)
#define FC_LOCKED	   	 (flag.unlock_sta)
#define FC_CMD_ID	   	 (flag.rc_loss)
#define FC_CMD_0       (0)
#define FC_CMD_1       (0)

//0X07  反馈速度
//#define HCA_VEL_X      (loc_ctrl_1.fb[X])
//#define HCA_VEL_Y      (loc_ctrl_1.fb[Y])
//#define HCA_VEL_Z      (loc_ctrl_1.fb[Z])
#define HCA_VEL_X      (g_nlt_tagframe0.result.vel_3d[X])
#define HCA_VEL_Y      (g_nlt_tagframe0.result.vel_3d[Y])
#define HCA_VEL_Z      (g_nlt_tagframe0.result.vel_3d[Z])

//0X0B  期望速度
#define HCA_TAR_VEL_X   (loc_ctrl_1.exp[X])
#define HCA_TAR_VEL_Y   (loc_ctrl_1.exp[Y])
#define HCA_TAR_VEL_Z   (loc_ctrl_1.exp[Z])

//0X0D
#define BAT_VOLTAGE_100 (get_battery_voltage()*100)

//0x20
#define PWM_1          (motor[0])
#define PWM_2          (motor[1])
#define PWM_3          (motor[2])
#define PWM_4          (motor[3])

//0x51 融合光流数据
#define MODE          (2)
#define STATE         (OF_STATE)
#define DX_2          (OF_DX2)
#define DY_2          (OF_DY2)
#define DX_FIX  	   	(OF_DX2FIX)
#define DY_FIX      	(OF_DY2FIX)
#define INTEG_X      	(OF_INTEG_X)
#define INTEG_Y				(OF_INTEG_Y)
#define QUALITY      	(OF_QUALITY)

void ANO_DT_Init(void)
{
//  //ACC-GRO
//  dt.txSet_u2[CSID_X01].fre_ms = 100;
  //ECP-TEM-BARO
  dt.txSet_u2[CSID_X02].fre_ms = 50;
  //ATT_ANG
  dt.txSet_u2[CSID_X03].fre_ms = 100;
  //height
  dt.txSet_u2[CSID_X05].fre_ms = 50;
//  //fc_mode
//  dt.txSet_u2[CSID_X06].fre_ms = 200;
//  //反馈速度
//  dt.txSet_u2[CSID_X07].fre_ms = 100;
//  //期望速度
//  dt.txSet_u2[CSID_X0B].fre_ms = 100;
//  //电压
//  dt.txSet_u2[CSID_X0D].fre_ms = 100;
//  //传感器状态
//  dt.txSet_u2[CSID_X0E].fre_ms = 100;
//  //PWM
//  dt.txSet_u2[CSID_X20].fre_ms = 100;
//  //UWB数据
//  dt.txSet_u2[CSID_X32].fre_ms = 100;
//  //遥控数据
//  dt.txSet_u2[CSID_X40].fre_ms = 100;
//  //光流数据
//  dt.txSet_u2[CSID_X51].fre_ms = 100;

#if(DEBUG_CONFIG == UART)

  //数传串口初始化
  Drv_Uart3Init(115200);

#elif (DEBUG_CONFIG == USB_CDC)

  //板载USB虚拟串口初始化
  AnoUsbCdcInit();

#endif

}







/*----------------------------------------------------------
							数据发送相关函数
----------------------------------------------------------*/
void ANO_DT_Send_Data(u8 *dataToSend, u8 length)
{
#if(DEBUG_CONFIG == UART)

  Drv_Uart3SendBuf(dataToSend, length);

#elif (DEBUG_CONFIG == USB_CDC)

  AnoUsbCdcSend(dataToSend, length);

#endif


}
/*----------------------------------------------------------
						根据ID填充数据帧的data区域
----------------------------------------------------------*/
static void DTFrameAddData(u8 frame_num, u8 *_cnt)
{
  s16 temp_data;
  s32 temp_data_32;

  switch(frame_num) {
  //惯性数据
  case 0x01: {
    temp_data = (s16)(ACC_RAW_X);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ACC_RAW_Y);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ACC_RAW_Z);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(GYR_RAW_X);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(GYR_RAW_Y);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(GYR_RAW_Z);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    //STATE
    CycleSendData[(*_cnt)++] = SHOCK_STA;
  }
  break;

  case 0x02: {
    //ECP
    temp_data = (s16)(ECP_RAW_X );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ECP_RAW_Y );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ECP_RAW_Z );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    //BARO_ALT
    temp_data_32 = (s32)(BARO_ALT );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE2(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE3(temp_data_32);
    //temperature
    temp_data = (s16)(TEMPERATURE * 1e2f);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    //BARO_STA
    CycleSendData[(*_cnt)++] = BARO_STA;
    //ECP_STA
    CycleSendData[(*_cnt)++] = ECP_STA;
  }
  break;

  //欧拉角姿态数据
  case 0x03: {
    temp_data = (s16)(ANGLE_ROL * 100 );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ANGLE_PIT * 100 );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(ANGLE_YAW * 100 );
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    //STATE
    CycleSendData[(*_cnt)++] = ATT_FUSION_STA;
  }
  break;

  //高度数据
  case 0x05: {
    temp_data_32 = (s32)(ALT_FU);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE2(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE3(temp_data_32);
    temp_data_32 = (s32)(ALT_ADD);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE2(temp_data_32);
    CycleSendData[(*_cnt)++] = BYTE3(temp_data_32);
    //
    CycleSendData[(*_cnt)++] = ALT_STA;
  }
  break;

  case 0x06: {
    //MODE	LOCKED	FUN	CMD
    CycleSendData[(*_cnt)++] = FC_MODE;
    CycleSendData[(*_cnt)++] = FC_LOCKED;
    CycleSendData[(*_cnt)++] = FC_CMD_ID;
    CycleSendData[(*_cnt)++] = FC_CMD_0;
    CycleSendData[(*_cnt)++] = FC_CMD_1;
  }
  break;

  //反馈速度
  case 0x07:
    temp_data = (s16)(HCA_VEL_X);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(HCA_VEL_Y);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)(HCA_VEL_Z);
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    break;

  //期望速度
  case 0x0B:
    temp_data = (s16)HCA_TAR_VEL_X;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)HCA_TAR_VEL_Y;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)HCA_TAR_VEL_Z;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    break;

  case 0x0D: {
    //电压
    temp_data = (s16)BAT_VOLTAGE_100;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    CycleSendData[(*_cnt)++] = 0;
    CycleSendData[(*_cnt)++] = 0;
  }
  break;

  case 0x0E: {
    //传感器状态
    CycleSendData[(*_cnt)++] = switchs.of_flow_on;
    CycleSendData[(*_cnt)++] = sens_hd_check.of_ok;
    CycleSendData[(*_cnt)++] = 0;
    CycleSendData[(*_cnt)++] = switchs.of_tof_on;
  }
  break;

  case 0x20: {
    temp_data = (s16)PWM_1;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)PWM_2;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)PWM_3;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
    temp_data = (s16)PWM_4;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);
  }
  break;

  //UWB数据
  case 0x32: {
    for(uint8_t i = 0; i < 3; i++) {
      s32 temp = (int32_t)g_nlt_tagframe0.result.pos_3d[i];
      CycleSendData[(*_cnt)++] = BYTE0(temp) ;
      CycleSendData[(*_cnt)++] = BYTE1(temp) ;
      CycleSendData[(*_cnt)++] = BYTE2(temp) ;
      CycleSendData[(*_cnt)++] = BYTE3(temp) ;
    }
  }
  break;

  //遥控器数据
  case 0x40: {
    for(char i = 0; i < CH_NUM; i++) {
      u16 temp = CH_N[i] + 1500;
      CycleSendData[(*_cnt)++] = BYTE0(temp) ;
      CycleSendData[(*_cnt)++] = BYTE1(temp) ;
    }

    CycleSendData[(*_cnt)++] = 0;
    CycleSendData[(*_cnt)++] = 0;
    CycleSendData[(*_cnt)++] = 0;
    CycleSendData[(*_cnt)++] = 0;
  }
  break;

  //光流数据
  case 0X51: {
    CycleSendData[(*_cnt)++] = MODE;
    CycleSendData[(*_cnt)++] = STATE;

    temp_data = (s16)DX_2;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    temp_data = (s16)DY_2;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    temp_data = (s16)DX_FIX;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    temp_data = (s16)DY_FIX;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    temp_data = (s16)INTEG_X;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    temp_data = (s16)INTEG_Y;
    CycleSendData[(*_cnt)++] = BYTE0(temp_data);
    CycleSendData[(*_cnt)++] = BYTE1(temp_data);

    CycleSendData[(*_cnt)++] = QUALITY;
  }
  break;

  default :
    break;
  }
}

//===========================================================
//根据ID组织发送数据帧
//===========================================================
static void DTFrameSend(u8 id_addr, u8 addr)
{
  u8 _cnt = 0;
  u8 _id;

  CycleSendData[_cnt++] = FRAME_HEAD;
  CycleSendData[_cnt++] = addr;

  if(id_addr >= CSID_NUM)
    return;

  _id = _cs_idlist[id_addr];
  CycleSendData[_cnt++] = _id;
  CycleSendData[_cnt++] = 0;
  //==
  DTFrameAddData(_id, &_cnt);
  //==
  CycleSendData[3] = _cnt - 4;
  //==
  u8 check_sum1 = 0, check_sum2 = 0;

  for(u8 i = 0; i < _cnt; i++) {
    check_sum1 += CycleSendData[i];
    check_sum2 += check_sum1;
  }

  CycleSendData[_cnt++] = check_sum1;
  CycleSendData[_cnt++] = check_sum2;

  ANO_DT_Send_Data(CycleSendData, _cnt);
}

//===========================================================
//非循环发送数据的写入缓冲区
//===========================================================
static u8 OtherSendDataAdd(u8 *_data, u8 _len)
{
  for(u8 i = 0; i < DT_ODNUM; i++) {
    if(OtherSendData[i].WTS == 0) {
      OtherSendData[i].type = 0x04;
      OtherSendData[i].len = _len;

      for(u8 j = 0; j < _len; j++)
        OtherSendData[i].data[j] = *(_data + j);

      OtherSendData[i].WTS = 1;
      return 1;
    }
  }

  return 0;
}
//===========================================================
//判断是否有需要发送的非循环数据，如有，则启动发送
//===========================================================
static void OtherSendDataCheck(void)
{
  for(u8 i = 0; i < DT_ODNUM; i++) {
    if(OtherSendData[i].WTS) {
      ANO_DT_Send_Data(OtherSendData[i].data, OtherSendData[i].len);
      OtherSendData[i].WTS = 0;
      return;
    }
  }
}
//===========================================================
//检查循环发送的数据有没有到指定的时间间隔
//===========================================================
static void CheckDotMs( u8 id_addr)
{
  u16 * _dot_ms = & dt.txSet_u2[id_addr].fre_ms;
  u16 * _dot_cnt_ms = & dt.txSet_u2[id_addr].time_cnt_ms;
  u8  * _dot_WTS  = & dt.txSet_u2[id_addr].WTS;


  if(id_addr >= CSID_NUM)
    return ;

  if((*_dot_ms) != 0) {
    if((*_dot_cnt_ms) < (*_dot_ms)) {
      (*_dot_cnt_ms)++;
    } else {
      //清除计时并置位等待发送标记
      (*_dot_WTS) = 1;
      (*_dot_cnt_ms) = 1;
    }
  }
}
//===========================================================
//检查有没有需要循环发送的数据
//===========================================================
u8 CheckDotWts(u8 id_addr)
{
  u8 _addr = 0xaf;
  u8  * _dot_WTS  = & dt.txSet_u2[id_addr].WTS;


  if(id_addr >= CSID_NUM) {
    return 0;
  }


  if((*_dot_WTS)) {
    //复位等待发送标记
    (*_dot_WTS) = 0;
    //实际发送
    DTFrameSend(id_addr, _addr);
    return 1;
  } else {
    return 0;
  }
}

//===========================================================
//发送Check帧，见协议文本
//===========================================================
static void SendCheck(u8 dest_addr, u8 cid, u8 sc, u8 ac)
{
  u8 _cnt = 0;

  otherDataTmp[_cnt++] = FRAME_HEAD;
  otherDataTmp[_cnt++] = dest_addr;
  otherDataTmp[_cnt++] = 0;
  otherDataTmp[_cnt++] = 0;

  otherDataTmp[_cnt++] = cid;
  otherDataTmp[_cnt++] = sc;
  otherDataTmp[_cnt++] = ac;

  otherDataTmp[3] = _cnt - 4;
  u8 check_sum1 = 0, check_sum2 = 0;

  for(u8 i = 0; i < _cnt; i++) {
    check_sum1 += otherDataTmp[i];
    check_sum2 += check_sum1;
  }

  otherDataTmp[_cnt++] = check_sum1;
  otherDataTmp[_cnt++] = check_sum2;

  OtherSendDataAdd(otherDataTmp, _cnt);
}
//===========================================================
//发送参数
//===========================================================
static void SendPar(u8 dest_addr, u16 p_id)
{
  s32 p_val = 0;
  p_val = AnoParRead(p_id);

  u8 _cnt = 0;

  otherDataTmp[_cnt++] = FRAME_HEAD;
  otherDataTmp[_cnt++] = dest_addr;
  otherDataTmp[_cnt++] = 0xE2;
  otherDataTmp[_cnt++] = 0;

  otherDataTmp[_cnt++] = BYTE0(p_id);
  otherDataTmp[_cnt++] = BYTE1(p_id);

  otherDataTmp[_cnt++] = BYTE0(p_val);
  otherDataTmp[_cnt++] = BYTE1(p_val);
  otherDataTmp[_cnt++] = BYTE2(p_val);
  otherDataTmp[_cnt++] = BYTE3(p_val);

  otherDataTmp[3] = _cnt - 4;
  u8 check_sum1 = 0, check_sum2 = 0;

  for(u8 i = 0; i < _cnt; i++) {
    check_sum1 += otherDataTmp[i];
    check_sum2 += check_sum1;
  }

  otherDataTmp[_cnt++] = check_sum1;
  otherDataTmp[_cnt++] = check_sum2;

  OtherSendDataAdd(otherDataTmp, _cnt);
}
//===========================================================
//发送字符串给上位机log区域显示
//===========================================================
void AnoDTSendStr(u8 dest_addr, u8 string_color, char *str)
{
  u8 _cnt = 0;

  otherDataTmp[_cnt++] = FRAME_HEAD;
  otherDataTmp[_cnt++] = dest_addr;
  otherDataTmp[_cnt++] = 0xA0;
  otherDataTmp[_cnt++] = 0;

  otherDataTmp[_cnt++] = string_color;
  u8 i = 0;

  while(*(str + i) != '\0') { //单引号字符，双引号字符串
    otherDataTmp[_cnt++] = *(str + i++);

    if(_cnt > 84) {
      break;
    }
  }

  otherDataTmp[3] = _cnt - 4;
  u8 check_sum1 = 0, check_sum2 = 0;

  for(u8 i = 0; i < _cnt; i++) {
    check_sum1 += otherDataTmp[i];
    check_sum2 += check_sum1;
  }

  otherDataTmp[_cnt++] = check_sum1;
  otherDataTmp[_cnt++] = check_sum2;

  OtherSendDataAdd(otherDataTmp, _cnt);
}
//===========================================================
//发送字符串给上位机log区域显示
//===========================================================
void AnoDTSendF1(u8 dest_addr, u8 d1)
{
  u8 _cnt = 0;

  otherDataTmp[_cnt++] = FRAME_HEAD;
  otherDataTmp[_cnt++] = dest_addr;
  otherDataTmp[_cnt++] = 0xF1;
  otherDataTmp[_cnt++] = 0;

  otherDataTmp[_cnt++] = d1;

  otherDataTmp[3] = _cnt - 4;
  u8 check_sum1 = 0, check_sum2 = 0;

  for(u8 i = 0; i < _cnt; i++) {
    check_sum1 += otherDataTmp[i];
    check_sum2 += check_sum1;
  }

  otherDataTmp[_cnt++] = check_sum1;
  otherDataTmp[_cnt++] = check_sum2;

  OtherSendDataAdd(otherDataTmp, _cnt);
}

/*============================================================================
******************************************************************************
******************************************************************************
数据接收相关函数
******************************************************************************
******************************************************************************
============================================================================*/
static void AnoDTDataAnl(u8 *data, u8 len);
//===========================================================
//根据不同通信方式，定义接收数据缓冲区
//===========================================================
typedef struct {
  u8 DT_RxBuffer[DT_RX_BUFNUM];
  u8 _data_len;
  u8 _data_cnt;
  u8 rxstate;
} _dt_rx_anl_st;
//===========================================================
//数据每接收一字节，调用本函数，进行数据解析
//===========================================================
u8 ano_dt_rec_data = 0;
void AnoDTRxOneByte(u8 data)
{
  static _dt_rx_anl_st rx_anl;

  switch (rx_anl.rxstate) {
  case 0: {
    if(data == 0xAA) {
      rx_anl.rxstate = 1;
      rx_anl.DT_RxBuffer[0] = data;
    }
  }
  break;

  case 1: {
    if((data == HW_TYPE || data == HW_ALL || data == 0x05)) {
      rx_anl.rxstate = 2;
      rx_anl.DT_RxBuffer[1] = data;
    } else {
      rx_anl.rxstate = 0;
    }
  }
  break;

  case 2: {
    rx_anl.rxstate = 3;
    rx_anl.DT_RxBuffer[2] = data;
  }
  break;

  case 3: {
    if(data < (DT_RX_BUFNUM - 5)) {
      rx_anl.rxstate = 4;
      rx_anl.DT_RxBuffer[3] = data;
      rx_anl._data_len = data;
      rx_anl._data_cnt = 0;
    } else {
      rx_anl.rxstate = 0;
    }
  }
  break;

  case 4: {
    if(rx_anl._data_len > 0) {
      rx_anl._data_len--;
      rx_anl.DT_RxBuffer[4 + rx_anl._data_cnt++] = data;

      if(rx_anl._data_len == 0) {
        rx_anl.rxstate = 5;
      }
    } else {
      rx_anl.rxstate = 0;
    }
  }
  break;

  case 5: {
    rx_anl.rxstate = 6;
    rx_anl.DT_RxBuffer[4 + rx_anl._data_cnt++] = data;
  }
  break;

  case 6: {
    u8 _data_cnt;
    rx_anl.rxstate = 0;
    rx_anl.DT_RxBuffer[4 + rx_anl._data_cnt] = data;
    _data_cnt = rx_anl._data_cnt + 5;
    AnoDTDataAnl(rx_anl.DT_RxBuffer, _data_cnt);
  }
  break;

  default : {
    rx_anl.rxstate = 0;
  }
  break;
  }
}
u8 test_cali_cnt;
//===========================================================
//AnoDTDataAnl函数是协议数据解析函数，函数参数是符合协议格式的一个数据帧，该函数会首先对协议数据进行校验
//===========================================================
static void AnoDTDataAnl(u8 *data, u8 len)
{
  u8 check_sum1 = 0, check_sum2 = 0;

  if(*(data + 3) != (len - 6)) {	//判断数据长度是否正确
    return;
  }

  for(u8 i = 0; i < len - 2; i++) {
    check_sum1 += *(data + i);
    check_sum2 += check_sum1;
  }

  if((check_sum1 != *(data + len - 2)) || (check_sum2 != *(data + len - 1))) {	//判断sum校验
    return;
  }

  if(*(data + 2) == 0XE0) {		//命令E0
    switch(*(data + 4)) {	//CID
    case 0x01: {
      if(*(data + 5) == 0x00) {
        if(*(data + 6) == 0x01) { //acc

        } else if(*(data + 6) == 0x02) { //gyro
        } else if(*(data + 6) == 0x03) { //horizontal

        } else if(*(data + 6) == 0x04) { //ecp

        } else if(*(data + 6) == 0x05) { //6 side

        } else if(*(data + 6) == 0x10) { //imu_reset

        } else if(*(data + 6) == 0x61) {	//存储航点

        } else if(*(data + 6) == 0x62) {	//清空航点

        } else if(*(data + 6) == 0xAA) {	//恢复默认PID
          PID_Rest();
          data_save();

        } else if(*(data + 6) == 0xAB) {	//恢复默认参数
          Parame_Reset(1);
          data_save();
        } else if(*(data + 6) == 0xAC) {	//恢复所有参数
          PID_Rest();
          Parame_Reset(2);
          data_save();
        }
      } else if(*(data + 5) == 0x01) {
        if(*(data + 6) == 0x01 ) { //飞控模式

        }
      } else if(*(data + 5) == 0x30) { //校准exe
        //校准命令
        if(*(data+6)==0x01 ) { //
//          if(flag.unlock_sta==0) {
//            test_cali_cnt++;
//            //开始6面校准
//						st_imu_cali.acc_cali_on = 1;
//            Ano_Parame.set.acc_calibrated = 0;
//          }
        } else if(*(data+6) == 0x02) {
          if(flag.unlock_sta==0) {
            //开始mag校准
            mag.mag_CALIBRATE = 2;
            Ano_Parame.set.mag_calibrated = 0;
          }
        }
      }
    }
    break;

    default:
      break;
    }
    //需返回CHECK帧
    SendCheck(SWJ_ADDR, *(data + 2), check_sum1, check_sum2);
  } else if(*(data + 2) == 0XE1) {
    //读取参数
    SendPar(SWJ_ADDR, *(u16*)(data + 4));

  } else if(*(data + 2) == 0xE2) {
    //写入参数
    AnoParWrite(*(u16*)(data + 4), *(s32*)(data + 6));
    //写入参数后需返回CHECK帧，告诉上位机参数写入成功
    SendCheck( SWJ_ADDR, *(data + 2), check_sum1, check_sum2);
  }
}


//===========================================================
//整体数据通信的调度器
//===========================================================
void dt_scheduler(void)
{

#if (DEBUG_CONFIG != CLOSE)

#if(DEBUG_CONFIG == UART)

  //数传响应
  int len = RingBuffer_GetCount(&U3rxring);
  u8 data = 0;

  for (; len != 0 ; len--) {
    RingBuffer_Pop(&U3rxring, &data);
    AnoDTRxOneByte(data);
  }
#elif (DEBUG_CONFIG == USB_CDC)

  static u8 usbdatarxbuf[100];

  u16 len = AnoUsbCdcRead(usbdatarxbuf,100);
  if(len) {
    for(u8 i=0; i<len; i++)
      AnoDTRxOneByte(usbdatarxbuf[i]);
  }
#endif

  //检查有没有非循环发送的数据需要发送
  OtherSendDataCheck();

  //检查循环发送的数据有没有需要发送的
  for(u8 i = 0; i < CSID_NUM; i++) {
    CheckDotMs(i);
  }

  //串口数据发送执行
  for(u8 i = 0; i < CSID_NUM; i++) {
    CheckDotWts(i);
  }

#endif


}
