/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
  * 作者   ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：光流数据解析
**********************************************************************************/
#include "Ano_OF.h"
#include "Ano_FcData.h"

/*
MODE：2表示本帧为惯导融合的光流数据
STATE：状态标记位， 0：无效，1：有效。
DX_2、DY_2：X、Y轴的光流信息，对应融合的移动的速度（地面速度,单位：厘米/秒）；
DX_FIX、DY_FIX：修正后的X、Y轴的移动速度，适用于积分计算（地面速度,单位：厘米/秒）；
INTEG_X、INTEG_Y：X、Y轴的速度积分值（单纯积分，仅供参考，单位厘米，-32768~+32767 循环）。
QUALITY：光流数据质量，数值越大，表示光流数据质量越好（0-255），仅供参考。
*/
//光流信息质量：QUA 
uint8_t 	OF_STATE, OF_QUALITY;
//原始光流信息 
int8_t	OF_DX, OF_DY;
//融合后的光流信息 
int16_t	OF_DX2, OF_DY2, OF_DX2FIX, OF_DY2FIX, OF_INTEG_X, OF_INTEG_Y;
//原始高度信息和融合后高度信息
uint32_t	OF_ALT, OF_ALT2;
//原始陀螺仪数据
int16_t	OF_GYR_X, OF_GYR_Y, OF_GYR_Z;
//原始加速度数据
int16_t	OF_ACC_X, OF_ACC_Y, OF_ACC_Z;

static uint8_t _datatemp[50];
static u8 _data_cnt = 0;

void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num);

//AnoOF_GetOneByte是初级数据解析函数，串口每接收到一字节光流数据，调用本函数一次，函数参数就是串口收到的数据
//当本函数多次被调用，最终接收到完整的一帧数据后，会自动调用数据解析函数AnoOF_DataAnl
void AnoOF_GetOneByte(uint8_t data)
{
  static u8 _data_len = 0;
  static u8 state = 0;

  if(state == 0 && data == 0xAA) {
    state = 1;
    _datatemp[0] = data;
  } else if(state == 1 && data == 0xFF) {		//目的地址
    state = 3;
    _datatemp[1] = data;
  } else if(state == 3) {		//功能字
    state = 4;
    _datatemp[2] = data;
  } else if(state == 4) {		//长度
    state = 5;
    _datatemp[3] = data;
    _data_len = data;
    _data_cnt = 0;
  } else if(state == 5 && _data_len > 0) {
    _data_len--;
    _datatemp[4 + _data_cnt++] = data;

    if(_data_len == 0)
      state = 6;
  } else if(state == 6) {
    state = 0;
    _datatemp[4 + _data_cnt] = data;
    AnoOF_DataAnl(_datatemp, _data_cnt + 5); //anoof_data_ok = 1 ;//
  } else
    state = 0;
}
static u8 of_check_f[2];
static u16 of_check_cnt[2] = { 10000, 10000 };
void AnoOF_Check(u8 dT_ms)
{
  for(u8 i = 0; i < 2; i++) {
    if(of_check_f[i] == 0 ) {
      if(of_check_cnt[i] < 10000) {
        of_check_cnt[i] += dT_ms;
      }
    } else {
      of_check_cnt[i] = 0;
    }


    of_check_f[i] = 0;
  }


  if(of_check_cnt[0] > 1000 || of_check_cnt[1] > 1000) {
    sens_hd_check.of_ok = 0;
  } else {
    sens_hd_check.of_ok = 1;
  }



}

//AnoOF_DataAnl为光流数据解析函数，可以通过本函数得到光流模块输出的各项数据
void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num)
{
  u8 sum = 0;

  for(u8 i = 0; i < (num - 1); i++)
    sum += *(data_buf + i);

  if(!(sum == *(data_buf + num - 1)))
    return;

  if(*(data_buf + 2) == 0X51) { //光流信息
    if(*(data_buf + 4) == 0) { //原始光流信息
      OF_STATE 		= *(data_buf + 5);
      OF_DX  		= *(data_buf + 6);
      OF_DY  		= *(data_buf + 7);
      OF_QUALITY  	= *(data_buf + 8);
    } else if(*(data_buf + 4) == 2) { //融合后光流信息
      OF_STATE 		= *(data_buf + 5);
      OF_DX2		= (int16_t)(*(data_buf + 7) << 8) | *(data_buf + 6) ;
      OF_DY2		= (int16_t)(*(data_buf + 9) << 8) | *(data_buf + 8) ;
      OF_DX2FIX	= (int16_t)(*(data_buf + 11) << 8) | *(data_buf + 10) ;
      OF_DY2FIX	= (int16_t)(*(data_buf + 13) << 8) | *(data_buf + 12) ;
      OF_INTEG_X	= (int16_t)(*(data_buf + 15) << 8) | *(data_buf + 14) ;
      OF_INTEG_Y	= (int16_t)(*(data_buf + 17) << 8) | *(data_buf + 16) ;
      OF_QUALITY  	= *(data_buf + 18);

      of_check_f[0] = 1;
      of_init_type = 1;
    }
  }

  //原始高度信息
  if(*(data_buf + 2) == 0X34) {
    OF_ALT = (uint32_t)(*(data_buf + 10) << 24) | *(data_buf + 9) << 16 | *(data_buf + 8) << 8 | *(data_buf + 7);
//      OF_ALT += 4;
    of_check_f[1] = 1;
  }

//  if(*(data_buf + 2) == 0x01) { //惯性数据
//    OF_ACC_X = (int16_t)(*(data_buf + 5) << 8) | *(data_buf + 4) ;
//    OF_ACC_Y = (int16_t)(*(data_buf + 7) << 8) | *(data_buf + 6) ;
//    OF_ACC_Z = (int16_t)(*(data_buf + 9) << 8) | *(data_buf + 8) ;

//    OF_GYR_X = (int16_t)(*(data_buf + 11) << 8) | *(data_buf + 10) ;
//    OF_GYR_Y = (int16_t)(*(data_buf + 13) << 8) | *(data_buf + 12) ;
//    OF_GYR_Z = (int16_t)(*(data_buf + 15) << 8) | *(data_buf + 14) ;
//  }

}

