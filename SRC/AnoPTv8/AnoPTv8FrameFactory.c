#include "AnoPTv8FrameFactory.h"
#include "AnoPTv8.h"
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
发送数据时，根据数据帧计算sumcheck和addcheck
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8CalFrameCheck(_un_frame_v8 *p)
{
    uint8_t sumcheck = 0, addcheck = 0;
    for (uint16_t i = 0; i < (ANOPTV8_FRAME_HEADLEN + p->frame.datalen); i++)
    {
        sumcheck += p->rawBytes[i];
        addcheck += sumcheck;
    }
    p->rawBytes[ANOPTV8_FRAME_HEADLEN + p->frame.datalen] = sumcheck;
    p->rawBytes[ANOPTV8_FRAME_HEADLEN + p->frame.datalen + 1] = addcheck;
    p->frame.sumcheck = sumcheck;
    p->frame.addcheck = addcheck;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
校验帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendCheck(uint8_t daddr, uint8_t id, uint8_t sc, uint8_t ac)
{
	uint8_t _cnt = 0;
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x00;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = id;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = sc;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = ac;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
设备信息帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendDevInfo(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE3;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = ANOPTV8_MYDEVID;
		uint16_t _tmp = ANOPTV8_HWVER;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		_tmp = ANOPTV8_SWVER;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		_tmp = ANOPTV8_BLVER;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		_tmp = ANOPTV8_PTVER;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		uint8_t i=0;
		char *str = "ANO_DEVTEST";
		while(*(str+i) != '\0')//单引号字符，双引号字符串
		{
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str+i++);
			if(i > 20)
			{
				break;
			}
		}
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
字符串信息发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendStr(uint8_t daddr, uint8_t string_color, char *str)
{
	uint8_t _cnt = 0;
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xA0;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = string_color;
		uint8_t i=0;
		while(*(str+i) != '\0')//单引号字符，双引号字符串
		{
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str+i++);
			if(i > 50)
			{
				break;
			}
		}
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendValStr(uint8_t daddr, float val, char *str)
{
	uint8_t _cnt = 0;
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xA1;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(val);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(val);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(val);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(val);
		uint8_t i=0;
		while(*(str+i) != '\0')//单引号字符，双引号字符串
		{
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(str+i++);
			if(i > 50)
			{
				break;
			}
		}
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
参数类帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendParNum(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE0;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 1;
		uint16_t _tmp = AnoPTv8GetParCount();
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendParVal(uint8_t daddr, uint16_t parid)
{
	uint8_t _cnt = 0;
	
	if(parid >= AnoPTv8GetParCount())
		return;
		
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE1;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(parid);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(parid);
		int32_t _tmp = AnoPTv8GetParVal(parid);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(_tmp);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendParInfo(uint8_t daddr, uint16_t parid)
{
	uint8_t _cnt = 0;
	
	if(parid >= AnoPTv8GetParCount())
		return;
		
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xE2;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(parid);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(parid);
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(AnoParInfoList[parid].min);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(AnoParInfoList[parid].min);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(AnoParInfoList[parid].min);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(AnoParInfoList[parid].min);
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(AnoParInfoList[parid].max);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(AnoParInfoList[parid].max);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(AnoParInfoList[parid].max);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(AnoParInfoList[parid].max);
		
		for(uint8_t i=0; i<20; i++)
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoParInfoList[parid].name[i];
		uint8_t i=0;
		while(AnoParInfoList[parid].info[i] != '\0')//单引号字符，双引号字符串
		{
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoParInfoList[parid].info[i++];
			if(i > 50)
			{
				break;
			}
		}
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令类帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendCmdNum(uint8_t daddr)
{
	uint8_t _cnt = 0;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xC1;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;
		uint16_t _tmp = AnoPTv8GetCmdCount();
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(_tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendCmdInfo(uint8_t daddr, uint16_t cmd)
{
	uint8_t _cnt = 0;
	
	if(cmd >= AnoPTv8GetCmdCount())
		return;
		
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0xC2;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(cmd);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(cmd);
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdid0;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdid1;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdid2;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval0;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval1;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval2;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval3;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval4;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval5;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval6;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].cmd.cmdval7;
		
		for(uint8_t i=0; i<20; i++)
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].name[i];
		uint8_t i=0;
		while(AnoCmdInfoList[cmd].info[i] != '\0')//单引号字符，双引号字符串
		{
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = AnoCmdInfoList[cmd].info[i++];
			if(i > 50)
			{
				break;
			}
		}
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
飞控数据帧发送
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void AnoPTv8SendFCD01(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x01;
		/***********************************************************************/
		temp_data = (int16_t)(DD_DAT_ACC_RAW_X);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		temp_data = (int16_t)(DD_DAT_ACC_RAW_Y);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		temp_data = (int16_t)(DD_DAT_ACC_RAW_Z);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		temp_data = (int16_t)(DD_DAT_GYR_RAW_X);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);
		temp_data = (int16_t)(DD_DAT_GYR_RAW_Y);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);
		temp_data = (int16_t)(DD_DAT_GYR_RAW_Z);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);	
		//STATE
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_SHOCK_STA;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD02(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	int32_t temp_data_32;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x02;
		/***********************************************************************/
		//ECP
		temp_data = (int16_t)(DD_DAT_ECP_RAW_X );
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		temp_data = (int16_t)(DD_DAT_ECP_RAW_Y );
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);	
		temp_data = (int16_t)(DD_DAT_ECP_RAW_Z );
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);				
		//BARO_ALT
		temp_data_32 = (int32_t)(DD_DAT_BARO_ALT );
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(temp_data_32);		
		//temperature
		temp_data = (int16_t)(DD_DAT_TEMPERATURE );
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		//BARO_STA
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_BARO_STA;
		//ECP_STA
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_ECP_STA;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD03(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x03;
		/***********************************************************************/
		//ROL PIT YAW
		temp_data = (int16_t)(DD_DAT_ANGLE_ROL*100);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);			
		temp_data = (int16_t)(DD_DAT_ANGLE_PIT*100);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);				
		temp_data = (int16_t)(DD_DAT_ANGLE_YAW*100);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);	
		//FUS_STA
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_ATT_FUSION_STA;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD04(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x04;
		/***********************************************************************/
		//ATT QUA
		temp_data = (int16_t)(DD_DAT_QUA0_10K);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);			
		temp_data = (int16_t)(DD_DAT_QUA1_10K);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);				
		temp_data = (int16_t)(DD_DAT_QUA2_10K);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);	
		temp_data = (int16_t)(DD_DAT_QUA3_10K);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_ATT_FUSION_STA;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD05(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int32_t temp_data_32;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x05;
		/***********************************************************************/
		//HEIGHT
		temp_data_32 = (int32_t)(DD_DAT_ALT_FU);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data_32);		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(temp_data_32);				
		temp_data_32 = (int32_t)(DD_DAT_ALT_ADD);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data_32);		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(temp_data_32);		
		//
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_ALT_STA;
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD06(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x06;
		/***********************************************************************/
		//MODE	LOCKED	FUN	CMD
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_FC_MODE;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_FC_LOCKED;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_FC_CMD_ID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_FC_CMD_0;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_FC_CMD_1;		
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD07(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x07;
		/***********************************************************************/
		//velocity
		temp_data = (int16_t)(DD_DAT_HCA_VEL_X);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);			
		temp_data = (int16_t)(DD_DAT_HCA_VEL_Y);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);				
		temp_data = (int16_t)(DD_DAT_HCA_VEL_Z);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);				
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD08(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int32_t temp_data_32;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x08;
		/***********************************************************************/
		//pos
		temp_data_32 = (int32_t)(DD_DAT_ULHCA_POS_X);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data_32);		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(temp_data_32);				
		temp_data_32 = (int32_t)(DD_DAT_ULHCA_POS_Y);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data_32);		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE2(temp_data_32);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE3(temp_data_32);	
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD09(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x09;
		/***********************************************************************/
		//wind_velocity
		temp_data = (int16_t)(DD_DAT_HCA_WIND_X);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);			
		temp_data = (int16_t)(DD_DAT_HCA_WIND_Y);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD0A(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x0A;
		/***********************************************************************/

		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD0B(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x0B;
		/***********************************************************************/

		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD0C(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x0C;
		/***********************************************************************/

		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD0D(uint8_t daddr)
{
	uint8_t _cnt = 0;
	int16_t temp_data;
	
	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x0D;
		/***********************************************************************/
		//电压
		temp_data = (int16_t)(DD_DAT_PWR_VOTAGE);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(temp_data);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(temp_data);	
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = 0;	
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD0E(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x0E;
		/***********************************************************************/
		//传感器状态	
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_SENSTA_VEL;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_SENSTA_POS;	
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_SENSTA_GPS;		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = DD_DAT_SENSTA_ALT;	
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD20(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x20;
		/***********************************************************************/
		//pwm1-8
		uint16_t tmp = DD_DAT_PWM_1;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_PWM_2;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_PWM_3;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_PWM_4;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
void AnoPTv8SendFCD21(uint8_t daddr)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = 0x21;
		/***********************************************************************/
		//ATT_Ctrl
		int16_t tmp = DD_DAT_CTRL_ROL;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_CTRL_PIT;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_CTRL_THR;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		tmp = DD_DAT_CTRL_YAW;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE0(tmp);
		AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = BYTE1(tmp);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}

void AnoPTv8SendBuf(uint8_t daddr, uint8_t fid, uint8_t *buf, uint16_t len)
{
	uint8_t _cnt = 0;

	int bufindex = AnoPTv8GetFreeTxBufIndex();
	
	if (bufindex >= 0)
	{
		AnoPTv8TxBuf[bufindex].used = 1;
		
		AnoPTv8TxBuf[bufindex].dataBuf.frame.head = ANOPTV8_FRAME_HEAD;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.sdevid = ANOPTV8_MYDEVID;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.ddevid = daddr;
		AnoPTv8TxBuf[bufindex].dataBuf.frame.frameid = fid;
		/***********************************************************************/
		for(uint16_t i=0; i<len; i++)
			AnoPTv8TxBuf[bufindex].dataBuf.frame.data[_cnt++] = *(buf+i);
		/***********************************************************************/
		AnoPTv8TxBuf[bufindex].dataBuf.frame.datalen = _cnt;
		AnoPTv8CalFrameCheck(&AnoPTv8TxBuf[bufindex].dataBuf);
		AnoPTv8TxBuf[bufindex].readyToSend = 1;
	}
}
