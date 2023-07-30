#include "AnoPTv8Run.h"
#include "AnoPTv8.h"
//接收缓冲
_recvST recvBuf;
//发送缓冲
_sendST AnoPTv8TxBuf[SENDBUFLEN];

int8_t 	anoPTv8FrameCheck(const _un_frame_v8 *p);
void 	anoPTv8FrameAnl(const _un_frame_v8 *p);
void 	anoPTv8CmdFrameAnl(const _un_frame_v8 *p);

void AnoPTv8RecvOneByte(uint8_t dat)
{
	if (recvBuf.recvSta == 0)
    {
        if (dat == ANOPTV8_FRAME_HEAD)
        {
            recvBuf.recvSta = 1;
            recvBuf.recvDataLen = 1;
            recvBuf.dataBuf.rawBytes[0] = dat;
        }
    }
    else if (recvBuf.recvSta == 1)
    {
        recvBuf.dataBuf.rawBytes[recvBuf.recvDataLen++] = dat;
        if (recvBuf.recvDataLen >= ANOPTV8_FRAME_HEADLEN)
            recvBuf.recvSta = 2;
    }
    else if (recvBuf.recvSta == 2)
    {
        recvBuf.dataBuf.rawBytes[recvBuf.recvDataLen++] = dat;
        if (recvBuf.recvDataLen >= ANOPTV8_FRAME_HEADLEN + recvBuf.dataBuf.frame.datalen)
            recvBuf.recvSta = 3;
    }
    else if (recvBuf.recvSta == 3)
    {
        recvBuf.dataBuf.rawBytes[recvBuf.recvDataLen++] = dat;
        recvBuf.dataBuf.frame.sumcheck = dat;
        recvBuf.recvSta = 4;
    }
    else if (recvBuf.recvSta == 4)
    {
        recvBuf.dataBuf.rawBytes[recvBuf.recvDataLen++] = dat;
        recvBuf.dataBuf.frame.addcheck = dat;
        recvBuf.recvSta = 0;
        //一帧数据接收完毕，进行校验和解析
        if (anoPTv8FrameCheck(&recvBuf.dataBuf))
            anoPTv8FrameAnl(&recvBuf.dataBuf);
    }
    else
    {
        recvBuf.recvSta = 0;
    }
}

static int8_t anoPTv8FrameCheck(const _un_frame_v8 *p)
{
	uint8_t sumcheck = 0, addcheck = 0;
    if (p->frame.head != ANOPTV8_FRAME_HEAD)
        return 0;
    for (uint16_t i = 0; i < (ANOPTV8_FRAME_HEADLEN + p->frame.datalen); i++)
    {
        sumcheck += p->rawBytes[i];
        addcheck += sumcheck;
    }
    if (sumcheck == p->frame.sumcheck && addcheck == p->frame.addcheck)
        return 1;
    else
        return 0;
}

static void anoPTv8FrameAnl(const _un_frame_v8 *p)
{
	switch (p->frame.frameid)
    {
    case 0xC0:
        //CMD命令
		anoPTv8CmdFrameAnl(p);
        AnoPTv8SendCheck(p->frame.sdevid, p->frame.frameid, p->frame.sumcheck, p->frame.addcheck);
        break;
    case 0xC1:
        //CMD读取命令
        if (p->frame.data[0] == 0)
        {
            //读取CMD个数
            AnoPTv8SendCmdNum(p->frame.sdevid);
        }
        else if (p->frame.data[0] == 1)
        {
            //读取CMD信息
            AnoPTv8SendCmdInfo(p->frame.sdevid, *(uint16_t *)(p->frame.data+1));
        }
        break;
    case 0xE0:
        //参数命令
        if (p->frame.data[0] == 0)
        {
            //读取设备信息
            AnoPTv8SendDevInfo(p->frame.sdevid);
        }
        else if (p->frame.data[0] == 1)
        {
            //读取参数个数
            AnoPTv8SendParNum(p->frame.sdevid);
        }
        else if (p->frame.data[0] == 2)
        {
            //读取参数值
            AnoPTv8SendParVal(p->frame.sdevid, *(uint16_t *)(p->frame.data+1));
        }
        else if (p->frame.data[0] == 3)
        {
            //读取参数信息
            AnoPTv8SendParInfo(p->frame.sdevid, *(uint16_t *)(p->frame.data+1));
        }
        break;
    case 0xE1:
        //参数写入
		AnoPTv8SetParVal(*(uint16_t *)(p->frame.data+0), *(int32_t *)(p->frame.data+2));
		AnoPTv8SendCheck(p->frame.sdevid, p->frame.frameid, p->frame.sumcheck, p->frame.addcheck);
        break;
    }
}

int AnoPTv8GetFreeTxBufIndex(void)
{
    static uint16_t lastIndex = 0;
    int ret = -1;

    for(int i=0; i<SENDBUFLEN; i++)
    {
        if(AnoPTv8TxBuf[lastIndex].used == 0)
        {
            AnoPTv8TxBuf[lastIndex].used = 1;
            ret = lastIndex;
            return ret;
        }
		else
		{
			lastIndex++;
			while(lastIndex >= SENDBUFLEN)
				lastIndex -= SENDBUFLEN;
		}
		
    }
    return -1;
}

void AnoPTv8TxRunThread1ms(void)
{
	static uint16_t lastIndex = 0;
	
	for(int i=0; i<SENDBUFLEN; i++)
    {
        if(AnoPTv8TxBuf[lastIndex].used == 1 && AnoPTv8TxBuf[lastIndex].readyToSend == 1)
        {
            //
			AnoPTv8HwSendBytes(AnoPTv8TxBuf[lastIndex].dataBuf.rawBytes, AnoPTv8TxBuf[lastIndex].dataBuf.frame.datalen+ANOPTV8_FRAME_HEADLEN+2);
			AnoPTv8TxBuf[lastIndex].used = 0;
            AnoPTv8TxBuf[lastIndex].readyToSend = 0;
			
            return ;
        }
		else
		{
			lastIndex++;
			while(lastIndex >= SENDBUFLEN)
				lastIndex -= SENDBUFLEN;
		}
    }
}


//
int32_t AnoPTv8GetParVal(uint16_t parid)
{
	if(parid < AnoPTv8GetParCount())
		return * AnoParInfoList[parid].pval;
	else
		return 0;
}

void  AnoPTv8SetParVal(uint16_t parid, int32_t val)
{
	if(parid < AnoPTv8GetParCount())
		* AnoParInfoList[parid].pval = LIMIT(val, AnoParInfoList[parid].min, AnoParInfoList[parid].max);
}

//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
命令帧解析，根据CmdID0、CmdID1、CmdID2，判断指令功能，然后根据函数指针，执行对应的功能函数
*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void anoPTv8CmdFrameAnl(const _un_frame_v8 *p)
{
    for(uint16_t i=0; i<AnoPTv8GetCmdCount(); i++)
    {
        if(p->frame.data[0] == AnoCmdInfoList[i].cmd.cmdid0 && p->frame.data[1] == AnoCmdInfoList[i].cmd.cmdid1 && p->frame.data[2] == AnoCmdInfoList[i].cmd.cmdid2)
        {
            AnoCmdInfoList[i].pFun(p);
            return;
        }
    }
}


