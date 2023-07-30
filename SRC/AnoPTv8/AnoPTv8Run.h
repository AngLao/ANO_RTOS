#ifndef __ANOPTV8RUN_H
#define __ANOPTV8RUN_H
#include "hardwareInterface.h"

#define ANOPTV8_FRAME_HEAD			0xAB
#define ANOPTV8_FRAME_HEADLEN 		6
#define ANOPTV8_FRAME_MAXDATALEN	100
#define ANOPTV8_FRAME_MAXFRAMELEN	ANOPTV8_FRAME_MAXDATALEN+ANOPTV8_FRAME_HEADLEN+2

#define LOG_COLOR_DEFAULT	0
#define LOG_COLOR_RED	  	1
#define LOG_COLOR_GREEN		2

#define SENDBUFLEN 			10

typedef struct {
    uint8_t head;
    uint8_t sdevid;
    uint8_t ddevid;
    uint8_t frameid;
    uint16_t datalen;
    uint8_t data[ANOPTV8_FRAME_MAXDATALEN];
    uint8_t sumcheck;
    uint8_t addcheck;
} __attribute__ ((__packed__)) _st_frame_v8;

typedef union {
    _st_frame_v8 frame;
    uint8_t rawBytes[sizeof(_st_frame_v8)];
} _un_frame_v8;

typedef struct
{
    uint8_t recvSta;
    uint16_t recvDataLen;
    _un_frame_v8 dataBuf;
} _recvST;


typedef struct
{
    uint8_t used;
    uint8_t readyToSend;
    _un_frame_v8 dataBuf;
} _sendST;

//·¢ËÍ»º³åÇø
extern _sendST AnoPTv8TxBuf[SENDBUFLEN];


void AnoPTv8RecvOneByte(uint8_t dat);
int AnoPTv8GetFreeTxBufIndex(void);
void AnoPTv8TxRunThread1ms(void);

int32_t AnoPTv8GetParVal(uint16_t parid);
void  AnoPTv8SetParVal(uint16_t parid, int32_t val);

#endif
