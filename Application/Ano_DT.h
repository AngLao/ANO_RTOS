#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "sysconfig.h"  
#define FRAME_HEAD		0xAA

#define LOG_COLOR_BLACK	0
#define LOG_COLOR_RED	  1
#define LOG_COLOR_GREEN	2

typedef enum
{
	CSID_X20,
	CSID_X21,
	CSID_X01,
	CSID_X02,
	CSID_X03,
	CSID_X04,
	CSID_X05,
	CSID_X06,
	CSID_X07,
	CSID_X08,
	CSID_X09,
	CSID_X0A,
	CSID_X0B,
	CSID_X0C,
	CSID_X0D,
	CSID_X0E,
	CSID_X0F,
	CSID_X30,
	CSID_X32,
	CSID_X33,
	CSID_X34,
	CSID_X40,
	CSID_X41,
	CSID_XFA,
	CSID_NUM
} _enu_cyclesendid;




extern u8 ano_dt_rec_data ;

void AnoDTRxOneByte(u8 data);
void AnoDTRxOneByteUart( u8 data );
void AnoDTRxOneByteUsb( u8 data );
void AnoDTSendStr(u8 dest_addr, u8 string_color,char *str);
void ANO_DT_Init(void);
void ANO_DT_Task1Ms(void);



#define		 ANO_DT_SendString(str ) 	AnoDTSendStr(LOG_COLOR_GREEN, 1,str)

#endif

