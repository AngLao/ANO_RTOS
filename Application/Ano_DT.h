#ifndef _DATA_TRANSFER_H
#define	_DATA_TRANSFER_H

#include "sysconfig.h"
#define FRAME_HEAD		0xAA

#define LOG_COLOR_BLACK	0
#define LOG_COLOR_RED	  1
#define LOG_COLOR_GREEN	2

 
#define CLOSE 1
#define USB_CDC 2
#define UART 3

//数传输出硬件选择
#define DEBUG_CONFIG UART

typedef enum {
  CSID_X01,
  CSID_X02,
  CSID_X03, 
  CSID_X05,
  CSID_X06,
  CSID_X07, 
  CSID_X0B, 
  CSID_X0D,
  CSID_X0E,  
  CSID_X20, 
  CSID_X32,
  CSID_X40,
  CSID_X51,
  CSID_NUM
} _enu_cyclesendid;

extern u8 ano_dt_rec_data ;

void AnoDTRxOneByte(u8 data);
void AnoDTRxOneByteUart( u8 data );
void AnoDTRxOneByteUsb( u8 data );
void AnoDTSendStr(u8 dest_addr, u8 string_color, char *str);
void ANO_DT_Init(void);
void dt_scheduler(void);



#define		 debugOutput(str) 	AnoDTSendStr(0xFF, LOG_COLOR_RED , str )

#endif

