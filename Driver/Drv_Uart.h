#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"
#include "ring_buffer.h"


#define USER_SERIAL_1  0
#define USER_SERIAL_2  1
#define USER_SERIAL_3  1
#define USER_SERIAL_4  1
#define USER_SERIAL_5  1
 

#if(USER_SERIAL_1 == 1)


void Drv_Uart1Init(uint32_t baudrate);
void Drv_Uart1SendBuf(u8 *data, u8 len);
void Drv_Uart1TxCheck(void);

#endif


#if(USER_SERIAL_2 == 1)
 
extern RINGBUFF_T uwbRingBuff;//外部环形缓冲区声明

void Drv_Uart2Init(uint32_t baudrate);
void Drv_Uart2SendBuf(u8 *data, u8 len);
void Drv_Uart2TxCheck(void);

#endif


#if(USER_SERIAL_3 == 1)

extern RINGBUFF_T U3rxring;

void Drv_Uart3Init(uint32_t baudrate);
void Drv_Uart3SendBuf(u8 *data, u8 len);
void Drv_Uart3TxCheck(void);

#endif


#if(USER_SERIAL_4 == 1)

void Drv_Uart4Init(uint32_t baudrate);
void Drv_Uart4SendBuf(u8 *data, u8 len);
void Drv_Uart4TxCheck(void);

#endif


#if(USER_SERIAL_5 == 1)
 
extern u8 openMV_res ;

void Drv_Uart5Init(uint32_t baudrate);
void Drv_Uart5SendBuf(u8 *data, u8 len);
void Drv_Uart5TxCheck(void);

#endif

#endif
