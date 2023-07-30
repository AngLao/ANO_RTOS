#ifndef __HARDWAREINTERFACE_H
#define __HARDWAREINTERFACE_H
#include <stdint.h>

//获取超过一字节数据的每字节数据，例如int16、int32、float等
#define BYTE0(dwTemp) (*((char *)(&dwTemp)))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))
	
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )



void AnoPTv8HwSendBytes(uint8_t *buf, uint16_t len);
void AnoPTv8HwRecvByte(uint8_t dat);
void AnoPTv8HwTrigger1ms(void);

void ano_helper_task(void *pvParameters);
#endif
