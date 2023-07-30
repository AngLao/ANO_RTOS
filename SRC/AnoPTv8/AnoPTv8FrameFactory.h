#ifndef __ANOPTV8FRAMEFACTORY_H
#define __ANOPTV8FRAMEFACTORY_H
#include "hardwareInterface.h"

void AnoPTv8SendCheck(uint8_t daddr, uint8_t id, uint8_t sc, uint8_t ac);
void AnoPTv8SendDevInfo(uint8_t daddr);
void AnoPTv8SendStr(uint8_t daddr, uint8_t string_color, char *str);
void AnoPTv8SendValStr(uint8_t daddr, float val, char *str);

void AnoPTv8SendParNum(uint8_t daddr);
void AnoPTv8SendParVal(uint8_t daddr, uint16_t parid);
void AnoPTv8SendParInfo(uint8_t daddr, uint16_t parid);

void AnoPTv8SendCmdNum(uint8_t daddr);
void AnoPTv8SendCmdInfo(uint8_t daddr, uint16_t cmd);

void AnoPTv8SendFCD01(uint8_t daddr);
void AnoPTv8SendFCD02(uint8_t daddr);
void AnoPTv8SendFCD03(uint8_t daddr);
void AnoPTv8SendFCD04(uint8_t daddr);
void AnoPTv8SendFCD05(uint8_t daddr);
void AnoPTv8SendFCD06(uint8_t daddr);
void AnoPTv8SendFCD07(uint8_t daddr);
void AnoPTv8SendFCD08(uint8_t daddr);
void AnoPTv8SendFCD09(uint8_t daddr);
void AnoPTv8SendFCD0A(uint8_t daddr);
void AnoPTv8SendFCD0B(uint8_t daddr);
void AnoPTv8SendFCD0C(uint8_t daddr);
void AnoPTv8SendFCD0D(uint8_t daddr);
void AnoPTv8SendFCD0E(uint8_t daddr);
void AnoPTv8SendFCD20(uint8_t daddr);
void AnoPTv8SendFCD21(uint8_t daddr);

void AnoPTv8SendBuf(uint8_t daddr, uint8_t fid, uint8_t *buf, uint16_t len);

#endif
