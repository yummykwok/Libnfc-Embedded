
#ifndef __CORX_STM32_F107_UART__
#define __CORX_STM32_F107_UART__

#include <stm32f10x.h>
#include <stm32f10x_it.h>
#include "log.h"

extern u8 flag_U1_rev_finish;
extern u8 RxBufferU1[30];


void InitUart();
void PutToLogBuffer(u8 ch);
int ReadByte(u32 timeout);
int SendByte(u8 byte, u32 timeout);
void SendDataU1(u8 *data,u8 length);
#endif//__CORX_STM32_F107_UART__

