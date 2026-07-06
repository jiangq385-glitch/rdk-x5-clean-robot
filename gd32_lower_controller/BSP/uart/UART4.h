#ifndef __UART4_H
#define __UART4_H

#include "sys.h"

void UART4_Init(u32 bound);
//void UART4_Printf(char *format, ...);
int UART_Write(u8 n, const u8 *buf, int Len);

/* 调试/示例：按官方例程方式触发一次中断发送（发送 transmitter_buffer），并在中断里收满 receiver_buffer */

#endif

