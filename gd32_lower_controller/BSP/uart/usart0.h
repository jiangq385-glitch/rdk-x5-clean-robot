
#ifndef __UART0_H
#define __UART0_H

#include "sys.h"
#include <stdint.h>

void uart0NVICInit(void);
void uart0Init(u32 bound);
void uart0WriteBuf(uint8_t *buf, uint8_t len);

#endif

