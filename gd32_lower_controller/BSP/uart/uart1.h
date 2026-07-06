
#ifndef __UART1_H
#define __UART1_H

#include "sys.h"
#include <stdint.h>

void uart1NVICInit(void);
void uart1Init(u32 bound);
void uart1WriteBuf(uint8_t *buf, uint8_t len);

#endif

