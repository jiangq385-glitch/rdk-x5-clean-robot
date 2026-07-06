
#ifndef __UART5_H
#define __UART5_H

#include "sys.h"
#include <stdint.h>

void uart5NVICInit(void);
void uart5Init(u32 bound);
void uart5WriteBuf(uint8_t *buf, uint8_t len);

#endif

