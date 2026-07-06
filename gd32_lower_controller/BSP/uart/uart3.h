
#ifndef __UART3_H
#define __UART3_H

#include "sys.h"
#include <stdint.h>

extern volatile uint16_t distance_value;
extern volatile uint8_t confidence_value;
extern volatile uint8_t data_ready;

void uart3NVICInit(void);
void uart3Init(u32 bound);
void uart3WriteBuf(uint8_t *buf, uint8_t len);

#endif

