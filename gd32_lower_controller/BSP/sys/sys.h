#ifndef __SYS_H
#define __SYS_H	
#include <stdint.h>
#include "gd32f4xx.h"

/* 兼容常见 BSP 写法(u8/u16/u32) */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;

/* 是否启用 OS 支持(FreeRTOS)。工程可在编译宏里覆盖为 0/1。 */
#ifndef SYSTEM_SUPPORT_OS
#define SYSTEM_SUPPORT_OS 1
#endif


//以下为汇编/内核相关函数
void WFI_SET(void);		//执行WFI指令
void INTX_DISABLE(void);	//关闭所有中断
void INTX_ENABLE(void);	//开启所有中断
void MSR_MSP(uint32_t addr);	//设置主堆栈指针(MSP)

#endif
