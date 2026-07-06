#ifndef __USART_H
#define __USART_H

#include "stdio.h"
#include "sys.h"
#include "message.h"
#include "led.h"
#include "motor.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//串口1初始化		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.csom
//修改日期:2011/6/14
//版本：V1.4
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了对printf的支持
//增加了串口接收命令功能.
//修正了printf第一个字符丢失的bug
//V1.4修改说明
//1,修改串口初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大允许接收的字节数(不大于2的14次方)
//4,修改了EN_USART1_RX的使能方式
////////////////////////////////////////////////////////////////////////////////// 	
#define USART_REC_LEN  			200  	//定义最大接收字节数 200
/* 使能（1）/禁止（0）串口接收：当前文件对应 USART2(PB10/PB11) */
#define EN_USART2_RX 			1


extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	

/* 最新一帧解析到的速度（vx, vy, vz），由串口接收中断侧更新 */
extern volatile float g_rx_vxyz[3];
/* 每更新一次 g_rx_vxyz 就自增一次（用于任务侧做一致性读取） */
extern volatile u32 g_rx_vxyz_seq;
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

/* 调试观测：USART2 接收路径状态 */
extern volatile uint32_t g_uart2_idle_cnt;
extern volatile uint32_t g_uart2_dma_ndtr;
extern volatile uint32_t g_uart2_fallback_bytes;
extern volatile uint8_t  g_uart2_last_fallback_byte;
/* 阻塞发送：通过当前调试串口（USART2）发送一段字节流 */
void UART_SendBytes(uint8_t *buf, uint16_t len);
void example_send_once(void);
#endif


