// #ifndef __ROBOT_INIT_H
// #define __ROBOT_INIT_H

#include "main.h"
#include "gd32f4xx_misc.h"

		
 void Bsp_init(void)
{
    //硬件初始化
    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);  //设置中断优先级分组：4位抢占，0位响应
    delay_init(168);                                 //初始化延时(基于168MHz时钟)
     bsp_can_init();                                  //初始化CAN
     LED_Init();                                      //初始化LED硬件
    KEY_Init();                                      //初始化按键
   uart_init(115200);                                  //初始化USART2，波特率115200
   uart3Init(115200);                                  //初始化UART3，激光测距模块
    uart1Init(9600);                                  //初始化UART1，波特率9600
   UART4_Init(115200);
    //uart5Init(115200);
    
   IM948_Init();
                //CAN回环自检
    SPI0_Init();                                        //初始化SPI0
    AS5048_init(1, SPI0, GPIOA, GPIO_PIN_4); //初始化 AS5048 编码器 1（SPI0 + PA4 片选）    
    Encoder_Init(1);
    Encoder_Init(2);
}

// #endif
