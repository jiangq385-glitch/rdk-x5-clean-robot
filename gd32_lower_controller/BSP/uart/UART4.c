// 该文件实现 IMU(UART) 初始化、发送，以及接收中断处理（接收字节后交给 Cmd_GetPkt 组包）

#include "uart4.h"            // 本模块对外的头文件（声明 UART6_Init / UART_Write 等）
#include <stdarg.h>            // 可变参数相关（下方注释掉的 printf 风格串口输出曾用到）
#include "uart4.h"            // 重复包含：保留原样不改动逻辑（一般可删除，但此处仅做注释说明）
#include <stdarg.h>            // 重复包含：同上
#include <stdio.h>             // vsprintf 等格式化函数（用于注释掉的调试输出示例）


#include "im948_CMD.h"        // IM948 协议解析/组包接口（Cmd_GetPkt）

/*
 * 说明：
 * - 当前 IM948 使用 UART4：PC12(TX) / PD2(RX)（复用 AF8）。
 * - 仍保留外部接口函数名 UART6_Init / UART_Write 不变（上层代码无需改名）。
 
#define BSP_IMU_UARTx           UART4      // 选用的 UART 外设实例（这里映射到 UART4）
#define BSP_IMU_UART_CLK        RCU_UART4  // UART4 的外设时钟
#define BSP_IMU_UART_IRQn       UART4_IRQn // UART4 的中断号
#define BSP_IMU_UART_AF         GPIO_AF_8  // GPIO 复用功能选择（UART4 常用 AF8）

#define BSP_IMU_TX_PORT         GPIOC       // TX 所在 GPIO 端口：C 口
#define BSP_IMU_TX_CLK          RCU_GPIOC   // GPIOC 时钟
#define BSP_IMU_TX_PIN          GPIO_PIN_12 // TX 引脚：PC12

#define BSP_IMU_RX_PORT         GPIOD       // RX 所在 GPIO 端口：D 口
#define BSP_IMU_RX_CLK          RCU_GPIOD   // GPIOD 时钟
#define BSP_IMU_RX_PIN          GPIO_PIN_2  // RX 引脚：PD2
*/
void UART4_Init(u32 bound) // 初始化 IMU 串口（实际为 UART4），并打开接收中断
{
    /* clocks */
    rcu_periph_clock_enable(RCU_GPIOC);    // 使能 TX GPIO 端口时钟（GPIOC）
    rcu_periph_clock_enable(RCU_GPIOD);    // 使能 RX GPIO 端口时钟（GPIOD）
    rcu_periph_clock_enable(RCU_UART4);    // 使能 UART 外设时钟（UART4）

    /* GPIO: PC12(TX) / PD2(RX) */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);
    gpio_af_set(GPIOC, GPIO_AF_8, GPIO_PIN_12);

    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
    gpio_af_set(GPIOD, GPIO_AF_8, GPIO_PIN_2);

    /* USART(UART) configure */
    usart_deinit(UART4);
    usart_baudrate_set(UART4, bound);
    usart_word_length_set(UART4, USART_WL_8BIT);
    usart_stop_bit_set(UART4, USART_STB_1BIT);
    usart_parity_config(UART4, USART_PM_NONE);
    usart_transmit_config(UART4, USART_TRANSMIT_ENABLE);
    usart_receive_config(UART4, USART_RECEIVE_ENABLE);

    /* RX interrupt */
    nvic_irq_enable(UART4_IRQn, 6, 0);                    // 使能 UART4 中断，抢占优先级 6，子优先级 0
    usart_interrupt_enable(UART4, USART_INT_RBNE);        // 使能接收缓冲非空（RBNE）中断
    usart_enable(UART4);                                 // 最后使能 UART 外设
} // UART6_Init end
//{
//    uint8_t i =0;
//    char String[100];    
//    va_list arg;                    
//    va_start(arg, format);        
//    vsprintf(String, format, arg);    
//    va_end(arg);                    
// 
//    for (i = 0; String[i] != '\0'; i ++)
//    {
//        USART_SendData(USART6, String[i]);    
//        while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);    
//    }
//}

int UART_Write(u8 n, const u8 *buf, int Len) // 串口发送函数：按通道号 n 选择外设并发送 buf[0..Len-1]
{
    int i;                                // 发送循环计数变量
    switch (n)                            // 根据通道号选择要写入的 UART
    {
    case 2:                               // 通道 2：这里约定为“UART6”（实际硬件映射为 UART4，用于 IM948）
        for (i = 0; i < Len; i++)         // 逐字节发送 Len 个数据
        {
            while (RESET == usart_flag_get(UART4 , USART_FLAG_TBE)) { // 等待发送数据寄存器空（TBE=1）
            }
            usart_data_transmit(UART4, buf[i]); // 写入一个字节到发送寄存器
        }
        while (RESET == usart_flag_get(UART4, USART_FLAG_TC)) {     // 等待最后一个字节发送完成（TC=1）
        }
        break;                           // 结束该 case

    default:                             // 其他通道号：当前未实现
        return 0;                        // 返回 0 表示未发送
    }

    return Len;                          // 返回实际发送的长度（这里固定等于 Len）
} // UART_Write end

// IM948：UART4 接收中断服务函数（每来一个字节就交给 Cmd_GetPkt）
void UART4_IRQHandler(void) // 注意：函数名必须与启动文件/向量表一致
{
	u16 RxByte; // 接收的单字节数据（用 u16 接住寄存器返回值，实际有效一般为低 8 位）
    /* RBNE: receive buffer not empty */
    if (usart_interrupt_flag_get(UART4, USART_INT_FLAG_RBNE) != RESET) { // 判断是否触发“接收缓冲非空”中断标志
        RxByte = usart_data_receive(UART4);                              // 读取接收数据寄存器（读取会清 RBNE 标志）
		Cmd_GetPkt(RxByte);                                                        // 将字节喂给 IM948 协议解析/组包状态机
    }

    /* ORERR: overrun error */
    if (usart_flag_get(UART4, USART_FLAG_ORERR) != RESET) {             // 判断是否发生溢出错误（未及时读取导致）
        usart_flag_clear(UART4, USART_FLAG_ORERR);                      // 清除溢出错误标志
        (void)usart_data_receive(UART4);                                // 再读一次数据寄存器以释放接收逻辑（丢弃该字节）
    }
} // UART4_IRQHandler end

