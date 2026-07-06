
#include "usart.h"
#include "servo.h"
#include "uart3.h"
 




////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//串口1初始化代码（支持printf打印）	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/6/10
//版本：V1.5
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved
//********************************************************************************
//V1.3修改说明 
//支持适应不同频率下的串口波特率设置.
//加入了支持printf函数的代码
//修改了串口接收数据的逻辑.
//修正了printf函数在特定情况下出现死机的bug
//V1.4修改说明
//1,修改了初始化IO的bug
//2,修改了USART_RX_STA,使得串口最大接收字节数为2的14次方
//3,增加了USART_REC_LEN,用于定义串口最大接收字节数
//4,修改了EN_USART3_RX的使能方式
//V1.5修改说明
//1,增加了对UCOSII的支持
////////////////////////////////////////////////////////////////////////////////// 	  
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	/* 调试串口：PD8/PD9 -> UART2 (AF7) */
	while(RESET == usart_flag_get(USART2, USART_FLAG_TBE)){
	}
	usart_data_transmit(USART2, (u8)ch);
	return ch;
}
#endif
 

//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15:	接收完成标志
//bit14:	接收到0x0d
//bit13~0:	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	
static uint16_t uart2_last_pos = 0;

/* 最新接收并解析到的 vx/vy/vz（供其它模块读取） */
// volatile float g_rx_vxyz[3] = {0.0f, 0.0f, 0.0f};
// volatile u32 g_rx_vxyz_seq = 0;
//电机速度解算
extern vel_cmd_t vel_cmd;

//bound:波特率
void uart_init(u32 bound)
{
	/*
     * GD32F407VET6 适配：PD8(TX)/PD9(RX)
     * - 对应 USART2
     * - 复用功能：AF7
	 */
	dma_single_data_parameter_struct dma_init;
    rcu_periph_clock_enable(RCU_GPIOD);
    rcu_periph_clock_enable(RCU_USART2);

    /* GPIO: PD8/ PD9 */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_af_set(GPIOD, GPIO_AF_7, GPIO_PIN_8 | GPIO_PIN_9);

    usart_deinit(USART2);
    usart_baudrate_set(USART2, bound);
    usart_word_length_set(USART2, USART_WL_8BIT);
    usart_stop_bit_set(USART2, USART_STB_1BIT);
    usart_parity_config(USART2, USART_PM_NONE);
    usart_transmit_config(USART2, USART_TRANSMIT_ENABLE);
    usart_receive_config(USART2, USART_RECEIVE_ENABLE);
    usart_enable(USART2);
	
	/*USART_ClearFlag(USART1, USART_FLAG_TC);*/
	
    

    /* 1) DMA 时钟：USART2_RX 更常见挂在 DMA0（类似 STM32F4 的 DMA1） */
    rcu_periph_clock_enable(RCU_DMA0);

    /* 2) 关闭 USART2 的字节接收中断（DMA 接收时别再在中断里读 DATA） */
    usart_interrupt_disable(USART2, USART_INT_RBNE);

    /* 3) DMA0 Channel1: 外设->内存，循环模式 */
    dma_deinit(DMA0, DMA_CH1);
    dma_single_data_para_struct_init(&dma_init);

        dma_init.periph_addr         = (uint32_t)&USART_DATA(USART2);
    dma_init.memory0_addr        = (uint32_t)USART_RX_BUF;
    dma_init.direction           = DMA_PERIPH_TO_MEMORY;
    dma_init.periph_memory_width = DMA_PERIPH_WIDTH_8BIT;
    dma_init.periph_inc          = DMA_PERIPH_INCREASE_DISABLE;
    dma_init.memory_inc          = DMA_MEMORY_INCREASE_ENABLE;
    dma_init.circular_mode       = DMA_CIRCULAR_MODE_ENABLE;
    dma_init.number              = USART_REC_LEN;
    dma_init.priority            = DMA_PRIORITY_HIGH;

        dma_single_data_mode_init(DMA0, DMA_CH1, &dma_init);

    /* 4) 选择子外设（DMA request 源选择，必须配） */
    dma_channel_subperipheral_select(DMA0, DMA_CH1, DMA_SUBPERI4);

    /* 5) 使能 DMA 通道 */
    dma_channel_enable(DMA0, DMA_CH1);

    /* 6) 使能 UART3 DMA 接收请求 */
    usart_dma_receive_config(USART2, USART_DENR_ENABLE);

	
    usart_interrupt_enable(USART2, USART_INT_IDLE);
    nvic_irq_enable(USART2_IRQn, 6, 0);
	
	
}

static host_frame_t frame;

static bool lidar_status_is_range_request(const host_frame_t *f)
{
    return (fabsf(f->x - 1.0f) < 0.001f) &&
           (fabsf(f->y) < 0.001f) &&
           (fabsf(f->z) < 0.001f);
}

static void uart2_send_lidar_range(void)
{
    uint8_t lidar_frame[FRAME_SIZE];
    float range_m = (float)distance_value / 1000.0f;

    set_message(TYPE_LIDAR_RANGE, range_m, 0.0f, 0.0f, 0, lidar_frame);
    UART_SendBytes(lidar_frame, FRAME_SIZE);
}

static void uart2_dispatch_frame(host_frame_t *f)
{
    host_frame_apply_sign(f);

    switch (f->type_id) {
    case TYPE_CMD_VEL:
        vel_cmd.seq1++;
        vel_cmd.vx = f->x;
        vel_cmd.vy = f->y;
        vel_cmd.vz = f->z;
        vel_cmd.seq2++;
        break;

    case TYPE_ARM0_JOINT_123:
        arm_cmd_write_joints(0, 0, f->x, f->y, f->z, 800u);
        break;

    case TYPE_ARM0_JOINT_456:
        arm_cmd_write_joints(0, 3, f->x, f->y, f->z, 800u);
        break;

    case TYPE_ARM1_JOINT_123:
        arm_cmd_write_joints(1, 0, f->x, f->y, f->z, 800u);
        break;

    case TYPE_ARM1_JOINT_456:
        arm_cmd_write_joints(1, 3, f->x, f->y, f->z, 800u);
        break;

    case TYPE_ARM0_HOME:
        arm_cmd_write_home(0, 1000u);
        break;

    case TYPE_ARM1_HOME:
        arm_cmd_write_home(1, 1000u);
        break;

    case TYPE_LIDAR_STATUS:
        if (lidar_status_is_range_request(f)) {
            uart2_send_lidar_range();
        }
        break;

    default:
        break;
    }
}
static void uart2_on_bytes(const uint8_t *data, uint16_t len)
{
   
	for (uint16_t i = 0; i < len; i++) {
		if (feed_host_frame(data[i], &frame))
		{
			uart2_dispatch_frame(&frame);
		}
	}
}


static void uart2_rx_drain_from_dma(void)
{
    // DMA 当前写入位置 = BUF_SIZE - NDTR
    uint16_t pos = (uint16_t)(USART_REC_LEN - dma_transfer_number_get(DMA0, DMA_CH1));

    if (pos == uart2_last_pos) return;

    if (pos > uart2_last_pos)
    {
        // 新数据在 [last_pos, pos)
        uart2_on_bytes(&USART_RX_BUF[uart2_last_pos], (uint16_t)(pos - uart2_last_pos));
    }
    else
    {
        // DMA 回卷了：先尾巴 [last_pos, end)，再头部 [0, pos)
        uart2_on_bytes(&USART_RX_BUF[uart2_last_pos], (uint16_t)(USART_REC_LEN - uart2_last_pos));
        if (pos > 0)
            uart2_on_bytes(&USART_RX_BUF[0], pos);
    }

    uart2_last_pos = pos;
}

void USART2_IRQHandler(void)              	// USART2 中断服务程序（用 IDLE 触发“取走DMA新增字节”）
{
    if (RESET != usart_interrupt_flag_get(USART2, USART_INT_FLAG_IDLE))
    {
        // 清 IDLE：按手册要求读 STAT0 再读 DATA
        volatile uint32_t tmp;
        tmp = USART_STAT0(USART2);
        tmp = USART_DATA(USART2);
        (void)tmp;

        // 把 DMA 环形缓冲里新增字节取出来处理
        uart2_rx_drain_from_dma();
		led0_turn(); // 调试用：每收到一批新字节就翻转 LED0
	

	

    }

    // 可选：ORE 清除（保险）
        if (RESET != usart_flag_get(USART2, USART_FLAG_ORERR))
    {
		usart_flag_clear(USART2, USART_FLAG_ORERR);
		(void)USART_DATA(USART2);
    }
}



//发送函数

void UART_SendBytes(uint8_t *buf, uint16_t len)
{
    while (len--) {
        while (RESET == usart_flag_get(USART2, USART_FLAG_TBE)) {
        }
        usart_data_transmit(USART2, *buf++);
    }
    while (RESET == usart_flag_get(USART2, USART_FLAG_TC)) {
    }
}

/*void example_send_once(void)
{
    uint8_t frame[12];
    set_message(1.234f, -0.500f, 0.120f, 0x00, frame);

     UART_SendBytes(frame, 12);
}
*/
