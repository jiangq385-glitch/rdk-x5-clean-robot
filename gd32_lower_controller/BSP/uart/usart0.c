#include "string.h"
#include "uart0.h"
#include "bool.h"
#include "LobotServoController.h"


#include "gd32f4xx_usart.h"

/*
 * GD32F407 适配说明：
 * - 舵机改用 USART1：PD5(TX) / PD6(RX)
 * - GPIO 复用 AF7
 */
#define BSP_SERVO_UARTx          USART0
#define BSP_SERVO_UART_IRQn      USART0_IRQn

#define BSP_SERVO_TX_PORT        GPIOA
#define BSP_SERVO_TX_CLK         RCU_GPIOA
#define BSP_SERVO_TX_PIN         GPIO_PIN_9

#define BSP_SERVO_RX_PORT        GPIOA
#define BSP_SERVO_RX_CLK         RCU_GPIOA
#define BSP_SERVO_RX_PIN         GPIO_PIN_10

#define BSP_SERVO_UART_CLK       RCU_USART0
#define BSP_SERVO_UART_AF        GPIO_AF_7
u8 UART_RX_BUF[16];
volatile bool isUartRxCompleted = false;

void uart0NVICInit(void) {
	/* UART0 NVIC */
	nvic_irq_enable(BSP_SERVO_UART_IRQn, 3, 3);
}

void uart0Init(u32 bound)
{
	/* 时钟 */
	rcu_periph_clock_enable(BSP_SERVO_TX_CLK);
	rcu_periph_clock_enable(BSP_SERVO_RX_CLK);
	rcu_periph_clock_enable(BSP_SERVO_UART_CLK);

	/* GPIO 复用：PD5(TX), PD6(RX) */
	gpio_mode_set(BSP_SERVO_TX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_SERVO_TX_PIN);
	gpio_output_options_set(BSP_SERVO_TX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SERVO_TX_PIN);
	gpio_af_set(BSP_SERVO_TX_PORT, BSP_SERVO_UART_AF, BSP_SERVO_TX_PIN);

	gpio_mode_set(BSP_SERVO_RX_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BSP_SERVO_RX_PIN);
	gpio_output_options_set(BSP_SERVO_RX_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, BSP_SERVO_RX_PIN);
	gpio_af_set(BSP_SERVO_RX_PORT, BSP_SERVO_UART_AF, BSP_SERVO_RX_PIN);

	/* UART 参数 */
	usart_deinit(BSP_SERVO_UARTx);
	usart_baudrate_set(BSP_SERVO_UARTx, bound);
	usart_word_length_set(BSP_SERVO_UARTx, USART_WL_8BIT);
	usart_stop_bit_set(BSP_SERVO_UARTx, USART_STB_1BIT);
	usart_parity_config(BSP_SERVO_UARTx, USART_PM_NONE);
	usart_transmit_config(BSP_SERVO_UARTx, USART_TRANSMIT_ENABLE);
	usart_receive_config(BSP_SERVO_UARTx, USART_RECEIVE_ENABLE);

	uart1NVICInit();
	usart_interrupt_enable(BSP_SERVO_UARTx, USART_INT_RBNE);
	usart_enable(BSP_SERVO_UARTx);
}

void uart0WriteBuf(uint8_t *buf, uint8_t len)
{
	while (len--) {
		while (RESET == usart_flag_get(BSP_SERVO_UARTx, USART_FLAG_TC)) {
		}
		usart_data_transmit(BSP_SERVO_UARTx, *buf++);
	}
}

extern uint8_t LobotRxBuf[16];

void USART0_IRQHandler(void)
{
	uint8_t Res;
	static bool isGotFrameHeader = false;
	static uint8_t frameHeaderCount = 0;
	static uint8_t dataLength = 2;
	static uint8_t dataCount = 0;
	if (RESET != usart_interrupt_flag_get(BSP_SERVO_UARTx, USART_INT_FLAG_RBNE)) { //判断接收中断
		Res = (uint8_t)usart_data_receive(BSP_SERVO_UARTx);
		if (!isGotFrameHeader) {  //判断帧头
			if (Res == FRAME_HEADER) {
				frameHeaderCount++;
				if (frameHeaderCount == 2) {
					frameHeaderCount = 0;
					isGotFrameHeader = true;
					dataCount = 1;
				}
			} else {
				isGotFrameHeader = false;
				dataCount = 0;
				frameHeaderCount = 0;
			}
		}
		if (isGotFrameHeader) { //接收接收数据部分
			UART_RX_BUF[dataCount] = Res;
			if (dataCount == 2) {
				dataLength = UART_RX_BUF[dataCount];
				if (dataLength < 2 || dataLength > 8) {
					dataLength = 2;
					isGotFrameHeader = false;
				}
			}
			dataCount++;
			if (dataCount == dataLength + 2) {
				if (isUartRxCompleted == false) {
					isUartRxCompleted = true;
					memcpy(LobotRxBuf, UART_RX_BUF, dataCount);
				}
				isGotFrameHeader = false;
			}
		}
	}
	/* 清溢出 */
	if (RESET != usart_flag_get(BSP_SERVO_UARTx, USART_FLAG_ORERR)) {
		usart_flag_clear(BSP_SERVO_UARTx, USART_FLAG_ORERR);
		(void)usart_data_receive(BSP_SERVO_UARTx);
	}
}
