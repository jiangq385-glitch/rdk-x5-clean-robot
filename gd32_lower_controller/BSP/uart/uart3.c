#include "string.h"
#include "stdlib.h"
#include "uart3.h"

#include "gd32f4xx_usart.h"

/*
 * GD32F407 适配说明：
 * - 串口三使用 UART3：PC10(TX) / PC11(RX)
 * - GPIO 复用 AF7
 */
#define BSP_SERVO_UARTx          UART3
#define BSP_SERVO_UART_IRQn      UART3_IRQn

#define BSP_SERVO_TX_PORT        GPIOC
#define BSP_SERVO_TX_CLK         RCU_GPIOC
#define BSP_SERVO_TX_PIN         GPIO_PIN_10

#define BSP_SERVO_RX_PORT        GPIOC
#define BSP_SERVO_RX_CLK         RCU_GPIOC
#define BSP_SERVO_RX_PIN         GPIO_PIN_11

#define BSP_SERVO_UART_CLK       RCU_UART3
#define BSP_SERVO_UART_AF        GPIO_AF_7

#define UART3_RECV_BUF_SIZE      16U
#define DISTANCE_MIN             0
#define DISTANCE_MAX             9999
#define CONFIDENCE_MAX           100

static uint8_t recv_buf[UART3_RECV_BUF_SIZE];
static uint8_t recv_index = 0;
static uint8_t parsing_state = 0;
static uint8_t comma_pos = 0;

volatile uint16_t distance_value = 0;
volatile uint8_t confidence_value = 0;
volatile uint8_t data_ready = 0;

static void USART3_Send_U8(uint8_t ch)
{
	while (RESET == usart_flag_get(BSP_SERVO_UARTx, USART_FLAG_TC)) {
	}
	usart_data_transmit(BSP_SERVO_UARTx, ch);
}

static void USART3_Send_ArrayU8(uint8_t *buffer_ptr)
{
	while (*buffer_ptr) {
		USART3_Send_U8(*buffer_ptr++);
	}
}

static void Processing_Data(uint8_t rx_data)
{
	char dist_str[6] = {0};
	char conf_str[3] = {0};
	uint8_t dist_len;
	uint8_t conf_start;
	uint8_t conf_len;

	if (recv_index >= UART3_RECV_BUF_SIZE) {
		recv_index = 0;
		parsing_state = 0;
		comma_pos = 0;
		return;
	}

	recv_buf[recv_index++] = rx_data;

	switch (parsing_state) {
		case 0:
			if (rx_data == 0x20) {
				parsing_state = 1;
				recv_index = 1;
			} else {
				recv_index = 0;
			}
			break;

		case 1:
			if (rx_data == 0x2C) {
				parsing_state = 2;
				comma_pos = recv_index - 1;
			}
			break;

		case 2:
			if (rx_data == 0x20) {
				parsing_state = 3;
			} else {
				parsing_state = 0;
				recv_index = 0;
				comma_pos = 0;
			}
			break;

		case 3:
			if (rx_data == 0x0A) {
				dist_len = (comma_pos > 1) ? (comma_pos - 1) : 0;
				if (dist_len > 5) {
					dist_len = 5;
				}
				memcpy(dist_str, &recv_buf[1], dist_len);
				dist_str[dist_len] = '\0';

				conf_start = comma_pos + 2;
				conf_len = (recv_index > conf_start) ? (recv_index - conf_start - 1) : 0;
				if (conf_len > 2) {
					conf_len = 2;
				}
				memcpy(conf_str, &recv_buf[conf_start], conf_len);
				conf_str[conf_len] = '\0';

				distance_value = (uint16_t)atoi(dist_str);
				confidence_value = (uint8_t)atoi(conf_str);

				if (distance_value < DISTANCE_MIN ||
					distance_value > DISTANCE_MAX ||
					confidence_value > CONFIDENCE_MAX) {
					distance_value = 0;
					confidence_value = 0;
				}

				data_ready = 1;
				recv_index = 0;
				parsing_state = 0;
				comma_pos = 0;
			}
			break;

		default:
			parsing_state = 0;
			recv_index = 0;
			comma_pos = 0;
			break;
	}
}

void uart3NVICInit(void) {
	/* UART3 NVIC */
	nvic_irq_enable(BSP_SERVO_UART_IRQn, 3, 3);
}

void uart3Init(u32 bound)
{
	/* 时钟 */
	rcu_periph_clock_enable(BSP_SERVO_TX_CLK);
	rcu_periph_clock_enable(BSP_SERVO_RX_CLK);
	rcu_periph_clock_enable(BSP_SERVO_UART_CLK);

	/* GPIO 复用：PC10(TX), PC11(RX) */
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

	uart3NVICInit();
	usart_interrupt_enable(BSP_SERVO_UARTx, USART_INT_RBNE);
	usart_enable(BSP_SERVO_UARTx);
}

void uart3WriteBuf(uint8_t *buf, uint8_t len)
{
	while (len--) {
		USART3_Send_U8(*buf++);
	}
}

void UART3_IRQHandler(void)
{
	uint8_t rx_byte;

	if (RESET != usart_interrupt_flag_get(BSP_SERVO_UARTx, USART_INT_FLAG_RBNE)) {
		rx_byte = (uint8_t)usart_data_receive(BSP_SERVO_UARTx);
		Processing_Data(rx_byte);
	}

	if (RESET != usart_flag_get(BSP_SERVO_UARTx, USART_FLAG_ORERR)) {
		usart_flag_clear(BSP_SERVO_UARTx, USART_FLAG_ORERR);
		(void)usart_data_receive(BSP_SERVO_UARTx);
	}
}
