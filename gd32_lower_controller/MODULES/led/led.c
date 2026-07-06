#include "led.h" 

//LED IO初始化
void LED_Init(void)
{    	 
	/* 使能 GPIOD 时钟 */
	rcu_periph_clock_enable(RCU_GPIOD);

	/* 配置 PD8/PD9 为推挽输出，上拉 */
	gpio_mode_set(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_10 | GPIO_PIN_11|GPIO_PIN_12);
	gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10 | GPIO_PIN_11|GPIO_PIN_12);

	/* 默认输出高电平：LED灭（低电平点亮） */
	gpio_bit_set(GPIOD, GPIO_PIN_10 | GPIO_PIN_11|GPIO_PIN_12);
}

void led0_on(void)  { gpio_bit_reset(GPIOD, GPIO_PIN_10); }	//LED0亮
void led0_off(void) { gpio_bit_set(GPIOD, GPIO_PIN_10); }		//LED0灭
void led1_on(void)  { gpio_bit_reset(GPIOD, GPIO_PIN_11); }	//LED1亮
void led1_off(void) { gpio_bit_set(GPIOD, GPIO_PIN_11); }		//LED1灭
void led2_on(void)  { gpio_bit_reset(GPIOD, GPIO_PIN_12); }	//LED2亮
void led2_off(void) { gpio_bit_set(GPIOD, GPIO_PIN_12); }		//LED2灭


void led0_turn(void)
{
	if (gpio_output_bit_get(GPIOD, GPIO_PIN_10) == SET) {
		gpio_bit_reset(GPIOD, GPIO_PIN_10);
	} else {
		gpio_bit_set(GPIOD, GPIO_PIN_10);
	}
}

void led1_turn(void)
{
	if (gpio_output_bit_get(GPIOD, GPIO_PIN_11) == SET) {
		gpio_bit_reset(GPIOD, GPIO_PIN_11);
	} else {
		gpio_bit_set(GPIOD, GPIO_PIN_11);
	}
}
void led2_turn(void)
{
	if (gpio_output_bit_get(GPIOD, GPIO_PIN_12) == SET) {
		gpio_bit_reset(GPIOD, GPIO_PIN_12);
	} else {
		gpio_bit_set(GPIOD, GPIO_PIN_12);
	}
}






