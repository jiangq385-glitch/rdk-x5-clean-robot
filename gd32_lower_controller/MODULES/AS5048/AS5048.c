  #include "AS5048.h"
#include "spi.h"

AS5048 AS5048s[AS5048_NUMBER];
static uint16_t as5048_spi_txrx16(uint32_t spi_periph, uint16_t tx)
{
	while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_TBE)) {
	}
	spi_i2s_data_transmit(spi_periph, tx);
	while (RESET == spi_i2s_flag_get(spi_periph, SPI_FLAG_RBNE)) {
	}
	return spi_i2s_data_receive(spi_periph);
}

void AS5048_init(int AS5048_ID, uint32_t spi, uint32_t gpiox, uint32_t gpio_pin)
{
   AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	
	AS5->AS5048_ID = AS5048_ID;
	AS5->spi_number = spi;
	AS5->gpio_periph = gpiox;
	AS5->gpio_pin = gpio_pin;

	/* 片选脚初始化为普通推挽输出（空闲拉高） */
	if (gpiox == GPIOA) {
		rcu_periph_clock_enable(RCU_GPIOA);
	} else if (gpiox == GPIOB) {
		rcu_periph_clock_enable(RCU_GPIOB);
	} else if (gpiox == GPIOC) {
		rcu_periph_clock_enable(RCU_GPIOC);
	} else if (gpiox == GPIOD) {
		rcu_periph_clock_enable(RCU_GPIOD);
	} else if (gpiox == GPIOE) {
		rcu_periph_clock_enable(RCU_GPIOE);
	}

	gpio_mode_set(gpiox, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpio_pin);
	gpio_output_options_set(gpiox, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, gpio_pin);
	gpio_bit_set(gpiox, gpio_pin);

	/* angle: 0~16383（单圈角度，counts） */
	AS5->angle = 0;
	/* total_angle: 跨圈累计角度（counts） */
	AS5->total_angle = 0;
	/* cirle: 圈数计数（注意：这里变量名为 cirle，沿用原代码） */
	AS5->cirle = 0;
	AS5->last_angle = AS5->angle;
	/* delta_dis: 本次相对上次的增量（counts） */
	AS5->delta_dis = 0;
}

uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress)
{
    uint8_t data[4] = {0, 0, 0,0};
	uint8_t cmd[4] = {0, 0,0,0};
	
	cmd[3] = registerAddress & 0xFF;
	cmd[2] = ( registerAddress >> 8 ) & 0xFF;
	cmd[1] = registerAddress & 0xFF;
	cmd[0] = ( registerAddress >> 8 ) & 0xFF;

	AS5048 *AS5 = AS5048s + AS5048_ID -1;

	/* CS 拉低开始一次 SPI 事务 */
	gpio_bit_reset(AS5->gpio_periph, AS5->gpio_pin);

	/* 你的 SPI 初始化目前是 16bit 帧，因此这里用 2 次 16bit 收发等价替代 4 字节 */
	uint16_t tx0 = (uint16_t)(((uint16_t)cmd[0] << 8) | cmd[1]);
	uint16_t tx1 = (uint16_t)(((uint16_t)cmd[2] << 8) | cmd[3]);
	uint16_t rx0 = as5048_spi_txrx16(AS5->spi_number, tx0);
	uint16_t rx1 = as5048_spi_txrx16(AS5->spi_number, tx1);

	/* 按小端方式放入 byte buffer：data[0]=低字节，data[1]=高字节 */
	data[0] = (uint8_t)(rx0 & 0xFF);
	data[1] = (uint8_t)((rx0 >> 8) & 0xFF);
	data[2] = (uint8_t)(rx1 & 0xFF);
	data[3] = (uint8_t)((rx1 >> 8) & 0xFF);

	/* CS 拉高结束一次 SPI 事务 */
	gpio_bit_set(AS5->gpio_periph, AS5->gpio_pin);

	return ((( data[1] & 0xFF) << 8) | (data[0] & 0xFF)) & ~0xC000;

}



void AS5048_getREGValue(const int AS5048_ID)
{
    /*
	 * 读取角度寄存器并更新 AS5->angle。
	 * 注意：此函数只“读值”，不做回绕/累计/增量计算。
	 */
    AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	AS5->angle =  AS5048_Read(AS5048_ID,SPI_REG_DATA);
}



void AS5048_dataUpdate(const int AS5048_ID)
{
    /*
	 * 根据 angle 和 last_angle 计算：
	 * - delta_dis：本周期增量（counts，可正可负）
	 * - total_angle：累计角度（counts，可跨多圈）
	 * - cirle：圈数（跨过 0 点时更新）
	 *
	 * 回绕判定：
	 * - 反向跨越 0 点：diff 会出现一个很大的正数（接近 +16384），这里用 >16000 判定
	 * - 正向跨越 0 点：diff 会出现一个很大的负数（接近 -16384），这里用 <-16000 判定
	 *
	 * 常量 16384 = 2^14 = 单圈 counts。
	 */
	AS5048 *AS5 = AS5048s + AS5048_ID -1;
	
	/* diff：当前角度相对上次角度的差值（未做回绕修正） */
	int diff = AS5->angle - AS5->last_angle;
	if (diff > 13108) {
		/* 发生回绕：last_angle 接近 0，angle 接近 16383（反向跨过 0 点） */
		AS5->cirle--;
		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
		AS5->delta_dis = diff - 16384;

	} else if (diff < -13108) {
		/* 发生回绕：last_angle 接近 16383，angle 接近 0（正向跨过 0 点） */
		AS5->cirle++;
		AS5->total_angle = AS5->angle + AS5->cirle * 16384;
		AS5->delta_dis = diff + 16384;
		
	}else if (diff >=0) {
		/* 未回绕，正向转动 */
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	
	} else if (diff < 0) {
		/* 未回绕，反向转动 */
		AS5->total_angle += diff;
		AS5->delta_dis = diff;
	}	
	/* 保存本次角度，供下次计算增量 */
	AS5->last_angle = AS5->angle;

}
