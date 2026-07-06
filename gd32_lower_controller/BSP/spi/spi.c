#include "spi.h"



/*
 * SPI2 引脚：PB3=SCK, PB4=MISO, PB5=MOSI
 * 复用功能：GPIO_AF_6（若你板子实际走 AF5，请把 GPIO_AF_6 改为 GPIO_AF_5）
 */
void SPI2_Init(void)
{
	spi_parameter_struct spi_init_struct;
	uint32_t spi_pins = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;

	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_SPI2);

	/* GPIO: PB3/4/5 -> AF */
	gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_pins);
	gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_pins);
	gpio_af_set(GPIOB, GPIO_AF_6, spi_pins);

	/* SPI2 参数 */
	spi_i2s_deinit(SPI2);
	spi_struct_para_init(&spi_init_struct);
	spi_init_struct.device_mode = SPI_MASTER;
	spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
	spi_init_struct.nss = SPI_NSS_SOFT;
	spi_init_struct.endian = SPI_ENDIAN_MSB;
	spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
	spi_init_struct.prescale = SPI_PSC_8;

	spi_init(SPI2, &spi_init_struct);

	/* CRC 默认关闭；多项式保持与历史代码一致 */
	spi_crc_polynomial_set(SPI2, 10);
	spi_crc_off(SPI2);

	/* 软件 NSS 建议拉高，避免 MODF */
	spi_nss_internal_high(SPI2);
	spi_enable(SPI2);
}


void SPI0_Init(void)
{
    spi_parameter_struct spi_init_struct;
    uint32_t spi_pins = GPIO_PIN_5 | GPIO_PIN_7;

    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_SPI0);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_pins);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_pins);
    gpio_af_set(GPIOA, GPIO_AF_5, spi_pins);

    spi_i2s_deinit(SPI0);
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size = SPI_FRAMESIZE_8BIT;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE;
    spi_init_struct.prescale = SPI_PSC_8;

    spi_init(SPI0, &spi_init_struct);
    spi_crc_polynomial_set(SPI0, 10);
    spi_crc_off(SPI0);
    spi_nss_internal_high(SPI0);
    spi_enable(SPI0);
}

uint8_t SPI0_WriteByte(uint8_t data)
{
    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE)) {
    }
    spi_i2s_data_transmit(SPI0, data);

    while (RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE)) {
    }
    return (uint8_t)spi_i2s_data_receive(SPI0);
}

/*
 * SPI1 引脚：PB13=SCK, PB14=MISO, PB15=MOSI
 * 复用功能：GPIO_AF_5
 */
void SPI1_Init(void)
{
    spi_parameter_struct spi_init_struct;
    uint32_t spi_pins = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;

    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_SPI1);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, spi_pins);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, spi_pins);
    gpio_af_set(GPIOB, GPIO_AF_5, spi_pins);

    spi_i2s_deinit(SPI1);
    spi_struct_para_init(&spi_init_struct);
    spi_init_struct.device_mode = SPI_MASTER;
    spi_init_struct.trans_mode = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.frame_size = SPI_FRAMESIZE_16BIT;
    spi_init_struct.nss = SPI_NSS_SOFT;
    spi_init_struct.endian = SPI_ENDIAN_MSB;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_LOW_PH_2EDGE;
    spi_init_struct.prescale = SPI_PSC_8;

    spi_init(SPI1, &spi_init_struct);
    spi_crc_polynomial_set(SPI1, 10);
    spi_crc_off(SPI1);
    spi_nss_internal_high(SPI1);
    spi_enable(SPI1);
}

/*uint16_t SPI_TxRx16_Spl(SPI_TypeDef* SPIx, uint16_t tx, uint32_t timeoutCycles)
{
while (SPI_I2S_GetFlagStatus(SPIx, SPI_I2S_FLAG_TXE) == RESET) {
if (timeoutCycles && --timeoutCycles == 0) return 0;
}
SPI_I2S_SendData(SPIx, tx);

}*/





/*// SPI3 全局中断服务函数（默认不启用任何 SPI 中断源时不会进入）
void SPI3_IRQHandler(void)
{
    // 保险：如果未来启用了错误中断，清一下 OVR，避免卡死
    if (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_OVR) != RESET)
    {
        (void)SPI3->DR;
        (void)SPI3->SR;
    }
}

// SPI2速度设置函数
// 参数：
//   SPI_BaudRatePrescaler: 波特率预分频值
// 可选值：
//   SPI_BaudRatePrescaler_2   2分频
//   SPI_BaudRatePrescaler_8   8分频
//   SPI_BaudRatePrescaler_16  16分频
//   SPI_BaudRatePrescaler_256 256分频
void SPI3_SetSpeed(u8 SPI_BaudRatePrescaler)
{
    assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
    SPI3->CR1 &= 0XFFC7; // 清除原有的预分频设置
    SPI3->CR1 |= SPI_BaudRatePrescaler; // 设置新的预分频值
    SPI_Cmd(SPI3, ENABLE); // 使能SPI3
}

// SPI2读写一个字节
// 参数：
//   TxData: 要发送的字节
// 返回值：
//   读取到的字节
u8 SPI3_ReadWriteByte(u8 TxData)
{
    u8 retry = 0;

    // 等待发送缓冲区为空
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_TXE) == RESET)
    {
        retry++;
        if (retry > 200) return 0; // 超时退出
    }

    // 发送数据
    SPI_I2S_SendData(SPI3, TxData);
    retry = 0;

    // 等待接收缓冲区非空
    while (SPI_I2S_GetFlagStatus(SPI3, SPI_I2S_FLAG_RXNE) == RESET)
    {
        retry++;
        if (retry > 200) return 0; // 超时退出
    }

    // 返回接收到的数据
    return SPI_I2S_ReceiveData(SPI3);
}


*/
