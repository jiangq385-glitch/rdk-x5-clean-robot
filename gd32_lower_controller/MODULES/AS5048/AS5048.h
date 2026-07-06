#ifndef __AS5048_H
#define __AS5048_H


#include "main.h"

#define AS5048_NUMBER 2  



#define SPI_CMD_READ				0x4000 // SPI 读操作标志位（与寄存器地址按位或组合）
#define SPI_CMD_WRITE				0x8000 // SPI 写操作标志位（与寄存器地址按位或组合）
#define SPI_NOP							0x0000 // NOP/空操作（用于 dummy clock/占位收发）
#define SPI_REG_AGC					0x7ffd // AGC 自动增益寄存器地址
#define SPI_REG_MAG					0x7ffe // MAG 磁场幅值寄存器地址
#define SPI_REG_DATA				0xffff // DATA 角度数据寄存器地址（14bit 有效）
#define SPI_REG_CLRERR			0x4001 // CLRERR 清错误寄存器地址
#define SPI_REG_ZEROPOS_HI	0x0016 // ZEROPOS 零位寄存器高字节地址
#define SPI_REG_ZEROPOS_LO	0x0017 // ZEROPOS 零位寄存器低字节地址

// 常用命令/寄存器（与上面的 SPI_REG_* / SPI_CMD_* 含义一致，历史遗留命名）
#define CMD_ANGLE            0xffff
#define CMD_AGC              0x7ffd
#define CMD_MAG              0x7ffe
#define CMD_CLAER            0x4001
#define CMD_NOP              0xc000


void AS5048_init(int AS5048_ID, uint32_t spi, uint32_t gpiox, uint32_t gpio_pin);
uint16_t AS5048_Read(const int AS5048_ID, uint16_t registerAddress);
void AS5048_getREGValue(const int AS5048_ID);
void AS5048_dataUpdate(const int AS5048_ID);
/**
 * @brief AS5048_STRUCT
 */
typedef struct {
  
	
	int AS5048_ID;
	uint32_t spi_number;        ///< SPI0/SPI1/SPI2...
	uint32_t gpio_periph;    ///< GPIOA/GPIOB...
	uint32_t gpio_pin;       ///< GPIO_PIN_x
  int    angle;
	int    last_angle; //  cc_direction
	int    total_angle;
	int    cirle;
	int    delta_dis;

} AS5048;

/**
 * @brief AS5048_OBJECT_
 */

extern AS5048 AS5048s[AS5048_NUMBER];

#endif
