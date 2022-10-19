#ifndef HALL_MMC5603NJ_H_
#define HALL_MMC5603NJ_H_

#include <stdint.h>
#include "i2c.h"
#include "Math.h"

#define HALL_MMC_ADDRESS			0x30

#define HALL_MMC_REG_XOUT0			0x00
#define HALL_MMC_REG_XOUT1			0x01
#define HALL_MMC_REG_XOUT2			0x06

#define HALL_MMC_REG_YOUT0			0x02
#define HALL_MMC_REG_YOUT1			0x03
#define HALL_MMC_REG_YOUT2			0x07

#define HALL_MMC_REG_ZOUT0			0x04
#define HALL_MMC_REG_ZOUT1			0x05
#define HALL_MMC_REG_ZOUT2			0x08

#define HALL_MMC_REG_TOUT			0x09
#define HALL_MMC_REG_STATUS1		0x18
#define HALL_MMC_REG_ODR			0x1A
#define HALL_MMC_REG_CONTROL0		0x1B
#define HALL_MMC_REG_CONTROL1		0x1C
#define HALL_MMC_REG_CONTROL2		0x1D

bool hallMMCAlive();
uint32_t hallMMCReadY();

#endif