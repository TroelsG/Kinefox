#ifndef ACCMC3635_H_
#define ACCMC3635_H_

#include <stdint.h>
#include "i2c.h"
#include "Math.h"

#define ACC_MC3635_ADDRESS			0x4C
#define ACC_MC3635_REG_RANGE_C		0x15
#define ACC_MC3635_REG_WAKE_C		0x11
#define ACC_MC3635_REG_MODE_C		0x10
#define ACC_MC3635_REG_FIFO_C		0x16
#define ACC_MC3635_REG_INTR_C		0x17
#define ACC_MC3635_REG_STATUS_2		0x09
#define ACC_MC3635_REG_STATUS_1		0x08
#define ACC_MC3635_REG_XOUT_LSB		0x02

bool accMC3635FifoEmpty();
bool accMC3635Alive();
uint8_t accMC3635ReadStatus();
uint8_t accMC3635ReadStatus2();
void accMC3635Read(int16_t *x, int16_t *y, int16_t *z);
uint8_t accMC3635ReadAndClearInterrupt();
void accMC3635Start54HzWithFIFO();
void accMC3635Start28HzWithFIFO();

#endif