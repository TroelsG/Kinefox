#ifndef ACCMC3419_H_
#define ACCMC3419_H_

#include <stdint.h>
#include "i2c.h"
#include "Math.h"

#define ACC_MC3419_ADDRESS			0x4C
#define REG_ACC_MODE				0x07
#define REG_ACC_RANGE				0x20
#define REG_ACC_SAMPLE_RATE			0x08
#define REG_ACC_FIFO_CTRL			0x2D
#define REG_ACC_FIFO_CTRL2_SR2		0x30
#define REG_ACC_FIFO_TH				0x2E
#define REG_ACC_FIFO_WR_P			0x0C
#define REG_ACC_FIFO_STAT			0x0A
#define REG_ACC_XOUT_LSB			0x0D

bool accIsAlive();
bool accStart25HzWithFIFO();
uint8_t accGetFifoLen();
bool accFifoEmpty();
bool accRead(int16_t *x, int16_t *y, int16_t *z);
bool accReadFloat(float *x, float *y, float *z);

uint8_t accGetPitch(int16_t x, int16_t y, int16_t z);
uint8_t accGetRoll(int16_t x, int16_t y, int16_t z);
uint32_t accMagnitude8G(int16_t x, int16_t y, int16_t z); // max. value = (56754 - 4096)

#endif