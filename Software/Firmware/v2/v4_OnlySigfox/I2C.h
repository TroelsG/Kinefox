#ifndef I2C_H_
#define I2C_H_

// AVR libs
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <stdio.h>

#define I2C_SETTING							200000
#define I2C_BAUD(F_SCL)						((((float)F_CPU / (float)F_SCL)) - 10) // IMPORTANT: needs to by >= 0 to work -> F_CPU = 1MHz -> max. 100kHz, F_CPU = 2MHz -> max. 200kHz, F_CPU = 5MHz -> max. 500kHz
#define TIMEOUT_US_WAIT						10
#define TIMEOUT_NUM_WAIT					5000 // ~50ms, 2 bytes

bool pollRIF();
bool pollWIF();

void i2cInit();
void i2cInit(uint8_t customBaud);
bool i2cStartRead(uint8_t deviceAddr);
bool i2cStartWrite(uint8_t deviceAddr);
uint8_t i2cRead(bool ack);
bool i2cWrite(uint8_t write_data);
void i2cStop();

bool i2cAlive(uint8_t i2cAddress);
uint8_t i2cReadRegister(uint8_t address, uint8_t reg, bool *error);
bool i2cWriteRegister(uint8_t address, uint8_t reg, uint8_t val);

#endif /* I2C_H_ */