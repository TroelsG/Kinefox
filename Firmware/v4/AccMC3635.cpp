#include "AccMC3635.h"

bool accMC3635FifoEmpty() {
	uint8_t reg;
	bool error = false;
	reg = i2cReadRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_STATUS_1, &error);
	reg &= 0x10;
	if(reg ^ 0x10) { return false; } // not empty
	return true;// empty
}

uint8_t accMC3635ReadStatus() {
	uint8_t reg;
	bool error = false;
	reg = i2cReadRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_STATUS_1, &error);
	return reg;
}

uint8_t accMC3635ReadStatus2() {
	uint8_t reg;
	bool error = false;
	reg = i2cReadRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_STATUS_2, &error);
	return reg;
}

void accMC3635Read(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t i = 0;
	uint8_t data[6] = { 0 };
	i2cStartWrite(ACC_MC3635_ADDRESS);
	i2cWrite(ACC_MC3635_REG_XOUT_LSB);
	i2cStartRead(ACC_MC3635_ADDRESS);
	while(i < (6 - 1)) {
		data[i] = i2cRead(true); // with ack
		i++;
	}
	data[i] = i2cRead(false); // last one: no ack
	i2cStop();
	
	*x = (int16_t)((((uint16_t)data[1]) << 8) | data[0]);
	*y = (int16_t)((((uint16_t)data[3]) << 8) | data[2]);
	*z = (int16_t)((((uint16_t)data[5]) << 8) | data[4]);
}

uint8_t accMC3635ReadAndClearInterrupt() {
	bool error = false;
	uint8_t reg = i2cReadRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_STATUS_2, &error);
	return reg;
}

bool accMC3635Alive() {
	if(!i2cStartRead(ACC_MC3635_ADDRESS)) {
		i2cStop();
		return false;
	}
	i2cStop();
	return true;
}

void accMC3635Start54HzWithFIFO() {
	// reset procedure
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_MODE_C, 0x01); // go to standby
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x24, 0x40); // reset
	_delay_ms(2); // wait for reset to complete
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x0D, 0x40); // i2c mode enabled
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x0F, 0x42); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x20, 0x01); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x21, 0x80); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x28, 0x00); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x1A, 0x00); // initialization
	
	// set configuration
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_RANGE_C, 0b00100100); // initialization: 8G range, 12 bits resolution
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_WAKE_C, 0x07); // low power mode then: 0x07 = 54 Hz, 2.7 uA
	// register 0x1C: default value = low power mode (nominal noise levels) -> ultra low power or normal mode also exist

	// set fifo
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_FIFO_C, 0b01011111); // fifo enabled, normal operation, 31 sample threshold
	//i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_INTR_C, 0b01000011); // fifo threshold interrupt enabled, active int drive (push-pull), active high
	//i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_INTR_C, 0b01000000); // fifo threshold interrupt enabled, open drain interrupt (pull-up needed), active low

	// go into cwake state
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_MODE_C, 0b101);
	_delay_ms(1);
}

void accMC3635Start28HzWithFIFO() {
	// reset procedure
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_MODE_C, 0x01); // go to standby
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x24, 0x40); // reset
	_delay_ms(2); // wait for reset to complete
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x0D, 0x40); // i2c mode enabled
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x0F, 0x42); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x20, 0x01); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x21, 0x80); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x28, 0x00); // initialization
	i2cWriteRegister(ACC_MC3635_ADDRESS, 0x1A, 0x00); // initialization
	
	// set configuration
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_RANGE_C, 0b00100100); // initialization: 8G range, 12 bits resolution
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_WAKE_C, 0x06); // low power mode then: 0x06 = 28 Hz, 1.6 uA
	// register 0x1C: default value = low power mode (nominal noise levels) -> ultra low power or normal mode also exist

	// set fifo
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_FIFO_C, 0b01011111); // fifo enabled, normal operation, 31 sample threshold
	//i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_INTR_C, 0b01000011); // fifo threshold interrupt enabled, active int drive (push-pull), active high
	//i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_INTR_C, 0b01000000); // fifo threshold interrupt enabled, open drain interrupt (pull-up needed), active low

	// go into cwake state
	i2cWriteRegister(ACC_MC3635_ADDRESS, ACC_MC3635_REG_MODE_C, 0b101);
	_delay_ms(1);
}