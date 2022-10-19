#include "Hall_MMC5603NJ.h"

bool hallMMCAlive() {
	if(!i2cStartRead(HALL_MMC_ADDRESS)) {
		i2cStop();
		return false;
	}
	i2cStop();
	return true;
}

uint32_t hallMMCReadY() {
	uint8_t reg = 0;
	uint16_t securityCounter = 0;
	uint32_t retVal = 0;
	bool error = false;
	
	_delay_ms(8); // datasheet says: 5 ms until operational
	i2cWriteRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_CONTROL1, 0b00000001); // BW = 01 = 3.5 ms measurement time, 2 - 4 mG RMS noise
	i2cWriteRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_CONTROL0, 0b00100001); // TM_M and Auto_SR
	while(true) {
		reg = i2cReadRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_STATUS1, &error);
		if(((reg >> 6) & 0x01) == 1) { break; }
		securityCounter++;
		if(securityCounter > 1000) { return 0; }
		if(error) { return 0; }
		_delay_ms(1);
	}
	//_delay_ms(10);
	
	uint8_t r0 = i2cReadRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_YOUT0, &error);
	uint8_t r1 = i2cReadRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_YOUT1, &error);
	uint8_t r2 = i2cReadRegister(HALL_MMC_ADDRESS, HALL_MMC_REG_YOUT2, &error);
	
	retVal = (uint32_t)((((uint32_t)r0) << 16) | (((uint32_t)r1) << 8) | r2);
	retVal = retVal >> 4;
	
	return retVal;
}