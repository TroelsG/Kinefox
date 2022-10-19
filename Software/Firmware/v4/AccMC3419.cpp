#include "AccMC3419.h"

/*
function isNotMoving($x, $y, $z) {
	$magnitude = sqrt(pow($x, 2) + pow($y, 2) + pow($z, 2));
	if($magnitude > 0.97 && $magnitude < 1.03) {
		return true;
	}
	return false;
}

function calcTilt($x, $y, $z){ // values in g!
	$magnitude = sqrt(pow($x, 2) + pow($y, 2) + pow($z, 2));
	$tilt = rad2deg(acos($z/$magnitude));
	return $tilt;
}
*/

uint8_t accGetPitch(int16_t x, int16_t y, int16_t z) {
	float x_Buff = float(x);
	float y_Buff = float(y);
	float z_Buff = float(z);
	float pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3f; // -180.xx to +180.xx
	pitch = ((127.f/180.f) * pitch) + 127.f; // -180 = 0, +180 = 254
	if(pitch < 0.f) { pitch = 0.f; }
	if(pitch > 254.f) { pitch = 254.f; }
	return ((uint8_t) pitch);
}

uint8_t accGetRoll(int16_t x, int16_t y, int16_t z) {
	float y_Buff = float(y);
	float z_Buff = float(z);
	float roll = atan2(y_Buff , z_Buff) * 57.3f; // -180.xx to +180.xx
	roll = ((127.f/180.f) * roll) + 127.f; // -180 = 0, +180 = 254
	if(roll < 0.f) { roll = 0.f; }
	if(roll > 254.f) { roll = 254.f; }
	return ((uint8_t) roll);
}

uint32_t accMagnitude8G(int16_t x, int16_t y, int16_t z) {
	// 0.244mg per LSB (factor 4096), 1g = 4096, 8g = 32768, range = -32768 to +32767
	// example: 1404,1515,-3468 -> -0.342, 0.369, -0.846 -> vectorlen = 0.9842 -> 15.7mg noise?
	int32_t x32 = (int32_t) x;
	int32_t y32 = (int32_t) y;
	int32_t z32 = (int32_t) z;
	
	x32 = x32 * x32;
	y32 = y32 * y32;
	z32 = z32 * z32;
	
	int32_t result = (int32_t) sqrt(x32 + y32 + z32); // maximum value = 56754 (8G on all axes)
	result = result - 4096; // substract 1g
	if(result < 0) { result = -result; } // make value positive
	
	return ((uint32_t) result);
}

bool accIsAlive() {
	if(!i2cStartRead(ACC_MC3419_ADDRESS)) {
		i2cStop();
		return false;
	}
	i2cStop();
	return true;
}

bool accStart25HzWithFIFO() { //
	// ACC requires 70uA (always, between 2.0 - 3.0V)
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_RANGE, 0b00100000)) { return false; } // +/- 8g range = 0.244mg per LSB (factor 4096)
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_SAMPLE_RATE, 0x10)) { return false; } // WARNING: should be 25 Hz (minimum), but in fact is 50 Hz!
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_FIFO_CTRL, 0b01100000)) { return false; } // enable fifo, stop fifo when full
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_FIFO_CTRL2_SR2, 0x01)) { return false; } // divide ODR / 2 for lower FIFO data -> to get actual 25 Hz!
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_FIFO_TH, 31)) { return false; } // can store 31 samples
	if(!i2cWriteRegister(ACC_MC3419_ADDRESS, REG_ACC_MODE, 0x01)) { return false; } // set awake
	return true;
}

uint8_t accGetFifoLen() {
	uint8_t length;
	bool error = false;
	length = i2cReadRegister(ACC_MC3419_ADDRESS, REG_ACC_FIFO_WR_P, &error);
	if(error) { return 0; }
	length = (length & 0b11111);
	return length;
}

bool accFifoEmpty() {
	uint8_t reg;
	bool error = false;
	reg = i2cReadRegister(ACC_MC3419_ADDRESS, REG_ACC_FIFO_STAT, &error);
	reg &= 0x01;
	if (reg ^ 0x01) { return false; } // not empty
	return true;// empty
}

bool accRead(int16_t *x, int16_t *y, int16_t *z) {
	uint8_t i = 0;
	uint8_t data[6];
	if(!i2cStartWrite(ACC_MC3419_ADDRESS)) { return false; }
	if(!i2cWrite(REG_ACC_XOUT_LSB)) { return false; }
	if(!i2cStartRead(ACC_MC3419_ADDRESS)) { return false; }
	while(i < (6 - 1)) {
		data[i] = i2cRead(true); // with ack
		i++;
	}
	data[i] = i2cRead(false); // last one: no ack
	i2cStop();
	
	*x = (int16_t)((((uint16_t)data[1]) << 8) | data[0]);
	*y = (int16_t)((((uint16_t)data[3]) << 8) | data[2]);
	*z = (int16_t)((((uint16_t)data[5]) << 8) | data[4]);
	
	return true;
}

bool accReadFloat(float *x, float *y, float *z) {
	uint8_t i = 0;
	uint8_t data[6];
	if(!i2cStartWrite(ACC_MC3419_ADDRESS)) { return false; }
	if(!i2cWrite(REG_ACC_XOUT_LSB)) { return false; }
	if(!i2cStartRead(ACC_MC3419_ADDRESS)) { return false; }
	while(i < (6 - 1)) {
		data[i] = i2cRead(true); // with ack
		i++;
	}
	data[i] = i2cRead(false); // last one: no ack
	i2cStop();
	
	int16_t x16 = (int16_t)((((uint16_t)data[1]) << 8) | data[0]);
	int16_t y16 = (int16_t)((((uint16_t)data[3]) << 8) | data[2]);
	int16_t z16 = (int16_t)((((uint16_t)data[5]) << 8) | data[4]);
	
	*x = (((float) (x16)) / 32768.0f) * 78.456f;
	*y = (((float) (y16)) / 32768.0f) * 78.456f;
	*z = (((float) (z16)) / 32768.0f) * 78.456f;
	
	return true;
}