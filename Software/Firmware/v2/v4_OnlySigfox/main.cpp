// F_CPU in project settings
#include "TinyFox.h"
#include "AccMC3635.h"
#include "SigfoxAxSip.h"
#include "GPS_L70R.h"
#include "I2C.h"
#include "UART.h"
#include "HelperFunctions.h"
#include "TestFunctions.h"
#include "Configuration.h"
#include "Math.h"

// TODO: emergency message when no regular message after X minutes/days
// TODO: EEPROM start counter coded into message (modulo 4?)

// execute following avrdude commands before flashing: cd D:\Dropbox\Promotion\_GITHUB\TickTag\TickTagSoftware\avrdude\bin
//C:\avrdude>avrdude -c jtag2updi -P com6 -p t1627 -C C:\avrdude\avrdude.conf -U fuse1:w:0b00011000:m
//C:\avrdude>avrdude -c jtag2updi -P com6 -p t1627 -C C:\avrdude\avrdude.conf -U fuse2:w:0b00000001:m
//C:\avrdude>avrdude -c jtag2updi -P com6 -p t1627 -C C:\avrdude\avrdude.conf -U fuse6:w:0b00000100:m
//C:\avrdude>avrdude -c jtag2updi -P com6 -p t1627 -C C:\avrdude\avrdude.conf -U fuse5:w:0b11110111:m

#define EEPROM_ACTIVATED									2							// 1 byte for storing if tag was already activated
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_WAIT_FOR_ACTIVATION, ST_TRACKING } tracker_state_t;
ISR(RTC_CNT_vect) { RTC.INTFLAGS = RTC_OVF_bm; }
ISR(TCA0_OVF_vect) { deviceIncrementTimer(); TCA0.SINGLE.INTFLAGS = TCA_SINGLE_OVF_bm; }  // Clear the interrupt flag (to reset TCA0.CNT)

uint8_t state = ST_FIRST_START_HARD_RESET;
uint32_t activation = 0;
uint32_t sleepCnt = 0;
uint32_t activationCnt = 0;
uint16_t batteryVoltage = 0;
uint8_t* eepromwritecount = 0;
uint8_t* eepromreadcount = 0;
uint8_t readcounter = 0;
uint8_t eepromvoltage = 0;
int16_t bufX[ACC_BURST_SAMPLES] = { 0 };
int16_t bufY[ACC_BURST_SAMPLES] = { 0 };
int16_t bufZ[ACC_BURST_SAMPLES] = { 0 };
uint64_t vedbaTotalSum = 0;
uint8_t lastError = 0;
	
// payload
uint32_t secondsTtf = 0;
uint8_t voltageDiff = 0;
uint8_t pitch = 0xFF;
uint8_t roll = 0xFF;
uint32_t vedbaBurstSum = 0;
int16_t tinyTemperature = 0;
uint8_t zeroCrossings = 0, zeroCrossingsAvgAmplitude = 0;
uint8_t maxMovement = 0; // only 0 .. 3

// data 
uint8_t sigfoxData[12] = { 0 };
uint8_t sigfoxDataPointer = 0;

void wait(uint16_t seconds) {
	deviceInitInternalRTCInterrupt(seconds);
	deviceStandbySleep();
	RTC.CTRLA = 0; // disable RTC interrupt
}

void waitMs(uint16_t milliseconds) {
	// needs implementation of interrupt routine and can only wakeup from standby sleep (RTC running):
	//ISR(RTC_CNT_vect) { RTC.INTFLAGS = RTC_OVF_bm; }
	// accuracy depends on PRESCALER -> smaller = better, but then max seconds smaller
	// currently (256 prescaler = 4Hz CNT updates): min. 0.25s - max. 2^16 / 4 = 16384 seconds = 4.55hrs
	cli();												// disable global interrupts
	while(RTC.STATUS > 0) {} 							// wait for all register to be synchronized
	RTC.CNT = 0;										// reset counter value
	while(RTC.STATUS > 0) {}							// wait until CNT is reset
	RTC.PER = milliseconds; 							// 16 bit wide maximum value, sets seconds between wakes (maximum value of cnt)
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; 					// running at 1.024Hz
	RTC.CTRLA = RTC_PRESCALER_DIV1_gc 					// prescaler, e.g. set to 1 -> 1024 / 1 = incrementing CNT with 1000Hz
	| 1 << RTC_RTCEN_bp         						// enable RTC
	| 1 << RTC_RUNSTDBY_bp;     						// run in standby, increases standby power consumption
	RTC.INTCTRL = 1 << RTC_OVF_bp; 						// overflow interrupt
	sei();												// enable global interrupts
	deviceStandbySleep();
	RTC.CTRLA = 0; // disable RTC interrupt
}

uint8_t calculatePitch(int16_t x, int16_t y, int16_t z) {
	float x_Buff = float(x) * ACC_LSB_VALUE_TO_G;
	float y_Buff = float(y) * ACC_LSB_VALUE_TO_G;
	float z_Buff = float(z) * ACC_LSB_VALUE_TO_G;
	float pitchFloat = atan2((-x_Buff), sqrt((y_Buff*y_Buff) + (z_Buff*z_Buff))) * 57.3f; // -180.xx to +180.xx
	//print("pitchFloat: "); printI16((int16_t) pitchFloat); print("\n\r");
	pitchFloat = ((127.f/180.f) * pitchFloat) + 127.f; // -180 = 0, +180 = 254
	if(pitchFloat < 0.f) { pitchFloat = 0.f; }
	if(pitchFloat > 254.f) { pitchFloat = 254.f; }
	return ((uint8_t) pitchFloat);
}

uint8_t calculateRoll(int16_t y, int16_t z) {
	float y_Buff = float(y) * ACC_LSB_VALUE_TO_G;
	float z_Buff = float(z) * ACC_LSB_VALUE_TO_G;
	float rollFloat = atan2(y_Buff, z_Buff) * 57.3f; // -180.xx to +180.xx
	//print("rollFloat: "); printI16((int16_t) rollFloat); print("\n\r");
	rollFloat = ((127.f/180.f) * rollFloat) + 127.f; // -180 = 0, +180 = 254
	if(rollFloat < 0.f) { rollFloat = 0.f; }
	if(rollFloat > 254.f) { rollFloat = 254.f; }
	return ((uint8_t) rollFloat);
}

void recordAccelerationBurst() {
	uint8_t buffIndex = 0;
	bool burstRecordingFinished = false;
	int32_t xSum = 0, ySum = 0, zSum = 0;
	int16_t xAvg = 0, yAvg = 0, zAvg = 0;
	int32_t x32 ,y32, z32;
	uint32_t vedba = 0;
	int8_t crossingState = 0; // 0 = first time
	int16_t amplitude; // UNTESTED
	uint32_t amplitudeSum; // UNTESTED
	bool alive = accMC3635Alive();
	
	#if (ACC_DEBUGGING == true)
	initSecondUARTwith9600();
	_delay_ms(100);
	#endif
	pitch = 0xFF;
	roll = 0xFF;
	maxMovement = 0;
	if(!alive) { lastError = 6; return; }
		
	accMC3635Start54HzWithFIFO();
	deviceSetCPUSpeed(OSC16_PRESCALER_CPU_4MHZ); // give the attiny more horsepower
	
	// record burst, store in global RAM
	while(!burstRecordingFinished) {
		waitMs(400); // 300 ms = 18 samples
		// NOT WORKING: go to sleep, wait on wake up by pin interrupt
		//set_sleep_mode(SLEEP_MODE_STANDBY); // standby sleep
		//sleep_enable(); // enable
		//sleep_cpu(); // stop
		while(!accMC3635FifoEmpty()) { // 60ms @ 1MHz for 26-27 entries
			accMC3635Read(&bufX[buffIndex], &bufY[buffIndex], &bufZ[buffIndex]);
			xSum += bufX[buffIndex];
			ySum += bufY[buffIndex];
			zSum += bufZ[buffIndex];
			buffIndex++;
			if(buffIndex >= ACC_BURST_SAMPLES) { burstRecordingFinished = true; break; }
		}
		//printU32(buffIndex); print("\n\r");
	}
		
	// calculate moving average
	if(buffIndex > 0) {
		xAvg = (int16_t) (xSum / buffIndex);
		yAvg = (int16_t) (ySum / buffIndex);
		zAvg = (int16_t) (zSum / buffIndex);
	}
	
	// process burst data
	vedbaBurstSum = 0;
	zeroCrossings = 0;
	amplitude = 0;
	amplitudeSum = 0;
	for(uint8_t i=0; i<buffIndex; i++) {
		// remove static acceleration
		bufX[i] -= xAvg;
		bufY[i] -= yAvg;
		bufZ[i] -= zAvg;
		
		// vedba calculation
		x32 = (int32_t) bufX[i]; // maximum value: 2048 (full 8G)
		y32 = (int32_t) bufY[i]; // maximum value: 2048 (full 8G)
		z32 = (int32_t) bufZ[i]; // maximum value: 2048 (full 8G)
		x32 = x32 * x32; // maximum value: 4194304
		y32 = y32 * y32; // maximum value: 4194304
		z32 = z32 * z32; // maximum value: 4194304
		vedba = (uint32_t) sqrt(x32 + y32 + z32); // maximum value: 3547.24 (full 8G on all axes)
		vedbaBurstSum += vedba; // maximum value @54 Hz in 1 s: 191550.96, 2 s: 383101.92, 3s: 574652.88, 4s: 766203.84
		
		// max movement bits
		if(vedba >= ACC_MAX_MOVEMENT_TRESHOLD_1) { maxMovement = 1; }
		if(vedba >= ACC_MAX_MOVEMENT_TRESHOLD_2) { maxMovement = 2; }
		if(vedba >= ACC_MAX_MOVEMENT_TRESHOLD_3) { maxMovement = 3; }
		
		// roll, pitch calculation
		if(vedba < ACC_PITCHROLL_THRESHOLD_NO_DYNAMIC_MOVEMENT) { // vedba is length of acceleration vector without static acceleration
			roll = calculateRoll(bufY[i] + yAvg, bufZ[i] + zAvg); // calculate roll based on raw data (static + dynamic acceleration)
			pitch = calculatePitch(bufX[i] + xAvg, bufY[i] + yAvg, bufZ[i] + zAvg); // calculate pitch based on raw data (static + dynamic acceleration)
		}
		
		// zero crossings (bufZ[i] already without static acceleration)
		if(bufZ[i] > ACC_ZERO_CROSSING_THRESHOLD) {
			if(crossingState == 0) { // first start, higher than minRange
				crossingState = 1;
				amplitude = bufZ[i]; // first value for current amplitude in wave
			}
			else if(crossingState == -1) { // changed from negative to positive
				if(zeroCrossings < 255) { zeroCrossings++; }
				crossingState = 1;
				if(amplitude < 0) { amplitude = -amplitude; } // look only at absolute values
				amplitudeSum += (uint32_t) amplitude; // add current amplitude to sum
				amplitude = bufZ[i]; // new amplitude = current value
			}
			else if(crossingState == 1) { // was above positive threshold before
				if(bufZ[i] > amplitude) { amplitude = bufZ[i]; } // check if current value is highest value in wave
			}
		}
		if(bufZ[i] < (-ACC_ZERO_CROSSING_THRESHOLD)) {
			if(crossingState == 0) { // first start, lower than minRange
				crossingState = -1;
				amplitude = bufZ[i]; // first value for current amplitude in wave
			}
			else if(crossingState == -1) { // was below negative threshold before
				if(bufZ[i] < amplitude) { amplitude = bufZ[i]; } // check if current value is smallest value in wave
			}
			else if(crossingState == 1) { // changed from positive to negative
				if(zeroCrossings < 255) { zeroCrossings++; }
				crossingState = -1;
				if(amplitude < 0) { amplitude = -amplitude; } // look only at absolute values
				amplitudeSum += (uint32_t) amplitude; // add current amplitude to sum
				amplitude = bufZ[i]; // new amplitude = current value
			}
		}
	}
	
	// calculate zero crossings amplitude
	zeroCrossingsAvgAmplitude = 0;
	if(zeroCrossings > 0) {
		amplitudeSum = amplitudeSum / zeroCrossings;
		amplitudeSum /= ACC_ZERO_CROSSING_AVG_AMPLITUDE_SCALE_FACTOR;
		if(amplitudeSum > 255) { amplitudeSum = 255; }
		zeroCrossingsAvgAmplitude = (uint8_t) amplitudeSum; 
	}
	
	//PORTC.PIN2CTRL = 0; // disable interrupt
	deviceSetCPUSpeed(OSC16_PRESCALER_CPU_1MHZ);
	
	// add burst sum of vedba to total sum
	vedbaTotalSum += vedbaBurstSum;
	
	// debugging output
	#if (ACC_DEBUGGING == true)
	/*for(uint8_t i=0; i<buffIndex; i++) {
		printI16(bufX[i] + xAvg); print(",");
		printI16(bufY[i] + yAvg); print(",");
		printI16(bufZ[i] + zAvg); print("\n\r");
	}*/
	print("xAvg: "); printI16(xAvg); print("\n\r");
	print("yAvg: "); printI16(yAvg); print("\n\r");
	print("zAvg: "); printI16(zAvg); print("\n\r");
	print("VeDBA: "); printU32(vedba); print("\n\r");
	print("VeDBABurstSum: "); printU32(vedbaBurstSum); print("\n\r");
	print("VeDBATotalSum: "); printU32(vedbaTotalSum); print("\n\r");
	print("Roll: "); printU32(roll); print("\n\r");
	print("Pitch: "); printU32(pitch); print("\n\r");
	print("ZeroCrossings: "); printU32(zeroCrossings); print("\n\r");
	print("zeroCrossingsAvgAmplitude: "); printU32(zeroCrossingsAvgAmplitude); print("\n\r");
	_delay_ms(20);
	initPrimaryUARTwith9600();
	#endif
}

void debugOutput(uint8_t *data, uint8_t dataLen) { // usePrintf() optional
	//usePrintf();
	initSecondUARTwith9600();
	_delay_ms(100);
	for(uint8_t i=0; i<dataLen; i++) {
		printU32(data[i]);
		print(" ");
	}
	print("\n\r");
	_delay_ms(100);
	initPrimaryUARTwith9600();
}

void tinyFoxStartPrint() {
	const uint8_t RESPONSE_LEN = 32;
	char pac[RESPONSE_LEN] = { 0 };
	char id[RESPONSE_LEN] = { 0 };
	println("AT$I=11");
	if(!uartRead('\n', pac, RESPONSE_LEN, 3000)) { return; }
	_delay_ms(100);
	println("AT$I=10");
	if(!uartRead('\n', id, RESPONSE_LEN, 3000)) { return; }
	
	//initSecondUARTwith9600(); // uart to communicate with external programmer
	println(SOFTWARE_VERSION);
	print("PAC:");
	println(pac);
	print("ID:");
	println(id);
	print("\n\r");
		
	_delay_ms(100);
	//initPrimaryUARTwith9600();
	//deinitSecondUartPins();
}

void compressData(gps_t *gpsData, uint8_t *buffer) {
	int32_t tempSigned;
	uint32_t temp, timestamp;
	
	timestamp = gpsGetUTCTimestamp(gpsData);
	if(timestamp < COMPRESSION_TIMESTAMP) { timestamp = 0; }
	else { timestamp -= COMPRESSION_TIMESTAMP; }
			
	tempSigned = gpsData->latitude + 9000000L;
	temp = tempSigned; // 25 bit
	buffer[0] = (temp >> 17) & 0xFF;
	buffer[1] = (temp >> 9) & 0xFF;
	buffer[2] = (temp >> 1) & 0xFF;
	buffer[3] = (temp << 7) & 0xFF;
	tempSigned = gpsData->longitude + 18000000L;
	temp = tempSigned; // 26 bit
	buffer[3] = buffer[3] | ((temp >> 19) & 0xFF);
	buffer[4] = (temp >> 11) & 0xFF;
	buffer[5] = (temp >> 3) & 0xFF;
	buffer[6] = (temp << 5) & 0xFF;
	temp = timestamp; // 29 bit
	buffer[6] = buffer[6] | ((temp >> 24) & 0xFF);
	buffer[7] = (temp >> 16) & 0xFF;
	buffer[8] = (temp >> 8) & 0xFF;
	buffer[9] = (temp) & 0xFF;	
}

/** Tracking function */

bool getGPSFix(gps_t *gpsData) {
	
	deviceInitTimer(); // for getting current system time (for ttf measurement)
	deviceL70powerOn();
		
	#if (GPS_DEBUGGING == true)
		deviceSigfoxPinWakeUp(); // OTHERWISE NO SNIFFING POSSIBLE!!
	#endif
	
	char messageBuffer[L70_START_BUFFER_SIZE];
	char *startMessage;
	uint16_t startTimeWaitAfterFirstFix = 0;
	initSecondUARTwith9600(); // init uart to communicate with L70
	bool gpsResult = false;
	bool trackingRunning = true;

	
	//deviceBlinkGreen(5);
	
	_delay_ms(2000); // normally takes < 900 ms until responsive
	print("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"); // request only GPGGA and GPRMC messages
	
	//if(gpsConfigureBeforeStart(messageBuffer)) {
	while(trackingRunning) {
		// wait on message from L70 (max. 2 * 80 Byte*(8+2) * (0.1042ms/bit) = 166.6ms)
		if(!uartReadMultipleLines(2, '\n', messageBuffer, L70_START_BUFFER_SIZE, 3000)) { // flush and memset done in function
			trackingRunning = false;
			break;
		}
			
		// switch to debug uart
		#if (GPS_DEBUGGING == true)
			initPrimaryUARTwith9600(); // uart to communicate with external programmer
		#endif
			
		// decode received message 1
		startMessage = strchr(messageBuffer, '$'); // remove possible noise before $
		if((startMessage != NULL) && (strlen(startMessage) > 6)) {
			// no need to check if startMessage[0] == '$' -> strchr would return NULL otherwise
			if((startMessage[3] == 'R') && (startMessage[4] == 'M') && (startMessage[5] == 'C')) { gpsDecodeGPRMCandGPGGANew(startMessage, gpsData, SENTENCE_TYPE_GPRMC); }
			else if((startMessage[3] == 'G') && (startMessage[4] == 'G') && (startMessage[5] == 'A')) { gpsDecodeGPRMCandGPGGANew(startMessage, gpsData, SENTENCE_TYPE_GPGGA); }
			#if (GPS_DEBUGGING == true)
				printf("%lu %s\n\r", deviceGetMillis(), startMessage); // print full string
			#endif
		}
			
		// decode received message 2
		if(strlen(startMessage) > 1) {
			startMessage = strchr(startMessage+1, '$'); // search for next $ after first $
			if((startMessage != NULL) && (strlen(startMessage) > 6)) {
				if((startMessage[3] == 'R') && (startMessage[4] == 'M') && (startMessage[5] == 'C')) { gpsDecodeGPRMCandGPGGANew(startMessage, gpsData, SENTENCE_TYPE_GPRMC); }
				else if((startMessage[3] == 'G') && (startMessage[4] == 'G') && (startMessage[5] == 'A')) { gpsDecodeGPRMCandGPGGANew(startMessage, gpsData, SENTENCE_TYPE_GPGGA); }
			}
		}
		
		// update current ttf in seconds
		secondsTtf = deviceGetMillis() / 1000;
		
		// early timeout: not even got time (all modes)
		if((secondsTtf > GET_FIX_TIMEOUT_NOT_EVEN_TIME_SECONDS) && (gpsData->year == 80) && (gpsData->hour == 0) && (gpsData->minute < ((GET_FIX_TIMEOUT_NOT_EVEN_TIME_SECONDS / 60) + 1))) {
			trackingRunning = false;
			break;
		}
				
		// check if timeout reached (only in sometimes mode, because continuous mode should not stop)
		if(secondsTtf > GET_FIX_TIMEOUT_SECONDS) { // upper level timeout  
			trackingRunning = false;
			break;
		}
			
		// check if fix found and store if yes
		if(gpsData->fix == 1) {
			// got a fix
			if(startTimeWaitAfterFirstFix == 0) { startTimeWaitAfterFirstFix = (uint16_t) secondsTtf; } // for SOMETIMES, first time getting a fix right now
			uint16_t maxWaitTime = MAX_WAIT_ON_GOOD_HDOP_SECONDS;

			if(((gpsData->hdop > 0) && (gpsData->hdop < MIN_HDOP))
				|| ((secondsTtf - startTimeWaitAfterFirstFix) > maxWaitTime)) { // only store if hdop is good enough
				// determine seconds
				gpsData->ttfSeconds = secondsTtf; // downcast from 32 to 16 bit
				trackingRunning = false;
				gpsResult = true;
				break;
			}
		}
		
	}
	//}

	// after the run: stop everything
	deviceL70powerOff();
	uartFlush();
	uartOff();
	deinitSecondUartPins(); // otherwise leakage
	return gpsResult;
}

void runTracking() {
	

	// blink to indicate start
	
	// TODO: if didnt get fix: just send message without GPS data
		//deviceBlinkGreen(15);
		sigfoxData[sigfoxDataPointer] = batteryVoltage >> 8; sigfoxDataPointer++;
		sigfoxData[sigfoxDataPointer] = batteryVoltage; sigfoxDataPointer++;

		eeprom_write_byte(eepromreadcount, map(batteryVoltage, 2800, 3700, 1, 255));
	    eepromreadcount = eepromreadcount + 1;

		// send data
		initPrimaryUARTwith9600();
		deviceSigfoxPinWakeUp();
		_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
		sigfoxFirstStart();
		_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
		sigfoxSend(sigfoxData, sigfoxDataPointer);
		_delay_ms(100);
		deviceSigfoxPinIdle();
		sigfoxSleep();
}

int main(void) {
	while(1) {
		RTC.CTRLA = 0; // disable RTC interrupt
		if(state == ST_FIRST_START_HARD_RESET) { deviceSetCPUSpeed(OSC16_PRESCALER_CPU_1MHZ);  } // 1MHz = 701uA @while(1), do this BEFORE reading supply voltage
		batteryVoltage = deviceReadSupplyVoltage(); // perform as first thing in every state
		if(state == ST_FIRST_START_HARD_RESET) { // will be re-entered after data download
			deviceInitPins();
			usePrintf(); // NOT NEEDED, requires a lot of flash memory
			initPrimaryUARTwith9600(); // init uart to communicate with sigfox module
			deviceSigfoxPinIdle();
			lastError = 0; // IMPORTANT: reset all errors

			wait(1); // sleep 1 second (sigfox module is running!)		
			
			while (readcounter < 200)
			{
				eepromvoltage = eeprom_read_byte(eepromreadcount);
				printU32(eepromvoltage);
				println("");
 				eepromreadcount = eepromreadcount + 1;
				readcounter = readcounter + 1;
			}
			
			eepromreadcount = 0;
						
			// test sensors
			i2cInit();
			devicePowerOn();
			if(!accMC3635Alive()) { lastError = 4; }		
			devicePowerOff();
			
			// read id and pac of sigfox module and put module to sleep
	
			deviceSigfoxPinWakeUp();
			_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
			sigfoxFirstStart(); // wake up module with UART
			//if(!sigfoxAlive()) { lastError = 1; }
			sigfoxAlive();
			_delay_ms(500);
			tinyFoxStartPrint(); // alternative function: sigfoxPrintIdAndPac();
			_delay_ms(500);
			deviceSigfoxPinIdle();
			//if(!sigfoxSleep()) { lastError = 2; }
			sigfoxSleep();
			
			// evaluate errors and transit into next state
			if(lastError > 0) {
				deviceBlinkGreen(lastError + 1);
				deviceInitInternalRTCInterrupt(60);	// try again after some seconds	
			}
			else {
			//	deviceLedGreenOn();
			//	wait(1);
			//	deviceLedGreenOff();
				vedbaTotalSum = 0;
				//deviceBlinkGreen(3); // indicate start
				state = ST_TRACKING;
				
				_delay_ms(100);
				runTracking();
				memset(sigfoxData, 0, 12);
				sigfoxDataPointer = 0;
				}
				//deviceInitInternalRTCInterrupt(5);
			}
		if (batteryVoltage > 3550) { // a lot of energy available
				memset(sigfoxData, 0, 12);
				sigfoxDataPointer = 0;
				runTracking();
				sleepCnt = 1;
				// backup power stays on
				deviceInitInternalRTCInterrupt(SHORT_SLEEP);
		}
		else if((state == ST_TRACKING) && (activationCnt == ACTIVATION_COUNT)) {
			if(batteryVoltage > 2900) { // battery low
				if(sleepCnt == SLEEP_COUNT) {
				memset(sigfoxData, 0, 12);
				sigfoxDataPointer = 0;
				runTracking();
				// backup power stays on
				deviceInitInternalRTCInterrupt(LONG_SLEEP);
				sleepCnt = 0;					
	
				}
                //else if((sleepCnt == RESEND_COUNT) && (activation == 0)) {
	        	// send data
				//deviceBlinkGreen(4);
				//_delay_ms(100);
	         	//initPrimaryUARTwith9600();
		        //deviceSigfoxPinWakeUp();
		        //_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
		        //sigfoxFirstStart();
		        //_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
		        //sigfoxSend(sigfoxData, sigfoxDataPointer);
		        //_delay_ms(100);
		        //deviceSigfoxPinIdle();
		        //sigfoxSleep();
				//memset(sigfoxData, 0, 12);
				//sigfoxDataPointer = 0;
				//deviceInitInternalRTCInterrupt(LONG_SLEEP);
				//}
				else if(activation == 1) {
				activation = 0;
				runTracking();
				// backup power stays on
				deviceInitInternalRTCInterrupt(LONG_SLEEP);		
				}
				else{
				deviceInitInternalRTCInterrupt(LONG_SLEEP);		
				}
			sleepCnt++;
			}
			else { // battery pretty much empty
				deviceInitInternalRTCInterrupt(14400);
			}
		}
		if (activationCnt < (ACTIVATION_COUNT)) {
			activationCnt++;
			activation = 1;
			deviceInitInternalRTCInterrupt(14400); //PLEASE REMEMBER TO CHANGE!!!
			}
		deinitAllUARTPins();
		deviceStandbySleep();
	}
}