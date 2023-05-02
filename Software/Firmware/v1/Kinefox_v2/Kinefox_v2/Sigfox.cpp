#include "Sigfox.h"

void sigfoxFirstStart() {
	println("\n"); // IMPORTANT: wake sigfox up
	_delay_ms(1000);	
}

bool sigfoxAlive() {
	println("AT");
	if(!sigfoxWaitForOkay(1000)) { return false; }
	return true;
}

bool sigfoxWaitForOkay(uint16_t timeoutMs) {
	char *startMessage;
	char messageBuffer[SIGFOX_START_BUFFER_SIZE];

	if(!uartRead('\n', messageBuffer, SIGFOX_START_BUFFER_SIZE, timeoutMs)) { return false; } // wait until L70 is booted, should return $PMTK010,00X*2E\r\n
	startMessage = strchr(messageBuffer, 'O'); // remove possible noise before $
	if((startMessage != NULL) && (strlen(startMessage) > 0)) {
		if(startMessage[0] == 'O') {
			return true;
		}
	}	
	return false;
}

bool sigfoxSleep() {
    // Set module in deep sleep
	println("AT$P=2");
	if(!sigfoxWaitForOkay(2000)) { return false; }
	return true;
}

bool sigfoxSend(uint8_t *data, uint8_t len) {
	if(len > 12) { return false; } // sigfox supports a maximum of 12 byte
	print("AT$SF=");
	int8_t i = 0;
	while(i<len) {
		printByteAsHex(data[i]); // with printf and without _delay_ms(1) other device receives corrupted data, seems to be printf problem
		i++;
	}
	print("\n");
	print("\r");
	if(!sigfoxWaitForOkay(12000)) { return false; } // wait for 10 seconds
	return true;
}

bool sigfoxPrintIdAndPac(const char* additionalInfo) {
	const uint8_t RESPONSE_LEN = 32;
	char pac[RESPONSE_LEN] = { 0 };
	char id[RESPONSE_LEN] = { 0 };
	println("AT$I=11");
	if(!uartRead('\n', pac, RESPONSE_LEN, 3000)) { return false; }
	_delay_ms(100);
	println("AT$I=10");
	if(!uartRead('\n', id, RESPONSE_LEN, 3000)) { return false; }
		
	initSecondUARTwith9600(); // uart to communicate with external programmer
	print("\n\r");
	println(additionalInfo);
	print("PAC: ");
	println(pac);
	print("ID: ");
	println(id);
	_delay_ms(100);
	initPrimaryUARTwith9600();
	deinitSecondUartPins();
	_delay_ms(100);
	return true;
}