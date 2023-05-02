// F_CPU in project settings
#include "TinyFox.h"
#include "Sigfox.h"
#include "UART.h"
#include "Configuration.h"

// execute following avrdude commands before flashing: cd D:\Dropbox\Promotion\_GITHUB\TickTag\TickTagSoftware\avrdude\bin
// no BOD in sleep, sampled:		avrdude -c jtag2updi -P com11 -p t1626 -C C:\avrdude\avrdude.conf -U fuse1:w:0b00011000:m		(000 = BODLEVEL 1.8V, 1 = BOD SAMP FREQ 125HZ, 10 = BOD SAMPLED, 00 = BOD DISABLED IN SLEEP)
// run @16MHz:						avrdude -c jtag2updi -P com11 -p t1626 -C C:\avrdude\avrdude.conf -U fuse2:w:0b00000001:m		(0 = OSC REG ACCESS, XXXXX, 01 = RUN AT 16MHZ)
// only 8ms start delay:			avrdude -c jtag2updi -P com11 -p t1626 -C C:\avrdude\avrdude.conf -U fuse6:w:0b00000100:m		(100 = 8ms)
// (keep EEPROM after flashing):	avrdude -c jtag2updi -P com11 -p t1626 -C C:\avrdude\avrdude.conf -U fuse5:w:0b11110111:m

// TODO: sample voltage difference during acceleration

ISR(RTC_CNT_vect) { RTC.INTFLAGS = RTC_OVF_bm; }

uint8_t state = ST_FIRST_START_HARD_RESET;
uint8_t sigfoxData[12] = { 0 };
uint16_t batteryVoltage = 0;
uint8_t count1 = 0;
uint8_t count2 = 1;


void wait(uint16_t seconds) {
	deviceInitInternalRTCInterrupt(seconds);
	deviceStandbySleep();
	RTC.CTRLA = 0; // disable RTC interrupt
}

int main(void) {
	while(1) {
		RTC.CTRLA = 0; // disable RTC interrupt
		if(state == ST_FIRST_START_HARD_RESET) { deviceSetCPUSpeed(OSC16_PRESCALER_CPU_1MHZ);  } // 1MHz = 701uA @while(1), do this BEFORE reading supply voltage
			
		batteryVoltage = deviceReadSupplyVoltage(); // perform as first thing in every state
		
		if(count1 == 21) {
			count2 = count2+1;
			count1 = 0;
			deviceInitInternalRTCInterrupt(60);
			deviceStandbySleep();
		}
		
		
		if(state == ST_FIRST_START_HARD_RESET) { // will be re-entered after data download
					uint8_t error = 0;
					deviceInitPins();
					PORTA.OUTSET = PIN6_bm; // RESET being pulled up
					//usePrintf(); // NOT NEEDED

			
					initPrimaryUARTwith9600(); // init uart to communicate with sigfox module
					deviceSigfoxPinIdle();

					wait(5);
					sigfoxFirstStart(); // wake up module with UART
					if(!sigfoxAlive()) { error = 2; }
					else {
						_delay_ms(500);
						sigfoxPrintIdAndPac(SOFTWARE_VERSION);
						_delay_ms(500);
						if(!sigfoxSleep()) { error = 3; }
					}
			
			if(error == 2) {
				////DEBUG
			    initSecondUARTwith9600(); // uart to communicate with external programmer
			    print("Error 2");
			    _delay_ms(100);
			    deinitSecondUartPins();
			    _delay_ms(100);
			    ////DEBUG
				deviceInitInternalRTCInterrupt(30);	// try again	
			}
			if(error == 3) {
				////DEBUG
				initSecondUARTwith9600(); // uart to communicate with external programmer
				print("Error 3");
				_delay_ms(100);
				deinitSecondUartPins();
				_delay_ms(100);
				////DEBUG
				deviceInitInternalRTCInterrupt(30);	// try again
			}
			else {
				state = ST_TRACKING;
			    deviceInitInternalRTCInterrupt(120);
			}
		}
		else if((state == ST_TRACKING) && (batteryVoltage > 2700) && (count2 < 5)) {
				// get voltage difference as indicator for sun conditions
				sigfoxData[0] = batteryVoltage >> 8; // These two lines are used to split the temp in two bytes
				sigfoxData[1] = batteryVoltage; // 
				
				count1 = count1+1;
				// send data
				deviceSigfoxPinWakeUp();
				_delay_ms(100); // wait until sigfox module booted (does not send ok after boot via wake up pin)
				// TODO: set pin to idle again here already?!
				sigfoxSend(sigfoxData, 2);
				_delay_ms(500);
				deviceSigfoxPinIdle();
				_delay_ms(500);
				sigfoxSleep();
			    deviceInitInternalRTCInterrupt(SLEEP_SECONDS);
		}
		else if (batteryVoltage < 2701) {
			
			    deviceInitInternalRTCInterrupt(SLEEP_SECONDS);
		}
		deviceStandbySleep();
	}
}
