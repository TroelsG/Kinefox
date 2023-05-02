#include "TinyFox.h"

void deviceInitPins() {
	// OUTPUT = lowest setting
	PORTA.DIR = (INPUT_DIR << PIN0_bp)	// PA0: UPDI
	| (OUTPUT_DIR << PIN1_bp)			// PA1: TXD
	| (INPUT_DIR << PIN2_bp)			// PA2: RXD 
	| (OUTPUT_DIR << PIN3_bp)			// PA3: WAKEUP
	| (OUTPUT_DIR << PIN4_bp)			// PA4: **UNUSED**
	| (OUTPUT_DIR << PIN5_bp)			// PA5: **UNUSED**
	| (OUTPUT_DIR << PIN6_bp)			// PA6: **UNUSED**
	| (OUTPUT_DIR << PIN7_bp);			// PA7: **UNUSED**
	PORTB.DIR = (OUTPUT_DIR << PIN0_bp)	// PB0: SCL
	| (OUTPUT_DIR << PIN1_bp)			// PB1: SDA
	| (OUTPUT_DIR << PIN2_bp)			// PB2: TXD2 
	| (OUTPUT_DIR << PIN3_bp)			// PB3: **UNUSED**
	| (OUTPUT_DIR << PIN4_bp)			// PB4: **UNUSED**
	| (OUTPUT_DIR << PIN5_bp);			// PB5: **UNUSED**
	PORTC.DIR = (OUTPUT_DIR << PIN0_bp)	// PC0: **UNUSED**
	| (OUTPUT_DIR << PIN1_bp)			// PC1: POWER
	| (OUTPUT_DIR << PIN2_bp)			// PC2: LED
	| (OUTPUT_DIR << PIN3_bp);			// PC3: **UNUSED**
	
	// enable PULL-UPs
	PORTA.PIN3CTRL = PORT_PULLUPEN_bm; // NEW: pullup enabled already in start 
}

void deviceStandbySleep() {		
	// disable I2C pull-ups (important, being set by )
	PORTB.PIN0CTRL = 0;
	PORTB.PIN1CTRL = 0;
	
	set_sleep_mode(SLEEP_MODE_STANDBY); 		// standby sleep
	sleep_enable();								// enable
	sleep_cpu();            					// stop
}

void deviceInitInternalRTCInterrupt(uint16_t seconds) {
	// needs implementation of interrupt routine and can only wakeup from standby sleep (RTC running):
	//ISR(RTC_CNT_vect) { RTC.INTFLAGS = RTC_OVF_bm; }
	// accuracy depends on PRESCALER -> smaller = better, but then max seconds smaller
	// currently (256 prescaler = 4Hz CNT updates): min. 0.25s - max. 2^16 / 4 = 16384 seconds = 4.55hrs
	cli();												// disable global interrupts
	while(RTC.STATUS > 0) {} 							// wait for all register to be synchronized
	RTC.CNT = 0;										// reset counter value
	while(RTC.STATUS > 0) {}							// wait until CNT is reset
	RTC.PER = seconds * 4; 								// 16 bit wide maximum value, sets seconds between wakes (maximum value of cnt)
	RTC.CLKSEL = RTC_CLKSEL_INT1K_gc; 					// running at 1.024Hz
	RTC.CTRLA = RTC_PRESCALER_DIV256_gc 				// prescaler, e.g. set to 256 -> 1024 / 256 = incrementing CNT with 4Hz
		| 1 << RTC_RTCEN_bp         					// enable RTC
		| 1 << RTC_RUNSTDBY_bp;     					// run in standby, increases standby power consumption
	RTC.INTCTRL = 1 << RTC_OVF_bp; 						// overflow interrupt
	sei();												// enable global interrupts			
}

uint16_t deviceReadSupplyVoltage() {
	#if defined (__AVR_ATtiny1626__)
		uint32_t res;
	
		ADC0.CTRLA = ADC_ENABLE_bm; // enable ADC
		//ADC0.CTRLB = ADC_PRESC_DIV2_gc; // DEFAULT setting
		ADC0.CTRLC = VREF_AC0REFSEL_1V024_gc | (TIMEBASE_VALUE << ADC_TIMEBASE0_bp); // Vref = 1.024V
		ADC0.CTRLE = 100; // sample duration ((100 * 2) / F_CPU seconds), 1 MHz = 0.2ms
		ADC0.MUXPOS = ADC_MUXPOS_DAC_gc; // using DAC as MUX voltage, ADC_MUXPOS_VDDDIV10_gc doesn't work
		ADC0.COMMAND = ADC_MODE_SINGLE_12BIT_gc; // single mode with 12 bit
		ADC0.COMMAND |= ADC_START_IMMEDIATE_gc; // start conversion

		while(true) {
			if(ADC0.INTFLAGS & ADC_RESRDY_bm) { // wait until measurement done
				res = (uint32_t) ADC0.RESULT; // get raw adc result
				if(res == 0) { res = 1; }
				res = (4096UL * 1024UL) / res; // convert result to mV
				break;
			}
		}
		ADC0.CTRLA = 0; // disable ADC
		return ((uint16_t) res);
	#else
		// the App Note AN2447 uses Atmel Start to configure Vref but we'll do it explicitly in our code
		VREF.CTRLA = VREF_ADC0REFSEL_1V1_gc;  			// set the Vref to 1.1V

		// the following section is directly taken from Microchip App Note AN2447 page 13     
		ADC0.MUXPOS = ADC_MUXPOS_INTREF_gc; 	   		// ADC internal reference, the Vbg
    
		// take care! ADC needs certain peripheral clock speed (50kHz - 1.5MHz after ADC_PRESC_..) for maximum resolution, here CLOCK = 1MHz divided by 4 = 0.25MHz = within range of 50kHz - 1.5MHz
		ADC0.CTRLC = ADC_PRESC_DIV4_gc					// CLK_PER divided by certain value
			| ADC_REFSEL_VDDREF_gc						// Vdd (Vcc) be ADC reference
			| 0 << ADC_SAMPCAP_bp;						// Sample Capacitance Selection: disabled
		ADC0.CTRLA = 1 << ADC_ENABLE_bp					// ADC Enable: enabled
			| 1 << ADC_FREERUN_bp						// ADC Free run mode: enabled
			| ADC_RESSEL_10BIT_gc;						// 10-bit mode
		//ADC0.CTRLD = ADC_INITDLY_DLY256_gc;			// init delay 256 CLK_ADC cycles (important!)
		ADC0.COMMAND |= 1;								// start running ADC
		_delay_ms(5);									// REALLY?! <- YES, important to wait for first measurement!
	
		#if (VCC_DO_NOT_USE_AVERAGE == true)
			uint32_t vcc, temp;
			while(true) {
				if(ADC0.INTFLAGS) {
					temp = (uint32_t) ADC0.RES;
					if(temp == 0) { temp = 1; }
					vcc = (1024UL * 1100UL) / temp;
					break;
				}
			}
		#else
			// read some samples and calculate average
			uint32_t vcc = 0, temp = 0;
			for(uint8_t i=0; i<VCC_SAMPLES; i++) {
				if(ADC0.INTFLAGS) {							// if an ADC result is ready
					temp = (uint32_t) ADC0.RES;				// e.g. 450 for 2.5V
					if(temp == 0) { temp = 1; }
					vcc += (1024UL * 1100UL) / temp; 		// 1100 = 1.1V
					_delay_ms(VCC_SAMPLE_DELAY_MS);
				}		
			}
			vcc /= VCC_SAMPLES;								// build integer average
		#endif
		// disable ADC again
		ADC0.CTRLA = 0 << ADC_ENABLE_bp					// ADC Enable: enabled (seems okay) -> no, changed that to 0! should be disabled!
			| 0 << ADC_FREERUN_bp						// ADC Free run mode: disabled (important for sleep)
			| ADC_RESSEL_10BIT_gc;						// 10-bit mode
			 
		#if defined (__AVR_ATtiny1606__)
			if(vcc >= 60) { vcc = vcc - 60; } // 1606 ADC seems to measure voltage higher than expected (only tested with one board)
		#endif
		          
		return (uint16_t) vcc;
	#endif
}

void deviceSetCPUSpeed(uint8_t prescalerDivision) {
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, prescalerDivision | 1 << CLKCTRL_PEN_bp); // first bit = prescaler, always enabled
	_PROTECTED_WRITE(CLKCTRL.MCLKCTRLA, 0); // use internal 16/20MHz oscillator
	while(CLKCTRL.MCLKSTATUS & CLKCTRL_SOSC_bm) { ; } // wait until clock changed
}

void deviceSigfoxPinIdle() {
	PORTA.OUTSET = PIN3_bm; // active
}

void deviceSigfoxPinWakeUp() {
	PORTA.OUTCLR = PIN3_bm;
}






