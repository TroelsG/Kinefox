#ifndef TinyFox_h
#define TinyFox_h

// AVR libs
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <util/atomic.h>

/** VCC MEASUREMENT */
#define TIMEBASE_VALUE					(uint8_t)ceil(F_CPU*0.000001) // only for ATTINY1626

/** CPU SPEED */
// CPU speed after writing OSCCFG fuse to use 20MHz (factory default)
#define OSC20_PRESCALER_CPU_0_315MHZ			CLKCTRL_PDIV_64X_gc		// based on 20MHz fuse setting (not 16MHz) -> 20/64 = 0.3125 MHz
#define OSC20_PRESCALER_CPU_0_625MHZ			CLKCTRL_PDIV_32X_gc		// see above
#define OSC20_PRESCALER_CPU_0_833MHZ			CLKCTRL_PDIV_24X_gc		// see above
#define OSC20_PRESCALER_CPU_1_25MHZ				CLKCTRL_PDIV_16X_gc		// see above
#define OSC20_PRESCALER_CPU_2_5MHZ				CLKCTRL_PDIV_8X_gc		// see above
#define OSC20_PRESCALER_CPU_5MHZ				CLKCTRL_PDIV_4X_gc		// see above
#define OSC20_PRESCALER_CPU_10MHZ				CLKCTRL_PDIV_2X_gc		// see above

// CPU speed after writing OSCCFG fuse to use 16MHz instead of default 20MHz (avrdude -c jtag2updi -P com4 -p t1616 -C ..\etc\avrdude.conf -U fuse2:w:0b00000001:m)
// power consumption values for: 3V, 1616-only (special board), TCD0 fix, BOD fully disabled, while(1)
// _delay_ms needs more power than while(1) (!)
#define OSC16_PRESCALER_CPU_0_25MHZ				CLKCTRL_PDIV_64X_gc		// 329uA
#define OSC16_PRESCALER_CPU_0_5MHZ				CLKCTRL_PDIV_32X_gc		
#define OSC16_PRESCALER_CPU_0_666MHZ			CLKCTRL_PDIV_24X_gc		
#define OSC16_PRESCALER_CPU_1MHZ				CLKCTRL_PDIV_16X_gc		// 556uA (16MHz/16)
#define OSC16_PRESCALER_CPU_1_66MHZ				CLKCTRL_PDIV_10X_gc
#define OSC16_PRESCALER_CPU_2MHZ				CLKCTRL_PDIV_8X_gc		// 859uA (UART @115200 works here and above) - 842uA (real board + sampled BOD 125Hz)
#define OSC16_PRESCALER_CPU_2_66MHZ				CLKCTRL_PDIV_6X_gc		// 1.07mA (default value at start-up )
#define OSC16_PRESCALER_CPU_4MHZ				CLKCTRL_PDIV_4X_gc

/** PIN SETTINGS */
#define UNUSED_PIN_SETTING						(0 << PORT_INVEN_bp) | (0 << PORT_PULLUPEN_bp) | (1 << PORT_ISC2_bp) // not inverted, no pull-up, disable input buffer (can't read input values)
#define OUTPUT_DIR								1
#define INPUT_DIR								0

void deviceInitPins();										// (old: @2.7V: 780uA (1MHz, BOD enabled))
void deviceStandbySleep();									// (old: @2.7V: 920nA with internal RTC running and BOD disabled)
void deviceInitInternalRTCInterrupt(uint16_t seconds);		// for standby sleep
uint16_t deviceReadSupplyVoltage();	// minimum 100kHz CPU clock speed and after prescaler between 50kHz and 1.5MHz (for max resolution), in V*1000 (2700 = 2.7V)
		
// CPU speeds
void deviceSetCPUSpeed(uint8_t prescalerDivision);
	
// Sigfox
void deviceSigfoxPinIdle();
void deviceSigfoxPinWakeUp();
#endif
