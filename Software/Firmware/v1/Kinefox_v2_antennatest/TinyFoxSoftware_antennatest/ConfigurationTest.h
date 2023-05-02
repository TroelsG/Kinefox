#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/** Important configuration - general */
#define SOFTWARE_VERSION									"TINYFOX VXX"
#define BLINK												true		// false, led on during transmission
#define VOLTAGE_SAMPLE_TIME_SECONDS							5			// 60, sample voltage difference over that amount of seconds
#define VOLTAGE_SEND										2900		// 2600, minimum voltage for sending a sigfox message
#define VOLTAGE_LOW											1000		// 2000, below that voltage sleep longer
#define SLEEP_SECONDS										5			// 60
#define SLEEP_SECONDS_LOW_VOLTAGE							5			// 7200
#define ACCELERATION_BURST_SECONDS							5			// 60

/** Enums */
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_TRACKING = 1 } tracker_state_t;

#define DATA_VOLTAGE_DIFF									0
#define DATA_ACCELERATION_SUM0								1
#define DATA_ACCELERATION_SUM1								2
#define DATA_ACCELERATION_SUM2								3
#define DATA_ACCELERATION_PITCH								4
#define DATA_ACCELERATION_ROLL								5

#endif