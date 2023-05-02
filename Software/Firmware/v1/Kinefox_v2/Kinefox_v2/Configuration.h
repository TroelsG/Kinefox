#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/** Important configuration - general */
#define SOFTWARE_VERSION									"TINYFOX V_3"
#define SLEEP_SECONDS										14400			// 1200 seconds = 20 minutes

/** Enums */
typedef enum { ST_FIRST_START_HARD_RESET = 0, ST_TRACKING = 1 } tracker_state_t;
	
#define DATA_VOLTAGE								        0

#endif