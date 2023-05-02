#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

/** Most important settings */
#define SOFTWARE_VERSION									"Kinefox_v3"

#define ACTIVATION_COUNT									1 // min 2	 
#define SLEEP_COUNT									        6	
#define RESEND_COUNT									    3		
#define SHORT_SLEEP										    14400	//3600
#define LONG_SLEEP										    14400   //14400
#define VOLTAGE_THRESHOLD_1									3900
#define VOLTAGE_THRESHOLD_2									3700
#define VOLTAGE_THRESHOLD_3									3500
#define MIN_VOLTAGE_FOR_ACTIVATION							3700

#define ACTIVATION_MAG_Y_THRESHOLD							700000UL
#define SLEEP_ACTIVATION_SECONDS							60
#define SLEEP_ACTIVATION_VOLTAGE_LOW_SECONDS				60

#define SLEEP_MODULO_2										6UL
#define SLEEP_MODULO_3										(6UL * 24UL)

/** On-board accelerometer data processing (acc is running at 54 Hz and +/- 8G, 1 LSB = 3.9 mg) */
#define ACC_BURST_SAMPLES									31*6		// maximum: 255 samples (4.72 seconds @ 54 Hz), 31*6 @ 54 Hz = 186 = 3.44 seconds
#define ACC_PITCHROLL_THRESHOLD_NO_DYNAMIC_MOVEMENT			13UL		// 1 = 3.9 mg, 13 = 50.7 mg
#define ACC_ZERO_CROSSING_THRESHOLD							13			// 1 = 3.9 mg, 13 = 50.7 mg
#define ACC_ZERO_CROSSING_AVG_AMPLITUDE_SCALE_FACTOR		4			// max. 255, 1 -> 255 = 0.994g, 2 -> 255 = 1.998g, 3 -> 255 = 3.996g, 4 -> 255 = 7.992g
#define ACC_LSB_VALUE_TO_G									0.0039f		// 1 LSB = 3.9 mg
#define ACC_MAX_MOVEMENT_TRESHOLD_1							20UL		// 20, in LSB, compares to VeDBA, max. value: 3547.24, 20 = 78 mg (w/o earth gravitation)
#define ACC_MAX_MOVEMENT_TRESHOLD_2							512UL		// 512, in LSB, compares to VeDBA, max. value: 3547.24, 512 = 2 g (w/o earth gravitation)
#define ACC_MAX_MOVEMENT_TRESHOLD_3							1024UL		// 1024, in LSB, compares to VeDBA, max. value: 3547.24, 1024 = 4 g (w/o earth gravitation)

/* Debugging */
#define GPS_DEBUGGING										false
#define PAYLOAD_DEBUGGING									false		// printing payload before sending
#define ACC_DEBUGGING										false		// printing fifo values and all calculation results

/** Important configuration - battery management */
#define MIN_VOLTAGE_UNDER_LOAD								3050U		// unit: mV, threshold for exiting under load! GPS only works down to 3V

/** Important configuration - timeout for both modes */
#define MIN_HDOP											30U			// unit: HDOP x 10
#define GET_FIX_TIMEOUT_NOT_EVEN_TIME_SECONDS				120		// 120/80, timeout triggered when did not even get a valid time (year still 80, hour = 0, minute < 3)
#define GET_FIX_TIMEOUT_SECONDS								300		// 254/100, only when tracking frequency > 5 ("sometimes"), timeout until GPS attempt is canceled
#define WAIT_TIME_FIRST_FIX_SECONDS							30			// 30, only when tracking frequency > 5 ("sometimes"), always wait that time when getting first fix in session to collect orbit data
#define MAX_WAIT_ON_GOOD_HDOP_SECONDS						9			// 9, only when tracking frequency > 5 ("sometimes"), maximum wait time after fix in case HDOP is not good enough

/** Data storage defines */
#define DATA_FIX_LENGTH										10			// 10 byte compressed (uncompressed: 4 byte timestamp, 4 byte lat, 4 byte lon)
#define METADATA_PREFIX_LENGTH_IN_MEMORY					16			// meta data before actual GPS fix data
#define METADATA_ADDRESS_OF_ADDRESS_POINTER					0			// meta data address 0 - 3, 4 bytes
#define METADATA_ADDRESS_OF_TTF_SUM							4			// meta data address, 4 - 7, 4 bytes
//#define METADATA_ADDRESS_OF_NO_FIX_CNT					8			// NOT USED ANYMORE, meta data address, 8 - 9, 2 bytes
#define METADATA_ADDRESS_OF_HDOP_SUM						10			// meta data address, 10 - 13, 4 bytes
// 2 BYTES LEFT IN HEADER!

/** Enums */
typedef enum { STORE_MEMORY_RETURN_ERROR = 0, STORE_MEMORY_RETURN_MEMORY_FULL = 1, STORE_MEMORY_RETURN_SUCCESS = 2 } store_memory_return_t;

#define COMPRESSION_TIMESTAMP								1618428394UL

#endif