#ifndef TEMP_SENSOR_AS6200_H_
#define TEMP_SENSOR_AS6200_H_

#include "os_type.h"

/**
 * Initializes the AS6200 sensor
 *
 * @return 0 for success, -1 otherwise
 */
int as6200_init( void );

/**
 * Configure the AS6200 sensor
 *
 * @return 0 for success, -1 otherwise
 */
int as6200_config( uint16_t config_value  );


/**
 * Reads the temperature
 *
 * @return temperature
 */
float as6200_read_temperature( void );


#endif /* TEMP_SENSOR_AS6200_H_ */
