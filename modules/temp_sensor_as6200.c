/*
 * Driver for the AS6200 temperature sensor made by AMS
 */

//*****************************************************************************
//
// Include files
//
//*****************************************************************************

#include "temp_sensor_as6200.h"
#include "brzo_i2c.h"
#include "osapi.h"
#include "os_type.h"


//*****************************************************************************
//
// Constant definitions
//
//*****************************************************************************

#define AS6200_SLAVE_ADDR			0x48

#define AS6200_REG_ADDR_TVAL		0x00
#define AS6200_REG_ADDR_CONFIG		0x01
#define AS6200_REG_ADDR_TLOW		0x02
#define AS6200_REG_ADDR_THIGH		0x03

#define AS6200_SAMPLE_SIZE			2		// bytes

//*****************************************************************************
//
// Macros
//
//*****************************************************************************

//*****************************************************************************
//
// Type definitions
//
//*****************************************************************************



//*****************************************************************************
//
// Static definitions
//
//*****************************************************************************

/* I2C read and write buffers */
uint8_t read_buff[ AS6200_SAMPLE_SIZE ];
uint8_t write_buff[3];


uint8_t return_code = 0;

//*****************************************************************************
//
// Extern data definitions
//
//*****************************************************************************

//*****************************************************************************
//
// Local function prototypes
//
//*****************************************************************************

static void invert_endianness( void* pData, uint16_t numBytes );

//*****************************************************************************
//
// Local function implementations
//
//*****************************************************************************

static void invert_endianness(void* pData, uint16_t numBytes)
{
	uint8_t 	temp;
	uint16_t 	i;

	for (i = 0 ; i < (numBytes >> 1) ; i++)
	{
		temp = *( (uint8_t*)pData + (i) );
		*( (uint8_t*)pData + (i) ) = *( (uint8_t*)pData + (numBytes - i - 1) );
		*( (uint8_t*)pData + (numBytes - i - 1) ) = temp;
	}
}


//*****************************************************************************
//
// Extern function implementations
//
//*****************************************************************************

int as6200_init( void )
{
	// Setup i2c with clock stretching timeout of 2000 usec
	brzo_i2c_setup(2000);

	return 0;
}

/* look at page 8 of the datasheet */
int as6200_config( uint16_t config_value )
{

#if 0
	os_printf("AS6300 config value: 0x%x\r\n", config_value);
#endif

	invert_endianness(&config_value, 2);

#if 0
	os_printf("AS6300 config value (inverted endianness): 0x%x\r\n", config_value);
#endif

	// Start a transaction to the ADT7420 at 400 KHz
	brzo_i2c_start_transaction(AS6200_SLAVE_ADDR, 400);

	write_buff[0] = AS6200_REG_ADDR_CONFIG; // select configuration register
	write_buff[1] = config_value;
	os_memcpy( &write_buff[1], &config_value, sizeof(config_value) );

	brzo_i2c_write(write_buff, 3, false); // Write three bytes with no repeated start

	return_code = brzo_i2c_end_transaction();

	return return_code;
}


float as6200_read_temperature( void )
{
	int16_t raw_temperature;
	float temperature;
	uint8_t msb;
	uint8_t lsb;

	// Start a transaction to the ADT7420 at 400 KHz
	brzo_i2c_start_transaction(AS6200_SLAVE_ADDR, 400);
	write_buff[0] = AS6200_REG_ADDR_TVAL;
	brzo_i2c_write(write_buff, 1, false);
	return_code = brzo_i2c_end_transaction();

	brzo_i2c_start_transaction(AS6200_SLAVE_ADDR, 400);
	brzo_i2c_read(read_buff, 2, false);
	return_code |= brzo_i2c_end_transaction();

#if 0
		os_printf("read_buff[0]: 0x%x\r\n", read_buff[0]);
		os_printf("read_buff[1]: 0x%x\r\n", read_buff[1]);
#endif


	if (return_code == 0)
	{
		/* temperature values are actually 12-bit long */
		msb = read_buff[0];
		lsb = read_buff[1];

		raw_temperature = ( msb << 8 ) | lsb;
		raw_temperature = raw_temperature >> 4; // shift 4 bits to the right

		temperature = raw_temperature * 0.0625;
#if 0
		os_printf("AS6200 temperature: %d\r\n", (int)temperature);
#endif
	}
	else
	{
	  // Error Handling here...
		os_printf("I2C ERROR");
	}

	return temperature;
}

//*****************************************************************************
//
// End of file
//
//*****************************************************************************
