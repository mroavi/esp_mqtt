/*
 * Uses the NEC Infrared Transmission Protocol to receive IR commands:
 *   http://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 *
 * NEC commands: https://arduino-info.wikispaces.com/IR-RemoteControl
 */

//*****************************************************************************
//
// Include files
//
//*****************************************************************************

#include "ir_receiver.h"
#include "user_interface.h"
#include "gpio.h"
#include "osapi.h"


//*****************************************************************************
//
// Constant definitions
//
//*****************************************************************************

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

//-----------------------------------------------------------------------------
// HARDWARE TIMER
//-----------------------------------------------------------------------------

#define FRC1_ENABLE_TIMER  	BIT7
#define FRC1_AUTO_LOAD  	BIT6

//TIMER PREDIVED MODE
typedef enum {
    DIVDED_BY_1 = 0,	//timer clock
    DIVDED_BY_16 = 4,	//divided by 16
    DIVDED_BY_256 = 8,	//divided by 256
} TIMER_PREDIVED_MODE;

typedef enum {			//timer interrupt mode
    TM_LEVEL_INT = 1,	// level interrupt
    TM_EDGE_INT   = 0,	//edge interrupt
} TIMER_INT_MODE;

typedef enum {
    FRC1_SOURCE = 0,
    NMI_SOURCE = 1,
} FRC1_TIMER_SOURCE_TYPE;

//*****************************************************************************
//
// Static definitions
//
//*****************************************************************************

static void (*callback)( const char * pressed_button, bool repeated_code );

static uint32 	intervalArr[70] = {0};
static uint32	edgeIndex = 0;

/* initialize to a very big number */
static uint32	minInterval = 0xFFFFFFFF;

/* this array will contain the raw IR message */
static uint32 	rawIrMsg[200] = {0}; // TODO: calculate exactly how large this array has to be
static uint32 	rawIrMsgLen = 0;

/* used to inform if the IR message contains a repeated code */
bool repeatCode;


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

static void 		hwTimerCallback( void );
static void 		gpioEdgeTrigger(void *arg);

static void 		getIrRawMessageBits( void );
static uint32_t 	getHexButtonCommand( void );
static const char * getPressedButtonName( uint32_t hex );

static uint32	 	usToTicks( uint32_t us );
static uint32 		ticksToUs( uint32_t ticks );


//*****************************************************************************
//
// Local function implementations
//
//*****************************************************************************

//-----------------------------------------------------------------------------
// GPIO EDGE INTERRUPT
//-----------------------------------------------------------------------------

static void gpioEdgeTrigger(void *arg)
{
	uint16 			gpio_status = 0;
	static uint32	currTicks = 0;
	static uint32	prevTicks = 0;

	/* Is this the first edge of the IR message frame? */
	if ( edgeIndex == 0 )
	{
		/* yes, then store counter value */
		prevTicks = RTC_REG_READ(FRC1_COUNT_ADDRESS);

		/* and start the HW TIMER */
		RTC_REG_WRITE(FRC1_CTRL_ADDRESS,
				DIVDED_BY_16 | FRC1_ENABLE_TIMER | TM_EDGE_INT);

		/* reset relevant variables */
		minInterval = 0xFFFFFFFF; // initialize to a very big number
		rawIrMsgLen = 0;

#if 0
		os_printf("\n\nBeginning of IR message frame detected\r\n");

		// Set GPIO0 to HIGH
		gpio_output_set( BIT0, 0, BIT0, 0 );
#endif
	}
	else
	{
		/* no, then record time of current edge */
		currTicks = RTC_REG_READ( FRC1_COUNT_ADDRESS );

		/* record time interval between current edge and previous edge */
		intervalArr[ edgeIndex - 1 ] = ticksToUs( prevTicks - currTicks );

		/* keep track the shortest interval */
		minInterval = ( intervalArr[ edgeIndex - 1 ] < minInterval ) ? intervalArr[ edgeIndex - 1 ] : minInterval;

		/* save time of current edge */
		prevTicks = currTicks;
	}

	edgeIndex++;
}



//-----------------------------------------------------------------------------
// HARDWARE TIMER
//-----------------------------------------------------------------------------

/* The hardware timer is used to indicate when an IR message frame should have
 * arrived completely so that it can start processing it and extracting the IR command.
 *
 * It is configured in "one-shot" mode. It is started when the beginning of an
 * IR message frame is detected and stopped after a fixed amount of time. This fixed
 * amount of time should be larger than the the "worse case" duration of a message frame,
 * i.e. it should be larger than the duration of the longest possible message frame.
 * In the NEC IR transmission protocol all message frames have a duration of approximately 67.5ms.
 */

/* assumes timer clk of 5MHz */
static uint32 usToTicks( uint32_t us )
{
	return ( us * 10) >> 1;
}

/* assumes timer clk of 5MHz */
static uint32 ticksToUs( uint32_t ticks )
{
	return ( ticks << 1 ) / 10;
}

static void hwTimerCallback( void )
{
	/* this variable will contain the decoded IR command */
	uint32 	irCmd = 0;

	/* this variable will contain the button pressed */
	const char * pressedButton;

	/* stop the HW TIMER */
	RTC_REG_WRITE(FRC1_CTRL_ADDRESS, DIVDED_BY_16 | TM_EDGE_INT);

#if 0
	//Set GPIO0 to LOW
	gpio_output_set(0, BIT0, BIT0, 0);
	os_printf("\r\nEnd of IR message frame\r\n");
#endif

	/* load the HW TIMER for next IR message frame */
	uint32 ticks = usToTicks(70000);
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/*  */
	getIrRawMessageBits();

	/* Decode NEC message frame (every message frame contains 32 coded bits) */
	irCmd = getHexButtonCommand();

	pressedButton = getPressedButtonName( irCmd );

	/* reset index */
	edgeIndex = 0;

	callback( pressedButton, repeatCode );

	return;
}


static void getIrRawMessageBits( void )
{
	int 	i, j;
	uint32 	logicState = 1;
	int 	logicStateLen = 0;

#if 0
		os_printf("Minimum interval: %d us\r\n\r\n", minInterval);
#endif

	/* derive the raw IR message frame */
	for( i = 0 ; i < ( edgeIndex - 1 ) ; i++)
	{
		/* find number of bits in current interval */
		logicStateLen = ( intervalArr[i] / minInterval );

		for( j = 0 ; j < logicStateLen ; j++)
		{
			rawIrMsg[ rawIrMsgLen ] = logicState;
			rawIrMsgLen++;
#if 0
		os_printf("rawIrMsgLen: %d\r\n", rawIrMsgLen);
#endif

		}

		/* toggle state */
		logicState ^= 1;

#if 0
		os_printf( "Interval%d: %d us\r\n", i, intervalArr[i] );
		os_printf( "Number of bits: %d\r\n\r\n", logicStateLen );
#endif
	}

#if 0
	os_printf("\r\nRAW IR CODE LENGTH: %d\r\n", rawIrMsgLen);

	/* print the received raw IR message frame */
	os_printf("RAW IR CODE: ");
	for ( i = 0 ; i < rawIrMsgLen ; i++ )
	{
		os_printf( "%d", rawIrMsg[i] );
	}
	os_printf( "\r\n");
#endif

}


static uint32_t getHexButtonCommand( void )
{
	int i, j;

	/* this variable will contain the decoded IR command */
	static uint32 	irCmd = 0;

	/* set index to the beginning of the coded bits */

	/* the message frame starts with a burst of 16 logic 1's, skip them all */
	i = 0 ;
	while ( rawIrMsg[i] == 1 ) i++;

	/* the message frame continues with a burst of 8 logic 0's, skip them all */
	j = 0;
	while (rawIrMsg[i] == 0)
	{
		i++;
		j++;
	}

	/* if the number of zeros is 4, then ignore the current message frame since
	 * it corresponds to a "REPEATED CODE".
	 */
	if ( j <= 4 )
	{
#if 0
		os_printf( "REPEATED CODE\r\n");
#endif
		repeatCode = true;
	}
	else
	{
		repeatCode = false;
	}

	/* decode raw message only if it is not a repeat code */
	if (repeatCode == false)
	{
		/* at this point 'i' contains the index of the beginning of the encoded bits */

		/* decode raw message
		 * - [1][0][0][0] 	represents a '1'
		 * - [1][0]			represents a '0'
		 */
		irCmd = 0;
		for (j = 0; j < 32; j++)
		{
			if (rawIrMsg[i + 2] == 0)
			{
				/* it is a '1', so left shift a '1' */
				irCmd = (irCmd << 1) | 1;

				/* move to the beginning of the next encoded bit
				 * (increment i until next 1 in raw message frame)
				 */
				do {i++;} while ( rawIrMsg[i] == 0 );
			}
			else {
				/* it is a '0', so left shift a '0' */
				irCmd = irCmd << 1;

				/* move to the beginning of the next encoded bit */
				i += 2;
			}
		}

#if 0
		/* print the received IR cmd */
		INFO("\r\nIR CMD: %x", irCmd);
#endif

		return irCmd;
	}
}


static const char * getPressedButtonName( uint32_t hex ) {

	const char * pressedButton;

	switch( hex )
	{
	case 0xFFA25D:
		pressedButton = button[0];
		break;

	case 0xFF629D:
		pressedButton = button[1];
		break;

	case 0xFFE21D:
		pressedButton = button[2];
		break;

	case 0xFF22DD:
		pressedButton = button[3];
		break;

	case 0xFF02FD:
		pressedButton = button[4];
		break;

	case 0xFFC23D:
		pressedButton = button[5];
		break;

	case 0xFFE01F:
		pressedButton = button[6];
		break;

	case 0xFFA857:
		pressedButton = button[7];
		break;

	case 0xFF906F:
		pressedButton = button[8];
		break;

	case 0xFF6897:
		pressedButton = button[9];
		break;

	case 0xFF9867:
		pressedButton = button[10];
		break;

	case 0xFFB04F:
		pressedButton = button[11];
		break;

	case 0xFF30CF:
		pressedButton = button[12];
		break;

	case 0xFF18E7:
		pressedButton = button[13];
		break;

	case 0xFF7A85:
		pressedButton = button[14];
		break;

	case 0xFF10EF:
		pressedButton = button[15];
		break;

	case 0xFF38C7:
		pressedButton = button[16];
		break;

	case 0xFF5AA5:
		pressedButton = button[17];
		break;

	case 0xFF42BD:
		pressedButton = button[18];
		break;

	case 0xFF4AB5:
		pressedButton = button[19];
		break;

	case 0xFF52AD:
		pressedButton = button[20];
		break;

	default:
		pressedButton = button[21];
		break;
	}

	return pressedButton;
}




//*****************************************************************************
//
// Extern function implementations
//
//*****************************************************************************

int ir_receiver_init( void (*cb)( const char * pressed_button, bool repeated_code ) )
{
	/* Register the callback */
	callback = cb;

	//-----------------------------------------------------------------------------
	// GPIO EDGE INTERRPUT
	//-----------------------------------------------------------------------------

	/* Set GPIO12 in GPIO mode */
	PIN_FUNC_SELECT( PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12 );

	/* Set GPIO12 as input */
	GPIO_DIS_OUTPUT( GPIO_ID_PIN(12) );

	/* Disable all GPIO interrupts */
	ETS_GPIO_INTR_DISABLE();

	/* Set a GPIO callback function */
//	ETS_GPIO_INTR_ATTACH( gpioCallback, NULL );

	/* Trigger GPIO interrupt for rising and falling edges */
	gpio_pin_intr_state_set( GPIO_ID_PIN(12), GPIO_PIN_INTR_ANYEDGE );

	/* Enable GPIO interrupts */
	ETS_GPIO_INTR_ENABLE();


	//-----------------------------------------------------------------------------
	// HARDWARE TIMER
	//-----------------------------------------------------------------------------

	/* The hardware timer is used to indicate when an IR message frame should have
	 * arrived completely so that it can start processing it and extracting the IR command.
	 *
	 * It is configured in "one-shot" mode. It is started when the beginning of an
	 * IR message frame is detected and stopped after a fixed amount of time. This fixed
	 * amount of time should be larger than the the "worse case" duration of a message frame,
	 * i.e. it should be larger than the duration of the longest possible message frame.
	 * In the NEC IR transmission protocol all message frames have a duration of approximately 67.5ms.
	 */

	/* load the HW TIMER */
	uint32 ticks = usToTicks(70000); // 70ms
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* register callback function */
	ETS_FRC_TIMER1_INTR_ATTACH( hwTimerCallback, NULL );

	/* enable the HW TIMER interrupts */
	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();

	/* don't start timer yet */
	/* the timer is started inside the GPIO INT callback */

	return 0;
}

/*
 * Example of how to return a function pointer:
 * 	- http://www.newty.de/fpt/fpt.html
 */
call_me_on_gpio_edge_interrupt get_function_pointer( void )
{
	return &gpioEdgeTrigger;
}

//*****************************************************************************
//
// End of file
//
//*****************************************************************************
