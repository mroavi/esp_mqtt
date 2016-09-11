#include "ir_receiver.h"
#include "user_interface.h"
#include "gpio.h"
#include "osapi.h"

void (*callback)( uint32_t ir_command);

/* ====================================== */
/* HARDWARE TIMER                         */
/* ====================================== */

#define FRC1_ENABLE_TIMER  BIT7
#define FRC1_AUTO_LOAD  BIT6

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


static uint32 	intervalArr[70] = {0};
static uint32	edgeIndex = 0;

/* initialize to a very big number */
static uint32	minInterval = 0xFFFFFFFF;

/* this array will contain the raw IR message */
static uint32 	rawIrMsg[100] = {0};
static uint32 	rawIrMsgLen = 0;

/* this variable will contain the decoded IR command */
static uint32 	irCmd = 0;

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


/* ====================================== */
/* HARDWARE TIMER                         */
/* ====================================== */

static void hwTimerCallback( void )
{
	int i, j;
	int logicState = 1;
	int logicStateLen = 0;
	bool repeatCode = false;

	/* stop the HW TIMER */
	RTC_REG_WRITE(FRC1_CTRL_ADDRESS, DIVDED_BY_16 | TM_EDGE_INT);

	//Set GPIO0 to LOW
	gpio_output_set(0, BIT0, BIT0, 0); // TODO: can be removed?

	/* load the HW TIMER for next IR message frame */
	uint32 ticks = usToTicks(70000);
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* derive the raw IR message frame */
	for( i = 0 ; i < ( edgeIndex - 1 ) ; i++)
	{
		/* find number of bits in current interval */
		logicStateLen = ( intervalArr[i] / minInterval );

		for( j = 0 ; j < logicStateLen ; j++)
		{
			rawIrMsg[ rawIrMsgLen ] = logicState;
			rawIrMsgLen++;
		}

		/* toggle state */
		logicState ^= 1;

#if 0
		os_printf( "\r\nDuration of interval %d: %d us\r\n", i, intervalArr[i] );
#endif
	}

#if 0
	/* print the received raw IR message frame */
	os_printf( "\r\nRAW IR CODE: ");
	for ( i = 0 ; i < rawIrMsgLen ; i++ )
	{
		os_printf( "%d", rawIrMsg[i] );
	}
#endif

	/**********************************************
	 * DECODE NEC MESSAGE FRAME!
	 * - every message frame contains 32 coded bits
	 **********************************************/

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
		os_printf( "\r\nREPEATED CODE");
#endif
		repeatCode = true;
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
		os_printf("\r\nIR CMD: %x", irCmd);
#endif

	}

	/**********************************************
	 * END - DECODE NEC MESSAGE FRAME!
	 * - every message frame contains 32 coded bits
	 **********************************************/

	/* reset index */
	edgeIndex = 0;

#if 0
	os_printf("\r\nEnd of IR message frame\r\n");
#endif

	callback( irCmd );

	return;
}


/* ====================================== */
/* GPIO INTERRPUT                         */
/* ====================================== */

static void gpioCallback(void *arg)
{
	uint16 			gpio_status = 0;
	static uint32	currTicks = 0;
	static uint32	prevTicks = 0;

	/* get the GPIO interrupt status */
	gpio_status = GPIO_REG_READ( GPIO_STATUS_ADDRESS );

	/* clear the interrupt */
	GPIO_REG_WRITE( GPIO_STATUS_W1TC_ADDRESS, gpio_status);

	/* did GPIO 12 (connected to IR receiver) generate the ISR? */
	if( gpio_status == BIT(12) )
	{
		/* yes, and is it the first edge of the IR message frame? */
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
			os_printf("\n\nBeginning of IR message frame detected");
#endif

			// Set GPIO0 to HIGH
			gpio_output_set( BIT0, 0, BIT0, 0 ); // TODO: can be removed?
		}
		else
		{
			/* record time of current edge */
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
}



int ir_receiver_init( void (*cb)( uint32_t ) )
{
	/* Register the callback */
	callback = cb;

	/* ====================================== */
	/* GPIO INTERRPUT                         */
	/* ====================================== */

	/* Set GPIO12 in GPIO mode */
	PIN_FUNC_SELECT( PERIPHS_IO_MUX_MTDI_U, FUNC_GPIO12 );

	/* Set GPIO12 as input */
	GPIO_DIS_OUTPUT( GPIO_ID_PIN(12) );

	/* Disable all GPIO interrupts */
	ETS_GPIO_INTR_DISABLE();

	/* Set a GPIO callback function */
	ETS_GPIO_INTR_ATTACH( gpioCallback, NULL );

	/* Trigger GPIO interrupt for rising and falling edges */
	gpio_pin_intr_state_set( GPIO_ID_PIN(12), GPIO_PIN_INTR_ANYEDGE );

	/* Enable GPIO interrupts */
	ETS_GPIO_INTR_ENABLE();


	/* ====================================== */
	/* HARDWARE TIMER                         */
	/* ====================================== */

	/* The hardware timer is used to indicate when a complete IR message frame should have
	 * arrived in order to process the received data and calculate the IR command.
	 *
	 * It is configured in "one-shot" mode. It is started when the beginning of an
	 * IR message frame is detected and stopped after the complete message frame has been read.
	 * This means that the duration of the HW timer should be longer than the duration of
	 * the longest possible message frame. In the NEC IR transmission protocol all message frames
	 * have a duration of approximately 67.5ms.
	 */

	/* load the HW TIMER */
	uint32 ticks = usToTicks(70000); // 70ms
	RTC_REG_WRITE(FRC1_LOAD_ADDRESS, ticks);

	/* register callback function */
	ETS_FRC_TIMER1_INTR_ATTACH( hwTimerCallback, NULL );

	/* enable the GPIO and HW TIMER interrupts */
	TM1_EDGE_INT_ENABLE();
	ETS_FRC1_INTR_ENABLE();

	/* don't start timer yet */
	/* the timer is started inside the GPIO INT callback */

	return 0;
}
