#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

#include "os_type.h"

static const char *button[] =
{
	"CH-",
	"CH",
	"CH+",
	"PREV",
	"NEXT",
	"PLAY/PAUSE",
	"VOL-",
	"VOL+",
	"EQ",
	"0",
	"100+",
	"200+",
	"1",
	"2",
	"3",
	"4",
	"5",
	"6",
	"7",
	"8",
	"9",
	"UNKNOWN"
};

typedef void (*call_me_on_gpio_edge_interrupt)(void * arg);


/**
 * Initializes the IR receiver module
 *
 * @param callback
 * @return 0 for success, -1 otherwise
 */
int ir_receiver_init( void (*cb)( const char * pressed_button, bool repeated_code ) );

/**
 * User must call this function on every rising or falling
 * edge of the GPIO connected to the IR receiver.
 *
 * @return Function address
 */
call_me_on_gpio_edge_interrupt get_function_pointer( void );


#endif /* IR_RECEIVER_H_ */
