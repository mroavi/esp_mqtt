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
	"OTHER"
};


/**
 * Initializes the IR receiver module
 *
 * @param callback
 * @return 0 for success, -1 otherwise
 */
int ir_receiver_init( void (*cb)( const char * pressed_button, bool repeated_code ) );


#endif /* IR_RECEIVER_H_ */
