#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

#include "os_type.h"

typedef enum button
{
    CH_MIN   		= 0xFFA25D,
    CH   			= 0xFF629D,
    CH_PLUS   		= 0xFFE21D,
	PREV   			= 0xFF22DD,
	NEXT   			= 0xFF02FD,
	PLAY_PAUSE   	= 0xFFC23D,
	VOL_MIN   		= 0xFFE01F,
	VOL_PLUS   		= 0xFFA857,
	EQ   			= 0xFF906F,
	NUM_0   		= 0xFF6897,
	NUM_100_PLUS 	= 0xFF9867,
	NUM_200_PLUS 	= 0xFFB04F,
	NUM_1   		= 0xFF30CF,
	NUM_2   		= 0xFF18E7,
	NUM_3   		= 0xFF7A85,
	NUM_4   		= 0xFF10EF,
	NUM_5   		= 0xFF38C7,
	NUM_6   		= 0xFF5AA5,
	NUM_7   		= 0xFF42BD,
	NUM_8   		= 0xFF4AB5,
	NUM_9   		= 0xFF52AD,
	OTHER			= 0xFFFFFF
} button_t;


/**
 * Initializes the IR receiver module
 *
 * @param callback
 * @return 0 for success, -1 otherwise
 */
int ir_receiver_init( void (*cb)( button_t pressed_button, bool repeated_code ) );


#endif /* IR_RECEIVER_H_ */
