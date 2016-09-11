#ifndef IR_RECEIVER_H_
#define IR_RECEIVER_H_

#include "os_type.h"

/**
 * Initializes the IR receiver module
 *
 * @param callback
 * @return 0 for success, -1 otherwise
 */
int ir_receiver_init( void (*cb)( uint32_t ir_command ) );


#endif /* IR_RECEIVER_H_ */
