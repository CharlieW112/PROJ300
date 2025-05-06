/*
 * dfu_jump.h
 *
 *  Created on: Apr 7, 2025
 *      Author: charl
 */

#ifndef INC_DFU_JUMP_H_
#define INC_DFU_JUMP_H_

#include <stdint.h>

void trigger_dfu(void);
void bootloader_check(void);
void jump_to_dfu(void);

#endif /* INC_DFU_JUMP_H_ */
