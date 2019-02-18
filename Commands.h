/*
 * Commands.h
 *
 *  Created on: Jul 28, 2016
 *      Author: rbonvent
 */

#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <stdio.h>
#include <string.h>

void setPreampGain(uint32_t chan_mask0, uint32_t chan_mask1, uint32_t chan_mask2, uint16_t value);
void setPreampThresh(uint32_t chan_mask0, uint32_t chan_mask1, uint32_t chan_mask2, uint16_t value);


#endif /* COMMANDS_H_ */
