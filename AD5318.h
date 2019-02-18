/*
 * AD5318.h
 *
 *  Created on: Jul 28, 2016
 *      Author: rbonvent
 */

#ifndef AD5318_H_
#define AD5318_H_


#include <stdio.h>
#include <string.h>
#include "setup.h"
#include "drivers/CoreSPI/core_spi.h"

// Set DAC value to a single channel (0-7)
// AD5318 is a 10-bit DAC with straight binary, 0<=value<1023 => 0 to 3.3V
void AD5318_write(spi_instance_t spi , uint8_t sync, uint8_t channel, uint16_t value);

#endif /* AD5318_H_ */
