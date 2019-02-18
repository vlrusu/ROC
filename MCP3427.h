/*
 * MCP3427.h
 *
 *  Created on: Jul 2, 2018
 *      Author: mu2e
 */

#ifndef MCP3427_H_
#define MCP3427_H_

#include "utils.h"
#include "I2C.h"

// Author: Jean-Francois Caron, February 2017
// This class is used together with I2cMaster and MCP23S17 to control
// an MCP3427 two-channel preamp with programmable gain and resolution.
// RJB: converted to SoftConsole

typedef struct {
	uint8_t _address;
    I2C *_i2c; // I2C object for communicating with DAC
} MCP3427;

// The i2cAddress value contains first a 4-bit "device code", then an address.
// The address ranges from 000 to 111, depending on resistors set on the no-amps.
// The values are as follows:
// R12? R14? Addr Label
// No   No   000  "0"
// Yes  No   101  "5"
// Yes  Yes  110  "6"
// No   Yes  111  "7"

// Writing to RDY in continuous mode does nothing.
// Setting RDY in one-shot mode starts a new conversion.
// Channel 0/1 is for selecting channels 1/2.
// Mode 0/1 is for selecting one-shot/continuous mode.
// Rate 0/1/2 is for selecting 240/60/15 samples per second (& 12/14/16 bit conversions).
// Gain 0/1/2/3 is for selecting x1/2/4/8 gain.
void MCP3427_setup(MCP3427 *self, I2C *i2c);
int32_t MCP3427_read(MCP3427 *self,int rdy, int channel, int mode, uint8_t rate, uint8_t gain);

#endif /* MCP3427_H_ */
