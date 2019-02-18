/*
 * MCP3427.c
 *
 *  Created on: Jul 2, 2018
 *      Author: mu2e
 */

#include "MCP3427.h"
#include "I2C.h"
#include "utils.h"

void MCP3427_setup(MCP3427 *self, I2C *i2c )
{
    self->_i2c = i2c;
    self->_address = (0x68); //the slow amps are hardwired to address 0
}

int32_t MCP3427_read(MCP3427 *self, int rdy, int channel, int mode, uint8_t rate, uint8_t gain)
{   // RDY should be 1 to initiate a read.  When reading back, 1 means the data hasn't been updated
	// channel = 0/1 for straw 0/1 (channel 1/2 in the datasheet)
	// mode = 0/1 for one shot/continuous mode
	// rate = 0/1/2 for 240/60/15 samples per second, at 12/14/16 bits per sample
	// gain = 0/1/2/3 for x1/2/4/8 internal gain
	// Good default values are read(self, 1, 0, 0, 2, 0)
	uint8_t value = (rdy << 7) | (channel << 5) | (mode << 4) | (rate << 2) | gain;

	// Send a "write" bit.
	if (I2C_start(self->_i2c, self->_address << 1 | I2C_WRITE)!=0) return -1;
	if (I2C_write(self->_i2c, value) != 0) return -1;
	I2C_stop(self->_i2c);
	delayUs(100);

	if (I2C_start(self->_i2c, self->_address << 1 | I2C_READ)!=0) return 0;
	uint8_t buf[3];
	uint8_t bufr[3];
	uint8_t i=0;
	buf[0] = I2C_read(self->_i2c, 0);
	buf[1] = I2C_read(self->_i2c, 0);
	buf[2] = I2C_read(self->_i2c, 1);
	I2C_stop(self->_i2c);

	for (i=0; i<3; i++){

		bufr[i] = 0;

	}
	// Reverse the bits in the bytes
	for (i = 0 ; i < 8 ;i++){
     	bufr[0] |= ((buf[0] & (1<<i)) >> i ) << (7-i);
     	bufr[1] |= ((buf[1] & (1<<i)) >> i ) << (7-i);
     	bufr[2] |= ((buf[2] & (1<<i)) >> i ) << (7-i);
	}
	// This bit magic constructs an int32_t with the 3 bytes.
	return (0x0000 | (bufr[0] << 16) | (bufr[1] << 8) | (bufr[2]));
}
