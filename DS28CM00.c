/*
 * DS28CM00.c
 *
 *  Created on: Dec 5, 2018
 *      Author: vrusu
 */

#include "DS28CM00.h"

#define DS28CM00_DEVICEADDRESS    0xA0
#define DS28CM00_UIDREGISTER      0x00
#define DS28CM00_CONTROLREGISTER  0x08

void DS28CM00_setup(DS28CM00 *self, I2C *i2c )
{
    self->_i2c = i2c;
    self->_address = DS28CM00_DEVICEADDRESS ;
}

int32_t getUID(DS28CM00 *self, uint8_t* buffer)
{

	// Send a "write" bit.
	if (I2C_start(self->_i2c, self->_address << 1 | I2C_WRITE)!=0) return -1;
	if (I2C_write(self->_i2c, DS28CM00_UIDREGISTER ) != 0) return -1;
	I2C_stop(self->_i2c);
	delayUs(100);

	if (I2C_start(self->_i2c, self->_address << 1 | I2C_READ)!=0) return 0;

	for (uint8_t i = 0; i < 7; i++) buffer[i] = I2C_read(self->_i2c, 0);
	buffer[7] = I2C_read(self->_i2c, 1);
	I2C_stop(self->_i2c);


}
