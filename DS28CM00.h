/*
 * DS28CM00.h
 *
 *  Created on: Dec 5, 2018
 *      Author: vrusu
 */

#ifndef DS28CM00_H_
#define DS28CM00_H_

#include "utils.h"
#include "I2C.h"

typedef struct {
	uint8_t _address;
    I2C *_i2c; // I2C object for communicating
} DS28CM00;

void DS28CM00_setup(DS28CM00 *self, I2C *i2c);
int32_t getUID(DS28CM00 *self, uint8_t* );

#endif /* DS28CM00_H_ */
