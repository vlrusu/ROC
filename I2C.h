/* Arduino I2cMaster Library
 * Copyright (C) 2010 by William Greiman
 *
 * This file is part of the Arduino I2cMaster Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Arduino I2cMaster Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef I2C_H
#define I2C_H

#include "MCP23S17.h"
#include <stdio.h>
#include <string.h>


/** Delay used for software I2C */
#define I2C_DELAY_USEC 4

/** Bit to or with address for read start and read restart */
#define I2C_READ 1

/** Bit to or with address for write start and write restart */
#define I2C_WRITE 0

typedef struct{
    MCP *_mcp_scl; // SPI expander for clock line of the I2C bus
    MCP *_mcp_sda; // SPI expander for data line of the I2c bus
    uint8_t _sdaPin; // pin number on _mcp_sda for sda pin
    uint8_t _sclPin; // pin number on _mcp_scl for scl pin
} I2C;

// sets mcp and pins, and sets pins to outputs and HIGH
void I2C_setup(I2C *self, MCP *mcp_sda, uint8_t sdaPin, MCP *mcp_scl, uint8_t sclPin);

// Performs an 8 bit read using 9 total clocks of I2C bus.
// During a multi byte read, master holds data low for each ack except for last read.
// If last == 1 master will not ack and the read will be ended.
uint8_t I2C_read(I2C* self, uint8_t last);

// Sets up a transfer by first setting sda and scl low and then
// writing the I2C address of the device using 9 clocks
// addressRW == Upper 7 bits are device I2C address, lowest bit is R/W
// the slave device will hold data low for ack. return value is this ack bit, so
// a return value of 1 means an error has occured.
uint8_t I2C_start(I2C* self, uint8_t addressRW);
uint8_t I2Cs_start(MCP *mcp, uint8_t *addressRWs, int calhv); // JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.

// Stops the transfer by setting sda low, then scl high, then sda high
void I2C_stop(I2C* self);
void I2Cs_stop(MCP *mcp, int calhv); // JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.

// Performs an 8 bit write using 9 total clocks of the I2C bus.
// During a write the slave device will hold data low for ack. return value is this ack bit, so
// a return value of 1 means an error has occured.
uint8_t I2C_write(I2C* self, uint8_t data);
uint8_t I2Cs_write(MCP *mcp, uint8_t *datas, int calhv); // JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.

#endif  // I2C_MASTER_H
