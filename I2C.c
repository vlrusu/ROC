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

#include "I2C.h"
#include "utils.h"

void I2C_setup(I2C *self, MCP *mcp_sda, uint8_t sdaPin, MCP *mcp_scl, uint8_t sclPin)
{
    self->_mcp_sda = mcp_sda;
    self->_sdaPin = sdaPin;
    self->_mcp_scl = mcp_scl;
    self->_sclPin = sclPin;
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    MCP_pinMode(self->_mcp_scl, self->_sclPin, MCP_OUTPUT);
}

uint8_t I2C_read(I2C *self, uint8_t last)
{
    uint8_t b = 0;
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_INPUT);
    // read byte
    uint8_t i;
    for (i = 0; i < 8; i++) {
 //       b <<= 1;
        delayUs(I2C_DELAY_USEC);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
	    if (MCP_pinRead(self->_mcp_sda, self->_sdaPin)) b |= (1 << i) ;
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    }
    // send Ack or Nak
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, last);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    delayUs(I2C_DELAY_USEC);

    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    return b;
}

uint8_t I2C_start(I2C *self, uint8_t addressRW) {
	MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
	MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
	delayUs(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    delayUs(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    return I2C_write(self, addressRW);
}

// JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.
uint8_t I2Cs_start(MCP *mcp, uint8_t *addressRWs, int calhv) {
    int datareg = GPIOA;
    int clkreg = GPIOB;
    if (calhv == 0){
        datareg = GPIOB;
        clkreg = GPIOA;
    }
    MCP_byteWrite(mcp, datareg, 0x0);
    delayUs(I2C_DELAY_USEC);
    MCP_byteWrite(mcp, clkreg, 0x0);
    return I2Cs_write(mcp, addressRWs, calhv);
}

void I2C_stop(I2C *self) {
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    delayUs(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    delayUs(I2C_DELAY_USEC);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 1);
    delayUs(I2C_DELAY_USEC);
}

// JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.
void I2Cs_stop(MCP *mcp, int calhv) {
    int datareg = GPIOA;
    int clkreg = GPIOB;
    if (calhv == 0){
        datareg = GPIOB;
        clkreg = GPIOA;
    }
    MCP_byteWrite(mcp, datareg, 0);
    delayUs(I2C_DELAY_USEC);
    MCP_byteWrite(mcp, clkreg, 0xFF);
    delayUs(I2C_DELAY_USEC);
    MCP_byteWrite(mcp, datareg, 0xFF);
    delayUs(I2C_DELAY_USEC);
}

uint8_t I2C_write(I2C *self, uint8_t data) {
    // write byte
    uint8_t m;
    for (m = 0X80; m != 0; m >>= 1) {
        MCP_pinWrite(self->_mcp_sda, self->_sdaPin, m & data);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
        delayUs(I2C_DELAY_USEC);
        MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    }

    // get Ack or Nak
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_INPUT);
    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    uint8_t rtn = MCP_pinRead(self->_mcp_sda, self->_sdaPin);

    MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    MCP_pinMode(self->_mcp_sda, self->_sdaPin, MCP_OUTPUT);
    MCP_pinWrite(self->_mcp_sda, self->_sdaPin, 0);
    return 0;
}

// JFNOTE: Do not know how to adapt this to the Slow-Amps, they do not live on the cal side.
uint8_t I2Cs_write(MCP *mcp, uint8_t *datas, int calhv) {
    int datareg = GPIOA;
    int datamode = IODIRA;
    int clkreg = GPIOB;
    if (calhv == 0){
        datareg = GPIOB;
        datamode = IODIRB;
        clkreg = GPIOA;
    }
    // write byte
    int i;
    uint8_t m;
    for (m = 0x80; m != 0; m >>= 1) {
        uint8_t data = 0x0;
        for (i=0;i<8;i++){
            data |= ((datas[i] & m) != 0x0 ? 0x1 : 0x0) << i;
        }
        MCP_byteWrite(mcp,datareg, data);
        MCP_byteWrite(mcp, clkreg, 0xFF);
        delayUs(I2C_DELAY_USEC);
        MCP_byteWrite(mcp, clkreg, 0x0);
    }

    // get Ack or Nak
    MCP_byteWrite(mcp, datamode, 0xFF); // input
    MCP_byteWrite(mcp, clkreg, 0xFF);
    uint8_t rtn = MCP_byteRead(mcp, datareg);

    MCP_byteWrite(mcp, clkreg, 0x0);
    MCP_byteWrite(mcp, datamode, 0x0); // output
    MCP_byteWrite(mcp, datareg, 0x0);
    return rtn;
}
