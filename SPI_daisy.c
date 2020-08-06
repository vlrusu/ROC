/*
 * SPI_daisy.h
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

#include "SPI_daisy.h"
#include "utils.h"

void SPI_daisy_setup(SPI_daisy *self, MCP *mcp_scl, uint8_t sclPin, MCP *mcp_sdi, uint8_t sdiPin, MCP *mcp_sdo, uint8_t sdoPin, MCP *mcp_cs, uint8_t csPin){
	self->_mcp_scl = mcp_scl;
	self->_sclPin = sclPin;
	self->_mcp_sdi = mcp_sdi;
	self->_sdiPin = sdiPin;
	self->_mcp_sdo = mcp_sdo;
	self->_sdoPin = sdoPin;
	self->_mcp_cs = mcp_cs;
	self->_csPin = csPin;

	MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
	MCP_pinMode(self->_mcp_scl, self->_sclPin, MCP_OUTPUT);
	MCP_pinWrite(self->_mcp_sdi, self->_sdiPin, 0);
	MCP_pinMode(self->_mcp_sdi, self->_sdiPin, MCP_OUTPUT);
	MCP_pinWrite(self->_mcp_sdo, self->_sdoPin, 0);
	MCP_pinMode(self->_mcp_sdo, self->_sdoPin, MCP_INPUT);
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 1);
	MCP_pinMode(self->_mcp_cs, self->_csPin, MCP_OUTPUT);

	return;
}

void SPI_daisy_read(SPI_daisy *self, uint8_t nbyte, uint8_t *data){
	for (uint8_t ibyte = 0; ibyte < nbyte; ibyte++)
		data[ibyte] = 0;
	for (uint8_t ibyte = 0; ibyte < nbyte; ibyte++){
		for (uint8_t i = 0; i < 8; i++){
			MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
			hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
			MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
			if (MCP_pinRead(self->_mcp_sdo, self->_sdoPin)) data[ibyte] |= ((uint8_t)0x1 << (7-i)) ;
			hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
		}
	}
	return;
}//most devices transmit MSB first, bits order already flipped

void SPI_daisy_write(SPI_daisy* self, uint8_t nbyte, uint8_t *data){
	for (uint8_t ibyte = 0; ibyte < nbyte; ibyte++){
		for (uint8_t m = (uint8_t)0x80 ; m != 0; m >>= 1){
			MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
			MCP_pinWrite(self->_mcp_sdi, self->_sdiPin, data[ibyte] & m);
			hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
			MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
			hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
		}
	}
	MCP_pinWrite(self->_mcp_sdi, self->_sdiPin, 0);//reset SDI pin
	return;
}

void SPI_daisy_rw_cycle(SPI_daisy *self, uint8_t wbyte, uint8_t *wdata, uint8_t rbyte, uint8_t *rdata){
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 0);
	hwdelay(2); //Allow 100 ns setup time
	SPI_daisy_write(self, wbyte, wdata);
	SPI_daisy_read(self, rbyte, rdata);
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 1);
	return;
}
