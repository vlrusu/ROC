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

	self->_tx_buffer = 0;
	self->_rx_buffer = 0;

	MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
	MCP_pinMode(self->_mcp_scl, self->_sclPin, MCP_OUTPUT);
	MCP_pinWrite(self->_mcp_sdi, self->_sdiPin, 0);
	MCP_pinMode(self->_mcp_sdi, self->_sdiPin, MCP_INPUT);
	MCP_pinWrite(self->_mcp_sdo, self->_sdoPin, 0);
	MCP_pinMode(self->_mcp_sdo, self->_sdoPin, MCP_OUTPUT);
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 1);
	MCP_pinMode(self->_mcp_cs, self->_csPin, MCP_OUTPUT);
}

uint32_t SPI_daisy_read(SPI_daisy* self, uint8_t nbyte){
	self->_rx_buffer = 0;
    for (uint8_t i = 0; i < nbyte*8; i++) {
    	MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
    	hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
    	MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
    	if (MCP_pinRead(self->_mcp_sdo, self->_sdoPin)) self->_rx_buffer |= ((uint32_t)0x1 << (nbyte*8-1-i)) ;
    	hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
    }
    return self->_rx_buffer;
}//most devices transmit MSB first, bits order already flipped

void SPI_daisy_write(SPI_daisy* self, uint8_t nbyte, uint32_t data){
	self->_tx_buffer = data;
	for (uint32_t m = (((uint32_t)0x80) << (8*nbyte)); m != 0; m >>= 1) {
		MCP_pinWrite(self->_mcp_scl, self->_sclPin, 0);
		MCP_pinWrite(self->_mcp_sdi, self->_sdiPin, (self->_tx_buffer) & m);
		hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
		MCP_pinWrite(self->_mcp_scl, self->_sclPin, 1);
		hwdelay(TICKPERUS*SPI_DAISY_HALF_CLK_DELAY);
	}
}

uint32_t SPI_daisy_rw_cycle(SPI_daisy* self, uint8_t wbyte, uint32_t wdata, uint8_t rbyte){
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 0);
	hwdelay(2); //Allow 100 ns setup time
	SPI_daisy_write(self, wbyte, wdata);
	SPI_daisy_read(self, rbyte);
	MCP_pinWrite(self->_mcp_cs, self->_csPin, 1);
	return self->_rx_buffer;
}
