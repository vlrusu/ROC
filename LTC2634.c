/*
 * LTC2634.c
 *
 *  Created on: Mar 8, 2018
 *      Author: rbonvent
 */

#include "LTC2634.h"
#include "utils.h"

void LTC2634_setup(LTC2634 *self, MCP *csnMCP, uint8_t csnPin, MCP *sclkMCP, uint8_t sclkPin, MCP *sdiMCP, uint8_t sdiPin)
{
	self->_csnPin = csnPin;
	self->_sclkPin = sclkPin;
	self->_sdiPin = sdiPin;
	self->_csnMCP = csnMCP;
	self->_sclkMCP = sclkMCP;
	self->_sdiMCP = sdiMCP;


    MCP_pinWrite(self->_csnMCP, self->_csnPin, 1);
    MCP_pinMode(self->_csnMCP, self->_csnPin, MCP_OUTPUT);
    MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 0);
    MCP_pinMode(self->_sclkMCP, self->_sclkPin, MCP_OUTPUT);
    MCP_pinWrite(self->_sdiMCP, self->_sdiPin, 0);
    MCP_pinMode(self->_sdiMCP, self->_sdiPin, MCP_OUTPUT);

	uint32_t dataWord = 0x7F0000;

	MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 0);
	MCP_pinWrite(self->_csnMCP, self->_csnPin, 0);
	delayTicks(LTC2634DELAY);
	for (int i=23;i>=0;i--){
		uint8_t thisbit;
		if ((0x1<<i) & dataWord)
			thisbit = 1;
		else
			thisbit = 0;

		MCP_pinWrite(self->_sdiMCP, self->_sdiPin, thisbit);
		delayTicks(LTC2634DELAY);
		MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 1);
		delayTicks(LTC2634DELAY);
		MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 0);
		delayTicks(LTC2634DELAY);
	}
	MCP_pinWrite(self->_csnMCP, self->_csnPin, 1);
	delayTicks(LTC2634DELAY);

}

void LTC2634_write(LTC2634 *self, uint8_t channel, uint16_t value)
{
	uint32_t command = 0x3;
	uint32_t dataWord = 0;


	dataWord = (command<<20) | ((uint32_t) (channel&0xF) << 16) | ((uint32_t) value << 6);

	MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 0);
	MCP_pinWrite(self->_csnMCP, self->_csnPin, 0);
	delayTicks(LTC2634DELAY);
	for (int i=23;i>=0;i--){
		uint8_t thisbit;
		if ((0x1<<i) & dataWord)
			thisbit = 1;
		else
			thisbit = 0;

		MCP_pinWrite(self->_sdiMCP, self->_sdiPin, thisbit);
		delayTicks(LTC2634DELAY);
		MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 1);
		delayTicks(LTC2634DELAY);
		MCP_pinWrite(self->_sclkMCP, self->_sclkPin, 0);
		delayTicks(LTC2634DELAY);
	}
	MCP_pinWrite(self->_csnMCP, self->_csnPin, 1);
	delayTicks(LTC2634DELAY);
}
