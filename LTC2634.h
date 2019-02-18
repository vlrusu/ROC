/*
 * LTC2634.h
 *
 *  Created on: Mar 8, 2018
 *      Author: rbonvent
 */

#ifndef LTC2634_H_
#define LTC2634_H_

#include "MCP23S17.h"

#define LTC2634DELAY 2

typedef struct {
	MCP *_csnMCP;
	MCP *_sclkMCP;
	MCP *_sdiMCP;
	uint8_t _csnPin;
	uint8_t _sclkPin;
	uint8_t _sdiPin;
	uint8_t _nbits;
} LTC2634;

void LTC2634_setup(LTC2634 *self, MCP *csnMCP, uint8_t csnPin, MCP *sclkMCP, uint8_t sclkPin, MCP *sdiMCP, uint8_t sdiPin);
void LTC2634_write(LTC2634 *self, uint8_t channel, uint16_t value);

#endif /* LTC2634_H_ */
