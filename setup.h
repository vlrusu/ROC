/*
 * setup.h
 *
 *  Created on: Jul 28, 2016
 *      Author: rbonvent
 */

#ifndef SETUP_H_
#define SETUP_H_

#include "MCP23S17.h"
#include "MCP3427.h"
#include "I2C.h"
#include "AD5318.h"
#include "LTC2634.h"
#include "BME280.h"
#include "bme280_defs.h"
#include "HDC2080.h"
#include "DS28CM00.h"

enum MCPs{MCPCAL0, MCPCAL1, MCPCALIB, MCPCAL2, MCPCAL3, MCPHV0, MCPHV1, MCPHV2, MCPHV3, MCPFC0, MCPFC1, MCPFC2};

extern const uint8_t MCPCALIBCHAN[8];
extern const uint8_t default_delay; // delay between calibration pulses in us (add 20 us for pulse time)
extern const uint16_t default_caldac[8]; // calibration dac setting in dac counts
extern const uint8_t calpulse_chanmap[8]; //mapping between physical premap position and calibration channels as set by the MCP chip

//extern const int channel_map[20];

int current_gains[16];
int current_thresholds[16];

uint8_t pulserOn;//whether to send a calibration pulse every loop
uint32_t pulserDelay; //delay between calibration pulses
uint16_t dutyCycle;
uint8_t pulserOdd;

typedef struct {
	LTC2634 *_ltc;
	uint8_t _thresh;
	uint8_t _gain;
} Straw;

MCP preampMCP[12];

LTC2634 dacs[96];
LTC2634 caldac0;
LTC2634 caldac1;

Straw strawsCal[96];
Straw strawsHV[96];



I2C I2CserialCal;
I2C I2CserialHV;

I2C i2c_ptscal[2];
I2C i2c_ptshv[2];

I2C     i2c_slowamps[48]; // "I2C" objects, they internally hold 2 MCP23S17 objects.
MCP3427 slowamps[48];     // MCP3427 objects, one per I2C object.

DS28CM00 DS28;

#endif /* SETUP_H_ */
