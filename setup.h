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
#include "DS28CM00.h"

#include <stdio.h>
#include <string.h>



#define CAL_CALEVEN      5
#define CAL_CALODD       6

#define LEDn             0

#define WHOAREYOU 0
#define SETPREAMPGAIN 1
#define SETPREAMPTHRESHOLD 2
#define SETCALDAC 3
#define SENDONECALPULSE 4
#define SETPULSERON 5
#define SETPULSEROFF 6
#define DUMPSETTINGS 7
#define READMONADCS 10
#define READBMES 11
#define MCPWRITEPIN 12
#define GETDEVICEID 13
#define TESTDDR 14
#define RESETROC 15

#define MAXCOM_MON 100 //100 monitoring commands ,rest reserved for future

enum MCPs{MCPCAL0, MCPCAL1, MCPCALIB, MCPCAL2, MCPCAL3, MCPHV0, MCPHV1, MCPHV2, MCPHV3};


extern const uint8_t MCPCALIBCHAN[8];


extern const uint16_t default_caldac[8]; // calibration dac setting in dac counts
extern const uint8_t calpulse_chanmap[8]; //mapping between physical premap position and calibration channels as set by the MCP chip




uint8_t pulserOn;//whether to send a calibration pulse every loop
uint32_t pulserDelay; //delay between calibration pulses
uint16_t dutyCycle;
uint8_t pulserOdd;

typedef struct {
	LTC2634 *_ltc;
	uint8_t _thresh;
	uint8_t _gain;
} Straw;

MCP preampMCP[9];


LTC2634 dacs[96];

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
