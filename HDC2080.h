#ifndef HDC2080_H
#define HDC2080_h

#include "utils.h"
#include "I2C.h"

//Unused constants are commented out to reduce space

//Define Register Map
#define TEMP_LOW 0x00
#define TEMP_HIGH 0x01
#define HUMID_LOW 0x02
#define HUMID_HIGH 0x03
//#define INTERRUPT_DRDY 0x04
//#define TEMP_MAX 0x05
//#define HUMID_MAX 0x06
//#define INTERRUPT_CONFIG 0x07
//#define TEMP_OFFSET_ADJUST 0x08
//#define HUM_OFFSET_ADJUST 0x09
//#define TEMP_THR_L 0x0A
//#define TEMP_THR_H 0x0B
//#define HUMID_THR_L 0x0C
//#define HUMID_THR_H 0x0D
#define CONFIG 0x0E
#define MEASUREMENT_CONFIG 0x0F
//#define MID_L 0xFC
//#define MID_H 0xFD
#define DEVICE_ID_L 0xFE
#define DEVICE_ID_H 0xFF

#define DEVICE_ID_L_VAL 0xD0
#define DEVICE_ID_H_VAL 0x07

//  Constants for setting measurement resolution
//  Address 0x0F: 7:6 TEMP 5:4 HUM
//#define FOURTEEN_BIT 0
//#define ELEVEN_BIT 1
//#define NINE_BIT  2

//  Constants for setting interrupt polarity
//  Address 0x0F: 2:1
//#define TEMP_AND_HUMID 0
//#define TEMP_ONLY	   1
//#define HUMID_ONLY	   2

//  Constants for setting interrupt mode
//  Address 0x0E: 1
//#define ACTIVE_LOW	   0
//#define ACTIVE_HIGH	   1

//  Constants for setting sensor mode
//  Address 0x0E: 0
//#define LEVEL_MODE		0
//#define COMPARATOR_MODE 1

//  Constants for setting sample rate
//  Address 0x0E: 6:4
//#define MANUAL			0
//#define TWO_MINS		1
//#define ONE_MINS		2
//#define TEN_SECONDS		3
//#define	FIVE_SECONDS	4
//#define ONE_HZ			5
//#define TWO_HZ			6
//#define FIVE_HZ			7

#define HDC2080_I2C_ADDR_PRIM 40
//#define HDC2080_I2C_ADDR_SEC  41

typedef struct {
	uint8_t _addr;
    I2C *_i2c; // I2C object for communicating
} HDC2080;

uint8_t hdc2080_open_reg(HDC2080 *self, uint8_t reg_addr);// Points to a given register, return 1 means comm failure
uint8_t hdc2080_get_reg(HDC2080 *self, uint8_t reg_addr, uint8_t *reg_data);// Reads a given register, returns 1 byte
uint8_t hdc2080_set_reg(HDC2080 *self, uint8_t reg_addr, uint8_t reg_data);// Writes a byte of data to one register
uint8_t hdc2080_setup(HDC2080 *self, uint8_t addr, I2C *i2c);// Initialize the HDC2080 device and I2C

uint8_t hdc2080_read_temp(HDC2080 *self, uint16_t *result);// Returns the temperature
uint8_t hdc2080_read_humidity(HDC2080 *self, uint16_t *result);// Returns the relative humidity
//uint8_t hdc2080_enable_heater(HDC2080 *self);// Enables the heating element
//uint8_t hdc2080_disable_heater(HDC2080 *self);// Disables the heating element
//uint8_t hdc2080_set_low_threshold_temp(HDC2080 *self, uint8_t temp);// Sets low threshold temperature
//uint8_t hdc2080_set_high_threshold_temp(HDC2080 *self, uint8_t temp);// Sets high threshold temperature
//uint8_t hdc2080_set_low_threshold_humidity(HDC2080 *self, uint8_t humid);// Sets low Humidity threshold
//uint8_t hdc2080_set_high_threshold_humidity(HDC2080 *self, uint8_t humid);// Sets high Humiditiy threshold
//uint8_t hdc2080_read_low_threshold_temp(HDC2080 *self, uint8_t *rslt);// Returns contents of low temperature threshold register
//uint8_t hdc2080_read_high_threshold_temp(HDC2080 *self, uint8_t *rslt);// Returns contents of high temperature threshold register
//uint8_t hdc2080_read_low_threshold_humidity(HDC2080 *self, uint8_t *rslt);// Returns contents of low humidity threshold register
//uint8_t hdc2080_read_high_threshold_humidity(HDC2080 *self, uint8_t *rslt);// Returns contents of high humidity threshold register
uint8_t hdc2080_trigger_measurement(HDC2080 *self);// Triggers a manual temperature/humidity reading
uint8_t hdc2080_reset(HDC2080 *self);// Triggers a software reset
//uint8_t hdc2080_enable_interrupt(HDC2080 *self);// Enables the interrupt/DRDY pin
//uint8_t hdc2080_disable_interrupt(HDC2080 *self);// Disables the interrupt/DRDY pin (High Z)
//uint8_t hdc2080_read_interrupt_status(HDC2080 *self, uint8_t *rslt);// Reads the status of the interrupt register
//uint8_t hdc2080_clear_max_temp(HDC2080 *self);// Clears the Maximum temperature register
//uint8_t hdc2080_clear_max_humidity(HDC2080 *self);// Clears the Maximum humidity register
//uint8_t hdc2080_read_max_temp(HDC2080 *self, uint8_t *rslt);// Reads the maximum temperature register
//uint8_t hdc2080_read_max_humidity(HDC2080 *self, uint8_t *rslt);// Reads the maximum humidity register
//uint8_t hdc2080_enable_threshold_interrupt(HDC2080 *self);// Enables high and low temperature/humidity interrupts
//uint8_t hdc2080_disable_threshold_interrupt(HDC2080 *self);// Disables high and low temperature/humidity interrupts
//uint8_t hdc2080_enable_DRDY_interrupt(HDC2080 *self);// Enables data ready interrupt
//uint8_t hdc2080_disable_DRDY_interrupt(HDC2080 *self);// Disables data ready interrupt

/* Sets Temperature & Humidity Resolution, 3 options
   0 - 14 bit
   1 - 11 bit
   2 - 9 bit
   default - 14 bit							*/
//uint8_t hdc2080_set_temp_res(HDC2080 *self, uint8_t resolution);
//uint8_t hdc2080_set_humid_res(HDC2080 *self, uint8_t resolution);

/* Sets measurement mode, 3 options
   0 - Temperature and Humidity
   1 - Temperature only
   2 - Humidity only
   default - Temperature & Humidity			*/
//uint8_t hdc2080_set_measurement_mode(HDC2080 *self, uint8_t mode);

/* Sets reading rate, 8 options
   0 - Manual
   1 - reading every 2 minutes
   2 - reading every minute
   3 - reading every ten seconds
   4 - reading every 5 seconds
   5 - reading every second
   6 - reading at 2Hz
   7 - reading at 5Hz
   default - Manual		*/
//uint8_t hdc2080_set_rate(HDC2080 *self, uint8_t rate);

/* Sets Interrupt polarity, 2 options
   0 - Active Low
   1 - Active High
   default - Active Low			*/
//uint8_t hdc2080_set_interrupt_polarity(HDC2080 *self, uint8_t polarity);

/* Sets Interrupt mode, 2 options
   0 - Level sensitive
   1 - Comparator mode
   default - Level sensitive	*/
//uint8_t hdc2080_set_interrupt_mode(HDC2080 *self, uint8_t polarity);

#endif
