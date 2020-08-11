/*
 * ADC124S051.c
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

#include "ADC124S051.h"

uint16_t ADC124S051_daisy_read(SPI_daisy *spi, uint8_t pin){
	uint8_t wbuffer[2];
	uint8_t rbuffer[2] = {0};
	wbuffer[0] = pin << 3;
	wbuffer[1] = 0;
	SPI_daisy_rw_cycle(spi, 2, wbuffer, 2, rbuffer);
	uint16_t result = (uint16_t)rbuffer[0];
	result = (result << 8) & ((uint16_t)rbuffer[1]);
	return result;
}

uint16_t ADC124S051_read(spi_instance_t *spi, uint8_t pin){
	uint8_t wbuffer[2];
	uint8_t rbuffer[2] = {0};
	wbuffer[0] = pin << 3;
	wbuffer[1] = 0;
	SPI_transfer_block(spi, wbuffer, 2, rbuffer, 2);
	uint16_t result = (uint16_t)rbuffer[0];
	result = (result << 8) & ((uint16_t)rbuffer[1]);
	return result;
}
