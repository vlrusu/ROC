/*
 * ADC124S051.c
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

#include "ADC124S051.h"

uint16_t ADC124S051_read(SPI_daisy *spi, uint8_t pin){
	uint32_t pin_mask = ((uint32_t)pin) << (3+8);
	return (uint16_t)((SPI_daisy_rw_cycle(spi, 2, pin_mask, 2)) & 0x0fff);
}
