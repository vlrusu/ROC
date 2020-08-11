/*
 * ADC124S051.h
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

#ifndef ADC124S051_H_
#define ADC124S051_H_

#include "SPI_daisy.h"
#include "drivers/CoreSPI/core_spi.h"

uint16_t ADC124S051_daisy_read(SPI_daisy *spi, uint8_t pin);
uint16_t ADC124S051_read(spi_instance_t *spi, uint8_t pin);

#endif /* ADC124S051_H_ */
