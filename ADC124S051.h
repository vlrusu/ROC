/*
 * ADC124S051.h
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

#ifndef ADC124S051_H_
#define ADC124S051_H_

#include "SPI_daisy.h"

uint16_t ADC124S051_read(SPI_daisy *spi, uint8_t pin);

#endif /* ADC124S051_H_ */
