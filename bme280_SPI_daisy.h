/*
 * bme280_SPI_daisy.h
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 *
 * Simplified version of bme280 connected on SPI daisy
 */

#ifndef BME280_SPI_DAISY_H_
#define BME280_SPI_DAISY_H_

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

#include "bme280_defs.h"
#include "SPI_daisy.h"

void bme280_set_regs(SPI_daisy *spi, uint8_t len, uint8_t *reg_addr, const uint8_t *reg_data);//limited by buffer size, maximally setting 4 regs at a time
void bme280_get_regs(SPI_daisy *spi, uint8_t starting_reg_addr, uint8_t len, uint8_t *reg_data);

uint8_t bme280_init_settings(SPI_daisy *spi);//write fixed settings to registers
void bme280_get_calib(SPI_daisy *spi, uint8_t* calib_data_tp,  uint8_t* calib_data_h);
void bme280_get_htp(SPI_daisy *spi, uint8_t* htp_data);

#ifdef __cplusplus
}
#endif /* End of CPP guard */
#endif /* BME280_H_ */
/** @}*/
