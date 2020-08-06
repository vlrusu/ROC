/*
 * bme280_SPI_daisy.c
 *
*  Created on: Jul 31, 2020
 *      Author: yongyiwu
 *
 * Simplified version of bme280 connected on SPI daisy
 */

#include "bme280_SPI_daisy.h"
#include "utils.h"

void bme280_set_regs(SPI_daisy *spi, uint8_t len, uint8_t *reg_addr, const uint8_t *reg_data){
	uint8_t tx_buffer[8] = {0};
	for (uint8_t i = 0; i < len; i++){
		tx_buffer[2*i] = reg_addr[i] & 0x7f; //make sure MSB is 0, which is WRITE
		tx_buffer[2*i+1] = reg_data[i];
	}
	MCP_pinWrite(spi->_mcp_cs, spi->_csPin, 0);
	hwdelay(2); //Allow 100 ns setup time
	SPI_daisy_write(spi, len*2, tx_buffer);
	MCP_pinWrite(spi->_mcp_cs, spi->_csPin, 1);
	return;
}

void bme280_get_regs(SPI_daisy *spi, uint8_t starting_reg_addr, uint8_t len, uint8_t *reg_data){
	uint8_t tx_data = starting_reg_addr | 0x80; //make sure MSB is 1, which is READ
	SPI_daisy_rw_cycle(spi, 1, &tx_data, len, reg_data);
	return;
}

uint8_t bme280_init_settings(SPI_daisy *spi){//write fixed settings to registers
	uint8_t settings_addr[3] = {BME280_CONFIG_ADDR, BME280_CTRL_HUM_ADDR, BME280_CTRL_MEAS_ADDR};
	uint8_t settings_data[3] = {0};
	settings_data[0] = ((BME280_FILTER_COEFF_16) << 2); //intend to use forced mode, no t_sb required; filter; 4-pin SPI
	settings_data[1] = BME280_OVERSAMPLING_1X; //osr_h
	settings_data[2] = ((BME280_OVERSAMPLING_2X) << 5)|((BME280_OVERSAMPLING_16X) << 2)| BME280_SLEEP_MODE; //osr_t|osr_p|mode

	//write settings
	bme280_set_regs(spi, 3, settings_addr, settings_data);

	hwdelay(TICKPERUS);
	uint8_t error = 0;
	//check register values
	uint8_t rxbuffer[4] = {0};
	bme280_get_regs(spi, BME280_CTRL_HUM_ADDR, 4, rxbuffer);
	if ((rxbuffer[0] != settings_data[1])||
			(rxbuffer[2] != settings_data[2])||
			(rxbuffer[3] != settings_data[0]))
		error = 1;
	return error;
}

void bme280_get_calib(SPI_daisy *spi, uint8_t* calib_data_tp,  uint8_t* calib_data_h){
	bme280_get_regs(spi, BME280_TEMP_PRESS_CALIB_DATA_ADDR, BME280_TEMP_PRESS_CALIB_DATA_LEN, calib_data_tp);
	bme280_get_regs(spi, BME280_HUMIDITY_CALIB_DATA_ADDR, BME280_HUMIDITY_CALIB_DATA_LEN, calib_data_h);
	return;
}

void bme280_get_htp(SPI_daisy *spi, uint8_t* htp_data){
	// in forced mode, need to write forced mode again to trigger converge
	uint8_t control_meas = 0;
	uint8_t control_addr = BME280_CTRL_MEAS_ADDR;
	bme280_get_regs(spi, control_addr, 1, &control_meas);
	control_meas |= BME280_FORCED_MODE;
	bme280_set_regs(spi, 1, &control_addr, &control_meas);
	hwdelay(TICKPERUS*1000*50);//allow 46.1 ms for converging

	bme280_get_regs(spi, BME280_DATA_ADDR, BME280_P_T_H_DATA_LEN, htp_data);
	return;
}
