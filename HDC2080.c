#include "HDC2080.h"

uint8_t hdc2080_open_reg(HDC2080 *self, uint8_t reg_addr){
	if (I2C_start(self->_i2c, self->_addr << 1 | I2C_WRITE) != 0) return 1;
	if (I2C_write(self->_i2c, reg_addr)!=0) return 1;
	return 0;
}

uint8_t hdc2080_get_reg(HDC2080 *self, uint8_t reg_addr, uint8_t *reg_data){
	uint8_t buf = 0xff;
	uint8_t bufr = 0;
	uint8_t i =0;
	if (hdc2080_open_reg(self, reg_addr)!=0)
		return 1;
	if (I2C_start(self->_i2c, self->_addr << 1 | I2C_READ)!=0)
		return 1;

	buf = I2C_read(self->_i2c, 1);
	for (i = 0 ; i < 8 ;i++){
		bufr |= ((buf & (1<<i)) >> i ) << (7-i);
	}
	*reg_data = bufr;

	I2C_stop(self->_i2c);
	return 0;
}

uint8_t hdc2080_set_reg(HDC2080 *self, uint8_t reg_addr, uint8_t reg_data){
	if (hdc2080_open_reg(self, reg_addr)!=0)
		return 1;
	if (I2C_write(self->_i2c, reg_data)!=0)
		return 1;
	I2C_stop(self->_i2c);
	return 0;
}

uint8_t hdc2080_setup(HDC2080 *self, uint8_t addr, I2C *i2c){
	self->_i2c = i2c;
	self->_addr = addr;
	uint8_t dev_id_l = 0;
	uint8_t dev_id_h = 0;

	hdc2080_reset(self);
	delay_ms(10);
	if (hdc2080_get_reg(self, DEVICE_ID_L, &dev_id_l)!=0) return 1;
	if (hdc2080_get_reg(self, DEVICE_ID_H, &dev_id_h)!=0) return 1;

	if ((dev_id_l == DEVICE_ID_L_VAL) && (dev_id_h == DEVICE_ID_H_VAL))
		return 0;
	else
		return 1;
}

uint8_t hdc2080_read_temp(HDC2080 *self, uint16_t *result){
	uint16_t temp = 0;
	uint8_t buf;

	if (hdc2080_get_reg(self, TEMP_LOW, &buf)!=0) return 1;
	temp = (uint16_t)buf & 0x00FF;
	if (hdc2080_get_reg(self, TEMP_HIGH, &buf)!=0) return 1;
	temp |= ((uint16_t)buf << 8) & 0xFF00;

	*result = temp;

	return 0;
}

uint8_t hdc2080_read_humidity(HDC2080 *self, uint16_t *result){
	uint16_t humidity = 0;
	uint8_t buf;

	if (hdc2080_get_reg(self, HUMID_LOW, &buf)!=0) return 1;
	humidity = (uint16_t)buf & 0x00FF;
	if (hdc2080_get_reg(self, HUMID_HIGH, &buf)!=0) return 1;
	humidity |= ((uint16_t)buf << 8) & 0xFF00;

	*result = humidity;

	return 0;
}

/*
uint8_t hdc2080_enable_heater(HDC2080 *self){
	uint8_t configContents;	//Stores current contents of config register
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	//set bit 3 to 1 to enable heater
	configContents = (configContents | 0x08);

	return (hdc2080_set_reg(self, CONFIG, configContents));
}

uint8_t hdc2080_disable_heater(HDC2080 *self){
	uint8_t configContents;	//Stores current contents of config register
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	//set bit 3 to 0 to disable heater (all other bits 1)
	configContents = (configContents & 0xF7);

	return (hdc2080_set_reg(self, CONFIG, configContents));
}

uint8_t hdc2080_set_low_threshold_temp(HDC2080 *self, uint8_t temp){
	return (hdc2080_set_reg(self, TEMP_THR_L, temp));
}

uint8_t hdc2080_set_high_threshold_temp(HDC2080 *self, uint8_t temp){
	return (hdc2080_set_reg(self, TEMP_THR_H, temp));
}

uint8_t hdc2080_set_low_threshold_humidity(HDC2080 *self, uint8_t humid){
	return (hdc2080_set_reg(self, HUMID_THR_L, humid));
}

uint8_t hdc2080_set_high_threshold_humidity(HDC2080 *self, uint8_t humid){
	return (hdc2080_set_reg(self, HUMID_THR_H, humid));
}

uint8_t hdc2080_read_low_threshold_temp(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, TEMP_THR_L, rslt));
}

uint8_t hdc2080_read_high_threshold_temp(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, TEMP_THR_H, rslt));
}

uint8_t hdc2080_read_low_threshold_humidity(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, HUMID_THR_L, rslt));
}

uint8_t hdc2080_read_high_threshold_humidity(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, HUMID_THR_H, rslt));
}
*/
uint8_t hdc2080_trigger_measurement(HDC2080 *self){
	uint8_t configContents;
	if (hdc2080_get_reg(self, MEASUREMENT_CONFIG, &configContents)!=0) return 1;

	//set last bit to 1 to trigger a measurement
	configContents = (configContents | 0x01);

	uint8_t rslt = hdc2080_set_reg(self, MEASUREMENT_CONFIG, configContents);
	delay_ms(2); // allow conversion time
	return rslt;
}

uint8_t hdc2080_reset(HDC2080 *self){ //delay after resets!
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	//set bit 7 of the CONFIG register to trigger a soft reset
	configContents = (configContents | 0x80);

	return (hdc2080_set_reg(self, CONFIG, configContents));
}
/*
uint8_t hdc2080_enable_interrupt(HDC2080 *self){
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	//set bit 2 of the CONFIG register to enable/disable the interrupt pin
	configContents = (configContents | 0x04);

	return (hdc2080_set_reg(self, CONFIG, configContents));
}

uint8_t hdc2080_disable_interrupt(HDC2080 *self){
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	//set bit 2 of the CONFIG register to enable/disable the interrupt pin
	configContents = (configContents & 0xFB);

	return (hdc2080_set_reg(self, CONFIG, configContents));
}

uint8_t hdc2080_read_interrupt_status(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, INTERRUPT_DRDY, rslt));
}

uint8_t hdc2080_clear_max_temp(HDC2080 *self){
	return (hdc2080_set_reg(self, TEMP_MAX, 0x00));
}

uint8_t hdc2080_clear_max_humidity(HDC2080 *self){
	return (hdc2080_set_reg(self, HUMID_MAX, 0x00));
}

uint8_t hdc2080_read_max_temp(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, TEMP_MAX, rslt));
}

uint8_t hdc2080_read_max_humidity(HDC2080 *self, uint8_t *rslt){
	return (hdc2080_get_reg(self, HUMID_MAX, rslt));
}

uint8_t hdc2080_enable_threshold_interrupt(HDC2080 *self){
	uint8_t regContents;
	if (hdc2080_get_reg(self, INTERRUPT_CONFIG, &regContents)!=0) return 1;

	//Enables the interrupt pin for comfort zone operation
	regContents = (regContents | 0x78);

	return (hdc2080_set_reg(self, INTERRUPT_CONFIG, regContents));
}

uint8_t hdc2080_disable_threshold_interrupt(HDC2080 *self){
	uint8_t regContents;
	if (hdc2080_get_reg(self, INTERRUPT_CONFIG, &regContents)!=0) return 1;

	//Disables the interrupt pin for comfort zone operation
	regContents = (regContents & 0x87);

	return (hdc2080_set_reg(self, INTERRUPT_CONFIG, regContents));
}

uint8_t hdc2080_enable_DRDY_interrupt(HDC2080 *self){
	uint8_t regContents;
	if (hdc2080_get_reg(self, INTERRUPT_CONFIG, &regContents)!=0) return 1;

	// enables the interrupt pin for DRDY operation
	regContents = (regContents | 0x80);

	return (hdc2080_set_reg(self, INTERRUPT_CONFIG, regContents));
}

uint8_t hdc2080_disable_DRDY_interrupt(HDC2080 *self){
	uint8_t regContents;
	if (hdc2080_get_reg(self, INTERRUPT_CONFIG, &regContents)!=0) return 1;

	// disables the interrupt pin for DRDY operation
	regContents = (regContents & 0x7F);

	return (hdc2080_set_reg(self, INTERRUPT_CONFIG, regContents));
}
*/
/* Upper two bits of the MEASUREMENT_CONFIG register controls
   the temperature resolution*/
/*
uint8_t hdc2080_set_temp_res(HDC2080 *self, uint8_t resolution){
	uint8_t configContents;
	if (hdc2080_get_reg(self, MEASUREMENT_CONFIG, &configContents)!=0) return 1;

	switch(resolution)
	{
		case FOURTEEN_BIT:
			configContents = (configContents & 0x3F);
			break;
//		case ELEVEN_BIT:
//			configContents = (configContents & 0x7F);
//			configContents = (configContents | 0x40);
//			break;
//		case NINE_BIT:
//			configContents = (configContents & 0xBF);
//			configContents = (configContents | 0x80);
//			break;
		default:
			configContents = (configContents & 0x3F);
	}

	return (hdc2080_set_reg(self, MEASUREMENT_CONFIG, configContents));
}
*/
/*  Bits 5 and 6 of the MEASUREMENT_CONFIG register controls
    the humidity resolution*/
/*
uint8_t hdc2080_set_humid_res(HDC2080 *self, uint8_t resolution){
	uint8_t configContents;
	if (hdc2080_get_reg(self, MEASUREMENT_CONFIG, &configContents)!=0) return 1;

	switch(resolution)
	{
		case FOURTEEN_BIT:
			configContents = (configContents & 0xCF);
			break;
//		case ELEVEN_BIT:
//			configContents = (configContents & 0xDF);
//			configContents = (configContents | 0x10);
//			break;
//		case NINE_BIT:
//			configContents = (configContents & 0xEF);
//			configContents = (configContents | 0x20);
//			break;
		default:
			configContents = (configContents & 0xCF);
	}

	return (hdc2080_set_reg(self, MEASUREMENT_CONFIG, configContents));
}
*/
/*  Bits 2 and 1 of the MEASUREMENT_CONFIG register controls
    the measurement mode  */
/*
uint8_t hdc2080_set_measurement_mode(HDC2080 *self, uint8_t mode){
	uint8_t configContents;
	if (hdc2080_get_reg(self, MEASUREMENT_CONFIG, &configContents)!=0) return 1;

	switch(mode)
	{
		case TEMP_AND_HUMID:
			configContents = (configContents & 0xF9);
			break;
//		case TEMP_ONLY:
//			configContents = (configContents & 0xFC);
//			configContents = (configContents | 0x02);
//			break;
//		case HUMID_ONLY:
//			configContents = (configContents & 0xFD);
//			configContents = (configContents | 0x04);
//			break;
		default:
			configContents = (configContents & 0xF9);
	}

	return (hdc2080_set_reg(self, MEASUREMENT_CONFIG, configContents));
}
*/
/*  Bits 6-4  of the CONFIG register controls the measurement
    rate  */
/*
uint8_t hdc2080_set_rate(HDC2080 *self, uint8_t rate){
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	switch(rate)
	{
		case MANUAL:
			configContents = (configContents & 0x8F);
			break;
//		case TWO_MINS:
//			configContents = (configContents & 0x9F);
//			configContents = (configContents | 0x10);
//			break;
//		case ONE_MINS:
//			configContents = (configContents & 0xAF);
//			configContents = (configContents | 0x20);
//			break;
//		case TEN_SECONDS:
//			configContents = (configContents & 0xBF);
//			configContents = (configContents | 0x30);
//			break;
//		case FIVE_SECONDS:
//			configContents = (configContents & 0xCF);
//			configContents = (configContents | 0x40);
//			break;
//		case ONE_HZ:
//			configContents = (configContents & 0xDF);
//			configContents = (configContents | 0x50);
//			break;
//		case TWO_HZ:
//			configContents = (configContents & 0xEF);
//			configContents = (configContents | 0x60);
//			break;
//		case FIVE_HZ:
//			configContents = (configContents | 0x70);
//			break;
		default:
			configContents = (configContents & 0x8F);
	}

	return (hdc2080_set_reg(self, CONFIG, configContents));
}
*/
/*  Bit 1 of the CONFIG register can be used to control the
    the interrupt pins polarity */
/*
uint8_t hdc2080_set_interrupt_polarity(HDC2080 *self, uint8_t polarity){
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	switch(polarity)
	{
		case ACTIVE_LOW:
			configContents = (configContents & 0xFD);
			break;
		case ACTIVE_HIGH:
			configContents = (configContents | 0x02);
			break;
		default:
			configContents = (configContents & 0xFD);
	}

	return (hdc2080_set_reg(self, CONFIG, configContents));
}
*/
/*  Bit 0 of the CONFIG register can be used to control the
    the interrupt pin's mode */
/*
uint8_t hdc2080_set_interrupt_mode(HDC2080 *self, uint8_t mode){
	uint8_t configContents;
	if (hdc2080_get_reg(self, CONFIG, &configContents)!=0) return 1;

	switch(mode)
	{
		case LEVEL_MODE:
			configContents = (configContents & 0xFE);
			break;
		case COMPARATOR_MODE:
			configContents = (configContents | 0x01);
			break;
		default:
			configContents = (configContents & 0xFE);
	}

	return (hdc2080_set_reg(self, CONFIG, configContents));
}
*/
