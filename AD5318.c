#include "AD5318.h"
#include "utils.h"

void AD5318_write(spi_instance_t spi, uint8_t sync, uint8_t channel, uint16_t value)
{

    uint16_t data = 0x0;
    data |= (channel << 12);
    data |= (value << 2);
    uint8_t master_tx_buffer[2];
    master_tx_buffer[0] = (uint8_t) ((data & 0xFF00) >> 8);
    master_tx_buffer[1] = (uint8_t) ((data & 0xFF));



	SPI_set_slave_select(&spi, sync);
	   SPI_transfer_block
	       (
	           &spi,
	           master_tx_buffer,
	           sizeof(master_tx_buffer),
	           0,
	           0);


	SPI_clear_slave_select(&spi, sync);



}
