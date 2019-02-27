#include "utils.h"
#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"
#include "hw_platform.h"

uint8_t outBuffer[1000]; // buffer for printing to serial port
uint8_t rx_buff[100];
uint8_t buffer[256]; // buffer for reading from serial port
uint32_t writePtr;


UART_instance_t g_uart;
spi_instance_t g_spi[4];
pwm_instance_t g_pwm;
gpio_instance_t g_gpio;



uint16_t readU16fromBytes(uint8_t data[])
{
	union u_tag{
		uint8_t b[2];
		uint16_t usval;
	} u;
	u.b[0] = data[0];
	u.b[1] = data[1];
	return u.usval;
}

uint32_t readU32fromBytes(uint8_t data[])
{
	union u_tag{
		uint8_t b[4];
		uint32_t ulval;
	} u;
	u.b[0] = data[0];
	u.b[1] = data[1];
	u.b[2] = data[2];
	u.b[3] = data[3];
	return u.ulval;
}


void delay_ms(uint32_t us)
{
    volatile uint32_t delay_count = SystemCoreClock / 1000. * us;

    while(delay_count > 0u)
    {
        --delay_count;
    }
}

void delayUs(int us)
{
	volatile uint32_t delay_count = SystemCoreClock / 1000000. * us;

	while(delay_count > 0u)
	{
		--delay_count;
	}
}

void delayTicks(uint8_t ticks)
{
	volatile uint8_t delay_count = ticks;
	while(delay_count > 0u)
	{
		--delay_count;
	}
}

void GPIO_write(uint8_t pin, uint8_t value)
{
	uint32_t register_value;
	if (value == 0)
		register_value &= ~(0x1<<pin);
	else
		register_value |= (0x1<<pin);
	*(registers_0_addr) = register_value;

}
uint32_t GPIO_read(uint8_t pin)
{
	uint32_t value = *(registers_0_addr);
	if ((0x1<<pin) & value)
		return 1;
	else
		return 0;
}



