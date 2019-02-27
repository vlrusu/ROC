#ifndef UTILS_H_
#define UTILS_H_


#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CorePWM/core_pwm.h"

#include "hw_platform.h"


extern uint8_t outBuffer[1000]; // buffer for printing to serial port
extern uint8_t rx_buff[100];
extern uint8_t buffer[256]; // buffer for reading from serial port
extern uint32_t writePtr;

extern UART_instance_t g_uart;
extern spi_instance_t g_spi[4];
extern gpio_instance_t g_gpio;
extern pwm_instance_t g_pwm;

volatile uint32_t * registers_0_addr;

void GPIO_write(uint8_t pin, uint8_t value);
uint32_t GPIO_read(uint8_t pin);

uint16_t readU16fromBytes(uint8_t data[]);
uint32_t readU32fromBytes(uint8_t data[]);
void delayUs(int us);
void delay_ms(uint32_t us);
void delayTicks(uint8_t ticks);



#endif /* UTILS_H_ */

