#ifndef UTILS_H_
#define UTILS_H_


#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CorePWM/core_pwm.h"
//#include <stdio.h>
//#include <string.h>
//#include <stdio.h>
#include "hw_platform.h"

#define IS_CAL 1

#define REGISTERBASEADDR          0x00042000UL

#define ADC_RAMP 4
#define ADC_SYNC 2
#define ADC_SKEW 1
#define ADC_REALDATA 0

#define MAXCOM_MON 256

//outBuffer CMDID, 1 byte. For all command except for  "RUN_STARTED", read next **2** bytes for buffer length
#define ADCRWCMDID 101
#define BITSLIPCMDID 102
#define AUTOBITSLIPCMDID 103
#define READRATESCMDID 104
#define READDATACMDID 105
#define STOPRUNCMDID 106
#define ADCINITINFOCMDID 107
#define RUN_STARTED 254
#define PACKAGETESTCMDID 151

//dataBuffer control character, 2 bytes
#define STARTTRG                    0xfffe//begin of trigger, read next **2** bytes to retrieve buffer length
#define STARTBUF                    0xfffd//begin of buffer for same trigger, read next **2** bytes to retrieve buffer length
#define EMPTY                       0xfffc//begin of data buffer, but it is empty
#define ENDOFDATA                   0xfffb//end of data stream

//extern const int channel_map[48];
extern int adc_map[6];

uint16_t bufcount;
int readout_obloc_place_holder;

extern char init_buff[16];
extern char dataBuffer[2000];
extern char outBuffer[2000]; // buffer for printing to serial port
extern uint8_t rx_buff[100];
extern uint8_t buffer[256]; // buffer for reading from serial port
extern uint32_t writePtr;


volatile uint32_t * registers_0_addr;

void GPIO_write(uint8_t pin, uint8_t value);
uint32_t GPIO_read(uint8_t pin);

extern int adc_phases[6];

extern uint32_t *register_base_addr;

void reset_fabric();

uint16_t readU16fromBytes(uint8_t data[]);
uint32_t readU32fromBytes(uint8_t data[]);
void delayUs(int us);
void delay_ms(uint32_t us);
void delayTicks(uint8_t ticks);
char * print_float(char *fchars, float value);

void read_data(int *delay_count, int *trigger_count);
void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit);
void get_rates(uint32_t channel_mask1, uint32_t channel_mask2, uint32_t channel_mask3, int num_delays, int num_samples);
void get_mapped_channels(uint32_t *channel_mask1, uint32_t *channel_mask2, uint32_t *channel_mask3, uint32_t *mapped_channel_mask1, uint32_t *mapped_channel_mask2);

void adc_spi(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint8_t adc_mask_r);
void adc_write(uint16_t address, uint8_t data, uint8_t adc_mask_r);
uint8_t adc_read(uint16_t address, uint8_t adc_num);
uint8_t init_adc(uint8_t adc_mask, uint8_t pattern, uint8_t phase);

void digi_write(uint8_t address, uint16_t data);
uint16_t digi_read(uint8_t address);

extern int readout_maxDelay;
extern int readout_obloc;
extern int readout_mode;
extern int readout_wordInTrigger;
extern int readout_wordsPerTrigger;
extern int readout_numTriggers;
extern int calibration_count[32];
extern uint32_t calibration_done;

extern UART_instance_t g_uart;
extern spi_instance_t g_spi[4];
extern gpio_instance_t g_gpio;
extern pwm_instance_t g_pwm;



#endif /* UTILS_H_ */

