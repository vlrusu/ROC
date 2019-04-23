#ifndef UTILS_H_
#define UTILS_H_


#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CorePWM/core_pwm.h"
#include "hw_platform.h"

//******************************************************************************
//                             Parameters
//******************************************************************************
#define ADC_RAMP 4
#define ADC_SYNC 2
#define ADC_SKEW 1
#define ADC_REALDATA 0

#define MAXCOM_MON 256 //256 monitoring commands ,rest reserved for future

#define CAL_CALEVEN      5
#define CAL_CALODD       6

#define LEDn             0

#define NUMTDCWORDS      4

//******************************************************************************
//                             Command ID
//******************************************************************************
//For all command except for  "RUN_STARTED", read next **2** bytes for buffer length

//ROC command ID (1 byte)

#define WHOAREYOU 0
#define SETPREAMPGAIN 1
#define SETPREAMPTHRESHOLD 2
#define SETCALDAC 3
#define SENDONECALPULSE 4
#define SETPULSERON 5
#define SETPULSEROFF 6
#define DUMPSETTINGS 7
#define READHISTO 8
#define READMONADCS 10
#define READBMES 11
#define MCPWRITEPIN 12
#define GETDEVICEID 13
#define TESTDDR 14
#define RESETROC 15
//#define TOGGLECALHV 16
#define DIGIRW 16

//"digi" command ID (1 byte)

#define ADCRWCMDID 101
#define BITSLIPCMDID 102
#define AUTOBITSLIPCMDID 103
#define READRATESCMDID 104
#define READDATACMDID 105
#define STOPRUNCMDID 106
#define ADCINITINFOCMDID 107

#define PACKAGETESTCMDID 151

//******************************************************************************
//                             Registers & Addresses
//******************************************************************************

//***********************DG_ADDR_* are the DIGI addresses
#define DG_ADDR_SAMPLE 0x03
#define DG_ADDR_LOOKBACK 0x04

#define DG_ADDR_SPI_SDIO 0x06
#define DG_ADDR_SPI_SDIO_DIR 0x07
#define DG_ADDR_SPI_CS 0x08
#define DG_ADDR_SPI_SCLK 0x09

#define DG_ADDR_TRIGGER_MODE 0x0A
#define DG_ADDR_MASK1 0x0B
#define DG_ADDR_ENABLE_PULSER 0x0C
#define DG_ADDR_MASK3 0x0D
#define DG_ADDR_MASK2 0x0E

#define DG_ADDR_RESET 0x10

#define DG_ADDR_READCHANNEL 0x15
#define DG_ADDR_GT0 0x16
#define DG_ADDR_GT1 0x17
#define DG_ADDR_GT2 0x18
#define DG_ADDR_GT3 0x19
#define DG_ADDR_HV 0x1A
#define DG_ADDR_CAL 0x1B
#define DG_ADDR_COINC 0x1C
#define DG_ADDR_LATCH 0x1D

#define DG_ADDR_BITSLIP0 0x30
#define DG_ADDR_BITSLIP1 0x31
#define DG_ADDR_BITSLIP2 0x32
#define DG_ADDR_BITSLIP3 0x33
#define DG_ADDR_BITSLIP4 0x34
#define DG_ADDR_BITSLIP5 0x35

//***********************REG_ROC_* are the ROC registers
#define REG_ROC_RE 0x11

//#define REG_ROC_DDR_NHITS 0x20
//#define REG_ROC_DDR_OFFSET 0x21
//#define REG_ROC_DDR_CS 0x22
//#define REG_ROC_DDR_WEN 0x23
//#define REG_ROC_DDR_REN 0x24
//#define REG_ROC_DDR_DMAEN 0x25
//#define REG_ROC_DDR_ERR 0x26
//#define REG_ROC_DDR_PATTERN 0x27
//#define REG_ROC_DDR_RADDR 0x28
//#define REG_ROC_DDR_DATA 0x29

#define REG_ROC_FIFO_RE 0x40
#define REG_ROC_FIFO_DATA 0x41
#define REG_ROC_FIFO_FULL 0x42
#define REG_ROC_FIFO_EMPTY 0x43
#define REG_ROC_FIFO_RESET 0x44
#define REG_ROC_FIFO_RDCNT 0x45
#define REG_ROC_FIFO_HOWMANY 0x46

#define REG_ROC_CAL_BUSY_P 0x61
#define REG_ROC_CAL_DATA_P 0x63
#define REG_ROC_CAL_ADDRESS_P 0x62
#define REG_ROC_CAL_INIT_P 0x60

#define REG_ROC_HV_BUSY_P 0x71
#define REG_ROC_HV_DATA_P 0x73
#define REG_ROC_HV_ADDRESS_P 0x72
#define REG_ROC_HV_INIT_P 0x70

//***********************ADC_ADDR_* are the ADC addresses
										//AD9212 memory map:
#define ADC_ADDR_CONFIG 0x00			// x00: Config: 0 | LSBFIRST | SOFTRESET | 1 | 1 | SOFTRESET | LSBFIRST | 0
#define ADC_ADDR_CID1 0x01				// x01: Chip ID  0x08
//#define ADC_ADDR_CID2 0x02				// x02: Chip ID2 0x00 or 0x10
//#define ADC_ADDR_CHAN 0x04				// x04: channel 7-4 enable 0x0F
#define ADC_ADDR_CLK 0x05				// x05: clock, channel 3-0 enable X | X | DCO | FCO | 3 | 2 | 1 | 0
#define ADC_ADDR_PWR 0x08				// x08: power: 000 normal, 001 power down, 010 standby, 011 reset
//#define ADC_ADDR_DCS 0x09				// x09: duty cycle stabilizer enabled by bit 0 (default is on)
#define ADC_ADDR_TESTIO 0x0D			// x0D: test_io  USERMODE[1:0] | PNRESETLONG | PNRESETSHORT | TESTMODE[3:0]
//#define ADC_ADDR_OMODE 0x14				// x14: output mode: X | LVDSPOWER | X | X | X | invert | BINARY[1:0]
//#define ADC_ADDR_ADJUST 0x15				// x15: adjust: X | X | termination[1:0] | X | X | X | DCO/FCO drive strength
#define ADC_ADDR_PHASE 0x16				// x16: phase: X | X | X | X | PHASEADJUST[3:0] (4-bits * 60 degrees, 0x0011 is default (180))
//#define ADC_ADDR_PTN1_LSB 0x19			// x19: user pattern1 lsb
//#define ADC_ADDR_PTN1_MSB 0x1A			// x1A: user pattern1 msb
//#define ADC_ADDR_PTN2_LSB 0x1B			// x1B: user pattern2 lsb
//#define ADC_ADDR_PTN2_MSB 0x1C			// x1C: user pattern2 msb
//#define ADC_ADDR_SRCTRL 0x21			// x21: serial control: LSBFIRST | X | X | X | SPEED | NBITS[2:0]
//#define ADC_ADDR_SRPWRWN 0x22			// x22: serial pwrdwn: XXXXXX | CH OUTPUT RESET | CH OUTPUT PWRDWN
//#define ADC_ADDR_LATCH 0x22				// xFF: latch settings by setting bit 0 high

// patterns:
//0000 = off (default)
//0001 = midscale short
//0010 = +FS short
//0011 = −FS short
//0100 = checkerboard output
//0101 = PN 23 sequence
//0110 = PN 9 sequence
//0111 = one-/zero-word toggle
//1000 = user input
//1001 = 1-/0-bit toggle
//1010 = 1× sync
//1011 = one bit high
//1100 = mixed bit frequency


//******************************************************************************
//                             Control Characters
//******************************************************************************
//command control character, 1 byte
#define RUN_STARTED 254
//dataBuffer control character, 2 bytes
#define STARTTRG                    0xfffe//begin of trigger, read next **2** bytes to retrieve buffer length
#define STARTBUF                    0xfffd//begin of buffer for same trigger, read next **2** bytes to retrieve buffer length
#define EMPTY                       0xfffc//begin of data buffer, but it is empty
#define ENDOFDATA                   0xfffb//end of data stream

//******************************************************************************
//                          Struct & Global Variables
//******************************************************************************

extern uint8_t hvcal;

volatile uint32_t channel_mask[3];
volatile uint32_t mapped_channel_mask[3];
extern uint32_t thischanmask;

extern uint8_t channel_map[96];
extern uint8_t adc_map[12];
extern uint8_t adc_phases[12];

extern uint16_t bufcount;
extern uint16_t bufcount_place_holder;
extern uint16_t readout_obloc;
extern uint16_t readout_obloc_place_holder;

extern char init_buff[30];
extern char dataBuffer[2000];
extern char outBuffer[2000]; // buffer for printing to serial port
extern uint8_t rx_buff[100];
extern uint8_t buffer[256]; // buffer for reading from serial port
extern uint32_t writePtr;

volatile uint32_t * registers_0_addr;

extern int readout_maxDelay;
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

//******************************************************************************
//                             Functions
//******************************************************************************

void GPIO_write(uint8_t pin, uint8_t value);
uint32_t GPIO_read(uint8_t pin);

//void reset_fabric();

uint16_t readU16fromBytes(uint8_t data[]);
uint32_t readU32fromBytes(uint8_t data[]);
void delayUs(int us);
void delay_ms(uint32_t us);
void delayTicks(uint8_t ticks);
char * print_float(char *fchars, float value);

void read_data(int *delay_count, int *trigger_count);
void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit);
void get_rates(int num_delays, int num_samples);
void get_mapped_channels();
void read_histogram(uint8_t channel, uint8_t hv_or_cal, uint16_t *output);

void adc_spi(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint16_t adc_mask_f);
void adc_spi_core(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint8_t adc_mask_h, uint8_t hvcal);
void adc_write(uint16_t address, uint8_t data, uint16_t adc_mask_f);
uint8_t adc_read(uint16_t address, uint8_t adc_num);
uint16_t init_adc(uint16_t adc_mask, uint8_t pattern, uint8_t phase);

void digi_write(uint8_t address, uint16_t data, uint8_t hvcal);
uint16_t digi_read(uint8_t address, uint8_t hvcal);
void bufWrite(char *outBuffer, uint16_t *bufcount, uint32_t data, uint16_t nbytes);
void outBufSend(UART_instance_t g_uart, char *outBuffer, uint16_t bufcount);
void resetFIFO();

#endif /* UTILS_H_ */

