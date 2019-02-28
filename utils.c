#include "utils.h"
#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"
#include "hw_platform.h"

// channel_map[index adc channel] = straw number
// adc_map[index adc channel order] = adc_number
#if IS_CAL
int channel_map[48] = {
		90,84,78,72,66,60,54,48,
		42,36,30,24,18,12,6,0,
		91,85,79,73,67,61,55,49,
		43,37,31,25,19,13,7,1,
		92,86,80,74,68,62,56,50,
		44,38,32,26,20,14,8,2};
int adc_map[6] = {0,1,2,3,4,5};
#endif
#if !IS_CAL
int channel_map[48] = {
		45,39,33,27,21,15,9,3,
		93,87,81,75,69,63,57,51,
		46,40,34,28,22,16,10,4,
		94,88,82,76,70,64,58,52,
		47,41,35,29,23,17,11,5,
		95,89,83,77,71,65,59,53};
int adc_map[6] = {0,1,2,3,4,5};
#endif

int adc_phases[6] = {0,0,0,0,0,0};


char outBuffer[1000]; // buffer for printing to serial port
char dataBuffer[2000];
char init_buff[16];// buffer for storing adc_init returns
uint8_t rx_buff[100];
uint8_t buffer[256]; // buffer for reading from serial port
uint32_t writePtr;

int readout_maxDelay;
int readout_obloc;
int readout_mode;
int readout_wordsPerTrigger;
int readout_numTriggers;

int calibration_count[32];
uint32_t calibration_done;

UART_instance_t g_uart;
spi_instance_t g_spi[4];
pwm_instance_t g_pwm;
gpio_instance_t g_gpio;

void reset_fabric(){
	/* RESET the fabric */
	//SYSREG->SOFT_RST_CR |= SYSREG_FPGA_SOFTRESET_MASK;
	/*take the fabric out of reset */
	//SYSREG->SOFT_RST_CR &= ~SYSREG_FPGA_SOFTRESET_MASK;

}

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

//char * print_float(char *fchars, float value)
//{
//    if (value < 0){
//        value *= -1;
//        sprintf(fchars,"%-d.%d%d",(int) value, (int) (value*10)%10, (int) (value*100)%10);
//    }else{
//        sprintf(fchars,"%d.%d%d",(int) value, (int) (value*10)%10, (int) (value*100)%10);
//    }
//    return fchars;
//}

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

void digi_write(uint8_t address, uint16_t data)
{
	volatile uint32_t *busy_p       = registers_0_addr+0x61;
	volatile uint32_t *data_p     = registers_0_addr+0x63;
	volatile uint32_t *address_p = registers_0_addr+0x62;
	volatile uint32_t *init_p     = registers_0_addr+0x60;

	*(data_p) = data;
	*(address_p) = address;
	*(init_p) = 1;
	while (*(busy_p) != 0){};
}

uint16_t digi_read(uint8_t address)
{

	volatile uint32_t *busy_p       = registers_0_addr+0x61;
	volatile uint32_t *data_p     = registers_0_addr+0x63;
	volatile uint32_t *address_p = registers_0_addr+0x62;
	volatile uint32_t *init_p     = registers_0_addr+0x60;

	*(address_p) = (0x1<<8) | address;
	*(init_p) = 1;
	while (*(busy_p) != 0){};
	return *(data_p);
}

uint8_t init_adc(uint8_t adc_mask, uint8_t pattern, uint8_t phase)
{
	//sprintf(outBuffer,"Init ADC %02x: pattern %02x phase %d\n",adc_mask,pattern,phase);
	//UART_polled_tx_string( &g_uart, outBuffer );
	init_buff[0] = adc_mask;
	init_buff[1] = pattern;
	init_buff[2] = phase;

	uint8_t errors = 0x0;
	// soft reset and check that it responds
	for (int i=0;i<6;i++){
		if ((0x1<<i) & adc_mask){
			adc_write(0x0,0x3C,(0x1<<i));
			int cleared = 0;
			for (int j=0;j<10;j++){
				if (adc_read(0x0,i) == 0x18){
					cleared = 1;
					break;
				}
			}
			init_buff[2*i+3]=adc_read(0x0,i);
			init_buff[2*i+4]=adc_read(0x01,i);

			if (!cleared){
				//sprintf(outBuffer,"ADC %d did not clear reset %02x\n",i,adc_read(0x0,i));
				//UART_polled_tx_string( &g_uart, outBuffer );
				errors |= (0x1<<i);
			}
			delayUs(100);
			// check chip ID
			if ( adc_read(0x01,i) != 0x08){
				//sprintf(outBuffer,"ADC %d wrong ID %02x\n",i,adc_read(0x01,i));
				//UART_polled_tx_string( &g_uart, outBuffer );
				errors |= (0x1<<i);
			}
		}
	}

	// enable DCO (not FCO)
	adc_write(0x05,0x3F,adc_mask);

	// set output phase
	adc_write(0x16,phase,adc_mask);

	// set test pattern
	adc_write(0x0D,pattern,adc_mask);

	init_buff[15] = errors;

	return errors;
}

// AD9212 memory map:
// x00: Config: 0 | LSBFIRST | SOFTRESET | 1 | 1 | SOFTRESET | LSBFIRST | 0
// x01: Chip ID  0x08
// x02: Chip ID2 0x00 or 0x10
// x04: channel 7-4 enable 0x0F
// x05: clock, channel 3-0 enable X | X | DCO | FCO | 3 | 2 | 1 | 0
// x08: power: 000 normal, 001 power down, 010 standby, 011 reset
// x09: duty cycle stabilizer enabled by bit 0 (default is on)
// x0D: test_io  USERMODE[1:0] | PNRESETLONG | PNRESETSHORT | TESTMODE[3:0]
// x14: output mode: X | LVDSPOWER | X | X | X | invert | BINARY[1:0]
// x15: adjust: X | X | termination[1:0] | X | X | X | DCO/FCO drive strength
// x16: phase: X | X | X | X | PHASEADJUST[3:0] (4-bits * 60 degrees, 0x0011 is default (180))
// x19: user pattern1 lsb
// x1A: user pattern1 msb
// x1B: user pattern2 lsb
// x1C: user pattern2 msb
// x21: serial control: LSBFIRST | X | X | X | SPEED | NBITS[2:0]
// x22: serial pwrdwn: XXXXXX | CH OUTPUT RESET | CH OUTPUT PWRDWN
// xFF: latch settings by setting bit 0 high

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

void adc_spi(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint8_t adc_mask_r)
{
	uint32_t spi_cs       = 0x8;
	uint32_t spi_sclk     = 0x9;
	uint32_t spi_sdio     = 0x6;
	uint32_t spi_sdio_dir = 0x7;

	// build instruction 16 bits
	// RW | W1 | W0 | A12 | A11 ... A2 | A1 | A0
	uint16_t instructions = ((0x1 & rw)<<15) | (((bytes-1)&0x3) << 13) | address;

	// set up initial state
	digi_write(spi_sclk,0);
	digi_write(spi_cs,0xFF);
	digi_write(spi_sdio,0x0);

	// set CS mask
	uint8_t adc_mask = ~(adc_mask_r);
	digi_write(spi_cs,adc_mask);

	delayTicks(4);

	// send instructions

	for (int i=15;i>=0;i--){
		uint8_t thisbit;
		if ((instructions & (0x1<<i)) != 0x0)
			thisbit = 1;
		else
			thisbit = 0;
		digi_write(spi_sdio,thisbit);
		delayTicks(4);
		digi_write(spi_sclk,1);
		delayTicks(4);
		digi_write(spi_sclk,0);
	}

	// if reading turn SDIO to Z
	if (rw == 1)
		digi_write(spi_sdio_dir,0x1);

	// read or write the set number of bytes
	for (int ibyte=0;ibyte<bytes;ibyte++){
		for (int i=7;i>=0;i--){
			// if writing need to send data
			if (rw == 0){
				uint8_t thisbit;
				if ((data[ibyte] & (0x1<<i)) != 0x0)
					thisbit = 1;
				else
					thisbit = 0;
				digi_write(spi_sdio,thisbit);
			}
			// if reading just wait for chip to set data
			delayTicks(4);
			// if reading now read this bit
			if (rw == 1){
				uint32_t readval = digi_read(spi_sdio);
				uint16_t dataval = (0x1<<i);
				if (readval != 0x0)
					data[ibyte] |= dataval;
			}
			digi_write(spi_sclk,1);
			delayTicks(4);
			digi_write(spi_sclk,0);
		}
	}

	delayTicks(4);
	digi_write(spi_cs,0xFF);
	digi_write(spi_sdio,0x0);
}

void adc_write(uint16_t address, uint8_t data, uint8_t adc_mask_r){
	adc_spi(0,1,address,&data,adc_mask_r);
	uint8_t mydata[1];
	mydata[0] = 0x1;
	adc_spi(0,1,0x0FF,mydata,adc_mask_r);
};
uint8_t adc_read(uint16_t address, uint8_t adc_num){
	uint8_t data = 0x0;
	adc_spi(1,1,address,&data,(0x1<<adc_num));
	return data;
};

void read_data(int *delay_count, int *trigger_count)
{
	uint16_t memlevel = 0;

	while (1){

		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + 0x13);
			memlevel = fifoinfo & 0x7FFF;
			if (memlevel < readout_wordsPerTrigger){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay){
					dataBuffer[0] = EMPTY & 0xff;
					dataBuffer[1] = EMPTY >>8;
					UART_send(&g_uart, dataBuffer ,2);
					break;
				}
				delayUs(1000);
				continue;
			}
		}

		readout_obloc = 0;
		dataBuffer[readout_obloc++] = STARTTRG & 0xff;
		dataBuffer[readout_obloc++] = STARTTRG >> 8;
		readout_obloc_place_holder = readout_obloc;
		readout_obloc += 2;

		// have enough data, see if can read
		for (int j=0;j<readout_wordsPerTrigger;j++){
			//if (readout_obloc > 600){//originally each package contains 1501=6+5*299 chars, which now becomes 2+2+299*2 bytes
			if (readout_obloc > 1500){
				//sprintf(&dataBuffer[readout_obloc],"\npause\n");
				dataBuffer[readout_obloc_place_holder] = (readout_obloc-4) & 0xff;
				dataBuffer[readout_obloc_place_holder+1] = (readout_obloc-4) >> 8;
				UART_send(&g_uart, dataBuffer ,readout_obloc);

				readout_obloc = 0;
				dataBuffer[readout_obloc++] = STARTBUF & 0xff;
				dataBuffer[readout_obloc++] = STARTBUF >> 8;
				readout_obloc_place_holder = readout_obloc;
				readout_obloc += 2;

				//sprintf(dataBuffer,"start\n");
				//readout_obloc = 6;
			}

			uint16_t digioutput;
			*(registers_0_addr + 0x11) = 1;
			digioutput = *(registers_0_addr + 0x12);
			memlevel -= 1;
			//sprintf(&dataBuffer[readout_obloc],"%04x ",digioutput);
			dataBuffer[readout_obloc++] = digioutput & 0xff;
			dataBuffer[readout_obloc++] = digioutput >> 8;
			//readout_obloc += 5;
		}
		dataBuffer[readout_obloc_place_holder] = (readout_obloc-4) & 0xff;
		dataBuffer[readout_obloc_place_holder+1] = (readout_obloc-4) >> 8;
		UART_send(&g_uart, dataBuffer ,readout_obloc);

		(*trigger_count)++;

		//update count obloc;

		if ((*trigger_count) >= readout_numTriggers){
			dataBuffer[0] = ENDOFDATA & 0xff;
			dataBuffer[1] = ENDOFDATA >>8;
			UART_send(&g_uart, dataBuffer ,2);
			break;
		}
	}
}

void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit)
{
	uint16_t memlevel = 0;
	while (1){
		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + 0x45);
			memlevel = fifoinfo & 0x7FFF;
			if (memlevel < readout_wordsPerTrigger){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay)
					break;
				delayUs(1000);
				continue;
			}
		}

		// have enough data, see if can read
		for (int j=0;j<readout_wordsPerTrigger;j++){
			uint16_t digioutput;
			*(registers_0_addr + 0x40) = 1;
			digioutput = *(registers_0_addr + 0x41);
			memlevel -= 1;
			lasthit[j] = digioutput;
		}
		(*trigger_count)++;
		if ((*trigger_count) >= readout_numTriggers)
			break;
	}
}

void get_rates(uint32_t channel_mask1, uint32_t channel_mask2, uint32_t channel_mask3, int num_delays, int num_samples)
{
	uint32_t mapped_channel_mask1 = 0x0;
	uint32_t mapped_channel_mask2 = 0x0;
	get_mapped_channels(&channel_mask1,&channel_mask2,&channel_mask3,&mapped_channel_mask1,&mapped_channel_mask2);

	//double total_global_time = 0;
	uint32_t total_time_counts = 0;
	uint32_t total_hv[96];
	uint32_t total_cal[96];
	uint32_t total_coinc[96];

	for (int k=0;k<96;k++){
		total_hv[k] = 0;
		total_cal[k] = 0;
		total_coinc[k] = 0;
	}

	volatile uint32_t *gt0 = (registers_0_addr + 0x16);
	volatile uint32_t *gt1 = (registers_0_addr + 0x17);
	volatile uint32_t *gt2 = (registers_0_addr + 0x18);
	volatile uint32_t *gt3 = (registers_0_addr + 0x19);
	volatile uint32_t *hv = (registers_0_addr + 0x1A);
	volatile uint32_t *cal = (registers_0_addr + 0x1B);
	volatile uint32_t *coinc = (registers_0_addr + 0x1C);
	volatile uint32_t *latch = (registers_0_addr + 0x1D);

	*(latch) = 1;
	uint64_t start_global_time = ((uint64_t)*(gt0)<<48) | ((uint64_t)*(gt1)<<32) | ((uint64_t)*(gt2)<<16) | ((uint64_t)*(gt3)<<0);
	//uint32_t start_hv = *(hv);
	//uint32_t start_cal = *(cal);
	//uint32_t start_coinc = *(coinc);
	for (int j=0;j<num_samples;j++){
		delayUs(num_delays);
		*(latch) = 1;
		uint64_t end_global_time = ((uint64_t)*(gt0)<<48) | ((uint64_t)*(gt1)<<32) | ((uint64_t)*(gt2)<<16) | ((uint64_t)*(gt3)<<0);

		//total_global_time += (end_global_time-start_global_time) * 20e-9;
		total_time_counts += (uint32_t) (end_global_time - start_global_time);

		for (int k=0;k<48;k++){
			if ((k<32 && ((0x1<<k) & mapped_channel_mask1)) || (k>=32 && ((0x1<<(k-32)) & mapped_channel_mask2))){
				int straw_num = channel_map[k];

				*(registers_0_addr + 0x15) = k;
				delayUs(1);
				uint32_t end_hv = *(hv);
				uint32_t end_cal = *(cal);
				uint32_t end_coinc = *(coinc);

		//total_hv += (end_hv-start_hv) % 65536;
		//total_cal += (end_cal-start_cal) % 65536;
		//total_coinc += (end_coinc-start_coinc) % 65536;
				total_hv[straw_num] += (end_hv) % 65536;
				total_cal[straw_num] += (end_cal) % 65536;
				total_coinc[straw_num] += (end_coinc) % 65536;
			}
		}


		start_global_time = end_global_time;
		//start_hv = end_hv;
		//start_cal = end_cal;
		//start_coinc = end_coinc;
	}
	for (int i=0;i<32;i++){
		if ((0x1<<i) & channel_mask1){
			//double total_global_time = total_time_counts*16e-9;
			//sprintf(outBuffer,"%d: HV %d Hz, CAL %d Hz, COINC %d Hz, %d %d %d %d\n",
			//		i,(int)(total_hv[i]/total_global_time),(int)(total_cal[i]/total_global_time),(int)(total_coinc[i]/total_global_time),
			//		total_hv[i],total_cal[i],total_coinc[i],total_time_counts);
			//UART_polled_tx_string( &g_uart, outBuffer );
			outBuffer[bufcount++] = i;
			outBuffer[bufcount++] = total_hv[i] & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_hv[i] >> 24;
			outBuffer[bufcount++] = total_cal[i] & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_cal[i] >> 24;
			outBuffer[bufcount++] = total_coinc[i] & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_coinc[i] >> 24;
			outBuffer[bufcount++] = total_time_counts & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 16) & 0xff;
			outBuffer[bufcount++] = total_time_counts >> 24;

		}
	}
	for (int j=0;j<32;j++){
		if ((0x1<<j) & channel_mask2){
			int i = j+32;
			//double total_global_time = total_time_counts*16e-9;
			//sprintf(outBuffer,"%d: HV %d Hz, CAL %d Hz, COINC %d Hz, %d %d %d %d\n",
			//		i,(int)(total_hv[i]/total_global_time),(int)(total_cal[i]/total_global_time),(int)(total_coinc[i]/total_global_time),
			//		total_hv[i],total_cal[i],total_coinc[i],total_time_counts);
			//UART_polled_tx_string( &g_uart, outBuffer );
			outBuffer[bufcount++] = i;
			outBuffer[bufcount++] = total_hv[i] & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_hv[i] >> 24;
			outBuffer[bufcount++] = total_cal[i] & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_cal[i] >> 24;
			outBuffer[bufcount++] = total_coinc[i] & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_coinc[i] >> 24;
			outBuffer[bufcount++] = total_time_counts & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 16) & 0xff;
			outBuffer[bufcount++] = total_time_counts >> 24;

		}
	}
	for (int j=0;j<32;j++){
		if ((0x1<<j) & channel_mask3){
			int i = j+64;
			//double total_global_time = total_time_counts*16e-9;
			//sprintf(outBuffer,"%d: HV %d Hz, CAL %d Hz, COINC %d Hz, %d %d %d %d\n",
			//		i,(int)(total_hv[i]/total_global_time),(int)(total_cal[i]/total_global_time),(int)(total_coinc[i]/total_global_time),
			//		total_hv[i],total_cal[i],total_coinc[i],total_time_counts);
			//UART_polled_tx_string( &g_uart, outBuffer );
			outBuffer[bufcount++] = i;
			outBuffer[bufcount++] = total_hv[i] & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_hv[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_hv[i] >> 24;
			outBuffer[bufcount++] = total_cal[i] & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_cal[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_cal[i] >> 24;
			outBuffer[bufcount++] = total_coinc[i] & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_coinc[i] >> 16) & 0xff;
			outBuffer[bufcount++] = total_coinc[i] >> 24;
			outBuffer[bufcount++] = total_time_counts & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 8) & 0xff;
			outBuffer[bufcount++] = ( total_time_counts >> 16) & 0xff;
			outBuffer[bufcount++] = total_time_counts >> 24;

		}
	}
}

void get_mapped_channels(uint32_t *channel_mask1, uint32_t *channel_mask2, uint32_t *channel_mask3, uint32_t *mapped_channel_mask1, uint32_t *mapped_channel_mask2)
{
	(*mapped_channel_mask1) = 0x0;
	(*mapped_channel_mask2) = 0x0;
	for (int i=0;i<32;i++){
		int found = 0;
		if ((0x1<<i) & (*channel_mask1)){
			for (int j=0;j<48;j++){
				if (channel_map[j] == i){
					found = 1;
					if (j < 32)
						(*mapped_channel_mask1) |= (0x1<<j);
					else
						(*mapped_channel_mask2) |= (0x1<<(j-32));
					break;
				}
			}
		}
		if (found == 0)
			 (*channel_mask1) &= ~(0x1<<i);
	}
	for (int i=0;i<32;i++){
		int found = 0;
		if ((0x1<<i) &  (*channel_mask2)){
			for (int j=0;j<48;j++){
				if (channel_map[j] == i+32){
					found = 1;
					if (j < 32)
						(*mapped_channel_mask1) |= (0x1<<j);
					else
						(*mapped_channel_mask2) |= (0x1<<(j-32));
					break;
				}
			}
		}
		if (found == 0)
			 (*channel_mask2) &= ~(0x1<<i);
	}
	for (int i=0;i<32;i++){
		int found = 0;
		if ((0x1<<i) & (*channel_mask3)){
			for (int j=0;j<48;j++){
				if (channel_map[j] == i+64){
					found = 1;
					if (j < 32)
						(*mapped_channel_mask1) |= (0x1<<j);
					else
						(*mapped_channel_mask2) |= (0x1<<(j-32));
					break;
				}
			}
		}
		if (found == 0)
			 (*channel_mask3) &= ~(0x1<<i);
	}
}

