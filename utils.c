#include "utils.h"
#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"
#include "hw_platform.h"
#include "setup.h"

// channel_map[index adc channel] = straw number
// adc_map[index adc channel order] = adc_number

uint16_t default_gains_cal[96] = {271,275,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,269,269,269,269,269,269,269,268,269,269,269,264,269,269,269,270};
uint16_t default_gains_hv[96] = {270,271,270,270,270,270,270,270,270,270,270,270,270,270,266,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,267,270,270,270,270,266,266,272,268,266,265,269,269,269,269,270};
uint16_t default_threshs_cal[96] = {331,316,339,309,317,317,319,335,341,321,319,359,347,320,323,332,352,334,325,339,304,312,340,330,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,305,294,311,307,340,332,321,336,335,327,322,356,344,304,324,325,315,325,325,341,348,343,343,319};
uint16_t default_threshs_hv[96] = {322,284,329,317,325,321,340,347,351,339,348,349,313,337,327,325,345,325,321,320,345,332,305,335,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,284,293,324,321,324,331,344,333,314,324,333,346,330,335,327,329,349,316,331,339,318,304,332,310};

uint8_t hvcal=1;//1 CAL 2 HV 0 both
uint8_t channel_map[96]={
		90,84,78,72,66,60,54,48,
		42,36,30,24,18,12,6,0,
		91,85,79,73,67,61,55,49,
		43,37,31,25,19,13,7,1,
		92,86,80,74,68,62,56,50,
		44,38,32,26,20,14,8,2,

		45,39,33,27,21,15,9,3,
		93,87,81,75,69,63,57,51,
		46,40,34,28,22,16,10,4,
		94,88,82,76,70,64,58,52,
		47,41,35,29,23,17,11,5,
		95,89,83,77,71,65,59,53};
uint8_t adc_map[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
uint8_t adcclk_map[12] = {0,0,2,3,3,3,6,6,6,9,11,11};

uint8_t adc_phases[12] = {0};

char outBuffer[2000]; // buffer for printing to serial port
char dataBuffer[2000];
char init_buff[30];// buffer for storing adc_init returns
uint8_t rx_buff[100];
uint8_t buffer[256]; // buffer for reading from serial port
uint32_t writePtr;

uint16_t bufcount;
uint16_t bufcount_place_holder;
uint16_t readout_obloc;
uint16_t readout_obloc_place_holder;

uint32_t thischanmask;

int readout_maxDelay;
int readout_mode;
int readout_wordsPerTrigger;
int readout_numTriggers;

int calibration_count[32];
uint32_t calibration_done;

UART_instance_t g_uart;
spi_instance_t g_spi[4];
pwm_instance_t g_pwm;
gpio_instance_t g_gpio;

//void reset_fabric(){
//	/* RESET the fabric */
//	//SYSREG->SOFT_RST_CR |= SYSREG_FPGA_SOFTRESET_MASK;
//	/*take the fabric out of reset */
//	//SYSREG->SOFT_RST_CR &= ~SYSREG_FPGA_SOFTRESET_MASK;
//
//}

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
	for (uint8_t i=0; i<4; i++)
		u.b[i] = data[i];

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
    volatile uint32_t delay_count = SystemCoreClock / 1000 * us;

    while(delay_count > 0u)
    {
        --delay_count;
    }
}

void delayUs(int us)
{
	volatile uint32_t delay_count = SystemCoreClock / 1000000 * us;

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

void digi_write(uint8_t address, uint16_t data, uint8_t hvcal)
{
	volatile uint32_t * busy_p_cal;
	volatile uint32_t * data_p_cal;
	volatile uint32_t * address_p_cal;
	volatile uint32_t * init_p_cal;
	volatile uint32_t * busy_p_hv;
	volatile uint32_t * data_p_hv;
	volatile uint32_t * address_p_hv;
	volatile uint32_t * init_p_hv;

	if ((hvcal==1)||(hvcal==0)){
		busy_p_cal = registers_0_addr+REG_ROC_CAL_BUSY_P;
		data_p_cal = registers_0_addr+REG_ROC_CAL_DATA_P;
		address_p_cal = registers_0_addr+REG_ROC_CAL_ADDRESS_P;
		init_p_cal = registers_0_addr+REG_ROC_CAL_INIT_P;

		*(data_p_cal) = data;
		*(address_p_cal) = address;
		*(init_p_cal) = 1;
		while (*(busy_p_cal) != 0){};
	}
	if ((hvcal==2)||(hvcal==0)){
		busy_p_hv = registers_0_addr+REG_ROC_HV_BUSY_P;
		data_p_hv = registers_0_addr+REG_ROC_HV_DATA_P;
		address_p_hv = registers_0_addr+REG_ROC_HV_ADDRESS_P;
		init_p_hv = registers_0_addr+REG_ROC_HV_INIT_P;

		*(data_p_hv) = data;
		*(address_p_hv) = address;
		*(init_p_hv) = 1;
		while (*(busy_p_hv) != 0){};
	}
}

uint16_t digi_read(uint8_t address, uint8_t hvcal)//hvcal can only be 1 or 2
{
	volatile uint32_t * busy_p_cal;
	volatile uint32_t * data_p_cal;
	volatile uint32_t * address_p_cal;
	volatile uint32_t * init_p_cal;
	volatile uint32_t * busy_p_hv;
	volatile uint32_t * data_p_hv;
	volatile uint32_t * address_p_hv;
	volatile uint32_t * init_p_hv;

	if (hvcal!=1 && hvcal!=2)
		hvcal = 1;//for safety.

	if (hvcal==1){
		busy_p_cal = registers_0_addr+REG_ROC_CAL_BUSY_P;
		data_p_cal = registers_0_addr+REG_ROC_CAL_DATA_P;
		address_p_cal = registers_0_addr+REG_ROC_CAL_ADDRESS_P;
		init_p_cal = registers_0_addr+REG_ROC_CAL_INIT_P;

		*(address_p_cal) = (0x1<<8) | address;
		*(init_p_cal) = 1;
		while (*(busy_p_cal) != 0){};
		return *(data_p_cal);
	}
	else{
		busy_p_hv = registers_0_addr+REG_ROC_HV_BUSY_P;
		data_p_hv = registers_0_addr+REG_ROC_HV_DATA_P;
		address_p_hv = registers_0_addr+REG_ROC_HV_ADDRESS_P;
		init_p_hv = registers_0_addr+REG_ROC_HV_INIT_P;

		*(address_p_hv) = (0x1<<8) | address;
		*(init_p_hv) = 1;
		while (*(busy_p_hv) != 0){};
		return *(data_p_hv);
	}
}

void read_histogram(uint8_t channel, uint8_t hv_or_cal, uint16_t *output){
	// select channel
	digi_write(DG_ADDR_HISTO_CHANNEL,((channel << 1) | (hv_or_cal & 0x1)),1);
	// tell digi to write histogram to sram
	digi_write(DG_ADDR_HISTO_RESET,0x1,1);
	digi_write(DG_ADDR_HISTO_RESET,0x0,1);
	delay_ms(1);
	// read from sram
	for (int i=0;i<256;i++){
		digi_write(DG_ADDR_HISTO_BIN,i,1);
		output[i] = digi_read(DG_ADDR_HISTO_VAL,1);
	}
	/*
	if (hv_or_cal == 1){
	volatile uint32_t * ewm = registers_0_addr + REG_ROC_EWM_SINGLE;
	*ewm = 1;
	*ewm = 0;
	}else{
	delay_ms(100);
	uint16_t ewm_counter = digi_read(DG_ADDR_EWMCNTER,1);
	output[0] = ewm_counter;
	delay_ms(100);
	ewm_counter = digi_read(DG_ADDR_EWMCNTER,1);
	output[1] = ewm_counter;
	}
	*/

}

uint16_t init_adc(uint16_t adc_mask, uint8_t pattern, uint8_t phase)
{
	//sprintf(outBuffer,"Init ADC %02x: pattern %02x phase %d\n",adc_mask,pattern,phase);
	//UART_polled_tx_string( &g_uart, outBuffer );
	bufcount = 0;
	bufWrite(init_buff, &bufcount, adc_mask, 2);
	init_buff[bufcount++] = pattern;
	init_buff[bufcount++] = phase;

//	if ((adc_mask >> 6) && ((adc_mask & 0x3f)==0))
//		hvcal = 2;
//	if ((adc_mask >> 6) && (adc_mask & 0x3f))
//		hvcal = 0;

	uint16_t errors = 0x0;
	// soft reset and check that it responds
	for (int i=0;i<12;i++){
		if ((0x1<<i) & adc_mask){
			adc_write(ADC_ADDR_CONFIG,0x3C,(0x1<<i));
			int cleared = 0;
			for (int j=0;j<10;j++){
				if (adc_read(ADC_ADDR_CONFIG,i) == 0x18){
					cleared = 1;
					break;
				}
			}
			init_buff[bufcount++]=adc_read(ADC_ADDR_CONFIG,i);
			init_buff[bufcount++]=adc_read(ADC_ADDR_CID1,i);

			if (!cleared){
				//sprintf(outBuffer,"ADC %d did not clear reset %02x\n",i,adc_read(0x0,i));
				//UART_polled_tx_string( &g_uart, outBuffer );
				errors |= (0x1<<i);
			}
			delayUs(100);
			// check chip ID
			if ( adc_read(ADC_ADDR_CID1,i) != 0x08){
				//sprintf(outBuffer,"ADC %d wrong ID %02x\n",i,adc_read(0x01,i));
				//UART_polled_tx_string( &g_uart, outBuffer );
				errors |= (0x1<<i);
			}
		}
	}

	// enable DCO (not FCO)
	adc_write(ADC_ADDR_CLK,0x3F,adc_mask);

	// set output phase
	adc_write(ADC_ADDR_PHASE,phase,adc_mask);

	// set test pattern
	adc_write(ADC_ADDR_TESTIO,pattern,adc_mask);

	bufWrite(init_buff, &bufcount, errors, 2);

	return errors;
}

void adc_spi(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint16_t adc_mask_f)
{
	uint8_t adc_mask_cal = adc_mask_f & 0x3f;
	uint8_t adc_mask_hv = (adc_mask_f>>6) & 0x3f;

	if (adc_mask_cal)
		adc_spi_core(rw, bytes, address, data, adc_mask_cal, 1);
	if (adc_mask_hv)
		adc_spi_core(rw, bytes, address, data, adc_mask_hv, 2);

}

void adc_spi_core(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint8_t adc_mask_h, uint8_t hvcal)
{
	// build instruction 16 bits
	// RW | W1 | W0 | A12 | A11 ... A2 | A1 | A0
	uint16_t instructions = ((0x1 & rw)<<15) | (((bytes-1)&0x3) << 13) | address;

	// set up initial state
	digi_write(DG_ADDR_SPI_SCLK, 0, hvcal);
	digi_write(DG_ADDR_SPI_CS, 0xFF, hvcal);
	digi_write(DG_ADDR_SPI_SDIO, 0x0, hvcal);

	// set CS mask
	uint8_t adc_mask = ~(adc_mask_h);
	digi_write(DG_ADDR_SPI_CS,adc_mask, hvcal);

	delayTicks(4);

	// send instructions

	for (int i=15;i>=0;i--){
		uint8_t thisbit;
		if ((instructions & (0x1<<i)) != 0x0)
			thisbit = 1;
		else
			thisbit = 0;
		digi_write(DG_ADDR_SPI_SDIO, thisbit, hvcal);
		delayTicks(4);
		digi_write(DG_ADDR_SPI_SCLK, 1, hvcal);
		delayTicks(4);
		digi_write(DG_ADDR_SPI_SCLK, 0, hvcal);
	}

	// if reading turn SDIO to Z
	if (rw == 1)
		digi_write(DG_ADDR_SPI_SDIO_DIR, 0x1, hvcal);

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
				digi_write(DG_ADDR_SPI_SDIO, thisbit, hvcal);
			}
			// if reading just wait for chip to set data
			delayTicks(4);
			// if reading now read this bit
			if (rw == 1){
				uint32_t readval = digi_read(DG_ADDR_SPI_SDIO, hvcal);
				uint16_t dataval = (0x1<<i);
				if (readval != 0x0)
					data[ibyte] |= dataval;
			}
			digi_write(DG_ADDR_SPI_SCLK, 1, hvcal);
			delayTicks(4);
			digi_write(DG_ADDR_SPI_SCLK, 0, hvcal);
		}
	}

	delayTicks(4);
	digi_write(DG_ADDR_SPI_CS, 0xFF, hvcal);
	digi_write(DG_ADDR_SPI_SDIO, 0x0, hvcal);
}

void adc_write(uint16_t address, uint8_t data, uint16_t adc_mask_f){
	adc_spi(0,1,address,&data,adc_mask_f);
	uint8_t mydata[1];
	mydata[0] = 0x1;
	adc_spi(0,1,0x0FF,mydata,adc_mask_f);
}

uint8_t adc_read(uint16_t address, uint8_t adc_num){
	uint8_t data = 0x0;
	adc_spi(1,1,address,&data,(0x1<<adc_num));
	return data;
}

void read_data(int *delay_count, int *trigger_count)
{

	volatile uint32_t * ewm = registers_0_addr + REG_ROC_EWM_SINGLE;
	for (int i=0;i<1000;i++){
	//	*ewm = 1;
	//	*ewm = 0;
	//	delayTicks(100);
	}
	uint16_t memlevel = 0;

	uint32_t rdcnt = REG_ROC_FIFO_RDCNT;
	uint32_t re = REG_ROC_FIFO_RE;
	uint32_t data_reg = REG_ROC_FIFO_DATA;

	while (1){

		readout_obloc = 0;

		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
			memlevel = fifoinfo & 0x1FFFF;
			if (memlevel < readout_wordsPerTrigger){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay){
					bufWrite(dataBuffer, &readout_obloc, EMPTY, 2);
					UART_send(&g_uart, dataBuffer ,2);
					break;
				}
				delayUs(1000);
				continue;
			}
		}

		bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
		readout_obloc_place_holder = readout_obloc;
		readout_obloc += 2;

		// have enough data, see if can read
		for (int j=0;j<readout_wordsPerTrigger;j++){



			//if (readout_obloc > 600){//originally each package contains 1501=6+5*299 chars, which now becomes 2+2+299*2 bytes
			if (readout_obloc > 1500){
				//sprintf(&dataBuffer[readout_obloc],"\npause\n");
				bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);
				UART_send(&g_uart, dataBuffer ,readout_obloc);

				readout_obloc = 0;
				bufWrite(dataBuffer, &readout_obloc, STARTBUF, 2);
				readout_obloc_place_holder = readout_obloc;
				readout_obloc += 2;

				//sprintf(dataBuffer,"start\n");
				//readout_obloc = 6;
			}

			volatile uint32_t digioutput;
			*(registers_0_addr + re) = 1;
			digioutput = *(registers_0_addr + data_reg);
			memlevel -= 1;
			//sprintf(&dataBuffer[readout_obloc],"%04x ",digioutput);
			bufWrite(dataBuffer, &readout_obloc, ((digioutput & 0xFFFF0000)>>16), 2);
			bufWrite(dataBuffer, &readout_obloc, (digioutput & 0xFFFF), 2);
			//readout_obloc += 5;
		}
		bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);
		UART_send(&g_uart, dataBuffer ,readout_obloc);

		(*trigger_count)++;

		//update count obloc;

		if ((*trigger_count) >= readout_numTriggers){
			readout_obloc = 0;
			bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
			UART_send(&g_uart, dataBuffer ,2);
			break;
		}
	}
}

void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit)
{
	uint16_t memlevel = 0;
	int fail_count = 0;

	uint32_t rdcnt = REG_ROC_FIFO_RDCNT;
	uint32_t re = REG_ROC_FIFO_RE;
	uint32_t data_reg = REG_ROC_FIFO_DATA;

	while (1){
		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
			memlevel = fifoinfo & 0x1FFFF;
			if (memlevel < readout_wordsPerTrigger){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay)
					break;
				delayUs(1000);
				continue;
			}
		}

		// have enough data, see if can read
		int failed = 0;
		for (int j=0;j<readout_wordsPerTrigger;j++){

			volatile uint32_t digioutput;
			*(registers_0_addr + re) = 1;
			digioutput = *(registers_0_addr + data_reg);
			memlevel -= 1;
			if ((j == 0) && ((digioutput&0x80000000) != 0x80000000)){
				failed = 1;
				fail_count++;
				break;
			}
			lasthit[j] = (uint16_t) ((digioutput & 0x3FF00000)>>20);
		}
		if (fail_count >= readout_wordsPerTrigger*10)
			break;
		if (failed)
			continue;
		(*trigger_count)++;
		if ((*trigger_count) >= readout_numTriggers)
			break;
	}
}

uint32_t get_rates(int num_delays, int num_samples, uint8_t channel, uint32_t* timecounts){
	mapped_channel_mask[0] = 0x0;
	mapped_channel_mask[1] = 0x0;
	mapped_channel_mask[2] = 0x0;
	get_mapped_channels();

	//double total_global_time = 0;
	uint32_t total_time_counts[2] = {0};
	uint32_t total_hv[96] = {0};
	uint32_t total_cal[96] = {0};
	uint32_t total_coinc[96] = {0};
	uint8_t ishv[96]={0};

	uint64_t start_global_time = 0;
	uint64_t end_global_time = 0;

	uint16_t end_hv = 0;
	uint16_t end_cal = 0;
	uint16_t end_coinc = 0;

	for (uint8_t ihvcal=1; ihvcal<3; ihvcal++){
		digi_write(DG_ADDR_LATCH, 1, ihvcal);
		start_global_time = (((uint64_t) digi_read(DG_ADDR_GT0,ihvcal))<<48) |
				(((uint64_t) digi_read(DG_ADDR_GT1,ihvcal))<<32) |
				(((uint64_t) digi_read(DG_ADDR_GT2,ihvcal))<<16) |
				((uint64_t) digi_read(DG_ADDR_GT3,ihvcal));
		for (uint8_t j=0;j<num_samples;j++){
			delayUs(num_delays);
			digi_write(DG_ADDR_LATCH, 1, ihvcal);
			end_global_time = (((uint64_t) digi_read(DG_ADDR_GT0,ihvcal))<<48) |
					(((uint64_t) digi_read(DG_ADDR_GT1,ihvcal))<<32) |
					(((uint64_t) digi_read(DG_ADDR_GT2,ihvcal))<<16) |
					((uint64_t) digi_read(DG_ADDR_GT3,ihvcal));
			total_time_counts[ihvcal-1] += (uint32_t) (end_global_time - start_global_time); //FIXME
			//total_time_counts[ihvcal-1] += num_delays;

			for (uint8_t k=(48*(ihvcal-1));k<48*ihvcal;k++){
				uint8_t condition = 0;
				uint8_t straw_num = channel_map[k];
				ishv[straw_num] = ihvcal-1;

				if (ihvcal == 1)
					condition =((k<32 && ((0x1<<k) & mapped_channel_mask[0])) || (k>=32 && ((0x1<<(k-32)) & mapped_channel_mask[1])));
				else if (ihvcal == 2)
					condition =((k<64 && ((0x1<<(k-32)) & mapped_channel_mask[1])) || (k>=64 && ((0x1<<(k-64)) & mapped_channel_mask[2])));

				if (condition){

					digi_write(DG_ADDR_READCHANNEL, k%48, ihvcal);
					delayUs(1);
					end_hv = digi_read(DG_ADDR_HV,ihvcal);
					end_cal = digi_read(DG_ADDR_CAL,ihvcal);
					end_coinc = digi_read(DG_ADDR_COINC,ihvcal);


					total_hv[straw_num] += end_hv;
					total_cal[straw_num] += end_cal;
					total_coinc[straw_num] += end_coinc;
				}
			}
			start_global_time = end_global_time;
		}
	}

	uint32_t result = 0;
	if (channel > 191){
		for (uint8_t i=0;i<96;i++){
			if ( ((i<32) && ((0x1<<(i%32)) & channel_mask[0]))||
					((i>=32) && (i<64) && ((0x1<<(i%32)) & channel_mask[1]))||
					((i>=64) && ((0x1<<(i%32)) & channel_mask[2])) ){
				//double total_global_time = total_time_counts*16e-9;
				//sprintf(outBuffer,"%d: HV %d Hz, CAL %d Hz, COINC %d Hz, %d %d %d %d\n",
				//		i,(int)(total_hv[i]/total_global_time),(int)(total_cal[i]/total_global_time),(int)(total_coinc[i]/total_global_time),
				//		total_hv[i],total_cal[i],total_coinc[i],total_time_counts);
				//UART_polled_tx_string( &g_uart, outBuffer );
				outBuffer[bufcount++] = i;
				bufWrite(outBuffer, &bufcount, total_hv[i], 4);
				bufWrite(outBuffer, &bufcount, total_cal[i], 4);
				bufWrite(outBuffer, &bufcount, total_coinc[i], 4);
				bufWrite(outBuffer, &bufcount, total_time_counts[ishv[i]], 4);
			}
		}
	}else{
		for (uint8_t i=0;i<96;i++){
			if (i == (channel%96)){
				result = total_hv[i]*(channel/96)+total_cal[i]*(1-channel/96);
				*timecounts=total_time_counts[ishv[i]];
				break;
			}
		}
	}
	return result;
}

void get_mapped_channels(){
	//mapped mask[0], mask[1][0:15] are CAl, rest HV

	mapped_channel_mask[0] = 0x0;
	mapped_channel_mask[1] = 0x0;
	mapped_channel_mask[2] = 0x0;
	for (uint8_t k=0;k<3;k++){
		for (uint8_t i=0;i<32;i++){
			uint8_t found = 0;
			if ((0x1<<i) & channel_mask[k]){
				for (uint8_t j=0;j<96;j++){
					if (channel_map[j] == i+32*k){
						found = 1;
						if (j < 32)
							mapped_channel_mask[0] |= (0x1<<j);
						else if (j<64)
							mapped_channel_mask[1] |= (0x1<<(j-32));
						else
							mapped_channel_mask[2] |= (0x1<<(j-64));
						break;
					}
				}
			}
			if (found == 0)
				 channel_mask[k] &= ~(0x1<<i);
		}
	}
}

void bufWrite(char *outBuffer, uint16_t *bufcount, uint32_t data, uint16_t nbytes){
	for (uint8_t i=0; i<nbytes; i++){
		outBuffer[(*bufcount)++] = (data>>(i*8)) & 0xff;
	}
}

void outBufSend(UART_instance_t g_uart, char *outBuffer, uint16_t bufcount){
	UART_polled_tx_string( &g_uart, "monitoring\n" );
	UART_send(&g_uart, outBuffer ,bufcount );
}

void resetFIFO(){
	digi_write(DG_ADDR_RESET, 0, 0);
	*(registers_0_addr + REG_ROC_FIFO_RESET) = 1;
	digi_write(DG_ADDR_RESET, 1, 0);
//	reset_fabric();
}

void setPreampGain(uint16_t channel, uint16_t value){
	if (channel < 96){
		LTC2634_write(strawsCal[channel]._ltc,strawsCal[channel]._gain,value);
		default_gains_cal[channel] = value;
		//						sprintf(outBuffer,"Set channel %d CAL gain to %d\n",channel,value);
	}
	if (channel >= 96){

		LTC2634_write(strawsHV[channel - 96]._ltc,strawsHV[channel - 96]._gain,value);
		default_gains_hv[channel-96] = value;
		//						sprintf(outBuffer,"Set channel %d HV gain to %d\n",channel,value);
	}
}

void setPreampThreshold(uint16_t channel, uint16_t value){
	if (channel < 96){
		LTC2634_write(strawsCal[channel]._ltc,strawsCal[channel]._thresh,value);
		//						sprintf(outBuffer,"Set channel %d CAL threshold to %d\n",channel,value);
		default_threshs_cal[channel] = value;
	}
	if (channel >= 96){

		LTC2634_write(strawsHV[channel - 96]._ltc,strawsHV[channel - 96]._thresh,value);
		//						sprintf(outBuffer,"Set channel %d HV threshold to %d\n",channel,value);
		default_threshs_hv[channel-96] = value;
	}
}

void findChThreshold(int num_delays, int num_samples, uint16_t channel, uint16_t target_rate, uint8_t verbose){
	uint16_t threshold = 0;
	uint32_t timecounts = 0;
	if (channel < 96)
		threshold = default_threshs_cal[channel];
	else
		threshold = default_threshs_hv[channel-96];
	uint32_t lastrate = 10000000;
	uint32_t thisrate = 0;
	uint16_t initThreshold = threshold;
	uint16_t lastThreshold = threshold;
	uint8_t zerocounts = 0;
	uint16_t nonzero = 0;
	uint32_t nonzerorate = 0;
	uint8_t panic_mode = 0;

	uint16_t local_bufcount_place_holder = bufcount;
	if (verbose==1)
		bufWrite(outBuffer, &bufcount, 0, 2);

	while(1){
		uint32_t thiscount = get_rates(num_delays, num_samples, channel, &timecounts);
		thisrate = (uint32_t)(((uint64_t)thiscount)*50000000/timecounts);

		if ((verbose==1)&&(bufcount<1950)){//buffer overflow protection
			bufWrite(outBuffer, &bufcount, threshold, 2);
			bufWrite(outBuffer, &bufcount, thisrate, 4);
		}

		if ( (panic_mode == 1)&&(threshold > (THRESHOLDEND-THRESHOLDSTEP)) ){
			setPreampThreshold(channel, initThreshold);
			bufWrite(outBuffer, &bufcount, initThreshold, 2);
			bufWrite(outBuffer, &bufcount, 0, 4);
			break;
		}

		if (thisrate == 0){
			if (panic_mode == 0) zerocounts += 1;
			if (nonzero==0){
				if (zerocounts==15){
					//if never see a rate for 15 iterations, go to panic mode
					//and scan the whole range
					panic_mode = 1;
					threshold = THRESHOLDSTART-THRESHOLDSTEP;
					zerocounts = 0;
				}
			}
			else {
				if (zerocounts==3){//saw a rate at one point but change to 0 again for 3 times, use the last non-zero threshold
					setPreampThreshold(channel, nonzero);
					bufWrite(outBuffer, &bufcount, nonzero, 2);
					bufWrite(outBuffer, &bufcount, nonzerorate, 4);
					break;
				}
			}
		}
		else {
			if (panic_mode == 0){
				nonzero = threshold;
				nonzerorate = thisrate;
				zerocounts = 0;
			}
			else{
				panic_mode = 0;
			}
		}

		if (lastrate!=10000000){
			if ( ((lastrate<=target_rate)&&(thisrate>target_rate))||
					((lastrate>target_rate)&&(thisrate<=target_rate)) ){
				if (threshold>lastThreshold){
					threshold = lastThreshold;
					thisrate = lastrate;
				}
				setPreampThreshold(channel, threshold);
				bufWrite(outBuffer, &bufcount, threshold, 2);
				bufWrite(outBuffer, &bufcount, thisrate, 4);
				break;
			}
		}

		lastThreshold = threshold;
		lastrate = thisrate;

		if (panic_mode == 0){
			if (thisrate > target_rate){
				threshold-=1;
				setPreampThreshold(channel, threshold);
				continue;
			}
			if (thisrate <= target_rate){
				threshold+=1;
				setPreampThreshold(channel, threshold);
				continue;
			}
		}
		else{
			threshold+=THRESHOLDSTEP;
			setPreampThreshold(channel, threshold);
		}
	}
	if (verbose==1)
		bufWrite(outBuffer, &local_bufcount_place_holder, (bufcount-local_bufcount_place_holder-2)/6, 2);
}
