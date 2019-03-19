#include "utils.h"
#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"
#include "hw_platform.h"

// channel_map[index adc channel] = straw number
// adc_map[index adc channel order] = adc_number

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
	volatile uint32_t *busy_p;
	volatile uint32_t *data_p;
	volatile uint32_t *address_p;
	volatile uint32_t *init_p;

	if ((hvcal==1)||(hvcal=0)){
		busy_p = registers_0_addr+REG_ROC_CAL_BUSY_P;
		data_p = registers_0_addr+REG_ROC_CAL_DATA_P;
		address_p = registers_0_addr+REG_ROC_CAL_ADDRESS_P;
		init_p = registers_0_addr+REG_ROC_CAL_INIT_P;

		*(data_p) = data;
		*(address_p) = address;
		*(init_p) = 1;
		while (*(busy_p) != 0){};
	}
	if ((hvcal==2)||(hvcal=0)){
		busy_p = registers_0_addr+REG_ROC_HV_BUSY_P;
		data_p = registers_0_addr+REG_ROC_HV_DATA_P;
		address_p = registers_0_addr+REG_ROC_HV_ADDRESS_P;
		init_p = registers_0_addr+REG_ROC_HV_INIT_P;

		*(data_p) = data;
		*(address_p) = address;
		*(init_p) = 1;
		while (*(busy_p) != 0){};
	}
}

uint16_t digi_read(uint8_t address, uint8_t hvcal)//hvcal can only be 1 or 2
{
	volatile uint32_t *busy_p;
	volatile uint32_t *data_p;
	volatile uint32_t *address_p;
	volatile uint32_t *init_p;

	if (hvcal!=1 && hvcal!=2)
		hvcal = 1;//for safety.

	if (hvcal==1){
		busy_p = registers_0_addr+REG_ROC_CAL_BUSY_P;
		data_p = registers_0_addr+REG_ROC_CAL_DATA_P;
		address_p = registers_0_addr+REG_ROC_CAL_ADDRESS_P;
		init_p = registers_0_addr+REG_ROC_CAL_INIT_P;

		*(address_p) = (0x1<<8) | address;
		*(init_p) = 1;
		while (*(busy_p) != 0){};
	}
	if (hvcal==2){
		busy_p = registers_0_addr+REG_ROC_HV_BUSY_P;
		data_p = registers_0_addr+REG_ROC_HV_DATA_P;
		address_p = registers_0_addr+REG_ROC_HV_ADDRESS_P;
		init_p = registers_0_addr+REG_ROC_HV_INIT_P;

		*(address_p) = (0x1<<8) | address;
		*(init_p) = 1;
		while (*(busy_p) != 0){};
	}
	return *(data_p);
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

void read_data(int *delay_count, int *trigger_count, uint8_t hvcal)
{
	uint16_t memlevel = 0;

	uint32_t rdcnt = REG_ROC_CAL_RDCNT;
	uint32_t re = REG_ROC_CAL_RE;
	uint32_t data_reg = REG_ROC_CAL_DATA;

	if (hvcal==2){
		rdcnt = REG_ROC_HV_RDCNT;
		re = REG_ROC_HV_RE;
		data_reg = REG_ROC_HV_DATA;
	}

	while (1){

		readout_obloc = 0;

		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
			memlevel = fifoinfo & 0x7FFF;
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

			uint16_t digioutput;
			*(registers_0_addr + re) = 1;
			digioutput = *(registers_0_addr + data_reg);
			memlevel -= 1;
			//sprintf(&dataBuffer[readout_obloc],"%04x ",digioutput);
			bufWrite(dataBuffer, &readout_obloc, digioutput, 2);
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

void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit, uint8_t hvcal)
{
	uint16_t memlevel = 0;
	int fail_count = 0;

	uint32_t rdcnt = REG_ROC_CAL_RDCNT;
	uint32_t re = REG_ROC_CAL_RE;
	uint32_t data_reg = REG_ROC_CAL_DATA;

	if (hvcal==2){
		rdcnt = REG_ROC_HV_RDCNT;
		re = REG_ROC_HV_RE;
		data_reg = REG_ROC_HV_DATA;
	}

	while (1){
		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
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
		int failed = 0;
		for (int j=0;j<readout_wordsPerTrigger;j++){
			uint16_t digioutput;
			*(registers_0_addr + re) = 1;
			digioutput = *(registers_0_addr + data_reg);
			memlevel -= 1;
			if ((j == 0) && ((digioutput&0x1000) != 0x1000)){
				failed = 1;
				fail_count++;
				break;
			}
			lasthit[j] = digioutput;
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

void get_rates(int num_delays, int num_samples)
{
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
			total_time_counts[ihvcal-1] += (uint32_t) (end_global_time - start_global_time);

			for (uint8_t k=(48*(ihvcal-1));k<48*ihvcal;k++){
				uint8_t condition = 0;
				uint8_t straw_num = channel_map[k];
				ishv[straw_num] = ihvcal-1;

				if (ihvcal == 1)
					condition =((k<32 && ((0x1<<k) & mapped_channel_mask[0])) || (k>=32 && ((0x1<<(k-32)) & mapped_channel_mask[1])));
				else if (ihvcal == 2)
					condition =((k<64 && ((0x1<<(k-32)) & mapped_channel_mask[1])) || (k>=64 && ((0x1<<(k-64)) & mapped_channel_mask[2])));

				if (condition){

					digi_write(DG_ADDR_READCHANNEL, k, ihvcal);
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

	for (int i=0;i<96;i++){
		if ( ((0x1<<(i%32)) & channel_mask[0])||
				((0x1<<(i%32)) & channel_mask[1])||
				((0x1<<(i%32)) & channel_mask[2]) ){
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

void bufWrite(char *outBuffer, uint16_t *bufcount, uint32_t data, uint8_t nbytes){
	for (uint8_t i=0; i<nbytes; i++){
		outBuffer[(*bufcount)++] = (data>>(i*8)) & 0xff;
	}
}

void outBufSend(UART_instance_t g_uart, char *outBuffer, uint16_t bufcount){
	UART_polled_tx_string( &g_uart, "monitoring\n" );
	UART_send(&g_uart, outBuffer ,bufcount );
}

void resetFIFO(uint8_t hvcal){
	digi_write(DG_ADDR_RESET, 0, hvcal);
	if (hvcal==1)
		*(registers_0_addr + REG_ROC_CAL_RESET) = 1;
	else if (hvcal==2)
		*(registers_0_addr + REG_ROC_HV_RESET) = 1;
	digi_write(DG_ADDR_RESET, 1, hvcal);
//	reset_fabric();
}
