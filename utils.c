#include "utils.h"

#include "hw_platform.h"
#include "setup.h"

// channel_map[index adc channel] = straw number
// adc_map[index adc channel order] = adc_number

uint16_t default_gains_cal[96] = {370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370};
uint16_t default_gains_hv[96] = {370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370,370};
uint16_t default_threshs_cal[96] = {420,436,410,464,415,437,446,419,417,444,412,415,437,415,418,423,420,446,442,434,438,458,420,425,449,428,446,430,452,454,439,445,420,458,427,417,423,414,447,459,433,418,455,436,412,411,414,409,419,423,450,426,320,448,399,421,414,447,457,424,456,413,411,449,440,417,424,448,418,412,457,454,409,431,437,455,416,432,446,449,445,443,424,436,408,442,443,441,422,438,440,436,457,439,418,437};
uint16_t default_threshs_hv[96] = {443,430,426,441,444,451,448,436,417,437,439,435,446,442,448,424,412,444,442,447,449,440,452,443,443,437,440,435,426,437,441,440,431,449,453,444,452,435,446,432,440,445,435,434,439,437,439,434,440,434,427,423,320,432,445,430,434,437,436,442,436,436,433,455,443,442,443,445,424,434,440,439,442,437,450,419,440,446,453,450,434,433,451,434,424,438,428,429,443,425,434,429,434,456,438,456};






uint8_t hvcal=1;//1 CAL 2 HV 0 both
uint8_t channel_map[96]={
		91,85,79,73,67,61,55,49,
		43,37,31,25,19,13,7,1,
		90,84,78,72,66,60,54,48,
		42,36,30,24,18,12,6,0,
		93,87,81,75,69,63,57,51,
		45,39,33,27,21,15,9,3,
	
44, 38, 32, 26, 20, 14,  8,  2,
92, 86, 80, 74, 68, 62, 56, 50,
47, 41, 35, 29, 23, 17, 11,  5,
95, 89, 83, 77, 71, 65, 59, 53,
46, 40, 34, 28, 22, 16, 10,  4,
94, 88, 82, 76, 70, 64, 58, 52};

uint8_t adc_map[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
//uint8_t adcclk_map[12] = {1,1,2,3,3,3,6,6,6,9,11,11};
//uint8_t adcclk_map[12] = {1,1,2,3,3,3,6,6,8,9,9,9};

uint8_t adc_phases[12] = {0};

char outBuffer[2000]; // buffer for printing to serial port
char dataBuffer[4096];
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
int readout_noUARTflag;

int calibration_count[32];
uint32_t calibration_done;


UART_instance_t g_uart;
spi_instance_t g_spi[5];
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

void hwdelay (uint32_t tdelay)//20ns per count
{
	volatile uint32_t retv = 0;
	*(registers_0_addr + REG_TIMERRESET) = 1;

	while (retv < tdelay)
		retv = *(registers_0_addr + REG_TIMERCOUNTER);

	*(registers_0_addr + REG_TIMERRESET) = 0;
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

void delay_ms(uint32_t ms)
{
//    volatile uint32_t delay_count = SYS_CLK_FREQ / 1000 * ms;
//
//    while(delay_count > 0u)
//    {
//        --delay_count;
//    }
	hwdelay (TICKPERUS * ms * 1000);
}

void delayUs(int us)
{
//	volatile uint32_t delay_count = SYS_CLK_FREQ / 1000000 * us;
//
//	while(delay_count > 0u)
//	{
//		--delay_count;
//	}
	hwdelay (TICKPERUS * us);
}

//void delayTicks(uint8_t ticks)
//{
//	volatile uint8_t delay_count = ticks;
//	while(delay_count > 0u)
//	{
//		--delay_count;
//	}
//}

//void delayCore(uint32_t cycles)
//{
//	asm volatile (
//			"MOV r0, %[cycles]\n\t"/* load value*/
//			"1:\n\t"
//			"SUB r0, #1\n\t"
//			"CMP r0, #0\n\t"
//			"BNE 1b\n\t"
//			: /* no outputs */
//			: [cycles] "r" (cycles)
//			: "r0"/*clobber list to prevent using these registers*/
//			);
//	return;
//}
//in-line assembly implementation of delay loop.
//Nominal 50MHz CPU(?) core rate, debugging mode runs 1.25x nominal time, sNvM ~3.75x

uint32_t DCS_pass_sim_param(uint8_t dtc_sim_en, uint8_t dtc_output, uint8_t dtc_opcode_or_seq_num, uint8_t dtc_packet_type_or_marker_type){
	uint32_t dtc_sim_param = 0;
	if (dtc_output == 0)
		dtc_sim_param = (dtc_sim_en<<28) | (dtc_output<<24) | (dtc_opcode_or_seq_num<<16) |  dtc_packet_type_or_marker_type;
	else
		dtc_sim_param = (dtc_sim_en<<28) | (dtc_output<<24) | (dtc_opcode_or_seq_num<<8) | (dtc_packet_type_or_marker_type<<4);
	*(registers_0_addr + REG_ROC_DTC_SIM_PARAM) = dtc_sim_param;
	return dtc_sim_param;
}

uint32_t DCS_pass_addr_data(uint16_t lsb, uint16_t msb, uint8_t is_data){
	uint32_t roc_addr = (is_data) ? REG_ROC_DTC_SIM_DATA : REG_ROC_DTC_SIM_ADDR;
	uint32_t content = (msb<<16) | (lsb & 0x0000FFFF);
	*(registers_0_addr + roc_addr) = content;
	return content;
}

void DCS_sim_packet_send(){
	delay_ms(1);
	*(registers_0_addr + REG_ROC_DTC_SIM_START) = 1;
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
	volatile uint32_t * init_p_both;

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
	if (hvcal==0){
		busy_p_cal = registers_0_addr+REG_ROC_CAL_BUSY_P;
		data_p_cal = registers_0_addr+REG_ROC_CAL_DATA_P;
		address_p_cal = registers_0_addr+REG_ROC_CAL_ADDRESS_P;
		init_p_cal = registers_0_addr+REG_ROC_CAL_INIT_P;

		*(data_p_cal) = data;
		*(address_p_cal) = address;
		busy_p_hv = registers_0_addr+REG_ROC_HV_BUSY_P;
		data_p_hv = registers_0_addr+REG_ROC_HV_DATA_P;
		address_p_hv = registers_0_addr+REG_ROC_HV_ADDRESS_P;
		init_p_hv = registers_0_addr+REG_ROC_HV_INIT_P;

		init_p_both = registers_0_addr+REG_ROC_BOTH_INIT_P;

		*(data_p_hv) = data;
		*(address_p_hv) = address;
		*(init_p_both) = 1;
		while (*(busy_p_hv) != 0){};
		while (*(busy_p_cal) != 0){};

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
    uint8_t real_channel = 0;
    for (size_t i=0;i<96;i++){
        if (channel_map[i] == channel){
            real_channel = i;
            break;
        }
    }
    uint8_t fpga = 1;
    if (real_channel >= 48){
        fpga = 2;
        real_channel -= 48;
    }
    digi_write(DG_ADDR_HISTO_CHANNEL,((real_channel << 1) | (hv_or_cal & 0x1)),fpga);
	// tell digi to write histogram to sram
	digi_write(DG_ADDR_HISTO_RESET,0x1,fpga);
	delayUs(1);
	for (int i=0;i<256;i++){
	    output[i] = digi_read(DG_ADDR_HISTO_VAL,fpga);
	    digi_write(DG_ADDR_HISTO_RESET,0x3,fpga);
	    digi_write(DG_ADDR_HISTO_RESET,0x1,fpga);
	    delayUs(1);
	}
	digi_write(DG_ADDR_HISTO_RESET,0x0,fpga);
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

	delayUs(4);

	// send instructions

	for (int i=15;i>=0;i--){
		uint8_t thisbit;
		if ((instructions & (0x1<<i)) != 0x0)
			thisbit = 1;
		else
			thisbit = 0;
		digi_write(DG_ADDR_SPI_SDIO, thisbit, hvcal);
		delayUs(4);
		digi_write(DG_ADDR_SPI_SCLK, 1, hvcal);
		delayUs(4);
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
			delayUs(4);
			// if reading now read this bit
			if (rw == 1){
				uint32_t readval = digi_read(DG_ADDR_SPI_SDIO, hvcal);
				uint16_t dataval = (0x1<<i);
				if (readval != 0x0)
					data[ibyte] |= dataval;
			}
			digi_write(DG_ADDR_SPI_SCLK, 1, hvcal);
			delayUs(4);
			digi_write(DG_ADDR_SPI_SCLK, 0, hvcal);
		}
	}

	delayUs(4);
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

#define max_buffer 10

void read_data(int *delay_count, int *trigger_count)
{

	volatile uint32_t * ewm = registers_0_addr + REG_ROC_EWM_SINGLE;
	//for (int i=0;i<1000;i++){
	//	*ewm = 1;
	//	*ewm = 0;
	//	delayTicks(100);
	//}
	uint32_t memlevel = 0;

	uint32_t rdcnt = REG_ROC_FIFO_RDCNT;
	uint32_t re = REG_ROC_FIFO_RE;
	uint32_t data_reg = REG_ROC_FIFO_DATA;

	if (readout_noUARTflag == 1){
		// wait until we have enough data in memory to read out all the triggers we need
		// or until we hit max delay, whichever comes first
		uint32_t requiredMemLevel = readout_wordsPerTrigger*readout_numTriggers;
		if (requiredMemLevel > 64000){
			requiredMemLevel = 64000 - (64000 % readout_wordsPerTrigger);
		}


		// MAKE SURE ALL CHANNEL MASKS ARE ENABLED
		// NO UART TRANSMISSION GOING NOW
		digi_write(DG_ADDR_MASK1,(uint16_t) (mapped_channel_mask[0] & 0xFFFF), 1);
		digi_write(DG_ADDR_MASK2,(uint16_t) ((mapped_channel_mask[0] & 0xFFFF0000)>>16), 1);
		digi_write(DG_ADDR_MASK3,(uint16_t) (mapped_channel_mask[1] & 0xFFFF), 1);
		digi_write(DG_ADDR_MASK1,(uint16_t) ((mapped_channel_mask[1] & 0xFFFF0000)>>16), 2);
		digi_write(DG_ADDR_MASK2,(uint16_t) (mapped_channel_mask[2] & 0xFFFF), 2);
		digi_write(DG_ADDR_MASK3,(uint16_t) ((mapped_channel_mask[2] & 0xFFFF0000)>>16), 2);
		while (1){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
			memlevel = fifoinfo & 0x1FFFF;
			// now only allow readout of as many triggers as currently in memory
			if (memlevel < requiredMemLevel){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay){
					// we delayed as much as we can
					break;
				}
				delayUs(1000);
				continue;
			}else{
				break;
			}
		}
		// disable more triggers during uart readout

		digi_write(DG_ADDR_MASK1,0x0, 1);
		digi_write(DG_ADDR_MASK2,0x0, 1);
		digi_write(DG_ADDR_MASK3,0x0, 1);
		digi_write(DG_ADDR_MASK1,0x0, 2);
		digi_write(DG_ADDR_MASK2,0x0, 2);
		digi_write(DG_ADDR_MASK3,0x0, 2);
	}

	// loop until read enough triggers or delay times out
	while (1){

		if (memlevel < readout_wordsPerTrigger){
			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
			memlevel = fifoinfo & 0x1FFFF;
			if (memlevel < readout_wordsPerTrigger){
				(*delay_count)++;
				if ((*delay_count) >= readout_maxDelay){
					// if not doing continuous readout, tell python to end the run now
					//if (readout_mode != 1){
						readout_obloc = 0;
						bufWrite(dataBuffer, &readout_obloc, EMPTY, 2);
						UART_send(&g_uart, dataBuffer ,2);
					//}
					break;
				}
				delayUs(1000);
				continue;
			}
		}


		// have enough data, read from FPGA FIFO

		readout_obloc = 0;
		bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
		bufWrite(dataBuffer, &readout_obloc, 4*readout_wordsPerTrigger,2);

		for (int j=0;j<readout_wordsPerTrigger;j++){
			volatile uint32_t digioutput;
			*(registers_0_addr + re) = 1;
			digioutput = *(registers_0_addr + data_reg);
			memlevel -= 1;
			bufWrite(dataBuffer, &readout_obloc, ((digioutput & 0xFFFF0000)>>16), 2);
			bufWrite(dataBuffer, &readout_obloc, ((digioutput & 0xFFFF)), 2);
		}

		UART_send(&g_uart, dataBuffer ,readout_obloc);
		(*trigger_count)++;


		if ((*trigger_count) >= readout_numTriggers){
			readout_obloc = 0;
			bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
			UART_send(&g_uart, dataBuffer ,2);
			break;
		}
	}
}

//void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit)
//{
//	uint32_t memlevel = 0;
//	//int fail_count = 0;
//
//	uint32_t rdcnt = REG_ROC_FIFO_RDCNT;
//	uint32_t re = REG_ROC_FIFO_RE;
//	uint32_t data_reg = REG_ROC_FIFO_DATA;
//
//	while (1){
//		if (memlevel < readout_wordsPerTrigger){
//			volatile uint32_t fifoinfo = *(registers_0_addr + rdcnt);
//			memlevel = fifoinfo & 0x1FFFF;
//			if (memlevel < readout_wordsPerTrigger){
//				(*delay_count)++;
//				if ((*delay_count) >= readout_maxDelay)
//					break;
//				delayUs(1000);
//				continue;
//			}
//		}
//
//		// have enough data, see if can read
//		//int failed = 0;
//		for (int j=0;j<readout_wordsPerTrigger;j++){
//
//			volatile uint32_t digioutput;
//			*(registers_0_addr + re) = 1;
//			digioutput = *(registers_0_addr + data_reg);
//			memlevel -= 1;
//			//if ((j == 0) && ((digioutput&0x80000000) != 0x80000000)){
//			//	failed = 1;
//			//	fail_count++;
//			//	break;
//			//}
//			lasthit[j] = (uint16_t) ((digioutput & 0x3FF00000)>>20);
//		}
//		//if (fail_count >= readout_wordsPerTrigger*10)
//		//	break;
//		//if (failed)
//		//	continue;
//		(*trigger_count)++;
//		if ((*trigger_count) >= readout_numTriggers)
//			break;
//	}
//}

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
			hwdelay((uint32_t)num_delays*50);//delayUs(num_delays);
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

//void bufWriteN(char *outBuffer, uint16_t bufaddr, uint32_t data, uint16_t nbytes){
//	for (uint8_t i=0; i<nbytes; i++){
//		outBuffer[bufaddr+i] = (data>>(i*8)) & 0xff;
//	}
//}

void outBufSend(UART_instance_t g_uart, char *outBuffer, uint16_t bufcount){
	UART_polled_tx_string( &g_uart, "monitoring\n" );
	UART_send(&g_uart, outBuffer ,bufcount );
}

int resetFIFO(){
	digi_write(DG_ADDR_RESET, 0, 0); // digi 0x10, sends tdc_reset_n which resets all fifos and TDCs in old version
	*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 0; //0xA3 (ROC reset_fifo_n, clears DigiInterface fifos and resets DigiController)
	delayUs(10);
	*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 1; //0xA3 (ROC reset_fifo_n, clears DigiInterface fifos and resets DigiController)
	delayUs(10);
	digi_write(DG_ADDR_RESET, 1, 0); // digi 0x10, sends tdc_reset_n which resets all fifos and TDCs in old version
	return 0;
	/*
//	for (int i=0;i<10;i++){
	  digi_write(DG_ADDR_RESET, 0, 0);
	  *(registers_0_addr + REG_ROC_FIFO_RESET) = 1; //0x44 (ROC DIGI_RESET, sends device reset when set to 0)
	  digi_write(DG_ADDR_RESET, 1, 0);

	  delayUs(1);
	  //uint16_t is_aligned = *(registers_0_addr + REG_ROC_SERDES_ALIGN);
	  //if (is_aligned == 0xF){
//		*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 0;
//		*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 1;
		return 0;
//	  }
//	}
	// failed to align correctly
//	*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 0;
//	*(registers_0_addr + REG_ROC_CR_FIFO_RESET) = 1;
//	return 1;
//	reset_fabric();
 */
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

void init_DIGIs(){
	for (uint8_t i=0;i<12;i++){

		adc_write(ADC_ADDR_PHASE,0,(0x1<<i));
		adc_write(ADC_ADDR_TESTIO,0,(0x1<<i));
	}

	digi_write(DG_ADDR_SAMPLE,10,0);
	digi_write(DG_ADDR_LOOKBACK,5,0);
	digi_write(DG_ADDR_MASK1,0xFFFF, 1);
	digi_write(DG_ADDR_MASK2,0xFFFF, 1);
	digi_write(DG_ADDR_MASK3,0xFFFF, 1);
	digi_write(DG_ADDR_MASK1,0xFFFF, 2);
	digi_write(DG_ADDR_MASK2,0xFFFF, 2);
	digi_write(DG_ADDR_MASK3,0xFFFF, 2);
	digi_write(DG_ADDR_TRIGGER_MODE,0,0);
	digi_write(DG_ADDR_ENABLE_PULSER,1,0);

	*(registers_0_addr + REG_ROC_EWW_PULSER) = 0;
	//*(registers_0_addr + REG_ROC_EWM_T) = max_total_delay;
	*(registers_0_addr + REG_ROC_EWM_T) = 0x0FFF;



	*(registers_0_addr + REG_ROC_FIFO_HOWMANY) = 10;

	delayUs(100);

	// reset fifo
	resetFIFO();

	*(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

}


//void findChThreshold(int num_delays, int num_samples, uint16_t channel, uint16_t target_rate, uint8_t verbose){
//	uint16_t threshold = 0;
//	uint32_t timecounts = 0;
//	if (channel < 96)
//		threshold = default_threshs_cal[channel];
//	else
//		threshold = default_threshs_hv[channel-96];
//	uint32_t lastrate = 10000000;
//	uint32_t thisrate = 0;
//	uint16_t initThreshold = threshold;
//	uint16_t lastThreshold = threshold;
//	uint8_t zerocounts = 0;
//	uint16_t nonzero = 0;
//	uint32_t nonzerorate = 0;
//	uint8_t panic_mode = 0;
//
//	uint16_t local_bufcount_place_holder = bufcount;
//	if (verbose==1)
//		bufWrite(outBuffer, &bufcount, 0, 2);
//
//	while(1){
//		uint32_t thiscount = get_rates(num_delays, num_samples, channel, &timecounts);
//		thisrate = (uint32_t)(((uint64_t)thiscount)*50000000/timecounts);
//
//		if ((verbose==1)&&(bufcount<1950)){//buffer overflow protection
//			bufWrite(outBuffer, &bufcount, threshold, 2);
//			bufWrite(outBuffer, &bufcount, thisrate, 4);
//		}
//
//		if ( (panic_mode == 1)&&(threshold > (THRESHOLDEND-THRESHOLDSTEP)) ){
//			setPreampThreshold(channel, initThreshold);
//			bufWrite(outBuffer, &bufcount, initThreshold, 2);
//			bufWrite(outBuffer, &bufcount, 0, 4);
//			break;
//		}
//
//		if (thisrate == 0){
//			if (panic_mode == 0) zerocounts += 1;
//			if (nonzero==0){
//				if (zerocounts==15){
//					//if never see a rate for 15 iterations, go to panic mode
//					//and scan the whole range
//					panic_mode = 1;
//					threshold = THRESHOLDSTART-THRESHOLDSTEP;
//					zerocounts = 0;
//				}
//			}
//			else {
//				if (zerocounts==3){//saw a rate at one point but change to 0 again for 3 times, use the last non-zero threshold
//					setPreampThreshold(channel, nonzero);
//					bufWrite(outBuffer, &bufcount, nonzero, 2);
//					bufWrite(outBuffer, &bufcount, nonzerorate, 4);
//					break;
//				}
//			}
//		}
//		else {
//			if (panic_mode == 0){
//				nonzero = threshold;
//				nonzerorate = thisrate;
//				zerocounts = 0;
//			}
//			else{
//				panic_mode = 0;
//			}
//		}
//
//		if ((lastrate!=10000000)&&(lastrate!=0)){
//			if ( ((lastrate<=target_rate)&&(thisrate>target_rate))||
//					((lastrate>target_rate)&&(thisrate<=target_rate)) ){
//				if (threshold>lastThreshold){
//					threshold = lastThreshold;
//					thisrate = lastrate;
//				}
//				setPreampThreshold(channel, threshold);
//				bufWrite(outBuffer, &bufcount, threshold, 2);
//				bufWrite(outBuffer, &bufcount, thisrate, 4);
//				break;
//			}
//		}
//
//		lastThreshold = threshold;
//		lastrate = thisrate;
//
//		if (panic_mode == 0){
//			if (thisrate > target_rate){
//				threshold-=1;
//				setPreampThreshold(channel, threshold);
//				continue;
//			}
//			if (thisrate <= target_rate){
//				threshold+=1;
//				setPreampThreshold(channel, threshold);
//				continue;
//			}
//		}
//		else{
//			threshold+=THRESHOLDSTEP;
//			setPreampThreshold(channel, threshold);
//		}
//	}
//	if (verbose==1)
//		bufWrite(outBuffer, &local_bufcount_place_holder, (bufcount-local_bufcount_place_holder-2)/6, 2);
//}
