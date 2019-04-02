/* Kernel includes. */
#include "hw_platform.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CoreSysServices_PF/core_sysservices_pf.h"

#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"

#include "setup.h"
#include "Commands.h"

#include "utils.h"
#include "version.h"

#define BAUD_VALUE                  57600
#define ENABLED_ADCS				0xfffu

uint16_t default_gains_cal[96] = {271,275,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,269,269,269,269,269,269,269,268,269,269,269,264,269,269,269,270};
uint16_t default_gains_hv[96] = {270,271,270,270,270,270,270,270,270,270,270,270,270,270,266,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,267,270,270,270,270,266,266,272,268,266,265,269,269,269,269,270};
uint16_t default_threshs_cal[96] = {331,316,339,309,317,317,319,335,341,321,319,359,347,320,323,332,352,334,325,339,304,312,340,330,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,305,294,311,307,340,332,321,336,335,327,322,356,344,304,324,325,315,325,325,341,348,343,343,319};
uint16_t default_threshs_hv[96] = {322,284,329,317,325,321,340,347,351,339,348,349,313,337,327,325,345,325,321,320,345,332,305,335,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,284,293,324,321,324,331,344,333,314,324,333,346,330,335,327,329,349,316,331,339,318,304,332,310};


const uint16_t default_caldac[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

const uint8_t calpulse_chanmap[8]={9,1,10,2,11,3,12,4};

const uint8_t default_delay = 200;

const uint8_t MCPCALIBCHAN[8] = {1,2,3,4,9,10,11,12};

int main()
{
	SystemCoreClockUpdate();
	UART_init( &g_uart, UART_BASE_ADDRESS, (SYS_M1_CLK_FREQ/(16 * BAUD_VALUE))-1, (DATA_8_BITS | NO_PARITY));

	const uint8_t greeting[] = "\n\r\"Welcome to the ROC\"\n"
			"\t - Sean Connery\n";

	/* Send greeting message over the UART_0 */
	UART_polled_tx_string( &g_uart, greeting );
	//		sprintf(outBuffer,"Last git revision: %s\n",LAST_GIT_REV);
	//			UART_polled_tx_string( &g_uart, outBuffer );

	GPIO_init( &g_gpio,    COREGPIO_BASE_ADDR, GPIO_APB_32_BITS_BUS );
	//	GPIO_config( &g_gpio, GPIO_0, GPIO_OUTPUT_MODE);
	GPIO_set_output( &g_gpio, GPIO_0, 0);


	uint8_t readout_enabled = 0;
	uint8_t calibration_enabled = 0;
	int readout_totalTriggers = 0;

	//register address for bit banging
	registers_0_addr = (volatile uint32_t *) REGISTERBASEADDR;

	uint8_t errors = init_adc(ENABLED_ADCS,0x02,0x03);

	/*Initialize the CoreSysService_PF driver*/
	SYS_init(CSS_PF_BASE_ADDRESS);

	uint16_t loopCount = 0;
	uint8_t ledPattern = 0x1;
	_Bool togglecal = 1;

	//sPI initialization -- make sure the memory map in the smart design is contiguous and the mapping works

	//	for (int ispi = 0; ispi<4; ispi++){
	//		unsigned int addr = ispi * 0x00001000UL;
	//		SPI_init( &g_spi[ispi], SPI0_BASE_ADDR + addr, 8 );
	//		SPI_configure_master_mode( &g_spi[ispi] );
	//	}

	SPI_init( &g_spi[0], SPI0_BASE_ADDR, 8 );
	SPI_configure_master_mode( &g_spi[0] );
	SPI_init( &g_spi[1], SPI1_BASE_ADDR, 8 );
	SPI_configure_master_mode( &g_spi[1] );
	SPI_init( &g_spi[2], HVSPI_BASE_ADDR, 8 );
	SPI_configure_master_mode( &g_spi[2] );
	SPI_init( &g_spi[3], CALSPI_BASE_ADDR, 8 );
	SPI_configure_master_mode( &g_spi[3] );


	//setup MCPs
	for (int imcp = MCPCAL0; imcp<=MCPHV3; imcp++){
		if (imcp < MCPHV0)
			MCP_setup(&preampMCP[imcp], g_spi[3], 0 , 0x20 + imcp);
		else
			MCP_setup(&preampMCP[imcp], g_spi[2], 0 , 0x20 + imcp - MCPHV0);
	}


	// outputs for calpulse enable

	for (uint8_t imcp = 0; imcp < 8; imcp++){
		MCP_pinMode(&preampMCP[MCPCALIB], MCPCALIBCHAN[imcp], MCP_OUTPUT);
	}


	//setup LTC2634, preamp DACs
	for (uint8_t idac = 0 ; idac < 96; idac++){
		if (idac<14)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPCAL0],idac+3,&preampMCP[MCPCAL0],2,&preampMCP[MCPCAL0],1);
		else if (idac<24)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPCAL1],idac-13,&preampMCP[MCPCAL0],2,&preampMCP[MCPCAL0],1);
		else if (idac<38)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPCAL2],idac-21,&preampMCP[MCPCAL2],2,&preampMCP[MCPCAL2],1);
		else if (idac<48)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPCAL3],idac-37,&preampMCP[MCPCAL2],2,&preampMCP[MCPCAL2],1);
		else if (idac<62)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPHV0],idac-45,&preampMCP[MCPHV0],2,&preampMCP[MCPHV0],1);
		else if (idac<72)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPHV1],idac-61,&preampMCP[MCPHV0],2,&preampMCP[MCPHV0],1);
		else if (idac<86)
			LTC2634_setup(&dacs[idac],&preampMCP[MCPHV2],idac-69,&preampMCP[MCPHV2],2,&preampMCP[MCPHV2],1);
		else
			LTC2634_setup(&dacs[idac],&preampMCP[MCPHV3],idac-85,&preampMCP[MCPHV2],2,&preampMCP[MCPHV2],1);
	}


	// Set default thresholds and gains
	for (uint8_t i = 0 ; i < 96 ; i++){
		strawsCal[i]._ltc = &dacs[i/2];
		strawsHV[i]._ltc = &dacs[48+i/2];
		if (i%2 == 0){
			strawsCal[i]._thresh = 1;
			strawsCal[i]._gain = 2;
			strawsHV[i]._thresh = 0;
			strawsHV[i]._gain = 2;
		}else{
			strawsCal[i]._thresh = 0;
			strawsCal[i]._gain = 3;
			strawsHV[i]._thresh = 1;
			strawsHV[i]._gain = 3;
		}
		LTC2634_write(strawsCal[i]._ltc,strawsCal[i]._gain,default_gains_cal[i]);
		LTC2634_write(strawsCal[i]._ltc,strawsCal[i]._thresh,default_threshs_cal[i]);
		LTC2634_write(strawsHV[i]._ltc,strawsHV[i]._gain,default_gains_hv[i]);
		LTC2634_write(strawsHV[i]._ltc,strawsHV[i]._thresh,default_threshs_hv[i]);
	}

	//adc_write(0x08,0x00,0x3F);

	adc_write(ADC_ADDR_PWR,0x01,ENABLED_ADCS);
	adc_write(ADC_ADDR_PWR,0x00,ENABLED_ADCS);

	digi_write(DG_ADDR_BITSLIP0,0x0,0);
	digi_write(DG_ADDR_BITSLIP1,0x0,0);
	digi_write(DG_ADDR_BITSLIP2,0x0,0);
	digi_write(DG_ADDR_BITSLIP3,0x0,0);
	digi_write(DG_ADDR_BITSLIP4,0x0,0);
	digi_write(DG_ADDR_BITSLIP5,0x0,0);

	digi_write(DG_ADDR_RESET,1,0);

	writePtr = 0;

	// set defaults

	I2C_setup(&i2c_ptscal[0], &preampMCP[MCPCAL0],1,&preampMCP[MCPCAL0],2);
	I2C_setup(&i2c_ptshv[0], &preampMCP[MCPHV0],1,&preampMCP[MCPHV0],2);


	struct bme280_dev ptscal;
	int8_t rslt = BME280_OK;
	ptscal.dev_id = BME280_I2C_ADDR_PRIM;
	ptscal.intf = BME280_I2C_INTF;
	ptscal._i2c = &i2c_ptscal[0];

	ptscal.delay_ms = delay_ms;

	bme280_init(&ptscal);
	uint8_t ptscalchipid = 0;
	bme280_get_regs(BME280_CHIP_ID_ADDR,&ptscalchipid,1,&ptscal);


	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	ptscal.settings.osr_h = BME280_OVERSAMPLING_1X;
	ptscal.settings.osr_p = BME280_OVERSAMPLING_16X;
	ptscal.settings.osr_t = BME280_OVERSAMPLING_2X;
	ptscal.settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &ptscal);


	//set the HV sensor
	struct bme280_dev ptshv;
	ptshv.dev_id = BME280_I2C_ADDR_PRIM;
	ptshv.intf = BME280_I2C_INTF;
	ptshv._i2c = &i2c_ptshv[0];

	ptshv.delay_ms = delay_ms;

	bme280_init(&ptshv);
	uint8_t ptshvchipid = 0;
	bme280_get_regs(BME280_CHIP_ID_ADDR,&ptshvchipid,1,&ptshv);


	//		sprintf(outBuffer,"Set up BME280 sensor for HV side %d\n",ptshvchipid);
	//		UART_polled_tx_string( &g_uart, outBuffer );


	/* Recommended mode of operation: Indoor navigation */
	ptshv.settings.osr_h = BME280_OVERSAMPLING_1X;
	ptshv.settings.osr_p = BME280_OVERSAMPLING_16X;
	ptshv.settings.osr_t = BME280_OVERSAMPLING_2X;
	ptshv.settings.filter = BME280_FILTER_COEFF_16;

	settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

	rslt = bme280_set_sensor_settings(settings_sel, &ptshv);



	//		sprintf(outBuffer,"Data Sensor HV: %d %d %d\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);
	//		UART_polled_tx_string( &g_uart, outBuffer );
	//
	*(registers_0_addr+12) = 1;
	for (uint8_t i=0;i<8;i++)
		AD5318_write(g_spi[3],1,i,default_caldac[i]);
	*(registers_0_addr+12) = 0;

	UART_polled_tx_string( &g_uart, "Initialization completed" );

	GPIO_set_output( &g_gpio, GPIO_0, 0);


	while(1)
	{
		if (readout_enabled){
			int delay_count = 0;
			int trigger_count = 0;
			UART_polled_tx_string( &g_uart, "datastream\n" );

			read_data(&delay_count,&trigger_count);
			readout_totalTriggers += trigger_count;
		}

		if (loopCount == 20000){
			ledPattern ^= 0x1;
			GPIO_set_output( &g_gpio, GPIO_0, (uint32_t)ledPattern);

			loopCount = 0;
		}
		delayUs(1);
		loopCount++;



		size_t rx_size =UART_get_rx(&g_uart, rx_buff, sizeof(rx_buff));
		for (uint16_t j=0;j<rx_size;j++){
			buffer[writePtr] = rx_buff[j];
			writePtr++;
		}

		if (writePtr <= 3)
			continue;
		if (writePtr > 0)
		{
			if ((buffer[0] != 0xAA) || buffer[1] != 0xAA){
				writePtr = 0;
				UART_polled_tx_string( &g_uart, "Problem synchronizing command\n" );
				continue;
			}

			uint8_t numBytes = buffer[2];
			if (writePtr >= (uint32_t)numBytes){
				uint8_t commandID = (uint8_t) buffer[3];
				bufcount = 0;

				//	    if (commandID < MAXCOM_MON)
				//	      UART_polled_tx_string( &g_uart, "monitoring\n" );

				if (commandID == SETPREAMPGAIN){
					uint16_t channel = readU16fromBytes(&buffer[4]);
					uint16_t value = readU16fromBytes(&buffer[6]);

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
					outBuffer[bufcount++] = SETPREAMPGAIN;
					bufWrite(outBuffer, &bufcount, 4, 2);
					bufWrite(outBuffer, &bufcount, channel, 2);
					bufWrite(outBuffer, &bufcount, value, 2);
					outBufSend(g_uart, outBuffer, bufcount);


					// Set preamp threshold
				}else if (commandID == SETPREAMPTHRESHOLD){
					uint16_t channel = readU16fromBytes(&buffer[4]);

					uint16_t value = readU16fromBytes(&buffer[6]);

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

					outBuffer[bufcount++] = SETPREAMPTHRESHOLD;
					bufWrite(outBuffer, &bufcount, 4, 2);
					bufWrite(outBuffer, &bufcount, channel, 2);
					bufWrite(outBuffer, &bufcount, value, 2);
					outBufSend(g_uart, outBuffer, bufcount);


				 }else if (commandID == SETCALDAC){

				 	uint8_t chan_mask = (uint8_t) buffer[4];
				 	uint16_t value = readU16fromBytes(&buffer[5]);
				 	*(registers_0_addr+REG_ROC_RE) = 1;
				 	for (uint8_t i=0;i<8;i++){
				 		if (chan_mask & (0x1<<i))
				 			AD5318_write(g_spi[3],1, i,value);
				 	}
				 	*(registers_0_addr+REG_ROC_RE) = 0;
				 	outBuffer[bufcount++] = SETCALDAC;
				 	bufWrite(outBuffer, &bufcount, 3, 2);
				 	outBuffer[bufcount++] = chan_mask;
				 	bufWrite(outBuffer, &bufcount, value, 2);
				 	outBufSend(g_uart, outBuffer, bufcount);


					//				}else if (commandID == SENDONECALPULSE){
					//					uint8_t chan_mask = (uint8_t) buffer[4];
					//
					//					for (int i=0;i<8;i++){
					//						if ((0x1<<i) & chan_mask){
					//							MCP_pinWrite(&preampMCP[MCPCALIB],calpulse_chanmap[i],1);
					//						}
					//					}
					//
					//					if (togglecal)
					//						GPIO_write(CAL_CALEVEN,1);
					//					else
					//						GPIO_write(CAL_CALEVEN,0);
					//
					//					togglecal = !togglecal;
					//
					//					outBuffer[bufcount++] = SENDONECALPULSE;
					//					outBuffer[bufcount++] = 1;
					//					outBuffer[bufcount++] = chan_mask;
					//
					//					UART_send(&g_uart, outBuffer ,bufcount );
					//

				}else if (commandID == SETPULSERON){


					for (uint8_t i = 0; i < 16; i++){
						MCP_pinWrite(&preampMCP[MCPCALIB],i+1,0);
					}
					uint8_t chan_mask = (uint8_t) buffer[4];
					pulserOdd = (uint8_t) buffer[5];
					for (uint8_t i=0;i<8;i++){
						if ((0x1<<i) & chan_mask){
							MCP_pinWrite(&preampMCP[MCPCALIB],calpulse_chanmap[i],1);
						}
					}
					dutyCycle=readU16fromBytes(&buffer[6]);
					pulserDelay=readU32fromBytes(&buffer[8]);

					//granularity is clock period=25ns -- period is (gr+1)*1000=50us
					//PWM_PERIOD = PWM_GRANULARITY * (period + 1) = 25 *1000 = 25us
					PWM_init( &g_pwm, COREPWM_BASE_ADDR, 1, pulserDelay );
					PWM_set_duty_cycle( &g_pwm, PWM_1,dutyCycle );//duty cycle is 4 x 25 = 100ns

					PWM_enable(&g_pwm,PWM_1);


					outBuffer[bufcount++] = SETPULSERON;
					bufWrite(outBuffer, &bufcount, 8, 2);
					outBuffer[bufcount++] = chan_mask;
					outBuffer[bufcount++] = pulserOdd;
					bufWrite(outBuffer, &bufcount, dutyCycle, 2);
					bufWrite(outBuffer, &bufcount, pulserDelay, 4);
					outBufSend(g_uart, outBuffer, bufcount);


				}else if (commandID == SETPULSEROFF){

					PWM_disable(&g_pwm,PWM_1);
					outBuffer[bufcount++] = SETPULSEROFF;
					bufWrite(outBuffer, &bufcount, 0, 2);
					outBufSend(g_uart, outBuffer, bufcount);

				 }else if (commandID == WHOAREYOU){

				 	outBuffer[bufcount++] = WHOAREYOU;
				 	bufWrite(outBuffer, &bufcount, 0, 2);
				 	outBufSend(g_uart, outBuffer, bufcount);

				 }else if (commandID == RESETROC){

					digi_write(DG_ADDR_RESET,0,0);
					digi_write(DG_ADDR_RESET,1,0);
					outBuffer[bufcount++] = RESETROC;
					bufWrite(outBuffer, &bufcount, 0, 2);
				 	outBufSend(g_uart, outBuffer, bufcount);

				 }else if (commandID == TESTDDR){
				 	uint8_t ddrcs = (uint8_t) buffer[4];
				 	uint8_t ddrwen = (uint8_t) buffer[5];
				 	uint8_t ddrren = (uint8_t) buffer[6];
				 	uint8_t ddrdmaen = (uint8_t) buffer[7];
				 	uint8_t ddrnhits = (uint8_t) buffer[8];
				 	uint8_t ddrpattern = (uint8_t) buffer[9];
				 	uint16_t ddrraddr = readU16fromBytes(&buffer[10]);
				 	uint32_t retv = 0xF;

				 	*(registers_0_addr + REG_ROC_DDR_NHITS) = ddrnhits;
				 	*(registers_0_addr + REG_ROC_DDR_OFFSET) = 0;
				 	*(registers_0_addr + REG_ROC_DDR_CS) = ddrcs;
				 	*(registers_0_addr + REG_ROC_DDR_WEN) = ddrwen;
				 	*(registers_0_addr + REG_ROC_DDR_REN) = ddrren;
				 	*(registers_0_addr + REG_ROC_DDR_DMAEN) = ddrdmaen;
				 	*(registers_0_addr + REG_ROC_DDR_PATTERN) = ddrpattern;
				 	*(registers_0_addr + REG_ROC_DDR_RADDR) = ddrraddr;
				 	retv = *(registers_0_addr + REG_ROC_DDR_ERR);
				 	uint32_t dataddr = *(registers_0_addr + REG_ROC_DDR_DATA);
				 	outBuffer[bufcount++] = TESTDDR;
				 	bufWrite(outBuffer, &bufcount, 16, 2);
				 	outBuffer[bufcount++] = ddrnhits;
				 	outBuffer[bufcount++] = ddrcs;
				 	outBuffer[bufcount++] = ddrwen;
				 	outBuffer[bufcount++] = ddrren;
				 	outBuffer[bufcount++] = ddrdmaen;
				 	outBuffer[bufcount++] = ddrpattern;
				 	bufWrite(outBuffer, &bufcount, retv, 4);
				 	bufWrite(outBuffer, &bufcount, ddrraddr, 2);
				 	bufWrite(outBuffer, &bufcount, dataddr, 4);
				 	outBufSend(g_uart, outBuffer, bufcount);

//				}else if (commandID == TESTDDR){
//					uint8_t ddrwen = (uint8_t) buffer[4];
//					uint8_t ddrren = (uint8_t) buffer[5];
//					uint8_t ddrwaddr = (uint8_t) buffer[6];
//					uint8_t ddrraddr = (uint8_t) buffer[7];
//					uint8_t ddrwdata = (uint8_t) buffer[8];
//					uint16_t retv = 0xF;
//
//					*(registers_0_addr + 0x03) = ddrwen;
//					*(registers_0_addr + 0x04) = ddrren;
//					*(registers_0_addr + 0x21) = ddrwaddr;
//					*(registers_0_addr + 0x22) = ddrraddr;
//					*(registers_0_addr + 0x0b) = ddrwdata;
//					retv = *(registers_0_addr + 0x1B);
//
//					outBuffer[bufcount++] = TESTDDR;
//					bufWrite(outBuffer, &bufcount, 7, 2);
//					outBuffer[bufcount++] = ddrwen;
//					outBuffer[bufcount++] = ddrren;
//					outBuffer[bufcount++] = ddrwaddr;
//					outBuffer[bufcount++] = ddrraddr;
//					outBuffer[bufcount++] = ddrwdata;
//					bufWrite(outBuffer, &bufcount, retv, 2);
//					outBufSend(g_uart, outBuffer, bufcount);

				}else if (commandID == DUMPSETTINGS){
					uint16_t channel = (uint16_t) buffer[4];
					outBuffer[bufcount++] = DUMPSETTINGS;
					if (channel>=0 && channel<96){

						bufWrite(outBuffer, &bufcount, 10, 2);
						bufWrite(outBuffer, &bufcount, channel, 2);

						bufWrite(outBuffer, &bufcount, default_gains_hv[channel], 2);
						bufWrite(outBuffer, &bufcount, default_threshs_hv[channel], 2);
						bufWrite(outBuffer, &bufcount, default_gains_cal[channel], 2);
						bufWrite(outBuffer, &bufcount, default_threshs_cal[channel], 2);
					}

					else{
						bufWrite(outBuffer, &bufcount, 768, 2);
						for (uint8_t ic = 0; ic < 96; ic++){
							bufWrite(outBuffer, &bufcount, default_gains_hv[ic], 2);
							bufWrite(outBuffer, &bufcount, default_threshs_hv[ic], 2);
							bufWrite(outBuffer, &bufcount, default_gains_cal[ic], 2);
							bufWrite(outBuffer, &bufcount, default_threshs_cal[ic], 2);
						}
					}
					outBufSend(g_uart, outBuffer, bufcount);


				 }else if (commandID == READMONADCS){

				 	//read currents

				 	uint32_t rx0;

				 	outBuffer[bufcount++] = READMONADCS;
				 	bufWrite(outBuffer, &bufcount, 32, 2);
				 	for (uint8_t i = 0 ; i < 2; i++){
				 		for (uint8_t j = 0 ; j < 8; j++){
				 			SPI_set_slave_select( &g_spi[i] , (j<4?SPI_SLAVE_0:SPI_SLAVE_1));
				 			uint16_t addr = (j%4 <<11 );
				 			SPI_transfer_frame( &g_spi[i], addr);
				 			rx0 = SPI_transfer_frame( &g_spi[i], addr);
				 			SPI_clear_slave_select( &g_spi[i] , (j<4?SPI_SLAVE_0:SPI_SLAVE_1));
				 			bufWrite(outBuffer, &bufcount, rx0, 2);
				 		}
				 	}

				 	outBufSend(g_uart, outBuffer, bufcount);
					//				}else if (commandID == MCPWRITEPIN){
					//
					//					uint16_t mcp = readU16fromBytes(&buffer[4]);;
					//					uint16_t channel = readU16fromBytes(&buffer[6]);
					//					uint8_t retv = 0;
					//
					//					if (mcp > MCPHV3) retv = 255;
					//				    MCP_pinWrite(&preampMCP[mcp], channel, 1);
					//				    MCP_pinMode(&preampMCP[mcp], channel, MCP_OUTPUT);
					//				    MCP_pinMode(&preampMCP[mcp], channel, MCP_INPUT);
					//				    retv = MCP_pinRead(&preampMCP[mcp], channel);
					//				    MCP_pinMode(&preampMCP[mcp], channel, MCP_OUTPUT);
					//
					//					outBuffer[bufcount++] = MCPWRITEPIN;
					//					outBuffer[bufcount++] = 4;
					//					outBuffer[bufcount++] = mcp;
					//					outBuffer[bufcount++] = retv;
					//					outBuffer[bufcount++] = channel & 0xff;
					//					outBuffer[bufcount++] = channel >> 8;
					//
					//					UART_send(&g_uart, outBuffer ,bufcount );
				 }else if (commandID == GETDEVICEID){

				 	uint8_t data_buffer[16];
				 	uint8_t status;
				 	status = SYS_get_serial_number(data_buffer, 0);
				 	outBuffer[bufcount++] = GETDEVICEID;
				 	bufWrite(outBuffer, &bufcount, 16, 2);
				 	for (uint8_t i = 0 ; i < 16; i++)
				 		outBuffer[bufcount++] = data_buffer[i];
				 	outBufSend(g_uart, outBuffer, bufcount);

				 }else if (commandID == READBMES){

				 	outBuffer[bufcount++] = READBMES;
				 	bufWrite(outBuffer, &bufcount, 24, 2);
				 	rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptscal);
				 	ptscal.delay_ms(40);
				 	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptscal);
				 	bufWrite(outBuffer, &bufcount, comp_data.temperature, 4);
				 	bufWrite(outBuffer, &bufcount, comp_data.pressure, 4);
				 	bufWrite(outBuffer, &bufcount, comp_data.humidity, 4);

				 	//					sprintf(outBuffer,"CAL %d %d %d\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);
				 	//					MSS_UART_polled_tx( &g_mss_uart1, outBuffer, strlen(outBuffer) );
				 	rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptshv);
				 	ptshv.delay_ms(40);
				 	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptshv);
				 	bufWrite(outBuffer, &bufcount, comp_data.temperature, 4);
				 	bufWrite(outBuffer, &bufcount, comp_data.pressure, 4);
				 	bufWrite(outBuffer, &bufcount, comp_data.humidity, 4);
				 	//					sprintf(outBuffer,"HV %d %d %d\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);
				 	//					MSS_UART_polled_tx( &g_mss_uart1, outBuffer, strlen(outBuffer) );
				 	outBufSend(g_uart, outBuffer, bufcount);

				 }else if (commandID == DIGIRW){
					 uint8_t rw = (uint8_t) buffer[4];
					 uint8_t thishvcal = (uint8_t) buffer[5];
					 uint8_t address = (uint8_t) buffer[6];
					 uint16_t data = readU16fromBytes(&buffer[7]);

					 outBuffer[bufcount++] = DIGIRW;
					 bufWrite(outBuffer, &bufcount, 5, 2);

					 if ( rw == 0 ){//read
						 data = digi_read(address, thishvcal);
					 }
					 else{
						 digi_write(address, data, thishvcal);
					 }
					 bufWrite(outBuffer, &bufcount, rw, 1);
					 bufWrite(outBuffer, &bufcount, thishvcal, 1);
					 bufWrite(outBuffer, &bufcount, address, 1);
					 bufWrite(outBuffer, &bufcount, data, 2);
					 outBufSend(g_uart, outBuffer, bufcount);


//***********************************begin of control_digi commands*******************************************************************************
				}else if (commandID == ADCRWCMDID){
					// adc read/write
					uint8_t adc_num = (uint8_t) buffer[4];
					uint8_t rw = (uint8_t) buffer[5];
					uint16_t address = readU16fromBytes(&buffer[6]);
					uint16_t data = readU16fromBytes(&buffer[8]);

					outBuffer[bufcount++] = ADCRWCMDID;
					bufWrite(outBuffer, &bufcount, 6, 2);

					if (rw == 1){
						uint8_t result = adc_read(address,adc_num);
						//						sprintf(outBuffer,"Read adc %d address %02x: %02x\n",adc_num,address,result);
						//						UART_polled_tx_string( &g_uart, outBuffer );

						outBuffer[bufcount++] = rw;
						outBuffer[bufcount++] = adc_num;
						bufWrite(outBuffer, &bufcount, address, 2);
						outBuffer[bufcount++] = result;
						outBuffer[bufcount++] = 0;
						outBufSend(g_uart, outBuffer, bufcount);
					}else{
						adc_write(address,(uint8_t) data,(0x1<<adc_num));
						//						sprintf(outBuffer,"Wrote adc %d address %02x: %02x\n",adc_num,address,data);
						//						UART_polled_tx_string( &g_uart, outBuffer );

						outBuffer[bufcount++] = rw;
						outBuffer[bufcount++] = adc_num;
						bufWrite(outBuffer, &bufcount, address, 2);
						bufWrite(outBuffer, &bufcount, data, 2);
						outBufSend(g_uart, outBuffer, bufcount);
					}

				}else if (commandID == BITSLIPCMDID){

//					// bitslip
//					uint16_t num_bits = readU16fromBytes(&buffer[4]);
//					channel_mask[0] = readU32fromBytes(&buffer[6]);
//					channel_mask[1] = readU32fromBytes(&buffer[10]);
//					channel_mask[2] = readU32fromBytes(&buffer[14]);
//					get_mapped_channels();
//
//
//	      //for (int i=0;i<num_bits;i++){
//					 *(registers_0_addr + 0x30) = ((mapped_channel_mask1 & 0xFF)>>0);
//					 *(registers_0_addr + 0x31) = ((mapped_channel_mask1 & 0xFF00)>>8);
//					 *(registers_0_addr + 0x32) = ((mapped_channel_mask1 & 0xFF0000)>>16);
//					 *(registers_0_addr + 0x33) = ((mapped_channel_mask1 & 0xFF000000)>>24);
//					 *(registers_0_addr + 0x34) = ((mapped_channel_mask2 & 0xFF)>>0);
//					 *(registers_0_addr + 0x35) = ((mapped_channel_mask2 & 0xFF00)>>8);
//					 *(registers_0_addr + 0x30) = 0x0;
//					 *(registers_0_addr + 0x31) = 0x0;
//					 *(registers_0_addr + 0x32) = 0x0;
//					 *(registers_0_addr + 0x33) = 0x0;
//					 *(registers_0_addr + 0x34) = 0x0;
//					 *(registers_0_addr + 0x35) = 0x0;
//	      delayUs(100);
//	      //}
//
//	      sprintf(outBuffer,"Activated bitslip %d times for channels %08x %08x %08x\n",num_bits,channel_mask1,channel_mask2,channel_mask3);
//	      UART_polled_tx_string( &g_uart, outBuffer );

					volatile uint32_t *empty_p = (registers_0_addr + REG_ROC_FIFO_EMPTY);
					volatile uint32_t *full_p = (registers_0_addr + REG_ROC_FIFO_FULL);
					volatile uint32_t *data_p = (registers_0_addr + REG_ROC_FIFO_DATA);

					uint32_t empty = *(empty_p);
					uint32_t full = *(full_p);
					uint32_t data1 = *(data_p);

					*(registers_0_addr + REG_ROC_FIFO_RE) = 1;
					uint32_t data2 = *(data_p);

					//					sprintf(outBuffer,"Empty: %d, Full: %d, data1: %04x, data2: %04x\n",empty,full,data1,data2);
					//					UART_polled_tx_string( &g_uart, outBuffer );
					outBuffer[bufcount++] = BITSLIPCMDID;
					bufWrite(outBuffer, &bufcount, 32, 2);
					bufWrite(outBuffer, &bufcount, empty, 4);
					bufWrite(outBuffer, &bufcount, full, 4);
					bufWrite(outBuffer, &bufcount, data1, 4);
					bufWrite(outBuffer, &bufcount, data2, 4);

					outBufSend(g_uart, outBuffer, bufcount);

				}else if (commandID == AUTOBITSLIPCMDID){
					// auto bitslip

					uint8_t clock = (uint8_t) buffer[4];
					uint8_t dophase = (uint8_t) buffer[5];
					channel_mask[0] = readU32fromBytes(&buffer[6]);
					channel_mask[1] = readU32fromBytes(&buffer[10]);
					channel_mask[2] = readU32fromBytes(&buffer[14]);
					int8_t adc_mode = 1;

					hvcal = 1;
					get_mapped_channels();

					digi_write(DG_ADDR_SAMPLE,1,0);
					digi_write(DG_ADDR_LOOKBACK,1,0);

					digi_write(DG_ADDR_TRIGGER_MODE,0,0);
					digi_write(DG_ADDR_ENABLE_PULSER,1,0);

					*(registers_0_addr + REG_ROC_FIFO_HOWMANY) = 1;

					outBuffer[bufcount++] = AUTOBITSLIPCMDID;
					bufWrite(outBuffer, &bufcount, 902, 2);
					outBuffer[bufcount++] = dophase;

					if (dophase){
						hvcal = 1;
						int8_t phases[96];
						for (uint8_t ichan=0;ichan<96;ichan++){
							phases[ichan] = -1;
							if (ichan > 47)
								hvcal =2;
							uint16_t adc_number = adc_map[ichan/8];
							thischanmask = (((uint32_t) 0x1)<<(ichan%32));
							if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
									((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
									((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
								bufWrite(outBuffer, &bufcount, 0, 9);
								continue;
							}
							if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
								bufWrite(outBuffer, &bufcount, 0, 9);
								continue;
							}
							digi_write(DG_ADDR_MASK1, 0x0, 0);
							digi_write(DG_ADDR_MASK2, 0x0, 0);
							digi_write(DG_ADDR_MASK3, 0x0, 0);
							if ((ichan%48) < 16)
								digi_write(DG_ADDR_MASK1, (uint16_t) (0x1<<(ichan%48)), hvcal);
							else if ((ichan%48) < 32)
								digi_write(DG_ADDR_MASK2, (uint16_t) (0x1<<((ichan%48)-16)), hvcal);
							else
								digi_write(DG_ADDR_MASK3, (uint16_t) (0x1<<((ichan%48)-32)), hvcal);
							uint16_t results[12];

							uint8_t iphase=0;
							for (iphase=0;iphase<12;iphase++){
								adc_write(ADC_ADDR_PHASE,iphase,(0x1<<(adc_number)));
								adc_write(ADC_ADDR_TESTIO,9,(0x1<<(adc_number)));

								// reset fifo
								resetFIFO(hvcal);

								readout_obloc = 0;
								readout_maxDelay = 50;
								readout_mode = 0;
								readout_wordsPerTrigger = 13;
								readout_numTriggers = 11;
								readout_totalTriggers = 0;

								int delay_count = 0;
								int trigger_count = 0;

								uint16_t lasthit[13];

								read_data2(&delay_count,&trigger_count,lasthit);
								if (trigger_count != 11){

									//									sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
									//									UART_polled_tx_string( &g_uart, outBuffer );
									break;
								}
								results[iphase] = lasthit[12];
							}
							outBuffer[bufcount++] = ichan;
							outBuffer[bufcount++] = iphase; // outputs the number of phases tested with enough triggers

							uint8_t maxdist = 0;
							//int bestclock = -1;
							uint8_t bestclock = 0xff;
							for (uint8_t i=0;i<12;i++){
								if (results[i] != 0x2AA && results[i] != 0x155){
									continue;
								} //find first occurrence of 0x2AA or 0x155
								uint8_t thisdist = 1;
								for (uint8_t j=1;j<12;j++){
									uint8_t i2 = (i+j) % 12;
									if ((results[i2] != 0x2AA && results[i2] != 0x155) || results[i2] != results[i]){
										break;
									}
									thisdist++;
								}
								uint8_t thisdist2 = 1;
								for (uint8_t j=1;j<12;j++){
									uint8_t i2 = (i-j+12) % 12;
									if ((results[i2] != 0x2AA && results[i2] != 0x155) || results[i2] != results[i]){
										break;
									}
									thisdist2++;
								}
								if (thisdist2 < thisdist)
									thisdist = thisdist2;
								if (thisdist > maxdist){
									maxdist = thisdist;
									bestclock = i;
								}
							}
							//							sprintf(outBuffer,"%d: Best clock phase: %d\n",ichan,bestclock);
							//							UART_polled_tx_string( &g_uart, outBuffer );

							outBuffer[bufcount++] = bestclock;

							//if (bestclock > -1)
							if (bestclock < 12){
								phases[ichan] = bestclock;
								bufWrite(outBuffer, &bufcount, 0, 6);
							}
							else{
								//								sprintf(outBuffer,"%d: Could not find useable clock phase %02x %02x %02x\n",ichan,results[0],results[3],results[6]);
								//								UART_polled_tx_string( &g_uart, outBuffer );
								bufWrite(outBuffer, &bufcount, results[0], 2);
								bufWrite(outBuffer, &bufcount, results[3], 2);
								bufWrite(outBuffer, &bufcount, results[6], 2);

							}
						}

						// get best phase for each adc
						for (uint8_t i=0;i<12;i++){
							int phasecount[12] = {0};
							for (uint8_t j=0;j<8;j++)
								if (phases[i*8+j] >= 0)
									phasecount[phases[i*8+j]]++;
							int maxcount = 0;
							adc_phases[adc_map[i]] = 0;
							for (uint8_t j=0;j<12;j++){
								if (phasecount[j] > maxcount){
									maxcount = phasecount[j];
									adc_phases[adc_map[i]] = j;
								}
							}
						}
					}else{
						bufWrite(outBuffer, &bufcount, 0, 9*96);
					}

					// now do bitslip
					for (uint8_t i=0;i<12;i++){

						if ((0x1<<i) & ENABLED_ADCS){
							if (clock < 99){
								adc_write(ADC_ADDR_PHASE,clock,(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,1,(0x1<<i));
							}else{
								adc_write(ADC_ADDR_PHASE,adc_phases[i],(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,1,(0x1<<i));
							}
						}
					}
					uint8_t ichan;
					hvcal = 1;
					for (ichan=0;ichan<96;ichan++){
						if (ichan > 47)
							hvcal =2;
						uint8_t adc_number = adc_map[ichan/8];
						thischanmask = (((uint32_t) 0x1)<<(ichan%32));
						if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
								((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
								((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++;
							continue;
						}
						if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++;
							continue;
						}
						digi_write(DG_ADDR_MASK1, 0x0, 0);
						digi_write(DG_ADDR_MASK2, 0x0, 0);
						digi_write(DG_ADDR_MASK3, 0x0, 0);
						if ((ichan%48) < 16)
							digi_write(DG_ADDR_MASK1,(uint16_t) (0x1<<(ichan%48)), hvcal);
						else if ((ichan%48) < 32)
							digi_write(DG_ADDR_MASK2,(uint16_t) (0x1<<((ichan%48)-16)), hvcal);
						else
							digi_write(DG_ADDR_MASK3,(uint16_t) (0x1<<((ichan%48)-32)), hvcal);
						uint8_t success = 0;
						for (uint8_t i=0;i<10;i++){
							resetFIFO(hvcal);

							readout_obloc = 0;
							readout_maxDelay = 50;
							readout_mode = 0;
							readout_wordsPerTrigger = 13;
							readout_numTriggers = 11;
							readout_totalTriggers = 0;

							int delay_count = 0;
							int trigger_count = 0;

							uint16_t lasthit[13];

							read_data2(&delay_count,&trigger_count,lasthit);
							if (trigger_count != 11){
								//								sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
								//								UART_polled_tx_string( &g_uart, outBuffer );
								break;
							}
							if (lasthit[12] == 0x001){
								success = 1;
								break;
							}
							//sprintf(outBuffer,"%d: %d: got %02x\n",ichan,i,lasthit[12]);
							//UART_polled_tx_string( &g_uart, outBuffer );

							if ((ichan%48) < 8)
								digi_write(DG_ADDR_BITSLIP0,((0x1<<((ichan%48)-0))), hvcal);
							else if ((ichan%48) < 16)
								digi_write(DG_ADDR_BITSLIP1,((0x1<<((ichan%48)-8))), hvcal);
							else if ((ichan%48) < 24)
								digi_write(DG_ADDR_BITSLIP2,((0x1<<((ichan%48)-16))), hvcal);
							else if ((ichan%48) < 32)
								digi_write(DG_ADDR_BITSLIP3,((0x1<<((ichan%48)-24))), hvcal);
							else if ((ichan%48) < 40)
								digi_write(DG_ADDR_BITSLIP4,((0x1<<((ichan%48)-32))), hvcal);
							else if ((ichan%48) < 48)
								digi_write(DG_ADDR_BITSLIP5,((0x1<<((ichan%48)-40))), hvcal);
							digi_write(DG_ADDR_BITSLIP0,0x0,hvcal);
							digi_write(DG_ADDR_BITSLIP1,0x0,hvcal);
							digi_write(DG_ADDR_BITSLIP2,0x0,hvcal);
							digi_write(DG_ADDR_BITSLIP3,0x0,hvcal);
							digi_write(DG_ADDR_BITSLIP4,0x0,hvcal);
							digi_write(DG_ADDR_BITSLIP5,0x0,hvcal);
						}

						//if (success){
						//	sprintf(outBuffer,"%d: Found correct word alignment\n",ichan);
						//	UART_polled_tx_string( &g_uart, outBuffer );
						//}else{
						//	sprintf(outBuffer,"%d: FAILED to find correct word alignment\n",ichan);
						//	UART_polled_tx_string( &g_uart, outBuffer );
						//}
						if (success)
							outBuffer[bufcount] = outBuffer[bufcount] | ( 1 << (ichan % 8));
						else
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
						if (((ichan+1)%8)==0) bufcount++; //gives 12 bytes describing whether the operation is successful
					}

					if (ichan<96){
						for (uint8_t i=ichan; i<96; i++) {
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (i % 8))));
							if (((i+1)%8)==0) bufcount++;
						}
					} //match the format if break in between

					// check at the end with mixed frequency ADC
					for (uint8_t i=0;i<12;i++){

						if ((0x1<<i) & ENABLED_ADCS){

							if (clock < 99){
								adc_write(ADC_ADDR_PHASE,clock,(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
							}else{
								adc_write(ADC_ADDR_PHASE,adc_phases[i],(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
							}
						}
					}
					uint32_t error_mask[3] = {0};

					hvcal = 1;
					for (ichan=0;ichan<96;ichan++){
						if (ichan > 47)
							hvcal =2;
						uint8_t adc_number = adc_map[ichan/8];
						thischanmask = (((uint32_t) 0x1)<<(ichan%32));
						if ( ((ichan<32) && ((thischanmask & mapped_channel_mask[0]) == 0x0))||
								((ichan>=32) && (ichan<64) && ((thischanmask & mapped_channel_mask[1]) == 0x0))||
								((ichan>=64) && ((thischanmask & mapped_channel_mask[2]) == 0x0))	){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
							continue;
						}
						if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
							continue;
						}
						digi_write(DG_ADDR_MASK1, 0x0, 0);
						digi_write(DG_ADDR_MASK2, 0x0, 0);
						digi_write(DG_ADDR_MASK3, 0x0, 0);
						if ((ichan%48) < 16)
							digi_write(DG_ADDR_MASK1,(uint16_t) (0x1<<(ichan%48)), hvcal);
						else if ((ichan%48) < 32)
							digi_write(DG_ADDR_MASK2,(uint16_t) (0x1<<((ichan%48)-16)), hvcal);
						else
							digi_write(DG_ADDR_MASK3,(uint16_t) (0x1<<((ichan%48)-32)), hvcal);

						resetFIFO(hvcal);

						readout_obloc = 0;
						readout_maxDelay = 50;
						readout_mode = 0;
						readout_wordsPerTrigger = 13;
						readout_numTriggers = 11;
						readout_totalTriggers = 0;

						int delay_count = 0;
						int trigger_count = 0;

						uint16_t lasthit[13];

						read_data2(&delay_count,&trigger_count,lasthit);
						if (trigger_count != 11){
							//sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
							//UART_polled_tx_string( &g_uart, outBuffer );
							break;
						}
						if (lasthit[12] != 0x319){
							error_mask[ichan/32]|= (((uint32_t)0x1)<<(ichan%32));
							//sprintf(outBuffer,"%d FAILED final check: %02x (instead of 0x319)\n",ichan,lasthit[12]);
							//UART_polled_tx_string( &g_uart, outBuffer );
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //gives 12 bytes describing whether final check is successful
						}
						else{
							outBuffer[bufcount] = outBuffer[bufcount] | ( 1 << (ichan % 8));
							if (((ichan+1)%8)==0) bufcount++;
						}
					}

					if (ichan<96){
						for (uint8_t i=ichan; i<96; i++) {
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((i+1)%8)==0) bufcount++;
						}
					} //match the format if break in between

					outBuffer[bufcount++] = ichan;

					//sprintf(outBuffer,"Error mask: %04x%08x\n",((uint32_t)(error_mask>>32)),(uint32_t)(error_mask&0xFFFFFFFF));
					//UART_polled_tx_string( &g_uart, outBuffer );

					for (uint8_t i=0; i<3; i++)
						bufWrite(outBuffer, &bufcount, error_mask[i], 4);
					outBufSend(g_uart, outBuffer, bufcount);

				}else if (commandID == READRATESCMDID){
					uint16_t num_lookback = readU16fromBytes(&buffer[4]);
					uint16_t num_samples = readU16fromBytes(&buffer[6]);
					channel_mask[0] = readU32fromBytes(&buffer[8]);
					channel_mask[1] = readU32fromBytes(&buffer[12]);
					channel_mask[2] = readU32fromBytes(&buffer[16]);

					outBuffer[bufcount++] = READRATESCMDID;
					bufcount_place_holder = bufcount;
					bufWrite(outBuffer, &bufcount, 0, 2);

					get_rates(num_lookback,num_samples);

					bufWrite(outBuffer, &bufcount_place_holder, (bufcount-3), 2);
					outBufSend(g_uart, outBuffer, bufcount);

				} else if (commandID == READDATACMDID){

					uint16_t adc_mode = (uint16_t) ((uint8_t) buffer[4]);
					uint16_t tdc_mode = (uint16_t) ((uint8_t) buffer[5]);
					uint16_t num_lookback = readU16fromBytes(&buffer[6]);
					uint16_t num_samples = readU16fromBytes(&buffer[8]);
					uint32_t num_triggers = readU32fromBytes(&buffer[10]);
					channel_mask[0] = readU32fromBytes(&buffer[14]);
					channel_mask[1] = readU32fromBytes(&buffer[18]);
					channel_mask[2] = readU32fromBytes(&buffer[22]);
					uint8_t clock = (uint8_t) buffer[26];
					uint8_t enable_pulser = (uint8_t) buffer[27];
					uint16_t max_total_delay = readU16fromBytes(&buffer[28]);
					uint8_t mode = buffer[30];

					for (uint8_t i=0;i<12;i++){
						if ((0x1<<i) & ENABLED_ADCS){
							if (clock < 99){
								adc_write(ADC_ADDR_PHASE,clock,(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,adc_mode,(0x1<<i));
							}else{
								adc_write(ADC_ADDR_PHASE,adc_phases[i],(0x1<<i));
								adc_write(ADC_ADDR_TESTIO,adc_mode,(0x1<<i));
							}
						}
					}

					hvcal = 1;
					get_mapped_channels();

					digi_write(DG_ADDR_SAMPLE,num_samples,0);
					digi_write(DG_ADDR_LOOKBACK,num_lookback,0);
					digi_write(DG_ADDR_MASK1,(uint16_t) (mapped_channel_mask[0] & 0xFFFF), 1);
					digi_write(DG_ADDR_MASK2,(uint16_t) ((mapped_channel_mask[0] & 0xFFFF0000)>>16), 1);
					digi_write(DG_ADDR_MASK3,(uint16_t) (mapped_channel_mask[1] & 0xFFFF), 1);
					digi_write(DG_ADDR_MASK1,(uint16_t) ((mapped_channel_mask[1] & 0xFFFF0000)>>16), 2);
					digi_write(DG_ADDR_MASK2,(uint16_t) (mapped_channel_mask[2] & 0xFFFF), 2);
					digi_write(DG_ADDR_MASK3,(uint16_t) ((mapped_channel_mask[2] & 0xFFFF0000)>>16), 2);
					digi_write(DG_ADDR_TRIGGER_MODE,tdc_mode,0);
					digi_write(DG_ADDR_ENABLE_PULSER,enable_pulser,0);

					*(registers_0_addr + REG_ROC_FIFO_HOWMANY) = num_samples;

					if ((mapped_channel_mask[2]!=0)||((mapped_channel_mask[1]>>16)!=0))
						hvcal = 2;

					num_samples = digi_read(DG_ADDR_SAMPLE, hvcal);
					enable_pulser = digi_read(DG_ADDR_ENABLE_PULSER, hvcal);

					char tdc_string[8];
					if (enable_pulser == 0){
					//	strncpy(tdc_string, "REAL  \0", 7);
						tdc_string[0]='R';
						tdc_string[1]='E';
						tdc_string[2]='A';
						tdc_string[3]='L';
						tdc_string[4]=' ';
						tdc_string[5]=' ';
						tdc_string[6]='\0';
					}
					else{
					//	strncpy(tdc_string, "PULSER\0", 7);
						tdc_string[0]='P';
						tdc_string[1]='U';
						tdc_string[2]='L';
						tdc_string[3]='S';
						tdc_string[4]='E';
						tdc_string[5]='R';
						tdc_string[6]='\0';
					}
					//sprintf(outBuffer,"Setup complete:\nSamples: %d\nLookback: %d\nChannel Mask: %08x\nADC Mode: %d\nTDC Mode: %d\nTrigger on: %s\nClock Mode: %d\nTriggers: %d\n",num_samples,num_lookback,channel_mask1,adc_mode,tdc_mode,tdc_string,clock,num_triggers);
					//UART_polled_tx_string( &g_uart, outBuffer );
					//sprintf(outBuffer,"CM: %08x %08x %08x, ep: %d\n",digi_read(0xb),digi_read(0xe),digi_read(0xd),digi_read(0xc));
					//UART_polled_tx_string( &g_uart, outBuffer );

					outBuffer[bufcount++] = READDATACMDID;
					//outBuffer[bufcount++] = 33;
					bufWrite(outBuffer, &bufcount, 35, 2);
					outBuffer[bufcount++] = enable_pulser;
					bufWrite(outBuffer, &bufcount, num_samples, 2);
					bufWrite(outBuffer, &bufcount, num_lookback, 2);
					bufWrite(outBuffer, &bufcount, channel_mask[0], 4);
					bufWrite(outBuffer, &bufcount, adc_mode, 2);
					bufWrite(outBuffer, &bufcount, tdc_mode, 2);
					for (uint8_t i=0; i<8; i++)
						outBuffer[bufcount++] = (uint8_t) tdc_string[i];
					outBuffer[bufcount++] = clock;
					bufWrite(outBuffer, &bufcount, num_triggers, 4);
					bufWrite(outBuffer, &bufcount, digi_read(DG_ADDR_MASK1,hvcal), 2);
					bufWrite(outBuffer, &bufcount, digi_read(DG_ADDR_MASK2,hvcal), 2);
					bufWrite(outBuffer, &bufcount, digi_read(DG_ADDR_MASK3,hvcal), 2);
					bufWrite(outBuffer, &bufcount, digi_read(DG_ADDR_ENABLE_PULSER,hvcal), 2);

					delayUs(100);

					// reset fifo
					resetFIFO(hvcal);

					//readout_obloc = 6;
					readout_obloc = 0;
					//sprintf(dataBuffer,"start\n");
					readout_maxDelay = max_total_delay*50;
					readout_mode = mode;
					readout_wordsPerTrigger = 12 + num_samples;
					readout_numTriggers = num_triggers;
					readout_totalTriggers = 0;

					outBuffer[bufcount++] = mode;
					outBufSend(g_uart, outBuffer, bufcount);

					if (mode == 0){
						int delay_count = 0;
						int trigger_count = 0;

						read_data(&delay_count,&trigger_count);

						//sprintf(&dataBuffer[readout_obloc],"\nend\n");
						//UART_polled_tx_string( &g_uart, dataBuffer );
						bufcount = 0;
						outBuffer[bufcount++] = READDATACMDID;
						bufWrite(outBuffer, &bufcount, 5, 2);
						outBuffer[bufcount++] = (uint8_t)(trigger_count == num_triggers);

						if (trigger_count == num_triggers){
							//sprintf(outBuffer,"SUCCESS! Delayed %d times\n",delay_count);
							bufWrite(outBuffer, &bufcount, (uint32_t)delay_count, 4);
						}else{
							//sprintf(outBuffer,"FAILED! Read %d triggers\n",trigger_count);
							bufWrite(outBuffer, &bufcount, (uint32_t)trigger_count, 4);
						}

						UART_send(&g_uart, outBuffer ,bufcount );

						//UART_polled_tx_string( &g_uart, outBuffer );

						//get_rates(0,10);
					}else{
						readout_enabled = 1;
						//sprintf(outBuffer,"Run started\n");
						//UART_polled_tx_string( &g_uart, outBuffer );
						outBuffer[0] = RUN_STARTED;
						UART_send(&g_uart, outBuffer ,1);
					}

				}else if (commandID == STOPRUNCMDID){

					outBuffer[bufcount++] = STOPRUNCMDID;
					bufWrite(outBuffer, &bufcount, 3, 2);
					outBuffer[bufcount++] = readout_enabled;

					if (readout_enabled == 0){
						//sprintf(outBuffer,"Error: no run to stop\n");
						//UART_polled_tx_string( &g_uart, outBuffer );
						bufWrite(outBuffer, &bufcount, 0, 2);
					}else{
						readout_enabled = 0;
						//sprintf(&dataBuffer[readout_obloc],"\nend\n");
						//UART_polled_tx_string( &g_uart, dataBuffer );
						//sprintf(outBuffer,"Run ended. Read %d triggers\n",readout_totalTriggers);
						//UART_polled_tx_string( &g_uart, outBuffer );
						bufWrite(outBuffer, &bufcount, readout_totalTriggers, 2);
					}
					outBufSend(g_uart, outBuffer, bufcount);

				}else if (commandID == ADCINITINFOCMDID){

					outBuffer[bufcount++] = ADCINITINFOCMDID;
					bufWrite(outBuffer, &bufcount, 30, 2);
					for (uint8_t i=0; i<30; i++)
						outBuffer[bufcount++] = init_buff[i];
					outBufSend(g_uart, outBuffer, bufcount);

				}else if (commandID == PACKAGETESTCMDID){
					outBuffer[bufcount++] = PACKAGETESTCMDID;
					outBuffer[bufcount++] = 0xCA;
					outBuffer[bufcount++] = 1;
					for (uint16_t i=0; i<458; i++)
						outBuffer[bufcount++] = i%256;
					outBufSend(g_uart, outBuffer, bufcount);

				}

				// If we didn't use the whole buffer, the rest must be the next command
				//memmove(&buffer[0],&buffer[numBytes],writePtr-numBytes);
				for (uint16_t j=0;j<writePtr-numBytes;j++){
					buffer[j] = buffer[j+numBytes];
				}
				writePtr -= numBytes;
			}
		}
	}

	return 0;
}

