/* Kernel includes. */
#include "hw_platform.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CoreSysServices_PF/core_sysservices_pf.h"

#include "./CMSIS/cortexm1_cfg.h"
#include "./CMSIS/system_cortexm1_cfg.h"
#include <stdio.h>
#include <string.h>


#include "setup.h"


#include "utils.h"
#include "version.h"


uint16_t default_gains_cal[96] = {271,275,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,269,269,269,269,269,269,269,268,269,269,269,264,269,269,269,270};
uint16_t default_gains_hv[96] = {270,271,270,270,270,270,270,270,270,270,270,270,270,270,266,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,267,270,270,270,270,266,266,272,268,266,265,269,269,269,269,270};
uint16_t default_threshs_cal[96] = {331,316,339,309,317,317,319,335,341,321,319,359,347,320,323,332,352,334,325,339,304,312,340,330,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,305,294,311,307,340,332,321,336,335,327,322,356,344,304,324,325,315,325,325,341,348,343,343,319};
uint16_t default_threshs_hv[96] = {322,284,329,317,325,321,340,347,351,339,348,349,313,337,327,325,345,325,321,320,345,332,305,335,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,284,293,324,321,324,331,344,333,314,324,333,346,330,335,327,329,349,316,331,339,318,304,332,310};



const uint16_t default_caldac[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

const uint8_t calpulse_chanmap[8]={9,1,10,2,11,3,12,4};




#define BAUD_VALUE                  57600
#define ENABLED_ADCS				0x01

const uint8_t MCPCALIBCHAN[8] = {1,2,3,4,9,10,11,12};

int main()
{


	SystemCoreClockUpdate();
	UART_init( &g_uart, UART_BASE_ADDRESS, (SYS_M1_CLK_FREQ/(16 * BAUD_VALUE))-1, (DATA_8_BITS | NO_PARITY));

	GPIO_init( &g_gpio,    COREGPIO_BASE_ADDR, GPIO_APB_32_BITS_BUS );
	//	GPIO_config( &g_gpio, GPIO_0, GPIO_OUTPUT_MODE);
	GPIO_set_output( &g_gpio, GPIO_0, 0);


	const uint8_t greeting[] = "\n\r\"Welcome to the ROC\"\n"
			"\t - Sean Connery\n";

	/* Send greeting message over the UART_0 */
	UART_polled_tx_string( &g_uart, greeting );
	//		sprintf(outBuffer,"Last git revision: %s\n",LAST_GIT_REV);
	//			UART_polled_tx_string( &g_uart, outBuffer );

	int readout_enabled = 0;
	int calibration_enabled = 0;
	int readout_totalTriggers = 0;

	//register address for bit banging
	registers_0_addr = (volatile uint32_t *) REGISTERBASEADDR;


	uint8_t errors = init_adc(0x3F,0x02,0x03);
	//sprintf(outBuffer,"ADCs initialized (errors: %02x)\n",errors);
	//UART_polled_tx_string( &g_uart, outBuffer );

	/*Initialize the CoreSysService_PF driver*/
	SYS_init(CSS_PF_BASE_ADDRESS);

	uint32_t loopCount = 0;
	uint32_t ledPattern = 0x1;
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
	for (uint8_t imcp = MCPCAL0; imcp<=MCPHV3; imcp++){
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

	adc_write(0x08,0x01,0x3F);
	adc_write(0x08,0x00,ENABLED_ADCS);

	uint8_t bitslip0 = 0x30;
	uint8_t bitslip1 = 0x31;
	uint8_t bitslip2 = 0x32;
	uint8_t bitslip3 = 0x33;
	uint8_t bitslip4 = 0x34;
	uint8_t bitslip5 = 0x35;

	digi_write(bitslip0,0x0);
	digi_write(bitslip1,0x0);
	digi_write(bitslip2,0x0);
	digi_write(bitslip3,0x0);
	digi_write(bitslip4,0x0);
	digi_write(bitslip5,0x0);

	digi_write(0x10,1);

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
			GPIO_set_output( &g_gpio, GPIO_0, ledPattern);

			loopCount = 0;
		}
		delayUs(1);
		loopCount++;



		size_t rx_size =UART_get_rx(&g_uart, rx_buff, sizeof(rx_buff));
		for (int j=0;j<rx_size;j++){
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

			uint32_t numBytes = buffer[2];
			if (writePtr >= numBytes){
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
					outBuffer[bufcount++] = 4;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = channel & 0xff;
					outBuffer[bufcount++] = channel >> 8;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );


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
					outBuffer[bufcount++] = 4;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = channel & 0xff;
					outBuffer[bufcount++] = channel >> 8;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == SETCALDAC){

					uint8_t chan_mask = (uint8_t) buffer[4];
					uint16_t value = readU16fromBytes(&buffer[5]);
					*(registers_0_addr+0x11) = 1;
					for (uint8_t i=0;i<8;i++){
						if (chan_mask & (0x1<<i))
							AD5318_write(g_spi[3],1, i,value);
					}
					*(registers_0_addr+0x11) = 0;
					outBuffer[bufcount++] = SETCALDAC;
					outBuffer[bufcount++] = 3;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = chan_mask;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );


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
					outBuffer[bufcount++] = 8;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = chan_mask;
					outBuffer[bufcount++] = pulserOdd;
					outBuffer[bufcount++] = dutyCycle & 0xff;
					outBuffer[bufcount++] = (dutyCycle>>8) & 0xff;
					outBuffer[bufcount++] = pulserDelay & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>8) & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>16) & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>24) & 0xff;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == SETPULSEROFF){

					PWM_disable(&g_pwm,PWM_1);
					outBuffer[bufcount++] = SETPULSEROFF;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = 0;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == WHOAREYOU){

					outBuffer[bufcount++] = WHOAREYOU;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = 0;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == RESETROC){
					*(registers_0_addr + 0x10) = 0;
					*(registers_0_addr + 0x10) = 1;
					outBuffer[bufcount++] = RESETROC;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = 0;


					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );
				}else if (commandID == TESTDDR){
					uint8_t ddrcs = (uint8_t) buffer[4];
					uint8_t ddrwen = (uint8_t) buffer[5];
					uint8_t ddrren = (uint8_t) buffer[6];
					uint8_t ddrdmaen = (uint8_t) buffer[7];
					uint8_t ddrnhits = (uint8_t) buffer[8];
					uint16_t ddrraddr = readU16fromBytes(&buffer[9]);
					uint32_t retv = 0xF;

					*(registers_0_addr + 0x20) = ddrnhits;
					*(registers_0_addr + 0x21) = 0;
					*(registers_0_addr + 0x22) = ddrcs;
					*(registers_0_addr + 0x23) = ddrwen;
					*(registers_0_addr + 0x24) = ddrren;
					*(registers_0_addr + 0x25) = ddrdmaen;
					*(registers_0_addr + 0x27) = ddrraddr;
					retv = *(registers_0_addr + 0x26);
					uint32_t dataddr = *(registers_0_addr + 0x28);
					outBuffer[bufcount++] = TESTDDR;
					outBuffer[bufcount++] = 15;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = ddrnhits;
					outBuffer[bufcount++] = ddrcs;
					outBuffer[bufcount++] = ddrwen;
					outBuffer[bufcount++] = ddrren;
					outBuffer[bufcount++] = ddrdmaen;
					outBuffer[bufcount++] = retv & 0xFF;
					outBuffer[bufcount++] = (retv>>8) & 0xFF;
					outBuffer[bufcount++] = (retv>>16) & 0xFF;
					outBuffer[bufcount++] = (retv>>24) & 0xFF;
					outBuffer[bufcount++] = ddrraddr & 0xFF;
					outBuffer[bufcount++] = (ddrraddr>>8) & 0xFF;
					outBuffer[bufcount++] =  dataddr& 0xFF;
					outBuffer[bufcount++] = (dataddr>>8) & 0xFF;
					outBuffer[bufcount++] = (dataddr>>16) & 0xFF;
					outBuffer[bufcount++] = (dataddr>>24) & 0xFF;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );
//				}else if (commandID == TESTDDR){
//					uint8_t ddrcs = (uint8_t) buffer[4];
//					uint8_t ddrwen = (uint8_t) buffer[5];
//					uint8_t ddrren = (uint8_t) buffer[6];
//					uint8_t ddrdmaen = (uint8_t) buffer[7];
//					uint8_t ddrnhits = (uint8_t) buffer[8];
//					uint16_t ddrraddr = readU16fromBytes(&buffer[9]);
//					uint32_t retv = 0xF;
//
//					*(registers_0_addr + 0x20) = ddrnhits;
//					*(registers_0_addr + 0x21) = 0;
//					*(registers_0_addr + 0x22) = ddrcs;
//					*(registers_0_addr + 0x23) = ddrwen;
//					*(registers_0_addr + 0x24) = ddrren;
//					*(registers_0_addr + 0x25) = ddrdmaen;
//					*(registers_0_addr + 0x27) = ddrraddr;
//					retv = *(registers_0_addr + 0x26);
//					uint32_t dataddr = *(registers_0_addr + 0x28);
//					outBuffer[bufcount++] = TESTDDR;
//					outBuffer[bufcount++] = 15;
//					outBuffer[bufcount++] = ddrnhits;
//					outBuffer[bufcount++] = ddrcs;
//					outBuffer[bufcount++] = ddrwen;
//					outBuffer[bufcount++] = ddrren;
//					outBuffer[bufcount++] = ddrdmaen;
//					outBuffer[bufcount++] = retv & 0xFF;
//					outBuffer[bufcount++] = (retv>>8) & 0xFF;
//					outBuffer[bufcount++] = (retv>>16) & 0xFF;
//					outBuffer[bufcount++] = (retv>>24) & 0xFF;
//					outBuffer[bufcount++] = ddrraddr & 0xFF;
//					outBuffer[bufcount++] = (ddrraddr>>8) & 0xFF;
//					outBuffer[bufcount++] =  dataddr& 0xFF;
//					outBuffer[bufcount++] = (dataddr>>8) & 0xFF;
//					outBuffer[bufcount++] = (dataddr>>16) & 0xFF;
//					outBuffer[bufcount++] = (dataddr>>24) & 0xFF;
//UART_polled_tx_string( &g_uart, "monitoring\n" );
//					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == DUMPSETTINGS){
					uint16_t channel = (uint16_t) buffer[4];
					outBuffer[bufcount++] = DUMPSETTINGS;
					if (channel>=0 && channel<96){

						outBuffer[bufcount++] = 10;
						outBuffer[bufcount++] = 0;
						outBuffer[bufcount++] = channel & 0xff;
						outBuffer[bufcount++] = channel >> 8;
						outBuffer[bufcount++] = default_gains_hv[channel] & 0xff;
						outBuffer[bufcount++] = default_gains_hv[channel] >> 8;
						outBuffer[bufcount++] = default_threshs_hv[channel] & 0xff;
						outBuffer[bufcount++] = default_threshs_hv[channel] >> 8;
						outBuffer[bufcount++] = default_gains_cal[channel] & 0xff;
						outBuffer[bufcount++] = default_gains_cal[channel] >> 8;
						outBuffer[bufcount++] = default_threshs_cal[channel] & 0xff;
						outBuffer[bufcount++] = default_threshs_cal[channel] >> 8;
					}

					else{
						outBuffer[bufcount++] = 768 & 0xff;
						outBuffer[bufcount++] = 768 >> 8;
						for (uint8_t ic = 0; ic < 96; ic++){
							outBuffer[bufcount++] = default_gains_hv[ic] & 0xff;
							outBuffer[bufcount++] = default_gains_hv[ic] >> 8;
							outBuffer[bufcount++] = default_threshs_hv[ic] & 0xff;
							outBuffer[bufcount++] = default_threshs_hv[ic] >> 8;
							outBuffer[bufcount++] = default_gains_cal[ic] & 0xff;
							outBuffer[bufcount++] = default_gains_cal[ic] >> 8;
							outBuffer[bufcount++] = default_threshs_cal[ic] & 0xff;
							outBuffer[bufcount++] = default_threshs_cal[ic] >> 8;
						}
					}
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == READMONADCS){

					//read currents

					uint32_t rx0;

					outBuffer[bufcount++] = READMONADCS;
					outBuffer[bufcount++] = 32;
					outBuffer[bufcount++] = 0;
					for (uint8_t i = 0 ; i < 8; i++){
						SPI_set_slave_select( &g_spi[0] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						uint16_t addr = (i%4 <<11 );
						SPI_transfer_frame( &g_spi[0], addr);
						rx0 = SPI_transfer_frame( &g_spi[0], addr);
						SPI_clear_slave_select( &g_spi[0] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						outBuffer[bufcount++] = rx0 & 0xFF;
						outBuffer[bufcount++] = (rx0 >> 8) & 0x0F;
					}

					for (uint8_t i = 0 ; i < 8; i++){
						SPI_set_slave_select( &g_spi[1] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						uint16_t addr = (i%4 <<11 );
						SPI_transfer_frame( &g_spi[1], addr);
						rx0 = SPI_transfer_frame( &g_spi[1], addr);
						SPI_clear_slave_select( &g_spi[1] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						outBuffer[bufcount++] = rx0 & 0xFF;
						outBuffer[bufcount++] = (rx0 >> 8) & 0x0F;
					}

					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );
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
					outBuffer[bufcount++] = 16;
					outBuffer[bufcount++] = 0;
					for (uint8_t i = 0 ; i < 16; i++)
						outBuffer[bufcount++] = data_buffer[i];
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );



				}else if (commandID == READBMES){

					outBuffer[bufcount++] = READBMES;
					outBuffer[bufcount++] = 24;
					rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptscal);
					ptscal.delay_ms(40);
					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptscal);
					outBuffer[bufcount++] = comp_data.temperature & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 24) & 0xFF;

					outBuffer[bufcount++] = comp_data.pressure & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 24) & 0xFF;

					outBuffer[bufcount++] = comp_data.humidity & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 24) & 0xFF;


					rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptshv);
					ptshv.delay_ms(40);
					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptshv);
					outBuffer[bufcount++] = comp_data.temperature & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.temperature >> 24) & 0xFF;

					outBuffer[bufcount++] = comp_data.pressure & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.pressure >> 24) & 0xFF;

					outBuffer[bufcount++] = comp_data.humidity & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 8) & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 16) & 0xFF;
					outBuffer[bufcount++] = (comp_data.humidity >> 24) & 0xFF;

					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}

				//begin of control_digi commands
				if (commandID == ADCRWCMDID){
					// adc read/write
					uint8_t adc_num = (uint8_t) buffer[4];
					uint8_t rw = (uint8_t) buffer[5];
					uint16_t address = readU16fromBytes(&buffer[6]);
					uint16_t data = readU16fromBytes(&buffer[8]);

					outBuffer[bufcount++] = ADCRWCMDID;
					outBuffer[bufcount++] = 6;
					outBuffer[bufcount++] = 0;

					if (rw == 1){
						uint8_t result = adc_read(address,adc_num);
						//						sprintf(outBuffer,"Read adc %d address %02x: %02x\n",adc_num,address,result);
						//						UART_polled_tx_string( &g_uart, outBuffer );

						outBuffer[bufcount++] = rw;
						outBuffer[bufcount++] = adc_num;
						outBuffer[bufcount++] = address & 0xff;
						outBuffer[bufcount++] = address >> 8;
						outBuffer[bufcount++] = result;
						outBuffer[bufcount++] = 0;
						UART_polled_tx_string( &g_uart, "monitoring\n" );
						UART_send(&g_uart, outBuffer ,bufcount );
					}else{
						adc_write(address,(uint8_t) data,(0x1<<adc_num));
						//						sprintf(outBuffer,"Wrote adc %d address %02x: %02x\n",adc_num,address,data);
						//						UART_polled_tx_string( &g_uart, outBuffer );

						outBuffer[bufcount++] = rw;
						outBuffer[bufcount++] = adc_num;
						outBuffer[bufcount++] = address & 0xff;
						outBuffer[bufcount++] = address >> 8;
						outBuffer[bufcount++] = data & 0xff;
						outBuffer[bufcount++] = data >> 8;
						UART_polled_tx_string( &g_uart, "monitoring\n" );
						UART_send(&g_uart, outBuffer ,bufcount );
					}

				}else if (commandID == BITSLIPCMDID){
					// bitslip
					uint16_t num_bits = readU16fromBytes(&buffer[4]);
					uint32_t channel_mask1 = readU32fromBytes(&buffer[6]);
					uint32_t channel_mask2 = readU32fromBytes(&buffer[10]);
					uint32_t channel_mask3 = readU32fromBytes(&buffer[14]);
					uint32_t mapped_channel_mask1;
					uint32_t mapped_channel_mask2;
					get_mapped_channels(&channel_mask1,&channel_mask2,&channel_mask3,&mapped_channel_mask1,&mapped_channel_mask2);

					/*
	      //for (int i=0;i<num_bits;i++){
					 *(registers_0_addr + 0x30) = ((mapped_channel_mask1 & 0xFF)>>0);
					 *(registers_0_addr + 0x31) = ((mapped_channel_mask1 & 0xFF00)>>8);
					 *(registers_0_addr + 0x32) = ((mapped_channel_mask1 & 0xFF0000)>>16);
					 *(registers_0_addr + 0x33) = ((mapped_channel_mask1 & 0xFF000000)>>24);
					 *(registers_0_addr + 0x34) = ((mapped_channel_mask2 & 0xFF)>>0);
					 *(registers_0_addr + 0x35) = ((mapped_channel_mask2 & 0xFF00)>>8);
					 *(registers_0_addr + 0x30) = 0x0;
					 *(registers_0_addr + 0x31) = 0x0;
					 *(registers_0_addr + 0x32) = 0x0;
					 *(registers_0_addr + 0x33) = 0x0;
					 *(registers_0_addr + 0x34) = 0x0;
					 *(registers_0_addr + 0x35) = 0x0;
	      delayUs(100);
	      //}

	      sprintf(outBuffer,"Activated bitslip %d times for channels %08x %08x %08x\n",num_bits,channel_mask1,channel_mask2,channel_mask3);
	      UART_polled_tx_string( &g_uart, outBuffer );
					 */
					volatile uint32_t *empty_p = (registers_0_addr + 0x43);
					volatile uint32_t *full_p = (registers_0_addr + 0x42);
					volatile uint32_t *data_p = (registers_0_addr + 0x41);

					uint32_t empty = *(empty_p);
					uint32_t full = *(full_p);
					uint32_t data1 = *(data_p);
					*(registers_0_addr + 0x40) = 1;
					uint32_t data2 = *(data_p);
					//					sprintf(outBuffer,"Empty: %d, Full: %d, data1: %04x, data2: %04x\n",empty,full,data1,data2);
					//					UART_polled_tx_string( &g_uart, outBuffer );
					outBuffer[bufcount++] = BITSLIPCMDID;
					outBuffer[bufcount++] = 16;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = empty & 0xff;
					outBuffer[bufcount++] = ( empty >> 8 ) & 0xff;
					outBuffer[bufcount++] = ( empty >> 16 ) & 0xff;
					outBuffer[bufcount++] = empty >> 24;
					outBuffer[bufcount++] = full & 0xff;
					outBuffer[bufcount++] = ( full >> 8 ) & 0xff;
					outBuffer[bufcount++] = ( full >> 16 ) & 0xff;
					outBuffer[bufcount++] = full >> 24;
					outBuffer[bufcount++] = data1 & 0xff;
					outBuffer[bufcount++] = ( data1 >> 8 ) & 0xff;
					outBuffer[bufcount++] = ( data1 >> 16 ) & 0xff;
					outBuffer[bufcount++] = data1 >> 24;
					outBuffer[bufcount++] = data2 & 0xff;
					outBuffer[bufcount++] = ( data2 >> 8 ) & 0xff;
					outBuffer[bufcount++] = ( data2 >> 16 ) & 0xff;
					outBuffer[bufcount++] = data2 >> 24;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == AUTOBITSLIPCMDID){
					// auto bitslip

					uint8_t clock = (uint8_t) buffer[4];
					uint8_t dophase = (uint8_t) buffer[5];
					uint32_t channel_mask1 = readU32fromBytes(&buffer[6]);
					uint32_t channel_mask2 = readU32fromBytes(&buffer[10]);
					uint32_t channel_mask3 = readU32fromBytes(&buffer[14]);
					int adc_mode = 1;


					uint32_t mapped_channel_mask1;
					uint32_t mapped_channel_mask2;
					get_mapped_channels(&channel_mask1,&channel_mask2,&channel_mask3,&mapped_channel_mask1,&mapped_channel_mask2);

					digi_write(0x3,1);
					digi_write(0x4,1);
					digi_write(0xb,(uint16_t) (mapped_channel_mask1 & 0xFFFF));
					digi_write(0xe,(uint16_t) ((mapped_channel_mask1 & 0xFFFF0000)>>16));
					digi_write(0xd,(uint16_t) (mapped_channel_mask2 & 0xFFFF));
					digi_write(0xa,0);
					digi_write(0xc,1);

					uint64_t all_channel_mask = (((uint64_t) (mapped_channel_mask2)) <<32) | (uint64_t) (mapped_channel_mask1);

					outBuffer[bufcount++] = AUTOBITSLIPCMDID;
					outBuffer[bufcount++] = (22+9*48) & 0xff;
					outBuffer[bufcount++] = (22+9*48) >> 8;
					outBuffer[bufcount++] = dophase;

					if (dophase){
						int phases[48];
						for (int ichan=0;ichan<48;ichan++){
							phases[ichan] = -1;
							uint8_t adc_number = adc_map[ichan/8];
							uint64_t thischanmask = (((uint64_t) 0x1)<<ichan);
							if ((thischanmask & all_channel_mask) == 0x0){
								for (int i=0; i<9;i++) outBuffer[bufcount++] = 0;
								continue;
							}
							if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
								for (int i=0; i<9;i++) outBuffer[bufcount++] = 0;
								continue;
							}
							digi_write(0xb,0x0);
							digi_write(0xe,0x0);
							digi_write(0xd,0x0);
							if (ichan < 16)
								digi_write(0xb,(uint16_t) (0x1<<ichan));
							else if (ichan < 32)
								digi_write(0xe,(uint16_t) (0x1<<(ichan-16)));
							else
								digi_write(0xd,(uint16_t) (0x1<<(ichan-32)));
							uint16_t results[12];

							int i=0;
							for (i=0;i<12;i++){
								adc_write(0x16,i,(0x1<<(adc_number)));
								adc_write(0x0D,9,(0x1<<(adc_number)));

								// reset fifo
								digi_write(0x10,0);
								*(registers_0_addr + 0x44) = 1;
								digi_write(0x10,1);
								reset_fabric();

								readout_obloc = 0;
								readout_maxDelay = 50;
								readout_mode = 0;
								readout_wordsPerTrigger = 13;
								readout_numTriggers = 1;
								readout_totalTriggers = 0;

								int delay_count = 0;
								int trigger_count = 0;

								uint16_t lasthit[13];

								read_data2(&delay_count,&trigger_count,lasthit);
								if (trigger_count != 1){

									//									sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
									//									UART_polled_tx_string( &g_uart, outBuffer );
									break;
								}
								results[i] = lasthit[12];
							}
							outBuffer[bufcount++] = ichan;
							outBuffer[bufcount++] = i; // outputs the number of triggers

							int maxdist = 0;
							//int bestclock = -1;
							int bestclock = 0xff;
							for (int i=0;i<12;i++){
								if (results[i] != 0x2AA && results[i] != 0x155){
									continue;
								}
								int thisdist = 0;
								for (int j=1;j<12;j++){
									int i2 = (i+j) % 12;
									if ((results[i2] != 0x2AA && results[i2] != 0x155) || results[i2] != results[i]){
										break;
									}
									thisdist++;
								}
								int thisdist2 = 0;
								for (int j=1;j<12;j++){
									int i2 = (i-j+12) % 12;
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
								for (int i=0; i<6;i++) outBuffer[bufcount++] = 0;
							}
							else{
								//								sprintf(outBuffer,"%d: Could not find useable clock phase %02x %02x %02x\n",ichan,results[0],results[3],results[6]);
								//								UART_polled_tx_string( &g_uart, outBuffer );
								outBuffer[bufcount++] = results[0] & 0xff;
								outBuffer[bufcount++] = results[0] >> 8;
								outBuffer[bufcount++] = results[3] & 0xff;
								outBuffer[bufcount++] = results[3] >> 8;
								outBuffer[bufcount++] = results[6] & 0xff;
								outBuffer[bufcount++] = results[6] >> 8;

							}
						}

						// get best phase for each adc
						for (int i=0;i<6;i++){
							int phasecount[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
							for (int j=0;j<8;j++)
								if (phases[i*8+j] >= 0)
									phasecount[phases[i*8+j]]++;
							int maxcount = 0;
							adc_phases[adc_map[i]] = 0;
							for (int j=0;j<12;j++){
								if (phasecount[j] > maxcount){
									maxcount = phasecount[j];
									adc_phases[adc_map[i]] = j;
								}
							}
						}
					}else{
						for (int i=0; i<9*48;i++) outBuffer[bufcount++] = 0;
					}

					// now do bitslip
					for (int i=0;i<6;i++){

						if ((0x1<<i) & ENABLED_ADCS){
							if (clock < 99){
								adc_write(0x16,clock,(0x1<<i));
								adc_write(0x0D,1,(0x1<<i));
							}else{
								adc_write(0x16,adc_phases[i],(0x1<<i));
								adc_write(0x0D,1,(0x1<<i));
							}
						}
					}
					int ichan;
					for (ichan=0;ichan<48;ichan++){
						uint8_t adc_number = adc_map[ichan/8];
						uint64_t thischanmask = (((uint64_t) 0x1)<<ichan);
						if ((thischanmask & all_channel_mask) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++;
							continue;
						}
						if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++;
							continue;
						}
						digi_write(0xb,0x0);
						digi_write(0xe,0x0);
						digi_write(0xd,0x0);
						if (ichan < 16)
							digi_write(0xb,(uint16_t) (0x1<<ichan));
						else if (ichan < 32)
							digi_write(0xe,(uint16_t) (0x1<<(ichan-16)));
						else
							digi_write(0xd,(uint16_t) (0x1<<(ichan-32)));
						int success = 0;
						for (int i=0;i<10;i++){
							digi_write(0x10,0);
							*(registers_0_addr + 0x44) = 1;
							digi_write(0x10,1);
							reset_fabric();

							readout_obloc = 0;
							readout_maxDelay = 50;
							readout_mode = 0;
							readout_wordsPerTrigger = 13;
							readout_numTriggers = 1;
							readout_totalTriggers = 0;

							int delay_count = 0;
							int trigger_count = 0;

							uint16_t lasthit[13];

							read_data2(&delay_count,&trigger_count,lasthit);
							if (trigger_count != 1){
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

							if (ichan < 8)
								digi_write(0x30,((0x1<<(ichan-0))));
							else if (ichan < 16)
								digi_write(0x31,((0x1<<(ichan-8))));
							else if (ichan < 24)
								digi_write(0x32,((0x1<<(ichan-16))));
							else if (ichan < 32)
								digi_write(0x33,((0x1<<(ichan-24))));
							else if (ichan < 40)
								digi_write(0x34,((0x1<<(ichan-32))));
							else if (ichan < 48)
								digi_write(0x35,((0x1<<(ichan-40))));
							digi_write(bitslip0,0x0);
							digi_write(bitslip1,0x0);
							digi_write(bitslip2,0x0);
							digi_write(bitslip3,0x0);
							digi_write(bitslip4,0x0);
							digi_write(bitslip5,0x0);
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
						if (((ichan+1)%8)==0) bufcount++; //gives 6 bytes describing whether the operation is successful
					}

					if (ichan<48){
						for (int i=ichan; i<48; i++) {
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (i % 8))));
							if (((i+1)%8)==0) bufcount++;
						}
					} //match the format if break in between

					// check at the end with mixed frequency ADC
					for (int i=0;i<6;i++){

						if ((0x1<<i) & ENABLED_ADCS){
							if (clock < 99){
								adc_write(0x16,clock,(0x1<<i));
								adc_write(0x0D,0xC,(0x1<<i));
							}else{
								adc_write(0x16,adc_phases[i],(0x1<<i));
								adc_write(0x0D,0xC,(0x1<<i));
							}
						}
					}
					uint64_t error_mask = 0x0;

					for (ichan=0;ichan<48;ichan++){
						uint8_t adc_number = adc_map[ichan/8];
						uint64_t thischanmask = (((uint64_t) 0x1)<<ichan);
						if ((thischanmask & all_channel_mask) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
							continue;
						}
						if (((0x1<<(adc_number)) & ENABLED_ADCS) == 0x0){
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //place holder, unchecked also gives 0
							continue;
						}
						digi_write(0xb,0x0);
						digi_write(0xe,0x0);
						digi_write(0xd,0x0);
						if (ichan < 16)
							digi_write(0xb,(uint16_t) (0x1<<ichan));
						else if (ichan < 32)
							digi_write(0xe,(uint16_t) (0x1<<(ichan-16)));
						else
							digi_write(0xd,(uint16_t) (0x1<<(ichan-32)));

						digi_write(0x10,0);
						*(registers_0_addr + 0x44) = 1;
						digi_write(0x10,1);
						reset_fabric();

						readout_obloc = 0;
						readout_maxDelay = 50;
						readout_mode = 0;
						readout_wordsPerTrigger = 13;
						readout_numTriggers = 1;
						readout_totalTriggers = 0;

						int delay_count = 0;
						int trigger_count = 0;

						uint16_t lasthit[13];

						read_data2(&delay_count,&trigger_count,lasthit);
						if (trigger_count != 1){
							//sprintf(outBuffer,"Didn't get enough triggers: %d\n",trigger_count);
							//UART_polled_tx_string( &g_uart, outBuffer );
							break;
						}
						if (lasthit[12] != 0x319){
							error_mask |= (((uint64_t)0x1)<<ichan);
							//sprintf(outBuffer,"%d FAILED final check: %02x (instead of 0x319)\n",ichan,lasthit[12]);
							//UART_polled_tx_string( &g_uart, outBuffer );
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((ichan+1)%8)==0) bufcount++; //gives 6 bytes describing whether final check is successful
						}
						else{
							outBuffer[bufcount] = outBuffer[bufcount] | ( 1 << (ichan % 8));
							if (((ichan+1)%8)==0) bufcount++;
						}
					}

					if (ichan<48){
						for (int i=ichan; i<48; i++) {
							outBuffer[bufcount] = outBuffer[bufcount] & (~((uint8_t)( 1 << (ichan % 8))));
							if (((i+1)%8)==0) bufcount++;
						}
					} //match the format if break in between

					outBuffer[bufcount++] = ichan;

					//sprintf(outBuffer,"Error mask: %04x%08x\n",((uint32_t)(error_mask>>32)),(uint32_t)(error_mask&0xFFFFFFFF));
					//UART_polled_tx_string( &g_uart, outBuffer );

					outBuffer[bufcount++] = error_mask & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 8 ) & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 16 ) & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 24 ) & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 32 ) & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 40 ) & 0xff;
					outBuffer[bufcount++] = ( error_mask >> 48 ) & 0xff;
					outBuffer[bufcount++] = error_mask >> 56;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == READRATESCMDID){
					uint16_t num_lookback = readU16fromBytes(&buffer[4]);
					uint16_t num_samples = readU16fromBytes(&buffer[6]);
					uint32_t channel_mask1 = readU32fromBytes(&buffer[8]);
					uint32_t channel_mask2 = readU32fromBytes(&buffer[12]);
					uint32_t channel_mask3 = readU32fromBytes(&buffer[16]);

					outBuffer[bufcount++] = READRATESCMDID;
					outBuffer[bufcount++] = (17*48) & 0xff;
					outBuffer[bufcount++] = (17*48) >> 8;

					get_rates(channel_mask1,channel_mask2,channel_mask3,num_lookback,num_samples);
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				} else if (commandID == READDATACMDID){

					uint16_t adc_mode = (uint16_t) ((uint8_t) buffer[4]);
					uint16_t tdc_mode = (uint16_t) ((uint8_t) buffer[5]);
					uint16_t num_lookback = readU16fromBytes(&buffer[6]);
					uint16_t num_samples = readU16fromBytes(&buffer[8]);
					uint32_t num_triggers = readU32fromBytes(&buffer[10]);
					uint32_t channel_mask1 = readU32fromBytes(&buffer[14]);
					uint32_t channel_mask2 = readU32fromBytes(&buffer[18]);
					uint32_t channel_mask3 = readU32fromBytes(&buffer[22]);
					uint8_t clock = (uint8_t) buffer[26];
					uint8_t enable_pulser = (uint8_t) buffer[27];
					uint16_t max_total_delay = readU16fromBytes(&buffer[28]);
					uint8_t mode = buffer[30];

					for (int i=0;i<6;i++){
						if ((0x1<<i) & ENABLED_ADCS){
							if (clock < 99){
								adc_write(0x16,clock,(0x1<<i));
								adc_write(0x0D,adc_mode,(0x1<<i));
							}else{
								adc_write(0x16,adc_phases[i],(0x1<<i));
								adc_write(0x0D,adc_mode,(0x1<<i));
							}
						}
					}

					uint32_t mapped_channel_mask1;
					uint32_t mapped_channel_mask2;
					get_mapped_channels(&channel_mask1,&channel_mask2,&channel_mask3,&mapped_channel_mask1,&mapped_channel_mask2);

					digi_write(0x3,num_samples);
					digi_write(0x4,num_lookback);
					digi_write(0xb,(uint16_t) (mapped_channel_mask1 & 0xFFFF));
					digi_write(0xe,(uint16_t) ((mapped_channel_mask1 & 0xFFFF0000)>>16));
					digi_write(0xd,(uint16_t) (mapped_channel_mask2 & 0xFFFF));
					digi_write(0xa,tdc_mode);
					digi_write(0xc,enable_pulser);

					num_samples = digi_read(0x3);
					enable_pulser = digi_read(0xc);

					char tdc_string[8];
					//if (enable_pulser == 0)
					//	strncpy(tdc_string, "REAL  \0", 7);
					//else
					//	strncpy(tdc_string, "PULSER\0", 7);

					//sprintf(outBuffer,"Setup complete:\nSamples: %d\nLookback: %d\nChannel Mask: %08x\nADC Mode: %d\nTDC Mode: %d\nTrigger on: %s\nClock Mode: %d\nTriggers: %d\n",num_samples,num_lookback,channel_mask1,adc_mode,tdc_mode,tdc_string,clock,num_triggers);
					//UART_polled_tx_string( &g_uart, outBuffer );
					//sprintf(outBuffer,"CM: %08x %08x %08x, ep: %d\n",digi_read(0xb),digi_read(0xe),digi_read(0xd),digi_read(0xc));
					//UART_polled_tx_string( &g_uart, outBuffer );

					outBuffer[bufcount++] = READDATACMDID;
					//outBuffer[bufcount++] = 33;
					outBuffer[bufcount++] = 27+8;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = enable_pulser;
					outBuffer[bufcount++] = num_samples & 0xff;
					outBuffer[bufcount++] = num_samples >> 8;
					outBuffer[bufcount++] = num_lookback & 0xff;
					outBuffer[bufcount++] = num_lookback >> 8;
					outBuffer[bufcount++] = channel_mask1 & 0xff;
					outBuffer[bufcount++] = ( channel_mask1 >> 8) & 0xff;
					outBuffer[bufcount++] = ( channel_mask1 >> 16) & 0xff;;
					outBuffer[bufcount++] = channel_mask1 >> 24;
					outBuffer[bufcount++] = adc_mode & 0xff;
					outBuffer[bufcount++] = adc_mode >> 8;
					outBuffer[bufcount++] = tdc_mode & 0xff;
					outBuffer[bufcount++] = tdc_mode >> 8;
					for (int i=0; i<8; i++)
						outBuffer[bufcount++] = (uint8_t) tdc_string[i];
					outBuffer[bufcount++] = clock;
					outBuffer[bufcount++] = num_triggers & 0xff;
					outBuffer[bufcount++] = ( num_triggers >> 8) & 0xff;
					outBuffer[bufcount++] = ( num_triggers >> 16) & 0xff;;
					outBuffer[bufcount++] = num_triggers >> 24;

					outBuffer[bufcount++] = digi_read(0xb) & 0xff;
					outBuffer[bufcount++] = digi_read(0xb) >> 8;
					outBuffer[bufcount++] = digi_read(0xe) & 0xff;
					outBuffer[bufcount++] = digi_read(0xe) >> 8;
					outBuffer[bufcount++] = digi_read(0xd) & 0xff;
					outBuffer[bufcount++] = digi_read(0xd) >> 8;
					outBuffer[bufcount++] = digi_read(0xc) & 0xff;
					outBuffer[bufcount++] = digi_read(0xc) >> 8;

					delayUs(100);

					// reset fifo
					digi_write(0x10,0);
					*(registers_0_addr + 0x44) = 1;
					digi_write(0x10,1);
					reset_fabric();

					//readout_obloc = 6;
					readout_obloc = 0;
					//sprintf(dataBuffer,"start\n");
					readout_maxDelay = max_total_delay*50;
					readout_mode = mode;
					readout_wordsPerTrigger = 12 + num_samples;
					readout_numTriggers = num_triggers;
					readout_totalTriggers = 0;

					outBuffer[bufcount++] = mode;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

					if (mode == 0){
						int delay_count = 0;
						int trigger_count = 0;

						read_data(&delay_count,&trigger_count);

						//sprintf(&dataBuffer[readout_obloc],"\nend\n");
						//UART_polled_tx_string( &g_uart, dataBuffer );
						bufcount = 0;
						outBuffer[bufcount++] = READDATACMDID;
						outBuffer[bufcount++] = 5 & 0xff;
						outBuffer[bufcount++] = 5 >> 8;
						outBuffer[bufcount++] = (uint8_t)(trigger_count == num_triggers);

						if (trigger_count == num_triggers){
							//sprintf(outBuffer,"SUCCESS! Delayed %d times\n",delay_count);
							outBuffer[bufcount++] = (uint32_t)delay_count & 0xff;
							outBuffer[bufcount++] = ( (uint32_t)delay_count >> 8) & 0xff;
							outBuffer[bufcount++] = ( (uint32_t)delay_count >> 16) & 0xff;;
							outBuffer[bufcount++] = (uint32_t)delay_count >> 24;
						}else{
							//sprintf(outBuffer,"FAILED! Read %d triggers\n",trigger_count);
							outBuffer[bufcount++] = (uint32_t)trigger_count & 0xff;
							outBuffer[bufcount++] = ( (uint32_t)trigger_count >> 8) & 0xff;
							outBuffer[bufcount++] = ( (uint32_t)trigger_count >> 16) & 0xff;;
							outBuffer[bufcount++] = (uint32_t)trigger_count >> 24;
						}

						UART_send(&g_uart, outBuffer ,bufcount );

						//UART_polled_tx_string( &g_uart, outBuffer );

						//get_rates(channel_mask1,channel_mask2,channel_mask3,0,10);
					}else{
						readout_enabled = 1;
						//sprintf(outBuffer,"Run started\n");
						//UART_polled_tx_string( &g_uart, outBuffer );
						outBuffer[0] = RUN_STARTED;
						UART_send(&g_uart, outBuffer ,1);
					}



				}else if (commandID == STOPRUNCMDID){

					outBuffer[bufcount++] = STOPRUNCMDID;
					outBuffer[bufcount++] = 3;
					outBuffer[bufcount++] = 0;
					outBuffer[bufcount++] = readout_enabled;

					if (readout_enabled == 0){
						//sprintf(outBuffer,"Error: no run to stop\n");
						//UART_polled_tx_string( &g_uart, outBuffer );
						outBuffer[bufcount++] = 0;
						outBuffer[bufcount++] = 0;
					}else{
						readout_enabled = 0;
						//sprintf(&dataBuffer[readout_obloc],"\nend\n");
						//UART_polled_tx_string( &g_uart, dataBuffer );
						//sprintf(outBuffer,"Run ended. Read %d triggers\n",readout_totalTriggers);
						//UART_polled_tx_string( &g_uart, outBuffer );
						outBuffer[bufcount++] = readout_totalTriggers & 0xff;
						outBuffer[bufcount++] = readout_totalTriggers >> 8;
					}
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == ADCINITINFOCMDID){

					outBuffer[bufcount++] = ADCINITINFOCMDID;
					outBuffer[bufcount++] = 16;
					outBuffer[bufcount++] = 0;
					for (int i=0; i<16; i++)
						outBuffer[bufcount++] = init_buff[i];
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == PACKAGETESTCMDID){
					outBuffer[bufcount++] = PACKAGETESTCMDID;
					outBuffer[bufcount++] = 0xCA;
					outBuffer[bufcount++] = 1;
					for (int i=0; i<458; i++)
						outBuffer[bufcount++] = i%256;
					UART_polled_tx_string( &g_uart, "monitoring\n" );
					UART_send(&g_uart, outBuffer ,bufcount );

				}

				// If we didn't use the whole buffer, the rest must be the next command
				//memmove(&buffer[0],&buffer[numBytes],writePtr-numBytes);
				for (int j=0;j<writePtr-numBytes;j++){
					buffer[j] = buffer[j+numBytes];
				}
				writePtr -= numBytes;
			}
		}
	}

	return 0;
}

