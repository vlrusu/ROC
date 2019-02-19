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
#include "Commands.h"

#include "utils.h"
#include "version.h"


uint16_t default_gains_cal[96] = {271,275,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,269,269,269,269,269,269,269,268,269,269,269,264,269,269,269,270};
uint16_t default_gains_hv[96] = {270,271,270,270,270,270,270,270,270,270,270,270,270,270,266,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,270,267,270,270,270,270,266,266,272,268,266,265,269,269,269,269,270};
uint16_t default_threshs_cal[96] = {331,316,339,309,317,317,319,335,341,321,319,359,347,320,323,332,352,334,325,339,304,312,340,330,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,305,294,311,307,340,332,321,336,335,327,322,356,344,304,324,325,315,325,325,341,348,343,343,319};
uint16_t default_threshs_hv[96] = {322,284,329,317,325,321,340,347,351,339,348,349,313,337,327,325,345,325,321,320,345,332,305,335,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,320,284,293,324,321,324,331,344,333,314,324,333,346,330,335,327,329,349,316,331,339,318,304,332,310};



const int default_caldac[8] = {1000,1000,1000,1000,1000,1000,1000,1000};

const int calpulse_chanmap[8]={9,1,10,2,11,3,12,4};

const int default_delay = 200;

#define BAUD_RATE                  57600

const uint8_t MCPCALIBCHAN[8] = {1,2,3,4,9,10,11,12};

int main()
{


	SystemCoreClockUpdate();
	UART_init( &g_uart, UART_BASE_ADDRESS, (SYS_M1_CLK_FREQ/(16 * BAUD_RATE))-1, (DATA_8_BITS | NO_PARITY));

	GPIO_init( &g_gpio,    COREGPIO_BASE_ADDR, GPIO_APB_32_BITS_BUS );
//	GPIO_config( &g_gpio, GPIO_0, GPIO_OUTPUT_MODE);
	GPIO_set_output( &g_gpio, GPIO_0, 0);


	const uint8_t greeting[] = "\n\r\"Welcome to the ROC\"\n"
			"\t - Sean Connery\n";
	/* Send greeting message over the UART_0 */
	UART_polled_tx_string( &g_uart, greeting );
	//		sprintf(outBuffer,"Last git revision: %s\n",LAST_GIT_REV);
	//			UART_polled_tx_string( &g_uart, outBuffer );


    /*Initialize the CoreSysService_PF driver*/
    SYS_init(CSS_PF_BASE_ADDRESS);

	uint32_t loopCount = 0;
	uint32_t ledPattern = 0x1;
	_Bool togglecal = 1;

	//register address for bit banging
	registers_0_addr = (volatile uint32_t *) REGISTERBASEADDR;

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

	for (int imcp = 0; imcp < 8; imcp++){
		MCP_pinMode(&preampMCP[MCPCALIB], MCPCALIBCHAN[imcp], MCP_OUTPUT);
	}


	//setup LTC2634, preamp DACs
	for (int idac = 0 ; idac < 96; idac++){
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
	for (int i = 0 ; i < 96 ; i++){
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



	writePtr = 0;

	// set defaults
	UART_polled_tx_string( &g_uart, "Init done\n" );

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
	for (int i=0;i<8;i++)
		AD5318_write(g_spi[3],1,i,default_caldac[i]);
	*(registers_0_addr+12) = 0;

	UART_polled_tx_string( &g_uart, "Initialization completed" );

	GPIO_set_output( &g_gpio, GPIO_0, 0);


	while(1)
	{


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
				uint16_t bufcount = 0;

				if (commandID < MAXCOM_MON)
					UART_polled_tx_string( &g_uart, "monitoring\n" );

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
					outBuffer[bufcount++] = channel & 0xff;
					outBuffer[bufcount++] = channel >> 8;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
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
					outBuffer[bufcount++] = channel & 0xff;
					outBuffer[bufcount++] = channel >> 8;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == SETCALDAC){

					uint8_t chan_mask = (uint8_t) buffer[4];
					uint16_t value = readU16fromBytes(&buffer[5]);
					*(registers_0_addr+0x11) = 1;
					for (int i=0;i<8;i++){
						if (chan_mask & (0x1<<i))
							AD5318_write(g_spi[3],1, i,value);
					}
					*(registers_0_addr+0x11) = 0;
					outBuffer[bufcount++] = SETCALDAC;
					outBuffer[bufcount++] = 3;
					outBuffer[bufcount++] = chan_mask;
					outBuffer[bufcount++] = value & 0xff;
					outBuffer[bufcount++] = value >> 8;
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


					for (int i = 0; i < 16; i++){
						MCP_pinWrite(&preampMCP[MCPCALIB],i+1,0);
					}
					uint8_t chan_mask = (uint8_t) buffer[4];
					pulserOdd = (uint8_t) buffer[5];
					for (int i=0;i<8;i++){
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
					outBuffer[bufcount++] = chan_mask;
					outBuffer[bufcount++] = pulserOdd;
					outBuffer[bufcount++] = dutyCycle & 0xff;
					outBuffer[bufcount++] = (dutyCycle>>8) & 0xff;
					outBuffer[bufcount++] = pulserDelay & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>8) & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>16) & 0xff;
					outBuffer[bufcount++] = (pulserDelay>>24) & 0xff;

					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == SETPULSEROFF){

					PWM_disable(&g_pwm,PWM_1);
					outBuffer[bufcount++] = SETPULSEROFF;
					outBuffer[bufcount++] = 0;

					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == WHOAREYOU){

					outBuffer[bufcount++] = WHOAREYOU;
					outBuffer[bufcount++] = 1;
					outBuffer[bufcount++] = 0;

					UART_send(&g_uart, outBuffer ,bufcount );

				}else if (commandID == RESETROC){
					*(registers_0_addr + 0x10) = 0;
					*(registers_0_addr + 0x10) = 1;
					outBuffer[bufcount++] = RESETROC;
					outBuffer[bufcount++] = 1;



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

					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == DUMPSETTINGS){
					uint16_t channel = (uint16_t) buffer[4];
					outBuffer[bufcount++] = DUMPSETTINGS;
					if (channel>=0 && channel<96){

						outBuffer[bufcount++] = 10;
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
						outBuffer[bufcount++] = 255;
						for (int ic = 0; ic < 96; ic++){
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
					UART_send(&g_uart, outBuffer ,bufcount );


				}else if (commandID == READMONADCS){

					//read currents

					uint32_t rx0;

					outBuffer[bufcount++] = READMONADCS;
					outBuffer[bufcount++] = 32;
					for (int i = 0 ; i < 8; i++){
						SPI_set_slave_select( &g_spi[0] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						uint16_t addr = (i%4 <<11 );
						SPI_transfer_frame( &g_spi[0], addr);
						rx0 = SPI_transfer_frame( &g_spi[0], addr);
						SPI_clear_slave_select( &g_spi[0] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						outBuffer[bufcount++] = rx0 & 0xFF;
						outBuffer[bufcount++] = (rx0 >> 8) & 0x0F;
					}

					for (int i = 0 ; i < 8; i++){
						SPI_set_slave_select( &g_spi[1] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						uint16_t addr = (i%4 <<11 );
						SPI_transfer_frame( &g_spi[1], addr);
						rx0 = SPI_transfer_frame( &g_spi[1], addr);
						SPI_clear_slave_select( &g_spi[1] , (i<4?SPI_SLAVE_0:SPI_SLAVE_1));
						outBuffer[bufcount++] = rx0 & 0xFF;
						outBuffer[bufcount++] = (rx0 >> 8) & 0x0F;
					}


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
					for (int i = 0 ; i < 16; i++)
						outBuffer[bufcount++] = data_buffer[i];
					UART_send(&g_uart, outBuffer ,bufcount );



//				}else if (commandID == READBMES){
//
//					outBuffer[bufcount++] = READBMES;
//					outBuffer[bufcount++] = 24;
//					rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptscal);
//					ptscal.delay_ms(40);
//					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptscal);
//					outBuffer[bufcount++] = comp_data.temperature & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 24) & 0xFF;
//
//					outBuffer[bufcount++] = comp_data.pressure & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 24) & 0xFF;
//
//					outBuffer[bufcount++] = comp_data.humidity & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 24) & 0xFF;
//
//
//					//					sprintf(outBuffer,"CAL %d %d %d\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);
//					//					MSS_UART_polled_tx( &g_mss_uart1, outBuffer, strlen(outBuffer) );
//					rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, &ptshv);
//					ptshv.delay_ms(40);
//					rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, &ptshv);
//					outBuffer[bufcount++] = comp_data.temperature & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.temperature >> 24) & 0xFF;
//
//					outBuffer[bufcount++] = comp_data.pressure & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.pressure >> 24) & 0xFF;
//
//					outBuffer[bufcount++] = comp_data.humidity & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 8) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 16) & 0xFF;
//					outBuffer[bufcount++] = (comp_data.humidity >> 24) & 0xFF;
//					//					sprintf(outBuffer,"HV %d %d %d\n",comp_data.temperature, comp_data.pressure, comp_data.humidity);
//					//					MSS_UART_polled_tx( &g_mss_uart1, outBuffer, strlen(outBuffer) );
//
//
//					UART_send(&g_uart, outBuffer ,bufcount );

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

