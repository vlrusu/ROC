/* Kernel includes. */
#include "hw_platform.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CoreSysServices_PF/core_sysservices_pf.h"

#include "riscv_hal.h"

// ALWAYS COMMENT this define in SoftConsole_variablesize
//#define DTCDDRTEST  // if uncommented, skips all code inside    #ifndef	DTCDDRTEST ... #endif
// if commented, skips all code inside      #ifdef  DTCDDRTEST ... #endif

// Comment this in SoftConsole_variablesize if code does not fit in uSVM
//#define FIXEDSIZETEST  // if commented, skips all code inside    #ifdef	FIXEDSIZETEST ... #endif

//END ALWAYS COMMENT


// Used to reduce size after adding DIGI programming
#define DIGITEST

// Used for commands to uProc from DTC
#define DTCPROGRAM

// Used for commands to uProc from DTC
#define RS485PROGRAM

// Used for code need in DRAC Standalone Tests
#define DRACTEST

// Used for DIGI programming
#define PROGRAMMING

#define IAP

#include "dpuser.h"
#include "dputil.h"
#include "dpDUTspi.h"
#include "dpalg.h"

#include "setup.h"
//#include "Commands.h"

#include "utils.h"
#include "autobitslip.h"
#include "version.h"

#include "iaputils.h"
#include "ilps22qs.h"

//#define BAUD_VALUE                  115200
#define BAUD_VALUE                  57600
//#define BAUD_VALUE 38400

const uint16_t default_caldac[8] = { 1000, 1000, 1000, 1000, 1000, 1000, 1000,
        1000 };

const uint8_t calpulse_chanmap[8] = { 1, 9, 2, 10, 3, 11, 4, 12 };
const uint8_t default_delay = 200;

const uint8_t MCPCALIBCHAN[8] = { 1, 2, 3, 4, 9, 10, 11, 12 };

int main() {

    UART_init(&g_uart, UART_BASE_ADDRESS,
            (SYS_CLK_FREQ / (16 * BAUD_VALUE)) - 1, (DATA_8_BITS | NO_PARITY));

    const uint8_t greeting[] = "\n\r\"Welcome to the ROC\"\n"
            "\t - Sean Connery\n";

    /* Send greeting message over the UART_0 */
    UART_polled_tx_string(&g_uart, greeting);
    //    sprintf(outBuffer,"Last git revision: %s\n",LAST_GIT_REV);
    //    UART_polled_tx_string( &g_uart, outBuffer );

    GPIO_init(&g_gpio, COREGPIO_BASE_ADDR, GPIO_APB_32_BITS_BUS);
    //	GPIO_config( &g_gpio, GPIO_0, GPIO_OUTPUT_MODE);
    GPIO_set_output(&g_gpio, GPIO_0, 0);

    uint8_t readout_enabled = 0;
    uint8_t calibration_enabled = 0;
    int readout_totalTriggers = 0;

    //register address for bit banging
    registers_0_addr = (volatile uint32_t*) REGISTERBASEADDR;

#ifdef DTCPROGRAM
    // MT added  register address for DTC commands
    registers_1_addr = (volatile uint32_t*) DTC_BASE_ADDR;
#endif

#ifdef RS485PROGRAM
    registers_2_addr = (volatile uint32_t*) RS485_BASE_ADDR;
#endif

    // give TWI control to microprocessor before calling DIGI_READ/WRITE or ADC_READ/WRITE
    //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

    //enabling hw counter
    //granularity is clock period=25ns -- period is (gr+1)*1000=50us
    //PWM_PERIOD = PWM_GRANULARITY * (period + 1) = 25 *1000 = 25us
    *(registers_0_addr + REG_TIMERENABLE) = 1;
    *(registers_0_addr + REG_TIMERRESET) = 0;


    /*Initialize the CoreSysService_PF driver*/
    SYS_init(CSS_PF_BASE_ADDRESS);


    SPI_init(&g_spi[0], SPI0_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_spi[0]);
    SPI_init(&g_spi[1], SPI1_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_spi[1]);
    SPI_init(&g_spi[2], SPI2_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_spi[2]);

    SPI_init(&g_spi[3], HVSPI_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_spi[3]);
    SPI_init(&g_spi[4], CALSPI_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_spi[4]);

    SPI_init(&g_cal_pro_spi, SPI_CAL_PROG_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_cal_pro_spi);
    SPI_init(&g_hv_pro_spi, SPI_HV_PROG_BASE_ADDR, 8);
    SPI_configure_master_mode(&g_hv_pro_spi);

    //setup MCPs
    for (int imcp = MCPCAL0; imcp <= MCPFC2; imcp++) {
        if (imcp < MCPHV0)
            MCP_setup(&preampMCP[imcp], g_spi[4], 0, 0x20 + imcp, 0);
        else if (imcp < MCPFC0)
            MCP_setup(&preampMCP[imcp], g_spi[3], 0, 0x20 + imcp - MCPHV0, 0);
        else
            MCP_setup(&preampMCP[imcp], g_spi[3], 1, 0x20 + imcp - MCPFC0, 0);
    }
    MCP_setup(&sensorMCP, g_spi[3], 2, 0x20, 1);

    // outputs for calpulse enable

    for (uint8_t imcp = 0; imcp < 8; imcp++) {
        MCP_pinMode(&preampMCP[MCPCALIB], MCPCALIBCHAN[imcp], MCP_OUTPUT);
    }

    // outputs for fuse control enable, initial all to 0

    for (uint8_t i = 0; i < 48; i++) {
        MCP_pinMode(&preampMCP[MCPFC0 + i / 16], i % 16 + 1, MCP_OUTPUT);
    }

    for (uint8_t i = 0; i < 48; i++) {
        MCP_pinWrite(&preampMCP[MCPFC0 + i / 16], i % 16 + 1, 0);
    }

    //setup LTC2634, preamp DACs
    for (uint8_t idac = 0; idac < 96; idac++) {
        if (idac < 14)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPCAL0], idac + 3,
                    &preampMCP[MCPCAL0], 2, &preampMCP[MCPCAL0], 1);
        else if (idac < 24)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPCAL1], idac - 13,
                    &preampMCP[MCPCAL0], 2, &preampMCP[MCPCAL0], 1);
        else if (idac < 38)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPCAL2], idac - 21,
                    &preampMCP[MCPCAL2], 2, &preampMCP[MCPCAL2], 1);
        else if (idac < 48)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPCAL3], idac - 37,
                    &preampMCP[MCPCAL2], 2, &preampMCP[MCPCAL2], 1);
        else if (idac < 62)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPHV0], idac - 45,
                    &preampMCP[MCPHV0], 2, &preampMCP[MCPHV0], 1);
        else if (idac < 72)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPHV1], idac - 61,
                    &preampMCP[MCPHV0], 2, &preampMCP[MCPHV0], 1);
        else if (idac < 86)
            LTC2634_setup(&dacs[idac], &preampMCP[MCPHV2], idac - 69,
                    &preampMCP[MCPHV2], 2, &preampMCP[MCPHV2], 1);
        else
            LTC2634_setup(&dacs[idac], &preampMCP[MCPHV3], idac - 85,
                    &preampMCP[MCPHV2], 2, &preampMCP[MCPHV2], 1);
    }

    LTC2634_setup(&caldac0, &preampMCP[MCPCAL1], 12, &preampMCP[MCPCAL0], 2,
            &preampMCP[MCPCAL0], 1);
    LTC2634_setup(&caldac1, &preampMCP[MCPCAL1], 13, &preampMCP[MCPCAL0], 2,
            &preampMCP[MCPCAL0], 1);

    // Set default thresholds and gains
    for (uint8_t i = 0; i < 96; i++) {
        strawsCal[i]._ltc = &dacs[i / 2];
        strawsHV[i]._ltc = &dacs[48 + i / 2];
        if (i % 2 == 1) {
            strawsCal[i]._thresh = 1;
            strawsCal[i]._gain = 2;
            strawsHV[i]._thresh = 0;
            strawsHV[i]._gain = 2;
        } else {
            strawsCal[i]._thresh = 0;
            strawsCal[i]._gain = 3;
            strawsHV[i]._thresh = 1;
            strawsHV[i]._gain = 3;
        }
        LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._gain,
                default_gains_cal[i]);
        LTC2634_write(strawsCal[i]._ltc, strawsCal[i]._thresh,
                default_threshs_cal[i]);
        LTC2634_write(strawsHV[i]._ltc, strawsHV[i]._gain, default_gains_hv[i]);
        LTC2634_write(strawsHV[i]._ltc, strawsHV[i]._thresh,
                default_threshs_hv[i]);
    }

    //set default caldac
    for (uint8_t i = 0; i < 4; i++) {
        LTC2634_write(&caldac0, i, default_caldac[i]);
        LTC2634_write(&caldac1, i, default_caldac[i + 4]);
    }



    // make sure DIGIs are powered up
    for (int i = 0; i < 4; i++) {
        digi_write(DG_ADDR_EWS, 0xDEAD, 1);
        uint32_t read_value = digi_read(DG_ADDR_EWS, 1);
        if (read_value != 0xDEAD) {
            *(registers_0_addr + DG_DEVICE_RESET) = 0;

            delayUs(100);
            *(registers_0_addr + DG_DEVICE_RESET) = 1;

            delayUs(1000000);
        } else {
            UART_polled_tx_string(&g_uart, "CAL OK\n");
            break;
        }
    }
    for (int i = 0; i < 4; i++) {
        digi_write(DG_ADDR_EWS, 0xDEAD, 2);
        uint32_t read_value = digi_read(DG_ADDR_EWS, 2);
        if (read_value != 0xDEAD) {
            *(registers_0_addr + DG_DEVICE_RESET) = 0;
            delayUs(100);
            *(registers_0_addr + DG_DEVICE_RESET) = 1;
            delayUs(1000000);
        } else {
            UART_polled_tx_string(&g_uart, "HV OK\n");
            break;
        }
    }



    //initialize the ADCs

    adc_write(ADC_ADDR_PWR, 0x01, 0xFFF);
    adc_write(ADC_ADDR_PWR, 0x00, ENABLED_ADCS);

    uint8_t errors = init_adc(ENABLED_ADCS, 0x02, 0x03);


    adc_write(0x100, 0x71, 0xFFF); // set to 10 bit, 40 MSPS
    adc_write(ADC_ADDR_SRCTRL, 0xC3, 0xFFF); //set to LSB first
    adc_write(0xFF, 0x01, 0xFFF); // latch the above change
    adc_write(0x14, 0x00, 0xFFF); // set to offset binary






    //are we still using those?
    *(registers_0_addr + REG_ROC_EWMSTART_PMT) = 0x0001;
    *(registers_0_addr + REG_ROC_EWMSTOP_PMT) = 0x0FFF;

    *(registers_0_addr + REG_ROC_EWM_T) = 0x13FD;


    //enable serdes reset from DTC
    *(registers_0_addr + 0xB4) = 0xF;



    init_DIGIs();

    uint8_t digierror = 0;
    uint32_t read_value = digi_read(0xC0, 1);
    if ((read_value & 0x3FF) != 0x047){
        digierror = 1;
        UART_polled_tx_string(&g_uart, "ROC->Cal X\n");
    }
    else
        UART_polled_tx_string(&g_uart, "ROC->Cal\n");
    read_value = digi_read(0xC0, 2);
    if ((read_value & 0x3FF) != 0x047){
        UART_polled_tx_string(&g_uart, "ROC->HV X\n");
        digierror = 1;
    }
    else
        UART_polled_tx_string(&g_uart, "ROC->HV\n");
    read_value = *(registers_0_addr + 0xC2);
    if ((read_value & 0xFF) != 0x11)
        UART_polled_tx_string(&g_uart, "CAL->ROC X\n");
    else
        UART_polled_tx_string(&g_uart, "CAL->ROC\n");
    if ((read_value & 0xFF00) != 0x1100)
        UART_polled_tx_string(&g_uart, "HV->ROC X\n");
    else
        UART_polled_tx_string(&g_uart, "HV->ROC\n");

    //try a single reset
    if (digierror) {
        *(registers_0_addr + DG_DEVICE_RESET) = 0;
        *(registers_0_addr + DG_DEVICE_RESET) = 1;

        init_DIGIs();
    }




    UART_polled_tx_string(&g_uart, "Initialization completed");

    GPIO_set_output(&g_gpio, GPIO_0, 0);
    GPIO_set_output(&g_gpio, GPIO_1, 0);

    //give TWI control back to fiber
    //*(registers_0_addr + REG_DIGIRW_SEL) = 0;

    int readout_requestedTriggers = 0;
    int readout_totalDelays = 0;
    uint16_t loopCount = 0;
    uint8_t ledPattern = 0x1;
    _Bool togglecal = 1;

    writePtr = 0;


    readout_totalTriggers = 0;
    while (1) {
        //if (readout_enabled) {
        if (readout_totalTriggers < readout_requestedTriggers) {
            uint16_t remaining = readout_requestedTriggers
                    - readout_totalTriggers;
            if (remaining > 50) {
                readout_numTriggers = 50;
            } else {
                readout_numTriggers = remaining;
            }
            int delay_count = 0;
            int trigger_count = 0;
            //UART_polled_tx_string( &g_uart, "datastream\n" );

            read_data(&delay_count, &trigger_count);
            readout_totalTriggers += trigger_count;
            readout_totalDelays += delay_count;
            if (readout_totalTriggers >= readout_requestedTriggers
                    || readout_totalDelays > readout_maxDelay) {
                if (readout_totalTriggers >= readout_requestedTriggers) {
                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                    UART_send(&g_uart, dataBuffer, 2);
                } else {
                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, EMPTY, 2);
                    UART_send(&g_uart, dataBuffer, 2);
                }
                //sprintf(&dataBuffer[readout_obloc],"\nend\n");
                //UART_polled_tx_string( &g_uart, dataBuffer );
                bufcount = 0;
                outBuffer[bufcount++] = READDATACMDID;
                bufWrite(outBuffer, &bufcount, 5, 2);
                outBuffer[bufcount++] = (uint8_t) (readout_totalTriggers
                        == readout_requestedTriggers);

                if (readout_totalTriggers == readout_requestedTriggers) {
                    //sprintf(outBuffer,"SUCCESS! Delayed %d times\n",delay_count);
                    bufWrite(outBuffer, &bufcount,
                            (uint32_t) readout_totalDelays, 4);
                } else {
                    //sprintf(outBuffer,"FAILED! Read %d triggers\n",trigger_count);
                    bufWrite(outBuffer, &bufcount,
                            (uint32_t) readout_totalTriggers, 4);
                }

                UART_send(&g_uart, outBuffer, bufcount);
                readout_requestedTriggers = 0;
                readout_totalTriggers = 0;
                readout_totalDelays = 0;
            }
        }

        if (loopCount == 20000) {
            ledPattern ^= 0x1;
            GPIO_set_output(&g_gpio, GPIO_0, (uint32_t) ledPattern);

            loopCount = 0;
        }
        delayUs(1);
        loopCount++;


#ifdef RS485PROGRAM

        volatile uint16_t rs485_rx_ready = *(registers_2_addr + CRRS485_RX_READY);

        uint16_t rs485_rx_cmd = RS485_MAX_CMD;
        while (rs485_rx_ready == 1) {   // if put first, it will have priority over "dtc_cmd_ready"??

            // this is the third byte of RS485 RX which encodes the command
            rs485_rx_cmd = *(registers_2_addr + CRRS485_RX_READ);

            // reset RX_READY and send RX acknowledge
            rs485_rx_ready = 0;
            *(registers_2_addr + CRRS485_RX_ACK) = 1;
        }
        *(registers_2_addr + CRRS485_RX_ACK) = 0;

        delayUs(1);


        // fill up 32-bit SPI bus. Firmware will take care of sending out 8-bit at the time
        uint32_t rs485_spi32;
        uint16_t rs485_addr;
        uint16_t rs485_tvs_val[4] = {0};
        uint8_t rs485_j;
        uint8_t rs485_hvcal;
        uint8_t rs485_i;

        volatile uint16_t rs485_tx_busy = 1;

        // Variables read are:


        //	  0: I3.3, 1: I2.5, 2: I1.8HV, 3: IHV5.0, 4: VDMBHV5, 5: V1.8HV, 6: V3.3HV, 7: V2.5,
        //	  8: A0, 9: A1, 10: A2, 11: A3, 12: I1.8CAL, 13: I1.2, 14: ICAL5.0, 15: SPARE,
        //	  16: V3.3, 17: VCAL5.0, 18: V1.8CAL, 19: V1.0, 20: ROCPCBT, 21: HVPCBT, 22: CALPCBT, 23: RTD,
        //	  24: V5KEY, 25: KEYPCBTEMP, 26: DCDCTEMP, 27: V2.5KEY
        //        28: ROC_RAIL_1V, 29: ROC_RAIL_1.8V, 30: ROC_RAIL_2.5V, 31: ROC_TEMP
        //        32: CAL_RAIL_1V, 33: CAL_RAIL_1.8V, 34: CAL_RAIL_2.5V, 35: CAL_TEMP
        //        36: HV_RAIL_1V, 37: HV_RAIL_1.8V, 38: HV_RAIL_2.5V, 39: HV_TEMP


        if (rs485_rx_cmd < RS485_MAX_CMD){


            if (rs485_rx_cmd < RS485_MAX_SPI){
                //there are three monitoring SPI buses. Each has 3 slaves max. And each ADC124s051 has 4 pins
                uint8_t spibus = rs485_rx_cmd / 12;
                uint8_t spislave = (rs485_rx_cmd % 12) / 4;
                uint8_t adcpin = (rs485_rx_cmd % 4);

                uint16_t adcread = ADC124S051_read(&g_spi[spibus], spislave, adcpin);
                *(registers_2_addr + CRRS485_TX_WRITE) = adcread;


            }


            else if (rs485_rx_cmd <  RS485_MAX_SPI + RS485_TVS){
                uint16_t tvscommand = rs485_rx_cmd - RS485_MAX_SPI;
                if (tvscommand < 4){
                    *(registers_0_addr+REG_ROC_TVS_ADDR) = rs485_rx_cmd - RS485_MAX_SPI;
                    delayUs(1);
                    *(registers_2_addr + CRRS485_TX_WRITE)    = *(registers_0_addr + REG_ROC_TVS_VAL);

                }
                else{
                    uint8_t ihvcal = tvscommand / 4;
                    digi_write(DG_ADDR_TVS_ADDR, tvscommand % 4, ihvcal);
                    delayUs(1);
                    *(registers_2_addr + CRRS485_TX_WRITE)   = digi_read(DG_ADDR_TVS_VAL, ihvcal);

                }
            }

            //ILP
            else if (rs485_rx_cmd <  RS485_MAX_SPI + RS485_TVS + RS485_ILP){
                ilp22qs_init();
                uint8_t ret = ilps22qs_id_get();


                ilps22qs_all_sources_t all_sources;
                ilps22qs_all_sources_get(&all_sources);
                ilps22qs_md_t md;
                md.odr = ILPS22QS_4Hz;
                md.avg = ILPS22QS_16_AVG;
                md.lpf = ILPS22QS_LPF_ODR_DIV_4;
                md.fs = ILPS22QS_4060hPa;
                ilps22qs_data_t data;
                ilps22qs_data_get(&md, &data);
                uint16_t ilpcommand = rs485_rx_cmd - RS485_MAX_SPI - RS485_TVS;
                if (ilpcommand == 0 ) {
                    *(registers_2_addr + CRRS485_TX_WRITE) =  data.heat.raw;

                }
                else if (ilpcommand == 1){
                    *(registers_2_addr + CRRS485_TX_WRITE) = (data.pressure.raw & 0xFFFF);

                }
                else if (ilpcommand == 1){
                    *(registers_2_addr + CRRS485_TX_WRITE) = (data.pressure.raw & 0xFFFF0000)>>16;

                }
            }
            else if (rs485_rx_cmd == RS485_TEST_CMD){
                *(registers_2_addr + CRRS485_TX_WRITE) = 0xBEEF;

            }

	    else if (rs485_rx_cmd == RS485_HB_CMD){
	      *(registers_2_addr + CRRS485_TX_WRITE) = loopCount & 0xFFFF;
	    }

        else if (rs485_rx_cmd == RS485_ID_CMD){
            uint8_t nvmadmin[4] = {0x00};
             uint8_t text[280] = {0x0};
            SYS_secure_nvm_read(PANELNVMADDRESS, 0, &nvmadmin[0], text, 252u, 0);
            uint16_t reply = (text[1] << 8) | text[0];
          *(registers_2_addr + CRRS485_TX_WRITE) = reply;
        }

        else {
            *(registers_2_addr + CRRS485_TX_WRITE) = 0xDEAD;
        }

            *(registers_2_addr + CRRS485_TX_START) = 1;
            *(registers_2_addr + CRRS485_TX_START) = 0;


        }




#endif  //RS485PROGRAM

        // this is an example of a DCS WR/RD of addr 64 for a "static" register
        // via DTCInterface/DRACRegister
        /*
 volatile uint16_t example_prev_rd = 0;
 volatile uint16_t example_rd = *(registers_1_addr + CRDCS_EXAMPLE_RD);
 if (example_rd != example_prev_rd) {
         *(registers_1_addr + CRDCS_EXAMPLE_WR) = example_rd;
 }
 example_prev_rd = example_rd;
         */

#ifdef DTCPROGRAM
        //
        // decoding of DCS write command in address space 0x100-200 from DCX_RX_BUFFER:
        // - start on DTC_CMD_READY (driven by SlowControls/DCSRegisters) seen high
        // - ends on CMD_STATUS[15] set high
        //
        //  CMD_STATUS bit (read via DRACRegister register 128) report:
        // bit(15) means end of executing command    (add 0x8000)
        // bit(14) means start of executing command (add 0x4000)
        // bit(13) means end of decoding command    (add 0x2000)
        // bit(12) means start of decoding command (add 0x1000)
        // bit(5)  bad payload: overflowing MAX TX BUFFER SIZE
        // bit(4)  bad command trailer word
        // bit(3)  bad command ID word
        // bit(2)  bad length word: FIFO overflow
        // bit(1)  bad length word: zero length
        // bit(0)  bad command header word

        volatile uint16_t dtc_cmd_ready = *(registers_1_addr + CRDCS_CMD_READY);

        volatile uint16_t get_cmd_rx = 0;
        uint16_t cmd_rx_cnt = 0;
        uint16_t cmd_rx_size = 0;
        uint16_t proc_commandID = 0;
        uint16_t data_to_write = 0;
        uint16_t data_zero = 0;
        uint16_t cmd_fail = 0;
        uint16_t dtcPtr = 0;
        uint16_t dtcbuffer[1024]; // size of DCS_RX(TX)_FIFO in firmware

        while (dtc_cmd_ready == 1) {

            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x1000 + cmd_fail;																	   
            // starts reading DCS_RX_BUFFER contents
            get_cmd_rx = *(registers_1_addr + CRDCS_READ_RX);

            if (cmd_rx_cnt == 0) {
                // upon starting WHILE loop for processor with CMD_READY seen,
                // clear status register and written words register
                *(registers_1_addr + CRDCS_CMD_STATUS) = 0;
                *(registers_1_addr + CRDCS_DIAG_DATA) = 0;
                cmd_fail = 0;
                data_zero = get_cmd_rx;

            } else if (cmd_rx_cnt == 1) {  // must be buffer HEADER word

                if (get_cmd_rx != CMDHEADER)
                    cmd_fail = cmd_fail + 1;

            } else if (cmd_rx_cnt == 2) {  // must be buffer LENGTH word

                cmd_rx_size = get_cmd_rx;
                if (get_cmd_rx == 0) {
                    cmd_fail = cmd_fail + 2;
                } else if (get_cmd_rx >= MAX_BUFFER_SIZE) {
                    // limit DCX_TX_BUFFER size to FIFO size in firmware
                    cmd_fail = cmd_fail + 4;
                    cmd_rx_size = MAX_BUFFER_SIZE - 4;
                }

            } else if (cmd_rx_cnt == 3) {  // must be COMMAND ID word

                proc_commandID = get_cmd_rx & 0x0FFF;
                if ((get_cmd_rx & 0xF000) != 0xC000)
                    cmd_fail = cmd_fail + 8;

            } else if (cmd_rx_cnt == (4 + cmd_rx_size)) { // must be at the end of buffer with TRAILER word

                dtc_cmd_ready = 0;
                *(registers_1_addr + CRDCS_DIAG_DATA) = dtcPtr; // DCS write payload size

                // pass number of payload words to DTCInterface/DRACRegister register 255
                // and CMD_FAIL to DTCInterface/DRACRegister register 128

                if (get_cmd_rx == CMDTRAILER) {
                    *(registers_1_addr + CRDCS_CMD_STATUS) = 0x2000 + cmd_fail;
                } else {
                    cmd_fail = cmd_fail + 16;
                }

            } else {  // must be payload => start filling DTCBUFFER

                // stop filling dtcbuffer when DCS_TX_BUFFER is overflowing
                if (cmd_rx_cnt > (MAX_BUFFER_SIZE - 1)) {
                    cmd_fail = cmd_fail + 32;
                } else {
                    dtcbuffer[dtcPtr] = get_cmd_rx;
                }

                dtcPtr++;

                data_to_write = dtcbuffer[0];
            }

            cmd_rx_cnt++;
        }
        delayUs(1);


        uint8_t  fiber_rw   = 0;        // 0 for read, 1 for write
        uint8_t  adc_num    = 0;
        uint8_t  chan_num   = 0;
        uint8_t  status     = 0xFF;
        uint16_t fiber_value= 0;
        uint16_t preamp_type= 0;
        uint8_t  chan_mask  = 16;
        uint16_t progr_action  = 0;
        uint32_t progr_param_1 = 0;
        uint32_t progr_param_2 = 0;

        switch (proc_commandID) {

        // example of single data output to DCS_TX_BUFFER
        case TESTDCSNGL:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // using only LBS 16-bit of 32 bits APB bus
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = cmd_rx_size;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = data_to_write;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

            // example of block data output to DCS_TX_BUFFER
        case TESTDCSBLK:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // writing only LBS 16-bit of 32 bits APB bus
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = data_to_write + 4;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            // fill up payload
            for (uint16_t i = 0; i < data_to_write; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) = i + 1;
            }
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

        // write to DCS_TX Buffer data as in READMONADCS
        case READSPI:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

             // write header
             *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
             *(registers_1_addr + CRDCS_WRITE_TX) = 36;
             *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;

             // fill up payload with 16-bit words. NB: use only LBS 16-bit of 32 bits APB bus
             uint32_t spi32;
             for (uint8_t i = 0 ; i < 2; i++) {
                 for (uint8_t j = 0 ; j < 12; j++) {
                     SPI_set_slave_select( &g_spi[i] , ((j>=8)?SPI_SLAVE_2:(j<4?SPI_SLAVE_0:SPI_SLAVE_1)));

                     uint16_t addr = (j%4 <<11 );
                     SPI_transfer_frame( &g_spi[i], addr);
                     SPI_transfer_frame( &g_spi[i], addr);
                     spi32 = SPI_transfer_frame( &g_spi[i], addr);

                     SPI_clear_slave_select( &g_spi[i] , ((j>=8)?SPI_SLAVE_2:(j<4?SPI_SLAVE_0:SPI_SLAVE_1)));

                     *(registers_1_addr + CRDCS_WRITE_TX) = (0x0000FFFF & spi32);
                 }
             }

             uint16_t tvs_val[4] = {0};
             for (uint8_t i =0; i<4; i++){
                 *(registers_0_addr+REG_ROC_TVS_ADDR) = i;
                 delayUs(1);
                 tvs_val[i] = *(registers_0_addr + REG_ROC_TVS_VAL);

                 *(registers_1_addr + CRDCS_WRITE_TX) = tvs_val[i];
                 delayUs(1);
             }

             // give TWI control to uProc before calling DIGI_READ/WRITE
             //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

             for (uint8_t ihvcal=1; ihvcal<3; ihvcal++){
                 for (uint8_t i =0; i<4; i++){
                         digi_write(DG_ADDR_TVS_ADDR, i, ihvcal);

                         delayUs(1);
                         tvs_val[i] = digi_read(DG_ADDR_TVS_VAL, ihvcal);

                         *(registers_1_addr + CRDCS_WRITE_TX) = tvs_val[i];
                         delayUs(1);
                 }
             }
             // return TWI control to fiber
             //*(registers_0_addr + REG_DIGIRW_SEL) = 0;
             // write trailer
             *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

        // read back payload from DCS BLOCK WR
       case READBACKBLK:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // writing only LBS 16-bit of 32 bits APB bus
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = cmd_rx_size;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            // fill up payload
            for (uint16_t i = 0; i < cmd_rx_size; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) = dtcbuffer[i];
            }
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

        // write to DCS_TX Buffer data as in GETDEVICEID
        case READDEVICE:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // writing only LBS 16-bit of 32 bits APB bus
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            // not until we can deal with DIGI_WRITE while fiber is in control of DIGI registers
            *(registers_1_addr + CRDCS_WRITE_TX) = 52;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;

            // fill payload with 16-bit words
            uint8_t data_buffer[16];
            uint8_t dinfo_buffer[36];

            status = SYS_get_serial_number(data_buffer, 0);
            status = SYS_get_design_info(dinfo_buffer, 0);

            for (uint16_t i = 0; i < 16; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) =
                        (0x00FF & data_buffer[i]);
            }
            for (uint16_t i = 0; i < 36; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) =
                        (0x00FF & dinfo_buffer[i]);
            }

            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case TALKTOADC:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            uint16_t adcaddr;
            uint16_t adcdata;
            uint8_t result;

            // for testing of ADC_READ:
            // ADC_ADDR_CONFIG = 0x00 should return result=0x18 for either ADCs
            // ADC_ADDR_CID1   = 0x01 should return result=0x08 for (old) ADC9212 and 0x93 for ADC9637
            /*
            fiber_rw = 0;                       // 0 for read, 1 for write
            adc_num = 0;
            //adcaddr =  ADC_ADDR_CID1;   // DOES NOT WORK, RETURNS 0???
            adcaddr =  ADC_ADDR_CONFIG;   // THIS WORKS
             */

            fiber_rw= (uint8_t) dtcbuffer[0];   // -w
            adc_num = (uint8_t) dtcbuffer[1];   // -A
            adcaddr = dtcbuffer[2];             // -a
            adcdata = dtcbuffer[3];             // -d

            // give TWI control to uProc before calling DIGI_READ/WRITE (called by ADC_READ/WRITE)
            //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

            if (fiber_rw == 0) { // for READ, report result from SPI ADC_READ
                result = adc_read(adcaddr, adc_num);
            } else {  // for WRITE, pass
                adc_write(adcaddr, (uint8_t) adcdata, (0x1 << adc_num));
            }

            // return TWI control to fiber
            //*(registers_0_addr + REG_DIGIRW_SEL) = 0;

            // fill DCS_RX_BUFFER
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 1;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = result;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case TALKTODIGI:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // defaults from Python
            fiber_rw = 0;                // (0 for RD, 1 for WR)
            uint8_t digi_hvcal = 0; // for DIGI selection:
                                    //  0 -> both CAL and HV, 1-> CAL only, 2 -> HV only)
                                    //  3 for specific register RW
                                    //  4-15 for ADC R/W to ADC channel 0-11
            uint16_t digi_addr = 16;
            uint32_t digi_data = 16;
            uint16_t data_low = 0;
            uint16_t data_high = 0;
            uint16_t adc_mask = 0;

            // used for testing read of addr=0 (read-only CHIP_configuration) of adc=1
            // Should return 0x18;
            /*
             fiber_rw = 0;
             digi_hvcal = 5;
             digi_addr =  ADC_ADDR_CONFIG;
             */

            fiber_rw    = (uint8_t) dtcbuffer[0];           // -w
            digi_hvcal  = (uint8_t) dtcbuffer[1];           // -h
            digi_addr   = dtcbuffer[2];                   // -a
            data_low    = dtcbuffer[3];                    // -d  (16 LSB)
            data_high   = dtcbuffer[4];                   // -d  (16 MSB)

            digi_data = ((data_high << 16) & 0XFFFF0000) + (data_low & 0x0000FFFF);

            // give TWI control to uProc before calling DIGI_READ/WRITE or ADC/READ_WRITE
            //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

            // Fixed 02/24/2024: was overridingdigi_addrR!!!
            uint16_t adc_addr = digi_addr;
            if (fiber_rw & 0x2) {
                adc_addr |= 0x100;
                //digi_addr |= 0x100;
                fiber_rw &= 0x1;
            }
            if (digi_hvcal < 3) {
                if (fiber_rw == 0) {                  //read
                    digi_data = digi_read(digi_addr, digi_hvcal);
                } else {
                    digi_write(digi_addr, digi_data, digi_hvcal);
                }
            } else {
                if (digi_hvcal == 3) {
                    if (fiber_rw == 0) {                //read
                        digi_data = *(registers_0_addr + digi_addr);
                    } else {
                        *(registers_0_addr + digi_addr) = digi_data;
                    }
                } else {
                    adc_num = digi_hvcal - 4;
                    if (fiber_rw == 0) {
                        digi_data = adc_read(adc_addr, adc_num);
                    } else {
                        adc_mask = (0x1 << adc_num);
                        adc_write(adc_addr, digi_data, adc_mask);
                    }
                }
            }

            // return TWI control to fiber
            //*(registers_0_addr + REG_DIGIRW_SEL) = 0;

            // fill DCS_RX_BUFFER
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 7;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = fiber_rw;
            *(registers_1_addr + CRDCS_WRITE_TX) = digi_hvcal;
            *(registers_1_addr + CRDCS_WRITE_TX) = digi_addr;
            *(registers_1_addr + CRDCS_WRITE_TX) = digi_data & 0xFFFF;
            *(registers_1_addr + CRDCS_WRITE_TX) = (digi_data >> 16);
            *(registers_1_addr + CRDCS_WRITE_TX) = adc_num;
            *(registers_1_addr + CRDCS_WRITE_TX) = adc_mask;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case FINDALIGN:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // give TWI control to uProc before calling DIGI_READ/WRITE or ADC/READ_WRITE
            //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

            //reset the ADCs first, at this point, clock better be stable
            for (int i=0;i<12;i++){
                uint16_t adc_mask = (0x1 << i);
                adc_write(0x08, 3, adc_mask);
            }

            for (int i=0;i<12;i++){
                uint16_t adc_mask = (0x1 << i);
                adc_write(0x08, 0, adc_mask);
            }

            adc_write(ADC_ADDR_SRCTRL, 0xC3, 0xFFF); //set to LSB first

            // defaults from Python
            uint16_t eye_monitor_width = 4;
            uint16_t init_adc_phase = 0;
            uint16_t ifcheck = 1;

            // load parameters from DCS Block Write
            eye_monitor_width   = dtcbuffer[0];             // -w
            init_adc_phase      = dtcbuffer[1];             // -p
            ifcheck             =  dtcbuffer[2];            // -ck
            chan_num    = (uint8_t) dtcbuffer[3];           // -c
            adc_num     = (uint8_t) dtcbuffer[4];           // -a
            channel_mask[0]     = (dtcbuffer[6]<<16)  +  dtcbuffer[5];     // -C
            channel_mask[1]     = (dtcbuffer[8]<<16)  +  dtcbuffer[7];      // -D
            channel_mask[2]     = (dtcbuffer[10]<<16) +  dtcbuffer[9];      // -E

            // correct for parameters out of allowed values
            if (eye_monitor_width > 7) eye_monitor_width = 7;
            if (init_adc_phase > 11) init_adc_phase = 0;

            //
            // start channel mapping
            if (chan_num >=0 && chan_num<96) {
                for (int i=0;i<96;i++) {
                    if (channel_map[i]==chan_num) adc_num = i/8;
                }
            }

            // new function to replace util.py/CHANNELMASK
            if( adc_num > 0 && adc_num<12 ) {
                for (int i=0; i<8; i++) mask_channels(channel_map[8*adc_num+i]);
            }


            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 2+(1+24+96+6+1);   // payload is 130 = 0x82
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;

            *(registers_1_addr + CRDCS_WRITE_TX) = eye_monitor_width;
            *(registers_1_addr + CRDCS_WRITE_TX) = ifcheck;

            //
            // start copy of uncommented sections of AUTOBITSLIP.C
            get_mapped_channels();

            uint8_t iteration = 0;
            uint16_t faulted_adc = 0;

            while (1)
            {
                faulted_adc = 0;

                digi_write(DG_ADDR_BITALIGN_RSETN, 0, 0);
                delayUs(8);
                digi_write(DG_ADDR_BITALIGN_RSETN, 1, 0);

                digi_write(DG_ADDR_SAMPLE,1,0);
                digi_write(DG_ADDR_LOOKBACK,1,0);

                digi_write(DG_ADDR_TRIGGER_MODE,0,0);
                digi_write(DG_ADDR_ENABLE_PULSER,1,0);

                uint16_t activeADC = 0;
                for (uint8_t i=0;i<12;i++){
                    if ((mapped_channel_mask[i/4] >> ((i%4)*8)) & 0xff){
                        activeADC |= (((uint16_t)0x1) << i);
                    }
                }

                //using pattern #9 (0x155/0x2aa) for alignment
                for (uint8_t i=0;i<12;i++){
                    if (((0x1<<i) & ENABLED_ADCS) & activeADC) {
                        //fix all adc at phase = 3 (default 180 deg)
                        adc_write(ADC_ADDR_PHASE,init_adc_phase,(0x1<<i));
                        adc_write(ADC_ADDR_TESTIO,0x9,(0x1<<i));
                    }
                    else //turn off the channels not in use
                        mapped_channel_mask[i/4] &= (~((uint32_t)0xff << ((i%4)*8)));
                }

                *(registers_1_addr + CRDCS_WRITE_TX) = init_adc_phase;


                //turn on pulser, read one word a time
                *(registers_0_addr + REG_ROC_FIFO_HOWMANY) = 1;
                *(registers_0_addr + REG_ROC_EWW_PULSER) = 1;


                //set minimum eye monitor width
                digi_write(DG_ADDR_BITALIGN_EWM_WIDTH, (uint16_t)eye_monitor_width,0);

                uint16_t active_ch[6] = {0};
                active_ch[0] = (uint16_t)(mapped_channel_mask[0] & 0xffff);
                active_ch[1] = (uint16_t)((mapped_channel_mask[0] >> 16) & 0xffff);
                active_ch[2] = (uint16_t)(mapped_channel_mask[1] & 0xffff);
                active_ch[3] = (uint16_t)((mapped_channel_mask[1] >> 16) & 0xffff);
                active_ch[4] = (uint16_t)(mapped_channel_mask[2] & 0xffff);
                active_ch[5] = (uint16_t)((mapped_channel_mask[2] >> 16) & 0xffff);

                digi_write(DG_ADDR_MASK1, active_ch[0], 1);
                digi_write(DG_ADDR_MASK2, active_ch[1], 1);
                digi_write(DG_ADDR_MASK3, active_ch[2], 1);
                digi_write(DG_ADDR_MASK1, active_ch[3], 2);
                digi_write(DG_ADDR_MASK2, active_ch[4], 2);
                digi_write(DG_ADDR_MASK3, active_ch[5], 2);

                for (uint8_t i = 0; i < 6; i++){
                    digi_write(DG_ADDR_RX_CH_MASK1+i%3, active_ch[i], 1+i/3);
                    *(registers_1_addr + CRDCS_WRITE_TX) = active_ch[i];
                }

                //start bit align, set corresponding channel's restart to 1, then back to 0 after 8 ticks
                digi_write(DG_ADDR_BITALIGN_RSTRT, 1, 0);
                delayUs(8);
                digi_write(DG_ADDR_BITALIGN_RSTRT, 0, 0);

                volatile uint16_t completion[6] = {0};
                volatile uint16_t error[6] = {0};

                //wait until all channels are completed; timeout after 30 secs
                for (uint8_t i=0; i<3; i++){
                    hwdelay(50000000);//check every 1 second if is done
                    uint8_t bitalign_done = 1;
                    for (uint8_t j = 0; j < 6; j++){
                        completion[j] = digi_read(DG_ADDR_BITALIGN_CMP1+j%3, 1+j/3);
                        error[j] = digi_read(DG_ADDR_BITALIGN_ERR1+j%3, 1+j/3);
                        if ((completion[j] & active_ch[j]) != active_ch[j])
                            bitalign_done = 0;
                    }

                    if (bitalign_done) break;
                }

                for (uint8_t i=0; i<6; i++){

                    *(registers_1_addr + CRDCS_WRITE_TX) = completion[i];
                    *(registers_1_addr + CRDCS_WRITE_TX) = error[i];

                    if ((~completion[i]) & 0x00ff)
                        faulted_adc |= ((uint16_t)0x1 << (2*i));
                    if ((~completion[i]) & 0xff00)
                        faulted_adc |= ((uint16_t)0x1 << (2*i+1));
                }

                ///////////////////////////////////
                //  DOING BITSLIP                //
                ///////////////////////////////////

                //switch to pattern #1 for bitslip
                for (uint8_t i=0;i<12;i++){
                    if ((0x1<<i) & ENABLED_ADCS) {
                        adc_write(ADC_ADDR_TESTIO,0x1,(0x1<<i));
                    }
                }

                uint8_t bitslipstep[96];
                for (uint8_t ichan=0; ichan<96; ichan++) bitslipstep[ichan] = 0xff;

                volatile uint16_t bitstlip_done[6] = {0};

                for (uint8_t i=0; i<10; i++){
                    hwdelay(50);

                    digi_write(DG_ADDR_BSC_OPERATION_TYPE, 0, 0);
                    digi_write(DG_ADDR_BITSLIP_STRT, 1, 0);
                    delayUs(10);
                    digi_write(DG_ADDR_BITSLIP_STRT, 0, 0);

                    for (uint8_t j = 0; j < 6; j++)
                        bitstlip_done[j] = digi_read(DG_ADDR_BITSLIP_DONE1+j%3, 1+j/3);

                    for (uint8_t ichan=0; ichan<96; ichan++){
                        uint16_t this_chan_mask = (uint16_t) 0x1 << (ichan%16);
                        if (((bitstlip_done[ichan/16] & this_chan_mask) != 0) && (bitslipstep[ichan] == 0xff))
                            bitslipstep[ichan] = i;
                    }
                }

                for (uint8_t i=0; i<6; i++){
                    *(registers_1_addr + CRDCS_WRITE_TX) = bitstlip_done[i];

                    if ((~bitstlip_done[i]) & 0x00ff)
                        faulted_adc |= ((uint16_t)0x1 << (2*i));
                    if ((~bitstlip_done[i]) & 0xff00)
                        faulted_adc |= ((uint16_t)0x1 << (2*i+1));
                }

                for (uint8_t i=0; i<96; i++) {
                    // NB: these are reported as single byte via serial
                    *(registers_1_addr + CRDCS_WRITE_TX) = (uint8_t) bitslipstep[i];
                }


                ///////////////////////////////////
                // CHECK RESULT                  //
                ///////////////////////////////////

                // check with mixed frequency ADC
                uint16_t pattern_match[6] = {0};
                if (ifcheck){
                    for (uint8_t i=0;i<12;i++){
                        if ((0x1<<i) & ENABLED_ADCS) {
                            adc_write(ADC_ADDR_TESTIO,0xC,(0x1<<i));
                        }
                    }

                    digi_write(DG_ADDR_BSC_OPERATION_TYPE, 1, 0);
                    digi_write(DG_ADDR_BITSLIP_STRT, 1, 0);
                    delayUs(10);
                    digi_write(DG_ADDR_BITSLIP_STRT, 0, 0);

                    for (uint8_t j = 0; j < 6; j++){
                        pattern_match[j] = digi_read(DG_ADDR_BSC_PATTERN_MATCH1+j%3, 1+j/3);
                        *(registers_1_addr + CRDCS_WRITE_TX) = pattern_match[j];

                        if ((pattern_match[j] & active_ch[j] & 0x00ff) != (active_ch[j] & 0x00ff))
                            faulted_adc |= ((uint16_t)0x1 << (2*j));
                        if ((pattern_match[j] & active_ch[j] & 0xff00) != (active_ch[j] & 0xff00))
                            faulted_adc |= ((uint16_t)0x1 << (2*j+1));
                    }
                }
                else{
                    for (uint8_t i=0;i<6;i++) *(registers_1_addr + CRDCS_WRITE_TX) = 0;
                }

                // ALLOW ONLY ONE ITERATION! Return faulted_adc status
                *(registers_1_addr + CRDCS_WRITE_TX) = faulted_adc;
                break;

                iteration ++;

                if (faulted_adc == 0) break;
                if (iteration == 6) break; //maximum: 3 trials at phase 0, 3 for faulted adc at different phase
                if (iteration == 3) init_adc_phase = (init_adc_phase+3)%12;
                if (iteration >= 3){
                    for (uint8_t i=0; i<3; i++) mapped_channel_mask[i] = 0;

                    for (uint8_t i=0; i<12; i++){
                        if (faulted_adc & ((uint16_t)0x1 << (i))){
                            mapped_channel_mask[i/4] |= ((uint32_t)0x000000ff)<<((i%4)*8);
                        }
                    }
                }

            } //end while(1)

            for (uint8_t i=0;i<12;i++){
                if ((0x1<<i) & ENABLED_ADCS) {
                    adc_write(ADC_ADDR_TESTIO,0x0,(0x1<<i));
                }
                //make sure find_alignment exits with tracking incoming data not ADC pattern
            }
            // return TWI control to fiber
            // *(registers_0_addr + REG_DIGIRW_SEL) = 0;

            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;
            break;


        case READDATA:
            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            *(registers_0_addr + 0xEE) = 0x1;
            delayUs(1);
            *(registers_0_addr + 0xEE) = 0x0;

            uint16_t adc_mode;
            uint16_t tdc_mode;
            uint16_t num_lookback;
            uint32_t num_triggers;
            uint16_t num_samples;
            uint8_t enable_pulser;
            uint8_t marker_clock;
            uint8_t mode;
            uint8_t clock;

            /*
             // pass test values as in command
             // read -a 0 -t 0 -s 1 -l 8 -T 10 -m 3 -p 1 -C 0 -D 1400 -E 880000
             adc_mode = 0;
             tdc_mode = 0;
             num_samples = 1;
             num_lookback = 8;
             num_triggers = 10;
             marker_clock = 3;
             enable_pulser = 1;
             channel_mask[0] = 0x00000000;
             channel_mask[1] = 0x1400;
             channel_mask[2] = 0x88000000;
             */

            // load parameters - Vadim says to ignore "message" (-M) and "chan_num" (-c)
            // also "max_total_delay" (-d) is not used
            adc_mode       = dtcbuffer[0];                             // -a
            tdc_mode       = dtcbuffer[1];                             // -t
            num_lookback   = dtcbuffer[2];                             // -l
            num_samples    = dtcbuffer[3];                             // -s
            num_triggers   = (dtcbuffer[5]<<16)  +  dtcbuffer[4];      // -T
            channel_mask[0]= (dtcbuffer[7]<<16)  +  dtcbuffer[6];      // -C
            channel_mask[1]= (dtcbuffer[9]<<16)  +  dtcbuffer[8];      // -D
            channel_mask[2]= (dtcbuffer[11]<<16) +  dtcbuffer[10];     // -E
            enable_pulser  = (uint8_t) dtcbuffer[12];                  // -p
            marker_clock   = (uint8_t) dtcbuffer[13];                  // -m
            mode           = (uint8_t) dtcbuffer[14];
            clock          = (uint8_t) dtcbuffer[15];

            if (marker_clock & 0x1)
                *(registers_0_addr + REG_ROC_ENABLE_FIBER_CLOCK) = 1;
            else
                *(registers_0_addr + REG_ROC_ENABLE_FIBER_CLOCK) = 0;

            if (marker_clock & 0x2)
                *(registers_0_addr + REG_ROC_ENABLE_FIBER_MARKER) = 1;
            else
                *(registers_0_addr + REG_ROC_ENABLE_FIBER_MARKER) = 0;

            // give TWI control to uProc before calling DIGI_READ/WRITE or ADC/READ_WRITE
            //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

            digi_write(DG_ADDR_ENABLE_PULSER,enable_pulser,0);
            if ((mode & 0x1) == 0x0){
                get_mapped_channels();
                uint32_t used_adcs = 0x0;
                for (int i=0;i<12;i++){
                    uint32_t test_mask = (0xFF<<(8*(i%4)));
                    if (mapped_channel_mask[i/4] & test_mask)
                        used_adcs |= (0x1<<i);
                }
                //used_adcs = 0x800;
                for (uint8_t i=0;i<12;i++){
                    if ((0x1<<i) & ENABLED_ADCS){
                        if (clock < 99){
                            //if ((0x1<<i) & used_adcs)
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

                // DISABLE CALIBRATION BEFORE ANY CHANGES TO SETTINGS
                digi_write(0x0F,0,0); // disable calibration

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

                *(registers_0_addr + REG_ROC_EWW_PULSER) = 0;

                *(registers_0_addr + REG_ROC_FIFO_HOWMANY) = num_samples;

                if ((mapped_channel_mask[2]!=0)||((mapped_channel_mask[1]>>16)!=0))
                    hvcal = 2;

                num_samples = digi_read(DG_ADDR_SAMPLE, hvcal);
                enable_pulser = digi_read(DG_ADDR_ENABLE_PULSER, hvcal);
            }

            delayUs(10000);

            if ((mode & 0x1) == 0x0){
                // reset fifo
                resetFIFO();
                resetFIFO();
                delayUs(10000);
                *(registers_0_addr + REG_ROC_EWW_PULSER) = 1;


                digi_write(0x0F,enable_pulser,0); // enable calibration IF running internal pulser
                /*    Vadim says to ignore this
                 readout_obloc = 0;
                 readout_maxDelay = max_total_delay*50;
                 readout_mode = mode;
                 readout_wordsPerTrigger = 8;//NUMTDCWORDS + 4*num_samples;
                 readout_numTriggers = num_triggers;
                 readout_totalTriggers = 0;

                 readout_noUARTflag = 0;
                 */
            }

            // Vadim says to ignore the rest of READDATACMDID
            /*
                      } else if (commandID == SETPREAMPGAIN){
                          uint16_t channel = readU16fromBytes(&buffer[4]);
                          uint16_t fiber_value = readU16fromBytes(&buffer[6]);

                          setPreampGain(channel, fiber_value);

                          outBuffer[bufcount++] = SETPREAMPGAIN;
                          bufWrite(outBuffer, &bufcount, 4, 2);
                          bufWrite(outBuffer, &bufcount, channel, 2);
                          bufWrite(outBuffer, &bufcount, fiber_value, 2);
                          outBufSend(g_uart, outBuffer, bufcount);

                          // Set preamp threshold
                      }else if (commandID == SETPREAMPTHRESHOLD){
                          uint16_t channel = readU16fromBytes(&buffer[4]);

                          uint16_t fiber_value = readU16fromBytes(&buffer[6]);

                          setPreampThreshold(channel, fiber_value);

                          outBuffer[bufcount++] = SETPREAMPTHRESHOLD;
                          bufWrite(outBuffer, &bufcount, 4, 2);
                          bufWrite(outBuffer, &bufcount, channel, 2);
                          bufWrite(outBuffer, &bufcount, fiber_value, 2);
                          outBufSend(g_uart, outBuffer, bufcount);
              */
             // return TWI control to fiber
             //*(registers_0_addr + REG_DIGIRW_SEL) = 0;

             *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
             *(registers_1_addr + CRDCS_WRITE_TX) = 20;
             *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
             *(registers_1_addr + CRDCS_WRITE_TX) = adc_mode;
             *(registers_1_addr + CRDCS_WRITE_TX) = tdc_mode;
             *(registers_1_addr + CRDCS_WRITE_TX) = num_lookback;
             *(registers_1_addr + CRDCS_WRITE_TX) = num_samples;
             *(registers_1_addr + CRDCS_WRITE_TX) = (num_triggers & 0xFFFF);
             *(registers_1_addr + CRDCS_WRITE_TX) = (num_triggers & 0xFFFF0000)>>16;
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[0] & 0xFFFF);
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[0] & 0xFFFF0000)>>16;
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[1] & 0xFFFF);
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[1] & 0xFFFF0000)>>16;
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[2] & 0xFFFF);
             *(registers_1_addr + CRDCS_WRITE_TX) = (channel_mask[2] & 0xFFFF0000)>>16;
             *(registers_1_addr + CRDCS_WRITE_TX) = enable_pulser;
             *(registers_1_addr + CRDCS_WRITE_TX) = marker_clock;
             *(registers_1_addr + CRDCS_WRITE_TX) = mode;
             *(registers_1_addr + CRDCS_WRITE_TX) = clock;
             *(registers_1_addr + CRDCS_WRITE_TX) = digi_read(DG_ADDR_MASK1,hvcal);
             *(registers_1_addr + CRDCS_WRITE_TX) = digi_read(DG_ADDR_MASK2,hvcal);
             *(registers_1_addr + CRDCS_WRITE_TX) = digi_read(DG_ADDR_MASK3,hvcal);
             *(registers_1_addr + CRDCS_WRITE_TX) = digi_read(DG_ADDR_ENABLE_PULSER,hvcal);
             *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

              // update CMD_STATUS
             *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

              break;


        // set PREAMP GAINS
        case PREAMPGAIN:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            /*
             // for testing by hand
             chan_num  = 1;
             fiber_value = 370;   // N.B. Expect that float to integer conversion has already happened!
             preamp_type = 1;
             */

            chan_num    = (uint8_t) dtcbuffer[0];     // -c
            fiber_value =           dtcbuffer[1];     // -d
            preamp_type =           dtcbuffer[2];     // -hv

            if (preamp_type == 1)
                chan_num = chan_num + 96;

            setPreampGain(chan_num, fiber_value);

            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 3;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = chan_num;
            *(registers_1_addr + CRDCS_WRITE_TX) = fiber_value;
            *(registers_1_addr + CRDCS_WRITE_TX) = preamp_type;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

        // set PREAMP THRESHOLD
        case PREAMPTHRESH:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // for testing by hand
            /*
             chan_num  = 1;
             fiber_value = 450;   // N.B. Expect that float to integer conversion has already happened!
             preamp_type = 1;
             */

            chan_num    = (uint8_t) dtcbuffer[0];     // -c
            fiber_value =           dtcbuffer[1];     // -d
            preamp_type =           dtcbuffer[2];     // -hv

            if (preamp_type == 1)
                chan_num = chan_num + 96;

            setPreampThreshold(chan_num, fiber_value);

            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 3;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = chan_num;
            *(registers_1_addr + CRDCS_WRITE_TX) = fiber_value;
            *(registers_1_addr + CRDCS_WRITE_TX) = preamp_type;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case PULSERON:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // initialize as in Python defaults:
            // - ignore oddoreven and pulser_odd (unused)
            // - ignore definition of pulserDelay using the frequency parameter
            uint8_t chan_mask = 16;
            uint16_t dutyCycle = 10;
            uint32_t pulserDelay = 1000;

            chan_mask   = (uint8_t) dtcbuffer[0];   // -C
            dutyCycle   =           dtcbuffer[1];   // -y
            pulserDelay = (dtcbuffer[3]<<16)  +  dtcbuffer[2];   // -d  or  integer

            for (uint8_t i = 0; i < 16; i++) {
                MCP_pinWrite(&preampMCP[MCPCALIB], i + 1, 0);
            }

            for (uint8_t i = 0; i < 8; i++) {
                if ((0x1 << i) & chan_mask) {
                    MCP_pinWrite(&preampMCP[MCPCALIB], calpulse_chanmap[i], 1);
                }
            }

            //granularity is clock period=25ns -- period is (gr+1)*1000=50us
            //PWM_PERIOD = PWM_GRANULARITY * (period + 1) = 25 *1000 = 25us
            PWM_init(&g_pwm, COREPWM_BASE_ADDR, 1, pulserDelay);
            PWM_set_duty_cycle(&g_pwm, PWM_1, dutyCycle); //duty cycle is 4 x 25 = 100ns

            PWM_enable(&g_pwm, PWM_1);

            // write output for BLOCK_READ
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 4;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            *(registers_1_addr + CRDCS_WRITE_TX) = chan_mask;
            *(registers_1_addr + CRDCS_WRITE_TX) = dutyCycle;
            *(registers_1_addr + CRDCS_WRITE_TX) = (pulserDelay & 0xFFFF);
            *(registers_1_addr + CRDCS_WRITE_TX) = (pulserDelay & 0xFFFF0000)>> 16;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case PULSEROFF:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            PWM_disable(&g_pwm, PWM_1);

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case MEASURETHRESH:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // retrieve the parameters passed via BLOCK_WRITE
            channel_mask[0] = (dtcbuffer[1]<<16)  +  dtcbuffer[0];     // -C
            channel_mask[1] = (dtcbuffer[3]<<16)  +  dtcbuffer[2];     // -D
            channel_mask[2] = (dtcbuffer[5]<<16)  +  dtcbuffer[4];     // -E

            // this returns MAPPED_CHANNEL_MASK
            get_mapped_channels();

            // give TWI control to uProc before calling DIGI_READ/WRITE or ADC/READ_WRITE
            //*(registers_0_addr + REG_DIGIRW_SEL) = 1;

            //enable pulser as "readstrawcmd" does
            digi_write(DG_ADDR_ENABLE_PULSER, 1, 0);

            uint16_t threshold_array[288];
            for (uint16_t i = 0; i < 288; i++)
                threshold_array[i] = 0xFFFF;

            for (uint8_t ihvcal = 1; ihvcal < 3; ihvcal++) {
                for (uint8_t k = (48 * (ihvcal - 1)); k < 48 * ihvcal; k++) {
                    uint8_t condition = 0;
                    uint8_t straw_num = channel_map[k];
                    if (ihvcal == 1)
                        condition = ((k < 32
                                && ((0x1 << k) & mapped_channel_mask[0]))
                                || (k >= 32
                                        && ((0x1 << (k - 32))
                                                & mapped_channel_mask[1])));
                    else if (ihvcal == 2)
                        condition = ((k < 64
                                && ((0x1 << (k - 32)) & mapped_channel_mask[1]))
                                || (k >= 64
                                        && ((0x1 << (k - 64))
                                                & mapped_channel_mask[2])));

                    if (condition) {
                        uint16_t gain_cal[3] = { 0,
                                default_gains_cal[straw_num],
                                default_gains_cal[straw_num] };
                        uint16_t gain_hv[3] = { default_gains_hv[straw_num], 0,
                                default_gains_hv[straw_num] };
                        //first zero cal, then hv, then both to default

                        digi_write(DG_ADDR_SELECTSMA, k % 48, ihvcal);
                        //select channel
                        for (uint8_t i = 0; i < 3; i++) {
                            setPreampGain(straw_num, gain_cal[i]);
                            setPreampGain(straw_num + 96, gain_hv[i]);
                            hwdelay(500000); //wait for 10ms gain to reach written value and for SMA to settle

                            digi_write(DG_ADDR_SMARDREQ, 1, ihvcal);
                            delayUs(5); //write READREQ to 1 and freeze SMA module

                            threshold_array[96 * i + straw_num] = digi_read(DG_ADDR_SMADATA, ihvcal);
                            digi_write(DG_ADDR_SMARDREQ, 0, ihvcal); //unfreeze SMA module
                        }
                    }
                }
            }
            // return TWI control to fiber
            //*(registers_0_addr + REG_DIGIRW_SEL) = 0;

            // write output for BLOCK_READ
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 288;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            for (uint16_t i = 0; i < 288; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) = threshold_array[i];
            }
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case READRATES:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // initialize as in Python defaults
            uint16_t lookback_number  = 100;
            uint16_t samples_number   = 10;

            // retrieve the parameters passed via BLOCK_WRITE
            lookback_number = dtcbuffer[0];                            // -d
            samples_number  = dtcbuffer[1];                            // -s
            channel_mask[0] = (dtcbuffer[3]<<16)  +  dtcbuffer[2];     // -C
            channel_mask[1] = (dtcbuffer[5]<<16)  +  dtcbuffer[4];     // -D
            channel_mask[2] = (dtcbuffer[7]<<16)  +  dtcbuffer[6];     // -E

            // MUST initialize GET_RATES outputs!
            uint32_t total_time_counts[2] = {0};
            uint32_t total_hv[96] = {0};
            uint32_t total_cal[96] = {0};
            uint32_t total_coinc[96] = {0};
            uint32_t timecounts;

            // give TWI control to uProc before calling DIGI_READ/WRITE or ADC/READ_WRITE
            *(registers_0_addr + REG_DIGIRW_SEL) = 1;

            get_rates(lookback_number,samples_number,0, &timecounts, total_hv, total_cal, total_coinc, total_time_counts);

            // return TWI control to fiber
            *(registers_0_addr + REG_DIGIRW_SEL) = 0;

            // write output for BLOCK_READ
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            *(registers_1_addr + CRDCS_WRITE_TX) = 2*290;
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            for (uint16_t i=0; i<96; i++) {
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_hv[i] & 0xFFFF);
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_hv[i] & 0xFFFF0000)>>16;
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_cal[i] & 0xFFFF);
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_cal[i] & 0xFFFF0000)>>16;
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_coinc[i] & 0xFFFF);
                *(registers_1_addr + CRDCS_WRITE_TX) =  (total_coinc[i] & 0xFFFF0000)>>16;
            }
            *(registers_1_addr + CRDCS_WRITE_TX) = (total_time_counts[0] & 0xFFFF);
            *(registers_1_addr + CRDCS_WRITE_TX) = (total_time_counts[0] & 0xFFFF0000)>>16;
            *(registers_1_addr + CRDCS_WRITE_TX) = (total_time_counts[1] & 0xFFFF);
            *(registers_1_addr + CRDCS_WRITE_TX) = (total_time_counts[1] & 0xFFFF0000)>>16;
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case READGITREV:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            fibercount = 0;
            fiberBuffer[fibercount++] = CMDHEADER;

            // save index to overwrite and fill payload size with temporary value
            fibercount_place_holder = fibercount;
            fiberBuffer[fibercount++] = fibercount;

            fiberBuffer[fibercount++] = 0xC000 + proc_commandID;


            for (uint8_t i = 0; i < sizeof(LAST_GIT_REV)-1; i++){
                fiberBuffer[fibercount++] = LAST_GIT_REV[i];
            }

            fiberBuffer[fibercount++] = CMDTRAILER;

            // overwrite final payload size value
            if (fibercount > 4) {
                fiberBuffer[fibercount_place_holder] =  fibercount - 4;
            } else {
                fiberBuffer[fibercount_place_holder] = 1;
            }

            // pass FIBERBUFFER to output FIFO
            for (uint8_t ib = 0; ib < fibercount; ib++) {
                *(registers_1_addr + CRDCS_WRITE_TX) = fiberBuffer[ib];
            }

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;


        case READILP:

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            // payload word
            *(registers_1_addr + CRDCS_WRITE_TX) = 4;
            // CMD ID
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;

            // payload
            ilp22qs_init();
            uint8_t ret = ilps22qs_id_get();
            *(registers_1_addr + CRDCS_WRITE_TX) = ret;

            ilps22qs_all_sources_t all_sources;
            ilps22qs_all_sources_get(&all_sources);
            ilps22qs_md_t md;
            md.odr = ILPS22QS_4Hz;
            md.avg = ILPS22QS_16_AVG;
            md.lpf = ILPS22QS_LPF_ODR_DIV_4;
            md.fs = ILPS22QS_4060hPa;
            ilps22qs_data_t data;
            ilps22qs_data_get(&md, &data);
            *(registers_1_addr + CRDCS_WRITE_TX) = data.heat.raw;
            *(registers_1_addr + CRDCS_WRITE_TX) = (data.pressure.raw & 0xFFFF);
            *(registers_1_addr + CRDCS_WRITE_TX) = (data.pressure.raw & 0xFFFF0000)>>16;

            //trailer
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

        case GETKEY:
            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // header
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            // payload word
            *(registers_1_addr + CRDCS_WRITE_TX) = 4;
            // CMD ID
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            // payload of diagnostic data

            uint16_t key_temp = ADC124S051_read(&g_spi[2], 0, 1);
            uint16_t v2p5 = ADC124S051_read(&g_spi[2], 0, 3);
            uint16_t v5p1 = ADC124S051_read(&g_spi[2], 0, 0);
            uint16_t dcdctemp = ADC124S051_read(&g_spi[2], 0, 2);

            *(registers_1_addr + CRDCS_WRITE_TX) = key_temp;
            *(registers_1_addr + CRDCS_WRITE_TX) = v2p5;
            *(registers_1_addr + CRDCS_WRITE_TX) = v5p1;
            *(registers_1_addr + CRDCS_WRITE_TX) = dcdctemp;

            //trailer
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

            break;

            // set gains and thresholds for all 96 CAL&HV channels
            // expected order is: 96xgain CAL, 96xgain HV, 96xthres CAL, 96xthres HV
            case WRITEPREAMP:

                // update CMD_STATUS
                *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

                for (uint8_t i=0;i<96;i++) {
                    setPreampGain(i, dtcbuffer[i]);
                    setPreampGain(i+96,dtcbuffer[i+96]);
                    setPreampThreshold(i,dtcbuffer[i+2*96]);
                    setPreampThreshold(i+96,dtcbuffer[i+3*96]);
                }

                // returns only 0x8000 from reg=128 and 0x1000 from reg=129

                // update CMD_STATUS
                *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

                break;


             case WRITECALDAC:

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

                 // assume that CHAN_MASK and FIBER_VALUE parameters are derived from other Python command parameters as:
                 // 1) if (channel >=0) chan_mask |= (0x1 << channel); where "channel" is passed by -c option
                 // 2) if (fvalue >=0)  fiber_value = fvalue/3.3 * 1023; where "fvalue" is passed by  -v option
                 chan_mask      = (uint8_t) dtcbuffer[0];   // -C
                 fiber_value    =           dtcbuffer[1];   // -d

                 for (uint8_t i = 0; i < 8; i++) {
                     if (chan_mask & (0x1 << i)) {
                         if (i < 4)
                             LTC2634_write(&caldac0, i, fiber_value);
                         else
                             LTC2634_write(&caldac1, i - 4, fiber_value);
                     }
                 }

                 // write copy of inputs as output for BLOCK_READ
                 *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
                 *(registers_1_addr + CRDCS_WRITE_TX) = 2;
                 *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
                 *(registers_1_addr + CRDCS_WRITE_TX) = chan_mask;
                 *(registers_1_addr + CRDCS_WRITE_TX) = fiber_value;
                 *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

                 break;


             case READPREAMP:

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

                 // pass channel number
                 chan_num   = (uint8_t) dtcbuffer[0];   // -c

                 if (chan_num >= 0 && chan_num < 96) {

                     *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
                     *(registers_1_addr + CRDCS_WRITE_TX) = 4;
                     *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
                     *(registers_1_addr + CRDCS_WRITE_TX) = default_gains_cal[chan_num];
                     *(registers_1_addr + CRDCS_WRITE_TX) = default_gains_hv[chan_num];
                     *(registers_1_addr + CRDCS_WRITE_TX) = default_threshs_cal[chan_num];
                     *(registers_1_addr + CRDCS_WRITE_TX) = default_threshs_hv[chan_num];
                     *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;
                 }

                 else {

                     *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
                     *(registers_1_addr + CRDCS_WRITE_TX) = 384;
                     *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;

                     for (uint8_t ic = 0; ic < 96; ic++) {
                         *(registers_1_addr + CRDCS_WRITE_TX) = default_gains_cal[ic];
                         *(registers_1_addr + CRDCS_WRITE_TX) = default_gains_hv[ic];
                         *(registers_1_addr + CRDCS_WRITE_TX) = default_threshs_cal[ic];
                         *(registers_1_addr + CRDCS_WRITE_TX) = default_threshs_hv[ic];
                     }
                     *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;
                 }

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

                 break;


             case INITBYFIBER:

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

                 uint16_t calerr = 0;
                 uint16_t hverr = 0;

                 // make sure DIGIs are powered up
                 digi_write(DG_ADDR_EWS, 0xDEAD, 1);
                 uint32_t read_value = digi_read(DG_ADDR_EWS, 1);
                 if (read_value != 0xDEAD) {
                     calerr |= 0x1;
                 }
                 digi_write(DG_ADDR_EWS, 0xDEAD, 2);
                 read_value = digi_read(DG_ADDR_EWS, 2);
                 if (read_value != 0xDEAD) {
                     hverr |= 0x1;
                 }

                 init_DIGIs();

                 *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
                 *(registers_1_addr + CRDCS_WRITE_TX) = 2;
                 *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
                 *(registers_1_addr + CRDCS_WRITE_TX) = calerr;
                 *(registers_1_addr + CRDCS_WRITE_TX) = hverr;
                 *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

                 // update CMD_STATUS
                 *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

                 break;


                 // define global parameters for a remote programming function
                case REMOTEPROG:

                    // update CMD_STATUS
                    *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

                    // reset PROG_RETURN variable;
                    *(registers_1_addr + CRCDS_PROG_RETURN) = 0;

                    // read ACTION from first 16-bit word
                    progr_action = dtcbuffer[0];

                    // set variable PROGR_PARAM_1 from second and third 16-bit word of BLOCK_WRITE
                    progr_param_1 = (dtcbuffer[2]<<16) + dtcbuffer[1];

                    // set variable PROGR_PARAM_2 from fourth and fifth 16-bit word of BLOCK_WRITE
                    progr_param_2 = (dtcbuffer[4]<<16) + dtcbuffer[3];

                    uint32_t startAddr  = 0;
                    uint32_t nBytes     = 0;
                    uint32_t nWords     = 0;
                    uint32_t fileSizeinbyte = 0;
                    uint16_t failMask   = 0;
                    uint16_t failCount  = 0;
                    uint32_t spiaddr_or_idx = 0;
                    uint16_t nImages    = 0;
                    uint8_t cmd         = 0;
                    uint8_t  manufacturer_id;
                    uint8_t  device_id;

                    uint32_t erase_address = 0;
                    uint32_t erase_count=0;

                    // FLASH_READ_BUF contains words read from FLASH: max number is 254*2
                    uint16_t flash_read_buf[512] = {0};
                    // FLASH_WRITE_BUF contains words to write to FLASH as uint8_t: max number is 1024 (1KB)
                    uint8_t flash_write_buf[1024] = {0};
                    // DIR_BUFFER contains starting address for up to 16 images written to FLASH (see FLASH_MAP file)
                    // Is is filled starting with the sixth word of BLOCK_WRITE
                    uint32_t dir_buffer[16] = {0};

                    //clear_iap_data_buffer();

                    // start responding to Remote Programming ACTIONS
                    switch (progr_action) {
                        case 1:
                            //execute_bitstream_authenticate(rindex);
                            break;

                        case 2:
                            //execute_iap_image_authenticate(rindex);
                            break;

                        case 3:
                            startAddr       = progr_param_1;
                            fileSizeinbyte  = progr_param_2;

                            FLASH_init();

                            FLASH_global_unprotect();
                            FLASH_read_device_id
                            (
                                    &manufacturer_id,
                                    &device_id
                            );

                            uint32_t file_size_in_blocks = 1 + fileSizeinbyte / 65536;
                            erase_address = startAddr; //only erase starting at  address
                            for(erase_count = 0;erase_count<file_size_in_blocks;erase_count++)
                            {

                                FLASH_erase_64k_block(erase_address);
                                delay1(50000);
                                erase_address+=0x10000;
                            }
                            break;

                        case 4:
                            cmd             = IAP_PROGRAM_BY_SPIIDX_CMD;
                            spiaddr_or_idx  = progr_param_1;

                            status = SYS_iap_service(cmd, spiaddr_or_idx);

                            // see CoreSysServices_PF/core_sysservices_pf.h  for meaning of STATUS word
                            *(registers_1_addr + CRCDS_PROG_RETURN) = status;
                            break;

                        case 5:
                            cmd             = IAP_PROGRAM_BY_SPIADDR_CMD;
                            spiaddr_or_idx  = progr_param_1;

                            status = SYS_iap_service(cmd, spiaddr_or_idx);

                            // see CoreSysServices_PF/core_sysservices_pf.h  for meaning of STATUS word
                            *(registers_1_addr + CRCDS_PROG_RETURN) = status;
                            break;

                        case 6:
                            cmd = IAP_AUTOUPDATE_CMD;

                            status = SYS_iap_service(cmd, spiaddr_or_idx);

                            // see CoreSysServices_PF/core_sysservices_pf.h  for meaning of STATUS word
                            *(registers_1_addr + CRCDS_PROG_RETURN) = status;
                            break;

                        case 7:
                            startAddr   = progr_param_1;    // start reading address
                            nBytes      = progr_param_2;    // no. of bytes to read from SPI
                            if (nWords>=255) nWords = 254;  // maximum allowed number of words to be read from FLASH

                            list_flash_dir_fiber(startAddr, nBytes, flash_read_buf);

                            // fill up payload
                            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
                            *(registers_1_addr + CRDCS_WRITE_TX) = nBytes/2;  // two consecutive 8-bit words from FLASH are packed in one 16-bit word
                            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
                            for (uint16_t i = 0; i < nBytes/2; i++) {
                                *(registers_1_addr + CRDCS_WRITE_TX) = flash_read_buf[i];
                            }
                            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

                            break;

                        case 8:
                            startAddr = progr_param_1;  // start writing address
                            nWords    = progr_param_2;  // no. of payload programming words: each encodes two 8-bit for the FLASH
                            for (int i=0;i<nWords;i++) {
                                flash_write_buf[i*2]    = (uint8_t) (dtcbuffer[i+5] & 0XFF);
                                flash_write_buf[i*2+1]  = (uint8_t) (dtcbuffer[i+5] >>8);
                            }

                            *(registers_1_addr + CRCDS_PROG_RETURN) = 0;

                            load_spi_flash_at_address_fiber(startAddr, flash_write_buf, failMask); //this loads a single image at address index

                            // "failMask" is an 8-bit mask returning 1 if the Nth subblock written to FLASH fails verification
                            // A sub-block is a write of 128B to FLASH
                            *(registers_1_addr + CRCDS_PROG_RETURN) = failMask;
                            break;

                        case 9:
                            nImages = (uint16_t) progr_param_1;  // no of images pointer to write to SPI directory
                            // each image pointer (max 16) is a 32-bit word
                            for (int i=0;i<16;i++) {
                                dir_buffer[i] = (dtcbuffer[2*i+6]<<16) + dtcbuffer[2*i+5];
                            }

                            write_flash_directory_fiber(nImages, dir_buffer, failCount);

                            // failCount increases by 1 for every DIR_BUFFER word that is not validated
                            *(registers_1_addr + CRCDS_PROG_RETURN) = failCount;
                            break;

                        default:
                            break;
                    }

                    // update CMD_STATUS
                    *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;

                    break;


        // read assorted diagnostic registers
        case DIAGDATA:
            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x4000 + cmd_fail;

            // header
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDHEADER;
            // payload word
            *(registers_1_addr + CRDCS_WRITE_TX) = 9;
            // CMD ID
            *(registers_1_addr + CRDCS_WRITE_TX) = 0xC000 + proc_commandID;
            // payload of diagnostic data
            *(registers_1_addr + CRDCS_WRITE_TX) = data_zero;
            *(registers_1_addr + CRDCS_WRITE_TX) = cmd_rx_size; // buffer length
            *(registers_1_addr + CRDCS_WRITE_TX) = cmd_fail;  // diagnostic word
            *(registers_1_addr + CRDCS_WRITE_TX) = cmd_rx_cnt; // number of word written to TX_BUFFER
            *(registers_1_addr + CRDCS_WRITE_TX) = data_to_write; // ie dtcbuffer[0]
            *(registers_1_addr + CRDCS_WRITE_TX) = dtcbuffer[1];
            *(registers_1_addr + CRDCS_WRITE_TX) = dtcbuffer[2];
            *(registers_1_addr + CRDCS_WRITE_TX) = dtcPtr; // number of words in payload
            *(registers_1_addr + CRDCS_WRITE_TX) = get_cmd_rx; // last read word
            //trailer
            *(registers_1_addr + CRDCS_WRITE_TX) = CMDTRAILER;

            // update CMD_STATUS
            *(registers_1_addr + CRDCS_CMD_STATUS) = 0x8000 + cmd_fail;
            break;

        default:
            break;
        }

        delayUs(1);

#endif

        size_t rx_size = UART_get_rx(&g_uart, rx_buff, sizeof(rx_buff));
        for (uint16_t j = 0; j < rx_size; j++) {
            buffer[writePtr] = rx_buff[j];
            writePtr++;
        }

        if (writePtr <= 3)
            continue;
        if (writePtr > 0) {
            if ((buffer[0] != 0xAA) || buffer[1] != 0xAA) {
                writePtr = 0;
                UART_polled_tx_string(&g_uart,
                        "Problem synchronizing command\n");
                continue;
            }

            uint8_t numBytes = buffer[2];
            if (writePtr >= (uint32_t) numBytes) {
                uint8_t commandID = (uint8_t) buffer[3];
                bufcount = 0;

                //	    if (commandID < MAXCOM_MON)
                //	      UART_polled_tx_string( &g_uart, "monitoring\n" );

                if (commandID == RESETROC) {
                    digi_write(DG_ADDR_RESET, 0, 0);
                    digi_write(DG_ADDR_RESET, 1, 0);
                    // Monica added 09/26/2019
                    *(registers_0_addr + REG_ROC_RESET) = 0;
                    outBuffer[bufcount++] = RESETROC;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

#ifdef DIGITEST
                } else if (commandID == SETPREAMPGAIN) {
                    uint16_t channel = readU16fromBytes(&buffer[4]);
                    uint16_t value = readU16fromBytes(&buffer[6]);

                    setPreampGain(channel, value);

                    outBuffer[bufcount++] = SETPREAMPGAIN;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, channel, 2);
                    bufWrite(outBuffer, &bufcount, value, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                    // Set preamp threshold
                } else if (commandID == SETPREAMPTHRESHOLD) {
                    uint16_t channel = readU16fromBytes(&buffer[4]);

                    uint16_t value = readU16fromBytes(&buffer[6]);

                    setPreampThreshold(channel, value);

                    outBuffer[bufcount++] = SETPREAMPTHRESHOLD;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, channel, 2);
                    bufWrite(outBuffer, &bufcount, value, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == SETCALDAC) {

                    uint8_t chan_mask = (uint8_t) buffer[4];
                    uint16_t value = readU16fromBytes(&buffer[5]);

                    for (uint8_t i = 0; i < 8; i++) {
                        if (chan_mask & (0x1 << i)) {
                            if (i < 4)
                                LTC2634_write(&caldac0, i, default_caldac[i]);
                            else
                                LTC2634_write(&caldac1, i - 4,
                                        default_caldac[i]);
                        }
                    }

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

                } else if (commandID == SETPULSERON) {

                    for (uint8_t i = 0; i < 16; i++) {
                        MCP_pinWrite(&preampMCP[MCPCALIB], i + 1, 0);
                    }
                    uint8_t chan_mask = (uint8_t) buffer[4];
                    pulserOdd = (uint8_t) buffer[5];
                    for (uint8_t i = 0; i < 8; i++) {
                        if ((0x1 << i) & chan_mask) {
                            MCP_pinWrite(&preampMCP[MCPCALIB],
                                    calpulse_chanmap[i], 1);
                        }
                    }
                    dutyCycle = readU16fromBytes(&buffer[6]);
                    pulserDelay = readU32fromBytes(&buffer[8]);

                    //granularity is clock period=25ns -- period is (gr+1)*1000=50us
                    //PWM_PERIOD = PWM_GRANULARITY * (period + 1) = 25 *1000 = 25us
                    PWM_init(&g_pwm, COREPWM_BASE_ADDR, 1, pulserDelay);
                    PWM_set_duty_cycle(&g_pwm, PWM_1, dutyCycle);//duty cycle is 4 x 25 = 100ns

                    PWM_enable(&g_pwm, PWM_1);

                    outBuffer[bufcount++] = SETPULSERON;
                    bufWrite(outBuffer, &bufcount, 8, 2);
                    outBuffer[bufcount++] = chan_mask;
                    outBuffer[bufcount++] = pulserOdd;
                    bufWrite(outBuffer, &bufcount, dutyCycle, 2);
                    bufWrite(outBuffer, &bufcount, pulserDelay, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == SETPULSEROFF) {

                    PWM_disable(&g_pwm, PWM_1);
                    outBuffer[bufcount++] = SETPULSEROFF;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                    //				}else if (commandID == WHOAREYOU){
                    //
                    //				 	outBuffer[bufcount++] = WHOAREYOU;
                    //				 	bufWrite(outBuffer, &bufcount, 0, 2);
                    //
                    //				 	GPIO_set_output( &g_gpio, GPIO_0, 1);
                    //				 	hwdelay(5000);
                    //				 	GPIO_set_output( &g_gpio, GPIO_0, 0);
                    //				 	outBufSend(g_uart, outBuffer, bufcount);

                    //				}else if (commandID == ROCREADREG){
                    //					outBuffer[bufcount++] = ROCREADREG;
                    //					bufWrite(outBuffer,&bufcount,5,2);
                    //					volatile uint32_t retv = 0xFFFFFFFF;
                    //					uint32_t raddr = (uint32_t) buffer[4];
                    //					retv = *(registers_0_addr + raddr);
                    //					outBuffer[bufcount++] = raddr;
                    //					bufWrite(outBuffer, &bufcount, retv, 4);
                    //					outBufSend(g_uart, outBuffer, bufcount);

#endif
                    //                }else if (commandID == RESETROC){

                    //					digi_write(DG_ADDR_RESET,0,0);
                    //					digi_write(DG_ADDR_RESET,1,0);
                    //					// Monica added 09/26/2019
                    //					*(registers_0_addr + REG_ROC_RESET) = 0;
                    //					outBuffer[bufcount++] = RESETROC;
                    //					bufWrite(outBuffer, &bufcount, 0, 2);
                    //				 	outBufSend(g_uart, outBuffer, bufcount);
                } else if (commandID == INITDIGIS) {
                    uint16_t calerr = 0;
                    uint16_t hverr = 0;


                    // make sure DIGIs are powered up
                    digi_write(DG_ADDR_EWS, 0xDEAD, 1);
                    uint32_t read_value = digi_read(DG_ADDR_EWS, 1);
                    if (read_value != 0xDEAD) {
                        calerr |= 0x1;
                    }
                    digi_write(DG_ADDR_EWS, 0xDEAD, 2);
                    read_value = digi_read(DG_ADDR_EWS, 2);
                    if (read_value != 0xDEAD) {
                        hverr |= 0x1;
                    }



                    init_DIGIs();

                    bufcount = 0;
                    outBuffer[bufcount++] = INITDIGIS;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, calerr, 2);
                    bufWrite(outBuffer, &bufcount, hverr, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == RESETDEVICE) {

                    // Monica added 09/26/2019
                    *(registers_0_addr + REG_ROC_FIFO_RESET) = 0;
                    delay_ms(10);
                    *(registers_0_addr + REG_ROC_FIFO_RESET) = 1;
                    outBuffer[bufcount++] = RESETDEVICE;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DUMPSETTINGS) {
                    uint16_t channel = (uint16_t) buffer[4];
                    outBuffer[bufcount++] = DUMPSETTINGS;
                    if (channel >= 0 && channel < 96) {

                        bufWrite(outBuffer, &bufcount, 10, 2);
                        bufWrite(outBuffer, &bufcount, channel, 2);

                        bufWrite(outBuffer, &bufcount,
                                default_gains_hv[channel], 2);
                        bufWrite(outBuffer, &bufcount,
                                default_threshs_hv[channel], 2);
                        bufWrite(outBuffer, &bufcount,
                                default_gains_cal[channel], 2);
                        bufWrite(outBuffer, &bufcount,
                                default_threshs_cal[channel], 2);
                    }

                    else {
                        bufWrite(outBuffer, &bufcount, 768, 2);
                        for (uint8_t ic = 0; ic < 96; ic++) {
                            bufWrite(outBuffer, &bufcount, default_gains_hv[ic],
                                    2);
                            bufWrite(outBuffer, &bufcount,
                                    default_threshs_hv[ic], 2);
                            bufWrite(outBuffer, &bufcount,
                                    default_gains_cal[ic], 2);
                            bufWrite(outBuffer, &bufcount,
                                    default_threshs_cal[ic], 2);
                        }
                    }
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == READMONADCS) {

                    //read currents

                    uint32_t rx0;

                    outBuffer[bufcount++] = READMONADCS;
                    bufWrite(outBuffer, &bufcount, 72, 2);
                    for (uint8_t i = 0; i < 2; i++) {
                        for (uint8_t j = 0; j < 12; j++) {
                            SPI_set_slave_select(&g_spi[i],
                                    ((j >= 8) ?
                                            SPI_SLAVE_2 :
                                            (j < 4 ? SPI_SLAVE_0 : SPI_SLAVE_1)));
                            uint16_t addr = (j % 4 << 11);
                            SPI_transfer_frame(&g_spi[i], addr);
                            SPI_transfer_frame(&g_spi[i], addr);
                            rx0 = SPI_transfer_frame(&g_spi[i], addr);
                            SPI_clear_slave_select(&g_spi[i],
                                    ((j >= 8) ?
                                            SPI_SLAVE_2 :
                                            (j < 4 ? SPI_SLAVE_0 : SPI_SLAVE_1)));

                            bufWrite(outBuffer, &bufcount, rx0, 2);
                        }
                    }

                    uint16_t tvs_val[4] = { 0 };

                    for (uint8_t i = 0; i < 4; i++) {
                        *(registers_0_addr + REG_ROC_TVS_ADDR) = i;
                        delayUs(1);
                        tvs_val[i] = *(registers_0_addr + REG_ROC_TVS_VAL);
                        bufWrite(outBuffer, &bufcount, tvs_val[i], 2);
                        delayUs(1);
                    }

                    for (uint8_t ihvcal = 1; ihvcal < 3; ihvcal++) {
                        for (uint8_t i = 0; i < 4; i++) {
                            digi_write(DG_ADDR_TVS_ADDR, i, ihvcal);
                            delayUs(1);
                            tvs_val[i] = digi_read(DG_ADDR_TVS_VAL, ihvcal);
                            bufWrite(outBuffer, &bufcount, tvs_val[i], 2);
                            delayUs(1);
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

                } else if (commandID == READHISTO) {
                    uint8_t channel = (uint8_t) buffer[4];
                    uint8_t hv_or_cal = (uint8_t) buffer[5];
                    uint16_t output[256];
                    read_histogram(channel, hv_or_cal, output);

                    outBuffer[bufcount++] = READHISTO;
                    bufWrite(outBuffer, &bufcount, 512, 2);
                    for (int i = 0; i < 256; i++) {
                        bufWrite(outBuffer, &bufcount, output[i], 2);
                    }
                    outBufSend(g_uart, outBuffer, bufcount);


                } else if (commandID == NVMWRITE) {
                    uint8_t rw = (uint8_t) buffer[4];

                    uint8_t user_key[24] = {0x00};
                    uint8_t cmd = SNVM_NON_AUTHEN_TEXT_REQUEST_CMD;
                    uint16_t addr = readU16fromBytes(&buffer[5]); //no need to pass more than 2 bytes. the writable part of the nvm starts at 0
                    uint16_t data = readU16fromBytes(&buffer[7]);
                    uint8_t nvmadmin[4] = {0x00};

                    uint8_t text[280] = {0x0};
                    text[0] = data & 0xFF;
                    text[1] = (data >> 8) & 0xFF;
                    uint8_t status = 0;

                    outBuffer[bufcount++] = NVMWRITE;
                    bufWrite(outBuffer, &bufcount, 5, 2);

                    if (rw >0 ) {

                        status = SYS_secure_nvm_write(cmd, addr, &text[0], user_key, 0);
                    }
                    else{
                        status = SYS_secure_nvm_read(addr, 0, &nvmadmin[0], text, 252u, 0);
                    }
                    uint16_t reply = (text[1] << 8) | text[0];
                    bufWrite(outBuffer, &bufcount, rw, 1);
                    bufWrite(outBuffer, &bufcount, status, 2);
                    bufWrite(outBuffer, &bufcount, reply, 2);

                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DIGIRW) {
                    uint8_t rw = (uint8_t) buffer[4];
                    uint8_t thishvcal = (uint8_t) buffer[5];
                    uint8_t address = (uint8_t) buffer[6];
                    uint16_t adcaddress = address;
                    if (rw & 0x2) {
                        adcaddress |= 0x100;
                        rw &= 0x1;
                    }
                    uint32_t data = readU32fromBytes(&buffer[7]);

                    outBuffer[bufcount++] = DIGIRW;
                    bufWrite(outBuffer, &bufcount, 7, 2);

                    if (thishvcal < 3) {
                        if (rw == 0) {					//read
                            data = digi_read(address, thishvcal);
                        } else {
                            digi_write(address, data, thishvcal);
                        }
                    } else {
                        if (thishvcal == 3) {
                            if (rw == 0) {					//read
                                data = *(registers_0_addr + address);
                            } else {
                                *(registers_0_addr + address) = data;
                            }
                        } else {
                            uint8_t adc_num = thishvcal - 4;
                            if (rw == 0) {
                                data = adc_read(adcaddress, adc_num);
                            } else {
                                uint16_t adc_mask = (0x1 << adc_num);
                                adc_write(adcaddress, data, adc_mask);
                            }
                        }
                    }
                    bufWrite(outBuffer, &bufcount, rw, 1);
                    bufWrite(outBuffer, &bufcount, thishvcal, 1);
                    bufWrite(outBuffer, &bufcount, address, 1);
                    bufWrite(outBuffer, &bufcount, data, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

#ifndef	DTCDDRTEST  // Monica added for allowing all other DDR test functions

                } else if (commandID == GETDEVICEID) {

                    uint8_t data_buffer[16];
                    uint8_t dinfo_buffer[36];
                    uint8_t status;
                    status = SYS_get_serial_number(data_buffer, 0);
                    status = SYS_get_design_info(dinfo_buffer, 0);
                    outBuffer[bufcount++] = GETDEVICEID;
                    bufWrite(outBuffer, &bufcount, 52, 2);
                    for (uint8_t i = 0; i < 16; i++)
                        outBuffer[bufcount++] = data_buffer[i];
                    for (uint8_t i = 0; i < 36; i++)
                        outBuffer[bufcount++] = dinfo_buffer[i];
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == GETILP) {
                    outBuffer[bufcount++] = GETILP;
                    bufWrite(outBuffer, &bufcount, 7, 2);

                    ilp22qs_init();
                    uint8_t ret = ilps22qs_id_get();
                    bufWrite(outBuffer, &bufcount, ret, 1);
                    ilps22qs_all_sources_t all_sources;
                    ilps22qs_all_sources_get(&all_sources);
                    ilps22qs_md_t md;
                    md.odr = ILPS22QS_4Hz;
                    md.avg = ILPS22QS_16_AVG;
                    md.lpf = ILPS22QS_LPF_ODR_DIV_4;
                    md.fs = ILPS22QS_4060hPa;
                    ilps22qs_data_t data;
                    ilps22qs_data_get(&md, &data);
                    bufWrite(outBuffer, &bufcount, data.heat.raw, 2);
                    bufWrite(outBuffer, &bufcount, data.pressure.raw, 4);

                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == GETGITREV){
                    outBuffer[bufcount++] = GETGITREV;
                    bufWrite(outBuffer, &bufcount, sizeof(LAST_GIT_REV)-1 , 2);
                    for (uint8_t i = 0; i < sizeof(LAST_GIT_REV)-1; i++){
                        outBuffer[bufcount++] = LAST_GIT_REV[i];
                    }
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == SETFUSEON) {
                    uint8_t preamp_number = (uint8_t) buffer[4];
                    uint8_t time = (uint8_t) buffer[5];

                    for (uint8_t i = 0; i < 48; i++) {
                        MCP_pinWrite(&preampMCP[MCPFC0 + i / 16], i % 16 + 1,
                                0);
                    }

                    MCP_pinWrite(&preampMCP[MCPFC0 + preamp_number / 16],
                            preamp_number % 16 + 1, 1);

                    if (time > 0) {
                        delay_ms(time * 1000);
                        MCP_pinWrite(&preampMCP[MCPFC0 + preamp_number / 16],
                                preamp_number % 16 + 1, 0);
                    }

                    outBuffer[bufcount++] = SETFUSEON;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount, preamp_number, 1);
                    bufWrite(outBuffer, &bufcount, time, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == SETFUSEOFF) {

                    for (uint8_t i = 0; i < 48; i++) {
                        MCP_pinWrite(&preampMCP[MCPFC0 + i / 16], i % 16 + 1,
                                0);
                    }

                    outBuffer[bufcount++] = SETFUSEOFF;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == READKEY) {
                    outBuffer[bufcount++] = READKEY;
                    bufWrite(outBuffer, &bufcount, 8, 2);

                    uint16_t key_temp = ADC124S051_read(&g_spi[2], 0, 1);
                    uint16_t v2p5 = ADC124S051_read(&g_spi[2], 0, 3);
                    uint16_t v5p1 = ADC124S051_read(&g_spi[2], 0, 0);
                    uint16_t dcdctemp = ADC124S051_read(&g_spi[2], 0, 2);

                    bufWrite(outBuffer, &bufcount, key_temp, 2);
                    bufWrite(outBuffer, &bufcount, v2p5, 2);
                    bufWrite(outBuffer, &bufcount, v5p1, 2);
                    bufWrite(outBuffer, &bufcount, dcdctemp, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRSTATUS) {
                    outBuffer[bufcount++] = DDRSTATUS;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDR_CTRLREADY), 1);
                    outBufSend(g_uart, outBuffer, bufcount);

#endif

                    //***********************************begin of DRAC Test commands****************************************************************************************
#ifdef  DRACTEST
                }else if (commandID == XCVRALIGN){
                    uint8_t align   = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DTC_ENABLE_RESET) = align; // enable PRBS data to Core_PCS

                    outBuffer[bufcount++] = XCVRALIGN;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, align, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSET) {
                    uint8_t prbs_en = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_PRBS_EN) = prbs_en; // enable PRBS data to Core_PCS

                    outBuffer[bufcount++] = PRBSET;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, prbs_en, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSSTATUS) {
                    outBuffer[bufcount++] = PRBSSTATUS;
                    bufWrite(outBuffer, &bufcount, 6, 2);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_PRBS_LOCK), 1);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_PRBS_ON), 1);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_PRBS_ERRORCNT), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSSTART) {
                    *(registers_0_addr + REG_ROC_PRBS_EN) = 1; // enable PRBS data to Core_PCS
                    outBuffer[bufcount++] = PRBSSTART;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSSTOP) {
                    *(registers_0_addr + REG_ROC_PRBS_EN) = 0; // disable PRBS data to Core_PCS
                    outBuffer[bufcount++] = PRBSSTOP;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSERRORIN) {
                    *(registers_0_addr + REG_ROC_PRBS_ERRORIN) = 1; // enable error as 0xEFFE in PRBS generator
                    outBuffer[bufcount++] = PRBSERRORIN;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == PRBSERRORCLR) {
                    *(registers_0_addr + REG_ROC_PRBS_ERRORCLR) = 1; // enable PRBS generator and checker logic
                    outBuffer[bufcount++] = PRBSERRORCLR;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRTESTEN) {
                    uint8_t ddr_test_en = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DDRTEST_EN) = ddr_test_en; // enable TEST DATA to DDR

                    outBuffer[bufcount++] = DDRTESTEN;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, ddr_test_en, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRTESTSETUP) {
                    uint32_t ddr_blockno = readU32fromBytes(&buffer[4]); //maximum is 256

                    // PATTERN_EN must be set BEFORE page_no to prevent write to DDR3 from standard digififo
                    *(registers_0_addr + REG_ROC_DDRTEST_BLKNO) = ddr_blockno;

                    outBuffer[bufcount++] = DDRTESTSETUP;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, ddr_blockno, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRTESTWRITE) {

                    *(registers_0_addr + REG_ROC_DDRTEST_EN) = 1; // enable TEST DATA to DDR
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDRTEST_WREN) = 1;
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDRTEST_EN) = 0; // disable TEST DATA to DDR

                    //                     uint32_t ddr_blockno = *(registers_0_addr + REG_ROC_DDRTEST_BLKNO);

                    outBuffer[bufcount++] = DDRTESTWRITE;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    //                    bufWrite(outBuffer, &bufcount, ddr_blockno, 4);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDRTEST_BLKNO), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRTESTREAD) {

                    *(registers_0_addr + REG_ROC_DDRTEST_EN) = 1; // enable TEST DATA from DDR
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDRTEST_RDEN) = 1;
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDRTEST_EN) = 0; // disable TEST DATA from DDR

                    outBuffer[bufcount++] = DDRTESTREAD;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDRTEST_BLKNO), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == DDRTESTSTATUS) {
                    outBuffer[bufcount++] = DDRTESTSTATUS;
                    bufWrite(outBuffer, &bufcount, 10, 2);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDR_CTRLREADY), 1);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDRTEST_STATUS), 1);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDRTEST_ERRCNT), 4);
                    bufWrite(outBuffer, &bufcount,
                            *(registers_0_addr + REG_ROC_DDRTEST_ERRLOC), 4);
                    outBufSend(g_uart, outBuffer, bufcount);


#endif
                    //***********************************begin of DDR commands****************************************************************************************
#ifdef	DTCDDRTEST

                }else if (commandID == DDRSTATUS){
                    outBuffer[bufcount++] = DDRSTATUS;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_CTRLREADY), 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DDRRAMREAD){
                    // MT 02/28/2021   simplified to pass a fixed size of RAM content to file, as specified by "ddr_addr2read"
                    uint32_t ddr_addr2read = readU32fromBytes(&buffer[4]);

                    outBuffer[bufcount++] = DDRRAMREAD;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, ddr_addr2read, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                    // read RAM
                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
                    readout_obloc_place_holder = readout_obloc;
                    readout_obloc += 2;

                    for (uint32_t i= 0; i < ddr_addr2read; i++){
                        *(registers_0_addr + REG_ROC_DDR_RAM_ADDR) = i;
                        delayUs(100);

                        *(registers_0_addr + REG_ROC_DDR_RAM_REN) = 1;
                        delayUs(100);

                        volatile uint32_t ramdata = *(registers_0_addr + REG_ROC_DDR_RAM_DATA);
                        bufWrite(dataBuffer, &readout_obloc, ramdata, 4);
                    }
                    bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);
                    UART_send(&g_uart, dataBuffer, readout_obloc);

                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                    UART_send(&g_uart, dataBuffer ,2);

                }else if (commandID == DDRSPEEDTEST){
                    uint8_t ddr_option = (uint8_t) buffer[4]; 	// which test: 1 -> WR;  2 -> RD&test;  3 -> WR&RD(&test)
                    uint8_t ddr_burst  = (uint8_t) buffer[5];  	// burst size: min=0, max=255)
                    uint8_t ddr_pattern= (uint8_t) buffer[6];	// pattern to write/test
                    uint32_t ddr_nhit   = readU32fromBytes(&buffer[7]);  // number of DDR bursts (1024 to fill FIFOs): max depends on burst size

                    *(registers_0_addr + REG_ROC_DDR_BURST) 	= ddr_burst;
                    *(registers_0_addr + REG_ROC_DDR_PATTERN) 	= ddr_pattern;
                    *(registers_0_addr + REG_ROC_DDR_NHITS) 	= ddr_nhit;

                    // PATTERN_EN needs to be 0 for this test and latched by SIM_EN (via DDR_CS)
                    *(registers_0_addr + REG_ROC_DDR_PATTERN_EN) = 0;
                    delayUs(100);

                    if (ddr_option == 1) { 			// write speed test
                        *(registers_0_addr + REG_ROC_DDR_WEN) = 1;
                    } else if (ddr_option == 2) { 	// read speed test
                        *(registers_0_addr + REG_ROC_DDR_REN) = 1;
                    } else if (ddr_option == 3) { 	// write & read speed test
                        *(registers_0_addr + REG_ROC_DDR_RWEN) = 1;
                    }

                    delay_ms(100);

                    // return FIFO status AFTER write
                    uint16_t ddr_fifo_diagn = *(registers_0_addr + REG_ROC_DDR_FIFODIA);

                    outBuffer[bufcount++] = DDRSPEEDTEST;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount, ddr_fifo_diagn, 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DDRSPEEDREAD){
                    uint8_t ddr_option = (uint8_t) buffer[4]; 	// which test: 1 -> WR;  2 -> RD&test;  3 -> WR&RD(&test)

                    //FIFO status BEFORE read
                    outBuffer[bufcount++] = DDRSPEEDREAD;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount,  *(registers_0_addr + REG_ROC_DDR_FIFODIA), 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                    volatile uint16_t ddr_write_empty, ddr_read_empty;
                    volatile uint32_t wrt_data, rdt_data;

                    // start reading appropriate timer FIFO until empty. ie while VALID is high
                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
                    readout_obloc_place_holder = readout_obloc;
                    readout_obloc += 2;

                    uint32_t wr_cnt = 0;
                    uint32_t rd_cnt = 0;
                    if (ddr_option == 1) {  			// write speed FIFO data

                        // loop to write to file only until FIFO is empty
                        for (uint32_t j=0;j<1000;j++) { // limiting to 1000 loops ensures that
                            // dataBuffer does not reach the size limit of 4096 bytes

                            volatile uint16_t ddr_fifo_diag = *(registers_0_addr + REG_ROC_DDR_FIFODIA);
                            ddr_write_empty = (ddr_fifo_diag & 0x0001);

                            if (ddr_write_empty == 0) {
                                wr_cnt++;
                                *(registers_0_addr + REG_ROC_DDR_WRT_REN) = 1;

                                delayUs(100);
                                wrt_data = *(registers_0_addr + REG_ROC_DDR_WRTIME);
                                bufWrite(dataBuffer, &readout_obloc, wrt_data, 4);
                            }
                        }

                    } else if (ddr_option == 2) {  // read speed FIFO data

                        for (uint32_t j=0;j<1000;j++) { // limiting to 1000 loops ensures that
                            // dataBuffer does not reach the size limit of 4096 bytes

                            volatile uint16_t ddr_fifo_diag = *(registers_0_addr + REG_ROC_DDR_FIFODIA);
                            ddr_read_empty = (ddr_fifo_diag & 0x0010);

                            if (ddr_read_empty == 0) {
                                rd_cnt++;
                                *(registers_0_addr + REG_ROC_DDR_RDT_REN) = 1;

                                delayUs(100);
                                rdt_data = *(registers_0_addr + REG_ROC_DDR_RDTIME);
                                bufWrite(dataBuffer, &readout_obloc, rdt_data, 4);
                            }
                        }
                    }
                    bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);
                    UART_send(&g_uart, dataBuffer, readout_obloc);

                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                    UART_send(&g_uart, dataBuffer ,2);

                    // FIFO status AFTER read
                    bufcount = 0;
                    outBuffer[bufcount++] = DDRSPEEDREAD;
                    bufWrite(outBuffer, &bufcount, 10, 2);
                    bufWrite(outBuffer, &bufcount, wr_cnt, 4);
                    bufWrite(outBuffer, &bufcount, rd_cnt, 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_FIFODIA), 2);
                    UART_send(&g_uart, outBuffer ,bufcount );


                }else if (commandID == DDRSPEEDERR){

                    // report and number of errors found
                    uint32_t ddr_error = *(registers_0_addr + REG_ROC_DDR_ERRCNT);

                    outBuffer[bufcount++] = DDRSPEEDERR;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, ddr_error, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                    if (ddr_error==0) {

                        readout_obloc = 0;
                        bufWrite(dataBuffer, &readout_obloc, EMPTY, 2);
                        UART_send(&g_uart, dataBuffer, readout_obloc);

                    } else {

                        readout_obloc = 0;
                        bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
                        readout_obloc_place_holder = readout_obloc;
                        readout_obloc += 2;

                        volatile uint16_t ddr_fifo_diag, ddr_err_empty;
                        volatile uint32_t err_loc_data, true_data_l, true_data_h, expc_data_l, expc_data_h;

                        for (uint16_t j=0;j<128;j++) {  // size of DATA_FIFO

                            ddr_fifo_diag = *(registers_0_addr + REG_ROC_DDR_FIFODIA);
                            ddr_err_empty = (ddr_fifo_diag & 0x0100);

                            if (ddr_err_empty == 0) {

                                *(registers_0_addr + REG_ROC_DDR_ERR_REN) = 1;

                                delayUs(100);
                                err_loc_data = *(registers_0_addr + REG_ROC_DDR_ERRLOC);
                                true_data_l  = *(registers_0_addr + REG_ROC_DDR_TRUEL);
                                true_data_h  = *(registers_0_addr + REG_ROC_DDR_TRUEH);
                                expc_data_l  = *(registers_0_addr + REG_ROC_DDR_EXPCL);
                                expc_data_h  = *(registers_0_addr + REG_ROC_DDR_EXPCH);

                                delayUs(100);
                                bufWrite(dataBuffer, &readout_obloc, err_loc_data,4);
                                bufWrite(dataBuffer, &readout_obloc, true_data_l, 4);
                                bufWrite(dataBuffer, &readout_obloc, true_data_h, 4);
                                bufWrite(dataBuffer, &readout_obloc, expc_data_l, 4);
                                bufWrite(dataBuffer, &readout_obloc, expc_data_h, 4);
                                bufWrite(dataBuffer, &readout_obloc, 0xABBA, 2);
                            }
                        }

                        bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);
                        UART_send(&g_uart, dataBuffer, readout_obloc);

                        readout_obloc = 0;
                        bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                        UART_send(&g_uart, dataBuffer, 2);
                    }


                    //				}else if (commandID == DDRSTATUS){
                    //					outBuffer[bufcount++] = DDRSTATUS;
                    //					bufWrite(outBuffer, &bufcount, 37, 2);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_CTRLREADY), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_CONV_DATA), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_FULL), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_PAGEWR), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_PAGERD), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_EMPTY), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_FULL), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_TEMPFIFO_EMPTY), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_TEMPFIFO_FULL), 1);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_DATA0), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_DATA1), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DIAG0), 4);
                    //					bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DIAG1), 4);
                    //					outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DDRPATTERNREAD){
                    // MT --same as DDRRAMREAD but only reads and returns location of error register
                    //    It has only one parameter (ddroffset = memory offset in multiple of 1 kB).
                    //    All other testDDR parameters fixed to write to DDR 1kB of increasing one data bits
                    uint32_t ddroffset = readU32fromBytes(&buffer[4]);
                    // MT 02/10/2020  add pattern parameter
                    uint32_t ddrpattern = readU32fromBytes(&buffer[8]);

                    *(registers_0_addr + REG_ROC_DDR_OFFSET) = ddroffset;
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDR_NHITS) = 1;
                    *(registers_0_addr + REG_ROC_DDR_PATTERN_EN) = 0;
                    delayUs(100);

                    *(registers_0_addr + REG_ROC_DDR_PATTERN) = ddrpattern;
                    delayUs(100);

                    // write pattern to DDR memory
                    *(registers_0_addr + REG_ROC_DDR_WEN) = 1;
                    delay_ms(10);

                    //read pattern from DDR memory to RAM
                    *(registers_0_addr + REG_ROC_DDR_REN) = 1;
                    delay_ms(10);

                    // read error location
                    uint32_t errloc = *(registers_0_addr + REG_ROC_DDR_ERRLOC);

                    outBuffer[bufcount++] = DDRPATTERNREAD;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, errloc, 4);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DDRRAMFILL){
                    // MT same as DDRSPEEDTEST for option 2 (read-only)
                    //	  plus extra register to specify DDR read address to start dumping to RAM
                    uint8_t ddrpattern = (uint8_t) buffer[4];
                    uint8_t ddrburst  = (uint8_t) buffer[5];  	// burst size: min=0, max=255)
                    uint32_t ddrnhit = readU32fromBytes(&buffer[6]);
                    uint32_t ddroffset = readU32fromBytes(&buffer[10]);

                    *(registers_0_addr + REG_ROC_DDR_PATTERN) 	= ddrpattern;
                    *(registers_0_addr + REG_ROC_DDR_BURST) 	= ddrburst;
                    *(registers_0_addr + REG_ROC_DDR_NHITS) 	= ddrnhit;
                    *(registers_0_addr + REG_ROC_DDR_LOCRAM)  	= ddroffset;

                    // PATTERN_EN needs to be 0 for this test and latched by SIM_EN (via DDR_CS)
                    *(registers_0_addr + REG_ROC_DDR_PATTERN_EN) = 0;
                    delayUs(100);

                    // read speed test
                    *(registers_0_addr + REG_ROC_DDR_REN) = 1;

                    delay_ms(1000);

                    outBuffer[bufcount++] = DDRRAMFILL;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DDRBURSTREAD){
                    // returns cnt of DDR WR-data and RD-data bursts
                    outBuffer[bufcount++] = DDRBURSTREAD;
                    bufWrite(outBuffer, &bufcount, 8, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_WRBCNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_RDBCNT), 4);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DDRSIMUSET){
                    // sets and returns ON/OFF status of DDR Simulator
                    uint8_t ddrsimuset = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DDR_SET) = ddrsimuset;

                    outBuffer[bufcount++] = DDRSIMUSET;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, ddrsimuset, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == SERIALSET){
                    // set and returns enablew touse SERIAL commands to DDRInterface
                    uint8_t ser_en   = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DDR_CS) = ser_en; // enable DTCInterface signals to Core_PCS

                    outBuffer[bufcount++] = SERIALSET;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, ser_en, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DDRSETUP){
                    uint32_t ddr_pageno = readU32fromBytes(&buffer[4]); //maximum is 256
                    uint8_t ddr_select = (uint8_t) buffer[8];   // unused since v12.6. Set when needed in DDRFILL/DDRREAD
                    uint8_t ddr_set    = (uint8_t) buffer[9];   // unused since v12.4
                    uint8_t ddr_pattern = (uint8_t) buffer[10];
                    uint8_t ddr_pattern_en = (uint8_t) buffer[11];

                    // PATTERN_EN must be set BEFORE page_no to prevent write to DDR3 from standard digififo
                    *(registers_0_addr + REG_ROC_DDR_PATTERN_EN)= ddr_pattern_en;
                    *(registers_0_addr + REG_ROC_DDR_PAGENO)    = ddr_pageno;
                    *(registers_0_addr + REG_ROC_DDR_PATTERN)   = ddr_pattern;

                    // latched configuration via SIM_EN (via DDR_CS)
                    delayUs(100);

                    outBuffer[bufcount++] = DDRSETUP;
                    bufWrite(outBuffer, &bufcount, 6, 2);
                    bufWrite(outBuffer, &bufcount, ddr_pageno, 4);
                    bufWrite(outBuffer, &bufcount, ddr_pattern, 1);
                    bufWrite(outBuffer, &bufcount, ddr_pattern_en, 1);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DDRFILL){
                    volatile uint32_t pages_read = *(registers_0_addr + REG_ROC_DDR_PAGERD);

                    outBuffer[bufcount++] = DDRFILL;
                    bufWrite(outBuffer, &bufcount, 8, 2);
                    bufWrite(outBuffer, &bufcount, pages_read, 4);

                    // enable simulated FIFO_RE
                    *(registers_0_addr + REG_ROC_DDR_SEL) = 1;

                    *(registers_0_addr + REG_ROC_DDR_FIFO_RE) = 1;
                    delay_ms(1);

                    // disable simulated FIFO_RE
                    *(registers_0_addr + REG_ROC_DDR_SEL) = 0;

                    pages_read = *(registers_0_addr + REG_ROC_DDR_PAGERD);
                    bufWrite(outBuffer, &bufcount, pages_read, 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DDRWRITE){

                    *(registers_0_addr + REG_ROC_DDR_FIFOWEN) = 1;

                    outBuffer[bufcount++] = DDRWRITE;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DDRREAD){

                    uint8_t iffull = *(registers_0_addr + REG_ROC_DDR_FULL);
                    uint32_t pages_written = *(registers_0_addr + REG_ROC_DDR_PAGEWR);
                    volatile uint32_t pages_read = *(registers_0_addr + REG_ROC_DDR_PAGERD);

                    outBuffer[bufcount++] = DDRREAD;
                    bufWrite(outBuffer, &bufcount, 6, 2);
                    bufWrite(outBuffer, &bufcount, pages_read, 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_EMPTY), 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_FULL), 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                    // read 1kB of MEMFIFO
                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, STARTTRG, 2);
                    readout_obloc_place_holder = readout_obloc;
                    readout_obloc += 2;

                    // enable simulated MEMFIFO_RE
                    *(registers_0_addr + REG_ROC_DDR_SEL) = 1;

                    for (uint16_t j=0;j<128;j++){ // 8kb in 64bit = 128 reads
                        volatile uint32_t digioutput0, digioutput1;
                        *(registers_0_addr + REG_ROC_DDR_MEMFIFO_RE) = 1;
                        delayUs(100);
                        //*(registers_0_addr + REG_ROC_DDR_MEMFIFO_RE) = 0; // this is a level now, not a pulse, to accommodate simulated clock

                        digioutput0 = *(registers_0_addr + REG_ROC_DDR_MEMFIFO_DATA0);
                        digioutput1 = *(registers_0_addr + REG_ROC_DDR_MEMFIFO_DATA1);
                        delayUs(100);

                        bufWrite(dataBuffer, &readout_obloc, ((digioutput0 & 0xFFFF0000)>>16), 2);
                        bufWrite(dataBuffer, &readout_obloc, (digioutput0 & 0xFFFF), 2);
                        bufWrite(dataBuffer, &readout_obloc, ((digioutput1 & 0xFFFF0000)>>16), 2);
                        bufWrite(dataBuffer, &readout_obloc, (digioutput1 & 0xFFFF), 2);
                    }
                    bufWrite(dataBuffer, &readout_obloc_place_holder, (readout_obloc-4), 2);

                    UART_send(&g_uart, dataBuffer, readout_obloc);

                    readout_obloc = 0;
                    bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                    UART_send(&g_uart, dataBuffer ,2);

                    // disable simulated MEMFIFO_RE
                    *(registers_0_addr + REG_ROC_DDR_SEL) = 0;

                    bufcount = 0;
                    outBuffer[bufcount++] = DDRREAD;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_EMPTY), 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_MEMFIFO_FULL), 1);
                    UART_send(&g_uart, outBuffer ,bufcount );
#endif
                    //*********************************** begin of DTC SIM commands****************************************************************************************
#ifdef DTCDDRTEST
                }else if (commandID == DCSREAD){
                    uint16_t dtc_addr  = readU16fromBytes(&buffer[4]);

                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to single read
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode = 0;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass address
                    DCS_pass_addr_data(dtc_addr, 0, 0);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // read data after DCS reply
                    delay_ms(10);
                    uint32_t dtcsim_read_data = *(registers_0_addr + REG_ROC_DTC_SIM_DATA_READ);

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    outBuffer[bufcount++] = DCSREAD;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, (dtcsim_read_data>>16) & 0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount, (dtcsim_read_data)     & 0xFFFF, 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSWRITE){
                    uint16_t dtc_addr  = readU16fromBytes(&buffer[4]);
                    uint16_t dtc_data  = readU16fromBytes(&buffer[6]);

                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to single write
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode = 1;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass address
                    DCS_pass_addr_data(dtc_addr, 0, 0);

                    // pass data
                    DCS_pass_addr_data(dtc_data, 0, 1);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSWRITE;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount,  dtc_addr, 2);
                    bufWrite(outBuffer, &bufcount,  dtc_data, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DCSMARKER){
                    uint8_t dtc_marker_type = (uint8_t) buffer[4];
                    uint8_t dtc_seq_num = (uint8_t) buffer[5];

                    // set Simulation Enable and Marker Simulator as output
                    uint8_t dtc_output = 1;

                    // after adding CFO simulator, need to disable BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    uint32_t dtc_sim_param = DCS_pass_sim_param(1, dtc_output, dtc_seq_num, dtc_marker_type);

                    // pass non-zero address to avoid clearing all other registers
                    DCS_pass_addr_data(0xFFFF, 0, 0);

                    // send out simulated marker
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_seq_num, dtc_marker_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSMARKER;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_param>>4) &0x0F, 1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_param>>8) &0x0F, 1);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSHEARTBEAT){
                    uint16_t dtc_evttag  = readU16fromBytes(&buffer[4]);
                    uint8_t dtc_onspill  = (uint8_t) buffer[6];
                    uint8_t dtc_rfmarker = (uint8_t) buffer[7];
                    uint8_t dtc_evtmode  = (uint8_t) buffer[8];

                    // set Simulation Enable, DTC Simulator as output and
                    // Packet Type to Heartbeat
                    uint8_t dtc_output = 0;
                    uint8_t dtc_packet_type = 1;

                    // after adding CFO simulator, need to disable BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, 0, dtc_packet_type);

                    // pass non-zero address to avoid clearing all other registers
                    DCS_pass_addr_data(0xFFFF, 0, 0);

                    // pass hearbeat packet info
                    uint32_t dtc_sim_spill = (dtc_onspill<<31) | (dtc_rfmarker<<24) | (dtc_evtmode<<16) | dtc_evttag;
                    *(registers_0_addr + REG_ROC_DTC_SIM_SPILL) = dtc_sim_spill;

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, 0, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSHEARTBEAT;
                    bufWrite(outBuffer, &bufcount, 6, 2);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_spill>>31)&0x01, 1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_spill>>24)&0x7F, 1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_spill>>16)&0xFF, 1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_spill)  &0xFFFF, 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSDATAREQ){
                    uint16_t dtc_evttag  = readU16fromBytes(&buffer[4]);

                    // set Simulation Enable, DTC Simulator as output and
                    // Packet Type to Data Request
                    uint8_t dtc_output = 0;
                    uint8_t dtc_packet_type = 2;

                    // after adding CFO simulator, need to disable it BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, 0, dtc_packet_type);

                    // pass non-zero address to avoid clearing all other registers
                    DCS_pass_addr_data(0xFFFF, 0, 0);

                    // pass event window
                    uint32_t dtc_sim_spill = dtc_evttag & 0x0000FFFF;
                    *(registers_0_addr + REG_ROC_DTC_SIM_SPILL) = dtc_sim_spill;
                    delay_ms(1);


                    // send out simulated packet
                    DCS_sim_packet_send();


                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, 0, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSDATAREQ;
                    bufWrite(outBuffer, &bufcount, 2, 2);
                    bufWrite(outBuffer, &bufcount, dtc_evttag,  2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSREPLY){
                    outBuffer[bufcount++] = DCSREPLY;

                    uint32_t dtcsim_read_data = *(registers_0_addr + REG_ROC_DTC_SIM_DATA_READ);

                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, (dtcsim_read_data>>16) & 0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount, (dtcsim_read_data)     & 0xFFFF, 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSBLKREAD){
                    uint8_t  blk_type = (uint8_t)  buffer[4];
                    uint16_t blk_word  = readU16fromBytes(&buffer[5]);
                    uint16_t blk_addr  = readU16fromBytes(&buffer[7]);

                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to appropriate Block Read
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode;
                    if      (blk_type==0)  dtc_opcode = 2;
                    else if (blk_type==1)  dtc_opcode = 4;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable it BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass starting address
                    DCS_pass_addr_data(blk_addr, 0, 0);

                    // pass block word count
                    DCS_pass_addr_data(0, blk_word, 1);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSBLKREAD;
                    bufWrite(outBuffer, &bufcount, 5, 2);
                    bufWrite(outBuffer, &bufcount, blk_type, 1);
                    bufWrite(outBuffer, &bufcount, blk_word, 2);
                    bufWrite(outBuffer, &bufcount, blk_addr, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DCSBLKWRITE){

                    uint8_t  blk_type = (uint8_t)  buffer[4];
                    uint16_t blk_word  = readU16fromBytes(&buffer[5]);
                    uint16_t blk_addr  = readU16fromBytes(&buffer[7]);
                    //uint8_t  blk_word = (uint8_t)  buffer[5];
                    //uint8_t  blk_addr = (uint8_t)  buffer[6];

                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to appropriate Block Read
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode;
                    if      (blk_type==0)  dtc_opcode = 3;
                    else if (blk_type==1)  dtc_opcode = 5;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable it BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass starting address
                    DCS_pass_addr_data(blk_addr, 0, 0);

                    // pass block word count
                    DCS_pass_addr_data(0, blk_word, 1);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSBLKWRITE;
                    //bufWrite(outBuffer, &bufcount, 3, 2);
                    //bufWrite(outBuffer, &bufcount, blk_type, 1);
                    //bufWrite(outBuffer, &bufcount, blk_word, 1);
                    //bufWrite(outBuffer, &bufcount, blk_addr, 1);
                    bufWrite(outBuffer, &bufcount, 5, 2);
                    bufWrite(outBuffer, &bufcount, blk_type, 1);
                    bufWrite(outBuffer, &bufcount, blk_word, 2);
                    bufWrite(outBuffer, &bufcount, blk_addr, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DCSMODREAD){
                    uint8_t  dtc_module = (uint8_t)  buffer[4];
                    uint16_t dtc_addr   = readU16fromBytes(&buffer[5]);

                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to Module Read
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode = 6;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable it BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass address
                    uint32_t dtc_sim_address = DCS_pass_addr_data(dtc_addr, (uint16_t)dtc_module, 0);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    delay_ms(10); // wait a bit longer to make sure DCS reply packets makes it back
                    uint32_t dtcsim_read_data = *(registers_0_addr + REG_ROC_DTC_SIM_DATA_READ);

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSMODREAD;
                    bufWrite(outBuffer, &bufcount, 5, 2);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_address>>16)&0xFF,   1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_address)    &0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount, (dtcsim_read_data)   & 0xFFFF,2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == DCSMODWRITE){
                    uint8_t  dtc_module = (uint8_t)  buffer[4];
                    uint16_t dtc_addr   = readU16fromBytes(&buffer[5]);
                    uint16_t dtc_data  = readU16fromBytes(&buffer[7]);


                    // set Simulation Enable, DTC Simulator as output,
                    // Packet Type to DCSRequest and OP_CODE to Module write
                    uint8_t dtc_output = 0;
                    uint8_t dtc_opcode = 7;
                    uint8_t dtc_packet_type = 0;

                    // after adding CFO simulator, need to disable it BEFORE any DCS simulation function!
                    *(registers_0_addr + REG_ROC_DDR_CFO_EN)  = 0;

                    // pass simulation parameters
                    DCS_pass_sim_param(1, dtc_output, dtc_opcode, dtc_packet_type);

                    // pass address
                    uint32_t dtc_sim_address = DCS_pass_addr_data(dtc_addr, (uint16_t)dtc_module, 0);

                    // pass data
                    DCS_pass_addr_data(dtc_data, 0, 1);

                    // send out simulated packet
                    DCS_sim_packet_send();

                    // clear simulation parameters
                    //DCS_pass_sim_param(0, 0, 0, 0);
                    DCS_pass_sim_param(0, dtc_output, dtc_opcode, dtc_packet_type);

                    // read back all relevant parameters
                    outBuffer[bufcount++] = DCSMODWRITE;
                    bufWrite(outBuffer, &bufcount, 5, 2);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_address>>16)&0xFF,   1);
                    bufWrite(outBuffer, &bufcount, (dtc_sim_address)    &0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount,  dtc_data,                	 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == DCSRAMWRITE){
                    uint8_t blk_size   = (uint8_t) buffer[4];
                    uint8_t blk_offset = (uint8_t) buffer[5];

                    uint8_t blk_address = blk_offset;
                    uint16_t blk_data;
                    // pass 16-bit word to RAM_DATA and enable RAM_WE for increasing RAM_ADDR starting from offset
                    for (uint8_t i=0; i<blk_size; i++) {

                        blk_data  = readU16fromBytes(&buffer[6+(i*2)]);
                        *(registers_0_addr + REG_ROC_DTC_SIM_BLK_DATA) = blk_data;

                        *(registers_0_addr + REG_ROC_DTC_SIM_BLK_ADDR) = blk_address;
                        if (i<(blk_size-1)) blk_address = blk_address + 1;

                        delay_ms(1);
                        *(registers_0_addr + REG_ROC_DTC_SIM_BLK_EN) = 1;
                        delay_ms(1);
                    }

                    outBuffer[bufcount++] = DCSRAMWRITE;
                    bufWrite(outBuffer, &bufcount, 4, 2);
                    bufWrite(outBuffer, &bufcount, blk_size,    1);
                    bufWrite(outBuffer, &bufcount, blk_address, 1);
                    bufWrite(outBuffer, &bufcount, blk_data,    2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == FORCEDDRRD){
                    uint8_t forcerd   = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DDR_FORCE_RD) = forcerd; // enable PRBS data to Core_PCS

                    outBuffer[bufcount++] = FORCEDDRRD;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == PRBSERRORDUMP){
                    uint16_t error_cnt = *(registers_0_addr + REG_ROC_PRBS_CDCWRCNT);
                    // read content of CDF FIFO and print number WRCNT before and after
                    *(registers_0_addr + REG_ROC_PRBS_CDCRE) = 1;
                    delayUs(100);
                    volatile uint32_t dataout = *(registers_0_addr + REG_ROC_PRBS_CDCOUT);

                    outBuffer[bufcount++] = PRBSERRORDUMP;
                    bufWrite(outBuffer, &bufcount, 9, 2);
                    bufWrite(outBuffer, &bufcount, error_cnt, 2);
                    bufWrite(outBuffer, &bufcount, (dataout>>16) & 0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount, (dataout) & 0xFFFF, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_PRBS_CDCEMPTY), 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_PRBS_CDCWRCNT), 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARSETUP){
                    uint8_t var_pattern_en = (uint8_t) buffer[4];
                    uint8_t use_lane = (uint8_t) buffer[5];
                    uint8_t use_uart = (uint8_t) buffer[6];

                    // enable variable pattern to DDR
                    *(registers_0_addr + REG_ROC_DDR_PATTERN_EN) = var_pattern_en;

                    // set which lanes are sending data
                    *(registers_0_addr + REG_ROC_USE_LANE) = use_lane;

                    // set data to serial FIFO
                    *(registers_0_addr + REG_ROC_USE_UART) = use_uart;

                    outBuffer[bufcount++] = VARSETUP;
                    bufWrite(outBuffer, &bufcount, 3, 2);
                    bufWrite(outBuffer, &bufcount, var_pattern_en, 1);
                    bufWrite(outBuffer, &bufcount, use_lane, 1);
                    bufWrite(outBuffer, &bufcount, use_uart, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARERRORBACK){
                    // report which error was found: bit[0]=1 => event error, bit[1]=1 => header1 error
                    //                              bit[2]=1 => header2 error, bit[3]=1 => data error
                    outBuffer[bufcount++] = VARERRORBACK;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ERROR_READ), 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARERRORSET){
                    // set which error to be read
                    uint8_t var_req_error = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_ROC_DDR_ERROR_REQ) = var_req_error;

                    outBuffer[bufcount++] = VARERRORSET;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, var_req_error, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARERRORREAD){
                    // read first instance of observed and expected data for requested error
                    outBuffer[bufcount++] = VARERRORREAD;
                    bufWrite(outBuffer, &bufcount, 16, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ERROR_SEENM), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ERROR_SEENL), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ERROR_EXPCM), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ERROR_EXPCL), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARSIZEREAD){
                    outBuffer[bufcount++] = VARSIZEREAD;
                    bufWrite(outBuffer, &bufcount, 8, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_SIZE_WR), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_SIZE_RD), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARCNTREAD){
                    outBuffer[bufcount++] = VARCNTREAD;
                    bufWrite(outBuffer, &bufcount, 32, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_NULLHB_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_HB_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_ONHOLD_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_PREF_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DREQ_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DREQRD_CNT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DREQSENT), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DREQNULL), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == VARTAGREAD){
                    outBuffer[bufcount++] = VARTAGREAD;
                    bufWrite(outBuffer, &bufcount, 24, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_SPILL_TAG), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_HB_TAG), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_PRE_TAG), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_FETCH_TAG), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_DREQ_TAG), 4);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_DDR_OFFSET_TAG), 4);
                    outBufSend(g_uart, outBuffer, bufcount);

                }else if (commandID == READALIGN){
                    outBuffer[bufcount++] = READALIGN;
                    bufWrite(outBuffer, &bufcount, 3, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_READ_ALIGNED), 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_READ_ALIGNMENT), 2);
                    outBufSend(g_uart, outBuffer, bufcount);

#endif

#ifdef PROGRAMMING
                } else if (commandID == PROGRAMDIGIS) {

                    uint8_t actionCodes[8] = {
                            DP_DEVICE_INFO_ACTION_CODE,
                            DP_READ_IDCODE_ACTION_CODE,
                            DP_ERASE_ACTION_CODE,
                            DP_PROGRAM_ACTION_CODE,
                            DP_VERIFY_ACTION_CODE,
                            DP_ENC_DATA_AUTHENTICATION_ACTION_CODE,
                            DP_VERIFY_DIGEST_ACTION_CODE };

                    uint8_t indata = (uint8_t) buffer[4];
                    _Bool whichdigi = indata & 0x80;
                    uint8_t action = indata & 0xF;
                    Action_code = actionCodes[action];
                    device_family = G5M_DEVICE_FAMILY;
                    outBuffer[bufcount++] = PROGRAMDIGIS;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    g_pro_spi = (whichdigi == 0 ? g_cal_pro_spi : g_hv_pro_spi);
                    outBufSend(g_uart, outBuffer, bufcount);
                    dp_top();
#endif

#ifdef IAP
                } else if (commandID == IAPROC) {

                    clear_iap_data_buffer();
                    uint32_t fd_buffer[16];
                    uint16_t num_image = 0;

                    uint8_t indata = (uint8_t) buffer[4];
                    uint8_t action = indata & 0xF;
                    uint32_t rindex =  readU32fromBytes(&buffer[5]); //this can have multiple meanings, look below in execute_iap
                    uint32_t length = readU32fromBytes(&buffer[9]);


                    if (action==9){
                        num_image = (uint16_t) rindex;
                        for (uint16_t i = 0; i < num_image; i++){
                            fd_buffer[i] = readU32fromBytes(&buffer[13+4*i]);
                        }
                    }
                    outBuffer[bufcount++] = IAPROC;
                    bufWrite(outBuffer, &bufcount, 0, 2);
                    outBufSend(g_uart, outBuffer, bufcount);

                    switch (action) {
                    case 1:
                        execute_bitstream_authenticate(rindex);
                        break;
                    case 2:
                        execute_iap_image_authenticate(rindex);
                        break;
                    case 3:
                        UART_polled_tx_string(&g_uart,
                                (const uint8_t*) "Erase the PolarFire device using the FlashPro ERASE action.\n\rNext Power cycle the board to initiate AutoProgramming on Blank device.\r\n");

                        break;
                    case 4:
                    case 5:
                    case 6:
                        execute_iap(action, rindex);
                        break;
                    case 7:
                        list_flash_dir(rindex, length);
                        break;
                    case 8:
                        load_spi_flash_at_address(rindex); //this loads a single image at address index
                        break;


                    case 9:


                        write_flash_directory(rindex, fd_buffer);

                        break;
                    default:
                        //                        display_user_options();
                        break;
                    }

#endif

#ifdef	FIXEDSIZETEST


                }else if (commandID == READERROR){
                    uint8_t  count_addr = (uint8_t) buffer[4];

                    // pass error addree, read error counter
                    *(registers_0_addr + REG_ERROR_ADDRESS)  = count_addr;
                    delay_ms(1);

                    outBuffer[bufcount++] = READERROR;
                    bufWrite(outBuffer, &bufcount, 3, 2);
                    bufWrite(outBuffer, &bufcount, count_addr, 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ERROR_COUNTER), 2);
                    outBufSend(g_uart, outBuffer, bufcount);


                }else if (commandID == READALIGN){
                    outBuffer[bufcount++] = READALIGN;
                    bufWrite(outBuffer, &bufcount, 3, 2);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_READ_ALIGNED), 1);
                    bufWrite(outBuffer, &bufcount, *(registers_0_addr + REG_ROC_READ_ALIGNMENT), 2);
                    outBufSend(g_uart, outBuffer, bufcount);
#endif

                } else if (commandID == SETDIGIRW) {

                    // set if fiber (0) or serial (1) drives signals to DIGIs
                    uint8_t digirw_sel = (uint8_t) buffer[4];
                    *(registers_0_addr + REG_DIGIRW_SEL) = digirw_sel;

                    outBuffer[bufcount++] = SETDIGIRW;
                    bufWrite(outBuffer, &bufcount, 1, 2);
                    bufWrite(outBuffer, &bufcount, digirw_sel, 1);
                    outBufSend(g_uart, outBuffer, bufcount);

                    //***********************************begin of control_digi commands*******************************************************************************

                } else if (commandID == ADCRWCMDID) {
                    // adc read/write
                    uint8_t adcnum  = (uint8_t) buffer[4];
                    uint8_t adcrw   = (uint8_t) buffer[5];
                    uint16_t address = readU16fromBytes(&buffer[6]);
                    uint16_t data = readU16fromBytes(&buffer[8]);

                    outBuffer[bufcount++] = ADCRWCMDID;
                    bufWrite(outBuffer, &bufcount, 6, 2);

                    if (adcrw == 1) {
                        uint8_t result = adc_read(address, adcnum);
                        //                      sprintf(outBuffer,"Read adc %d address %02x: %02x\n",adcnum,address,result);
                        //                      UART_polled_tx_string( &g_uart, outBuffer );

                        outBuffer[bufcount++] = adcrw;
                        outBuffer[bufcount++] = adcnum;
                        bufWrite(outBuffer, &bufcount, address, 2);
                        outBuffer[bufcount++] = result;
                        outBuffer[bufcount++] = 0;
                        outBufSend(g_uart, outBuffer, bufcount);
                    } else {
                        adc_write(address, (uint8_t) data, (0x1 << adcnum));
                        //                      sprintf(outBuffer,"Wrote adc %d address %02x: %02x\n",adc_num,address,data);
                        //                      UART_polled_tx_string( &g_uart, outBuffer );

                        outBuffer[bufcount++] = adcrw;
                        outBuffer[bufcount++] = adcnum;
                        bufWrite(outBuffer, &bufcount, address, 2);
                        bufWrite(outBuffer, &bufcount, data, 2);
                        outBufSend(g_uart, outBuffer, bufcount);
                    }

                    //				}else if (commandID == BITSLIPCMDID){
                    //
                    ////					// bitslip
                    ////					uint16_t num_bits = readU16fromBytes(&buffer[4]);
                    ////					channel_mask[0] = readU32fromBytes(&buffer[6]);
                    ////					channel_mask[1] = readU32fromBytes(&buffer[10]);
                    ////					channel_mask[2] = readU32fromBytes(&buffer[14]);
                    ////					get_mapped_channels();
                    ////
                    ////
                    ////	      //for (int i=0;i<num_bits;i++){
                    ////					 *(registers_0_addr + 0x30) = ((mapped_channel_mask1 & 0xFF)>>0);
                    ////					 *(registers_0_addr + 0x31) = ((mapped_channel_mask1 & 0xFF00)>>8);
                    ////					 *(registers_0_addr + 0x32) = ((mapped_channel_mask1 & 0xFF0000)>>16);
                    ////					 *(registers_0_addr + 0x33) = ((mapped_channel_mask1 & 0xFF000000)>>24);
                    ////					 *(registers_0_addr + 0x34) = ((mapped_channel_mask2 & 0xFF)>>0);
                    ////					 *(registers_0_addr + 0x35) = ((mapped_channel_mask2 & 0xFF00)>>8);
                    ////					 *(registers_0_addr + 0x30) = 0x0;
                    ////					 *(registers_0_addr + 0x31) = 0x0;
                    ////					 *(registers_0_addr + 0x32) = 0x0;
                    ////					 *(registers_0_addr + 0x33) = 0x0;
                    ////					 *(registers_0_addr + 0x34) = 0x0;
                    ////					 *(registers_0_addr + 0x35) = 0x0;
                    ////	      delayUs(100);
                    ////	      //}
                    ////
                    ////	      sprintf(outBuffer,"Activated bitslip %d times for channels %08x %08x %08x\n",num_bits,channel_mask1,channel_mask2,channel_mask3);
                    ////	      UART_polled_tx_string( &g_uart, outBuffer );
                    //
                    //					volatile uint32_t *empty_p = (registers_0_addr + REG_ROC_FIFO_EMPTY);
                    //					volatile uint32_t *full_p = (registers_0_addr + REG_ROC_FIFO_FULL);
                    //					volatile uint32_t *data_p = (registers_0_addr + REG_ROC_FIFO_DATA);
                    //
                    //					uint32_t empty = *(empty_p);
                    //					uint32_t full = *(full_p);
                    //					uint32_t data1 = *(data_p);
                    //
                    //					*(registers_0_addr + REG_ROC_FIFO_RE) = 1;
                    //					uint32_t data2 = *(data_p);
                    //
                    //					//					sprintf(outBuffer,"Empty: %d, Full: %d, data1: %04x, data2: %04x\n",empty,full,data1,data2);
                    //					//					UART_polled_tx_string( &g_uart, outBuffer );
                    //					outBuffer[bufcount++] = BITSLIPCMDID;
                    //					bufWrite(outBuffer, &bufcount, 32, 2);
                    //					bufWrite(outBuffer, &bufcount, empty, 4);
                    //					bufWrite(outBuffer, &bufcount, full, 4);
                    //					bufWrite(outBuffer, &bufcount, data1, 4);
                    //					bufWrite(outBuffer, &bufcount, data2, 4);
                    //
                    //					outBufSend(g_uart, outBuffer, bufcount);

#ifdef DIGITEST

                } else if (commandID == AUTOBITSLIPCMDID) {
                    //reset the ADCs first, at this point, clock better be stable
                    //digiRW(1,4+i,0x08,3)
                    for (int i = 0; i < 12; i++) {
                        uint16_t adc_mask = (0x1 << i);
                        adc_write(0x08, 3, adc_mask);
                    }

                    for (int i = 0; i < 12; i++) {
                        uint16_t adc_mask = (0x1 << i);
                        adc_write(0x08, 0, adc_mask);
                    }

                    adc_write(ADC_ADDR_SRCTRL, 0xC3, 0xFFF); //set to LSB first

                    autobitslip();

                } else if (commandID == READRATESCMDID) {
                    uint16_t num_lookback = readU16fromBytes(&buffer[4]);
                    uint16_t num_samples = readU16fromBytes(&buffer[6]);
                    channel_mask[0] = readU32fromBytes(&buffer[8]);
                    channel_mask[1] = readU32fromBytes(&buffer[12]);
                    channel_mask[2] = readU32fromBytes(&buffer[16]);

                    outBuffer[bufcount++] = READRATESCMDID;
                    bufcount_place_holder = bufcount;
                    bufWrite(outBuffer, &bufcount, 0, 2);

                    // old GET_RATES functions used for serial
                    // get_rates(num_lookback,num_samples,255,NULL);

                    // new GET_RATES function with extra outputs to read rates via fiber
                    // MUST initialize GET_RATES outputs!
                    uint32_t total_time_counts[2] = {0};
                    uint32_t total_hv[96] = {0};
                    uint32_t total_cal[96] = {0};
                    uint32_t total_coinc[96] = {0};
                    uint32_t timecounts;

                    get_rates(num_lookback,num_samples,255,&timecounts, total_hv, total_cal, total_coinc, total_time_counts);

                    bufWrite(outBuffer, &bufcount_place_holder, (bufcount - 3),
                            2);
                    outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == READDATACMDID) {
                    *(registers_0_addr + 0xEE) = 0x1;
                    delayUs(1);
                    *(registers_0_addr + 0xEE) = 0x0;

                    uint16_t adc_mode = (uint16_t) ((uint8_t) buffer[4]);
                    uint16_t tdc_mode = (uint16_t) ((uint8_t) buffer[5]);
                    uint16_t num_lookback = readU16fromBytes(&buffer[6]);
                    uint16_t num_samples = readU16fromBytes(&buffer[8]);
                    uint32_t num_triggers = readU32fromBytes(&buffer[10]);
                    channel_mask[0] = readU32fromBytes(&buffer[14]);
                    channel_mask[1] = readU32fromBytes(&buffer[18]);
                    channel_mask[2] = readU32fromBytes(&buffer[22]);
                    uint8_t clock = (uint8_t) buffer[26]; // ???? hard-coded in python
                    uint8_t enable_pulser = (uint8_t) buffer[27];
                    uint16_t max_total_delay = readU16fromBytes(&buffer[28]);
                    //uint8_t mode = buffer[30];
                    uint8_t mode = 0;
                    uint8_t marker_clock = buffer[30];

                    if (marker_clock & 0x1)
                        *(registers_0_addr + REG_ROC_ENABLE_FIBER_CLOCK) = 1;
                    else
                        *(registers_0_addr + REG_ROC_ENABLE_FIBER_CLOCK) = 0;

                    if (marker_clock & 0x2)
                        *(registers_0_addr + REG_ROC_ENABLE_FIBER_MARKER) = 1;
                    else
                        *(registers_0_addr + REG_ROC_ENABLE_FIBER_MARKER) = 0;

                    digi_write(DG_ADDR_ENABLE_PULSER, enable_pulser, 0);
                    if ((mode & 0x1) == 0x0) {
                        get_mapped_channels();
                        uint32_t used_adcs = 0x0;
                        for (int i = 0; i < 12; i++) {
                            uint32_t test_mask = (0xFF << (8 * (i % 4)));
                            if (mapped_channel_mask[i / 4] & test_mask)
                                used_adcs |= (0x1 << i);
                        }
                        //used_adcs = 0x800;
                        for (uint8_t i = 0; i < 12; i++) {
                            if ((0x1 << i) & ENABLED_ADCS) {
                                if (clock < 99) {
                                    //if ((0x1<<i) & used_adcs)
                                    adc_write(ADC_ADDR_PHASE, clock,
                                            (0x1 << i));
                                    adc_write(ADC_ADDR_TESTIO, adc_mode,
                                            (0x1 << i));
                                } else {
                                    adc_write(ADC_ADDR_PHASE, adc_phases[i],
                                            (0x1 << i));
                                    adc_write(ADC_ADDR_TESTIO, adc_mode,
                                            (0x1 << i));
                                }
                                //uint8_t readbackphase = adc_read(ADC_ADDR_PHASE,i);
                                //uint8_t readbackphase2 = adc_read(ADC_ADDR_PHASE,i);
                            }

                        }
                        //adc_write(ADC_ADDR_TESTIO,2,(0x1<<10));
                        //adc_write(ADC_ADDR_TESTIO,9,(0x1<<11));
                        //uint8_t phase[12];
                        //phase[0] = adc_read(ADC_ADDR_PHASE,0);
                        //phase[1] = adc_read(ADC_ADDR_PHASE,1);
                        //phase[2] = adc_read(ADC_ADDR_PHASE,2);
                        //phase[3] = adc_read(ADC_ADDR_PHASE,3);
                        //phase[4] = adc_read(ADC_ADDR_PHASE,4);
                        //phase[5] = adc_read(ADC_ADDR_PHASE,5);
                        //phase[6] = adc_read(ADC_ADDR_PHASE,6);
                        //phase[7] = adc_read(ADC_ADDR_PHASE,7);
                        //phase[8] = adc_read(ADC_ADDR_PHASE,8);
                        //phase[9] = adc_read(ADC_ADDR_PHASE,9);
                        //phase[10] = adc_read(ADC_ADDR_PHASE,10);
                        //phase[11] = adc_read(ADC_ADDR_PHASE,11);

                        hvcal = 1;
                        get_mapped_channels();

                        // DISABLE CALIBRATION BEFORE ANY CHANGES TO SETTINGS
                        digi_write(0x0F, 0, 0); // disable calibration

                        digi_write(DG_ADDR_SAMPLE, num_samples, 0);
                        digi_write(DG_ADDR_LOOKBACK, num_lookback, 0);
                        digi_write(DG_ADDR_MASK1,
                                (uint16_t) (mapped_channel_mask[0] & 0xFFFF),
                                1);
                        digi_write(DG_ADDR_MASK2,
                                (uint16_t) ((mapped_channel_mask[0] & 0xFFFF0000)
                                        >> 16), 1);
                        digi_write(DG_ADDR_MASK3,
                                (uint16_t) (mapped_channel_mask[1] & 0xFFFF),
                                1);
                        digi_write(DG_ADDR_MASK1,
                                (uint16_t) ((mapped_channel_mask[1] & 0xFFFF0000)
                                        >> 16), 2);
                        digi_write(DG_ADDR_MASK2,
                                (uint16_t) (mapped_channel_mask[2] & 0xFFFF),
                                2);
                        digi_write(DG_ADDR_MASK3,
                                (uint16_t) ((mapped_channel_mask[2] & 0xFFFF0000)
                                        >> 16), 2);

			if (tdc_mode < 4)
			  digi_write(DG_ADDR_TRIGGER_MODE, tdc_mode, 0);
                        digi_write(DG_ADDR_ENABLE_PULSER, enable_pulser, 0);

                        *(registers_0_addr + REG_ROC_EWW_PULSER) = 0;
                        //*(registers_0_addr + REG_ROC_EWM_T) = max_total_delay;
                        //*(registers_0_addr + REG_ROC_EWM_T) = 0x0FFF;

                        //					max_total_delay = 1;

                        *(registers_0_addr + REG_ROC_FIFO_HOWMANY) =
                                num_samples;

                        if ((mapped_channel_mask[2] != 0)
                                || ((mapped_channel_mask[1] >> 16) != 0))
                            hvcal = 2;

                        num_samples = digi_read(DG_ADDR_SAMPLE, hvcal);
                        enable_pulser = digi_read(DG_ADDR_ENABLE_PULSER, hvcal);
                    }

                    char tdc_string[8];
                    if (enable_pulser == 0) {
                        //	strncpy(tdc_string, "REAL  \0", 7);
                        tdc_string[0] = 'R';
                        tdc_string[1] = 'E';
                        tdc_string[2] = 'A';
                        tdc_string[3] = 'L';
                        tdc_string[4] = ' ';
                        tdc_string[5] = ' ';
                        tdc_string[6] = '\0';
                    } else {
                        //	strncpy(tdc_string, "PULSER\0", 7);
                        tdc_string[0] = 'P';
                        tdc_string[1] = 'U';
                        tdc_string[2] = 'L';
                        tdc_string[3] = 'S';
                        tdc_string[4] = 'E';
                        tdc_string[5] = 'R';
                        tdc_string[6] = '\0';
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
                    for (uint8_t i = 0; i < 8; i++)
                        outBuffer[bufcount++] = (uint8_t) tdc_string[i];
                    outBuffer[bufcount++] = clock;
                    bufWrite(outBuffer, &bufcount, num_triggers, 4);
                    bufWrite(outBuffer, &bufcount,
                            digi_read(DG_ADDR_MASK1, hvcal), 2);
                    bufWrite(outBuffer, &bufcount,
                            digi_read(DG_ADDR_MASK2, hvcal), 2);
                    bufWrite(outBuffer, &bufcount,
                            digi_read(DG_ADDR_MASK3, hvcal), 2);
                    bufWrite(outBuffer, &bufcount,
                            digi_read(DG_ADDR_ENABLE_PULSER, hvcal), 2);

                    outBuffer[bufcount++] = mode;
                    outBufSend(g_uart, outBuffer, bufcount);

                    delayUs(10000);

                    if ((mode & 0x1) == 0x0) {
                        // reset fifo
                        //						*(registers_0_addr + REG_ROC_USE_LANE) = 0xF;
                        resetFIFO();
                        resetFIFO();
                        delayUs(10000);
                        *(registers_0_addr + REG_ROC_EWW_PULSER) = 1;

                        digi_write(0x0F, enable_pulser, 0); // enable calibration IF running internal pulser

                        //readout_obloc = 6;
                        readout_obloc = 0;
                        //sprintf(dataBuffer,"start\n");
                        readout_maxDelay = max_total_delay * 50;
                        readout_mode = mode;
                        readout_wordsPerTrigger = 8; //NUMTDCWORDS + 4*num_samples;
                        readout_numTriggers = num_triggers;
                        readout_totalTriggers = 0;

                        readout_noUARTflag = 0;
                    }
                    readout_numTriggers = num_triggers;

                    // During this mode, readout is disabled while triggers are enabled
                    // readout occurs
                    if ((mode & 0x2) != 0x0)
                        readout_noUARTflag = 1;

                    if ((mode & 0x1) == 0x0) {
                        int delay_count = 0;
                        int trigger_count = 0;

                        readout_requestedTriggers = num_triggers;

                        if (num_triggers == 0) {

                            readout_obloc = 0;
                            bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                            UART_send(&g_uart, dataBuffer, 2);
                            //sprintf(&dataBuffer[readout_obloc],"\nend\n");
                            //UART_polled_tx_string( &g_uart, dataBuffer );
                            bufcount = 0;
                            outBuffer[bufcount++] = READDATACMDID;
                            bufWrite(outBuffer, &bufcount, 5, 2);
                            outBuffer[bufcount++] =
                                    (uint8_t) (readout_totalTriggers
                                            == readout_requestedTriggers);
                            //sprintf(outBuffer,"SUCCESS! Delayed %d times\n",delay_count);
                            bufWrite(outBuffer, &bufcount,
                                    (uint32_t) readout_totalDelays, 4);

                            UART_send(&g_uart, outBuffer, bufcount);
                            readout_requestedTriggers = 0;
                            readout_totalTriggers = 0;
                        }

                        /*
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
                         */

                        //UART_polled_tx_string( &g_uart, outBuffer );
                        //get_rates(0,10);
                    } else {

                        readout_obloc = 0;

                        int delay_count = 0;
                        int trigger_count = 0;

                        read_data(&delay_count, &trigger_count);

                        //sprintf(&dataBuffer[readout_obloc],"\nend\n");
                        //UART_polled_tx_string( &g_uart, dataBuffer );
                        bufcount = 0;
                        outBuffer[bufcount++] = READDATACMDID;
                        bufWrite(outBuffer, &bufcount, 5, 2);
                        outBuffer[bufcount++] = (uint8_t) (trigger_count
                                == num_triggers);

                        if (trigger_count == num_triggers) {
                            //sprintf(outBuffer,"SUCCESS! Delayed %d times\n",delay_count);
                            bufWrite(outBuffer, &bufcount,
                                    (uint32_t) delay_count, 4);
                        } else {
                            //sprintf(outBuffer,"FAILED! Read %d triggers\n",trigger_count);
                            bufWrite(outBuffer, &bufcount,
                                    (uint32_t) trigger_count, 4);
                        }

                        UART_send(&g_uart, outBuffer, bufcount);

                        //readout_mode = 1;
                        //sprintf(outBuffer,"Run started\n");
                        //UART_polled_tx_string( &g_uart, outBuffer );
                        //outBuffer[0] = RUN_STARTED;
                        //UART_send(&g_uart, outBuffer ,1);

                    }

                    //				}else if (commandID == STOPRUNCMDID){
                    //
                    //					readout_mode = 0;
                    //					// first send end data flag to break out of loop on python
                    //					readout_obloc = 0;
                    //					bufWrite(dataBuffer, &readout_obloc, ENDOFDATA, 2);
                    //					UART_send(&g_uart, dataBuffer ,2);
                    //
                    //					// now send READDATACMDID response to finish there
                    //					bufcount = 0;
                    //					outBuffer[bufcount++] = READDATACMDID;
                    //					bufWrite(outBuffer, &bufcount, 5, 2);
                    //					outBuffer[bufcount++] = 0; // as a hack for now we have it fail so it tells us total number of triggers
                    //					bufWrite(outBuffer, &bufcount, (uint32_t)readout_totalTriggers, 4);
                    //					UART_send(&g_uart, outBuffer ,bufcount );
                    //
                    //					/*
                    //					outBuffer[bufcount++] = STOPRUNCMDID;
                    //					bufWrite(outBuffer, &bufcount, 3, 2);
                    //					outBuffer[bufcount++] = readout_enabled;
                    //
                    //					if (readout_enabled == 0){
                    //						//sprintf(outBuffer,"Error: no run to stop\n");
                    //						//UART_polled_tx_string( &g_uart, outBuffer );
                    //						bufWrite(outBuffer, &bufcount, 0, 2);
                    //					}else{
                    //						readout_enabled = 0;
                    //						//sprintf(&dataBuffer[readout_obloc],"\nend\n");
                    //						//UART_polled_tx_string( &g_uart, dataBuffer );
                    //						//sprintf(outBuffer,"Run ended. Read %d triggers\n",readout_totalTriggers);
                    //						//UART_polled_tx_string( &g_uart, outBuffer );
                    //						bufWrite(outBuffer, &bufcount, readout_totalTriggers, 2);
                    //					}
                    //					outBufSend(g_uart, outBuffer, bufcount);
                    //					*/

                    //				}else if (commandID == ADCINITINFOCMDID){
                    //
                    //					outBuffer[bufcount++] = ADCINITINFOCMDID;
                    //					bufWrite(outBuffer, &bufcount, 30, 2);
                    //					for (uint8_t i=0; i<30; i++)
                    //						outBuffer[bufcount++] = init_buff[i];
                    //					outBufSend(g_uart, outBuffer, bufcount);

                    //				}else if (commandID == PACKAGETESTCMDID){
                    //					outBuffer[bufcount++] = PACKAGETESTCMDID;
                    //					outBuffer[bufcount++] = 0xCA;
                    //					outBuffer[bufcount++] = 1;
                    //					for (uint16_t i=0; i<458; i++)
                    //						outBuffer[bufcount++] = i%256;
                    //					outBufSend(g_uart, outBuffer, bufcount);

                    //				}else if (commandID == FINDTHRESHOLDSCMDID){
                    //					uint16_t num_lookback = readU16fromBytes(&buffer[4]);
                    //					uint16_t num_samples = readU16fromBytes(&buffer[6]);
                    //					channel_mask[0] = readU32fromBytes(&buffer[8]);
                    //					channel_mask[1] = readU32fromBytes(&buffer[12]);
                    //					channel_mask[2] = readU32fromBytes(&buffer[16]);
                    //					uint16_t target_rate = readU16fromBytes(&buffer[20]);
                    //					uint8_t verbose = (uint8_t) buffer[22];
                    //					//if single channel, verbose = 1 prints the detailed process for cal/hv
                    //
                    //					outBuffer[bufcount++] = FINDTHRESHOLDSCMDID;
                    //					bufcount_place_holder = bufcount;
                    //					bufWrite(outBuffer, &bufcount, 0, 2);
                    //					bufWrite(outBuffer, &bufcount, num_lookback, 2);
                    //					bufWrite(outBuffer, &bufcount, num_samples, 2);
                    //					bufWrite(outBuffer, &bufcount, target_rate, 2);
                    //					bufWrite(outBuffer, &bufcount, verbose, 1);
                    //
                    //					//disable pulser
                    //					digi_write(DG_ADDR_ENABLE_PULSER,0,0);
                    //
                    //					for (uint8_t channel=0; channel<96; channel++){
                    //						thischanmask = (((uint32_t) 0x1)<<(channel%32));
                    //						if 	( ((channel<32) && ((thischanmask & channel_mask[0]) == 0x0))||
                    //								((channel>=32) && (channel<64) && ((thischanmask & channel_mask[1]) == 0x0))||
                    //								((channel>=64) && ((thischanmask & channel_mask[2]) == 0x0))	){
                    //							continue;
                    //						}
                    //						//prints initial settings
                    //						bufWrite(outBuffer, &bufcount, channel, 1);
                    //						bufWrite(outBuffer, &bufcount, default_gains_cal[channel], 2);
                    //						bufWrite(outBuffer, &bufcount, default_threshs_cal[channel], 2);
                    //						bufWrite(outBuffer, &bufcount, default_gains_hv[channel], 2);
                    //						bufWrite(outBuffer, &bufcount, default_threshs_hv[channel], 2);
                    //
                    //						findChThreshold(num_lookback, num_samples, channel, target_rate, verbose);
                    //						findChThreshold(num_lookback, num_samples, channel+96, target_rate, verbose);
                    //					}
                    //
                    //					bufWrite(outBuffer, &bufcount_place_holder, (bufcount-3), 2);
                    //					outBufSend(g_uart, outBuffer, bufcount);

                } else if (commandID == MEASURETHRESHOLDCMDID) {
                    channel_mask[0] = readU32fromBytes(&buffer[4]);
                    channel_mask[1] = readU32fromBytes(&buffer[8]);
                    channel_mask[2] = readU32fromBytes(&buffer[12]);
                    get_mapped_channels();

                    outBuffer[bufcount++] = MEASURETHRESHOLDCMDID;
                    bufcount_place_holder = bufcount;
                    bufWrite(outBuffer, &bufcount, 0, 2);

                    //enable pulser as readstrawcmd does
                    digi_write(DG_ADDR_ENABLE_PULSER, 1, 0);

                    uint16_t threshold_array[288];
                    for (uint16_t i = 0; i < 288; i++)
                        threshold_array[i] = 0xFFFF;

                    for (uint8_t ihvcal = 1; ihvcal < 3; ihvcal++) {
                        for (uint8_t k = (48 * (ihvcal - 1)); k < 48 * ihvcal;
                                k++) {
                            uint8_t condition = 0;
                            uint8_t straw_num = channel_map[k];
                            if (ihvcal == 1)
                                condition =
                                        ((k < 32
                                                && ((0x1 << k)
                                                        & mapped_channel_mask[0]))
                                                || (k >= 32
                                                        && ((0x1 << (k - 32))
                                                                & mapped_channel_mask[1])));
                            else if (ihvcal == 2)
                                condition =
                                        ((k < 64
                                                && ((0x1 << (k - 32))
                                                        & mapped_channel_mask[1]))
                                                || (k >= 64
                                                        && ((0x1 << (k - 64))
                                                                & mapped_channel_mask[2])));

                            if (condition) {
                                uint16_t gain_cal[3] = { 0,
                                        default_gains_cal[straw_num],
                                        default_gains_cal[straw_num] };
                                uint16_t gain_hv[3] = {
                                        default_gains_hv[straw_num], 0,
                                        default_gains_hv[straw_num] };
                                //first zero cal, then hv, then both to default

                                digi_write(DG_ADDR_SELECTSMA, k % 48, ihvcal);
                                //select channel
                                for (uint8_t i = 0; i < 3; i++) {
                                    setPreampGain(straw_num, gain_cal[i]);
                                    setPreampGain(straw_num + 96, gain_hv[i]);
                                    hwdelay(500000);//wait for 10ms gain to reach written value and for SMA to settle

                                    digi_write(DG_ADDR_SMARDREQ, 1, ihvcal);
                                    delayUs(5);	//write READREQ to 1 and freeze SMA module

                                    threshold_array[96 * i + straw_num] =
                                            digi_read(DG_ADDR_SMADATA, ihvcal);
                                    digi_write(DG_ADDR_SMARDREQ, 0, ihvcal); //unfreeze SMA module
                                }
                            }
                        }
                    }

                    for (uint16_t i = 0; i < 288; i++)
                        bufWrite(outBuffer, &bufcount, threshold_array[i], 2);

                    bufWrite(outBuffer, &bufcount_place_holder, (bufcount - 3),
                            2);
                    outBufSend(g_uart, outBuffer, bufcount);

#endif
                } // end of commands if

                // If we didn't use the whole buffer, the rest must be the next command
                //memmove(&buffer[0],&buffer[numBytes],writePtr-numBytes);
                for (uint16_t j = 0; j < writePtr - numBytes; j++) {
                    buffer[j] = buffer[j + numBytes];
                }
                writePtr -= numBytes;
            }
        }
    }

    return 0;
}

