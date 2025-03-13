#ifndef UTILS_H_
#define UTILS_H_


#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreGPIO/core_gpio.h"
#include "drivers/CoreSPI/core_spi.h"
#include "drivers/CorePWM/core_pwm.h"
#include "hw_platform.h"

#define TICKPERUS 50

#define ENABLED_ADCS				0xFFFu
//#define ENABLED_ADCS				0x3Fu

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

#define NUMTDCWORDS      5 //1 event window counter + 4 header words
#define HVANDCAL         0
#define CALONLY          1
#define HVONLY           2

#define THRESHOLDSTART   350
#define THRESHOLDEND     500
#define THRESHOLDSTEP    5

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
#define DIGIRW 16
#define RESETDEVICE 17
#define ROCREADREG 18
#define SETFUSEON 19
#define SETFUSEOFF 20
#define READKEY 21
#define GETILP 22
#define GETGITREV 23
#define NVMWRITE 24

//DDR command ID
#define DDRSETUP 50
#define DDRREAD 51
#define DDRSTATUS 52
#define DDRFILL 53
#define DDRRAMREAD 54
#define DDRWRITE 55
#define DDRPATTERNREAD 56
#define DDRRAMFILL 57
#define DDRBURSTREAD 58
#define DDRSIMUSET 59

// DTC SIM command ID
#define DCSRAMWRITE  60
#define DCSREPLY     61
#define DCSREAD  	 62
#define DCSWRITE 	 63
#define DCSMODREAD   64
#define DCSMODWRITE  65
#define DCSMARKER  	 66
#define DCSHEARTBEAT 67
#define DCSDATAREQ   68
#define DCSBLKREAD   69
#define DCSBLKWRITE  70

//DDR speed tests
#define	DDRSPEEDTEST	71
#define	DDRSPEEDREAD  	72
#define	DDRSPEEDERR		73
// "DIGI" command ID (1 byte)

// PRBS commands
#define PRBSSTATUS   74
#define PRBSSTART    75
#define PRBSSTOP     76
#define PRBSERRORIN  77
#define PRBSERRORCLR 78
#define PRBSERRORDUMP 79

#define PRBSET      80
#define SERIALSET   81
#define XCVRALIGN   82
#define FORCEDDRRD  83

// variable size DDR tests
#define VARSETUP		84
#define VARERRORSET		85
#define VARERRORBACK	86
#define VARERRORREAD	87
#define VARSIZEREAD		88
#define VARCNTREAD		89
#define VARTAGREAD		90
#define CFOEMUL			91
#define READERROR		92
#define READALIGN		93
#define SETDIGIRW       94

#define DDRTESTWRITE  95
#define DDRTESTREAD   96
#define DDRTESTSETUP  97
#define DDRTESTSTATUS 98
#define DDRTESTEN     99

#define ADCRWCMDID 101
#define BITSLIPCMDID 102
#define AUTOBITSLIPCMDID 103
#define READRATESCMDID 104
#define READDATACMDID 105
#define STOPRUNCMDID 106
#define ADCINITINFOCMDID 107
#define FINDTHRESHOLDSCMDID 108
#define MEASURETHRESHOLDCMDID 109

#define PACKAGETESTCMDID 151

#define PROGRAMDIGIS 152
#define IAPROC 153
#define INITDIGIS 155

// any command below is coming from DCS write to that address
// address space is 0x100-0x200
#define TESTDCSNGL  256 // 0x100 - example of Single WR to value V and Single RD back of value V
#define TESTDCSBLK  257 // 0x101 - example of BLOCK read of N values specified by a DCS WRITE
#define READSPI     258 // 0x102 - fiber version of READMONADCS: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of 36 ADCs values
#define READBACKBLK 259 // 0x103 - write and read back BLOCK_WRITE of N values via "rocUtil block_write -a 259 -c N"
#define READDEVICE  260 // 0x104 - fiber version of GETDEVICEID: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of 52 ID values
#define READSENSOR  261 // 0x105 - fiber version of READBMES: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of 52 ID values
#define TALKTOADC   262 // 0x106 - fiber version of ADCRWCMDID with parameters passed via BLOCK_WRITE and results read via simple read
#define TALKTODIGI  263 // 0x107 - fiber version of DIGIRW with parameters passed via BLOCK_WRITE
#define FINDALIGN   264 // 0x108 - fiber version of AUTOBITSLIPCMDID with parameters passed via SINGLE or BLOCK_WRITE - Does only 1 iteration!!
#define READDATA    265 // 0x109 - fiber version of READDATACMDID with parameters passed via BLOCK_WRITE
#define PREAMPGAIN  266 // 0x10A - fiber version of SETPREAMPGAIN with parameters passed via BLOCK_WRITE
#define PREAMPTHRESH  267 // 0x10B - fiber version of SETPREAMPTHRESHOLD with parameters passed via BLOCK_WRITE
#define PULSERON    268 // 0x10C - fiber version of SETPULSERON with parameters passed via BLOCK_WRITE
#define PULSEROFF   269 // 0x10D - fiber version of SETPULSEROFF with parameters passed via  DCS WRITE
#define MEASURETHRESH 270 // 0x10E - fiber version of MEASURETHRESHOLDCMDID with parameters passed via BLOCK_WRITE
#define READRATES   271 // 0x10F - fiber version of READRATESCMDID with parameters passed via BLOCK_WRITE
#define READGITREV  272 // 0x110 - fiber version of GETGITREV: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of size returned by reg 129
#define READILP     273 // 0x111 - fiber version of GETILP: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of 4 values
#define GETKEY      274 // 0x112 - fiber version of READKEY: execute with Single DCS_WRITE of any value (ignored) followed by DCS_BLOCK_READ of of 4 values
#define DIAGDATA    511 // 0x1FF - Block RD of assorted diagnostics, started by a single WR of any value

// RS485 commands: can be from 0 to 255
#define TEST                255
#define READSPI_I33         1
#define READSPI_I25         2
#define READSPI_ROCRAIL_1V  25  //(0x19)
#define READSPI_CALRAIL_1V  29  //(0x1D)
//******************************************************************************
//                             Registers & Addresses
//******************************************************************************

// MT: RS485 specific
#define CRRS485_RX_READY    0x1   // RS485 has received data
#define CRRS485_RX_READ     0x2   // uProc reads data from RS485 logic
#define CRRS485_TX_WRITE    0x3   // uProc writes data to RS485 logic
#define CRRS485_RX_ACK      0x5   // uProc has seen data from RS485
#define CRRS485_TX_BUSY     0x6   // RS485 is busy sending data out
#define CRRS485_TX_START    0x7   // RS485 can start sending one 32-bit word
#define CRRS485_ADDRESS     0xC3

// MT DTC
//*********************** CR_DTC* are the address offsets for DTC commands
#define CRDCS_CMD_STATUS    0x0
#define CRDCS_CMD_READY     0x1
#define CRDCS_READ_RX       0x2
#define CRDCS_WRITE_TX      0x3
#define CRDCS_DIAG_DATA     0x4

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
#define DG_ADDR_DIGIRW_SEL 0x11

#define DG_ADDR_READCHANNEL 0x15
#define DG_ADDR_GT0 0x16
#define DG_ADDR_GT1 0x17
#define DG_ADDR_GT2 0x18
#define DG_ADDR_GT3 0x19
#define DG_ADDR_HV 0x1A
#define DG_ADDR_CAL 0x1B
#define DG_ADDR_COINC 0x1C
#define DG_ADDR_LATCH 0x1D

#define DG_ADDR_HISTO_RESET 0x20
#define DG_ADDR_HISTO_BIN 0x21
#define DG_ADDR_HISTO_VAL 0x22
#define DG_ADDR_HISTO_CHANNEL 0x23

//#define DG_ADDR_BITSLIP0 0x30
//#define DG_ADDR_BITSLIP1 0x31
//#define DG_ADDR_BITSLIP2 0x32
//#define DG_ADDR_BITSLIP3 0x33
//#define DG_ADDR_BITSLIP4 0x34
//#define DG_ADDR_BITSLIP5 0x35

#define DG_ADDR_TVS_VAL 0x40
#define DG_ADDR_TVS_ADDR 0x41

#define DG_ADDR_SELECTSMA 0x50
#define DG_ADDR_SMARDREQ 0x51
#define DG_ADDR_SMADATA 0x52

#define DG_ADDR_RX_CH_MASK1 0x60
//#define DG_ADDR_RX_CH_MASK2 0x61
//#define DG_ADDR_RX_CH_MASK3 0x62
#define DG_ADDR_BITALIGN_EWM_WIDTH 0x63
#define DG_ADDR_BITALIGN_RSTRT 0x64
#define DG_ADDR_BITSLIP_STRT 0x65
#define DG_ADDR_BITALIGN_CMP1 0x66
//#define DG_ADDR_BITALIGN_CMP2 0x67
//#define DG_ADDR_BITALIGN_CMP3 0x68
#define DG_ADDR_BITALIGN_ERR1 0x69
//#define DG_ADDR_BITALIGN_ERR2 0x6A
//#define DG_ADDR_BITALIGN_ERR3 0x6B
#define DG_ADDR_BITSLIP_DONE1 0x6C
//#define DG_ADDR_BITSLIP_DONE2 0x6D
//#define DG_ADDR_BITSLIP_DONE3 0x6E
#define DG_ADDR_BITALIGN_RSETN 0x6F
#define DG_ADDR_BSC_OPERATION_TYPE 0x70
#define DG_ADDR_BSC_PATTERN_MATCH1 0x71
//#define DG_ADDR_BSC_PATTERN_MATCH2 0x72
//#define DG_ADDR_BSC_PATTERN_MATCH3 0x73

//#define DG_ADDR_EWMCNTER 0x80
#define DG_ADDR_EWS 0x81 // (event window start 160MHz clock ticks)
#define DG_ADDR_EWE 0x82 // (event window end 160 MHz clock ticks)
#define DG_ADDR_DIGINUMBER 0x90 // (0=cal, 1=hv, gets added to channel number)

//***********************REG_ROC_* are the ROC registers
#define REG_ROC_RESET 0x10
#define REG_INVERT_CAL_SPI_CLK 0x11
#define REG_TIMERENABLE 0x12
#define REG_TIMERRESET 0x13
#define REG_TIMERCOUNTER 0x14

#define REG_ROC_DDR_PATTERN_EN 	0x20  // W enable pattern to DDR
#define REG_ROC_DDR_ERROR_READ	0x21  // R error seen:[0] event_error, [1] header1_error, [2] header2_error, [3] data_error
#define REG_ROC_DDR_ERROR_REQ 	0x22  // W request error to read:[0] event_error, [1] header1_error, [2] header2_error, [3] data_error
#define REG_ROC_DDR_ERROR_SEENL	0x23  // R 32 LBS of word read when first requested error seen
#define REG_ROC_DDR_ERROR_SEENM	0x24  // R 32 MBS of word read when first requested error seen
#define REG_ROC_DDR_ERROR_EXPCL	0x25  // R 32 LBS of word expected when first requested error seen
#define REG_ROC_DDR_ERROR_EXPCM	0x26  // R 32 MBS of word expected when first requested error seen
#define REG_ROC_DDR_SPILL_TAG	0x27  // R no of HB from start of SPILL
#define REG_ROC_DDR_NULLHB_CNT	0x28  // R no of null HB from start of SPILL
#define REG_ROC_DDR_HB_CNT		0x29  // R no of HeartBeat
#define REG_ROC_DDR_ONHOLD_CNT	0x2A  // R no of HeartBeat not processed
#define REG_ROC_DDR_PREF_CNT	0x2B  // R no of Prefetch seen
#define REG_ROC_DDR_DREQ_CNT	0x2C  // R no of Data Request received from DTC
#define REG_ROC_DDR_DREQRD_CNT	0x2D  // R no of Data Request read from memory
#define REG_ROC_DDR_DREQSENT	0x2E  // R no of Data Request sent to DTC
#define REG_ROC_DDR_DREQNULL	0x2F  // R no of null size Data Request sent to DTC

#define REG_ROC_DDR_SERIAL_SET 	0x30  // W enable serial commands to DDR
#define REG_ROC_DDR_HB_TAG 		0x31  // R [31:0] of last Heartbeat tag
#define REG_ROC_DDR_PRE_TAG 	0x32  // R [31:0] of last Prefetch tag
#define REG_ROC_DDR_FETCH_TAG 	0x33  // R [31:0] of last Fetched tag
#define REG_ROC_DDR_DREQ_TAG 	0x34  // R [31:0] of last Data Request tag
#define REG_ROC_DDR_OFFSET_TAG 	0x35  // R [31:0] of offset tag in present SPILL
#define REG_ROC_DDR_CFO_EN 		0x36  // W enable CFO emulation
#define REG_ROC_DDR_CFO_START	0x37  // W start CFO emulation
#define REG_ROC_DDR_CFO_OFFSET	0x38  // W HB tag offset for CFO emulator
#define REG_ROC_DDR_CFO_DELTAT	0x39  // W HB DeltaT for CFO emulator
#define REG_ROC_DDR_CFO_NUMBER	0x3A  // W number for CFO emulator
#define REG_ROC_DDR_PREF_EN 	0x3B  // W prefetch enable for CFO emulation
#define REG_ROC_DDR_SIZE_WR		0x3C  // R [16:0] number of size word written to DREQ_FIFO; [31] DREQ_FIFO_FULL
#define REG_ROC_DDR_SIZE_RD		0x3D  // R [16:0] number of size word reead from DREQ_FIFO; [31] DREQ_FIFO_EMPTY
#define REG_ROC_DDR_CTRLREADY 	0x3E

#define REG_ROC_FIFO_RE 0x40
#define REG_ROC_FIFO_DATA 0x41
#define REG_ROC_FIFO_FULL 0x42
#define REG_ROC_FIFO_EMPTY 0x43
#define REG_ROC_FIFO_RESET 0x44
#define REG_ROC_FIFO_RDCNT 0x45
#define REG_ROC_FIFO_HOWMANY 0x46
#define REG_ROC_SERDES_ALIGN 0x47
#define REG_ROC_USE_LANE 0x48

#define REG_ROC_DTC_SIM_EN 0x50    //W  enable DTC Simulated signals to CORE_PCS (in loopback)																							  
#define REG_ROC_DTC_SIM_START 0x51
#define REG_ROC_DTC_SIM_PARAM 0x52
#define REG_ROC_DTC_SIM_ADDR  0x53
#define REG_ROC_DTC_SIM_DATA  0x54
#define REG_ROC_DTC_SIM_SPILL 0x55
#define REG_ROC_DTC_SIM_BLK_EN   0x56
#define REG_ROC_DTC_SIM_BLK_DATA 0x57
#define REG_ROC_DTC_SIM_BLK_ADDR 0x58
#define REG_ROC_DTC_SIM_DATA_READ 0x59

#define REG_ROC_CAL_BUSY_P 0x61
#define REG_ROC_CAL_DATA_P 0x63
#define REG_ROC_CAL_ADDRESS_P 0x62
#define REG_ROC_CAL_INIT_P 0x60

#define REG_ROC_HV_BUSY_P 0x71
#define REG_ROC_HV_DATA_P 0x73
#define REG_ROC_HV_ADDRESS_P 0x72
#define REG_ROC_HV_INIT_P 0x70

#define REG_ROC_BOTH_INIT_P 0x76

#define REG_ROC_EWM_SINGLE 0x80 //(send a single event window marker)
#define REG_ROC_EWW_PULSER 0x81 //(enable EWM pulser)
#define REG_ROC_EWM_T 0x82 // (ewm period in 40MHz clock ticks) 
#define REG_ROC_EWMSTART_PMT 0x84 // (event window start  for PMT)
#define REG_ROC_EWMSTOP_PMT 0x85 // (event window stop for PMT)

#define REG_ROC_TVS_VAL 0x90
#define REG_ROC_TVS_ADDR 0x91

#define REG_ROC_CR_FIFO_RESET 0xA3

#define REG_ROC_ENABLE_FIBER_CLOCK 0xB0
#define REG_ROC_ENABLE_FIBER_MARKER 0xB1
#define REG_ROC_DTC_ENABLE_RESET 0xB4
#define REG_ROC_READ_ALIGNED 0xB6
#define REG_ROC_READ_ALIGNMENT 0xB7

#define PANELNVMADDRESS 0x0


//#define REG_ROC_DDR_TRUEL	0xC0 	//R  LSB 32-bit of DDR read data when error seen
//#define REG_ROC_DDR_TRUEH	0xC1 	//R  MSB 32-bit of DDR read data when error seen
//#define REG_ROC_DDR_EXPCL	0xC2 	//R  LSB 32-bit of DDR expected data when error seen
//#define REG_ROC_DDR_EXPCH	0xC3 	//R  MSB 32-bit of DDR expected data when error seen
//#define REG_ROC_DDR_BURST	0xC4 	//R  DDR BURST size (ex:0x3 = 256 bits, 0x7F = 1kB)
//#define REG_ROC_DDR_RWEN	0xC5	//W  enable simultaneous write & read of pattern generator to memory - self clearing
//#define REG_ROC_DDR_ERR_REN 0xC6	//W  enable read of DDR error FIFOs - self clearing
//#define REG_ROC_DDR_WRT_REN 0xC7	//W  enable read of DDR WRITE timer FIFO - self clearing
//#define REG_ROC_DDR_RDT_REN 0xC8	//W  enable read of DDR READ timer FIFO - self clearing
//#define REG_ROC_DDR_FIFODIA 0xC9	//R  DDR timer and error FIFO diagnostic:
//#define REG_ROC_DDR_WRTIME  0xCA	//R  32-bit timer for DDR writes
//#define REG_ROC_DDR_RDTIME  0xCB	//R  32-bit timer for DDR read
//#define REG_ROC_DDR_ERRCNT  0xCC 	//R  32-bit DDR error counter
//#define REG_ROC_DDR_LOCRAM  0xCD	//R  32-bit offset to signal first DDR address to write to TPSRAM
//#define REG_ROC_DDR_WRBCNT 	0xCE	//R  returns number of WR-data burst to DDR
//#define REG_ROC_DDR_RDBCNT 	0xCF	//R  returns number of RD-data burst to DDR

#define REG_ROC_PRBS_EN		    0xD0 //W: enable PRBS to CORE_PCS (in loopback)
#define REG_ROC_PRBS_START		0xD1 //W: PRBS sequence enable (level: high between START and STOP)
#define REG_ROC_PRBS_ERRORIN	0xD2 //W: Inject PRBS sequence error (0xEFFE)  (self clearing)
#define REG_ROC_PRBS_ERRORCLR	0xD3 //W: Reset error count and CDF FIFO (self clearing)
#define REG_ROC_PRBS_CDCRE		0xD4 //W: CDC FIFO Read Enable
#define REG_ROC_PRBS_CDCOUT		0xD5 //R: CDC FIFO OUT Data [31:16] received sequence; [15:0] expected sequence
#define REG_ROC_PRBS_CDCFULL	0xD6 //R: CDC FIFO Full
#define REG_ROC_PRBS_CDCEMPTY	0xD7 //R: CDC FIFO Empty
#define REG_ROC_PRBS_CDCWRCNT	0xD8 //R: CDC FIFO WR word counter
#define REG_ROC_PRBS_ERRORCNT	0xD9 //R: PRBS sequence error counter
#define REG_ROC_PRBS_ON		    0xDA //R: PRBS sequence ON
#define REG_ROC_PRBS_LOCK		0xDB //R: PRBS fiber is locked
#define REG_ROC_PRBS_PCSDATA	0xDC //R: PCS Diagnostics: [1:0]=TX invalid K; [3]=TX aligned [5:4]=RX error; [9:8]=RX bad disparity; [13:12]=RX bad 8to10

#define REG_ERROR_ADDRESS   0xE0
#define REG_ERROR_COUNTER   0xE1

#define REG_ROC_DDRTEST_WREN     0xE2
#define REG_ROC_DDRTEST_RDEN     0xE3
#define REG_ROC_DDRTEST_BLKNO    0xE4
#define REG_ROC_DDRTEST_STATUS   0xE5
#define REG_ROC_DDRTEST_ERRCNT   0xE6
#define REG_ROC_DDRTEST_ERRLOC   0xE7
#define REG_ROC_DDRTEST_EN       0xE8

#define REG_ROC_USE_UART    0xF1
#define REG_DIGIRW_SEL      0xF2   // WO   select between fiber (0) or serial (1) to drive CLK and MARKERS to DIGIs

//LEAK SENSOR REGS
#define REG_ROC_LEAK_MUX    0xF3
#define REG_ROC_LEAK_SDIR    0xF4
#define REG_ROC_LEAK_SCL    0xF5
#define REG_ROC_LEAK_SDA    0xF6

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
#define ADC_ADDR_SRCTRL 0x21			// x21: serial control: LSBFIRST | X | X | X | SPEED | NBITS[2:0]
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

// MT added for DTS commands
#define CMDHEADER           0xAABB  // start of DTC command to/from processor
#define CMDTRAILER          0xFFEE  // end of DTC command to/from processor
#define CMDERROR            0xDEAD  // DTC command error
#define MAX_CMD_LENGTH      0x1FFB  // max. DTC Write command length in units of 16-bit payload
#define MAX_BUFFER_SIZE     0x400   // max. DTC Write command length in units of 16-bit payload


//******************************************************************************
//                          Struct & Global Variables
//******************************************************************************

extern uint8_t hvcal;

extern uint16_t default_gains_cal[96];
extern uint16_t default_gains_hv[96];
extern uint16_t default_threshs_cal[96];
extern uint16_t default_threshs_hv[96];

volatile uint32_t channel_mask[3];
volatile uint32_t mapped_channel_mask[3];
extern uint32_t thischanmask;

extern uint8_t channel_map[96];
extern uint8_t adc_map[12];
extern uint8_t adcclk_map[12];
extern uint8_t adc_phases[12];

extern uint16_t bufcount;
extern uint16_t bufcount_place_holder;
extern uint16_t readout_obloc;
extern uint16_t readout_obloc_place_holder;
extern uint16_t fibercount;
extern uint16_t fibercount_place_holder;

extern char init_buff[30];
extern char dataBuffer[4096];
extern char outBuffer[2000]; // buffer for printing to serial port
//extern char fiberBuffer[1024];  // buffer for 16-bit words to DTC
extern uint16_t fiberBuffer[1024];  // buffer for 16-bit words to DTC
                                // same size as DCS_TX/RX_FIFOs in firmare
extern uint8_t rx_buff[100];
extern uint8_t buffer[256]; // buffer for reading from serial port
extern uint32_t writePtr;

volatile uint32_t * registers_0_addr;
// MT added for DCS commands
volatile uint32_t * registers_1_addr;
// MT added for RS485 commands
volatile uint32_t * registers_2_addr;

extern int readout_maxDelay;
extern int readout_mode;
extern int readout_wordInTrigger;
extern int readout_wordsPerTrigger;
extern int readout_numTriggers;
extern int readout_noUARTflag;
extern int calibration_count[32];
extern uint32_t calibration_done;

extern UART_instance_t g_uart;
extern spi_instance_t g_spi[5];
extern gpio_instance_t g_gpio;
extern pwm_instance_t g_pwm;


extern spi_instance_t g_cal_pro_spi;
extern spi_instance_t g_hv_pro_spi;
extern spi_instance_t g_pro_spi;

//******************************************************************************
//                             Functions
//******************************************************************************

void GPIO_write(uint8_t pin, uint8_t value);
uint32_t GPIO_read(uint8_t pin);

//void reset_fabric();

uint16_t readU16fromBytes(uint8_t data[]);
uint32_t readU32fromBytes(uint8_t data[]);
void delayUs(int us);
void delay_ms(uint32_t ms);
//void delayTicks(uint8_t ticks);
void hwdelay (uint32_t tdelay);
//void delayCore(uint32_t cycles);
//char * print_float(char *fchars, float value);

uint32_t DCS_pass_sim_param(uint8_t dtc_sim_en, uint8_t dtc_output, uint8_t dtc_opcode_or_seq_num, uint8_t dtc_packet_type_or_marker_type);
uint32_t DCS_pass_addr_data(uint16_t lsb, uint16_t msb, uint8_t is_data);
void DCS_sim_packet_send();

void read_data(int *delay_count, int *trigger_count);
//void read_data2(int *delay_count, int *trigger_count, uint16_t *lasthit);
// 1/24/2024 MT changed to used rates outputs for fiber READRATES function
//uint32_t get_rates(int num_delays, int num_samples, uint8_t channel, uint32_t* timecounts);
uint32_t get_rates(int num_delays, int num_samples, uint8_t channel, uint32_t* timecounts, uint32_t* total_hv, uint32_t* total_cal, uint32_t* total_coinc, uint32_t* total_time_counts);
void get_mapped_channels();
void read_histogram(uint8_t channel, uint8_t hv_or_cal, uint16_t *output);

void adc_spi(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint16_t adc_mask_f);
void adc_spi_core(uint8_t rw, uint8_t bytes, uint16_t address, uint8_t *data, uint8_t adc_mask_h, uint8_t hvcal);
void adc_write(uint16_t address, uint8_t data, uint16_t adc_mask_f);
uint8_t adc_read(uint16_t address, uint8_t adc_num);
uint16_t init_adc(uint16_t adc_mask, uint8_t pattern, uint8_t phase);
void init_DIGIs();

void digi_write(uint8_t address, uint16_t data, uint8_t hvcal);
uint16_t digi_read(uint8_t address, uint8_t hvcal);
void bufWrite(char *outBuffer, uint16_t *bufcount, uint32_t data, uint16_t nbytes);
//void bufWriteN(char *outBuffer, uint16_t bufaddr, uint32_t data, uint16_t nbytes);
void outBufSend(UART_instance_t g_uart, char *outBuffer, uint16_t bufcount);
int resetFIFO();
void setPreampGain(uint16_t channel, uint16_t value);
void setPreampThreshold(uint16_t channel, uint16_t value);
//void findChThreshold(int num_delays, int num_samples, uint16_t channel, uint16_t target_rate, uint8_t verbose);

// added by Monica
void mask_channels(uint8_t channel);

#endif /* UTILS_H_ */

