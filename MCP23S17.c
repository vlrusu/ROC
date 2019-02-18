/*
  MCP23S17.cpp  Version 0.1
  Microchip MCP23S17 SPI I/O Expander Class for Arduino
  Created by Cort Buffington & Keith Neufeld
  March, 2011

  Features Implemented (by word and bit):
    I/O Direction
    Pull-up on/off
    Input inversion
    Output write
    Input read

  Interrupt features are not implemented in this version
  byte based (portA, portB) functions are not implemented in this version

  NOTE:  Addresses below are only valid when IOCON.BANK=0 (register addressing mode)
         This means one of the control register values can change register addresses!
         The default values is 0, so that's how we're using it.

         All registers except ICON (0xA and 0xB) are paired as A/B for each 8-bit GPIO port.
         Comments identify the port's name, and notes on how it is used.

         *THIS CLASS ENABLES THE ADDRESS PINS ON ALL CHIPS ON THE BUS WHEN THE FIRST CHIP OBJECT IS INSTANTIATED!

  USAGE: All Read/Write functions except wordWrite are implemented in two different ways.
         Individual pin values are set by referencing "pin #" and On/Off, Input/Output or High/Low where
         portA represents pins 0-7 and portB 8-15. So to set the most significant bit of portB, set pin # 15.
         To Read/Write the values for the entire chip at once, a word mode is supported buy passing a
         single argument to the function as 0x(portB)(portA). I/O mode Output is represented by 0.
         The wordWrite function was to be used internally, but was made public for advanced users to have
         direct and more efficient control by writing a value to a specific register pair.
*/

#include "MCP23S17.h"            // Header files for this class
#include "utils.h"
#include "setup.h"

// Defines to keep logical information symbolic go here

#define    HIGH          (1)
#define    LOW           (0)
#define    ON            (1)
#define    OFF           (0)

// Here we have things for the SPI bus configuration

// Control byte and configuration register information - Control Byte: "0100 A2 A1 A0 R/W" -- W=0

#define    OPCODEW       (0b01000000)  // Opcode for MCP23S17 with LSB (bit0) set to write (0), address OR'd in later, bits 1-3
#define    OPCODER       (0b01000001)  // Opcode for MCP23S17 with LSB (bit0) set to read (1), address OR'd in later, bits 1-3
#define    ADDR_ENABLE   (0b00001000)  // Configuration register for MCP23S17, the only thing we change is enabling hardware addressing

// Constructor to instantiate an instance of MCP to a specific chip (address)


void MCP_setup(MCP *mcp, spi_instance_t spi, uint8_t ss, uint8_t address) {
	mcp->spi = spi;
	mcp->ss = ss;

	mcp->_address     = address;
	mcp->_modeCache   = 0xFFFF;                // Default I/O mode is all input, 0xFFFF
	mcp->_outputCache = 0x0000;                // Default output state is all off, 0x0000
	mcp->_pullupCache = 0x0000;                // Default pull-up state is all off, 0x0000
	mcp->_invertCache = 0x0000;                // Default input inversion state is not inverted, 0x0000

	MCP_byteWrite(mcp, IOCON, ADDR_ENABLE);

//  _modeCache = wordRead(IODIRA);
//  _outputCache = wordRead(GPIOA);
//  _pullupCache = wordRead(GPPUA);
//  _invertCache = wordRead(IPOLA);
};



void MCP_byteWrite(MCP *mcp, uint8_t reg, uint8_t value) {      // Accept the register and byte

	uint8_t master_tx_buffer[3];
	master_tx_buffer[0] = OPCODEW | (mcp->_address << 1);
	master_tx_buffer[1] = reg;
	master_tx_buffer[2] = value;
	SPI_set_slave_select(&(mcp->spi), mcp->ss);


    SPI_transfer_block
       (
           &(mcp->spi),
           master_tx_buffer,
           sizeof(master_tx_buffer),
           0,
           0);

	SPI_clear_slave_select(&(mcp->spi), mcp->ss);



}

// GENERIC WORD WRITE - will write two bytes to a register pair, LSB to first register, MSB to next higher value register

void MCP_wordWrite(MCP *mcp, uint8_t reg, unsigned int word) {  // Accept the start register and word

	uint8_t master_tx_buffer[4];
	master_tx_buffer[0] = OPCODEW | (mcp->_address << 1);
	master_tx_buffer[1] = reg;
	master_tx_buffer[2] = word & 0xFF;
	master_tx_buffer[3] = (word >> 8) & 0xFF;
	SPI_set_slave_select(&(mcp->spi), mcp->ss);


	   SPI_transfer_block
	       (
	           &(mcp->spi),
	           master_tx_buffer,
	           sizeof(master_tx_buffer),
	           0,
	           0);


	SPI_clear_slave_select(&(mcp->spi), mcp->ss);




}

// MODE SETTING FUNCTIONS - BY PIN AND BY WORD

void MCP_pinMode(MCP *mcp, uint8_t pin, uint8_t mode) {  // Accept the pin # and I/O mode
  if ((pin < 1) | (pin > 16)) return;               // If the pin value is not valid (1-16) return, do nothing and return
  if (mode == MCP_INPUT) {                          // Determine the mode before changing the bit state in the mode cache
    mcp->_modeCache |= 1 << (pin - 1);               // Since input = "HIGH", OR in a 1 in the appropriate place
  } else {
    mcp->_modeCache &= ~(1 << (pin - 1));            // If not, the mode must be output, so and in a 0 in the appropriate place
  }
  MCP_wordWrite(mcp, IODIRA, mcp->_modeCache);                // Call the generic word writer with start register and the mode cache
}

void MCP_pinModeAll(MCP *mcp, unsigned int mode) {    // Accept the word�
  MCP_wordWrite(mcp, IODIRA, mode);                // Call the the generic word writer with start register and the mode cache
  mcp->_modeCache = mode;
}

// THE FOLLOWING WRITE FUNCTIONS ARE NEARLY IDENTICAL TO THE FIRST AND ARE NOT INDIVIDUALLY COMMENTED

// WEAK PULL-UP SETTING FUNCTIONS - BY WORD AND BY PIN

void MCP_pullupMode(MCP *mcp, uint8_t pin, uint8_t mode) {
  if ((pin < 1) | (pin > 16)) return;
  if (mode == ON) {
    mcp->_pullupCache |= 1 << (pin - 1);
  } else {
    mcp->_pullupCache &= ~(1 << (pin -1));
  }
  MCP_wordWrite(mcp, GPPUA, mcp->_pullupCache);
}


void MCP_pullupModeAll(MCP *mcp, unsigned int mode) {
  MCP_wordWrite(mcp, GPPUA, mode);
  mcp->_pullupCache = mode;
}


// INPUT INVERSION SETTING FUNCTIONS - BY WORD AND BY PIN

void MCP_inputInvert(MCP *mcp, uint8_t pin, uint8_t mode) {
  if ((pin < 1) | (pin > 16)) return;
  if (mode == ON) {
    mcp->_invertCache |= 1 << (pin - 1);
  } else {
    mcp->_invertCache &= ~(1 << (pin - 1));
  }
  MCP_wordWrite(mcp, IPOLA, mcp->_invertCache);
}

void MCP_inputInvertAll(MCP *mcp, unsigned int mode) {
  MCP_wordWrite(mcp, IPOLA, mode);
  mcp->_invertCache = mode;
}


// WRITE FUNCTIONS - BY WORD AND BY PIN

void MCP_pinWrite(MCP *mcp, uint8_t pin, uint8_t value) {
  if ((pin < 1) | (pin > 16)) return;
  if (value) {
    mcp->_outputCache |= 1 << (pin - 1);
  } else {
    mcp->_outputCache &= ~(1 << (pin - 1));
  }
  MCP_wordWrite(mcp, GPIOA, mcp->_outputCache);
}

void MCP_pinWriteAll(MCP *mcp, unsigned int value) {
  MCP_wordWrite(mcp, GPIOA, value);
  mcp->_outputCache = value;
}


// READ FUNCTIONS - BY WORD, BYTE AND BY PIN

unsigned int MCP_pinReadAll(MCP *mcp) {       // This function will read all 16 bits of I/O, and return them as a word in the format 0x(portB)(portA)

	return MCP_wordRead(mcp,GPIOA);


}

unsigned int MCP_wordRead(MCP *mcp, uint8_t reg) {

	uint8_t master_tx_buffer[2];
	uint8_t rx_buffer[2];
	unsigned int value = 0x0;
	master_tx_buffer[0] = OPCODER | (mcp->_address << 1);
	master_tx_buffer[1] = reg;
	SPI_set_slave_select(&(mcp->spi), mcp->ss);


	   SPI_transfer_block
	       (
	           &(mcp->spi),
	           master_tx_buffer,
	           sizeof(master_tx_buffer),
	           rx_buffer,
	           sizeof(rx_buffer));



	SPI_clear_slave_select(&(mcp->spi), mcp->ss);

	value |= rx_buffer[0];

	value |= (rx_buffer[1] << 8);

	return value;
}

uint8_t MCP_byteRead(MCP *mcp, uint8_t reg) {

	uint8_t master_tx_buffer[2];
	uint8_t rx_buffer = 0;

	master_tx_buffer[0] = OPCODER | (mcp->_address << 1);
	master_tx_buffer[1] = reg;


	SPI_set_slave_select(&(mcp->spi), mcp->ss);


	   SPI_transfer_block
	       (
	           &(mcp->spi),
	           master_tx_buffer,
	           sizeof(master_tx_buffer),
	           &rx_buffer,
	           1);



	return rx_buffer;


}

uint8_t MCP_pinRead(MCP *mcp, uint8_t pin) {              // Return a single bit value, supply the necessary bit (1-16)
    if ((pin < 1) | (pin > 16)) return 0x0;                  // If the pin value is not valid (1-16) return, do nothing and return
    return MCP_pinReadAll(mcp) & (1 << (pin - 1)) ? HIGH : LOW;  // Call the word reading function, extract HIGH/LOW information from the requested pin
}
