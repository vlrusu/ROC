/*
 * SPI_daisy.h
 *
 *  Created on: Jul 24, 2020
 *      Author: yongyiwu
 */

// Coded for CPOL = 1 (SCLK IDLE at 1),
//           CPHA = 1 (data match trailing edge of clock)

#ifndef SPI_DAISY_H_
#define SPI_DAISY_H_

#include "MCP23S17.h"

#define SPI_DAISY_HALF_CLK_DELAY 2

typedef struct{
    MCP *_mcp_scl; // SPI expander for clock line
    MCP *_mcp_sdi; // SPI expander for data-in line
    MCP *_mcp_sdo; // SPI expander for data-out line
    MCP *_mcp_cs;  // SPI expander for chip select line
    uint8_t _sclPin; // pin number on _mcp_scl for scl pin
    uint8_t _sdiPin; // pin number on _mcp_sdi for sdi pin
    uint8_t _sdoPin; // pin number on _mcp_sdo for sdo pin
    uint8_t _csPin;  // pin number on _mcp_cs for cs pin
} SPI_daisy;

// sets mcp and pins, and configure pins to an initial state
void SPI_daisy_setup(SPI_daisy *self, MCP *mcp_scl, uint8_t sclPin, MCP *mcp_sdi, uint8_t sdiPin, MCP *mcp_sdo, uint8_t sdoPin, MCP *mcp_cs, uint8_t csPin);
void SPI_daisy_read(SPI_daisy *self, uint8_t nbyte, uint8_t *data); //maximal 32 byte at once
void SPI_daisy_write(SPI_daisy *self, uint8_t nbyte, uint8_t *data); //maximal 4 byte
void SPI_daisy_rw_cycle(SPI_daisy *self, uint8_t wbyte, uint8_t *wdata, uint8_t rbyte, uint8_t *rdata);

#endif /* SPI_DAISY_H_ */
