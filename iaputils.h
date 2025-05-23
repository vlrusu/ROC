/*
 * iaputils.h
 *
 *  Created on: Dec 11, 2023
 *      Author: vrusu
 */

#ifndef IAPUTILS_H_
#define IAPUTILS_H_

#include <stdio.h>
#include <string.h>
#include "drivers/mt25ql01gbbb/micron1gflash.h"

/******************************************************************************
 * Maximum buffer size.
 *****************************************************************************/
#define MAX_RX_DATA_SIZE    256
#define MASTER_TX_BUFFER    10
#define DATA_LENGTH_32_BYTES  32

/*==============================================================================
  Macro
 */
#define   VALID                   0U
#define   INVALID                 1U
#define   ENTER                   13u

/******************************************************************************
 * CoreUARTapb instance data.
 *****************************************************************************/

uint16_t get_input_data
(
    uint8_t* location,
    uint16_t size,
    const uint8_t* msg,
    uint16_t msg_size
);
void get_key
(
    uint8_t key_type,
    uint8_t* location,
    uint8_t size,
    const uint8_t* msg,
    uint8_t msg_size
);
uint16_t get_data_from_uart
(
    uint8_t* src_ptr,
    uint16_t size,
    const uint8_t* msg,
    uint16_t msg_size
);

void display_output
(
    uint8_t* in_buffer,
    uint32_t byte_length
);

void delay1(volatile uint32_t n);
void write_flash_directory(uint16_t nImages,uint32_t* addresses);

void clear_iap_data_buffer(void);

void copy_to_flash(uint8_t * g_buffer);
static uint32_t read_page_from_host_through_uart(uint8_t * g_buffer);
uint32_t number_size(uint8_t *ptr);
void execute_usercode_service(void);
void execute_designinfo_service(void);
void execute_iap(uint8_t option, uint32_t index);
void execute_bitstream_authenticate(uint32_t img_index);
void execute_iap_image_authenticate(uint32_t img_index);
void list_flash_dir(uint32_t start, uint32_t length);
void load_spi_flash_at_address(uint32_t index_address);
void programming_done(void);


// MT add
void list_flash_dir_fiber(uint32_t start_addr, uint32_t length, uint16_t* flash_read_buf);
void write_flash_directory_fiber(uint16_t nImages, uint32_t* addresses, uint16_t g16_fail);
void load_spi_flash_at_address_fiber(uint32_t index_address, uint8_t* flash_write_buf, uint16_t fail_mask);


#endif /* IAPUTILS_H_ */
