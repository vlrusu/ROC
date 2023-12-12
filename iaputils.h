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
void write_flash_directory();
void clear_iap_data_buffer(void);

void copy_to_flash(uint8_t * g_buffer,uint32_t length);
static uint32_t read_page_from_host_through_uart(uint8_t * g_buffer,uint32_t length);
uint32_t number_size(uint8_t *ptr);
void load_spi_flash_with_images_thruough_uart_intf();
void execute_usercode_service(void);
void execute_designinfo_service(void);
void execute_iap(uint8_t option);
void execute_bitstream_authenticate(void);
void execute_iap_image_authenticate(void);










#endif /* IAPUTILS_H_ */
