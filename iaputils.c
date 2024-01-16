/*
 * iaputils.c
 *
 *  Created on: Dec 11, 2023
 *      Author: vrusu
 */

#include "iaputils.h"
#include "../utils.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreSysServices_PF/core_sysservices_pf.h"
#include "drivers/CoreSPI/core_spi.h"



#define MAX_FILES 3

#define BUFFER_SIZE 4096

#define IMAGE_IDX                               2u
#define GOLDEN_IMAGE_SPI_ADDRESS 0x400
#define UPDATE_IMAGE_SPI_ADDRESS 0xA00000
#define IAP_IMAGE_SPI_ADDRESS 0x1400000
//extern void delay(volatile int i);
//*&************************************************************
uint8_t g_write_buffer[BUFFER_SIZE];
uint8_t no_of_files =0;
uint32_t flash_address[3] = {GOLDEN_IMAGE_SPI_ADDRESS,UPDATE_IMAGE_SPI_ADDRESS,IAP_IMAGE_SPI_ADDRESS};
uint8_t g_read_buf[BUFFER_SIZE];
uint32_t g_src_image_target_address =0;
uint32_t g_file_size = 0;
uint32_t g_flash_address = 0;

uint8_t iap_data_buffer [1024];




static const uint8_t g_separator[] =
"\r\n\
------------------------------------------------------------------------------\r\n";


/*==============================================================================
  Function to clear local variable and array.
 */
static void clear_variable(uint8_t *p_var, uint16_t size)
{
    uint16_t inc;

    for(inc = 0; inc < size; inc++)
    {
        *p_var = 0x00;
        p_var++;
    }
}

/*==============================================================================
  Function to get the input data from user.
 */
uint16_t get_input_data
(
    uint8_t* location,
    uint16_t size,
    const uint8_t* msg,
    uint16_t msg_size
)
{
    uint16_t count = 0u;

    /* Clear the memory location. */
    clear_variable(location, size);

    /* Read data from UART terminal. */
    count = get_data_from_uart(location, size, msg, msg_size);

    return count;
}

/*==============================================================================
  Function to get the key from user.
 */
void get_key
(
    uint8_t key_type,
    uint8_t* location,
    uint8_t size,
    const uint8_t* msg,
    uint8_t msg_size
)
{
    uint8_t status = 0u;
    const uint8_t invalid_ms[] = "\r\n Invalid key type. ";

    if(status == VALID)
    {
        /* Read the 16 bytes of input data from UART terminal. */
        get_input_data(location, size, msg, msg_size);
    }
    else
    {
        UART_send(&g_uart, invalid_ms, sizeof( invalid_ms));
    }
}

/*==============================================================================
  Convert ASCII value to hex value.
 */
uint8_t convert_ascii_to_hex(uint8_t* dest, const uint8_t* src)
{
    uint8_t error_flag = 0u;

     if((*src >= '0') && (*src <= '9'))
    {
        *dest = (*src - '0');
    }
    else if((*src >= 'a') && (*src <= 'f'))
    {
        *dest = (*src - 'a') + 10u;
    }
    else if((*src >= 'A') && (*src <= 'F'))
    {
        *dest =  (*src - 'A') + 10u;
    }
    else if(*src != 0x00u)
    {
        UART_send(&g_uart, (const uint8_t *)"\r\n Invalid data.", sizeof("\r\n Invalid data."));
        error_flag = 1u;
    }
     return error_flag;
}

/*==============================================================================
  Validate the input hex value .
 */
uint8_t validate_input(uint8_t ascii_input)
{
    uint8_t valid_key = 0u;

    if(((ascii_input >= 'A') && (ascii_input <= 'F')) ||        \
       ((ascii_input >= 'a') && (ascii_input <= 'f')) ||        \
       ((ascii_input >= '0') && (ascii_input <= '9')))
    {
        valid_key = 1u;
    }
    else
    {
        valid_key = 0u;
    }
    return valid_key;
}

const uint8_t hex_chars[16] = { '0', '1', '2', '3', '4', '5', '6', '7',
                             '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

/*==============================================================================
  Display content of buffer passed as parameter as hex values.
 */
void display_output
(
    uint8_t* in_buffer,
    uint32_t byte_length
)
{
    uint32_t inc;
    uint8_t byte = 0;

    UART_send(&g_uart, (const uint8_t*)" ", sizeof(" "));
    for(inc = 0; inc < byte_length; ++inc)
    {
        if((inc > 1u) &&(0u == (inc % 16u)))
        {
            UART_send(&g_uart, (const uint8_t*)"\r\n ", sizeof("\r\n "));
        }

        byte = in_buffer[inc];
        UART_send(&g_uart, &hex_chars[((byte & 0xF0) >> 4) ], 1);
        UART_send(&g_uart, &hex_chars[(byte & 0x0F)], 1);
    }

}

/*==============================================================================
  Function to read data from UART terminal and stored it.
 */


uint16_t get_data_from_uart
(
    uint8_t* src_ptr,
    uint16_t size,
    const uint8_t* msg,
    uint16_t msg_size
)
{
    uint8_t complete = 0u;
    uint8_t rx_buff[1];
    uint8_t rx_size = 0u;
    uint16_t count = 0u;
    uint16_t ret_size = 0u;
    uint8_t first = 0u;
    uint16_t src_ind = 0u;
    uint8_t prev = 0;
    uint8_t curr = 0;
    uint8_t temp = 0;
    uint8_t next_byte = 0;
    uint16_t read_data_size = 0;

    UART_send(&g_uart, g_separator, sizeof(g_separator));
    UART_send(&g_uart, msg, msg_size);

    read_data_size = size * 2;

    /* Read the key size sent by user and store it. */
    count = 0u;
    while(!complete)
    {
        rx_size = UART_get_rx(&g_uart, rx_buff, sizeof(rx_buff));
        if(rx_size > 0u)
        {
            /* Is it to terminate from the loop */
            if(ENTER == rx_buff[0])
            {
                complete = 1u;
            }
            /* Is entered key valid */
            else if(validate_input(rx_buff[0]) != 1u)
            {
                UART_send(&g_uart, rx_buff, sizeof(rx_buff));
                UART_send(&g_uart, (const uint8_t *)"\r\n Invalid input.",
                          sizeof("\r\n Invalid input."));
                UART_send(&g_uart, msg, msg_size);
                complete = 0u;
                count = 0u;
                first = 0u;
            }
            else
            {
                if(next_byte == 0)
                {
                    convert_ascii_to_hex(&src_ptr[src_ind], &rx_buff[0]);
                    prev = src_ptr[src_ind];
                    next_byte = 1;
                }
                else
                {
                    convert_ascii_to_hex(&curr, &rx_buff[0]);
                    temp = ((prev << 4) & 0xF0);
                    src_ptr[src_ind] = (temp | curr);
                    next_byte = 0;
                    src_ind++;
                }


                /* Switching to next line after every 8 bytes */
                if(((count % 32u) == 0x00u) && (count > 0x00u) && (complete != 0x01u))
                {
                    UART_send(&g_uart, (const uint8_t *)"\n\r", sizeof("\n\r"));
                    first = 0u;
                }

                if(first == 0u)
                {
                    UART_send(&g_uart, (const uint8_t *)" ", sizeof(" "));
                    first++;
                }
                UART_send(&g_uart, rx_buff, sizeof(rx_buff));
                count++;
                if(read_data_size == count)
                {
                   complete = 1u;
                }
            }
        }
    }

    if((count%2) == 0)
    {
        ret_size = count/2;
    }
    else
    {
        if(size != 1)
        {
            temp = src_ptr[src_ind];
            src_ptr[src_ind] = ((temp << 4) & 0xF0);

            ret_size = (count/2)+1;
        }
        else
        {
            ret_size = 1;
        }
    }

    return ret_size;
}



void clear_iap_data_buffer(void)
{
    uint32_t idx=0;

    while( idx < 1024 )
    {
        iap_data_buffer[idx++] = 0x00;
    }
}

void delay1(volatile uint32_t n)
{
    while(n)
        n--;
}
void write_flash_directory()
{
    uint32_t address1 = 0x400,address2 = 0xA00000,address3 = 0x1400000;
    uint8_t buf[4];
    buf[3] = (uint8_t)((address1 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address1 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address1 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address1 & 0xFF);
    FLASH_program(0 , buf, 4);
    FLASH_read(0,g_read_buf,4);
    buf[3] = (uint8_t)((address2 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address2 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address2 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address2 & 0xFF);
    FLASH_program(4 , buf, 4);
    FLASH_read(4,g_read_buf,4);
    buf[3] = (uint8_t)((address3 >> 24) & 0xFF);
    buf[2] = (uint8_t)((address3 >> 16) & 0xFF);
    buf[1] = (uint8_t)((address3 >> 8) & 0xFF);
    buf[0] = (uint8_t)(address3 & 0xFF);
    FLASH_program(8 , buf, 4);
    FLASH_read(8,g_read_buf,4);
}
void copy_to_flash(uint8_t * g_buffer,uint32_t length)
{
    uint32_t i=0;
    for(i=0;i<8;i++)
    {
        FLASH_program(g_flash_address+i*512 , &g_buffer[i*512], 512);
    }
    g_flash_address = g_flash_address + BUFFER_SIZE;
}

static uint32_t read_page_from_host_through_uart
(
        uint8_t * g_buffer,
        uint32_t length
)
{
    uint32_t num_bytes,factor,temp;
    volatile uint32_t i = 0;
    num_bytes = length;
    char crc;
    size_t rx_size = 0;


    uint8_t rx_buff[1],temp_add[2];
    //Write Ack "b" to indicate beginning of the transaction from the target

    if(g_src_image_target_address + length > g_file_size )
    {
        num_bytes = g_file_size - g_src_image_target_address;
    }
    if(g_src_image_target_address>= g_file_size)
    {
        return 0;
    }
    CRCFAIL:


    UART_send(&g_uart, (const uint8_t * )"b",1);
    //poll for Ack message from the host as an acknowledgment that the host is ready for receiving the transaction

    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
        ;
    //for(i=0;i<2500;i++);
    //transmit the address to the host
#if 1
    temp = g_src_image_target_address/4096;
    temp_add[0] = temp&0xFF;
    temp_add[1] = (temp>>8)&0xFF;
    if(rx_buff[0]== 'a')
    {
        UART_send(&g_uart,&temp_add[0],1);
        for(i=0;i<500;i++);
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;

        if(rx_buff[0]== 'a')
            UART_send(&g_uart,&temp_add[1],1);
        for(i=0;i<500;i++);
        //UART_send(&g_uart, (const uint8_t * )&temp, 4 );
        //UART_polled_tx_string(&g_uart, (const uint8_t * )&temp);
    }

    //poll for Ack message from the host as an acknowledgment that the host received the address
    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
        ;
#endif
    //transmit the returnbytes to the host
    temp_add[0] = num_bytes&0xFF;
    temp_add[1] = (num_bytes>>8)&0xFF;
    if(rx_buff[0]== 'a')
    {
        //UART_send(&g_uart, (const uint8_t * )&num_bytes, 4 );
        //UART_polled_tx_string(&g_uart,(const uint8_t * )&num_bytes);
        UART_send(&g_uart,&temp_add[0],1);
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        for(i=0;i<500;i++);
        if(rx_buff[0]== 'a')
            UART_send(&g_uart,&temp_add[1],1);
        for(i=0;i<500;i++);
    }

    //poll for Ack message from the host as an acknowledgment that the host received the returnbytes
    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
        ;
    /*if(rx_buff[0]== 'f')
    {
        UART_send(&g_uart, (const uint8_t * )&num_bytes, 4 ); // sending the num of bytes again... as previous failed
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
                ;
    }*/
    //for(i=0;i<2000;i++);
    //read the data from the host for the request number of bytes
#if 1
    if(rx_buff[0]== 'n')
    {
        for(i=0;i<num_bytes;i++)
        {
            rx_buff[0] = 0;
            while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
                ;
            g_buffer[i] = rx_buff[0];

        }
        rx_size = num_bytes;
    }
#endif

    //send Ack message to indicate one transaction is done
    UART_send(&g_uart, (const uint8_t * )"a",1);
    //Recive 1-byte CRC for data of size num_bytes
    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
        ;
    factor = 1;
    crc = 0;
    while((num_bytes-1)/factor)
    {
        crc = crc^g_buffer[factor];
        factor = factor*2;
    }
    if(crc == (char)rx_buff[0])
    {
        g_src_image_target_address += rx_size;

        UART_send(&g_uart, (const uint8_t * )"a",1);
    }
    else
    {
        UART_send(&g_uart,(const uint8_t * )"n",1);
        goto CRCFAIL;
    }

    return rx_size;
}

uint32_t number_size(uint8_t *ptr)
{
    uint32_t temp = 0,i=0;
    while(*ptr != '\0' && i<9)
    {
        temp = temp*10+*ptr -'0';
        ptr++;
        i++;
    }
    return temp;
}
void load_spi_flash_with_images_thruough_uart_intf()
{
    volatile uint32_t erase_address = 0;
    volatile uint32_t erase_count=0;
    volatile uint32_t i = 0,length = 0;


    uint8_t rx_buff[8],num[9];
    uint8_t  manufacturer_id;
    uint8_t  device_id,led_state = 0;


    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
        ;
    if(rx_buff[0] == 's') //signal from host PC to proceed to erase the flash.
        UART_send(&g_uart, (const uint8_t * )"a",1);

    FLASH_init();

    FLASH_global_unprotect();
    FLASH_read_device_id
    (
            &manufacturer_id,
            &device_id
    );
    //UART_init( &g_uart, UART_BASE_ADDRESS, 59, (DATA_8_BITS | NO_PARITY) );


    erase_address = erase_count = 0;
    for(erase_count = 0;erase_count<=640;erase_count++)  //40MB erase 640
    {

        FLASH_erase_64k_block(erase_address);
        delay1(500);
        FLASH_read(erase_address,g_read_buf,32);
        erase_address+=0x10000;
        if(g_read_buf[0] != 0xFF)
        {

            break;
        }
        led_state = led_state ^ 1;
    }
//    if(erase_count == 641)
//    {
//        GPIO_set_output( &g_gpio_out, GPIO_0,1); //erase successful
//    }
//    else
//    {
//        GPIO_set_output( &g_gpio_out, GPIO_0,0); //erase failed
//    }
    write_flash_directory();

#if 1
    while(no_of_files<MAX_FILES)
    {

        g_flash_address = flash_address[no_of_files];
        g_src_image_target_address = 0;
        /* start the handshake with the host */
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        if(rx_buff[0] == 'h')
            UART_send(&g_uart, (const uint8_t * )"a",1);
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        if(rx_buff[0] == 'n')
            UART_send(&g_uart, (const uint8_t * )"d", 1 );
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        if(rx_buff[0] == 's')
            UART_send(&g_uart, (const uint8_t * )"h", 1 );
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        if(rx_buff[0] == 'a')
            UART_send(&g_uart, (const uint8_t * )"k", 1 );
        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        if(rx_buff[0] == 'e')
        {
            UART_send(&g_uart, (const uint8_t * )"r", 1 );
        }
        /* poll for starting Ack message from the host as an acknowledgment
                           that the host is ready to send file size */

        while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
            ;
        UART_send(&g_uart, (const uint8_t * )"a",1);


        /*poll for file size*/
        UART_send(&g_uart, (const uint8_t * )"z",1);
        i=0;
        while(i<9)
        {
            while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
                ;

            num[i] = rx_buff[0];
            i++;
        }
        g_file_size = number_size(num);//;atoi((const char*)rx_buff);

        UART_send(&g_uart, (const uint8_t * )"a",1);

        do
        {
            length = read_page_from_host_through_uart(g_write_buffer, BUFFER_SIZE);
            if(length>0)
            {
                copy_to_flash(g_write_buffer,length);
                //memcpy(ddr_add1,g_page_buffer,length);
                //ddr_add1 +=  length;
            }
            led_state = led_state ^ 1;

        }while(length!=0);
        //ddr_add += 0x30000;

        no_of_files++;
    }
//    if(no_of_files==MAX_FILES)
//    {
//        GPIO_set_output( &g_gpio_out, GPIO_1,1);
//    }
#endif
}

#if 0 //will be supported  Rev E silicon onwards
void execute_digest_check(void)
{
    uint32_t options = 0x4;
    uint32_t mb_offset = 0;
    uint8_t uart_input[8]={0};
    uint16_t ret_val = 0;
    uint32_t* ptr = 0;
    uint32_t count=0;

    UART_polled_tx_string(&g_uart,(const uint8_t*)"\r\nDigest check for sNVM is in progress...");

    ret_val = SYS_digest_check_service(options, mb_offset);

    if(0 == ret_val)
    {
        UART_polled_tx_string(&g_uart,(const uint8_t*)"\r\nDigest check Success.");
    }
    else
    {
        UART_polled_tx_string(&g_uart,(const uint8_t*)"\r\nDigest check return value: ");
        display_output((uint8_t*)&ret_val, 1);
    }
    UART_polled_tx_string(&g_uart,(const uint8_t*)"\r\n\r\n");
}
#endif


void execute_usercode_service(void)
{
    uint8_t status,code[4];

    UART_polled_tx_string(&g_uart, (const uint8_t*)"32bit USERCODE/Silicon Signature (MSB first): ");
    status = SYS_get_user_code(iap_data_buffer, 0);
    code[0] = iap_data_buffer[3];
    code[1] = iap_data_buffer[2];
    code[2] = iap_data_buffer[1];
    code[3] = iap_data_buffer[0];
    if(SYS_SUCCESS == status)
    {
        display_output(code, USERCODE_RESP_LEN);
    }
    else
    {
        UART_polled_tx_string(&g_uart,(const uint8_t*)"USERCODE Service failed.\n\r");

    }

    UART_polled_tx_string(&g_uart, (const uint8_t*)"\n\r");
}

void execute_designinfo_service(void)
{
    uint8_t status;
    status = SYS_get_design_info(iap_data_buffer, 0);
    if(SYS_SUCCESS == status)
    {

        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nDesign Version(MSB first): ");
        display_output((iap_data_buffer + 33), 1);
        display_output((iap_data_buffer + 32), 1);

        UART_polled_tx_string(&g_uart,(const uint8_t*)"\r\n");
    }
    else
    {
        UART_polled_tx_string(&g_uart,(const uint8_t*)"DesignInfo Service failed.\r\n");
    }


}



uint8_t iap_data_buffer [1024];

/*==============================================================================
  Display greeting message when application is started.
 */



void execute_iap(uint8_t option)
{
    uint8_t cmd = 0;
    uint32_t spiaddr_or_idx = 0;
    uint8_t status = 0xFF;
    switch(option)
    {
    case 4:
        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nIAP PROGRAM for image at index 2 is in progress...\n\rIt takes approximately 28 seconds\n\r");

        cmd = IAP_PROGRAM_BY_SPIIDX_CMD;
        spiaddr_or_idx = IMAGE_IDX;
        break;
    case 5:
        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nIAP PROGRAM for image at address 0x1400000 is in progress...\n\rIt takes approximately 28 seconds\n\r");
        cmd = IAP_PROGRAM_BY_SPIADDR_CMD;
        spiaddr_or_idx = IAP_IMAGE_SPI_ADDRESS;
        break;
    case 6:
        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nAuto update is in progress...\r\nIt takes approximately 28 seconds\n\r");

        cmd = IAP_AUTOUPDATE_CMD;
        break;

    default: UART_polled_tx_string(&g_uart,(const uint8_t*)
            "Invalid input.\r\n");
    }


    status = SYS_iap_service(cmd, spiaddr_or_idx);
    UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nStatus code returned: ");
    display_output(&status, 1);
    UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\n");
}

void execute_bitstream_authenticate(void)
{

    uint32_t mb_offset = 0;
    uint8_t status = 0xFF;
    UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nBitstream authentication for image at address 0x1400000 is in progress...\n\r");
    status = SYS_bitstream_authenticate_service(IAP_IMAGE_SPI_ADDRESS, mb_offset);
    UART_polled_tx_string(&g_uart, (const uint8_t*)"Authentication status: ");
    if(SYS_SUCCESS == status)
    {
        UART_polled_tx_string(&g_uart, (const uint8_t*)"SUCCESS ");
    }
    else
    {
        display_output(&status, 1);
    }
    UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\n");
}

void execute_iap_image_authenticate(void)
{

    uint8_t status = 0xFF;


    UART_polled_tx_string(&g_uart, "\r\nIAP image authentication for image at index 2 is in progress...\n\r");
    status = SYS_IAP_image_authenticate_service(IMAGE_IDX);
    UART_polled_tx_string(&g_uart, "Authentication status: ");
    if(SYS_SUCCESS == status)
    {
        UART_polled_tx_string(&g_uart,"SUCCESS ");
    }
    else
    {
        display_output(&status, 1);
    }
    UART_polled_tx_string(&g_uart, "\r\nProgDone\n");

}
