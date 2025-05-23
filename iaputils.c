/*
 * iaputils.c
 *
 *  Created on: Dec 11, 2023
 *      Author: vrusu
 */

#include "iaputils.h"
#include "../utils.h"
#include "dputil.h"
#include "drivers/CoreUARTapb/core_uart_apb.h"
#include "drivers/CoreSysServices_PF/core_sysservices_pf.h"
#include "drivers/CoreSPI/core_spi.h"



#define MAX_FILES 3
#define MAXFAIL  100000

#define BUFFER_SIZE 4096


//extern void delay(volatile int i);
//*&************************************************************
uint8_t g_write_buffer[BUFFER_SIZE];
uint8_t no_of_files =0;
uint8_t g_read_buf[BUFFER_SIZE];
uint32_t g_src_image_target_address =0;
uint32_t g_flash_address = 0;
uint32_t g_file_size = 0;
uint32_t g_fail = 0;
uint32_t g16_fail = 0;


uint8_t iap_data_buffer [1024];

// MT adds to service FLASH_READ and FLASH_PROGRAM request from fiber
uint16_t flash_to_fiber[512];
uint16_t fiber_to_flash[512];


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

void list_flash_dir(uint32_t start, uint32_t length){

    uint8_t  manufacturer_id;
    uint8_t  device_id;


    FLASH_init();

    FLASH_global_unprotect();
    FLASH_read_device_id
    (
            &manufacturer_id,
            &device_id
    );
    FLASH_read(start,g_read_buf,length);
    for(uint8_t i=0;i<length;i++){
        dp_display_value(g_read_buf[i],HEX);
    }
    programming_done();

}

// same as serial function above but return requested FLASH data via FLASH_TO_FIBER for block read
void list_flash_dir_fiber(uint32_t start_addr, uint32_t length, uint16_t* flash_to_fiber){

    uint8_t  manufacturer_id;
    uint8_t  device_id;

    FLASH_init();

    FLASH_global_unprotect();
    FLASH_read_device_id
    (
            &manufacturer_id,
            &device_id
    );
    FLASH_read(start_addr,g_read_buf,length);


// fill FLASH_TO_FIBER buffer to send back to the DTC
    uint8_t ieven = 0;
    for(uint8_t i=0;i<length;i+=2){
        flash_to_fiber[ieven] = (g_read_buf[i+1]<<8) + g_read_buf[i];
        ieven++;
   }

}

void write_flash_directory(uint16_t nImages,uint32_t* addresses)
{

    g_fail = 0;

    FLASH_init();

        FLASH_global_unprotect();

    FLASH_erase_64k_block(0);
    delay1(500);

    for (uint8_t i_image = 0 ; i_image < nImages;){
        uint32_t address1 = addresses[i_image];
        uint8_t buf[4];
           buf[3] = (uint8_t)((address1 >> 24) & 0xFF);
           buf[2] = (uint8_t)((address1 >> 16) & 0xFF);
           buf[1] = (uint8_t)((address1 >> 8) & 0xFF);
           buf[0] = (uint8_t)(address1 & 0xFF);
           FLASH_program(4*i_image , buf, 4);
           delay1(5000);
           FLASH_read(4*i_image,g_read_buf,4);
           if (memcmp(buf,g_read_buf,4)) {
               FLASH_erase_64k_block(address1);
               delay1(100000);
               g_fail++;
               if (g_fail < MAXFAIL)
                   continue; //this will get stuck here until MAXFAIL reached
           }

           i_image++;


    }
    programming_done();

}


// same as serial version above but skip call to PROGRAMMING_DONE
void write_flash_directory_fiber(uint16_t nImages, uint32_t* addresses, uint16_t g16_fail)
{

    g16_fail = 0;

    FLASH_init();

    FLASH_global_unprotect();

    FLASH_erase_64k_block(0);

    delay1(500);

    for (uint8_t i_image = 0 ; i_image < nImages;){

        uint32_t address1 = addresses[i_image];
        uint8_t buf[4];

        buf[3] = (uint8_t)((address1 >> 24) & 0xFF);
        buf[2] = (uint8_t)((address1 >> 16) & 0xFF);
        buf[1] = (uint8_t)((address1 >> 8) & 0xFF);
        buf[0] = (uint8_t)(address1 & 0xFF);

        FLASH_program(4*i_image , buf, 4);
        delay1(5000);

        FLASH_read(4*i_image,g_read_buf,4);

        // when written vs read-back word comparison fails
        if (memcmp(buf,g_read_buf,4)) {
            FLASH_erase_64k_block(address1);
            delay1(100000);
            g16_fail++;
            if (g16_fail < MAXFAIL)
                continue; //this will get stuck here until MAXFAIL reached
           }

        i_image++;
    }

}

void copy_to_flash(uint8_t * g_buffer)
{
    uint32_t i=0;
    uint32_t readback_address = g_flash_address;
    uint16_t BLOCK = 64;
    uint8_t success = 1;
    for(i=0;i<BUFFER_SIZE/BLOCK;) //this  assumes BLOCK write blocks, need to try random FIXME
    {

        FLASH_program(g_flash_address+i*BLOCK , &g_buffer[i*BLOCK], BLOCK);
        //readback what I just wrote
        delay1(5000); //increase the delay
        FLASH_read(readback_address+i*BLOCK,&g_read_buf[i*BLOCK],BLOCK);
//add another delay here
        if (memcmp(&g_buffer[i*BLOCK],&g_read_buf[i*BLOCK],BLOCK)) {  //this will get stuck here until MAXFAIL reached
            FLASH_erase_64k_block(g_flash_address+i*BLOCK);
            dp_display_value(g_flash_address+i*BLOCK, HEX);
            g_fail ++;
            delay1(100000);
            if (g_fail < MAXFAIL)
                continue;

        }
        i++;
    }

    //        memcpy(&g_readbackbuffer[i*BLOCK],&g_buffer[i*BLOCK], BLOCK);

    g_flash_address = g_flash_address + BUFFER_SIZE;
}

static uint32_t read_page_from_host_through_uart
(
        uint8_t * g_buffer
)
{
    uint32_t num_bytes,factor,temp;
    volatile uint32_t i = 0;
    num_bytes = BUFFER_SIZE;
    char crc;
    size_t rx_size = 0;

    uint8_t addressOffset[4]={0x0, 0x0, 0x0, 0x0};
    uint8_t numBytes[4];
    uint32_t return_bytes;

    //    uint8_t rx_buff[1],temp_add[2];
    uint8_t rx_buff[1];
    //Write Ack "b" to indicate beginning of the transaction from the target

    if(g_src_image_target_address + BUFFER_SIZE > g_file_size )
    {
        num_bytes = g_file_size - g_src_image_target_address;
    }
    if(g_src_image_target_address>= g_file_size)
    {
        return 0;
    }
    CRCFAIL:


    // Send the command to request data
    UART_polled_tx_string (&g_uart, "ProgRequest\n");
    //           while(MSS_UART_tx_complete(gp_my_uart) == 0);

    addressOffset[0] = g_src_image_target_address & 0xff;
    addressOffset[1] = (g_src_image_target_address >> 8) & 0xff;
    addressOffset[2] = (g_src_image_target_address >> 16) & 0xff;
    addressOffset[3] = (g_src_image_target_address >> 24) & 0xff;

    UART_send (&g_uart, addressOffset, 4);

    //           while(MSS_UART_tx_complete(gp_my_uart) == 0);

    numBytes[0] = num_bytes & 0xff;
    numBytes[1] = (num_bytes >> 8) & 0xff;
    numBytes[2] = (num_bytes >> 16) & 0xff;
    numBytes[3] = (num_bytes >> 24) & 0xff;

    UART_send (&g_uart, numBytes, 4);

    // Get the number of valid bytes sent
    rx_size = 0;
    while (rx_size < 4)
    {
        rx_size += UART_get_rx(&g_uart, &numBytes[rx_size],4);
    }
    return_bytes = numBytes[0];
    return_bytes |= (numBytes[1] << 8);
    return_bytes |= (numBytes[2] << 16);
    return_bytes |= (numBytes[3] << 24);

    rx_size = 0;
    while (rx_size < return_bytes)
    {
        rx_size += UART_get_rx(&g_uart, &g_buffer[rx_size],return_bytes);
    }


    //check the CRC
    while(!(UART_get_rx ( &g_uart, rx_buff, 1 )));
    factor = 1;
    crc = 0;
    while((num_bytes-1)/factor)
    {
        crc = crc^g_buffer[factor];
        factor = factor*2;
    }
    if(crc == (char)rx_buff[0])
    {
        g_src_image_target_address += return_bytes;

        UART_send(&g_uart, (const uint8_t * )"a",1);
    }
    else
    {
        UART_send(&g_uart,(const uint8_t * )"n",1);
        goto CRCFAIL;
    }

    return return_bytes;

    //   UART_send(&g_uart, (const uint8_t * )"b",1);
    //poll for Ack message from the host as an acknowledgment that the host is ready for receiving the transaction

    //   while(!(UART_get_rx ( &g_uart, rx_buff, 1 )))
    //       ;
    //for(i=0;i<2500;i++);
    //transmit the address to the host

#if 0

#if 0
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

#endif
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


void programming_done(){
    uint8_t failures[4];

    failures[0]= g_fail & 0xFF;
    failures[1] = (g_fail >> 8 ) & 0xFF;
    failures[2] = (g_fail >> 16 ) & 0xFF;
    failures[3] = (g_fail >> 24 ) & 0xFF;
    UART_polled_tx_string (&g_uart, "ProgDone\n");
    UART_send (&g_uart, failures, 4);

}



void load_spi_flash_at_address(uint32_t index_address)
{
    volatile uint32_t erase_address = 0;
    volatile uint32_t erase_count=0;
    volatile uint32_t i = 0,length = 0;
    uint8_t numBytes[4];
    uint32_t return_bytes;

    uint8_t rx_buff[8],num[9];
    uint8_t  manufacturer_id;
    uint8_t  device_id;

    g_fail = 0;

    //get the files size

    // Get the number of valid bytes sent
    UART_polled_tx_string (&g_uart, "ProgFileSize\n");
    uint8_t rx_size = 0;
    while (rx_size < 4)
    {
        rx_size += UART_get_rx(&g_uart, &numBytes[rx_size],4);
    }

    g_file_size = numBytes[0];
    g_file_size |= (numBytes[1] << 8);
    g_file_size |= (numBytes[2] << 16);
    g_file_size |= (numBytes[3] << 24);


    FLASH_init();

    FLASH_global_unprotect();
    FLASH_read_device_id
    (
            &manufacturer_id,
            &device_id
    );

    uint32_t file_size_in_blocks = 1 + g_file_size / 65536;
    erase_address = index_address; //only erase starting at  address
    for(erase_count = 0;erase_count<=file_size_in_blocks;erase_count++)
    {

        FLASH_erase_64k_block(erase_address);
        delay1(50000);
//        FLASH_read(erase_address,g_read_buf,32);
        erase_address+=0x10000;
//        erase_address+=0x1000;
/*        if(g_read_buf[0] != 0xFF)
        {

            break;
        }*/

    }


    //this redoes the flash directory, since we are doing a single image, don't do this, will keep the same memory map
    //    write_flash_directory();

    g_flash_address = index_address;
    g_src_image_target_address = 0;




    do
    {
        length = read_page_from_host_through_uart(g_write_buffer);
        if(length>0)
        {
            copy_to_flash(g_write_buffer);
 //           for (uint16_t i = 0 ; i < BUFFER_SIZE; i++){
 //               UART_send (&g_uart, g_read_buffer, BUFFER_SIZE);
 //           }
            //memcpy(ddr_add1,g_page_buffer,length);
            //ddr_add1 +=  length;
        }

    }while(length!=0);

    programming_done();
}

void load_spi_flash_at_address_fiber(uint32_t index_address, uint8_t* fiber_to_flash, uint16_t fail_mask)
{
    volatile uint32_t erase_address = 0;
    volatile uint32_t erase_count=0;
    uint8_t  manufacturer_id;
    uint8_t  device_id;
    uint32_t readback_address = index_address;
    uint16_t BLOCK = 128;

    // mimic COPY_TO_FLASH for a 1kB block of data, in miniblocks of 128 words
    for (int  i=0; i<1024/BLOCK; i++) {
        FLASH_program(index_address+i*BLOCK , &fiber_to_flash[i*BLOCK], BLOCK);

        //readback what I just wrote
        delay1(5000); //increase the delay
        FLASH_read(readback_address+i*BLOCK,&g_read_buf[i*BLOCK],BLOCK);

        if (memcmp(&fiber_to_flash[i*BLOCK],&g_read_buf[i*BLOCK],BLOCK)) {  //this will get stuck here until MAXFAIL reached
            fail_mask += (i<<1);
        }
    }

}

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




/*==============================================================================
  Display greeting message when application is started.
 */



void execute_iap(uint8_t option, uint32_t img_index)
{

    //index here can be either image index or spi address
    uint8_t cmd = 0;
    uint32_t spiaddr_or_idx = 0;
    uint8_t status = 0xFF;
    switch(option)
    {
    case 4:
        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nIAP PROGRAM for image at index  is in progress...\n\rIt takes approximately 28 seconds\n\r");

        cmd = IAP_PROGRAM_BY_SPIIDX_CMD;
        spiaddr_or_idx = img_index;
        break;
    case 5:
        UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nIAP PROGRAM for image at address  is in progress...\n\rIt takes approximately 28 seconds\n\r");
        cmd = IAP_PROGRAM_BY_SPIADDR_CMD;
        spiaddr_or_idx = img_index;
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

void execute_bitstream_authenticate(uint32_t img_index)
{

    uint32_t mb_offset = 0;
    uint8_t status = 0xFF;
    UART_polled_tx_string(&g_uart, (const uint8_t*)"\r\nBitstream authentication for image at address  is in progress...\n\r");
    status = SYS_bitstream_authenticate_service(img_index, mb_offset);
    UART_polled_tx_string(&g_uart, (const uint8_t*)"Authentication status: ");
    if(SYS_SUCCESS == status)
    {
        UART_polled_tx_string(&g_uart, (const uint8_t*)"SUCCESS ");
    }
    else
    {
        display_output(&status, 1);
    }
    programming_done();
}

void execute_iap_image_authenticate(uint32_t img_index)
{

    uint8_t status = 0xFF;


    UART_polled_tx_string(&g_uart, "\r\nIAP image authentication for image at index is in progress...\n\r");
    status = SYS_IAP_image_authenticate_service(img_index);
    UART_polled_tx_string(&g_uart, "Authentication status: ");
    if(SYS_SUCCESS == status)
    {
        UART_polled_tx_string(&g_uart,"SUCCESS ");
    }
    else
    {
        display_output(&status, 1);
    }

    programming_done();


}
