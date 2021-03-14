/* ************ MICROSEMI SOC CORP. DIRECTC LICENSE AGREEMENT ************* */
/* ------------------------------------------------------------------------ 
PLEASE READ: BEFORE INSTALLING THIS SOFTWARE, CAREFULLY READ THE FOLLOWING 
MICROSEMI SOC CORP LICENSE AGREEMENT REGARDING THE USE OF THIS SOFTWARE. 
INSTALLING THIS SOFTWARE INDICATES THAT YOU ACCEPT AND UNDERSTAND THIS AGREEMENT 
AND WILL ABIDE BY IT. 

Note: This license agreement (�License�) only includes the following software: 
DirectC. DirectC is licensed under the following terms and conditions.

Hereinafter, Microsemi SoC Corp. shall be referred to as �Licensor� or �Author,� 
whereas the other party to this License shall be referred to as �Licensee.� Each 
party to this License shall be referred to, singularly, as a �Party,� or, 
collectively, as the �Parties.�

Permission to use, copy, modify, and/or distribute DirectC for any purpose, with
or without fee, is hereby granted by Licensor to Licensee, provided that the 
above Copyright notice and this permission notice appear in all copies, 
modifications and/or distributions of DirectC.

DIRECTC IS PROVIDED "AS IS" AND THE AUTHOR/LICENSOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO DIRECTC INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND 
FITNESS. IN NO EVENT SHALL AUTHOR/LICENSOR BE LIABLE TO LICENSEE FOR ANY DAMAGES, 
INCLUDING SPECIAL, DIRECT,INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES 
WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF 
CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION 
WITH THE USE OR PERFORMANCE OF DIRECTC.

Export Control: Information furnished to Licensee may include United States 
origin technical data. Accordingly, Licensee is responsible for complying with, 
and warrants to Licensor that it will comply with, all U.S. export control laws 
and regulations, including the provisions of the Export Administration Act of 
1979 and the Export Administration Regulations promulgated thereunder, the Arms 
Export Control Act, and the sanctions laws administered by the Office of Foreign 
Assets Control including any other U.S. Government regulation applicable to the 
export, re-export, or disclosure of such controlled technical data (or the 
products thereof) to Foreign Nationals, whether within or without the U.S., 
including those employed by, or otherwise associated with, Licensee. Licensee 
shall obtain Licensor�s written consent prior to submitting any request for 
authority to export any such technical data.

ADR: Any dispute between the Parties arising from or related to this License or 
the subject matter hereof, including its validity, construction or performance 
thereunder, shall be exclusively resolved through arbitration by a mutually 
acceptable impartial and neutral arbitrator appointed by the Judicial 
Arbitration and Mediation Services (JAMS) in accordance with its rules and 
procedures. If the Parties are not able to agree on an arbitrator within 10 days 
of the date of request for mediation is served, then JAMS shall appoint an 
arbitrator. Notice of arbitration shall be served and filed with the JAMS main 
offices in Irvine, California. Each Party shall be responsible for all costs 
associated with the preparation and representation by attorneys, or any other 
persons retained thereby, to assist in connection with any such Arbitration. 
However, all costs charged by the mutually agreed upon Arbitration entity shall 
be equally shared by the Parties. The Party seeking Mediation and/or Arbitration 
as provided herein agrees that the venue for any such Mediation and Arbitration 
shall be selected by the other Party and that such venue must be Los Angeles, 
California; New York, New York; or Chicago, Illinois; whereby the applicable law 
and provisions of the Evidence Code of the State selected thereby shall be 
applicable and shall govern the validity, construction and performance of this 
License.

Governing Law: This license will be governed by the laws of the State of 
California, without regard to its conflict of law provisions.

Entire Agreement: This document constitutes the entire agreement between the 
Parties with respect to the subject matter herein and supersedes all other 
communications whether written or oral.                                     */

/* ************************************************************************ */
/*                                                                          */
/*  SPI_DirectC     Copyright (C) Microsemi Corporation                     */
/*  Version 2.1     Release date January 29, 2018                           */
/*                                                                          */
/* ************************************************************************ */
/*                                                                          */
/*  Module:         dpG5alg.c                                               */
/*                                                                          */
/*  Description:    Contains PolarFire programming functions                */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"
#include "dputil.h"
#include "dpcom.h"
#include "dpalg.h"
#include "dpG5alg.h"
#include "dpG5spi.h"

#ifdef ENABLE_G5M_SUPPORT
DPUCHAR G5M_shared_buf[100]; /* Maximum of 768 */

/****************************************************************************
* Purpose: main entry function
*  This function needs to be called from the main application function with
*  the approppriate action code set to intiate the desired action.
****************************************************************************/
DPUCHAR dp_top_g5m (void)
{
    dp_G5M_init_vars();
    dp_G5M_check_action();
    if (error_code == DPE_SUCCESS)
    {
        dp_G5M_perform_action();
    }
    return error_code;
}

void dp_G5M_init_vars(void)
{
    pgmmode_flag = FALSE;
    pgmmode = 0u;
    
    return;
}

void dp_G5M_check_action(void)
{
    if (! (
    (Action_code == DP_READ_IDCODE_ACTION_CODE) ||
    (Action_code == DP_DEVICE_INFO_ACTION_CODE) ||
    (Action_code == DP_ERASE_ACTION_CODE) ||
    (Action_code == DP_PROGRAM_ACTION_CODE) ||
    (Action_code == DP_VERIFY_ACTION_CODE) ||
    (Action_code == DP_ENC_DATA_AUTHENTICATION_ACTION_CODE) ||
    (Action_code == DP_VERIFY_DIGEST_ACTION_CODE)
    ))
    {
        error_code = DPE_ACTION_NOT_SUPPORTED;
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nInvalid action.");
        #endif
    }
    #ifndef ENABLE_DISPLAY
    if ( (Action_code == DP_READ_IDCODE_ACTION_CODE) ||
    (Action_code == DP_DEVICE_INFO_ACTION_CODE) )
    error_code = DPE_ACTION_NOT_SUPPORTED;
    #endif
    return;
}

void dp_G5M_perform_action (void)
{
    #ifdef ENABLE_DISPLAY
    if (Action_code == DP_READ_IDCODE_ACTION_CODE)
    {
        dp_read_idcode_action();
        action_performed = TRUE;
    }
    else if (Action_code == DP_DEVICE_INFO_ACTION_CODE)
    {
        dp_G5M_device_info_action();
        action_performed = TRUE;
    }
    #endif
    if (action_performed == FALSE)
    {
        dp_check_image_crc();
        if (error_code == DPE_SUCCESS)
        {
            dp_check_device_ID();
            if (error_code == DPE_SUCCESS)
            {
                switch (Action_code)
                {
                    case DP_ERASE_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G5M_erase_action();
                    break;
                    case DP_PROGRAM_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G5M_program_action();
                    break;
                    case DP_VERIFY_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G5M_verify_action();
                    break;
                    case DP_ENC_DATA_AUTHENTICATION_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G5M_enc_data_authentication_action();
                    break;
                    case DP_VERIFY_DIGEST_ACTION_CODE: 
                    action_performed = TRUE;               
                    dp_G5M_verify_digest_action();
                    break;
                }
            }
        }
    }
    dp_G5M_exit();
    return;
}

void dp_G5M_read_idcode(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_IDCODE;
    G5M_SPI_SCAN_out(0u,32u);
    
    device_ID = (DPULONG)spi_output_buffer[0] | (DPULONG)spi_output_buffer[1] << 8u | (DPULONG)spi_output_buffer[2] << 16u | (DPULONG)spi_output_buffer[3] << 24u;
    device_rev = (DPULONG) (device_ID >> 28);
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nID read: ");
    dp_display_value(device_ID,HEX);
    #endif
    
    if ((device_ID & 0xfffu) != MICROSEMI_ID)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nError: Invalid ID ");
        dp_display_value(device_ID,HEX);
        #endif
        error_code = DPE_IDCODE_ERROR;
    }
    
    return;
}

void dp_G5M_erase_action(void)
{
    dp_G5M_initialize();
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nErasing the device takes few seconds.  Please wait...");
    #endif
    
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x1u;
        dp_G5M_set_mode();
        
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,GXM_NUMOFCOMPONENT_OFFSET,GXM_NUMOFCOMPONENT_BYTE_LENGTH);
            global_uint2 = global_uint1 - ((DPUINT)dp_get_bytes(Header_ID,GXM_ERASEDATASIZE_OFFSET,GXM_DATASIZE_BYTE_LENGTH) - 1u);
            dp_G5M_process_data(GXM_erasedatastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_ERASE_ERROR;
            }
        }
    }
    return;
}

void dp_G5M_program_action(void)
{
    dp_G5M_initialize();
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming...");
    #endif
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x1u;
        dp_G5M_set_mode();
        
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(GXM_Header_ID,GXM_DATASIZE_OFFSET,GXM_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G5M_process_data(GXM_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_PROGRAM_ERROR;
            }
        }
    }
    return;
}

void dp_G5M_verify_action(void)
{
    dp_G5M_initialize();
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nPerforming verify...");
    #endif
    
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x2u;
        dp_G5M_set_mode();
        
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(GXM_Header_ID,GXM_DATASIZE_OFFSET,GXM_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G5M_process_data(GXM_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_PROGRAM_ERROR;
            }
        }
    }
    return;
}

void dp_G5M_enc_data_authentication_action(void)
{
    dp_G5M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x0u;
        dp_G5M_set_mode();
        
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(GXM_Header_ID,GXM_DATASIZE_OFFSET,GXM_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G5M_process_data(GXM_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_AUTHENTICATION_FAILURE;
            }
        }
    }
    return;
}

void dp_G5M_verify_digest_action(void)
{
    dp_G5M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        dp_G5M_query_security();
    }
    if ((error_code == DPE_SUCCESS) && ((G5M_shared_buf[1] & G5M_UL_EXTERNAL_DIGEST_CHECK) == G5M_UL_EXTERNAL_DIGEST_CHECK) )
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("r\nExternal digest check via JTAG/SPI Slave is disabled.");
        #endif
        error_code = DPE_VERIFY_DIGEST_ERROR;
    }
    
    if (error_code == DPE_SUCCESS)
    {
        dp_flush_spi_buffers();
        spi_input_buffer[0] = 0x1u;
        spi_command_buffer = G5M_CHECK_DIGESTS;
        G5M_SPI_SCAN_out(16u,16u);
        
        {
            if (spi_output_buffer[1] == 0x40u)
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\nFailed to verify digest.");
                #endif
                error_code = DPE_VERIFY_DIGEST_ERROR;
            }
            else 
            {
                #ifdef ENABLE_DISPLAY
                if ((spi_output_buffer[0] & 0x1u )== 0x1u)
                {
                    dp_display_text("\r\n --- FPGA Fabric digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- FPGA Fabric digest verification: FAIL");
                }
                if ((spi_output_buffer[0] & 0x2u )== 0x2u)
                {
                    dp_display_text("\r\n --- Fabric Configuration digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- Fabric Configuration digest verification: FAIL");
                }
                if ((spi_output_buffer[0] & 0x4u )== 0x4u)
                {
                    dp_display_text("\r\n --- sNVM digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- sNVM digest verification: FAIL");
                }
                if ((spi_output_buffer[0] & 0x8u )== 0x8u)
                {
                    dp_display_text("\r\n --- User security policies segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- User security policies segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[0] & 0x10u )== 0x10u)
                {
                    dp_display_text("\r\n --- SMK segment digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- SMK segment digest verification: FAIL");
                }
                if ((spi_output_buffer[0] & 0x20u )== 0x20u)
                {
                    dp_display_text("\r\n --- User Public Key segment digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- User Public Key segment digest verification: FAIL");
                }
                if ((spi_output_buffer[0] & 0x40u )== 0x40u)
                {
                    dp_display_text("\r\n --- UPK1 segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- UPK1 segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[0] & 0x80u )== 0x80u)
                {
                    dp_display_text("\r\n --- UEK1 segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- UEK1 segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[1] & 0x1u )== 0x1u)
                {
                    dp_display_text("\r\n --- DPK segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- DPK segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[1] & 0x2u )== 0x2u)
                {
                    dp_display_text("\r\n --- UPK2 segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- UPK2 segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[1] & 0x4u )== 0x4u)
                {
                    dp_display_text("\r\n --- UEK2 segment digest verification: PASS" );
                }
                else
                {
                    dp_display_text("\r\nWarning: --- UEK2 segment digest verification: FAIL" );
                }
                if ((spi_output_buffer[1] & 0x10u )== 0x10u)
                {
                    dp_display_text("\r\n --- Factory row and factory key segment digest verification: PASS");
                }
                else
                {
                    dp_display_text("\r\nWarning: --- Factory row and factory key segment digest verification: FAIL");
                }
                #endif
            }
        }
    }
    return;   
}


#ifdef ENABLE_DISPLAY
void dp_G5M_device_info_action(void)
{
    if (error_code == DPE_SUCCESS)
    {
        dp_G5M_read_user_code();
        if (error_code == DPE_SUCCESS)
        {
            dp_G5M_read_design_info();
            if (error_code == DPE_SUCCESS)
            {
                dp_G5M_read_debug_info();
                if (error_code == DPE_SUCCESS)
                {
                    dp_G5M_read_fsn();
                    if (error_code == DPE_SUCCESS)
                    {
                        dp_G5M_query_security();
                    }
                }
            }
        }
    }
    return;
}


void dp_G5M_read_user_code(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_USERCODE;
    G5M_SPI_SCAN_out(0u,32u);
    
    dp_display_text("\r\nSILSIG: ");
    dp_display_array(spi_output_buffer,4u,HEX);
    
    return;
}

void dp_G5M_read_design_info(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_DESIGN_INFO;
    G5M_SPI_SCAN_in(0u);
    
    dp_G5M_read_shared_buffer(3u);
    
    
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nDesign Name: ");
        for (global_uchar1 = 2u; global_uchar1 < 32u; global_uchar1++)
        {
            dp_display_value(G5M_shared_buf[global_uchar1],CHR);
        }
        dp_display_text("\r\nChecksum: ");
        dp_display_array(G5M_shared_buf,2u,HEX);
        dp_display_text("\r\nDesign Info: ");
        dp_display_array(G5M_shared_buf,34u,HEX);
        dp_display_text("\r\nDESIGNVER: ");
        dp_display_array(&G5M_shared_buf[32],2u,HEX);
        dp_display_text("\r\nBACKLEVEL: ");
        dp_display_array(&G5M_shared_buf[34],2u,HEX);
    }
    
    return;
}

void dp_G5M_read_debug_info(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_DEBUG_INFO;
    G5M_SPI_SCAN_in(0u);
    
    dp_G5M_read_shared_buffer(5u);
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nDebug Info: ");
        dp_display_array(G5M_shared_buf,80u,HEX);
    }
    return;
}

void dp_G5M_read_fsn(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_SN;
    G5M_SPI_SCAN_out(0u,SPI_BUFFER_BIT_SIZE);
    dp_display_text("\r\n=====================================================================");
    dp_display_text("\r\nDSN: ");
    dp_display_array(spi_output_buffer, SPI_BUFFER_BYTE_SIZE, HEX);
    dp_display_text("\r\n=====================================================================");
    return;
}
#endif

void dp_G5M_read_shared_buffer(DPUCHAR ucNumOfBlocks)
{
    
    for (global_uchar1 = 0u; global_uchar1 < ucNumOfBlocks; global_uchar1++)
    {
        dp_flush_spi_buffers();
        spi_command_buffer = G5M_SPI_READ_BUFFER;
        spi_input_buffer[0] = global_uchar1;
        G5M_SPI_SCAN_out(SPI_BUFFER_BIT_SIZE,SPI_BUFFER_BIT_SIZE);
        
        for (global_uchar2 = 0;global_uchar2 < SPI_BUFFER_BYTE_SIZE; global_uchar2++)
        {
            G5M_shared_buf[global_uchar1*SPI_BUFFER_BYTE_SIZE + global_uchar2] = spi_output_buffer[global_uchar2];
        }
    }
    
    return;
}

void dp_G5M_perform_isc_enable(void)
{
    pgmmode_flag = TRUE;
    dp_flush_spi_buffers();
    spi_input_buffer[0] |= (G5M_ALGO_VERSION & 0x3fu);
    spi_input_buffer[2] |= (G5M_DIRECTC_VERSION & 0x3fu) << 1u;
    spi_input_buffer[2] |= (DIRECTC_PROGRAMMING & 0x7u) << 7u;
    spi_input_buffer[3] |= (DIRECTC_PROGRAMMING & 0x7u) >> 1u;
    spi_input_buffer[3] |= 0x28u;
    
    spi_command_buffer = G5M_SPI_ISC_ENABLE;
    G5M_SPI_SCAN_out(32u,32u);
    
    if ((spi_output_buffer[0] & 0x1) == 1)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to enter ming mode.");
        #endif
        error_code = DPE_INIT_FAILURE;
    }
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nISC_ENABLE_RESULT: ");
    dp_display_array(spi_output_buffer,4u,HEX);
    #endif
    
    
    
    /* Display CRCERR */
    #ifdef ENABLE_DISPLAY
    global_uchar1 = spi_output_buffer[0] & 0x1u;
    dp_display_text("\r\nCRCERR: ");
    dp_display_value(global_uchar1,HEX);
    #endif
    
    
    return;
}

/* Enter programming mode */
void dp_G5M_initialize(void)
{
    dp_G5M_query_security();
    if ((error_code == DPE_SUCCESS) && ((G5M_shared_buf[7] & G5M_UL_USER_KEY1) == G5M_UL_USER_KEY1) )
    {
        dp_G5M_unlock_upk1();
    }
    if ((error_code == DPE_SUCCESS) && ((G5M_shared_buf[7] & G5M_UL_USER_KEY2) == G5M_UL_USER_KEY2) )
    {
        dp_G5M_unlock_upk2();
    }
    if (error_code == DPE_SUCCESS)
    {
        dp_G5M_perform_isc_enable();
    }
    
    return;
}


/* Function is used to exit programming mode */
void dp_G5M_exit(void)
{
    if (pgmmode_flag == TRUE)
    {
        dp_flush_spi_buffers();
        spi_command_buffer = G5M_SPI_ISC_DISABLE;
        G5M_SPI_SCAN_in(0u);
        dp_delay(1000u);
    }
    #ifdef ENABLE_DISPLAY
    dp_G5M_read_fsn();
    #endif
    
    spi_command_buffer = G5M_SPI_RELEASE;
    G5M_SPI_SCAN_in(0);
    
    spi_command_buffer = G5M_SPI_RELEASE;
    G5M_SPI_SCAN_in(0);
    
    UART_polled_tx_string (&g_uart, "ProgDone\n");

    return;
}

void dp_G5M_set_mode(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_G4M_FRAME_INIT;
    spi_input_buffer[0] = pgmmode;
    
    G5M_SPI_SCAN_in(8u);
    
    return;
}


void dp_G5M_process_data(DPUCHAR BlockID)
{
    #ifdef ENABLE_DISPLAY
    /* Global unit1 is used to hold the number of components */
    /* Loop through the number of components */
    dp_display_text("\r\n");
    #endif
    
    
    DataIndex = 0u;
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        global_ulong1 = dp_get_bytes(GXM_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
        global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
        global_ulong1 &= 0x3FFFFFu;
        
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nProcessing component ");
        dp_display_value(global_uint2,DEC);
        dp_display_text(". Please wait...");
        old_progress = 0;
        #endif
        
        for (global_ulong2 = 1u; global_ulong2 <= global_ulong1; global_ulong2++)
        {
            #ifdef ENABLE_DISPLAY
            new_progress = (global_ulong2 *100 / global_ulong1);
            if (new_progress != old_progress)
            {
//                dp_report_progress(new_progress);
                old_progress = new_progress;
            }
            #endif
            
            spi_command_buffer = G5M_SPI_G4M_FRAME_DATA;
            dp_G5M_get_and_shift_in(BlockID, SPI_BUFFER_BIT_SIZE, DataIndex);
            
            if ( error_code != DPE_SUCCESS )
            {
                dp_G5M_get_data_status();
                #ifdef ENABLE_DISPLAY            
                dp_display_text("\r\ncomponentNo: ");
                dp_display_value(global_uint2, DEC);
                dp_display_text("\r\nblockNo: ");
                dp_display_value(global_ulong2, DEC);
                dp_display_text("\r\nDATA_STATUS_RESULT: ");
                dp_display_array(spi_output_buffer,4u,HEX);
                dp_display_text("\r\nERRORCODE: ");
                dp_display_value(spi_output_buffer[1],HEX);
                #endif
                global_uint2 = global_uint1;
                break;
            }
            DataIndex += SPI_BUFFER_BIT_SIZE;
        }
    }
    
    return;
}

void dp_G5M_get_data_status(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_G4M_FRAME_STATUS;
    G5M_SPI_SCAN_out(0u,64u);
    
    if (spi_output_buffer[0] & 0x4u)
    {
        error_code = DPE_PROCESS_DATA_ERROR;
    }
    
    return;
}

void dp_G5M_query_security(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_QUERY_SECURITY;
    G5M_SPI_SCAN_in(0);
    dp_G5M_read_shared_buffer(1u);
    
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\n--- Security locks and configuration settings ---\r\n");
    dp_display_array(G5M_shared_buf,16u,HEX);
    #endif
    
    return;
}

void dp_G5M_unlock_upk1(void)
{
    dp_get_data(GXM_UPK1_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY      
        dp_display_text("\r\nWarning: UPK1 data is missing.");
        #endif
    }
    else
    {
        dp_G5M_load_upk1();
        if (error_code == DPE_SUCCESS)
        {
            dp_flush_spi_buffers();
            spi_command_buffer = G5M_UNLOCK_USER_PASSCODE1;
            G5M_SPI_SCAN_out(0u,8u);
        }
        if ((error_code != DPE_SUCCESS) || ((spi_output_buffer[0] & 0x3u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock user pass key 1.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        #ifdef ENABLE_DISPLAY
        else
        {
            dp_display_text("\r\nUser security (DPK1) is unlocked.");
        }
        #endif
    }
    return;
}

void dp_G5M_unlock_upk2(void)
{
    dp_get_data(GXM_UPK2_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: UPK2 data is missing.");
        #endif
    }
    else
    {
        dp_G5M_load_upk2();
        if (error_code == DPE_SUCCESS)
        {
            dp_flush_spi_buffers();
            spi_command_buffer = G5M_UNLOCK_USER_PASSCODE2;
            G5M_SPI_SCAN_out(0u,8u);
        }
        if ((error_code != DPE_SUCCESS) || ((spi_output_buffer[0] & 0x3u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock user pass key 2.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        #ifdef ENABLE_DISPLAY
        else
        {
            dp_display_text("\r\nUser security (DPK2) is unlocked.");
        }
        #endif
    }
    return;
}

void dp_G5M_load_upk1(void)
{
    spi_command_buffer = G5M_SPI_LEY_LO;
    dp_G5M_get_and_shift_in(GXM_UPK1_ID, SPI_BUFFER_BIT_SIZE, 0u);
    spi_command_buffer = G5M_SPI_LEY_HI;
    dp_G5M_get_and_shift_in(GXM_UPK1_ID, SPI_BUFFER_BIT_SIZE, SPI_BUFFER_BIT_SIZE);
    
    return;
}

void dp_G5M_load_upk2(void)
{
    spi_command_buffer = G5M_SPI_LEY_LO;
    dp_G5M_get_and_shift_in(GXM_UPK2_ID, SPI_BUFFER_BIT_SIZE, 0u);
    spi_command_buffer = G5M_SPI_LEY_HI;
    dp_G5M_get_and_shift_in(GXM_UPK2_ID, SPI_BUFFER_BIT_SIZE, SPI_BUFFER_BIT_SIZE);
    
    return;
}
#endif

/*   *************** End of File *************** */

