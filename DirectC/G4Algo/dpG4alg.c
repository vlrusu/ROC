/* ************ MICROSEMI SOC CORP. DIRECTC LICENSE AGREEMENT ************* */
/* ------------------------------------------------------------------------ 
PLEASE READ: BEFORE INSTALLING THIS SOFTWARE, CAREFULLY READ THE FOLLOWING 
MICROSEMI SOC CORP LICENSE AGREEMENT REGARDING THE USE OF THIS SOFTWARE. 
INSTALLING THIS SOFTWARE INDICATES THAT YOU ACCEPT AND UNDERSTAND THIS AGREEMENT 
AND WILL ABIDE BY IT. 

Note: This license agreement (“License”) only includes the following software: 
DirectC. DirectC is licensed under the following terms and conditions.

Hereinafter, Microsemi SoC Corp. shall be referred to as “Licensor” or “Author,” 
whereas the other party to this License shall be referred to as “Licensee.” Each 
party to this License shall be referred to, singularly, as a “Party,” or, 
collectively, as the “Parties.”

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
shall obtain Licensor’s written consent prior to submitting any request for 
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
/*  Module:         dpG4alg.c                                               */
/*                                                                          */
/*  Description:    Contains SmartFusion2 and IGLOO2 programming functions  */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"
#include "dputil.h"
#include "dpcom.h"
#include "dpalg.h"
#include "dpG4alg.h"
#include "dpG4spi.h"

#ifdef ENABLE_G4M_SUPPORT
DPUCHAR shared_buf[96]; /* Maximum of 768 */
DPUCHAR envm_only_flag;
DPUCHAR security_queried;
DPUCHAR sec_ul;
DPUCHAR SYSREG_TEMP[4];
DPUCHAR component_type;


/****************************************************************************
* Purpose: main entry function
*  This function needs to be called from the main application function with
*  the approppriate action code set to intiate the desired action.
****************************************************************************/
DPUCHAR dp_top_g4m (void)
{
    dp_G4M_init_vars();
    dp_G4M_check_action();
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_perform_action();
    }
    return error_code;
}

void dp_G4M_init_vars(void)
{
    pgmmode_flag = FALSE;
    security_queried = FALSE;
    pgmmode = 0u;
    sec_ul = 0u;
    envm_only_flag = FALSE;
    
    return;
}

void dp_G4M_check_action(void)
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

void dp_G4M_perform_action (void)
{
    #ifdef ENABLE_DISPLAY
    if (Action_code == DP_READ_IDCODE_ACTION_CODE)
    {
        dp_read_idcode_action();
        action_performed = TRUE;
    }
    else if (Action_code == DP_DEVICE_INFO_ACTION_CODE)
    {
        dp_G4M_device_info_action();
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
                    dp_G4M_erase_action();
                    break;
                    case DP_PROGRAM_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G4M_program_action();
                    break;
                    case DP_VERIFY_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G4M_verify_action();
                    break;
                    case DP_ENC_DATA_AUTHENTICATION_ACTION_CODE:
                    action_performed = TRUE;
                    dp_G4M_enc_data_authentication_action();
                    break;
                    case DP_VERIFY_DIGEST_ACTION_CODE: 
                    action_performed = TRUE;
                    dp_G4M_verify_digest_action();
                    break;
                }
            }
        }
    }
    dp_G4M_exit();
    return;
}

void dp_G4M_read_idcode(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_READ_IDCODE;
    SPI_SCAN_out();
    
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

void dp_G4M_erase_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_prepare_bitstream();
        pgmmode = 0x1u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_NUMOFCOMPONENT_OFFSET,G4M_NUMOFCOMPONENT_BYTE_LENGTH);
            global_uint2 = global_uint1 - ((DPUINT)dp_get_bytes(Header_ID,G4M_ERASEDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) - 1u);
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nErasing the device takes few seconds.  Please wait...");
            #endif
            dp_G4M_process_data(G4M_erasedatastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_ERASE_ERROR;
            }
        }
    }
    return;
}

void dp_G4M_program_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_prepare_bitstream();
    }
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nProgramming...");
    #endif
    
    
    if ((error_code == DPE_SUCCESS) && (sec_ul & 0x2u))
    {
        pgmmode = 0x1u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_NUMOFCOMPONENT_OFFSET,G4M_NUMOFCOMPONENT_BYTE_LENGTH);
            global_uint2 = global_uint1 - ((DPUINT)dp_get_bytes(Header_ID,G4M_ERASEDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) - 1u);
            dp_G4M_process_data(G4M_erasedatastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_ERASE_ERROR;
            }
        }
    }
    
    if (error_code == DPE_SUCCESS)
    {
        global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
        global_uint2 = 1u;
        dp_G4M_setup_eNVM(G4M_datastream_ID);
        
        pgmmode = 0x1u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(G4M_Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G4M_process_data(G4M_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_PROGRAM_ERROR;
            }
            else
            {
                dp_G4M_post_setup_eNVM();
            }
        }
    }
    
    return;
}

void dp_G4M_verify_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_prepare_bitstream();
    }
    
    
    global_uint2 = 1u;
    global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
    dp_G4M_setup_eNVM(G4M_datastream_ID);
    
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x2u;
        dp_set_mode();
        
        if (error_code == DPE_SUCCESS)
        {
            if ((DPUINT)dp_get_bytes(Header_ID,G4M_VERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) != 0u)
            {
                /* Both global_unit1 and global_unit2 have DATASIZE, ENVMDATASIZE and ENVMVERIFYDATASIZE; */
                global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
                (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
                (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMVERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                global_uint2 = global_uint1 + 1u;
                global_uint1 += (DPUINT)dp_get_bytes(Header_ID,G4M_VERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                dp_G4M_process_data(G4M_VerifyDataStream_ID);
            }
            else
            {
                global_uint2 = 1u;
                global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
                dp_G4M_process_data(G4M_datastream_ID);
            }
            
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_VERIFY_ERROR;
            }
            else
            {
                dp_G4M_post_setup_eNVM();
            }
        }
    }
    
    return;
}

void dp_G4M_enc_data_authentication_action(void)
{
    dp_G4M_initialize();
    if (error_code == DPE_SUCCESS)
    {
        pgmmode = 0x0u;
        dp_set_mode();
        
        
        if (error_code == DPE_SUCCESS)
        {
            /* Global unit1 is used to hold the number of components */
            global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
            global_uint2 = 1u;
            
            dp_G4M_process_data(G4M_datastream_ID);
            if(error_code != DPE_SUCCESS)
            {
                error_code = DPE_AUTHENTICATION_FAILURE;
            }
        }
        
    }
    return;
}

void dp_G4M_verify_digest_action(void)
{
    dp_G4M_initialize();
    dp_flush_spi_buffers();
    if (error_code == DPE_SUCCESS)
    {
        
        DataIndex = 0u;
        global_uint1 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH);
        for (global_uint2 = 1u; global_uint2 <= global_uint1; global_uint2++)
        {
            /* get the number of blocks */
            /* Global ulong1 is used to hold the number of blocks within the components */
            global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
            global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
            global_ulong1 &= 0x3FFFFFu;
            
            component_type = (DPUCHAR) dp_get_bytes(GXM_datastream_ID,COMPONENT_TYPE_IN_HEADER_BYTE+DataIndex/8,1);
            if (component_type == G4M_FPGA)
            {
                spi_input_buffer[0] |= 0x1u;
            }
            else if (component_type == G4M_ENVM)
            {
                /* Component type is no longer needed since we know it is eNVM.  We need to know which module */
                component_type = (DPUCHAR) dp_get_bytes(G4M_datastream_ID,COMPONENT_TYPE_IN_HEADER_BYTE+ENVM_MODULE_ID_IN_HEADER_BYTE+DataIndex/8,1);
                if (component_type == 0u)
                {
                    spi_input_buffer[0] |= 0x2u;
                }
                else if (component_type == 1u)
                {
                    spi_input_buffer[0] |= 0x4u;
                }
            }
            DataIndex += G4M_FRAME_BIT_LENGTH * global_ulong1;
        }
        global_ulong1 = dp_get_bytes(Header_ID,G4M_ID_MASK_OFFSET,4U);
        if ( ( (device_ID & global_ulong1) == M2S090_ID) || ( (device_ID & global_ulong1) == M2S150_ID) )
        {
            spi_input_buffer[0] |= 0xcu;
        }
        spi_input_buffer[1] |= 0x3cu;
        
        
        
        spi_command_buffer = SPI_CHECK_DIGESTS;
        SPI_SCAN_out();
        
        if (error_code == DPE_SUCCESS)
        {
            if (( spi_output_buffer[1] & 0x3c0u) == 0x3cu)
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\nDigest request from SPI/JTAG is protected by user pass key 1.");
                #endif
                error_code = DPE_VERIFY_DIGEST_ERROR;
            }
            else if (( spi_output_buffer[1] & 0x40u) == 0x40u)
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\nFailed to verify digest.");
                #endif
                error_code = DPE_VERIFY_DIGEST_ERROR;
            }
            if ((spi_input_buffer[0] & 0x1u) == 0x1u)
            {
                if (( spi_output_buffer[0] & 0x1u) == 0x1u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nFPGA Fabric digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nFPGA Fabric digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
                if (( spi_output_buffer[0] & 0x80u) == 0x80u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nFabric configuration segment digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nFabric configuration segment digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            if ((spi_input_buffer[0] & 0x2u) == 0x2u)
            {
                if (( spi_output_buffer[0] & 0x2u) == 0x2u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\neNVM_0 digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\neNVM_0 digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            if ((spi_input_buffer[0] & 0x4u) == 0x4u)
            {
                if (( spi_output_buffer[0] & 0x4u) == 0x4u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\neNVM_1 digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\neNVM_1 digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            if ((sec_ul & 0x2u) == 0x2u)
            {
                if (( spi_output_buffer[0] & 0x8u) == 0x8u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser security policies segment digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser security policies segment digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            dp_get_data(G4M_UPK1_ID,0u);
            if (return_bytes != 0u)
            {
                if (( spi_output_buffer[0] & 0x10u) == 0x10u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser key set 1 segment digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser key set 1 segment digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            dp_get_data(G4M_UPK2_ID,0u);
            if (return_bytes != 0u)
            {
                if (( spi_output_buffer[0] & 0x20u) == 0x20u)
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser key set 2 segment digest verification: PASS");
                    #endif
                }
                else
                {
                    #ifdef ENABLE_DISPLAY
                    dp_display_text("\r\nUser key set 2 segment digest verification: FAIL");
                    #endif
                    error_code = DPE_VERIFY_DIGEST_ERROR;
                }
            }
            if (( spi_output_buffer[0] & 0x40u) == 0x0u)
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\nFactory row and factory key segment digest verification: FAIL");
                #endif
                error_code = DPE_VERIFY_DIGEST_ERROR;
            }
        }
    }
    return;
}


#ifdef ENABLE_DISPLAY
void dp_G4M_device_info_action(void)
{
    dp_G4M_read_design_info();
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_read_prog_info();
        if (error_code == DPE_SUCCESS)
        {
            dp_G4M_read_bitstream_digest();
            if (error_code == DPE_SUCCESS)
            {
                dp_G4M_read_fsn();
                if (error_code == DPE_SUCCESS)
                {
                    dp_G4M_read_security();
                }
            }
        }
    }
    return;
}


void dp_G4M_read_design_info(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_READ_DESIGN_INFO;
    SPI_SCAN_in();
    dp_check_hwstatus(RXBUSY|TXBUSY);
    dp_read_shared_buffer(3u);
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nDesign Name: ");
        for (global_uchar1 = 2u; global_uchar1 < 32u; global_uchar1++)
        {
            dp_display_value(shared_buf[global_uchar1],CHR);
        }
        dp_display_text("\r\nChecksum: ");
        dp_display_array(shared_buf,2u,HEX);
        dp_display_text("\r\nDesign Info: ");
        dp_display_array(shared_buf,34u,HEX);
        dp_display_text("\r\nDESIGNVER: ");
        dp_display_array(&shared_buf[32],2u,HEX);
        dp_display_text("\r\nBACKLEVEL: ");
        dp_display_array(&shared_buf[34],2u,HEX);
    }
    return;
}

void dp_G4M_read_prog_info(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_READ_PROG_INFO;
    SPI_SCAN_out();
    
    
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nPROG_INFO: ");
        dp_display_array(spi_output_buffer,SPI_BUFFER_BYTE_SIZE,HEX);
        if ((spi_output_buffer[6] & 0x1) == 0u)
        {
            dp_display_text("\r\nVCC was programmed at 1.2V");
        }
        else
        {
            dp_display_text("\r\nVCC was programmed at 1.0v");
        }
        if  ( ((spi_output_buffer[8] & 0x3f) != 0u) && ((spi_output_buffer[8] & 0x3f) != 0x3fu) )
        {
            dp_display_text("\r\nAlgorithm Version: ");
            dp_display_value((spi_output_buffer[8] & 0x3f),DEC);
        }
        
        global_uchar1 = ((spi_output_buffer[8] >> 6) | (spi_output_buffer[9] << 2)) & 0xfu;
        dp_display_text("\r\nProgrammer code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((spi_output_buffer[10] >> 1)) & 0x3fu;
        dp_display_text("\r\nSoftware version code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((spi_output_buffer[10] >> 7) | (spi_output_buffer[11] << 1)) & 0x7u;
        dp_display_text("\r\nProgramming Software code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((spi_output_buffer[11] >> 2)) & 0x7u;
        dp_display_text("\r\nProgramming Interface Protocol code: ");
        dp_display_value(global_uchar1, DEC);
        
        global_uchar1 = ((spi_output_buffer[11] >> 5)) & 0x7u;
        dp_display_text("\r\nProgramming File Type code: ");
        dp_display_value(global_uchar1, DEC);
    }
    
    return;
}

void dp_G4M_read_fsn(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_READ_SN;
    SPI_SCAN_out();
    dp_display_text("\r\n=====================================================================");
    dp_display_text("\r\nDSN: ");
    dp_display_array(spi_output_buffer, SPI_BUFFER_BYTE_SIZE, HEX);
    dp_display_text("\r\n=====================================================================");
    return;
}

void dp_G4M_read_bitstream_digest(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_G4M_READ_DIGESTS;
    SPI_SCAN_in();
    dp_read_shared_buffer(6u);
    if (error_code == DPE_SUCCESS)
    {
        dp_display_text("\r\nBitstream Fabric Digest: ");
        dp_display_array(shared_buf,32u,HEX);
        dp_display_text("\r\nBitstream eNVM0 Digest: ");
        dp_display_array(&shared_buf[32],32u,HEX);
        dp_display_text("\r\nBitstream eNVM1 Digest: ");
        dp_display_array(&shared_buf[64],32u,HEX);
    }
    return;
}

#endif

void dp_read_shared_buffer(DPUCHAR ucNumOfBlocks)
{
    
    for (global_uchar1 = 0u; global_uchar1 < ucNumOfBlocks; global_uchar1++)
    {
        dp_flush_spi_buffers();
        spi_command_buffer = SPI_READ_BUFFER;
        spi_input_buffer[0] = global_uchar1;
        SPI_SCAN_out();
        
        for (global_uchar2 = 0;global_uchar2 < SPI_BUFFER_BYTE_SIZE; global_uchar2++)
        {
            shared_buf[global_uchar1*SPI_BUFFER_BYTE_SIZE + global_uchar2] = spi_output_buffer[global_uchar2];
        }
    }
    
    return;
}

void dp_G4M_perform_isc_enable(void)
{
    pgmmode_flag = TRUE;
    dp_flush_spi_buffers();
    spi_input_buffer[0] |= (G4M_ALGO_VERSION & 0x3fu);
    spi_input_buffer[2] |= (G4M_DIRECTC_VERSION & 0x3fu) << 1u;
    spi_input_buffer[2] |= (DIRECTC_PROGRAMMING & 0x7u) << 7u;
    spi_input_buffer[3] |= (DIRECTC_PROGRAMMING & 0x7u) >> 1u;
    spi_input_buffer[3] |= (JTAG_PROGRAMMING_PROTOCOL & 0x7u) << 2u;
    
    spi_command_buffer = SPI_ISC_ENABLE;
    SPI_SCAN_out();
    
    if ((spi_output_buffer[0] & 0x1) == 1)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nFailed to enter programming mode.");
        #endif
        error_code = DPE_INIT_FAILURE;
    }
    
    #ifdef ENABLE_DISPLAY
    
    dp_display_text("\r\nISC_ENABLE_RESULT: ");
    dp_display_array(spi_output_buffer,4u,HEX);
    
    /* Display CRCERR */
    global_uchar1 = spi_output_buffer[0] & 0x1u;
    dp_display_text("\r\nCRCERR: ");
    dp_display_value(global_uchar1,HEX);
    
    /* Display EDCERR */
    global_uchar1 = (spi_output_buffer[0] & 0x2u) >> 1u;
    dp_display_text("\r\nEDCERR: ");
    dp_display_value(global_uchar1,HEX);
    
    /* Display TEMPRANGE */
    global_uchar1 = (spi_output_buffer[0] >> 2) & 0x7u;
    dp_display_text("\r\nTEMPRANGE:");
    dp_display_value(global_uchar1,HEX);
    if (global_uchar1 == 0u)
    {
        dp_display_text("\r\nTEMPRANGE: COLD");
    }
    else if (global_uchar1 == 1u)
    {
        dp_display_text("\r\nTEMPRANGE: ROOM");
    }
    if (global_uchar1 == 2u)
    {
        dp_display_text("\r\nTEMPRANGE: HOT");
    }
    
    /* Display VPPRANGE */
    global_uchar1 = (spi_output_buffer[0] >> 5) & 0x7u;
    dp_display_text("\r\nVPPRANGE:");
    dp_display_value(global_uchar1,HEX);
    if (global_uchar1 == 0u)
    {
        dp_display_text("\r\nVPPRANGE: LOW");
    }
    else if (global_uchar1 == 1u)
    {
        dp_display_text("\r\nVPPRANGE: NOMINAL");
    }
    if (global_uchar1 == 2u)
    {
        dp_display_text("\r\nVPPRANGE: HIGH");
    }
    
    /* Display TEMP */
    dp_display_text("\r\nTEMP:");
    dp_display_value(spi_output_buffer[1],HEX);
    
    /* Display VPP */
    dp_display_text("\r\nVPP:");
    dp_display_value(spi_output_buffer[2],HEX);
    
    
    #endif
    
    return;
}

/* Enter programming mode */
void dp_G4M_initialize(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_MODE;
    SPI_SCAN_in();
    
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_read_security();
        if ((error_code == DPE_SUCCESS) && (sec_ul & 0x2))
        {
            dp_G4M_unlock_upk1();
            if (error_code == DPE_SUCCESS)
            {
                dp_G4M_unlock_upk2();
            }
        }
    }
    if (error_code == DPE_SUCCESS)
    {
        dp_G4M_perform_isc_enable();
    }
    
    return;
}


/* Function is used to exit programming mode */
void dp_G4M_exit(void)
{
    if (pgmmode_flag == TRUE)
    {
        dp_flush_spi_buffers();
        spi_command_buffer = SPI_ISC_DISABLE;
        SPI_SCAN_in();
        dp_delay(100);
    }
    #ifdef ENABLE_DISPLAY
    dp_G4M_read_fsn();
    #endif
    
    spi_command_buffer = SPI_RELEASE;
    SPI_SCAN_in();
    
    return;
}

void dp_set_mode(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_G4M_FRAME_INIT;
    spi_input_buffer[0] = pgmmode;
    SPI_SCAN_in();
    
    return;
}


void dp_G4M_process_data(DPUCHAR BlockID)
{
    DataIndex = 0u;
    /* Global unit1 is used to hold the number of components */
    /* Loop through the number of components */
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\n");
    #endif
    
    
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
        global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
        global_ulong1 &= 0x3FFFFFu;
        
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nProcessing component ");
        dp_display_value(global_uint2,DEC);
        dp_display_text(". Please wait...");
        #endif
        
        for (global_ulong2 = 1u; global_ulong2 <= global_ulong1; global_ulong2++)
        {
            #ifdef ENABLE_DISPLAY
            new_progress = (global_ulong2 * 100u / global_ulong1);
            if (new_progress != old_progress)
            {
                dp_report_progress(new_progress);
                old_progress = new_progress;
            }
            #endif
            
            spi_command_buffer = SPI_G4M_FRAME_DATA;
            dp_get_and_shift_in(BlockID, SPI_BUFFER_BIT_SIZE, DataIndex);
            dp_G4M_get_data_status();
            
            if ( error_code != DPE_SUCCESS )
            {
                #ifdef ENABLE_DISPLAY
                dp_display_text("\r\ncomponentNo: ");
                dp_display_value(global_uint2, DEC);
                dp_display_text("\r\nblockNo: ");
                dp_display_value(global_ulong2, DEC);
                dp_display_text("\r\nDATA_STATUS_RESULT: ");
                dp_display_array(spi_output_buffer,4u,HEX);
                dp_display_text("\r\nERRORCODE: ");
                dp_display_value((spi_output_buffer[1]>>3u) & 0x1fu,HEX);
                dp_display_text("\r\nAUTHERRCODE: ");
                dp_display_value(spi_output_buffer[2],HEX);
                #endif
                global_uint2 = global_uint1;
                break;
            }
            DataIndex += SPI_BUFFER_BIT_SIZE;
        }
    }
    
    return;
}

void dp_G4M_get_data_status(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_G4M_FRAME_STATUS;
    SPI_SCAN_out();
    
    if (spi_output_buffer[1] & 0xcu)
    {
        error_code = DPE_PROCESS_DATA_ERROR;
    }
    return;
}

void dp_G4M_read_security(void)
{
    dp_G4M_query_security();
    if ((error_code == DPE_SUCCESS) && (security_queried == FALSE) )
    {
        dp_G4M_unlock_dpk();
        if (error_code == DPE_SUCCESS)
        {
            dp_G4M_query_security();
            #ifdef ENABLE_DISPLAY
            if ((error_code == DPE_SUCCESS) && (security_queried == FALSE) )
            {
                dp_display_text("\r\nWarning: Security cannot be read even after unlocking debug pass key.");
            }
            #endif
        }
    }
    sec_ul = shared_buf[4] >> 2u;
    
    return;
}

void dp_G4M_query_security(void)
{
    dp_flush_spi_buffers();
    spi_command_buffer = SPI_QUERY_SECURITY;
    SPI_SCAN_in();
    dp_read_shared_buffer(3u);
    
    
    for (global_uchar1 = 0u; global_uchar1 < 40u; global_uchar1++)
    {
        if (shared_buf[global_uchar1] != 0u)
        {
            security_queried = TRUE;
            break;
        }
    }
    
    #ifdef ENABLE_DISPLAY
    if (security_queried == TRUE)
    {
        dp_display_text("\r\n--- Security locks and configuration settings ---\r\n");
        dp_display_array(shared_buf,42u,HEX);
        if ( ((shared_buf[8] & 0x1cu) == 0u) && ((shared_buf[9] & 0x4u) == 0u) )
        {
            dp_display_text("\r\nFPGA Array is programmed and enabled.");
        }
        else
        {
            dp_display_text("\r\nFPGA Array is not enabled.");
        }
    }
    #endif
    return;
}


void dp_G4M_unlock_dpk(void)
{
    dp_get_data(G4M_DPK_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: DPK data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_dpk();
        if (error_code == DPE_SUCCESS)
        {
            dp_flush_spi_buffers();
            spi_command_buffer = SPI_MACTH_DKEY;
            SPI_SCAN_out();
        }
        if ((error_code != DPE_SUCCESS) || ((spi_output_buffer[0] & 0x1u) != 0x1u) )
        {
            #ifdef ENABLE_DISPLAY
            dp_display_text("\r\nFailed to unlock debug pass key.");
            #endif
            error_code = DPE_MATCH_ERROR;
        }
        #ifdef ENABLE_DISPLAY
        else
        {
            dp_display_text("\r\nDebug security (DPK) is unlocked.");
        }
        #endif
    }
    return;
}

void dp_G4M_unlock_upk1(void)
{
    dp_get_data(G4M_UPK1_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: UPK1 data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_upk1();
        if (error_code == DPE_SUCCESS)
        {
            dp_flush_spi_buffers();
            spi_command_buffer = SPI_MACTH_UKEY;
            SPI_SCAN_out();
        }
        if ((error_code != DPE_SUCCESS) || ((spi_output_buffer[0] & 0x1u) != 0x1u) )
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

void dp_G4M_unlock_upk2(void)
{
    dp_get_data(G4M_UPK2_ID,0u);
    if (return_bytes == 0u)
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nWarning: UPK2 data is missing.");
        #endif
    }
    else
    {
        dp_G4M_load_upk2();
        if (error_code == DPE_SUCCESS)
        {
            dp_flush_spi_buffers();
            spi_command_buffer = SPI_MACTH_VKEY;
            SPI_SCAN_out();
        }
        if ((error_code != DPE_SUCCESS) || ((spi_output_buffer[0] & 0x1u) != 0x1u) )
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

void dp_G4M_load_dpk(void)
{
    spi_command_buffer = SPI_LEY_LO;
    dp_get_and_shift_in(G4M_DPK_ID, SPI_BUFFER_BIT_SIZE, 0u);
    spi_command_buffer = SPI_LEY_HI;
    dp_get_and_shift_in(G4M_DPK_ID, SPI_BUFFER_BIT_SIZE, SPI_BUFFER_BIT_SIZE);
    
    return;
}

void dp_G4M_load_upk1(void)
{
    spi_command_buffer = SPI_LEY_LO;
    dp_get_and_shift_in(G4M_UPK1_ID, SPI_BUFFER_BIT_SIZE, 0u);
    spi_command_buffer = SPI_LEY_HI;
    dp_get_and_shift_in(G4M_UPK1_ID, SPI_BUFFER_BIT_SIZE, SPI_BUFFER_BIT_SIZE);
    
    return;
}

void dp_G4M_load_upk2(void)
{
    spi_command_buffer = SPI_LEY_LO;
    dp_get_and_shift_in(G4M_UPK2_ID, SPI_BUFFER_BIT_SIZE, 0u);
    spi_command_buffer = SPI_LEY_HI;
    dp_get_and_shift_in(G4M_UPK2_ID, SPI_BUFFER_BIT_SIZE, SPI_BUFFER_BIT_SIZE);
    
    return;
}

void dp_G4M_setup_eNVM(DPUCHAR BlockID)
{
    envm_only_flag = FALSE;
    DataIndex = 0u;
    
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
        global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
        global_ulong1 &= 0x3FFFFFu;
        
        /* Determine component types in bitstream */
        global_uchar1 = (DPUCHAR) dp_get_bytes(BlockID,48u+DataIndex/8u,1u);
        if ( (global_uchar1 == G4M_FPGA) || (global_uchar1 == G4M_KEYS) )
        {
            envm_only_flag = FALSE;
            break;
        }
        else if(global_uchar1 == G4M_ENVM)
        {
            envm_only_flag = TRUE;
        }
        DataIndex += (G4M_FRAME_BIT_LENGTH * global_ulong1);
    }
    
    
    
    if (envm_only_flag == TRUE)
    {
        dp_MSS_ADDR_CONFIG();
        if (error_code == DPE_SUCCESS)
        {
            dp_MSS_RD_DATA_SETUP();
            if (error_code == DPE_SUCCESS)
            {
                dp_MSS_ADDR_CONFIG();
                if (error_code == DPE_SUCCESS)
                {
                    dp_MSS_WR_DATA_SETUP();
                }
            }
        }
    }
    return;
}

void dp_G4M_post_setup_eNVM(void)
{
    if (envm_only_flag == TRUE)
    {
        dp_MSS_ADDR_CONFIG();
        if (error_code == DPE_SUCCESS)
        {
            dp_MSS_WR_DATA();
        }
    }
    return;
}

void dp_MSS_ADDR_CONFIG(void)
{
    dp_flush_spi_buffers();
    spi_input_buffer[0] = 0x0cu;
    spi_input_buffer[1] = 0x80u;
    spi_input_buffer[2] = 0x03u;
    spi_input_buffer[3] = 0x40u;
    spi_input_buffer[4] = 0x02u;
    spi_input_buffer[5] = 0x00u;
    spi_input_buffer[6] = 0x00u;
    spi_input_buffer[7] = 0x00u;
    
    spi_command_buffer = SPI_MSSADDR;
    SPI_SCAN_in();
    
    return;
}

void dp_MSS_RD_DATA_SETUP(void)
{
    DPUCHAR index;
    
    dp_flush_spi_buffers();
    spi_input_buffer[0] = 0x04u;
    
    spi_command_buffer = SPI_MSSRD;
    SPI_SCAN_in();
    dp_check_hwstatus(RXBUSY|TXBUSY);
    
    dp_read_shared_buffer(1u);
    
    for (index = 0u; index < 4u; index++)
    {
        SYSREG_TEMP[index] = shared_buf[index];
    }
    
    return;
}

void dp_MSS_WR_DATA_SETUP(void)
{
    DPUCHAR index;
    dp_flush_spi_buffers();
    
    for(index = 0u; index < 4u; index++)
    {
        spi_input_buffer[index] = SYSREG_TEMP[index];
    }
    spi_input_buffer[0] |= 0xe0u;
    spi_input_buffer[1] |= 0x1fu;
    spi_command_buffer = SPI_MSSWR;
    SPI_SCAN_in();
    
    return;
}

void dp_MSS_WR_DATA(void)
{
    DPUCHAR index;
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nRestore user FREQRNG setting ");
    dp_display_array(SYSREG_TEMP,4u,HEX);
    dp_display_text("\r\n");
    #endif
    dp_flush_spi_buffers();
    
    for(index = 0u; index < 4u; index++)
    {
        spi_input_buffer[index] = SYSREG_TEMP[index];
    }
    
    spi_command_buffer = SPI_MSSWR;
    SPI_SCAN_in();
    
    return;
}

/* global_uint2 is iCurComponentNo
global_uint1 is iCurNumOfComponents 
*/
void dp_G4M_prepare_bitstream(void)
{
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\nPreparing bitstream...");
    #endif
    pgmmode = 0x2u;
    dp_set_mode();
    
    if ((error_code == DPE_SUCCESS) && (sec_ul & 0x2u))
    {
        if ((DPUINT)dp_get_bytes(Header_ID,G4M_VERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) != 0u)
        {
            /* Both global_unit1 and global_unit2 have DATASIZE, ENVMDATASIZE and ENVMVERIFYDATASIZE; */
            global_uint2 = (DPUINT)dp_get_bytes(Header_ID,G4M_DATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
            (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) +
            (DPUINT)dp_get_bytes(Header_ID,G4M_ENVMVERIFYDATASIZE_OFFSET,G4M_DATASIZE_BYTE_LENGTH) + 1u;
            global_uint1 = global_uint2 + 1u;
            dp_G4M_process_predata(G4M_VerifyDataStream_ID);
        }
        else
        {
            global_uint2 = 1u;
            global_uint1 = 2u;
            dp_G4M_process_predata(G4M_datastream_ID);
        }
    }
    else
    {
        global_uint2 = 1u;
        global_uint1 = 2u;
        dp_G4M_process_predata(G4M_datastream_ID);
    }
    return;
}

void dp_G4M_process_predata(DPUCHAR BlockID)
{
    DataIndex = 0u;  
    /* Global unit1 is used to hold the number of components */
    /* Loop through the number of components */
    #ifdef ENABLE_DISPLAY
    dp_display_text("\r\n");
    #endif
    
    
    for (; global_uint2 <= global_uint1; global_uint2++)
    {
        /* get the number of blocks */
        /* Global ulong1 is used to hold the number of blocks within the components */
        if (global_uint2 != global_uint1)
        {
            global_ulong1 = dp_get_bytes(G4M_NUMBER_OF_BLOCKS_ID,(DPULONG)(((global_uint2 - 1u) * 22u) / 8u),4u);
            global_ulong1 >>= ((global_uint2 - 1U)* 22u) % 8u;
            global_ulong1 &= 0x3FFFFFu;
        }
        else
        {
            global_ulong1 = 91u;
        }
        
        spi_command_buffer = SPI_G4M_FRAME_DATA;
        dp_get_and_shift_in(BlockID, SPI_BUFFER_BIT_SIZE, DataIndex);
        dp_G4M_get_data_status();
        
        DataIndex += SPI_BUFFER_BIT_SIZE;
    }
    
    return;
}


#endif

/*   *************** End of File *************** */
