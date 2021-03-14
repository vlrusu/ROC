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
/*  Module:         dpG5spi.c                                               */
/*                                                                          */
/*  Description:    Contains PolarFire SPI specific functions               */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"
#include "dpalg.h"
#include "dpG5alg.h"
#include "dpG5spi.h"
#include "dpcom.h"
#include "dputil.h"
#include "CoreSPI/core_spi.h"

#ifdef ENABLE_G5M_SUPPORT
void dp_G5M_check_hwstatus(DPUCHAR mask)
{
    DPULONG timeout;
    DPUCHAR spi_hwcmd;
    timeout = 0xffffffffu;
    do_SPI_SCAN_in(G5M_SPI_HWSTATUS, 0u, (DPUCHAR*)(DPUCHAR*)DPNULL);
    while (timeout != 0)
    {
    	spi_hwcmd = G5M_SPI_HWSTATUS;
        do_SPI_SCAN_out(8u, &spi_hwcmd, 8u,&spi_hwstatus_buffer );
        
 //       if ((spi_hwstatus_buffer & mask) == (G5_READY_BIT & mask))

 //       {
            // Silicon bug. SAR 803711 Need to add one more read to inspect Bits 7:2
        	spi_hwcmd = G5M_SPI_HWSTATUS;
            do_SPI_SCAN_out(8u, &spi_hwcmd, 8u, &spi_hwstatus_buffer);
            break;
//        }
        timeout--;
    }
    if (timeout == 0u)
    {
        #ifdef ENABLE_DISPLAY
//    	UART_polled_tx_string( &g_uart,"\r\nError: SPI polling timeout encountered.");
    	dp_display_text("\r\nError: SPI polling timeout encountered.");
        #endif
        error_code = DPE_POLL_ERROR;
    }
    
//    if(spi_hwstatus_buffer & G5_SPI_VIOLATION_BIT)
//    {
//        #ifdef ENABLE_DISPLAY
//        dp_display_text("\r\nSPI VIOLATION...");
//        #endif
//        error_code = DPE_POLL_ERROR;
//    }
//    if(spi_hwstatus_buffer & G5_SPIERR_BIT)
//    {
//        #ifdef ENABLE_DISPLAY
//        dp_display_text("\r\nSPI Error...");
//        #endif
//        error_code = DPE_POLL_ERROR;
//    }
//
    return;
}

void G5M_SPI_SCAN_in(DPUCHAR total_dat_bits)
{
    dp_G5M_check_hwstatus(G5_BUSY_BIT);
    do_SPI_SCAN_in(spi_command_buffer, total_dat_bits, spi_input_buffer);
    
    return;
}

void G5M_SPI_SCAN_out(DPUCHAR total_dat_bits_in, DPUCHAR total_dat_bits_out)
{
    G5M_SPI_SCAN_in(total_dat_bits_in);
    dp_G5M_check_hwstatus(G5_BUSY_BIT | G5_READY_BIT);
    dp_flush_spi_buffers();
    spi_command_buffer = G5M_SPI_READ_DATA;
    do_SPI_SCAN_out(SPI_COMMAND_BIT_SIZE, &spi_command_buffer,total_dat_bits_out,spi_output_buffer);
    
    return;
}


void dp_G5M_get_and_shift_in(DPUCHAR Variable_ID,DPUINT total_bits_to_shift, DPULONG start_bit_index)
{
    requested_bytes =  total_bits_to_shift / 8u;
    page_buffer_ptr = dp_get_data(Variable_ID,start_bit_index);
    if (return_bytes >= requested_bytes )
    {
        return_bytes = requested_bytes;
    }
    #ifdef ENABLE_DISPLAY
    else
    {
        dp_display_text("\r\nError: not enough bytes returned in dp_get_and_shift_in.");
    }
    #endif
    dp_G5M_check_hwstatus(G5_BUSY_BIT);
    do_SPI_SCAN_in(spi_command_buffer, SPI_BUFFER_BIT_SIZE, page_buffer_ptr);
    
    return;
}

#endif

/*   *************** End of File *************** */
