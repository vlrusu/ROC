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
/*  Module:        dpalg.c                                                  */
/*                                                                          */
/*  Description:   Contains initialization and device ID checking functions */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"
#include "dpalg.h"
#include "dpcom.h"
#include "dputil.h"
#include "dpDUTspi.h"

#include "dpG5alg.h"
#include "dpcom.h"

/* DirectC Variables */
DPUCHAR error_code;
DPUCHAR action_performed;
DPUCHAR Action_code;

DPULONG device_ID;      /* Holds the device ID */
DPUCHAR device_rev;     /* Holds the device revision */
DPUCHAR device_family;  /* Read from the data file */

#ifdef ENABLE_DISPLAY
DPULONG old_progress = 0;
DPULONG new_progress;
#endif

DPULONG DataIndex;
DPUCHAR pgmmode;
DPUCHAR pgmmode_flag;	/* Used to identify if the ISC_ENABLE was executed */

DPUCHAR dp_top (void)
{
    dp_init_com_vars();
    dp_init_vars();
    #ifdef ENABLE_G4M_SUPPORT
    if (device_family == G4M_DEVICE_FAMILY)
    {
        dp_G4M_read_idcode();
        if ( (error_code == DPE_SUCCESS) && ( (device_ID & FAMILY_ID_MASK) == G4M_FAMILY_ID) )
        {
            dp_top_g4m();
        }
    }
    #endif
    #ifdef ENABLE_G5M_SUPPORT
    if (device_family == G5M_DEVICE_FAMILY)
    {
        dp_G5M_read_idcode();
        if ( (error_code == DPE_SUCCESS) && ( (device_ID & FAMILY_ID_MASK) == G5M_FAMILY_ID) )
        {
            dp_top_g5m();
        }
    }
    #endif
    
    if (action_performed == FALSE)
    {
        error_code = DPE_ACTION_NOT_FOUND;
    }
    return error_code;
}

void dp_init_vars(void)
{
    error_code = DPE_SUCCESS;
    action_performed = FALSE;
    device_ID = 0u;
    device_rev = 0u;
    dp_flush_spi_buffers();
    
    return;
}

#ifdef ENABLE_DISPLAY
void dp_read_idcode_action(void)
{
    dp_display_text("\r\nActID = ");
    dp_display_value(device_ID,HEX);
    return;
}
#endif

/* Checking device ID function.  ID is already read in dpalg.c */
void dp_check_device_ID(void)
{
    /* DataIndex is a variable used for loading the array data but not used now.
    * Therefore, it can be used to store the Data file ID for */
    DataIndex = dp_get_bytes(GXM_Header_ID,GXM_ID_OFFSET,GXM_ID_BYTE_LENGTH);
    
    
    global_ulong1 = dp_get_bytes(GXM_Header_ID,GXM_ID_MASK_OFFSET,4U);
    
    device_ID &= global_ulong1;
    DataIndex &= global_ulong1;
    /* Identifying target device and setting its parms */
    
    if ( device_ID == DataIndex )
    {
        #ifdef ENABLE_DISPLAY
        dp_display_text("\r\nActID = ");
        dp_display_value(device_ID,HEX);
        dp_display_text(" ExpID = ");
        dp_display_value(DataIndex,HEX);
        dp_display_text("\r\nDevice Rev = ");
        dp_display_value(device_rev,HEX);
        #endif
    }
    else
    {
        error_code = DPE_IDCODE_ERROR;
    }
    
    return;
}

/*   *************** End of File *************** */

