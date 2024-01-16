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
/*  Module:         dpalg.h                                                 */
/*                                                                          */
/*  Description:    Contains function prototypes                            */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"

#ifndef INC_DPALG_H
#define INC_DPALG_H

#define GXM_Header_ID                        0u
#define GXM_USER_INFO_ID                     1u
#define GXM_ACT_UROW_DESIGN_NAME_ID          2u
#define GXM_BsrPattern_ID                    3u
#define GXM_BsrPatternMask_ID                4u
#define GXM_NUMBER_OF_BLOCKS_ID              5u
#define GXM_UPK1_ID                          6u
#define GXM_UPK2_ID                          7u
#define GXM_datastream_ID                    8u
#define GXM_erasedatastream_ID               9u
#define GXM_VerifyDataStream_ID              10u
#define GXM_EnvmDataStream_ID                11u
#define GXM_EnvmVerifyDataStream_ID          12u

#define GXM_DPK_ID                           13u

#define GXM_DEVICE_FAMILY_OFFSET             36u
#define GXM_DEVICE_FAMILY_BYTE_LENGTH        1u
#define GXM_ID_OFFSET                        37u
#define GXM_ID_BYTE_LENGTH                   4u
#define GXM_ID_MASK_OFFSET                   41u
#define GXM_ID_MASK_BYTE_LENGTH              4u
#define GXM_SILSIG_OFFSET                    45u
#define GXM_SILSIG_BYTE_LENGTH               4u
#define GXM_CHECKSUM_OFFSET                  49u
#define GXM_CHECKSUM_BYTE_LENGTH             2u
#define GXM_NUMOFBSRBITS_OFFSET              51u
#define GXM_NUMOFBSRBITS_BYTE_LENGTH         2u
#define GXM_NUMOFCOMPONENT_OFFSET            53u
#define GXM_NUMOFCOMPONENT_BYTE_LENGTH       2u
#define GXM_DATASIZE_OFFSET                  55u
#define GXM_DATASIZE_BYTE_LENGTH             2u
#define GXM_ERASEDATASIZE_OFFSET             57u
#define GXM_ERASEDATASIZE_BYTE_LENGTH        2u
#define GXM_VERIFYDATASIZE_OFFSET            59u
#define GXM_VERIFYDATASIZE_BYTE_LENGTH       2u
#define GXM_ENVMDATASIZE_OFFSET              61u
#define GXM_ENVMDATASIZE_BYTE_LENGTH         2u
#define GXM_ENVMVERIFYDATASIZE_OFFSET        63u
#define GXM_ENVMVERIFYDATASIZE_BYTE_LENGTH   2u

#define DIRECTC_PROGRAMMING                  2u
#define SPI_PROGRAMMING_PROTOCOL             2u
#define JTAG_PROGRAMMING_PROTOCOL            3u

#define FAMILY_ID_MASK        0xFFF0FFFu
#define G4M_FAMILY_ID         0xF8001CFu
#define G5M_FAMILY_ID         0xF8101CFu
#define G4M_DEVICE_FAMILY     0x5u
#define G5M_DEVICE_FAMILY     0x7u


extern DPULONG DataIndex;
extern DPUCHAR error_code;
extern DPUCHAR Action_code; /* used to hold the action codes as */
extern DPUCHAR action_performed;
extern DPUCHAR Target_family;
extern DPULONG device_ID;  /* Holds the device ID */
extern DPUCHAR device_rev; /* Holds the device revision */
extern DPUCHAR device_family;    /* Read from the data file */

extern DPULONG DataIndex;
extern DPUCHAR pgmmode;
extern DPUCHAR pgmmode_flag;	/* Used to identify if the ISC_ENABLE was executed */
#ifdef ENABLE_DISPLAY
extern DPULONG old_progress;
extern DPULONG new_progress;
#endif


DPUCHAR dp_top (void);
void dp_init_vars(void);
void dp_read_idcode_action(void);
void dp_check_device_ID(void);

#endif /* INC_DPG4ALG_H */

/*   *************** End of File *************** */

