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
/*  Module:         dpG4alg.h                                               */
/*                                                                          */
/*  Description:    Contains function prototypes                            */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"

#ifndef INC_DPG4ALG_H
#define INC_DPG4ALG_H

/*
* Data block ID definitions
*/
#define G4M_ALGO_VERSION                     1u
#define G4M_DIRECTC_VERSION                  0u

#define G4M_Header_ID                        0u
#define G4M_USER_INFO_ID                     1u
#define G4M_ACT_UROW_DESIGN_NAME_ID          2u
#define G4M_BsrPattern_ID                    3u
#define G4M_BsrPatternMask_ID                4u
#define G4M_NUMBER_OF_BLOCKS_ID              5u
#define G4M_UPK1_ID                          6u
#define G4M_UPK2_ID                          7u
#define G4M_datastream_ID                    8u
#define G4M_erasedatastream_ID               9u
#define G4M_VerifyDataStream_ID              10u
#define G4M_EnvmDataStream_ID                11u
#define G4M_EnvmVerifyDataStream_ID          12u

#define G4M_DPK_ID                           13u


#define G4M_DEVICE_FAMILY_OFFSET             36u
#define G4M_DEVICE_FAMILY_BYTE_LENGTH        1u
#define G4M_ID_OFFSET                        37u
#define G4M_ID_BYTE_LENGTH                   4u
#define G4M_ID_MASK_OFFSET                   41u
#define G4M_ID_MASK_BYTE_LENGTH              4u
#define G4M_SILSIG_OFFSET                    45u
#define G4M_SILSIG_BYTE_LENGTH               4u
#define G4M_CHECKSUM_OFFSET                  49u
#define G4M_CHECKSUM_BYTE_LENGTH             2u
#define G4M_NUMOFBSRBITS_OFFSET              51u
#define G4M_NUMOFBSRBITS_BYTE_LENGTH         2u
#define G4M_NUMOFCOMPONENT_OFFSET            53u
#define G4M_NUMOFCOMPONENT_BYTE_LENGTH       2u
#define G4M_DATASIZE_OFFSET                  55u
#define G4M_DATASIZE_BYTE_LENGTH             2u
#define G4M_ERASEDATASIZE_OFFSET             57u
#define G4M_ERASEDATASIZE_BYTE_LENGTH        2u
#define G4M_VERIFYDATASIZE_OFFSET            59u
#define G4M_VERIFYDATASIZE_BYTE_LENGTH       2u
#define G4M_ENVMDATASIZE_OFFSET              61u
#define G4M_ENVMDATASIZE_BYTE_LENGTH         2u
#define G4M_ENVMVERIFYDATASIZE_OFFSET        63u
#define G4M_ENVMVERIFYDATASIZE_BYTE_LENGTH   2u

/* SPI opcode Definitions */
#define MICROSEMI_ID          0x1CFu
#define RXBUSY                0x1u
#define TXBUSY                0x2u
#define SPIERR                0x8u

#define G4M_SPI_HWSTATUS      0xffu
#define SPI_READ_DATA         0x5u

#define SPI_READ_IDCODE       0x21u
#define SPI_READ_USERCODE     0x22u
#define SPI_READ_DESIGN_INFO  0x1du
#define SPI_READ_PROG_INFO    0x1eu
#define SPI_READ_BUFFER       0x30u

#define SPI_ISC_ENABLE        0x14u
#define SPI_ISC_DISABLE       0x15u
#define SPI_RELEASE           0x23u
#define SPI_G4M_FRAME_INIT    0x02u
#define SPI_QUERY_SECURITY    0x20u
#define SPI_MACTH_UKEY        0x1au
#define SPI_MACTH_VKEY        0x24u
#define SPI_MACTH_DKEY        0x25u
#define SPI_LEY_LO            0x12u
#define SPI_LEY_HI            0x13u

#define SPI_G4M_FRAME_DATA    0x1u
#define SPI_G4M_FRAME_STATUS  0x4u
#define SPI_READ_SN           0x18u
#define SPI_MODE              0x06u
#define SPI_MSSADDR           0x2bu
#define SPI_MSSRD             0x2cu
#define SPI_MSSWR             0x2du
#define SPI_CHECK_DIGESTS		0x2fu
#define SPI_G4M_READ_DIGESTS  0x2eu

#define M2S090_ID						0x0F8071CFu
#define M2S150_ID						0x0F8061CFu

#define COMPONENT_TYPE_IN_HEADER_BYTE	48u
#define ENVM_MODULE_ID_IN_HEADER_BYTE	301u
#define G4M_FPGA              1u
#define G4M_KEYS              2u
#define G4M_ENVM              3u

DPUCHAR dp_top_g4m (void);
void dp_G4M_init_vars(void);
void dp_G4M_check_action(void);
void dp_G4M_perform_action (void);
void dp_G4M_read_idcode(void);

/* Supported Actions */
void dp_G4M_device_info_action(void);
void dp_G4M_erase_action(void);
void dp_G4M_program_action(void);
void dp_G4M_verify_action(void);
void dp_G4M_enc_data_authentication_action(void);
void dp_G4M_verify_digest_action(void);


void dp_G4M_read_design_info(void);
void dp_G4M_read_prog_info(void);
void dp_G4M_read_fsn(void);
void dp_G4M_read_bitstream_digest(void);
void dp_G4M_read_security(void);
void dp_G4M_query_security(void);
void dp_G4M_unlock_dpk(void);
void dp_G4M_unlock_upk1(void);
void dp_G4M_unlock_upk2(void);
void dp_G4M_load_dpk(void);
void dp_G4M_load_upk1(void);
void dp_G4M_load_upk2(void);
void dp_read_shared_buffer(DPUCHAR ucNumOfBlocks);
void dp_set_pgm_mode(void);
void dp_G4M_perform_isc_enable(void);
void dp_G4M_process_data(DPUCHAR BlockID);
void dp_G4M_process_predata(DPUCHAR BlockID);
void dp_G4M_prepare_bitstream(void);
void dp_G4M_get_data_status(void);
void dp_G4M_setup_eNVM(DPUCHAR BlockID);
void dp_G4M_post_setup_eNVM(void);
void dp_MSS_ADDR_CONFIG(void);
void dp_MSS_RD_DATA_SETUP(void);
void dp_MSS_WR_DATA_SETUP(void);
void dp_MSS_WR_DATA(void);

/* Initialization functions */
void dp_G4M_initialize(void);
void dp_G4M_exit(void);
void dp_set_mode(void);

/* Erase function */
void dp_G4M_erase(void);

#endif /* INC_DPG4ALG_H */

/*   *************** End of File *************** */

