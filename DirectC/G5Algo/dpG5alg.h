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
/*  Module:         dpG5alg.h                                               */
/*                                                                          */
/*  Description:    Contains function prototypes.                           */
/*                                                                          */
/* ************************************************************************ */

#include "dpuser.h"

#ifndef INC_DPG5ALG_H
#define INC_DPG5ALG_H

/*
* Data block ID definitions
*/
#define G5M_ALGO_VERSION            1u
#define G5M_DIRECTC_VERSION         0u

/* SPI opcode Definitions */
#define MICROSEMI_ID                0x1CFu
#define RXBUSY                      0x1u
#define TXBUSY                      0x2u
#define SPIERR                      0x8u

#define SPI_HWSTATUS                0xffu
#define SPI_READ_DATA               0x5u

#define SPI_READ_IDCODE             0x21u
#define SPI_READ_USERCODE           0x22u
#define SPI_READ_DESIGN_INFO        0x1du
#define SPI_READ_PROG_INFO          0x1eu
#define SPI_READ_BUFFER             0x30u

#define SPI_ISC_ENABLE              0x14u
#define SPI_ISC_DISABLE             0x15u
#define SPI_RELEASE                 0x23u
#define SPI_G4M_FRAME_INIT          0x02u
#define SPI_QUERY_SECURITY          0x20u
#define SPI_MACTH_UKEY              0x1au
#define SPI_MACTH_VKEY              0x24u
#define SPI_MACTH_DKEY              0x25u
#define SPI_LEY_LO                  0x12u
#define SPI_LEY_HI                  0x13u

#define SPI_G4M_FRAME_DATA          0x1u
#define SPI_G4M_FRAME_STATUS        0x4u
#define SPI_READ_SN                 0x18u
#define SPI_MODE                    0x06u
#define SPI_MSSADDR                 0x2bu
#define SPI_MSSRD                   0x2cu
#define SPI_MSSWR                   0x2du

#define G4M_FPGA                    1u
#define G4M_KEYS                    2u
#define G4M_ENVM                    3u


#define G5M_UL_USER_KEY1            0x2u
#define G5M_UL_USER_KEY2            0x4u

#define G5_BUSY_BIT                 0x1u
#define G5_READY_BIT                0x2u
#define G5_SPI_VIOLATION_BIT        0x4u
#define G5_SPIERR_BIT               0x8u


#define G5M_SPI_HWSTATUS            0x00u
#define G5M_SPI_READ_DATA           0x01u
#define G5M_SPI_RELEASE             0x23u
#define G5M_SPI_LOOPBACK            0x24u

#define G5M_SPI_READ_IDCODE         0x0Fu
#define G5M_SPI_READ_USERCODE       0x0Eu
#define G5M_SPI_READ_SN             0xF0u
#define G5M_SPI_ISC_DISABLE         0x0Cu
#define G5M_SPI_QUERY_SECURITY      0xB8u
#define G5M_SPI_READ_BUFFER         0xF2u
#define G5M_SPI_LEY_LO              0xEBu
#define G5M_SPI_LEY_HI              0xECu

#define G5M_UNLOCK_USER_PASSCODE1   0xA8u
#define G5M_UNLOCK_USER_PASSCODE2   0xAAu

#define G5M_SPI_ISC_ENABLE          0x0Bu
#define G5M_SPI_ISC_DISABLE         0x0Cu
#define G5M_SPI_G4M_FRAME_INIT      0xAEu
#define G5M_SPI_G4M_FRAME_DATA      0xEEu
#define G5M_SPI_G4M_FRAME_STATUS    0xD8u

#define G5M_SPI_READ_DESIGN_INFO    0xA6u
#define G5M_SPI_READ_DEBUG_INFO     0xE7u
#define G5M_SPI_VERSION             0xCDu
#define G5M_CHECK_DIGESTS				0xbcu

#define G5M_UL_EXTERNAL_DIGEST_CHECK	0x4u

DPUCHAR dp_top_g5m (void);
void dp_G5M_init_vars(void);
void dp_G5M_check_action(void);
void dp_G5M_perform_action (void);
void dp_G5M_read_idcode(void);


/* Supported Actions */
void dp_G5M_read_idcode(void);
void dp_G5M_device_info_action(void);
void dp_G5M_erase_action(void);
void dp_G5M_program_action(void);
void dp_G5M_verify_action(void);
void dp_G5M_enc_data_authentication_action(void);
void dp_G5M_verify_digest_action(void);

void dp_G5M_read_user_code(void);

void dp_G5M_read_design_info(void);
void dp_G5M_read_debug_info(void);
void dp_G4M_read_prog_info(void);
void dp_G5M_read_version_code(void);
void dp_G5M_read_fsn(void);
void dp_G5M_query_security(void);
void dp_G4M_unlock_dpk(void);
void dp_G5M_unlock_upk1(void);
void dp_G5M_unlock_upk2(void);
void dp_G4M_load_dpk(void);
void dp_G5M_load_upk1(void);
void dp_G5M_load_upk2(void);
void dp_G5M_read_shared_buffer(DPUCHAR ucNumOfBlocks);
void dp_set_pgm_mode(void);
void dp_G5M_perform_isc_enable(void);
void dp_G5M_process_data(DPUCHAR BlockID);
void dp_G5M_get_data_status(void);

/* Initialization functions */
void dp_G5M_initialize(void);
void dp_G5M_exit(void);
void dp_G5M_set_mode(void);


/* Erase function */
void dp_G4M_erase(void);

#endif /* INC_DPG5ALG_H */

/*   *************** End of File *************** */

