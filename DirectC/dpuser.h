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
/*  Module:         dpuser.h                                                */
/*                                                                          */
/*  Description:    dpuser specific file                                    */
/*                  users should define their own functions                 */
/*                                                                          */
/* ************************************************************************ */

#ifndef INC_DPUSER_H
#define INC_DPUSER_H

#include "../utils.h"
/* Compiler switches */
#define ENABLE_DEBUG
#define ENABLE_DISPLAY
#define PERFORM_CRC_CHECK
//#define ENABLE_G4M_SUPPORT
#define ENABLE_G5M_SUPPORT

#define USE_PAGING
#define ENABLE_UART_HOSTLOADER

/*************** End of compiler switches ***********************************/



/***********************************************/
/* DPCHAR    -- 8-bit Windows (ANSI) character */
/*              i.e. 8-bit signed integer      */
/* DPINT     -- 16-bit signed integer          */
/* DPLONG    -- 32-bit signed integer          */
/* DPBOOL    -- boolean variable (0 or 1)      */
/* DPUCHAR   -- 8-bit unsigned integer         */
/* DPUSHORT  -- 16-bit unsigned integer        */
/* DPUINT    -- 16-bit unsigned integer        */
/* DPULONG   -- 32-bit unsigned integer        */
/***********************************************/
typedef unsigned char  DPUCHAR;
typedef unsigned short DPUSHORT;
typedef unsigned int   DPUINT;
typedef unsigned long  DPULONG;
typedef unsigned char  DPBOOL;
typedef          char  DPCHAR;
typedef   signed int   DPINT;
typedef   signed long  DPLONG;
#define DPNULL ((void*)0)
#define FALSE 0u
#define TRUE 1u

/************************************************************/
/* Available actions                                        */
/************************************************************/
#define DP_NO_ACTION_FOUND                      0u
#define DP_DEVICE_INFO_ACTION_CODE              1u
#define DP_READ_IDCODE_ACTION_CODE              2u
#define DP_ERASE_ACTION_CODE                    3u
#define DP_PROGRAM_ACTION_CODE                  4u
#define DP_VERIFY_ACTION_CODE                   5u
#define DP_ENC_DATA_AUTHENTICATION_ACTION_CODE  6u
#define DP_VERIFY_DIGEST_ACTION_CODE			   7u

/************************************************************/
/* Error code definitions                                   */
/************************************************************/
#define DPE_SUCCESS                 0u
#define DPE_PROCESS_DATA_ERROR      2u
#define DPE_IDCODE_ERROR            6u
#define DPE_POLL_ERROR              7u
#define DPE_INIT_FAILURE            25u
#define DPE_ERASE_ERROR             8u
#define DPE_PROGRAM_ERROR           10u
#define DPE_VERIFY_ERROR            11u
#define DPE_AUTHENTICATION_FAILURE  18u
#define DPE_MATCH_ERROR             19u
#define DPE_VERIFY_DIGEST_ERROR     20u
#define DPE_CRC_MISMATCH            100u
#define DPE_ACTION_NOT_FOUND        150u
#define DPE_ACTION_NOT_SUPPORTED    151u
#define DPE_BYTES_RECEIVED_ERROR    152u
#define DPE_DAT_FILE_ACCESS_ERROR   165u


/* User defined */
///* Memory device SPI-Setting */
//#define MEM_SPI_INSTANCE                &g_mss_spi0
//#define MEM_SPI_SLAVE                   MSS_SPI_SLAVE_0
//#define MEM_SPI_PCLK_DIVIDER            64
//#define MEM_SPI_FRAME_BIT_LENGTH        8

/* Target device SPI-Setting */
#define DUT_SPI_INSTANCE                &g_pro_spi
#define DUT_SPI_SLAVE                   0
#define DUT_SPI_PCLK_DIVIDER            64
#define DUT_SPI_FRAME_BIT_LENGTH    8
/* End of User defined */


#endif

/*   *************** End of File *************** */

