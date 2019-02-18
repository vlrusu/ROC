/*******************************************************************************
 * (c) Copyright 2016 Microsemi SoC Products Group. All rights reserved.
 * 
 *  CMSIS system initialization.
 *
 * SVN $Revision: 8245 $
 * SVN $Date: 2016-02-15 18:19:35 +0000 (Mon, 15 Feb 2016) $
 */

#ifndef SYSTEM_M2SXXX_H
#define SYSTEM_M2SXXX_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif 

/* Standard CMSIS global variables. */
extern uint32_t SystemCoreClock;    /*!< System Clock Frequency (Core Clock) */


/***************************************************************************//**
 * The SystemInit() is a standard CMSIS function called during system startup.
 * It is meant to perform low level hardware setup such as configuring DDR and
 * SERDES controllers.
 */
void SystemInit(void);

/***************************************************************************//**
 * The SystemCoreClockUpdate() is a standard CMSIS function which can be called
 * by the application in order to ensure that the SystemCoreClock global
 * variable contains the up to date Cortex-M3 core frequency. Calling this
 * function also updates the global variables containing the frequencies of the
 * APB busses connecting the peripherals.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif
