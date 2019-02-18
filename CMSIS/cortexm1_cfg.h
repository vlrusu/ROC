/*******************************************************************************
 * (c) Copyright 2016 Microsemi SoC Products Group. All rights reserved.
 *
 *  Microsemi Cortex-M1 Microcontroller Software Interface - Peripheral
 *  Access Layer.
 *
 *  This file describes the interrupt assignment and peripheral registers for
 *  Microsmei's Configurable Cortex-M1.
 *
 * SVN $Revision: 8270 $
 * SVN $Date: 2016-02-29 16:30:02 +0000 (Mon, 29 Feb 2016) $
 */
#ifndef __MICROSEMI_CORTEXM1_CMSIS_HAL_H__
#define __MICROSEMI_CORTEXM1_CMSIS_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */

typedef enum IRQn
{
/******  Cortex-M1 Processor Exceptions Numbers *********************************************************/
  NonMaskableInt_IRQn             = -14,      /*!< 2 Non Maskable Interrupt -               */
  HardFault_IRQn                  = -13,      /*!< 2 Hard Fault Interrupt                               */
  SVCall_IRQn                     = -5,       /*!< 11 Cortex-M1 SV Call Interrupt                       */
  DebugMonitor_IRQn               = -4,       /*!< 12 Cortex-M1 Debug Monitor Interrupt                 */
  PendSV_IRQn                     = -2,       /*!< 14 Cortex-M1 Pend SV Interrupt                       */
  SysTick_IRQn                    = -1,       /*!< 15 Cortex-M1 System Tick Interrupt                   */

/******  Microsemi specific Cortex-M1 external Interrupt Numbers *******************************************/
  External0_IRQn                 = 0,       /*!< FPGA External interrupt 0                              */
  External1_IRQn                 = 1,       /*!< FPGA External interrupt 1                              */
  External2_IRQn                 = 2,       /*!< FPGA External interrupt 2                              */
  External3_IRQn                 = 3,       /*!< FPGA External interrupt 3                              */
  External4_IRQn                 = 4,       /*!< FPGA External interrupt 4                              */
  External5_IRQn                 = 5,       /*!< FPGA External interrupt 5                              */
  External6_IRQn                 = 6,       /*!< FPGA External interrupt 6                              */
  External7_IRQn                 = 7,       /*!< FPGA External interrupt 7                              */
  External8_IRQn                 = 8,       /*!< FPGA External interrupt 8                              */
  External9_IRQn                 = 9,       /*!< FPGA External interrupt 9                              */
  External10_IRQn                = 10,       /*!< FPGA External interrupt 10                             */
  External11_IRQn                = 11,       /*!< FPGA External interrupt 11                             */
  External12_IRQn                = 12,       /*!< FPGA External interrupt 12                             */
  External13_IRQn                = 13,       /*!< FPGA External interrupt 13                             */
  External14_IRQn                = 14,       /*!< FPGA External interrupt 14                             */
  External15_IRQn                = 15,       /*!< FPGA External interrupt 15                             */
  External16_IRQn                = 16,       /*!< FPGA External interrupt 0                              */
  External17_IRQn                = 17,       /*!< FPGA External interrupt 1                              */
  External18_IRQn                = 18,       /*!< FPGA External interrupt 2                              */
  External19_IRQn                = 19,       /*!< FPGA External interrupt 3                              */
  External20_IRQn                = 20,       /*!< FPGA External interrupt 4                              */
  External21_IRQn                = 21,       /*!< FPGA External interrupt 5                              */
  External22_IRQn                = 22,       /*!< FPGA External interrupt 6                              */
  External23_IRQn                = 23,       /*!< FPGA External interrupt 7                              */
  External24_IRQn                = 24,       /*!< FPGA External interrupt 8                              */
  External25_IRQn                = 25,       /*!< FPGA External interrupt 9                              */
  External26_IRQn                = 26,       /*!< FPGA External interrupt 10                             */
  External27_IRQn                = 27,       /*!< FPGA External interrupt 11                             */
  External28_IRQn                = 28,       /*!< FPGA External interrupt 12                             */
  External29_IRQn                = 29,       /*!< FPGA External interrupt 13                             */
  External30_IRQn                = 30,       /*!< FPGA External interrupt 14                             */
  External31_IRQn                = 31,       /*!< FPGA External interrupt 15                             */
} IRQn_Type;

/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* CM0 used as */
#define __CM0_REV                 0x0000    /*!< Core revision r0p0                              */
#define __MPU_PRESENT             0         /*!< MPU present or not                              */
#define __NVIC_PRIO_BITS          2         /*!< Number of Bits used for Priority Levels         */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used    */

#include <core_cm0.h>   /* Cortex-M1 processor and core peripherals           */

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
#if defined   ( __CC_ARM   )
  /* Enable anonymous unions when building using Keil-MDK */
  #pragma anon_unions
#endif


/*----------------------------------------------------------------------------*/
/*------------------------------ System Registers ----------------------------*/
/*---------------------------- No Microsemi defined ones as of now -----------*/
/*----------------------------------------------------------------------------*/
#if 0
typedef struct
{


} SYSREG_TypeDef;
#endif
/*----------------------------------------------------------------------------*/
/*-------------------------- CoreCM1Config Registers -------------------------*/
/*-------------------------- Not in current core -----------------------------*/
/*----------------------------------------------------------------------------*/
typedef struct {
    __IO     uint32_t   CONFIG_DONE;
    __I      uint32_t   INIT_DONE;
    __IO     uint32_t   CLR_INIT_DONE;
    __I      uint32_t   CONFIG_SR;
    __IO     uint32_t   SOFT_RESET_CR;
    __I      uint32_t   IP_VERSION_SR;
} CoreCM1Config_TypeDef;

#ifdef __cplusplus
}
#endif

#endif  /* __MICROSEMI_CORTEXM1_CMSIS_PAL_H__ */



