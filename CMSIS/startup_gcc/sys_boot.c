/*******************************************************************************
 * (c) Copyright 2016 Actel Corporation.  All rights reserved.
 *
 * Cortex-M1 boot code.
 * This function is called from startup_cortexm1.S
 *
 * SVN $Revision: 8517 $
 * SVN $Date: 2016-08-26 14:37:15 +0100 (Fri, 26 Aug 2016) $
 */
#ifdef __cplusplus
extern "C" {
#endif

#include "cortexm1_cfg.h"

#define ITCM_LOW_START_ADDRESS     (unsigned int *)0x00000000  /* defined by ARM */
#define ITCM_HIGH_START_ADDRESS     (unsigned int *)0x10000000  /* defined by ARM */

/*------------------------------------------------------------------------------
 * main() function prototype as is called from this file.
 */


void SystemInit(void);

/*------------------------------------------------------------------------------
 * Symbols from the linker script used to locate the text, data and bss sections.
 */

extern unsigned int __text_load;
extern unsigned int __text_start;
extern unsigned int _etext;
extern unsigned int _uPROM_start;
extern unsigned int _uPROM_end;

extern unsigned int __data_load;
extern unsigned int __data_start;
extern unsigned int _edata;

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;

extern unsigned int __sc_load;
extern unsigned int __sc_start;
extern unsigned int __sc_end;

extern unsigned int __vector_table_load;
extern unsigned int _vector_table_end_load;

#ifndef __ARM_ARCH_6M__
#error "Microsemi ARM Cortex-M1 CMSIS Hardware Abstraction Layer must be compiled with -mcpu=cortex-m1 -mthumb or -march=armv6-m -mthumb"
#endif /* __ARM_ARCH_6M__ */

extern unsigned int _startup_option;

__attribute__((section(".ram_codetext"))) void switch_program(void);

/*------------------------------------------------------------------------------
 * _start() function called invoked
 * This function is callled from  startup_cortexm1.S on power up and warm reset.
 */
void _start_c( void)
{
    uint32_t *acr = (uint32_t *)0xE000E008; /* address of auxiliary control register */
    /*
     * Copy text section if required (copy executable from LMA to VMA).
     */
    {
        unsigned int * text_lma = &__text_load;
        unsigned int * end_text_vma = &_etext;
        unsigned int * text_vma = &__text_start;

        if ( text_vma != text_lma)
        {
            while ( text_vma <= end_text_vma)
            {
                *text_vma++ = *text_lma++;
            }
        }
    }

    /*
     * Copy data section if required (initialized variables).
     */
    {
        unsigned int * data_lma = &__data_load;
        unsigned int * end_data_vma = &_edata;
        unsigned int * data_vma = &__data_start;

        if ( data_vma != data_lma )
        {
            while ( data_vma <= end_data_vma )
            {
                *data_vma++ = *data_lma++;
            }
        }
    }

    /*
     * Zero out the bss section (set non-initialized variables to 0).
     */
    {
        unsigned int * bss = &__bss_start__;
        unsigned int * bss_end = &__bss_end__;

        if ( bss_end > bss)
        {
            while ( bss <= bss_end )
            {
                *bss++ = 0;
            }
        }
    }

    /*
     *  Copy a vector table if not located at zero. e.g. A program loaded by a boot-loader and running from DDR
     */
    if ( &_startup_option == (unsigned int *)2) /* Only required to be true if this program's vector table not located at zero */
    {
        unsigned int * vec_table_rom = &__vector_table_load;
        unsigned int * vec_table_end = &_vector_table_end_load;
        unsigned int * vec_table_ram = ITCM_HIGH_START_ADDRESS;

        if ( vec_table_rom > 0 )
        {
            *acr |= 0x04;  /* make sure ITCM mapped to 0x10000000 */
            asm ("DSB  ");                      /*; ensure that store completes before      */
                                                /*; executing the next instruction          */
            asm ("ISB  ");                      /*; executing synchronization instruction   */
            /*ITCM is now located at 0x00000000 */
            while ( vec_table_rom <= vec_table_end )
            {
                *vec_table_ram++ = *vec_table_rom++;
            }
            *acr |= 0x08;  /* make sure ITCM mapped to 0x00000000 */
            asm ("DSB  ");                      /*; ensure that store completes before      */
                                                /*; executing the next instruction          */
            asm ("ISB  ");                      /*; executing synchronization instruction   */
            /*ITCM is now located at 0x00000000 */
        }
    }

    /*
     * Copy switch code routine to RAM. Required when switching from uPROM to different program in ITCM
     */
    {
        unsigned int * sc_lma = &__sc_load;
        unsigned int * end_sc_vma = &__sc_end;
        unsigned int * sc_vma = &__sc_start;
        if (&_startup_option == (unsigned int *)3)
        {
            if ( sc_vma != sc_lma )
            {
                while ( sc_vma <= end_sc_vma )
                {
                    *sc_vma++ = *sc_lma++;
                }
            }
        }
    }

    /*
     * Copy program to ITCM and start running from ITCM
     */
    if (&_startup_option == (unsigned int *)1)
    {
        switch_program();
    }

    /*
     * SystemInit(void) as required by CMSIS
     */
    SystemInit();

}

/*
 * This routine will switch to running the program from ITCM
 * The auxiliary control register contains bits that control the memory space the ITCM occupies
 * It occupies 0x10000000 when bit 4 set to one
 * It occupies 0x00000000 when bit 3 set to one
 * Both these bits can be set to a known state on startup in the Libero design
 * In normal operation, configure in Libero so
 * bit 3 is 0
 * bit 4 is 1
 * which means ITCM will be located at 0x10000000 on start-up
 * uPROM will be located at 0x0000000 on startup
 * The routine below  will copy the program from uPROM/LSRAM to ITCM located at 0x10000000
 * It will then set bit 4 in the ACR, which maps ITCM to 0x0000000
 * The program will then be running from ITCM
 */
void switch_program(void)
{
    uint32_t asp_value;
    uint32_t *acr = (uint32_t *)0xE000E008; /* address of auxiliary control register */
    //unsigned int * text_lma = &__text_load;
    unsigned int * uprom_start = &_uPROM_start;
    unsigned int * uprom_end = &_uPROM_end;
    unsigned int * itcm = ITCM_HIGH_START_ADDRESS;

    asp_value = *acr;           /* Line below is TRUE if it is a cold reset */
    if(!(asp_value & 0x8))      /* if ITCM not mapped to zero, copy code there and map now */
    {
        /* copy ROM code to ITCM  */
        while ( uprom_start <= uprom_end )
        {
            *itcm++ = *uprom_start++;
        }
        /* set bit in aux control register so ITCM appears at 0x0000000 */
        __disable_irq();
        asp_value |= 0x8;
        *acr = asp_value;
        asm ("DSB  ");                      /*; ensure that store completes before      */
                                            /*; executing the next instruction          */
        asm ("ISB  ");                      /*; executing synchronization instruction   */
        /*ITCM is now located at 0x00000000 */
        /* it is the sane as code that was in uPROM so you can just continue on as before */
        __enable_irq();
    }
}

/*
 * Example:
 * This routine is used to jump to a new program in ITCM, that is loaded by a boot-loader application
 * running from uPROM/LSRAM in the RTG4, when the program being loaded is different from the boot-loader.
 * Look at Cortex-M1 TCM Initialization Considerations for System Designers for guidance on the ARM web-site.
 * The reset and stack pointer must be located at 0x00000000 and 0x00000004
 * The application must first be copied from storage or serial port to ITCM and then then this routine
 * is called to run it.
 */
#if LOADING_PROGRAM_TO_ITCM
__attribute__((section(".ram_codetext"))) static void JumpToNewApplicationInITCM(unsigned int * uprom_start, unsigned int * uprom_end = &_uPROM_end)
{
    uint32_t asp_value;
    uint32_t *acr = (uint32_t *)0xE000E008; /* address of auxiliary control register */
    unsigned int * itcm = ITCM_HIGH_START_ADDRESS;

    asp_value = *acr;           /* Line below is TRUE if it is a cold reset */
    if(!(asp_value & 0x8))      /* if ITCM not mapped to zero, copy code there and map now */
    {
        /* copy ROM code to ITCM  */
        while ( uprom_start <= uprom_end )
        {
            *itcm++ = *uprom_start++;
        }
        /* set bit in aux control register so ITCM appears at 0x0000000 */
        __disable_irq();
        asp_value |= 0x8;
        *acr = asp_value;

        /* you should de-init any drivers */
        /*Load main stack pointer with application stack pointer initial value,
          stored at first location of application area */
        asm volatile("ldr r0, =0x00000000");
        asm volatile("ldr r0, [r0]");
        asm volatile("mov sp, r0");

        /*Load program counter with application reset vector address, located at
          second word of application area. */
        asm volatile("mov r0, #0");  /*  ; no arguments  */
        asm volatile("mov r1, #0");  /*  ; no argv either */
        asm volatile("ldr r3, =0x00000004");
        asm volatile("ldr r3, [r3]");
        asm volatile("mov pc, r3");
        /*New application execution should now start and never return here.... */
    }
}
#endif


#ifdef __cplusplus
}
#endif
