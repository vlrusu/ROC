 /*******************************************************************************
 * (c) Copyright 2016 Microsemi SoC Products Group.  All rights reserved.
 * 
 * @file startup_m2sxxx.S
 * @author Microsemi SoC Products Group
 * @brief Cortex-M1 vector table and startup code for CodeSourcery G++.
 *
 * SVN $Revision: 8240 $
 * SVN $Date: 2016-02-12 16:32:14 +0000 (Fri, 12 Feb 2016) $
 */

    .syntax unified
    .cpu cortex-m1
    .thumb
    .code 16
    

/*==============================================================================
 * Vector table  
 */
    .global     g_pfnVectors
    .section    .isr_vector,"ax",%progbits    /* added "x" so appears in .lst file even though not executable code- to help in debug process */

    .type       g_pfnVectors, %object
    .size       g_pfnVectors, .-g_pfnVectors
    
g_pfnVectors:
    .word  _estack
    .word  Reset_Handler
    .word    NMI_Handler            /* NMI handler, external input */
    .word    HardFault_Handler      /* Fault handler */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    0                      /* Reserved */
    .word    SVC_Handler            /* System service call with SVC instruction. Note: OS extensions must be enabled in Libero Cortex-M1 dialog */
    .word    0                      /* Reserved. */
    .word    0                      /* Reserved. */
    .word    PendSV_Handler         /* Pendable request for system service. Note: OS extensions must be enabled in Libero Cortex-M1 dialog */
    .word    SysTick_Handler        /* System tick timer. Note: OS extensions must be enabled in Libero Cortex-M1 dialog*/
    .word    IRQ0_IRQHandler        /* External interrupt 0. */
    .word    IRQ1_IRQHandler        /* External interrupt 1. */
    .word    IRQ2_IRQHandler        /* External interrupt 2. */
    .word    IRQ3_IRQHandler        /* External interrupt 3. */
    .word    IRQ4_IRQHandler        /* External interrupt 4. */
    .word    IRQ5_IRQHandler        /* External interrupt 5. */
    .word    IRQ6_IRQHandler        /* External interrupt 6. */
    .word    IRQ7_IRQHandler        /* External interrupt 7. */
    .word    IRQ8_IRQHandler        /* External interrupt 8. */
    .word    IRQ9_IRQHandler        /* External interrupt 9. */
    .word    IRQ10_IRQHandler        /* External interrupt 10. */
    .word    IRQ11_IRQHandler        /* External interrupt 11. */
    .word    IRQ12_IRQHandler        /* External interrupt 12. */
    .word    IRQ13_IRQHandler        /* External interrupt 13. */
    .word    IRQ14_IRQHandler        /* External interrupt 14. */
    .word    IRQ15_IRQHandler        /* External interrupt 15. */
    .word    IRQ16_IRQHandler        /* External interrupt 16. */
    .word    IRQ17_IRQHandler        /* External interrupt 17. */
    .word    IRQ18_IRQHandler        /* External interrupt 18. */
    .word    IRQ19_IRQHandler        /* External interrupt 19. */
    .word    IRQ20_IRQHandler        /* External interrupt 20. */
    .word    IRQ21_IRQHandler        /* External interrupt 21. */
    .word    IRQ22_IRQHandler        /* External interrupt 22. */
    .word    IRQ23_IRQHandler        /* External interrupt 23. */
    .word    IRQ24_IRQHandler        /* External interrupt 24. */
    .word    IRQ25_IRQHandler        /* External interrupt 25. */
    .word    IRQ26_IRQHandler        /* External interrupt 26. */
    .word    IRQ27_IRQHandler        /* External interrupt 27. */
    .word    IRQ28_IRQHandler        /* External interrupt 28. */
    .word    IRQ29_IRQHandler        /* External interrupt 29. */
    .word    IRQ30_IRQHandler        /* External interrupt 30. */
    .word    IRQ31_IRQHandler        /* External interrupt 31. */

/*==============================================================================
 * Reset_Handler
 * Register R6 is used to keep track of whether we need to initialize RAMs
 * because ECC/ SECDED is enabled.
 */
    .global Reset_Handler
    .section    .boot_code,"ax",%progbits            
    .type   Reset_Handler, %function
Reset_Handler:
clear_stack:
    ldr r3, =__vector_table_load  /* load the correct stack pointer value */
    ldr r1, [r3]                  /* sp is the first value stored in the vector table */
    mov sp, r1
    movs r0,#0      /*  ; no arguments  */
    movs r1,#0      /*  ; no argv either */
    ldr r3, =_start_c
    mov  r4,pc
    adds  r4,#5
    mov  lr,r4
    blx  r3

/*------------------------------------------------------------------------------
 * Call global constructors
 */
    /*
     * Align to word and use 32-bits LDR instruction to ensure the ADD instruction
     * taking PC as argument is aligned on a word boundary.
     */
call_glob_ctor:
    .align 4
    ldr r3, =__libc_init_array
    mov  r4,pc
    adds  r4,#5
    mov  lr,r4
    bx   r3

/*------------------------------------------------------------------------------
 * branch to main.
 */
branch_to_main:
    movs r0, #0      /*  ; no arguments  */
    movs r1, #0      /*  ; no argv either */
	.align 4
    ldr r3, =main
    mov  r4,pc
    adds  r4,#5
    mov  lr,r4
    bx   r3

ExitLoop:		/* shoulnot return from main so will never reach here */
    B ExitLoop

/*==============================================================================
 * NMI_Handler
 */
    .weak   NMI_Handler
    .type   NMI_Handler, %function
NMI_Handler:
    B .

/*==============================================================================
 * HardFault_Handler
 */
    .weak   HardFault_Handler
    .type   HardFault_Handler, %function
HardFault_Handler:
    B .

/*==============================================================================
 * MemManage_Handler
 */
    .weak   MemManage_Handler
    .type   MemManage_Handler, %function
MemManage_Handler:
    B .

/*==============================================================================
 * BusFault_Handler
 */
    .weak   BusFault_Handler
    .type   BusFault_Handler, %function
BusFault_Handler:
    B .

/*==============================================================================
 * UsageFault_Handler
 */
    .weak   UsageFault_Handler
    .type   UsageFault_Handler, %function
UsageFault_Handler:
    B .

/*==============================================================================
 * SVC_Handler
 */
    .weak   SVC_Handler
    .type   SVC_Handler, %function
SVC_Handler:
    B .

/*==============================================================================
 * DebugMon_Handler
 */
    .weak   DebugMon_Handler
    .type   DebugMon_Handler, %function
DebugMon_Handler:
    B .

/*==============================================================================
 * PendSV_Handler
 */
    .weak   PendSV_Handler
    .type   PendSV_Handler, %function
PendSV_Handler:
    B .

/*==============================================================================
 * SysTick_Handler
 */
    .weak   SysTick_Handler
    .type   SysTick_Handler, %function
SysTick_Handler:
    B .

/*==============================================================================
 *IRQ0_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ0_IRQHandler
    .type   IRQ0_IRQHandler, %function
IRQ0_IRQHandler:
    B .

/*==============================================================================
 *IRQ1_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ1_IRQHandler
    .type   IRQ1_IRQHandler, %function
IRQ1_IRQHandler:
    B .

/*==============================================================================
 *IRQ2_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ2_IRQHandler
    .type   IRQ2_IRQHandler, %function
IRQ2_IRQHandler:
    B .

/*==============================================================================
 *IRQ3_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ3_IRQHandler
    .type   IRQ3_IRQHandler, %function
IRQ3_IRQHandler:
    B .

/*==============================================================================
 *IRQ4_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ4_IRQHandler
    .type   IRQ4_IRQHandler, %function
IRQ4_IRQHandler:
    B .

/*==============================================================================
 *IRQ5_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ5_IRQHandler
    .type   IRQ5_IRQHandler, %function
IRQ5_IRQHandler:
    B .

/*==============================================================================
 *IRQ6_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ6_IRQHandler
    .type   IRQ6_IRQHandler, %function
IRQ6_IRQHandler:
    B .

/*==============================================================================
 *IRQ7_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ7_IRQHandler
    .type   IRQ7_IRQHandler, %function
IRQ7_IRQHandler:
    B .

/*==============================================================================
 *IRQ8_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ8_IRQHandler
    .type   IRQ8_IRQHandler, %function
IRQ8_IRQHandler:
    B .

/*==============================================================================
 *IRQ9_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ9_IRQHandler
    .type   IRQ9_IRQHandler, %function
IRQ9_IRQHandler:
    B .

/*==============================================================================
 *IRQ10_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ10_IRQHandler
    .type   IRQ10_IRQHandler, %function
IRQ10_IRQHandler:
    B .

/*==============================================================================
 *IRQ11_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ11_IRQHandler
    .type   IRQ11_IRQHandler, %function
IRQ11_IRQHandler:
    B .

/*==============================================================================
 *IRQ12_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ12_IRQHandler
    .type   IRQ12_IRQHandler, %function
IRQ12_IRQHandler:
    B .

/*==============================================================================
 *IRQ13_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ13_IRQHandler
    .type   IRQ13_IRQHandler, %function
IRQ13_IRQHandler:
    B .

/*==============================================================================
 *IRQ14_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ14_IRQHandler
    .type   IRQ14_IRQHandler, %function
IRQ14_IRQHandler:
    B .

/*==============================================================================
 *IRQ15_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ15_IRQHandler
    .type   IRQ15_IRQHandler, %function
IRQ15_IRQHandler:
    B .

/*==============================================================================
 *IRQ16_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ16_IRQHandler
    .type   IRQ16_IRQHandler, %function
IRQ16_IRQHandler:
    B .

/*==============================================================================
 *IRQ17_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ17_IRQHandler
    .type   IRQ17_IRQHandler, %function
IRQ17_IRQHandler:
    B .

/*==============================================================================
 *IRQ18_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ18_IRQHandler
    .type   IRQ18_IRQHandler, %function
IRQ18_IRQHandler:
    B .

/*==============================================================================
 *IRQ19_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ19_IRQHandler
    .type   IRQ19_IRQHandler, %function
IRQ19_IRQHandler:
    B .

/*==============================================================================
 *IRQ20_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ20_IRQHandler
    .type   IRQ20_IRQHandler, %function
IRQ20_IRQHandler:
    B .

/*==============================================================================
 *IRQ21_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ21_IRQHandler
    .type   IRQ21_IRQHandler, %function
IRQ21_IRQHandler:
    B .

/*==============================================================================
 *IRQ22_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ22_IRQHandler
    .type   IRQ22_IRQHandler, %function
IRQ22_IRQHandler:
    B .

/*==============================================================================
 *IRQ23_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ23_IRQHandler
    .type   IRQ23_IRQHandler, %function
IRQ23_IRQHandler:
    B .

/*==============================================================================
 *IRQ24_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ24_IRQHandler
    .type   IRQ24_IRQHandler, %function
IRQ24_IRQHandler:
    B .

/*==============================================================================
 *IRQ25_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ25_IRQHandler
    .type   IRQ25_IRQHandler, %function
IRQ25_IRQHandler:
    B .

/*==============================================================================
 *IRQ26_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ26_IRQHandler
    .type   IRQ26_IRQHandler, %function
IRQ26_IRQHandler:
    B .

/*==============================================================================
 *IRQ27_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ27_IRQHandler
    .type   IRQ27_IRQHandler, %function
IRQ27_IRQHandler:
    B .

/*==============================================================================
 *IRQ28_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ28_IRQHandler
    .type   IRQ28_IRQHandler, %function
IRQ28_IRQHandler:
    B .

/*==============================================================================
 *IRQ29_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ29_IRQHandler
    .type   IRQ29_IRQHandler, %function
IRQ29_IRQHandler:
    B .

/*==============================================================================
 *IRQ30_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ30_IRQHandler
    .type   IRQ30_IRQHandler, %function
IRQ30_IRQHandler:
    B .

/*==============================================================================
 *IRQ31_IRQHandler - hardware input to Cortex-M1
 */
    .weak   IRQ31_IRQHandler
    .type   IRQ31_IRQHandler, %function
IRQ31_IRQHandler:
    B .



/*==============================================================================
 * mscc_post_hw_cfg_init
 */
    .weak   mscc_post_hw_cfg_init
    .type   mscc_post_hw_cfg_init, %function
mscc_post_hw_cfg_init:
    BX LR

 
.end
