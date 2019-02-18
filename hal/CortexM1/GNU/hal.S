#-------------------------------------------------------------------------------
# (c) Copyright 2016 Actel Corporation.  All rights reserved.
# 
# Interrupt disabling/restoration for critical section protection.
#
# SVN $Revision: 8292 $
# SVN $Date: 2016-03-08 16:32:41 +0000 (Tue, 08 Mar 2016) $
#

#ifndef __ARM_ARCH_6M__
#error "Microsemi ARM Cortex-M1 CMSIS Hardware Abstraction Layer must be compiled with -mcpu=cortex-m1 -mthumb or -march=armv6-m -mthumb"
#endif /* __ARM_ARCH_6M__ */

    .text
    .code 16
    .syntax unified
    
    .type HAL_disable_interrupts, function
    .type HAL_restore_interrupts, function
    
    .global HAL_disable_interrupts
    .global HAL_restore_interrupts

#-------------------------------------------------------------------------------
# 
#
HAL_disable_interrupts:
    mrs r0, PRIMASK
    cpsid I
    bx lr

#-------------------------------------------------------------------------------
#
#
HAL_restore_interrupts:
    msr PRIMASK, r0
    bx lr

.end
