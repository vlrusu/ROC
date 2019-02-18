/*******************************************************************************
 * (c) Copyright 2016 Microsemi SoC Products Group. All rights reserved.
 *
 *  Cortex-M1 Microcontroller Software Interface - Peripheral
 *  Access Layer.
 *
 *  This file provides interfaces to perform register and register bit level
 *  read / write operations. These interfaces support bit-banding in case of
 *  Cortex-M3 CPU.
 *
 *
 * SVN $Revision: 8315 $
 * SVN $Date: 2016-03-14 14:25:46 +0000 (Mon, 14 Mar 2016) $
 */

#ifndef HW_REG_IO_H_
#define HW_REG_IO_H_

#include <stdint.h>                       /* Include standard types */

#if defined ( __CC_ARM   )
  #define __INLINE         __inline       /*!< inline keyword for ARM Compiler       */

#elif defined ( __ICCARM__ )
  #define __INLINE        inline          /*!< inline keyword for IAR Compiler. Only avaiable in High optimization mode! */

#elif defined   (  __GNUC__  )
  #define __INLINE        inline          /*!< inline keyword for GNU Compiler       */
#endif

/*****************************************************************************************
 * Definitions for register access withthe Cortex-M1 only
 * For accessing Core peripheral registers, the HAL register accesss functions
 * should be used. Examples of how to use these are found in the bare metal
 * driver implimentaions if required.
 */

#define HW_REG(addr)            (*((volatile uint32_t *) (addr)))

static __INLINE void write_reg32(volatile uint32_t * reg, uint32_t val)
{
    HW_REG(reg) = val;
}
static __INLINE void write_reg16(volatile uint16_t * reg, uint16_t val)
{
    HW_REG(reg) = val;
}
static __INLINE void write_reg8(volatile uint8_t * reg, uint8_t val)
{
    HW_REG(reg) = val;
}

static __INLINE uint32_t read_reg32(volatile uint32_t * reg)
{
    return ( HW_REG(reg) );
}
static __INLINE uint16_t read_reg16(volatile uint16_t * reg)
{
    return ( HW_REG(reg) );
}
static __INLINE uint8_t read_reg8(volatile uint8_t * reg)
{
    return ( HW_REG(reg) );
}

#endif /* HW_REG_IO_H_ */
