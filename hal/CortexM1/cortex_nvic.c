/*******************************************************************************
 * (c) Copyright 2016 Actel Corporation.  All rights reserved.
 * 
 * Legacy Actel HAL Cortex NVIC control functions.
 * The use of these functions should be replaced by calls to the equivalent
 * CMSIS function in your application code.
 *
 * SVN $Revision: 8283 $
 * SVN $Date: 2016-03-02 16:33:39 +0000 (Wed, 02 Mar 2016) $
 */

#include "cortex_nvic.h"
#ifdef MSCC_NO_RELATIVE_PATHS
#include "cortexm1_cfg_assert.h"
#else
#include "../../CMSIS/cortexm1_cfg_assert.h"
#endif


/***************************************************************************//**
 * 
 */
void NVIC_init( void )
{
    /*
     * Please use the NVIC control functions provided by the SmartFusion2 CMSIS
     * Hardware Abstraction Layer. The use of the Actel HAL NVIC control
     * functions is obsolete on SmartFusion2 devices.
     *
     * Simply remove the call to NVIC_init() from your application code.
     */
    ASSERT(0);
}


/***************************************************************************//**
 * 
 */
void NVIC_set_priority
(
    uint32_t interrupt_number,
    uint32_t priority_level
)
{
    /*
     * Please use the NVIC control functions provided by the SmartFusion2 CMSIS
     * Hardware Abstraction Layer. The use of the Actel HAL NVIC control
     * functions is obsolete on SmartFusion2 devices.
     *
     * Please replace calls to NVIC_set_priority() with a call to the CMSIS
     * void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority) function where
     * IRQn is one of the following values:
     *      - FabricIrq0_IRQn
     *      - FabricIrq1_IRQn
     *      - FabricIrq2_IRQn
     *      - FabricIrq3_IRQn
     *      - FabricIrq4_IRQn
     *      - FabricIrq5_IRQn
     *      - FabricIrq6_IRQn
     *      - FabricIrq7_IRQn
     *      - FabricIrq8_IRQn
     *      - FabricIrq9_IRQn
     *      - FabricIrq10_IRQn
     *      - FabricIrq11_IRQn
     *      - FabricIrq12_IRQn
     *      - FabricIrq13_IRQn
     *      - FabricIrq14_IRQn
     *      - FabricIrq15_IRQn
     */
    ASSERT(0);
}

/***************************************************************************//**
 * 
 */
void NVIC_enable_interrupt( uint32_t interrupt_number )
{
    /*
     * Please use the NVIC control functions provided by the SmartFusion2 CMSIS
     * Hardware Abstraction Layer. The use of the Actel HAL NVIC control
     * functions is obsolete on SmartFusion2 devices.
     *
     * Please replace calls to NVIC_enable_interrupt() with a call to the CMSIS
     * void NVIC_EnableIRQ(IRQn_Type IRQn) function where IRQn is one of the
     *      - ExternalIrq0_IRQn
     *      - ExternalIrq1_IRQn
     	    ...
     *      - ExternalIrq31_IRQn
     */
    ASSERT(0);
}

/***************************************************************************//**
 * 
 */
void NVIC_disable_interrupt( uint32_t interrupt_number )
{
    /*
     * Please use the NVIC control functions provided by the SmartFusion2 CMSIS
     * Hardware Abstraction Layer. The use of the Actel HAL NVIC control
     * functions is obsolete on SmartFusion2 devices.
     *
     * Please replace calls to NVIC_disable_interrupt() with a call to the CMSIS
     * void NVIC_DisableIRQ(IRQn_Type IRQn) function where IRQn is one of the
     * following values:
     *      - ExternalIrq0_IRQn
     *      - ExternalIrq1_IRQn
     	    ...
     *      - ExternalIrq31_IRQn
     */
    ASSERT(0);
}

/***************************************************************************//**
 * 
 */
void NVIC_clear_interrupt( uint32_t interrupt_number )
{
    /*
     * Please use the NVIC control functions provided by the SmartFusion2 CMSIS
     * Hardware Abstraction Layer. The use of the Actel HAL NVIC control
     * functions is obsolete on SmartFusion2 devices.
     *
     * Please replace calls to NVIC_clear_interrupt() with a call to the CMSIS
     * void NVIC_ClearPendingIRQ(IRQn_Type IRQn) function where IRQn is one of the
     * following values:
     *      - ExternalIrq0_IRQn
     *      - ExternalIrq1_IRQn
     	    ...
     *      - ExternalIrq31_IRQn
     */
    ASSERT(0);
}
