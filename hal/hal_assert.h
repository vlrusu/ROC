/*******************************************************************************
 * (c) Copyright 2008-2013 Microsemi SoC Products Group. All rights reserved.
 * 
 * SVN $Revision: 8283 $
 * SVN $Date: 2016-03-02 16:33:39 +0000 (Wed, 02 Mar 2016) $
 */
#ifndef HAL_ASSERT_HEADER
#define HAL_ASSERT_HEADER

#ifdef MSCC_NO_RELATIVE_PATHS
#include "cortexm1_assert.h"
#else
#include "../CMSIS/cortexm1_cfg_assert.h"
#endif

#if defined(NDEBUG)
/***************************************************************************//**
 * HAL_ASSERT() is defined out when the NDEBUG symbol is used.
 ******************************************************************************/
#define HAL_ASSERT(CHECK)

#else
/***************************************************************************//**
 * Default behaviour for HAL_ASSERT() macro:
 *------------------------------------------------------------------------------
 * Using the HAL_ASSERT() macro is the same as directly using the SmartFusion2
 * CMSIS ASSERT() macro. The behaviour is toolchain specific and project
 * setting specific.
 ******************************************************************************/
#define HAL_ASSERT(CHECK)     ASSERT(CHECK);

#endif  /* NDEBUG */

#endif  /* HAL_ASSERT_HEADER */
