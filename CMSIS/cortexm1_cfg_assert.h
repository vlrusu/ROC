/*******************************************************************************
 * (c) Copyright 2006 Microsemi SoC Products Group. All rights reserved.
 *
 * Assertion implementation.
 *
 * This file provides the implementation of the ASSERT macro. This file can be
 * modified to cater for project specific requirements regarding the way
 * assertions are handled.
 *
 * SVN $Revision: 8245 $
 * SVN $Date: 2016-02-15 18:19:35 +0000 (Mon, 15 Feb 2016) $
 */
#ifndef __MSS_ASSERT_H_
#define __MSS_ASSERT_H_
#ifdef __cplusplus
extern "C" {
#endif

#if defined(NDEBUG)

#define ASSERT(CHECK)

#else   /* NDEBUG */

#include <assert.h>

#if defined ( __GNUC__   )

/*
 * SoftConsole assertion handling
 */
#define ASSERT(CHECK)  \
    do { \
        if (!(CHECK)) \
        { \
            __asm volatile ("BKPT\n\t"); \
        } \
    } while (0);


#elif defined ( __ICCARM__ )
/*
 * IAR Embedded Workbench assertion handling.
 * Call C library assert function which should result in error message
 * displayed in debugger.
 */
#define ASSERT(X)   assert(X)

#else
/*
 * Keil assertion handling.
 * Call C library assert function which should result in error message
 * displayed in debugger.
 */

#ifndef __MICROLIB
  #define ASSERT(X)   assert(X)
#else
  #define ASSERT(X)
#endif

#endif  /* Tool Chain */

#endif  /* NDEBUG */
#ifdef __cplusplus
}
#endif

#endif  /* __MSS_ASSERT_H_ */
