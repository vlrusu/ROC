/*******************************************************************************
 * (c) Copyright 2016 Microsemi SoC Products Group.  All rights reserved.
 *
 * Stubs for Newlib system calls.
 *
 * SVN $Revision: 6665 $
 * SVN $Date: 2014-07-03 16:56:22 +0100 (Thu, 03 Jul 2014) $
 */
#include <stdio.h>
#include <stdlib.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/times.h>
#include <errno.h>

/*==============================================================================
 * Redirection of standard output to a CoreUARTapb instance.
 *------------------------------------------------------------------------------
 * A default implementation for the redirection of the output of printf() and scanf()
 * is provided in this file. This redirection is enabled by
 * adding the symbol/define MSCC_STDIO_THRU_CORE_UART_APB to your project and
 * specifying the base address of the CoreUARTapb instance
 * The baud rate defaults to 115200 but can be modified by adding a
 * definition of MSCC_STDIO_BAUD_VALUE to the project with the value set to the required
 * baud rate
 * Instructions to use:
 * 1. Add definition for MSCC_STDIO_THRU_CORE_UART_APB to your project
 * 2. Find comment relating to MSCC_STDIO_UART_BASE_ADDR in hw_platform.h and
 *    modify as instructed.
 * 3. The default baud rate of 115200 can be modified by defining MSCC_STDIO_BAUD_VALUE
 *    with the desired baud rate in your project.
 */
#ifdef MSCC_STDIO_THRU_CORE_UART_APB

#ifdef MSCC_NO_RELATIVE_PATHS
#include "core_uart_apb.h"
#include "hw_platform.h"
#else
#include "../../drivers/CoreUARTapb/core_uart_apb.h"
#include "../../hw_platform.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MSCC_STDIO_UART_BASE_ADDR
/*
 * Let the user know he needs to define a base address
 */
#error MSCC_STDIO_UART_BASE_ADDR not defined- Obtain the base address from the Libero design (hw_platform.h)
#endif

#ifndef MSCC_STDIO_BAUD_VALUE
/*
 * The MSCC_STDIO_BAUD_VALUE define should be set in your project's settings to
 * specify the baud value used by the standard output CoreUARTapb instance for
 * generating the UART's baud rate if you want a different baud rate from the
 * default of 115200 baud
 */
#define MSCC_STDIO_BAUD_VALUE           115200
#endif

/*------------------------------------------------------------------------------
 * CoreUARTapb instance data for the CoreUARTapb instance used for standard
 * output.
 */
static UART_instance_t g_stdio_uart;

/*==============================================================================
 * Flag used to indicate if the UART driver needs to be initialized.
 */
static int g_stdio_uart_init_done = 0;


#define LSR_THRE_MASK   0x20u

/*
 * Disable semihosting apis
 */
#pragma import(__use_no_semihosting_swi)

/*==============================================================================
 * sendchar()
 */
int sendchar(int ch)
{
    /*--------------------------------------------------------------------------
    * Initialize the UART driver if it is the first time this function is
    * called.
    */
    if ( !g_stdio_uart_init_done )
    {
        /******************************************************************************
         * Baud value:
         * This value is calculated using the following equation:
         *      BAUD_VALUE = (CLOCK / (16 * BAUD_RATE)) - 1
         *****************************************************************************/
        UART_init( &g_stdio_uart, MSCC_STDIO_UART_BASE_ADDR, SYS_M1_CLK_FREQ/((16 * MSCC_STDIO_BAUD_VALUE)-1), (DATA_8_BITS | NO_PARITY));
        g_stdio_uart_init_done = 1;
    }

    /*--------------------------------------------------------------------------
    * Output text to the UART.
    */
    UART_send( &g_stdio_uart, (uint8_t *)&ch, 1 );

    return (ch);
}

/*==============================================================================
 * getachar()
 */
int getachar(void)
{
    uint8_t rx_size;
    uint8_t rx_byte;

    if ( !g_stdio_uart_init_done )
    {
        /******************************************************************************
         * Baud value:
         * This value is calculated using the following equation:
         *      BAUD_VALUE = (CLOCK / (16 * BAUD_RATE)) - 1
         *****************************************************************************/
        UART_init( &g_stdio_uart, MSCC_STDIO_UART_BASE_ADDR, SYS_M1_CLK_FREQ/((16 * MSCC_STDIO_BAUD_VALUE)-1), (DATA_8_BITS | NO_PARITY));
        g_stdio_uart_init_done = 1;
    }

    do {
        rx_size = UART_get_rx(&g_stdio_uart, &rx_byte, 1);
    } while(0u == rx_size);

    return rx_byte;
}

/*==============================================================================
 *
 */
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;


/*==============================================================================
 * fputc()
 */
int fputc(int ch, FILE *f)
{
  return (sendchar(ch));
}

/*==============================================================================
 * fgetc()
 */

int fgetc(FILE *f)
{
    return 0;
}

/*==============================================================================
 * ferror()
 */
#ifndef ferror
int ferror(FILE *f)
{
  /* Your implementation of ferror */
  return EOF;
}
#endif

/*==============================================================================
 * _ttywrch()
 */
void _ttywrch(int ch)
{
  sendchar(ch);
}

/*==============================================================================
 * _sys_exit()
 */
void _sys_exit(int return_code)
{
    for(;;)
    {
        ;  /* endless loop */
    }
}

#endif  /* MSCC_STDIO_THRU_CORE_UART_APB */



/*==============================================================================
 * Environment variables.
 * A pointer to a list of environment variables and their values. For a minimal
 * environment, this empty list is adequate:
 */
char *__env[1] = { 0 };
char **environ = __env;

/*==============================================================================
 * Close a file.
 */
int _close(int file)
{
    return -1;
}

/*==============================================================================
 * Transfer control to a new process.
 */
int _execve(char *name, char **argv, char **env)
{
    errno = ENOMEM;
    return -1;
}

/*==============================================================================
 * Exit a program without cleaning up files.
 */
void _exit( int code )
{
    /* Should we force a system reset? */
    while( 1 )
    {
        ;
    }
}

/*==============================================================================
 * Create a new process.
 */
int _fork(void)
{
    errno = EAGAIN;
    return -1;
}

/*==============================================================================
 * Status of an open file.
 */
int _fstat(int file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/*==============================================================================
 * Process-ID
 */
int _getpid(void)
{
    return 1;
}

/*==============================================================================
 * Query whether output stream is a terminal.
 */
int _isatty(int file)
{
    return 1;
}

/*==============================================================================
 * Send a signal.
 */
int _kill(int pid, int sig)
{
    errno = EINVAL;
    return -1;
}

/*==============================================================================
 * Establish a new name for an existing file.
 */
int _link(char *old, char *new)
{
    errno = EMLINK;
    return -1;
}

/*==============================================================================
 * Set position in a file.
 */
int _lseek(int file, int ptr, int dir)
{
    return 0;
}

/*==============================================================================
 * Open a file.
 */
int _open(const char *name, int flags, int mode)
{
    return -1;
}
#if 1
/*==============================================================================
 * Read from a file.
 */
int _read(int file, char *ptr, int len)
{
    return 0;
}

#ifdef MSCC_STDIO_THRU_CORE_UART_APB
extern int getachar(void);
extern int sendchar(int ch);
#endif

int _read_r( void * reent, int file, char *ptr, int len)
{
#ifdef MSCC_STDIO_THRU_CORE_UART_APB

    int  i;

    for (i = 0; i < len; i++)
    {
        ptr[i] = getachar();
        sendchar(ptr[i]);
 #ifdef UART_AUTO_ECHO
      _uart_putc (buf[i]);
#endif
      /* Return partial buffer if we get EOL */
      if (('\r' == ptr[i])||('\n' == ptr[i]))
        {
          ptr[i] = '\n';
          return  i;
        }
    }

    return  i;          /* Filled the buffer */
#else
    return 0;
#endif
}

/*==============================================================================
 * Write to a file. libc subroutines will use this system routine for output to
 * all files, including stdout—so if you need to generate any output, for
 * example to a serial port for debugging, you should make your minimal write
 * capable of doing this.
 */
int _write_r( void * reent, int file, char * ptr, int len )
{
#ifdef MSCC_STDIO_THRU_CORE_UART_APB
    int count_out;
    /*--------------------------------------------------------------------------
     * Output text to the UART.
     */
    count_out = 0;
    while(len--)
    {
        sendchar(ptr[count_out]);
        count_out++;
    }

    return len;
#else   /* MSCC_STDIO_THRU_CORE_UART_APB */
    return 0;
#endif  /* MSCC_STDIO_THRU_CORE_UART_APB */
}
#endif
/*==============================================================================
 * Increase program data space. As malloc and related functions depend on this,
 * it is useful to have a working implementation. The following suffices for a
 * standalone system; it exploits the symbol _end automatically defined by the
 * GNU linker.
 */
caddr_t _sbrk(int incr)
{
    extern char _end;       /* Defined by the linker */
    static char *heap_end;
    char *prev_heap_end;
    char * stack_ptr;

    if (heap_end == 0)
    {
      heap_end = &_end;
    }

    prev_heap_end = heap_end;

    asm volatile ("MRS %0, msp" : "=r" (stack_ptr) );
    if(heap_end < stack_ptr)
    {
        /*
         * Heap is at an address below the stack, growing up toward the stack.
         * The stack is above the heap, growing down towards the heap.
         * Make sure the stack and heap do not run into each other.
         */
        if (heap_end + incr > stack_ptr)
        {
          _write_r ((void *)0, 1, "Heap and stack collision\n", 25);
          _exit (1);
        }
    }
    else
    {
        /*
         * If the heap and stack are not growing towards each other then use the
         * _eheap linker script symbol to figure out if there is room left on
         * the heap.
         * Please note that this use case makes sense when the stack is located
         * in internal eSRAM in the 0x20000000 address range and the heap is
         * located in the external memory in the 0xA0000000 memory range.
         * Please note that external memory should not be accessed using the
         * 0x00000000 memory range to read/write variables/data because of the
         * SmartFusion2 cache design.
         */
        extern char _eheap;     /* Defined by the linker */
        char *top_of_heap;

        top_of_heap = &_eheap;
        if(heap_end + incr  > top_of_heap)
        {
          _write_r ((void *)0, 1, "Out of heap memory\n", 25);
          _exit (1);
        }
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

/*==============================================================================
 * Status of a file (by name).
 */
int _stat(char *file, struct stat *st)
{
    st->st_mode = S_IFCHR;
    return 0;
}

/*==============================================================================
 * Timing information for current process.
 */
int _times(struct tms *buf)
{
    return -1;
}

/*==============================================================================
 * Remove a file's directory entry.
 */
int _unlink(char *name)
{
    errno = ENOENT;
    return -1;
}

/*==============================================================================
 * Wait for a child process.
 */
int _wait(int *status)
{
    errno = ECHILD;
    return -1;
}

#ifdef __cplusplus
}
#endif

