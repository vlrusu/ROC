/*******************************************************************************
 * (c) Copyright 2016 Microsemi Corporation.  All rights reserved.
 * 
 * Example Cortex-M1 linker script for debug and release builds.
 *
 * SVN $Revision: 8322 $
 * SVN $Date: 2016-03-15 17:33:51 +0000 (Tue, 15 Mar 2016) $
 */
 /* 
  * Below is a suggested memory map. The memory map is constrained by what is allowed
  * by the Cortex-M1.
  * Cortex-M1 map as defined by ARM(c)
  *
  *  ---Address Range--------------R/W/X-------mem type -----------------------
  *  ---0x00000000-0x0FFFFFFF------R/W/X-------Code-uPROM location on startup--
  *  ---0x10000000-0x1FFFFFFF------R/W/X-------Code-ITCM location on startup---
  *  ---0x20000000-0x3FFFFFFF------R/W/X!------SRAM-----note: X!= when DTCM ---
  *  ---0x40000000-0x5FFFFFFF------R/W---------PERIPHERAL----------------------
  *  ---0x60000000-0x9FFFFFFF------R/W/X-------SRAM----------------------------
  *  ---0xA0000000-0xDFFFFFFF------R/W---------External Device-----------------
  *  ---0xE0000000-0xE00FFFFF------R/W---------Internal private peripheral bus-
  *  ---0xE0100000-0xFFFFFFFF------R/W/X-------Reserved------------------------
  *  
  *  Suggested memory map for consistency across Libero designs
  *
  *  ---Start Address---------contains -----------------mem type---------------
  *  ---0x00000000------------Code----------------------RTG4 uPROM-------------
  *  ---0x10000000------------Code copied here----------ITCM-------------------
  *  ---0x20000000------------BSS,Stack,heap, init data-RAM-DTCM---------------
  *  ---0x30000000------------Code to switch uPROM/ITCM-RAM--------------------
  *  ---0x40000000------------Peripherals, CoreUARTapb, CoreGPIO etc-----------
  *  ---0x60000000------------External memory e.g. DDR3 east on RTG4-----------
  *  ---0x70000000------------External memory e.g. DDR3 west on RTG4-----------
 */
 
OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm",
          "elf32-littlearm")
GROUP(-lc -lgcc -lm)
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)
SEARCH_DIR(.)
__DYNAMIC  =  0;

/*******************************************************************************
 * Start of board customization.
 *******************************************************************************/
MEMORY
{
    /* note: ROM declared below will be RAM located at 0x00000000 when debugging */
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 128k           /* This is uPROM and then switched to ITCM */
    switch_code (rx) : ORIGIN = 0x30000000, LENGTH = 0k     /* This is used to switch from the boot loader to the main program */
                                                            /* it is used when boot-loader is loading a program (option 0 detailed below) */
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 128k           /* DTCM */    
    ram_batt (rwx) : ORIGIN = 0x80000000, LENGTH = 0k       /* battery backed RAM- none on RTG4 eval kit */    
}

RAM_START_ADDRESS = 0x20000000;  /* Must be the same value MEMORY region ram ORIGIN above. */
RAM_SIZE            = 128k;       /* Must be the same value MEMORY region ram LENGTH above. */
MAIN_STACK_SIZE     = 2k;        /* Cortex main stack size. */
PROCESS_STACK_SIZE  = 0k;        /* Cortex process stack size (only available with OS extensions).*/
MIN_SIZE_HEAP       = 0k;        /* needs to be calculated for your application */

/*******************************************************************************
 * End of board customization.
 *******************************************************************************/
/*
*  Instructions for startup code. Option 1 is the default option.
*  set _memory_remap_option to one of the following depending on program being written:
*
*  1: Cold boot program- Remapping of ITCM to zero when cold booting from a uPROM.
*     Copies program at 0x00000000 (normally uPROM on RTG4) to ITCM at 0x10000000 and remaps ITCM to
*     0x00000000 and then runs from ITCM. This is the configuration recommended from RTG4 if not 
*     using a boot-loader.
*     Note: When debugging, the debugger will locate ITCM to zero, download, and run from there.
*  2: A program called from a boot-loader and run from DDR (vector table not located at 0x00000000 at start)
*     The program is copied from uPROM or external FLASH by a boot-loader and loaded to DDDR
*     On startup, the following must be done:
*     a. The stack pointer modified to correct value ( On start it will contain the boot-loaders value )
*     b. The vector table needs to be relocated to 0x00000000. This should be ITCM.
*     c. Recommended- Relocate interrupt routines to ITCM for performance reasons along with any other routines that will 
*        help performance
*     d: Recommended-Locate the stack in ITCM if size OK.
*     Note 1: When debugging, the debugger can just download to DDR, no need for the debugger
*     to remap. (Assumption is the DDR is initialized independently by ABC block in RTG4)
*     Note 2: The debugger does not need to do anything other than load the program. The program itself
*     relocates the vector table and makes sure the stack pointer is correct.
*  3: Cold boot program- No remapping- keeping running from 0x00000000. This is normally a boot-loader. 
*     On an RTG4, it will physically be a uPROM on the RTG4.
*     On the polarfire, it will be ITCM which will be filled from non-volatile memory using a UIC script
*     before Cortex-m1 is let out of reset.
*     Note 1: The debugger will set the auxillary control register so ITCM is located at zero and copy 
*     the program to there before starting to debug.
*     Note 2: This is the normal configuration if the program is a boot-loader. 
*     Normally option 0 above will be used where you are not using a boot-loader.
*     In that case, the program is copied to ITCM and run from there for performance reasons.
*/

PROVIDE (_startup_option = 1);  /* This is option 1 by default */
 
PROVIDE (__main_stack_start = RAM_START_ADDRESS + RAM_SIZE);

SECTIONS
{
    .vector_table : ALIGN(0x4)
    {
        __vector_table_load = LOADADDR(.vector_table);
        __vector_table_start = .;
        __vector_table_vma_base_address = .;
        KEEP(*(.isr_vector))
        . = ALIGN(0x4);
        _vector_table_end_load = .;
        _evector_table = .;
        __vector_table_vma_base_address = .;            /* required by debugger for start address */
    } >rom
    
    .boot_code : ALIGN(0x4) /* When all code in the same location, no requirement for this section- but adds clarity when looking at .lst file */
    {                        
        *(.boot_code)
        . = ALIGN(0x4);
    } >rom

    .text :
    {
        CREATE_OBJECT_SYMBOLS
        __text_load = LOADADDR(.text);
        __text_start = .;
        *(.text .text.* .gnu.linkonce.t.*)
        *(.plt)
        *(.gnu.warning)
        *(.glue_7t) *(.glue_7) *(.vfp11_veneer)
        . = ALIGN(0x4);
        /* These are for running static constructors and destructors under ELF.  */
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))
        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

        *(.rodata .rodata.* .gnu.linkonce.r.*)

        *(.ARM.extab* .gnu.linkonce.armextab.*)
        *(.gcc_except_table) 
        *(.eh_frame_hdr)
        *(.eh_frame)

        KEEP (*(.init))
        KEEP (*(.fini))

        PROVIDE_HIDDEN (__preinit_array_start = .);
        KEEP (*(.preinit_array))
        PROVIDE_HIDDEN (__preinit_array_end = .);
        PROVIDE_HIDDEN (__init_array_start = .);
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        PROVIDE_HIDDEN (__init_array_end = .);
        PROVIDE_HIDDEN (__fini_array_start = .);
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        PROVIDE_HIDDEN (__fini_array_end = .);
    } >rom
  
    /* .ARM.exidx is sorted, so has to go in its own output section.  */
    __exidx_start = .;
    .ARM.exidx :
    {
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
    } >rom
    __exidx_end = .;
    _etext = .;
    PROVIDE(__text_end = .);
    
    /* 
    *   The .ram_code section will contain the code when switching memory and program. 
    *   This must be run from memory that is not being switched if the programs are different.
    *   Code is copied by startup code and run when switching from code running in uPROM at address 
    *   0x00000000 to a different program from ITCM and mapped to 0x00000000.  
    *   If the programs are the same, there is no need to relocate the switch code to RAM before running
    *   option 0: relocate when switching to a program. This will be called from your boot-loader application
    *   code. Relplace the relocate line below (>rom) with (>switch_code AT>rom)
    *   option 1: no need to relocate, leave section location line as (>rom)
    */
    .ram_code :
    {
        . = ALIGN (4);
        __sc_load = LOADADDR (.ram_code);
        __sc_start = .;
        *(.ram_codetext)        /* .ram_codetext sections (code) */
        *(.ram_codetext*)       /* .ram_codetext* sections (code)  */
        *(.ram_coderodata)      /* read-only data (constants) */
        *(.ram_coderodata*)
        . = ALIGN (4);
        __sc_end = .;
    } >rom

    .data :
    {
        __data_load = LOADADDR (.data);
        __data_start = .;
        KEEP(*(.jcr))
        *(.got.plt) *(.got)
        *(.shdata)
        *(.data .data.* .gnu.linkonce.d.*)
        . = ALIGN (4);
        _edata = .;
    } >ram AT>rom 
  
  /* The _uPROM_start and _uPROM_end are used when copying contents to ITCM. It includes the text section, although no real need */
  /* in our configuration, as will not be copied on a soft reset */
  _uPROM_start = LOADADDR (.vector_table);
  _uPROM_end  = (LOADADDR (.data) + SIZEOF(.data));
    
    .bss :
    {
        __bss_start__ = . ;
        *(.shbss)
        *(.bss .bss.* .gnu.linkonce.b.*)
        *(COMMON)
        . = ALIGN (4);
        __bss_end__ = .;
        _end = .;
        __end = _end;
        PROVIDE(end = .);
    } >ram
    
    .heap : ALIGN(0x4)
    {
        __heap_start__ = .;
        . += MIN_SIZE_HEAP;         /* will generate error if this minimum size not available */
        . += ((ABSOLUTE(RAM_START_ADDRESS) + RAM_SIZE - (MAIN_STACK_SIZE + PROCESS_STACK_SIZE)) - .); /* assumes stack starts after heap */
        _eheap = .;
    } >ram

    /* 
    * The .stack section is only specified here in order for the linker to generate
    * an error if the ram is full.
    */
    .stack :
    {
        __stack_start__ = .;
        . = ALIGN(4);
        . += PROCESS_STACK_SIZE;
        . = ALIGN(4);
        . += MAIN_STACK_SIZE;
        . = ALIGN(4);
        _estack = .;
    } >ram 
    
    /* 
    *  Uninitialized data section- This is battery backed up in external RAM
    * To use, declare the following in you code
    * #define NON_INIT_RAM_SECTION __attribute__ ((section (".noinit_sec_ram")))
    * and then each variable you want to place here, put above define
    * in front of the declaration e.g. 
    * NON_INIT_RAM_SECTION char array_not_initalized[12];
    */
    . = ALIGN(4);
    .noinit_section_ram :
    {
        _snoinit_sec_ram = .;         
        *(.noinit_section_ram)
        *(.noinit_section_ram*) 
        *(COMMON)
        . = ALIGN(4);
        _enoinit_sec_ram = .;         
    } >ram_batt

    .stab 0 (NOLOAD) :
    {
        *(.stab)
    }

  .stabstr 0 (NOLOAD) :
  {
    *(.stabstr)
  }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) *(.isr_vector) }
}
