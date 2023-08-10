/*
 * Copyright (c) 2008-2020 Thomas Roell.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Thomas Roell, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVINCED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */


/* Linker script to place sections and symbol values. Should be used together
 * with other linker script that defines memory regions FLASH and RAM.
 * It references following symbols, which must be defined in code:
 *   Reset_Handler : Entry of reset handler
 * 
 * It defines following symbols, which code can use without definition:
 *   __exidx_start
 *   __exidx_end
 *   __copy_table_start__
 *   __copy_table_end__
 *   __zero_table_start__
 *   __zero_table_end__
 *   __etext
 *   __preinit_array_start
 *   __preinit_array_end
 *   __init_array_start
 *   __init_array_end
 *   __fini_array_start
 *   __fini_array_end
 *   __data_start__
 *   __data_end__
 *   __bss_start__
 *   __bss_end__
 *   __end__
 *   end
 *   __HeapLimit
 *   __StackTop
 *   __stack
 */

MEMORY
{
   BOOT        (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00004000 /* FLASH   16k                          */
   APPLICATION (rx)  : ORIGIN = 0x08004000, LENGTH = 0x00060000 /* FLASH   384k                         */
   FWU         (rx)  : ORIGIN = 0x08064000, LENGTH = 0x00060000 /* FLASH   384k                         */
   EEPROM      (r)   : ORIGIN = 0x080c4000, LENGTH = 0x00008000 /* FLASH   32k                          */
   WIRELESS          : ORIGIN = 0x080cc000, LENGTH = 0x00034000 /* FLASH   208k (Wireless/SafeBoot/FUS) */
   CCRAM       (rx)  : ORIGIN = 0x10008000, LENGTH = 0x00001000 /* SRAM2b  4k                           */
   SRAM        (rw)  : ORIGIN = 0x20000000, LENGTH = 0x00030000 /* SRAM1   192k                         */
   IPCC        (rw)  : ORIGIN = 0x20030000, LENGTH = 0x00001000 /* SRAM2a  4k                           */
   BACKUP      (rw)  : ORIGIN = 0x20031000, LENGTH = 0x00001800 /* SRAM2a  6k                           */
   RTT         (rw)  : ORIGIN = 0x20039000, LENGTH = 0x00002800 /* SRAM2b  10k                          */
}

GROUP(libgcc.a libc.a libm.a libnosys.a)

ENTRY(Reset_Handler)

PHDRS
{
    boot      PT_LOAD  FLAGS(5);
    text      PT_LOAD  FLAGS(5);
    rodata    PT_LOAD  FLAGS(4);
    data      PT_LOAD  FLAGS(6);
    bss       PT_NULL  FLAGS(6);
    noinit    PT_NULL  FLAGS(6);
    cctext    PT_LOAD  FLAGS(5);
    ipcc      PT_NULL  FLAGS(6);
    backup    PT_NULL  FLAGS(6);
    rtt       PT_NULL  FLAGS(6);
}

SECTIONS
{
   .boot_vectors :
   {
       KEEP(*stm32wb_boot.o(.boot.vectors))
   } > BOOT :boot

   .boot_info :
   {
       KEEP(*stm32wb_boot.o(.boot.info))
   } > BOOT :boot

   .boot_rodata :
   {
       *stm32wb_boot.o(.rodata .rodata.*)
   } > BOOT :boot

   .boot_data :
   {
     __boot_data_start__ = .;
       *stm32wb_boot.o(.data .data.* .gnu.linkonce.d.*)
       . = ALIGN(8);
     __boot_data_end__ = .;
       . = ALIGN(8);
   } > SRAM AT >BOOT :data

  __boot_data_flash__ = LOADADDR(.boot_data);

   .boot_bss (NOLOAD) :
   {
     __boot_bss_start__ = .;
       *stm32wb_boot.o(.bss .bss.* .gnu.linkonce.b.*)
       *stm32wb_boot.o(COMMON)
       . = ALIGN(8);
     __boot_bss_end__ = .;
   } > SRAM :bss

   .boot_noinit (NOLOAD) :
   {
       *stm32wb_boot.o(.noinit .noinit.*)
       . = ALIGN(8);
   } > SRAM :noinit

   .boot_stack (NOLOAD) :
   {
       KEEP(*stm32wb_boot.o(.stack .stack.*))
       . = ALIGN(8);
     __boot_stack_top__ = .;
   } > SRAM

   .boot_text :
   {
       . = ALIGN(16384) - 14336;
     __boot_text_start__ = .;
       *stm32wb_boot.o(.text .text.*)
       . = ALIGN(16384) - 64;
       KEEP(*stm32wb_boot.o(.boot.aes128_get_key))
       . = ALIGN(16384);
   } > BOOT :boot

   .text :
   {
     __Vectors = .;
       KEEP(*(.vectors))
       
       . = 0x140;

     __info_start__ = .;
       KEEP(*(.info.uuid))
       KEEP(*(.info.version))
       KEEP(*(.info.sequence))
       . += 24;
       . += 16;
       . += 256;
     __info_end__ = .;
       . = ALIGN(8);
       
       *(.text .text.* .gnu.linkonce.t.*)

       KEEP(*(.init))
       KEEP(*(.fini))

       /* .ctors */
       *crtbegin.o(.ctors)
       *crtbegin?.o(.ctors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
       *(SORT(.ctors.*))
       *(.ctors)

       /* .dtors */
       *crtbegin.o(.dtors)
       *crtbegin?.o(.dtors)
       *(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
       *(SORT(.dtors.*))
       *(.dtors)
       . = ALIGN(8);
   } > APPLICATION :text

   __etext = . ;

   .rodata :
   {
       *(.rodata .rodata.* .gnu.linkonce.r.*)

       KEEP(*(.eh_frame*))
       . = ALIGN(4);

       /* preinit data */
       PROVIDE_HIDDEN (__preinit_array_start = .);
       KEEP(*(.preinit_array))
       PROVIDE_HIDDEN (__preinit_array_end = .);
       . = ALIGN(4);

       /* init data */
       PROVIDE_HIDDEN (__init_array_start = .);
       KEEP(*(SORT(.init_array.*)))
       KEEP(*(.init_array))
       PROVIDE_HIDDEN (__init_array_end = .);
       . = ALIGN(4);

       /* finit data */
       PROVIDE_HIDDEN (__fini_array_start = .);
       KEEP(*(SORT(.fini_array.*)))
       KEEP(*(.fini_array))
       PROVIDE_HIDDEN (__fini_array_end = .);

     __stm32wb_rtc_info__ = .;
       LONG  (DEFINED(__RTC_EPOCH__) ? __RTC_EPOCH__ : 0)
       SHORT (((DEFINED(__RTC_ZONE__) ? __RTC_ZONE__ : 0) + 43200) / 900)
       BYTE  ((DEFINED(__RTC_DST__) ? __RTC_DST__ : 0) / 3600)
       BYTE  ((DEFINED(__RTC_LEAP_SECONDS__) ? __RTC_LEAP_SECONDS__ : 18) + 128)
       
     __copy_table_start__ = .;
       LONG (LOADADDR(.data))
       LONG (__data_start__)
       LONG (__data_end__ - __data_start__)
     __copy_table_end__ = .;
     
     __zero_table_start__ = .;
       LONG (__bss_start__)
       LONG (__bss_end__ - __bss_start__)
     __zero_table_end__ = .;
       . = ALIGN(8);
   } > APPLICATION :rodata

   .ARM.extab :
   {
       *(.ARM.extab* .gnu.linkonce.armextab.*)
       . = ALIGN(8);
   } > APPLICATION :rodata

   .ARM.exidx :
   {
       PROVIDE_HIDDEN(__exidx_start = .);
       *(.ARM.exidx* .gnu.linkonce.armexidx.*)
       PROVIDE_HIDDEN(__exidx_end = .);
       . = ALIGN(8);
   } > APPLICATION :rodata

   .data :
   {
     __data_start__ = .;
       *(vtable)
       *(.data .data.* .gnu.linkonce.d.*)
       . = ALIGN(8);
     __data_end__ = .;
       . = ALIGN(8);
   } > SRAM AT >APPLICATION :data

   .bss (NOLOAD) :
   {
     __bss_start__ = .;
       *(.bss .bss.* .gnu.linkonce.b.*)
       *(COMMON)
       . = ALIGN(8);
     __bss_end__ = .;
   } > SRAM :bss

   .noinit (NOLOAD) :
   {
       *(.noinit .noinit.*)
       . = ALIGN(8);
   } > SRAM :noinit

   __end__ = .;
   PROVIDE(end = .);

   .heap (NOLOAD) :
   {
     __HeapBase = .;
       *(.heap .heap.*)
       . = ALIGN(32);
     __HeapLimit = .;
   } > SRAM

   /* Set stack top to end of RAM, and stack limit move down by
    * size of stack_dummy section */
   .stack_dummy (NOLOAD):
   {
       *(.stack .stack.*)
       . = ALIGN(32);
   } > SRAM

   __StackTop = ORIGIN(SRAM) + LENGTH(SRAM);
   __StackLimit = __StackTop - SIZEOF(.stack_dummy);
   PROVIDE(__HeapLimit = __StackLimit);
   PROVIDE(__stack = __StackTop);

   .ipcc (NOLOAD) :
   {
     __ipcc_start__ = .;
       KEEP(*(.ipcc .ipcc.*))
       . = ALIGN(8);
     __ipcc_end__ = .;
   } >IPCC :ipcc

   .backup_bss (NOLOAD) :
   {
     __backup_bss_start__ = .;
       *(.backup.bss .backup.bss.*)
       . = ALIGN(8);
     __backup_bss_end__ = .;
   } > BACKUP :backup

   .backup_noinit (NOLOAD) :
   {
       *(.backup.noinit .backup.noinit.*)
       . = ALIGN(8);
   } > BACKUP :backup

   .cctext :
   {
     __cctext_start__ = .;
       *(.cctext .cctext.*)
       . = ALIGN(8);
     __cctext_end__ = .;
   } >CCRAM AT >APPLICATION :cctext

   __cctext_flash__ = LOADADDR(.cctext);

   .rtt_control (NOLOAD) :
   {
       KEEP(*(.rtt_control))
       . = ALIGN(8);
   } > RTT :rtt

   .rtt_data (NOLOAD) :
   {
       KEEP(*(.rtt_data .rtt_data.*))
       . = ALIGN(8);
   } > RTT :rtt

   __application_flash__ = LOADADDR(.cctext) + SIZEOF(.cctext);

   __FlashBase = ALIGN(LOADADDR(.cctext) + SIZEOF(.cctext), 4096);
   __FlashLimit = ORIGIN(APPLICATION) + LENGTH(APPLICATION);

   __ccram_start__ = ORIGIN(CCRAM);
   __ccram_end__ = ORIGIN(CCRAM) + LENGTH(CCRAM);

   __application_start__ = ORIGIN(APPLICATION);
   __application_end__ = ORIGIN(APPLICATION) + LENGTH(APPLICATION);

   __fwu_start__ = ORIGIN(FWU);
   __fwu_end__ = ORIGIN(FWU) + LENGTH(FWU);

   __eeprom_start__ = ORIGIN(EEPROM);
   __eeprom_end__ = ORIGIN(EEPROM) + LENGTH(EEPROM);

   __wireless_start__ = ORIGIN(WIRELESS);
}
