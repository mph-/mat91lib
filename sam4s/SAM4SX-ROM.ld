SECTIONS
{
  /* Reset and interrupt vectors.  */
  . = 0x0000000;
  .vectors :
  {
    . = ALIGN(4);
    __vectors_start__ = .;
    *(.vectors)
  } >FLASH = 0

  /* Application code.  */
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text.*)
    *(.glue_7t) *(.glue_7)

        /* Support C constructors, and C destructors in both user code
           and the C library. This also provides support for C++ code. */
        . = ALIGN(4);
        KEEP(*(.init))
        . = ALIGN(4);
        __preinit_array_start = .;
        KEEP (*(.preinit_array))
        __preinit_array_end = .;

        . = ALIGN(4);
        __init_array_start = .;
        KEEP (*(SORT(.init_array.*)))
        KEEP (*(.init_array))
        __init_array_end = .;

        . = ALIGN(0x4);
        KEEP (*crtbegin.o(.ctors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
        KEEP (*(SORT(.ctors.*)))
        KEEP (*crtend.o(.ctors))

        . = ALIGN(4);
        KEEP(*(.fini))

        . = ALIGN(4);
        __fini_array_start = .;
        KEEP (*(.fini_array))
        KEEP (*(SORT(.fini_array.*)))
        __fini_array_end = .;

        KEEP (*crtbegin.o(.dtors))
        KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
        KEEP (*(SORT(.dtors.*)))
        KEEP (*crtend.o(.dtors))

  } >FLASH
  . = ALIGN(4);

  /* Read-only data (constants).  */
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata .rodata.*)
  } >FLASH
  . = ALIGN(4);

  /* Exception unwinding tables needed for EABI.  */
  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
  __exidx_end = .;

  __text_end__ = .;

  /* Data/code stored in FLASH that gets relocated into SRAM
     (__data_start__ and __data_end__ are addresses in SRAM).  */
  .data : AT (__text_end__)
  {
    . = ALIGN(64);
    __data_start__ = .;
    /* Vectors that can be modified at run-time.  */
    *(.dynamic_vectors)

    /* Program code stored in FLASH that gets relocated into SRAM.  */
    *(.ramtext)
    *(.ramfunc)

    /* Initialized data stored in FLASH that gets relocated into SRAM.  */
    *(.data)
    *(.data.*)

    . = ALIGN(4);
    __data_end__ = .;
    __data_load__ = LOADADDR (.data);
  } >SRAM

  /* Uninitialized data.  */
  .bss :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
    _end = .;
    end = .;
  } 

  /* Stack.  */
   __stack_start__ = ORIGIN (SRAM) + LENGTH (SRAM);

  /* Stabs debugging sections.  */
  .stab          0 : { *(.stab) }
  .stabstr       0 : { *(.stabstr) }
  .stab.excl     0 : { *(.stab.excl) }
  .stab.exclstr  0 : { *(.stab.exclstr) }
  .stab.index    0 : { *(.stab.index) }
  .stab.indexstr 0 : { *(.stab.indexstr) }
  .comment       0 : { *(.comment) }

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
}
