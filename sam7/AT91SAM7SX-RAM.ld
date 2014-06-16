SECTIONS
{
  /* Reset and interrupt vectors.  */
  . = 0x0000000;
  .vectors :
  {
    *(.vectors)
  } >SRAM = 0

  /* Application code.  */
  .text :
  {
    *(.text)
    *(.glue_7t) *(.glue_7)
    __ramtext_load__ = .;
    __ramtext_start__ = .;
    *(.ramtext)
    __ramtext_end__ = .;
  } >SRAM
  . = ALIGN(4);

  /* Read-only data (constants).  */
  .rodata :
  {
    *(.rodata .rodata.*)
  } >SRAM
  . = ALIGN(4);

  /* Exception unwinding tables needed for EABI.  */
  __exidx_start = .;
  .ARM.exidx : { *(.ARM.exidx* .gnu.linkonce.armexidx.*) }
  __exidx_end = .;

  __text_end__ = . ;

  /* Initialized data.  */
  .data : AT (__text_end__)
  {
    __data_start__ = . ;
    *(.data)
    SORT(CONSTRUCTORS)
  } >SRAM
  . = ALIGN(4);
  __data_end__ = . ;
  __data_load__ = LOADADDR (.data);

  /* Uninitialized data.  */
  .bss :
  {
    __bss_start__ = . ;
    *(.bss)
    *(COMMON)
  } 
  . = ALIGN(4);
  __bss_end__ = . ;
  _end = .;
  end = .;

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
