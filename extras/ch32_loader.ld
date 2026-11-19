SECTIONS {
  . = 0x20000000;
  .text : { *(.text.entry) *(.text*) *(.rodata*) }
  /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
