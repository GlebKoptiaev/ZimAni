ENTRY(Reset_Handler)
MEMORY { FLASH(rx) : ORIGIN = 0x08000000, LENGTH = 2048K
         RAM(xrw) : ORIGIN = 0x20000000, LENGTH = 192K }
_estack = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
 .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) } > FLASH
 .text : { . = ALIGN(4); *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) . = ALIGN(4); } > FLASH
 .ARM.extab : { *(.ARM.extab*) } > FLASH
 .ARM.exidx : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } > FLASH
 _sidata = LOADADDR(.data);
 .data : { . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .; } > RAM AT> FLASH
 .bss (NOLOAD) : { . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .; } > RAM
 .heap_stack (NOLOAD) : { . = ALIGN(8); end = .; _end = .; . += 4096; . = ALIGN(8); } > RAM
 /DISCARD/ : { *(.comment) *(.note*) }
}
ASSERT(_ebss + 4096 <= _estack, "RAM overflow")
