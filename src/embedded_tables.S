/* phys2d - embedded precomputed data tables (.incbin).
   The blob path is provided by the build system through PHYS2D_TABLE_BLOB. */

#ifndef PHYS2D_TABLE_BLOB
#define PHYS2D_TABLE_BLOB "phys2d_tables.bin"
#endif

#if defined(_WIN32) || defined(__CYGWIN__)
    .section .rdata,"dr"
#elif defined(__APPLE__)
    .section __TEXT,__const
#else
    .section .rodata
#endif

    .balign 64
    .globl phys2d_embedded_tables
phys2d_embedded_tables:
    .incbin PHYS2D_TABLE_BLOB
    .globl phys2d_embedded_tables_end
phys2d_embedded_tables_end:

/* COFF/PE cannot emit a relocated symbol difference inside .quad, so the size
   is folded into an absolute symbol with .set first (works on ELF too). */
    .set phys2d_table_byte_count, phys2d_embedded_tables_end - phys2d_embedded_tables

    .balign 8
    .globl phys2d_embedded_tables_size
phys2d_embedded_tables_size:
    .quad phys2d_table_byte_count

#if !defined(_WIN32) && !defined(__CYGWIN__) && !defined(__APPLE__)
    .section .note.GNU-stack,"",%progbits
#endif
