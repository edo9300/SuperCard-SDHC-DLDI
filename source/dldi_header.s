// SPDX-License-Identifier: Zlib
//
// Copyright (C) 2006-2016 Michael Chisholm (Chishm)
// Copyright (C) 2006-2016 Dave Murphy (WinterMute)

#include <nds/arm9/dldi_asm.h>

    .syntax unified
    .section ".crt0","ax"
    .global _start
    .align    4
    .arm

@ Driver patch file standard header -- 16 bytes

    .word   0xBF8DA5ED                  @ DLDI identifier - magic number
    .asciz  " Chishm"                   @ DLDI identifier - magic string (8 bytes with null terminator)
    .byte   0x01                        @ DLDI identifier - DLDI version number
#if defined(__BLOCKSDS__ )
    .byte   __dldi_header_driver_size   @ Log [base-2] of the size of this driver in bytes.
                                        @ Calculated automatically in the link script.
    .byte   __dldi_header_fix_flags     @ Sections to fix.
                                        @ Calculated automatically in the link script.
#else
	.byte	DLDI_SIZE_4KB
	.byte	FIX_BSS	@ Sections to fix
#endif
    .byte   0x00                        @ Space allocated in the .nds file; leave empty.

@ Text identifier - can be anything up to 47 chars + terminating null -- 48 bytes

    .align  4
    .asciz "SuperCard (SD Card)"

@ Offsets to important sections within the data -- 32 bytes

    .align  6
    .word   __text_start    @ data start
    .word   __data_end      @ data end
    .word   __glue_start    @ Interworking glue start -- Needs address fixing
    .word   __glue_end      @ Interworking glue end
    .word   __got_start     @ GOT start               -- Needs address fixing
    .word   __got_end       @ GOT end
    .word   __bss_start     @ bss start               -- Needs setting to zero
    .word   __bss_end       @ bss end

@ IO_INTERFACE data -- 32 bytes

    .ascii  "SCSD"          @ ioType (Normally "DLDI")
#if defined(ARM9) || !defined(__BLOCKSDS__ )
    .word   FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE | FEATURE_SLOT_GBA
#else
    .word   FEATURE_MEDIUM_CANREAD | FEATURE_MEDIUM_CANWRITE | FEATURE_SLOT_GBA | FEATURE_ARM7_CAPABLE
#endif
    .word   startup         @ Function pointers to standard device driver functions
    .word   isInserted
    .word   readSectors
    .word   writeSectors
    .word   clearStatus
    .word   shutdown

_start:

    .align
    .pool
    .end
