#include <nds/asminc.h>

.syntax unified
.arm

.macro LOAD_U32_ALIGNED_2WORDS srcreg, dstreg, maskreg
	ldmia  \srcreg, {r0-r7}
	and	r3, r3, \maskreg
	and	r7, r7, \maskreg
	orr	r3, r3, r1, lsr #16
	orr	r7, r7, r5, lsr #16
	stmia  \dstreg!, {r3,r7}
.endm

.macro LOAD_U32_ALIGNED_1WORDS srcreg, dstreg, maskreg
	ldmia  \srcreg, {r0-r3}
	and	r3, r3, \maskreg
	orr	r3, r3, r1, lsr #16
	str	r3, [\dstreg], #4
.endm

.macro LOAD_U32_ALIGNED_U16 srcreg, dstreg
	ldmia  \srcreg, {r0-r1}
	lsr r1, r1, #16
	strh r1, [\dstreg], #2
.endm

# void SCSD_readData32(uint32_t* buff_u32);
BEGIN_ASM_FUNC SCSD_readData32
	push {r4-r11}
	mov r8, r0
	add r9, r8, #512
	ldr r10, =#0x09100000
	ldr r11, =#0xFFFF0000
read32_loop:
	LOAD_U32_ALIGNED_2WORDS r10, r8, r11
	LOAD_U32_ALIGNED_2WORDS r10, r8, r11
	cmp r8, r9
	blt read32_loop
	pop {r4-r11}
	bx lr

# void SCSD_readData16(uint16_t* buff_u16);
BEGIN_ASM_FUNC SCSD_readData16
	push {r4-r11}
	mov r8, r0
	add r9, r8, #512
	sub r9, #2
	ldr r10, =#0x09100000
	ldr r11, =#0xFFFF0000
	# read 6 bytes in total, and performs an halfword store so that
	# the buffer becomes u32 aligned and then a single word store
	ldmia  r10, {r0-r5}
	lsr r1, r1, #16
	and	r5, r5, r11
	orr	r5, r5, r3, lsr #16
	strh r1, [r8], #2
	str	r5, [r8], #4

	LOAD_U32_ALIGNED_2WORDS r10, r8, r11
read16_loop:
	LOAD_U32_ALIGNED_2WORDS r10, r8, r11
	LOAD_U32_ALIGNED_2WORDS r10, r8, r11
	cmp r8, r9
	blt read16_loop
	LOAD_U32_ALIGNED_U16 r10, r8
	pop {r4-r11}
	bx lr
