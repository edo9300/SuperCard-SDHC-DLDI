#include <nds/asminc.h>

.syntax unified
.arm

.macro WRITE_SINGLE_U16 srcreg, secondreg, dstreg
	orr \srcreg, \srcreg, \srcreg, lsl #20
	lsr \secondreg, \srcreg, #8
	stmia \dstreg, {\srcreg-\secondreg}
.endm

# void SCSD_writeData8(uint8_t* buff_u8);
BEGIN_ASM_FUNC SCSD_writeData8
	orr     r1, r1, r1, lsl #20
	mov     r2, r1
	lsr     r3, r1, #8
	stm     r0, {r2-r3}
	bx      lr

.macro WRITE_U16 srcreg, dstreg
	ldrh r0, [\srcreg], #2
	WRITE_SINGLE_U16 r0, r1, \dstreg
.endm

.macro WRITE_U32 srcreg, dstreg, maskreg
	ldr r0, [\srcreg], #4
	lsr r2, r0, #16
	and r0, r0, \maskreg
	WRITE_SINGLE_U16 r0, r1, \dstreg
	WRITE_SINGLE_U16 r2, r3, \dstreg
.endm

# void SCSD_writeBuffer32(uint32_t* buff_u32, uint32_t size);
BEGIN_ASM_FUNC SCSD_writeBuffer32
	push {r4,r5,r6,r7}
	mov r6, r0
	add r7, r6, r1
	ldr r5, =#0x09000000
	ldr r4, =#0xffff
write32_loop:
	WRITE_U32 r6, r5, r4
	WRITE_U32 r6, r5, r4
	cmp r6, r7
	blt write32_loop
	pop {r4,r5,r6,r7}
	bx lr

# void SCSD_writeData16(uint16_t* buff_u16);
BEGIN_ASM_FUNC SCSD_writeData16
	push {r4,r5,r6,r7}
	mov r6, r0
	add r7, r6, #512
	sub r7, #2
	ldr r5, =#0x09000000
	ldr r4, =#0xffff
	WRITE_U16 r6, r5
	WRITE_U32 r6, r5, r4
write16_loop:
	WRITE_U32 r6, r5, r4
	WRITE_U32 r6, r5, r4
	cmp r6, r7
	blt write16_loop

	WRITE_U16 r6, r5

	pop {r4,r5,r6,r7}
	bx lr
