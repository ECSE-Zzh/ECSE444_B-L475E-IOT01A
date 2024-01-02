/*
 * transcendentalFuncAsm.s
 *
 *  Created on: Sep 12, 2023
 *      Author: Zoey
 */

 //unified indicates that we're using a mix of different ARM instructions,
 //e.g., 16-bit Thumb and 32-bit ARM instructuions may be present (and are)
 .syntax unified

 //.global exports the label asmMax, which is expected by lab1math.h
 .global cosAsm

 //.sections marks a new section in assembly. .text identified it as source code;
 //.rodata marks it as read-only, setting it to go in FLASH, not SRAM
 .section .text.rodata
 //R0 = *x
 //R4 = counter
 //S0 = w(move in S1)
 //S1 = y(w moved in later)
 //S2 = nothing(y moved in later)
 //S3 = guess
 //S4 = updatedGuess
 //S5, S6, S7 = temp

 cosAsm:
 	VPUSH.F32 {S1, S2, S3, S4, S5, S6, S7}
 	MOV R4, #0
	VMOV.F32 S3, #0.5
	VMOV.F32 S4, #0.125
	VMOV.F32 S5, S1
	VMOV.F32 S1, S0 //S1=w, S0: used for cos（S0）
	VMOV.F32 S2, S5 //S2=y
	B WHILE

 WHILE:
 	ADD R4, R4, #1
 	VMOV.F32 S5, #2
 	VMUL.F32 S5, S3, S5 //2*guess
 	VMUL.F32 S6, S1, S3 //w*guess
 	VADD.F32 S6, S6, S2 //w*guess+y

	VMOV.F32 S0, S6 //S0 = w*guess+y
	PUSH {LR}
	BL arm_sin_f32 //sin(w*guess+y) = sin(S0)
	POP {LR}
	VMUL.F32 S7, S1, S0 //S7 = w*sin(w*guess+y)
	VADD.F32 S5, S7, S5 //S5 = w*sin(w*guess+y)+2*guess
	VCMP.F32 S5, #0
  	VMRS APSR_nzvc, FPSCR
  	BEQ NOSOLUTION

	VMOV.F32 S0, S6 //S0 = w*guess+y
	PUSH {LR}
	BL arm_cos_f32 //cos(S0)
	POP {LR}
	VMUL.F32 S7, S3, S3//guess^2
	VSUB.F32 S4, S0, S7//S4=cos(w*guess+y)-guess^2
	VDIV.F32 S4, S4, S5//S4 = [cos(w*guess+y)-guess^2]/[w*sin(w*guess+y)+2*guess]
	VADD.F32 S4, S4, S3//S4 = guess + [cos(w*guess+y)-guess^2]/[w*sin(w*guess+y)+2*guess]

	CMP R4, #100
 	BGT NOSOLUTION
 	B COMPARE

 UPDATE:
 	VMOV.F32 S3, S4
 	B WHILE

 COMPARE:
 	VCMP.F32 S4, S3
  	VMRS APSR_nzvc, FPSCR
  	ITE LE
  	VSUBLE.F32 S6, S3, S4
  	VSUBGT.F32 S6, S4, S3

 	VMOV.F32 S5, #0.125 //S5=0.125
 	VMUL.F32 S5, S5, S5 //S5=0.125^2
 	VMUL.F32 S5, S5, S5 //S5=0.125^4
 	VCMP.F32 S6, S5
	VMRS APSR_nzvc, FPSCR
  	BLE SOLUTION
  	B UPDATE

 SOLUTION:
	VSTR.F32 S4, [R0]
	VPOP.F32 {S1, S2, S3, S4, S5, S6, S7}
	BX LR

 NOSOLUTION:
 	VMOV.F32 S4, #31
 	VSTR.F32 S4, [R0]
	VPOP.F32 {S1, S2, S3, S4, S5, S6, S7}
	BX LR

