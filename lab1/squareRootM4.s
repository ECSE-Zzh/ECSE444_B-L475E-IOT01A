/*
 * squareRootM4.s
 *
 *  Created on: Sep 12, 2023
 *      Author: Zoey
 */

 //unified indicates that we're using a mix of different ARM instructions,
 //e.g., 16-bit Thumb and 32-bit ARM instructuions may be present (and are)
 .syntax unified

 //.global exports the label asmMax, which is expected by lab1math.h
 .global squareRootM4

 //.sections marks a new section in assembly. .text identified it as source code;
 //.rodata marks it as read-only, setting it to go in FLASH, not SRAM
 .section .text.rodata


/**
 * S0默认是浮点数, call function时浮点数x的值自动默认给S0
 */
 //R0 = pointer to output
 squareRootM4:
 	VCMP.F32 S0, #0
 	VMRS APSR_nzvc, FPSCR
 	BLT NO_SOLUTION

 	VSQRT.F32 S1, S0
 	VSTR.F32 S1, [R0]
 	BX LR

 NO_SOLUTION:
 	VMOV.F32 S2, #31
 	VSTR.F32 S2, [R0]
 	BX LR
