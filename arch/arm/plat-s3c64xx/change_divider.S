/**
* linux/arch/arm/plat-s3c64xx/change_divider.S
*
* Copyright (c) 2009 Barnes and Noble, Inc
* All rights reserved
*
* Module author: Intrinsyc Software, Inc. <dbolcsfoldi@intrinsyc.com>
*
* S3C64XX ARMCLK divider change routine.
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*/

#include <linux/linkage.h>
#include <asm/assembler.h>

#include <plat/map.h>
#include <plat/regs-clock.h>

		.text
ENTRY(s3c64xx_change_divider)
		stmfd sp!, {r0-r5}

		ldr r1, =S3C_CLK_DIV0
		mov r3, #0
loopcd:	
		mov r4, #0
		mcr p15, 0, r4, c7, c10, 4 @ data synchronization barrier instruction
		mcr p15, 0, r4, c7, c10, 5 @ data memory barrier operation
		cmp r3, #1
		streq r0, [r1]
		mcr p15, 0, r4, c7, c5, 4 @ flush prefetch buffer
loop1000:
		add r4, r4, #1
		cmp r4, #0x1000
		bne loop1000
		cmp r3, #1
		add r3, r3, #1
		bne loopcd
		ldmfd sp!,{r0-r5}
		mov pc, lr
		.end
