/* arch/arm/mach-ns2816/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2010 NUFRONT Ltd. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef DEBUG_LL_UART_OFFSET
#define DEBUG_LL_UART_OFFSET	0x0010c000
#endif

#define UART01x_FR_TXFF		0x060
#define UART01x_FR_BUSY		0x001

#define AMBA_UART_DR		0x00
#define AMBA_UART_FR		0x14
#define AMBA_UART_ST		0x7c


		.macro	addruart,rx
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx,      #0x04000000
		movne	\rx,      #0xfb000000	@ virtual base
		orr	\rx, \rx, #0x00050000 	@DEBUG_LL_UART_OFFSET
		orr	\rx, \rx, #0x01100000 	@DEBUG_LL_UART_OFFSET
		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx, #AMBA_UART_DR]
		.endm

		.macro	waituart,rd,rx
1001:		ldr	\rd, [\rx, #AMBA_UART_FR]
		tst	\rd, #UART01x_FR_TXFF
		beq	1001b
		.endm

		.macro	busyuart,rd,rx
1002:		ldr	\rd, [\rx, #AMBA_UART_ST]
		tst	\rd, #UART01x_FR_BUSY
		bne	1002b

		.endm

