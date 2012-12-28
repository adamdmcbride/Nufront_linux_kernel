/* arch/arm/mach-ns115/include/mach/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 2010 NUFRONT Ltd. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include "hardware.h"
#include "board-ns115.h"

		.macro	addruart, rp, rv
		ldr	\rp, = NS115_UART0_BASE               	@ physical
		ldr	\rv, = IO_ADDRESS(NS115_UART0_BASE)	@ virtual
		.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
