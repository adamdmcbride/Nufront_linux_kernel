/* linux/arch/arm/mach-ns115/dma.c
 *
 * Copyright (c) 2011 Nufront Co., Ltd.
 *		http://www.nufront.com
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/platform_device.h>
#include <linux/dma-mapping.h>

//#include <plat/devs.h>
//#include <plat/irqs.h>

//#include <mach/map.h>
#include <mach/irqs.h>

#include <mach/board-ns115.h>
#include <mach/nusmart-pl330-pdata.h>

static u64 dma_dmamask = DMA_BIT_MASK(32);

static struct resource pl330_dma_resource[] = {
    [0] = {
        .start	= NS115_DMA_330S_BASE,
        .end	= NS115_DMA_330S_BASE + SZ_4K,
        .flags	= IORESOURCE_MEM,
    },
    [1] = {
        .start	= IRQ_NS115_DMA330_INTR0,
        .end	= IRQ_NS115_DMA330_INTR27,
        .flags	= IORESOURCE_IRQ,
    },
};

static struct nusmart_pl330_platdata pl330_dma_pdata = {
    .peri = {
        [0] = DMACH_I2C0_RX,
        [1] = DMACH_I2C0_TX,
        [2] = DMACH_I2C1_RX,
        [3] = DMACH_I2C1_TX,
        [4] = DMACH_I2C2_RX,
        [5] = DMACH_I2C2_TX,
        [6] = DMACH_I2C3_RX,
        [7] = DMACH_I2C3_TX,
        [8] = DMACH_SSI0_RX,
        [9] = DMACH_SSI0_TX,
        [10] = DMACH_SSI1_RX,
        [11] = DMACH_SSI1_TX,
        [12] = DMACH_SSI2_RX,
        [13] = DMACH_SSI2_TX,
        [14] = DMACH_UART0_RX,
        [15] = DMACH_UART0_TX,
        [16] = DMACH_UART1_RX,
        [17] = DMACH_UART1_TX,
        [18] = DMACH_UART2_RX,
        [19] = DMACH_UART2_TX,
        [20] = DMACH_UART3_RX,
        [21] = DMACH_UART3_TX,
        [22] = DMACH_I2S0_RX,
        [23] = DMACH_I2S0_TX,
        [24] = DMACH_I2S1_RX,
        [25] = DMACH_I2S1_TX,
        [26] = DMACH_SSI3_RX,
        [27] = DMACH_SSI3_TX,
        [28] = DMACH_MAX,
        [29] = DMACH_MAX,
        [30] = DMACH_MAX,
        [31] = DMACH_MAX,
    },
};

static struct platform_device pl330_device_dma = {
    .name		= "nusmart-pl330",
    .id		= 0,
    .num_resources	= ARRAY_SIZE(pl330_dma_resource),
    .resource	= pl330_dma_resource,
    .dev		= {
        .dma_mask = &dma_dmamask,
        .coherent_dma_mask = DMA_BIT_MASK(32),
        .platform_data = &pl330_dma_pdata,
    },
};

static struct platform_device *nusmart_dmacs[] __initdata = {
    &pl330_device_dma,
};

static int __init nusmart_dma_init(void)
{
    platform_add_devices(nusmart_dmacs, ARRAY_SIZE(nusmart_dmacs));

    return 0;
}
//arch_initcall(nusmart_dma_init);
