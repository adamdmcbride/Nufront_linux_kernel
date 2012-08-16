/** ============================================================================
 *  @file   zsp800m_hal_pwr.c
 *
 *  @path   $(APUDRV)/gpp/src/arch/ZSP800M/
 *
 *  @desc   Power management module.
 *
 *  @ver    0.01.00.00
 *  ============================================================================
 *  Copyright (C) 2011-2012, Nufront Incorporated - http://www.nufront.com/
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the
 *  Free Software Foundation version 2.
 *
 *  This program is distributed "as is" WITHOUT ANY WARRANTY of any kind,
 *  whether express or implied; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  General Public License for more details.
 *  ============================================================================
 */
/*  ----------------------------------- OS Specific Headers         */
#include <linux/clk.h>
#include <asm/io.h>

/*  ----------------------------------- APU Driver                  */
#include <_apudrv.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <zsp800m_hal.h>
#include <zsp800m_hal_power.h>


#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
 *  @macro  COMPONENT_ID
 *
 *  @desc   Component and Subcomponent Identifier.
 *  ============================================================================
 */
#define  COMPONENT_ID       ID_ARCH_HAL_PWR


/** ============================================================================
 *  @macro  SET_FAILURE_REASON
 *
 *  @desc   Sets failure reason.
 *  ============================================================================
 */
#if defined (DDSP_DEBUG)
#define SET_FAILURE_REASON   TRC_SetReason (status, FID_C_ARCH_HAL_PWR, __LINE__)
#else
#define SET_FAILURE_REASON
#endif /* if defined (DDSP_DEBUG) */


/*  ============================================================================
 *  @const  LPSC_GEM
 *
 *  @desc   GEM module number in PSC.
 *  ============================================================================
 */
#define LPSC_GEM            1

/*  ============================================================================
 *  @const  DSP_BT/PTCMD/PTSTAT/MDSTAT_DSP/MDCTL_DSP
 *
 *  @desc   Offset of the PSC module registers from the base of the CFG
 *          memory.
 *  ============================================================================
 */
#define DSP_BT              0x0014
#define EPCPR               0x1070
#define PTCMD               0x1120
#define PTSTAT              0x1128
#define PDSTAT              0x1200
#define PDCTL               0x1300
#define MDSTAT_DSP          0x1800 + (4 * LPSC_GEM)
#define MDCTL_DSP           0x1A00 + (4 * LPSC_GEM)

/*  ============================================================================
 *  @const  MDCTL_NEXT_MASK
 *
 *  @desc   Mask to start power transition.
 *  ============================================================================
 */
#define MDCTL_NEXT_MASK     (0x0000001Fu)

/*  ============================================================================
 *  @const  MDCTL_NEXT_SHIFT
 *
 *  @desc   Mask to start power transition.
 *  ============================================================================
 */
#define MDCTL_NEXT_SHIFT    (0x00000000u)

/*  ============================================================================
 *  @const  PDCTL_NEXT_MASK
 *
 *  @desc   Mask to start power transition.
 *  ============================================================================
 */
#define PDCTL_NEXT_MASK     (0x00000001u)

/*  ============================================================================
 *  @const  PDCTL_NEXT_SHIFT
 *
 *  @desc   Mask to start power transition.
 *  ============================================================================
 */
#define PDCTL_NEXT_SHIFT    (0x00000000u)

/*  ============================================================================
*  @const  BITPOS_DSP_BOOT_RESET
*
*  @desc   Bit position of the zsp_reset-n interrupt in the
*         CLKCONTRL register.
*  ============================================================================
*/
#define BITPOS_DSP_BOOT_RESET         14

/*  ============================================================================
*  @const  BITPOS_DSP_BOOT_RESET
*
*  @desc   Bit position of the zsp_reset-n interrupt in the
*         CLKCONTRL register.
*  ============================================================================
*/
#define BITPOS_DSP_POWER_STATE        7

/*  ============================================================================
*  @const  OFFSET_DSPCLK
*
*  @desc   Offset of the DSP clock control register from power reset and clock module base.
*  ============================================================================
*/
#if PLATFORM_PBX
#define  OFFSET_DSPCLK          0x2CL
#else
#define  OFFSET_DSPCLK          0xb0L
#endif

/*  ============================================================================
*  @const  OFFSET_DSP_POWER_STATE
*
*  @desc   Offset of the DSP clock control register from power reset and clock module base.
*  ============================================================================
*/
#if PLATFORM_PBX
#define  OFFSET_DSP_POWER_STATE          0x2CL
#else
#define  OFFSET_DSP_POWER_STATE          0x8L
#endif

/** ============================================================================
 *  @func   ZSP800M_halPscEnable
 *
 *  @desc   Enables the PSC for GEM Module.
 *
 *  @modif  None.
 *  ============================================================================
 */
#if 0
NORMAL_API
DSP_STATUS
ZSP800M_halPscEnable (IN  Uint32 addr)
{
    DSP_STATUS status    = DSP_SOK ;
    Uint32     i ;

    TRC_1ENTER ("ZSP800M__halPscEnable", addr) ;

    /* Check if the DSP is in host-boot mode */
    if (TEST_BIT (REG (addr + DSP_BT), 17) == 0u) {

        /* Check if the DSP is not already powered up */
        if ((REG (addr  + MDSTAT_DSP) & 0x001f) != 0x03u) {
            /*
             *  Step 1 - Wait for PTSTAT.GOSTAT to clear
             */
            while (REG (addr + PTSTAT) & 0x00000001) ;

            /*
             *  Step 2 - Set MDCTLx.NEXT to new state
             */
            REG (addr + MDCTL_DSP) &= (~MDCTL_NEXT_MASK) ;
            REG (addr + MDCTL_DSP) |= (0x3u) ;

            /*
             *  Step 3 - Start power transition ( set PTCMD.GO to 1 )
             */
            REG (addr + PTCMD) = 1 ;

            /*
             *  Step 4 - Wait for PTSTAT.GOSTAT to clear
             */
            while( REG (addr + PTSTAT) & 0x00000001 ) ;

            /*
             *  Step 5 - Verify state changed
             */
            while( ( REG (addr + MDSTAT_DSP) & 0x001f ) != 0x3u );
        }
    }

    for (i = 0; i < 10000; i++) {
    }

    TRC_1LEAVE ("ZSP800M_halPscEnable", status) ;

    return status ;
}
#endif

/** ============================================================================
 *  @func   ZSP800M_halPscDisable
 *
 *  @desc   Disables the PSC for GEM Module.
 *
 *  @modif  None.
 *  ============================================================================
 */
#if 0
NORMAL_API
DSP_STATUS
ZSP800M_halPscDisable (IN  Uint32 addr)
{
    DSP_STATUS status = DSP_SOK ;

    TRC_1ENTER ("ZSP800M_halPscDisable", addr) ;

    (Void) addr ;

    TRC_1LEAVE ("ZSP800M_halPscDisable", status) ;

    return status ;
}
#endif

/** ============================================================================
 *  @func   ZSP800M_halPwrCtrl
 *
 *  @desc   Power conrtoller.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halPwrCtrl (IN         Pvoid          halObj,
                      IN         DSP_PwrCtrlCmd cmd,
                      IN OUT     Pvoid          arg)
{
    DSP_STATUS         status  = DSP_SOK ;
    ZSP800M_HalObj * halObject = NULL  ;
    Uint32               addr          ;

    TRC_3ENTER ("ZSP800M_halPwrCtrl", halObj, cmd, arg) ;

    DBC_Require (NULL != halObj) ;

    halObject = (ZSP800M_HalObj *) halObj ;

    switch (cmd) {
        case DSP_PwrCtrlCmd_PowerUp:
        {
            Uint32 val;
            addr =  halObject->basePRCMBus + OFFSET_DSP_POWER_STATE ;
            val = readl (addr);
            if (TEST_BIT (val, BITPOS_DSP_POWER_STATE)) {
                /** do nothing */
            } else {
                struct clk * clock = (struct clk*)halObject->handleClock ;
                if (clock == NULL) {
                    status = DSP_EFAIL ;
                    SET_FAILURE_REASON ;
                } else {
                    Int32 ret;
                    ret = clk_enable (clock);
                    if(ret != 0) {
                        status = DSP_EFAIL ;
                        SET_FAILURE_REASON ;
                    }
                    /* check if power up */
                    val = readl (addr);
                    if (!TEST_BIT (val, BITPOS_DSP_POWER_STATE)) {
                        status = DSP_EFAIL ;
                        SET_FAILURE_REASON ;
                    }
                }
            }
        }
        break ;

        case DSP_PwrCtrlCmd_PowerDown:
        {
            Uint32 val;
            addr =  halObject->basePRCMBus + OFFSET_DSP_POWER_STATE ;
            val = readl (addr);
            if (TEST_BIT (val, BITPOS_DSP_POWER_STATE)) {
                struct clk * clock = (struct clk *)halObject->handleClock ;

                if (clock == NULL) {
                    status = DSP_EFAIL ;
                    SET_FAILURE_REASON ;
                } else {
                    clk_disable(clock);
                    /* check if power down */
                    val = readl (addr);
                    if (TEST_BIT (val, BITPOS_DSP_POWER_STATE)) {
                        status = DSP_EFAIL ;
                        SET_FAILURE_REASON ;
                    }
                }
            }
        }
        break ;

        case DSP_PwrCtrlCmd_Reset:
        {
            Uint32 val;
            addr  =  halObject->basePRCMBus + OFFSET_DSPCLK ;
            /** stop run  */
            val = readl(addr);
            CLEAR_BIT (val, BITPOS_DSP_BOOT_RESET);
            writel(val, addr);
        }
        break ;

        case DSP_PwrCtrlCmd_Release:
        {
            Uint32 val;

            /* Release the DSP from reset. */
            addr =  halObject->basePRCMBus + OFFSET_DSPCLK ;
            val  = readl (addr);
            SET_BIT (val, BITPOS_DSP_BOOT_RESET);
            writel (val, addr);
        }
        break ;

        default:
        {
            /* Unsupported interrupt control command */
            status = DSP_EINVALIDARG ;
            SET_FAILURE_REASON ;
        }
        break ;
    }

    TRC_1LEAVE ("ZSP800M_halPwrCtrl", status) ;

    return status ;
}


#if defined (__cplusplus)
}
#endif

