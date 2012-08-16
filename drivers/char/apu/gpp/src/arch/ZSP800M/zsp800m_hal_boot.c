/** ============================================================================
*  @file   zsp800m_hal_boot.c
*
*  @path   $(APUDRV)/gpp/src/arch/ZSP800M/
*
*  @desc   Boot control module.
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
#include <asm/io.h>

/*  ----------------------------------- APU Driver               */
#include <apudrv.h>
#include <_apudrv.h>

/*  ----------------------------------- Trace & Debug               */
#include <_trace.h>
#include <trc.h>
#include <print.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <zsp800m_hal.h>
#include <zsp800m_hal_boot.h>


#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
*  @macro  COMPONENT_ID
*
*  @desc   Component and Subcomponent Identifier.
*  ============================================================================
*/
#define  COMPONENT_ID       ID_ARCH_HAL_BOOT


/** ============================================================================
*  @macro  SET_FAILURE_REASON
*
*  @desc   Sets failure reason.
*  ============================================================================
*/
#if defined (DDSP_DEBUG)
#define SET_FAILURE_REASON   TRC_SetReason (status, FID_C_ARCH_HAL_BOOT, __LINE__)
#else
#define SET_FAILURE_REASON
#endif /* if defined (DDSP_DEBUG) */


/*  ============================================================================
*  @const  BOOT_BASE_ADDR
*
*  @desc   Offset of the DSP BOOT ADDR register from the base of the CFG
*          memory.
*  ============================================================================
*/
#define BOOT_BASE_ADDR      0x08

/*  ============================================================================
*  @const  OFFSET_DSPCONFIG
*
*  @desc   Offset of the DSP config from system control module base.
*  ============================================================================
*/
#define OFFSET_DSPCONFIG          0x70

/*  ============================================================================
*  @const  MASK_CFG_SVTADDR
*
*  @desc   mask of system interrupt vector table address.
*  ============================================================================
*/
#define MASK_CFG_SVTADDR          0xffffff00L

/** ============================================================================
*  @func   ZSP800M_halBootCtrl
*
*  @desc   Boot controller.
*
*  @modif  None.
*  ============================================================================
*/
NORMAL_API
DSP_STATUS
ZSP800M_halBootCtrl (IN    Pvoid  halObj,
IN       DSP_BootCtrlCmd  cmd,
IN OUT       Pvoid  arg)
{
    DSP_STATUS           status  = DSP_SOK ;
    ZSP800M_HalObj * halObject = NULL  ;
    Uint32               addr              ;

    TRC_3ENTER ("ZSP800M_halBootCtrl", halObj, cmd, arg) ;

    DBC_Require (NULL != halObj) ;

    halObject = (ZSP800M_HalObj *) halObj ;
    //addr  =  halObject->baseCfgBus
    //       + halObject->offsetSysModule
    //       + BOOT_BASE_ADDR ;

    switch (cmd) {
        case DSP_BootCtrlCmd_SetEntryPoint:
        {
            Uint32 val;
            /* set the  System Interrupt Vector Table adrress is 0x0               */
            /* when the System Mode Register (%smode) uvt bit is cleared,    */
            /* the interrupt vector table address is determined by the value     */
            /* applied to the SVTADDR[31:8] input pins at the core periphery  */
            addr  =  halObject->baseSCMBus + OFFSET_DSPCONFIG ;
            val = readl(addr);
            val &= ~MASK_CFG_SVTADDR;
            writel(val, addr);
            /* the boot address always be __start symble address for ZSP  */
        }
        break ;

        case DSP_BootCtrlCmd_SetBootComplete:
        {

        }
        break ;

        case DSP_BootCtrlCmd_ResetBootComplete:
        {

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

    TRC_1LEAVE ("ZSP800M_halBootCtrl", status) ;

    return status ;
}



#if defined (__cplusplus)
}
#endif
