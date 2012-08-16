/** ============================================================================
 *  @file   zsp800m_hal_intgen.c
 *
 *  @path   $(APUDRV)/gpp/src/arch/ZSP800M/
 *
 *  @desc   Hardware Abstraction Layer for DavinciHD.
 *          Defines necessary functions for Interrupt Handling.
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
 
/*  ----------------------------------- OS Headers                  */
#include <asm/io.h>
/* for debug */
#if defined(DEBUG_ZSPSIDE)
#include <linux/kernel.h>
#endif
/*  ----------------------------------- APU Driver               */
#include <_apudrv.h>
/* for debug */
#if defined(DEBUG_ZSPSIDE)
#include <osal.h>
#endif

/*  ----------------------------------- Trace & Debug               */
#include <_trace.h>
#include <trc.h>
#include <print.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <zsp800m_hal.h>
#include <zsp800m_hal_intgen.h>


#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
 *  @macro  COMPONENT_ID
 *
 *  @desc   Component and Subcomponent Identifier.
 *  ============================================================================
 */
#define  COMPONENT_ID       ID_ARCH_HAL_INTGEN


/** ============================================================================
 *  @macro  SET_FAILURE_REASON
 *
 *  @desc   Sets failure reason.
 *  ============================================================================
 */
#if defined (DDSP_DEBUG)
#define SET_FAILURE_REASON   TRC_SetReason (status, FID_C_ARCH_HAL_INTGEN, __LINE__)
#else
#define SET_FAILURE_REASON
#endif /* if defined (DDSP_DEBUG) */


/*  ============================================================================
 *  @const  OFFSET_DSPINT
 *
 *  @desc   Offset of the DSPINT ARM to DSP interrupt status from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_DSPINT             0x006C  //??

/*  ============================================================================
 *  @const  OFFSET_DSPINTSET
 *
 *  @desc   Offset of the DSPINTSET ARM to DSP interrupt set from
 *          system module base.
 *  ============================================================================
 */
#define OFFSET_DSPINTSET	      0x006C

/*  ============================================================================
 *  @const  OFFSET_DSPINTCLR
 *
 *  @desc   Offset of the DSPINTCLR ARM to DSP interrupt clear from
 *          system module base.
 *  ============================================================================
 */
//#define  OFFSET_DSPINTCLR         0x68       //??

/*  ============================================================================
 *  @const  OFFSET_DSPINTCLR
 *
 *  @desc   Offset of the DSPINTCLR ARM to DSP interrupt clear from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_DSPINTMASK         0x68  //??

/*  ============================================================================
 *  @const  OFFSET_DSPRO
 *
 *  @desc   Offset of the DSPINTCLR ARM to DSP interrupt clear from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_DSPRO              0x74

/*  ============================================================================
 *  @const  OFFSET_DSPNMINTSET
 *
 *  @desc   Offset of the DSP no-mask interrupt set from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_DSPNMINTSET         0x78


/*  ============================================================================
 *  @const  OFFSET_ARMINT
 *
 *  @desc   Offset of the ARMINT DSP to ARM interrupt status from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_ARMINT            0x70 //??

/*  ============================================================================
 *  @const  OFFSET_ARMINTSET
 *
 *  @desc   Offset of the ARMINTSET DSP to ARM interrupt set from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_ARMINTSET         0x0068

/*  ============================================================================
 *  @const  OFFSET_ARMINTCLR
 *
 *  @desc   Offset of the ARMINTCLR DSP to ARM interrupt clear from
 *          system module base.
 *  ============================================================================
 */
#define  OFFSET_ARMINTCLR         0x78 //??


/*  ============================================================================
 *  @const  BITPOS_ARM2DSPINTSET
 *
 *  @desc   Start position of the ARM2DSP interrupt set bits in the DSPINTSET
 *          register.
 *  ============================================================================
 */
#define  BITPOS_ARM2DSPINTSET   0

/*  ============================================================================
 *  @const  BITPOS_ARM2DSPSTATUS
 *
 *  @desc   Start position of the ARM2DSP interrupt status bits in the DSPINT
 *          register.
 *  ============================================================================
 */
#define  BITPOS_ARM2DSPSTATUS   0

/*  ============================================================================
 *  @const  BITPOS_DSP2ARMSTATUS
 *
 *  @desc   Start position of the DSP2ARM interrupt status/clear bits in the
 *          ARMINT register.
 *  ============================================================================
 */
#define  BITPOS_DSP2ARMSTATUS   0

/*  ============================================================================
 *  @const  BITPOS_ARM2DSPINTCLR
 *
 *  @desc   Start position of the ARM2DSP interrupt status/clear bits in the
 *          INTGEN register.
 *  ============================================================================
 */
#define  BITPOS_ARM2DSPINTCLR   0

/*  ============================================================================
 *  @const  BITPOS_DSP2ARMINTCLR
 *
 *  @desc   Start position of the DSP2ARM interrupt status bits in the ARMINT
 *          register.
 *  ============================================================================
 */
#define  BITPOS_DSP2ARMINTCLR   0

/*  ============================================================================
 *  @const  BASE_ARM2DSP_INTID
 *
 *  @desc   Interrupt number of the first ARM2DSP interrupt.
 *  ============================================================================
 */
#define  BASE_ARM2DSP_INTID     16 //??

/*  ============================================================================
 *  @const  BASE_DSP2ARM_INTID
 *
 *  @desc   Interrupt number of the first DSP2ARM interrupt.
 *  ============================================================================
 */
#if PLATFORM_PBX
//#define  BASE_DSP2ARM_INTID     68
#else
//#define  BASE_DSP2ARM_INTID     183
#endif


/** ============================================================================
 *  @const  MAX_POLL_COUNT
 *
 *  @desc   Indicates the maximum count to wait for interrupt to be cleared
 *          before timing out.
 *  ============================================================================
 */
#define MAX_POLL_COUNT          0x0FFFFFFF

/*  ============================================================================
 *  @macro  ARM2DSP_INT_INDEX
 *
 *  @desc   Index of the ARM2DSP interrupt (0/1/2/3) based on the interrupt
 *          number.
 *  ============================================================================
 */
//#define  ARM2DSP_INT_INDEX(intId)  (intId - BASE_ARM2DSP_INTID)

/*  ============================================================================
 *  @macro  DSP2ARM_INT_INDEX
 *
 *  @desc   Index of the DSP2ARM interrupt (0) based on the interrupt number.
 *  ============================================================================
 */
//#define  DSP2ARM_INT_INDEX(intId)  (intId - BASE_DSP2ARM_INTID)


/** ============================================================================
 *  @func   ZSP800M_halIntCtrl
 *
 *  @desc   Interrupt Controller.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halIntCtrl (IN         Pvoid          halObj,
                    IN         DSP_IntCtrlCmd cmd,
                    IN         Uint32         intId,
                    IN OUT     Pvoid          arg)
{
    DSP_STATUS         status    = DSP_SOK ;
    ZSP800M_HalObj * halObject = NULL    ;
    Uint32             addr              ;

    TRC_3ENTER ("ZSP800M__halIntCtrl", halObj, cmd, arg) ;

    DBC_Require (NULL != halObj) ;

    halObject = (ZSP800M_HalObj *) halObj ;

    switch (cmd) {
        case DSP_IntCtrlCmd_Enable:
        {
            /* Do nothing */
        }
        break ;

        case DSP_IntCtrlCmd_Disable:
        {
            /* Do nothing */
        }
        break ;

        case DSP_IntCtrlCmd_Send:
        {
            addr  =  halObject->baseSCMBus + OFFSET_DSPINTSET ;
            /* addr  =  halObject->baseSCMBus + OFFSET_DSPNMINTSET ; */
            writel(0x1, addr);
            /* for debug */
#if defined(DEBUG_ZSPSIDE)
            DPC_msleep(20, 1);
            printk("debug DWORD 0 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug)    ));
            printk("debug DWORD 1 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 1));
            printk("debug DWORD 2 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 2));
            printk("debug DWORD 3 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 3));
            printk("debug DWORD 4 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 4));
            printk("debug DWORD 5 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 5));
            printk("debug DWORD 6 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 6));
            printk("debug DWORD 7 is 0x%lx\n", *(((Uint32 *)halObject->baseDebug) + 7));
#endif
        }
        break ;

        case DSP_IntCtrlCmd_Clear:
        {
            addr  =  halObject->baseSCMBus + OFFSET_ARMINTSET ;
            writel(0x0, addr);
        }
        break ;

        case DSP_IntCtrlCmd_Check:
        {
            /* Do nothing here for DavinciHD Gem */
            *((Bool *) arg) = TRUE ;
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

    TRC_1LEAVE ("ZSP800M__halIntCtrl", status) ;

    return status ;
}


#if defined (__cplusplus)
}
#endif

