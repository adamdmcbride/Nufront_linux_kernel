/** ============================================================================
 *  @file   zsp800m_hal.c
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

/*  ----------------------------------- APU Driver               */
#include <apudrv.h>
#include <_apudrv.h>

/*  ----------------------------------- Trace & Debug               */
#include <_trace.h>
#include <trc.h>
#include <print.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <zsp800m_hal.h>
#if (ZSP800M_PHYINTERFACE == SHMEM_INTERFACE)
#include <zsp800m_phy_shmem.h>
#endif /* if (ZSP800M_PHYINTERFACE == SHMEM_INTERFACE) */



#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
 *  @macro  COMPONENT_ID
 *
 *  @desc   Component and Subcomponent Identifier.
 *  ============================================================================
 */
#define  COMPONENT_ID       ID_ARCH_HAL

/** ============================================================================
 *  @macro  SET_FAILURE_REASON
 *
 *  @desc   Sets failure reason.
 *  ============================================================================
 */
#if defined (DDSP_DEBUG)
#define SET_FAILURE_REASON      TRC_SetReason (status, FID_C_ARCH_HAL, __LINE__)
#else
#define SET_FAILURE_REASON
#endif /* if defined (DDSP_DEBUG) */


/** ============================================================================
 *  @func   ZSP800M_halInitialize
 *
 *  @desc   Initializes the HAL object.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halInit (IN     Pvoid *halObj,
                   IN     Pvoid initParams)
{
    DSP_STATUS         status   = DSP_SOK ;
    ZSP800M_HalObj   * halObject ;

    TRC_2ENTER ("ZSP800M_halInit", halObj, initParams) ;

    DBC_Require (halObj != NULL) ;

    (void) initParams ;

    status = MEM_Alloc ((Pvoid) &halObject,
                        sizeof (ZSP800M_HalObj),
                        MEM_DEFAULT) ;
    if (DSP_FAILED (status)) {
        SET_FAILURE_REASON ;
    }
    else {
        *halObj = (Pvoid) halObject ;
#if (ZSP800M_PHYINTERFACE == SHMEM_INTERFACE)
        halObject->interface = &ZSP800M_shmemInterface ;
#endif /* if (OMAP3530_PHYINTERFACE == SHMEM_INTERFACE) */
        status = halObject->interface->phyInit ((Pvoid) halObject) ;
        if (DSP_FAILED (status)) {
            FREE_PTR (halObject) ;
            SET_FAILURE_REASON ;
        }
        /* for debug the DSP side code */
        halObject->baseDebug = (Uint32)initParams;
    }


    TRC_1LEAVE ("ZSP800M_halInit", status) ;

    return status ;
}


/** ============================================================================
 *  @func   ZSP800M_halExit
 *
 *  @desc   Finalizes the HAL object.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halExit (IN     Pvoid *halObj)
{
    DSP_STATUS status   = DSP_SOK ;
    ZSP800M_HalObj * halObject ;

    TRC_1ENTER ("ZSP800M__halExit", halObj) ;

    DBC_Require (halObj != NULL) ;

    halObject = (ZSP800M_HalObj *) (*halObj);

    //iounmap((void*)halObject->baseSCMBus);
    //halObject->baseSCMBus = 0;
    //release_mem_region(SCM_BASE, 0x80);

    //iounmap((void*)halObject->basePRCMBus);
    //halObject->basePRCMBus = 0;
    //release_mem_region(PRCM_BASE, 0x80);
    status = halObject->interface->phyExit ((Pvoid) halObject) ;
    if (DSP_FAILED (status)) {
        SET_FAILURE_REASON ;
    }

    FREE_PTR (halObject) ;

    TRC_1LEAVE ("ZSP800M__halExit", status) ;

    return status ;
}


#if defined (__cplusplus)
}
#endif

