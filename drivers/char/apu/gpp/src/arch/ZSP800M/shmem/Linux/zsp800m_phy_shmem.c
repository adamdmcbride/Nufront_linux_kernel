/** ============================================================================
 *  @file   zsp800m_phy_shmem.c
 *
 *  @path   $(APUDRV)/gpp/src/arch/ZSP800M/shmem/Linux/
 *
 *  @desc   Linux shmem driver interface file.
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
#include <linux/clk.h>

/*  ----------------------------------- APU Driver               */
#include <apudrv.h>
#include <_apudrv.h>

/*  ----------------------------------- Trace & Debug               */
#include <_trace.h>
#include <trc.h>
#include <print.h>

/*  ----------------------------------- OSAL Headers                */
#include <osal.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <zsp800m_hal.h>
#include <zsp800m_hal_boot.h>
#include <zsp800m_hal_intgen.h>
#include <zsp800m_hal_power.h>
#include <zsp800m_phy_shmem.h>

#if defined (__cplusplus)
extern "C" {
#endif


/** ============================================================================
 *  @macro  COMPONENT_ID
 *
 *  @desc   Component and Subcomponent Identifier.
 *  ============================================================================
 */
#define  COMPONENT_ID       ID_ARCH_SHMEM_PHY

/** ============================================================================
 *  @macro  SET_FAILURE_REASON
 *
 *  @desc   Sets failure reason.
 *  ============================================================================
 */
#if defined (DDSP_DEBUG)
#define SET_FAILURE_REASON TRC_SetReason (status,FID_C_ARCH_PHY_SHMEM,__LINE__)
#else
#define SET_FAILURE_REASON
#endif /* if defined (DDSP_DEBUG) */


/** ============================================================================
 *  @name   ZSP800M_shmemInterface
 *
 *  @desc   Interface functions exported by the Shared Driver subcomponent.
 *  ============================================================================
 */
HAL_Interface ZSP800M_shmemInterface = { &ZSP800M_phyShmemInit,
                                          &ZSP800M_phyShmemExit,
                                          &ZSP800M_halBootCtrl,
                                          &ZSP800M_halIntCtrl,
                                          NULL, //&ZSP800M_halMmuCtrl,
                                          NULL,
                                          &ZSP800M_halPwrCtrl,
                                          NULL,
                                          NULL,
                                          NULL,
                                          NULL} ;




#define PHY_IOREMAP(base, i_size, dest) \
    mapInfo.src = base;\
    mapInfo.size = i_size;\
    mapInfo.memAttrs = MEM_UNCACHED;\
    status = MEM_Map (&mapInfo) ;\
    if (DSP_FAILED (status)) {\
        SET_FAILURE_REASON ;\
    }\
    dest = mapInfo.dst ;
    

#define PHY_IOUNMAP(address, i_size) \
    unmapInfo.addr = address;\
    unmapInfo.size = i_size ;\
    if (unmapInfo.addr != 0) {\
        MEM_Unmap (&unmapInfo) ;\
        if (DSP_FAILED (status)) {\
            SET_FAILURE_REASON ;\
        }\
    }\
    address = 0 ;
    

/** ============================================================================
 *  @func   ZSP800M_phyShmemInit
 *
 *  @desc   Initializes Shared/OMAP Gem device.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_phyShmemInit (IN Pvoid halObj)
{
    DSP_STATUS        status    = DSP_SOK ;
    ZSP800M_HalObj * halObject = NULL    ;
    MemMapInfo  mapInfo ;
    Char8*     dspId     = "ns115_zsp_clk";
    struct     clk*  clk = NULL;

    TRC_1ENTER ("ZSP800M_phyShmemInit", halObj) ;

    DBC_Require (NULL != halObj) ;

    halObject = (ZSP800M_HalObj *) halObj ;

    PHY_IOREMAP(SCM_BASE, 0x80, halObject->baseSCMBus);
    PHY_IOREMAP(PRCM_BASE, 0x80, halObject->basePRCMBus);

    clk = clk_get (NULL, dspId);
    if (clk != NULL)
        halObject->handleClock = (Uint32)clk;

    TRC_1LEAVE ("ZSP800M_phyShmemInit", status) ;

    return status ;
}


/** ============================================================================
 *  @func   ZSP800M_phyShmemExit
 *
 *  @desc   Finalizes Shared/OMAP Gem device.
 *
 *  @modif  None.
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_phyShmemExit (IN Pvoid halObj)
{
    DSP_STATUS     status = DSP_SOK ;
    ZSP800M_HalObj * halObject = (ZSP800M_HalObj *) halObj ;
    MemUnmapInfo      unmapInfo                              ;

    TRC_1ENTER ("ZSP800M_phyShmemExit", halObj) ;

    DBC_Require (NULL != halObject) ;

    PHY_IOUNMAP(halObject->baseSCMBus, 0x80);
    PHY_IOUNMAP(halObject->basePRCMBus, 0x80);
    halObject->handleClock = 0;

    TRC_1LEAVE ("ZSP800M_phyShmemExit", status) ;

    return status ;
}


#if defined (__cplusplus)

}
#endif
