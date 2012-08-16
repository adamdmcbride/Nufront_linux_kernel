/** ============================================================================
 *  @file   zsp800m_hal.h
 *
 *  @path   $(APUDRV)/gpp/src/inc/sys/arch/ZSP800M/
 *
 *  @desc   Hardware Abstraction Layer for Power Controller (PWR)
 *          module in Davinci. Declares necessary functions for
 *          PSC request handling.
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


#if !defined (ZSP800M_HAL_H)
#define ZSP800M_HAL_H


/*  ----------------------------------- APU Driver               */
#include <apudrv.h>
#include <_apudrv.h>

/*  ----------------------------------- Trace & Debug               */
#include <_trace.h>

/*  ----------------------------------- Hardware Abstraction Layer  */
#include <hal.h>

/*  ----------------------------------- OSAL Headers                */
#include <osal.h>


#if defined (__cplusplus)
extern "C" {
#endif

/*  ============================================================================
 *  @macro  REG
 *
 *  @desc   Regsiter access method.
 *  ============================================================================
 */
/* #define REG(x)              *((volatile u32 *) (x)) */


/** ============================================================================
 *  @name   ZSP800M_HalObj
 *
 *  @desc   Hardware Abstraction object.
 *
 *  @field  interface
 *              Pointer to HAL interface table.
 *  @field  baseCfgBus
 *              base address of the configuration bus peripherals memory range.
 *  @field  offsetSysModule
 *              Offset of the system module from CFG base.
 *  ============================================================================
 */
typedef struct ZSP800M_HalObj_tag {
    HAL_Interface * interface ;
    Uint32          baseSCMBus;
    Uint32          basePRCMBus;
    Uint32          handleClock;
    Uint32          procId;
    Uint32          baseDebug;
} ZSP800M_HalObj ;


/** ============================================================================
 *  @func   ZSP800M_halInitialize
 *
 *  @desc   Initializes the HAL object.
 *
 *  @arg    halObj.PRCM_BASE
 *              Pointer to HAL object.
 *  @arg    initParams.
 *              Parameters for initialize (optional).
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EMEMORY
 *              A memory allocation failure occurred.
 *          DSP_EINVALIDARG
 *              An invalid argument was specified.
 *
 *  @enter  None.
 *
 *  @leave  None.
 *
 *  @see    None
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halInit (IN     Pvoid  *halObj,
                 IN     Pvoid  initParams) ;


/** ============================================================================
 *  @func   ZSP800M_halExit
 *
 *  @desc   Finalizes the HAL object.
 *
 *  @arg    halObj.
 *              Pointer to HAL object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              All other error conditions.
 *
 *  @enter  None.
 *
 *  @leave  None.
 *
 *  @see    None
 *  ============================================================================
 */
NORMAL_API
DSP_STATUS
ZSP800M_halExit (IN     Pvoid   *halObj) ;


#if defined (__cplusplus)
}
#endif


#endif  /* !defined (ZSP800M_HAL_H) */

