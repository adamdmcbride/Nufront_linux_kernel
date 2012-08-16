/** ============================================================================
 *  @file   ldrv.h
 *
 *  @path   $(APUDRV)/gpp/src/ldrv/
 *
 *  @desc   Defines constants and interfaces to initialize and finalize
 *          sub-component LDRV.
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


#if !defined (LDRV_H)
#define LDRV_H


/*  ----------------------------------- APU DRIVER Headers       */
#include <apudrv.h>
#include <linkcfgdefs.h>
#include <cfgmap.h>


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @name   LDRV_LinkCfgPtr
 *
 *  @desc   Global pointer containing APUDRV configuration information for the
 *          driver.
 *  ============================================================================
 */
extern LINKCFG_Object * LDRV_LinkCfgPtr ;

/** ============================================================================
 *  @name   CFGMAP_Config
 *
 *  @desc   Global structure containing APUDRV configuration mapping
 *          information for APU DRIVER.
 *  ============================================================================
 */
extern CFGMAP_Object  * CFGMAP_Config [] ;



/** ============================================================================
 *  @func   LDRV_moduleInit
 *
 *  @desc   Initializes the LDRV sub-component for the Gpp.
 *
 *  @arg    linkCfg
 *              Pointer to the user-side system configuration object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EMEMORY
 *              Out of memory error.
 *          DSP_ECONFIG
 *              Incorrect configuration.
 *          DSP_EFAIL
 *              General error from GPP-OS.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    LDRV_exit
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
LDRV_moduleInit (IN LINKCFG_Object * linkCfg) ;


/** ============================================================================
 *  @func   LDRV_moduleExit
 *
 *  @desc   Releases resources used by LDRV sub-component for the Gpp.
 *
 *  @arg    none.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EMEMORY
 *              Out of memory error.
 *          DSP_EFAIL
 *              General error from GPP-OS.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    LDRV_init
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
LDRV_moduleExit (Void) ;


/** ============================================================================
 *  @func   LDRV_init
 *
 *  @desc   Initializes the LDRV sub-component for a DSP.
 *
 *  @arg    procId
 *              Processor Identifier.
 *  @arg    dspCfg
 *              Pointer to the user-side DSP configuration object.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EMEMORY
 *              Out of memory error.
 *          DSP_ECONFIG
 *              Incorrect configuration.
 *          DSP_EFAIL
 *              General error from GPP-OS.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    LDRV_exit
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
LDRV_init (IN ProcessorId procId, IN LINKCFG_DspConfig * dspCfg) ;


/** ============================================================================
 *  @func   LDRV_exit
 *
 *  @desc   Releases resources used by LDRV sub-component for a DSP.
 *
 *  @arg    procId
 *              Processor Identifier.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EMEMORY
 *              Out of memory error.
 *          DSP_EFAIL
 *              General error from GPP-OS.
 *
 *  @enter  None
 *
 *  @leave  None
 *
 *  @see    LDRV_init
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
LDRV_exit (IN ProcessorId procId) ;


#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* !defined (LDRV_H) */
