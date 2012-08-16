/** ============================================================================
 *  @file   zsp800m_dspclk.h
 *
 *  @path   $(APUDRV)/gpp/inc/sys/arch/ZSP800M/
 *
 *  @desc   Defines the interfaces and data structures for the DSPCLK access.
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



#if !defined (ZSP800MDSPCLK_H)
#define ZSP800MDSPCLK_H


/*  ----------------------------------- APU Driver               */
#include <apudrv.h>
#include <_apudrv.h>


#if defined (__cplusplus)
extern "C" {
#endif /* defined (__cplusplus) */


/** ============================================================================
 *  @func   ZSP800M_getDspClkRate
 *
 *  @desc   Get DSP clock frequency
 *
 *  @arg    dspIdentifier
 *              String which describes DSP
 *  @arg    cpuFreq
 *              DSP clock frequency
 *              number of bytes to copy.
 *
 *  @ret    DSP_SOK
 *              Operation successfully completed.
 *          DSP_EFAIL
 *              Failed to copy.
 *
 *  @enter  dspIdentifier, cpuFreq must be valid.
 *
 *  @leave  None
 *
 *  @see    None
 *  ============================================================================
 */
EXPORT_API
DSP_STATUS
ZSP800M_getDspClkRate (IN Void * dspIdentifier, OUT Uint32 * cpuFreq);




#if defined (__cplusplus)
}
#endif /* defined (__cplusplus) */


#endif /* !define (ZSP800MDSPCLK_H) */

